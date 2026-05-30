[🇺🇸 English](../README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Universal Communication Protocol)

Módulo de controle de congestionamento TCP para ambientes VPS de largura de banda compartilhada que combina a máquina de estados do BBRv1 com um filtro de Kalman para estimativa do atraso de propagação.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Visão Geral do Algoritmo

O TCP UCP implementa um módulo de controle de congestionamento no lado do remetente para o kernel Linux como um `tcp_ucp.ko` carregável. A função de controle de congestionamento `ucp_main()` é invocada em cada ACK a partir de `tcp_ack()`, recebendo uma estrutura `rate_sample` que contém amostras de largura de banda e RTT do kernel, juntamente com contagens de entrega e perda. O algoritmo opera em dois regimes temporais: um **caminho rápido por ACK** que atualiza o estado das medições e calcula alvos instantâneos de pacing e janela, e um **caminho lento por rodada** que avalia condições de transição de estado e recalcula ganhos.

O pipeline principal de medição consiste em dois componentes:

1. **Filtro de largura de banda máxima com janela deslizante** (`minmax_running_max` de `linux/win_minmax.h`): janela cobrindo os últimos `ucp_bw_rt_cycle_len` (padrão 10) ciclos de ida e volta. Fornece a estimativa `max_bw` compatível com BBR.

2. **Estimador de atraso de propagação por filtro de Kalman**: substitui o RTT mínimo de janela deslizante do BBRv1. Um filtro de Kalman de estado único (Kalman 1960) operando em unidades de ponto fixo `ucp_kalman_scale` × µs, modelando o atraso de propagação real como um passeio aleatório:
   - Estado: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Observação: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Convenções de ponto fixo: `BW_UNIT = 1 << 24` para largura de banda (segmentos * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` como a unidade de ganho adimensional.

## Máquina de Estados

```
    ┌───> STARTUP ────┐
    │       │         │
    │       ▼         │
    │     DRAIN  ─────┤
    │       │         │
    │       ▼         │
    └─── PROBE_BW ────┘
    │      ^    │
    │      │    │
    │      └────┘
    │
    └─── PROBE_RTT <──┘
```

Quatro modos codificados no campo de 2 bits `mode` em `struct ucp`:

- **STARTUP (0)**: Estado inicial. pacing_gain ≈ 2.885x (`ucp_high_gain_val`), cwnd_gain também 2.885x. Sondagem exponencial de largura de banda.
- **DRAIN (1)**: Entrada após saída do STARTUP. pacing_gain ≈ 0.347x (`ucp_drain_gain_val`), cwnd_gain permanece 2.885x. Drena a fila acumulada durante o STARTUP.
- **PROBE_BW (2)**: Estado estacionário. Percorre uma tabela de ganhos de 256 slots (padrão de 8 fases repetido: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: Periodicamente drena o tráfego em voo para `ucp_cwnd_min_target` (padrão 4 segmentos) para obter uma amostra RTT fresca.

### STARTUP → DRAIN

Acionado quando `full_bw_reached` é definido — após `ucp_full_bw_cnt` (padrão 3) rodadas consecutivas onde `max_bw` não cresce pelo menos `ucp_full_bw_thresh_val` (padrão 1.25x) comparado ao pico observado anteriormente. O BDP com ganho 1.0x é escrito em `snd_ssthresh`. `qdelay_avg` é redefinido para zero para evitar que o acúmulo de fila do STARTUP afete o PROBE_BW.

### DRAIN → PROBE_BW

Acionado quando o tráfego em voo estimado no EDT ≤ tráfego em voo alvo com ganho BDP de 1.0x. **Otimização de pulo do DRAIN**: quando o filtro de Kalman está convergido E `qdelay_avg` está abaixo de `ucp_drain_skip_qdelay_us` (padrão 1000 µs), a fase DRAIN é pulada — converte antecipadamente para PROBE_BW.

Na entrada do PROBE_BW, o índice da fase do ciclo é randomizado: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (padrão `len − 1 − rand(8)`), o que descorrelaciona fluxos concorrentes compartilhando um gargalo.

### PROBE_BW → PROBE_RTT

Acionado quando o intervalo do filtro PROBE_RTT expira — o timestamp `min_rtt_stamp` não foi atualizado dentro do intervalo calculado. cwnd é salvo em `prior_cwnd`, pacing definido para drenar.

### PROBE_RTT → PROBE_BW

Após o tráfego em voo cair para `ucp_cwnd_min_target` ou um limite de rodada ser observado, persiste por pelo menos `ucp_probe_rtt_mode_ms_val` (padrão 200 ms) e pelo menos uma rodada completa observada, então sai. cwnd é restaurado para pelo menos `prior_cwnd`, pacing é temporariamente substituído por `ucp_high_gain_val` para reabastecimento rápido do pipe.

### Recuperação e Perda

- Em TCP_CA_Loss: `full_bw` e `full_bw_cnt` são redefinidos, `round_start` é definido como 1, `packet_conservation` é limpo para 0. Se LT BW não estiver ativo, injeta um evento de perda sintético para acionar a amostragem LT.
- Entrada de recuperação (TCP_CA_Recovery): `packet_conservation` habilitado, cwnd = inflight + acked.
- Saída de recuperação: restaurado para `prior_cwnd`, `packet_conservation` limpo.
- `ucp_undo_cwnd()`: redefine `full_bw` e `full_bw_cnt` (preservando `full_bw_reached`), limpa o estado LT BW.

## Medições Principais

### Estimativa de Largura de Banda

Filtro de largura de banda máxima com janela deslizante (`minmax_running_max` de `linux/win_minmax.h`) ao longo de `ucp_bw_rt_cycle_len` (padrão 10) rodadas. bw instantânea = `delivered × BW_UNIT / interval_us` calculada por ACK. Alimentada na janela deslizante apenas quando não está limitada pelo aplicativo ou quando bw ≥ máximo atual (regra BBR).

Quando `lt_use_bw` está ativo, a estimativa de largura de banda ativa muda para `lt_bw` (estimativa de largura de banda de longo prazo).

### Filtro de Kalman

Recursão escalar de Kalman de estado único (complexidade O(1)):

```
Predizer:
  x_pred = x_est          (transição de estado identidade)
  p_pred = p_est + Q      (predição de covariância)

Atualizar:
  innov   = z − x_pred    (inovação)
  K       = p_pred / (p_pred + R)   (ganho de Kalman [0,1])
  x_est   = x_pred + K × innov      (atualização de estado)
  p_est   = (1 − K) × p_pred        (covariância posteriori)
```

**Ruído de processo adaptativo Q**:
```
Q_base   = ucp_kalman_q (padrão 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Ruído de medição adaptativo R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Portão de outliers**: limite dinâmico `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Aplicado apenas quando `p_pred ≤ ucp_kalman_converged_p_est_val`. Após `ucp_kalman_max_consec_reject` (padrão 25) rejeições consecutivas, a próxima amostra é forçadamente aceita para evitar bloqueio auto-reforçado.

**Estimativa de ruído com correspondência de covariância (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Modo de combinação: modo 0 = apenas heurístico, modo 1 = máximo (padrão), modo 2 = mistura ponderada.

**Assunção de controle do Kalman**: quando `x_est > 0` e `sample_cnt ≥ ucp_kalman_min_samples` (padrão 5), `min_rtt_us` é substituído por `x_est / ucp_kalman_scale`. `min_rtt_stamp` não é atualizado — o gatilho do intervalo PROBE_RTT permanece independente.

**Limite de margem x_est**: O `model_rtt` derivado de Kalman é limitado a `min_rtt_us × (100 + ucp_kalman_xest_margin_pct) / 100` (padrão 8%).

## Melhorias do BBR

### Decaimento de Ganho

Habilitado pelo bitmap de 256 bits `ucp_cycle_decay_mask[]` para fases específicas do PROBE_BW. Fórmula de decaimento (na amostra de Kalman aceita):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = escalonamento inverso de p_est (BBR_UNIT no máximo)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Escalonamento de confiança do Kalman: quando `p_est > ucp_kalman_converged_p_est`, o decaimento é reduzido proporcionalmente, evitando recuo excessivo quando o filtro está incerto.

### ECN Backoff

Condições de ativação (todas devem ser verdadeiras):
1. `ucp_ecn_enable_val != 0`
2. Kalman convergiu (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (marcações CE observadas)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (padrão 2000 µs)
5. Modo NÃO é PROBE_BW (cwnd_gain é fixo em 2x no PROBE_BW)

Durante fases de sondagem (`pacing_gain > BBR_UNIT`), o ECN backoff é graduado por `BBR_UNIT² / pacing_gain` — ~80% de recuo na sonda 1.25x, ~65% no ganho STARTUP 2.89x.

EWMA da razão de marcações ECN: atualizada nos limites de rodada por `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (padrão 3/4), com decaimento suave por ACK de `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (padrão 31/32) em cada ACK sem novas marcações CE.

### Detecção de Fluxo Único

Quando o UCP detecta que o fluxo provavelmente está sozinho no gargalo (baixo atraso de fila, baixo jitter, sem marcações ECN, sem agregação de ACK, sem largura de banda LT), ele faz uma transição automática para um modo BBR puro:

- `ucp_get_model_rtt()` retorna `min_rtt_us` diretamente (evitando a estimativa suavizada de Kalman, que tem um pequeno viés positivo devido ao ruído de medição unilateral).
- `ucp_ecn_backoff()` é configurável via `ucp_alone_bypass_ecn` (padrão 1) — em um caminho de fluxo único, as marcas ECN são falsos positivos do AQM porque não há outro emissor competindo. Ignorá-lo corresponde ao comportamento ECN zero do BBR. Defina como 0 para manter o backoff ECN mesmo no modo isolado (conservador).
- A condição LT BW (policer) é configurável via `ucp_alone_bypass_lt_bw` (padrão 1) — um caminho de fluxo único não tem policer, portanto o LT BW não pode ser ativado legitimamente. Ignorá-la evita saídas espúrias do modo isolado por falsos disparos. Defina como 0 para o comportamento rigoroso original.

Isso elimina a lacuna de desempenho em fluxo único entre UCP e BBR, preservando ao mesmo tempo o loop de proteção completo do UCP (Kalman, backoff ECN, decaimento de ganho, largura de banda LT) para cenários de múltiplos fluxos.

**Histérese**: A entrada requer `ucp_alone_confirm_rounds` (padrão 3) rodadas consecutivas qualificadas — evitando oscilações durante breves períodos de calma na competição de múltiplos fluxos ("conservador para acelerar"). A saída é imediata — qualquer falha de qualificação limpa a flag e redefine o contador de confirmação ("agressivo para frear").

Condições de qualificação (todas as seis devem ser atendidas em um limite de rodada):
0. Kalman convergido (`sample_cnt >= ucp_kalman_min_samples`) — confiar em qdelay/jitter como sinais de fila
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (padrão 1000 us) — fila quase vazia
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (padrão 2000 us) — apenas micro-jitter de relógio ACK
3. `ecn_ewma == 0` — sem marcas de congestionamento do AQM
4. `lt_use_bw == 0` — não está no modo de taxa limitada detectado pelo policer
5. `agg_state <= max` conforme `ucp_alone_agg_state_level` (padrão 1) — três níveis de rigor de agregação ACK: 0 = apenas IDLE (mais rigoroso, zero agregação), 1 = ≤ SUSPECTED (padrão, permite agregação transitória), 2 = ≤ CONFIRMED (mais permissivo, bloqueia apenas agregação persistente)

### Intervalo PROBE_RTT Dinâmico

Mapeia `p_est` do Kalman para um intervalo PROBE_RTT por conexão:

```
p_est ≤ converged:              intervalo = dyn_max (padrão 30s)
p_est ≥ high (= mult × conv):   intervalo = base (padrão 10s)
converged < p_est < high:       interpolação linear
```

Reduz a frequência do PROBE_RTT quando a confiança é alta (baixo `p_est`), diminuindo o jitter de throughput em caminhos estáveis. Reverte para o intervalo clássico de 10 segundos quando a confiança é baixa.

**Jitter de entrada por fluxo**: Para evitar que todos os fluxos coexistentes entrem em PROBE_RTT simultaneamente (drenando para 4 pacotes agregados ~1.8 Mbps e depois reabastecendo a 2.89×), cada fluxo adiciona um jitter derivado de hash (distribuição de 0–845 ms) ao seu intervalo PROBE_RTT. No máximo ~1 fluxo está em PROBE_RTT em qualquer instante, eliminando o colapso simultâneo de drenagem/reabastecimento que induz RTO.

### Estimativa de Largura de Banda LT (Long-Term)

Estimador de limite inferior acionado por perda. Intervalo de amostragem abrange [4, 16] RTTs. Válido quando a taxa de perda ≥ 5.9% (`ucp_lt_loss_thresh` padrão 15/256). Largura de banda `bw = delivered × BW_UNIT / interval_us`.

Diferente da média simples do BBR (`(bw + lt_bw) >> 1`), o UCP usa uma EMA configurável (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, padrão 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

A ativação difere do BBR: o UCP armazena `lt_bw` no primeiro intervalo válido mas NÃO define `lt_use_bw`; é necessária consistência com um intervalo anterior — reduz ativação falsa por ruído de medição.

**Portão de congestionamento de limiar duplo**: Antes de definir `lt_use_bw = 1`, tanto uma verificação de fila EWMA persistente (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) QUANTO uma verificação de fila instantânea baseada em SRTT (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, padrão 5000 µs) são avaliadas. Quando a congestão é detectada, a amostragem LT BW é abortada. A verificação SRTT funciona sem alocação `ext`, fornecendo uma rede de segurança contra falha de alocação.

Impulso de sonda LT BW (`ucp_lt_bw_probe_pct`, padrão 10%): amplifica pacing_gain em `1 + probe_pct/100` em todas as fases PROBE_BW. Componente de rampa: aumento de `+1% a cada 8 RTTs`, limitado a `2 × probe_pct`.

Autorecuperação LT BW (`ucp_lt_restore_ratio_num/den`, padrão 5/4 = 1.25x): quando `max_bw > lt_bw × ratio` por `ucp_lt_restore_consec_acks` (padrão 3) ACKs consecutivos, o LT BW sai automaticamente e a sondagem normal PROBE_BW é retomada.

### Compensação de Agregação ACK Baseada em Confiança (inspirado no BBRplus)

Adiciona uma segunda camada com portão de confiança sobre o estimador tradicional de slot duplo extra-acked.

**Quatro fatores ortogonais** (cada um contribui com `ucp_agg_factor_weight` pontos, padrão 256):
1. Kalman convergiu (`p_est < converged` + `sample_cnt >= min_samples`)
2. Não em recuperação de perda (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT dentro de `min_rtt_us + ucp_agg_factor3_qdelay_us` (padrão 2ms) do atraso de propagação real
4. `extra_acked` dentro de `ucp_agg_factor4_ratio_num/den` (padrão 1.5x) do máximo da janela

**Quatro estados**: IDLE (< `ucp_agg_thresh_suspected`=256), SUSPECTED (≥256), CONFIRMED (≥512), TRUSTED (≥768).

**Camada de sinal** (sempre ativa): a confiança interpola linearmente o fator de escala R `[r_min, r_max]`. R sobe instantaneamente (resposta rápida), decai a `ucp_agg_r_hysteresis`% (padrão 75% retido, ~4 RTTs até a linha de base) por RTT.

**Camada de controle** (`agg_state ≥ CONFIRMED`): compensação de cwnd com portão de segurança de cinco camadas:
1. Bloqueia se atraso de fila > `ucp_agg_safety_qdelay_us` (padrão 4ms)
2. Bloqueia durante recuperação de perda
3. Bloqueia se cwnd > `BDP × ucp_agg_safety_bdp_mult` (padrão 3x)
4. Bloqueia se tráfego em voo > cwnd seguro + meta de segmentos TSO
5. Watchdog: rebaixa CONFIRMED→SUSPECTED após `ucp_agg_max_comp_duration` (padrão 8) RTTs consecutivos

### Reinicialização do qdelay_avg no DRAIN

Na transição para DRAIN, `qdelay_avg` é redefinido para zero, impedindo que a estimativa de fila do STARTUP persista no PROBE_BW.

### Adaptação do Divisor TSO

`ucp_min_tso_segs()` ajusta o divisor do limiar de taxa com base no estado do Kalman:
- Kalman convergiu + `jitter_ewma < 1000 µs`: divisor reduzido à metade (8→4), rajadas TSO maiores
- `jitter_ewma > 4000 µs`: divisor dobrado (8→16), rajadas TSO menores para suprimir jitter

## Taxa de Pacing e Cwnd

### Taxa de Pacing

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // ajuste de ganho
rate = rate × USEC_PER_SEC >> BW_SCALE            // converter para bytes/s
rate = rate × margin_div / 100                    // margem de pacing (padrão 1%, matching BBR)
```

Mudanças de taxa são aplicadas imediatamente (sem suavização), igual ao BBR (Cardwell et al. 2016). Após `full_bw_reached`: todas as mudanças de taxa são escritas imediatamente. Em STARTUP/DRAIN: apenas aumentos são aplicados (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP base
// limites de tráfego em voo (não-STARTUP: clamp lo~hi; STARTUP: apenas piso lo)
target = quantization_budget(target)              // margem TSO + arredondamento + bônus fase-0
target += ack_agg_bonus + agg_compensation        // compensação de agregação ACK

// progressão cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // convergir para o alvo
else (STARTUP):
    cwnd = cwnd + acked                          // crescimento exponencial

cwnd = max(cwnd, cwnd_min_target)                 // piso absoluto 4
Modo PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // tráfego em voo mínimo
```

## Parâmetros do Módulo

Parâmetros são expostos em `/proc/sys/net/ucp/`. Escritas acionam `ucp_init_module_params()` (validação + clamp + cálculo de valor derivado). Escritas de parâmetros array acionam `ucp_rebuild_gain_table()`.

### Intervalos PROBE_RTT

| Parâmetro | Padrão | Mín | Máx | Unid | Descrição |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Intervalo base PROBE_RTT |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Limite superior para caminhos de RTT longo |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Intervalo dinâmico máximo; 0 desabilita |

### Ganhos

| Parâmetro | Padrão | Mín | Máx | Descrição |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Ganho cwnd base para PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Multiplicador de bônus de agregação ACK |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Ganho STARTUP (≈2.885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Ganho DRAIN (≈0.347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Limite inferior de tráfego em voo (1.25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Limite superior de tráfego em voo (2.0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | Padrão BBRv1 (256 slots) | 0/1 | — | Ganho de pacing por slot |
| `ucp_cycle_decay_mask[8]` | 0 (todos zero) | 0 | 0x7FFFFFFF | Bitmap de decaimento de 256 bits |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Saída limitada de sondagem ascendente (0=desligado) |

### Kalman Base

| Parâmetro | Padrão | Mín | Máx | Descrição |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Ruído de processo base Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Ruído de medição base R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | Máximo absoluto de p_est |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | Limiar de convergência |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | p_est inicial |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | Piso de p_est |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | Escala de ponto fixo (potência de dois) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Amostras mínimas antes da assunção |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Limiar base de outlier |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Multiplicador Q-boost |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Constante de tempo Q-boost |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Teto Q |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Limite de escala Q |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Máximo de rejeições consecutivas antes de aceitação forçada |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Teto RTT do Kalman |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | Multiplicador de impulso R máximo |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Multiplicador de teto dinâmico RTT |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Divisor RTT de adaptação Q |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | Multiplicador de banda de transição PROBE_RTT |

### Extras do Kalman (tipo num/den)

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Multiplicador de jitter de outlier |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Fator mínimo Q |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Divisor RTT de inicialização p_est |

### Estimativa de Ruído BBR-S

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Taxa de aprendizado da estimativa Q |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Taxa de aprendizado da estimativa R |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Modo de combinação (0=desligado, 1=máx, 2=média ponderada) |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Limite superior da estimativa Q |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | Limite superior da estimativa R |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Piso por estimativa |

### Decaimento de Ganho (Sondagem)

| Parâmetro | Padrão | Faixa | Unid | Descrição |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | Limiar de decaimento qdelay |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | Escala de decaimento qdelay |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Limiar de decaimento de jitter |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | Escala de decaimento de jitter |

### R Adaptativo (Acionado por Jitter)

| Parâmetro | Padrão | Faixa | Unid | Descrição |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | Limiar de jitter para aumento de R |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | Divisor de escala de aumento de R |

### ECN

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | Interruptor principal ECN |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Fração de recuo ECN |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | Limiar de qdelay ECN |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Pesos EWMA ECN |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Decaimento ECN ocioso |

### min_rtt

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Contagem de queda rápida |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Divisor de limiar de queda rápida |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Razão de queda pegajosa |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Razão de guarda SRTT |

### Largura de Banda LT

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Comprimento mínimo do intervalo |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Multiplicador de tempo limite do intervalo |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Razão de perda mínima |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Tolerância relativa |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | Tolerância absoluta |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | RTTs ativos máximos LT BW |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | Impulso de sonda LT BW |

### Autorecuperação LT

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Razão de gatilho de recuperação |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Contagem de ACKs consecutivos para gatilho |

### Confiança de Agregação ACK

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Interruptor principal |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | Limiar de confiança para compensação cwnd |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % do BDP | Limite de compensação cwnd |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | Tempo limite do watchdog |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | Histerese de decaimento R |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Faixa de escala R (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Margem qdelay do fator 3 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Razão do fator 4 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Guarda de segurança 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Multiplicador BDP de segurança |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | Janela de limite extra_acked |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Taxa de decaimento do watchdog |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Período de rotação da janela |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Pontuação por fator |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Confiança máxima |

### Coeficientes EWMA

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA qdelay |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA jitter |

### Diversos

| Parâmetro | Padrão | Faixa | Descrição |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | Fases do ciclo PROBE_BW (potência de dois) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Deslocamento aleatório da fase do ciclo |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Limiar de crescimento para saída do STARTUP |
| `ucp_full_bw_cnt` | 3 | 1-3 | Rodadas sem crescimento para sair |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Duração da estadia em PROBE_RTT |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Margem de pacing (0 = nenhuma) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | Bônus cwnd da fase 0 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | rodadas | Comprimento da janela deslizante BW |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | Cwnd mínimo (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | Piso min_rtt do BDP |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | Limiar EDT "quase agora" |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | Limiar de taxa baixa TSO |
| `ucp_min_tso_rate_div` | 8 | 1-256 | Divisor de taxa TSO (base adaptativa) |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | Máximo de segmentos TSO |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | Multiplicador de margem TSO |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Fator de expansão do buffer de envio |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | Limite de época ACK |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Janela máxima de agregação ACK |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Limiar de RTT longo |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Divisor de intervalo de RTT longo |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Limiar qdelay para pular DRAIN |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | rodadas | Rodadas antes de ativar o modo de fluxo único |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Atraso de fila máx para detecção de fluxo único |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | Jitter máx para detecção de fluxo único |
| ucp_alone_agg_state_level | 1 | 0-2 | — | Rigor de agregação (0=apenas IDLE, 1=≤SUSPECTED padrão, 2=≤CONFIRMED muito agressivo) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | Ignorar backoff ECN no modo isolado (1=ignorar, 0=ativo) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | Ignorar condição LT BW no modo isolado (1=ignorar, 0=ativo) |

## Caminho de Dados

```
ACK Chega (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► Pipeline de confiança de agregação ACK (quando ucp_agg_enable)
    │      medir → avaliar → estado → watchdog
    │      ├── Camada de sinal: escala R do Kalman (sempre ativa)
    │      └── Camada de controle: compensação cwnd (CONFIRMED+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              janela deslizante max BW
    │      ├── ucp_update_ecn_ewma()        razão de marcações ECN-CE
    │      ├── ucp_update_ack_aggregation()  janela dupla extra_acked
    │      ├── ucp_update_cycle_phase()     avanço de fase PROBE_BW
    │      ├── ucp_check_full_bw_reached()  detecção de saída do STARTUP
    │      ├── ucp_check_drain()            entrada/saída DRAIN + pulo DRAIN
    │      ├── ucp_update_min_rtt()         Kalman + janela min-RTT + PROBE_RTT
    │      └── Atribuição de ganho específica do modo
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            recuo ECN (apenas cwnd_gain)
    │
    ├──► ucp_set_pacing_rate()              imediato, regra BBR
    │
    └──► ucp_set_cwnd()                    BDP + limites + compensação de agregação
```

## Fluxo Interno do Filtro de Kalman

```
Amostra RTT (rtt_us)
    │
    ├── Inválida (≥0 e < dynamic_max)? Não → descartar
    │
    ├── Partida fria (sample_cnt==0)? Sim → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           (ignora portão máximo RTT)
    │
    ├── Q Adaptativo: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R Adaptativo: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Inovação: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Predizer: p_pred = p_est + Q
    │
    ├── Portão de outlier: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Sim & reject_cnt < max → rejeitar, ++consec_reject_cnt, retornar
    │   └── Sim & reject_cnt ≥ max → aceitar forçadamente (antitrava)
    │
    └── Atualização Kalman:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (fixado não negativo)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Atualização EWMA de jitter
         ├── Atualização EWMA de qdelay
         ├── Estimativa de ruído com correspondência de covariância BBR-S
         └── sample_cnt++
```

## Diagnóstico

Interface de diagnóstico compatível com BBR via `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: estimativa de largura de banda de 64 bits (bytes/s)
bbr_min_rtt:         min_rtt_us atual
bbr_pacing_gain:     ganho de pacing atual (BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       ganho cwnd atual (BBR_UNIT)
```

## Performance Summary

Test environment: China → US LAX, 212ms RTT, 8 parallel flows, 26% packet loss, 1 Gbps shared VPS bottleneck.

| Metric | UCP v1.0 | BBR (control) | Delta |
|--------|----------|---------------|-------|
| Average throughput | 1,010 Mbps | 937 Mbps | **+7.8%** |
| Intra-UCP unfairness | 3.1× | 6.2× (BBR) | **−50%** |
| Worst single flow | 60.6 Mbps | 30.8 Mbps | **+97%** |
| Retransmits | 150K/10s | 137K/10s | +9.5% |
| Round-3 stability | 959 Mbps | 883 Mbps | **+8.6%** |

Retransmits are slightly higher — a trade-off consistent with maintaining high link utilisation under loss. UCP's Kalman-augmented min_rtt estimation provides a more accurate BDP baseline, allowing the algorithm to sustain higher throughput than BBRv1 on the same path.

## Referências

| Tag | Citation / Link |
|-----|----------------|
| BBR | Cardwell et al., "BBR: Congestion-Based Congestion Control", ACM Queue, Vol. 14 No. 5, 2016 — https://dl.acm.org/doi/10.1145/3009824 |
| BBR-S | "BBR-S: A Low-Latency BBR Modification for Fast-Varying Connections", 2021 — https://ieeexplore.ieee.org/document/9438951 |
| RBBR | "RBBR: A Receiver-Driven BBR in QUIC for Low-Latency in Cellular Networks", 2022 — https://ieeexplore.ieee.org/document/9703289 |
| ERCC | "ERCC: Fine-grained RDMA Congestion Control via Kalman Filter-based Multi-bit ECN Feedback Reconstruction", 2025 — https://dl.acm.org/doi/10.1145/3769270.3770124 |
| Linux BBR | Linux kernel BBR reference — https://github.com/torvalds/linux/blob/master/net/ipv4/tcp_bbr.c |
| Google BBR | BBR project page — https://github.com/google/bbr |
| BBRplus | "BBRplus: Adaptive Cycle Randomization, Drain-to-Target, and ACK Aggregation Compensation for BBR Convergence and Stall Prevention" — https://blog.csdn.net/dog250/article/details/80629551 |
| IETF 101 | "BBR Congestion Control Work at Google IETF 101 Update" — https://datatracker.ietf.org/meeting/101/materials/slides-101-iccrg-an-update-on-bbr-work-at-google-00 |
