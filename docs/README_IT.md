[🇺🇸 English](README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Protocollo di Comunicazione Universale)

Modulo di controllo della congestione TCP per ambienti VPS a larghezza di banda condivisa che combina la macchina a stati BBRv1 con un filtro di Kalman per la stima del ritardo di propagazione.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Panoramica dell'Algoritmo

TCP UCP implementa un modulo di controllo della congestione lato mittente per il kernel Linux come modulo caricabile `tcp_ucp.ko`. La funzione di controllo della congestione `ucp_main()` viene invocata ad ogni ACK da `tcp_ack()`, ricevendo una struttura `rate_sample` che contiene campioni di larghezza di banda e RTT del kernel insieme a contatori di consegna e perdita. L'algoritmo opera in due regimi temporali: un **percorso rapido per-ACK** che aggiorna lo stato delle misurazioni e calcola obiettivi istantanei di pacing e finestra, e un **percorso lento per-round** che valuta le condizioni di transizione di stato e ricalcola i guadagni.

La pipeline di misurazione centrale è composta da due componenti:

1. **Filtro di larghezza di banda massima a finestra scorrevole** (`minmax_running_max` da `linux/win_minmax.h`): finestra che copre gli ultimi `ucp_bw_rt_cycle_len` (default 10) round trip. Fornisce la stima `max_bw` compatibile con BBR.

2. **Stimatore del ritardo di propagazione tramite filtro di Kalman**: sostituisce l'RTT minimo a finestra scorrevole di BBRv1. Un filtro di Kalman a stato singolo (Kalman 1960) che opera in unità in virgola fissa di `ucp_kalman_scale` × µs, modellando il ritardo di propagazione reale come una camminata casuale:
   - Stato: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Osservazione: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Convenzioni in virgola fissa: `BW_UNIT = 1 << 24` per la larghezza di banda (segmenti * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` come unità di guadagno adimensionale.

## Macchina a Stati

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

Quattro modalità codificate come campo `mode` a 2 bit in `struct ucp`:

- **STARTUP (0)**: Stato iniziale. `pacing_gain` ≈ 2,885x (`ucp_high_gain_val`), anche `cwnd_gain` a 2,885x. Esplorazione esponenziale della larghezza di banda.
- **DRAIN (1)**: Entrato dopo l'uscita da STARTUP. `pacing_gain` ≈ 0,347x (`ucp_drain_gain_val`), `cwnd_gain` rimane a 2,885x. Drena la coda accumulata durante STARTUP.
- **PROBE_BW (2)**: Stato stazionario. Cicla attraverso una tabella di guadagni a 256 slot (pattern predefinito a 8 fasi ripetuto: 1,25x/0,75x/8×1,0x).
- **PROBE_RTT (3)**: Drena periodicamente il traffico in volo a `ucp_cwnd_min_target` (default 4 segmenti) per ottenere un campione RTT fresco.

### STARTUP → DRAIN

Attivato quando `full_bw_reached` è impostato — dopo `ucp_full_bw_cnt` (default 3) round consecutivi in cui `max_bw` non riesce a crescere di almeno `ucp_full_bw_thresh_val` (default 1,25x) rispetto al picco osservato in precedenza. Il BDP con guadagno 1,0x viene scritto in `snd_ssthresh`. `qdelay_avg` viene azzerato per evitare che l'accumulo della coda di STARTUP influenzi PROBE_BW.

### DRAIN → PROBE_BW

Attivato quando il traffico in volo stimato a EDT ≤ traffico in volo target con guadagno BDP 1,0x. **Ottimizzazione salto DRAIN**: quando il filtro di Kalman è converguto E `qdelay_avg` è inferiore a `ucp_drain_skip_qdelay_us` (default 1000 µs), la fase DRAIN viene saltata — conversione anticipata a PROBE_BW.

All'ingresso di PROBE_BW, l'indice di fase del ciclo viene randomizzato: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (default `len − 1 − rand(8)`), che decorrela i flussi concorrenti che condividono un collegamento congestionato.

### PROBE_BW → PROBE_RTT

Attivato quando l'intervallo del filtro PROBE_RTT scade — il timestamp `min_rtt_stamp` non è stato aggiornato entro l'intervallo calcolato. cwnd viene salvato in `prior_cwnd`, il pacing viene impostato per drenare.

### PROBE_RTT → PROBE_BW

Dopo che il traffico in volo scende a `ucp_cwnd_min_target` o viene osservato un limite di round, persiste per almeno `ucp_probe_rtt_mode_ms_val` (default 200 ms) e almeno un round completo osservato, poi esce. cwnd viene ripristinato ad almeno `prior_cwnd`, il pacing viene temporaneamente sovrascritto con `ucp_high_gain_val` per un rapido riempimento del tubo.

### Recupero e Perdita

- Su TCP_CA_Loss: `full_bw` e `full_bw_cnt` vengono resettati, `round_start` impostato a 1, `packet_conservation` azzerato a 0. Se LT BW non è attivo, inietta un evento di perdita sintetico per attivare il campionamento LT.
- Ingresso recupero (TCP_CA_Recovery): `packet_conservation` abilitato, cwnd = in volo + accusato.
- Uscita recupero: ripristinato a `prior_cwnd`, `packet_conservation` azzerato.
- `ucp_undo_cwnd()`: resetta `full_bw` e `full_bw_cnt` (preservando `full_bw_reached`), azzera lo stato LT BW.

## Misurazioni Principali

### Stima della Larghezza di Banda

Filtro di larghezza di banda massima a finestra scorrevole (`minmax_running_max` da `linux/win_minmax.h`) su `ucp_bw_rt_cycle_len` (default 10) round. bw istantaneo = `delivered × BW_UNIT / interval_us` calcolato per ACK. Alimentato nella finestra scorrevole solo quando non è limitato dall'applicazione o quando bw ≥ bw massimo corrente (regola BBR).

Quando `lt_use_bw` è attivo, la stima attiva della larghezza di banda passa a `lt_bw` (stima di larghezza di banda a lungo termine).

### Filtro di Kalman

Ricorsione di Kalman scalare a stato singolo (complessità O(1)):

```
Predici:
  x_pred = x_est          (transizione di stato identità)
  p_pred = p_est + Q      (predizione covarianza)

Aggiorna:
  innov   = z − x_pred    (innovazione)
  K       = p_pred / (p_pred + R)   (guadagno di Kalman [0,1])
  x_est   = x_pred + K × innov      (aggiornamento stato)
  p_est   = (1 − K) × p_pred        (covarianza a posteriori)
```

**Rumore di processo adattativo Q**:
```
Q_base   = ucp_kalman_q (default 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Rumore di misurazione adattativo R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Cancello outlier**: soglia dinamica `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Applicato solo quando `p_pred ≤ ucp_kalman_converged_p_est_val`. Dopo `ucp_kalman_max_consec_reject` (default 25) rifiuti consecutivi, il campione successivo viene forzatamente accettato per prevenire un blocco auto-rafforzante.

**Stima del rumore tramite covarianza accoppiata (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Modalità di combinazione: modalità 0 = solo euristico, modalità 1 = max (default), modalità 2 = miscela pesata.

**Presa in carico di Kalman**: quando `x_est > 0` e `sample_cnt ≥ ucp_kalman_min_samples` (default 5), `min_rtt_us` viene sostituito da `x_est / ucp_kalman_scale`. `min_rtt_stamp` non viene aggiornato — il trigger dell'intervallo PROBE_RTT rimane indipendente.

**Limite del margine x_est**: Il `model_rtt` derivato da Kalman è limitato a `min_rtt_us × (100 + ucp_kalman_xest_margin_pct) / 100` (predefinito 8%).

## Miglioramenti BBR

### Decadimento del Guadagno

Abilitato dalla bitmap a 256 bit `ucp_cycle_decay_mask[]` per fasi specifiche di PROBE_BW. Formula di decadimento (su campione Kalman accettato):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = scalatura inversa di p_est (BBR_UNIT al massimo)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Scalatura della confidenza di Kalman: quando `p_est > ucp_kalman_converged_p_est`, il decadimento viene proporzionalmente ridotto, evitando un arretramento eccessivo quando il filtro è incerto.

### Arretramento ECN

Condizioni di attivazione (tutte devono essere soddisfatte):
1. `ucp_ecn_enable_val != 0`
2. Kalman converguto (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (marchi CE osservati)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (default 2000 µs)
5. La modalità NON è PROBE_BW (cwnd_gain è fisso a 2x in PROBE_BW)

Durante le fasi di esplorazione (`pacing_gain > BBR_UNIT`), l'arretramento ECN viene graduato da `BBR_UNIT² / pacing_gain` — ~80% di arretramento a sonda 1,25x, ~65% a guadagno STARTUP 2,89x.

Rapporto marchio ECN EWMA: aggiornato ai limiti di round da `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (default 3/4), con delicato decadimento per-ACK di `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (default 31/32) su ogni ACK senza nuovi marchi CE.

### Rilevamento Flusso Singolo

Quando UCP rileva che il flusso è probabilmente da solo nel collo di bottiglia (basso ritardo di coda, basso jitter, nessun marchio ECN, nessuna aggregazione ACK, nessuna larghezza di banda LT), passa automaticamente a una modalità BBR pura:

- `ucp_get_model_rtt()` restituisce direttamente `min_rtt_us` (evitando la stima livellata di Kalman, che ha un piccolo bias positivo dovuto al rumore di misura unilaterale).
- `ucp_ecn_backoff()` è configurabile tramite `ucp_alone_bypass_ecn` (predefinito 1) — su un percorso a flusso singolo, i marchi ECN sono falsi positivi dell'AQM perché non c'è un altro mittente in competizione. Saltarlo corrisponde al comportamento ECN zero di BBR. Impostare a 0 per mantenere il backoff ECN anche in modalità singola (conservativo).
- La condizione LT BW (policer) è configurabile tramite `ucp_alone_bypass_lt_bw` (predefinito 1) — un percorso a flusso singolo non ha policer, quindi LT BW non può attivarsi legittimamente. Saltarla evita uscite spurie dalla modalità singola causate da falsi trigger. Impostare a 0 per il comportamento rigoroso originale.

Ciò elimina il divario di prestazioni in flusso singolo tra UCP e BBR, preservando al contempo il ciclo di protezione completo di UCP (Kalman, arretramento ECN, decadimento del guadagno, larghezza di banda LT) per scenari multi-flusso.

**Isteresi**: L'ingresso richiede `ucp_alone_confirm_rounds` (default 3) round consecutivi qualificati — evitando oscillazioni durante brevi periodi di calma nella competizione multi-flusso ("conservativo per accelerare"). L'uscita è immediata — qualsiasi fallimento di qualifica cancella il flag e resetta il contatore di conferma ("aggressivo per rallentare").

Condizioni di qualifica (tutte e sei devono essere soddisfatte al confine di un turno):
0. Kalman convergente (`sample_cnt >= ucp_kalman_min_samples`) — fidarsi di qdelay/jitter come segnali di coda
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (predefinito 1000 us) — coda quasi vuota
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (predefinito 2000 us) — solo micro-jitter di clock ACK
3. `ecn_ewma == 0` — nessun contrassegno di congestione da AQM
4. `lt_use_bw == 0` — non in modalità a velocità limitata rilevata dal policer
5. `agg_state <= max` secondo `ucp_alone_agg_state_level` (predefinito 1) — tre livelli di rigore di aggregazione ACK: 0 = solo IDLE (più rigido, zero aggregazione), 1 = ≤ SUSPECTED (predefinito, consente aggregazione transitoria), 2 = ≤ CONFIRMED (più permissivo, blocca solo aggregazione persistente)

### Intervallo PROBE_RTT Dinamico

Mappa `p_est` di Kalman su un intervallo PROBE_RTT per connessione:

```
p_est ≤ converged:              interval = dyn_max (default 30s)
p_est ≥ high (= mult × conv):   interval = base (default 10s)
converged < p_est < high:       interpolazione lineare
```

Riduce la frequenza di PROBE_RTT quando la confidenza è alta (`p_est` basso), diminuendo il jitter del throughput su percorsi stabili. Torna all'intervallo classico di 10 secondi quando la confidenza è bassa.

**Jitter di ingresso per flusso**: Per evitare che tutti i flussi coesistenti entrino contemporaneamente in PROBE_RTT (svuotandosi a 4 pacchetti aggregati ~1.8 Mbps e poi ricaricandosi a 2.89×), ogni flusso aggiunge un jitter derivato da hash (distribuzione 0–845 ms) al proprio intervallo PROBE_RTT. Al massimo ~1 flusso è in PROBE_RTT in qualsiasi istante, eliminando il collasso simultaneo svuotamento/ricarica che induce RTO.

### Stima della Larghezza di Banda LT

Stimatore del limite inferiore attivato da perdita. L'intervallo di campionamento copre [4, 16] RTT. Valido quando il rapporto di perdita ≥ 5,9% (`ucp_lt_loss_thresh` default 15/256). Larghezza di banda `bw = delivered × BW_UNIT / interval_us`.

A differenza della media semplice di BBR (`(bw + lt_bw) >> 1`), UCP utilizza un EMA configurabile (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, default 1/2 = 0,5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

L'attivazione differisce da BBR: UCP memorizza `lt_bw` al primo intervallo valido ma NON imposta `lt_use_bw`; è richiesta coerenza con un intervallo precedente — riduce la falsa attivazione da rumore di misurazione.

**Cancello di congestione a doppia soglia**: Prima di impostare `lt_use_bw = 1`, vengono valutati sia un controllo EWMA persistente della coda (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) che un controllo istantaneo della coda basato su SRTT (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, default 5000 µs). Quando viene rilevata congestione, il campionamento LT BW viene interrotto. Il controllo SRTT funziona senza allocazione `ext`, fornendo una rete di sicurezza contro il fallimento dell'allocazione.

Boost della sonda LT BW (`ucp_lt_bw_probe_pct`, default 10%): amplifica `pacing_gain` di `1 + probe_pct/100` su tutte le fasi PROBE_BW. Componente rampa: aumento di `+1% ogni 8 RTT`, limitato a `2 × probe_pct`.

Auto-recupero LT BW (`ucp_lt_restore_ratio_num/den`, default 5/4 = 1,25x): quando `max_bw > lt_bw × ratio` per `ucp_lt_restore_consec_acks` (default 3) ACK consecutivi, LT BW esce automaticamente e la normale esplorazione PROBE_BW riprende.

### Compensazione Basata sulla Confidenza di Aggregazione ACK (ispirata a BBRplus)

Aggiunge un secondo strato con cancello di confidenza sopra lo stimatore tradizionale a doppio slot extra-acked.

**Quattro fattori ortogonali** (ciascuno contribuisce `ucp_agg_factor_weight` punti, default 256):
1. Kalman converguto (`p_est < converged` + `sample_cnt >= min_samples`)
2. Non in recupero perdita (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT entro `min_rtt_us + ucp_agg_factor3_qdelay_us` (default 2ms) dal ritardo di propagazione reale
4. `extra_acked` entro `ucp_agg_factor4_ratio_num/den` (default 1,5x) del massimo finestrato

**Quattro stati**: INATTIVO (< `ucp_agg_thresh_suspected`=256), SOSPETTO (≥256), CONFERMATO (≥512), FIDATO (≥768).

**Strato di segnale** (sempre attivo): la confidenza interpola linearmente il fattore di scala R `[r_min, r_max]`. R sale istantaneamente (risposta rapida), decade al `ucp_agg_r_hysteresis`% (default 75% trattenuto, ~4 RTT per tornare alla baseline) per RTT.

**Strato di controllo** (`agg_state ≥ CONFIRMED`): compensazione cwnd con cancello di sicurezza a cinque livelli:
1. Blocca se il ritardo di coda > `ucp_agg_safety_qdelay_us` (default 4ms)
2. Blocca durante il recupero perdita
3. Blocca se cwnd > `BDP × ucp_agg_safety_bdp_mult` (default 3x)
4. Blocca se in volo > cwnd sicuro + obiettivo segmenti TSO
5. Watchdog: declassa CONFERMATO→SOSPETTO dopo `ucp_agg_max_comp_duration` (default 8) RTT consecutivi

### Reset di qdelay_avg in DRAIN

Alla transizione verso DRAIN, `qdelay_avg` viene azzerato, impedendo alla stima della coda di STARTUP di persistere in PROBE_BW.

### Adattamento del Divisore TSO

`ucp_min_tso_segs()` regola il divisore della soglia di velocità in base allo stato di Kalman:
- Kalman converguto + `jitter_ewma < 1000 µs`: divisore dimezzato (8→4), raffiche TSO più grandi
- `jitter_ewma > 4000 µs`: divisore raddoppiato (8→16), raffiche TSO più piccole per sopprimere il jitter

## Tasso di Pacing e Cwnd

### Tasso di Pacing

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // regolazione guadagno
rate = rate × USEC_PER_SEC >> BW_SCALE            // conversione in bytes/s
rate = rate × margin_div / 100                    // margine di pacing (default 1%, matching BBR)
```

Le variazioni di velocità vengono applicate immediatamente (nessun livellamento), come in BBR (Cardwell et al. 2016). Dopo `full_bw_reached`: tutte le variazioni di velocità vengono scritte immediatamente. In STARTUP/DRAIN: vengono applicati solo gli aumenti (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP base
// limiti traffico in volo (non-STARTUP: clamp lo~hi; STARTUP: solo pavimento lo)
target = quantization_budget(target)              // margine TSO + round pari + bonus fase-0
target += ack_agg_bonus + agg_compensation        // compensazione aggregazione ACK

// progressione cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // convergere al target
else (STARTUP):
    cwnd = cwnd + acked                          // crescita esponenziale

cwnd = max(cwnd, cwnd_min_target)                 // pavimento assoluto 4
modalità PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // traffico in volo minimo
```

## Parametri del Modulo

I parametri sono esposti sotto `/proc/sys/net/ucp/`. Le scritture attivano `ucp_init_module_params()` (validazione + limitazione + calcolo del valore derivato). Le scritture di parametri array attivano `ucp_rebuild_gain_table()`.

### Intervalli PROBE_RTT

| Parametro | Default | Min | Max | Unità | Descrizione |
|-----------|---------|-----|-----|-------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Intervallo PROBE_RTT base |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Limite superiore per percorsi RTT lungo |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Intervallo dinamico max; 0 disabilita |

### Guadagni

| Parametro | Default | Min | Max | Descrizione |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Guadagno cwnd base per PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Moltiplicatore bonus aggregazione ACK |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Guadagno STARTUP (≈2,885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Guadagno DRAIN (≈0,347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Limite inferiore traffico in volo (1,25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Limite superiore traffico in volo (2,0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | Pattern BBRv1 (256 slot) | 0/1 | — | Guadagno pacing per slot |
| `ucp_cycle_decay_mask[8]` | 0 (tutti zero) | 0 | 0x7FFFFFFF | Bitmap decadimento a 256 bit |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Uscita limitata probe-up (0=spento) |

### Kalman Base

| Parametro | Default | Min | Max | Descrizione |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Rumore di processo base Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Rumore di misurazione base R |
| `ucp_kalman_p_est_max` | 1.000.000 | 1 | 100M | p_est massimo assoluto |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | Soglia di convergenza |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | p_est iniziale |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | Pavimento p_est |
| `ucp_kalman_scale` | 1024 | 64 | 1.048.576 | Scala in virgola fissa (potenza di due) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Campioni minimi prima della presa in carico |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Soglia base outlier |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Moltiplicatore Q-boost |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Costante di tempo Q-boost |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Soffitto Q |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Limite scala Q |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Max rifiuti consecutivi prima di forzare accettazione |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Soffitto RTT Kalman |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | Moltiplicatore boost max R |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Moltiplicatore soffitto dinamico RTT |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Divisore RTT adattamento Q |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | Moltiplicatore banda transizione PROBE_RTT |

### Kalman Extra (tipo num/den)

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Moltiplicatore jitter per outlier |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Fattore minimo Q |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Divisore RTT inizializzazione p_est |

### Stima del Rumore BBR-S

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Tasso di apprendimento stima Q |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Tasso di apprendimento stima R |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Modalità combinazione (0=spento, 1=max, 2=media pesata) |
| `ucp_kalman_q_est_max` | 1.000.000.000 | 1-2 Mld | Limite superiore stima Q |
| `ucp_kalman_r_est_max` | 1.000.000.000 | 1-2 Mld | Limite superiore stima R |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Limite inferiore per stima |

### Decadimento del Guadagno (Esplorazione)

| Parametro | Default | Intervallo | Unità | Descrizione |
|-----------|---------|------------|-------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | Soglia decadimento qdelay |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | Scala decadimento qdelay |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Soglia decadimento jitter |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | Scala decadimento jitter |

### R Adattativo (Guidato dal Jitter)

| Parametro | Default | Intervallo | Unità | Descrizione |
|-----------|---------|------------|-------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | Soglia jitter per aumento R |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | Divisore scala aumento R |

### ECN

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | Interruttore principale ECN |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Frazione arretramento ECN |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | Soglia qdelay ECN |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Pesi EWMA ECN |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Decadimento ECN inattivo |

### min_rtt

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Conteggio caduta rapida |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Divisore soglia caduta rapida |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Rapporto caduta persistente |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Rapporto guardia SRTT |

### Larghezza di Banda LT

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTT | Lunghezza intervallo minima |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Moltiplicatore timeout intervallo |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Rapporto perdita minimo |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Tolleranza relativa |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | Tolleranza assoluta |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTT | RTT attivi max LT BW |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | Boost sonda LT BW |

### Auto-Recupero LT

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Rapporto attivazione recupero |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Conteggio ACK consecutivi attivazione |

### Confidenza di Aggregazione ACK

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Interruttore principale |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | Soglia confidenza compensazione cwnd |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % del BDP | Limite compensazione cwnd |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTT | Timeout watchdog |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | Decadimento isteresi R |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Intervallo scala R (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Margine qdelay fattore 3 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Rapporto fattore 4 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Guardia sicurezza 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Moltiplicatore BDP guardia sicurezza |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | Finestra limite extra_acked |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Tasso decadimento watchdog |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTT | Periodo rotazione finestra |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Punteggio per fattore |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Confidenza massima |

### Coefficienti EWMA

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA qdelay |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA jitter |

### Vari

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | Fasi ciclo PROBE_BW (potenza di due) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Offset casuale fase ciclo |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Soglia crescita uscita STARTUP |
| `ucp_full_bw_cnt` | 3 | 1-3 | Round senza crescita per uscire |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Durata permanenza PROBE_RTT |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Margine pacing (0 = nessuno) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segm | Bonus cwnd fase-0 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | round | Lunghezza finestra scorrevole BW |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segm | Cwnd minimo (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | Pavimento min_rtt BDP |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | Soglia EDT quasi-ora |
| `ucp_min_tso_rate` | 1.200.000 | 1-1 Mld | bytes/s | Soglia bassa velocità TSO |
| `ucp_min_tso_rate_div` | 8 | 1-256 | Divisore velocità TSO (base adattativa) |
| `ucp_tso_max_segs` | 127 | 1-65535 | segm | Segmenti TSO massimi |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | Moltiplicatore margine TSO |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Fattore espansione buffer invio |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | Limite epoca ACK |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Finestra max aggregazione ACK |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Soglia RTT lungo |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Divisore intervallo RTT lungo |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Soglia qdelay salto DRAIN |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | round | Round prima di attivare la modalità flusso singolo |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Ritardo di coda max per rilevamento flusso singolo |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | Jitter max per rilevamento flusso singolo |
| ucp_alone_agg_state_level | 1 | 0-2 | — | Rigore di aggregazione (0=solo IDLE, 1=≤SUSPECTED predef., 2=≤CONFIRMED troppo aggressivo) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | Salta backoff ECN in modalità singola (1=salta, 0=attivo) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | Salta condizione LT BW in modalità singola (1=salta, 0=attivo) |

## Percorso Dati

```
ACK Arriva (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► Pipeline confidenza aggregazione ACK (quando ucp_agg_enable)
    │      misurare → valutare → stato → watchdog
    │      ├── Strato segnale: scalatura R Kalman (sempre attivo)
    │      └── Strato controllo: compensazione cwnd (CONFERMATO+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              BW massima a finestra scorrevole
    │      ├── ucp_update_ecn_ewma()        Rapporto marchio ECN-CE
    │      ├── ucp_update_ack_aggregation()  extra_acked a doppia finestra
    │      ├── ucp_update_cycle_phase()     Avanzamento fase PROBE_BW
    │      ├── ucp_check_full_bw_reached()  Rilevamento uscita STARTUP
    │      ├── ucp_check_drain()            Ingresso/uscita DRAIN + salto DRAIN
    │      ├── ucp_update_min_rtt()         Kalman + finestra min-RTT + PROBE_RTT
    │      └── Assegnazione guadagno specifica modalità
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            Arretramento ECN (solo cwnd_gain)
    │
    ├──► ucp_set_pacing_rate()              immediato, regola BBR
    │
    └──► ucp_set_cwnd()                    BDP + limiti + compensazione agg
```

## Flusso Interno del Filtro di Kalman

```
Campione RTT (rtt_us)
    │
    ├── Invalido (≥0 e < dynamic_max)? Sì → scarta
    │
    ├── Avvio a freddo (sample_cnt==0)? Sì → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                              (bypassa cancello max RTT)
    │
    ├── Q adattativo: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R adattativo: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovazione: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Predici: p_pred = p_est + Q
    │
    ├── Cancello outlier: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Sì e reject_cnt < max → rifiuta, ++consec_reject_cnt, ritorna
    │   └── Sì e reject_cnt ≥ max → forza accettazione (anti-blocco)
    │
    └── Aggiornamento Kalman:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (bloccato non negativo)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Aggiornamento EWMA jitter
         ├── Aggiornamento EWMA qdelay
         ├── Stima rumore tramite covarianza accoppiata BBR-S
         └── sample_cnt++
```

## Diagnostica

Interfaccia diagnostica compatibile BBR tramite `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: stima larghezza di banda 64 bit (bytes/s)
bbr_min_rtt:         min_rtt_us corrente
bbr_pacing_gain:     guadagno pacing corrente (BBR_UNIT, 256=1,0x)
bbr_cwnd_gain:       guadagno cwnd corrente (BBR_UNIT)
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

## Riferimenti

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
