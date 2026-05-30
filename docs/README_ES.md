[🇺🇸 English](README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Protocolo Universal de Comunicación)

Módulo de control de congestión TCP para entornos VPS de ancho de banda compartido que combina la máquina de estados de BBRv1 con un filtro de Kalman para la estimación del retardo de propagación.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Resumen del Algoritmo

TCP UCP implementa un módulo de control de congestión del lado del emisor para el kernel de Linux como un módulo cargable `tcp_ucp.ko`. La función de control de congestión `ucp_main()` se invoca en cada ACK desde `tcp_ack()`, recibiendo una estructura `rate_sample` que contiene muestras de ancho de banda y RTT del kernel junto con contadores de entrega y pérdida. El algoritmo opera en dos regímenes temporales: un **camino rápido por-ACK** que actualiza el estado de medición y calcula objetivos instantáneos de pacing y ventana, y un **camino lento por-ronda** que evalúa condiciones de transición de estado y recalcula ganancias.

El pipeline de medición central consta de dos componentes:

1. **Filtro de ancho de banda máximo de ventana deslizante** (`minmax_running_max` de `linux/win_minmax.h`): ventana que cubre las últimas `ucp_bw_rt_cycle_len` (por defecto 10) rondas. Proporciona la estimación `max_bw` compatible con BBR.

2. **Estimador de retardo de propagación por filtro de Kalman**: reemplaza el RTT mínimo de ventana deslizante de BBRv1. Un filtro de Kalman de estado único (Kalman 1960) que opera en unidades de punto fijo de `ucp_kalman_scale` × µs, modelando el retardo de propagación real como un camino aleatorio:
   - Estado: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Observación: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Convenciones de punto fijo: `BW_UNIT = 1 << 24` para ancho de banda (segmentos * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` como unidad de ganancia adimensional.

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

Cuatro modos codificados como el campo `mode` de 2 bits en `struct ucp`:

- **STARTUP (0)**: Estado inicial. `pacing_gain` ≈ 2.885x (`ucp_high_gain_val`), `cwnd_gain` también 2.885x. Sondeo exponencial de ancho de banda.
- **DRAIN (1)**: Ingresado después de la salida de STARTUP. `pacing_gain` ≈ 0.347x (`ucp_drain_gain_val`), `cwnd_gain` permanece en 2.885x. Drena la cola acumulada durante STARTUP.
- **PROBE_BW (2)**: Estado estable. Cicla a través de una tabla de ganancias de 256 ranuras (patrón de 8 fases por defecto repetido: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: Drena periódicamente el tráfico en vuelo a `ucp_cwnd_min_target` (por defecto 4 segmentos) para obtener una muestra RTT fresca.

### STARTUP → DRAIN

Se activa cuando `full_bw_reached` se establece — después de `ucp_full_bw_cnt` (por defecto 3) rondas consecutivas donde `max_bw` no logra crecer al menos `ucp_full_bw_thresh_val` (por defecto 1.25x) en comparación con el pico observado previamente. El BDP con ganancia 1.0x se escribe en `snd_ssthresh`. `qdelay_avg` se reinicia a cero para evitar que la acumulación de la cola de STARTUP afecte a PROBE_BW.

### DRAIN → PROBE_BW

Se activa cuando el tráfico en vuelo estimado a EDT ≤ tráfico en vuelo objetivo con ganancia BDP de 1.0x. **Optimización de salto de DRAIN**: cuando el filtro de Kalman está convergido Y `qdelay_avg` está por debajo de `ucp_drain_skip_qdelay_us` (por defecto 1000 µs), la fase DRAIN se omite — se convierte temprano a PROBE_BW.

Al ingresar a PROBE_BW, el índice de fase del ciclo se aleatoriza: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (por defecto `len − 1 − rand(8)`), lo que descorrelaciona flujos concurrentes que comparten un enlace congestionado.

### PROBE_BW → PROBE_RTT

Se activa cuando el intervalo del filtro PROBE_RTT expira — la marca de tiempo `min_rtt_stamp` no se ha actualizado dentro del intervalo calculado. cwnd se guarda en `prior_cwnd`, el pacing se establece para drenar.

### PROBE_RTT → PROBE_BW

Después de que el tráfico en vuelo cae a `ucp_cwnd_min_target` o se observa un límite de ronda, persiste durante al menos `ucp_probe_rtt_mode_ms_val` (por defecto 200 ms) y al menos una ronda completa observada, luego sale. cwnd se restaura al menos a `prior_cwnd`, el pacing se sobrescribe temporalmente con `ucp_high_gain_val` para un relleno rápido de tubería.

### Recuperación y Pérdida

- En TCP_CA_Loss: `full_bw` y `full_bw_cnt` se reinician, `round_start` se establece en 1, `packet_conservation` se limpia a 0. Si LT BW no está activo, inyecta un evento de pérdida sintético para activar el muestreo LT.
- Entrada de recuperación (TCP_CA_Recovery): `packet_conservation` habilitado, cwnd = en vuelo + acusado.
- Salida de recuperación: se restaura a `prior_cwnd`, `packet_conservation` limpio.
- `ucp_undo_cwnd()`: reinicia `full_bw` y `full_bw_cnt` (preservando `full_bw_reached`), limpia el estado de LT BW.

## Mediciones Centrales

### Estimación de Ancho de Banda

Filtro de ancho de banda máximo de ventana deslizante (`minmax_running_max` de `linux/win_minmax.h`) sobre `ucp_bw_rt_cycle_len` (por defecto 10) rondas. bw instantáneo = `delivered × BW_UNIT / interval_us` calculado por ACK. Se alimenta a la ventana deslizante solo cuando no está limitado por aplicación o cuando bw ≥ bw máximo actual (regla BBR).

Cuando `lt_use_bw` está activo, la estimación de ancho de banda activa cambia a `lt_bw` (estimación de ancho de banda a largo plazo).

### Filtro de Kalman

Recursión de Kalman escalar de estado único (complejidad O(1)):

```
Predecir:
  x_pred = x_est          (transición de estado identidad)
  p_pred = p_est + Q      (predicción de covarianza)

Actualizar:
  innov   = z − x_pred    (innovación)
  K       = p_pred / (p_pred + R)   (ganancia de Kalman [0,1])
  x_est   = x_pred + K × innov      (actualización de estado)
  p_est   = (1 − K) × p_pred        (covarianza posterior)
```

**Ruido de proceso adaptativo Q**:
```
Q_base   = ucp_kalman_q (por defecto 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Ruido de medición adaptativo R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: cuando `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Compuerta de valores atípicos**: umbral dinámico `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Se aplica solo cuando `p_pred ≤ ucp_kalman_converged_p_est_val`. Después de `ucp_kalman_max_consec_reject` (por defecto 25) rechazos consecutivos, la siguiente muestra se fuerza a aceptar para evitar un bloqueo auto-reforzante.

**Estimación de ruido por covarianza coincidente (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Modo de combinación: modo 0 = solo heurístico, modo 1 = max (por defecto), modo 2 = mezcla ponderada.

**Toma de control de Kalman**: cuando `x_est > 0` y `sample_cnt ≥ ucp_kalman_min_samples` (por defecto 5), `min_rtt_us` se reemplaza por `x_est / ucp_kalman_scale`. `min_rtt_stamp` no se actualiza — el disparador del intervalo PROBE_RTT permanece independiente.

Modelo min-rtt x_est: El model_rtt derivado de Kalman usa min(x_est_us, min_rtt_us) — el menor de los dos.

## Mejoras BBR

### Decaimiento de Ganancia

Habilitado por el mapa de bits de 256 bits `ucp_cycle_decay_mask[]` para fases específicas de PROBE_BW. Fórmula de decaimiento (en muestra de Kalman aceptada):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = escalado inverso de p_est (BBR_UNIT a pleno)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Escalado de confianza de Kalman: cuando `p_est > ucp_kalman_converged_p_est`, el decaimiento se reduce proporcionalmente, evitando una retroceso excesivo cuando el filtro es incierto.

### Retroceso ECN

Condiciones de activación (todas deben cumplirse):
1. `ucp_ecn_enable_val != 0`
2. Kalman convergido (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (marcas CE observadas)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (por defecto 2000 µs)
5. El modo NO es PROBE_BW (cwnd_gain es fijo en 2x en PROBE_BW)

Durante las fases de sondeo (`pacing_gain > BBR_UNIT`), el retroceso ECN es graduado por `BBR_UNIT² / pacing_gain` — ~80% de retroceso en sonda 1.25x, ~65% en ganancia STARTUP 2.89x.

Relación de marca ECN EWMA: se actualiza en los límites de ronda por `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (por defecto 3/4), con decaimiento suave por-ACK de `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (por defecto 31/32) en cada ACK sin nuevas marcas CE.

### Detección de Flujo Único

Cuando UCP detecta que el flujo probablemente está solo en el cuello de botella (bajo retardo de cola, baja fluctuación, sin marcas ECN, sin agregación de ACK, sin ancho de banda LT), realiza una transición automática a un modo BBR puro:

- `ucp_get_model_rtt()` devuelve `min_rtt_us` directamente (evitando la estimación suavizada de Kalman, que tiene un pequeño sesgo positivo debido al ruido de medición unilateral).
- `ucp_ecn_backoff()` se puede configurar mediante `ucp_alone_bypass_ecn` (predeterminado 1) — en una ruta de flujo único, las marcas ECN son falsos positivos del AQM porque no hay otro emisor compitiendo. Saltarlo iguala el comportamiento ECN cero de BBR. Configúrelo en 0 para mantener el backoff ECN incluso en modo individual (conservador).
- La condición LT BW (policer) se puede configurar mediante `ucp_alone_bypass_lt_bw` (predeterminado 1) — una ruta de flujo único no tiene policer, por lo que LT BW no puede activarse legítimamente. Saltarlo evita salidas espurias del modo individual por falsos disparos. Configúrelo en 0 para el comportamiento estricto original.

Esto elimina la brecha de rendimiento en flujo único entre UCP y BBR, preservando al mismo tiempo el bucle de protección completo de UCP (Kalman, retroceso ECN, decaimiento de ganancia, ancho de banda LT) para escenarios de múltiples flujos.

**Histéresis**: La entrada requiere `ucp_alone_confirm_rounds` (predeterminado 3) rondas consecutivas calificadas — evitando oscilaciones durante breves períodos de calma en la competencia de múltiples flujos ("conservador para acelerar"). La salida es inmediata — cualquier fallo de calificación borra la bandera y restablece el contador de confirmación ("agresivo para frenar").

Condiciones de calificación (las seis deben cumplirse en un límite de ronda):
0. Kalman convergido (`sample_cnt >= ucp_kalman_min_samples`) — confiar en qdelay/jitter como señales de cola
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (predeterminado 1000 us) — cola casi vacía
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (predeterminado 2000 us) — solo micro-fluctuación de reloj ACK
3. `ecn_ewma == 0` — sin marcas de congestión de AQM
4. `lt_use_bw == 0` — no en modo de tasa limitada detectado por el policer
5. `agg_state <= max` según `ucp_alone_agg_state_level` (predeterminado 1) — tres niveles de rigor de agregación ACK: 0 = solo IDLE (más estricto, cero agregación), 1 = ≤ SUSPECTED (predeterminado, permite agregación transitoria), 2 = ≤ CONFIRMED (más permisivo, solo bloquea agregación persistente)

### Intervalo PROBE_RTT Dinámico

Mapa `p_est` de Kalman a un intervalo PROBE_RTT por conexión:

```
p_est ≤ converged:              interval = dyn_max (por defecto 30s)
p_est ≥ high (= mult × conv):   interval = base (por defecto 10s)
converged < p_est < high:       interpolación lineal
```

Reduce la frecuencia de PROBE_RTT cuando la confianza es alta (`p_est` bajo), reduciendo la fluctuación de rendimiento en rutas estables. Vuelve al intervalo clásico de 10 segundos cuando la confianza es baja.

**Jitter de entrada por flujo**: Para evitar que todos los flujos coexistentes entren en PROBE_RTT simultáneamente (drenando a 4 paquetes agregados ~1.8 Mbps y luego recargando a 2.89×), cada flujo agrega un jitter derivado de hash (distribución de 0–845 ms) a su intervalo PROBE_RTT. Como máximo ~1 flujo está en PROBE_RTT en cualquier instante, eliminando el colapso simultáneo de drenaje/recarga que induce RTO.

### Estimación de Ancho de Banda LT

Estimador de límite inferior activado por pérdida. El intervalo de muestreo abarca [4, 16] RTTs. Válido cuando la relación de pérdida ≥ 5.9% (`ucp_lt_loss_thresh` por defecto 15/256). Ancho de banda `bw = delivered × BW_UNIT / interval_us`.

A diferencia del promedio simple de BBR (`(bw + lt_bw) >> 1`), UCP usa un EMA configurable (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, por defecto 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

La activación difiere de BBR: UCP almacena `lt_bw` en el primer intervalo válido pero NO establece `lt_use_bw`; se requiere consistencia con un intervalo anterior — reduce la activación falsa por ruido de medición.

**Compuerta de congestión de doble umbral**: Antes de establecer `lt_use_bw = 1`, se evalúan tanto una verificación de cola EWMA persistente (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) como una verificación de cola instantánea basada en SRTT (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, predeterminado 5000 µs). Cuando se detecta congestión, el muestreo LT BW se aborta. La verificación SRTT funciona sin asignación `ext`, proporcionando una red de seguridad contra fallas de asignación.

Impulso de sonda LT BW (`ucp_lt_bw_probe_pct`, por defecto 10%): amplifica `pacing_gain` por `1 + probe_pct/100` en todas las fases de PROBE_BW. Componente de rampa: aumento de `+1% por 8 RTTs`, limitado a `2 × probe_pct`.

Auto-recuperación LT BW (`ucp_lt_restore_ratio_num/den`, por defecto 5/4 = 1.25x): cuando `max_bw > lt_bw × ratio` durante `ucp_lt_restore_consec_acks` (por defecto 3) ACKs consecutivos, LT BW sale automáticamente y se reanuda el sondeo normal de PROBE_BW.

### Compensación Basada en Confianza de Agregación ACK (inspirado en BBRplus)

Agrega una segunda capa con compuerta de confianza sobre el estimador tradicional de doble ranura extra-acked.

**Cuatro factores ortogonales** (cada uno contribuye `ucp_agg_factor_weight` puntos, por defecto 256):
1. Kalman convergido (`p_est < converged` + `sample_cnt >= min_samples`)
2. No en recuperación de pérdida (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT dentro de `min_rtt_us + ucp_agg_factor3_qdelay_us` (por defecto 2ms) del retardo de propagación real
4. `extra_acked` dentro de `ucp_agg_factor4_ratio_num/den` (por defecto 1.5x) del máximo ventaneado

**Cuatro estados**: IDLE (< `ucp_agg_thresh_suspected`=256), SOSPECHOSO (≥256), CONFIRMADO (≥512), CONFIABLE (≥768).

**Capa de señal** (siempre activa): la confianza interpola linealmente el factor de escalado R `[r_min, r_max]`. R sube instantáneamente (respuesta rápida), decae a `ucp_agg_r_hysteresis`% (por defecto 75% retenido, ~4 RTTs a la línea base) por RTT.

**Capa de control** (`agg_state ≥ CONFIRMED`): compensación de cwnd con compuerta de seguridad de cinco capas:
1. Bloquea si el retardo de cola > `ucp_agg_safety_qdelay_us` (por defecto 4ms)
2. Bloquea durante la recuperación de pérdida
3. Bloquea si cwnd > `BDP × ucp_agg_safety_bdp_mult` (por defecto 3x)
4. Bloquea si en vuelo > cwnd seguro + objetivo de segmentos TSO
5. Vigilante: degrada CONFIRMADO→SOSPECHOSO después de `ucp_agg_max_comp_duration` (por defecto 8) RTTs consecutivos

### Reinicio de qdelay_avg en DRAIN

En la transición a DRAIN, `qdelay_avg` se reinicia a cero, evitando que la estimación de cola de STARTUP persista en PROBE_BW.

### Adaptación del Divisor TSO

`ucp_min_tso_segs()` ajusta el divisor del umbral de tasa basado en el estado de Kalman:
- Kalman convergido + `jitter_ewma < 1000 µs`: divisor reducido a la mitad (8→4), ráfagas TSO más grandes
- `jitter_ewma > 4000 µs`: divisor duplicado (8→16), ráfagas TSO más pequeñas para suprimir la fluctuación

## Tasa de Pacing y Cwnd

### Tasa de Pacing

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // ajuste de ganancia
rate = rate × USEC_PER_SEC >> BW_SCALE            // convertir a bytes/s
rate = rate × margin_div / 100                    // margen de pacing (por defecto 1%, matching BBR)
```

Los cambios de tasa se aplican inmediatamente (sin suavizado), coincidiendo con BBR (Cardwell et al. 2016). Después de `full_bw_reached`: todos los cambios de tasa se escriben inmediatamente. En STARTUP/DRAIN: solo se aplican aumentos (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP base
// límites de tráfico en vuelo (no-STARTUP: pinza lo~hi; STARTUP: solo piso lo)
target = quantization_budget(target)              // espacio libre TSO + ronda par + bonificación fase-0
target += ack_agg_bonus + agg_compensation        // compensación de agregación ACK

// progresión de cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // converger al objetivo
else (STARTUP):
    cwnd = cwnd + acked                          // crecimiento exponencial

cwnd = max(cwnd, cwnd_min_target)                 // piso absoluto 4
modo PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // tráfico en vuelo mínimo
```

## Parámetros del Módulo

Los parámetros se exponen bajo `/proc/sys/net/ucp/`. Las escrituras activan `ucp_init_module_params()` (validación + sujeción + cálculo de valor derivado). Las escrituras de parámetros de matriz activan `ucp_rebuild_gain_table()`.

### Intervalos PROBE_RTT

| Parámetro | Por defecto | Mín | Máx | Unidad | Descripción |
|-----------|-------------|-----|-----|--------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Intervalo base PROBE_RTT |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Límite superior para rutas de RTT largo |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Intervalo dinámico máximo; 0 deshabilita |

### Ganancias

| Parámetro | Por defecto | Mín | Máx | Descripción |
|-----------|-------------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Ganancia de cwnd base para PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Multiplicador de bonificación de agregación ACK |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Ganancia STARTUP (≈2.885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Ganancia DRAIN (≈0.347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Límite inferior de tráfico en vuelo (1.25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Límite superior de tráfico en vuelo (2.0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | Patrón BBRv1 (256 ranuras) | 0/1 | — | Ganancia de pacing por ranura |
| `ucp_cycle_decay_mask[8]` | 0 (todos cero) | 0 | 0x7FFFFFFF | Mapa de bits de decaimiento de 256 bits |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Salida limitada de sondeo ascendente (0=apagado) |

### Kalman Base

| Parámetro | Por defecto | Mín | Máx | Descripción |
|-----------|-------------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Ruido de proceso base Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Ruido de medición base R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est máximo absoluto |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | Umbral de convergencia |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | p_est inicial |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est mínimo |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | Escala de punto fijo (potencia de dos) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Muestras mínimas antes de toma de control |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Umbral base de valores atípicos |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Multiplicador Q-boost |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Constante de tiempo Q-boost |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Techo Q |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Límite de escala Q |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Máximo de rechazos consecutivos antes de forzar aceptación |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Techo de RTT de Kalman |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | Multiplicador de impulso máximo R |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Multiplicador de techo dinámico RTT |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Divisor RTT de adaptación Q |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | Multiplicador de banda de transición PROBE_RTT |

### Kalman Extras (tipo num/den)

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Multiplicador de fluctuación para valores atípicos |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Factor mínimo Q |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Divisor RTT de inicialización p_est |

### Estimación de Ruido BBR-S

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Tasa de aprendizaje de estimación Q |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Tasa de aprendizaje de estimación R |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Modo de combinación (0=off, 1=max, 2=promedio ponderado) |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Límite superior de estimación Q |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | Límite superior de estimación R |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Límite inferior por estimación |

### Decaimiento de Ganancia (Sondeo)

| Parámetro | Por defecto | Rango | Unidad | Descripción |
|-----------|-------------|-------|--------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | Umbral de decaimiento qdelay |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | Escala de decaimiento qdelay |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Umbral de decaimiento de fluctuación |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | Escala de decaimiento de fluctuación |

### R Adaptativo (Impulsado por Fluctuación)

| Parámetro | Por defecto | Rango | Unidad | Descripción |
|-----------|-------------|-------|--------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | Umbral de fluctuación para aumento de R |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | Divisor de escala de aumento R |

### ECN

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | Interruptor maestro ECN |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Fracción de retroceso ECN |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | Umbral qdelay ECN |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Pesos EWMA ECN |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Decaimiento ECN inactivo |

### min_rtt

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Contador de caída rápida |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Divisor de umbral de caída rápida |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Relación de caída persistente |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Relación de guarda SRTT |

### Ancho de Banda LT

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Longitud mínima de intervalo |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Multiplicador de tiempo de espera de intervalo |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Relación de pérdida mínima |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Tolerancia relativa |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | Tolerancia absoluta |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | RTTs activos máximos de LT BW |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | Impulso de sonda LT BW |

### Auto-Recuperación LT

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Relación de activación de recuperación |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Conteo de ACKs consecutivos de activación |

### Confianza de Agregación ACK

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Interruptor maestro |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | Umbral de confianza de compensación cwnd |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % de BDP | Límite de compensación cwnd |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | Tiempo de espera del vigilante |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | Decaimiento de histéresis R |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Rango de escalado R (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Margen qdelay del factor 3 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Relación del factor 4 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Guarda de seguridad 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Multiplicador BDP de guarda de seguridad |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | Ventana de límite extra_acked |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Tasa de decaimiento del vigilante |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Período de rotación de ventana |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Puntuación por factor |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Confianza máxima |

### Coeficientes EWMA

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA qdelay |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA de fluctuación |

### Misceláneos

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | Fases del ciclo PROBE_BW (potencia de dos) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Desplazamiento aleatorio de fase de ciclo |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Umbral de crecimiento de salida de STARTUP |
| `ucp_full_bw_cnt` | 3 | 1-3 | Rondas sin crecimiento para salir |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Duración de permanencia en PROBE_RTT |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Margen de pacing (0 = ninguno) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | Bonificación cwnd de fase 0 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | rondas | Longitud de ventana deslizante de BW |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | Cwnd mínimo (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | Piso min_rtt de BDP |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | Umbral EDT casi-ahora |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | Umbral de tasa baja TSO |
| `ucp_min_tso_rate_div` | 8 | 1-256 | Divisor de tasa TSO (base adaptativa) |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | Segmentos TSO máximos |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | Multiplicador de espacio libre TSO |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Factor de expansión de búfer de envío |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | Límite de época ACK |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Ventana máxima de agregación ACK |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Umbral de RTT largo |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Divisor de intervalo de RTT largo |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Umbral qdelay de salto de DRAIN |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | rondas | Rondas antes de activar el modo de flujo único |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Retardo máximo de cola para detección de flujo único |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | Fluctuación máxima para detección de flujo único |
| ucp_alone_agg_state_level | 1 | 0-2 | — | Rigor de agregación (0=solo IDLE, 1=≤SUSPECTED predet., 2=≤CONFIRMED demasiado agresivo) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | Omitir backoff ECN en modo individual (1=omitir, 0=activo) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | Omitir condición LT BW en modo individual (1=omitir, 0=activo) |

## Ruta de Datos

```
Llega ACK (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► Pipeline de confianza de agregación ACK (cuando ucp_agg_enable)
    │      medir → evaluar → estado → vigilante
    │      ├── Capa de señal: escalado R de Kalman (siempre activa)
    │      └── Capa de control: compensación cwnd (CONFIRMADO+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              BW máximo de ventana deslizante
    │      ├── ucp_update_ecn_ewma()        Relación de marca ECN-CE
    │      ├── ucp_update_ack_aggregation()  extra_acked de doble ventana
    │      ├── ucp_update_cycle_phase()     Avance de fase PROBE_BW
    │      ├── ucp_check_full_bw_reached()  Detección de salida de STARTUP
    │      ├── ucp_check_drain()            Entrada/salida de DRAIN + salto de DRAIN
    │      ├── ucp_update_min_rtt()         Kalman + ventana min-RTT + PROBE_RTT
    │      └── Asignación de ganancia específica del modo
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            Retroceso ECN (solo cwnd_gain)
    │
    ├──► ucp_set_pacing_rate()              inmediato, regla BBR
    │
    └──► ucp_set_cwnd()                    BDP + límites + compensación agg
```

## Flujo Interno del Filtro de Kalman

```
Muestra RTT (rtt_us)
    │
    ├── ¿Inválida (≥0 y < dynamic_max)? Sí → descartar
    │
    ├── ¿Arranque en frío (sample_cnt==0)? Sí → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                              (omite compuerta máxima RTT)
    │
    ├── Q adaptativo: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R adaptativo: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovación: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Predecir: p_pred = p_est + Q
    │
    ├── Compuerta de valores atípicos: ¿|innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Sí y reject_cnt < max → rechazar, ++consec_reject_cnt, retornar
    │   └── Sí y reject_cnt ≥ max → forzar aceptación (anti-bloqueo)
    │
    └── Actualización de Kalman:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (sujeto a no negativo)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Actualización EWMA de fluctuación
         ├── Actualización EWMA de qdelay
         ├── Estimación de ruido por covarianza coincidente BBR-S
         └── sample_cnt++
```

## Diagnósticos

Interfaz de diagnóstico compatible con BBR a través de `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: estimación de ancho de banda de 64 bits (bytes/s)
bbr_min_rtt:         min_rtt_us actual
bbr_pacing_gain:     ganancia de pacing actual (BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       ganancia de cwnd actual (BBR_UNIT)
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

---

*UCP v1.0 — construido sobre BBRv1 (Cardwell et al. 2016, ACM Queue) y el filtro de Kalman (Kalman 1960).*

## Referencias

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
