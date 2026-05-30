[🇺🇸 English](README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Universelles Kommunikationsprotokoll)

TCP-Überlastungssteuerungsmodul für Shared-Bandwidth-VPS-Umgebungen, das die BBRv1-Zustandsmaschine mit einem Kalman-Filter zur Schätzung der Ausbreitungsverzögerung kombiniert.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Algorithmenübersicht

TCP UCP implementiert ein senderseitiges Überlastungssteuerungsmodul für den Linux-Kernel als ladbares `tcp_ucp.ko`. Die Überlastungssteuerungsfunktion `ucp_main()` wird bei jedem ACK von `tcp_ack()` aufgerufen und erhält eine `rate_sample`-Struktur, die Kernel-Bandbreiten- und RTT-Messwerte sowie Liefer- und Verlustzähler enthält. Der Algorithmus arbeitet in zwei zeitlichen Regimen: einem **pro-ACK-Schnellpfad**, der den Messzustand aktualisiert und sofortige Pacing- und Fensterziele berechnet, und einem **pro-Runde-Langsampfad**, der Zustandsübergangsbedingungen auswertet und Verstärkungen neu berechnet.

Die zentrale Messpipeline besteht aus zwei Komponenten:

1. **Maximalbandbreitenfilter mit gleitendem Fenster** (`minmax_running_max` aus `linux/win_minmax.h`): Fenster über die letzten `ucp_bw_rt_cycle_len` (Standard 10) Umläufe. Liefert die BBR-kompatible `max_bw`-Schätzung.

2. **Kalman-Filter-Ausbreitungsverzögerungsschätzer**: ersetzt BBRv1's gleitendes Fenster-Minimum-RTT. Ein Einzustands-Kalman-Filter (Kalman 1960), der in `ucp_kalman_scale` × µs Festkomma-Einheiten arbeitet und die wahre Ausbreitungsverzögerung als Zufallsbewegung modelliert:
   - Zustand: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Beobachtung: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Festkomma-Konventionen: `BW_UNIT = 1 << 24` für Bandbreite (Segmente * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` als dimensionslose Verstärkungseinheit.

## Zustandsmaschine

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

Vier Modi, kodiert als 2-Bit-`mode`-Feld in `struct ucp`:

- **STARTUP (0)**: Anfangszustand. `pacing_gain` ≈ 2,885x (`ucp_high_gain_val`), `cwnd_gain` ebenfalls 2,885x. Exponentielle Bandbreitenerkundung.
- **DRAIN (1)**: Wird nach STARTUP-Austritt betreten. `pacing_gain` ≈ 0,347x (`ucp_drain_gain_val`), `cwnd_gain` bleibt bei 2,885x. Entleert die während STARTUP aufgebaute Warteschlange.
- **PROBE_BW (2)**: Stationärer Zustand. Durchläuft eine 256-Slot-Verstärkungstabelle (Standard 8-Phasen-Muster wiederholt: 1,25x/0,75x/8×1,0x).
- **PROBE_RTT (3)**: Entleert periodisch den Inflight-Verkehr auf `ucp_cwnd_min_target` (Standard 4 Segmente), um eine frische RTT-Messung zu erhalten.

### STARTUP → DRAIN

Ausgelöst, wenn `full_bw_reached` gesetzt ist — nach `ucp_full_bw_cnt` (Standard 3) aufeinanderfolgenden Runden, in denen `max_bw` nicht um mindestens `ucp_full_bw_thresh_val` (Standard 1,25x) gegenüber dem zuvor beobachteten Spitzenwert wächst. Das BDP bei 1,0x Verstärkung wird in `snd_ssthresh` geschrieben. `qdelay_avg` wird auf Null zurückgesetzt, um zu verhindern, dass der STARTUP-Warteschlangenaufbau PROBE_BW beeinflusst.

### DRAIN → PROBE_BW

Ausgelöst, wenn der geschätzte Inflight-Verkehr bei EDT ≤ Ziel-Inflight bei 1,0x BDP-Verstärkung. **Drain-Überspring-Optimierung**: wenn der Kalman-Filter konvergiert ist UND `qdelay_avg` unter `ucp_drain_skip_qdelay_us` (Standard 1000 µs) liegt, wird die DRAIN-Phase übersprungen — frühzeitige Umwandlung zu PROBE_BW.

Beim Eintritt in PROBE_BW wird der Zyklenphasenindex randomisiert: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (Standard `len − 1 − rand(8)`), was parallele Ströme, die sich einen Engpasslink teilen, dekorreliert.

### PROBE_BW → PROBE_RTT

Ausgelöst, wenn das PROBE_RTT-Filterintervall abläuft — der Zeitstempel `min_rtt_stamp` wurde innerhalb des berechneten Intervalls nicht aktualisiert. cwnd wird in `prior_cwnd` gespeichert, Pacing wird auf Entleeren gesetzt.

### PROBE_RTT → PROBE_BW

Nachdem der Inflight-Verkehr auf `ucp_cwnd_min_target` fällt oder eine Rundengrenze beobachtet wird, besteht für mindestens `ucp_probe_rtt_mode_ms_val` (Standard 200 ms) und mindestens eine beobachtete vollständige Runde, dann Austritt. cwnd wird auf mindestens `prior_cwnd` wiederhergestellt, Pacing wird vorübergehend mit `ucp_high_gain_val` für schnelles Rohrfüllen überschrieben.

### Wiederherstellung und Verlust

- Bei TCP_CA_Loss: `full_bw` und `full_bw_cnt` werden zurückgesetzt, `round_start` auf 1 gesetzt, `packet_conservation` auf 0 gelöscht. Wenn LT BW nicht aktiv ist, wird ein synthetisches Verlustereignis injiziert, um die LT-Abtastung auszulösen.
- Wiederherstellungseintritt (TCP_CA_Recovery): `packet_conservation` aktiviert, cwnd = Inflight + bestätigt.
- Wiederherstellungsaustritt: auf `prior_cwnd` zurückgesetzt, `packet_conservation` gelöscht.
- `ucp_undo_cwnd()`: setzt `full_bw` und `full_bw_cnt` zurück (unter Beibehaltung von `full_bw_reached`), löscht den LT-BW-Zustand.

## Kernmessungen

### Bandbreitenschätzung

Maximalbandbreitenfilter mit gleitendem Fenster (`minmax_running_max` aus `linux/win_minmax.h`) über `ucp_bw_rt_cycle_len` (Standard 10) Runden. Momentane Bandbreite = `delivered × BW_UNIT / interval_us`, pro ACK berechnet. Wird nur dann in das gleitende Fenster eingespeist, wenn die Anwendung nicht begrenzt ist oder wenn die Bandbreite ≥ aktuelle Maximalbandbreite ist (BBR-Regel).

Wenn `lt_use_bw` aktiv ist, wechselt die aktive Bandbreitenschätzung zu `lt_bw` (Langzeit-Bandbreitenschätzung).

### Kalman-Filter

Skalare Einzustands-Kalman-Rekursion (O(1)-Komplexität):

```
Vorhersage:
  x_pred = x_est          (Identitätszustandsübergang)
  p_pred = p_est + Q      (Kovarianzvorhersage)

Aktualisierung:
  innov   = z − x_pred    (Innovation)
  K       = p_pred / (p_pred + R)   (Kalman-Verstärkung [0,1])
  x_est   = x_pred + K × innov      (Zustandsaktualisierung)
  p_est   = (1 − K) × p_pred        (posteriore Kovarianz)
```

**Adaptives Prozessrauschen Q**:
```
Q_base   = ucp_kalman_q (Standard 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Adaptives Messrauschen R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: wenn `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Ausreißer-Sperre**: dynamischer Schwellwert `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Wird nur angewendet, wenn `p_pred ≤ ucp_kalman_converged_p_est_val`. Nach `ucp_kalman_max_consec_reject` (Standard 25) aufeinanderfolgenden Ablehnungen wird die nächste Messung zwangsweise akzeptiert, um eine sich selbst verstärkende Blockade zu verhindern.

**Kovarianz-angepasste Rauschschätzung (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Kombinationsmodus: Modus 0 = nur heuristisch, Modus 1 = max (Standard), Modus 2 = gewichtete Mischung.

**Kalman-Übernahme**: wenn `x_est > 0` und `sample_cnt ≥ ucp_kalman_min_samples` (Standard 5), wird `min_rtt_us` durch `x_est / ucp_kalman_scale` ersetzt. `min_rtt_stamp` wird nicht aktualisiert — der PROBE_RTT-Intervallauslöser bleibt unabhängig.

x_est Min-RTT-Modell: Das Kalman-abgeleitete model_rtt verwendet min(x_est_us, min_rtt_us) — den kleineren der beiden Werte.

## BBR-Erweiterungen

### Verstärkungsabfall

Aktiviert durch die 256-Bit-Bitmap `ucp_cycle_decay_mask[]` für bestimmte PROBE_BW-Phasen. Abfallformel (bei akzeptierter Kalman-Messung):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = inverse Skalierung von p_est (BBR_UNIT bei voll)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Kalman-Konfidenzskalierung: wenn `p_est > ucp_kalman_converged_p_est`, wird der Abfall proportional reduziert, was übermäßigen Rückgang bei unsicherem Filter vermeidet.

### ECN-Rücknahme

Aktivierungsbedingungen (alle müssen erfüllt sein):
1. `ucp_ecn_enable_val != 0`
2. Kalman konvergiert (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (CE-Markierungen beobachtet)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (Standard 2000 µs)
5. Modus ist NICHT PROBE_BW (cwnd_gain ist in PROBE_BW fest auf 2x)

Während der Erkundungsphasen (`pacing_gain > BBR_UNIT`) wird die ECN-Rücknahme durch `BBR_UNIT² / pacing_gain` abgestuft — ~80% Rücknahme bei 1,25x-Sonde, ~65% bei 2,89x-STARTUP-Verstärkung.

ECN-Markierungsverhältnis EWMA: wird an Rundengrenzen durch `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (Standard 3/4) aktualisiert, mit sanftem pro-ACK-Abfall von `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (Standard 31/32) bei jedem ACK ohne neue CE-Markierungen.

### Einzelfluss-Erkennung

Wenn UCP erkennt, dass der Fluss wahrscheinlich allein am Engpass ist (niedrige Warteschlangenverzögerung, niedriger Jitter, keine ECN-Markierungen, keine ACK-Aggregation, keine LT-Bandbreite), wechselt es automatisch in einen reinen BBR-Modus:

- `ucp_get_model_rtt()` gibt direkt `min_rtt_us` zurück (vermeidet die geglättete Kalman-Schätzung, die aufgrund des einseitigen Messrauschens eine kleine positive Verzerrung aufweist).
- `ucp_ecn_backoff()` ist über `ucp_alone_bypass_ecn` (Standard 1) konfigurierbar — auf einem Einzelfluss-Pfad sind ECN-Markierungen Fehlalarme des AQM, da kein anderer Sender konkurriert. Das Überspringen entspricht dem Null-ECN-Verhalten von BBR. Auf 0 setzen, um ECN-Backoff auch im Alleinmodus beizubehalten (konservativ).
- Die LT BW Bedingung (Policer) ist über `ucp_alone_bypass_lt_bw` (Standard 1) konfigurierbar — ein Einzelfluss-Pfad hat keinen Policer, sodass LT BW nicht legitim auslösen kann. Das Überspringen verhindert ein unerwünschtes Verlassen des Alleinmodus durch Fehlauslösungen. Auf 0 setzen für das ursprüngliche strenge Verhalten.

Dies beseitigt die Leistungslücke bei Einzelflüssen zwischen UCP und BBR, während die vollständige Schutzschleife von UCP (Kalman, ECN-Rücknahme, Verstärkungsabfall, LT-Bandbreite) für Multi-Fluss-Szenarien erhalten bleibt.

**Hysterese**: Der Eintritt erfordert `ucp_alone_confirm_rounds` (Standard 3) aufeinanderfolgende qualifizierte Runden — vermeidet Oszillationen während kurzer Ruhephasen im Multi-Fluss-Wettbewerb ("konservativ beim Beschleunigen"). Der Austritt erfolgt sofort — jeder Qualifikationsfehler löscht das Flag und setzt den Bestätigungszähler zurück ("aggressiv beim Abbremsen").

Qualifikationsbedingungen (alle sechs müssen an einer Rundengrenze erfüllt sein):
0. Kalman konvergiert (`sample_cnt >= ucp_kalman_min_samples`) — qdelay/jitter als Warteschlangensignale vertrauen
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (Standard 1000 us) — Warteschlange fast leer
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (Standard 2000 us) — nur ACK-Takt-Mikrojitter
3. `ecn_ewma == 0` — keine Überlastungsmarkierungen von AQM
4. `lt_use_bw == 0` — nicht im vom Policer erkannten ratenbegrenzten Modus
5. `agg_state <= max` gemäß `ucp_alone_agg_state_level` (Standard 1) — drei konfigurierbare ACK-Aggregationsstufen: 0 = nur IDLE (strengste, keine Aggregation), 1 = ≤ SUSPECTED (Standard, erlaubt vorübergehende Aggregation), 2 = ≤ CONFIRMED (permessivste, blockiert nur persistente Aggregation)

### Dynamisches PROBE_RTT-Intervall

Bildet Kalman `p_est` auf ein verbindungsspezifisches PROBE_RTT-Intervall ab:

```
p_est ≤ converged:              interval = dyn_max (Standard 30s)
p_est ≥ high (= mult × conv):   interval = base (Standard 10s)
converged < p_est < high:       lineare Interpolation
```

Reduziert die PROBE_RTT-Häufigkeit bei hoher Konfidenz (niedrigem `p_est`), was den Durchsatz-Jitter auf stabilen Pfaden verringert. Kehrt zum klassischen 10-Sekunden-Intervall zurück, wenn die Konfidenz niedrig ist.

**Per-Flow-Eintrags-Jitter**: Um zu verhindern, dass alle koexistierenden Flüsse gleichzeitig in PROBE_RTT eintreten (Entleeren auf 4 Pakete aggregiert ~1.8 Mbps, dann Nachfüllen mit 2.89×), fügt jeder Fluss einen hash-abgeleiteten Jitter (0–845 ms Streuung) zu seinem PROBE_RTT-Intervall hinzu. Zu jedem Zeitpunkt ist maximal ~1 Fluss in PROBE_RTT, wodurch der RTO-induzierende gleichzeitige Entleerungs-/Nachfüllkollaps beseitigt wird.

### LT-Bandbreitenschätzung

Verlustgetriggerter Untergrenzenschätzer. Das Abtastintervall umfasst [4, 16] RTTs. Gültig, wenn das Verlustverhältnis ≥ 5,9% (`ucp_lt_loss_thresh` Standard 15/256). Bandbreite `bw = delivered × BW_UNIT / interval_us`.

Im Gegensatz zu BBRs einfachem Durchschnitt (`(bw + lt_bw) >> 1`) verwendet UCP einen konfigurierbaren EMA (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, Standard 1/2 = 0,5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

Die Aktivierung unterscheidet sich von BBR: UCP speichert `lt_bw` beim ersten gültigen Intervall, setzt aber NICHT `lt_use_bw`; Konsistenz mit einem vorherigen Intervall ist erforderlich — reduziert Fehlaktivierung durch Messrauschen.

**Doppelschwellen-Überlastungstor**: Bevor `lt_use_bw = 1` gesetzt wird, werden sowohl eine persistente EWMA-Warteschlangenprüfung (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) ALS AUCH eine sofortige SRTT-basierte Warteschlangenprüfung (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, Standard 5000 µs) ausgewertet. Wenn eine Überlastung erkannt wird, wird die LT-BW-Abtastung abgebrochen. Die SRTT-Prüfung funktioniert ohne `ext`-Zuweisung und bietet ein Sicherheitsnetz gegen Zuweisungsfehler.

LT-BW-Sondenverstärkung (`ucp_lt_bw_probe_pct`, Standard 10%): verstärkt `pacing_gain` um `1 + probe_pct/100` über alle PROBE_BW-Phasen. Rampenkomponente: `+1% pro 8 RTTs` Anstieg, gedeckelt bei `2 × probe_pct`.

LT-BW-Auto-Wiederherstellung (`ucp_lt_restore_ratio_num/den`, Standard 5/4 = 1,25x): wenn `max_bw > lt_bw × ratio` für `ucp_lt_restore_consec_acks` (Standard 3) aufeinanderfolgende ACKs, wird LT BW automatisch beendet und die normale PROBE_BW-Erkundung wird fortgesetzt.

### ACK-Aggregations-Konfidenzbasierte Kompensation (BBRplus-inspiriert)

Fügt eine konfidenzgesteuerte zweite Schicht über dem traditionellen Dual-Slot-Extra-Acked-Schätzer hinzu.

**Vier orthogonale Faktoren** (jeder trägt `ucp_agg_factor_weight` Punkte bei, Standard 256):
1. Kalman konvergiert (`p_est < converged` + `sample_cnt >= min_samples`)
2. Nicht in Verlustwiederherstellung (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT innerhalb von `min_rtt_us + ucp_agg_factor3_qdelay_us` (Standard 2ms) der wahren Ausbreitungsverzögerung
4. `extra_acked` innerhalb von `ucp_agg_factor4_ratio_num/den` (Standard 1,5x) des fensterbasierten Maximums

**Vier Zustände**: IDLE (< `ucp_agg_thresh_suspected`=256), VERDÄCHTIG (≥256), BESTÄTIGT (≥512), VERTRAUENSWÜRDIG (≥768).

**Signalschicht** (immer aktiv): Konfidenz interpoliert linear den R-Skalierungsfaktor `[r_min, r_max]`. R steigt sofort an (schnelle Reaktion), fällt mit `ucp_agg_r_hysteresis`% (Standard 75% beibehalten, ~4 RTTs zur Basislinie) pro RTT.

**Kontrollschicht** (`agg_state ≥ CONFIRMED`): fünffach sicherheitsgesteuerte cwnd-Kompensation:
1. Blockiert, wenn die Warteschlangenverzögerung > `ucp_agg_safety_qdelay_us` (Standard 4ms)
2. Blockiert während der Verlustwiederherstellung
3. Blockiert, wenn cwnd > `BDP × ucp_agg_safety_bdp_mult` (Standard 3x)
4. Blockiert, wenn Inflight > sicheres cwnd + TSO-Segmentziel
5. Watchdog: stuft BESTÄTIGT→VERDÄCHTIG nach `ucp_agg_max_comp_duration` (Standard 8) aufeinanderfolgenden RTTs herab

### qdelay_avg-Zurücksetzung in DRAIN

Beim Übergang zu DRAIN wird `qdelay_avg` auf Null zurückgesetzt, wodurch verhindert wird, dass die STARTUP-Warteschlangenschätzung in PROBE_BW fortbesteht.

### TSO-Divisor-Anpassung

`ucp_min_tso_segs()` passt den Ratenschwellwertdivisor basierend auf dem Kalman-Zustand an:
- Kalman konvergiert + `jitter_ewma < 1000 µs`: Divisor halbiert (8→4), größere TSO-Bursts
- `jitter_ewma > 4000 µs`: Divisor verdoppelt (8→16), kleinere TSO-Bursts zur Unterdrückung von Jitter

## Pacing-Rate und Cwnd

### Pacing-Rate

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // Verstärkungsanpassung
rate = rate × USEC_PER_SEC >> BW_SCALE            // Umrechnung in bytes/s
rate = rate × margin_div / 100                    // Pacing-Marge (Standard 1%, matching BBR)
```

Ratenänderungen werden sofort angewendet (keine Glättung), entsprechend BBR (Cardwell et al. 2016). Nach `full_bw_reached`: alle Ratenänderungen werden sofort geschrieben. In STARTUP/DRAIN: nur Erhöhungen werden angewendet (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // Basis-BDP
// Inflight-Grenzen (Nicht-STARTUP: lo~hi-Clamp; STARTUP: nur lo-Untergrenze)
target = quantization_budget(target)              // TSO-Headroom + gerade Runde + Phase-0-Bonus
target += ack_agg_bonus + agg_compensation        // ACK-Aggregationskompensation

// cwnd-Fortschritt
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // zum Ziel konvergieren
else (STARTUP):
    cwnd = cwnd + acked                          // exponentielles Wachstum

cwnd = max(cwnd, cwnd_min_target)                 // absolute Untergrenze 4
PROBE_RTT-Modus: cwnd = min(cwnd, cwnd_min_target) // minimaler Inflight
```

## Modulparameter

Parameter werden unter `/proc/sys/net/ucp/` bereitgestellt. Schreibvorgänge lösen `ucp_init_module_params()` aus (Validierung + Begrenzung + Berechnung abgeleiteter Werte). Array-Parameter-Schreibvorgänge lösen `ucp_rebuild_gain_table()` aus.

### PROBE_RTT-Intervalle

| Parameter | Standard | Min | Max | Einheit | Beschreibung |
|-----------|----------|-----|-----|---------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Basis-PROBE_RTT-Intervall |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Obergrenze für Lang-RTT-Pfade |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Max. dynamisches Intervall; 0 deaktiviert |

### Verstärkungen

| Parameter | Standard | Min | Max | Beschreibung |
|-----------|----------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Basis-cwnd-Verstärkung für PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK-Aggregations-Bonusmultiplikator |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP-Verstärkung (≈2,885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN-Verstärkung (≈0,347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Inflight-Untergrenze (1,25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Inflight-Obergrenze (2,0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | BBRv1-Muster (256 Slots) | 0/1 | — | Pro-Slot-Pacing-Verstärkung |
| `ucp_cycle_decay_mask[8]` | 0 (alle Null) | 0 | 0x7FFFFFFF | 256-Bit-Abfall-Bitmap |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Begrenzte Probe-Up-Beendigung (0=aus) |

### Kalman-Basis

| Parameter | Standard | Min | Max | Beschreibung |
|-----------|----------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Basis-Prozessrauschen Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Basis-Messrauschen R |
| `ucp_kalman_p_est_max` | 1.000.000 | 1 | 100M | p_est absolutes Maximum |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | Konvergenzschwellwert |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | Anfängliches p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est-Untergrenze |
| `ucp_kalman_scale` | 1024 | 64 | 1.048.576 | Festkomma-Skalierung (Zweierpotenz) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Mindestmessungen vor Übernahme |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Basis-Ausreißerschwellwert |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-Boost-Multiplikator |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-Boost-Zeitkonstante |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Q-Obergrenze |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Q-Skalierungsbegrenzung |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Max. aufeinanderfolgende Ablehnungen vor Zwangsannahme |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Kalman-RTT-Obergrenze |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | R-Max-Boost-Multiplikator |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT-dynamischer-Obergrenzen-Multiplikator |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Q-Anpassungs-RTT-Divisor |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT-Übergangsband-Multiplikator |

### Kalman-Zusätze (num/den-Typ)

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Ausreißer-Jitter-Multiplikator |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q-Minimalfaktor |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est-Init-RTT-Divisor |

### BBR-S-Rauschschätzung

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q-Schätzungs-Lernrate |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R-Schätzungs-Lernrate |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Kombinationsmodus (0=aus, 1=max, 2=gewichteter Durchschnitt) |
| `ucp_kalman_q_est_max` | 1.000.000.000 | 1-2 Mrd. | Q-Schätzungs-Obergrenze |
| `ucp_kalman_r_est_max` | 1.000.000.000 | 1-2 Mrd. | R-Schätzungs-Obergrenze |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Untergrenze pro Schätzung |

### Verstärkungsabfall (Erkundung)

| Parameter | Standard | Bereich | Einheit | Beschreibung |
|-----------|----------|---------|---------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay-Abfallschwellwert |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay-Abfallskalierung |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Jitter-Abfallschwellwert |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | Jitter-Abfallskalierung |

### Adaptives R (Jitter-gesteuert)

| Parameter | Standard | Bereich | Einheit | Beschreibung |
|-----------|----------|---------|---------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | Jitter-Schwellwert für R-Erhöhung |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | R-Erhöhungs-Skalierungsdivisor |

### ECN

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | ECN-Hauptschalter |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN-Rücknahme-Anteil |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN-qdelay-Schwellwert |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN-EWMA-Gewichte |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Leerlauf-ECN-Abfall |

### min_rtt

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Schnellabfall-Zähler |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Schnellabfall-Schwellwertdivisor |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Haftendes-Abfall-Verhältnis |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT-Schutzverhältnis |

### LT-Bandbreite

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Minimale Intervalllänge |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Intervall-TimeOut-Multiplikator |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Mindestverlustverhältnis |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Relative Toleranz |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | Absolute Toleranz |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW max. aktive RTTs |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | LT-BW-Sondenverstärkung |

### LT-Auto-Wiederherstellung

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Wiederherstellungsauslöseverhältnis |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Auslöse-Anzahl aufeinanderfolgender ACKs |

### ACK-Aggregations-Konfidenz

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Hauptschalter |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | cwnd-Kompensations-Konfidenzschwellwert |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % des BDP | cwnd-Kompensationsgrenze |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | Watchdog-Timeout |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | R-Hysterese-Abfall |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R-Skalierungsbereich (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Faktor-3-qdelay-Spielraum |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Faktor-4-Verhältnis |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Sicherheitsschutz 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Sicherheitsschutz-BDP-Multiplikator |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked-Grenzfenster |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Watchdog-Abfallrate |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Fensterrotationsperiode |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Punktzahl pro Faktor |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Maximale Konfidenz |

### EWMA-Koeffizienten

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay-EWMA-Gewicht |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Jitter-EWMA-Gewicht |

### Sonstiges

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW-Zyklenphasen (Zweierpotenz) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Zufälliger Phasenversatz |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP-Austritts-Wachstumsschwellwert |
| `ucp_full_bw_cnt` | 3 | 1-3 | Nicht-Wachstumsrunden zum Austritt |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT-Verweildauer |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Pacing-Marge (0 = keine) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | Segmente | Phase-0-cwnd-Bonus |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | Runden | BW-Fensterlänge (gleitend) |
| `ucp_cwnd_min_target` | 4 | 1-1000 | Segmente | Min. cwnd (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP-min_rtt-Untergrenze |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | EDT-Beinahe-Jetzt-Schwellwert |
| `ucp_min_tso_rate` | 1.200.000 | 1-1 Mrd. | bytes/s | TSO-Niedrigratenschwellwert |
| `ucp_min_tso_rate_div` | 8 | 1-256 | TSO-Ratendivisor (adaptive Basis) |
| `ucp_tso_max_segs` | 127 | 1-65535 | Segmente | Max. TSO-Segmente |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | TSO-Headroom-Multiplikator |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Sendepuffer-Expansionsfaktor |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK-Epochengrenze |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Max. ACK-Aggregationsfenster |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Lang-RTT-Schwellwert |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Lang-RTT-Intervall-Divisor |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Drain-Überspring-qdelay-Schwellwert |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | Runden | Runden vor Aktivierung des Einzelfluss-Modus |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Max. Warteschlangenverzögerung für Einzelflusserkennung |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | Max. Jitter für Einzelflusserkennung |
| ucp_alone_agg_state_level | 1 | 0-2 | — | Aggregationsstrenge (0=nur IDLE, 1=≤SUSPECTED Standard, 2=≤CONFIRMED zu aggressiv) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | ECN-Backoff im Alleinmodus überspringen (1=überspringen, 0=aktiv) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | LT BW Bedingung im Alleinmodus überspringen (1=überspringen, 0=aktiv) |

## Datenpfad

```
ACK kommt an (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► ACK-Aggregations-Konfidenzpipeline (wenn ucp_agg_enable)
    │      messen → bewerten → zustand → watchdog
    │      ├── Signalschicht: Kalman-R-Skalierung (immer aktiv)
    │      └── Kontrollschicht: cwnd-Kompensation (BESTÄTIGT+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              Max-BW mit gleitendem Fenster
    │      ├── ucp_update_ecn_ewma()        ECN-CE-Markierungsverhältnis
    │      ├── ucp_update_ack_aggregation()  Doppelfenster-extra_acked
    │      ├── ucp_update_cycle_phase()     PROBE_BW-Phasenfortschritt
    │      ├── ucp_check_full_bw_reached()  STARTUP-Austrittserkennung
    │      ├── ucp_check_drain()            DRAIN-Eintritt/Austritt + Drain-Überspringen
    │      ├── ucp_update_min_rtt()         Kalman + Fenster-min-RTT + PROBE_RTT
    │      └── Modus-spezifische Verstärkungszuweisung
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            ECN-Rücknahme (nur cwnd_gain)
    │
    ├──► ucp_set_pacing_rate()              sofortig, BBR-Regel
    │
    └──► ucp_set_cwnd()                    BDP + Grenzen + Agg-Kompensation
```

## Kalman-Filter Interner Ablauf

```
RTT-Messung (rtt_us)
    │
    ├── Ungültig (≥0 und < dynamic_max)? Ja → verwerfen
    │
    ├── Kaltstart (sample_cnt==0)? Ja → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                          (umgeht RTT-Max-Sperre)
    │
    ├── Adaptives Q: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   Adaptives R: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovation: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Vorhersage: p_pred = p_est + Q
    │
    ├── Ausreißer-Sperre: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Ja & reject_cnt < max → ablehnen, ++consec_reject_cnt, zurück
    │   └── Ja & reject_cnt ≥ max → Zwangsannahme (Anti-Sperre)
    │
    └── Kalman-Aktualisierung:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (auf nicht-negativ begrenzt)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Jitter-EWMA-Aktualisierung
         ├── qdelay-EWMA-Aktualisierung
         ├── BBR-S-Kovarianz-angepasste Rauschschätzung
         └── sample_cnt++
```

## Diagnose

BBR-kompatible Diagnoseschnittstelle über `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: 64-Bit-Bandbreitenschätzung (bytes/s)
bbr_min_rtt:         aktuelles min_rtt_us
bbr_pacing_gain:     aktuelle Pacing-Verstärkung (BBR_UNIT, 256=1,0x)
bbr_cwnd_gain:       aktuelle cwnd-Verstärkung (BBR_UNIT)
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

*UCP v1.0 — basiert auf BBRv1 (Cardwell et al. 2016, ACM Queue) und dem Kalman-Filter (Kalman 1960).*

## Referenzen

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
