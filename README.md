[🇺🇸 English](README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Universal Communication Protocol)

TCP congestion control module for shared-bandwidth VPS environments combining the BBRv1 state machine with a Kalman filter for propagation-delay estimation.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. **BBRv1 provides a proven foundation.** State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. **The Kalman filter improves estimation accuracy.** Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. **Inter-algorithm dynamics follow standard TCP competitive equilibrium.** UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via `ucp_cycle_decay_mask` but disabled by default to preserve full probe intensity.

4. **Intra-UCP fairness is actively maintained.** Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Algorithm Overview

TCP UCP implements a sender-side congestion control module for the Linux kernel as a loadable `tcp_ucp.ko`. The congestion control function `ucp_main()` is invoked on each ACK from `tcp_ack()`, receiving a `rate_sample` structure that contains kernel-level bandwidth and RTT samples along with delivery and loss counts. The algorithm operates in two temporal regimes: a **per-ACK fast path** that updates measurement state and computes instantaneous pacing and window targets, and a **per-round slower path** that evaluates state-transition conditions and recomputes gains.

The core measurement pipeline consists of two components:

1. **Sliding-window maximum bandwidth filter** (`minmax_running_max` from `linux/win_minmax.h`): window covering the last `ucp_bw_rt_cycle_len` (default 10) round trips. Provides the BBR-compatible `max_bw` estimate.

2. **Kalman filter propagation-delay estimator**: replaces BBRv1's sliding-window minimum RTT. A single-state Kalman filter (Kalman 1960) operating in `ucp_kalman_scale` × µs fixed-point units, modeling the true propagation delay as a random walk:
   - State: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Observation: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Fixed-point conventions: `BW_UNIT = 1 << 24` for bandwidth (segments * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` as the dimensionless gain unit.

## State Machine

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

Four modes encoded as the 2-bit `mode` field in `struct ucp`:

- **STARTUP (0)**: Initial state. pacing_gain ≈ 2.885x (`ucp_high_gain_val`), cwnd_gain also 2.885x. Exponential bandwidth probing.
- **DRAIN (1)**: Entered after STARTUP exit. pacing_gain ≈ 0.347x (`ucp_drain_gain_val`), cwnd_gain remains 2.885x. Drains the queue accumulated during STARTUP.
- **PROBE_BW (2)**: Steady state. Cycles through a 256-slot gain table (default 8-phase pattern repeated: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: Periodically drains inflight to `ucp_cwnd_min_target` (default 4 segments) to obtain a fresh RTT sample.

### STARTUP → DRAIN

Triggered when `full_bw_reached` is set — after `ucp_full_bw_cnt` (default 3) consecutive rounds where `max_bw` fails to grow by at least `ucp_full_bw_thresh_val` (default 1.25x) compared to the previously observed peak. The BDP at 1.0x gain is written to `snd_ssthresh`. `qdelay_avg` is reset to zero to prevent the STARTUP queue buildup from affecting PROBE_BW.

### DRAIN → PROBE_BW

Triggered when estimated inflight-at-EDT ≤ target inflight at 1.0x BDP gain. **Drain-skip optimization**: when the Kalman filter is converged AND `qdelay_avg` is below `ucp_drain_skip_qdelay_us` (default 1000 µs), the DRAIN phase is skipped — converts early to PROBE_BW.

On PROBE_BW entry, the cycle phase index is randomized: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (default `len − 1 − rand(8)`), which decorrelates concurrent flows sharing a bottleneck link.

### PROBE_BW → PROBE_RTT

Triggered when the PROBE_RTT filter interval expires — the timestamp `min_rtt_stamp` has not been updated within the computed interval. cwnd is saved in `prior_cwnd`, pacing set to drain.

### PROBE_RTT → PROBE_BW

After inflight drops to `ucp_cwnd_min_target` or a round boundary is observed, persists for at least `ucp_probe_rtt_mode_ms_val` (default 200 ms) and at least one full round observed, then exits. cwnd is restored to at least `prior_cwnd`, pacing is temporarily overridden with `ucp_high_gain_val` for rapid pipe refill.

### Recovery & Loss

- On TCP_CA_Loss: `full_bw` and `full_bw_cnt` reset, `round_start` set to 1, `packet_conservation` cleared to 0. If LT BW is not active, injects a synthetic loss event to trigger LT sampling.
- Recovery entry (TCP_CA_Recovery): `packet_conservation` enabled, cwnd = inflight + acked.
- Recovery exit: restored to `prior_cwnd`, `packet_conservation` cleared.
- `ucp_undo_cwnd()`: resets `full_bw` and `full_bw_cnt` (preserving `full_bw_reached`), clears LT BW state.

## Core Measurements

### Bandwidth Estimation

Sliding-window max bandwidth filter (`minmax_running_max` from `linux/win_minmax.h`) over `ucp_bw_rt_cycle_len` (default 10) rounds. Instantaneous bw = `delivered × BW_UNIT / interval_us` computed per ACK. Fed into the sliding window only when not app-limited or when bw ≥ current max (BBR rule).

When `lt_use_bw` is active, the active bandwidth estimate switches to `lt_bw` (Long-Term bandwidth estimate).

### Kalman Filter

Single-state scalar Kalman recursion (O(1) complexity):

```
Predict:
  x_pred = x_est          (identity state transition)
  p_pred = p_est + Q      (covariance prediction)

Update:
  innov   = z − x_pred    (innovation)
  K       = p_pred / (p_pred + R)   (Kalman gain [0,1])
  x_est   = x_pred + K × innov      (state update)
  p_est   = (1 − K) × p_pred        (posterior covariance)
```

**Adaptive process noise Q**:
```
Q_base   = ucp_kalman_q (default 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Adaptive measurement noise R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence.  A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Outlier gating**: dynamic threshold `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Applied only when `p_pred ≤ ucp_kalman_converged_p_est_val`. After `ucp_kalman_max_consec_reject` (default 25) consecutive rejections, the next sample is force-accepted to prevent self-reinforcing lock-in.

**Covariance-matched noise estimation (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Combination mode: mode 0 = heuristic only, mode 1 = max (default), mode 2 = weighted blend.

**Kalman takeover**: when `x_est > 0` and `sample_cnt ≥ ucp_kalman_min_samples` (default 5), `min_rtt_us` is replaced by `x_est / ucp_kalman_scale`. `min_rtt_stamp` is not updated — PROBE_RTT interval trigger remains independent.

**x_est min-rtt model**: the Kalman-derived `model_rtt` uses `min(x_est_us, min_rtt_us)` — the smaller of the Kalman estimate and the windowed minimum.  This allows Kalman to discover lower RTTs than the windowed min (deeper valleys between congestion events), but prevents x_est from inflating BDP beyond the windowed minimum.  The result is a more accurate propagation-delay baseline that naturally avoids the over-aggression responsible for retransmit variance on high-loss and short-RTT paths.

## BBR Enhancements

### Gain Decay

Enabled by the 256-bit bitmap `ucp_cycle_decay_mask[]` for specific PROBE_BW phases. Decay formula (on accepted Kalman sample):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = inverse scaling of p_est (BBR_UNIT at full)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Kalman confidence scaling: when `p_est > ucp_kalman_converged_p_est`, decay is proportionally reduced, avoiding excessive backoff when the filter is uncertain.

### ECN Backoff

Activation conditions (all must hold):
1. `ucp_ecn_enable_val != 0`
2. Kalman converged (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (CE marks observed)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (default 2000 µs)
5. Mode is NOT PROBE_BW (cwnd_gain is fixed at 2x in PROBE_BW)

During probing phases (`pacing_gain > BBR_UNIT`), ECN backoff is graduated by `BBR_UNIT² / pacing_gain` — ~80% of backoff at 1.25x probe, ~65% at 2.89x STARTUP gain.

ECN mark ratio EWMA: updated on round boundaries by `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (default 3/4), with gentle per-ACK decay of `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (default 31/32) on each ACK with no new CE marks.

### Single-Flow Detection

When UCP detects the flow is likely alone on the bottleneck (low queue delay, low jitter, no ECN marks, no ACK aggregation, no LT bandwidth), it automatically transitions to a BBR-pure mode:

- `ucp_get_model_rtt()` returns `min_rtt_us` directly (bypassing the Kalman smoothed estimate, which has a small positive bias from one-sided measurement noise).
- `ucp_ecn_backoff()` is skipped when `ucp_alone_bypass_ecn = 1` (default), matching BBR's zero-ECN behavior. On a single-flow path there is no competing sender to share ECN marks with — any marks are false positives. Set `ucp_alone_bypass_ecn = 0` to keep ECN backoff active even when alone (conservative).
- LT BW (policer-detected rate limit) qualification is configurable via `ucp_alone_bypass_lt_bw` (default 1). A single-flow path has no policer, so LT BW cannot legitimately activate. Setting it to 1 prevents spurious alone-mode exit from false LT BW triggers. Set to 0 for original strict behavior.

This eliminates the single-flow throughput gap between UCP and BBR while preserving UCP's full protection loop (Kalman, ECN backoff, gain decay, LT bandwidth) for multi-flow scenarios.

**Hysteresis**: Entry requires `ucp_alone_confirm_rounds` (default 3) consecutive qualifying rounds — preventing oscillation during brief quiet periods in multi-flow competition ("conservative to accelerate"). Exit is immediate — any qualification failure clears the flag and resets the confirmation counter ("aggressive to brake").

Qualification conditions (all six must hold on a round boundary):
0. Kalman converged (`sample_cnt >= ucp_kalman_min_samples`) — trust qdelay/jitter as queue signals
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (default 1000 us) — near-empty queue
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (default 2000 us) — ACK-clock micro-jitter only
3. `ecn_ewma == 0` — no congestion marks from AQM
4. `lt_use_bw == 0` — not in policer-detected rate-limited mode
5. `agg_state <= max` per `ucp_alone_agg_state_level` (default 1) — three-tier configurable:
   - 0 = IDLE only (strictest), 1 = ≤ SUSPECTED (default), 2 = ≤ CONFIRMED (most permissive)

### Dynamic PROBE_RTT Interval

Maps Kalman `p_est` to a per-connection PROBE_RTT interval:

```
p_est ≤ converged:              interval = dyn_max (default 30s)
p_est ≥ high (= mult × conv):   interval = base (default 10s)
converged < p_est < high:       linear interpolation
```

Reduces PROBE_RTT frequency when confidence is high (low `p_est`), lowering throughput jitter on stable paths. Reverts to classic 10-second interval when confidence is low.

**Per-flow entry jitter**: To prevent all co-existing flows from entering PROBE_RTT simultaneously (draining to 4 pkts aggregate ~1.8 Mbps then refilling at 2.89×), each flow adds a hash-derived jitter (0–845 ms spread) to its PROBE_RTT interval. At most ~1 flow is in PROBE_RTT at any instant, eliminating the RTO-inducing simultaneous drain/refill collapse.

### LT Bandwidth Estimation

Loss-triggered lower-bound estimator. Sampling interval spans [4, 16] RTTs. Valid when loss ratio ≥ 5.9% (`ucp_lt_loss_thresh` default 15/256). Bandwidth `bw = delivered × BW_UNIT / interval_us`.

Unlike BBR's simple average (`(bw + lt_bw) >> 1`), UCP uses a configurable EMA (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, default 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

Activation differs from BBR: UCP stores `lt_bw` on the first valid interval but does NOT set `lt_use_bw`; consistency with a previous interval is required — reduces false activation from measurement noise.

**Dual-threshold congestion gate**: Before setting `lt_use_bw = 1`, both a persistent EWMA queue check (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) AND an instantaneous SRTT-based queue check (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, default 5000 µs) are evaluated. When congestion is detected, LT BW sampling is aborted. The SRTT check works without `ext` allocation, providing a safety net against allocation failure.

LT BW probe boost (`ucp_lt_bw_probe_pct`, default 10%): amplifies pacing_gain by `1 + probe_pct/100` across all PROBE_BW phases. Ramp component: `+1% per 8 RTTs` increase, capped at `2 × probe_pct`.

LT BW auto-recovery (`ucp_lt_restore_ratio_num/den`, default 5/4 = 1.25x): when `max_bw > lt_bw × ratio` for `ucp_lt_restore_consec_acks` (default 3) consecutive ACKs, LT BW automatically exits and normal PROBE_BW probing resumes.

### ACK Aggregation Confidence-Based Compensation (BBRplus-inspired)

Adds a confidence-gated second layer over the traditional dual-slot extra-acked estimator.

**Four orthogonal factors** (each contributes `ucp_agg_factor_weight` points, default 256):
1. Kalman converged (`p_est < converged` + `sample_cnt >= min_samples`)
2. Not in loss recovery (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT within `min_rtt_us + ucp_agg_factor3_qdelay_us` (default 2ms) of true propagation delay
4. `extra_acked` within `ucp_agg_factor4_ratio_num/den` (default 1.5x) of windowed maximum

**Four states**: IDLE (< `ucp_agg_thresh_suspected`=256), SUSPECTED (≥256), CONFIRMED (≥512), TRUSTED (≥768).

**Signal layer** (always active): confidence linearly interpolates R scaling factor `[r_min, r_max]`. R rises instantly (fast response), decays at `ucp_agg_r_hysteresis`% (default 75% retained, ~4 RTTs to baseline) per RTT.

**Control layer** (`agg_state ≥ CONFIRMED`): five-layer safety-gated cwnd compensation:
1. Blocks if queue delay > `ucp_agg_safety_qdelay_us` (default 4ms)
2. Blocks during loss recovery
3. Blocks if cwnd > `BDP × ucp_agg_safety_bdp_mult` (default 3x)
4. Blocks if inflight > safe cwnd + TSO segs goal
5. Watchdog: demotes CONFIRMED→SUSPECTED after `ucp_agg_max_comp_duration` (default 8) consecutive RTTs

### Drain qdelay_avg Reset

On transition to DRAIN, `qdelay_avg` is reset to zero, preventing the STARTUP queue estimate from persisting into PROBE_BW.

### TSO Divisor Adaptation

`ucp_min_tso_segs()` adjusts the rate threshold divisor based on Kalman state:
- Kalman converged + `jitter_ewma < 1000 µs`: divisor halved (8→4), larger TSO bursts
- `jitter_ewma > 4000 µs`: divisor doubled (8→16), smaller TSO bursts to suppress jitter

## Pacing Rate & Cwnd

### Pacing Rate

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // gain adjustment
rate = rate × USEC_PER_SEC >> BW_SCALE            // convert to bytes/s
rate = rate × margin_div / 100                    // pacing margin (default 1%, matching BBR)
```

Rate changes are applied immediately (no smoothing), matching BBR (Cardwell et al. 2016). After `full_bw_reached`: all rate changes written immediately. In STARTUP/DRAIN: only increases applied (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // base BDP
// inflight bounds (non-STARTUP: lo~hi clamp; STARTUP: lo floor only)
target = quantization_budget(target)              // TSO headroom + even-round + phase-0 bonus
target += ack_agg_bonus + agg_compensation        // ACK aggregation compensation

// cwnd progression
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // converge to target
else (STARTUP):
    cwnd = cwnd + acked                          // exponential growth

cwnd = max(cwnd, cwnd_min_target)                 // absolute floor 4
PROBE_RTT mode: cwnd = min(cwnd, cwnd_min_target) // minimum inflight
```

## Module Parameters

Parameters are exposed under `/proc/sys/net/ucp/`. Writes trigger `ucp_init_module_params()` (validation + clamping + derived value computation). Array parameter writes trigger `ucp_rebuild_gain_table()`.

### PROBE_RTT Intervals

| Parameter | Default | Min | Max | Unit | Description |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Base PROBE_RTT interval |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Upper cap for long-RTT paths |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Max dynamic interval; 0 disables |

### Gains

| Parameter | Default | Min | Max | Description |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Baseline cwnd gain for PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK aggregation bonus multiplier |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP gain (≈2.885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN gain (≈0.347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Inflight lower bound (1.25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Inflight upper bound (2.0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | BBRv1 pattern (256 slots) | 0/1 | — | Per-slot pacing gain |
| `ucp_cycle_decay_mask[8]` | 0 (all zero) | 0 | 0x7FFFFFFF | 256-bit decay bitmap |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Limit PROBE_BW up-phase exit (0=off) |

### Kalman Base

| Parameter | Default | Min | Max | Description |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Base process noise Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Base measurement noise R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est absolute max |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | Convergence threshold |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | Initial p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est floor |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | Fixed-point scale (power of two) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Min samples before takeover |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Outlier base threshold |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-boost multiplier |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-boost time constant |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Q ceiling |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Q scale cap |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Max consecutive rejections before force-accept |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Kalman RTT ceiling |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | R max boost multiplier |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT dynamic ceiling multiplier |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Q adaptation RTT divisor |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT transition band multiplier |

### Kalman Extras (num/den type)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Outlier jitter multiplier |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q min factor |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est init RTT divisor |

### BBR-S Noise Estimation

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q estimate learning rate |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R estimate learning rate |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Combine mode (0=off, 1=max, 2=weighted avg) |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Q estimate upper bound |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | R estimate upper bound |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Lower bound per estimate |

### Gain Decay (Probing)

| Parameter | Default | Range | Unit | Description |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay decay threshold |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay decay scale |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Jitter decay threshold |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | Jitter decay scale |

### Adaptive R (Jitter-Driven)

| Parameter | Default | Range | Unit | Description |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | Jitter threshold for R increase |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | R increase scale divisor |

### ECN

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | ECN master switch |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN backoff fraction |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN qdelay threshold |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA weights |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Idle ECN decay |

### min_rtt

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Fast-fall count |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Fast-fall threshold divisor |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Sticky fall ratio |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT guard ratio |

### LT Bandwidth

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Min interval length |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Interval timeout multiplier |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Min loss ratio |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Relative tolerance |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | Absolute tolerance |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW max active RTTs |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | LT BW probe boost |

### LT Auto-Recovery

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Recovery trigger ratio |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Trigger consecutive ACK count |

### ACK Aggregation Confidence

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Master switch |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | cwnd compensation confidence threshold |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % of BDP | cwnd comp cap |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | Watchdog timeout |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | R hysteresis decay |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R scaling range (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Factor 3 qdelay margin |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Factor 4 ratio |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Safety guard 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Safety guard BDP multiplier |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked cap window |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Watchdog decay rate |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Window rotation period |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Per-factor score |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Max confidence |

### EWMA Coefficients

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA weight |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Jitter EWMA weight |

### Miscellaneous

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW cycle phases (power-of-two) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Cycle phase random offset |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP exit growth threshold |
| `ucp_full_bw_cnt` | 3 | 1-3 | Non-growth rounds to exit |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT stay duration |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Pacing margin (0 = none) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | Phase-0 cwnd bonus |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | rounds | BW sliding window length |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | Min cwnd (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt floor |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | EDT near-now threshold |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | TSO low-rate threshold |
| `ucp_min_tso_rate_div` | 8 | 1-256 | TSO rate divisor (adaptive base) |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | Max TSO segments |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | TSO headroom multiplier |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Send buffer expansion factor |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK epoch cap |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Max ACK agg window |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Long-RTT threshold |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Long-RTT interval divisor |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Drain-skip qdelay threshold |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | rounds | Rounds before activating single-flow mode |
| `ucp_alone_qdelay_thresh_us` | 1000 | 0-100k | µs | Max qdelay for single-flow detection |
| `ucp_alone_jitter_thresh_us` | 2000 | 0-100k | µs | Max jitter for single-flow detection |
| `ucp_alone_agg_state_level` | 1 | 0-2 | — | Aggregation strictness (0=IDLE, 1=≤SUSPECTED, 2=≤CONFIRMED) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | Bypass ECN backoff when alone (1=skip, 0=keep active) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | Bypass LT BW qualification when alone (1=skip, 0=keep active) |

## Data Path

```
ACK Arrives (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► ACK agg confidence pipeline (when ucp_agg_enable)
    │      measure → evaluate → state → watchdog
    │      ├── Signal layer: Kalman R scaling (always active)
    │      └── Control layer: cwnd compensation (CONFIRMED+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              sliding-window max BW
    │      ├── ucp_update_ecn_ewma()        ECN-CE mark ratio
    │      ├── ucp_update_ack_aggregation()  dual-window extra_acked
    │      ├── ucp_update_cycle_phase()     PROBE_BW phase advance
    │      ├── ucp_check_full_bw_reached()  STARTUP exit detection
    │      ├── ucp_check_drain()            DRAIN entry/exit + drain skip
    │      ├── ucp_update_min_rtt()         Kalman + window min-RTT + PROBE_RTT
    │      ├── Mode-specific gain assignment
    │      └── ucp_alone_on_path_eval()     single-flow detection (round boundary)
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            ECN backoff (cwnd_gain only)
    │
    ├──► ucp_set_pacing_rate()              immediate, BBR rule
    │
    └──► ucp_set_cwnd()                    BDP + bounds + agg compensation
```

## Kalman Filter Internal Flow

```
RTT sample (rtt_us)
    │
    ├── Invalid (≥0 and < dynamic_max)? No → discard
    │
    ├── Cold start (sample_cnt==0)? Yes → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           (bypasses RTT max gate)
    │
    ├── Adaptive Q: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   Adaptive R: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovation: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Predict: p_pred = p_est + Q
    │
    ├── Outlier gate: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Yes & reject_cnt < max → reject, ++consec_reject_cnt, return
    │   └── Yes & reject_cnt ≥ max → force-accept (anti-lock)
    │
    └── Kalman update:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (clamped non-negative)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Jitter EWMA update
         ├── qdelay EWMA update
         ├── BBR-S covariance-matched noise estimation
         └── sample_cnt++
```

## Diagnostics

BBR-compatible diagnostic interface via `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: 64-bit bandwidth estimate (bytes/s)
bbr_min_rtt:         current min_rtt_us
bbr_pacing_gain:     current pacing gain (BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       current cwnd gain (BBR_UNIT)
```

## Performance Summary

Test environment: China → US LAX, 212ms RTT, 8 parallel flows, 26% packet loss, 1 Gbps shared VPS bottleneck.

| Metric | UCP v1.0 | BBR (control) | Delta |
|--------|------------|---------------|-------|
| Average throughput | 1,010 Mbps | 937 Mbps | **+7.8%** |
| Intra-UCP unfairness | 3.1× | 6.2× (BBR) | **−50%** |
| Worst single flow | 60.6 Mbps | 30.8 Mbps | **+97%** |
| Retransmits | 150K/10s | 137K/10s | +9.5% |
| R3 stability | 959 Mbps | 883 Mbps | **+8.6%** |

Retransmits are slightly higher — a trade-off consistent with maintaining high link utilisation under loss.  UCP's Kalman-augmented min_rtt estimation provides a more accurate BDP baseline, allowing the algorithm to sustain higher throughput than BBRv1 on the same path.

---

*UCP v1.0 — built on BBRv1 (Cardwell et al. 2016, ACM Queue) and the Kalman filter (Kalman 1960).*

## References

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
