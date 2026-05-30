[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

# TCP UCP v1.0 (유니버설 통신 프로토콜)

공유 대역폭 VPS 환경을 위한, BBRv1 상태 머신과 칼만 필터를 결합하여 전파 지연을 추정하는 TCP 혼잡 제어 모듈입니다.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## 알고리즘 개요

TCP UCP는 Linux 커널용 로더블 `tcp_ucp.ko`로 구현된 송신측 혼잡 제어 모듈입니다. 혼잡 제어 함수 `ucp_main()`은 `tcp_ack()`에서 각 ACK를 수신할 때마다 호출되며, 커널 수준의 대역폭 및 RTT 샘플과 전송 및 손실 카운트를 포함하는 `rate_sample` 구조체를 전달받습니다. 이 알고리즘은 두 가지 시간 영역에서 작동합니다: **ACK별 고속 경로**는 측정 상태를 업데이트하고 즉각적인 페이싱 및 윈도우 목표를 계산하며, **라운드별 저속 경로**는 상태 전이 조건을 평가하고 게인을 재계산합니다.

핵심 측정 파이프라인은 두 가지 구성 요소로 이루어져 있습니다:

1. **슬라이딩 윈도우 최대 대역폭 필터**(`linux/win_minmax.h`의 `minmax_running_max`): 최근 `ucp_bw_rt_cycle_len`(기본값 10) 라운드를 포함하는 윈도우입니다. BBR 호환 `max_bw` 추정값을 제공합니다.

2. **칼만 필터 전파 지연 추정기**: BBRv1의 슬라이딩 윈도우 최소 RTT를 대체합니다. `ucp_kalman_scale` × µs 고정소수점 단위로 작동하는 단일 상태 칼만 필터(Kalman 1960)로, 실제 전파 지연을 랜덤 워크로 모델링합니다:
   - 상태: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - 관측: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

고정소수점 규칙: 대역폭은 `BW_UNIT = 1 << 24`(세그먼트 * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256`을 무차원 게인 단위로 사용합니다.

## 상태 머신

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

4가지 모드는 `struct ucp`의 2비트 `mode` 필드로 인코딩됩니다:

- **STARTUP (0)**: 초기 상태. `pacing_gain` ≈ 2.885x(`ucp_high_gain_val`), `cwnd_gain`도 2.885x. 지수적 대역폭 프로빙.
- **DRAIN (1)**: STARTUP 종료 후 진입. `pacing_gain` ≈ 0.347x(`ucp_drain_gain_val`), `cwnd_gain`은 2.885x 유지. STARTUP 중 누적된 큐를 비웁니다.
- **PROBE_BW (2)**: 정상 상태. 256개 슬롯 게인 테이블 순환(기본 8-페이즈 패턴 반복: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: 주기적으로 in-flight를 `ucp_cwnd_min_target`(기본값 4 세그먼트)까지 비워 새로운 RTT 샘플을 획득합니다.

### STARTUP → DRAIN

`full_bw_reached`가 설정되면 트리거됩니다——`ucp_full_bw_cnt`(기본값 3)회 연속 라운드에서 `max_bw`가 이전에 관측된 피크 대비 `ucp_full_bw_thresh_val`(기본값 1.25x) 이상 성장하지 못한 경우. 1.0x 게인에서의 BDP가 `snd_ssthresh`에 기록됩니다. `qdelay_avg`는 0으로 리셋되어 STARTUP 큐 누적이 PROBE_BW에 영향을 미치는 것을 방지합니다.

### DRAIN → PROBE_BW

추정된 EDT에서의 in-flight가 1.0x BDP 게인의 목표 in-flight 이하가 되면 트리거됩니다. **드레인 스킵 최적화**: 칼만 필터가 수렴되고 `qdelay_avg`가 `ucp_drain_skip_qdelay_us`(기본값 1000 µs) 미만인 경우 DRAIN 단계를 건너뛰고 조기에 PROBE_BW로 전환합니다.

PROBE_BW 진입 시, 사이클 페이즈 인덱스가 무작위화됩니다: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)`(기본값 `len − 1 − rand(8)`). 이는 병목 링크를 공유하는 동시 흐름의 상관관계를 제거합니다.

### PROBE_BW → PROBE_RTT

PROBE_RTT 필터 간격이 만료되면 트리거됩니다——타임스탬프 `min_rtt_stamp`가 계산된 간격 내에 업데이트되지 않은 경우. cwnd는 `prior_cwnd`에 저장되고, 페이싱은 드레인 모드로 설정됩니다.

### PROBE_RTT → PROBE_BW

in-flight가 `ucp_cwnd_min_target`까지 떨어지거나 라운드 경계가 관측된 후, 최소 `ucp_probe_rtt_mode_ms_val`(기본값 200 ms) 이상 그리고 최소 1회 완전 라운드가 관측된 후 종료됩니다. cwnd는 최소 `prior_cwnd`까지 복원되고, 페이싱은 `ucp_high_gain_val`로 일시적으로 오버라이드되어 파이프를 빠르게 재충전합니다.

### 복구 및 손실

- TCP_CA_Loss 시: `full_bw`와 `full_bw_cnt`가 리셋되고, `round_start`가 1로 설정되며, `packet_conservation`이 0으로 클리어됩니다. LT BW가 활성화되지 않은 경우, 합성 손실 이벤트를 주입하여 LT 샘플링을 트리거합니다.
- 복구 진입(TCP_CA_Recovery): `packet_conservation` 활성화, cwnd = in-flight + acked.
- 복구 종료: `prior_cwnd`로 복원, `packet_conservation` 클리어.
- `ucp_undo_cwnd()`: `full_bw`와 `full_bw_cnt`를 리셋하고(`full_bw_reached` 유지), LT BW 상태를 클리어합니다.

## 핵심 측정

### 대역폭 추정

슬라이딩 윈도우 최대 대역폭 필터(`linux/win_minmax.h`의 `minmax_running_max`), 범위는 `ucp_bw_rt_cycle_len`(기본값 10) 라운드. 순시 대역폭 = `delivered × BW_UNIT / interval_us`, ACK마다 계산. 앱 제한 상태가 아니거나 대역폭이 현재 최대값 이상인 경우(BBR 규칙)에만 슬라이딩 윈도우에 공급됩니다.

`lt_use_bw`가 활성화되면, 활성 대역폭 추정이 `lt_bw`(장기 대역폭 추정값)로 전환됩니다.

### 칼만 필터

단일 상태 스칼라 칼만 재귀(O(1) 복잡도):

```
예측:
  x_pred = x_est          (항등 상태 전이)
  p_pred = p_est + Q      (공분산 예측)

갱신:
  innov   = z − x_pred    (혁신)
  K       = p_pred / (p_pred + R)   (칼만 게인 [0,1])
  x_est   = x_pred + K × innov      (상태 갱신)
  p_est   = (1 − K) × p_pred        (사후 공분산)
```

**적응형 프로세스 잡음 Q**:
```
Q_base   = ucp_kalman_q (기본값 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**적응형 측정 잡음 R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**이상치 게이팅**: 동적 임계값 `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. `p_pred ≤ ucp_kalman_converged_p_est_val`인 경우에만 적용됩니다. `ucp_kalman_max_consec_reject`(기본값 25)회 연속 거부 후, 다음 샘플은 강제 수용되어 자기 강화 잠금을 방지합니다.

**공분산 일치 잡음 추정(BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. 결합 모드: 모드 0 = 휴리스틱 전용, 모드 1 = 최대값(기본값), 모드 2 = 가중 혼합.

**칼만 인계**: `x_est > 0`이고 `sample_cnt ≥ ucp_kalman_min_samples`(기본값 5)인 경우, `min_rtt_us`가 `x_est / ucp_kalman_scale`로 대체됩니다. `min_rtt_stamp`는 업데이트되지 않습니다——PROBE_RTT 간격 트리거는 독립적으로 유지됩니다.

x_est 최소RTT 모델：칼만 파생 model_rtt는 min(x_est_us, min_rtt_us)를 사용 — 칼만 추정치와 윈도우 최소값 중 작은 쪽.

## BBR 개선 사항

### 게인 감쇠

특정 PROBE_BW 페이즈에 대해 256비트 비트맵 `ucp_cycle_decay_mask[]`로 활성화됩니다. 감쇠 공식(수용된 칼만 샘플 기준):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = p_est의 역 스케일링(완전 신뢰 시 BBR_UNIT)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

칼만 신뢰도 스케일링: `p_est > ucp_kalman_converged_p_est`인 경우, 감쇠가 비례적으로 감소하여 필터가 불확실할 때 과도한 백오프를 방지합니다.

### ECN 백오프

활성화 조건(모두 충족 필요):
1. `ucp_ecn_enable_val != 0`
2. 칼만 수렴됨(`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0`(CE 마크 관측)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val`(기본값 2000 µs)
5. 모드가 PROBE_BW가 아님(PROBE_BW에서는 cwnd_gain이 2x로 고정)

프로빙 페이즈 중(`pacing_gain > BBR_UNIT`), ECN 백오프는 `BBR_UNIT² / pacing_gain`에 따라 단계적으로 적용됩니다——1.25x 프로브에서 약 80% 백오프, 2.89x STARTUP 게인에서 약 65%.

ECN 마크 비율 EWMA: 라운드 경계에서 `ucp_ecn_ewma_retained / ucp_ecn_ewma_total`(기본값 3/4)로 업데이트되며, 새로운 CE 마크가 없는 각 ACK에서 `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den`(기본값 31/32)의 완만한 감쇠가 적용됩니다.

### 단일 흐름 감지

UCP가 병목 지점에서 흐름이 단독일 가능성이 높다고 감지하면(낮은 큐 지연, 낮은 지터, ECN 마크 없음, ACK 집계 없음, LT 대역폭 없음), 자동으로 BBR 순수 모드로 전환됩니다:

- `ucp_get_model_rtt()`가 `min_rtt_us`를 직접 반환합니다(단측 측정 잡음으로 인한 작은 양의 편향을 가진 칼만 평활 추정치를 우회).
- `ucp_ecn_backoff()` 응답은 `ucp_alone_bypass_ecn`(기본값 1)로 설정 가능——단일 흐름 경로에는 ECN 표시를 공유할 경쟁 발신자가 없으며, 표시는 AQM 오탐이므로 건너뛰어 BBR의 제로 ECN 동작과 일치시킴. 0으로 설정 시 단독 모드에서도 ECN 백오프 유지(보수적).
- LT BW(폴리서 감지) 조건은 `ucp_alone_bypass_lt_bw`(기본값 1)로 설정 가능——단일 흐름 경로에 폴리서가 없으므로 LT BW가 정당하게 발동할 수 없으며, 건너뛰어 가짜 트리거로 인한 단독 모드 종료를 방지함. 0으로 설정 시 원래 엄격한 동작으로 복귀.

이를 통해 UCP와 BBR 간의 단일 흐름 처리량 격차가 제거되며, 다중 흐름 시나리오에서는 UCP의 완전한 보호 루프(칼만, ECN 백오프, 이득 감쇠, LT 대역폭)가 유지됩니다.

**히스테리시스**: 진입에는 `ucp_alone_confirm_rounds`(기본값 3)개의 연속 적격 라운드가 필요합니다——다중 흐름 경쟁 중 짧은 조용한 기간 동안의 진동을 방지합니다("보수적 가속"). 퇴장은 즉시 이루어집니다——어떤 조건이라도 실패하면 플래그가 지워지고 확인 카운터가 재설정됩니다("공격적 제동").

적격 조건(라운드 경계에서 여섯 가지 모두 충족되어야 함):
0. 칼만 수렴(`sample_cnt >= ucp_kalman_min_samples`) — qdelay/jitter를 큐 신호로 신뢰
1. `qdelay_avg < ucp_alone_qdelay_thresh_us`(기본값 1000 us) — 큐가 거의 비어 있음
2. `jitter_ewma < ucp_alone_jitter_thresh_us`(기본값 2000 us) — ACK 클록 마이크로 지터만
3. `ecn_ewma == 0` — AQM의 혼잡 마크 없음
4. `lt_use_bw == 0` — 폴리서 감지 속도 제한 모드 아님
5. `agg_state <= max` `ucp_alone_agg_state_level` 기준 (기본값 1) — 3단계 ACK 집계 엄격도:
   - 0 = IDLE만 (가장 엄격, 제로 집계), 1 = ≤ SUSPECTED (기본값, 일시적 집계 허용), 2 = ≤ CONFIRMED (가장 허용적, 지속적 집계만 차단)

### 동적 PROBE_RTT 간격

칼만 `p_est`를 연결별 PROBE_RTT 간격으로 매핑합니다:

```
p_est ≤ converged:              간격 = dyn_max (기본값 30s)
p_est ≥ high (= mult × conv):   간격 = base (기본값 10s)
converged < p_est < high:       선형 보간
```

신뢰도가 높은 경우(`p_est` 낮음) PROBE_RTT 빈도를 줄여 안정적인 경로에서 처리량 지터를 낮춥니다. 신뢰도가 낮은 경우 기존 10초 간격으로 되돌아갑니다.

**흐름별 엔트리 지터**: 모든 공존 흐름이 동시에 PROBE_RTT에 진입하는 것을 방지하기 위해(4개 패킷 집계 ~1.8 Mbps로 드레인한 후 2.89×로 재충전), 각 흐름은 해시 파생 지터(0–845 ms 분포)를 자체 PROBE_RTT 간격에 추가합니다. 임의의 순간에 최대 ~1개의 흐름만 PROBE_RTT에 있어 RTO를 유발하는 동시 드레인/재충전 붕괴를 제거합니다.

### LT 대역폭 추정

손실 트리거 기반 하한 추정기. 샘플링 간격은 [4, 16] RTT 범위. 손실률이 5.9%(`ucp_lt_loss_thresh` 기본값 15/256) 이상일 때 유효. 대역폭 `bw = delivered × BW_UNIT / interval_us`.

BBR의 단순 평균(`(bw + lt_bw) >> 1`)과 달리, UCP는 설정 가능한 EMA를 사용합니다(`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, 기본값 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

활성화 방식이 BBR과 다릅니다: UCP는 첫 번째 유효 간격에서 `lt_bw`를 저장하지만 `lt_use_bw`는 설정**하지 않습니다**; 이전 간격과의 일관성이 필요하며, 측정 잡음으로 인한 잘못된 활성화를 줄입니다.

**이중 임계값 혼잡 게이트**: `lt_use_bw = 1`을 설정하기 전에, 지속적인 EWMA 큐 검사(`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`)와 SRTT 기반 순시 큐 검사(`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, 기본값 5000 µs)가 모두 평가됩니다. 혼잡이 감지되면 LT BW 샘플링이 중단됩니다. SRTT 검사는 `ext` 할당 없이 작동하여 할당 실패에 대한 안전망을 제공합니다.

LT BW 프로브 부스트(`ucp_lt_bw_probe_pct`, 기본값 10%): 모든 PROBE_BW 페이즈에서 `pacing_gain`을 `1 + probe_pct/100`배 증폭. 램프 구성 요소: `8 RTT마다 +1%` 증가, 상한은 `2 × probe_pct`.

LT BW 자동 복구(`ucp_lt_restore_ratio_num/den`, 기본값 5/4 = 1.25x): `max_bw > lt_bw × ratio`가 `ucp_lt_restore_consec_acks`(기본값 3)회 연속 ACK 동안 지속되면 LT BW가 자동으로 종료되고 정상 PROBE_BW 프로빙이 재개됩니다.

### ACK 집계 신뢰도 기반 보상(BBRplus에서 유래)

기존 듀얼 슬롯 extra-acked 추정기 위에 신뢰도 게이트가 적용된 두 번째 레이어를 추가합니다.

**4개의 직교 요소**(각각 `ucp_agg_factor_weight` 포인트(기본값 256) 기여):
1. 칼만 수렴됨(`p_est < converged` + `sample_cnt >= min_samples`)
2. 손실 복구 중이 아님(`icsk_ca_state < TCP_CA_Recovery`)
3. RTT가 `min_rtt_us + ucp_agg_factor3_qdelay_us`(기본값 2ms) 이내로 실제 전파 지연에 가까움
4. `extra_acked`가 `ucp_agg_factor4_ratio_num/den`(기본값 1.5x) 이내로 윈도우화 최대값에 가까움

**4가지 상태**: IDLE(< `ucp_agg_thresh_suspected`=256), SUSPECTED(≥256), CONFIRMED(≥512), TRUSTED(≥768).

**신호 레이어**(항상 활성): 신뢰도가 R 스케일링 계수 `[r_min, r_max]`를 선형 보간. R은 즉시 상승(빠른 응답), 각 RTT에서 `ucp_agg_r_hysteresis`%(기본값 75% 유지, 약 4 RTT 후 기준선 복귀)로 감쇠.

**제어 레이어**(`agg_state ≥ CONFIRMED`): 5계층 안전 게이트 cwnd 보상:
1. 큐 지연 > `ucp_agg_safety_qdelay_us`(기본값 4ms)면 차단
2. 손실 복구 중이면 차단
3. cwnd > `BDP × ucp_agg_safety_bdp_mult`(기본값 3x)면 차단
4. in-flight > 안전 cwnd + TSO 세그먼트 목표면 차단
5. 워치독: `ucp_agg_max_comp_duration`(기본값 8)회 연속 RTT 후 CONFIRMED을 SUSPECTED로 강등

### 드레인 qdelay_avg 리셋

DRAIN으로 전환 시 `qdelay_avg`가 0으로 리셋되어 STARTUP 큐 추정값이 PROBE_BW에 지속되는 것을 방지합니다.

### TSO 제수 적응

`ucp_min_tso_segs()`는 칼만 상태에 따라 속도 임계값 제수를 조정합니다:
- 칼만 수렴됨 + `jitter_ewma < 1000 µs`: 제수 절반(8→4), 더 큰 TSO 버스트
- `jitter_ewma > 4000 µs`: 제수 두 배(8→16), 더 작은 TSO 버스트로 지터 억제

## 페이싱 속도 및 Cwnd

### 페이싱 속도

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // 게인 조정
rate = rate × USEC_PER_SEC >> BW_SCALE            // bytes/s로 변환
rate = rate × margin_div / 100                    // 페이싱 마진(기본값 1%, matching BBR)
```

속도 변경은 즉시 적용되며(평활화 없음), BBR(Cardwell et al. 2016)과 일치합니다. `full_bw_reached` 이후: 모든 속도 변경이 즉시 기록됩니다. STARTUP/DRAIN 중: 증가(`rate > sk_pacing_rate`)만 적용됩니다.

### Cwnd

```
target = BDP(bw, gain, ext)                       // 기본 BDP
// in-flight 경계(비-STARTUP: lo~hi 클램프; STARTUP: lo 플로어만)
target = quantization_budget(target)              // TSO 헤드룸 + 짝수 라운드 + phase-0 보너스
target += ack_agg_bonus + agg_compensation        // ACK 집계 보상

// cwnd 진행
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // 목표로 수렴
else (STARTUP):
    cwnd = cwnd + acked                          // 지수적 성장

cwnd = max(cwnd, cwnd_min_target)                 // 절대 하한 4
PROBE_RTT mode: cwnd = min(cwnd, cwnd_min_target) // 최소 in-flight
```

## 모듈 파라미터

파라미터는 `/proc/sys/net/ucp/`에서 노출됩니다. 쓰기는 `ucp_init_module_params()`(검증 + 클램핑 + 파생값 계산)를 트리거합니다. 배열 파라미터 쓰기는 `ucp_rebuild_gain_table()`을 트리거합니다.

### PROBE_RTT 간격

| 파라미터 | 기본값 | 최소 | 최대 | 단위 | 설명 |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | 기본 PROBE_RTT 간격 |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | 긴 RTT 경로의 상한 |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | 최대 동적 간격; 0은 비활성화 |

### 게인

| 파라미터 | 기본값 | 최소 | 최대 | 설명 |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | PROBE_BW의 기본 cwnd 게인 |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK 집계 보너스 승수 |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP 게인(≈2.885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN 게인(≈0.347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | in-flight 하한(1.25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | in-flight 상한(2.0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | BBRv1 패턴(256 슬롯) | 0/1 | — | 슬롯별 페이싱 게인 |
| `ucp_cycle_decay_mask[8]` | 0(모두 0) | 0 | 0x7FFFFFFF | 256비트 감쇠 비트맵 |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | 프로브-업 제한 종료（0=꺼짐） |

### 칼만 기본

| 파라미터 | 기본값 | 최소 | 최대 | 설명 |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | 기본 프로세스 잡음 Q |
| `ucp_kalman_r` | 400 | 0 | 100k | 기본 측정 잡음 R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est 절대 최대값 |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | 수렴 임계값 |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | 초기 p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est 플로어 |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | 고정소수점 스케일(2의 거듭제곱) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | 인계 전 최소 샘플 수 |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | 이상치 기본 임계값 |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-Boost 승수 |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-Boost 시정수 |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Q 상한 |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Q 스케일 상한 |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | 강제 수용 전 최대 연속 거부 횟수 |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | 칼만 RTT 상한 |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | R 최대 부스트 승수 |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT 동적 상한 승수 |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Q 적응 RTT 제수 |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT 전이 대역 승수 |

### 칼만 추가(num/den 유형)

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | 이상치 지터 승수 |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q 최소 인자 |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est 초기 RTT 제수 |

### BBR-S 잡음 추정

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q 추정 학습률 |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R 추정 학습률 |
| `ucp_kalman_noise_mode` | 1 | 0-2 | 결합 모드(0=끄기, 1=최대값, 2=가중 평균) |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Q 추정 상한 |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | R 추정 상한 |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | 추정별 하한 |

### 게인 감쇠(프로빙)

| 파라미터 | 기본값 | 범위 | 단위 | 설명 |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay 감쇠 임계값 |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay 감쇠 스케일 |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | 지터 감쇠 임계값 |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | 지터 감쇠 스케일 |

### 적응형 R(지터 기반)

| 파라미터 | 기본값 | 범위 | 단위 | 설명 |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | R 증가를 위한 지터 임계값 |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | R 증가 스케일 제수 |

### ECN

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | ECN 마스터 스위치 |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN 백오프 비율 |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN qdelay 임계값 |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA 가중치 |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | 유휴 ECN 감쇠 |

### min_rtt

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | 고속 하강 카운트 |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | 고속 하강 임계값 제수 |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | 스티키 하강 비율 |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT 가드 비율 |

### LT 대역폭

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | 최소 간격 길이 |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | 간격 타임아웃 승수 |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | 최소 손실률 |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | 상대 허용 오차 |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | 절대 허용 오차 |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW 최대 활성 RTTs |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | LT BW 프로브 부스트 |

### LT 자동 복구

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | 복구 트리거 비율 |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | 트리거 연속 ACK 수 |

### ACK 집계 신뢰도

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | 마스터 스위치 |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | cwnd 보상 신뢰도 임계값 |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % of BDP | cwnd 보상 상한 |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | 워치독 타임아웃 |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | R 히스테리시스 감쇠 |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R 스케일링 범위(256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | 요소 3 qdelay 마진 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | 요소 4 비율 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | 안전 가드 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | 안전 가드 BDP 승수 |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked 상한 윈도우 |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | 워치독 감쇠율 |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | 윈도우 로테이션 주기 |
| `ucp_agg_factor_weight` | 256 | 1-1024 | 요소별 점수 |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | 최대 신뢰도 |

### EWMA 계수

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA 가중치 |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | 지터 EWMA 가중치 |

### 기타

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW 사이클 페이즈 수(2의 거듭제곱) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | 사이클 페이즈 무작위 오프셋 |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP 종료 성장 임계값 |
| `ucp_full_bw_cnt` | 3 | 1-3 | 종료할 비성장 라운드 수 |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT 체류 기간 |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | 페이싱 마진(0 = 없음) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | 페이즈 0 cwnd 보너스 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | rounds | 대역폭 슬라이딩 윈도우 길이 |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | 최소 cwnd(PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt 플로어 |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | EDT 현재 시간 근접 임계값 |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | TSO 저속 임계값 |
| `ucp_min_tso_rate_div` | 8 | 1-256 | TSO 속도 제수(적응형 기준) |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | 최대 TSO 세그먼트 수 |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | TSO 헤드룸 승수 |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | 송신 버퍼 확장 인자 |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK 에포크 상한 |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | 최대 ACK 집계 윈도우 |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | 긴 RTT 임계값 |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | 긴 RTT 간격 제수 |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | 드레인 스킵 qdelay 임계값 |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | 라운드 | 단일 흐름 모드 활성화 전 확인 라운드 수 |
| `ucp_alone_qdelay_thresh_us` | 1000 | 0-100k | µs | 단일 흐름 감지 최대 큐 지연 |
| `ucp_alone_jitter_thresh_us` | 2000 | 0-100k | µs | 단일 흐름 감지 최대 지터 |
| ucp_alone_agg_state_level | 1 | 0-2 | — | 집계 엄격도 (0=IDLE만, 1=≤SUSPECTED기본값, 2=≤CONFIRMED과도하게공격적) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | 단독 모드 시 ECN 백오프 건너뛰기 (1=건너뛰기, 0=활성 유지) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | 단독 모드 시 LT BW 조건 건너뛰기 (1=건너뛰기, 0=활성 유지) |

## 데이터 경로

```
ACK 도착 (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► ACK 집계 신뢰도 파이프라인(ucp_agg_enable 활성화 시)
    │      측정 → 평가 → 상태 → 워치독
    │      ├── 신호 레이어: 칼만 R 스케일링(항상 활성)
    │      └── 제어 레이어: cwnd 보상(CONFIRMED+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              슬라이딩 윈도우 최대 BW
    │      ├── ucp_update_ecn_ewma()        ECN-CE 마크 비율
    │      ├── ucp_update_ack_aggregation()  듀얼 윈도우 extra_acked
    │      ├── ucp_update_cycle_phase()     PROBE_BW 페이즈 진행
    │      ├── ucp_check_full_bw_reached()  STARTUP 종료 감지
    │      ├── ucp_check_drain()            DRAIN 진입/종료 + 드레인 스킵
    │      ├── ucp_update_min_rtt()         칼만 + 윈도우 min-RTT + PROBE_RTT
    │      └── 모드별 게인 할당
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            ECN 백오프(cwnd_gain만)
    │
    ├──► ucp_set_pacing_rate()              즉시, BBR 규칙
    │
    └──► ucp_set_cwnd()                    BDP + 경계 + 집계 보상
```

## 칼만 필터 내부 흐름

```
RTT 샘플 (rtt_us)
    │
    ├── 유효하지 않음(≥0 and < dynamic_max)? 아니오 → 폐기
    │
    ├── 콜드 스타트(sample_cnt==0)? 예 → 초기화: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           (RTT 최대 게이트 우회)
    │
    ├── 적응형 Q: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   적응형 R: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── 혁신: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── 예측: p_pred = p_est + Q
    │
    ├── 이상치 게이트: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── 예 & reject_cnt < max → 거부, ++consec_reject_cnt, 반환
    │   └── 예 & reject_cnt ≥ max → 강제 수용(잠금 방지)
    │
    └── 칼만 갱신:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov(음수가 아니도록 클램프)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── 지터 EWMA 갱신
         ├── qdelay EWMA 갱신
         ├── BBR-S 공분산 일치 잡음 추정
         └── sample_cnt++
```

## 진단

`ss -i`(`INET_DIAG_BBRINFO`)를 통한 BBR 호환 진단 인터페이스:

```
bbr_bw_lo/bbr_bw_hi: 64비트 대역폭 추정값(bytes/s)
bbr_min_rtt:         현재 min_rtt_us
bbr_pacing_gain:     현재 페이싱 게인(BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       현재 cwnd 게인(BBR_UNIT)
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

## 참고문헌

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
