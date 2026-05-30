[🇺🇸 English](../README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Universal Communication Protocol)

共有帯域幅 VPS 環境向け、BBRv1 状態マシンと伝搬遅延推定のためのカルマンフィルタを組み合わせた TCP 輻輳制御モジュール。

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## アルゴリズム概要

TCP UCP は、ロード可能な `tcp_ucp.ko` として Linux カーネルに送信側輻輳制御モジュールを実装します。輻輳制御関数 `ucp_main()` は、`tcp_ack()` からの各 ACK で呼び出され、カーネルレベルの帯域幅と RTT サンプル、および配信数と損失数を含む `rate_sample` 構造体を受け取ります。アルゴリズムは 2 つの時間領域で動作します。**ACK ごとの高速パス** は測定状態を更新し、瞬時のペーシングとウィンドウ目標を計算します。**ラウンドごとの低速パス** は状態遷移条件を評価し、ゲインを再計算します。

コア測定パイプラインは 2 つのコンポーネントで構成されます：

1. **スライディングウィンドウ最大帯域幅フィルタ**（`linux/win_minmax.h` の `minmax_running_max`）：最後の `ucp_bw_rt_cycle_len`（デフォルト 10）ラウンドトリップをカバーするウィンドウ。BBR 互換の `max_bw` 推定値を提供します。

2. **カルマンフィルタ伝搬遅延推定器**：BBRv1 のスライディングウィンドウ最小 RTT を置き換えます。`ucp_kalman_scale` × µs 固定小数点単位で動作する単一状態カルマンフィルタ（Kalman 1960）で、真の伝搬遅延をランダムウォークとしてモデル化します：
   - 状態：`x[k] = x[k−1] + w[k]`、`w ~ N(0, Q)`
   - 観測：`z[k] = x[k] + v[k]`、`v ~ N(0, R)`

固定小数点の規約：`BW_UNIT = 1 << 24` は帯域幅用（セグメント * 2^24 / µs）、`BBR_UNIT = 1 << 8 = 256` は無次元ゲイン単位。

## 状態機械

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

`struct ucp` の 2 ビット `mode` フィールドにエンコードされた 4 つのモード：

- **STARTUP (0)**：初期状態。pacing_gain ≈ 2.885x（`ucp_high_gain_val`）、cwnd_gain も 2.885x。指数関数的な帯域幅プロービング。
- **DRAIN (1)**：STARTUP 終了後に入移行。pacing_gain ≈ 0.347x（`ucp_drain_gain_val`）、cwnd_gain は 2.885x のまま。STARTUP 中に蓄積されたキューを排出。
- **PROBE_BW (2)**：定常状態。256 スロットのゲインテーブルを循環（デフォルトの 8 フェーズパターンを繰り返し：1.25x/0.75x/8×1.0x）。
- **PROBE_RTT (3)**：定期的にインフライトを `ucp_cwnd_min_target`（デフォルト 4 セグメント）まで排出し、新しい RTT サンプルを取得。

### STARTUP → DRAIN

`full_bw_reached` が設定されるとトリガー — `ucp_full_bw_cnt`（デフォルト 3）回の連続ラウンドで `max_bw` が以前のピークと比較して少なくとも `ucp_full_bw_thresh_val`（デフォルト 1.25x）成長しなかった場合。1.0x ゲインでの BDP が `snd_ssthresh` に書き込まれる。`qdelay_avg` はゼロにリセットされ、STARTUP のキュー蓄積が PROBE_BW に影響するのを防ぐ。

### DRAIN → PROBE_BW

推定 EDT でのインフライト ≤ 1.0x BDP ゲインでの目標インフライトの場合にトリガー。**Drain-skip 最適化**：カルマンフィルタが収束し、かつ `qdelay_avg` が `ucp_drain_skip_qdelay_us`（デフォルト 1000 µs）未満の場合、DRAIN フェーズはスキップされ、早期に PROBE_BW に移行する。

PROBE_BW エントリ時に、サイクルフェーズインデックスはランダム化される：`cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)`（デフォルト `len − 1 − rand(8)`）。これにより、ボトルネックリンクを共有する同時フロー間の相関が除去される。

### PROBE_BW → PROBE_RTT

PROBE_RTT フィルタ間隔が期限切れになるとトリガー — 計算された間隔内にタイムスタンプ `min_rtt_stamp` が更新されていない場合。cwnd は `prior_cwnd` に保存され、ペーシングは排出モードに設定される。

### PROBE_RTT → PROBE_BW

インフライトが `ucp_cwnd_min_target` まで低下するか、ラウンド境界が観測された後、少なくとも `ucp_probe_rtt_mode_ms_val`（デフォルト 200 ms）以上かつ少なくとも 1 完全ラウンドが観測された後に持続し、その後終了。cwnd は少なくとも `prior_cwnd` に復元され、ペーシングは一時的に `ucp_high_gain_val` で上書きされ、パイプを迅速に再充填する。

### リカバリと損失

- TCP_CA_Loss 時：`full_bw` と `full_bw_cnt` がリセットされ、`round_start` が 1 に設定され、`packet_conservation` が 0 にクリアされる。LT BW がアクティブでない場合、合成損失イベントを注入して LT サンプリングをトリガーする。
- リカバリエントリ（TCP_CA_Recovery）：`packet_conservation` が有効化、cwnd = inflight + acked。
- リカバリ終了：`prior_cwnd` に復元、`packet_conservation` がクリアされる。
- `ucp_undo_cwnd()`：`full_bw` と `full_bw_cnt` をリセット（`full_bw_reached` は保持）、LT BW 状態をクリア。

## コア計測

### 帯域幅推定

スライディングウィンドウ最大帯域幅フィルタ（`linux/win_minmax.h` の `minmax_running_max`）を `ucp_bw_rt_cycle_len`（デフォルト 10）ラウンドにわたって適用。瞬時 bw = `delivered × BW_UNIT / interval_us` を ACK ごとに計算。アプリ制限されていない場合、または bw ≥ 現在の最大値（BBR ルール）の場合にのみスライディングウィンドウに入力される。

`lt_use_bw` がアクティブな場合、アクティブな帯域幅推定は `lt_bw`（長期帯域幅推定）に切り替わる。

### カルマンフィルタ

単一状態スカラーカルマン再帰（O(1) 複雑度）：

```
予測：
  x_pred = x_est          （恒等状態遷移）
  p_pred = p_est + Q      （共分散予測）

更新：
  innov   = z − x_pred    （イノベーション）
  K       = p_pred / (p_pred + R)   （カルマンゲイン [0,1]）
  x_est   = x_pred + K × innov      （状態更新）
  p_est   = (1 − K) × p_pred        （事後共分散）
```

**適応的プロセスノイズ Q**：
```
Q_base   = ucp_kalman_q（デフォルト 100）
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**適応的測定ノイズ R**：
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**外れ値ゲート**：動的しきい値 `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`。`p_pred ≤ ucp_kalman_converged_p_est_val` の場合のみ適用。`ucp_kalman_max_consec_reject`（デフォルト 25）回の連続棄却後、次のサンプルは強制受理され、自己強化ロックインを防止。

**共分散整合ノイズ推定（BBR-S）**：`q_est = (1−α) × q_est + α × (K × innov)²`、`r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`。組み合わせモード：モード 0 = ヒューリスティックのみ、モード 1 = 最大値（デフォルト）、モード 2 = 加重混合。

**カルマン制御引き継ぎ**：`x_est > 0` かつ `sample_cnt ≥ ucp_kalman_min_samples`（デフォルト 5）の場合、`min_rtt_us` は `x_est / ucp_kalman_scale` に置き換えられる。`min_rtt_stamp` は更新されない — PROBE_RTT 間隔トリガーは独立したまま。

x_est 最小RTTモデル：カルマン導出の model_rtt は min(x_est_us, min_rtt_us) を使用 — カルマン推定値とウィンドウ最小値の小さい方。

## BBR 拡張機能

### ゲイン減衰

特定の PROBE_BW フェーズに対して 256 ビットビットマップ `ucp_cycle_decay_mask[]` で有効化。減衰式（受理されたカルマンサンプル時）：

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = p_est の逆スケーリング（最大時 BBR_UNIT）
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

カルマン信頼度スケーリング：`p_est > ucp_kalman_converged_p_est` の場合、減衰は比例的に減少し、フィルタが不確かな場合の過度なバックオフを回避。

### ECN バックオフ

活性化条件（すべて成立する必要あり）：
1. `ucp_ecn_enable_val != 0`
2. カルマン収束済み（`p_est < converged`、`sample_cnt >= min_samples`）
3. `ecn_ewma > 0`（CE マーク観測）
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val`（デフォルト 2000 µs）
5. モードが PROBE_BW ではない（PROBE_BW では cwnd_gain は 2x で固定）

プロービングフェーズ中（`pacing_gain > BBR_UNIT`）、ECN バックオフは `BBR_UNIT² / pacing_gain` で段階的に適用 — 1.25x プローブで約 80% のバックオフ、2.89x STARTUP ゲインで約 65%。

ECN マーク比率 EWMA：ラウンド境界で `ucp_ecn_ewma_retained / ucp_ecn_ewma_total`（デフォルト 3/4）によって更新され、新しい CE マークのない各 ACK で `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den`（デフォルト 31/32）の穏やかな ACK ごとの減衰が行われる。

### 単一フロー検出

UCP がボトルネック上でフローが単独であると推定した場合（低キュー遅延、低ジッター、ECN マークなし、ACK 集約なし、LT 帯域幅なし）、自動的に BBR ピュアモードに移行します：

- `ucp_get_model_rtt()` は `min_rtt_us` を直接返します（片側測定ノイズによる小さい正バイアスを持つカルマン平滑化推定をバイパス）。
- `ucp_ecn_backoff()` の応答は `ucp_alone_bypass_ecn`（デフォルト 1）で設定可能——単一フローパスには ECN マークを共有する競合送信者が存在せず、マークは AQM の誤検出であるため、スキップして BBR のゼロ ECN 動作に一致させる。0 に設定すると単独モードでも ECN バックオフを維持（保守的）。
- LT BW（ポリサー検出）条件は `ucp_alone_bypass_lt_bw`（デフォルト 1）で設定可能——単一フローパスにポリサーは存在せず、LT BW が正当に発動することはないため、スキップして偽トリガーによる単独モード退出を防止する。0 に設定すると元の厳格な動作に戻る。

これにより、UCP と BBR の間の単一フロースループット格差が解消され、マルチフローシナリオでは UCP の完全な保護ループ（カルマン、ECN バックオフ、ゲイン減衰、LT 帯域幅）が維持されます。

**ヒステリシス**: エントリには `ucp_alone_confirm_rounds`（デフォルト 3）回の連続した適格ラウンドが必要です——マルチフロー競合中の短い静穏期間での振動を防止します（「保守的に加速」）。退出は即時です——いずれかの条件が失敗するとフラグがクリアされ、確認カウンターがリセットされます（「積極的に制動」）。

適格条件（ラウンド境界で 6 つすべてが満たされる必要があります）：
0. カルマン収束（`sample_cnt >= ucp_kalman_min_samples`） — qdelay/jitter をキュー信号として信頼
1. `qdelay_avg < ucp_alone_qdelay_thresh_us`（デフォルト 1000 us） — キューがほぼ空
2. `jitter_ewma < ucp_alone_jitter_thresh_us`（デフォルト 2000 us） — ACK クロックのマイクロジッターのみ
3. `ecn_ewma == 0` — AQM からの輻輳マークなし
4. `lt_use_bw == 0` — ポリサー検出のレート制限モードではない
5. `agg_state <= max` `ucp_alone_agg_state_level` に従う（デフォルト 1）— 3段階の ACK 集約厳格度:
   - 0 = IDLE のみ（最も厳格、ゼロ集約）、1 = ≤ SUSPECTED（デフォルト、一時的集約を許容）、2 = ≤ CONFIRMED（最も許容的、持続的集約のみ遮断）

### 動的 PROBE_RTT 間隔

カルマン `p_est` を接続ごとの PROBE_RTT 間隔にマッピング：

```
p_est ≤ converged：              間隔 = dyn_max（デフォルト 30 秒）
p_est ≥ high（= mult × conv）：   間隔 = base（デフォルト 10 秒）
converged < p_est < high：       線形補間
```

信頼度が高い（低 `p_est`）場合に PROBE_RTT 頻度を減らし、安定した経路でのスループットジッタを低減。信頼度が低い場合は従来の 10 秒間隔に戻る。

**フローエントリジッタ**: すべての共存フローが同時に PROBE_RTT に入るのを防ぐため（4 パケット集約 ~1.8 Mbps まで排出し、その後 2.89× で再充填）、各フローはハッシュ派生ジッタ（0–845 ms の分布）を自身の PROBE_RTT 間隔に追加します。任意の瞬間に最大 ~1 フローが PROBE_RTT にあり、RTO を誘発する同時排出/再充填の崩壊を排除します。

### LT 帯域幅推定

損失トリガー型下限推定器。サンプリング間隔は [4, 16] RTT の範囲。損失率 ≥ 5.9%（`ucp_lt_loss_thresh` デフォルト 15/256）の場合に有効。帯域幅 `bw = delivered × BW_UNIT / interval_us`。

BBR の単純平均（`(bw + lt_bw) >> 1`）とは異なり、UCP は設定可能な EMA（`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`、デフォルト 1/2 = 0.5）を使用：

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

活性化は BBR と異なる：UCP は最初の有効な間隔で `lt_bw` を保存するが、`lt_use_bw` は設定しない。前の間隔との整合性が必要 — 測定ノイズによる誤活性化を低減。

**二重閾値輻輳ゲート**: `lt_use_bw = 1` を設定する前に、永続的 EWMA キュー検査（`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`）と SRTT ベースの瞬時キュー検査（`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`、デフォルト 5000 µs）の両方が評価されます。輻輳が検出されると、LT BW サンプリングは中止されます。SRTT 検査は `ext` 割り当てなしで動作し、割り当て失敗に対するセーフティネットを提供します。


LT BW プローブブースト（`ucp_lt_bw_probe_pct`、デフォルト 10%）：すべての PROBE_BW フェーズで pacing_gain を `1 + probe_pct/100` だけ増幅。ランプ成分：`8 RTT ごとに +1%` 増加、最大 `2 × probe_pct`。

LT BW 自動回復（`ucp_lt_restore_ratio_num/den`、デフォルト 5/4 = 1.25x）：`ucp_lt_restore_consec_acks`（デフォルト 3）回の連続 ACK で `max_bw > lt_bw × ratio` の場合、LT BW は自動的に終了し、通常の PROBE_BW プロービングが再開される。

### 信頼度ベース ACK 集約補償（BBRplus 由来）

従来のデュアルスロット extra-acked 推定器の上に信頼度ゲート付きの第 2 層を追加。

**4 つの直交因子**（各因子は `ucp_agg_factor_weight` ポイント（デフォルト 256）を寄与）：
1. カルマン収束済み（`p_est < converged` + `sample_cnt >= min_samples`）
2. 損失リカバリ中でない（`icsk_ca_state < TCP_CA_Recovery`）
3. RTT が真の伝搬遅延から `min_rtt_us + ucp_agg_factor3_qdelay_us`（デフォルト 2ms）以内
4. `extra_acked` がウィンドウ最大値の `ucp_agg_factor4_ratio_num/den`（デフォルト 1.5x）以内

**4 つの状態**：IDLE（< `ucp_agg_thresh_suspected`=256）、SUSPECTED（≥256）、CONFIRMED（≥512）、TRUSTED（≥768）。

**信号層**（常時活性）：信頼度が R スケーリング係数 `[r_min, r_max]` を線形補間。R は瞬時に上昇（高速応答）、RTT ごとに `ucp_agg_r_hysteresis`%（デフォルト 75% 保持、約 4 RTT でベースラインに）で減衰。

**制御層**（`agg_state ≥ CONFIRMED`）：5 層の安全ゲート付き cwnd 補償：
1. キュー遅延 > `ucp_agg_safety_qdelay_us`（デフォルト 4ms）の場合はブロック
2. 損失リカバリ中はブロック
3. cwnd > `BDP × ucp_agg_safety_bdp_mult`（デフォルト 3x）の場合はブロック
4. インフライト > 安全 cwnd + TSO セグメント目標の場合はブロック
5. ウォッチドッグ：`ucp_agg_max_comp_duration`（デフォルト 8）回の連続 RTT 後に CONFIRMED→SUSPECTED に降格

### DRAIN 時 qdelay_avg リセット

DRAIN への移行時に `qdelay_avg` はゼロにリセットされ、STARTUP のキュー推定が PROBE_BW に持続するのを防止。

### TSO 除数適応

`ucp_min_tso_segs()` はカルマン状態に基づいてレートしきい値除数を調整：
- カルマン収束済み + `jitter_ewma < 1000 µs`：除数が半減（8→4）、より大きな TSO バースト
- `jitter_ewma > 4000 µs`：除数が倍増（8→16）、より小さな TSO バーストでジッタ抑制

## ペーシングレートと Cwnd

### ペーシングレート

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // ゲイン調整
rate = rate × USEC_PER_SEC >> BW_SCALE            // バイト/秒に変換
rate = rate × margin_div / 100                    // ペーシングマージン（デフォルト 1%、matching BBR）
```

レート変更は即座に適用され（平滑化なし）、BBR と同様（Cardwell et al. 2016）。`full_bw_reached` 後：すべてのレート変更は即座に書き込まれる。STARTUP/DRAIN 中：増加のみ適用（`rate > sk_pacing_rate`）。

### Cwnd

```
target = BDP(bw, gain, ext)                       // 基本 BDP
// インフライト境界（非 STARTUP：lo~hi クランプ、STARTUP：lo フロアのみ）
target = quantization_budget(target)              // TSO ヘッドルーム + 偶数ラウンド + フェーズ 0 ボーナス
target += ack_agg_bonus + agg_compensation        // ACK 集約補償

// cwnd 進行
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // 目標に収束
else (STARTUP):
    cwnd = cwnd + acked                          // 指数関数的成長

cwnd = max(cwnd, cwnd_min_target)                 // 絶対フロア 4
PROBE_RTT モード：cwnd = min(cwnd, cwnd_min_target) // 最小インフライト
```

## モジュールパラメータ

パラメータは `/proc/sys/net/ucp/` で公開。書き込みは `ucp_init_module_params()`（検証 + クランプ + 派生値計算）をトリガー。配列パラメータの書き込みは `ucp_rebuild_gain_table()` をトリガー。

### PROBE_RTT 間隔

| パラメータ | デフォルト | 最小 | 最大 | 単位 | 説明 |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | 秒 | 基本 PROBE_RTT 間隔 |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | 秒 | 長 RTT 経路の上限 |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | 秒 | 最大動的間隔、0 で無効 |

### ゲイン

| パラメータ | デフォルト | 最小 | 最大 | 説明 |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | PROBE_BW のベースライン cwnd ゲイン |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK 集約ボーナス乗数 |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP ゲイン（≈2.885x） |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN ゲイン（≈0.347x） |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | インフライト下限（1.25x BDP） |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | インフライト上限（2.0x BDP） |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | BBRv1 パターン（256 スロット） | 0/1 | — | スロットごとのペーシングゲイン |
| `ucp_cycle_decay_mask[8]` | 0（すべてゼロ） | 0 | 0x7FFFFFFF | 256 ビット減衰ビットマップ |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | プローブアップの制限付き終了（0=オフ） |

### カルマンベース

| パラメータ | デフォルト | 最小 | 最大 | 説明 |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | 基本プロセスノイズ Q |
| `ucp_kalman_r` | 400 | 0 | 100k | 基本測定ノイズ R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est 絶対最大値 |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | 収束しきい値 |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | 初期 p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est フロア |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | 固定小数点スケール（2 のべき乗） |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | 引き継ぎ前の最小サンプル数 |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | 外れ値基本しきい値 |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-boost 乗数 |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-boost 時定数 |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Q 上限 |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Q スケール上限 |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | 強制受理前の最大連続棄却数 |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | カルマン RTT 上限 |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | R 最大ブースト乗数 |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT 動的上限乗数 |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Q 適応 RTT 除数 |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT 遷移帯域乗数 |

### カルマン追加（num/den 型）

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | 外れ値ジッタ乗数 |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q 最小係数 |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est 初期化 RTT 除数 |

### BBR-S ノイズ推定

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q 推定学習率 |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R 推定学習率 |
| `ucp_kalman_noise_mode` | 1 | 0-2 | 組み合わせモード（0=オフ、1=最大、2=加重平均） |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Q 推定上限 |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | R 推定上限 |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | 推定ごとの下限 |

### ゲイン減衰（プロービング）

| パラメータ | デフォルト | 範囲 | 単位 | 説明 |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay 減衰しきい値 |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay 減衰スケール |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | ジッタ減衰しきい値 |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | ジッタ減衰スケール |

### 適応的 R（ジッタ駆動）

| パラメータ | デフォルト | 範囲 | 単位 | 説明 |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | R 増加のジッタしきい値 |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | R 増加スケール除数 |

### ECN

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | ECN マスタースイッチ |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN バックオフ割合 |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN qdelay しきい値 |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA 重み |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | アイドル ECN 減衰 |

### min_rtt

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | 高速低下カウント |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | 高速低下しきい値除数 |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | 粘着低下比率 |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT ガード比率 |

### LT 帯域幅

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTT | 最小間隔長 |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | 間隔タイムアウト乗数 |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | 最小損失率 |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | 相対許容差 |
| `ucp_lt_bw_diff` | 500 | 0-100k | バイト/秒 | 絶対許容差 |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTT | LT BW 最大アクティブ RTT 数 |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | LT BW プローブブースト |

### LT 自動回復

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | 回復トリガー比率 |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | トリガー連続 ACK 数 |

### ACK 集約信頼度

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | マスタースイッチ |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | cwnd 補償信頼度しきい値 |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | BDP の % | cwnd 補償上限 |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTT | ウォッチドッグタイムアウト |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | R ヒステリシス減衰 |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R スケーリング範囲（256=1x） |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | 因子 3 qdelay マージン |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | 因子 4 比率 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | 安全ガード 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | 安全ガード BDP 乗数 |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked 上限ウィンドウ |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | ウォッチドッグ減衰率 |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTT | ウィンドウ回転期間 |
| `ucp_agg_factor_weight` | 256 | 1-1024 | 因子ごとのスコア |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | 最大信頼度 |

### EWMA 係数

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA 重み |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | ジッタ EWMA 重み |

### その他

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW サイクルフェーズ数（2 のべき乗） |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | サイクルフェーズランダムオフセット |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP 終了成長しきい値 |
| `ucp_full_bw_cnt` | 3 | 1-3 | 終了までの非成長ラウンド数 |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT 滞在時間 |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | ペーシングマージン（0 = なし） |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | セグ | フェーズ 0 cwnd ボーナス |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | ラウンド | BW スライディングウィンドウ長 |
| `ucp_cwnd_min_target` | 4 | 1-1000 | セグ | 最小 cwnd（PROBE_RTT） |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt フロア |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | EDT ニアナウしきい値 |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | バイト/秒 | TSO 低レートしきい値 |
| `ucp_min_tso_rate_div` | 8 | 1-256 | TSO レート除数（適応的ベース） |
| `ucp_tso_max_segs` | 127 | 1-65535 | セグ | 最大 TSO セグメント数 |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | TSO ヘッドルーム乗数 |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | 送信バッファ拡張係数 |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | バイト | ACK エポック上限 |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | 最大 ACK 集約ウィンドウ |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | 長 RTT しきい値 |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | 長 RTT 間隔除数 |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | DRAIN スキップ qdelay しきい値 |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | ラウンド | 単一フローモードを有効にする前の確認ラウンド数 |
| `ucp_alone_qdelay_thresh_us` | 1000 | 0-100k | µs | 単一フロー検出の最大キュー遅延 |
| `ucp_alone_jitter_thresh_us` | 2000 | 0-100k | µs | 単一フロー検出の最大ジッター |
| `ucp_alone_agg_state_level` | 1 | 0-2 | — | 集約厳格度（0=IDLEのみ, 1=≤SUSPECTEDデフォルト, 2=≤CONFIRMED過度に攻撃的） |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | 単独モード時の ECN バックオフスキップ（1=スキップ, 0=有効のまま） |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | 単独モード時の LT BW 条件スキップ（1=スキップ, 0=有効のまま） |

## データパス

```
ACK 到着（rate_sample）
    │
    ▼
ucp_main()
    │
    ├──► ACK 集約信頼度パイプライン（ucp_agg_enable 時）
    │      測定 → 評価 → 状態 → ウォッチドッグ
    │      ├── 信号層：カルマン R スケーリング（常時活性）
    │      └── 制御層：cwnd 補償（CONFIRMED+）
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              スライディングウィンドウ最大 BW
    │      ├── ucp_update_ecn_ewma()        ECN-CE マーク比率
    │      ├── ucp_update_ack_aggregation()  デュアルウィンドウ extra_acked
    │      ├── ucp_update_cycle_phase()     PROBE_BW フェーズ進行
    │      ├── ucp_check_full_bw_reached()  STARTUP 終了検出
    │      ├── ucp_check_drain()            DRAIN 入退移行 + DRAIN スキップ
    │      ├── ucp_update_min_rtt()         カルマン + ウィンドウ min-RTT + PROBE_RTT
    │      └── モード固有のゲイン割り当て
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            ECN バックオフ（cwnd_gain のみ）
    │
    ├──► ucp_set_pacing_rate()              即時、BBR ルール
    │
    └──► ucp_set_cwnd()                     BDP + 境界 + 集約補償
```

## カルマンフィルタ内部フロー

```
RTT サンプル（rtt_us）
    │
    ├── 無効（≥0 かつ < dynamic_max）？いいえ → 破棄
    │
    ├── コールドスタート（sample_cnt==0）？はい → 初期化：x_est=z、p_est=max(p_init, rtt_us/div)
    │                                          （RTT 最大ゲートをバイパス）
    │
    ├── 適応的 Q：Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   適応的 R：R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── イノベーション：innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── 予測：p_pred = p_est + Q
    │
    ├── 外れ値ゲート：|innov| > dyn_thresh && p_pred ≤ converged？
    │   ├── はい & reject_cnt < max → 棄却、++consec_reject_cnt、戻る
    │   └── はい & reject_cnt ≥ max → 強制受理（アンチロック）
    │
    └── カルマン更新：
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov（非負にクランプ）
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── ジッタ EWMA 更新
         ├── qdelay EWMA 更新
         ├── BBR-S 共分散整合ノイズ推定
         └── sample_cnt++
```

## 診断

`ss -i`（`INET_DIAG_BBRINFO`）経由の BBR 互換診断インターフェース：

```
bbr_bw_lo/bbr_bw_hi：64 ビット帯域幅推定（バイト/秒）
bbr_min_rtt：         現在の min_rtt_us
bbr_pacing_gain：     現在のペーシングゲイン（BBR_UNIT、256=1.0x）
bbr_cwnd_gain：       現在の cwnd ゲイン（BBR_UNIT）
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

## 参考文献

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
