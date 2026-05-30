[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP UCP v1.0（Universal Communication Protocol）

面向共享頻寬 VPS 環境，結合 BBRv1 狀態機與卡爾曼濾波器進行傳播延遲估計的 TCP 壅塞控制模組。

## 設計理念

壅塞控制演算法需要在吞吐量、延遲、公平性和丟包容忍度之間取得平衡。UCP 採用務實的方案：

1. **BBRv1 提供了經過驗證的基礎。** 狀態機、pacing、週期增益、STARTUP/DRAIN/PROBE_BW/PROBE_RTT —— UCP 直接採用這些機制，不做修改。

2. **卡爾曼濾波器提升估計精度。** 將真實傳播延遲從排隊延遲和測量抖動中分離出來，得到更準確的 min_rtt 估計，從而實現更緊緻的 BDP 計算、更精準的 CWND 校準、更穩定的 pacing。

3. **跨演算法動態遵循標準 TCP 競爭平衡。** UCP 不會人為限制發送速率以回應來自外部流的佇列信號。增益衰減（基於排隊的探測力度降低）可透過 `ucp_cycle_decay_mask` 選擇啟用，但預設關閉以保持完整探測強度。

4. **主動維護 UCP 內部公平性。** 卡爾曼收斂確保共享瓶頸的 UCP 流獲得一致的傳播延遲估計，消除了純 BBR 多流部署中導致嚴重不公平的贏家通吃回饋循環。

## 算法

TCP UCP 以可載入核心模組 `tcp_ucp.ko` 實現傳送端壅塞控制。`ucp_main()` 於每次收到 ACK 時被呼叫，接收 `rate_sample` 結構（含頻寬樣本、RTT 樣本、交付/遺失計數）。算法在兩個時間域運作：**每 ACK 快速路徑**更新測量狀態並計算即時傳送速率和 cwnd；**每輪慢速路徑**評估狀態轉換條件並重新計算增益。

核心測量管線由兩部分組成：

1. **滑動視窗最大頻寬濾波器**（`linux/win_minmax.h` 的 `minmax_running_max`）：視窗覆蓋最近 `ucp_bw_rt_cycle_len`（預設 10）個往返。提供 BBR 相容的 `max_bw` 估計。

2. **卡爾曼濾波器傳播延遲估計**：替代 BBRv1 的滑動視窗最小 RTT。單狀態卡爾曼濾波器（Kalman 1960）在 `ucp_kalman_scale` × µs 定點單位下運作，將真實傳播延遲建模為隨機漫步：
   - 狀態方程：`x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - 觀測方程：`z[k] = x[k] + v[k]`, `v ~ N(0, R)`

定點單位：`BW_UNIT = 1 << 24` 作為頻寬單位（segments * 2^24 / µs），`BBR_UNIT = 1 << 8 = 256` 作為無因次增益單位。

## 狀態機

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

使用 `struct ucp` 的 2 位元 `mode` 欄位編碼四種模式：

- **STARTUP (0)**：初始狀態。pacing_gain ≈ 2.885x（`ucp_high_gain_val`），cwnd_gain 同為 2.885x。指數級頻寬探測。
- **DRAIN (1)**：STARTUP 退出後進入。pacing_gain ≈ 0.347x（`ucp_drain_gain_val`），cwnd_gain 保持 2.885x。排空 STARTUP 累積佇列。
- **PROBE_BW (2)**：穩態。按 256 槽位增益表循環（預設 8 相位重複：1.25x/0.75x/8×1.0x）。
- **PROBE_RTT (3)**：定期排空至 `ucp_cwnd_min_target`（預設 4 段）以獲取新鮮 RTT 樣本。

### STARTUP → DRAIN

當 `full_bw_reached` 置位時觸發——`ucp_full_bw_cnt`（預設 3）個連續輪次中 `max_bw` 未能比先前峰值增長至少 `ucp_full_bw_thresh_val`（預設 1.25x）。BDP 在 1.0x 增益下寫入 `snd_ssthresh`。`qdelay_avg` 歸零以防止 STARTUP 佇列積累影響 PROBE_BW。

### DRAIN → PROBE_BW

EDT 在途資料量 ≤ 1.0x BDP 目標時觸發。**排水跳過優化**：卡爾曼已收斂且 `qdelay_avg` 低於 `ucp_drain_skip_qdelay_us`（預設 1000 µs）時跳過 DRAIN——提前轉 PROBE_BW。

進入 PROBE_BW 時循環相位索引隨機化：`cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)`（預設 `len − 1 − rand(8)`），使共享瓶頸的並發流去同步。

### PROBE_BW → PROBE_RTT

PROBE_RTT 間隔到期時觸發（`min_rtt_stamp` 在計算間隔內未更新）。cwnd 存至 `prior_cwnd`，pacing 設為排水。

### PROBE_RTT → PROBE_BW

在途資料降至 `ucp_cwnd_min_target` 或輪邊界後，駐留至少 `ucp_probe_rtt_mode_ms_val`（預設 200 ms）且至少一輪完整輪次後退出。cwnd 恢復至 `prior_cwnd`，pacing 暫用 `ucp_high_gain_val` 覆寫以快速填充管道。

### 恢復與遺失

- TCP_CA_Loss：`full_bw` 和 `full_bw_cnt` 重置，`round_start` 設 1，`packet_conservation` 清 0。若 LT BW 未激活，注入合成遺失事件觸發 LT 取樣。
- 恢復入（TCP_CA_Recovery）：啟用 `packet_conservation`，cwnd = 在途 + 已 ACK。
- 恢復出：恢復至 `prior_cwnd`，`packet_conservation` 清 0。
- `ucp_undo_cwnd()`：重置 `full_bw` 和 `full_bw_cnt`（保留 `full_bw_reached`），清除 LT BW 狀態。

## 核心測量

### 頻寬估計

滑動視窗最大頻寬濾波器（`minmax_running_max`）在 `ucp_bw_rt_cycle_len`（預設 10）輪視窗。即時 bw = `delivered × BW_UNIT / interval_us`。僅非應用受限或 bw ≥ 現有最大值時饋入滑動視窗（BBR 規則）。

當 `lt_use_bw` 激活時，主動頻寬估計切換至 `lt_bw`（LT 頻寬估計）。

### 卡爾曼濾波器

單狀態純量卡爾曼遞迴（O(1) 複雜度）：

```
預測:
  x_pred = x_est          (恆等轉移)
  p_pred = p_est + Q      (共變異數預測)

更新:
  innov   = z − x_pred    (創新值)
  K       = p_pred / (p_pred + R)   (卡爾曼增益 [0,1])
  x_est   = x_pred + K × innov      (狀態更新)
  p_est   = (1 − K) × p_pred        (後驗共變異數)
```

**自適應過程雜訊 Q**：
```
Q_base   = ucp_kalman_q (預設 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**自適應測量雜訊 R**：
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost 路徑變化檢測（置信度門控 + 冷卻）**：當 `|innovation| > ucp_kalman_q_boost_thresh_val`（預設約 4 ms RTT 偏移）且濾波器已收斂（`p_est ≤ ucp_kalman_converged_p_est_val`，預設 500）時，`p_est` 被重置為 `ucp_kalman_p_est_init_val`，將卡爾曼增益提升至接近 1.0 以快速收斂。兩次連續 qboost 事件之間存在 `ucp_kalman_qboost_cdwn`（預設 15）個樣本的冷卻間隔，防止在高抖動丟包路徑上失控觸發。

**異常值門控**：動態閾值 `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`。僅 `p_pred ≤ ucp_kalman_converged_p_est_val` 時應用。連續 `ucp_kalman_max_consec_reject`（預設 25）次拒絕後強制接受以防止自增強鎖定。

**共變異數匹配雜訊估計（BBR-S）**：`q_est = (1−α) × q_est + α × (K × innov)²`，`r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`。組合模式：mode 0 = 僅啟發式，mode 1 = max（預設），mode 2 = 加權混合。

**卡爾曼接管**：`x_est > 0` 且 `sample_cnt ≥ ucp_kalman_min_samples`（預設 5）時，`min_rtt_us` 被替換為 `x_est / ucp_kalman_scale`。`min_rtt_stamp` 不更新——PROBE_RTT 間隔獨立保留。

**x_est 上界封頂**：卡爾曼推導的 `model_rtt` 被限制在 `min_rtt_us × (100 + ucp_kalman_xest_margin_pct) / 100`（預設 8%）。

## BBR 增強特性

### 增益衰減（Gain Decay）

PROBE_BW 特定相位中透過 256 位元位圖 `ucp_cycle_decay_mask[]` 選擇啟用。衰減公式（已接受卡爾曼樣本觸發）：

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = p_est 的反比縮放 (BBR_UNIT 滿值)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

卡爾曼置信度縮放：`p_est > ucp_kalman_converged_p_est` 時衰減量線性減少，避免濾波器不確定時過度退避。

### ECN 退避

激活條件（全部滿足）：
1. `ucp_ecn_enable_val != 0`
2. 卡爾曼已收斂（`p_est < converged`，`sample_cnt >= min_samples`）
3. `ecn_ewma > 0`（觀測到 CE 標記）
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val`（預設 2000 µs）
5. 模式非 PROBE_BW（PROBE_BW 中 cwnd_gain 固定為 2x）

探測階段（`pacing_gain > BBR_UNIT`）中，ECN 退避按 `BBR_UNIT² / pacing_gain` 比例縮小——1.25x 探測時約保留 80% 退避，2.89x STARTUP 增益時約 65%。

ECN 標記率 EWMA：輪邊界按 `ucp_ecn_ewma_retained / ucp_ecn_ewma_total`（預設 3/4）更新，無 CE 標記的 ACK 按 `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den`（預設 31/32）衰退。

### 單流偵測

當 UCP 偵測到該流在瓶頸處可能為獨享狀態（低佇列延遲、低抖動、無 ECN 標記、無 ACK 聚合、無 LT 頻寬），它會自動切換至 BBR 純淨模式：

- `ucp_get_model_rtt()` 直接回傳 `min_rtt_us`（繞過卡爾曼平滑估計——該估計因單側量測雜訊存在小幅正偏差）。
- `ucp_ecn_backoff()` 回應可透過 `ucp_alone_bypass_ecn`（預設 1）設定——單流路徑不存在其他競爭發送方來共享 ECN 標記，任何標記均為 AQM 誤報，跳過以匹配 BBR 的零 ECN 行為。設為 0 可在獨狼模式下仍保留 ECN 退避（保守模式）。
- LT BW（限速器偵測）條件可透過 `ucp_alone_bypass_lt_bw`（預設 1）設定——單流路徑不存在 policer，LT BW 無法合法觸發，跳過以防止假觸發導致獨狼模式退出。設為 0 恢復原始嚴格行為。

這消除了 UCP 與 BBR 之間的單流吞吐量差距，同時在多流場景下保留了 UCP 的完整保護迴路（卡爾曼、ECN 退避、增益衰減、LT 頻寬）。

**遲滯機制**：進入需要 `ucp_alone_confirm_rounds`（預設 3）個連續合格輪次——防止在多流競爭中的短暫安靜期出現振盪（「保守加速」）。退出即時生效——任何條件不滿足都將清除旗標並重置確認計數器（「激進制動」）。

合格條件（在輪次邊界上必須同時滿足全部六項）：
0. 卡爾曼已收斂（`sample_cnt >= ucp_kalman_min_samples`）—— 信任 qdelay/jitter 作為佇列訊號
1. `qdelay_avg < ucp_alone_qdelay_thresh_us`（預設 1000 us）—— 佇列接近為空
2. `jitter_ewma < ucp_alone_jitter_thresh_us`（預設 2000 us）—— 僅 ACK 時鐘微抖動
3. `ecn_ewma == 0` — AQM 無壅塞標記
4. `lt_use_bw == 0` — 未處於限速器偵測到的速率限制模式
5. `agg_state <= max` 按 `ucp_alone_agg_state_level`（預設 1）—— 三級可配置 ACK 聚合嚴格度：
   - 0 = 僅 IDLE（最嚴格，零聚合），1 = ≤ SUSPECTED（預設，允許瞬時聚合），2 = ≤ CONFIRMED（最寬鬆，僅攔截持續聚合）

### 動態 PROBE_RTT 間隔

將卡爾曼 `p_est` 映射至每連線 PROBE_RTT 間隔：

```
p_est ≤ converged:              interval = dyn_max (預設 30s)
p_est ≥ high (= mult × conv):   interval = base (預設 10s)
converged < p_est < high:       線性插值
```

高置信度（`p_est` 低）時減少 PROBE_RTT 頻率，降低穩定路徑吞吐抖動。低置信度時恢復經典 10s 間隔。

**逐流入口抖動**：為防止所有共存流同時進入 PROBE_RTT（排空至 4 包聚合 ~1.8 Mbps 然後以 2.89× 充填），每個流向其 PROBE_RTT 間隔添加哈希派生的抖動（0–845 ms 分佈）。任意時刻最多 ~1 個流處於 PROBE_RTT，消除了導致 RTO 的同時排空/充填崩潰。

### LT 頻寬估計

遺失事件觸發的下限估計器。取樣間隔長度 [4, 16] RTTs，遺失率 ≥ 5.9%（`ucp_lt_loss_thresh` 預設 15/256）時有效。頻寬 `bw = delivered × BW_UNIT / interval_us`。

與 BBR 的簡單平均不同，UCP 使用可配置 EMA（`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`，預設 1/2 = 0.5）：

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

激活同樣不同：UCP 在首次有效間隔儲存 `lt_bw` 但不設 `lt_use_bw`；需與先前間隔的一致性檢驗——減少測量雜訊導致的錯誤激活。

**雙閾值壅塞門控**：在設置 `lt_use_bw = 1` 之前，評估持久 EWMA 佇列檢查（`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`）和基於 SRTT 的瞬時佇列檢查（`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`，預設 5000 µs）。檢測到壅塞時，LT BW 取樣被中止。SRTT 檢查無需 `ext` 分配即可工作。

LT BW 探測增強（`ucp_lt_bw_probe_pct`，預設 10%）：PROBE_BW 所有相位中 pacing_gain 放大 `1 + probe_pct/100`。斜坡部分：`+1% per 8 RTTs` 持續增加，上限 `2 × probe_pct`。

LT BW 自動恢復（`ucp_lt_restore_ratio_num/den`，預設 5/4 = 1.25x）：`max_bw > lt_bw × ratio` 持續 `ucp_lt_restore_consec_acks`（預設 3）個連續 ACK 時自動退出 LT BW。

### ACK 聚合置信度補償（BBRplus 啟發）

在傳統雙槽額外已確認估計器之上增加置信度門控二層架構。

**四個正交因子**（各貢獻 `ucp_agg_factor_weight` 分，預設 256）：
1. 卡爾曼已收斂（`p_est < converged` + `sample_cnt >= min_samples`）
2. 不在遺失恢復中（`icsk_ca_state < TCP_CA_Recovery`）
3. RTT 在 `min_rtt_us + ucp_agg_factor3_qdelay_us`（預設 2ms）以內
4. `extra_acked` 在視窗最大值的 `ucp_agg_factor4_ratio_num/den`（預設 1.5x）以內

**四個狀態**：IDLE（< `ucp_agg_thresh_suspected`=256）、SUSPECTED（≥256）、CONFIRMED（≥512）、TRUSTED（≥768）。

**訊號層**（始終激活）：置信度線性插值 R 縮放因子 `[r_min, r_max]`。R 即時上升，按 `ucp_agg_r_hysteresis`%（預設保留 75%，~4 RTTs 恢復）每 RTT 遲滯衰退。

**控制層**（`agg_state ≥ CONFIRMED`）：五層安全門控 cwnd 補償。看門狗：連續 `ucp_agg_max_comp_duration`（預設 8）個 RTT 後 CONFIRMED 降級為 SUSPECTED。

### 排水 qdelay_avg 歸零

DRAIN 轉換時 `qdelay_avg` 歸零，防止 STARTUP 佇列估計影響 PROBE_BW。

### TSO 除數自適應

`ucp_min_tso_segs()` 根據卡爾曼狀態調整速率閾值除數：
- 卡爾曼已收斂 + `jitter_ewma < 1000 µs`：除數減半（8→4）
- `jitter_ewma > 4000 µs`：除數加倍（8→16）

## 傳送速率與 Cwnd

### 傳送速率

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // gain 調整
rate = rate × USEC_PER_SEC >> BW_SCALE            // 轉換為 bytes/s
rate = rate × margin_div / 100                    // pacing 餘量（預設 1%，匹配 BBR）
```

速率立即應用（無平滑），匹配 BBR（Cardwell et al. 2016）。`full_bw_reached` 後：所有速率變化立即寫入。STARTUP/DRAIN 中：僅應用增長（`rate > sk_pacing_rate`）。

### Cwnd

```
target = BDP(bw, gain, ext)                       // 基礎 BDP
// 在途資料邊界（非 STARTUP：lo~hi 夾緊；STARTUP：僅 lo 下限）
target = quantization_budget(target)              // TSO 頭部空間 + 偶數捨入 + 相位 0 獎金
target += ack_agg_bonus + agg_compensation        // ACK 聚合補償

if full_bw_reached:
    cwnd = min(cwnd + acked, target)
else (STARTUP):
    cwnd = cwnd + acked

cwnd = max(cwnd, cwnd_min_target)                 // 絕對下限 4
PROBE_RTT 時：cwnd = min(cwnd, cwnd_min_target)   // 最小在途
```

## 模組參數

參數透過 `/proc/sys/net/ucp/` 暴露。寫入觸發 `ucp_init_module_params()`（驗證 + 夾緊 + 計算導數）。陣列參數寫入觸發 `ucp_rebuild_gain_table()`。

### PROBE_RTT 間隔

| 參數 | 預設 | 最小 | 最大 | 單位 | 描述 |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | 基礎 PROBE_RTT 間隔 |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | 長 RTT 路徑的上限 |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | 動態最大間隔；0 禁用 |

### 增益

| 參數 | 預設 | 最小 | 最大 | 描述 |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | PROBE_BW 基線 cwnd 增益 |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK 聚合獎金乘數 |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP 增益（≈2.885x） |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN 增益（≈0.347x） |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | 在途下限（1.25x BDP） |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | 在途上限（2.0x BDP） |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | BBRv1 模式（256 槽） | 0/1 | — | 每槽 pacing 增益 |
| `ucp_cycle_decay_mask[8]` | 0（全零） | 0 | 0x7FFFFFFF | 256 位元衰退位圖 |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | 限制探測提升退出（0=關閉） |

### 卡爾曼基礎

| 參數 | 預設 | 最小 | 最大 | 描述 |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | 基礎過程雜訊 Q |
| `ucp_kalman_r` | 400 | 0 | 100k | 基礎測量雜訊 R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est 絕對上限 |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | 收斂閾值 |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | 初始 p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est 下限 |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | 定點縮放（2 的冪） |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | 接管前的最小樣本數 |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | 異常值基礎閾值 |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-boost 乘數 |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-boost 時間常數 |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Q 上限 |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Q 縮放上限 |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | 強制接受前最大連續拒絕 |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | 卡爾曼 RTT 上限 |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | R 最大增強乘數 |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT 動態上限乘數 |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Q 適應 RTT 除數 |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT 過渡帶乘數 |

### 卡爾曼額外（num/den 型）

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | 異常值抖動乘數 |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q 最小因子 |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est 初始化 RTT 除數 |

### BBR-S 雜訊估計

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q 估計學習率 |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R 估計學習率 |
| `ucp_kalman_noise_mode` | 1 | 0-2 | 組合模式（0=關，1=最大值，2=加權平均） |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Q 估計上限 |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | R 估計上限 |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | 各估計下限 |

### 增益衰退（探測）

| 參數 | 預設 | 範圍 | 單位 | 描述 |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay 衰退閾值 |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay 衰退縮放 |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | 抖動衰退閾值 |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | 抖動衰退縮放 |

### 自適應 R（抖動驅動）

| 參數 | 預設 | 範圍 | 單位 | 描述 |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | R 增加的抖動閾值 |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | R 增加縮放除數 |

### ECN

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | ECN 主開關 |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN 退避比例 |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN qdelay 閾值 |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA 權重 |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | 空閒 ECN 衰退 |

### min_rtt

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | 快速下降計數 |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | 快速下降閾值除數 |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | 黏性下降比例 |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT 守衛比例 |

### LT 頻寬

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | 最小間隔長度 |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | 間隔超時乘數 |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | 最小遺失率 |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | 相對容差 |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | 絕對容差 |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW 最大有效 RTTs |
| `ucp_lt_bw_ema_num` / `ucp_lt_bw_ema_den` | 1 / 2 | 0-100 / 1-100k | LT BW EMA 權重 |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | LT BW 探測增強 |

### LT 自動恢復

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | 恢復觸發比例 |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | 恢復觸發連續 ACK 數 |

### ACK 聚合置信度

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | 主開關 |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | cwnd 補償置信度門限 |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % of BDP | cwnd 補償上限 |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | 看門狗超時 |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | R 遲滯衰退 |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R 縮放範圍（256=1x） |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | 因子 3 qdelay 餘量 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | 因子 4 比例 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | 安全守衛 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | 安全守衛 BDP 乘數 |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked 上限視窗 |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | 看門狗衰退速率 |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | 視窗輪轉間隔 |
| `ucp_agg_factor_weight` | 256 | 1-1024 | 每因子評分 |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | 置信度最大值 |

### EWMA 係數

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA 權重 |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | 抖動 EWMA 權重 |

### 雜項

| 參數 | 預設 | 範圍 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW 循環相位數（2 的冪） |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | 循環相位隨機偏移 |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP 退出增長閾值 |
| `ucp_full_bw_cnt` | 3 | 1-3 | 退出所需的非增長輪數 |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT 駐留時長 |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | pacing 餘量（0=無） |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | 相位 0 cwnd 獎金 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | 輪次 | 頻寬滑動視窗長度 |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | 最小 cwnd（PROBE_RTT） |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt 下限 |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | EDT 近現閾值 |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | TSO 低速率閾值 |
| `ucp_min_tso_rate_div` | 8 | 1-256 | TSO 速率除數（自適應基值） |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | 最大 TSO 分段 |
| `ucp_tso_segs_low` | 1 | 1-65535 | segs | 低速率 TSO 分段數 |
| `ucp_tso_segs_default` | 2 | 1-65535 | segs | 正常速率 TSO 分段數 |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | TSO 頭部空間乘數 |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | 傳送緩衝區擴展因子 |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK 紀元上限 |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | 最大 ACK 聚合視窗 |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | 長 RTT 閾值 |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | 長 RTT 間隔除數 |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | 排水跳過 qdelay 閾值 |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | 輪次 | 啟用單流模式前的確認輪次數 |
| `ucp_alone_qdelay_thresh_us` | 1000 | 0-100k | µs | 單流偵測最大佇列延遲 |
| `ucp_alone_jitter_thresh_us` | 2000 | 0-100k | µs | 單流偵測最大抖動 |
| ucp_alone_agg_state_level | 1 | 0-2 | — | 聚合嚴格度（0=僅IDLE，1=≤SUSPECTED預設，2=≤CONFIRMED過度激進） |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | 獨狼模式下跳過 ECN 退避（1=跳過，0=保持啟用） |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | 獨狼模式下跳過 LT BW 條件檢查（1=跳過，0=保持啟用） |

## 資料路徑

```
ACK 到達 (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► ACK 聚合置信度管線（ucp_agg_enable 時激活）
    │      測量 → 評估 → 狀態映射 → 看門狗
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              滑動視窗最大頻寬
    │      ├── ucp_update_ecn_ewma()        ECN-CE 標記率
    │      ├── ucp_update_ack_aggregation()  雙視窗額外已確認
    │      ├── ucp_update_cycle_phase()      PROBE_BW 階段推進
    │      ├── ucp_check_full_bw_reached()   STARTUP 退出檢測
    │      ├── ucp_check_drain()             DRAIN 進入/退出 + 跳過
    │      ├── ucp_update_min_rtt()          卡爾曼 + 視窗 min-RTT + PROBE_RTT
    │      └── 按模式分配增益
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            ECN 退避（僅 cwnd_gain）
    │
    ├──► ucp_set_pacing_rate()              即時應用，BBR 規則
    │
    └──► ucp_set_cwnd()                     BDP + 邊界 + 聚合補償
```

## 卡爾曼濾波器內部流程

```
RTT 樣本 (rtt_us)
    │
    ├── 無效（≥0 且 < dynamic_max）? 否 → 丟棄
    │
    ├── 冷啟動（sample_cnt==0）? 是 → 初始化：x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           （繞過 RTT 上限門）
    │
    ├── 自適應 Q：Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   自適應 R：R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── 創新值 innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && 冷卻到期？
    │   ├── 是: p_est = p_est_init, 冷卻 = 15, 標記 qboost_fired
    │   └── 否:  冷卻-- 如果激活中
    │
    ├── 預測: p_pred = p_est + Q
    │
    ├── 異常值門控: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── 是且拒絕計數 < max → 拒絕，++consec_reject_cnt，返回
    │   └── 是且拒絕計數 ≥ max → 強制接受（防鎖定）
    │
    └── 卡爾曼更新:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov（截斷至非負並下限）
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── 抖動 EWMA 更新
         ├── qdelay EWMA 更新
         ├── BBR-S 共變異數匹配雜訊估計
         └── sample_cnt++
```

## 診斷

與 BBR 相容的診斷介面透過 `ss -i`（`INET_DIAG_BBRINFO`）報告：

```
bbr_bw_lo/bbr_bw_hi: 64 位元頻寬估計（bytes/s）
bbr_min_rtt:         目前 min_rtt_us
bbr_pacing_gain:     目前 pacing 增益（BBR_UNIT，256=1.0x）
bbr_cwnd_gain:       目前 cwnd 增益（BBR_UNIT）
```

## 使用方式

```sh
# 編譯核心模組
make

# 開發載入（insmod，不解析依賴）
sudo make load

# 安裝並正式載入（modprobe）
sudo make install
sudo make modload

# 卸載
sudo make unload

# 選擇 UCP 算法
echo ucp > /proc/sys/net/ipv4/tcp_congestion_control
```

## 並發與安全模型

UCP 刻意不對自身資料結構使用 READ_ONCE/WRITE_ONCE 或 RCU。此設計與 BBR、CUBIC 等所有核心內 CC 模組一致。

`ucp_init()` 在行程上下文（socket 建立期間）執行，早於 socket 暴露給任何 softirq。`ucp_release()` 在核心保證無 softirq 仍在處理此 socket 的 ACK 後執行。全域模組參數的短暫陳舊值最多影響一個 ACK，下個 ACK 即修正。

唯一例外：`sk->sk_pacing_rate` / `sk->sk_pacing_shift` 為 socket 層欄位，使用者空間可透過 `setsockopt` 同時修改，故保留 BBR 的 WRITE_ONCE/READ_ONCE 慣例。

## 效能總結 (Performance Summary)

測試環境：中國 → 美西 LAX，212ms RTT，8 並行流，26% 丟包率，1 Gbps 共享 VPS 瓶頸。

| 指標 | UCP v1.0 | BBR (對照組) | 差距 |
|------|----------|-------------|------|
| 平均吞吐量 | 1,010 Mbps | 937 Mbps | **+7.8%** |
| UCP 內部不公平度 | 3.1× | 6.2× (BBR) | **−50%** |
| 最差單流 | 60.6 Mbps | 30.8 Mbps | **+97%** |
| 重傳次數 | 150K/10s | 137K/10s | +9.5% |
| 第三輪穩定性 | 959 Mbps | 883 Mbps | **+8.6%** |

重傳略高 —— 這是在高丟包路徑上維持高鏈路利用率的取捨。UCP 的卡爾曼增強 min_rtt 估計提供了更準確的 BDP 基線，使演算法能夠在相同路徑上實現比 BBRv1 更高的吞吐量。

---

*UCP v1.0 — 基於 BBRv1（Cardwell et al. 2016, ACM Queue）和卡爾曼濾波器（Kalman 1960）構建。*

## 參考文獻

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
