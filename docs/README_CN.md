[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP UCP v1.0（Universal Communication Protocol）

面向共享带宽 VPS 环境，结合 BBRv1 状态机与卡尔曼滤波器进行传播延迟估计的 TCP 拥塞控制模块。

## 设计理念

拥塞控制算法需要在吞吐量、延迟、公平性和丢包容忍度之间取得平衡。UCP 采用务实的方案：

1. **BBRv1 提供了经过验证的基础。** 状态机、pacing、周期增益、STARTUP/DRAIN/PROBE_BW/PROBE_RTT —— UCP 直接采用这些机制，不做修改。

2. **卡尔曼滤波器提升估计精度。** 将真实传播延迟从排队延迟和测量抖动中分离出来，得到更准确的 min_rtt 估计，从而实现更紧致的 BDP 计算、更精准的 CWND 校准、更稳定的 pacing。

3. **跨算法动态遵循标准 TCP 竞争平衡。** UCP 不会人为限制发送速率以响应来自外部流的队列信号。增益衰减（基于排队的探测力度降低）可通过 `ucp_cycle_decay_mask` 选择启用，但默认关闭以保持完整探测强度。

4. **主动维护 UCP 内部公平性。** 卡尔曼收敛确保共享瓶颈的 UCP 流获得一致的传播延迟估计，消除了纯 BBR 多流部署中导致严重不公平的赢家通吃反馈循环。

## 算法

TCP UCP 以可加载内核模块 `tcp_ucp.ko` 实现发送端拥塞控制。`ucp_main()` 在每次收到 ACK 时被调用，接收 `rate_sample` 结构（含带宽样本、RTT 样本、交付/丢失计数）。算法在两个时间域运作：**每 ACK 快速路径**更新测量状态并计算即时发送速率和 cwnd；**每轮慢速路径**评估状态转换条件并重新计算增益。

核心测量管线由两部分组成：

1. **滑动窗口最大带宽滤波器**（`linux/win_minmax.h` 的 `minmax_running_max`）：窗口覆盖最近 `ucp_bw_rt_cycle_len`（默认 10）个往返。提供 BBR 兼容的 `max_bw` 估计。

2. **卡尔曼滤波器传播延迟估计**：替代 BBRv1 的滑动窗口最小 RTT。单状态卡尔曼滤波器（Kalman 1960）在 `ucp_kalman_scale` × µs 定点单位下运作，将真实传播延迟建模为随机游走：
   - 状态方程：`x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - 观测方程：`z[k] = x[k] + v[k]`, `v ~ N(0, R)`

定点单位：`BW_UNIT = 1 << 24` 作为带宽单位（segments * 2^24 / µs），`BBR_UNIT = 1 << 8 = 256` 作为无量纲增益单位。

## 状态机

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

使用 `struct ucp` 的 2 位 `mode` 字段编码四种模式：

- **STARTUP (0)**：初始状态。pacing_gain ≈ 2.885x（`ucp_high_gain_val`），cwnd_gain 同为 2.885x。指数级带宽探测。
- **DRAIN (1)**：STARTUP 退出后进入。pacing_gain ≈ 0.347x（`ucp_drain_gain_val`），cwnd_gain 保持 2.885x。排空 STARTUP 累积队列。
- **PROBE_BW (2)**：稳态。按 256 槽位增益表循环（默认 8 相位重复：1.25x/0.75x/8×1.0x）。
- **PROBE_RTT (3)**：定期排空至 `ucp_cwnd_min_target`（默认 4 段）以获取新鲜 RTT 样本。

### STARTUP → DRAIN

当 `full_bw_reached` 置位时触发——`ucp_full_bw_cnt`（默认 3）个连续轮次中 `max_bw` 未能比先前峰值增长至少 `ucp_full_bw_thresh_val`（默认 1.25x）。BDP 在 1.0x 增益下写入 `snd_ssthresh`。`qdelay_avg` 归零以防止 STARTUP 队列积累影响 PROBE_BW。

### DRAIN → PROBE_BW

EDT 在途数据量 ≤ 1.0x BDP 目标时触发。**排水跳过优化**：卡尔曼已收敛且 `qdelay_avg` 低于 `ucp_drain_skip_qdelay_us`（默认 1000 µs）时跳过 DRAIN——提前转 PROBE_BW。

进入 PROBE_BW 时循环相位索引随机化：`cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)`（默认 `len − 1 − rand(8)`），使共享瓶颈的并发流去同步。

### PROBE_BW → PROBE_RTT

PROBE_RTT 间隔到期时触发（`min_rtt_stamp` 在计算间隔内未更新）。cwnd 存至 `prior_cwnd`，pacing 设为排水。

### PROBE_RTT → PROBE_BW

在途数据降至 `ucp_cwnd_min_target` 或轮边界后，驻留至少 `ucp_probe_rtt_mode_ms_val`（默认 200 ms）且至少一轮完整轮次后退出。cwnd 恢复至 `prior_cwnd`，pacing 暂用 `ucp_high_gain_val` 覆盖以快速填充管道。

### 恢复与丢失

- TCP_CA_Loss：`full_bw` 和 `full_bw_cnt` 重置，`round_start` 设 1，`packet_conservation` 清 0。若 LT BW 未激活，注入合成丢失事件触发 LT 采样。
- 恢复入（TCP_CA_Recovery）：启用 `packet_conservation`，cwnd = 在途 + 已 ACK。
- 恢复出：恢复至 `prior_cwnd`，`packet_conservation` 清 0。
- `ucp_undo_cwnd()`：重置 `full_bw` 和 `full_bw_cnt`（保留 `full_bw_reached`），清除 LT BW 状态。

## 核心测量

### 带宽估计

滑动窗口最大带宽滤波器（`minmax_running_max`）在 `ucp_bw_rt_cycle_len`（默认 10）轮窗口。即时 bw = `delivered × BW_UNIT / interval_us`。仅非应用受限或 bw ≥ 现有最大值时馈入滑动窗口（BBR 规则）。

当 `lt_use_bw` 激活时，主动带宽估计切换至 `lt_bw`（LT 带宽估计）。

### 卡尔曼滤波器

单状态标量卡尔曼递推（O(1) 复杂度）：

```
预测:
  x_pred = x_est          (恒等转移)
  p_pred = p_est + Q      (协方差预测)

更新:
  innov   = z − x_pred    (创新值)
  K       = p_pred / (p_pred + R)   (卡尔曼增益 [0,1])
  x_est   = x_pred + K × innov      (状态更新)
  p_est   = (1 − K) × p_pred        (后验协方差)
```

**自适应过程噪声 Q**：
```
Q_base   = ucp_kalman_q (默认 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**自适应测量噪声 R**：
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost 路径变化检测（置信度门控 + 冷却）**：当 `|innovation| > ucp_kalman_q_boost_thresh_val`（默认约 4 ms RTT 偏移）且滤波器已收敛（`p_est ≤ ucp_kalman_converged_p_est_val`，默认 500）时，`p_est` 被重置为 `ucp_kalman_p_est_init_val`，将卡尔曼增益提升至接近 1.0 以快速收敛。两次连续 qboost 事件之间存在 `ucp_kalman_qboost_cdwn`（默认 15）个样本的冷却间隔，防止在高抖动丢包路径上失控触发。

**异常值门控**：动态阈值 `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`。仅 `p_pred ≤ ucp_kalman_converged_p_est_val` 时应用。连续 `ucp_kalman_max_consec_reject`（默认 25）次拒绝后强制接受以防止自增强锁定。

**协方差匹配噪声估计（BBR-S）**：`q_est = (1−α) × q_est + α × (K × innov)²`，`r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`。组合模式：mode 0 = 仅启发式，mode 1 = max（默认），mode 2 = 加权混合。

**卡尔曼接管**：`x_est > 0` 且 `sample_cnt ≥ ucp_kalman_min_samples`（默认 5）时，`min_rtt_us` 被替换为 `x_est / ucp_kalman_scale`。`min_rtt_stamp` 不更新——PROBE_RTT 间隔独立保留。

**x_est 上界封顶**：卡尔曼推导的 `model_rtt` 被限制在 `min_rtt_us × (100 + ucp_kalman_xest_margin_pct) / 100`（默认 8%）。

## BBR 增强特性

### 增益衰减（Gain Decay）

PROBE_BW 特定相位中通过 256 位位图 `ucp_cycle_decay_mask[]` 选择启用。衰减公式（已接受卡尔曼样本触发）：

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = p_est 的反比缩放 (BBR_UNIT 满值)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

卡尔曼置信度缩放：`p_est > ucp_kalman_converged_p_est` 时衰减量线性减少，避免滤波器不确定时过度退避。

### ECN 退避

激活条件（全部满足）：
1. `ucp_ecn_enable_val != 0`
2. 卡尔曼已收敛（`p_est < converged`，`sample_cnt >= min_samples`）
3. `ecn_ewma > 0`（观测到 CE 标记）
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val`（默认 2000 µs）
5. 模式非 PROBE_BW（PROBE_BW 中 cwnd_gain 固定为 2x）

探测阶段（`pacing_gain > BBR_UNIT`）中，ECN 退避按 `BBR_UNIT² / pacing_gain` 比例缩小——1.25x 探测时约保留 80% 退避，2.89x STARTUP 增益时约 65%。

ECN 标记率 EWMA：轮边界按 `ucp_ecn_ewma_retained / ucp_ecn_ewma_total`（默认 3/4）更新，无 CE 标记的 ACK 按 `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den`（默认 31/32）衰退。

### 单流检测

当 UCP 检测到该流在瓶颈处可能为独享状态（低队列延迟、低抖动、无 ECN 标记、无 ACK 聚合、无 LT 带宽），它会自动切换至 BBR 纯净模式：

- `ucp_get_model_rtt()` 直接返回 `min_rtt_us`（绕过卡尔曼平滑估计——该估计因单侧测量噪声存在小幅正偏差）。
- `ucp_ecn_backoff()` 响应可通过 `ucp_alone_bypass_ecn`（默认 1）配置——单流路径不存在其他竞争发送方来共享 ECN 标记，任何标记均为 AQM 误报，跳过以匹配 BBR 的零 ECN 行为。设为 0 可在独狼模式下仍保留 ECN 退避（保守模式）。
- LT BW（限速器检测）条件可通过 `ucp_alone_bypass_lt_bw`（默认 1）配置——单流路径不存在 policer，LT BW 无法合法触发，跳过以防止假触发导致独狼模式退出。设为 0 恢复原始严格行为。

这消除了 UCP 与 BBR 之间的单流吞吐量差距，同时在多流场景下保留了 UCP 的完整保护环路（卡尔曼、ECN 退避、增益衰减、LT 带宽）。

**迟滞机制**：进入需要 `ucp_alone_confirm_rounds`（默认 3）个连续合格轮次——防止在多流竞争中的短暂安静期出现振荡（"保守加速"）。退出即时生效——任何条件不满足都将清除标志并重置确认计数器（"激进制动"）。

合格条件（在轮次边界上必须同时满足全部六项）：
0. 卡尔曼已收敛（`sample_cnt >= ucp_kalman_min_samples`）—— 信任 qdelay/jitter 作为队列信号
1. `qdelay_avg < ucp_alone_qdelay_thresh_us`（默认 1000 us）—— 队列接近为空
2. `jitter_ewma < ucp_alone_jitter_thresh_us`（默认 2000 us）—— 仅 ACK 时钟微抖动
3. `ecn_ewma == 0` — AQM 无拥塞标记
4. `lt_use_bw == 0` — 未处于限速器检测到的速率限制模式
5. `agg_state <= max` 按 `ucp_alone_agg_state_level`（默认 1）—— 三级可配置 ACK 聚合严格度：
   - 0 = 仅 IDLE（最严格，零聚合），1 = ≤ SUSPECTED（默认，允许瞬时聚合），2 = ≤ CONFIRMED（最宽松，仅拦截持续聚合）

### 动态 PROBE_RTT 间隔

将卡尔曼 `p_est` 映射至每连接 PROBE_RTT 间隔：

```
p_est ≤ converged:              interval = dyn_max (默认 30s)
p_est ≥ high (= mult × conv):   interval = base (默认 10s)
converged < p_est < high:       线性插值
```

高置信度（`p_est` 低）时减少 PROBE_RTT 频率，降低稳定路径吞吐抖动。低置信度时恢复经典 10s 间隔。

**逐流入口抖动**：为防止所有共存流同时进入 PROBE_RTT（排空至 4 包聚合 ~1.8 Mbps 然后以 2.89× 充填），每个流向其 PROBE_RTT 间隔添加哈希派生的抖动（0–845 ms 分布）。任意时刻最多 ~1 个流处于 PROBE_RTT，消除了导致 RTO 的同时排空/充填崩溃。

### LT 带宽估计

丢失事件触发的下限估计器。采样间隔长度 [4, 16] RTTs，丢失率 ≥ 5.9%（`ucp_lt_loss_thresh` 默认 15/256）时有效。带宽 `bw = delivered × BW_UNIT / interval_us`。

与 BBR 的简单平均不同，UCP 使用可配置 EMA（`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`，默认 1/2 = 0.5）：

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

激活同样不同：UCP 在首次有效间隔存储 `lt_bw` 但不设 `lt_use_bw`；需与先前间隔的一致性检验——减少测量噪声导致的错误激活。

**双阈值拥塞门控**：在设置 `lt_use_bw = 1` 之前，评估持久 EWMA 队列检查（`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`）和基于 SRTT 的瞬时队列检查（`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`，默认 5000 µs）。检测到拥塞时，LT BW 采样被中止。SRTT 检查无需 `ext` 分配即可工作。

LT BW 探测增强（`ucp_lt_bw_probe_pct`，默认 10%）：PROBE_BW 所有相位中 pacing_gain 放大 `1 + probe_pct/100`。斜坡部分：`+1% per 8 RTTs` 持续增加，上限 `2 × probe_pct`。

LT BW 自动恢复（`ucp_lt_restore_ratio_num/den`，默认 5/4 = 1.25x）：`max_bw > lt_bw × ratio` 持续 `ucp_lt_restore_consec_acks`（默认 3）个连续 ACK 时自动退出 LT BW。

### ACK 聚合置信度补偿（BBRplus 启发）

在传统双槽额外已确认估计器之上增加置信度门控二层架构。

**四个正交因子**（各贡献 `ucp_agg_factor_weight` 分，默认 256）：
1. 卡尔曼已收敛（`p_est < converged` + `sample_cnt >= min_samples`）
2. 不在丢失恢复中（`icsk_ca_state < TCP_CA_Recovery`）
3. RTT 在 `min_rtt_us + ucp_agg_factor3_qdelay_us`（默认 2ms）以内
4. `extra_acked` 在窗口最大值的 `ucp_agg_factor4_ratio_num/den`（默认 1.5x）以内

**四个状态**：IDLE（< `ucp_agg_thresh_suspected`=256）、SUSPECTED（≥256）、CONFIRMED（≥512）、TRUSTED（≥768）。

**信号层**（始终激活）：置信度线性插值 R 缩放因子 `[r_min, r_max]`。R 即时上升，按 `ucp_agg_r_hysteresis`%（默认保留 75%，~4 RTTs 恢复）每 RTT 迟滞衰退。

**控制层**（`agg_state ≥ CONFIRMED`）：五层安全门控 cwnd 补偿。看门狗：连续 `ucp_agg_max_comp_duration`（默认 8）个 RTT 后 CONFIRMED 降级为 SUSPECTED。

### 排水 qdelay_avg 归零

DRAIN 转换时 `qdelay_avg` 归零，防止 STARTUP 队列估计影响 PROBE_BW。

### TSO 除数自适应

`ucp_min_tso_segs()` 根据卡尔曼状态调整速率阈值除数：
- 卡尔曼已收敛 + `jitter_ewma < 1000 µs`：除数减半（8→4）
- `jitter_ewma > 4000 µs`：除数加倍（8→16）

## 发送速率与 Cwnd

### 发送速率

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // gain 调整
rate = rate × USEC_PER_SEC >> BW_SCALE            // 转换为 bytes/s
rate = rate × margin_div / 100                    // pacing 余量（默认 1%，匹配 BBR）
```

速率立即应用（无平滑），匹配 BBR（Cardwell et al. 2016）。`full_bw_reached` 后：所有速率变化立即写入。STARTUP/DRAIN 中：仅应用增长（`rate > sk_pacing_rate`）。

### Cwnd

```
target = BDP(bw, gain, ext)                       // 基础 BDP
// 在途数据边界（非 STARTUP：lo~hi 夹紧；STARTUP：仅 lo 下限）
target = quantization_budget(target)              // TSO 头部空间 + 偶数舍入 + 相位 0 奖金
target += ack_agg_bonus + agg_compensation        // ACK 聚合补偿

if full_bw_reached:
    cwnd = min(cwnd + acked, target)
else (STARTUP):
    cwnd = cwnd + acked

cwnd = max(cwnd, cwnd_min_target)                 // 绝对下限 4
PROBE_RTT 时：cwnd = min(cwnd, cwnd_min_target)   // 最小在途
```

## 模块参数

参数通过 `/proc/sys/net/ucp/` 暴露。写入触发 `ucp_init_module_params()`（验证 + 夹紧 + 计算导数）。数组参数写入触发 `ucp_rebuild_gain_table()`。

### PROBE_RTT 间隔

| 参数 | 默认 | 最小 | 最大 | 单位 | 描述 |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | 基础 PROBE_RTT 间隔 |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | 长 RTT 路径的上限 |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | 动态最大间隔；0 禁用 |

### 增益

| 参数 | 默认 | 最小 | 最大 | 描述 |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | PROBE_BW 基线 cwnd 增益 |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK 聚合奖金乘数 |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP 增益（≈2.885x） |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN 增益（≈0.347x） |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | 在途下限（1.25x BDP） |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | 在途上限（2.0x BDP） |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | BBRv1 模式（256 槽） | 0/1 | — | 每槽 pacing 增益 |
| `ucp_cycle_decay_mask[8]` | 0（全零） | 0 | 0x7FFFFFFF | 256 位衰减位图 |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | 限制探测提升退出（0=关闭） |

### 卡尔曼基础

| 参数 | 默认 | 最小 | 最大 | 描述 |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | 基础过程噪声 Q |
| `ucp_kalman_r` | 400 | 0 | 100k | 基础测量噪声 R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est 绝对上限 |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | 收敛阈值 |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | 初始 p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | p_est 下限 |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | 定点缩放（2 的幂） |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | 接管前的最小样本数 |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | 异常值基础阈值 |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-boost 乘数 |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-boost 时间常数 |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Q 上限 |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Q 缩放上限 |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | 强制接受前最大连续拒绝 |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | µs | 卡尔曼 RTT 上限 |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | R 最大增强乘数 |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT 动态上限乘数 |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Q 适应 RTT 除数 |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT 过渡带乘数 |

### 卡尔曼额外（num/den 型）

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | 异常值抖动乘数 |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q 最小因子 |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est 初始化 RTT 除数 |

### BBR-S 噪声估计

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q 估计学习率 |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R 估计学习率 |
| `ucp_kalman_noise_mode` | 1 | 0-2 | 组合模式（0=关，1=最大值，2=加权平均） |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Q 估计上限 |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | R 估计上限 |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | 各估计下限 |

### 增益衰减（探测）

| 参数 | 默认 | 范围 | 单位 | 描述 |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay 衰减阈值 |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay 衰减缩放 |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | 抖动衰减阈值 |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | 抖动衰减缩放 |

### 自适应 R（抖动驱动）

| 参数 | 默认 | 范围 | 单位 | 描述 |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | R 增加的抖动阈值 |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | R 增加缩放除数 |

### ECN

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | ECN 主开关 |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN 退避比例 |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN qdelay 阈值 |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA 权重 |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | 空闲 ECN 衰减 |

### min_rtt

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | 快速下降计数 |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | 快速下降阈值除数 |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | 粘性下降比例 |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT 守卫比例 |

### LT 带宽

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | 最小间隔长度 |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | 间隔超时乘数 |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | 最小丢失率 |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | 相对容差 |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | 绝对容差 |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW 最大有效 RTTs |
| `ucp_lt_bw_ema_num` / `ucp_lt_bw_ema_den` | 1 / 2 | 0-100 / 1-100k | LT BW EMA 权重 |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | LT BW 探测增强 |

### LT 自动恢复

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | 恢复触发比例 |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | 恢复触发连续 ACK 数 |

### ACK 聚合置信度

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | 主开关 |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | cwnd 补偿置信度门限 |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % of BDP | cwnd 补偿上限 |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | 看门狗超时 |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | R 迟滞衰减 |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R 缩放范围（256=1x） |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | 因子 3 qdelay 余量 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | 因子 4 比例 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | 安全守卫 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | 安全守卫 BDP 乘数 |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked 上限窗口 |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | 看门狗衰减速率 |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | 窗口轮转间隔 |
| `ucp_agg_factor_weight` | 256 | 1-1024 | 每因子评分 |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | 置信度最大值 |

### EWMA 系数

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA 权重 |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | 抖动 EWMA 权重 |

### 杂项

| 参数 | 默认 | 范围 | 描述 |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW 循环相位（2 的幂） |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | 循环相位随机偏移 |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP 退出增长阈值 |
| `ucp_full_bw_cnt` | 3 | 1-3 | 退出所需的非增长轮数 |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT 驻留时长 |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | pacing 余量（0=无） |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | 相位 0 cwnd 奖金 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | 轮次 | 带宽滑动窗口长度 |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | 最小 cwnd（PROBE_RTT） |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt 下限 |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | ns | EDT 近现阈值 |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | TSO 低速率阈值 |
| `ucp_min_tso_rate_div` | 8 | 1-256 | TSO 速率除数（自适应基值） |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | 最大 TSO 分段 |
| `ucp_tso_segs_low` | 1 | 1-65535 | segs | 低速率 TSO 分段数 |
| `ucp_tso_segs_default` | 2 | 1-65535 | segs | 正常速率 TSO 分段数 |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | TSO 头部空间乘数 |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | 发送缓冲区扩展因子 |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK 纪元上限 |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | 最大 ACK 聚合窗口 |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | 长 RTT 阈值 |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | 长 RTT 间隔除数 |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | 排水跳过 qdelay 阈值 |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | 轮次 | 激活单流模式前的确认轮次数 |
| `ucp_alone_qdelay_thresh_us` | 1000 | 0-100k | µs | 单流检测最大队列延迟 |
| `ucp_alone_jitter_thresh_us` | 2000 | 0-100k | µs | 单流检测最大抖动 |
| ucp_alone_agg_state_level | 1 | 0-2 | — | 聚合严格度（0=仅IDLE，1=≤SUSPECTED默认，2=≤CONFIRMED过度激进） |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | 独狼模式下跳过 ECN 退避（1=跳过，0=保持启用） |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | 独狼模式下跳过 LT BW 条件检查（1=跳过，0=保持启用） |

## 数据路径

```
ACK 到达 (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► ACK 聚合置信度管线（ucp_agg_enable 时激活）
    │      测量 → 评估 → 状态映射 → 看门狗
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              滑动窗口最大带宽
    │      ├── ucp_update_ecn_ewma()        ECN-CE 标记率
    │      ├── ucp_update_ack_aggregation()  双窗口额外已确认
    │      ├── ucp_update_cycle_phase()      PROBE_BW 阶段推进
    │      ├── ucp_check_full_bw_reached()   STARTUP 退出检测
    │      ├── ucp_check_drain()             DRAIN 进入/退出 + 跳过
    │      ├── ucp_update_min_rtt()          卡尔曼 + 窗口 min-RTT + PROBE_RTT
    │      └── 按模式分配增益
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            ECN 退避（仅 cwnd_gain）
    │
    ├──► ucp_set_pacing_rate()              即时应用，BBR 规则
    │
    └──► ucp_set_cwnd()                     BDP + 边界 + 聚合补偿
```

## 卡尔曼滤波器内部流程

```
RTT 样本 (rtt_us)
    │
    ├── 无效（≥0 且 < dynamic_max）? 否 → 丢弃
    │
    ├── 冷启动（sample_cnt==0）? 是 → 初始化：x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           （绕过 RTT 上限门）
    │
    ├── 自适应 Q：Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   自适应 R：R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── 创新值 innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && 冷却到期？
    │   ├── 是: p_est = p_est_init, 冷却 = 15, 标记 qboost_fired
    │   └── 否:  冷却-- 如果激活中
    │
    ├── 预测: p_pred = p_est + Q
    │
    ├── 异常值门控: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── 是且拒绝计数 < max → 拒绝，++consec_reject_cnt，返回
    │   └── 是且拒绝计数 ≥ max → 强制接受（防锁定）
    │
    └── 卡尔曼更新:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov（截断至非负并下限）
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── 抖动 EWMA 更新
         ├── qdelay EWMA 更新
         ├── BBR-S 协方差匹配噪声估计
         └── sample_cnt++
```

## 诊断

与 BBR 兼容的诊断接口通过 `ss -i`（`INET_DIAG_BBRINFO`）报告：

```
bbr_bw_lo/bbr_bw_hi: 64 位带宽估计（bytes/s）
bbr_min_rtt:         当前 min_rtt_us
bbr_pacing_gain:     当前 pacing 增益（BBR_UNIT，256=1.0x）
bbr_cwnd_gain:       当前 cwnd 增益（BBR_UNIT）
```

## 使用方式

```sh
# 编译内核模块
make

# 开发加载（insmod，不解析依赖）
sudo make load

# 安装并正式加载（modprobe）
sudo make install
sudo make modload

# 卸载
sudo make unload

# 选择 UCP 算法
echo ucp > /proc/sys/net/ipv4/tcp_congestion_control
```

## 并发与安全模型

UCP 刻意不对自身数据结构使用 READ_ONCE/WRITE_ONCE 或 RCU。此设计与 BBR、CUBIC 等所有内核内 CC 模块一致。

`ucp_init()` 在进程上下文（socket 创建期间）执行，早于 socket 暴露给任何 softirq。`ucp_release()` 在内核保证无 softirq 仍在处理此 socket 的 ACK 后执行。全局模块参数的短暂陈旧值最多影响一个 ACK，下个 ACK 即修正。

唯一例外：`sk->sk_pacing_rate` / `sk->sk_pacing_shift` 为 socket 层字段，用户空间可通过 `setsockopt` 同时修改，故保留 BBR 的 WRITE_ONCE/READ_ONCE 惯例。

## 性能总结 (Performance Summary)

测试环境：中国 → 美西 LAX，212ms RTT，8 并行流，26% 丢包率，1 Gbps 共享 VPS 瓶颈。

| 指标 | UCP v1.0 | BBR (对照组) | 差距 |
|------|----------|-------------|------|
| 平均吞吐量 | 1,010 Mbps | 937 Mbps | **+7.8%** |
| UCP 内部不公平度 | 3.1× | 6.2× (BBR) | **−50%** |
| 最差单流 | 60.6 Mbps | 30.8 Mbps | **+97%** |
| 重传次数 | 150K/10s | 137K/10s | +9.5% |
| 第三轮稳定性 | 959 Mbps | 883 Mbps | **+8.6%** |

重传略高 —— 这是在高丢包路径上维持高链路利用率的取舍。UCP 的卡尔曼增强 min_rtt 估计提供了更准确的 BDP 基线，使算法能够在相同路径上实现比 BBRv1 更高的吞吐量。

---

*UCP v1.0 — 基于 BBRv1（Cardwell et al. 2016, ACM Queue）和卡尔曼滤波器（Kalman 1960）构建。*

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
