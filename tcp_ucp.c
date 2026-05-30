/* ---- tcp_ucp.c : TCP UCP Congestion Control Module v1.0 ------------------ */
/*
 * Universal Communication Protocol (UCP) v1.0
 *
 * A TCP congestion control module for shared-bandwidth VPS environments
 * (e.g. 1-10 Gbps multi-tenant hosting) combining the BBRv1 state machine
 * (Cardwell et al. 2016) with a single-state Kalman filter (Kalman 1960)
 * for propagation-delay estimation.
 *
 * DESIGN PRINCIPLES
 *
 *   Congestion control algorithms must balance throughput, latency,
 *   fairness, and loss tolerance.  UCP takes a pragmatic approach:
 *
 *   1. BBRv1 PROVIDES a well-understood state machine (STARTUP, DRAIN,
 *      PROBE_BW, PROBE_RTT) with proven pacing and gain-cycling
 *      strategies.  UCP adopts these mechanisms without modification.
 *
 *   2. The KALMAN FILTER augments the BBRv1 foundation by estimating
 *      true propagation delay separately from queuing delay and
 *      measurement jitter.  A more accurate min_rtt estimate yields
 *      tighter BDP computation, better-calibrated CWND, and more
 *      stable pacing — all of which improve bandwidth utilisation
 *      and reduce unnecessary queue buildup.
 *
 *   3. Intra-UCP fairness is actively maintained: the Kalman filter
 *      converges all UCP flows sharing a bottleneck to a consistent
 *      propagation-delay estimate, eliminating the winner-takes-all
 *      feedback loop that can cause severe unfairness in pure BBR
 *      multi-flow deployments.  Inter-algorithm dynamics (BBR, CUBIC,
 *      Reno) are left to the standard TCP competitive equilibrium —
 *      UCP neither prioritises nor penalises external flows.
 *
 *   The metrics that guide development: throughput, latency (P95/P99),
 *   retransmit efficiency, convergence time, and jitter.  The Kalman
 *   extensions aim to improve each of these relative to the BBRv1
 *   baseline without sacrificing robustness.
 *
 * ARCHITECTURE
 *
 *   BBRv1 state machine augmented with a single-state Kalman filter
 *   for propagation-delay estimation.  Key deviations from BBRv1:
 *
 *   - min_rtt is estimated by a Kalman filter (state x_est = true
 *     prop delay, covariance p_est) instead of a sliding-window min.
 *   - Directional state update: positive innovations (queue noise)
 *     are skipped — only negative innovations (clean samples) and
 *     qboost-triggered path-change updates enter the filter.
 *   - Confidence-gated qboost: large innovations only trigger gain
 *     reset when the filter is converged (p_est <= converged_p_est),
 *     with a configurable cooldown (ucp_kalman_qboost_cdwn, default 15
 *     samples) between events to prevent runaway on lossy paths.
 *   - Adaptive process/measurement noise (Q, R) based on jitter and
 *     min_rtt, with BBR-S covariance-matched estimation.
 *   - Outlier gating with dynamic threshold derived from jitter_ewma.
 *   - Hysteresis on Kalman takeover (3 confirming rounds).
 *   - Cold-start overshoot correction at sample_cnt==1.
 *   - Gain decay in PROBE_BW (opt-in via ucp_cycle_decay_mask,
 *     disabled by default — conserving full probe intensity).
 *   - Proactive cwnd reduction when Kalman covariance is low and
 *     qdelay rises (ECN backoff).
 *   - Dynamic PROBE_RTT interval based on Kalman covariance p_est.
 *   - Long-Term (LT) bandwidth estimation triggered on loss events.
 *   - Single-flow detection with automatic BBR-pure fallback.
 *
 * REFERENCES
 *
 *   [BBR]   Cardwell et al., "BBR: Congestion-Based Congestion Control",
 *           ACM Queue, Vol. 14 No. 5, 2016.
 *           https://dl.acm.org/doi/10.1145/3009824
 *
 *   [BBR-S] "BBR-S: A Low-Latency BBR Modification for Fast-Varying
 *           Connections", 2021.
 *           https://ieeexplore.ieee.org/document/9438951
 *
 *   [RBBR]  "RBBR: A Receiver-Driven BBR in QUIC for Low-Latency in
 *           Cellular Networks", 2022.
 *           https://ieeexplore.ieee.org/document/9703289
 *
 *   [ERCC]  "ERCC: Fine-grained RDMA Congestion Control via Kalman
 *           Filter-based Multi-bit ECN Feedback Reconstruction", 2025.
 *           https://dl.acm.org/doi/10.1145/3769270.3770124
 *
 *   [Kernel] Linux BBR reference implementation
 *            https://github.com/torvalds/linux/blob/master/net/ipv4/tcp_bbr.c
 *            https://github.com/google/bbr
 *
 *   [BBRplus] "BBRplus: Adaptive Cycle Randomization, Drain-to-Target,
 *             and ACK Aggregation Compensation for BBR Convergence
 *             and Stall Prevention"
 *             https://blog.csdn.net/dog250/article/details/80629551
 *
 *   [IETF101] "BBR Congestion Control Work at Google IETF 101 Update"
 *             https://datatracker.ietf.org/meeting/101/materials/slides-101-iccrg-an-update-on-bbr-work-at-google-00
 *
 * Copyright (c) 2017 ~ 2035 PPP PRIVATE NETWORK(TM) X
 * SPDX-License-Identifier: Dual BSD/GPL
 */

#include <linux/module.h>       /* module init/exit and licensing macros */
#include <linux/version.h>      /* KERNEL_VERSION and LINUX_VERSION_CODE */
#include <net/tcp.h>            /* core TCP structures: tcp_sock, tcp_congestion_ops, rate_sample */
#include <linux/inet_diag.h>    /* INET_DIAG_BBRINFO for ss -i diagnostics */
#include <linux/win_minmax.h>   /* sliding-window max/min (minmax_running_max) */
#include <linux/math64.h>       /* div_u64, mul_u64_u32_shr */
#include <linux/random.h>       /* prandom_u32_max / get_random_u32_below */

 /*
  * BTF (BPF Type Format) / kfunc support for struct_ops BPF programs.
  * UCP_KFUNC decorates callback functions that may be invoked by BPF
  * struct_ops dispatchers.  Pre-5.16 kernels lack kfunc infrastructure;
  * the macro is a no-op on those kernels.
  */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) /* kernel 5.16+ has btf.h */
#include <linux/btf.h>               /* BTF ID macros for kfunc registration */
#include <linux/btf_ids.h>           /* BTF_ID / BTF_ID_FLAGS macro definitions */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0) /* 6.3+ requires __bpf_kfunc */
#define UCP_KFUNC __bpf_kfunc        /* decorate as BPF kernel function (6.3+) */
#else
#define UCP_KFUNC                     /* no-op: kfunc attribute not required */
#endif
#else                                 /* kernel < 5.16: no BTF/kfunc support */
#define UCP_KFUNC                     /* no-op: pre-5.16 kernel */
#endif
  /*
   * BTF set macros were renamed across kernel versions:
   *   6.9+: BTF_KFUNCS_START / BTF_KFUNCS_END
   *   6.0+: BTF_SET8_START / BTF_SET8_END
   *   5.16+: BTF_SET_START / BTF_SET_END
   */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0) /* 6.9+: BTF_KFUNCS_START */
#define BTF_SETS_START(name) BTF_KFUNCS_START(name)   /* 6.9+ kfunc set start */
#define BTF_SETS_END(name)   BTF_KFUNCS_END(name)     /* 6.9+ kfunc set end */
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0) /* 6.0+: BTF_SET8_START */
#define BTF_SETS_START(name) BTF_SET8_START(name)     /* 6.0+ kfunc set start */
#define BTF_SETS_END(name)   BTF_SET8_END(name)       /* 6.0+ kfunc set end */
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) /* 5.16+: BTF_SET_START */
#define BTF_SETS_START(name) BTF_SET_START(name)       /* 5.16+ kfunc set start */
#define BTF_SETS_END(name)   BTF_SET_END(name)         /* 5.16+ kfunc set end */
#endif
   /*
    * Kernel 6.2+ renamed prandom_u32_max() to get_random_u32_below().
    * The wrapper ucp_random_below(x) provides a uniform interface.
    */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0) /* 6.2+ uses get_random_u32_below */
#define ucp_random_below(x) get_random_u32_below(x) /* uniform random [0, x) */
#else                                               /* pre-6.2 uses prandom_u32_max */
#define ucp_random_below(x) prandom_u32_max(x)      /* uniform random [0, x) */
#endif
    /*
     * tcp_snd_cwnd_set() / tcp_snd_cwnd() were introduced in 5.14.
     * Provide inline fallbacks for older kernels.
     */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
static inline void tcp_snd_cwnd_set(struct tcp_sock* tp, u32 val) { WRITE_ONCE(tp->snd_cwnd, val); }
static inline u32 tcp_snd_cwnd(const struct tcp_sock* tp) { return READ_ONCE(tp->snd_cwnd); }
#endif
/* ---- Fixed-Point Scales --------------------------------------------- */
/*
 * BW_SCALE  = 24: bandwidth stored in units of BW_UNIT = segments*(1<<24) per
 *                 usec.  BDP calculation divides by BW_SCALE at the end.
 * BBR_SCALE =  8: pacing_gain and cwnd_gain stored as fixed-point
 *                 multiples of BBR_UNIT = 256.  A gain of 1.0x = BBR_UNIT.
 */
#define BW_SCALE 24            /* bitshift: BW_UNIT = segments*(1<<24) per usec */
#define BW_UNIT  (1 << BW_SCALE)   /* 16777216: for BDP bw * rtt_us calc */
#define BBR_SCALE 8            /* bitshift for BBR_UNIT: 256 = 1.0x gain */
#define BBR_UNIT  (1 << BBR_SCALE) /* 256 = 1.0x gain reference (Cardwell et al. 2016) */

 /*
  * UCP_GAIN_SLOTS = 256: number of phases in the PROBE_BW cycled gain table.
  * UCP_DECAY_MASK_WORDS = 8: bitmask covering 256 bits via 8 x 32-bit words.
  * Each bit controls whether the corresponding cycle phase enables
  * queuing-delay/jitter-based gain decay.
  */
#define UCP_GAIN_SLOTS 256     /* total PROBE_BW gain table entries (0..255) */
#define UCP_DECAY_MASK_WORDS 8 /* 256 bits stored as 8 x 32-bit words */

  /* UCP_KALMAN_INNOV_SQ_CAP: sqrt(S64_MAX) ≈ 3.037e9, cap at 3e9 for headroom */
#define UCP_KALMAN_INNOV_SQ_CAP 3000000000ULL /* overflow guard: sqrt(S64_MAX) */
       /* S64_MAX (2^63-1) stored as unsigned for min_t() type compatibility. */
#define UCP_S64_MAX ((u64)9223372036854775807ULL)

        /* Aggregation confidence state constants (must precede all usages) */
#define UCP_AGG_IDLE      0  /* no aggregation detected or untrusted */
#define UCP_AGG_SUSPECTED 1  /* possible aggregation, only adjust Kalman R */
#define UCP_AGG_CONFIRMED 2  /* confirmed aggregation, R + light cwnd compensation */
#define UCP_AGG_TRUSTED   3  /* highly trusted, full compensation */

#define UCP_DEFAULT_GAIN_CYCLE_LEN 8   /* default number of entries in a single gain cycle */
#define UCP_MIN_RTT_UNINIT        ~0U /* sentinel: min_rtt_us not yet measured. Note: U32_MAX is used as sentinel; all guards check != UCP_MIN_RTT_UNINIT before arithmetic. */
#define UCP_PCT_BASE              100 /* percentage base for ratio/fraction arithmetic */
#define UCP_MSTAMP_HI_SHIFT       32  /* shift for u64-timestamp hi/lo split/recombine */
#define UCP_DECAY_MASK_LSB        1   /* LSB extraction in decay mask bit test */
#define UCP_PROBE_RTT_JITTER_HASH_MASK 0xFF /* per-flow jitter hash mask (sk_hash & MASK) */
#define UCP_PROBE_RTT_JITTER_DIV       64   /* per-flow jitter divisor (spread ~0..2 min_rtt) */
#define UCP_DRAIN_SKIP_MIN_RTT_DIV     8    /* drain-skip guard: min_rtt >> 3 before skip allowed */
#define UCP_DRAIN_TARGET_MAX_RTTS      4    /* drain-to-target safety timeout: max RTTs in drain */
#define UCP_JITTER_SEED_DIV            4    /* cold-start jitter EWMA init seed divisor */
#define UCP_LT_BW_PROBE_RAMP_RTTS      8    /* LT BW probe ramp: 1% bonus per N RTTs */
#define UCP_DECAY_WORD_BITS       5   /* bits per word in decay mask (1<<5 = 32) */
#define UCP_DECAY_BIT_MASK        31  /* bit mask per word in decay mask (32-1) */
#define UCP_CEIL_ADDEND           1   /* addend for ceiling division (e.g. BBR_UNIT - UCP_CEIL_ADDEND) */
#define UCP_EWMA_NEW_WEIGHT       1   /* implicit weight of new sample in EWMA formula */
#define UCP_BITFIELD_2BIT_MAX      3    /* 2-bit bitfield max (2^2 - 1) */
#define UCP_GAIN_MAX              1023 /* 10-bit pacing/cwnd_gain field max (2^10 - 1) */
#define UCP_LT_RTT_CNT_MAX        4095 /* 12-bit lt_rtt_cnt field max (2^12 - 1) */
#define UCP_PROBE_RTT_MAX_SEC     86400 /* PROBE_RTT interval max (seconds, 24h) */
#define UCP_DYN_PROBE_HYPER_NUM    3     /* dynamic probe interval hyper-converged multiplier numerator */
#define UCP_DYN_PROBE_HYPER_DEN    2     /* dynamic probe interval hyper-converged multiplier denominator */
#define UCP_TSO_LOW_JITTER_THRESH_US   1000  /* TSO burst sizing: jitter below 1ms → halve divisor */
#define UCP_TSO_HIGH_JITTER_THRESH_US  4000  /* TSO burst sizing: jitter above 4ms → double divisor */
#define UCP_AGG_CONFIDENCE_MAX         1024  /* ACK aggregation confidence score upper bound */

/* ---- UCP FSM Modes --------------------------------------------------- */
/*
 * UCP state machine mirrors BBRv1 (Cardwell et al. 2016) with four states:
 *
 *   STARTUP   — rapid exponential probing with pacing_gain approx 2.89x.
 *               Exits when full_bw_reached (pipe filled to capacity).
 *
 *   DRAIN     — briefly drains the queue built during STARTUP.
 *               pacing_gain approx 0.35x (less than 1.0).  Exits when
 *               estimated inflight drops to the BDP at 1.0x gain.
 *
 *   PROBE_BW  — steady-state: cycles through a table of pacing gains
 *               (some >1.0x to probe, some =1.0x to cruise, some <1.0x
 *               to drain).  Each phase lasts approx 1 min_rtt.
 *
 *   PROBE_RTT — periodically drains inflight to min_target to obtain
 *               a fresh min_rtt sample.  Triggered when the PROBE_RTT
 *               interval expires without a min_rtt update.
 */
enum ucp_mode {                    /* FSM operating mode, mirrors BBRv1 states */
    UCP_STARTUP = 0,            /* initial rapid probing, high_gain approx 2.89x */
    UCP_DRAIN = 1,            /* drain excess queue after full_bw detected */
    UCP_PROBE_BW = 2,            /* steady-state cycling through probe gains */
    UCP_PROBE_RTT = 3,            /* force min inflight to re-sample min_rtt */
};                                /* enum ucp_mode */

/* ---- Extended State (heap, not size-constrained) --------------------- */
 /*
  * struct ucp_ext - Per-connection extended state (heap-allocated).
  *
  * The base struct ucp must fit within ICSK_CA_PRIV_SIZE (104 bytes).
  * Kalman state, queuing-delay EWMA, jitter EWMA, and ACK-aggregation
  * epoch counters are stored here because they are too large for the
  * in-sock CA slot.
  */
struct ucp_ext {                                     /* extended per-connection state (heap) */
    /* ---- Kalman filter state (Kalman 1960) ---- */
    u32 x_est;                                       /* State: true propagation delay (us * kalman_scale) */
    u32 p_est;                                       /* Error covariance, bounded [floor, max] */

    u32 qdelay_avg;                                  /* EWMA-smoothed queuing delay (us) */

    u32 sample_cnt;                                  /* Number of accepted Kalman updates (Kalman 1960) */

    u32 jitter_ewma;                                 /* EWMA-smoothed absolute innovation (us) */

    /*
     * Consecutive outlier rejection counter.
     * If too many consecutive samples are rejected by the outlier gate,
     * the dynamic threshold (which grows with jitter) could permanently
     * block all future samples, freezing the Kalman filter.
     * After ucp_kalman_max_consec_reject (default 25) consecutive
     * rejections, the next sample is force-accepted regardless of gate.
     */
    u32 consec_reject_cnt;                           /* consecutive outlier rejections */

    /*
     * Covariance-matched noise estimates (BBR-S adaptive Q/R estimation).
     * These are updated on every accepted Kalman sample using:
     *   q_est = (1-alpha) * q_est + alpha * (K * innov)^2 / S^2
     *   r_est = (1-beta)  * r_est + beta  * max(0, innov^2/S^2 - p_pred)
     * (Welch & Bishop 2006, covariance matching method)
     * They serve as a slow calibration channel; the final Q/R used
     * in the filter is max(heuristic_QR, covariance_matched_QR).
     */
    u32 q_est;                                       /* covariance-matched process noise (same units as Q) */
    u32 r_est;                                       /* covariance-matched measurement noise (same units as R) */

    /*
     * ECN (Explicit Congestion Notification) state.
     * When enabled (ucp_ecn_enable != 0), CE-marked segments are tracked
     * via an EWMA of the ECN-mark ratio.  If ecn_ewma > 0 and Kalman
     * qdelay_avg exceeds ucp_ecn_qdelay_thresh_us, cwnd_gain and
     * pacing_gain are reduced proportionally by ucp_ecn_backoff.
     * Scaled to BBR_UNIT (256 = 100%).
     */
    u32 ecn_ewma;                                    /* EWMA of ECN-CE mark ratio in BBR_UNIT (0..256) */
    u32 last_delivered_ce;                           /* tp->delivered_ce at last ECN EWMA update */

    /* ---- ACK aggregation epoch tracking (dual-window sliding max) ---- */
    /*
     * UCP extends BBR's ACK-agg compensation: two alternating
     * windows each spanning approx 5 RTTs, sliding max over each window,
     * and using the maximum of the two as the extra_acked bonus.
     * extra_acked_win_rtts and extra_acked_win_idx are u32 here
     * (heap-allocated) rather than bitfields in BBR's inet_csk_ca slot.
     */
    u64 ack_epoch_mstamp;                            /* tcp_mstamp at epoch start */
    u32 extra_acked[2];                              /* dual-window sliding max (segments) */
    u32 ack_epoch_acked;                             /* bytes ACKed in current epoch (capped approx 1M) */
    u32 extra_acked_win_rtts;                        /* RTTs in current window (0..31) */
    u32 extra_acked_win_idx;                         /* which window is active (0 or 1) */

    /*
     * ACK aggregation confidence-based compensation (BBRplus-inspired).
     * Unlike BBRplus which directly adds extra_acked to cwnd, UCP uses
     * extra_acked as a signal-quality indicator: high aggregation reduces
     * Kalman filter trust in RTT samples (by scaling up measurement noise R)
     * and only enables cwnd compensation at high confidence levels.
     * All fields guarded by ucp_agg_enable module param (default 1).
     */
    u32 agg_extra_acked;                             /* current window extra_acked estimate (segments) */
    u32 agg_extra_acked_max;                         /* windowed maximum (dual-slot) */
    u16 agg_confidence;                              /* confidence score 0..1024 (4-factor evaluation) */
    u8  agg_state;                                   /* 0=IDLE, 1=SUSPECTED, 2=CONFIRMED, 3=TRUSTED */
    u8  agg_comp_duration;                           /* consecutive RTTs with compensation active (watchdog) */
    u32 agg_r_scaled;                                 /* persisting R noise scale for hysteresis (256=1x) */

    /*
     * Dynamic PROBE_RTT interval in jiffies.
     * 0 → use global defaults (ucp_probe_rtt_base_jiffies).
     * Set by ucp_update_dyn_probe_interval() based on p_est.
     */
    u32 dyn_probe_rtt_interval_jiffies;               /* per-connection dynamic probe interval */

    /* ---- Single-flow detection (hysteresis) ---- */
    u8  alone_confirm_cnt;                            /* consecutive rounds qualifying as alone (0..255) */
    u8  qboost_cdwn;                                 /* cooldown counter: minimum samples between qboost events */
};

/*
 * CONCURRENCY & SAFETY MODEL:
 *
 * UCP follows BBR exactly: only socket-layer fields use READ_ONCE/WRITE_ONCE.
 * Module parameters are read directly — a transiently stale value is harmless.
 */
 /* ---- struct ucp_ext (end) ---- */

 /* ---- Per-Connection State (fits ICSK_CA_PRIV_SIZE = 104) ------------ */
 /*
  * struct ucp - Per-connection congestion-control state.
  *
  * Must fit within ICSK_CA_PRIV_SIZE (typically 104 bytes on x86_64).
  * Uses bitfields and careful packing.  Extended state (Kalman, etc.)
  * lives in struct ucp_ext on the heap, pointed to by ucp->ext.
  */
struct ucp {                                          /* per-connection CA state, fits ICSK_CA_PRIV_SIZE */
    /* core measurement state */
    u32 min_rtt_us;                                   /* Current minimum RTT estimate (us) */
    u32 min_rtt_stamp;                                /* tcp_jiffies32 when min_rtt_us last updated */
    u32 probe_rtt_done_stamp;                         /* tcp_jiffies32 deadline to exit PROBE_RTT, 0 = not entered */

    struct minmax bw;                                 /* Sliding-window max bandwidth tracker (win_minmax.h) */

    u32 rtt_cnt;                                      /* Monotonic round-trip counter */
    u32 next_rtt_delivered;                           /* tp->delivered at next expected round boundary */

    u32 cycle_mstamp_lo;                              /* Low 32 bits of PROBE_BW cycle phase MSTAMP */
    u32 cycle_mstamp_hi;                              /* High 32 bits of cycle MSTAMP */

    /* ---- Bitfield word 1: 32 bits (mode + flags + counters) ---- */
    struct {
        u32 mode : 2;                             /* enum ucp_mode (0..3) */
        u32 prev_ca_state : 3;                    /* last TCP CA state (Open/Disorder/Recovery/Loss) */
        u32 round_start : 1;                      /* 1 = this ACK begins a new round */
        u32 idle_restart : 1;                     /* 1 = was app-limited, needs restart logic */
        u32 probe_rtt_round_done : 1;             /* 1 = one round elapsed in PROBE_RTT */
        u32 packet_conservation : 1;              /* 1 = in recovery packet-conservation */
        u32 lt_is_sampling : 1;                   /* 1 = collecting LT BW samples */
        u32 lt_rtt_cnt : 12;                      /* RTT counter for LT interval (0..4095) */
        u32 min_rtt_fast_fall_cnt : 2;            /* sticky counter for fast min_rtt drops */
        u32 cycle_idx : 8;                        /* PROBE_BW cycle phase index (0..255) */
    };

    /* ---- Bitfield word 2: 32 bits (flags + gains in BBR_SCALE) ---- */
    u32 full_bw_reached : 1;                          /* 1 = pipe capacity detected (Cardwell et al. 2016) */
    u32 full_bw_cnt : 2;                              /* consecutive rounds below growth threshold */
    u32 has_seen_rtt : 1;                             /* 1 = tp->srtt_us has been sampled */
    u32 probe_rtt_restored : 1;                       /* 1 = cwnd restore needed after PROBE_RTT exit */
    u32 lt_use_bw : 1;                                /* 1 = pace using lt_bw instead of max_bw */
    u32 lt_restore_cnt : 5;                           /* consecutive ACKs with max_bw > ratio*lt_bw (0..31) */
    u32 pacing_gain : 10;                             /* Current pacing gain (0..1023, BBR_UNIT units) */
    u32 cwnd_gain : 10;                               /* Current cwnd gain (0..1023, BBR_UNIT units) */
    u32 alone_on_path : 1;                            /* 1 = single-flow detected, bypass Kalman/ECN guards */

    /* standalone u32 fields */
    u32 prior_cwnd;                                   /* cwnd saved before recovery or PROBE_RTT */

    u32 full_bw;                                      /* Peak bandwidth when full_bw_reached was set */

    /* ---- LT BW (Long-Term Bandwidth) estimation state ---- */
    /*
     * Activated on loss events when not in lt_use_bw mode.
     * Tracks a stable lower-bound bandwidth estimate over an interval.
     * When lt_bw is consistent over multiple intervals, lt_use_bw = 1
     * and pacing switches to this stable estimate.
     */
    u32 lt_bw;                                        /* Current LT bandwidth estimate (BW_UNIT units) */
    u32 lt_last_delivered;                            /* tp->delivered at start of current LT interval */
    u32 lt_last_stamp;                                /* Timestamp at LT interval start (ms) */
    u32 lt_last_lost;                                 /* tp->lost at start of current LT interval */

    struct ucp_ext* ext;                                    /* Heap-allocated extended state (may be NULL) */
};                                                     /* struct ucp */

/* ---- Module Parameters (num/den pairs, BBR core + Kalman) ---------- */
/*
 * All module parameters are exposed under /proc/sys/net/ucp/.
 * Writing any parameter triggers ucp_init_module_params() which
 * validates, clamps, and computes derived values.
 *
 * Two callback types are used:
 *   ucp_param_ops    — for scalar int params: set → ucp_init_module_params()
 *   ucp_gain_proc_handler — for array params (gain tables, decay mask):
 *                            set → ucp_rebuild_gain_table()
 */

static void ucp_init_module_params(void);             /* forward declaration: clamp + compute derived values */
static void ucp_rebuild_gain_table(void);              /* forward declaration: recompute ucp_cycle_gain_table[] */

/*
 * ucp_param_set_int - Wrapper around param_set_int.
 * After writing a raw parameter, calls ucp_init_module_params() to
 * recompute all derived values (gain tables, jiffies conversions, etc.).
 */
static int ucp_param_set_int(const char* val, const struct kernel_param* kp) /* custom param setter */
{
    int ret = param_set_int(val, kp);                 /* delegate to kernel int setter */
    if (ret == 0) {                                   /* write succeeded */
        ucp_init_module_params();                     /* recompute derived on successful write */
    }

    return ret;                                       /* return set result (0 = success) */
}
static const struct kernel_param_ops ucp_param_ops = { /* parameter ops for scalar int sysctl entries */
    .set = ucp_param_set_int,                         /* custom setter: validate then recompute */
    .get = param_get_int,                             /* standard kernel int getter */
};                                                     /* ucp_param_ops */

/* ---- PROBE_RTT intervals (seconds) ----------------------------------- */
/*
 * ucp_probe_rtt_base_sec  — Base interval between PROBE_RTT episodes (s).
 *                            Used when Kalman confidence is low or Kalman
 *                            not yet converged.  Default 10.
 * ucp_probe_rtt_max_sec   — Maximum interval when min_rtt is "long"
 *                            (> ucp_probe_rtt_long_rtt_us). Default 15.
 * ucp_probe_rtt_dyn_max_sec — Maximum dynamic interval when Kalman
 *                            p_est is at/below converged threshold.
 *                            Default 30.  If 0, dynamic interval disabled.
 */
static int ucp_probe_rtt_base_sec = 10;               /* base PROBE_RTT interval (seconds) */
module_param_cb(ucp_probe_rtt_base_sec, &ucp_param_ops, &ucp_probe_rtt_base_sec, 0644); /* sysctl: ucp_probe_rtt_base_sec */
static int ucp_probe_rtt_max_sec = 15;                /* max PROBE_RTT interval for long RTT paths (seconds) */
module_param_cb(ucp_probe_rtt_max_sec, &ucp_param_ops, &ucp_probe_rtt_max_sec, 0644); /* sysctl: ucp_probe_rtt_max_sec */
static int ucp_probe_rtt_dyn_max_sec = 30;             /* max dynamic interval when Kalman converged (seconds) */
module_param_cb(ucp_probe_rtt_dyn_max_sec, &ucp_param_ops, &ucp_probe_rtt_dyn_max_sec, 0644); /* sysctl: ucp_probe_rtt_dyn_max_sec */

/* ---- Congestion window gain (num/den, default 2x) -------------------- */
/*
 * ucp_cwnd_gain_num / ucp_cwnd_gain_den — Target cwnd multiplier for
 * PROBE_BW mode.  Default num=2, den=1 → 2.0x BDP.
 */
static int ucp_cwnd_gain_num = 2;                     /* CWND gain numerator (default 2x BDP) */
module_param_cb(ucp_cwnd_gain_num, &ucp_param_ops, &ucp_cwnd_gain_num, 0644); /* sysctl: ucp_cwnd_gain_num */
static int ucp_cwnd_gain_den = 1;                     /* CWND gain denominator (default 1) */
module_param_cb(ucp_cwnd_gain_den, &ucp_param_ops, &ucp_cwnd_gain_den, 0644); /* sysctl: ucp_cwnd_gain_den */

/* ---- ACK aggregation compensation gain (num/den, default 1x) --------- */
/*
 * ucp_extra_acked_gain_num / ucp_extra_acked_gain_den — Scaling factor
 * applied to the max extra_acked window value when computing the cwnd
 * bonus for ACK aggregation compensation.
 * Default num=1, den=1 → 1.0x.  Set num=0 to disable compensation.
 */
static int ucp_extra_acked_gain_num = 1;              /* ACK-agg gain numerator (0 disables) */
module_param_cb(ucp_extra_acked_gain_num, &ucp_param_ops, &ucp_extra_acked_gain_num, 0644); /* sysctl: ucp_extra_acked_gain_num */
static int ucp_extra_acked_gain_den = 1;              /* ACK-agg gain denominator (default 1) */
module_param_cb(ucp_extra_acked_gain_den, &ucp_param_ops, &ucp_extra_acked_gain_den, 0644); /* sysctl: ucp_extra_acked_gain_den */

/* ---- PROBE_BW 256-slot gain table + decay mask (num/den) ------------- */
/*
 * ucp_gain_num[i] / ucp_gain_den[i]: pacing gain for phase i of PROBE_BW
 * cycle.  The effective gain = min((num/den)*BBR_UNIT, 1023) is stored
 * in ucp_cycle_gain_table[i].
 *
 * ucp_cycle_decay_mask[]: 256-bit mask (8x32-bit words).
 * If bit i = 1, the gain for phase i is eligible for queuing-delay and
 * jitter-based decay (reduction toward 1.0x).
 * Default: disabled (all zeros).  A common pattern is 0x01010101 per word
 * (every 8th slot, 32 slots total) for selective decay on high-gain phases.
 */
static int ucp_gain_num[UCP_GAIN_SLOTS];              /* PROBE_BW gain numerator array (256 entries) */
static int ucp_gain_den[UCP_GAIN_SLOTS];              /* PROBE_BW gain denominator array (256 entries) */
static bool ucp_gain_table_defaulted = true;            /* true until user writes gain_num/den via sysctl */
static int ucp_cycle_decay_mask[UCP_DECAY_MASK_WORDS] = { /* decay mask: 8x32-bit = 256 bits — default disabled */
    0, 0, 0, 0,                                         /* word[0..3]: no decay slots by default */
    0, 0, 0, 0                                          /* word[4..7]: no decay slots by default */
};                                                      /* 0 decay slots: probing phase preserved at full 1.25x */
/*
 * Custom array setter for gain/decay-mask parameters.
 * Uses the kernel's standard param_array_ops to parse comma-separated
 * integers; after a successful write, calls ucp_rebuild_gain_table()
 * to keep ucp_cycle_gain_table[] consistent.
 * This ensures writes via /sys/module/tcp_ucp/parameters/ and
 * /proc/sys/net/ucp/ (handled by ucp_gain_proc_handler) both
 * trigger a gain-table rebuild.
 */
extern const struct kernel_param_ops param_array_ops;

static int ucp_gain_array_set(const char* val, const struct kernel_param* kp)  /* custom setter: standard parse + rebuild */
{
    int ret = param_array_ops.set(val, kp);                                      /* delegate to kernel array setter */
    if (ret == 0) {                                                               /* write succeeded */
        ucp_gain_table_defaulted = false;                                          /* user explicitly configured gains */
        ucp_rebuild_gain_table();                                                  /* recompute ucp_cycle_gain_table[] */
    }

    return ret;                                                                      /* propagate error or success */
}
/*
 * Custom kernel_param_ops for array parameters.
 * .set wraps param_array_ops.set + ucp_rebuild_gain_table().
 * .get and .free forward to the kernel's standard param_array_ops
 * via local wrapper functions (necessary because param_array_ops
 * is an extern symbol and its members are not compile-time
 * constants for static initializers).
 */
static int ucp_gain_array_get(char* buffer, const struct kernel_param* kp)       /* forwarding wrapper */
{
    return param_array_ops.get(buffer, kp);                                        /* delegate to standard getter */
}

static void ucp_gain_array_free(void* arg)                                        /* forwarding wrapper (noop) */
{
    param_array_ops.free(arg);                                                      /* delegate to standard free */
}

static const struct kernel_param_ops ucp_gain_array_ops = {                     /* custom ops: set triggers rebuild */
    .set = ucp_gain_array_set,                                                  /* custom setter with rebuild hook */
    .get = ucp_gain_array_get,                                                  /* wrapper around param_array_ops.get */
    .free = ucp_gain_array_free,                                                 /* wrapper around param_array_ops.free */
};                                                                               /* ucp_gain_array_ops */

/* kparam_array descriptors: tell the kernel the array layout */
static struct kparam_array __param_arr_ucp_gain_num = {                         /* descriptor for ucp_gain_num[] */
    .max = UCP_GAIN_SLOTS,                                                      /* element count */
    .num = NULL,                                                                /* no runtime count tracking */
    .ops = &param_ops_int,                                                      /* per-element int setter/getter */
    .elem = ucp_gain_num,                                                        /* array base */
};                                                                               /* __param_arr_ucp_gain_num */
static struct kparam_array __param_arr_ucp_gain_den = {                         /* descriptor for ucp_gain_den[] */
    .max = UCP_GAIN_SLOTS,                                                      /* element count */
    .num = NULL,                                                                /* no runtime count tracking */
    .ops = &param_ops_int,                                                      /* per-element int setter/getter */
    .elem = ucp_gain_den,                                                        /* array base */
};                                                                               /* __param_arr_ucp_gain_den */
static struct kparam_array __param_arr_ucp_cycle_decay_mask = {                 /* descriptor for ucp_cycle_decay_mask[] */
    .max = UCP_DECAY_MASK_WORDS,                                                /* element count (8 words) */
    .num = NULL,                                                                /* no runtime count tracking */
    .ops = &param_ops_int,                                                      /* per-element int setter/getter */
    .elem = ucp_cycle_decay_mask,                                                /* array base */
};                                                                               /* __param_arr_ucp_cycle_decay_mask */

module_param_cb(ucp_gain_num, &ucp_gain_array_ops, &__param_arr_ucp_gain_num, 0644);                /* /sys/module + rebuild */
module_param_cb(ucp_gain_den, &ucp_gain_array_ops, &__param_arr_ucp_gain_den, 0644);                /* /sys/module + rebuild */
module_param_cb(ucp_cycle_decay_mask, &ucp_gain_array_ops, &__param_arr_ucp_cycle_decay_mask, 0644); /* /sys/module + rebuild */

/* ---- Kalman filter base noise (raw integer, scaled by kalman_scale) -- */
/*
 * ucp_kalman_q — Base process noise covariance Q (Kalman 1960).
 *                Internally adapted as Q' = Q * max(q_min_factor, min_rtt_us/1000).
 *                Default 100.
 *
 * ucp_kalman_r — Base measurement noise covariance R (Kalman 1960).
 *                Internally adapted as R' = R + (jitter - jr_thresh) * R / jr_scale.
 *                Default 400.
 */
static int ucp_kalman_q = 100;                        /* base process noise covariance (Kalman 1960) */
module_param_cb(ucp_kalman_q, &ucp_param_ops, &ucp_kalman_q, 0644); /* sysctl: ucp_kalman_q */
static int ucp_kalman_r = 400;                        /* base measurement noise covariance (Kalman 1960) */
module_param_cb(ucp_kalman_r, &ucp_param_ops, &ucp_kalman_r, 0644); /* sysctl: ucp_kalman_r */

/*
 * ucp_kalman_q_rtt_div — RTT-to-ms divisor for adaptive Q scaling.
 *     Q scaled by max(q_min_factor, min_rtt_us / div).  On a 100ms path
 *     with div=1000, yields 100× scaling.  default 1000.
 */
static int ucp_kalman_q_rtt_div = 1000;             /* Q adaptation RTT divisor (us → ms) */
module_param_cb(ucp_kalman_q_rtt_div, &ucp_param_ops, &ucp_kalman_q_rtt_div, 0644); /* sysctl */

/* ---- STARTUP high / drain gains (num/den, permille style) ------------ */
/*
 * ucp_high_gain_num / ucp_high_gain_den — pacing gain during STARTUP.
 *   Default: 2885/1000 = 2.885x (BBRv1 standard, Cardwell et al. 2016).
 *
 * ucp_drain_gain_num / ucp_drain_gain_den — pacing gain during DRAIN.
 *   Default: 347/1000 approx 0.347x (BBRv1 drain factor, Cardwell et al. 2016).
 */
static int ucp_high_gain_num = 2885;                  /* STARTUP pacing gain numerator */
module_param_cb(ucp_high_gain_num, &ucp_param_ops, &ucp_high_gain_num, 0644); /* sysctl: ucp_high_gain_num */
static int ucp_high_gain_den = 1000;                  /* STARTUP pacing gain denominator */
module_param_cb(ucp_high_gain_den, &ucp_param_ops, &ucp_high_gain_den, 0644); /* sysctl: ucp_high_gain_den */
static int ucp_drain_gain_num = 347;                  /* DRAIN pacing gain numerator */
module_param_cb(ucp_drain_gain_num, &ucp_param_ops, &ucp_drain_gain_num, 0644); /* sysctl: ucp_drain_gain_num */
static int ucp_drain_gain_den = 1000;                 /* DRAIN pacing gain denominator */
module_param_cb(ucp_drain_gain_den, &ucp_param_ops, &ucp_drain_gain_den, 0644); /* sysctl: ucp_drain_gain_den */

/* ---- PROBE_BW cycle length (int) ------------------------------------- */
/*
 * ucp_probe_bw_cycle_len — Number of phases per PROBE_BW cycle.
 * Rounded up to a power of two (for efficient wrapping via mask).
 * Default 8, range [2, 256].
 */
static int ucp_probe_bw_cycle_len = 8;                /* PROBE_BW cycle length (power-of-two after clamp) */
module_param_cb(ucp_probe_bw_cycle_len, &ucp_param_ops, &ucp_probe_bw_cycle_len, 0644); /* sysctl: ucp_probe_bw_cycle_len */

/* ---- Full BW detection threshold (num/den, default 125/100) ---------- */
/*
 * ucp_full_bw_thresh_num / ucp_full_bw_thresh_den — Growth threshold
 * for full_bw detection.  When max_bw >= full_bw * threshold, bandwidth
 * is still growing.  Default 125/100 = 1.25x (BBRv1, Cardwell et al. 2016).
 *
 * ucp_full_bw_cnt — Number of consecutive rounds below the growth
 * threshold required to declare full_bw_reached.  Default 3.
 */
static int ucp_full_bw_thresh_num = 125;              /* full-BW threshold numerator (growth ratio) */
module_param_cb(ucp_full_bw_thresh_num, &ucp_param_ops, &ucp_full_bw_thresh_num, 0644); /* sysctl: ucp_full_bw_thresh_num */
static int ucp_full_bw_thresh_den = 100;              /* full-BW threshold denominator */
module_param_cb(ucp_full_bw_thresh_den, &ucp_param_ops, &ucp_full_bw_thresh_den, 0644); /* sysctl: ucp_full_bw_thresh_den */
static int ucp_full_bw_cnt = 3;                       /* rounds without growth to declare full_bw */
module_param_cb(ucp_full_bw_cnt, &ucp_param_ops, &ucp_full_bw_cnt, 0644); /* sysctl: ucp_full_bw_cnt */

/* ---- Pacing margin (num/den, default 0/100 = 0%) --------------------- */
/*
 * ucp_pacing_margin_num / ucp_pacing_margin_den — Pacing rate headroom.
 * Effective divisor = 100 - (num*100/den).
 * Default 1/100 -> divisor = 99 -> rate = raw_rate * 99% (matches BBR's 1% margin).
 * Num is capped at 50 to prevent negative divisor.
 */
static int ucp_pacing_margin_num = 1;                 /* pacing margin numerator (default 1% margin to match BBR) */
module_param_cb(ucp_pacing_margin_num, &ucp_param_ops, &ucp_pacing_margin_num, 0644); /* sysctl: ucp_pacing_margin_num */
static int ucp_pacing_margin_den = 100;               /* pacing margin denominator */
module_param_cb(ucp_pacing_margin_den, &ucp_param_ops, &ucp_pacing_margin_den, 0644); /* sysctl: ucp_pacing_margin_den */

/* ---- Inflight gain bounds (num/den, percent style) ------------------- */
/*
 * ucp_inflight_low_gain_num / ucp_inflight_low_gain_den — Lower bound on
 * inflight as multiples of BDP.  Target cwnd >= bdp * low_gain.
 * Default 125/100 = 1.25x (above BBRv1's 1.0x; matches BBRv2).
 * Higher lo keeps inflight elevated during cruise, increasing
 * throughput on under-buffered paths.  100 = BBRv1 parity.
 *
 * ucp_inflight_high_gain_num / ucp_inflight_high_gain_den — Upper bound
 * on inflight in non-STARTUP modes.  Target cwnd <= bdp * high_gain.
 * Default 200/100 = 2.0x (BBRv1 standard).
 */
static int ucp_inflight_low_gain_num = 125;           /* inflight lower bound numerator */
module_param_cb(ucp_inflight_low_gain_num, &ucp_param_ops, &ucp_inflight_low_gain_num, 0644); /* sysctl: ucp_inflight_low_gain_num */
static int ucp_inflight_low_gain_den = 100;           /* inflight lower bound denominator */
module_param_cb(ucp_inflight_low_gain_den, &ucp_param_ops, &ucp_inflight_low_gain_den, 0644); /* sysctl: ucp_inflight_low_gain_den */
static int ucp_inflight_high_gain_num = 200;          /* inflight upper bound numerator */
module_param_cb(ucp_inflight_high_gain_num, &ucp_param_ops, &ucp_inflight_high_gain_num, 0644); /* sysctl: ucp_inflight_high_gain_num */
static int ucp_inflight_high_gain_den = 100;          /* inflight upper bound denominator */
module_param_cb(ucp_inflight_high_gain_den, &ucp_param_ops, &ucp_inflight_high_gain_den, 0644); /* sysctl: ucp_inflight_high_gain_den */

/* ---- Kalman filter bounds (int) -------------------------------------- */
/*
 * ucp_kalman_p_est_max      — Absolute upper bound on p_est.  Default 1,000,000.
 * ucp_kalman_converged_p_est — Threshold for filter convergence (Kalman 1960).
 *                             When p_est < this, cwnd reduction and dynamic
 *                             PROBE_RTT interval logic activate.  Default 500.
 * ucp_kalman_q_boost_mult   — Multiplier for Q-boost threshold.
 *   threshold = mult * ucp_kalman_q_boost_ms * 1000 * kalman_scale.
 *   Default 4.
 * ucp_kalman_q_max          — Ceiling on adaptive Q.  Default 2000.
 * ucp_kalman_q_scale_cap    — Cap on Q adaptation factor (min_rtt_us/1000).
 *                            Default 20.
 * ucp_kalman_min_samples    — Minimum Kalman updates before the filter
 *                            may overwrite min_rtt_us.  Default 5.
 */
static int ucp_kalman_p_est_max = 1000000;            /* absolute upper bound on p_est covariance */
module_param_cb(ucp_kalman_p_est_max, &ucp_param_ops, &ucp_kalman_p_est_max, 0644); /* sysctl: ucp_kalman_p_est_max */
static int ucp_kalman_converged_p_est = 500;          /* p_est convergence threshold (Kalman 1960) */
module_param_cb(ucp_kalman_converged_p_est, &ucp_param_ops, &ucp_kalman_converged_p_est, 0644); /* sysctl: ucp_kalman_converged_p_est */

/*
 * ucp_drain_skip_qdelay_us — When the Kalman filter is converged AND
 *     qdelay_avg is below this threshold (us), skip the drain phase
 *     of the PROBE_BW cycle entirely.  This leverages Kalman's trusted
 *     zero-queue detection to avoid wasting 1/8 of each cycle on
 *     unnecessary draining, converting the drain phase into an
 *     additional cruise phase.  Default 1000 us (1 ms).
 */
static int ucp_drain_skip_qdelay_us = 1000;          /* qdelay below which drain phase is skipped (us) */
module_param_cb(ucp_drain_skip_qdelay_us, &ucp_param_ops, &ucp_drain_skip_qdelay_us, 0644); /* sysctl: ucp_drain_skip_qdelay_us */

/*
 * ucp_kalman_probe_band_mult — Upper bound multiplier for PROBE_RTT
 *     interval transition band.  When p_est is between converged_p_est
 *     and mult × converged_p_est, interval is linearly interpolated.
 *     Above this band, uses base (conservative) interval.  default 4.
 */
static int ucp_kalman_probe_band_mult = 4;          /* probe interval transition band multiplier */
module_param_cb(ucp_kalman_probe_band_mult, &ucp_param_ops, &ucp_kalman_probe_band_mult, 0644); /* sysctl */
static int ucp_kalman_q_boost_mult = 4;               /* Q-boost threshold multiplier */
module_param_cb(ucp_kalman_q_boost_mult, &ucp_param_ops, &ucp_kalman_q_boost_mult, 0644); /* sysctl: ucp_kalman_q_boost_mult */
static int ucp_kalman_q_max = 2000;                   /* maximum adaptive Q (Kalman 1960) */
module_param_cb(ucp_kalman_q_max, &ucp_param_ops, &ucp_kalman_q_max, 0644); /* sysctl: ucp_kalman_q_max */
static int ucp_kalman_q_scale_cap = 20;               /* cap on Q adaptation factor */
module_param_cb(ucp_kalman_q_scale_cap, &ucp_param_ops, &ucp_kalman_q_scale_cap, 0644); /* sysctl: ucp_kalman_q_scale_cap */
static int ucp_kalman_min_samples = 5;                /* min Kalman samples before min_rtt takeover */
module_param_cb(ucp_kalman_min_samples, &ucp_param_ops, &ucp_kalman_min_samples, 0644); /* sysctl: ucp_kalman_min_samples */

/* ---- RTT sample bounds (us, int) ------------------------------------- */
/*
 * ucp_rtt_sample_max_us — RTT samples exceeding this value are discarded
 * by the Kalman filter to prevent extreme outliers from distorting x_est.
 * Default 500,000 us = 500 ms.
 */
static int ucp_rtt_sample_max_us = 500000;            /* discard RTT samples > this value (us) */
module_param_cb(ucp_rtt_sample_max_us, &ucp_param_ops, &ucp_rtt_sample_max_us, 0644); /* sysctl: ucp_rtt_sample_max_us */

/*
 * ucp_kalman_rtt_dyn_mult — Dynamic RTT ceiling multiplier.
 *     rtt_max = max(ucp_rtt_sample_max_us, min_rtt_us * mult).
 *     default 2 → GEO satellite (600ms RTT) lifts 500ms floor to 1.2s.
 */
static int ucp_kalman_rtt_dyn_mult = 2;              /* dynamic RTT ceiling multiplier */
module_param_cb(ucp_kalman_rtt_dyn_mult, &ucp_param_ops, &ucp_kalman_rtt_dyn_mult, 0644); /* sysctl */

/* ---- Min-RTT tracking (num/den, percent style) ----------------------- */
/*
 * ucp_minrtt_fast_fall_cnt   — Consecutive samples below min_rtt_us/4
 *                              needed to force immediate min_rtt drop.
 *                              Default 3 (must fit in 2 bits: 0..3).
 * ucp_minrtt_sticky_num/den  — Sticky ratio for gradual min_rtt decreases.
 *   If new_rtt < min_rtt * sticky_num/sticky_den, min_rtt is reduced
 *   by sticky_num/sticky_den per sample.  Default 75/100 = 0.75.
 * ucp_minrtt_srtt_guard_num/den — SRTT sanity guard: if the smoothed RTT
 *   (SRTT/8) < min_rtt * guard_ratio, min_rtt is overridden by SRTT/8.
 *   Default 90/100 = 0.90.
 */
static int ucp_minrtt_fast_fall_cnt = 3;              /* consecutive fast-fall samples needed (fits 2 bits) */
module_param_cb(ucp_minrtt_fast_fall_cnt, &ucp_param_ops, &ucp_minrtt_fast_fall_cnt, 0644); /* sysctl: ucp_minrtt_fast_fall_cnt */
static int ucp_minrtt_sticky_num = 75;                /* sticky ratio numerator (gradual min_rtt decrease) */
module_param_cb(ucp_minrtt_sticky_num, &ucp_param_ops, &ucp_minrtt_sticky_num, 0644); /* sysctl: ucp_minrtt_sticky_num */
static int ucp_minrtt_sticky_den = 100;               /* sticky ratio denominator */
module_param_cb(ucp_minrtt_sticky_den, &ucp_param_ops, &ucp_minrtt_sticky_den, 0644); /* sysctl: ucp_minrtt_sticky_den */
static int ucp_minrtt_srtt_guard_num = 90;            /* SRTT guard ratio numerator */
module_param_cb(ucp_minrtt_srtt_guard_num, &ucp_param_ops, &ucp_minrtt_srtt_guard_num, 0644); /* sysctl: ucp_minrtt_srtt_guard_num */
static int ucp_minrtt_srtt_guard_den = 100;           /* SRTT guard ratio denominator */
module_param_cb(ucp_minrtt_srtt_guard_den, &ucp_param_ops, &ucp_minrtt_srtt_guard_den, 0644); /* sysctl: ucp_minrtt_srtt_guard_den */

/*
 * ucp_minrtt_fast_fall_div — Divisor for fast-fall threshold.
 *     When new RTT < min_rtt_us / div, immediately commit (bypass sticky).
 *     default 4 → trigger at 25% of min_rtt.
 */
static int ucp_minrtt_fast_fall_div = 4;              /* min_rtt fast-fall threshold divisor */
module_param_cb(ucp_minrtt_fast_fall_div, &ucp_param_ops, &ucp_minrtt_fast_fall_div, 0644); /* sysctl */

/* ---- BDP calculation bounds (us, int) -------------------------------- */
/*
 * ucp_bdp_min_rtt_us — Floor for min_rtt_us in BDP calculation.
 * If model_rtt < this (and Kalman not yet converged), BDP returns
 * TCP_INIT_CWND.  Default 1 us (effectively disabled; matches BBR behavior).
 */
static int ucp_bdp_min_rtt_us = 1;                   /* BDP min-RTT floor (us); below -> TCP_INIT_CWND */
module_param_cb(ucp_bdp_min_rtt_us, &ucp_param_ops, &ucp_bdp_min_rtt_us, 0644); /* sysctl: ucp_bdp_min_rtt_us */

/* ---- TSO/quantization (int) ------------------------------------------ */
/*
 * ucp_probe_cwnd_bonus — Extra segments added to cwnd target during
 * PROBE_BW phase 0 (highest-gain probe).  Helps discover bandwidth
 * above the sliding-window max.  Default 2.
 */
static int ucp_probe_cwnd_bonus = 2;                  /* extra segments in phase 0 probe (Cardwell et al. 2016) */
module_param_cb(ucp_probe_cwnd_bonus, &ucp_param_ops, &ucp_probe_cwnd_bonus, 0644); /* sysctl: ucp_probe_cwnd_bonus */

/* ---- EDT near-now threshold (ns) ------------------------------------- */
/*
 * ucp_edt_near_now_ns — If earliest departure time (EDT) is within this
 * many nanoseconds of now, ucp_packets_in_net_at_edt() treats delivered-
 * at-edt as zero (no packets will drain before next send).
 * Default 1000 ns = 1 us.
 */
static int ucp_edt_near_now_ns = 1000;                /* EDT near-now threshold (ns), default 1 us */
module_param_cb(ucp_edt_near_now_ns, &ucp_param_ops, &ucp_edt_near_now_ns, 0644); /* sysctl: ucp_edt_near_now_ns */

/* ---- TSO rate/segs (int) --------------------------------------------- */
/*
 * ucp_min_tso_rate — Pacing rate (bytes/s) below which min_tso_segs()
 * returns 1 instead of 2.  Reduces bursts on slow paths.
 * Default 1,200,000 bytes/s (approx 9.6 Mbps).
 *
 * ucp_tso_max_segs — Maximum TSO segments per GSO skb.
 * Default 127.
 */
static int ucp_min_tso_rate = 1200000;                /* pacing rate threshold for min TSO segs (bytes/s) */
module_param_cb(ucp_min_tso_rate, &ucp_param_ops, &ucp_min_tso_rate, 0644); /* sysctl: ucp_min_tso_rate */
static int ucp_tso_max_segs = 127;                    /* maximum TSO segments per GSO skb */
module_param_cb(ucp_tso_max_segs, &ucp_param_ops, &ucp_tso_max_segs, 0644); /* sysctl: ucp_tso_max_segs */
/*
 * ucp_tso_segs_low — TSO segments returned by ucp_min_tso_segs() on low-rate
 *     paths (below ucp_min_tso_rate).  Default 1.
 */
static int ucp_tso_segs_low = 1;
module_param_cb(ucp_tso_segs_low, &ucp_param_ops, &ucp_tso_segs_low, 0644);
/*
 * ucp_tso_segs_default — TSO segments returned by ucp_min_tso_segs() on
 *     normal-rate paths.  Default 2.
 */
static int ucp_tso_segs_default = 2;
module_param_cb(ucp_tso_segs_default, &ucp_param_ops, &ucp_tso_segs_default, 0644);

/*
 * ucp_tso_headroom_mult — TSO/GSO headroom multiplier for cwnd target.
 *     cwnd += mult × tso_segs_goal(sk).  default 3 (BBR standard).
 *     Setting 0 disables TSO headroom.
 */
static int ucp_tso_headroom_mult = 3;               /* TSO headroom multiplier (×tso_segs_goal) */
module_param_cb(ucp_tso_headroom_mult, &ucp_param_ops, &ucp_tso_headroom_mult, 0644); /* sysctl */

/*
 * ucp_min_tso_rate_div — Divisor for min_tso_rate comparison.
 *     ucp_min_tso_segs returns 1 if pacing < rate/div, else 2.
 *     default 8 (more generous than BBR's /2).
 */
static int ucp_min_tso_rate_div = 8;                /* TSO rate threshold divisor */
module_param_cb(ucp_min_tso_rate_div, &ucp_param_ops, &ucp_min_tso_rate_div, 0644); /* sysctl */

/* ---- Jitter/Qdelay probe scaling (us) -------------------------------- */
/*
 * ucp_jitter_probe_thresh_us — Jitter threshold above which PROBE_BW
 *     gain decay activates.  Default 4000 us.
 * ucp_jitter_probe_scale_us — Scaling divisor for jitter-based gain
 *     reduction = (jitter - threshold) * BBR_UNIT / scale.  Default 16000 us.
 * ucp_qdelay_probe_thresh_us — Queuing delay threshold for gain decay.
 *     Default 5000 us.
 * ucp_qdelay_probe_scale_us — Scaling divisor for qdelay-based gain
 *     reduction.  Default 20000 us.
 * ucp_jitter_r_thresh_us — Jitter threshold above which measurement noise
 *     R is increased: R' = R + (jitter - thresh) * R / scale.
 *     Default 2000 us.
 * ucp_jitter_r_scale — Scaling divisor for adaptive R.  Default 8000.
 * ucp_kalman_r_max_boost — Maximum multiplier for jitter-based R boost.
 *     R_boost = (jitter - thresh) * base_R / scale, capped at
 *     base_R * r_max_boost.  Prevents extreme R values from freezing
 *     the Kalman gain on paths with persistent high jitter (e.g., WiFi
 *     bursts).  Default 8 → max R ≤ 9× base_R, keeping K ≥ ~10%.
 */
static int ucp_jitter_probe_thresh_us = 4000;         /* jitter threshold for PROBE_BW gain decay (us) */
module_param_cb(ucp_jitter_probe_thresh_us, &ucp_param_ops, &ucp_jitter_probe_thresh_us, 0644); /* sysctl: ucp_jitter_probe_thresh_us */
static int ucp_jitter_probe_scale_us = 16000;          /* jitter scaling divisor for gain decay (us) */
module_param_cb(ucp_jitter_probe_scale_us, &ucp_param_ops, &ucp_jitter_probe_scale_us, 0644); /* sysctl: ucp_jitter_probe_scale_us */
static int ucp_qdelay_probe_thresh_us = 5000;          /* qdelay threshold for PROBE_BW gain decay (us) */
module_param_cb(ucp_qdelay_probe_thresh_us, &ucp_param_ops, &ucp_qdelay_probe_thresh_us, 0644); /* sysctl: ucp_qdelay_probe_thresh_us */
static int ucp_qdelay_probe_scale_us = 20000;          /* qdelay scaling divisor for gain decay (us) */
module_param_cb(ucp_qdelay_probe_scale_us, &ucp_param_ops, &ucp_qdelay_probe_scale_us, 0644); /* sysctl: ucp_qdelay_probe_scale_us */
static int ucp_jitter_r_thresh_us = 2000;               /* jitter threshold for adaptive R (Kalman 1960) */
module_param_cb(ucp_jitter_r_thresh_us, &ucp_param_ops, &ucp_jitter_r_thresh_us, 0644); /* sysctl: ucp_jitter_r_thresh_us */
static int ucp_jitter_r_scale = 8000;                   /* scaling divisor for adaptive R (Kalman 1960) */
module_param_cb(ucp_jitter_r_scale, &ucp_param_ops, &ucp_jitter_r_scale, 0644); /* sysctl: ucp_jitter_r_scale */
static int ucp_kalman_r_max_boost = 8;                 /* max R boost multiplier (prevents gain freeze) */
module_param_cb(ucp_kalman_r_max_boost, &ucp_param_ops, &ucp_kalman_r_max_boost, 0644); /* sysctl: ucp_kalman_r_max_boost */

/* ---- PROBE_RTT trigger thresholds (us) ------------------------------- */
/*
 * ucp_probe_rtt_long_rtt_us — When min_rtt_us exceeds this value,
 * the PROBE_RTT interval is divided by ucp_probe_rtt_long_interval_div
 * (default 1 = no scaling, matching BBR's fixed 10s interval).
 * Default 20,000 us = 20 ms.
 */
static int ucp_probe_rtt_long_rtt_us = 20000;           /* long-RTT threshold (us); interval halved above */
module_param_cb(ucp_probe_rtt_long_rtt_us, &ucp_param_ops, &ucp_probe_rtt_long_rtt_us, 0644); /* sysctl: ucp_probe_rtt_long_rtt_us */

/*
 * ucp_probe_rtt_long_interval_div — Divisor for PROBE_RTT interval on long-RTT
 *     paths.  Interval = base / div when min_rtt > long_rtt_threshold.
 *     default 1 (1 = no scaling, match BBR fixed 10s).  div=1 disables.
 */
static int ucp_probe_rtt_long_interval_div = 1;        /* PROBE_RTT interval divisor for long paths (1 = no scaling, match BBR fixed 10s) */
module_param_cb(ucp_probe_rtt_long_interval_div, &ucp_param_ops, &ucp_probe_rtt_long_interval_div, 0644); /* sysctl */


/* ---- LT BW (Long-Term Bandwidth) ------------------------------------ */
/*
 * ucp_lt_intvl_min_rtts — Minimum RTTs before an LT BW estimate
 *     can be produced.  Default 4.
 * ucp_lt_loss_thresh — Minimum loss ratio (BBR_UNIT units) for the
 *     LT sampling interval to be valid.  Default 15 (approx 5.9%).
 *     Suitable for WiFi/4G/5G (1-5% loss), satellite, and high-
 *     interference links.  Raise to 25-50 for very high loss paths
 *     where LT should rarely trigger.
 * ucp_lt_bw_ratio_num/den — Relative tolerance for LT BW update.
 *     |bw - lt_bw| <= ratio * lt_bw -> accept new estimate.
 *     Default 1/8 = 12.5%.
 * ucp_lt_bw_diff — Absolute byte-rate tolerance for LT BW update.
 *     Default 500 bytes/s.
 * ucp_lt_bw_max_rtts — Maximum RTTs with LT BW active before reset.
 *     Must fit in 12-bit bitfield (< 4095).  Default 48.
 */
static int ucp_lt_intvl_min_rtts = 4;                     /* minimum RTTs for LT BW estimate validity */
module_param_cb(ucp_lt_intvl_min_rtts, &ucp_param_ops, &ucp_lt_intvl_min_rtts, 0644); /* sysctl: ucp_lt_intvl_min_rtts */
static int ucp_lt_loss_thresh = 15;                       /* minimum loss ratio (BBR_UNIT, 15=5.9%) for LT interval */
module_param_cb(ucp_lt_loss_thresh, &ucp_param_ops, &ucp_lt_loss_thresh, 0644); /* sysctl: ucp_lt_loss_thresh */
static int ucp_lt_intvl_max_mult = 4;                    /* LT BW sampling timeout = mult * min_rtts */
module_param_cb(ucp_lt_intvl_max_mult, &ucp_param_ops, &ucp_lt_intvl_max_mult, 0644); /* sysctl */
static int ucp_lt_bw_ratio_num = 1;                       /* LT BW relative tolerance numerator */
module_param_cb(ucp_lt_bw_ratio_num, &ucp_param_ops, &ucp_lt_bw_ratio_num, 0644); /* sysctl: ucp_lt_bw_ratio_num */
static int ucp_lt_bw_ratio_den = 8;                       /* LT BW relative tolerance denominator */
module_param_cb(ucp_lt_bw_ratio_den, &ucp_param_ops, &ucp_lt_bw_ratio_den, 0644); /* sysctl: ucp_lt_bw_ratio_den */
static int ucp_lt_bw_diff = 500;                          /* LT BW absolute byte-rate tolerance (bytes/s) */
module_param_cb(ucp_lt_bw_diff, &ucp_param_ops, &ucp_lt_bw_diff, 0644); /* sysctl: ucp_lt_bw_diff */
/*
 * ucp_lt_bw_ema_num / _den — LT BW EMA update coefficients.
 *     lt_bw = (new * num + old * (den - num)) / den.
 *     Default 1/2 gives exponential moving average.
 */
static int ucp_lt_bw_ema_num = 1;
module_param_cb(ucp_lt_bw_ema_num, &ucp_param_ops, &ucp_lt_bw_ema_num, 0644);
static int ucp_lt_bw_ema_den = 2;
module_param_cb(ucp_lt_bw_ema_den, &ucp_param_ops, &ucp_lt_bw_ema_den, 0644);
static int ucp_lt_bw_probe_pct = 10;                       /* LT BW probe percentage above 1.0x (0-100) */
module_param_cb(ucp_lt_bw_probe_pct, &ucp_param_ops, &ucp_lt_bw_probe_pct, 0644); /* sysctl: ucp_lt_bw_probe_pct */
static int ucp_lt_bw_inst_qdelay_thresh_us = 5000;      /* LT BW gate: instantaneous qdelay threshold (µs) */
module_param_cb(ucp_lt_bw_inst_qdelay_thresh_us, &ucp_param_ops, &ucp_lt_bw_inst_qdelay_thresh_us, 0644); /* sysctl: ucp_lt_bw_inst_qdelay_thresh_us */
static int ucp_lt_bw_max_rtts = 48;                       /* max RTTs with LT BW before reset (fits 12 bits) */
module_param_cb(ucp_lt_bw_max_rtts, &ucp_param_ops, &ucp_lt_bw_max_rtts, 0644); /* sysctl: ucp_lt_bw_max_rtts */

/* ---- LT BW auto-recovery (num/den ratio, int counter) ----------------- */
/*
 * ucp_lt_restore_ratio_num / ucp_lt_restore_ratio_den — When the
 *     sliding-window max bandwidth exceeds lt_bw by this ratio over
 *     ucp_lt_restore_consec_acks consecutive ACKs, LT BW mode is
 *     automatically exited and normal PROBE_BW probing resumes.
 *     Default 5/4 = 1.25x.  Set num=0 or den=0 to disable auto-recovery.
 *
 * ucp_lt_restore_consec_acks — Number of consecutive ACKs where
 *     max_bw > lt_bw * num/den must hold before triggering recovery.
 *     Must fit in the 5-bit lt_restore_cnt bitfield (max 31).
 *     Default 3.
 */
static int ucp_lt_restore_ratio_num = 5;                 /* LT BW auto-recovery ratio numerator (default 5/4 = 1.25x) */
module_param_cb(ucp_lt_restore_ratio_num, &ucp_param_ops, &ucp_lt_restore_ratio_num, 0644); /* sysctl */
static int ucp_lt_restore_ratio_den = 4;                 /* LT BW auto-recovery ratio denominator (default 5/4 = 1.25x) */
module_param_cb(ucp_lt_restore_ratio_den, &ucp_param_ops, &ucp_lt_restore_ratio_den, 0644); /* sysctl */
static int ucp_lt_restore_consec_acks = 3;               /* consecutive ACK threshold for LT BW auto-recovery (max 31) */
module_param_cb(ucp_lt_restore_consec_acks, &ucp_param_ops, &ucp_lt_restore_consec_acks, 0644); /* sysctl */

/* ---- Kalman filter core constants (Kalman 1960) -------------------- */
/*
 * ucp_kalman_p_est_init   — Initial p_est on cold start or Q-boost reset.
 *                           Default 1000.
 * ucp_kalman_p_est_floor  — Lower bound for posterior p_est.
 *                           Default 10.
 * ucp_kalman_outlier_ms   — Base outlier threshold in milliseconds.
 *   Effective threshold = max(outlier_ms * 1000 * scale,
 *                             jitter_ewma * outlier_jitter_mult * scale).
 *   Default 5 ms.
 * ucp_kalman_q_boost_ms   — Time constant for Q-boost threshold (ms).
 *   Default 1 ms.
 * ucp_kalman_scale        — Fixed-point scaling factor for the Kalman state.
 *   x_est = measured in rtt_us * scale units.
 *   Rounded up to next power of two for fast division via shift.
 *   Default 1024.
 */
static int ucp_kalman_p_est_init = 1000;                  /* initial p_est on cold start (Kalman 1960) */
module_param_cb(ucp_kalman_p_est_init, &ucp_param_ops, &ucp_kalman_p_est_init, 0644); /* sysctl: ucp_kalman_p_est_init */
static int ucp_kalman_p_est_floor = 10;                   /* lower bound for posterior p_est (Kalman 1960) */
module_param_cb(ucp_kalman_p_est_floor, &ucp_param_ops, &ucp_kalman_p_est_floor, 0644); /* sysctl: ucp_kalman_p_est_floor */
static int ucp_kalman_outlier_ms = 5;                     /* base outlier gate threshold (ms) */
module_param_cb(ucp_kalman_outlier_ms, &ucp_param_ops, &ucp_kalman_outlier_ms, 0644); /* sysctl: ucp_kalman_outlier_ms */
static int ucp_kalman_q_boost_ms = 1;                     /* Q-boost time constant (ms) */
module_param_cb(ucp_kalman_q_boost_ms, &ucp_param_ops, &ucp_kalman_q_boost_ms, 0644); /* sysctl: ucp_kalman_q_boost_ms */
static int ucp_kalman_qboost_cdwn = 15;              /* Q-boost cooldown: min samples between events (prevents runaway on lossy paths) */
module_param_cb(ucp_kalman_qboost_cdwn, &ucp_param_ops, &ucp_kalman_qboost_cdwn, 0644); /* sysctl: ucp_kalman_qboost_cdwn */
static int ucp_kalman_xest_margin_pct = 8;           /* x_est margin above min_rtt in percent (0..100, default 8) */
module_param_cb(ucp_kalman_xest_margin_pct, &ucp_param_ops, &ucp_kalman_xest_margin_pct, 0644); /* sysctl: ucp_kalman_xest_margin_pct */
static int ucp_kalman_scale = 1024;                       /* Kalman fixed-point scale (power-of-two) */
module_param_cb(ucp_kalman_scale, &ucp_param_ops, &ucp_kalman_scale, 0644); /* sysctl: ucp_kalman_scale */

/* ---- Kalman filter extra num/den tunables (Kalman 1960) ------------ */
/*
 * ucp_kalman_outlier_jitter_mult_num/den — Jitter multiplier for
 *     dynamic outlier threshold.  Default 4/1 = 4.
 * ucp_kalman_q_min_factor_num/den — Minimum multiplier for adaptive Q.
 *     Q_adapted = Q * max(factor, min_rtt_us/1000).  Default 10/1 = 10.
 * ucp_kalman_p_est_init_rtt_div_num/den — Alternate p_est initializer
 *     in terms of RTT: p_est = max(p_est_init, rtt_us / div).
 *     Default 10/1 = 10.
 */
static int ucp_kalman_outlier_jitter_mult_num = 4;       /* outlier jitter multiplier numerator */
module_param_cb(ucp_kalman_outlier_jitter_mult_num, &ucp_param_ops, &ucp_kalman_outlier_jitter_mult_num, 0644); /* sysctl */
static int ucp_kalman_outlier_jitter_mult_den = 1;       /* outlier jitter multiplier denominator */
module_param_cb(ucp_kalman_outlier_jitter_mult_den, &ucp_param_ops, &ucp_kalman_outlier_jitter_mult_den, 0644); /* sysctl */
static int ucp_kalman_q_min_factor_num = 10;              /* Q min factor numerator */
module_param_cb(ucp_kalman_q_min_factor_num, &ucp_param_ops, &ucp_kalman_q_min_factor_num, 0644); /* sysctl */
static int ucp_kalman_q_min_factor_den = 1;               /* Q min factor denominator */
module_param_cb(ucp_kalman_q_min_factor_den, &ucp_param_ops, &ucp_kalman_q_min_factor_den, 0644); /* sysctl */
static int ucp_kalman_p_est_init_rtt_div_num = 10;        /* p_est init RTT divisor numerator */
module_param_cb(ucp_kalman_p_est_init_rtt_div_num, &ucp_param_ops, &ucp_kalman_p_est_init_rtt_div_num, 0644); /* sysctl */
static int ucp_kalman_p_est_init_rtt_div_den = 1;         /* p_est init RTT divisor denominator */
module_param_cb(ucp_kalman_p_est_init_rtt_div_den, &ucp_param_ops, &ucp_kalman_p_est_init_rtt_div_den, 0644); /* sysctl */

/*
 * ucp_ewma_qdelay_num/den — EWMA for qdelay smoothing.
 *   qdelay_avg = (qdelay_avg * num + new * 1) / den.
 *   Default 7/8 -> weight 0.875 old, 0.125 new.
 *
 * ucp_ewma_jitter_num/den — EWMA for jitter smoothing.
 *   jitter_avg = (jitter_avg * num + new * 1) / den.
 *   Default 7/8 -> weight 0.875 old, 0.125 new.
 */
static int ucp_ewma_qdelay_num = 7;                      /* EWMA qdelay numerator (old weight) */
module_param_cb(ucp_ewma_qdelay_num, &ucp_param_ops, &ucp_ewma_qdelay_num, 0644); /* sysctl: ucp_ewma_qdelay_num */
static int ucp_ewma_qdelay_den = 8;                      /* EWMA qdelay denominator (total) */
module_param_cb(ucp_ewma_qdelay_den, &ucp_param_ops, &ucp_ewma_qdelay_den, 0644); /* sysctl: ucp_ewma_qdelay_den */
static int ucp_ewma_jitter_num = 7;                      /* EWMA jitter numerator (old weight) */
module_param_cb(ucp_ewma_jitter_num, &ucp_param_ops, &ucp_ewma_jitter_num, 0644); /* sysctl: ucp_ewma_jitter_num */
static int ucp_ewma_jitter_den = 8;                      /* EWMA jitter denominator (total) */
module_param_cb(ucp_ewma_jitter_den, &ucp_param_ops, &ucp_ewma_jitter_den, 0644); /* sysctl: ucp_ewma_jitter_den */

/* ---- BBR-S Covariance-Matched Noise Estimation (num/den ratios) ----- */
/*
 * ucp_kalman_noise_alpha_num / ucp_kalman_noise_alpha_den — Learning rate
 *     for covariance-matched Q estimation (BBR-S method).  alpha controls
 *     how quickly q_est adapts: q_est = q_est*(1-alpha) + alpha*(K*innov)^2.
 *     Default 1/10 = 0.1.
 *
 * ucp_kalman_noise_beta_num / ucp_kalman_noise_beta_den — Learning rate
 *     for covariance-matched R estimation.
 *     Default 1/10 = 0.1.
 *
 * ucp_kalman_q_est_max — Upper bound on q_est.  Default 1,000,000,000.
 *     q_est is in (us * kalman_scale)^2 units (same implicit scale as Q).
 *     For a 10us innovation at K~0.5: (K*innov)^2 ~ 2.6e7, well within bound.
 * ucp_kalman_r_est_max — Upper bound on r_est.  Default 1,000,000,000.
 *     r_est is in (us * kalman_scale)^2 units (same implicit scale as R).
 * ucp_kalman_q_est_floor — Lower bound on q_est.  Default 1.
 * ucp_kalman_r_est_floor — Lower bound on r_est.  Default 1.
 *
 * ucp_kalman_noise_mode — Selects how covariance-matched estimates
 *     combine with heuristic Q/R:
 *       0 = disabled (use heuristic only)
 *       1 = max(heuristic, matched)  — conservative (default)
 *       2 = weighted blend (num/den configurable via noise_avg) — default (1/2) avg
 */
static int ucp_kalman_noise_alpha_num = 1;               /* adaptive Q learning rate numerator (BBR-S) */
module_param_cb(ucp_kalman_noise_alpha_num, &ucp_param_ops, &ucp_kalman_noise_alpha_num, 0644); /* sysctl: ucp_kalman_noise_alpha_num */
static int ucp_kalman_noise_alpha_den = 10;               /* adaptive Q learning rate denominator */
module_param_cb(ucp_kalman_noise_alpha_den, &ucp_param_ops, &ucp_kalman_noise_alpha_den, 0644); /* sysctl: ucp_kalman_noise_alpha_den */
static int ucp_kalman_noise_beta_num = 1;                 /* adaptive R learning rate numerator (BBR-S) */
module_param_cb(ucp_kalman_noise_beta_num, &ucp_param_ops, &ucp_kalman_noise_beta_num, 0644); /* sysctl: ucp_kalman_noise_beta_num */
static int ucp_kalman_noise_beta_den = 10;                /* adaptive R learning rate denominator */
module_param_cb(ucp_kalman_noise_beta_den, &ucp_param_ops, &ucp_kalman_noise_beta_den, 0644); /* sysctl: ucp_kalman_noise_beta_den */
static int ucp_kalman_q_est_max = 1000000000;              /* upper bound on covariance-matched Q estimate */
module_param_cb(ucp_kalman_q_est_max, &ucp_param_ops, &ucp_kalman_q_est_max, 0644); /* sysctl: ucp_kalman_q_est_max */
static int ucp_kalman_r_est_max = 1000000000;              /* upper bound on covariance-matched R estimate */
module_param_cb(ucp_kalman_r_est_max, &ucp_param_ops, &ucp_kalman_r_est_max, 0644); /* sysctl: ucp_kalman_r_est_max */
static int ucp_kalman_q_est_floor = 1;                    /* lower bound on covariance-matched Q estimate */
module_param_cb(ucp_kalman_q_est_floor, &ucp_param_ops, &ucp_kalman_q_est_floor, 0644); /* sysctl: ucp_kalman_q_est_floor */
static int ucp_kalman_r_est_floor = 1;                    /* lower bound on covariance-matched R estimate */
module_param_cb(ucp_kalman_r_est_floor, &ucp_param_ops, &ucp_kalman_r_est_floor, 0644); /* sysctl: ucp_kalman_r_est_floor */
static int ucp_kalman_noise_mode = 1;                     /* combination mode: 0=off, 1=max, 2=weighted blend (BBR-S integration) */
module_param_cb(ucp_kalman_noise_mode, &ucp_param_ops, &ucp_kalman_noise_mode, 0644); /* sysctl: ucp_kalman_noise_mode */
/*
 * ucp_kalman_noise_avg_num / _den — Weighted blend ratio for noise mode 2.
 *     blend = (heuristic × (den−num) + matched × num) / den.
 *     Default 1/2 gives simple average (heuristic + matched) / 2.
 */
static int ucp_kalman_noise_avg_num = 1;
module_param_cb(ucp_kalman_noise_avg_num, &ucp_param_ops, &ucp_kalman_noise_avg_num, 0644);
static int ucp_kalman_noise_avg_den = 2;
module_param_cb(ucp_kalman_noise_avg_den, &ucp_param_ops, &ucp_kalman_noise_avg_den, 0644);

/* ---- Single-flow detection hysteresis --------------------------------- */
/*
 * ucp_alone_confirm_rounds — Number of consecutive qualifying rounds
 *     (low qdelay + low jitter + no ECN + no agg + no LT BW) before
 *     activating single-flow mode.  Higher values add hysteresis to
 *     prevent oscillation during brief quiet periods in multi-flow
 *     competition.  Default 3 rounds.  Range [1, 32].
 */
static int ucp_alone_confirm_rounds = 3;             /* qualifying rounds before alone mode */
module_param_cb(ucp_alone_confirm_rounds, &ucp_param_ops, &ucp_alone_confirm_rounds, 0644);

/*
 * ucp_alone_qdelay_thresh_us — Queuing delay upper bound (us) for
 *     single-flow detection.  qdelay_avg must be strictly below this
 *     value for the flow to qualify as alone.  Default 1000 us (1 ms).
 *     Lower = stricter (only enters on truly idle paths); higher =
 *     more permissive (enters even with minor queue).
 *     Range [0, 100000] us.
 */
static int ucp_alone_qdelay_thresh_us = 1000;          /* max qdelay for single-flow detection (us) */
module_param_cb(ucp_alone_qdelay_thresh_us, &ucp_param_ops, &ucp_alone_qdelay_thresh_us, 0644);

/*
 * ucp_alone_jitter_thresh_us — Jitter upper bound (us) for
 *     single-flow detection.  jitter_ewma must be strictly below this
 *     value.  Competing flows induce inter-packet timing variance;
 *     a quiet single-flow path shows only ACK-clock micro-jitter.
 *     Default 2000 us (2 ms).  Range [0, 100000] us.
 */
static int ucp_alone_jitter_thresh_us = 2000;           /* max jitter for single-flow detection (us) */
module_param_cb(ucp_alone_jitter_thresh_us, &ucp_param_ops, &ucp_alone_jitter_thresh_us, 0644);

/*
 * ucp_alone_agg_state_level — ACK aggregation strictness for single-flow
 *     detection.  Controls how much aggregation is tolerated before
 *     disqualifying the flow from alone mode:
 *       0 = IDLE only     (strict: zero aggregation; highest safety)
 *       1 = ≤ SUSPECTED   (moderate: allow transient agg; default)
 *       2 = < CONFIRMED   (permissive: block only persistent agg)
 *     Default 1.  Range [0, 2].
 */
static int ucp_alone_agg_state_level = 1;              /* agg strictness level for alone detection [0,2] */
module_param_cb(ucp_alone_agg_state_level, &ucp_param_ops, &ucp_alone_agg_state_level, 0644);

/*
 * ucp_alone_bypass_ecn — When alone_on_path is active, skip ECN backoff
 *     (default 1).  On a single-flow path there is no competing sender
 *     to share ECN marks with; any marks are false positives from an
 *     over-sensitive AQM.  Bypassing matches BBR's zero-ECN behavior
 *     and recovers the throughput gap in single-flow scenarios.
 *     Set to 0 to keep ECN backoff active even when alone (conservative).
 *     Range [0, 1].
 */
static int ucp_alone_bypass_ecn = 1;                   /* bypass ECN when alone (default: skip) */
module_param_cb(ucp_alone_bypass_ecn, &ucp_param_ops, &ucp_alone_bypass_ecn, 0644);

/*
 * ucp_alone_bypass_lt_bw — When alone_on_path is active, ignore LT BW
 *     (policer-detected rate-limited mode) in the qualification check
 *     (default 1).  A single-flow path has no policer, so LT BW cannot
 *     legitimately activate.  Bypassing avoids spurious alone-mode exit
 *     if a false LT BW trigger occurs from measurement noise.
 *     When set to 0, LT BW activation always forces exit from alone mode,
 *     matching the original hardcoded behavior.
 *     Range [0, 1].
 */
static int ucp_alone_bypass_lt_bw = 1;                 /* ignore LT BW in alone qualification (default: skip) */
module_param_cb(ucp_alone_bypass_lt_bw, &ucp_param_ops, &ucp_alone_bypass_lt_bw, 0644);

/* ---- ECN (Explicit Congestion Notification) --------------------------- */
/*
 * ucp_ecn_enable — Master switch for ECN-aware backoff.
 *     0 = disabled (no ECN tracking, zero overhead).
 *     1 = enabled (default).  Reads tp->delivered_ce from the TCP stack.
 *
 * ucp_ecn_backoff_num / ucp_ecn_backoff_den — Fraction by which
 *     cwnd_gain and pacing_gain (if > 1.0x) are reduced when ECN
 *     conditions are met (see ucp_ecn_backoff() for full trigger logic).
 *     Default (20/100) × BBR_UNIT ≈ 20% reduction.
 *
 * ucp_ecn_qdelay_thresh_us — Qdelay threshold (us) for ECN backoff
 *     activation via queue buildup.  Default 2000 us.
 * ucp_ecn_ewma_retained / ucp_ecn_ewma_total — EWMA weights for ECN
 *     mark ratio.  Default 3/4 -> weight 0.75 old, 0.25 new.
 */

static int ucp_ecn_enable = 1;                           /* ECN master switch: 0=disabled, 1=enabled */
module_param_cb(ucp_ecn_enable, &ucp_param_ops, &ucp_ecn_enable, 0644); /* sysctl: ucp_ecn_enable */
static int ucp_ecn_backoff_num = 20;                     /* ECN backoff percentage numerator */
module_param_cb(ucp_ecn_backoff_num, &ucp_param_ops, &ucp_ecn_backoff_num, 0644); /* sysctl: ucp_ecn_backoff_num */
static int ucp_ecn_backoff_den = 100;                    /* ECN backoff percentage denominator */
module_param_cb(ucp_ecn_backoff_den, &ucp_param_ops, &ucp_ecn_backoff_den, 0644); /* sysctl: ucp_ecn_backoff_den */
static int ucp_ecn_qdelay_thresh_us = 2000;              /* qdelay threshold for ECN backoff (us) */
module_param_cb(ucp_ecn_qdelay_thresh_us, &ucp_param_ops, &ucp_ecn_qdelay_thresh_us, 0644); /* sysctl: ucp_ecn_qdelay_thresh_us */
static int ucp_ecn_ewma_retained = 3;                    /* ECN EWMA retained weight (old) */
module_param_cb(ucp_ecn_ewma_retained, &ucp_param_ops, &ucp_ecn_ewma_retained, 0644); /* sysctl: ucp_ecn_ewma_retained */
static int ucp_ecn_ewma_total = 4;                       /* ECN EWMA total weight (old + new) */
module_param_cb(ucp_ecn_ewma_total, &ucp_param_ops, &ucp_ecn_ewma_total, 0644); /* sysctl: ucp_ecn_ewma_total */

/*
 * ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den — Per-ACK gentle decay
 *     rate applied to ecn_ewma on every ACK where no new CE marks are
 *     detected (ce_delta == 0).  This is much slower than the round_start
 *     decay (which uses ucp_ecn_ewma_retained/total) and prevents ecn_ewma
 *     from persisting indefinitely on steady connections where round
 *     boundaries are infrequent.
 *     Default 31/32 -> ~3.2% decay per ACK,
 *     ~28% per typical RTT of 10 ACKs, halving in ~2 RTTs.
 */
static int ucp_ecn_idle_decay_num = 31;                 /* per-ACK ECN idle decay numerator */
module_param_cb(ucp_ecn_idle_decay_num, &ucp_param_ops, &ucp_ecn_idle_decay_num, 0644); /* sysctl */
static int ucp_ecn_idle_decay_den = 32;                 /* per-ACK ECN idle decay denominator */
module_param_cb(ucp_ecn_idle_decay_den, &ucp_param_ops, &ucp_ecn_idle_decay_den, 0644); /* sysctl */

/* ---- Kalman outlier rejection limit (int) ----------------------------- */
/*
 * ucp_kalman_max_consec_reject — Maximum consecutive outlier rejections
 *     before the Kalman filter force-accepts the next sample.  Prevents
 *     a self-reinforcing lock-in where jitter increases the dynamic
 *     rejection threshold, causing more rejections.  Default 25.
 */
static int ucp_kalman_max_consec_reject = 25;           /* consecutive reject limit before force-accept */
module_param_cb(ucp_kalman_max_consec_reject, &ucp_param_ops, &ucp_kalman_max_consec_reject, 0644); /* sysctl */

/* ---- PROBE_BW cycle rand tunable ------------------------------------- */
/*
 * ucp_probe_bw_cycle_rand — Random offset range for initializing
 * PROBE_BW cycle phase on entry.  cycle_idx starts at
 * (cycle_len - 1 - rand[0, probe_bw_cycle_rand)).
 * Default 8.  Randomization prevents phase synchronization across flows
 * (Cardwell et al. 2016).
 */
static int ucp_probe_bw_cycle_rand = 8;                  /* random offset range for cycle_idx init */
module_param_cb(ucp_probe_bw_cycle_rand, &ucp_param_ops, &ucp_probe_bw_cycle_rand, 0644); /* sysctl: ucp_probe_bw_cycle_rand */
static int ucp_probe_bw_up_limit = 0;                  /* limit PROBE_BW up-phase exit conditions (0=off, 1=on) */
module_param_cb(ucp_probe_bw_up_limit, &ucp_param_ops, &ucp_probe_bw_up_limit, 0644); /* sysctl: ucp_probe_bw_up_limit */

/* ---- ACK aggregation max time window ms (num/den) -------------------- */
/*
 * ucp_extra_acked_max_ms_num/den — Maximum ACK aggregation epoch
 * duration in ms.  extra CWND cap = (bw * max_ms * 1000) / BW_UNIT.
 * Default 150/1 = 150 ms.
 */
static int ucp_extra_acked_max_ms_num = 150;              /* ACK-agg max window numerator (ms) */
module_param_cb(ucp_extra_acked_max_ms_num, &ucp_param_ops, &ucp_extra_acked_max_ms_num, 0644); /* sysctl: ucp_extra_acked_max_ms_num */
static int ucp_extra_acked_max_ms_den = 1;                /* ACK-agg max window denominator */
module_param_cb(ucp_extra_acked_max_ms_den, &ucp_param_ops, &ucp_extra_acked_max_ms_den, 0644); /* sysctl: ucp_extra_acked_max_ms_den */

/* ---- ACK Aggregation Confidence-based Compensation (BBRplus-inspired) - */
/*
 * ucp_agg_enable — Master switch: 0 = disabled, 1 = enabled (default).
 *     When enabled, extra_acked signals feed into Kalman noise adjustment
 *     (always) and cwnd compensation (only at confidence >= threshold).
 *
 * ucp_agg_confidence_thresh — Minimum confidence score (0..1024) to
 *     enable cwnd compensation.  Default 512 (CONFIRMED state).
 *     Set > ucp_agg_confidence_max to disable cwnd compensation
 *     while keeping signal layer active.
 *
 * ucp_agg_max_comp_ratio — Maximum cwnd compensation as percentage of BDP.
 *     Default 75 (75% of BDP).  0 = no cwnd compensation.
 *
 * ucp_agg_max_comp_duration — Maximum consecutive RTTs with compensation
 *     active before watchdog forces confidence downgrade.  Default 8.
 *     Prevents stale extra_acked from persisting beyond the event.
 *
 * ucp_agg_r_hysteresis — R recovery hysteresis percentage.
 *     R increases immediately; recovery decays at (100-pct)% per RTT.
 *     Default 75 (25% decay per RTT, ~4 RTTs to return to baseline).
 *
 * ucp_agg_r_multiplier_min / max — Range for Kalman R noise scaling.
 *     256 = 1x (no scaling), 512 = 2x, 2048 = 8x.  Default 256..2048.
 */
static int ucp_agg_enable = 1;                           /* ACK agg compensation master switch */
module_param_cb(ucp_agg_enable, &ucp_param_ops, &ucp_agg_enable, 0644); /* sysctl: ucp_agg_enable */
static int ucp_agg_confidence_thresh = 512;              /* min confidence to enable cwnd compensation (0..1024) */
module_param_cb(ucp_agg_confidence_thresh, &ucp_param_ops, &ucp_agg_confidence_thresh, 0644); /* sysctl */
static int ucp_agg_max_comp_ratio = 75;                  /* max cwnd comp as % of BDP */
module_param_cb(ucp_agg_max_comp_ratio, &ucp_param_ops, &ucp_agg_max_comp_ratio, 0644); /* sysctl */
static int ucp_agg_max_comp_duration = 8;                /* max consecutive RTTs with compensation */
module_param_cb(ucp_agg_max_comp_duration, &ucp_param_ops, &ucp_agg_max_comp_duration, 0644); /* sysctl */
static int ucp_agg_r_hysteresis = 75;                    /* R recovery hysteresis (% retained per RTT) */
module_param_cb(ucp_agg_r_hysteresis, &ucp_param_ops, &ucp_agg_r_hysteresis, 0644); /* sysctl */
static int ucp_agg_r_multiplier_min = 256;               /* R noise scaling floor (256 = 1x) */
module_param_cb(ucp_agg_r_multiplier_min, &ucp_param_ops, &ucp_agg_r_multiplier_min, 0644); /* sysctl */
static int ucp_agg_r_multiplier_max = 2048;              /* R noise scaling ceiling (2048 = 8x) */
module_param_cb(ucp_agg_r_multiplier_max, &ucp_param_ops, &ucp_agg_r_multiplier_max, 0644); /* sysctl */

/*
 * ucp_agg_factor3_qdelay_us — Queue delay threshold for confidence
 *     Factor 3: RTT is considered "near min_rtt" if within this margin.
 *     Default 2000 us (2ms).
 *
 * ucp_agg_factor4_ratio_num / ucp_agg_factor4_ratio_den — Maximum ratio
 *     of current extra_acked to windowed max for Factor 4 to score.
 *     Default 3/2 = 1.5x.  Values within this ratio are not transient spikes.
 *
 * ucp_agg_safety_qdelay_us — Max allowed RTT above min_rtt before
 *     safety guard 1 triggers and blocks compensation.  Default 4000 us.
 *
 * ucp_agg_safety_bdp_mult — BDP multiplier for cwnd ceiling in safety
 *     guards 3 and 4.  Default 3 (3x BDP).  Compensation is blocked
 *     if cwnd or inflight exceeds this multiple of BDP.
 *
 * ucp_agg_max_window_ms — Time window (ms) for the extra_acked cap
 *     in ucp_measure_ack_aggregation: cap = bw * window_ms.
 *     Default 100 ms.
 *
 * ucp_agg_max_decay_pct — Percentage of agg_extra_acked_max retained
 *     per RTT decay in the watchdog.  75 means 25% decay per RTT.
 *     Default 75.
 *
 * ucp_agg_max_per_ack_decay — Gentle per-ACK decay of agg_extra_acked_max
 *     at round fractions (out of 128).  Prevents a single transient spike
 *     from inflating Factor 4 for an entire long RTT.  128 = no per-ACK
 *     decay (default).  127 = ~0.8% per ACK, reaching ~50% after ~87 ACKs.
 */
static int ucp_agg_factor3_qdelay_us = 2000;         /* Factor 3 qdelay margin (us) */
module_param_cb(ucp_agg_factor3_qdelay_us, &ucp_param_ops, &ucp_agg_factor3_qdelay_us, 0644); /* sysctl */
static int ucp_agg_factor4_ratio_num = 3;            /* Factor 4 ratio numerator */
module_param_cb(ucp_agg_factor4_ratio_num, &ucp_param_ops, &ucp_agg_factor4_ratio_num, 0644); /* sysctl */
static int ucp_agg_factor4_ratio_den = 2;            /* Factor 4 ratio denominator */
module_param_cb(ucp_agg_factor4_ratio_den, &ucp_param_ops, &ucp_agg_factor4_ratio_den, 0644); /* sysctl */
static int ucp_agg_safety_qdelay_us = 4000;          /* Safety Guard 1 max qdelay (us) */
module_param_cb(ucp_agg_safety_qdelay_us, &ucp_param_ops, &ucp_agg_safety_qdelay_us, 0644); /* sysctl */
static int ucp_agg_safety_bdp_mult = 3;              /* Safety Guard 3/4 BDP multiplier */
module_param_cb(ucp_agg_safety_bdp_mult, &ucp_param_ops, &ucp_agg_safety_bdp_mult, 0644); /* sysctl */
static int ucp_agg_max_window_ms = 100;              /* extra_acked cap window (ms) */
module_param_cb(ucp_agg_max_window_ms, &ucp_param_ops, &ucp_agg_max_window_ms, 0644); /* sysctl */
static int ucp_agg_max_decay_pct = 75;               /* watchdog: agg_extra_acked_max retained % per RTT */
module_param_cb(ucp_agg_max_decay_pct, &ucp_param_ops, &ucp_agg_max_decay_pct, 0644); /* sysctl */
static int ucp_agg_max_per_ack_decay = 128;          /* per-ACK gentle decay: 128=no decay, 127=~0.8%/ACK */
module_param_cb(ucp_agg_max_per_ack_decay, &ucp_param_ops, &ucp_agg_max_per_ack_decay, 0644); /* sysctl */
static int ucp_agg_max_per_ack_decay_den = 128;      /* per-ACK decay denominator: decay = value/den */
module_param_cb(ucp_agg_max_per_ack_decay_den, &ucp_param_ops, &ucp_agg_max_per_ack_decay_den, 0644); /* sysctl */
static int ucp_agg_window_rotation_rtts = 5;         /* ACK agg window rotation period (RTTs) */
module_param_cb(ucp_agg_window_rotation_rtts, &ucp_param_ops, &ucp_agg_window_rotation_rtts, 0644); /* sysctl */
static int ucp_extra_acked_win_rtts_max = 31;         /* max RTTs for dual-window ACK aggregation rotation (default 31) */
module_param_cb(ucp_extra_acked_win_rtts_max, &ucp_param_ops, &ucp_extra_acked_win_rtts_max, 0644); /* sysctl: ucp_extra_acked_win_rtts_max */

/*
 * ucp_agg_factor_weight — Per-factor confidence score increment.
 *     default 256; with 4 factors, max total = 4 × 256 = 1024.
 *
 * ucp_agg_confidence_max — Confidence scaling denominator (max range).
 *     default 1024; maps confidence [0, max] to [0, r_max - r_min] range.
 *
 * ucp_agg_thresh_suspected — Confidence >= this → SUSPECTED state (default 256).
 * ucp_agg_thresh_confirmed — Confidence >= this → CONFIRMED state (default 512).
 * ucp_agg_thresh_trusted — Confidence >= this → TRUSTED state (default 768).
 */
static int ucp_agg_factor_weight = 256;            /* per-factor confidence score increment */
module_param_cb(ucp_agg_factor_weight, &ucp_param_ops, &ucp_agg_factor_weight, 0644); /* sysctl */
static int ucp_agg_confidence_max = 1024;          /* confidence scaling denominator (max range) */
module_param_cb(ucp_agg_confidence_max, &ucp_param_ops, &ucp_agg_confidence_max, 0644); /* sysctl */
static int ucp_agg_thresh_suspected = 256;         /* SUSPECTED state threshold (confidence >= this) */
module_param_cb(ucp_agg_thresh_suspected, &ucp_param_ops, &ucp_agg_thresh_suspected, 0644); /* sysctl */
static int ucp_agg_thresh_confirmed = 512;         /* CONFIRMED state threshold (confidence >= this) */
module_param_cb(ucp_agg_thresh_confirmed, &ucp_param_ops, &ucp_agg_thresh_confirmed, 0644); /* sysctl */
static int ucp_agg_thresh_trusted = 768;           /* TRUSTED state threshold (confidence >= this) */
module_param_cb(ucp_agg_thresh_trusted, &ucp_param_ops, &ucp_agg_thresh_trusted, 0644); /* sysctl */

/* ---- PROBE_RTT mode duration ms (num/den) ---------------------------- */
/*
 * ucp_probe_rtt_mode_ms_num/den — Minimum time (ms) spent in PROBE_RTT
 * after inflight drops to min_target.  Default 200/1 = 200 ms (BBRv1,
 * Cardwell et al. 2016).
 */
static int ucp_probe_rtt_mode_ms_num = 200;               /* PROBE_RTT stay duration numerator (ms) */
module_param_cb(ucp_probe_rtt_mode_ms_num, &ucp_param_ops, &ucp_probe_rtt_mode_ms_num, 0644); /* sysctl: ucp_probe_rtt_mode_ms_num */
static int ucp_probe_rtt_mode_ms_den = 1;                 /* PROBE_RTT stay duration denominator */
module_param_cb(ucp_probe_rtt_mode_ms_den, &ucp_param_ops, &ucp_probe_rtt_mode_ms_den, 0644); /* sysctl: ucp_probe_rtt_mode_ms_den */

/* ---- Other misc constants -------------------------------------------- */
/*
 * ucp_bw_rt_cycle_len — Number of round-trip windows for the sliding max
 * bandwidth filter (minmax).  Default 10 (BBRv1, Cardwell et al. 2016).
 * ucp_cwnd_min_target — Absolute minimum cwnd floor in segments.
 * Default 4 (BBRv1, Cardwell et al. 2016).
 */
static int ucp_bw_rt_cycle_len = 10;                      /* sliding max BW filter window (rounds) */
module_param_cb(ucp_bw_rt_cycle_len, &ucp_param_ops, &ucp_bw_rt_cycle_len, 0644); /* sysctl: ucp_bw_rt_cycle_len */
static int ucp_cwnd_min_target = 4;                       /* absolute minimum cwnd (segments) */
module_param_cb(ucp_cwnd_min_target, &ucp_param_ops, &ucp_cwnd_min_target, 0644); /* sysctl: ucp_cwnd_min_target */

/*
 * ucp_sndbuf_expand_factor — Send buffer = N × cwnd (BBR standard: 3x).
 *     default 3, range 2-100.
 */
static int ucp_sndbuf_expand_factor = 3;              /* sndbuf expansion factor (×cwnd) */
module_param_cb(ucp_sndbuf_expand_factor, &ucp_param_ops, &ucp_sndbuf_expand_factor, 0644); /* sysctl */

/*
 * ucp_ack_epoch_max — Epoch byte accumulator cap (~1M default).
 *     Prevents u32 overflow in extra_acked = ack_epoch_acked - expected_acked.
 *     When approaching this cap, the epoch resets.  default 0xFFFFF ≈ 1M.
 */
static int ucp_ack_epoch_max = 0xFFFFF;               /* epoch accumulator cap (bytes) */
module_param_cb(ucp_ack_epoch_max, &ucp_param_ops, &ucp_ack_epoch_max, 0644); /* sysctl */

/* ---- Internal Derived Variables -------------------------------------- */
/*
 * These are computed by ucp_init_module_params() from the raw module
 * parameters.  No concurrent-write protection needed — see the
 * "CONCURRENCY & SAFETY MODEL" comment at struct ucp_ext for details.
 */
static u32 ucp_probe_rtt_base_jiffies;                    /* ucp_probe_rtt_base_sec * HZ (computed at module init) */
static u32 ucp_probe_rtt_max_jiffies;                     /* ucp_probe_rtt_max_sec * HZ (computed at module init) */
static u32 ucp_probe_rtt_dyn_max_jiffies;                 /* ucp_probe_rtt_dyn_max_sec * HZ (computed at module init) */

static u32 ucp_cwnd_gain_val;                             /* num/den * BBR_UNIT (clamped 0..1023) (computed at module init) */
static u32 ucp_cycle_gain_table[UCP_GAIN_SLOTS];           /* pre-computed PROBE_BW gains (read-on-connect) */

/*
 * ucp_rebuild_gain_table - Recompute the 256-slot cycle gain table.
 *
 * For each slot i:
 *   effective_gain = min( BBR_UNIT * num[i] / den[i] , 1023 )
 *
 * den is floored at 1, num at 0 to prevent invalid values.
 * Called at module init and whenever ucp_gain_num[] or ucp_gain_den[]
 * is written via sysctl (via ucp_gain_proc_handler).
 *
 * Note: raw array params are not clamped at sysctl write time; the
 * effective gain is capped at UCP_GAIN_MAX (1023) at computation time.
 */
static void ucp_rebuild_gain_table(void)                    /* recompute ucp_cycle_gain_table[] from num/den arrays */
{
    int i;                                                  /* loop index over UCP_GAIN_SLOTS */
    for (i = 0; i < UCP_GAIN_SLOTS; i++) {                  /* iterate through all 256 gain slots */
        int num = ucp_gain_num[i];
        int den = ucp_gain_den[i];
        if (den < 1) {
            den = 1;
        }                           /* floor: prevent div-by-zero */
        if (num < 0) {
            num = 0;
        }                           /* floor: gain cannot be negative */
        ucp_cycle_gain_table[i] = (u32)min_t(u64, ((u64)BBR_UNIT * (u32)num) / (u32)den, UCP_GAIN_MAX);
        /* Use corrected locals; do NOT write back to raw params */
    }
}
/*
 * ucp_cycle_decay_enabled - Query whether the decay bit is set for a given
 * cycle phase index.
 *
 * The decay mask is a 256-bit bitmap stored as 8 x 32-bit words.
 *   idx >> 5  -> word index (0..7)
 *   idx & 31  -> bit index (0..31)
 */
static inline bool ucp_cycle_decay_enabled(u32 idx)         /* check if decay bit is set for phase idx */
{
    return ((unsigned int)ucp_cycle_decay_mask[(idx >> UCP_DECAY_WORD_BITS) & (UCP_DECAY_MASK_WORDS - 1)] >> (idx & UCP_DECAY_BIT_MASK)) & UCP_DECAY_MASK_LSB;
}
/*
 * ucp_gain_proc_handler - Custom sysctl handler for the three array-type
 * parameters: ucp_gain_num[], ucp_gain_den[], ucp_cycle_decay_mask[].
 *
 * Delegates to proc_dointvec() for array read/write, then calls
 * ucp_rebuild_gain_table() after any successful write to refresh the
 * ucp_cycle_gain_table[] cache.
 */
static int ucp_gain_proc_handler(struct ctl_table* ctl, int write, /* custom sysctl handler for gain arrays */
    void* buffer, size_t* lenp, loff_t* ppos)         /* standard sysctl callback signature */
{
    int ret = proc_dointvec(ctl, write, buffer, lenp, ppos);       /* delegate array read/write to kernel */
    if (write && ret == 0) {                                       /* write succeeded */
        ucp_gain_table_defaulted = false;                           /* user explicitly configured via sysctl */
        ucp_rebuild_gain_table();                                  /* refresh pre-computed gain table */
    }

    return ret;                                                    /* return result from proc_dointvec */
}
/* Derived scalars — all populated by ucp_init_module_params() */
static u32 ucp_extra_acked_gain_val;                       /* ACK-agg gain in BBR_UNIT (computed at module init) */
static u32 ucp_high_gain_val;                              /* STARTUP pacing gain in BBR_UNIT (computed at module init) */
static u32 ucp_drain_gain_val;                             /* DRAIN pacing gain in BBR_UNIT (computed at module init) */
static u32 ucp_probe_bw_cycle_len_val;                     /* clamped & power-of-two cycle length (computed at module init) */
static u32 ucp_full_bw_thresh_val;                         /* full-BW detection threshold in BBR_UNIT (computed at module init) */
static u32 ucp_full_bw_cnt_val;                            /* clamped full-BW round count (computed at module init) */

static u32 ucp_inflight_low_gain_val;                      /* inflight lower bound in BBR_UNIT (computed at module init) */
static u32 ucp_inflight_high_gain_val;                     /* inflight upper bound in BBR_UNIT (computed at module init) */
static u32 ucp_kalman_p_est_max_val;                       /* clamped p_est max (computed at module init) */
static u32 ucp_kalman_converged_p_est_val;                  /* clamped convergence p_est (computed at module init) */
static u32 ucp_drain_skip_qdelay_us_val;                   /* clamped drain skip qdelay (computed at module init) */
static u32 ucp_kalman_q_boost_thresh_val;                    /* computed Q-boost threshold (computed at module init) */
static u32 ucp_kalman_qboost_cdwn_val;                       /* clamped Q-boost cooldown (computed at module init) */
static int ucp_kalman_q_max_val;                            /* clamped Q max (computed at module init) */
static int ucp_kalman_q_scale_cap_val;                      /* clamped Q scale cap (computed at module init) */
static int ucp_kalman_min_samples_val;                      /* clamped min samples for takeover (computed at module init) */
static u32 ucp_rtt_sample_max_us_val;                       /* clamped max RTT sample (computed at module init) */
static int ucp_minrtt_fast_fall_cnt_val;                    /* clamped fast-fall count (computed at module init) */
static u32 ucp_minrtt_sticky_num_val;                       /* cached sticky num (computed at module init) */
static u32 ucp_minrtt_sticky_den_val;                       /* cached sticky den (computed at module init) */
static u32 ucp_minrtt_srtt_guard_num_val;                   /* cached SRTT guard num (computed at module init) */
static u32 ucp_minrtt_srtt_guard_den_val;                   /* cached SRTT guard den (computed at module init) */
static u32 ucp_bdp_min_rtt_us_val;                          /* clamped BDP min RTT (computed at module init) */
static u32 ucp_probe_cwnd_bonus_val;                        /* clamped probe cwnd bonus (computed at module init) */
static int ucp_edt_near_now_ns_val;                         /* clamped EDT near-now threshold (computed at module init) */
static int ucp_min_tso_rate_val;                            /* clamped min TSO rate (computed at module init) */
static int ucp_tso_max_segs_val;                            /* clamped max TSO segs (computed at module init) */
static int ucp_agg_enable_val;                           /* clamped agg enable flag (computed at module init) */
static int ucp_agg_confidence_thresh_val;                /* clamped confidence threshold (computed at module init) */
static int ucp_agg_max_comp_ratio_val;                   /* clamped max comp ratio (computed at module init) */
static int ucp_agg_max_comp_duration_val;                /* clamped max comp duration (computed at module init) */
static int ucp_agg_r_hysteresis_val;                     /* clamped R hysteresis (computed at module init) */
static u32 ucp_agg_r_multiplier_min_val;                 /* clamped R mult min (computed at module init) */
static u32 ucp_agg_r_multiplier_max_val;                 /* clamped R mult max (computed at module init) */
static int ucp_agg_factor3_qdelay_us_val;            /* clamped Factor 3 qdelay (computed at module init) */
static int ucp_agg_factor4_ratio_num_val;            /* snapped Factor 4 ratio num (computed at module init) */
static int ucp_agg_factor4_ratio_den_val;            /* snapped Factor 4 ratio den (computed at module init) */
static int ucp_agg_safety_qdelay_us_val;             /* clamped safety qdelay (computed at module init) */
static int ucp_agg_safety_bdp_mult_val;              /* clamped safety BDP mult (computed at module init) */
static int ucp_agg_max_window_ms_val;                /* clamped max window ms (computed at module init) */
static int ucp_agg_max_decay_pct_val;                /* clamped max decay pct (computed at module init) */
static int ucp_agg_max_per_ack_decay_val;            /* clamped per-ACK decay (computed at module init) */
static int ucp_agg_max_per_ack_decay_den_val;        /* clamped per-ACK decay denom (computed at module init) */
static int ucp_agg_window_rotation_rtts_val;         /* clamped window rotation RTTs (computed at module init) */
static int ucp_lt_bw_ema_num_val;                       /* cached LT BW EMA numerator (computed at module init) */
static int ucp_lt_bw_ema_den_val;                       /* cached LT BW EMA denominator (computed at module init) */
static int ucp_lt_bw_probe_pct_val;                     /* cached LT BW probe percentage (computed at module init) */
static int ucp_kalman_noise_avg_num_val;                 /* cached noise avg numerator (computed at module init) */
static int ucp_kalman_noise_avg_den_val;                 /* cached noise avg denominator (computed at module init) */
static int ucp_tso_segs_low_val;                         /* cached TSO segs low (computed at module init) */
static int ucp_tso_segs_default_val;                     /* cached TSO segs default (computed at module init) */
static int ucp_extra_acked_win_rtts_max_val;              /* cached extra acked win rtts max (computed at module init) */

static u32 ucp_kalman_noise_alpha_complement;           /* precomputed alpha_d - alpha_n (computed at module init) */
static u32 ucp_kalman_noise_beta_complement;             /* precomputed beta_d - beta_n (computed at module init) */
static bool ucp_agg_per_ack_decay_active;                 /* precomputed: per_ack_decay < den (computed at module init) */

static int ucp_jitter_probe_thresh_us_val;                  /* clamped jitter threshold for gain decay (computed at module init) */
static int ucp_jitter_probe_scale_us_val;                   /* clamped jitter scale for gain decay (computed at module init) */
static int ucp_qdelay_probe_thresh_us_val;                  /* clamped qdelay threshold for gain decay (computed at module init) */
static int ucp_qdelay_probe_scale_us_val;                   /* clamped qdelay scale for gain decay (computed at module init) */
static int ucp_jitter_r_thresh_us_val;                      /* clamped jitter threshold for adaptive R (computed at module init) */
static int ucp_jitter_r_scale_val;                          /* clamped jitter scale for adaptive R (computed at module init) */
static int ucp_kalman_r_max_boost_val;                  /* clamped R max boost (computed at module init) */
static int ucp_probe_rtt_long_rtt_us_val;                   /* clamped long-RTT threshold (computed at module init) */
static u32 ucp_pacing_margin_div_val;                       /* computed pacing divisor [1, 100] (computed at module init) */

static u32 ucp_kalman_p_est_init_val;                       /* clamped initial p_est (computed at module init) */
static u32 ucp_kalman_p_est_floor_val;                      /* clamped p_est floor (computed at module init) */
static u32 ucp_kalman_outlier_ms_val;                       /* clamped outlier base (ms) (computed at module init) */
static u32 ucp_kalman_scale_shift_val;                    /* ilog2(kalman_scale) for shift optimization (computed at module init) */
static u64 ucp_kalman_outlier_thresh_scaled_val;     /* precomputed scaled outlier base threshold (computed at module init) */
static u64 ucp_kalman_shift_cap_val;                 /* precomputed U64_MAX >> scale_shift (computed at module init) */
static int ucp_kalman_noise_alpha_num_val;               /* snapped noise alpha numerator (computed at module init) */
static int ucp_kalman_noise_alpha_den_val;               /* snapped noise alpha denominator (computed at module init) */
static int ucp_kalman_noise_beta_num_val;                /* snapped noise beta numerator (computed at module init) */
static int ucp_kalman_noise_beta_den_val;                /* snapped noise beta denominator (computed at module init) */
static int ucp_kalman_q_est_max_val;                     /* clamped Q estimate max (computed at module init) */
static int ucp_kalman_r_est_max_val;                     /* clamped R estimate max (computed at module init) */
static int ucp_kalman_q_est_floor_val;                   /* clamped Q estimate floor (computed at module init) */
static int ucp_kalman_r_est_floor_val;                   /* clamped R estimate floor (computed at module init) */
/* Cached clamped Q/R for hot-path use (avoid raw-param read race during sysctl) */
static int ucp_kalman_q_val;                             /* clamped Kalman Q (computed at module init) */
static int ucp_kalman_r_val;                             /* clamped Kalman R (computed at module init) */
static int ucp_kalman_noise_mode_val;                    /* clamped noise combination mode (computed at module init) */
static int ucp_alone_confirm_rounds_val;               /* clamped alone confirmation rounds (computed at module init) */
static int ucp_alone_qdelay_thresh_us_val;              /* clamped alone qdelay threshold (computed at module init) */
static int ucp_alone_jitter_thresh_us_val;              /* clamped alone jitter threshold (computed at module init) */
static int ucp_alone_agg_state_level_val;               /* clamped alone agg state level (computed at module init) */
static int ucp_alone_bypass_ecn_val;                    /* clamped alone bypass ECN flag (computed at module init) */
static int ucp_alone_bypass_lt_bw_val;                  /* clamped alone bypass LT BW flag (computed at module init) */

static int ucp_ecn_enable_val;                           /* clamped ECN enable flag (computed at module init) */
static u32 ucp_ecn_backoff_val;                          /* derived ECN backoff ratio in BBR_UNIT (computed at module init) */
static int ucp_ecn_qdelay_thresh_us_val;                 /* clamped ECN qdelay threshold (computed at module init) */
static int ucp_ecn_ewma_retained_val;                    /* cached ECN EWMA retained weight (computed at module init) */
static int ucp_ecn_ewma_total_val;                       /* cached ECN EWMA total weight (computed at module init) */
static int ucp_kalman_max_consec_reject_val;             /* clamped consec reject limit (computed at module init) */
static int ucp_ecn_idle_decay_num_val;                   /* snapped per-ACK ECN idle decay num (computed at module init) */
static int ucp_ecn_idle_decay_den_val;                   /* snapped per-ACK ECN idle decay den (computed at module init) */
static int ucp_ewma_qdelay_num_val;                         /* cached EWMA qdelay numerator (computed at module init) */
static int ucp_ewma_qdelay_den_val;                         /* cached EWMA qdelay denominator (computed at module init) */
static int ucp_ewma_jitter_num_val;                         /* cached EWMA jitter numerator (computed at module init) */
static int ucp_ewma_jitter_den_val;                         /* cached EWMA jitter denominator (computed at module init) */
static int ucp_probe_bw_cycle_rand_val;                     /* clamped PROBE_BW rand range (computed at module init) */
static int ucp_probe_bw_up_limit_val;                      /* clamped PROBE_BW up-phase limit flag (computed at module init) */
static int ucp_lt_intvl_min_rtts_val;                       /* clamped LT interval min RTTs (computed at module init) */
static int ucp_lt_intvl_max_mult_val;                      /* clamped LT interval max multiplier (computed at module init) */
static u32 ucp_lt_loss_thresh_val;                          /* clamped LT loss threshold (computed at module init) */
static u32 ucp_lt_bw_ratio_val;                             /* derived LT BW ratio in BBR_UNIT (computed at module init) */
static u32 ucp_lt_bw_diff_val;                              /* clamped LT BW absolute diff (computed at module init) */
static int ucp_lt_bw_max_rtts_val;                          /* clamped LT BW max RTTs (< 4095) (computed at module init) */
static int ucp_lt_restore_ratio_num_val;                  /* snapped restore ratio num (computed at module init) */
static int ucp_lt_restore_ratio_den_val;                  /* snapped restore ratio den (computed at module init) */
static int ucp_lt_restore_consec_acks_val;                /* clamped restore consecutive ACKs [1,31] (computed at module init) */

static u32 ucp_extra_acked_max_ms_val;                       /* derived ACK max aggregation ms (computed at module init) */
static u32 ucp_probe_rtt_mode_ms_val;                        /* derived PROBE_RTT stay-duration ms (computed at module init) */
static u32 ucp_minrtt_fast_fall_div_val;                    /* clamped minrtt fast-fall divisor (computed at module init) */
static u32 ucp_kalman_probe_band_mult_val;                  /* clamped probe interval band multiplier (computed at module init) */
static u32 ucp_kalman_q_rtt_div_val;                        /* clamped Q RTT division factor (computed at module init) */
static u32 ucp_kalman_rtt_dyn_mult_val;                     /* clamped rtt dynamic multiplier (computed at module init) */
static u32 ucp_tso_headroom_mult_val;                       /* derived TSO headroom multiplier (computed at module init) */
static u32 ucp_min_tso_rate_div_val;                        /* derived min TSO rate divisor (computed at module init) */
static int ucp_probe_rtt_long_interval_div_val;             /* snapped long interval divisor (computed at module init) */
static int ucp_agg_factor_weight_val;                       /* snapped factor weight [0,100] (computed at module init) */
static int ucp_agg_confidence_max_val;                      /* clamped confidence max (computed at module init) */
static int ucp_agg_thresh_suspected_val;                    /* snapped suspected threshold (computed at module init) */
static int ucp_agg_thresh_confirmed_val;                    /* snapped confirmed threshold (computed at module init) */
static int ucp_agg_thresh_trusted_val;                      /* snapped trusted threshold (computed at module init) */
static u32 ucp_kalman_outlier_jitter_mult_val;              /* clamped outlier jitter multiplier (computed at module init) */
static u32 ucp_kalman_q_min_factor_val;                     /* clamped Q-min factor (computed at module init) */
static u32 ucp_kalman_p_est_init_rtt_div_val;               /* clamped p_est init RTT divisor (computed at module init) */
static u32 ucp_bw_rt_cycle_len_val;                         /* clamped BW cycle length (computed at module init) */
static u32 ucp_cwnd_min_target_val;                          /* clamped cwnd min target (computed at module init) */
static u32 ucp_sndbuf_expand_factor_val;                     /* clamped sndbuf expand factor (computed at module init) */
static u32 ucp_ack_epoch_max_val;                            /* clamped ACK epoch max (computed at module init) */

/*
 * ucp_init_module_params - Validate all raw module parameters against
 * their legal ranges, clamp out-of-bounds values, and compute all
 * derived fixed-point quantities.
 *
 * Called at module load and whenever any scalar parameter is written
 * via sysctl.  Also called by ucp_rebuild_gain_table's caller path.
 *
 * No concurrent-write protection needed — see the
 * "CONCURRENCY & SAFETY MODEL" comment at struct ucp_ext for details.
 */
static void ucp_init_module_params(void)                          /* clamp all params + compute derived values */
{
    int i;                                                                  /* loop index for decay mask init */
    /*
     * Clamp all raw module-parameter integers to their legal ranges.
     * For denominator-style params, lo=1 prevents division by zero.
     * For numerator-style params, lo=0 allows disabling the feature.
     */
    ucp_probe_rtt_base_sec = clamp(ucp_probe_rtt_base_sec, 1, UCP_PROBE_RTT_MAX_SEC);   /* PROBE_RTT base interval [1s, 24h] */
    ucp_probe_rtt_max_sec = clamp(ucp_probe_rtt_max_sec, 1, UCP_PROBE_RTT_MAX_SEC);      /* PROBE_RTT max interval [1s, 24h] */
    ucp_probe_rtt_dyn_max_sec = clamp(ucp_probe_rtt_dyn_max_sec, 0, UCP_PROBE_RTT_MAX_SEC); /* PROBE_RTT dyn max [0, 24h] */

    ucp_cwnd_gain_num = clamp(ucp_cwnd_gain_num, 0, 100000);            /* cwnd gain numerator [0, 100k] */
    ucp_cwnd_gain_den = clamp(ucp_cwnd_gain_den, 1, 100000);            /* cwnd gain denominator [1, 100k] */
    ucp_extra_acked_gain_num = clamp(ucp_extra_acked_gain_num, 0, 100000); /* ACK-agg gain num [0, 100k] */
    ucp_extra_acked_gain_den = clamp(ucp_extra_acked_gain_den, 1, 100000); /* ACK-agg gain den [1, 100k] */

    ucp_kalman_q = clamp(ucp_kalman_q, 0, 100000);        /* process noise Q [0, 100k] (Kalman 1960) */
    ucp_kalman_r = clamp(ucp_kalman_r, 0, 100000);        /* measurement noise R [0, 100k] (Kalman 1960) */

    ucp_high_gain_num = clamp(ucp_high_gain_num, 0, 100000);            /* STARTUP gain numerator [0, 100k] */
    ucp_high_gain_den = clamp(ucp_high_gain_den, 1, 100000);            /* STARTUP gain denominator [1, 100k] */
    ucp_drain_gain_num = clamp(ucp_drain_gain_num, 0, 100000);          /* DRAIN gain numerator [0, 100k] */
    ucp_drain_gain_den = clamp(ucp_drain_gain_den, 1, 100000);          /* DRAIN gain denominator [1, 100k] */

    /* PROBE_BW cycle length: [2, 256], must be power-of-two
     * so that cycle_idx & (len-1) wraps correctly. */
    ucp_probe_bw_cycle_len = clamp(ucp_probe_bw_cycle_len, 2, 256);     /* cycle length [2, 256] */
    ucp_probe_bw_cycle_len = roundup_pow_of_two(ucp_probe_bw_cycle_len); /* round up to power of two */

    ucp_full_bw_thresh_num = clamp(ucp_full_bw_thresh_num, 0, 100000); /* full-BW threshold num [0, 100k] */
    ucp_full_bw_thresh_den = clamp(ucp_full_bw_thresh_den, 1, 100000); /* full-BW threshold den [1, 100k] */
    ucp_full_bw_cnt = clamp(ucp_full_bw_cnt, 1, 3);                    /* full-BW rounds [1, 3], 2-bit field */

    ucp_pacing_margin_num = clamp(ucp_pacing_margin_num, 0, 50);        /* pacing margin num [0, 50], prevent >=100% */
    ucp_pacing_margin_den = clamp(ucp_pacing_margin_den, 1, 100000);    /* pacing margin den [1, 100k] */

    ucp_inflight_low_gain_num = clamp(ucp_inflight_low_gain_num, 0, 100000);   /* inflight lo num [0, 100k] */
    ucp_inflight_low_gain_den = clamp(ucp_inflight_low_gain_den, 1, 100000);   /* inflight lo den [1, 100k] */
    ucp_inflight_high_gain_num = clamp(ucp_inflight_high_gain_num, 0, 100000); /* inflight hi num [0, 100k] */
    ucp_inflight_high_gain_den = clamp(ucp_inflight_high_gain_den, 1, 100000); /* inflight hi den [1, 100k] */

    ucp_kalman_p_est_max = clamp(ucp_kalman_p_est_max, 1, 100000000);  /* p_est max [1, 100M] */
    ucp_kalman_converged_p_est = clamp(ucp_kalman_converged_p_est, 1, 1000000); /* convergence p_est [1, 1M] */
    ucp_drain_skip_qdelay_us = clamp(ucp_drain_skip_qdelay_us, 0, 100000);  /* drain skip qdelay [0, 100k us] */
    ucp_kalman_q_boost_mult = clamp(ucp_kalman_q_boost_mult, 1, 10000); /* Q-boost mult [1, 10k] */
    ucp_kalman_q_max = clamp(ucp_kalman_q_max, 1, 100000);              /* Q max [1, 100k] */
    ucp_kalman_q_scale_cap = clamp(ucp_kalman_q_scale_cap, 1, 10000);   /* Q scale cap [1, 10k] */
    ucp_kalman_min_samples = clamp(ucp_kalman_min_samples, 3, 20);      /* min Kalman samples [3, 20] */

    ucp_rtt_sample_max_us = clamp(ucp_rtt_sample_max_us, 1, 10000000);  /* max RTT sample [1, 10M us] */
    ucp_minrtt_fast_fall_cnt = clamp(ucp_minrtt_fast_fall_cnt, 0, 3);   /* fast-fall count [0, 3], 2-bit field */
    ucp_minrtt_sticky_num = clamp(ucp_minrtt_sticky_num, 0, 1000);      /* sticky ratio num [0, 1000] */
    ucp_minrtt_sticky_den = clamp(ucp_minrtt_sticky_den, 1, 100000);    /* sticky ratio den [1, 100k] */
    ucp_minrtt_sticky_num = min_t(int, ucp_minrtt_sticky_num, ucp_minrtt_sticky_den);
    ucp_minrtt_srtt_guard_num = clamp(ucp_minrtt_srtt_guard_num, 0, 1000); /* SRTT guard num [0, 1000] */
    ucp_minrtt_srtt_guard_den = clamp(ucp_minrtt_srtt_guard_den, 1, 100000); /* SRTT guard den [1, 100k] */
    ucp_minrtt_srtt_guard_num = min_t(int, ucp_minrtt_srtt_guard_num, ucp_minrtt_srtt_guard_den);

    ucp_bdp_min_rtt_us = clamp(ucp_bdp_min_rtt_us, 0, 100000);          /* BDP min-RTT floor [0, 100k us] */
    ucp_probe_cwnd_bonus = clamp(ucp_probe_cwnd_bonus, 0, 100);          /* probe cwnd bonus [0, 100] */
    ucp_edt_near_now_ns = clamp(ucp_edt_near_now_ns, 0, 10000000);       /* EDT near-now [0, 10M ns] */
    ucp_min_tso_rate = clamp(ucp_min_tso_rate, 1, 1000000000);           /* min TSO rate [1, 1B bytes/s] */
    ucp_tso_max_segs = clamp(ucp_tso_max_segs, 1, 65535);                /* max TSO segs [1, 65535] */
    ucp_tso_segs_low = clamp(ucp_tso_segs_low, 1, 65535);
    ucp_tso_segs_default = clamp(ucp_tso_segs_default, 1, 65535);
    ucp_jitter_probe_thresh_us = clamp(ucp_jitter_probe_thresh_us, 0, 100000);   /* jitter probe thresh [0, 100k] */
    ucp_jitter_probe_scale_us = clamp(ucp_jitter_probe_scale_us, 1, 100000);     /* jitter probe scale [1, 100k] */
    ucp_qdelay_probe_thresh_us = clamp(ucp_qdelay_probe_thresh_us, 0, 100000);   /* qdelay probe thresh [0, 100k] */
    ucp_qdelay_probe_scale_us = clamp(ucp_qdelay_probe_scale_us, 1, 100000);     /* qdelay probe scale [1, 100k] */
    ucp_jitter_r_thresh_us = clamp(ucp_jitter_r_thresh_us, 0, 100000);   /* adaptive R jitter thresh [0, 100k] */
    ucp_jitter_r_scale = clamp(ucp_jitter_r_scale, 1, 100000);           /* adaptive R scale [1, 100k] */
    ucp_kalman_r_max_boost = clamp(ucp_kalman_r_max_boost, 1, 1000);    /* R max boost [1, 1000] */
    ucp_probe_rtt_long_rtt_us = clamp(ucp_probe_rtt_long_rtt_us, 0, 10000000); /* long-RTT thresh [0, 10M] */

    /* Kalman scale must be power-of-two for shift-based division (Kalman 1960) */
    ucp_kalman_p_est_init = clamp(ucp_kalman_p_est_init, 1, 10000000);   /* p_est init [1, 10M] */
    ucp_kalman_p_est_floor = clamp(ucp_kalman_p_est_floor, 1, 100000);   /* p_est floor [1, 100k] */
    ucp_kalman_p_est_floor = min(ucp_kalman_p_est_floor, ucp_kalman_p_est_max); /* floor <= max invariant */
    ucp_kalman_p_est_init = min(ucp_kalman_p_est_init, ucp_kalman_p_est_max);   /* init <= max invariant */
    ucp_kalman_outlier_ms = clamp(ucp_kalman_outlier_ms, 0, 10000);      /* outlier ms [0, 10k] */
    ucp_kalman_q_boost_ms = clamp(ucp_kalman_q_boost_ms, 1, 5000);       /* Q-boost ms [1, 5000] */
    ucp_kalman_qboost_cdwn = clamp(ucp_kalman_qboost_cdwn, 1, 255);      /* Q-boost cooldown [1, 255] */
    ucp_kalman_xest_margin_pct = clamp(ucp_kalman_xest_margin_pct, 0, 100); /* x_est margin pct [0, 100] */
    ucp_kalman_scale = clamp(ucp_kalman_scale, 64, 1048576);              /* kalman scale [64, 1M] */
    ucp_kalman_scale = roundup_pow_of_two(ucp_kalman_scale);              /* round up to power of two */

    ucp_kalman_outlier_jitter_mult_num = clamp(ucp_kalman_outlier_jitter_mult_num, 0, 1000);     /* jitter mult num [0, 1000] */
    ucp_kalman_outlier_jitter_mult_den = clamp(ucp_kalman_outlier_jitter_mult_den, 1, 100000);   /* jitter mult den [1, 100k] */
    ucp_kalman_q_min_factor_num = clamp(ucp_kalman_q_min_factor_num, 0, 1000);                    /* Q min factor num [0, 1000] */
    ucp_kalman_q_min_factor_den = clamp(ucp_kalman_q_min_factor_den, 1, 100000);                  /* Q min factor den [1, 100k] */
    ucp_kalman_p_est_init_rtt_div_num = clamp(ucp_kalman_p_est_init_rtt_div_num, 1, 100000);      /* p_est init RTT div num [1, 100k] */
    ucp_kalman_p_est_init_rtt_div_den = clamp(ucp_kalman_p_est_init_rtt_div_den, 1, 100000);      /* p_est init RTT div den [1, 100k] */

    ucp_ewma_qdelay_num = clamp(ucp_ewma_qdelay_num, 0, 100);              /* EWMA qdelay num [0, 100] */
    ucp_ewma_qdelay_den = clamp(ucp_ewma_qdelay_den, 1, 100000);           /* EWMA qdelay den [1, 100k] */
    ucp_ewma_qdelay_num = min_t(int, ucp_ewma_qdelay_num, ucp_ewma_qdelay_den);
    ucp_ewma_jitter_num = clamp(ucp_ewma_jitter_num, 0, 100);              /* EWMA jitter num [0, 100] */
    ucp_ewma_jitter_den = clamp(ucp_ewma_jitter_den, 1, 100000);           /* EWMA jitter den [1, 100k] */
    ucp_ewma_jitter_num = min_t(int, ucp_ewma_jitter_num, ucp_ewma_jitter_den);

    /* BBR-S covariance-matched noise estimation params */
    ucp_kalman_noise_alpha_num = clamp(ucp_kalman_noise_alpha_num, 0, 100);    /* alpha num [0, 100] */
    ucp_kalman_noise_alpha_den = clamp(ucp_kalman_noise_alpha_den, 1, 100000); /* alpha den [1, 100k] */
    ucp_kalman_noise_alpha_num = min_t(int, ucp_kalman_noise_alpha_num, ucp_kalman_noise_alpha_den); /* alpha_num <= alpha_den */
    ucp_kalman_noise_beta_num = clamp(ucp_kalman_noise_beta_num, 0, 100);     /* beta num [0, 100] */
    ucp_kalman_noise_beta_den = clamp(ucp_kalman_noise_beta_den, 1, 100000);  /* beta den [1, 100k] */
    ucp_kalman_noise_beta_num = min_t(int, ucp_kalman_noise_beta_num, ucp_kalman_noise_beta_den);   /* beta_num <= beta_den */
    ucp_kalman_q_est_max = clamp(ucp_kalman_q_est_max, 1, 2000000000);          /* Q est max [1, 2e9] */
    ucp_kalman_r_est_max = clamp(ucp_kalman_r_est_max, 1, 2000000000);          /* R est max [1, 2e9] */
    ucp_kalman_q_est_floor = clamp(ucp_kalman_q_est_floor, 1, 100000);        /* Q est floor [1, 100k] */
    ucp_kalman_r_est_floor = clamp(ucp_kalman_r_est_floor, 1, 100000);        /* R est floor [1, 100k] */
    ucp_kalman_noise_mode = clamp(ucp_kalman_noise_mode, 0, 2);               /* mode [0=off, 1=max, 2=avg] */
    ucp_kalman_noise_avg_num = clamp(ucp_kalman_noise_avg_num, 0, 100);
    ucp_kalman_noise_avg_den = clamp(ucp_kalman_noise_avg_den, 1, 100000);
    ucp_kalman_noise_avg_num = min_t(int, ucp_kalman_noise_avg_num, ucp_kalman_noise_avg_den);

    /* Single-flow detection hysteresis */
    ucp_alone_confirm_rounds = clamp(ucp_alone_confirm_rounds, 1, 32);       /* alone confirm [1, 32] */
    ucp_alone_qdelay_thresh_us = clamp(ucp_alone_qdelay_thresh_us, 0, 100000); /* alone qdelay [0, 100k] us */
    ucp_alone_jitter_thresh_us = clamp(ucp_alone_jitter_thresh_us, 0, 100000); /* alone jitter [0, 100k] us */
    ucp_alone_agg_state_level = clamp(ucp_alone_agg_state_level, 0, 2);         /* alone agg level [0, 2] */
    ucp_alone_bypass_ecn = clamp(ucp_alone_bypass_ecn, 0, 1);                   /* alone bypass ECN [0, 1] */
    ucp_alone_bypass_lt_bw = clamp(ucp_alone_bypass_lt_bw, 0, 1);               /* alone bypass LT BW [0, 1] */

    /* ECN params */
    ucp_ecn_enable = clamp(ucp_ecn_enable, 0, 1);                                /* ECN enable [0, 1] */
    ucp_ecn_backoff_num = clamp(ucp_ecn_backoff_num, 0, 100);                    /* ECN backoff num [0, 100] */
    ucp_ecn_backoff_den = clamp(ucp_ecn_backoff_den, 1, 100000);                 /* ECN backoff den [1, 100k] */
    ucp_ecn_qdelay_thresh_us = clamp(ucp_ecn_qdelay_thresh_us, 0, 100000);       /* ECN qdelay thresh [0, 100k] */
    ucp_ecn_ewma_retained = clamp(ucp_ecn_ewma_retained, 0, 100);                /* ECN EWMA retained [0, 100] */
    ucp_ecn_ewma_total = clamp(ucp_ecn_ewma_total, 1, 100000);                   /* ECN EWMA total [1, 100k] */
    /* EWMA formula requires retained <= total, otherwise new-sample weight > 1 */
    ucp_ecn_ewma_retained = min_t(int, ucp_ecn_ewma_retained, ucp_ecn_ewma_total); /* enforce retained <= total */
    /* ECN idle decay: must stay in [1, den] region, and num < den to guarantee actual decay */
    ucp_ecn_idle_decay_den = clamp(ucp_ecn_idle_decay_den, 2, 100000);              /* den [2, 100k], prevents div-by-zero */
    ucp_ecn_idle_decay_num = clamp(ucp_ecn_idle_decay_num, 1, ucp_ecn_idle_decay_den - 1); /* num [1, den-1], decay factor < 1.0 */

    /* Kalman outlier rejection limit */
    ucp_kalman_max_consec_reject = clamp(ucp_kalman_max_consec_reject, 1, 1000);   /* consec reject [1, 1000] */

    ucp_probe_bw_cycle_rand = clamp(ucp_probe_bw_cycle_rand, 1, ucp_probe_bw_cycle_len);   /* rand range [1, cycle_len] */
    ucp_probe_bw_up_limit = clamp(ucp_probe_bw_up_limit, 0, 1);                            /* PROBE_BW up-phase limit [0, 1] */

    /* LT BW: max RTTs must fit in 12-bit counter (< 4095) */
    ucp_lt_intvl_min_rtts = clamp(ucp_lt_intvl_min_rtts, 1, 127);          /* LT min RTTs [1, 127] */
    ucp_lt_loss_thresh = clamp(ucp_lt_loss_thresh, 1, 65535);              /* LT loss thresh [1, 65535] */
    ucp_lt_bw_ratio_num = clamp(ucp_lt_bw_ratio_num, 0, 100000);           /* LT BW ratio num [0, 100k] */
    ucp_lt_bw_ratio_den = clamp(ucp_lt_bw_ratio_den, 1, 100000);           /* LT BW ratio den [1, 100k] */
    ucp_lt_bw_diff = clamp(ucp_lt_bw_diff, 0, 100000);                     /* LT BW diff [0, 100k] */
    ucp_lt_bw_ema_num = clamp(ucp_lt_bw_ema_num, 0, 100);
    ucp_lt_bw_ema_den = clamp(ucp_lt_bw_ema_den, 1, 100000);
    ucp_lt_bw_ema_num = min_t(int, ucp_lt_bw_ema_num, ucp_lt_bw_ema_den);
    ucp_lt_bw_max_rtts = clamp(ucp_lt_bw_max_rtts, 1, 4094);               /* LT BW max RTTs [1, 4094], 12-bit field max = 4095 */
    ucp_lt_bw_probe_pct = clamp(ucp_lt_bw_probe_pct, 0, 100);              /* LT BW probe pct [0, 100] */
    ucp_lt_bw_inst_qdelay_thresh_us = clamp(ucp_lt_bw_inst_qdelay_thresh_us, 0, 100000); /* LT BW inst qdelay thresh [0, 100k us] */
    ucp_lt_intvl_max_mult = clamp(ucp_lt_intvl_max_mult, 1, 32);            /* LT timeout mult [1, 32] */

    /* LT BW auto-recovery params */
    ucp_lt_restore_ratio_num = clamp(ucp_lt_restore_ratio_num, 0, 100000); /* restore ratio num [0, 100k] */
    ucp_lt_restore_ratio_den = clamp(ucp_lt_restore_ratio_den, 1, 100000); /* restore ratio den [1, 100k] */
    ucp_lt_restore_consec_acks = clamp(ucp_lt_restore_consec_acks, 1, 31); /* consec ACKs [1, 31], 5-bit field max = 31 */
    /* ratio=0 means disabled, already clamped above */

    ucp_extra_acked_max_ms_num = clamp(ucp_extra_acked_max_ms_num, 0, 100000);          /* ACK-agg max ms num [0, 100k] */
    ucp_extra_acked_max_ms_den = clamp(ucp_extra_acked_max_ms_den, 1, 100000);          /* ACK-agg max ms den [1, 100k] */

    ucp_probe_rtt_mode_ms_num = clamp(ucp_probe_rtt_mode_ms_num, 1, 100000);            /* PROBE_RTT mode ms num [1, 100k] */
    ucp_probe_rtt_mode_ms_den = clamp(ucp_probe_rtt_mode_ms_den, 1, 100000);            /* PROBE_RTT mode ms den [1, 100k] */

    ucp_bw_rt_cycle_len = clamp(ucp_bw_rt_cycle_len, 2, 256);                            /* BW sliding window [2, 256] */
    ucp_cwnd_min_target = clamp(ucp_cwnd_min_target, 1, 1000);                            /* min cwnd target [1, 1000] */

    /* ACK agg confidence-based compensation params */
    ucp_agg_enable = clamp(ucp_agg_enable, 0, 1);                          /* agg enable [0, 1] */
    ucp_agg_confidence_thresh = clamp(ucp_agg_confidence_thresh, 0, 10000); /* confidence thresh [0, 10k] */
    ucp_agg_max_comp_ratio = clamp(ucp_agg_max_comp_ratio, 0, 100);        /* max comp ratio [0, 100] */
    ucp_agg_max_comp_duration = clamp(ucp_agg_max_comp_duration, 1, 128);  /* max comp duration [1, 128] */
    ucp_agg_r_hysteresis = clamp(ucp_agg_r_hysteresis, 0, 100);            /* R hysteresis [0, 100] */
    ucp_agg_r_multiplier_min = clamp(ucp_agg_r_multiplier_min, 1, 10000);  /* R mult min [1, 10000] */
    ucp_agg_r_multiplier_max = clamp(ucp_agg_r_multiplier_max, 1, 10000);  /* R mult max [1, 10000] */
    ucp_agg_r_multiplier_max = max(ucp_agg_r_multiplier_max, ucp_agg_r_multiplier_min);  /* ensure max >= min */
    ucp_agg_factor3_qdelay_us = clamp(ucp_agg_factor3_qdelay_us, 0, 100000);     /* factor3 qdelay [0, 100k] */
    ucp_agg_factor4_ratio_num = clamp(ucp_agg_factor4_ratio_num, 1, 100000);    /* factor4 num [1, 100k] */
    ucp_agg_factor4_ratio_den = clamp(ucp_agg_factor4_ratio_den, 1, 100000);    /* factor4 den [1, 100k] */
    ucp_agg_safety_qdelay_us = clamp(ucp_agg_safety_qdelay_us, 0, 100000);      /* safety qdelay [0, 100k] */
    ucp_agg_safety_bdp_mult = clamp(ucp_agg_safety_bdp_mult, 1, 100);           /* safety bdp mult [1, 100] */
    ucp_agg_max_window_ms = clamp(ucp_agg_max_window_ms, 1, 10000);             /* max window ms [1, 10k] */
    ucp_agg_max_decay_pct = clamp(ucp_agg_max_decay_pct, 0, 100);               /* max decay pct [0, 100] */
    ucp_agg_max_per_ack_decay = clamp(ucp_agg_max_per_ack_decay, 0, 128);       /* per-ACK decay [0, 128]; 128=disabled */
    ucp_agg_max_per_ack_decay_den = clamp(ucp_agg_max_per_ack_decay_den, 1, 65535); /* per-ACK decay den [1, 65535] */
    ucp_agg_window_rotation_rtts = clamp(ucp_agg_window_rotation_rtts, 1, 65535);   /* window rotation RTTs [1, 65535] */
    ucp_extra_acked_win_rtts_max = clamp(ucp_extra_acked_win_rtts_max, 1, 65535);
    ucp_agg_window_rotation_rtts = min_t(int, ucp_agg_window_rotation_rtts, ucp_extra_acked_win_rtts_max);

    /* Bitmask values: all 32 bits per word are valid (256 phases across 8 words).
     * Cast through unsigned to preserve bit 31 (which would be negative as signed int). */
    for (i = 0; i < UCP_DECAY_MASK_WORDS; i++) {
        ucp_cycle_decay_mask[i] = (int)((u32)ucp_cycle_decay_mask[i]);
    }

    /*
     * Compute derived values and assign to the _val cache.
     * No concurrent-read protection needed — see "CONCURRENCY & SAFETY MODEL"
     * at struct ucp_ext for details.
     */

     /* PROBE_RTT intervals: sec * HZ -> jiffies, guarded against overflow */
    ucp_probe_rtt_base_jiffies = (u32)min_t(u64, (u64)ucp_probe_rtt_base_sec * HZ, U32_MAX);         /* sec * HZ capped at U32_MAX */
    ucp_probe_rtt_max_jiffies = (u32)min_t(u64, (u64)ucp_probe_rtt_max_sec * HZ, U32_MAX);          /* sec * HZ capped at U32_MAX */
    /* dyn_max must be > base_sec for valid interpolation range in ucp_update_dyn_probe_interval().
     * Enforce this constraint on the derived jiffies value without mutating the raw sysctl param. */
    {
        int dyn_sec = ucp_probe_rtt_dyn_max_sec;
        if (dyn_sec != 0 && dyn_sec <= ucp_probe_rtt_base_sec) {
            dyn_sec = (ucp_probe_rtt_base_sec < UCP_PROBE_RTT_MAX_SEC) ? (ucp_probe_rtt_base_sec + 1) : ucp_probe_rtt_base_sec;
        }
        ucp_probe_rtt_dyn_max_jiffies = (u32)min_t(u64, (u64)dyn_sec * HZ, U32_MAX);
    }

    /*
     * CWND gain: num/den * BBR_UNIT, clamped to fit 10-bit pacing_gain field.
     * ACK-agg gain is not clamped (read as multiplier, fits u32).
     */
    ucp_cwnd_gain_val = min_t(u32, (u32)((u64)BBR_UNIT * (u32)ucp_cwnd_gain_num / (u32)ucp_cwnd_gain_den), UCP_GAIN_MAX); /* clamp to 10 bits */
    ucp_extra_acked_gain_val = (u32)((u64)BBR_UNIT * (u32)ucp_extra_acked_gain_num / (u32)ucp_extra_acked_gain_den); /* num/den * BBR_UNIT */

    /*
     * STARTUP high_gain: ceiling division so that 2885/1000 maps to
     * ceil(2885 * 256 / 1000) = 739 (approx 2.887x BBR_UNIT)
     * (Cardwell et al. 2016).
     * Both high_gain and drain_gain are capped at 1023 to prevent bitfield
     * overflow in the 10-bit pacing_gain field.
     */
    ucp_high_gain_val = min_t(u32, (u32)(((u64)BBR_UNIT * (u32)ucp_high_gain_num + (u32)ucp_high_gain_den - 1) / (u32)ucp_high_gain_den), UCP_GAIN_MAX); /* ceil(num/den * BBR_UNIT) */
    ucp_drain_gain_val = min_t(u32, (u32)(((u64)BBR_UNIT * (u32)ucp_drain_gain_num + (u32)ucp_drain_gain_den - 1) / (u32)ucp_drain_gain_den), UCP_GAIN_MAX); /* ceil(num/den * BBR_UNIT), cap to UCP_GAIN_MAX */

    /* Cycle length and full-BW threshold */
    ucp_probe_bw_cycle_len_val = (u32)ucp_probe_bw_cycle_len;     /* publish cycle length */
    ucp_full_bw_thresh_val = (u32)((u64)BBR_UNIT * (u32)ucp_full_bw_thresh_num / (u32)ucp_full_bw_thresh_den); /* num/den * BBR_UNIT */
    ucp_full_bw_cnt_val = (u32)ucp_full_bw_cnt;                                  /* publish full-BW round count (already clamped to [1,3] at line 1548, fits 2-bit field) */

    /* Inflight bounds as BBR_UNIT multiples */
    ucp_inflight_low_gain_val = (u32)((u64)BBR_UNIT * (u32)ucp_inflight_low_gain_num / (u32)ucp_inflight_low_gain_den); /* num/den * BBR_UNIT */
    ucp_inflight_high_gain_val = (u32)((u64)BBR_UNIT * (u32)ucp_inflight_high_gain_num / (u32)ucp_inflight_high_gain_den); /* num/den * BBR_UNIT */

    /* Kalman clamped scalars (Kalman 1960) */
    ucp_kalman_p_est_max_val = (u32)ucp_kalman_p_est_max;         /* publish p_est max */
    ucp_kalman_converged_p_est_val = (u32)ucp_kalman_converged_p_est; /* publish converged p_est threshold */
    ucp_drain_skip_qdelay_us_val = (u32)ucp_drain_skip_qdelay_us;   /* publish drain skip qdelay */
    /* Cache clamped Q/R for hot-path use (avoids raw-param read race) */
    ucp_kalman_q_val = ucp_kalman_q;                                /* publish clamped Q */
    ucp_kalman_r_val = ucp_kalman_r;                                /* publish clamped R */

    /*
     * Q-boost threshold: when |innovation| exceeds this value, the filter
     * resets p_est to p_est_init, re-entering the high-gain phase to
     * rapidly track the changed path characteristic.
     * Formula: boost_mult * boost_ms * 1000 us/ms * kalman_scale.
     */
     /* Q-boost threshold: guard each multiply against u64 overflow.
      * At extreme parameters (mult=10000, ms=5000, scale=1048576) the product
      * exceeds U64_MAX; we check each step and saturate to U64_MAX safely. */
    {
        u64 qbt = (u64)ucp_kalman_q_boost_mult;
        if (qbt <= U64_MAX / (u64)ucp_kalman_q_boost_ms) {
            qbt *= (u64)ucp_kalman_q_boost_ms;
        }
        else {
            qbt = U64_MAX;
        }
        if (qbt <= U64_MAX / USEC_PER_MSEC) {
            qbt *= USEC_PER_MSEC;
        }
        else {
            qbt = U64_MAX;
        }
        if (qbt <= U64_MAX / (u64)ucp_kalman_scale) {
            qbt *= (u64)ucp_kalman_scale;
        }
        else {
            qbt = U64_MAX;
        }
        ucp_kalman_q_boost_thresh_val = (u32)min_t(u64, qbt, U32_MAX);
    }
    ucp_kalman_qboost_cdwn_val = (u32)ucp_kalman_qboost_cdwn;     /* publish Q-boost cooldown */
    ucp_kalman_q_max_val = ucp_kalman_q_max;                       /* publish Q max ceiling */
    ucp_kalman_q_scale_cap_val = ucp_kalman_q_scale_cap;           /* publish Q scale cap */
    ucp_kalman_min_samples_val = ucp_kalman_min_samples;           /* publish min Kalman samples */
    ucp_rtt_sample_max_us_val = (u32)ucp_rtt_sample_max_us;        /* publish max RTT sample */
    ucp_minrtt_fast_fall_cnt_val = ucp_minrtt_fast_fall_cnt;       /* publish fast-fall count */
    ucp_minrtt_fast_fall_div = clamp(ucp_minrtt_fast_fall_div, 1, 256);       /* prevent div-by-zero */
    ucp_minrtt_fast_fall_div_val = ucp_minrtt_fast_fall_div;       /* publish fast-fall divisor */
    ucp_kalman_probe_band_mult = clamp(ucp_kalman_probe_band_mult, 1, 32);    /* prevent u32 overflow */
    ucp_kalman_probe_band_mult_val = ucp_kalman_probe_band_mult;    /* publish probe band mult */
    ucp_kalman_q_rtt_div = clamp(ucp_kalman_q_rtt_div, 1, 1000000);           /* prevent div-by-zero */
    ucp_kalman_q_rtt_div_val = ucp_kalman_q_rtt_div;                /* publish Q RTT divisor */
    ucp_kalman_rtt_dyn_mult = clamp(ucp_kalman_rtt_dyn_mult, 1, 100);         /* prevent u32 overflow */
    ucp_kalman_rtt_dyn_mult_val = ucp_kalman_rtt_dyn_mult;          /* publish RTT dyn mult */

    /* Cache min-RTT sticky ratio and SRTT guard ratio as num/den pairs */
    {
        u32 snum = (u32)ucp_minrtt_sticky_num;                          /* read sticky num to local */
        u32 sden = (u32)ucp_minrtt_sticky_den;                          /* read sticky den to local */
        ucp_minrtt_sticky_num_val = snum;                     /* publish cached sticky num */
        ucp_minrtt_sticky_den_val = sden;                     /* publish cached sticky den */
        snum = (u32)ucp_minrtt_srtt_guard_num;                       /* read SRTT guard num to local */
        sden = (u32)ucp_minrtt_srtt_guard_den;                       /* read SRTT guard den to local */
        ucp_minrtt_srtt_guard_num_val = snum;                  /* publish cached SRTT guard num */
        ucp_minrtt_srtt_guard_den_val = sden;                  /* publish cached SRTT guard den */
    }
    ucp_bdp_min_rtt_us_val = (u32)ucp_bdp_min_rtt_us;             /* publish BDP min-RTT floor */
    ucp_probe_cwnd_bonus_val = ucp_probe_cwnd_bonus;              /* publish probe cwnd bonus */
    ucp_edt_near_now_ns_val = ucp_edt_near_now_ns;                /* publish EDT near-now threshold */
    ucp_min_tso_rate_val = ucp_min_tso_rate;                       /* publish min TSO rate */
    ucp_tso_max_segs_val = ucp_tso_max_segs;                       /* publish max TSO segs */
    ucp_tso_segs_low_val = ucp_tso_segs_low;
    ucp_tso_segs_default_val = ucp_tso_segs_default;
    ucp_tso_headroom_mult = clamp(ucp_tso_headroom_mult, 0, 1000);
    ucp_tso_headroom_mult_val = ucp_tso_headroom_mult;             /* publish TSO headroom mult */
    ucp_min_tso_rate_div = clamp(ucp_min_tso_rate_div, 1, 256);               /* prevent div-by-zero */
    ucp_min_tso_rate_div_val = ucp_min_tso_rate_div;               /* publish min TSO rate divisor */
    ucp_jitter_probe_thresh_us_val = ucp_jitter_probe_thresh_us;  /* publish jitter probe thresh */
    ucp_jitter_probe_scale_us_val = ucp_jitter_probe_scale_us;    /* publish jitter probe scale */
    ucp_qdelay_probe_thresh_us_val = ucp_qdelay_probe_thresh_us;  /* publish qdelay probe thresh */
    ucp_qdelay_probe_scale_us_val = ucp_qdelay_probe_scale_us;    /* publish qdelay probe scale */
    ucp_jitter_r_thresh_us_val = ucp_jitter_r_thresh_us;          /* publish adaptive R jitter thresh */
    ucp_jitter_r_scale_val = ucp_jitter_r_scale;                   /* publish adaptive R jitter scale */
    ucp_kalman_r_max_boost_val = ucp_kalman_r_max_boost;          /* publish R max boost */
    ucp_probe_rtt_long_rtt_us_val = ucp_probe_rtt_long_rtt_us;     /* publish long-RTT threshold */
    ucp_probe_rtt_long_interval_div = clamp(ucp_probe_rtt_long_interval_div, 1, 1000); /* prevent div-by-zero */
    ucp_probe_rtt_long_interval_div_val = ucp_probe_rtt_long_interval_div; /* publish long-RTT interval div */

    /* ACK agg confidence compensation: publish clamped values */
    ucp_agg_enable_val = ucp_agg_enable;                        /* publish agg enable */
    ucp_agg_confidence_thresh_val = ucp_agg_confidence_thresh;  /* publish confidence thresh */
    ucp_agg_max_comp_ratio_val = ucp_agg_max_comp_ratio;        /* publish max comp ratio */
    ucp_agg_max_comp_duration_val = ucp_agg_max_comp_duration;  /* publish max comp duration */
    ucp_agg_r_hysteresis_val = ucp_agg_r_hysteresis;            /* publish R hysteresis */
    ucp_agg_r_multiplier_min_val = (u32)ucp_agg_r_multiplier_min; /* publish R mult min */
    ucp_agg_r_multiplier_max_val = (u32)ucp_agg_r_multiplier_max; /* publish R mult max */
    ucp_agg_factor3_qdelay_us_val = ucp_agg_factor3_qdelay_us;       /* publish factor3 qdelay */
    ucp_agg_factor4_ratio_num_val = ucp_agg_factor4_ratio_num;       /* publish factor4 ratio num */
    ucp_agg_factor4_ratio_den_val = ucp_agg_factor4_ratio_den;       /* publish factor4 ratio den */
    ucp_agg_safety_qdelay_us_val = ucp_agg_safety_qdelay_us;         /* publish safety qdelay */
    ucp_agg_safety_bdp_mult_val = ucp_agg_safety_bdp_mult;           /* publish safety bdp mult */
    ucp_agg_max_window_ms_val = ucp_agg_max_window_ms;               /* publish max window ms */
    ucp_agg_max_decay_pct_val = ucp_agg_max_decay_pct;               /* publish max decay pct */
    ucp_agg_max_per_ack_decay_val = ucp_agg_max_per_ack_decay;       /* publish per-ACK decay */
    ucp_agg_max_per_ack_decay_den_val = ucp_agg_max_per_ack_decay_den; /* publish per-ACK decay denom */
    ucp_agg_window_rotation_rtts_val = ucp_agg_window_rotation_rtts; /* publish window rotation RTTs */
    ucp_extra_acked_win_rtts_max_val = ucp_extra_acked_win_rtts_max;
    ucp_agg_per_ack_decay_active = ((u32)ucp_agg_max_per_ack_decay_val < (u32)ucp_agg_max_per_ack_decay_den_val); /* precompute per-ACK decay gate */
    ucp_agg_factor_weight = clamp(ucp_agg_factor_weight, 1, UCP_AGG_CONFIDENCE_MAX);            /* clamp per-factor weight */
    ucp_agg_thresh_suspected = clamp(ucp_agg_thresh_suspected, 1, UCP_AGG_CONFIDENCE_MAX);      /* clamp SUSPECTED threshold */
    ucp_agg_thresh_confirmed = clamp(ucp_agg_thresh_confirmed, 2, UCP_AGG_CONFIDENCE_MAX);      /* clamp CONFIRMED threshold */
    ucp_agg_thresh_trusted = clamp(ucp_agg_thresh_trusted, 3, UCP_AGG_CONFIDENCE_MAX);          /* clamp TRUSTED threshold */
    /* enforce strict ordering: suspected < confirmed < trusted, clamp within [1..1024] */
    ucp_agg_thresh_confirmed = min_t(u32, max(ucp_agg_thresh_confirmed, ucp_agg_thresh_suspected + 1), 1024);
    ucp_agg_thresh_trusted = min_t(u32, max(ucp_agg_thresh_trusted, ucp_agg_thresh_confirmed + 1), 1024);
    ucp_agg_thresh_suspected_val = ucp_agg_thresh_suspected;        /* publish SUSPECTED threshold */
    ucp_agg_thresh_confirmed_val = ucp_agg_thresh_confirmed;        /* publish CONFIRMED threshold */
    ucp_agg_thresh_trusted_val = ucp_agg_thresh_trusted;          /* publish TRUSTED threshold */

    /*
     * Pacing margin: divisor = 100 - (num * 100 / den).
     * With num=1, den=100 -> divisor=99: rate = raw_rate * 99 / 100.
     * Clamped [1, 100] to prevent overflow in rate_bytes_per_sec.
     */
    {
        u32 num = (u32)ucp_pacing_margin_num;                               /* read pacing margin num */
        u32 den = (u32)ucp_pacing_margin_den;                               /* read pacing margin den */
        s32 margin;
        num = min_t(u32, num, den);
        margin = 100 - (s32)((u64)num * 100 / (u64)max_t(u32, den, 1));                        /* compute margin percentage */
        ucp_pacing_margin_div_val = (u32)clamp(margin, 1, 100);   /* publish divisor clamped [1, 100] */
    }
    /* Kalman core cached values (Kalman 1960) */
    ucp_kalman_p_est_init_val = (u32)ucp_kalman_p_est_init;       /* publish p_est init */
    ucp_kalman_p_est_floor_val = (u32)ucp_kalman_p_est_floor;     /* publish p_est floor */
    ucp_kalman_outlier_ms_val = (u32)ucp_kalman_outlier_ms;       /* publish outlier ms */
    ucp_kalman_scale_shift_val = (u32)ilog2(ucp_kalman_scale);      /* publish Kalman shift (scale = 1<<shift) */

    /* Precompute Kalman outlier rejection constants.
     * These are invariant between sysctl writes — recomputing them
     * on every Kalman invocation is wasteful on the hot path. */
    {
        u64 ms_us = (u64)ucp_kalman_outlier_ms_val * USEC_PER_MSEC;
        u32 shift = ucp_kalman_scale_shift_val;
        ucp_kalman_shift_cap_val = U64_MAX >> shift;
        ucp_kalman_outlier_thresh_scaled_val = min_t(u64, ms_us, ucp_kalman_shift_cap_val) << shift;
    }

    /* Derived Kalman ratios (num/den -> single value) (Kalman 1960) */
    {
        u32 num = (u32)ucp_kalman_outlier_jitter_mult_num;                  /* read outlier jitter mult num */
        u32 den = (u32)ucp_kalman_outlier_jitter_mult_den;                  /* read outlier jitter mult den */
        ucp_kalman_outlier_jitter_mult_val = num / den; /* publish (num/den) */
    }
    {
        u32 num = (u32)ucp_kalman_q_min_factor_num;                         /* read Q min factor num */
        u32 den = (u32)ucp_kalman_q_min_factor_den;                         /* read Q min factor den */
        ucp_kalman_q_min_factor_val = num / den;        /* publish (num/den) */
    }
    {
        u32 num = (u32)ucp_kalman_p_est_init_rtt_div_num;                   /* read p_est init RTT div num */
        u32 den = (u32)ucp_kalman_p_est_init_rtt_div_den;                   /* read p_est init RTT div den */
        u32 val = max_t(u32, num / den, 1U);                      /* compute divisor, floor at 1 (den clamped >= 1) */
        ucp_kalman_p_est_init_rtt_div_val = val;                  /* publish p_est init RTT divisor */
    }
    /* EWMA coefficients */
    ucp_ewma_qdelay_num_val = ucp_ewma_qdelay_num;                /* publish EWMA qdelay num */
    ucp_ewma_qdelay_den_val = ucp_ewma_qdelay_den;                /* publish EWMA qdelay den */
    ucp_ewma_jitter_num_val = ucp_ewma_jitter_num;                /* publish EWMA jitter num */
    ucp_ewma_jitter_den_val = ucp_ewma_jitter_den;                /* publish EWMA jitter den */

    /* BBR-S covariance-matched noise estimation: publish snapped/clamped values */
    ucp_kalman_noise_alpha_num_val = ucp_kalman_noise_alpha_num;   /* publish noise alpha num */
    ucp_kalman_noise_alpha_den_val = ucp_kalman_noise_alpha_den;   /* publish noise alpha den */
    ucp_kalman_noise_beta_num_val = ucp_kalman_noise_beta_num;     /* publish noise beta num */
    ucp_kalman_noise_beta_den_val = ucp_kalman_noise_beta_den;     /* publish noise beta den */
    ucp_kalman_q_est_max_val = ucp_kalman_q_est_max;               /* publish Q est max */
    ucp_kalman_r_est_max_val = ucp_kalman_r_est_max;               /* publish R est max */
    ucp_kalman_q_est_floor_val = ucp_kalman_q_est_floor;           /* publish Q est floor */
    ucp_kalman_r_est_floor_val = ucp_kalman_r_est_floor;           /* publish R est floor */
    ucp_kalman_noise_mode_val = ucp_kalman_noise_mode;             /* publish noise mode */
    ucp_kalman_noise_avg_num_val = ucp_kalman_noise_avg_num;
    ucp_kalman_noise_avg_den_val = ucp_kalman_noise_avg_den;

    /* Precompute noise estimation complements for hot-path use (BBR-S) */
    ucp_kalman_noise_alpha_complement = (u32)ucp_kalman_noise_alpha_den_val - (u32)ucp_kalman_noise_alpha_num_val; /* alpha_d - alpha_n */
    ucp_kalman_noise_beta_complement = (u32)ucp_kalman_noise_beta_den_val - (u32)ucp_kalman_noise_beta_num_val;   /* beta_d - beta_n */

    /* ECN: publish clamped values and derived backoff ratio */
    ucp_alone_confirm_rounds_val = ucp_alone_confirm_rounds;         /* publish alone confirm rounds */
    ucp_alone_qdelay_thresh_us_val = ucp_alone_qdelay_thresh_us;    /* publish alone qdelay threshold */
    ucp_alone_jitter_thresh_us_val = ucp_alone_jitter_thresh_us;    /* publish alone jitter threshold */
    ucp_alone_agg_state_level_val = ucp_alone_agg_state_level;      /* publish alone agg state level */
    ucp_alone_bypass_ecn_val = ucp_alone_bypass_ecn;                /* publish alone bypass ECN flag */
    ucp_alone_bypass_lt_bw_val = ucp_alone_bypass_lt_bw;            /* publish alone bypass LT BW flag */
    ucp_ecn_enable_val = ucp_ecn_enable;                            /* publish ECN enable */
    ucp_ecn_backoff_val = (u32)((u64)BBR_UNIT * (u32)ucp_ecn_backoff_num / (u32)ucp_ecn_backoff_den); /* backoff = BBR_UNIT * num / den */
    ucp_ecn_qdelay_thresh_us_val = ucp_ecn_qdelay_thresh_us;       /* publish ECN qdelay threshold */
    ucp_ecn_ewma_retained_val = ucp_ecn_ewma_retained;              /* publish ECN EWMA retained */
    ucp_ecn_ewma_total_val = ucp_ecn_ewma_total;                    /* publish ECN EWMA total */

    /* ECN idle decay: publish snapped values */
    ucp_ecn_idle_decay_num_val = ucp_ecn_idle_decay_num;             /* publish ECN idle decay num */
    ucp_ecn_idle_decay_den_val = ucp_ecn_idle_decay_den;             /* publish ECN idle decay den */

    /* Kalman outlier rejection: publish clamped consecutive reject limit */
    ucp_kalman_max_consec_reject_val = ucp_kalman_max_consec_reject; /* publish consec reject limit */

    /* PROBE_BW random offset and LT BW derived values */
    ucp_probe_bw_cycle_rand_val = ucp_probe_bw_cycle_rand;        /* publish PROBE_BW rand range */
    ucp_probe_bw_up_limit_val = ucp_probe_bw_up_limit;          /* publish PROBE_BW up-phase limit flag */
    ucp_lt_intvl_min_rtts_val = ucp_lt_intvl_min_rtts;            /* publish LT min RTTs */
    ucp_lt_intvl_max_mult_val = ucp_lt_intvl_max_mult;             /* publish LT timeout mult */
    ucp_lt_loss_thresh_val = (u32)ucp_lt_loss_thresh;                  /* publish LT loss threshold */
    ucp_lt_bw_ratio_val = (u32)((u64)BBR_UNIT * (u32)ucp_lt_bw_ratio_num / (u32)ucp_lt_bw_ratio_den); /* num/den * BBR_UNIT */
    ucp_lt_bw_diff_val = (u32)ucp_lt_bw_diff;                     /* publish LT BW absolute diff */
    ucp_lt_bw_max_rtts_val = ucp_lt_bw_max_rtts;                  /* publish LT BW max RTTs */
    ucp_lt_bw_ema_num_val = ucp_lt_bw_ema_num;
    ucp_lt_bw_ema_den_val = ucp_lt_bw_ema_den;
    ucp_lt_bw_probe_pct_val = ucp_lt_bw_probe_pct;

    /* LT BW auto-recovery: publish snapped values */
    ucp_lt_restore_ratio_num_val = ucp_lt_restore_ratio_num;      /* publish restore ratio num */
    ucp_lt_restore_ratio_den_val = ucp_lt_restore_ratio_den;      /* publish restore ratio den */
    ucp_lt_restore_consec_acks_val = ucp_lt_restore_consec_acks;   /* publish restore consec ACKs */

    /* Pacing rate double threshold, ACK aggregation max, PROBE_RTT mode duration */
    {
        u32 num = (u32)ucp_extra_acked_max_ms_num;                           /* read ACK-agg max ms num */
        u32 den = (u32)ucp_extra_acked_max_ms_den;                           /* read ACK-agg max ms den */
        ucp_extra_acked_max_ms_val = num / den;          /* publish (num/den) (den clamped >= 1) */
    }
    {
        u32 num = (u32)ucp_probe_rtt_mode_ms_num;                            /* read PROBE_RTT mode ms num */
        u32 den = (u32)ucp_probe_rtt_mode_ms_den;                            /* read PROBE_RTT mode ms den */
        u32 val = max_t(u32, num / den, 1U);                      /* compute stay duration, floor at 1 (den clamped >= 1) */
        ucp_probe_rtt_mode_ms_val = val;                           /* publish PROBE_RTT mode ms */
    }
    /*
     * If the gain table has never been explicitly configured (still at
     * static-zero defaults), populate the first cycle_len entries with
     * BBRv1-compatible values.  This ensures UCP probes for bandwidth
     * at 1.25x gain on phase 0 and drains at 0.75x on phase 1, matching
     * the BBR PROBE_BW cycle shape that is essential for discovering
     * available bandwidth above the sliding-window maximum.
     *
     * The pattern repeats modulo cycle_len over the full 256-slot table.
     */
    if (ucp_gain_table_defaulted) {
        int k, phase = 0;
        for (k = 0; k < UCP_GAIN_SLOTS; k++) {
            switch (phase) {
            case 0:  ucp_gain_num[k] = 5; ucp_gain_den[k] = 4; break; /*  5/4 = 1.25x probe */
            case 1:  ucp_gain_num[k] = 3; ucp_gain_den[k] = 4; break; /*  3/4 = 0.75x drain */
            default: ucp_gain_num[k] = 1; ucp_gain_den[k] = 1; break; /*  1/1 = 1.0x cruise */
            }
            if (++phase >= ucp_probe_bw_cycle_len) {
                phase = 0;
            }
        }
    }
    /* Rebuild the cycle gain table from the (possibly updated) arrays */
    ucp_rebuild_gain_table();                                                 /* recompute ucp_cycle_gain_table[] */

    ucp_bw_rt_cycle_len_val = ucp_bw_rt_cycle_len;                 /* publish BW RT cycle length */
    ucp_cwnd_min_target_val = ucp_cwnd_min_target;                 /* publish min cwnd target */
    ucp_sndbuf_expand_factor = clamp(ucp_sndbuf_expand_factor, 2, 100);       /* clamp sndbuf factor */
    ucp_sndbuf_expand_factor_val = ucp_sndbuf_expand_factor;        /* publish sndbuf expand factor */
    ucp_ack_epoch_max = clamp(ucp_ack_epoch_max, 65536, 0x7FFFFFFF);         /* clamp epoch cap [64K, 2G] */
    ucp_ack_epoch_max_val = (u32)ucp_ack_epoch_max;                 /* publish epoch cap */
}                                                                             /* ucp_init_module_params */

/* ---- Forward Declarations -------------------------------------------- */
static void ucp_check_probe_rtt_done(struct sock* sk);                        /* forward: check PROBE_RTT exit */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)                            /* kernel 6.10+ adds ack/flags to cong_control */
void ucp_main(struct sock* sk, u32 ack __maybe_unused, int flags __maybe_unused, const struct rate_sample* rs); /* main entry (6.10+ sig) */
#else                                                                          /* pre-6.10 signature */
void ucp_main(struct sock* sk, const struct rate_sample* rs);                /* main entry (legacy sig) */
#endif
static void ucp_update_model(struct sock* sk, const struct rate_sample* rs,   /* forward: per-ACK model update */
    struct ucp_ext* ext);                                        /* extended state (may be NULL) */
static void ucp_alone_on_path_eval(struct sock* sk, struct ucp_ext* ext); /* forward: single-flow detection */
static void ucp_apply_cwnd_constraints(struct sock* sk, struct ucp_ext* ext); /* forward: apply cwnd gain caps */
static u32 ucp_ack_aggregation_cwnd(struct sock* sk, struct ucp_ext* ext, u32 bw);    /* forward: ACK agg cwnd bonus */
/* ACK aggregation confidence module forward declarations */
static u32 ucp_measure_ack_aggregation(struct sock* sk, const struct rate_sample* rs, struct ucp_ext* ext);
static u16 ucp_evaluate_agg_confidence(struct sock* sk, struct ucp_ext* ext, u32 extra_acked);
static u8 ucp_agg_state_from_confidence(u16 confidence);
static u32 ucp_agg_cwnd_compensation(struct sock* sk, struct ucp_ext* ext, u32 extra_acked, u16 confidence, u32 bw);
/* UCP_KFUNC functions — non-static for BTF kfunc registration */
void ucp_init(struct sock* sk);                                                /* per-connection init */
u32 ucp_min_tso_segs(struct sock* sk);                                          /* minimum TSO segments */
void ucp_cwnd_event(struct sock* sk, enum tcp_ca_event event);                  /* congestion event handler */
u32 ucp_sndbuf_expand(struct sock* sk);                                          /* send buffer expansion factor */
u32 ucp_undo_cwnd(struct sock* sk);                                               /* cwnd undo on spurious loss */
u32 ucp_ssthresh(struct sock* sk);                                                 /* ssthresh query */
void ucp_set_state(struct sock* sk, u8 new_state);                                  /* CA state transition handler */

/* ---- Extended State Helpers ------------------------------------------- */

static inline struct ucp_ext* ucp_ext_get(const struct sock* sk)
{
    return ((struct ucp*)inet_csk_ca(sk))->ext;
}

/*
 * ucp_ext_destruct - Null the ext pointer and free the extended-state block.
 * @sk: TCP socket.
 *
 * Called from ucp_release() on socket close (non-softirq context).
 * No RCU needed -- see "CONCURRENCY & SAFETY MODEL" at struct ucp_ext.
 */
static void ucp_ext_destruct(struct sock* sk)
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);
    struct ucp_ext* ext = ucp->ext;

    if (!ext) {
        return;
    }

    ucp->ext = NULL;
    kfree(ext);
}
/*
 * ucp_release - Release callback invoked when a UCP connection is closed.
 * @sk: TCP socket.
 */
static void ucp_release(struct sock* sk)                                     /* socket close callback */
{
    ucp_ext_destruct(sk);                                                    /* destroy extended state */
}
/*
 * ucp_full_bw_reached - Query whether pipe-fill has been detected
 * (Cardwell et al. 2016).
 * @sk: TCP socket.
 *
 * Returns the 1-bit full_bw_reached flag.  Once set, UCP transitions
 * from STARTUP to DRAIN, and subsequent PROBE_BW uses this flag to
 * enable cwnd-to-target convergence (vs. exponential growth).
 */
static bool ucp_full_bw_reached(const struct sock* sk)                       /* check if pipe capacity detected */
{
    return ((struct ucp*)inet_csk_ca(sk))->full_bw_reached;                  /* return full_bw_reached bitfield */
}
/*
 * ucp_max_bw - Return the sliding-window maximum bandwidth estimate.
 * @sk: TCP socket.
 *
 * Reads the max from the struct minmax running over ucp_bw_rt_cycle_len
 * round-trip windows.  Stored in BW_UNIT (segments * (1<<24) per usec).
 */
static u32 ucp_max_bw(const struct sock* sk)                                  /* get sliding-window max BW */
{
    return minmax_get(&((struct ucp*)inet_csk_ca(sk))->bw);                 /* extract current max from minmax */
}
/*
 * ucp_bw - Return the active bandwidth estimate (either max_bw or lt_bw).
 * @sk: TCP socket.
 *
 * When lt_use_bw == 1 (long-term BW is active and consistent), returns
 * the LT BW estimate.  Otherwise returns the sliding-window max.
 *
 * Rationale: on lossy paths, the sliding-window max may track transient
 * spikes above the true fair-share rate; the LT BW estimate provides a
 * more stable, conservative baseline derived from loss episodes.
 */
static u32 ucp_bw(const struct sock* sk)                                      /* get active BW estimate */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get CA private area */

    return ucp->lt_use_bw ? ucp->lt_bw : ucp_max_bw(sk);                      /* LT BW active ? lt_bw : max_bw */
}
/*
 * ucp_get_cycle_mstamp - Reconstruct the 64-bit cycle timestamp from the
 * hi/lo 32-bit halves stored in the ucp struct.
 * @ucp: per-connection UCP state.
 */
static inline u64 ucp_get_cycle_mstamp(const struct ucp* ucp)                 /* reconstruct 64-bit cycle mstamp */
{
    return ((u64)ucp->cycle_mstamp_hi << UCP_MSTAMP_HI_SHIFT) | ucp->cycle_mstamp_lo;          /* combine hi<<UCP_MSTAMP_HI_SHIFT | lo */
}
/*
 * ucp_set_cycle_mstamp - Store a 64-bit tcp_mstamp as two 32-bit halves.
 * @ucp: per-connection UCP state.
 * @val: the 64-bit timestamp value.
 *
 * Used to record the start of each PROBE_BW cycle phase.  The hi/lo split
 * saves two u32 words in the size-constrained struct ucp.
 */
static inline void ucp_set_cycle_mstamp(struct ucp* ucp, u64 val)             /* store 64-bit mstamp as hi+lo */
{
    ucp->cycle_mstamp_hi = (u32)(val >> UCP_MSTAMP_HI_SHIFT);                                  /* store high 32 bits */
    ucp->cycle_mstamp_lo = (u32)(val);                                       /* store low 32 bits */
}
/* ---- Pacing and Rate Helpers ----------------------------------------- */

/*
 * ucp_rate_bytes_per_sec - Convert bandwidth (BW_UNIT) * gain (BBR_UNIT)
 * into a pacing rate in bytes/second, with pacing margin applied.
 * @sk:   TCP socket (for mss_cache).
 * @rate: bandwidth in BW_UNIT (segments * (1<<24) per usec).
 * @gain: pacing gain in BBR_UNIT units (256 = 1.0x).
 *
 * Formula (standard BBR pacing rate calc, Cardwell et al. 2016):
 *   step1:  (rate * gain) >> BBR_SCALE           -> gain-adjusted BW
 *   step2:  step1 * mss_cache                     -> raw bytes/interval
 *   step3:  step2 * USEC_PER_SEC >> BW_SCALE      -> bytes per second
 *   step4:  step3 * pacing_margin_div / 100       -> apply margin (e.g., 99/100)
 *
 * Overflow guard: before step4, cap at U64_MAX / divisor.
 */
static u64 ucp_rate_bytes_per_sec(struct sock* sk, u64 rate, u32 gain)        /* compute paced bytes/sec */
{
    unsigned int mss = tcp_sk(sk)->mss_cache;

    rate *= mss;
    rate = mul_u64_u32_shr(rate, gain, BBR_SCALE);
    rate = mul_u64_u32_shr(rate, USEC_PER_SEC, BW_SCALE);
    rate = rate * ucp_pacing_margin_div_val / UCP_PCT_BASE;
    return rate;
}
/*
 * ucp_bw_to_pacing_rate - Compute sk_pacing_rate from BW and gain,
 * capped by sk_max_pacing_rate (socket-level upper bound from e.g. SO_MAX_PACING_RATE).
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT units.
 * @gain: gain in BBR_UNIT units.
 */
static u64 ucp_bw_to_pacing_rate(struct sock* sk, u64 bw, u32 gain)           /* convert BW+gain to pacing rate */
{
    return min_t(u64, ucp_rate_bytes_per_sec(sk, bw, gain),                    /* computed bytes/sec */
        READ_ONCE(sk->sk_max_pacing_rate));                                          /* cap at socket max (READ_ONCE: settable via SO_MAX_PACING_RATE) */
}
/*
 * ucp_init_pacing_rate_from_rtt - Bootstrap pacing rate from cwnd and SRTT
 * before any bandwidth samples are available.
 * @sk: TCP socket.
 *
 * If SRTT is known: rate = (cwnd * BW_UNIT / srtt_us) * high_gain.
 * Otherwise: uses a 1 ms fallback RTT.
 * This ensures the connection has a valid pacing rate from the first ACK.
 */
static void ucp_init_pacing_rate_from_rtt(struct sock* sk)                    /* bootstrap pacing from cwnd+SRTT */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */
    u32 rtt_us;                                                              /* RTT estimate in us */
    u64 bw;                                                                  /* computed bandwidth */

    if (tp->srtt_us) {                                                       /* SRTT sample available */
        /* SRTT is stored as 8 * smoothed RTT in us; >>3 recovers actual us */
        rtt_us = max_t(u32, tp->srtt_us >> 3, 1U);                            /* extract smoothed RTT, floor at 1 */
        ucp->has_seen_rtt = 1;                                               /* mark: SRTT has been sampled */
    }
    else {                                                                 /* no SRTT yet */
        rtt_us = USEC_PER_MSEC;                                              /* fallback: 1 ms */
    }

    /* bw = cwnd * BW_UNIT / rtt_us  (bandwidth proxy from BDP) */
    bw = (u64)tcp_snd_cwnd(tp) * BW_UNIT;                                         /* cwnd in BW_UNIT scale */
    bw = div_u64(bw, rtt_us);                                   /* 64-bit / 32-bit division: bw in BW_UNIT */

    WRITE_ONCE(sk->sk_pacing_rate, ucp_bw_to_pacing_rate(sk, bw, ucp_high_gain_val)); /* set pacing rate with high gain */
}
/*
 * ucp_set_pacing_rate - Set the socket pacing rate.
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT units.
 * @gain: pacing gain in BBR_UNIT units.
 *
 * Rate application policy (matches BBR, Cardwell et al. 2016):
 *   - full_bw_reached: apply ALL rate changes immediately.  The pipe
 *     capacity is known; the pacing engine tracks bw directly.
 *   - STARTUP / DRAIN (not yet full_bw): only apply rate INCREASES.
 *     Transient dips are ignored — the bandwidth estimate should only
 *     grow during pipe-filling.
 *   - No rate smoothing is applied.  Smoothing acts as a low-pass filter
 *     that prevents bandwidth discovery from the 1.25x probe phase.
 */
static void ucp_set_pacing_rate(struct sock* sk, u64 bw, u32 gain)            /* set sk_pacing_rate (no smoothing — matches BBR, Cardwell et al. 2016) */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */
    u64 rate = ucp_bw_to_pacing_rate(sk, bw, gain);                            /* compute target pacing rate */

    /* Bootstrap: on the first SRTT sample, initialize pacing from RTT */
    if (unlikely(!ucp->has_seen_rtt && tp->srtt_us)) {                        /* first SRTT sample available */
        ucp_init_pacing_rate_from_rtt(sk);                                    /* bootstrap pacing from RTT */
    }

    /*
     * BBR rule (Cardwell et al. 2016): in steady state (full_bw_reached),
     * apply ALL rate changes immediately - the pipe capacity is known and
     * the pacing engine should track the bandwidth estimate directly.
     * Smoothing during steady state acts as a low-pass filter that
     * prevents bandwidth discovery from the 1.25x probe phase.
     *
     * During STARTUP (full_bw not yet reached), only apply increases.
     */
    if (likely(ucp_full_bw_reached(sk))) {
        /* Steady state: instant apply, matching BBR behavior */
        WRITE_ONCE(sk->sk_pacing_rate, rate);
        return;
    }

    /*
     * STARTUP / DRAIN / early PROBE_BW (full_bw not yet reached):
     * Only apply rate INCREASES, matching BBR's behavior:
     *   if (bbr_full_bw_reached(sk) || rate > sk_pacing_rate)
     *       WRITE_ONCE(sk->sk_pacing_rate, rate);
     * Decreases are ignored: during STARTUP the bandwidth estimate should
     * only grow, and transient dips should not pull down the pacing rate.
     */
    if (rate > READ_ONCE(sk->sk_pacing_rate)) {                               /* only apply increases */
        WRITE_ONCE(sk->sk_pacing_rate, rate);
    }
}
/*
 * ucp_min_tso_segs - Minimum TSO segments for the current pacing rate.
 * @sk: TCP socket.
 *
 * Returns 1 for low pacing rates (< ucp_min_tso_rate / divisor), 2 otherwise.
 * The divisor is adaptive: halved (4) when Kalman is converged with low jitter
 * (larger TSO bursts for high-confidence clean paths), doubled (16) when jitter
 * is high (smaller bursts for jittery paths).  Default base divisor is 8.
 */
UCP_KFUNC u32 ucp_min_tso_segs(struct sock* sk)                        /* compute minimum TSO segments */
{
    u32 div = ucp_min_tso_rate_div_val;                                        /* base divisor (default 8) */
    u32 tso_rate_thresh;                                                       /* rate threshold */
    struct ucp_ext* ext = ucp_ext_get(sk);                                     /* extended state (may be NULL) */
    if (ext) {
        if (ext->p_est < ucp_kalman_converged_p_est_val &&          /* Kalman converged + low jitter */
            ext->jitter_ewma < UCP_TSO_LOW_JITTER_THRESH_US) {                                         /* jitter < 1ms: halve divisor */
            div = max_t(u32, 2, div >> 1);                                    /* 8 → 4: larger TSO bursts (div/2) */
        }
        else if (ext->jitter_ewma > UCP_TSO_HIGH_JITTER_THRESH_US) {                                    /* jitter > 4ms: double divisor */
            div = min_t(u32, 32, div << 1);                                   /* 8 → 16: smaller TSO bursts (div*2) */
        }
    }
    tso_rate_thresh = max_t(u32, 1, ucp_min_tso_rate_val / div);               /* compute threshold */
    return READ_ONCE(sk->sk_pacing_rate) < tso_rate_thresh
        ? (u32)ucp_tso_segs_low_val
        : (u32)ucp_tso_segs_default_val;
}
/*
 * ucp_tso_segs_goal - Target number of TSO segments for GSO skb creation.
 * @sk: TCP socket.
 *
 * Formula (BBR standard, Cardwell et al. 2016):
 *   1. bytes = min(pacing_rate >> pacing_shift, GSO_MAX_SIZE - 1 - MAX_TCP_HEADER)
 *   2. segs  = max(bytes / mss_cache, min_tso_segs)
 *   3. segs  = min(segs, tso_max_segs)
 *
 * pacing_rate >> pacing_shift converts the byte-per-second rate into the
 * byte budget for one pacing interval (approx 1 ms).
 */
static u32 ucp_tso_segs_goal(struct sock* sk)                                 /* compute TSO segment target */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    u32 bytes, segs;                                                         /* intermediate bytes and segs */

    bytes = min_t(unsigned long,                                             /* compute byte budget per pacing interval */
        READ_ONCE(sk->sk_pacing_rate) >> READ_ONCE(sk->sk_pacing_shift),           /* rate -> bytes per interval */
        GSO_MAX_SIZE - 1 - MAX_TCP_HEADER);                             /* cap at GSO max minus headers */
    if (unlikely(!tp->mss_cache)) {
        return ucp_min_tso_segs(sk);
    }

    segs = max_t(u32, bytes / tp->mss_cache, ucp_min_tso_segs(sk));           /* convert to segments, floor at min */
    return min_t(u32, segs, ucp_tso_max_segs_val);                  /* cap at configured max TSO segs */
}
/* ---- CWND Save/Restore ----------------------------------------------- */

/*
 * ucp_save_cwnd - Save the current cwnd for later restoration.
 * @sk: TCP socket.
 *
 * BBR logic (Cardwell et al. 2016): when entering recovery or PROBE_RTT,
 * record cwnd so it can be restored afterward.  If already in a recovery
 * state, keep the maximum of prior_cwnd and current cwnd (since recovery
 * may have already reduced cwnd, we want to restore to the pre-recovery peak).
 */
static void ucp_save_cwnd(struct sock* sk)                                    /* save cwnd for later restore */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */

    if (ucp->prev_ca_state < TCP_CA_Recovery && ucp->mode != UCP_PROBE_RTT) { /* not already in recovery/ProbeRTT */
        ucp->prior_cwnd = tcp_snd_cwnd(tp);                                       /* first entry to recovery/ProbeRTT: save cwnd */
    }
    else {                                                                 /* already in recovery/ProbeRTT */
        ucp->prior_cwnd = max_t(u32, ucp->prior_cwnd, tcp_snd_cwnd(tp));          /* keep max of saved and current */
    }
}
/*
 * ucp_cwnd_event - Handle TCP CA events.
 * @sk:    TCP socket.
 * @event: congestion event type (e.g., CA_EVENT_TX_START).
 *
 * On TX_START when app_limited (connection was idle):
 *   - Sets idle_restart = 1 (triggers exponential cwnd ramp).
 *   - Resets ACK aggregation epoch.
 *   - In PROBE_BW: resets pacing to 1.0x of current bw estimate.
 *   - In PROBE_RTT: checks if probe can end (pipe already drained by idle).
 */
UCP_KFUNC void ucp_cwnd_event(struct sock* sk, enum tcp_ca_event event) /* handle CA event */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */

    if (event == CA_EVENT_TX_START && tp->app_limited) {                       /* idle restart on TX_START */
        struct ucp_ext* ext = ucp_ext_get(sk);                               /* retrieve ext (with UAF guard) */

        ucp->idle_restart = 1;                                               /* set idle restart flag */
        if (ext) {                                                            /* ext available */
            ext->ack_epoch_mstamp = tp->tcp_mstamp;                           /* reset agg time base */
            ext->ack_epoch_acked = 0;                                         /* reset agg byte count */
        }
        /* BBR rule: reset pacing rate to 1.0x of current bw estimate on idle
         * restart.  This prevents the connection from bursting at the old
         * PROBE_BW phase's gain (e.g., 1.25x probes would overshoot into an
         * idle pipe, 0.75x drain would under-utilize it).  BBR does this via
         * bbr_set_pacing_rate(sk, bbr_bw(sk), BBR_UNIT). */
        if (ucp->mode == UCP_PROBE_BW) {
            ucp_set_pacing_rate(sk, ucp_bw(sk), BBR_UNIT);
        }
        /* In PROBE_RTT, the idle period naturally drained the pipe.
         * Check if we can exit PROBE_RTT early to avoid a redundant wait. */
        if (ucp->mode == UCP_PROBE_RTT) {
            ucp_check_probe_rtt_done(sk);
        }
    }
}

/*
 * ucp_get_model_rtt - Return the RTT estimate used for BDP calculation.
 * @sk:  TCP socket.
 * @ext: extended state (may be NULL if allocation failed).
 *
 * Priority:
 *   1. If Kalman filter has converged (x_est valid + sample_cnt >= min_samples)
 *      (Kalman 1960): return max(x_est / kalman_scale, min_rtt_us).
 *      The max prevents a Kalman under-estimate from shrinking BDP artificially.
 *   2. Otherwise: return the sliding-window min_rtt_us.
 */
static u32 ucp_get_model_rtt(const struct sock* sk,                            /* get model RTT for BDP */
    const struct ucp_ext* ext)                                   /* extended state (may be NULL) */
{
    const struct ucp* ucp = (const struct ucp*)inet_csk_ca(sk);              /* get const UCP CA state */

    /*
     * When alone on path: use min_rtt_us directly, matching BBR's pure
     * minimum.  The Kalman smoothed estimate has a small positive bias
     * from one-sided measurement noise (queues only add, never subtract),
     * which inflates BDP and causes deeper queues in single-flow scenarios.
     */
    if (ucp->alone_on_path) {
        return ucp->min_rtt_us;                                              /* BBR-style exact minimum */
    }

    if (unlikely(!ext || !ext->x_est || ext->sample_cnt < ucp_kalman_min_samples_val)) { /* Kalman not converged */
        return ucp->min_rtt_us;                                              /* fall back to window min_rtt */
    }

    {
        u32 x_est_us = ext->x_est >> ucp_kalman_scale_shift_val;           /* descale Kalman estimate to µs */
        u32 cap_us = ucp->min_rtt_us;                                    /* default: no margin (margin == 0) */
        u32 model_rtt = max_t(u32, x_est_us, ucp->min_rtt_us);           /* floor at min_rtt_us */
        if (ucp_kalman_xest_margin_pct > 0) {
            u64 cap64 = (u64)ucp->min_rtt_us * (u32)(UCP_PCT_BASE + ucp_kalman_xest_margin_pct) / UCP_PCT_BASE;
            cap_us = (u32)min_t(u64, cap64, U32_MAX);
        }
        return min_t(u32, model_rtt, cap_us);                            /* cap at min_rtt * (100+margin)/100 */
    }
}

/* ---- BDP Calculation (Cardwell et al. 2016) ------------------------- */

/*
 * ucp_bdp - Compute the bandwidth-delay product in segments.
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT (segments * (1<<24) per usec).
 * @gain: cwnd gain in BBR_UNIT (256 = 1.0x).
 * @ext:  extended state (for Kalman RTT).
 *
 * Formula (standard BBR BDP calculation with ceiling):
 *   w       = bw * model_rtt_us
 *   bdp_raw = (w * gain) >> BBR_SCALE
 *   bdp_seg = ceil(bdp_raw / BW_UNIT) = (bdp_raw + BW_UNIT - 1) >> BW_SCALE
 *
 * Returns TCP_INIT_CWND (approx 10) if min_rtt_us is invalid or below floor.
 */
static u32 ucp_bdp(struct sock* sk, u32 bw, u32 gain,                        /* compute BDP in segments */
    struct ucp_ext* ext)                                              /* extended state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */
    u32 model_rtt;                                                           /* chosen RTT estimate */
    u64 w;                                                                   /* intermediate product */
    u64 bdp64;                                                               /* ceiling-safe BDP in BW_UNIT scale */

    /* Get effective RTT estimate first — this way the floor check
     * sees the RTT that will actually be used in the BDP calculation.
     * This is critical: after Kalman takeover, min_rtt_us holds the
     * Kalman estimate which may be below ucp_bdp_min_rtt_us_val on
     * low-latency paths (e.g., 200us < 1000us default floor).
     * The model RTT is already floored at min_rtt_us, so it is a
     * superset of our knowledge. */
    model_rtt = ucp_get_model_rtt(sk, ext);

    /* Floor check: only applies when Kalman has NOT taken over.
     * The Kalman filter provides a converged estimate that is trusted
     * below the traditional floor.  The traditional window-based
     * min_rtt may not have found the true minimum yet, so a floor
     * prevents under-estimation. */
    if (unlikely(ucp->min_rtt_us == UCP_MIN_RTT_UNINIT)) {
        return TCP_INIT_CWND;
    }

    /* Traditional floor: only for non-Kalman paths.
     * Instead of bailing out with TCP_INIT_CWND, floor the model_rtt to the
     * configured minimum.  Returning TCP_INIT_CWND starves cwnd growth during
     * STARTUP on low-latency paths where the windowed min_rtt sits below the
     * configured floor. */
    {
        u32 bdp_floor = ucp_bdp_min_rtt_us_val;                             /* single volatile read for both condition and body */

        if (unlikely(!(ext && ext->x_est &&
            ext->sample_cnt >= ucp_kalman_min_samples_val) &&
            model_rtt < bdp_floor)) {
            model_rtt = bdp_floor;                                      /* cached floor value */
        }
    }

    /* w = bw (seg*BW_UNIT/usec) * rtt_us (us) -> intermediate segments */
    w = (u64)bw * model_rtt;                                                 /* bandwidth-delay product intermediate */
    /* Match BBR: (w * gain >> BBR_SCALE + BW_UNIT - 1) / BW_UNIT */
    bdp64 = mul_u64_u32_shr(w, gain, BBR_SCALE);
    bdp64 += BW_UNIT - 1;
    return (u32)(bdp64 >> BW_SCALE);
}
/*
 * ucp_quantization_budget - Add headroom for TSO/GSO bursts, delayed ACK,
 * and probing bonuses (Cardwell et al. 2016).
 * @sk:   TCP socket.
 * @cwnd: base cwnd in segments.
 *
 * Headroom breakdown (BBR standard):
 *   1. +3 * tso_segs_goal: TSO/GSO burst accommodation.
 *   2. Round to even: accommodate standard delayed-ACK factor of 2.
 *   3. +probe_cwnd_bonus during PROBE_BW phase 0: extra headroom for
 *      the highest-gain probe phase to push past the sliding-window max.
 *   4. Clamp to snd_cwnd_clamp (socket-level upper bound).
 */
static u32 ucp_quantization_budget(struct sock* sk, u32 cwnd)                 /* add headroom to cwnd target */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */

    cwnd += ucp_tso_headroom_mult_val * ucp_tso_segs_goal(sk);   /* TSO/GSO burst headroom */
    cwnd = (cwnd + 1) & ~1U;                                                 /* round to even for delayed-ACK */
    if (ucp->mode == UCP_PROBE_BW && ucp->cycle_idx == 0) {                  /* highest-gain probe phase */
        cwnd += ucp_probe_cwnd_bonus_val;                         /* add extra probe cwnd bonus */
    }
    cwnd = min_t(u32, cwnd, tp->snd_cwnd_clamp);                             /* clamp to socket max */
    return cwnd;                                                             /* return budgeted cwnd */
}
/* ---- ECN (Explicit Congestion Notification) ---------------------------- */

/*
 * ucp_update_ecn_ewma - Update the EWMA of the ECN-CE mark ratio.
 * @sk:  TCP socket.
 * @rs:  rate sample from this ACK.
 * @ext: extended state (ecn_ewma, last_delivered_ce).
 *
 * Reads tp->delivered_ce from the TCP stack (RFC 3168, cumulative count
 * of CE-marked segments delivered to the receiver).  Computes the delta
 * since the last update and converts to a ratio scaled to BBR_UNIT.
 *
 * EWMA: ecn_ewma = (ecn_ewma * retained + instant) / total.
 * Default 3/4 -> 75% old, 25% new weight.
 *
 * On round boundaries with no new CE marks, a strong decay at the EWMA rate
 * is applied; on non-round ACKs, a gentle per-ACK idle decay prevents
 * ecn_ewma from persisting indefinitely on steady connections.
 */
static void ucp_update_ecn_ewma(struct sock* sk, const struct rate_sample* rs, /* update ECN-CE EWMA (function signature split) */
    struct ucp_ext* ext)                            /* parameter: extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                            /* TCP socket */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                             /* UCP CA state */
    u32 ce_delta, instant = 0;                                        /* CE delta, instantaneous ratio */
    u32 cur_ce;                                                       /* snapshot of cumulative delivered_ce counter */
    u64 total_u64;                                                    /* total pkts in interval (delivered + losses) */

    if (!ext || !ucp_ecn_enable_val) {                                /* ECN disabled or no ext */
        return;                                                                   /* no-op */
    }

    cur_ce = tp->delivered_ce;                                                     /* read cumulative CE counter (RFC 3168) */
    if (rs->delivered == 0) {
        return;
    }

    total_u64 = (u64)rs->delivered + rs->losses;
    if (total_u64 == 0) {  /* defensive: unreachable when delivered>0 but guards against integer wrap */
        return;
    }

    ce_delta = cur_ce - ext->last_delivered_ce;                                    /* CE delta: unsigned wrap well-defined modulo 2^32 */
    ext->last_delivered_ce = cur_ce;                                               /* save for next delta computation */

    if (ce_delta > 0) {                                                            /* CE marks in this interval */
        /* instant = ce_delta * BBR_UNIT / total (BBR_UNIT = 256 = 100%) */
        u64 inst64 = ((u64)ce_delta * BBR_UNIT) / total_u64;
        instant = (u32)min_t(u64, inst64, BBR_UNIT);
        if (ext->ecn_ewma == 0) {                                                   /* first CE sample */
            ext->ecn_ewma = min_t(u32, instant, BBR_UNIT);                           /* initialize directly */
        }
        else {                                                                     /* EWMA update */
            u32 v = (ext->ecn_ewma * ucp_ecn_ewma_retained_val + instant) / /* old * retained + new */
                ucp_ecn_ewma_total_val;                                   /* divided by total */
            ext->ecn_ewma = min_t(u32, v, BBR_UNIT);                                    /* clamp to max */
        }
    }
    else {                                                                       /* no CE marks in this interval */
        if (ext->ecn_ewma > 0) {                                                    /* non-zero EWMA */
            if (ucp->round_start) {                                                  /* round boundary: strong decay */
                /* Fast decay at round boundaries: ecn_ewma *= retained / total.
                 * Default 3/4 -> 25% reduction per round. */
                ext->ecn_ewma = ext->ecn_ewma * ucp_ecn_ewma_retained_val / /* decay by retained/total */
                    ucp_ecn_ewma_total_val;                         /* gradual reduction */
            }
            else {                                                                 /* non-round-boundary: gentle per-ACK decay */
                /* Slow per-ACK decay to prevent ecn_ewma from persisting
                 * indefinitely on steady connections with infrequent
                 * round boundaries.  Default 31/32 -> ~3.2% per ACK,
                 * halving in ~2 RTTs at 10 ACKs/RTT. */
                ext->ecn_ewma = (u32)((u64)ext->ecn_ewma *                                /* numerator */
                    ucp_ecn_idle_decay_num_val /                                  /* * idle decay num */
                    (u64)ucp_ecn_idle_decay_den_val);                               /* / idle decay den */
            }
        }
    }
}
/*
 * ucp_ecn_backoff - Reduce cwnd_gain on ECN congestion signal.
 * @sk:  TCP socket.
 * @ext: extended state (ecn_ewma, qdelay_avg).
 *
 * Activation conditions (all must be true):
 *   1. ucp_ecn_enable != 0.
 *   2. ext valid and Kalman filter converged (p_est < converged_p_est,
 *      sample_cnt >= min_samples).
 *   3. ecn_ewma > 0 (CE marks have been observed).
 *   4. qdelay_avg > ucp_ecn_qdelay_thresh_us (queue buildup confirms
 *       congestion).
 *   5. Not in PROBE_BW mode (cwnd_gain remains at 2x matching BBR).
 *   6. During probing, backoff is graduated (scaled to BBR_UNIT/gain)
 *      rather than fully suppressed — severe ECN marks can still
 *      partially reduce cwnd during probe phases.
 *
 * When triggered, cwnd_gain is reduced by the configured backoff factor.
 * BBR has no ECN backoff; UCP adds this as an intelligent response to
 * confirmed congestion signals.
 *
 * The reduction is proportional to the configured backoff percentage
 * (default 20%).  This drains the queue earlier than loss-based
 * congestion signals, improving P99 latency on ECN-enabled paths.
 */
static void ucp_ecn_backoff(struct sock* sk, struct ucp_ext* ext)                /* ECN-aware proactive backoff */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                             /* UCP CA state */
    u32 ecn_backoff, factor;                                           /* backoff fraction and remaining scaling factor */

    /* When alone on path and ECN bypass is enabled (default),
     * skip ECN backoff — ECN marks on a single-flow path are
     * false positives from an over-sensitive AQM.  Bypassing
     * matches BBR's zero-ECN behavior and recovers the throughput
     * gap in single-flow scenarios.
     * When ucp_alone_bypass_ecn = 0, ECN backoff remains active
     * even when alone (conservative mode). */
    if (ucp->alone_on_path && ucp_alone_bypass_ecn_val) {
        return;
    }

    if (!ucp_ecn_enable_val || !ext) {                                 /* ECN disabled or no ext */
        return;                                                                     /* no-op */
    }

    if (ext->p_est >= ucp_kalman_converged_p_est_val) {               /* filter not converged */
        return;                                                                     /* wait for convergence */
    }

    if (ext->sample_cnt < (u32)ucp_kalman_min_samples_val) {               /* insufficient samples */
        return;                                                                     /* wait for min samples */
    }

    if (ext->ecn_ewma == 0) {                                                      /* no CE marks observed */
        return;                                                                     /* nothing to react to */
    }

    /* ECN backoff: reduces cwnd_gain when queue is confirmed (qdelay > threshold).
     * Suppressed during PROBE_BW entirely — cwnd_gain stays at 2x matching BBR.
     * During probing (pacing_gain > BBR_UNIT), backoff is graduated: the
     * suppression scales from 0% (at BBR_UNIT) to 80% (at 5/4 probe) to
     * ~65% (at 2.89x high_gain).  Severe ECN marks can still partially
     * reduce cwnd during probing — binary suppression was too conservative
     * and ignored real congestion signals during probe phases. */
    ecn_backoff = ucp_ecn_backoff_val;                                     /* single volatile read hoisted above branch */
    if (!ecn_backoff) {                                                            /* backoff disabled (zero) */
        return;
    }

    if (ucp->pacing_gain > BBR_UNIT) {                                             /* probing: graduated suppression */
        u32 ecn_scale = BBR_UNIT * BBR_UNIT / ucp->pacing_gain;                    /* BBR_UNIT^2 / gain: 1.0x @ cruise, 0.8x @ 1.25x, 0.35x @ 2.89x */
        ecn_backoff = ecn_backoff * ecn_scale / BBR_UNIT;                          /* scale backoff by probe factor */
    }

    /* factor = BBR_UNIT - min(ecn_backoff, BBR_UNIT-1) */
    factor = BBR_UNIT - min_t(u32, ecn_backoff, BBR_UNIT - UCP_CEIL_ADDEND);                    /* remaining gain factor */

    /* cwnd_gain: only reduce when queue is confirmed (qdelay > threshold)
     * AND we are NOT in PROBE_BW.  BBR never reduces cwnd_gain in PROBE_BW
     * — it is fixed at 2x.  During PROBE_BW, ECN signals affect pacing_gain
     * (via the probing-suppression gate above) but never the inflight ceiling. */
    if (ucp->mode != UCP_PROBE_BW &&
        ext->qdelay_avg > (u32)ucp_ecn_qdelay_thresh_us_val) {
        ucp->cwnd_gain = min_t(u32, ucp->cwnd_gain,
            max_t(u32, 1U,
                ucp->cwnd_gain * factor / BBR_UNIT));
    }
}
/* ---- PROBE_BW Cycle Phase --------------------------------------------- */

/*
 * ucp_get_cycle_pacing_gain - Return the pacing gain for the current PROBE_BW
 * cycle phase, after applying optional queuing-delay/jitter decay.
 * @sk:  TCP socket.
 * @ext: extended state (provides qdelay_avg and jitter_ewma).
 *
 * Algorithm:
 *   1. Base gain = ucp_cycle_gain_table[cycle_idx & (cycle_len − 1)].
 *   2. If decay is enabled for this phase (bit in ucp_cycle_decay_mask set)
 *      AND base_gain > 1.0x (BBR_UNIT) AND ext is valid:
 *      a. Qdelay reduction (Cardwell et al. 2016):
 *         r_q = (qdelay_avg - qdelay_thresh) * BBR_UNIT / qdelay_scale
 *         base_gain -= min(r_q, max_red).
 *      b. Jitter reduction (additive):
 *         r_j = (jitter_ewma - jitter_thresh) * BBR_UNIT / jitter_scale
 *         base_gain -= min(r_j, max_red).
 *   3. Floor at BBR_UNIT (1.0x): the gain never drops below cruise level.
 *
 * The linear reduction model is:
 *   gain = max(1.0, probe_gain - alpha*(qdelay-T_q) - beta*(jitter-T_j))
 * where alpha = BBR_UNIT / qdelay_scale, beta = BBR_UNIT / jitter_scale.
 */
static u32 ucp_get_cycle_pacing_gain(const struct sock* sk,                   /* get decay-adjusted cycle gain */
    struct ucp_ext* ext)                                    /* extended state */
{
    const struct ucp* ucp = (const struct ucp*)inet_csk_ca(sk);              /* get const UCP CA state */
    u32 idx = (u32)ucp->cycle_idx & (ucp_probe_bw_cycle_len_val - 1);     /* len is power-of-two, mask is equivalent to % */
    u32 base_gain = ucp_cycle_gain_table[idx];                     /* read base gain for this phase */

    /*
     * Gain decay via queuing delay and jitter.
     * If ext is NULL (e.g., allocation failed at init), decay is skipped.
     * This is benign because the first real ACK processed after ext
     * becomes available will apply decay on the next phase advance.
     *
     * IMPORTANT: Floor decayed probing gains at BBR_UNIT (1.0x) so that
     * decay never pushes a probe below cruise.  Drain phases (gain < 1.0x
     * by design, e.g. 0.75x BBR_UNIT) are NOT decayed (the decay guard
     * `base_gain > BBR_UNIT` prevents entry for drain phases), so they
     * pass through unchanged at their native sub-1.0x value.
     */
    if (ucp_cycle_decay_enabled(idx) && base_gain > BBR_UNIT && ext) {        /* decay enabled + above cruise + ext valid */
        u32 max_red = base_gain - BBR_UNIT;                                   /* maximum reduction budget: down to 1.0x */
        u32 qthresh = (u32)ucp_qdelay_probe_thresh_us_val;              /* read qdelay threshold */
        u32 qscale = (u32)ucp_qdelay_probe_scale_us_val;                /* read qdelay scale divisor */
        u32 conv = (u32)ucp_kalman_converged_p_est_val;           /* cache converged threshold (single read for consistency) */

        /* Qdelay reduction: linearly proportional to excess qdelay.
         * Scale by Kalman confidence: when p_est > converged, the filter
         * is uncertain and qdelay_avg may be noise — reduce decay impact
         * proportionally.  When converged, full decay applies. */
        u32 conf_scale = BBR_UNIT;
        if (ext->p_est > conv) {
            u32 p_max = ucp_kalman_p_est_max_val;
            if (p_max > conv) {
                conf_scale = (u32)((u64)BBR_UNIT * (p_max - ext->p_est) / (p_max - conv));
            }
            else {
                conf_scale = 0;
            }
        }
        if (conf_scale > 0 && ext->qdelay_avg > qthresh) {                 /* qdelay exceeds threshold */
            u32 raw = min_t(u32, ((u64)(ext->qdelay_avg - qthresh) * BBR_UNIT) / qscale, max_red); /* qdelay reduction */
            u32 r = raw * conf_scale / BBR_UNIT;                               /* scale by confidence */
            base_gain -= r; max_red -= r;                                     /* apply qdelay reduction, update budget */
        }
        /* Jitter reduction: any remaining max_red budget, also confidence-scaled */
        if (max_red > 0 && conf_scale > 0) {                                    /* remaining reduction budget */
            u32 jitter = ext->jitter_ewma;                                    /* read jitter EWMA */
            u32 jthresh = (u32)ucp_jitter_probe_thresh_us_val;           /* read jitter threshold */
            u32 jscale = (u32)ucp_jitter_probe_scale_us_val;             /* read jitter scale divisor */
            if (jitter > jthresh) {                                      /* jitter exceeds threshold */
                u32 jr = min_t(u32, ((u64)(jitter - jthresh) * BBR_UNIT) / jscale, max_red); /* jitter reduction */
                u32 jr_scaled = jr * conf_scale / BBR_UNIT;                  /* scale by confidence */
                base_gain -= jr_scaled;                                         /* apply jitter reduction */
            }
        }
        /* Floor: decay should never push a probe below 1.0x cruise level.
         * Deliberate drain phases (base_gain < BBR_UNIT) never enter this
         * block (guard: base_gain > BBR_UNIT), so they pass through below. */
        base_gain = max_t(u32, base_gain, BBR_UNIT);
    }
    return base_gain;                                                         /* return gain (may be < BBR_UNIT for deliberate drain phases) */
}
/*
 * ucp_advance_cycle_phase - Transition to the next PROBE_BW cycle phase
 * (Cardwell et al. 2016).
 * @sk:  TCP socket.
 * @ext: extended state (for decay computation).
 *
 * Increments cycle_idx (wraps via mask), records the phase-start timestamp,
 * and updates pacing_gain (with decay if applicable).
 */
static void ucp_advance_cycle_phase(struct sock* sk)                           /* advance to next cycle phase */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                          /* get UCP CA state */

    /* Wrap cycle_idx: (idx + 1) % cycle_len, using mask since len is power-of-two */
    ucp->cycle_idx = (ucp->cycle_idx + 1) & (ucp_probe_bw_cycle_len_val - 1); /* advance + wrap */
    ucp_set_cycle_mstamp(ucp, tp->delivered_mstamp);                          /* mark phase start time */

    /* pacing_gain is set by the mode-specific block in ucp_update_model()
     * after all phase-advance and mode-transition logic completes.
     * Setting it here is redundant (always overwritten by the caller)
     * and wastes hot-path cycles on every PROBE_BW phase advance. */
}
/*
 * ucp_update_dyn_probe_interval - Recompute the dynamic PROBE_RTT interval
 * based on Kalman error covariance p_est (Kalman 1960).
 * @ext: extended state (modified in-place).
 *
 * Mapping function (linear interpolation):
 *
 *   p_est <= p_est_floor                        -> interval = 2.5x dyn_max (75s, hyper-converged)
 *   p_est_floor < p_est <= converged_p_est      -> linear from 2.5x → 1.0x dyn_max
 *   converged_p_est < p_est < band*conv          -> linear from dyn_max → base
 *   p_est >= band * converged_p_est              -> interval = base (conservative)
 *
 *   interval = base + (max_jif - base) * (4*conv - p_est) / (3*conv)
 *
 * Rationale: when the Kalman filter has high confidence (low p_est),
 * the propagation delay estimate is stable, and PROBE_RTT can be
 * performed less frequently — reducing periodic throughput drops.
 * At extreme confidence (p_est near floor), the interval extends to
 * 2.5x dyn_max (75s), further reducing the performance penalty of
 * periodic min_rtt probing.  When p_est is high (low confidence), the
 * interval reverts to the base (conservative) value.
 */
static void ucp_update_dyn_probe_interval(struct ucp_ext* ext)                 /* recompute dynamic PROBE_RTT interval */
{
    u32 base = ucp_probe_rtt_base_jiffies;                         /* read base interval (jiffies) */
    u32 max_jif = ucp_probe_rtt_dyn_max_jiffies;                   /* read max dynamic interval (jiffies) */

    if (max_jif == 0 || !ext) {                            /* dynamic disabled OR no ext */
        return;                                                               /* skip: nothing to compute */
    }

    /* Guard: if dyn_max <= base, the linear interpolation produces nonsense.
     * Clamp dyn_max to at least base+1 to guarantee valid interpolation range. */
    if (max_jif <= base) { max_jif = (base < U32_MAX) ? base + 1 : base; }                              /* prevent underflow in (max - base) */
    {
        u32 conv = ucp_kalman_converged_p_est_val;                  /* convergence threshold p_est */
        u32 high = (u32)((u64)ucp_kalman_probe_band_mult_val * conv); /* u64 intermediate prevents overflow */
        u32 p = ext->p_est;                                                   /* current p_est value */
        u32 interval;                                                         /* computed dynamic interval */

        if (p <= conv) {                                                      /* fully converged: high confidence */
            u32 p_floor = ucp_kalman_p_est_floor_val;              /* Kalman floor (near-certainty) */
            if (p <= p_floor && p_floor < conv) {                              /* hyper-converged: extend to 2.5x */
                interval = max_jif + (u32)((u64)max_jif * UCP_DYN_PROBE_HYPER_NUM / UCP_DYN_PROBE_HYPER_DEN);             /* 30s → 75s at extreme confidence */
            }
            else if (p_floor < conv) {                                       /* converged but not floor */
                interval = max_jif + (u32)(((u64)max_jif * UCP_DYN_PROBE_HYPER_NUM / UCP_DYN_PROBE_HYPER_DEN) *
                    (conv - p) / (conv - p_floor));                              /* linear from 2.5x → 1x */
            }
            else {
                interval = max_jif;                                              /* use maximum interval */
            }
        }
        else if (p >= high) {                                               /* low confidence: at or above upper band */
            interval = base;                                                  /* use base (conservative) interval */
        }
        else {                                                              /* in transition band: linear interp */
            /* Linear interpolation: closer to conv -> closer to max_jif */
            if (high <= conv) {
                high = conv + 1;
            }

            interval = base + (u32)((u64)(max_jif - base) *                    /* base + (max-base)*(high-p)/(high-conv) */
                (high - p) / (high - conv));                           /* linear interpolation ratio */
        }

        ext->dyn_probe_rtt_interval_jiffies = interval;                       /* store dynamic interval in ext */
    }
}
/*
 * ucp_get_probe_rtt_interval - Determine the current PROBE_RTT interval
 * in jiffies, preferring the dynamic interval from the Kalman filter
 * when available (Kalman 1960).
 * @sk: TCP socket.
 *
 * Priority:
 *   1. If Kalman-converged && dyn_probe_rtt_interval_jiffies > 0:
 *      use dynamic interval (set by ucp_update_dyn_probe_interval).
 *   2. Fallback (classic BBRv1, Cardwell et al. 2016):
 *      base_interval = ucp_probe_rtt_base_jiffies.
 *      If min_rtt_us > long_rtt_threshold, halve (long paths need more
 *      frequent min_rtt re-verification).
 *      Cap at ucp_probe_rtt_max_jiffies.
 */
static u32 ucp_get_probe_rtt_interval(const struct sock* sk,                    /* get effective PROBE_RTT interval */
    struct ucp_ext* ext)                                                  /* ext from caller (avoids redundant ucp_ext_get) */
{
    const struct ucp* ucp = (const struct ucp*)inet_csk_ca(sk);               /* get const UCP CA state */

    /*
     * Dynamic interval: Kalman-converged -> wider probe gap.
     * Requires ext, valid x_est, sufficient sample count, and
     * a non-zero dynamic interval.
     */
    if (ext && ext->x_est &&
        ext->sample_cnt >= ucp_kalman_min_samples_val &&
        ext->dyn_probe_rtt_interval_jiffies > 0) {

        return ext->dyn_probe_rtt_interval_jiffies;
    }

    /* Classic BBRv1 fallback (Cardwell et al. 2016) */
    {
        u64 interval = ucp_probe_rtt_base_jiffies;                  /* start with base interval */

        /* BBRv1: long-RTT paths probe more frequently — interval
         * divided by the configured divisor (default 1 = no scaling
         * to match BBR, div=2 emulates BBRv1 halving) */
        if (ucp->min_rtt_us > (u32)ucp_probe_rtt_long_rtt_us_val) { /* RTT exceeds long-RTT threshold */
            interval = max_t(u64, interval / ucp_probe_rtt_long_interval_div_val, 1);       /* shrink by configured divisor */
        }

        return (u32)min_t(u64, interval, ucp_probe_rtt_max_jiffies); /* cap at max interval */
    }
}

/* ---- CWND Constraints ------------------------------------------------- */

/*
 * ucp_apply_cwnd_constraints - Apply cwnd_gain constraints.
 * Applies ECN-aware backoff (ucp_ecn_backoff) when ECN-CE marks
 * coincide with elevated queuing delay.  This is the only runtime
 * cwnd_gain reduction mechanism — BBR's 2x cwnd_gain in PROBE_BW
 * is proven optimal and is preserved in all non-ECN paths.
 * @sk:  TCP socket.
 * @ext: extended state (for ECN EWMA).
 */
static void ucp_apply_cwnd_constraints(struct sock* sk,                        /* apply cwnd_gain caps */
    struct ucp_ext* ext)                                   /* extended state */
{
    /* BBR maintains full 2.89x throughout STARTUP with no loss-based
     * reduction.  UCP matches this — the full_bw detector already
     * handles the STARTUP->DRAIN transition when the pipe is full
     * (3 rounds without 25% bw growth).  Loss events naturally
     * trigger LT BW sampling and eventual mode transitions.
     *
     * In PROBE_BW mode, reducing cwnd_gain below 2x is unsafe: inflight
     * shrinks after each probe cycle, creating a bandwidth floor that
     * prevents the next 1.25x probe from discovering more bandwidth.
     * BBR always uses 2x in PROBE_BW for this reason (Cardwell et al.
     * 2016).  ECN backoff (ucp_ecn_backoff) provides congestion-responsive
     * cwnd reduction that is compatible with the gain cycle.
     */

     /* ECN-aware backoff: reduces cwnd_gain when ECN-CE marks coincide
      * with elevated queuing delay.
      */
    ucp_ecn_backoff(sk, ext);                                                       /* ECN proactive backoff (RFC 3168) */
}
/*
 * ucp_inflight - Compute the target inflight in segments for the given
 * bandwidth, gain, and model RTT (Cardwell et al. 2016).
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT.
 * @gain: cwnd gain in BBR_UNIT.
 * @ext:  extended state (for Kalman RTT).
 *
 * inflight = ucp_quantization_budget(ucp_bdp(bw, gain, ext)).
 */
static u32 ucp_inflight(struct sock* sk, u32 bw, u32 gain,                      /* compute target inflight */
    struct ucp_ext* ext)                                                /* extended state */
{
    return ucp_quantization_budget(sk, ucp_bdp(sk, bw, gain, ext));              /* BDP + quantization headroom */
}
/*
 * ucp_packets_in_net_at_edt - Estimate inflight at the earliest departure time
 * (Cardwell et al. 2016).
 * @sk:           TCP socket.
 * @inflight_now: current tcp_packets_in_flight().
 *
 * Formula: delivered_at_edt = bw * (edt - now) in us >> BW_SCALE.
 *          inflight_at_edt = inflight_now + (pacing_gain > 1x ? tso_segs_goal : 0)
 *          return max(0, inflight_at_edt - delivered_at_edt).
 *
 * This is the BBR "is the pipe still full?" check, used for deciding
 * when to advance to the next PROBE_BW cycle phase.  When pacing_gain > 1x
 * (probing up), we add one TSO burst to estimate inflight at edt's send time.
 *
 * @bw: bandwidth estimate in BW_UNIT for computing delivered-at-EDT.
 *
 * If EDT is within ucp_edt_near_now_ns, treat delivered_at_edt = 0
 * (the pipe won't drain at all before the next send window).
 */
static u32 ucp_packets_in_net_at_edt(struct sock* sk, u32 inflight_now, u32 bw)          /* estimate inflight at EDT */
{
    struct tcp_sock* tp = tcp_sk(sk);                                           /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                             /* get UCP CA state */
    u64 now_ns = tp->tcp_clock_cache;                                            /* current time in ns */
    u64 edt_ns = max_t(u64, tp->tcp_wstamp_ns, now_ns);                          /* earliest departure time, >= now */
    u32 delivered;                                                               /* estimated delivered-at-EDT */
    u32 inflight_at_edt = inflight_now;                                          /* inflight at EDT, start with now */

    /* EDT within "near now" threshold -> pipe hasn't drained at all */
    if (edt_ns <= now_ns || (edt_ns - now_ns) <= (u64)ucp_edt_near_now_ns_val) {                  /* EDT is effectively "now" */
        delivered = 0;                                                            /* nothing drains by EDT */
    }
    else {                                                                      /* EDT is in the future */
        /* delivered = bw * (edt - now) >> BW_SCALE, matching BBR's pattern */
        u64 delta_us = div_u64(edt_ns - now_ns, NSEC_PER_USEC);
        u64 delivered64;
        delivered64 = ((u64)bw * delta_us) >> BW_SCALE;
        delivered = (u32)delivered64;
    }

    /* When probing above 1x gain, add one TSO burst to the estimate */
    if (ucp->pacing_gain > BBR_UNIT) {                                             /* probing up: add TSO burst */
        inflight_at_edt += ucp_tso_segs_goal(sk);                                    /* add one TSO burst worth of segs */
    }

    if (delivered >= inflight_at_edt) {                                            /* pipe will empty by EDT */
        return 0;                                                                  /* return 0 inflight at EDT */
    }

    return inflight_at_edt - delivered;                                            /* remaining inflight at EDT */
}
/* ---- Recovery Entry/Exit ---------------------------------------------- */

/*
 * ucp_set_cwnd_to_recover_or_restore - Handle cwnd adjustments on TCP
 * recovery entry and exit (Cardwell et al. 2016).
 * @sk:       TCP socket.
 * @rs:       rate sample (for losses).
 * @acked:    bytes ACKed.
 * @new_cwnd: [out] computed cwnd value.
 *
 * Returns true if in packet-conservation mode (recovery with cwnd pinned
 * to inflight + acked).
 *
 * On recovery entry:
 *   - Enable packet_conservation flag.
 *   - Set cwnd = inflight + acked (conservative; don't send more than in flight).
 * On recovery exit:
 *   - Restore cwnd to max(current, prior_cwnd).
 * If losses present: subtract losses from cwnd.
 */
static bool ucp_set_cwnd_to_recover_or_restore(                                 /* handle recovery cwnd transitions */
    struct sock* sk, const struct rate_sample* rs, u32 acked, u32* new_cwnd)     /* input params + output cwnd */
{
    struct tcp_sock* tp = tcp_sk(sk);                                             /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                               /* get UCP CA state */
    u8 prev_state = ucp->prev_ca_state, state = inet_csk(sk)->icsk_ca_state;      /* previous and current CA states */
    u32 cwnd = tcp_snd_cwnd(tp);                                                       /* start with current cwnd */

    /* Loss: reduce cwnd by the number of lost segments (floor at 1) */
    if (rs->losses > 0) {                                                           /* losses present */
        if (cwnd > rs->losses) {
            cwnd -= rs->losses;
        }
        else {
            cwnd = 1;
        }
    }

    /* Recovery entry: transition from non-recovery -> recovery */
    if (state == TCP_CA_Recovery && prev_state != TCP_CA_Recovery) {                 /* entering recovery */
        ucp->packet_conservation = 1;                                                 /* enable packet conservation */
        ucp->next_rtt_delivered = tp->delivered;                                      /* start round now (match BBR, Cardwell et al. 2016) */
        cwnd = tcp_packets_in_flight(tp) + acked;                                     /* conservative cwnd = inflight + acked */
    }
    /* Recovery exit: transition from recovery -> non-recovery */
    else if (prev_state >= TCP_CA_Recovery && state < TCP_CA_Recovery) {              /* exiting recovery */
        cwnd = max_t(u32, cwnd, ucp->prior_cwnd);                                      /* restore to at least pre-recovery cwnd */
        ucp->packet_conservation = 0;                                                   /* disable packet conservation */
    }

    /* Update tracked previous CA state only on actual transition
     * to avoid unnecessary cache-line writes on every ACK. */
    if (state != prev_state) {
        ucp->prev_ca_state = state;
    }

    if (ucp->packet_conservation) {                                                       /* in packet conservation mode */
        *new_cwnd = max_t(u32, cwnd, tcp_packets_in_flight(tp) + acked);                  /* cwnd >= inflight + acked */
        return true;                                                                        /* return true: in conservation */
    }

    *new_cwnd = cwnd;                                                                       /* store computed cwnd */
    return false;                                                                             /* return false: not in conservation */
}
/* ---- CWND Setting (Cardwell et al. 2016) ----------------------------- */

/*
 * ucp_set_cwnd - Update the congestion window on each ACK.
 * @sk:    TCP socket.
 * @rs:    rate sample.
 * @acked: segments ACKed (rs->acked_sacked, not bytes).
 * @bw:    bandwidth estimate in BW_UNIT.
 * @gain:  current cwnd_gain in BBR_UNIT.
 * @ext:   extended state.
 *
 * Algorithm (standard BBR per-ACK cwnd update):
 *   1. Skip if no data ACKed (acked_segs == 0).
 *   2. Handle recovery entry/exit via ucp_set_cwnd_to_recover_or_restore().
 *   3. Compute BDP target = quantization_budget(BDP(bw, gain, ext)).
 *   4. Apply inflight bounds:
 *      - STARTUP: lower-bound target at BDP * inflight_low_gain (no upper bound).
 *      - Other modes: clamp target between [lo, hi].
 *   5. Add ACK aggregation bonus.
 *   6. CWND progression:
 *      - full_bw_reached: cwnd = min(cwnd + acked, target)  (convergent).
 *      - STARTUP:         cwnd = cwnd + acked               (exponential probe).
 *   7. Floor at cwnd_min_target.
 *   8. If PROBE_RTT just ended (probe_rtt_restored): restore prior_cwnd.
 *   9. Clamp to snd_cwnd_clamp.
 *  10. In PROBE_RTT mode: enforce cap at cwnd_min_target (min inflight).
 */
static void ucp_set_cwnd(struct sock* sk, const struct rate_sample* rs,             /* update snd_cwnd */
    u32 acked, u32 bw, u32 gain,                                           /* input parameters */
    struct ucp_ext* ext)                                                   /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                  /* get UCP CA state */
    u32 cwnd = tcp_snd_cwnd(tp), target;                                                   /* current cwnd and target */

    /* PROBE_RTT exit: restore cwnd to at least the pre-probe value.
     * Must clear the flag BEFORE early exits (acked==0, recovery) so
     * that the first ACK after PROBE_RTT always gets cwnd restoration
     * and the pacing override in ucp_update_model doesn't persist. */
    if (unlikely(ucp->probe_rtt_restored)) {                                                               /* PROBE_RTT just ended */
        cwnd = max_t(u32, cwnd, ucp->prior_cwnd);                                                           /* restore to pre-probe cwnd */
        ucp->probe_rtt_restored = 0;                                                                          /* clear restore flag */
    }

    if (unlikely(!acked)) {                                                                        /* no data ACKed: uncommon case */
        goto done;                                                                         /* skip to clamp enforcement */
    }

    if (ucp_set_cwnd_to_recover_or_restore(sk, rs, acked, &cwnd)) {                       /* handle recovery transitions */
        goto done;                                                                          /* packet-conservation mode */
    }

    /* Core BDP target (segments) — without quantization budget.
     * Quantization headroom (TSO/even-round/probe) is added AFTER
     * inflight bounds to avoid stripping it (see below). */
    target = ucp_bdp(sk, bw, gain, ext);

    /* Inflight bounds (BBR standard, Cardwell et al. 2016).
     * Apply bounds to the *raw* BDP before adding quantization and
     * ACK-aggregation bonuses.  This matches BBR behavior where
     * bbr_quantization_budget() adds headroom on top of the
     * already-bounded target_cwnd. */
    {
        bool bdp_ready = (ucp->min_rtt_us != UCP_MIN_RTT_UNINIT && bw > 0);   /* min_rtt_us > 0 implied by != UNINIT (UNINIT = ~0U) */
        if (likely(bdp_ready)) {
            u64 bdp = ((u64)bw * ucp->min_rtt_us) >> BW_SCALE;
            u32 lo = max_t(u32, TCP_INIT_CWND,
                mul_u64_u32_shr(bdp, ucp_inflight_low_gain_val, BBR_SCALE));
            if (ucp->mode == UCP_STARTUP) {
                target = max_t(u32, target, lo);
            }
            else {
                u32 hi_gain = max_t(u32, ucp->cwnd_gain, ucp_inflight_high_gain_val);  /* ≥ runtime cwnd_gain for safety */
                u32 hi = max_t(u32, lo,
                    mul_u64_u32_shr(bdp, hi_gain, BBR_SCALE));
                target = clamp(target, lo, hi);
            }
        }
        /* Now add quantization headroom on top of the bounded target.
         * This matches BBR's order: bdp() → bounded → quantization(). */
        target = ucp_quantization_budget(sk, target);
        /* Inflight bounds valid: add ACK aggregation bonuses */
        if (likely(bdp_ready)) {
            target += ucp_ack_aggregation_cwnd(sk, ext, bw);

            /* ACK aggregation compensation: confidence-gated second layer */
            if (ucp_agg_enable_val && ext && ext->agg_state >= UCP_AGG_CONFIRMED) {
                u32 agg_comp = ucp_agg_cwnd_compensation(sk, ext, ext->agg_extra_acked, ext->agg_confidence, bw);
                target = min_t(u32, target + agg_comp, tp->snd_cwnd_clamp);
            }
        }
    }

    /*
     * CWND progression policy:
     * - full_bw_reached: converge to target.
     *   cwnd = min(cwnd + acked, target)
     * - STARTUP (not yet full): exponential ramp.
     *   cwnd = cwnd + acked (unbounded above; inflight_bounds prevent overshoot)
     */
    if (likely(ucp_full_bw_reached(sk))) {                                                              /* pipe full: converge to target */
        cwnd = min_t(u32, cwnd + acked, target);                                                          /* converge upward to target */
    }
    else if (unlikely(cwnd < target || tp->delivered < TCP_INIT_CWND)) {                                        /* STARTUP: growth needed */
        cwnd = cwnd + acked;                                                                               /* exponential ramp */
    }

    cwnd = max_t(u32, cwnd, ucp_cwnd_min_target_val);                                         /* floor at min cwnd */

done:                                                                                                          /* done label: enforce clamp */
    tcp_snd_cwnd_set(tp, min_t(u32, cwnd, tp->snd_cwnd_clamp));                                  /* use setter: maintains snd_cwnd_cnt */

    /* PROBE_RTT enforcement: keep cwnd at minimum to drain the pipe */
    if (unlikely(ucp->mode == UCP_PROBE_RTT)) {                                                                   /* in PROBE_RTT mode */
        tcp_snd_cwnd_set(tp, min_t(u32, tcp_snd_cwnd(tp), ucp_cwnd_min_target_val));               /* cap at min target */
    }
}
/* ---- Cycle Phase Check (Cardwell et al. 2016) ------------------------ */

/*
 * ucp_is_next_cycle_phase - Determine whether the current PROBE_BW phase
 * has completed (should advance to the next phase).
 * @sk:  TCP socket.
 * @rs:  rate sample.
 * @ext: extended state.
 *
 * Decision logic (from BBRv1, adapted for UCP gain decay):
 *   1. Phase must have lasted at least one min_rtt (is_full_length).
 *   2. Additional termination conditions depend on pacing_gain:
 *      - gain > 1.0x (probing up):   exit if loss occurred OR
 *                                     inflight_at_edt >= target_inflight.
 *      - gain < 1.0x (probing down): drain-to-target.
 *        Exit when is_full_length AND inflight_at_edt <=
 *        target at 1x gain, with a safety timeout after
 *        UCP_DRAIN_TARGET_MAX_RTTS (default 4) RTTs.
 *        Prevents premature drain exit that leaves
 *        residual inflight on multi-flow paths.
 *      - gain == 1.0x (cruise):      exit when is_full_length.
 */
static bool ucp_is_next_cycle_phase(struct sock* sk,                              /* check if phase should advance */
    const struct rate_sample* rs,                                 /* rate sample */
    struct ucp_ext* ext)                                          /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                              /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                /* get UCP CA state */
    u32 raw = tcp_stamp_us_delta(tp->delivered_mstamp, ucp_get_cycle_mstamp(ucp));
    u64 delta = raw;                            /* tcp_stamp_us_delta returns u32, promote for comparison */
    bool is_full_length;                                                               /* flag: at least one min_rtt elapsed */

    if (unlikely(ucp->min_rtt_us == UCP_MIN_RTT_UNINIT)) {
        return false;
    }

    /* is_full_length: strictly GREATER than one min_rtt (not >=).
     * BBR uses `>`, not `>=`: the phase must last strictly longer
     * than min_rtt to avoid advancing the cycle on a sample that
     * lands exactly at the min_rtt boundary (Cardwell et al. 2016). */
    is_full_length = delta > (u64)ucp->min_rtt_us;

    if (ucp->pacing_gain > BBR_UNIT) {                                                   /* probing up (> 1x): rare (~1/8 phases) */
        u32 etd_bw = ucp_bw(sk);                                                         /* active BW for delivered-at-EDT (match BBR using bbr_bw internally) */
        u32 max_bw = ucp_max_bw(sk);                                                     /* max BW for inflight target (match BBR: bw = bbr_max_bw(sk)) */
        u32 inet_edt = ucp_packets_in_net_at_edt(sk, rs->prior_in_flight, etd_bw);       /* hoist: single call for both paths */
        /*
         * Multi-condition probe-up exit:
         *
         * PROBE_UP (1.25x pacing) tries to discover untapped
         * bandwidth by raising inflight above BDP.  The
         * original BBRv1 exits when either loss occurs or
         * inflight reaches the 1.25x target.
         *
         * When ucp_probe_up_limit is enabled (default: off),
         * two additional conditions prevent futile probing:
         *
         *   rs->is_app_limited:
         *     The application is not writing fast enough to
         *     saturate the pipe.  Inflating inflight at 1.25x
         *     pacing cannot raise delivery rate beyond what
         *     the app provides — the probe cannot discover
         *     more bandwidth and only inflates the queue.
         *
         *   !tcp_send_head(sk):
         *     The sender has no data ready to transmit (send
         *     queue empty).  Inflight cannot grow regardless
         *     of pacing gain — continuing PROBE_UP serves no
         *     purpose and adds queuing delay.
         *
         * Disabled by default — on high-throughput multi-flow
         * VPS paths the standard BBRv1 probe-up heuristic
         * (loss or inflight target only) performs better.
         */
        return is_full_length &&                                                            /* RTT elapsed AND */
            (rs->losses ||                                                                 /* loss occurred OR */
                inet_edt >=                                                            /* inflight at EDT >= */
                ucp_inflight(sk, max_bw, ucp->pacing_gain, ext) ||                           /* target inflight at max_bw */
                (ucp_probe_bw_up_limit_val &&                                      /* up-phase limit enabled AND */
                    (rs->is_app_limited || !tcp_send_head(sk))));              /* app-limited or no data queued */
    }

    if (ucp->pacing_gain < BBR_UNIT) {                                                           /* probing down (< 1x): rare (~1/8 phases) */
        u32 etd_bw = ucp_bw(sk);                                                         /* active BW for delivered-at-EDT (match BBR) */
        u32 max_bw = ucp_max_bw(sk);                                                     /* max BW for inflight target (match BBR) */
        /* Kalman drain skip: when the filter is converged AND confirms
         * near-zero queue (< drain_skip_qdelay_us), the preceding probe
         * did NOT create standing queue.  Skip the drain phase entirely
         * — convert it to an early cruise phase.  This saves 1/8 cycle
         * of reduced pacing when the path is truly empty. */
        if (ext && ext->p_est < ucp_kalman_converged_p_est_val &&
            ext->qdelay_avg < ucp_drain_skip_qdelay_us_val &&
            delta >(u64)(ucp->min_rtt_us / UCP_DRAIN_SKIP_MIN_RTT_DIV)) {
            return true;
        }
        /*
         * Drain-to-target: fix BBRv1's premature drain exit.
         *
         * BBRv1 drain mechanism:
         *
         *   PROBE_UP (1.25x) ─── creates standing queue ───►
         *   DRAIN (0.75x) ─── empties the queue ───►
         *   CRUISE (1.0x) ─── maintains BDP-level inflight
         *
         * In the original BBRv1 (Cardwell et al. 2016), drain
         * exits when EITHER one min_rtt has elapsed OR inflight
         * drops to BDP — whichever comes first:
         *
         *   return is_full_length || (inet_edt <= BDP_target);
         *
         * This is broken on multi-flow paths:
         *
         *   1. Eight flows share a 1 Gbps bottleneck.
         *   2. During PROBE_UP they collectively overshoot, building
         *      a queue that can exceed 1–2× BDP per flow.
         *   3. DRAIN at 0.75× pacing begins.  After 1 RTT, inflight
         *      has only partially drained — residual queue remains
         *      because the aggregate drain rate (8×0.75=6×) cannot
         *      clear 8×BDP of standing queue in a single RTT.
         *   4. The `is_full_length` branch fires.  Drain exits
         *      prematurely.  Residual inflight carries over into
         *      the next cycle.
         *   5. The next PROBE_UP starts from an already-elevated
         *      inflight baseline — overshoot happens earlier and
         *      harder — loss spikes — CWND collapses — throughput
         *      oscillates 550–1300 Mbps within 1–2 seconds.
         *
         *   Timeline (8-flow, 212 ms RTT, 1 Gbps shared bottleneck):
         *
         *     ┌──────────────────────────────────────────────────────┐
         *     │ PROBE_UP (1.25x)                                      │
         *     │   queue builds: inflight climbs to 2–3× BDP          │
         *     │   aggregate = 8×1.25 = 10× → 25% over line rate      │
         *     ├──────────────────────────────────────────────────────┤
         *     │ DRAIN (0.75x)                                         │
         *     │   t=0.00:  inflight = 2.5× BDP (post-probe peak)    │
         *     │   t=0.21:  inflight ≈ 1.8× BDP (1 RTT elapsed)     │
         *     │            ┌── BBRv1 OR-gate: is_full_length ✓ →     │
         *     │            │   EXITS DRAIN immediately                │
         *     │            │   residual = 0.8× BDP above target       │
         *     ├────────────┴─────────────────────────────────────────┤
         *     │ CRUISE (1.0x)  [residual inflight persists]          │
         *     │   queue never fully cleared                          │
         *     ├──────────────────────────────────────────────────────┤
         *     │ PROBE_UP (1.25x)  [next cycle]                       │
         *     │   starts from 1.8× BDP baseline → immediate loss     │
         *     │   CWND cut → throughput collapse                     │
         *     └──────────────────────────────────────────────────────┘
         *
         * UCP fix — drain-to-target (AND-gate):
         *
         *   return (is_full_length && drained) || safety_timeout;
         *
         *   ┌──────────────────────────────────────────────────────┐
         *   │ DRAIN (0.75x)                                         │
         *   │   t=0.00:  inflight = 2.5× BDP (post-probe peak)    │
         *   │   t=0.21:  inflight ≈ 1.8× BDP (1 RTT elapsed)     │
         *   │            is_full_length ✓  but drained ✗ →          │
         *   │            CONTINUES DRAINING                         │
         *   │   t=0.42:  inflight ≈ 1.3× BDP (2 RTTs)             │
         *   │            drained ✗ → CONTINUES                      │
         *   │   t=0.63:  inflight ≈ 1.0× BDP (3 RTTs)             │
         *   │            drained ✓ → EXITS to CRUISE                │
         *   │            queue genuinely empty                      │
         *   └──────────────────────────────────────────────────────┘
         *
         * Safety timeout (4× min_rtt, ~848 ms at 212 ms RTT):
         * prevents infinite drain on paths where inflight can
         * never reach BDP due to persistent cross-traffic.
         */


        {
            bool drained = ucp_packets_in_net_at_edt(sk, rs->prior_in_flight, etd_bw) <=
                ucp_inflight(sk, max_bw, BBR_UNIT, ext);
            return (is_full_length && drained) ||
                delta > (u64)ucp->min_rtt_us * UCP_DRAIN_TARGET_MAX_RTTS;
        }
    }

    /* Cruise (== 1x): advance after one min_rtt */
    return is_full_length;                                                                                /* single condition: full RTT */
}
/*
 * ucp_update_cycle_phase - Check and advance the PROBE_BW cycle phase.
 * @sk:  TCP socket.
 * @rs:  rate sample.
 * @ext: extended state.
 *
 * Only acts in PROBE_BW mode.  Calls ucp_advance_cycle_phase() when
 * ucp_is_next_cycle_phase() returns true.
 */
static void ucp_update_cycle_phase(struct sock* sk,                                   /* check + advance PROBE_BW phase */
    const struct rate_sample* rs,                                      /* rate sample */
    struct ucp_ext* ext)                                               /* extended state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                    /* get UCP CA state */
    if (likely(ucp->mode == UCP_PROBE_BW) && ucp_is_next_cycle_phase(sk, rs, ext)) {             /* PROBE_BW + time to advance */
        ucp_advance_cycle_phase(sk);                                                  /* advance to next phase */
    }
}
/*
 * ucp_reset_mode - Transition to STARTUP or PROBE_BW after DRAIN completes
 * or after exiting PROBE_RTT (Cardwell et al. 2016).
 * @sk: TCP socket.
 *
 * If full_bw_reached:
 *   -> PROBE_BW, with randomized initial cycle phase (decorrelation via random
 *     offset to prevent phase synchronization across flows sharing a
 *     bottleneck).
 * Else:
 *   -> STARTUP (pipe is not yet fully characterized).
 */
static void ucp_reset_mode(struct sock* sk)                                            /* transition from DRAIN/ProbeRTT */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                     /* get UCP CA state */
    struct ucp_ext* ext = ucp_ext_get(sk);                                                 /* get ext (with UAF guard) */

    if (!ucp_full_bw_reached(sk)) {                                                         /* pipe not yet full */
        ucp->mode = UCP_STARTUP;                                                              /* re-enter STARTUP */
    }
    else {                                                                                    /* pipe full: enter PROBE_BW */
        ucp->mode = UCP_PROBE_BW;                                                                 /* set PROBE_BW mode */
        /* Random start phase: cycle_idx = len - 1 - rand(range).
         * Spreads flows across phases to reduce correlation. */
        if (ext) {
            ucp->cycle_idx = ucp_probe_bw_cycle_len_val - 1 -                               /* start near end */
                ucp_random_below(ucp_probe_bw_cycle_rand_val);                            /* randomized offset */
            /* BBR calls bbr_advance_cycle_phase() after setting cycle_idx,
             * which (a) increments cycle_idx, (b) records cycle_mstamp, and
             * (c) sets pacing_gain.  Match this behavior exactly so the
             * first PROBE_BW phase has a valid timestamp for is_full_length. */
            ucp_advance_cycle_phase(sk);                                                   /* flip to next phase + set mstamp */
        }
        else {
            ucp->cycle_idx = 0;                                                              /* fallback: phase 0 */
            ucp->pacing_gain = BBR_UNIT;                                                       /* cruise at 1.0x */
            ucp->cwnd_gain = ucp_cwnd_gain_val;                                                  /* baseline cwnd gain */
            ucp_set_cycle_mstamp(ucp, tcp_sk(sk)->delivered_mstamp);                             /* seed phase timestamp */
        }
    }
}
/* ---- LT BW (Long-Term Bandwidth) ---------------------------------------- */

/*
 * ucp_reset_lt_bw_sampling_interval - Reset the interval counters for a
 * new LT BW sampling episode.
 * @sk: TCP socket.
 *
 * Records the current delivered, lost, and timestamp for the start of
 * a new sampling interval.  The lt_rtt_cnt is reset to 0.
 */
static void ucp_reset_lt_bw_sampling_interval(struct sock* sk)                    /* start new LT BW interval */
{
    struct tcp_sock* tp = tcp_sk(sk);                                              /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                /* get UCP CA state */

    ucp->lt_last_stamp = div_u64(tp->delivered_mstamp, (u32)USEC_PER_MSEC);        /* record interval start (ms) */
    ucp->lt_last_delivered = tp->delivered;                                          /* record delivered at interval start */
    ucp->lt_last_lost = tp->lost;                                                      /* record lost at interval start */
    ucp->lt_rtt_cnt = 0;                                                                 /* reset RTT counter */
}
/*
 * ucp_reset_lt_bw_sampling - Fully disable LT BW sampling and clear
 * the LT estimate and use flag.
 * @sk: TCP socket.
 */
static void ucp_reset_lt_bw_sampling(struct sock* sk)                               /* disable LT BW + clear state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                  /* get UCP CA state */

    ucp->lt_bw = 0;                                                                     /* clear LT BW estimate */
    ucp->lt_use_bw = 0;                                                                   /* disable LT BW pacing */
    ucp->lt_is_sampling = 0;                                                               /* disable sampling flag */
    ucp_reset_lt_bw_sampling_interval(sk);                                                   /* reset interval counters */
}
/*
 * ucp_lt_bw_interval_done - Process a completed LT BW interval.
 * @sk: TCP socket.
 * @bw: bandwidth estimate for the just-completed interval (BW_UNIT).
 *
 * Consistency check: the new estimate bw must be within a certain
 * tolerance of the existing lt_bw (if lt_bw > 0):
 *   - Relative: |bw - lt_bw| <= ratio * lt_bw  (default ratio = 1/8).
 *   - Absolute: byte-rate diff <= ucp_lt_bw_diff (default 500 bytes/s).
 *
 * If consistent: update lt_bw to the exponential moving average of
 * (bw + lt_bw) / 2, set lt_use_bw = 1, reset pacing to 1.0x.
 * If inconsistent: replace lt_bw with the new estimate, restart interval.
 */
static void ucp_lt_bw_interval_done(struct sock* sk, u64 bw)                        /* process completed LT BW interval */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                  /* get UCP CA state */
    u64 diff;                                                                          /* absolute bandwidth difference (u64: may exceed 2^32) */

    if (ucp->lt_bw) {                                                                    /* existing LT BW estimate */
        diff = (bw > ucp->lt_bw) ? bw - ucp->lt_bw : ucp->lt_bw - bw;                   /* absolute difference */
        /* Check both relative tolerance (BBR_UNIT ratio) and absolute diff */
        if (((u64)diff * BBR_UNIT <= (u64)ucp_lt_bw_ratio_val * ucp->lt_bw) ||  /* within relative tolerance */
            (ucp_rate_bytes_per_sec(sk, (u64)diff, BBR_UNIT) <=                            /* OR within absolute tolerance */
                (u64)ucp_lt_bw_diff_val)) {                                         /* bytes/s diff check */
            /* Consistent: smooth update using EMA */
                    {
                        u32 en = ucp_lt_bw_ema_num_val;
                        u32 ed = ucp_lt_bw_ema_den_val;
                        ucp->lt_bw = (u32)min_t(u64,
                            (bw * en + (u64)ucp->lt_bw * (ed - en)) / ed, U32_MAX);
                    }
                    /*
                     * Only activate LT BW when the loss is from a bandwidth
                     * policer, not from self-inflicted congestion.  When
                     * qdelay_avg is elevated, the queue is building from
                     * UCP's own over-sending — capping the bandwidth here
                     * would lock the flow into a death spiral where low
                     * bandwidth prevents recovery from the very congestion
                     * that triggered LT BW.
                     *
                     * Two congestion signals (either is sufficient):
                     * 1. qdelay_avg > ecn_qdelay_thresh: persistent EWMA queue (needs ext)
                     * 2. srtt - min_rtt > inst_thresh: instantaneous burst queue
                     *    (works without ext, protects against allocation failure)
                     */
                    {
                        struct ucp_ext* ext = ucp_ext_get(sk);
                        u32 qthresh = (u32)ucp_ecn_qdelay_thresh_us_val;
                        u32 ithresh = (u32)ucp_lt_bw_inst_qdelay_thresh_us;
                        struct tcp_sock* tp = tcp_sk(sk);
                        u32 srtt_us = tp->srtt_us >> 3;                 /* SRTT in µs (kernel stores as 8x) */

                        if (ext && ext->qdelay_avg > qthresh) {          /* persistent queue → congestion */
                            ucp_reset_lt_bw_sampling(sk);                 /* abort LT BW activation */
                            return;
                        }
                        if (srtt_us > ucp->min_rtt_us + ithresh) {       /* burst queue > threshold → congestion */
                            ucp_reset_lt_bw_sampling(sk);                 /* abort: works even without ext */
                            return;
                        }
                    }
                    ucp->lt_use_bw = 1;                                                                /* enable LT BW for pacing */
                    ucp->pacing_gain = BBR_UNIT;                                                         /* reset to cruise gain */
                    ucp->lt_rtt_cnt = 0;                                                                  /* reset RTT counter */
                    return;                                                                                /* done: consistent update */
        }
    }

    /* First estimate or inconsistent: start fresh */
    ucp->lt_bw = (u32)min_t(u64, bw, U32_MAX);                                                      /* store new LT BW estimate, clamp to u32 */
    ucp_reset_lt_bw_sampling_interval(sk);                                                           /* restart interval */
}
/*
 * ucp_lt_bw_sampling - Main LT BW sampling state machine, called per-ACK.
 * @sk: TCP socket.
 * @rs: rate sample.
 *
 * Two modes:
 * A) lt_use_bw == 1 (LT BW active):
 *    - Count round trips.  After lt_bw_max_rtts rounds in PROBE_BW,
 *      reset LT BW and mode (periodically re-probe the path).
 *
 * B) lt_use_bw == 0 (not active):
 *    - Sampling triggers on first loss event.
 *    - Collects up to 4 * lt_intvl_min_rtts rounds of data.
 *    - After at least lt_intvl_min_rtts rounds, if loss ratio >= threshold,
 *      compute bw = delivered * BW_UNIT / interval_time and call
 *      ucp_lt_bw_interval_done().
 *
 * Exits: on app_limited, after timeout (4* min_rtts), or on bad timestamp.
 */
static void ucp_lt_bw_sampling(struct sock* sk, const struct rate_sample* rs)        /* LT BW sampling state machine */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                   /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                     /* get UCP CA state */
    u32 lost, delivered;                                                                   /* interval lost and delivered */
    u64 bw;                                                                                 /* computed interval bandwidth */
    u64 t_us;                                                                               /* interval duration (us), u64 guards against overflow with extreme sysctl configs */

    /* ---- Mode A: LT BW already active ---- */
    if (ucp->lt_use_bw) {                                                                       /* LT BW is active */
        /* Periodically re-probe: reset after lt_bw_max_rtts rounds in PROBE_BW */
        if (ucp->mode == UCP_PROBE_BW && ucp->round_start) {                                     /* PROBE_BW + new round */
            u32 cnt = ucp->lt_rtt_cnt + 1;
            if (cnt >= UCP_LT_RTT_CNT_MAX) {
                cnt = UCP_LT_RTT_CNT_MAX;
            }

            ucp->lt_rtt_cnt = cnt;
            if (cnt >= ucp_lt_bw_max_rtts_val) {
                ucp_reset_lt_bw_sampling(sk);                                                            /* clear LT BW state */
                ucp_reset_mode(sk);                                                                        /* restart from PROBE_BW */
            }
        }
        return;                                                                                        /* done: LT BW active path */
    }

    /* ---- Mode B: Not active; trigger on loss ---- */
    if (!ucp->lt_is_sampling) {                                                                         /* not yet sampling */
        if (!rs->losses) {                                                                                /* no loss this ACK */
            return;                                                                                        /* wait for first loss */
        }

        ucp_reset_lt_bw_sampling_interval(sk);                                                             /* start sampling episode */
        ucp->lt_is_sampling = 1;                                                                            /* set sampling flag */
    }

    /* Abort if app-limited (cannot trust bw estimate) */
    if (rs->is_app_limited) {                                                                               /* app-limited ACK */
        ucp_reset_lt_bw_sampling(sk);                                                                        /* abort sampling */
        return; /* early return */
    }

    /* Count RTT boundaries */
    if (ucp->round_start) {                                                                                   /* round boundary */
        u32 cnt = ucp->lt_rtt_cnt + 1;
        if (cnt >= UCP_LT_RTT_CNT_MAX) {
            cnt = UCP_LT_RTT_CNT_MAX;
        }
        ucp->lt_rtt_cnt = cnt;                                                  /* increment and saturate RTT counter */
    }

    /* Too few RTTs yet; wait for lt_intvl_min_rtts rounds */
    if (ucp->lt_rtt_cnt < (u32)ucp_lt_intvl_min_rtts_val) {                                         /* insufficient rounds */
        return; /* early return */
    }

    /* Timeout: max_mult * min_rtts without enough loss -> abort */
    {
        u32 lt_to = ucp_lt_intvl_max_mult_val * ucp_lt_intvl_min_rtts_val;
        if (ucp->lt_rtt_cnt >= lt_to) { /* exceeded max interval */
            ucp_reset_lt_bw_sampling(sk);                                                                            /* abort: timeout */
            return;
        }
    }

    /* ---- Compute loss ratio over the interval ---- */
    lost = tp->lost - ucp->lt_last_lost;                                                                           /* interval lost pkts */
    delivered = tp->delivered - ucp->lt_last_delivered;                                                              /* interval delivered pkts */

    /* Require some delivered data AND loss ratio >= threshold (BBR_UNIT).
     * Comparison uses scaled integer math: compare (lost*256) < (threshold*delivered).
     * Parenthesize << — C precedence makes << bind looser than <. */
    if (!delivered || ((u64)lost << BBR_SCALE) < ((u64)ucp_lt_loss_thresh_val * delivered)) {
        return; /* early return */
    }

    /* ---- Compute bandwidth over the interval ---- */
    /*
     * BBR uses u32 for the interval because its LT timeout is a compile-time
     * constant (4 * bbr_lt_intvl_min_rtts = 16 RTTs), so t *= USEC_PER_MSEC
     * never overflows u32.
     *
     * UCP's ucp_lt_intvl_max_mult and ucp_lt_intvl_min_rtts are runtime sysctl
     * parameters (capped at 32 and 127 respectively). Worst case:
     *   4064 RTTs * 10 s/RTT * 1000 = 40,640,000 ms * 1000 > U32_MAX
     * Therefore the interval must use u64 to avoid overflow, and the
     * divisor must use u64 div64_u64() instead of BBR's faster u32 do_div().
     */
    t_us = div_u64(tp->delivered_mstamp, USEC_PER_MSEC) - ucp->lt_last_stamp;
    if ((s64)t_us < 1) {                                                                                                    /* interval less than 1 ms, wait for more data */
        return; /* early return */
    }

    t_us *= USEC_PER_MSEC;                                                                                                     /* convert ms -> us, u64 prevents overflow with extreme sysctl configs */
    bw = (u64)delivered * BW_UNIT;                                                                                            /* delivered in BW_UNIT scale */
    bw = div64_u64(bw, t_us);                                                                                                     /* bw = delivered * BW_UNIT / interval_us, u64 divisor required because t_us may exceed u32 range */
    ucp_lt_bw_interval_done(sk, bw);                                                                                            /* process interval result */
}
/* ---- Bandwidth Update (Cardwell et al. 2016) ------------------------- */

/*
 * ucp_update_bw - Update the sliding-window max bandwidth estimate.
 * @sk:  TCP socket.
 * @rs:  rate sample from the current ACK.
 * @ext: extended state (unused here, for consistent API).
 *
 * Per-ACK updates:
 *   1. Validate the rate sample (interval > 0, delivered >= 0).
 *   2. Detect round boundaries: when prior_delivered >= next_rtt_delivered,
 *      a new round starts.  On round start:
 *        - Increment rtt_cnt.
 *        - Reset packet_conservation (exit recovery mode at round boundary).
 *   3. Run LT BW sampling state machine.
 *   4. Compute instantaneous bw = delivered * BW_UNIT / interval_us.
 *   5. Feed into the sliding-window max via minmax_running_max().
 *      Window length = ucp_bw_rt_cycle_len (default 10 rounds).
 *      If app-limited: only update if new bw >= existing max (BBR rule).
 */
static void ucp_update_bw(struct sock* sk, const struct rate_sample* rs,               /* update sliding-window max BW */
    struct ucp_ext* ext)                                                      /* extended state (unused) */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                     /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                       /* get UCP CA state */
    u64 bw;                                                                                  /* instantaneous bandwidth */

    /* Validate rate sample — match BBR: only skip on truly invalid samples.
     * BBR uses `delivered < 0` (negative), not `delivered <= 0` (zero).
     * Zero delivered is valid: the ACK carries no new data but may still
     * cross a round boundary (prior_delivered >= next_rtt_delivered).
     * Skipping zero-delivered ACKs delays round counting and full_bw detection. */
    if (unlikely(rs->interval_us == 0)) {                                                            /* invalid interval: skip */
        return;
    }

    if (unlikely(rs->delivered < 0)) {
        return;                                                                                    /* truly invalid: negative delivered */
    }

    ucp->round_start = 0;                                                                        /* reset round_start flag */

    /* Round boundary detection (BBR round counting).
     * prior_delivered is s32, but delivered wraps at 2^31 (4B pkts).
     * BBR's before() uses unsigned comparison matching Linux's monotonic
     * sequence number arithmetic — safe because next_rtt_delivered
     * is always ahead of or equal to prior_delivered in a single round. */
    if (unlikely(ucp->next_rtt_delivered == 0)) {                                                /* first ACK: initialize */
        ucp->next_rtt_delivered = tp->delivered;                                                   /* set initial delivered baseline */
        ucp->round_start = 1;                                                                       /* mark round start */
    }
    else if (!before(rs->prior_delivered, ucp->next_rtt_delivered)) {                             /* prior_delivered >= next_rtt_delivered */
        ucp->next_rtt_delivered = tp->delivered;                                                      /* update next round baseline */
        ucp->rtt_cnt++;                                                                                /* increment round counter */
        ucp->round_start = 1;                                                                           /* mark round start */
        ucp->packet_conservation = 0;                                                                    /* exit packet conservation at round boundary */
    }
    /* LT BW sampling (must run before bw update to use raw rs) */
    ucp_lt_bw_sampling(sk, rs);                                                                          /* run LT BW state machine */

    /* Instantaneous bandwidth: delivered segments * BW_UNIT / interval_us */
    bw = div_u64((u64)rs->delivered * BW_UNIT, rs->interval_us);                                           /* compute instant bandwidth */

    /* BBR rule: if not app-limited OR new bw >= existing max, update sliding max.
     * App-limited samples are excluded unless they record a new peak. */
    if (!rs->is_app_limited || bw >= ucp_max_bw(sk)) {                                                       /* acceptable sample */
        minmax_running_max(&ucp->bw, ucp_bw_rt_cycle_len_val, ucp->rtt_cnt, (u32)bw);                       /* feed to sliding max */
    }

    /*
     * Auto-recovery from LT BW mode: if the sliding-window max bandwidth
     * consistently exceeds lt_bw by the configured ratio over the
     * configured number of consecutive ACKs, LT BW has become stale.
     * Reset LT sampling and re-enter normal PROBE_BW probing.
     * All parameters tunable via /proc/sys/net/ucp/.
     */
    if (unlikely(ucp->lt_use_bw)) {                                                                     /* LT BW mode active */
        int ratio_num = ucp_lt_restore_ratio_num_val;                                       /* configured ratio numerator */
        int ratio_den = ucp_lt_restore_ratio_den_val;                                       /* configured ratio denominator */
        int consec = ucp_lt_restore_consec_acks_val;                                       /* configured consecutive ACK threshold */
        u32 cur_max = ucp_max_bw(sk);                                                                   /* sliding-window peak */

        if (ratio_num > 0 && ratio_den > 0 &&                                                            /* auto-recovery enabled */
            (u64)cur_max * ratio_den > (u64)ucp->lt_bw * ratio_num) {                                    /* max_bw > lt_bw * num/den (no division, no overflow) */
            if (ucp->lt_restore_cnt < 31) {
                ucp->lt_restore_cnt++;
            }

            /* saturating increment (5-bit bitfield) */
            if (ucp->lt_restore_cnt >= (u32)consec) {                                                        /* threshold reached */
                ucp_reset_lt_bw_sampling(sk);                                                                 /* clear LT BW state */
                ucp_reset_mode(sk);                                                                             /* restart PROBE_BW probing */
                ucp->lt_restore_cnt = 0;                                                                         /* reset counter */
            }
        }
        else {                                                                                               /* below threshold or disabled */
            ucp->lt_restore_cnt = 0;                                                                             /* reset counter on any drop */
        }
    }
}
/* ---- Full BW Reached Detection (Cardwell et al. 2016) ---------------- */

/*
 * ucp_check_full_bw_reached - Detect when the pipe has been filled to
 * capacity (STARTUP -> DRAIN transition criterion).
 * @sk: TCP socket.
 * @rs: rate sample.
 *
 * Algorithm (BBRv1):
 *   - Skip if already full, not at round_start, or app-limited.
 *   - Compute bw_thresh = full_bw * full_bw_threshold (default 1.25x).
 *   - If max_bw >= bw_thresh -> bandwidth still growing; update full_bw.
 *   - Else: increment full_bw_cnt.
 *   - When full_bw_cnt >= full_bw_cnt_val (default 3 rounds without growth):
 *     set full_bw_reached = 1.
 */
static void ucp_check_full_bw_reached(struct sock* sk,                               /* check if pipe is full */
    const struct rate_sample* rs)                                  /* rate sample */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                   /* get UCP CA state */
    u32 bw_thresh;                                                                       /* bandwidth growth threshold */

    if (likely(ucp_full_bw_reached(sk) || !ucp->round_start || rs->is_app_limited)) {             /* skip if already full or invalid */
        return; /* early return */
    }

    /* bw_thresh = full_bw * full_bw_thresh_val / BBR_UNIT (125% default) */
    bw_thresh = (u32)(((u64)ucp->full_bw * ucp_full_bw_thresh_val) >> BBR_SCALE);         /* compute growth threshold */
    {
        u32 cur_max = ucp_max_bw(sk);                                                     /* hoist: single minmax read */
        if (cur_max >= bw_thresh) {                                                       /* bandwidth still growing */
            ucp->full_bw = cur_max;                                                         /* record new peak bandwidth */
            ucp->full_bw_cnt = 0;                                                                    /* reset stagnation counter */
            return; /* early return */
        }
    }

    /* No growth this round: increment stagnation counter */
    ucp->full_bw_cnt = min_t(u32, ucp->full_bw_cnt + 1, UCP_BITFIELD_2BIT_MAX);                     /* saturate at 2-bit field max */
    /* After configured rounds without growth: declare pipe full */
    ucp->full_bw_reached = (ucp->full_bw_cnt >= ucp_full_bw_cnt_val);                      /* set full_bw_reached if threshold met */
}
/* ---- Drain Check (Cardwell et al. 2016) ------------------------------ */

/*
 * ucp_check_drain - Handle STARTUP -> DRAIN transition and DRAIN -> PROBE_BW exit.
 * @sk:  TCP socket.
 * @rs:  rate sample.
 * @ext: extended state.
 *
 * STARTUP -> DRAIN: full_bw_reached triggers mode change to DRAIN,
 *   followed by setting ssthresh to the BDP at 1.0x gain.
 *
 * DRAIN -> PROBE_BW: when estimated inflight at EDT <= target inflight
 *   at 1.0x gain, the queue has been drained; reset mode to PROBE_BW.
 */
static void ucp_check_drain(struct sock* sk, const struct rate_sample* rs,            /* handle drain transitions */
    struct ucp_ext* ext)                                                   /* extended state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                     /* get UCP CA state */

    if (unlikely(ucp->mode == UCP_STARTUP && ucp_full_bw_reached(sk))) {                              /* STARTUP -> DRAIN */
        ucp->mode = UCP_DRAIN;                                                                /* transition: pipe full -> drain excess */
        WRITE_ONCE(tcp_sk(sk)->snd_ssthresh, ucp_inflight(sk, ucp_max_bw(sk),                            /* set ssthresh = BDP at 1x */
            BBR_UNIT, ext));                                                  /* cwnd_gain = BBR_UNIT, ext state */
        /* Reset qdelay_avg to prevent the STARTUP queue buildup from
         * persisting into PROBE_BW and triggering unjustified cwnd reduction.
         * The DRAIN phase ensures the actual queue is emptied before PROBE_BW. */
        if (ext) {
            ext->qdelay_avg = 0;
        }
    }

    if (unlikely(ucp->mode == UCP_DRAIN)) {                                                              /* in DRAIN mode */
        u32 max_bw = ucp_max_bw(sk);                                                            /* hoist max BW for drain */
        if (ucp_packets_in_net_at_edt(sk, tcp_packets_in_flight(tcp_sk(sk)),
            max_bw) <=                                                               /* inflight at EDT <= */
            ucp_inflight(sk, max_bw, BBR_UNIT, ext)) {                                         /* target at 1.0x */
            ucp_reset_mode(sk);                                                                           /* queue drained -> enter PROBE_BW */
        }
    }
}
/* ---- PROBE_RTT Done Check (Cardwell et al. 2016) --------------------- */

/*
 * ucp_check_probe_rtt_done - Check whether PROBE_RTT should end.
 * @sk: TCP socket.
 *
 * Conditions for exit:
 *   - probe_rtt_done_stamp is set (we entered stay period).
 *   - tcp_jiffies32 > probe_rtt_done_stamp (stay duration elapsed).
 *
 * On exit:
 *   - Update min_rtt_stamp (fresh sample obtained).
 *   - Restore cwnd to at least prior_cwnd.
 *   - Set probe_rtt_restored flag (cwnd restore happens in ucp_set_cwnd).
 *   - Reset to PROBE_BW mode.
 *   - Override pacing rate with high_gain to quickly refill pipe.
 */
static void ucp_check_probe_rtt_done(struct sock* sk)                               /* check if PROBE_RTT can exit */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                  /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                     /* get UCP CA state */

    if (unlikely(!ucp->probe_rtt_done_stamp ||                                                      /* stay stamp not set OR */
        !after(tcp_jiffies32, ucp->probe_rtt_done_stamp))) {                                  /* stay duration not elapsed */
        return;                                                                                 /* not yet time to exit */
    }

    ucp->min_rtt_stamp = tcp_jiffies32;                                                        /* fresh min_rtt obtained */
    tcp_snd_cwnd_set(tp, max_t(u32, tcp_snd_cwnd(tp), ucp->prior_cwnd));                        /* restore cwnd to pre-probe level */
    ucp->probe_rtt_restored = 1;                                                                  /* flag for next cwnd update to restore */

    ucp_reset_mode(sk);                                                                            /* transition to PROBE_BW */

    /*
     * Override with high_gain to quickly refill the pipe after the
     * forced drain of PROBE_RTT, bypassing the random cycle phase
     * set by ucp_reset_mode().
     */
    WRITE_ONCE(sk->sk_pacing_rate, ucp_bw_to_pacing_rate(sk, ucp_max_bw(sk), ucp_high_gain_val)); /* refill pacing rate */
}
/* ---- Kalman Filter (Kalman 1960) -------------------------------------
 *
 * Single-state Kalman filter for propagation-delay estimation.
 *
 * State-space model:
 *   State equation:  x[k] = x[k-1] + w   (random walk; w ~ N(0, Q))
 *   Observation:     z[k] = x[k] + v     (v ~ N(0, R))
 *
 * where:
 *   x = true propagation delay (us * kalman_scale)
 *   z = observed RTT = rtt_us * kalman_scale
 *   Q = process noise covariance (adaptive)
 *   R = measurement noise covariance (adaptive)
 *
 * Standard Kalman filter equations (predict + update):
 *   Predict:
 *     x_pred = x_est          (identity state transition)
 *     p_pred = p_est + Q      (predicted error covariance)
 *
 *   Update (upon receiving z):
 *     innovation = z - x_pred
 *     K = p_pred / (p_pred + R)           (Kalman gain)
 *     x_est = x_pred + K * innovation     (state update)
 *     p_est = (1 - K) * p_pred            (covariance update)
 *
 * The above scalar K is implemented as gain_num/gain_den:
 *     K = gain_num / gain_den = p_pred / (p_pred + R)
 *
 * Enhancements over standard Kalman:
 *   - Adaptive Q: scaled by min_rtt_us/1000 to account for path length.
 *   - Adaptive R: increased when jitter exceeds threshold.
 *   - Q-boost: resets p_est to p_est_init when innovation is very large
 *     (path change recovery).
 *   - Outlier gating: rejects samples where |innovation| exceeds a
 *     dynamic threshold, preventing pollution of x_est by transient spikes.
 *   - Consecutive rejection guard: force-accepts after max consecutive
 *     rejections to prevent self-reinforcing lockout.
 *   - BBR-S covariance-matched noise estimation (Welch & Bishop 2006):
 *     online Q and R estimation via innovation and Kalman gain statistics.
 *   - EWMA smoothing of qdelay (queuing delay) and jitter for use in
 *     gain decay, cwnd reduction, and PROBE_RTT interval adjustment.
 */
static void ucp_kalman_update(struct sock* sk, u32 rtt_us,                           /* Kalman filter update */
    struct ucp_ext* ext)                                                /* extended state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                    /* get UCP CA state */
    u64 z;                                                                                /* measurement in scaled units */
    u32 gain_num, gain_den, q, r, p_pred;                                                  /* Kalman gain + noise + predicted cov */
    u32 rtt_max;                                                                            /* dynamic RTT sample rejection ceiling */

    if (unlikely(!ext)) {                                                                         /* no ext: skip update */
        return;
    }

    /*
     * Zero RTT sample: at line rates ≥ 25 Gbps, the serialization
     * time of a 1500-byte packet falls below 1 µs.  The kernel's
     * microsecond-granularity RTT clock can legitimately read 0 µs
     * when consecutive ACKs land within the same microsecond tick.
     *
     * Packet serialization time (1500 bytes, 10^9 bps = 1 Gbps):
     *
     *   Rate (Gbps)   Serialization (ns)
     *   ───────────   ─────────────────
     *   10            1200
     *   25             480
     *   40             300
     *   50             240
     *   100            120
     *   200             60
     *   400             30
     *   800             15
     *   1200            10
     *
     * At 25 Gbps, a 1500-byte frame serializes in 480 ns — well under
     * 1 µs.  Consecutive ACKs (data → ACK → next ACK) can thus land
     * in the same microsecond, producing a legitimate rtt_us = 0
     * measurement.  Discarding such samples distorts state estimation
     * on high-speed paths.
     *
     * We floor rtt_us at 1 µs — the smallest representable meaningful
     * delay — to bound distortion while preserving measurement
     * existence (the round-trip occurred and produced a valid ACK).
     */
    rtt_us = max_t(u32, rtt_us, 1U);

    /* Measurement z = rtt_us * kalman_scale (fixed-point scale) */
    z = (u64)rtt_us << ucp_kalman_scale_shift_val;                                             /* convert to scaled units (shift = ilog2(scale)) */

    /* ---- Cold start: initialize state directly from first sample ---- */
    if (unlikely(ext->sample_cnt == 0)) {                                                           /* no prior state estimate */
        ext->x_est = (u32)min_t(u64, z, U32_MAX);                                                        /* set x_est = first measurement, clamp u64->u32 */

        /* Initialize Kalman covariance: p_est = max(p_est_init, rtt_us / divisor) */
        ext->p_est = max_t(u32, ucp_kalman_p_est_init_val,
            rtt_us / max_t(u32, ucp_kalman_p_est_init_rtt_div_val, 1U));
        ext->qdelay_avg = 0;                                                                              /* no qdelay on first sample */
        ext->jitter_ewma = max_t(u32, rtt_us / UCP_JITTER_SEED_DIV, 1U);                                                       /* seed jitter from rtt_us/4, floor 1us */
        ext->sample_cnt = 1;                                                                                /* first sample accepted */
        return;                                                                                             /* cold start complete */
    }

    /*
     * One-time cold-start overshoot correction: if the first sample was
     * inflated by queue delay (x_est > min_rtt_us * scale), cap it now.
     * The bootstrapped min_rtt_us (from 3WHS) provides a realistic upper
     * bound for the propagation delay.  After this single correction, the
     * directional update (negative innovations only) prevents queue-noise
     * drift, and qboost freely tracks genuine path changes upward.
     */
    if (unlikely(ext->sample_cnt == 1) &&
        ucp->min_rtt_us != UCP_MIN_RTT_UNINIT) {
        u32 ceiling = (u32)min_t(u64,
            (u64)ucp->min_rtt_us << ucp_kalman_scale_shift_val, U32_MAX);
        if (ext->x_est > ceiling) {
            ext->x_est = ceiling;
        }
    }

    /* Discard excessively large RTT samples.  The threshold is dynamic:
     * ucp_rtt_sample_max_us_val is the configured floor (default 500ms),
     * but for paths with baseline RTT > half the floor (e.g. GEO satellite
     * with 600ms RTT), the effective threshold lifts to min_rtt_us * 2
     * so Kalman can still converge.  Cold start has already returned
     * above, so sample_cnt > 0 is guaranteed here. */
    rtt_max = ucp_rtt_sample_max_us_val;                                       /* configured floor: 500ms default */
    if (ucp->min_rtt_us != UCP_MIN_RTT_UNINIT && ucp->min_rtt_us > 0 &&                                      /* valid min_rtt */
        ucp->min_rtt_us * (u64)ucp_kalman_rtt_dyn_mult_val > rtt_max) {              /* u64 prevents overflow */
        rtt_max = (u32)min_t(u64, (u64)ucp->min_rtt_us * (u64)ucp_kalman_rtt_dyn_mult_val, U32_MAX);           /* lift to dynamic threshold */
    }

    if (unlikely(rtt_us > rtt_max)) {                                                           /* cap exceeded: discard */
        return;
    }

    /* ---- Adaptive Q: scaled by min_rtt_us / divisor (Kalman 1960) ---- */
    /*
     * Base Q is multiplied by max(q_min_factor, min_rtt_us / ucp_kalman_q_rtt_div)
     * Q = Q_base * max(q_min_factor, min_rtt_us / q_rtt_div)
     * Capped at Q_base * q_scale_cap to prevent runaway on very long paths.
     * The scaling accounts for the fact that random-walk variance on a
     * longer path is proportionally larger.
     */
    {
        u64 q64;                                                                                           /* 64-bit Q accumulator */
        u32 q_rtt_factor = (ucp->min_rtt_us != UCP_MIN_RTT_UNINIT) ? ucp->min_rtt_us / (u32)ucp_kalman_q_rtt_div_val : 0;
        q64 = (u64)ucp_kalman_q_val;                                                                  /* base process noise Q (clamped cache) */
        q64 *= (u64)max_t(u32, ucp_kalman_q_min_factor_val,                                         /* multiply by max(factor, */
            q_rtt_factor);                  /* min_rtt_us / configured divisor */
        q64 = min_t(u64, q64,                                                                                     /* cap 1: Q_base * q_scale_cap */
            (u64)ucp_kalman_q_scale_cap_val * (u64)ucp_kalman_q_val);                  /* cap = cap_val * Q_base */
        q64 = min_t(u64, q64, (u64)ucp_kalman_q_max_val);                                         /* cap 2: absolute Q upper bound */
        q = (u32)q64;                                                                                               /* store adaptive Q */
    }
    /* ---- Adaptive R: increased by jitter (Kalman 1960) ---- */
    /*
     * R = base_R + min(max(0, jitter - jr_thresh) * base_R / jr_scale,
     *                  base_R * ucp_kalman_r_max_boost)
     * Boost capped to prevent gain freeze on paths with persistent high
     * Measurement noise increases when RTT jitter increases,
     * causing the Kalman gain to decrease (the filter trusts
     * measurements less when jitter is high).
     */
    {
        u32 base_r = max_t(u32, ucp_kalman_r_val, 1U);                                                                     /* base measurement noise R (clamped cache) */
        u32 jitter = ext->jitter_ewma;                                                                              /* current jitter EWMA */
        u32 jr_thresh = (u32)ucp_jitter_r_thresh_us_val;                                                       /* jitter threshold for R increase */
        u32 jr_scale = (u32)ucp_jitter_r_scale_val;                                                             /* scaling divisor for R increase */

        if (jitter > (u32)jr_thresh) {                                                                                   /* jitter exceeds threshold */
            u64 r_boost = (u64)(jitter - jr_thresh) * (u64)base_r / (u64)jr_scale;                       /* linear R boost in u64 to avoid truncation */
            u64 r_cap = (u64)base_r * (u32)ucp_kalman_r_max_boost_val;                                 /* cap: base_R * max_boost (u64 safe) */
            r = base_r + (u32)min_t(u64, r_boost, r_cap);                                                                       /* capped R = base + min(boost, cap) */
        }
        else {                                                                                                           /* jitter within threshold */
            r = base_r;                                                                                                      /* use base R unchanged */
        }
    }

    /*
     * Combine heuristic Q/R with covariance-matched estimates (BBR-S method).
     * Mode 0 (disabled): use heuristic Q/R only.
     * Mode 1 (max):       use max(heuristic, matched) — conservative, default.
     * Mode 2 (avg):       use (heuristic + matched) / 2 — balanced.
     * The matched estimates (q_est, r_est) are initialized at module-init
     * to the base Q/R values and updated by covariance matching on each
     * accepted Kalman sample (Welch & Bishop 2006).
     */
    {
        int mode = ucp_kalman_noise_mode_val;                                        /* noise combination mode */
        if (mode > 0 && ext->q_est > 0 && ext->r_est > 0) {                                    /* enabled and estimates valid */
            if (mode == 1) {                                                                    /* mode 1: max(heuristic, matched) */
                q = max_t(u32, q, ext->q_est);                                                  /* Q = max(adaptive_Q, matched_Q) */
                r = max_t(u32, r, ext->r_est);                                                  /* R = max(adaptive_R, matched_R) */
            }
            else if (mode == 2) {                                                                 /* mode 2: weighted average */
                {
                    u32 na = ucp_kalman_noise_avg_num_val;
                    u32 da = ucp_kalman_noise_avg_den_val;
                    q = (u32)(((u64)q * (da - na) + (u64)ext->q_est * na) / da);
                    r = (u32)(((u64)r * (da - na) + (u64)ext->r_est * na) / da);
                }
            }
        }
    }

    /* ACK aggregation noise adjustment with hysteresis (BBRplus-inspired).
     * R increases instantly when confidence warrants (fast response to
     * detected aggregation).  R recovers gradually at round boundaries
     * by decaying agg_r_scaled toward min_mult, preventing Kalman
     * filter oscillation from abrupt R changes.
     *
     * NOTE: Confidence evaluation runs BEFORE ucp_update_model in ucp_main,
     * so agg_confidence read here reflects the current ACK's evaluation.
     * The Kalman R scale now updates synchronously with detection. */
    {
        u16 conf = ext->agg_confidence;                                                    /* ext is non-NULL after early return guard at top of function */
        u32 r_min = ucp_agg_r_multiplier_min_val;
        u32 r_max = ucp_agg_r_multiplier_max_val;

        if (conf >= (u16)ucp_agg_thresh_suspected_val) {
            /* R scales up instantly with confidence: linear interpolation between min and max */
            u32 target_mult = r_min + (r_max - r_min) * (u32)conf / (u32)ucp_agg_confidence_max_val;
            /* Clamp the linear interpolation result to [r_min, r_max] to prevent
             * overshoot when conf exceeds confidence_max (possible when factor_weight
             * is configured independently from confidence_max). */
            ext->agg_r_scaled = clamp_t(u32, target_mult, r_min, r_max);  /* instant increase, clamped */
        }
        else if (ucp->round_start && ext->agg_r_scaled > r_min) {
            /* Round boundary: decay R toward baseline at hysteresis rate.
             * Default 75% retained = 25% decay per RTT, ~4 RTTs to baseline. */
            u32 hysteresis_pct = ucp_agg_r_hysteresis_val;
            ext->agg_r_scaled = (ext->agg_r_scaled * hysteresis_pct + r_min * (UCP_PCT_BASE - hysteresis_pct)) / UCP_PCT_BASE;
        }

        /* Apply the persisting R scale to current measurement noise */
        if (ext->agg_r_scaled > BBR_UNIT) {
            r = (u32)min_t(u64, ((u64)r * ext->agg_r_scaled) >> BBR_SCALE, U32_MAX);  /* u64 intermediate, clamp to u32 */
        }
    }

    /* ---- Core Kalman update (Kalman 1960) ---- */
    {
        /* innovation = z - x_est (in scaled units; may be negative) */
        s64 innovation = (s64)z - (s64)ext->x_est;                                                                           /* innovation = measurement - state */
        u64 abs_innov = innovation >= 0 ? (u64)innovation : (u64)(-(innovation + 1)) + 1;                                       /* absolute value of innovation */
        u64 corr_abs = 0;                                                                                                      /* correction magnitude (hoisted for covariance matching) */
        bool qboost_fired = false;                                                                                                  /* Q-boost flag: skip outlier gate */

        /*
         * Q-boost: if innovation exceeds the boost threshold AND the
         * filter has converged (p_est <= converged_p_est) AND the
         * cooldown has expired, reset p_est to p_est_init.  This
         * causes the Kalman gain to spike, allowing the filter to
         * rapidly track a genuine path change (e.g., route change,
         * mobility event).
         *
         * Confidence gate: only fire qboost when the filter is
         * confident.  When p_est is large (filter uncertain —
         * recent qboost or startup), large innovations are treated
         * as noise/jitter and pass through normal outlier rejection.
         *
         * Cooldown: after each qboost, block further qboost for
         * ucp_kalman_qboost_cdwn_val (default 15) samples.  On noisy paths
         * with RTT jitter > qboost_thresh (4ms), the confidence gate
         * alone is insufficient — p_est drops below converged after
         * ~1-2 samples (high Kalman gain from the p_est reset), and
         * the next jitter spike would re-trigger qboost.  The
         * cooldown forces a minimum observation window between
         * qboost events (~1 RTT at 200ms, 5 samples), ensuring
         * only persistent path changes (not transient jitter) can
         * defeat the directional-update guard.
         *
         * These gates together prevent the Kalman filter from
         * degrading to a simple low-pass on lossy paths:
         *   1. p_est stays converged (no perpetual reset),
         *   2. Outlier rejection gates transient spikes,
         *   3. x_est tracks propagation delay, not queued RTT,
         *   4. Flow fairness is preserved (all flows converge to
         *      similar BDP through shared min_rtt).
         * The reset happens BEFORE outlier rejection so that large
         * innovations from path changes can enter the filter.
         */
        if (unlikely(ext->qboost_cdwn == 0 &&
            ext->p_est <= ucp_kalman_converged_p_est_val &&
            abs_innov > ucp_kalman_q_boost_thresh_val)) {                                        /* cooldown expired + converged + large innovation: path change */
            ext->p_est = ucp_kalman_p_est_init_val;                                                                      /* reset covariance for high gain */
            ext->qboost_cdwn = (u8)ucp_kalman_qboost_cdwn_val;                                                                   /* start cooldown */
            qboost_fired = true;                                                                                                     /* mark Q-boost: skip outlier gate below */
        }
        else if (ext->qboost_cdwn > 0) {                                                                                         /* cooldown active */
            ext->qboost_cdwn--;                                                                                                       /* decrement toward expiration */
        }

        /* ---- Prediction step: p_pred = p_est + Q (Kalman 1960) ---- */
        /* p_est ≤ 100M clamped, q ≤ 100k clamped, sum ≤ 100.1M << U32_MAX */
        p_pred = min_t(u32, ext->p_est + q, ucp_kalman_p_est_max_val);                                               /* p_pred = min(p_est+Q, p_est_max) */
        /* ---- Outlier rejection (Kalman 1960) ---- */
        /*
         * Dynamic threshold = max(outlier_ms * 1000 * scale,
         *                         jitter_ewma * outlier_jitter_mult * scale)
         *
         * If abs(innovation) > threshold AND p_pred <= converged_p_est
         * (filter is confident enough that this is truly an outlier, not
         *  a genuine path change), reject the sample.
         *
         * When rejected:
         *   - sample_cnt is NOT incremented (filter state unchanged).
         *   - jitter_ewma is updated even on rejection, to prevent the
         *     dynamic threshold from locking in at an old value.
         *   - During high jitter, the Kalman-min_rtt takeover is
         *     intentionally delayed (needs min_samples clean updates).
         * When force-accepted (after max_consec_reject):
         *   - Falls through to full Kalman update below.
         *   - sample_cnt, x_est, p_est, jitter_ewma, qdelay_avg are
         *     all updated normally on the force-accepted sample.
         */
        {
            u64 dyn_thresh = ucp_kalman_outlier_thresh_scaled_val; /* base outlier threshold (precomputed) */
            u64 jitter_prod = (u64)ext->jitter_ewma * (u64)ucp_kalman_outlier_jitter_mult_val;
            u64 jitter_thresh = min_t(u64, jitter_prod, ucp_kalman_shift_cap_val) << ucp_kalman_scale_shift_val;

            if (jitter_thresh > dyn_thresh) {                                                                                                /* jitter threshold exceeds base */
                dyn_thresh = jitter_thresh;                                                                                                  /* use jitter-based threshold */
            }

            if (unlikely(!qboost_fired && abs_innov > dyn_thresh && p_pred <= ucp_kalman_converged_p_est_val)) {                          /* outlier + confident AND Q-boost not fired */
                /*
                 * Force-accept if too many consecutive rejections have occurred.
                 * Without this, a self-reinforcing cycle can lock in: high
                 * jitter raises the dynamic threshold, causing more rejections,
                 * which raise jitter further (jitter is updated even on rejection).
                 * After ucp_kalman_max_consec_reject_val consecutive rejections,
                 * the gate is bypassed and the sample enters the filter.
                 */
                u32 raw_jitter;                                                                                          /* jitter in us (C90: decl before stmt) */
                if (ext->consec_reject_cnt < (u32)ucp_kalman_max_consec_reject_val) {                  /* still within rejection budget */
                    ext->consec_reject_cnt++;                                                                    /* increment rejection counter */
                    /* Reject outlier: do not update x_est or p_est.
                     * Update jitter for threshold dynamics but skip state update. */
                    raw_jitter = (u32)min_t(u64, abs_innov >> ucp_kalman_scale_shift_val, U32_MAX);           /* |innov| back to us */
                    ext->jitter_ewma = ext->jitter_ewma ?                                                         /* if existing EWMA */
                        ((u64)ext->jitter_ewma * ucp_ewma_jitter_num_val +                                  /* old * num + */
                            raw_jitter * UCP_EWMA_NEW_WEIGHT) / ucp_ewma_jitter_den_val :                                      /* new * 1 / den */
                        raw_jitter;                                                                                    /* first sample: use raw */
                    return;                                                                                            /* rejection: no further processing */
                }

                /* Fall through: force-accept after max consecutive rejections.
                 * Jitter EWMA will be updated by the normal Kalman post-update
                 * path below; this prevents the stale outlier-jitter feedback. */
            }

            /* Reset rejection counter on any accepted sample */
            if (ext->consec_reject_cnt) { ext->consec_reject_cnt = 0; }                                                                              /* clear: sample passed gate */
        }

        /* ---- Compute Kalman gain: K_num/K_den = p_pred / (p_pred + R) (Kalman 1960) ---- */
        gain_num = p_pred;                                                                                                                         /* numerator = predicted covariance */

        /* gain_den = p_pred + r.  With clamped params, p_pred ≤ 100M and r ≤ 100M,
         * so p_pred + r ≤ 200M << U32_MAX.  No overflow guard needed.
         * gain_den >= 1 is guaranteed by p_pred >= p_est_floor >= 1 and r >= 1. */
        gain_den = p_pred + r;                                                                                                                  /* denominator = p_pred + meas noise */

        /* ---- State update: x_est = x_est + K * innovation (Kalman 1960) ---- */
        /*
         * correction = (abs_innov * gain_num) / gain_den
         * x_est = x_est +/- correction (sign follows innovation)
         *
         * Directional update policy (UCP-specific):
         *
         * The Kalman model z = x + v assumes zero-mean measurement noise,
         * but on a congested path the observation z = x + q + v includes
         * non-negative queue delay q.  Positive innovations (z > x_est)
         * may be either path changes or queue buildup — the filter
         * cannot distinguish.  Updating on queue noise causes x_est to
         * drift toward the AVERAGE RTT rather than the propagation delay.
         *
         * Therefore:
         *   - Negative innovation (z < x_est): always update.  The observed
         *     RTT is below the current estimate — the propagation delay
         *     has likely decreased.  Pull x_est DOWN toward the clean sample.
         *   - Positive innovation + Q-boost fired: update.  The innovation
         *     is large enough to indicate a genuine path change (route
         *     switch, mobility event).  Pull x_est UP toward the new path.
         *   - Positive innovation, no Q-boost: SKIP state update.  The
         *     innovation is queue delay, not a propagation delay change.
         *     The covariance update (below) still runs — the measurement
         *     provides information about filter uncertainty.
         *
         * This transforms the Kalman into an asymmetric estimator that
         * only tracks propagation delay DECREASES (plus Q-boosted
         * increases), matching the physical constraint that queue delay
         * is non-negative and propagation delay changes are rare.
         *
         * Overflow guard: if abs_innov * gain_num would overflow u64,
         * cap the product at U64_MAX.
         */
        {
            u64 prod;                                                                                                                                  /* product abs_innov * gain_num */
            s64 correction;                                                                                                                               /* signed correction */

            if (gain_num > 0 && abs_innov > U64_MAX / (u64)gain_num) {                                                                                     /* overflow check */
                prod = U64_MAX;                                                                                                                              /* cap product at max */
            }
            else {                                                                                                                                         /* safe to multiply */
                prod = abs_innov * (u64)gain_num;                                                                                                              /* product = |innov| * gain_num */
            }

            /* Compute correction magnitude unconditionally:
             * needed for both state update AND BBR-S covariance-matched
             * noise estimation (Welch & Bishop 2006).  When the state
             * update is skipped (positive innovation = queue noise), the
             * correction magnitude still informs matched Q/R estimation. */
            corr_abs = div_u64(prod, gain_den);                                                                                                                /* absolute correction = prod / den */

            if (innovation < 0 || qboost_fired) {
                /* Downward update (clean sample) OR Q-boost (path change):
                 * apply correction in the direction of the innovation. */
                correction = (innovation >= 0) ? (s64)min_t(u64, corr_abs, UCP_S64_MAX) : -(s64)min_t(u64, corr_abs, UCP_S64_MAX);                                     /* sign follows innovation (saturate to avoid -(s64)U64_MAX UB) */

                {                                                                                                                                                      /* x_est update: ext->x_est is u32, correction is s64 bounded to ±S64_MAX */
                    s64 new_x = (s64)ext->x_est + correction;                                                                     /* cannot overflow s64: u32 max + S64_MAX < S64_MAX + 4e9 ≤ S64_MAX */
                    if (new_x > 0) {                                                                                             /* x_est must stay positive */
                        ext->x_est = (u32)min_t(s64, new_x, (s64)U32_MAX);                                                       /* clamp to u32 range */
                    }
                    else {                                                                                                                                              /* correction would make state <= 0 */
                        u64 floor = (ucp->min_rtt_us != UCP_MIN_RTT_UNINIT) ?                                                                                    /* guard: min_rtt_us valid */
                            min_t(u64, (u64)ucp->min_rtt_us << ucp_kalman_scale_shift_val, U32_MAX) : 0;                                              /* floor = min_rtt * scale or 0 */
                        ext->x_est = (u32)max_t(u64,                                                                                                                         /* floor = */
                            (u64)1U << ucp_kalman_scale_shift_val,                                                                                                 /* max(scale = 1<<shift, */
                            floor);                                                                                                                                            /* min_rtt * scale) */
                    }
                }
            }
            else {
                /* Positive innovation without Q-boost: queue noise.
                 * Skip state update — x_est stays at current estimate.
                 * The covariance update (below) still runs to reflect
                 * that we incorporated a measurement. */
            }
        }

        /* Anti-drift: the directional update (above) skips positive
         * innovations (queue noise), so x_est can only drift DOWN
         * from clean samples or UP via qboost (path changes).
         *
         * No hard cap is needed:
         *  - Cold-start overshoot is corrected once (sample_cnt==1).
         *  - Queue noise cannot lift x_est (positive innovations skipped).
         *  - Qboost-driven path-change tracking is desirable: when
         *    the innovation exceeds the qboost threshold, a genuine
         *    path change is likely, and x_est should be allowed to
         *    exceed min_rtt_us temporarily.
         *
         * Self-correction: a false-positive qboost overshoot is
         * pulled back down by subsequent negative innovations
         * (RTT samples below the over-estimate). */

         /* ---- Update jitter EWMA from accepted innovation ---- */
         /*
          * raw_jitter = |innovation| / scale (back to us)
          * jitter_ewma = (jitter_ewma * num + raw_jitter) / den
          */
        {
            u32 raw_jitter = (u32)min_t(u64, abs_innov >> ucp_kalman_scale_shift_val, U32_MAX);                                                               /* jitter in us */
            ext->jitter_ewma = ext->jitter_ewma ?                                                                                                                     /* if existing jitter EWMA */
                (u32)(((u64)ext->jitter_ewma * ucp_ewma_jitter_num_val +                                                                                               /* old * num + */
                    raw_jitter * UCP_EWMA_NEW_WEIGHT) / ucp_ewma_jitter_den_val) :                                                                                                 /* new / den */
                raw_jitter;                                                                                                                                              /* first sample */
        }
        /* ---- Covariance update: p_est = (1 - K) * p_pred (Kalman 1960) ---- */
        /*
         * p_new = p_pred - p_pred * gain_num / gain_den
         *       = p_pred * (1 - K)
         * Floor at p_est_floor_val to prevent filter lock-in.
         */
        {
            u64 p_new = (u64)p_pred -                                                                                                                                    /* p_pred - correction */
                div_u64((u64)p_pred * gain_num, gain_den);                                                                                                              /* p_pred * K */
            ext->p_est = max_t(u32, (u32)p_new, ucp_kalman_p_est_floor_val);                                                                                      /* floor at configurable minimum */
        }
        /* ---- Update EWMA queuing delay ---- */
        /*
         * qdelay_instant = max(0, (z - x_est) / scale)
         *   i.e., observed RTT minus estimated prop delay, zero if negative.
         * qdelay_avg = (qdelay_avg * num + instant) / den
         */
        {
            u32 qdelay_instant = (z > ext->x_est) ?                                                                                                                          /* if measurement > estimate */
                (u32)((z - ext->x_est) >> ucp_kalman_scale_shift_val) : 0;                                                                                                 /* u64 division, cast to u32 after */
            if (ext->sample_cnt == 1) {                                                                                                                                        /* second sample (first after init) */
                ext->qdelay_avg = qdelay_instant;                                                                                                                               /* init qdelay EWMA directly */
            }
            else {                                                                                                                                                             /* normal EWMA update */
                ext->qdelay_avg = (u32)(((u64)ext->qdelay_avg * ucp_ewma_qdelay_num_val +                                                                                        /* old * num + */
                    qdelay_instant * UCP_EWMA_NEW_WEIGHT) / ucp_ewma_qdelay_den_val);                                                                                              /* new * 1 / den */
            }
        }

        if (ext->sample_cnt < U32_MAX) {                                        /* accepted update: saturating increment */
            ext->sample_cnt++;
        }
        /*
         * BBR-S covariance-matched noise estimation (Welch & Bishop 2006).
         * Updates q_est and r_est using the latest innovation and Kalman gain.
         * Only runs on accepted samples (after outlier gate and Q-boost check).
         *
         * Q estimate: q_est = (1-alpha) * q_est + alpha * (K * innov)^2
         * R estimate: r_est = (1-beta)  * r_est + beta  * max(0, innov^2 - p_pred)
         *
         * innov and K*innov are in scaled units (us * kalman_scale).
         * Their squares are in (us * S)^2, while p_est, Q, and R are in
         * plain µs².  Shift right by 2*scale_shift (= S²) to convert the
         * matched estimates back to µs² before blending with q_est/r_est.
         * Without this, the matched terms dominate by ~10⁶×, saturating
         * q_est at its ceiling and disabling outlier rejection.
         */
        if (ucp_kalman_noise_mode_val > 0) {                                                             /* noise estimation enabled */
            u64 corr = corr_abs;                                                                                    /* K * |innov| in scaled units */
            u64 innov_sq, keps_sq;                                                                                  /* squared innovation and squared correction */
            s64 r_contrib;                                                 /* R noise contribution (signed, may be negative) */
            u64 q_new, r_new;                                              /* new Q and R estimate accumulators for EWMA blend */
            u32 alpha_n = (u32)ucp_kalman_noise_alpha_num_val;                                                /* alpha numerator */
            u32 alpha_d = (u32)ucp_kalman_noise_alpha_den_val;                                                /* alpha denominator */
            u32 beta_n = (u32)ucp_kalman_noise_beta_num_val;                                                 /* beta numerator */
            u32 beta_d = (u32)ucp_kalman_noise_beta_den_val;                                                 /* beta denominator */
            u32 q_max = (u32)ucp_kalman_q_est_max_val;                                                      /* Q est upper bound */
            u32 r_max = (u32)ucp_kalman_r_est_max_val;                                                      /* R est upper bound */
            u32 q_floor = (u32)ucp_kalman_q_est_floor_val;                                                    /* Q est lower bound */
            u32 r_floor = (u32)ucp_kalman_r_est_floor_val;                                                    /* R est lower bound */

            /* Innov^2: guard against overflow with extreme configs (scale up to 1M, RTT up to 10s).
             * Cap abs_innov at sqrt(S64_MAX) ~ 3e9 before squaring to keep innov^2 within s64 range. */
            if (abs_innov > UCP_KALMAN_INNOV_SQ_CAP) {                                              /* overflow guard: sqrt(S64_MAX) */
                abs_innov = UCP_KALMAN_INNOV_SQ_CAP;
            }

            innov_sq = (u64)abs_innov * abs_innov;                                                                  /* innov^2 in (us*S)^2 units, fits in s64 */
            innov_sq >>= (u32)ucp_kalman_scale_shift_val * 2;                                                                /* rescale (us*S)^2 → µs² (divide by S²) */

            /* (K*innov)^2: cap corr like abs_innov before squaring. */
            if (corr > UCP_KALMAN_INNOV_SQ_CAP) {                                                    /* overflow guard: sqrt(S64_MAX) */
                corr = UCP_KALMAN_INNOV_SQ_CAP;
            }

            keps_sq = corr * corr;                                                                                  /* (K*innov)^2 in (us*S)^2 units */
            keps_sq >>= (u32)ucp_kalman_scale_shift_val * 2;                                                                /* rescale (us*S)^2 → µs² (divide by S²) */

            /* Q estimate (covariance matching): Q = (1-alpha)*Q + alpha * (K*innov)^2
             * Cap keps_sq before multiplication to prevent u64 overflow.
             * After S² rescale (>> 2*shift), keps_sq fits well within u64;
             * this guard is a belt-and-suspenders for extreme scale configs. */
            if (keps_sq > U64_MAX / (u64)max_t(u32, alpha_n, 1U)) {
                keps_sq = U64_MAX / (u64)max_t(u32, alpha_n, 1U);
            }

            {
                u64 t1 = (u64)ext->q_est * (u64)ucp_kalman_noise_alpha_complement;
                u64 t2 = (u64)alpha_n * keps_sq; u64 s = (t1 > U64_MAX - t2) ? U64_MAX : t1 + t2;
                q_new = s / (u64)alpha_d;
            }          /* EWMA blend: old*(1-alpha) + alpha*sample */
            ext->q_est = (u32)clamp_t(u64, q_new, (u64)q_floor, (u64)q_max);                                       /* publish Q estimate, clamped */

            /* R estimate: R = (1-beta)*R + beta * max(0, innov^2 - p_pred)
             * Cap r_contrib before multiplication to prevent u64 overflow.
             * After S² rescale, innov_sq fits well within u64;
             * this guard is a belt-and-suspenders for extreme scale configs. */
            r_contrib = (s64)innov_sq - (s64)p_pred;                                                                /* E[innov^2] = P + R, so R_contrib = innov^2 - P */
            if (r_contrib < 0) {                                                                     /* floor at 0: negative contribution */
                r_contrib = 0;
            }

            {
                u64 r_contrib_u64 = min_t(u64, (u64)r_contrib, U64_MAX / (u64)max_t(u32, beta_n, 1U));
                r_contrib = (r_contrib_u64 > UCP_S64_MAX) ? (s64)UCP_S64_MAX : (s64)r_contrib_u64;
            }

            {
                u64 t1 = (u64)ext->r_est * (u64)ucp_kalman_noise_beta_complement; u64 t2 = (u64)beta_n * (u64)r_contrib;
                u64 s = (t1 > U64_MAX - t2) ? U64_MAX : t1 + t2;
                r_new = s / (u64)beta_d;
            }       /* EWMA blend: old*(1-beta) + beta*sample */
            ext->r_est = (u32)clamp_t(u64, r_new, (u64)r_floor, (u64)r_max);                                       /* publish R estimate, clamped */
        }
    }
}
/* ---- Min RTT Update ---------------------------------------------------- */

/*
 * ucp_update_min_rtt - Update the min_rtt_us estimate using both the
 * traditional window-based filter and the Kalman filter (Kalman 1960).
 * @sk:  TCP socket.
 * @rs:  rate sample from this ACK.
 * @ext: extended state (for Kalman filter update).
 *
 * Processing sequence:
 *   1. Track delayed-ACK status.
 *   2. Check if PROBE_RTT filter interval has expired.
 *   3. Run Kalman filter update (feeds rtt_us into ucp_kalman_update).
 *   4. Traditional min_rtt window update (only when Kalman has NOT taken over):
 *      - Sticky fall: gradual reduction using sticky_num/sticky_den ratio.
 *      - Fast fall: immediate reduction when rtt < min_rtt / 4 for fast_fall_cnt
 *        consecutive samples.
 *   5. SRTT guard: override min_rtt if SRTT < min_rtt * guard_ratio.
 *   6. PROBE_RTT entry: if filter_expired and not idle_restart and mode != PROBE_RTT.
 *   7. PROBE_RTT management: determine stay period and exit conditions.
 *   8. Kalman takeover: when x_est is valid and sample_cnt >= min_samples,
 *      replace min_rtt_us with x_est / kalman_scale, and compute dynamic
 *      PROBE_RTT interval.
 *
 * NOTE: min_rtt_stamp is NOT refreshed when Kalman sets min_rtt_us,
 * so that PROBE_RTT can still periodically re-probe the true path
 * propagation delay (the Kalman estimate may drift on path changes).
 */
static void ucp_update_min_rtt(struct sock* sk, const struct rate_sample* rs,        /* update min_rtt + Kalman + PROBE_RTT */
    struct ucp_ext* ext)                                                /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                    /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                       /* get UCP CA state */
    bool filter_expired;                                                                     /* flag: PROBE_RTT interval expired */

    u32 now, rtt_clamped;                                                                    /* hoisted: jiffies timestamp and clamped RTT */

    now = tcp_jiffies32;                                                                    /* cache volatile jiffies for entire function */
    rtt_clamped = rs->rtt_us >= 0 ? (u32)min_t(s64, rs->rtt_us, U32_MAX) : 0;               /* hoist clamped RTT: used 4x below */
    /*
     * filter_expired: PROBE_RTT interval elapsed since last min_rtt update.
     *
     * Per-flow jitter prevents all co-existing flows from entering
     * PROBE_RTT simultaneously.  Without jitter, 8 flows drain to
     * min_target (4 pkts) in the same ~200 ms window (aggregate
     * ~1.8 Mbps) and then simultaneously refill at 2.89x, creating a
     * 3x overshoot that drives the tail flow into RTO and zero
     * throughput.  Jitter spreads the drains over 0..2 min_rtt so at
     * most ~1 flow is in PROBE_RTT at any instant.
     *
     * Jitter derived from sk->sk_hash: stable per-flow, unique across
     * flows.  Refill pacing unchanged (2.89x high_gain, matching BBR).
     */
    {
        u32 interval = ucp_get_probe_rtt_interval(sk, ext);
        u32 jitter_jif = 0;

        if (ucp->min_rtt_us != UCP_MIN_RTT_UNINIT && ucp->min_rtt_us > 0) {
            jitter_jif = usecs_to_jiffies(
                (u32)(sk->sk_hash & UCP_PROBE_RTT_JITTER_HASH_MASK) * ucp->min_rtt_us / UCP_PROBE_RTT_JITTER_DIV);
        }
        filter_expired = after(now, ucp->min_rtt_stamp + interval + jitter_jif);
    }

    /* Kalman filter update: feed every valid RTT sample into the filter (Kalman 1960) */
    if (likely(rs->rtt_us >= 0)) {                                                                         /* valid RTT sample */
        ucp_kalman_update(sk, rtt_clamped, ext);                             /* run Kalman update */
    }

    /* ---- Traditional min_rtt window update ---- */
    /*
     * Only apply window-based min_rtt tracking when the Kalman filter
     * has NOT taken over (i.e., either ext is NULL, x_est is 0, or
     * sample_cnt < min_samples).
     *
     * Conditions for updating min_rtt:
     *   - rtt_us < min_rtt_us (new minimum), OR
     *   - filter expired AND not delayed ACK (re-probe the min)
     */
    if (rs->rtt_us >= 0 &&                                                                           /* valid RTT */
        (rtt_clamped < ucp->min_rtt_us ||                                                        /* new minimum OR */
            (filter_expired && !rs->is_ack_delayed))) {                                                   /* expired filter + valid sample */
        rtt_clamped = max_t(u32, rtt_clamped, 1U);                          /* floor at 1 us (kernel clock granularity) */
        if (ucp->min_rtt_us == UCP_MIN_RTT_UNINIT) {
            ucp->min_rtt_us = rtt_clamped;
            ucp->min_rtt_stamp = now;
            goto done_min_rtt;
        }

        if (rtt_clamped < (u64)ucp->min_rtt_us * ucp_minrtt_sticky_num_val /                          /* rtt < min_rtt * sticky_ratio */
            ucp_minrtt_sticky_den_val) {                                                     /* below sticky threshold */
            /*
             * Sticky fall: new RTT is significantly lower than current
             * min_rtt (e.g., 25% lower at 0.75 ratio).  Two sub-cases:
             *   1. Very large drop (> 75%): immediate update (fast fall reset).
             *   2. Moderate drop: count consecutive sticky samples;
             *      after fast_fall_cnt, commit the drop.
             */
            if (rtt_clamped < ucp->min_rtt_us / (u32)ucp_minrtt_fast_fall_div_val) {      /* rtt < min_rtt / div: fast fall */
                ucp->min_rtt_us = rtt_clamped;                                                                 /* immediate update */
                ucp->min_rtt_fast_fall_cnt = 0;                                                                  /* reset fast-fall counter */
            }
            else {                                                                                               /* moderate drop: sticky */
                ucp->min_rtt_fast_fall_cnt = min_t(u32, ucp->min_rtt_fast_fall_cnt + 1, UCP_BITFIELD_2BIT_MAX); /* saturate at 2-bit field max */
                if (ucp->min_rtt_fast_fall_cnt >= ucp_minrtt_fast_fall_cnt_val) {                          /* counter reached threshold */
                    ucp->min_rtt_us = rtt_clamped;                                                                     /* commit the drop */
                    ucp->min_rtt_fast_fall_cnt = 0;                                                                      /* reset counter */
                }
                else {                                                                                                  /* still counting */
                    /* Partial decrease for first sticky sample per round */
                    if (ucp->round_start) {
                        ucp->min_rtt_us = max_t(u32, 1U,
                            (u64)ucp->min_rtt_us *
                            ucp_minrtt_sticky_num_val /
                            ucp_minrtt_sticky_den_val);
                    }
                }
            }
        }
        else {                                                                                                               /* normal update */
            ucp->min_rtt_us = rtt_clamped;                                                                                       /* straightforward min_rtt update */
            ucp->min_rtt_fast_fall_cnt = 0;                                                                                        /* reset fast-fall counter */
        }

        ucp->min_rtt_stamp = now;                                                                                        /* record update time */
    }
    else if (rs->rtt_us >= 0 && !filter_expired &&                                                                /* valid RTT + filter not expired */
        rtt_clamped >= ucp->min_rtt_us) {                                                                                      /* no new minimum: end of fast-fall episode */
        ucp->min_rtt_fast_fall_cnt = 0;                                                                                                /* reset fast-fall counter */
    }

done_min_rtt:
    /* ---- SRTT guard ---- */
    /*
     * If the smoothed RTT (SRTT/8) is anomalously lower than min_rtt_us,
     * it means min_rtt_us has become stale.  Override it with SRTT/8.
     * Guard ratio default: 90% -> SRTT < 90% of min_rtt triggers override.
     * Apply to min_rtt_us regardless of Kalman state — SRTT below
     * min_rtt means our estimate is stale in all cases.
     */
    if (
        tp->srtt_us && ucp->min_rtt_us && ucp->min_rtt_us != UCP_MIN_RTT_UNINIT &&                                                /* SRTT + min_rtt valid + init */
        (tp->srtt_us >> 3) < (u64)ucp->min_rtt_us *                                                                                     /* SRTT/8 < min_rtt * ratio */
        ucp_minrtt_srtt_guard_num_val / ucp_minrtt_srtt_guard_den_val) {                                     /* SRTT guard ratio check */
        ucp->min_rtt_us = tp->srtt_us >= 8 ? tp->srtt_us >> 3 : 1;                                                                                          /* override with smoothed RTT */
        ucp->min_rtt_stamp = now;                                                                                          /* refresh stamp */
    }

    /* ---- PROBE_RTT entry (Cardwell et al. 2016) ---- */
    /*
     * When the PROBE_RTT filter interval expires:
     *   - If not idle-restarting and not already in PROBE_RTT:
     *     Enter PROBE_RTT mode, save cwnd, clear done_stamp.
     * Note: idle_restart flag is cleared on any data delivery
     * (rs->delivered > 0), so an idle-restarted connection
     *     correctly skips PROBE_RTT entry on the first ACK post-restart.
     */
    if (unlikely(filter_expired && !ucp->idle_restart && ucp->mode != UCP_PROBE_RTT)) {                                                        /* interval expired + valid state */
        ucp->mode = UCP_PROBE_RTT;                                                                                                     /* enter PROBE_RTT */
        ucp_save_cwnd(sk);                                                                                                              /* save cwnd for later restore */
        ucp->probe_rtt_done_stamp = 0;                                                                                                   /* clear: stay period not yet started */
    }

    /* ---- PROBE_RTT management ---- */
    if (unlikely(ucp->mode == UCP_PROBE_RTT)) {                                                                                                      /* active PROBE_RTT mode */
        /* app_limited = delivered + inflight; ensures app-limited is nonzero
         * so the pacing engine doesn't think the connection is idle */
        u32 app_limited_val = (u32)((u64)tp->delivered + tcp_packets_in_flight(tp));                                                     /* compute app-limited value */
        tp->app_limited = app_limited_val ? app_limited_val : 1;                                                                              /* set app_limited (never 0) */

        if (!ucp->probe_rtt_done_stamp) {                                                                                                      /* stay period not yet entered */
            if (tcp_packets_in_flight(tp) <= ucp_cwnd_min_target_val ||                                                              /* inflight at min OR */
                ucp->round_start) {                                                                                                              /* round boundary reached */
                /* Inflight has dropped to minimum OR we are at a round boundary.
                 * Start the stay timer (default 200ms). */
                ucp->probe_rtt_done_stamp = now +                                                                                      /* now + stay duration */
                    msecs_to_jiffies(ucp_probe_rtt_mode_ms_val);                                                                       /* convert ms to jiffies */
                ucp->probe_rtt_round_done = 0;                                                                                                     /* clear round done flag */
                ucp->next_rtt_delivered = tp->delivered;                                                                                           /* reset round baseline */
            }
        }
        else {                                                                                                                                     /* in stay period */
            if (ucp->round_start) {                                                                                                                      /* new round boundary */
                ucp->probe_rtt_round_done = 1;                                                                                                           /* mark round done */
            }
            if (ucp->probe_rtt_round_done) {                                                                                                              /* at least one round elapsed */
                ucp_check_probe_rtt_done(sk);                                                                                                              /* check exit conditions */
            }
        }
    }

    /* Clear idle_restart on any data delivery — enables PROBE_RTT entry
     * on next expired filter (the PROBE_RTT entry guard at line 4413 checks
     * !ucp->idle_restart). */
    if (rs->delivered > 0) {                                                                                                                               /* data delivered */
        ucp->idle_restart = 0;                                                                                                                               /* clear idle_restart */
    }

    /* ---- Kalman min-rtt pull-down (Kalman 1960) ---- */
    /*
     * When the Kalman filter has converged (valid x_est and sufficient
     * samples), allow it to pull min_rtt_us DOWN if its estimate is
     * lower than the windowed min.  The windowed min (updated above)
     * provides an upper-bound safety net against Kalman upward drift.
     *
     * Hysteresis: require ucp_minrtt_fast_fall_cnt consecutive Kalman
     * estimates below min_rtt_us before committing the pull-down.
     * On a long-RTT path (212ms) the Kalman gain K ≈ 0.86 produces
     * corrections up to ~8ms per sample from statistical jitter — a
     * single-sample overshoot would permanently lower min_rtt_us
     * (directional update prevents upward correction) and deflate
     * BDP by several percent for up to 10 seconds until the next
     * PROBE_RTT window expiry.
     *
     * Reuses min_rtt_fast_fall_cnt as a shared confirmation counter:
     * both the sliding-window sticky-fall and the Kalman takeover
     * agree that RTT is trending lower — the counter accumulates
     * evidence from both sources and commits when the threshold is
     * reached.  Default threshold = 3 consecutive confirming rounds.
     *
     * Update min_rtt_stamp so the next PROBE_RTT entry is governed
     * by the normal filter_expired window (10s or dynamic interval),
     * not by the age of a pre-takeover stamp.  This prevents premature
     * PROBE_RTT: a lower Kalman estimate improves min_rtt_us without
     * forcing an immediate bandwidth crash.
     *
     * Does NOT apply during PROBE_RTT (we want the raw min in that mode).
     */
    if (ext && ext->x_est && ext->sample_cnt >= ucp_kalman_min_samples_val &&                                                                  /* Kalman converged */
        ucp->mode != UCP_PROBE_RTT) {                                                                                                                          /* not in PROBE_RTT mode */
        u32 krtt = (u32)min_t(u64, ext->x_est >> ucp_kalman_scale_shift_val, U32_MAX);                                                                        /* Kalman RTT estimate */
        if (krtt < ucp->min_rtt_us) {                                                                                                                           /* Kalman lower than windowed min */
            ucp->min_rtt_fast_fall_cnt = min_t(u32,                                                                                                                 /* saturating increment */
                ucp->min_rtt_fast_fall_cnt + 1, UCP_BITFIELD_2BIT_MAX);                                                                                               /* 2-bit ceiling = 3 */
            if (ucp->min_rtt_fast_fall_cnt >= (u32)ucp_minrtt_fast_fall_cnt_val) {                                                                                     /* N consecutive confirming rounds */
                ucp->min_rtt_us = krtt;                                                                                                                             /* commit Kalman pull-down */
                ucp->min_rtt_fast_fall_cnt = 0;                                                                                                                      /* reset counter after commit */
                ucp->min_rtt_stamp = now;                                                                                                                            /* refresh stamp: prevent stale-stamp immediate PROBE_RTT */
                ucp_update_dyn_probe_interval(ext);                                                                                                                  /* recompute dynamic interval */
            }
        }
        else {                                                                                                                                                     /* Kalman estimate NOT lower */
            ucp->min_rtt_fast_fall_cnt = 0;                                                                                                                          /* reset counter: trend broken */
        }
    }
}
/* ---- ACK Aggregation Compensation (Cardwell et al. 2016) ------------ */

/*
 * ucp_update_ack_aggregation - Track extra ACKed data beyond the bandwidth
 * estimate to compensate for ACK aggregation (delayed/stretched ACKs).
 * @sk:  TCP socket.
 * @rs:  rate sample from this ACK.
 * @ext: extended state (ack epoch tracking fields).
 *
 * Algorithm (BBR-style dual-window sliding max):
 *   - Two windows (indices 0 and 1) each spanning approx 5 RTTs.
 *   - Within each window, track the maximum extra_acked value observed.
 *   - The cwnd bonus for ACK aggregation is gain * max(win[0], win[1]).
 *
 * On each ACK:
 *   1. If round_start: increment window RTT counter, rotate windows at 5 RTTs.
 *   2. Compute epoch elapsed time and expected_acked = bw * epoch_us.
 *   3. If expected_acked >= ack_epoch_acked (more expected than received):
 *      reset the epoch (prevents accumulating stale extra_acked).
 *   4. Compute extra_acked = ack_epoch_acked - expected_acked.
 *   5. Update the sliding max in the current window.
 *
 * Epoch reset conditions:
 *   - ack_epoch_acked <= expected_acked (ACKs caught up), OR
 *   - ack_epoch_acked + this_acked >= 1M (epoch cap; prevents overflow).
 */
static void ucp_update_ack_aggregation(struct sock* sk,                                        /* track ACK aggregation */
    const struct rate_sample* rs,                                           /* rate sample */
    struct ucp_ext* ext)                                                    /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                            /* get TCP socket state */
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                               /* get UCP CA state */
    u64 epoch_us; u32 expected_acked, extra_acked;                                                       /* epoch vars */

    if (!ext || !ucp_extra_acked_gain_val) {                                                /* disabled or no ext */
        return; /* early return */
    }

    if (rs->acked_sacked == 0 || rs->delivered < 0 || rs->interval_us == 0) {                                               /* invalid sample */
        return; /* early return */
    }

    /* Window rotation: each window lasts approx 5 RTTs */
    if (ucp->round_start) {                                                                            /* new round boundary */
        ext->extra_acked_win_rtts = min_t(u32, ext->extra_acked_win_rtts + 1, (u32)ucp_extra_acked_win_rtts_max_val);                      /* increment window RTT count */
        if (ext->extra_acked_win_rtts >= (u32)ucp_agg_window_rotation_rtts_val) {                     /* configured RTTs elapsed */
            ext->extra_acked_win_rtts = 0;                                                                 /* reset RTT counter */
            ext->extra_acked_win_idx = ext->extra_acked_win_idx ? 0 : 1;                                    /* rotate to other window */
            ext->extra_acked[ext->extra_acked_win_idx] = 0;                                                  /* clear new window max */
        }
    }

    /* Epoch elapsed time since last reset (us). Guard against negative delta
     * (monotonic clock reorder on some kernels/NIC drivers). */
    epoch_us = max_t(s64, tcp_stamp_us_delta(tp->delivered_mstamp, ext->ack_epoch_mstamp), 0);

    /* Expected ACKed data based on bandwidth estimate and epoch duration */
    {
        u64 bw_val = ucp_bw(sk);
        if (unlikely(epoch_us > U64_MAX / max_t(u64, bw_val, 1ULL))) {
            expected_acked = U32_MAX;
        }
        else {
            expected_acked = (u32)min_t(u64, (bw_val * epoch_us) / BW_UNIT, U32_MAX);
        }
    }

    /*
     * Epoch reset: either we've received less than expected (ACKs caught up),
     * or we're approaching the configured epoch cap (prevents u32 overflow).
     */
    if (ext->ack_epoch_acked <= expected_acked ||                                                              /* ACKs caught up OR */
        ext->ack_epoch_acked >= ucp_ack_epoch_max_val) {               /* epoch cap reached (direct comparison) */
        ext->ack_epoch_acked = 0;                                                                                    /* reset acked counter */
        ext->ack_epoch_mstamp = tp->delivered_mstamp;                                                                 /* start new epoch */
        expected_acked = 0;                                                                                            /* reset expected */
    }

    {
        u64 new_acked = (u64)ext->ack_epoch_acked + rs->acked_sacked;
        ext->ack_epoch_acked = (u32)min_t(u64, ucp_ack_epoch_max_val, new_acked);
    } /* accumulate acked (capped) */

    extra_acked = (ext->ack_epoch_acked > expected_acked) ?
        ext->ack_epoch_acked - expected_acked : 0;                                                               /* excess beyond expected */
    extra_acked = min_t(u32, extra_acked, tp->snd_cwnd);                                                                /* cap at current cwnd */

    /* Sliding max over the current window */
    if (extra_acked > ext->extra_acked[ext->extra_acked_win_idx]) {                                                       /* new window max */
        ext->extra_acked[ext->extra_acked_win_idx] = min_t(u32, extra_acked, U32_MAX);                                  /* store as u32 */
    }
}
/*
 * ucp_ack_aggregation_cwnd - Compute the ACK aggregation cwnd bonus
 * (Cardwell et al. 2016).
 * @sk:  TCP socket.
 * @ext: extended state.
 * @bw:  bandwidth estimate in BW_UNIT (used to compute the max-agg-cwnd cap).
 *
 * Bonus = gain * max(extra_acked[0], extra_acked[1]) / BBR_UNIT.
 * Capped at max_aggr_cwnd = bw * max_ms * 1000 / BW_UNIT (default 100ms worth of data).
 *
 * Returns 0 if aggregation compensation is disabled (gain == 0),
 * full_bw not reached, or ext is NULL.
 */
static u32 ucp_ack_aggregation_cwnd(struct sock* sk, struct ucp_ext* ext, u32 bw)                    /* compute ACK-agg cwnd bonus */
{
    u32 max_aggr_cwnd = 0, aggr_cwnd = 0;                                                          /* max cap and computed bonus */

    if (ucp_extra_acked_gain_val && ucp_full_bw_reached(sk) && ext) {                   /* enabled + full_bw + ext valid */
        { /* saturating multiply: bw * max_ms * USEC_PER_MSEC */
            u64 max_ms = (u64)ucp_extra_acked_max_ms_val * USEC_PER_MSEC;
            u64 product;
            if (max_ms == 0 || bw > U64_MAX / max_ms) {
                product = U64_MAX;
            }
            else {
                product = bw * max_ms;
            }

            max_aggr_cwnd = (u32)min_t(u64, product >> BW_SCALE, U32_MAX);
        }

        {
            u64 aggr64 = ((u64)ucp_extra_acked_gain_val *
                max_t(u32, ext->extra_acked[0], ext->extra_acked[1])) >> BBR_SCALE;
            aggr_cwnd = (u32)min_t(u64, aggr64, max_aggr_cwnd);
        }
    }
    return aggr_cwnd;                                                                                      /* return bonus (0 if disabled) */
}

/* ---- ACK Aggregation Confidence-based Compensation --------------------
 *
 * BBRplus-inspired enhancement: uses extra_acked as a signal-quality
 * indicator for the Kalman filter rather than a direct cwnd adder.
 *
 * Five modules:
 *   1. ucp_measure_ack_aggregation: compute extra_acked estimate
 *   2. ucp_evaluate_agg_confidence: score 0..1024 based on 4 factors
 *   3. ucp_agg_cwnd_compensation: safe cwnd bonus with safety valve
 *   4. ucp_agg_safety_check: four-guard validation before compensation
 *   5. ucp_agg_watchdog: demote confidence after N RTTs + decay max
 */

 /*
  * ucp_measure_ack_aggregation - Compute the excess ACKed data beyond
  * the bandwidth expectation.  Returns extra segments (not bytes).
  */
static u32 ucp_measure_ack_aggregation(struct sock* sk, const struct rate_sample* rs, /* socket + rate sample */
    struct ucp_ext* ext)                                              /* extended state for agg tracking */
{
    struct tcp_sock* tp = tcp_sk(sk);
    u32 expected_acked, extra;
    u32 cur_bw;

    if (!ext || rs->interval_us == 0) {
        return 0;
    }

    cur_bw = ucp_bw(sk);

    /* expected_acked = bw * interval_us / BW_UNIT (segments) */
    expected_acked = (u32)(((u64)cur_bw * rs->interval_us) >> BW_SCALE);

    if (rs->acked_sacked > expected_acked) {
        extra = rs->acked_sacked - expected_acked;
    }
    else {
        extra = 0;
    }

    /* Cap 1: not more than current cwnd */
    extra = min_t(u32, extra, tcp_snd_cwnd(tp));

    /* Cap 2: not more than bw * window_ms worth of data */
    {
        u64 max_ms2 = (u64)ucp_agg_max_window_ms_val * USEC_PER_MSEC;
        u64 bw_prod;
        u64 bw_cap;
        if (max_ms2 == 0 || (u64)cur_bw > U64_MAX / max_ms2) {
            bw_prod = U64_MAX;
        }
        else {
            bw_prod = (u64)cur_bw * max_ms2;
        }

        bw_cap = bw_prod >> BW_SCALE;
        extra = min_t(u32, extra, (u32)min_t(u64, bw_cap, U32_MAX));
    }

    /* Update dual-slot windowed maximum */
    if (extra > ext->agg_extra_acked_max) {
        ext->agg_extra_acked_max = extra;
    }

    ext->agg_extra_acked = extra;
    return extra;
}

/*
 * ucp_evaluate_agg_confidence - Score the trustworthiness of the current
 * extra_acked signal on a 0..1024 scale using four orthogonal factors,
 * each contributing 256 points.  Any single false signal cannot reach
 * CONFIRMED (512) alone.
 */
static u16 ucp_evaluate_agg_confidence(struct sock* sk, struct ucp_ext* ext, /* CA state + extended state */
    u32 extra_acked)                                                    /* current ACK's extra_acked estimate */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);
    u16 conf = 0;

    if (!ext) {
        return 0;
    }

    /* Factor 1: Kalman filter converged (estimate is reliable). Also requires
     * minimum sample count to avoid scoring before the filter has meaningful data. */
    if (ext->p_est < ucp_kalman_converged_p_est_val &&
        ext->sample_cnt >= ucp_kalman_min_samples_val) {
        conf += (u16)ucp_agg_factor_weight_val;  /* + configured weight */
    }

    /* Factor 2: No loss signal (no real congestion) */
    if (inet_csk(sk)->icsk_ca_state < TCP_CA_Recovery) {
        conf += (u16)ucp_agg_factor_weight_val;  /* + configured weight */
    }

    /* Factor 3: No sustained queue delay (x_est near min_rtt).
     * Requires valid Kalman state — cold start scores 0, not a free pass. */
    if (ext->x_est > 0 && ucp->min_rtt_us != UCP_MIN_RTT_UNINIT && ucp->min_rtt_us > 0) {
        u32 est_rtt = ext->x_est >> ucp_kalman_scale_shift_val;
        if (est_rtt <= ucp->min_rtt_us + (u32)ucp_agg_factor3_qdelay_us_val) {  /* within configurable margin */
            conf += (u16)ucp_agg_factor_weight_val;  /* + configured weight */
        }
    }
    /* No else: cold start with no estimate scores 0 for this factor */

    /* Factor 4: extra_acked magnitude check vs history (not a transient spike) */
    if (extra_acked == 0 || ext->agg_extra_acked_max == 0 ||
        (u64)extra_acked * (u64)ucp_agg_factor4_ratio_den_val <=
        (u64)ext->agg_extra_acked_max * (u64)ucp_agg_factor4_ratio_num_val) {
        conf += (u16)ucp_agg_factor_weight_val;  /* +configured weight, within configurable ratio of windowed max */
    }

    return conf;  /* 0..1024 */
}

/*
 * ucp_agg_state_from_confidence - Map confidence score to state enum.
 */
static u8 ucp_agg_state_from_confidence(u16 confidence)                 /* confidence score 0..1024 */
{
    if (confidence >= (u16)ucp_agg_thresh_trusted_val) {
        return UCP_AGG_TRUSTED;
    }

    if (confidence >= (u16)ucp_agg_thresh_confirmed_val) {
        return UCP_AGG_CONFIRMED;
    }

    if (confidence >= (u16)ucp_agg_thresh_suspected_val) {
        return UCP_AGG_SUSPECTED;
    }

    return UCP_AGG_IDLE;
}

/*
 * ucp_agg_safety_check - Four-guard validation before cwnd compensation.
 * Returns true if compensation is safe.
 */
static bool ucp_agg_safety_check(struct sock* sk, struct ucp_ext* ext, u32 bw) /* CA state + ext state + bw (BW_UNIT) */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);
    struct tcp_sock* tp = tcp_sk(sk);
    u32 safe_cwnd;
    u64 bdp_est;

    if (!ext) {
        return false;
    }

    /* Guard 1: Queue delay rising? Skip if Kalman cold (x_est == 0). */
    if (ucp->min_rtt_us != UCP_MIN_RTT_UNINIT && ucp->min_rtt_us > 0 && ext->x_est > 0) {
        u32 est_rtt = ext->x_est >> ucp_kalman_scale_shift_val;
        if ((u64)est_rtt > (u64)ucp->min_rtt_us + (u64)ucp_agg_safety_qdelay_us_val) {  /* >configurable margin */
            return false;  /* queue building, stop compensation */
        }
    }

    /* Guard 2: In loss recovery? */
    if (inet_csk(sk)->icsk_ca_state >= TCP_CA_Recovery) {
        return false;
    }

    /* Guard 3: CWND already > N x BDP?  (hard ceiling) */
    if (ucp->min_rtt_us == UCP_MIN_RTT_UNINIT || ucp->min_rtt_us == 0) {                              /* guard: min_rtt not yet measured */
        return false;
    }

    bdp_est = ((u64)bw * ucp->min_rtt_us) >> BW_SCALE;                /* u64 cast prevents overflow */
    safe_cwnd = (u32)min_t(u64, bdp_est * ucp_agg_safety_bdp_mult_val, U32_MAX);        /* u64 for safety against large BDP */
    if (tp->snd_cwnd >= safe_cwnd) {
        return false;
    }

    /* Guard 4: Inflight already excessive? */
    if (tcp_packets_in_flight(tp) >= (u64)safe_cwnd + ucp_tso_segs_goal(sk)) {
        return false;
    }

    return true;
}

/*
 * ucp_agg_cwnd_compensation - Compute safe cwnd bonus from aggregation signal.
 * Five-layer safety: confidence gate -> safety check -> progressive scaling
 * -> hard cap at BDP/2 -> watchdog timer.
 * Returns extra cwnd segments to add (0 = no compensation).
 */
static u32 ucp_agg_cwnd_compensation(struct sock* sk, struct ucp_ext* ext, /* socket + extended state */
    u32 extra_acked, u16 confidence, u32 bw)                            /* current extra_acked + confidence score + bw (BW_UNIT) */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);
    u32 comp = 0, agg_est = 0, bdp = 0;
    int thr;

    if (!ext || !ucp_agg_enable_val) {
        return 0;
    }

    /* Single cached read of threshold for both gate and computation. */
    thr = ucp_agg_confidence_thresh_val;                    /* dynamic threshold (clamped cache) */

    /* Layer 1: Confidence must reach CONFIRMED (512) */
    if (confidence < (u16)thr) {
        return 0;
    }

    /* Layer 2: Safety check must pass */
    if (!ucp_agg_safety_check(sk, ext, bw)) {
        return 0;
    }

    /* Layer 3: Progressive scaling: maps [threshold, confidence_max] → [0, agg_est].
     * Uses the configured threshold (not hardcoded 512) for both gating
     * and scaling range.  Denominator is (confidence_max - threshold) with div-by-zero
     * guard for threshold ≥ confidence_max. */
    agg_est = max_t(u32, extra_acked, ext->agg_extra_acked_max);
    {
        u32 conf_max = (u32)ucp_agg_confidence_max_val;          /* configured max confidence range */
        if (likely(thr < (int)conf_max)) {                                   /* threshold in valid range */
            comp = (u32)((u64)agg_est * (u32)(confidence - thr) / (conf_max - (u32)thr)); /* proportional: [thr,conf_max] → [0,agg_est] */
        }
        else {
            comp = 0;
        }
    }

    /* Layer 4: Hard cap at max_comp_ratio % of BDP */
    {
        u64 bdp64 = ((u64)bw * ucp->min_rtt_us) >> BW_SCALE;
        bdp = (u32)min_t(u64, bdp64, U32_MAX);
    }                      /* u64 cast prevents overflow */

    {
        u32 max_comp = (u32)((u64)bdp * (u32)ucp_agg_max_comp_ratio_val / UCP_PCT_BASE);  /* u64 for safety */
        comp = min_t(u32, comp, max_comp);
    }

    return comp;
}

/*
 * ucp_agg_watchdog - Demote confidence if compensation persists too long.
 * Called at round boundaries only (ucp->round_start == 1).
 * Also decays agg_extra_acked_max to prevent one spike from permanently
 * boosting confidence via Factor 4.
 * Does not return (state is modified in-place).
 */
static void ucp_agg_watchdog(struct sock* sk, struct ucp_ext* ext)
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);
    int max_dur;

    if (!ext || !ucp_agg_enable_val) {                            /* disabled or no ext */
        return;
    }

    /* Per-ACK gentle decay of windowed max: prevents a transient spike
     * (e.g. sudden burst) from inflating Factor 4 for an entire RTT
     * round.  Decays at value/denominator per ACK (both configurable).
     * Default 128/128 = 1.0 (no per-ACK decay). */
    {
        u32 per_ack = (u32)ucp_agg_max_per_ack_decay_val;
        u32 per_ack_den = (u32)ucp_agg_max_per_ack_decay_den_val;
        if (per_ack < per_ack_den && per_ack_den > 0) {
            ext->agg_extra_acked_max = (u32)((u64)ext->agg_extra_acked_max * per_ack / per_ack_den);
        }
    }

    if (!ucp->round_start) {                                                  /* only act on round boundaries */
        return;
    }

    /* Decay windowed max: 25% reduction per RTT to expire stale peaks */
    {
        u32 pct = (u32)ucp_agg_max_decay_pct_val;
        ext->agg_extra_acked_max = (u32)((u64)ext->agg_extra_acked_max * pct / UCP_PCT_BASE); /* u64 cast prevents overflow */
    }

    max_dur = ucp_agg_max_comp_duration_val;

    if (ext->agg_state >= UCP_AGG_CONFIRMED) {
        if (ext->agg_comp_duration < U8_MAX) {
            ext->agg_comp_duration++;
        }

        if ((u32)ext->agg_comp_duration > (u32)max_dur) {
            /* Demote: may be undetected congestion */
            ext->agg_state = UCP_AGG_SUSPECTED;
            ext->agg_comp_duration = 0;
        }
    }
    else {
        ext->agg_comp_duration = 0;
    }
}

/* ---- Model Update Pipeline (Cardwell et al. 2016) -------------------- */

/*
 * ucp_update_model - Execute the full per-ACK model update pipeline.
 * @sk:  TCP socket.
 * @rs:  rate sample from the current ACK.
 * @ext: extended state (may be NULL).
 *
 * Processing order (reflects data dependencies):
 *   1. Bandwidth update (sliding-window max).
 *   2. ECN-CE EWMA update (RFC 3168).
 *   3. ACK aggregation tracking.
 *   4. Cycle phase advance check (PROBE_BW only).
 *   5. Full BW reached detection.
 *   6. Drain check (STARTUP -> DRAIN -> PROBE_BW transition).
 *   7. Min RTT update (includes Kalman filter, traditional window, PROBE_RTT).
 *   8. Set pacing_gain and cwnd_gain based on current mode:
 *      - STARTUP:   high_gain / high_gain.
 *      - DRAIN:     drain_gain / high_gain (cwnd stays aggressive during drain).
 *      - PROBE_BW:  cycle_gain (or BBR_UNIT if LT BW active) / cwnd_gain_val.
 *      - PROBE_RTT: BBR_UNIT / BBR_UNIT (cruise, min inflight).
 */
static void ucp_update_model(struct sock* sk, const struct rate_sample* rs,            /* per-ACK model pipeline */
    struct ucp_ext* ext)                                                    /* extended state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                       /* get UCP CA state */

    ucp_update_bw(sk, rs, ext);                                                              /* 1. sliding-window max bw */
    ucp_update_ecn_ewma(sk, rs, ext);                                                         /* 2b. ECN-CE mark ratio EWMA (RFC 3168) */
    ucp_update_ack_aggregation(sk, rs, ext);                                                   /* 3. ACK agg tracking */
    ucp_update_cycle_phase(sk, rs, ext);                                                        /* 4. PROBE_BW phase advance */
    ucp_check_full_bw_reached(sk, rs);                                                           /* 5. pipe-full detection */
    ucp_check_drain(sk, rs, ext);                                                                /* 6. drain transitions */
    ucp_update_min_rtt(sk, rs, ext);                                                             /* 7. min-RTT + Kalman + PROBE_RTT */

    /* Mode-specific gain assignment (Cardwell et al. 2016) */
    switch (ucp->mode) {                                                                          /* dispatch based on FSM mode */
    case UCP_STARTUP:                                                                          /* STARTUP mode */
        ucp->pacing_gain = ucp_high_gain_val;                                        /* pacing_gain approx 2.89x */
        ucp->cwnd_gain = ucp_high_gain_val;                                        /* cwnd_gain approx 2.89x */
        break;                                                                                   /* exit switch */
    case UCP_DRAIN:                                                                              /* DRAIN mode */
        ucp->pacing_gain = ucp_drain_gain_val;                                        /* pacing_gain approx 0.35x */
        ucp->cwnd_gain = ucp_high_gain_val;                                            /* cwnd at high_gain to keep cwnd (match BBR, Cardwell et al. 2016) */
        break;                                                                                     /* exit switch */
    case UCP_PROBE_BW:                                                                             /* PROBE_BW mode */
        /* After PROBE_RTT exit, override with high_gain for one ACK to quickly
         * refill the pipe.  BBRv1 preserves pacing_gain across the transition;
         * UCP must explicitly check probe_rtt_restored because ucp_reset_mode()
         * already set a random cycle phase and ucp_get_cycle_pacing_gain() would
         * return the cycle gain (<= 1.25x), causing slow refill. */
        if (unlikely(ucp->probe_rtt_restored)) {                                                                 /* just exited PROBE_RTT: rare */
            ucp->pacing_gain = ucp_high_gain_val;                                            /* refill at high_gain (2.89x) */
        }
        else if (ucp->lt_use_bw) {
            /* Cycle pacing through normal probe/drain/cruise phases
             * like BBR does in LT BW mode.  ucp_get_cycle_pacing_gain()
             * applies decay only when explicitly enabled per-phase via
             * the decay mask.  ucp_lt_bw_probe_pct amplifies probe
             * amplitude on ALL phases (drain phases < 1.0x remain
             * below cruise). */
            ucp->pacing_gain = ucp_get_cycle_pacing_gain(sk, ext);
            if (ucp_lt_bw_probe_pct_val > 0) {
                u32 probe_pct = ucp_lt_bw_probe_pct_val;                  /* base bonus (default 10%) */
                u32 ramp = min_t(u32, probe_pct, ucp->lt_rtt_cnt / UCP_LT_BW_PROBE_RAMP_RTTS);  /* +1% per N RTT, cap at 2x base */
                probe_pct += ramp;                                        /* adapt to sustained LT BW duration */
                ucp->pacing_gain = min_t(u32, ucp->pacing_gain * (UCP_PCT_BASE + probe_pct) / UCP_PCT_BASE, UCP_GAIN_MAX);
            }
        }
        else {                                                                                          /* normal PROBE_BW */
            if (likely(ext)) {
                ucp->pacing_gain = ucp_get_cycle_pacing_gain(sk, ext);                                   /* cycle gain with decay */
            }
            else {
                u32 idx = (u32)ucp->cycle_idx & (ucp_probe_bw_cycle_len_val - 1);           /* no ext: read table directly */
                ucp->pacing_gain = ucp_cycle_gain_table[idx];                              /* base gain, no decay possible */
            }
        }

        ucp->cwnd_gain = ucp_cwnd_gain_val;                                                /* baseline 2x */
        break;                                                                                           /* exit switch */
    case UCP_PROBE_RTT:                                                                                    /* PROBE_RTT mode */
        ucp->pacing_gain = BBR_UNIT;                                                                       /* cruise at 1.0x */
        ucp->cwnd_gain = BBR_UNIT;                                                                       /* cruise at 1.0x */
        break;                                                                                              /* exit switch */
    }

    /* Re-evaluate single-flow heuristic after all stats are fresh */
    ucp_alone_on_path_eval(sk, ext);
}
/*
 * ucp_alone_on_path_eval - Detect single-flow scenario for BBR-pure bypass.
 * @sk:  TCP socket.
 * @ext: extended state (for qdelay, jitter, ECN, agg state).
 *
 * Runs once per round boundary.  When all queues are nearly empty,
 * no ECN marks exist, and no ACK aggregation is confirmed, the flow
 * is likely alone on the bottleneck.  In this scenario, UCP's
 * protective mechanisms (Kalman model_rtt positive bias, ECN backoff)
 * reduce single-flow throughput compared to BBR.
 *
 * When alone_on_path is set:
 *   - ucp_get_model_rtt returns ucp->min_rtt_us (BBR-style exact min),
 *     bypassing the Kalman smoothed estimate which has a small positive
 *     bias from one-sided measurement noise.
 *   - ucp_ecn_backoff returns immediately (no ECN reaction needed).
 *
 * The flag is cleared immediately when any queue, loss, ECN, or
 * aggregation signal appears — restoring full UCP protection.
 */
static void ucp_alone_on_path_eval(struct sock* sk,                            /* evaluate single-flow heuristic */
    struct ucp_ext* ext)                                                       /* extended state */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                            /* UCP CA state */

    /* Only re-evaluate on round boundaries for hysteresis.
     * Per-ACK evaluation would cause noise in the confirmation
     * counter and lead to oscillating back and forth between
     * normal and single-flow mode within a single RTT.  Round-boundary
     * evaluation provides ~1 RTT of natural hysteresis. */
    if (!ucp->round_start) {                                                    /* not a round boundary */
        return;                                                                 /* keep current state */
    }

    /* Without extended state (allocation failure at init), we cannot
     * evaluate the required signals (qdelay, jitter, ECN, agg).
     * Fall back to not-alone — safe, preserves UCP protection.
     * Clear the flag directly here; the counter lives in ext so
     * there is nothing to reset when ext is NULL. */
    if (!ext) {                                                                  /* no extended state available */
        ucp->alone_on_path = 0;                                                   /* exit single-flow mode */
        return;                                                                    /* no counter to reset */
    }

    /*
     * Single-flow indicators (six orthogonal signals):
     *
     * 0. sample_cnt >= ucp_kalman_min_samples_val:
     *    Kalman filter must have converged before we can trust
     *    qdelay_avg and jitter_ewma as meaningful queue signals.
     *    During early startup, these values are a random walk —
     *    acting on them would produce false positives.
     *
     * 1. qdelay_avg < ucp_alone_qdelay_thresh_us_val (default 1000 us):
     *    Queue must be nearly empty (< 1 ms by default).  BBR-style
     *    1 % pacing margin plus one TSO burst create ~1 - 2 ms of
     *    queue on a loaded path; a sub-millisecond queue means there
     *    is no competing bulk traffic.
     *
     * 2. jitter_ewma < ucp_alone_jitter_thresh_us_val (default 2000 us):
     *    Low packet - timing variance (< 2 ms by default).  Competing
     *    flows induce inter - packet gaps that push jitter well above
     *    this threshold.  A quiet single - flow path shows only
     *    ACK - clock micro - jitter.
     *
     * 3. ecn_ewma == 0:
     *    Zero congestion marks.  Any AQM(CoDel, FQ - CoDel, RED)
     *    marks packets when queue exceeds the marking threshold.
     *    Absence of marks implies an empty bottleneck buffer.
     *
     * 4. lt_use_bw == 0:
     *    Not in policer - detected rate - limited mode.  When LT BW
     *    is active, the path rate is constrained by a policer —
     *    we are effectively competing with a fixed - rate limiter
     *    and should maintain UCP's protective LT probe behavior.
     *
     * 5. agg_state <= max per ucp_alone_agg_state_level_val:
     *    Configurable ACK aggregation strictness for alone detection.
     *    TCP delayed-ACK produces natural SUSPECTED-state aggregation
     *    even on a quiet single-flow path — requiring IDLE blocks
     *    alone mode in practice.  Three levels via sysctl:
     *      0 = IDLE only      (strict: zero aggregation)
     *      1 = ≤ SUSPECTED    (moderate: allow transient agg; default)
     *      2 = ≤ CONFIRMED    (permissive: block only trusted/persistent agg)
     *    Because CONFIRMED=2 and SUSPECTED=1 in UCP's enum, level 1
     *    and <CONFIRMED are equivalent; level 2 uses <TRUSTED for a
     *    genuinely more permissive tier.
     *
     * Entry: requires N consecutive qualifying rounds (hysteresis).
     * The confirmation counter increments once per round boundary
     * when all six conditions hold.  This prevents oscillation
     * during brief quiet windows in multi-flow competition —
     * "conservative to accelerate".
     *
     * Exit: immediate — any single qualification failure clears
     * the flag AND resets the counter.  Full UCP protection
     * (Kalman, ECN backoff, gain decay) re-engages on the first
     * sign of competition — "aggressive to brake".
     */
    {
        /* Map alone_agg_state_level to max allowed agg_state for comparison.
         * Level 0 = IDLE(0) only, Level 1 = ≤ SUSPECTED(1), Level 2 = ≤ CONFIRMED(2). */
        u8 max_agg;
        switch (ucp_alone_agg_state_level_val) {
        case 0: max_agg = UCP_AGG_IDLE; break;       /* strict: zero aggregation */
        case 2: max_agg = UCP_AGG_CONFIRMED; break;  /* permissive: allow up to CONFIRMED */
        default: max_agg = UCP_AGG_SUSPECTED; break; /* moderate: allow SUSPECTED (default) */
        }
        if (ext->sample_cnt >= ucp_kalman_min_samples_val &&                            /* Kalman must be converged */
            ext->qdelay_avg < (u32)ucp_alone_qdelay_thresh_us_val &&                   /* queue below configurable threshold */
            ext->jitter_ewma < (u32)ucp_alone_jitter_thresh_us_val &&                  /* jitter below configurable threshold */
            ext->ecn_ewma == 0 &&                                                       /* no ECN marks from AQM */
            (ucp_alone_bypass_lt_bw_val || ucp->lt_use_bw == 0) &&                      /* LT BW: configurable bypass (default skip) */
            ext->agg_state <= max_agg) {                                                /* configurable agg strictness */
            /* All six conditions hold on this round boundary.
             * Increment the consecutive confirmation counter.
             * Wrap at 255 (u8 max) to prevent overflow on connections
             * running millions of rounds in single-flow mode — the
             * counter only needs to reach confirm_rounds_val (≤ 32). */
            if (ext->alone_confirm_cnt < 255) {                                      /* guard against u8 wrap */
                ext->alone_confirm_cnt++;                                             /* increment counter */
            }
            if (ext->alone_confirm_cnt >= (u8)ucp_alone_confirm_rounds_val) {        /* N rounds satisfied */
                ucp->alone_on_path = 1;                                               /* activate single-flow mode */
            }
        }
        else {
            /* At least one signal indicates competition or path stress.
             * Clear the flag IMMEDIATELY (aggressive brake) and reset
             * the confirmation counter so re-entry requires another
             * full N rounds of clean conditions. */
            ucp->alone_on_path = 0;                                                   /* exit single-flow mode */
            ext->alone_confirm_cnt = 0;                                               /* reset counter for re-entry */
        }
    }
}
/* ---- Main Per-ACK Entry Point ----------------------------------------- */

/*
 * ucp_main - Main congestion control callback invoked on each ACK.
 * @sk:  TCP socket.
 * @rs:  rate sample (delivered, interval_us, rtt_us, losses, etc.).
 * @ack: (kernel 6.10+) ACK number.
 * @flags: (kernel 6.10+) ACK flags.
 *
 * Processing sequence:
 *   1. Retrieve extended state (may be NULL).
 *   2. Run ucp_update_model (bandwidth/RTT/loss/Kalman/gain updates).
 *   3. Apply cwnd constraints (STARTUP loss-gate, Kalman qdelay reduction).
 *   4. Set pacing rate using current bw and pacing_gain.
 *   5. Set cwnd using current bw and cwnd_gain.
 *
 * This function reads global module-parameter caches (e.g. ucp_cwnd_gain_val,
 * ucp_kalman_q_val) without READ_ONCE.  This is deliberate:
 *   - A stale value affects at most one ACK; the next ACK corrects it.
 *   - All kernel CC modules (BBR, CUBIC, Westwood, etc.) do the same.
 *   - See "CONCURRENCY & SAFETY MODEL" at struct ucp_ext for the full
 *     justification.
 *
 * This is the single entry point for all UCP per-ACK processing.
 * The function is marked UCP_KFUNC for BPF struct_ops compatibility.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)                                           /* kernel 6.10+ signature */
UCP_KFUNC void ucp_main(struct sock* sk, u32 ack __maybe_unused, int flags __maybe_unused, const struct rate_sample* rs) /* main ACK handler (6.10+) */
#else                                                                                           /* pre-6.10 signature */
UCP_KFUNC void ucp_main(struct sock* sk, const struct rate_sample* rs)                    /* main ACK handler (legacy) */
#endif
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                               /* get UCP CA state */
    struct ucp_ext* ext;                                                                             /* extended state (may be NULL) */
    u32 bw;                                                                                          /* active bandwidth estimate */

    ext = ucp_ext_get(sk);                                                                           /* retrieve ext (with UAF guard) */

    /* ACK aggregation confidence — must run BEFORE ucp_update_model
     * so that the Kalman filter sees fresh agg_confidence on the same
     * ACK where aggregation is first detected.  Previously evaluated
     * after the model update, which caused a 1-ACK lag: the aggregate
     * RTT sample polluted x_est on the detection ACK before R was
     * raised to compensate. */
    if (likely(ucp_agg_enable_val && ext)) {                                                     /* common case: agg enabled + ext valid */
        u32 extra = ucp_measure_ack_aggregation(sk, rs, ext);
        u16 conf = ucp_evaluate_agg_confidence(sk, ext, extra);
        ext->agg_confidence = conf;
        ext->agg_state = ucp_agg_state_from_confidence(conf);

        /* Watchdog runs AFTER confidence evaluation: demotions persist
         * until the next ACK, preventing the immediate re-promotion bug
         * where confidence scoring overwrites watchdog demotion.
         * Gated: round boundaries always, otherwise only if per-ACK
         * max decay is active (default disabled). */
        if (ucp->round_start || ucp_agg_per_ack_decay_active) {
            ucp_agg_watchdog(sk, ext);
        }
    }

    ucp_update_model(sk, rs, ext);                                                                   /* full per-ACK model update */

    ucp_apply_cwnd_constraints(sk, ext);                                                              /* loss/qdelay-based cap on cwnd_gain */

    bw = ucp_bw(sk);                                                                                   /* active bw estimate (max_bw or lt_bw) */
    ucp_set_pacing_rate(sk, bw, ucp->pacing_gain);                                                       /* update sk_pacing_rate */

    ucp_set_cwnd(sk, rs, rs->acked_sacked,                                                           /* update tp->snd_cwnd: acked_sacked is u32, always >= 0 */
        bw, ucp->cwnd_gain, ext);                                                                      /* using bw, cwnd_gain, ext */
}
/* ---- Module Callbacks -------------------------------------------------- */

/*
 * ucp_init - Initialize per-connection UCP state when a connection starts
 * using the "ucp" congestion-control algorithm.
 * @sk: TCP socket.
 *
 * Steps:
 *   1. Zero-initialize the struct ucp (ICSK_CA_PRIV slot).
 *   2. Set prev_ca_state = TCP_CA_Open.
 *   3. Bootstrap min_rtt_us from the TCP stack's recorded min RTT.
 *   4. Set min_rtt_stamp to now.
 *   5. Initialize pacing rate from cwnd and SRTT.
 *   6. Enable pacing on the socket.
 *   7. Allocate and initialize extended state (struct ucp_ext) on the heap.
 *      On allocation failure, UCP runs without Kalman/ACK-agg features
 *      (fallback to sliding-window-only min_rtt).
 */
UCP_KFUNC void ucp_init(struct sock* sk)                                             /* per-connection init callback */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                           /* get CA private area */
    struct ucp_ext* ext;                                                                         /* extended state pointer */

    memset(ucp, 0, sizeof(*ucp));                                                                /* zero the CA private area */
    /* Match BBR: set snd_ssthresh to TCP_INFINITE_SSTHRESH so the TCP stack
     * never imposes its own ssthresh-based cwnd clamp.  UCP manages cwnd
     * entirely through its own state machine (STARTUP/DRAIN/PROBE_BW).
     * Without this, memset zeros ssthresh → 0, which can prematurely
     * limit cwnd in several TCP stack code paths. */
    WRITE_ONCE(tcp_sk(sk)->snd_ssthresh, TCP_INFINITE_SSTHRESH);
    ucp->prev_ca_state = TCP_CA_Open;                                                             /* initial CA state: Open */
    ucp->next_rtt_delivered = tcp_sk(sk)->delivered;                                                /* initial round-trip baseline: match BBR, Cardwell et al. 2016 */
    /* Bootstrap min_rtt_us from TCP stack's 3WHS measurement, matching BBR's
     * bbr->min_rtt_us = tcp_min_rtt(tp).  Without this, ucp_bdp() returns
     * TCP_INIT_CWND (~10) until the first RTT sample arrives, starving cwnd. */
    ucp->min_rtt_us = tcp_min_rtt(tcp_sk(sk));                                     /* use stack's handshake RTT measurement */
    ucp->min_rtt_stamp = tcp_jiffies32;                                                            /* set initial min_rtt timestamp */

    ucp_init_pacing_rate_from_rtt(sk);                                                              /* initial pacing rate from RTT+cwnd */
    cmpxchg(&sk->sk_pacing_status, SK_PACING_NONE, SK_PACING_NEEDED);                       /* enable pacing, preserve if already set */

    ext = kzalloc(sizeof(*ext), GFP_KERNEL);                                                          /* allocate extended state block (kzalloc zeroes memory) */
    if (likely(ext)) {                                                                                  /* allocation succeeded */

        ext->p_est = ucp_kalman_p_est_init_val;                                                 /* initialize Kalman covariance (Kalman 1960) */
        ext->q_est = (u32)ucp_kalman_q_val;                                                       /* init Q estimate to base process noise */
        ext->r_est = (u32)ucp_kalman_r_val;                                                       /* init R estimate to base measurement noise */
        ext->ecn_ewma = 0;                                                                                   /* init ECN EWMA: no CE marks yet */
        ext->last_delivered_ce = tcp_sk(sk)->delivered_ce;                                                    /* snapshot initial CE counter */
        ext->ack_epoch_mstamp = tcp_sk(sk)->tcp_mstamp;                                                     /* start aggregation epoch timestamp */
        ext->agg_extra_acked = 0;
        ext->agg_extra_acked_max = 0;
        ext->agg_confidence = 0;
        ext->agg_state = UCP_AGG_IDLE;
        ext->agg_comp_duration = 0;
        ext->agg_r_scaled = ucp_agg_r_multiplier_min_val;                 /* start at configured R floor */
        ext->x_est = 0;                             /* kalman cold start sentinel */
        ext->sample_cnt = 0;                         /* no accepted samples yet */
        ext->consec_reject_cnt = 0;                  /* no rejections yet */
        ext->dyn_probe_rtt_interval_jiffies = 0;      /* disabled until kalman converges */
        ucp->ext = ext;
    }
    else {                                                                                                   /* allocation failed */
        pr_warn_once("UCP: ext alloc failed, advanced features disabled\n");                                     /* warn: running degraded */
    }
}
/*
 * ucp_sndbuf_expand - Return the factor by which the socket send buffer
 * should be expanded relative to cwnd.
 * @sk: TCP socket.
 *
 * Returns the configurable sndbuf expansion factor (default 3× cwnd, BBR standard).
 * This provides enough buffer for pacing without head-of-line blocking.
 */
UCP_KFUNC u32 ucp_sndbuf_expand(struct sock* sk)                                       /* send buffer expansion factor */
{
    return (u32)ucp_sndbuf_expand_factor_val;                                  /* configurable sndbuf expansion factor */
}
/*
 * ucp_undo_cwnd - Handle a TCP undo operation (spurious loss detection).
 * @sk: TCP socket.
 *
 * Resets full_bw detection state and LT BW sampling, then returns the
 * current cwnd (the stack will decide the actual undo).
 */
UCP_KFUNC u32 ucp_undo_cwnd(struct sock* sk)                                             /* handle spurious loss undo */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                               /* get UCP CA state */
    /* Match BBR: reset only full_bw and full_bw_cnt, NOT full_bw_reached.
     * BBR's rationale (Cardwell et al. 2016): once the pipe is known to be
     * full, a spurious loss detection (e.g., reordering misinterpreted as
     * loss) does NOT change the pipe capacity.  Clearing full_bw_reached
     * forces a re-entry into STARTUP mode, causing massive overshoot and
     * unnecessary queue buildup after each spurious undo event. */
    ucp->full_bw = 0;                                                                                /* reset full_bw estimate */
    ucp->full_bw_cnt = 0;                                                                            /* reset full_bw counter */
    ucp_reset_lt_bw_sampling(sk);                                                                      /* clear LT BW state */
    return tcp_snd_cwnd(tcp_sk(sk));                                                                        /* return current cwnd */
}
/*
 * ucp_ssthresh - Return the slow-start threshold after a loss event.
 * @sk: TCP socket.
 *
 * Saves cwnd for later restoration via ucp_save_cwnd(), then returns
 * the current ssthresh (UCP does not modify ssthresh on its own;
 * the TCP stack uses the current value).
 */
UCP_KFUNC u32 ucp_ssthresh(struct sock* sk)                                               /* ssthresh query after loss */
{
    ucp_save_cwnd(sk);                                                                              /* save cwnd for later restore */
    return tcp_sk(sk)->snd_ssthresh;                                                                 /* return current ssthresh */
}
/* ---- Diagnostic Encoding (standard BBR format) ----------------------- */

/*
 * ucp_get_info - Encode UCP state for diagnostic tools (e.g., ss -i).
 * @sk:       TCP socket.
 * @ext_mask: INET_DIAG extension bitmask.
 * @attr:     [out] diagnostic attribute type (INET_DIAG_BBRINFO).
 * @info:     [out] union tcp_cc_info to fill.
 *
 * Outputs a struct tcp_bbr_info compatible with standard BBR diagnostics
 * (Cardwell et al. 2016):
 *   - bbr_bw_lo / bbr_bw_hi: 64-bit bandwidth in bytes/s (via mss_cache conversion).
 *   - bbr_min_rtt:           current min_rtt_us (Kalman or window-based).
 *   - bbr_pacing_gain / bbr_cwnd_gain: current gains in BBR_UNIT.
 *
 * Returns 0 if neither BBR nor VEGAS diagnostic extensions are requested.
 */
static size_t ucp_get_info(struct sock* sk, u32 ext_mask, int* attr,                           /* encode diagnostics for ss */
    union tcp_cc_info* info)                                                         /* output info struct */
{
    if (ext_mask & (1 << (INET_DIAG_BBRINFO - 1)) ||                                              /* BBR_INFO bit set OR */
        ext_mask & (1 << (INET_DIAG_VEGASINFO - 1))) {                                              /* VEGAS_INFO bit set */
        struct tcp_sock* tp = tcp_sk(sk);                                                            /* get TCP socket state */
        struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                              /* get UCP CA state */
        u64 bw_raw;
        u64 bw;
        if (unlikely(!tp->mss_cache)) {
            return 0;
        }

        bw_raw = (u64)ucp_bw(sk) * tp->mss_cache;
        if (bw_raw > U64_MAX / USEC_PER_SEC) {
            bw = U64_MAX;
        }
        else {
            bw = (bw_raw * USEC_PER_SEC) >> BW_SCALE;
        }

        memset(&info->bbr, 0, sizeof(info->bbr));                                                       /* zero the BBR info struct */
        info->bbr.bbr_bw_lo = (u32)bw;                                       /* low 32 bits of BW (plain truncation, matches BBR) */
        info->bbr.bbr_bw_hi = (u32)(bw >> UCP_MSTAMP_HI_SHIFT);                                                     /* high 32 bits of BW */
        info->bbr.bbr_min_rtt = ucp->min_rtt_us;                                                     /* min RTT in us */
        info->bbr.bbr_pacing_gain = ucp->pacing_gain;                                                     /* pacing gain (BBR_UNIT) */
        info->bbr.bbr_cwnd_gain = ucp->cwnd_gain;                                                       /* cwnd gain (BBR_UNIT) */

        *attr = INET_DIAG_BBRINFO;                                                                          /* set diagnostic attribute type */
        return sizeof(info->bbr);                                                                            /* return size of BBR info */
    }
    return 0;                                                                                                 /* no diagnostics requested */
}
/*
 * ucp_set_state - Handle TCP CA state transitions (Open, Disorder, Recovery, Loss).
 * @sk:        TCP socket.
 * @new_state: new CA state.
 *
 * On TCP_CA_Loss (RTO timeout or SACK loss):
 *   - Reset full_bw and full_bw_cnt (allow redetection of peak bandwidth).
 *   - full_bw_reached and FSM mode are preserved: loss does not shrink pipe
 *     capacity; re-entering STARTUP on every loss would cause overshoot.
 *   - If not in LT BW mode, seed LT BW sampling with a synthetic loss event.
 *   - Set round_start to 1, clear packet_conservation flag.
 */
UCP_KFUNC void ucp_set_state(struct sock* sk, u8 new_state)                               /* handle CA state transitions */
{
    struct ucp* ucp = (struct ucp*)inet_csk_ca(sk);                                                /* get UCP CA state */

    if (new_state == TCP_CA_Loss) {                                                                    /* transitioned to Loss */
        /* Match BBR (Cardwell et al. 2016): on TCP_CA_Loss, reset full_bw
         * (to allow redetection of the new peak bandwidth) but do NOT clear
         * full_bw_reached or change the FSM mode.  The pipe capacity doesn't
         * suddenly shrink on loss — forcing a re-entry to STARTUP would cause
         * massive overshoot and unnecessary DRAIN cycles. */
        ucp->full_bw = 0;                                                                                /* reset peak bw estimate */
        ucp->full_bw_cnt = 0;                                                                             /* reset stagnation counter */
        ucp->prev_ca_state = TCP_CA_Loss;                                                                   /* track previous CA state */
        ucp->round_start = 1;                                                                                 /* force round start */
        ucp->packet_conservation = 0;                                                                          /* clear conservation flag */

        if (!ucp->lt_use_bw) {                                                                               /* LT BW not active */
            struct rate_sample rs = {};                                                                        /* zero-init rate sample */
            rs.losses = 1;                                                                                      /* synthetic: one loss */
            ucp_lt_bw_sampling(sk, &rs);                                                                         /* trigger LT BW sampling */
        }
    }
}
/* ---- Congestion Ops Structure ----------------------------------------- */

/*
 * tcp_ucp_cong_ops - Registration structure for the "ucp" congestion control
 * algorithm in the Linux TCP stack.
 *
 * Fields mapped to the UCP implementation:
 *   .flags          = TCP_CONG_NON_RESTRICTED (no CAP_NET_ADMIN required)
 *   .name           = "ucp"
 *   .init           = ucp_init (per-connection state allocation)
 *   .release        = ucp_release (per-connection state deallocation)
 *   .cong_control   = ucp_main (main per-ACK callback, Cardwell et al. 2016)
 *   .sndbuf_expand  = ucp_sndbuf_expand (send buffer sizing factor)
 *   .undo_cwnd      = ucp_undo_cwnd (spurious loss undo)
 *   .cwnd_event     = ucp_cwnd_event (congestion event handler)
 *   .ssthresh       = ucp_ssthresh (slow-start threshold query)
 *   .min_tso_segs   = ucp_min_tso_segs (minimum TSO segments)
 *   .get_info       = ucp_get_info (diagnostic state encoding)
 *   .set_state      = ucp_set_state (CA state transition handler)
 */
static struct tcp_congestion_ops tcp_ucp_cong_ops __read_mostly = {                      /* UCP CC ops registration */
    .flags = TCP_CONG_NON_RESTRICTED,                                             /* any user may select this CC */
    .name = "ucp",                                                                /* algorithm name for setsockopt() */
    .owner = THIS_MODULE,                                                           /* module owner reference */
    .init = ucp_init,                                                               /* connection init callback */
    .release = ucp_release,                                                             /* connection close callback */
    .cong_control = ucp_main,                                                                 /* per-ACK processing (Cardwell et al. 2016) */
    .sndbuf_expand = ucp_sndbuf_expand,                                                        /* sndbuf scaling factor */
    .undo_cwnd = ucp_undo_cwnd,                                                             /* cwnd undo handler */
    .cwnd_event = ucp_cwnd_event,                                                              /* CA events handler */
    .ssthresh = ucp_ssthresh,                                                                 /* ssthresh query */
    .min_tso_segs = ucp_min_tso_segs,                                                              /* minimum TSO segs */
    .get_info = ucp_get_info,                                                                    /* diagnostics (ss -i) */
    .set_state = ucp_set_state,                                                                    /* CA state transitions */
};                                                                                                        /* tcp_ucp_cong_ops */

/* ---- Sysctl Interface -------------------------------------------------- */

/*
 * Sysctl table header (registered at /proc/sys/net/ucp/ entries).
 * All entries use the custom ucp_proc_handler which chains to
 * proc_dointvec() and then calls ucp_init_module_params() to
 * recompute all derived values after any write.
 */
static struct ctl_table_header* ucp_ctl_header;                                                       /* sysctl table registration cookie */

/*
 * ucp_proc_handler - Per-entry sysctl handler.
 *
 * Calls proc_dointvec() for standard integer read/write.
 * After a successful write, triggers ucp_init_module_params() to
 * recompute clamped/derived values.
 */
static int ucp_proc_handler(struct ctl_table* ctl, int write,                                          /* sysctl per-entry handler */
    void* buffer, size_t* lenp, loff_t* ppos)                                               /* standard sysctl signature */
{
    int ret = proc_dointvec(ctl, write, buffer, lenp, ppos);                                             /* delegate to kernel handler */
    if (write && ret == 0) {                                                                              /* write succeeded */
        ucp_init_module_params();                                                                           /* re-validate + recompute derived */
    }

    return ret;                                                                                              /* return proc_dointvec result */
}
/*
 * ucp_ctl_table - Sysctl table of all UCP module parameters.
 * The .procname entries are exposed as /proc/sys/net/ucp/ + procname.
 * Array-type parameters (gain_num, gain_den, cycle_decay_mask) use
 * ucp_gain_proc_handler which additionally calls ucp_rebuild_gain_table().
 * A sentinel entry (empty .procname) marks the end of the table.
 */
static struct ctl_table ucp_ctl_table[] = {                                                              /* UCP sysctl registration table */
    /* PROBE_RTT intervals */
    {.procname = "ucp_probe_rtt_base_sec",      .data = &ucp_probe_rtt_base_sec,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* base probe interval (s) */
    {.procname = "ucp_probe_rtt_max_sec",       .data = &ucp_probe_rtt_max_sec,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* max probe interval (s) */
    {.procname = "ucp_probe_rtt_dyn_max_sec",   .data = &ucp_probe_rtt_dyn_max_sec,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* dyn max probe interval (s) */
    /* CWND and ACK-agg gains */
    {.procname = "ucp_cwnd_gain_num",           .data = &ucp_cwnd_gain_num,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* cwnd gain numerator */
    {.procname = "ucp_cwnd_gain_den",           .data = &ucp_cwnd_gain_den,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* cwnd gain denominator */
    {.procname = "ucp_extra_acked_gain_num",    .data = &ucp_extra_acked_gain_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ACK-agg gain numerator */
    {.procname = "ucp_extra_acked_gain_den",    .data = &ucp_extra_acked_gain_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ACK-agg gain denominator */
    /* PROBE_BW gain table arrays (use ucp_gain_proc_handler for rebuild) */
    {.procname = "ucp_gain_num",            .data = ucp_gain_num,            .maxlen = sizeof(ucp_gain_num),          .mode = 0644, .proc_handler = ucp_gain_proc_handler }, /* cycle gain numerators */
    {.procname = "ucp_gain_den",            .data = ucp_gain_den,            .maxlen = sizeof(ucp_gain_den),          .mode = 0644, .proc_handler = ucp_gain_proc_handler }, /* cycle gain denominators */
    {.procname = "ucp_cycle_decay_mask",    .data = ucp_cycle_decay_mask,    .maxlen = sizeof(ucp_cycle_decay_mask),  .mode = 0644, .proc_handler = ucp_gain_proc_handler }, /* decay mask bitmap */
    /* Kalman base noise (Kalman 1960) */
    {.procname = "ucp_kalman_q",               .data = &ucp_kalman_q,               .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* process noise Q */
    {.procname = "ucp_kalman_r",               .data = &ucp_kalman_r,               .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* measurement noise R */
    /* STARTUP and DRAIN gains (Cardwell et al. 2016) */
    {.procname = "ucp_high_gain_num",           .data = &ucp_high_gain_num,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* STARTUP gain numerator */
    {.procname = "ucp_high_gain_den",           .data = &ucp_high_gain_den,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* STARTUP gain denominator */
    {.procname = "ucp_drain_gain_num",          .data = &ucp_drain_gain_num,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* DRAIN gain numerator */
    {.procname = "ucp_drain_gain_den",          .data = &ucp_drain_gain_den,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* DRAIN gain denominator */
    /* PROBE_BW cycle and full-BW detection */
    {.procname = "ucp_probe_bw_cycle_len",      .data = &ucp_probe_bw_cycle_len,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* cycle phase count */
    {.procname = "ucp_full_bw_thresh_num",      .data = &ucp_full_bw_thresh_num,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* full-BW thresh numerator */
    {.procname = "ucp_full_bw_thresh_den",      .data = &ucp_full_bw_thresh_den,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* full-BW thresh denominator */
    {.procname = "ucp_full_bw_cnt",             .data = &ucp_full_bw_cnt,             .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* full-BW round count */
    /* Pacing margin */
    {.procname = "ucp_pacing_margin_num",       .data = &ucp_pacing_margin_num,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* pacing margin numerator */
    {.procname = "ucp_pacing_margin_den",       .data = &ucp_pacing_margin_den,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* pacing margin denominator */
    /* Inflight gain bounds */
    {.procname = "ucp_inflight_low_gain_num",   .data = &ucp_inflight_low_gain_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* inflight low bound numerator */
    {.procname = "ucp_inflight_low_gain_den",   .data = &ucp_inflight_low_gain_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* inflight low bound denominator */
    {.procname = "ucp_inflight_high_gain_num",  .data = &ucp_inflight_high_gain_num,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* inflight high bound numerator */
    {.procname = "ucp_inflight_high_gain_den",  .data = &ucp_inflight_high_gain_den,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* inflight high bound denominator */
    /* Kalman bounds (Kalman 1960) */
    {.procname = "ucp_kalman_p_est_max",        .data = &ucp_kalman_p_est_max,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* p_est absolute max */
    {.procname = "ucp_kalman_converged_p_est",  .data = &ucp_kalman_converged_p_est,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* p_est convergence threshold */
    {.procname = "ucp_drain_skip_qdelay_us",     .data = &ucp_drain_skip_qdelay_us,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* drain skip qdelay threshold */
    {.procname = "ucp_kalman_q_boost_mult",     .data = &ucp_kalman_q_boost_mult,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q-boost multiplier */
    {.procname = "ucp_kalman_q_max",            .data = &ucp_kalman_q_max,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q max ceiling */
    {.procname = "ucp_kalman_q_scale_cap",      .data = &ucp_kalman_q_scale_cap,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q scale cap */
    {.procname = "ucp_kalman_min_samples",      .data = &ucp_kalman_min_samples,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* min Kalman samples */
    /* RTT / min-RTT tracking */
    {.procname = "ucp_rtt_sample_max_us",       .data = &ucp_rtt_sample_max_us,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* max RTT sample (us) */
    {.procname = "ucp_minrtt_fast_fall_cnt",    .data = &ucp_minrtt_fast_fall_cnt,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* fast fall count */
    {.procname = "ucp_minrtt_sticky_num",       .data = &ucp_minrtt_sticky_num,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* sticky ratio numerator */
    {.procname = "ucp_minrtt_sticky_den",       .data = &ucp_minrtt_sticky_den,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* sticky ratio denominator */
    {.procname = "ucp_minrtt_srtt_guard_num",   .data = &ucp_minrtt_srtt_guard_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* SRTT guard numerator */
    {.procname = "ucp_minrtt_srtt_guard_den",   .data = &ucp_minrtt_srtt_guard_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* SRTT guard denominator */
    /* BDP / TSO / EDT */
    {.procname = "ucp_bdp_min_rtt_us",          .data = &ucp_bdp_min_rtt_us,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* BDP min-RTT floor (us) */
    {.procname = "ucp_probe_cwnd_bonus",        .data = &ucp_probe_cwnd_bonus,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* probe cwnd bonus (segs) */
    {.procname = "ucp_edt_near_now_ns",         .data = &ucp_edt_near_now_ns,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* EDT near-now threshold (ns) */
    {.procname = "ucp_min_tso_rate",            .data = &ucp_min_tso_rate,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* min TSO rate (bytes/s) */
    {.procname = "ucp_tso_max_segs",            .data = &ucp_tso_max_segs,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* max TSO segments */
    /* Jitter / qdelay probe tuning */
    {.procname = "ucp_jitter_probe_thresh_us",  .data = &ucp_jitter_probe_thresh_us,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* jitter threshold for gain decay (us) */
    {.procname = "ucp_jitter_probe_scale_us",   .data = &ucp_jitter_probe_scale_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* jitter scale for gain decay (us) */
    {.procname = "ucp_qdelay_probe_thresh_us",  .data = &ucp_qdelay_probe_thresh_us,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* qdelay threshold for gain decay (us) */
    {.procname = "ucp_qdelay_probe_scale_us",   .data = &ucp_qdelay_probe_scale_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* qdelay scale for gain decay (us) */
    {.procname = "ucp_jitter_r_thresh_us",      .data = &ucp_jitter_r_thresh_us,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* jitter threshold for adaptive R (us) */
    {.procname = "ucp_jitter_r_scale",          .data = &ucp_jitter_r_scale,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* jitter scale for adaptive R */
    {.procname = "ucp_kalman_r_max_boost",      .data = &ucp_kalman_r_max_boost,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* R max boost cap */
    /* Long RTT threshold */
    {.procname = "ucp_probe_rtt_long_rtt_us",   .data = &ucp_probe_rtt_long_rtt_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* long-RTT threshold (us) */
    /* LT BW parameters */
    {.procname = "ucp_lt_intvl_min_rtts",       .data = &ucp_lt_intvl_min_rtts,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW min interval RTTs */
    {.procname = "ucp_lt_intvl_max_mult",       .data = &ucp_lt_intvl_max_mult,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW timeout multiplier */
    {.procname = "ucp_lt_loss_thresh",          .data = &ucp_lt_loss_thresh,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW loss threshold (BBR_UNIT) */
    {.procname = "ucp_lt_bw_ratio_num",         .data = &ucp_lt_bw_ratio_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW ratio numerator */
    {.procname = "ucp_lt_bw_ratio_den",         .data = &ucp_lt_bw_ratio_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW ratio denominator */
    {.procname = "ucp_lt_bw_diff",              .data = &ucp_lt_bw_diff,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW absolute diff (bytes/s) */
    {.procname = "ucp_lt_bw_max_rtts",          .data = &ucp_lt_bw_max_rtts,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW max RTTs */
    {.procname = "ucp_lt_bw_probe_pct",         .data = &ucp_lt_bw_probe_pct,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW probe percentage */
    {.procname = "ucp_lt_bw_inst_qdelay_thresh_us", .data = &ucp_lt_bw_inst_qdelay_thresh_us, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW instantaneous qdelay threshold (µs) */
    /* LT BW auto-recovery */
    {.procname = "ucp_lt_restore_ratio_num",    .data = &ucp_lt_restore_ratio_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* restore ratio numerator */
    {.procname = "ucp_lt_restore_ratio_den",    .data = &ucp_lt_restore_ratio_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* restore ratio denominator */
    {.procname = "ucp_lt_restore_consec_acks",  .data = &ucp_lt_restore_consec_acks,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* restore consecutive ACK threshold (max 31) */
    /* Kalman core (Kalman 1960) */
    {.procname = "ucp_kalman_p_est_init",       .data = &ucp_kalman_p_est_init,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* initial p_est */
    {.procname = "ucp_kalman_p_est_floor",      .data = &ucp_kalman_p_est_floor,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* p_est floor */
    {.procname = "ucp_kalman_outlier_ms",       .data = &ucp_kalman_outlier_ms,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* outlier base timeout (ms) */
    {.procname = "ucp_kalman_q_boost_ms",       .data = &ucp_kalman_q_boost_ms,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q-boost time constant (ms) */
    {.procname = "ucp_kalman_qboost_cdwn",       .data = &ucp_kalman_qboost_cdwn,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q-boost cooldown (samples) */
    {.procname = "ucp_kalman_xest_margin_pct",  .data = &ucp_kalman_xest_margin_pct,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* x_est margin above min_rtt (%) */
    {.procname = "ucp_kalman_scale",            .data = &ucp_kalman_scale,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Kalman fixed-point scale */
    {.procname = "ucp_kalman_outlier_jitter_mult_num", .data = &ucp_kalman_outlier_jitter_mult_num, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* outlier jitter mult numerator */
    {.procname = "ucp_kalman_outlier_jitter_mult_den", .data = &ucp_kalman_outlier_jitter_mult_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* outlier jitter mult denominator */
    {.procname = "ucp_kalman_q_min_factor_num", .data = &ucp_kalman_q_min_factor_num, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q min factor numerator */
    {.procname = "ucp_kalman_q_min_factor_den", .data = &ucp_kalman_q_min_factor_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q min factor denominator */
    {.procname = "ucp_kalman_p_est_init_rtt_div_num", .data = &ucp_kalman_p_est_init_rtt_div_num, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* p_est init RTT divisor numerator */
    {.procname = "ucp_kalman_p_est_init_rtt_div_den", .data = &ucp_kalman_p_est_init_rtt_div_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* p_est init RTT divisor denominator */
    /* BBR-S covariance-matched noise estimation (Kalman 1960) */
    {.procname = "ucp_kalman_noise_alpha_num",   .data = &ucp_kalman_noise_alpha_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise alpha numerator */
    {.procname = "ucp_kalman_noise_alpha_den",   .data = &ucp_kalman_noise_alpha_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise alpha denominator */
    {.procname = "ucp_kalman_noise_beta_num",    .data = &ucp_kalman_noise_beta_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise beta numerator */
    {.procname = "ucp_kalman_noise_beta_den",    .data = &ucp_kalman_noise_beta_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise beta denominator */
    {.procname = "ucp_kalman_q_est_max",         .data = &ucp_kalman_q_est_max,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q estimate upper bound */
    {.procname = "ucp_kalman_r_est_max",         .data = &ucp_kalman_r_est_max,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* R estimate upper bound */
    {.procname = "ucp_kalman_q_est_floor",       .data = &ucp_kalman_q_est_floor,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q estimate lower bound */
    {.procname = "ucp_kalman_r_est_floor",       .data = &ucp_kalman_r_est_floor,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* R estimate lower bound */
    {.procname = "ucp_kalman_noise_mode",        .data = &ucp_kalman_noise_mode,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise combination mode (0=off, 1=max, 2=avg) */
    /* ECN */
    {.procname = "ucp_alone_confirm_rounds",     .data = &ucp_alone_confirm_rounds,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* alone mode hysteresis rounds */
    {.procname = "ucp_alone_qdelay_thresh_us",   .data = &ucp_alone_qdelay_thresh_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* alone mode qdelay threshold */
    {.procname = "ucp_alone_jitter_thresh_us",   .data = &ucp_alone_jitter_thresh_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* alone mode jitter threshold */
    {.procname = "ucp_alone_agg_state_level",    .data = &ucp_alone_agg_state_level,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* alone mode agg strictness level */
    {.procname = "ucp_alone_bypass_ecn",         .data = &ucp_alone_bypass_ecn,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* alone mode ECN bypass (0=off, 1=on) */
    {.procname = "ucp_alone_bypass_lt_bw",       .data = &ucp_alone_bypass_lt_bw,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* alone mode LT BW bypass (0=off, 1=on) */
    {.procname = "ucp_ecn_enable",              .data = &ucp_ecn_enable,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN master enable switch */
    {.procname = "ucp_ecn_backoff_num",         .data = &ucp_ecn_backoff_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN backoff percentage numerator */
    {.procname = "ucp_ecn_backoff_den",         .data = &ucp_ecn_backoff_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN backoff percentage denominator */
    {.procname = "ucp_ecn_qdelay_thresh_us",    .data = &ucp_ecn_qdelay_thresh_us,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN qdelay threshold */
    {.procname = "ucp_ecn_ewma_retained",       .data = &ucp_ecn_ewma_retained,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN EWMA retained weight */
    {.procname = "ucp_ecn_ewma_total",          .data = &ucp_ecn_ewma_total,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN EWMA total weight */
    {.procname = "ucp_ecn_idle_decay_num",      .data = &ucp_ecn_idle_decay_num,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN per-ACK idle decay numerator */
    {.procname = "ucp_ecn_idle_decay_den",      .data = &ucp_ecn_idle_decay_den,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ECN per-ACK idle decay denominator */
    /* Kalman */
    {.procname = "ucp_kalman_max_consec_reject", .data = &ucp_kalman_max_consec_reject, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* consecutive reject limit before force-accept */
    /* EWMA */
    {.procname = "ucp_ewma_qdelay_num",         .data = &ucp_ewma_qdelay_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* EWMA qdelay numerator */
    {.procname = "ucp_ewma_qdelay_den",         .data = &ucp_ewma_qdelay_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* EWMA qdelay denominator */
    {.procname = "ucp_ewma_jitter_num",         .data = &ucp_ewma_jitter_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* EWMA jitter numerator */
    {.procname = "ucp_ewma_jitter_den",         .data = &ucp_ewma_jitter_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* EWMA jitter denominator */

    /* Misc */
    {.procname = "ucp_probe_bw_cycle_rand",     .data = &ucp_probe_bw_cycle_rand,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* PROBE_BW random offset range */
    {.procname = "ucp_probe_bw_up_limit",       .data = &ucp_probe_bw_up_limit,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* limit PROBE_BW up-phase exit */
    /* ACK aggregation and PROBE_RTT duration */
    {.procname = "ucp_extra_acked_max_ms_num",  .data = &ucp_extra_acked_max_ms_num,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ACK-agg max ms numerator */
    {.procname = "ucp_extra_acked_max_ms_den",  .data = &ucp_extra_acked_max_ms_den,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* ACK-agg max ms denominator */
    /* ACK agg confidence compensation */
    {.procname = "ucp_agg_enable",              .data = &ucp_agg_enable,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* agg compensation master switch */
    {.procname = "ucp_agg_confidence_thresh",   .data = &ucp_agg_confidence_thresh,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* min confidence for cwnd comp */
    {.procname = "ucp_agg_max_comp_ratio",      .data = &ucp_agg_max_comp_ratio,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* max cwnd comp % of BDP */
    {.procname = "ucp_agg_max_comp_duration",   .data = &ucp_agg_max_comp_duration,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* max consecutive comp RTTs */
    {.procname = "ucp_agg_r_hysteresis",        .data = &ucp_agg_r_hysteresis,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* R recovery hysteresis % */
    {.procname = "ucp_agg_r_multiplier_min",    .data = &ucp_agg_r_multiplier_min,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* R scaling floor */
    {.procname = "ucp_agg_r_multiplier_max",    .data = &ucp_agg_r_multiplier_max,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* R scaling ceiling */
    {.procname = "ucp_agg_factor3_qdelay_us",    .data = &ucp_agg_factor3_qdelay_us,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Factor 3 qdelay margin */
    {.procname = "ucp_agg_factor4_ratio_num",    .data = &ucp_agg_factor4_ratio_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Factor 4 ratio num */
    {.procname = "ucp_agg_factor4_ratio_den",    .data = &ucp_agg_factor4_ratio_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Factor 4 ratio den */
    {.procname = "ucp_agg_safety_qdelay_us",     .data = &ucp_agg_safety_qdelay_us,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* safety qdelay guard */
    {.procname = "ucp_agg_safety_bdp_mult",      .data = &ucp_agg_safety_bdp_mult,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* safety bdp multiplier */
    {.procname = "ucp_agg_max_window_ms",        .data = &ucp_agg_max_window_ms,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* extra_acked cap window */
    {.procname = "ucp_agg_max_decay_pct",        .data = &ucp_agg_max_decay_pct,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* watchdog decay pct */
    {.procname = "ucp_agg_max_per_ack_decay",    .data = &ucp_agg_max_per_ack_decay,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* per-ACK gentle decay */
    {.procname = "ucp_agg_max_per_ack_decay_den", .data = &ucp_agg_max_per_ack_decay_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* per-ACK decay denominator */
    {.procname = "ucp_agg_window_rotation_rtts",  .data = &ucp_agg_window_rotation_rtts,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* window rotation period */
    {.procname = "ucp_agg_factor_weight",          .data = &ucp_agg_factor_weight,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* per-factor score increment */
    {.procname = "ucp_agg_confidence_max",         .data = &ucp_agg_confidence_max,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* confidence scaling denominator */
    {.procname = "ucp_agg_thresh_suspected",       .data = &ucp_agg_thresh_suspected,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* SUSPECTED state threshold */
    {.procname = "ucp_agg_thresh_confirmed",       .data = &ucp_agg_thresh_confirmed,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* CONFIRMED state threshold */
    {.procname = "ucp_agg_thresh_trusted",         .data = &ucp_agg_thresh_trusted,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* TRUSTED state threshold */
    {.procname = "ucp_probe_rtt_mode_ms_num",   .data = &ucp_probe_rtt_mode_ms_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* PROBE_RTT mode ms numerator */
    {.procname = "ucp_probe_rtt_mode_ms_den",   .data = &ucp_probe_rtt_mode_ms_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* PROBE_RTT mode ms denominator */
    /* Other misc */
    {.procname = "ucp_bw_rt_cycle_len",         .data = &ucp_bw_rt_cycle_len,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* BW sliding window length (rounds) */
    {.procname = "ucp_cwnd_min_target",         .data = &ucp_cwnd_min_target,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* absolute min cwnd (segs) */
    /* TSO / Sndbuf / Epoch */
    {.procname = "ucp_tso_headroom_mult",       .data = &ucp_tso_headroom_mult,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* TSO headroom multiplier */
    {.procname = "ucp_min_tso_rate_div",        .data = &ucp_min_tso_rate_div,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* TSO rate threshold divisor */
    {.procname = "ucp_sndbuf_expand_factor",    .data = &ucp_sndbuf_expand_factor,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* sndbuf expansion factor */
    {.procname = "ucp_ack_epoch_max",           .data = &ucp_ack_epoch_max,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* epoch accumulator cap */
    /* Kalman / MinRTT / PROBE_RTT extra */
    {.procname = "ucp_kalman_rtt_dyn_mult",     .data = &ucp_kalman_rtt_dyn_mult,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* dynamic RTT ceiling multiplier */
    {.procname = "ucp_kalman_probe_band_mult",  .data = &ucp_kalman_probe_band_mult,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* probe interval transition band mult */
    {.procname = "ucp_kalman_q_rtt_div",        .data = &ucp_kalman_q_rtt_div,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* Q adaptation RTT divisor */
    {.procname = "ucp_minrtt_fast_fall_div",    .data = &ucp_minrtt_fast_fall_div,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* fast-fall min_rtt divisor */
    {.procname = "ucp_probe_rtt_long_interval_div", .data = &ucp_probe_rtt_long_interval_div, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* long-RTT probe interval divisor */
    {.procname = "ucp_lt_bw_ema_num",         .data = &ucp_lt_bw_ema_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW EMA numerator */
    {.procname = "ucp_lt_bw_ema_den",         .data = &ucp_lt_bw_ema_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* LT BW EMA denominator */
    {.procname = "ucp_kalman_noise_avg_num",  .data = &ucp_kalman_noise_avg_num,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise EMA averaging numerator (mode=2 only) */
    {.procname = "ucp_kalman_noise_avg_den",  .data = &ucp_kalman_noise_avg_den,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* noise EMA averaging denominator (mode=2 only) */
    {.procname = "ucp_tso_segs_low",          .data = &ucp_tso_segs_low,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* TSO segments at low pacing rate */
    {.procname = "ucp_tso_segs_default",      .data = &ucp_tso_segs_default,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* TSO segments at normal pacing rate */
    {.procname = "ucp_extra_acked_win_rtts_max", .data = &ucp_extra_acked_win_rtts_max, .maxlen = sizeof(int), .mode = 0644, .proc_handler = ucp_proc_handler }, /* max dual-window RTTs before rotation */
    {} /* sentinel: end of table */
}; /* end of ucp_ctl_table[] */
/*
 * ---- BTF kfunc Registration (for BPF struct_ops) ----------------------
 *
 * On kernels >= 5.16, UCP registers its callback functions as BTF kfuncs
 * so that BPF struct_ops programs can invoke them.
 *
 * The BTF set macros vary by kernel version:
 *   6.9+: BTF_KFUNCS_START / BTF_KFUNCS_END
 *   6.0+: BTF_SET8_START / BTF_SET8_END
 *   5.16+: BTF_SET_START / BTF_SET_END
 *
 * Additionally, 6.0+ uses BTF_ID_FLAGS with the 'func' flag; pre-6.0
 * uses BTF_ID.  The registration is gated on CONFIG_X86 and
 * CONFIG_DYNAMIC_FTRACE (required for kfunc infrastructure on x86).
 *
 * On 5.18+ the set is registered via register_btf_kfunc_id_set();
 * on 5.16-5.17 it uses register_kfunc_btf_id_set() with a different API.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)                                            /* kernel 5.16+: BTF support */

BTF_SETS_START(tcp_ucp_check_kfunc_ids)                                                        /* start BTF kfunc ID set */
#ifdef CONFIG_X86                                                                                /* kfunc only on x86 */
#ifdef CONFIG_DYNAMIC_FTRACE                                                                       /* requires dynamic ftrace */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)                                                    /* 6.0+: BTF_ID_FLAGS */
BTF_ID_FLAGS(func, ucp_init)                                                                         /* register ucp_init as kfunc */
BTF_ID_FLAGS(func, ucp_main)                                                                          /* register ucp_main as kfunc */
BTF_ID_FLAGS(func, ucp_sndbuf_expand)                                                                  /* register ucp_sndbuf_expand as kfunc */
BTF_ID_FLAGS(func, ucp_undo_cwnd)                                                                       /* register ucp_undo_cwnd as kfunc */
BTF_ID_FLAGS(func, ucp_cwnd_event)                                                                       /* register ucp_cwnd_event as kfunc */
BTF_ID_FLAGS(func, ucp_ssthresh)                                                                         /* register ucp_ssthresh as kfunc */
BTF_ID_FLAGS(func, ucp_min_tso_segs)                                                                     /* register ucp_min_tso_segs as kfunc */
BTF_ID_FLAGS(func, ucp_set_state)                                                                        /* register ucp_set_state as kfunc */
#else                                                                                                      /* pre-6.0: BTF_ID macro */
BTF_ID(func, ucp_init)                                                                                     /* register ucp_init as kfunc (legacy) */
BTF_ID(func, ucp_main)                                                                                      /* register ucp_main as kfunc (legacy) */
BTF_ID(func, ucp_sndbuf_expand)                                                                             /* register ucp_sndbuf_expand as kfunc (legacy) */
BTF_ID(func, ucp_undo_cwnd)                                                                                 /* register ucp_undo_cwnd as kfunc (legacy) */
BTF_ID(func, ucp_cwnd_event)                                                                                /* register ucp_cwnd_event as kfunc (legacy) */
BTF_ID(func, ucp_ssthresh)                                                                                   /* register ucp_ssthresh as kfunc (legacy) */
BTF_ID(func, ucp_min_tso_segs)                                                                                /* register ucp_min_tso_segs as kfunc (legacy) */
BTF_ID(func, ucp_set_state)                                                                                    /* register ucp_set_state as kfunc (legacy) */
#endif                                                                                                           /* end BTF_ID version switch */
#endif /* CONFIG_DYNAMIC_FTRACE */                                                                                 /* end DYNAMIC_FTRACE gate */
#endif /* CONFIG_X86 */                                                                                             /* end CONFIG_X86 gate */
BTF_SETS_END(tcp_ucp_check_kfunc_ids)                                                                                /* end BTF kfunc ID set */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)                                                                     /* 5.18+: new registration API */
static const struct btf_kfunc_id_set tcp_ucp_kfunc_set = {                                                              /* BTF kfunc set descriptor */
    .owner = THIS_MODULE,                                                                                                 /* module owner */
    .set = &tcp_ucp_check_kfunc_ids,                                                                                     /* pointer to kfunc ID set */
};                                                                                                                          /* tcp_ucp_kfunc_set */
#else                                                                                                                         /* 5.16-5.17: legacy API */
static DEFINE_KFUNC_BTF_ID_SET(&tcp_ucp_check_kfunc_ids, tcp_ucp_kfunc_btf_set);                                             /* define legacy kfunc set */
#endif                                                                                                                          /* end version switch */

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) */                                                                     /* end BTF support block */

/* ---- Module Init / Exit ----------------------------------------------- */

/*
 * ucp_register - Module initialization function.
 *
 * Steps:
 *   1. Verify struct ucp fits within ICSK_CA_PRIV_SIZE (compile-time check).
 *   2. Seed ucp_gain_num[] / ucp_gain_den[] with BBRv1 defaults (Cardwell et al. 2016):
 *      [5/4, 3/4, 1/1, 1/1, 1/1, 1/1, 1/1, 1/1] repeated across 256 slots.
 *      This ensures sysctl reports real values from the start.
 *   3. Call ucp_init_module_params() to clamp and compute all derived values.
 *   4. Register sysctl interface under /proc/sys/net/ucp/.
 *   5. Register BTF kfunc set for BPF struct_ops (5.16+, 5.18+).
 *   6. Register the congestion_control ops with the TCP stack.
 *
 * Cleanup on failure: unregister_sysctl -> return error.
 */
static int __init ucp_register(void)                                                                                        /* module init entry */
{
    static const int dfl_num[UCP_DEFAULT_GAIN_CYCLE_LEN] = { 5,3,1,1,1,1,1,1 };                                                                         /* BBRv1 gain numerator pattern (Cardwell et al. 2016) */
    static const int dfl_den[UCP_DEFAULT_GAIN_CYCLE_LEN] = { 4,4,1,1,1,1,1,1 };                                                                         /* BBRv1 gain denominator pattern */
    int ret = -ENOMEM, i;                                                                                                                /* return code and loop index */

    /* Compile-time guard: struct ucp must fit in the CA private slot */
    BUILD_BUG_ON(sizeof(struct ucp) > ICSK_CA_PRIV_SIZE);                                                                       /* compile-time size check */

    /*
     * Initialize gain arrays with BBRv1 default cycle:
     *   5/4 (=1.25x), 3/4 (=0.75x), 1/1 (=1.0x), 1/1, 1/1, 1/1, 1/1, 1/1.
     * This pattern repeats across the 256-slot table.
     * ucp_rebuild_gain_table() reads from these arrays to compute the
     * effective gains in ucp_cycle_gain_table[].
     */
    for (i = 0; i < UCP_GAIN_SLOTS; i++) {                                                                                      /* iterate all 256 gain slots */
        ucp_gain_num[i] = dfl_num[i % UCP_DEFAULT_GAIN_CYCLE_LEN];                                                                                         /* set numerator from BBRv1 pattern */
        ucp_gain_den[i] = dfl_den[i % UCP_DEFAULT_GAIN_CYCLE_LEN];                                                                                         /* set denominator from BBRv1 pattern */
    }
    ucp_init_module_params();                                                                                                      /* clamp + compute all derived values */

    /* Register sysctl at /proc/sys/net/ucp/ */
    ucp_ctl_header = register_sysctl("net/ucp", ucp_ctl_table);                                                                     /* register sysctl table */
    if (!ucp_ctl_header) {
        pr_warn("UCP: failed to register sysctl\n");
        goto unregister_sysctl;
    }

    /* Register as a TCP congestion control algorithm */
    ret = tcp_register_congestion_control(&tcp_ucp_cong_ops);                                                                            /* register CC ops */
    if (ret) {                                                                                                                             /* registration failed */
        goto unregister_sysctl;                                                                                                              /* clean up */
    }
    /* ---- BTF kfunc registration (kernel >= 5.18) ---- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)                                                                                   /* 5.18+: direct registration */
#if defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    ret = register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &tcp_ucp_kfunc_set);                                                    /* register kfunc set */
    if (ret < 0) {                                                                                                                     /* registration failed */
        goto unregister_cc;                                                                                                              /* clean up: unregister CC */
    }
#endif
#endif
    /* ---- BTF kfunc registration (kernel 5.16-5.17, legacy API) ---- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)                                        /* 5.16-5.17: legacy API */
#if defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    ret = register_kfunc_btf_id_set(&bpf_tcp_ca_kfunc_list, &tcp_ucp_kfunc_btf_set);                                                               /* register via legacy API */
    if (ret < 0) {                                                                                                                                   /* registration failed */
        pr_warn("UCP: legacy kfunc registration failed (err %d); BPF struct_ops unavailable\n", ret);                             /* non-fatal: CC still works */
    }
#endif
#endif
    return 0;                                                                                                                                  /* success */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) && defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    unregister_cc:                                                                                                                                /* BTF registration failed after CC registered */
    tcp_unregister_congestion_control(&tcp_ucp_cong_ops);                                                                                   /* unregister CC */
#endif

unregister_sysctl:                                                                                                                                /* error cleanup label */
    if (ucp_ctl_header) {                                                                                                                          /* sysctl was registered */
        unregister_sysctl_table(ucp_ctl_header);                                                                                                     /* unregister sysctl */
        ucp_ctl_header = NULL;                                                                                                                         /* clear header pointer */
    }
    return ret;                                                                                                                                        /* return error code */
}
/*
 * ucp_unregister - Module exit function.
 *
 * Reverse of ucp_register:
 *   1. Unregister legacy BTF kfunc set (5.16-5.17).
 *   2. Unregister congestion control ops.
 *   3. Unregister sysctl table.
 *
 * Note: BTF kfunc sets registered via register_btf_kfunc_id_set() (5.18+)
 * are automatically cleaned up by the kernel on module unload.
 */
static void __exit ucp_unregister(void)                                                                                                              /* module exit handler */
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)                                                    /* legacy BTF API */
#if defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    unregister_kfunc_btf_id_set(&bpf_tcp_ca_kfunc_list, &tcp_ucp_kfunc_btf_set);                                                                        /* unregister legacy kfunc set */
#endif
#endif
    tcp_unregister_congestion_control(&tcp_ucp_cong_ops);                                                                                                 /* unregister CC ops */
    if (ucp_ctl_header) {                                                                                                                                  /* sysctl table registered */
        unregister_sysctl_table(ucp_ctl_header);                                                                                                             /* unregister sysctl table */
        ucp_ctl_header = NULL;                                                                                                                                 /* clear header pointer */
    }
}

module_init(ucp_register);                                                                                                                                     /* register module init callback */
module_exit(ucp_unregister);                                                                                                                                     /* register module exit callback */

MODULE_AUTHOR("PPP PRIVATE NETWORK(TM) X");                                                                                                                     /* primary module author */
MODULE_AUTHOR("Original BBR: Van Jacobson, Neal Cardwell, Yuchung Cheng, "                                                                                    /* BBR algorithm authors */
    "Soheil Hassas Yeganeh (Google)");                                                                                                                    /* (Cardwell et al. 2016) */
MODULE_LICENSE("Dual BSD/GPL");                                                                                                                                    /* module license identifier */
MODULE_DESCRIPTION("TCP UCP v1.0 - BBRv1 state machine with Kalman-filter propagation-delay estimation");                                                          /* module description */
MODULE_VERSION("1.0");                                                                                                                                             /* module version string */
