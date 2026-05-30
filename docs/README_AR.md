[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP UCP v1.0 (بروتوكول الاتصال العالمي)

وحدة التحكم في ازدحام TCP لبيئات VPS ذات النطاق الترددي المشترك، تجمع بين آلة حالة BBRv1 ومرشح كالمان لتقدير زمن الانتشار.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## نظرة عامة على الخوارزمية

يقوم TCP UCP بتنفيذ وحدة تحكم في الازدحام من جانب المرسل لنواة لينكس كوحدة قابلة للتحميل `tcp_ucp.ko`. يتم استدعاء دالة التحكم في الازدحام `ucp_main()` عند كل ACK من `tcp_ack()`، وتستقبل بنية `rate_sample` التي تحتوي على عينات عرض النطاق الترددي و RTT على مستوى النواة بالإضافة إلى عدد التسليمات والخسائر. تعمل الخوارزمية في نظامين زمنيين: **مسار سريع لكل ACK** يحدّث حالة القياس ويحسب أهداف السرعة اللحظية والنافذة، و**مسار أبطأ لكل دورة** يقيّم شروط انتقال الحالة ويعيد حساب الكسوب.

تتكون خط أنابيب القياس الأساسي من مكونين:

1. **مرشح عرض النطاق الترددي الأقصى بنافذة منزلقة** (`minmax_running_max` من `linux/win_minmax.h`): نافذة تغطي آخر `ucp_bw_rt_cycle_len` (افتراضي 10) دورة. يوفّر تقدير `max_bw` المتوافق مع BBR.

2. **مقدّر زمن الانتشار بمرشح كالمان**: يستبدل الحد الأدنى لـ RTT للنافذة المنزلقة في BBRv1. مرشح كالمان أحادي الحالة (Kalman 1960) يعمل بوحدات النقطة الثابتة `ucp_kalman_scale` × ميكروثانية، يصمّم زمن الانتشار الحقيقي كمسيرة عشوائية:
   - الحالة: `x[k] = x[k−1] + w[k]`، `w ~ N(0, Q)`
   - المشاهدة: `z[k] = x[k] + v[k]`، `v ~ N(0, R)`

اصطلاحات النقطة الثابتة: `BW_UNIT = 1 << 24` لعرض النطاق الترددي (شرائح * 2^24 / ميكروثانية)، `BBR_UNIT = 1 << 8 = 256` كوحدة كسوب لا بعدية.

## آلة الحالة

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

أربعة أنماط مشفّرة كحقل `mode` ثنائي البت في `struct ucp`:

- **STARTUP (0)**: الحالة الابتدائية. pacing_gain ≈ 2.885x (`ucp_high_gain_val`)، cwnd_gain أيضاً 2.885x. استكشاف أسي لعرض النطاق الترددي.
- **DRAIN (1)**: الدخول بعد الخروج من STARTUP. pacing_gain ≈ 0.347x (`ucp_drain_gain_val`)، cwnd_gain يبقى 2.885x. يفرّغ الطابور المتراكم خلال STARTUP.
- **PROBE_BW (2)**: الحالة المستقرة. يتنقل عبر جدول كسوب من 256 خانة (نمط من 8 مراحل متكرر: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: يفرّغ بشكل دوري الحزم قيد الطيران إلى `ucp_cwnd_min_target` (افتراضي 4 شرائح) للحصول على عينة RTT جديدة.

### STARTUP → DRAIN

يُفعل عند تعيين `full_bw_reached` — بعد `ucp_full_bw_cnt` (افتراضي 3) دورات متتالية يفشل فيها `max_bw` في النمو بمقدار `ucp_full_bw_thresh_val` على الأقل (افتراضي 1.25x) مقارنة بالذروة الملاحظة سابقاً. يتم كتابة BDP عند كسوب 1.0x إلى `snd_ssthresh`. يتم إعادة تعيين `qdelay_avg` إلى الصفر لمنع تأثير تراكم طابور STARTUP على PROBE_BW.

### DRAIN → PROBE_BW

يُفعل عندما تكون الحزم قيد الطيران المقدرة عند EDT ≤ الحزم المستهدفة عند كسوب BDP 1.0x. **تحسين تخطي التفريغ**: عندما يكون مرشح كالمان متقارباً و `qdelay_avg` أقل من `ucp_drain_skip_qdelay_us` (افتراضي 1000 ميكروثانية)، يتم تخطي مرحلة DRAIN — التحويل المبكر إلى PROBE_BW.

عند الدخول إلى PROBE_BW، يتم عشوائية مؤشر مرحلة الدورة: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (افتراضي `len − 1 − rand(8)`)، مما يفك ارتباط التدفقات المتزامنة المشاركة لنفس رابط الاختناق.

### PROBE_BW → PROBE_RTT

يُفعل عند انتهاء فترة مرشح PROBE_RTT — الطابع الزمني `min_rtt_stamp` لم يتم تحديثه خلال الفترة المحسوبة. يتم حفظ cwnd في `prior_cwnd`، وتعيين السرعة للتفريغ.

### PROBE_RTT → PROBE_BW

بعد انخفاض الحزم قيد الطيران إلى `ucp_cwnd_min_target` أو ملاحظة حد دورة، يستمر لمدة `ucp_probe_rtt_mode_ms_val` على الأقل (افتراضي 200 ملي ثانية) ودورة كاملة واحدة على الأقل، ثم يخرج. يتم استعادة cwnd إلى `prior_cwnd` على الأقل، ويتم تجاوز السرعة مؤقتاً بـ `ucp_high_gain_val` لإعادة ملء الأنبوب بسرعة.

### الاسترداد والخسارة

- عند TCP_CA_Loss: إعادة تعيين `full_bw` و `full_bw_cnt`، تعيين `round_start` إلى 1، مسح `packet_conservation` إلى 0. إذا كان LT BW غير نشط، يحقن حدث خسارة اصطناعي لتفعيل أخذ عينات LT.
- دخول الاسترداد (TCP_CA_Recovery): تفعيل `packet_conservation`، cwnd = inflight + acked.
- خروج الاسترداد: استعادة إلى `prior_cwnd`، مسح `packet_conservation`.
- `ucp_undo_cwnd()`: يعيد تعيين `full_bw` و `full_bw_cnt` (مع الحفاظ على `full_bw_reached`)، ويمسح حالة LT BW.

## القياسات الأساسية

### تقدير عرض النطاق الترددي

مرشح عرض النطاق الترددي الأقصى بنافذة منزلقة (`minmax_running_max` من `linux/win_minmax.h`) عبر `ucp_bw_rt_cycle_len` (افتراضي 10) دورة. bw اللحظي = `delivered × BW_UNIT / interval_us` يُحسب لكل ACK. يُغذى إلى النافذة المنزلقة فقط عندما لا يكون التطبيق محدوداً أو عندما bw ≥ max الحالي (قاعدة BBR).

عندما يكون `lt_use_bw` نشطاً، يتحول تقدير عرض النطاق الترددي النشط إلى `lt_bw` (تقدير عرض النطاق طويل الأمد).

### مرشح كالمان

استدعاء كالمان العددي أحادي الحالة (تعقيد O(1)):

```
التنبؤ:
  x_pred = x_est          (انتقال حالة الهوية)
  p_pred = p_est + Q      (تنبؤ التباين)

التحديث:
  innov   = z − x_pred    (الابتكار)
  K       = p_pred / (p_pred + R)   (كسب كالمان [0,1])
  x_est   = x_pred + K × innov      (تحديث الحالة)
  p_est   = (1 − K) × p_pred        (التباين البعدي)
```

**ضوضاء العملية التكيفية Q**:
```
Q_base   = ucp_kalman_q (افتراضي 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**ضوضاء القياس التكيفية R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**بوابة القيم الشاذة**: عتبة ديناميكية `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. تُطبق فقط عندما `p_pred ≤ ucp_kalman_converged_p_est_val`. بعد `ucp_kalman_max_consec_reject` (افتراضي 25) رفضاً متتالياً، يتم قبول العينة التالية قسراً لمنع الإغلاق الذاتي المعزز.

**تقدير الضوضاء المطابق للتباين (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`، `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. وضع الدمج: الوضع 0 = استرشادي فقط، الوضع 1 = الحد الأقصى (افتراضي)، الوضع 2 = مزيج مرجح.

**استلام كالمان**: عندما `x_est > 0` و `sample_cnt ≥ ucp_kalman_min_samples` (افتراضي 5)، يتم استبدال `min_rtt_us` بـ `x_est / ucp_kalman_scale`. لا يتم تحديث `min_rtt_stamp` — يبقى مؤقت PROBE_RTT مستقلاً.

نموذج min-rtt لـ x_est: يستخدم model_rtt المشتق من كالمان min(x_est_us, min_rtt_us) — الأصغر من الاثنين.

## تحسينات BBR

### توهين الكسوب

مفعّل بواسطة خريطة البت 256 بت `ucp_cycle_decay_mask[]` لمراحل PROBE_BW محددة. معادلة التوهين (على عينة كالمان المقبولة):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = القياس العكسي لـ p_est (BBR_UNIT عند الكامل)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

قياس ثقة كالمان: عندما `p_est > ucp_kalman_converged_p_est`، يتم تقليل التوهين نسبياً، مما يتجنب التراجع المفرط عندما يكون المرشح غير مؤكد.

### التراجع عن ECN

شروط التفعيل (يجب أن تتحقق جميعها):
1. `ucp_ecn_enable_val != 0`
2. كالمان متقارب (`p_est < converged`، `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (تم رصد علامات CE)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (افتراضي 2000 ميكروثانية)
5. الوضع ليس PROBE_BW (cwnd_gain ثابت عند 2x في PROBE_BW)

خلال مراحل الاستكشاف (`pacing_gain > BBR_UNIT`)، يكون التراجع عن ECN متدرجاً بـ `BBR_UNIT² / pacing_gain` — ~80% من التراجع عند استكشاف 1.25x، ~65% عند كسوب STARTUP 2.89x.

EWMA نسبة علامات ECN: تُحدّث على حدود الدورة بواسطة `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (افتراضي 3/4)، مع توهين لطيف لكل ACK بقيمة `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (افتراضي 31/32) عند كل ACK بدون علامات CE جديدة.

### اكتشاف التدفق الفردي

عندما يكتشف UCP أن التدفق على الأرجح وحيد عند عنق الزجاجة (تأخير منخفض في الطابور، تقلب منخفض، لا توجد علامات ECN، لا يوجد تجميع ACK، لا يوجد عرض نطاق LT)، فإنه ينتقل تلقائيًا إلى وضع BBR النقي:

- `ucp_get_model_rtt()` يعيد `min_rtt_us` مباشرة (متجاوزًا تقدير كالمان السلس الذي يحتوي على انحياز إيجابي طفيف بسبب ضوضاء القياس أحادية الجانب).
- `ucp_ecn_backoff()` قابل للتكوين عبر `ucp_alone_bypass_ecn` (افتراضي 1) — على مسار تدفق فردي، علامات ECN هي إيجابيات كاذبة من AQM لعدم وجود مرسل منافس. تخطيها يطابق سلوك ECN الصفري في BBR. اضبط على 0 للاحتفاظ بتراجع ECN حتى في الوضع الفردي (متحفظ).
- شرط LT BW (policer) قابل للتكوين عبر `ucp_alone_bypass_lt_bw` (افتراضي 1) — مسار التدفق الفردي لا يحتوي على policer، لذا لا يمكن لـ LT BW التنشيط بشكل مشروع. تخطيه يمنع الخروج الزائف من الوضع الفردي بسبب المحفزات الكاذبة. اضبط على 0 للسلوك الصارم الأصلي.

هذا يزيل فجوة الإنتاجية للتدفق الفردي بين UCP وBBR، مع الحفاظ على حلقة الحماية الكاملة لـ UCP (كالمان، تراجع ECN، توهين الكسب، عرض نطاق LT) لسيناريوهات التدفقات المتعددة.

**التباطؤ**: يتطلب الدخول `ucp_alone_confirm_rounds` (افتراضي 3) جولات متتالية مؤهلة——يمنع التذبذب خلال فترات الهدوء القصيرة أثناء تنافس تدفقات متعددة ("تسارع محافظ"). الخروج فوري——أي شرط يفشل يمسح العلم ويعيد عداد التأكيد ("كبح عدواني").

شروط التأهيل (يجب استيفاء جميع الشروط الستة عند حدود الجولة):
0. تقارب كالمان (`sample_cnt >= ucp_kalman_min_samples`) — الثقة في qdelay/jitter كإشارات طابور
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (الافتراضي 1000 us) — الطابور شبه فارغ
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (الافتراضي 2000 us) — تقلب ميكروي لساعة ACK فقط
3. `ecn_ewma == 0` — لا توجد علامات ازدحام من AQM
4. `lt_use_bw == 0` — ليس في وضع تحديد المعدل المكتشف بواسطة policer
5. `agg_state <= max` وفقًا لـ `ucp_alone_agg_state_level` (الافتراضي 1) — ثلاثة مستويات من صرامة تجميع ACK: 0 = IDLE فقط (الأكثر صرامة، تجميع صفري)، 1 = ≤ SUSPECTED (افتراضي، يسمح بالتجميع العابر)، 2 = ≤ CONFIRMED (الأكثر تساهلاً، يحظر التجميع المستمر فقط)

### فترة PROBE_RTT الديناميكية

تربط `p_est` لكالمان بفترة PROBE_RTT لكل اتصال:

```
p_est ≤ converged:              الفترة = dyn_max (افتراضي 30 ثانية)
p_est ≥ high (= mult × conv):   الفترة = base (افتراضي 10 ثوانٍ)
converged < p_est < high:       استيفاء خطي
```

يقلل من تواتر PROBE_RTT عندما تكون الثقة عالية (p_est منخفض)، مما يقلل من تذبذب الإنتاجية على المسارات المستقرة. يعود إلى فترة 10 ثوانٍ التقليدية عندما تكون الثقة منخفضة.

**تذبذب الدخول لكل تدفق**: لمنع جميع التدفقات المتعايشة من الدخول المتزامن إلى PROBE_RTT (التفريغ إلى 4 حزم مجمعة ~1.8 Mbps ثم إعادة الملء بـ 2.89×)، يضيف كل تدفق تذبذباً مشتقاً من التجزئة (انتشار 0–845 ملي ثانية) إلى فترة PROBE_RTT الخاصة به. في أي لحظة، يوجد تدفق واحد على الأكثر في PROBE_RTT، مما يلغي الانهيار المتزامن للتفريغ/إعادة الملء المسبب لـ RTO.

### تقدير عرض النطاق طويل الأمد (LT)

مقدّر الحد الأدنى المحفّز بالخسارة. تمتد فترة أخذ العينات بين [4, 16] RTT. صالح عندما تكون نسبة الخسارة ≥ 5.9% (`ucp_lt_loss_thresh` افتراضي 15/256). عرض النطاق `bw = delivered × BW_UNIT / interval_us`.

على عكس المتوسط البسيط لـ BBR (`(bw + lt_bw) >> 1`)، يستخدم UCP EMA قابل للتكوين (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`، افتراضي 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

يختلف التفعيل عن BBR: يخزّن UCP `lt_bw` في أول فترة صالحة لكنه لا يعيّن `lt_use_bw`؛ الاتساق مع فترة سابقة مطلوب — يقلل من التفعيل الخاطئ الناتج عن ضوضاء القياس.

**بوابة الازدحام ثنائية العتبة**: قبل تعيين `lt_use_bw = 1`، يتم تقييم كل من فحص طابور EWMA المستمر (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) وفحص الطابور الفوري المعتمد على SRTT (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`، افتراضي 5000 ميكروثانية). عند اكتشاف ازدحام، يتم إحباط أخذ عينات LT BW. يعمل فحص SRTT دون تخصيص `ext`، مما يوفر شبكة أمان ضد فشل التخصيص.

تعزيز استكشاف LT BW (`ucp_lt_bw_probe_pct`، افتراضي 10%): يضخّم pacing_gain بـ `1 + probe_pct/100` عبر جميع مراحل PROBE_BW. المكون التراكمي: `+1% لكل 8 RTT` زيادة، محدود بـ `2 × probe_pct`.

الاسترداد التلقائي لـ LT BW (`ucp_lt_restore_ratio_num/den`، افتراضي 5/4 = 1.25x): عندما `max_bw > lt_bw × ratio` لـ `ucp_lt_restore_consec_acks` (افتراضي 3) ACK متتالية، يخرج LT BW تلقائياً ويستأنف استكشاف PROBE_BW العادي.

### تعويض تجميع ACK المعتمد على الثقة (مستوحى من BBRplus)

يضيف طبقة ثانية محكومة بالثقة فوق مقدر extra-acked التقليدي ثنائي الفتحة.

**أربعة عوامل متعامدة** (يساهم كل منها بـ `ucp_agg_factor_weight` نقطة، افتراضي 256):
1. كالمان متقارب (`p_est < converged` + `sample_cnt >= min_samples`)
2. ليس في استرداد الخسارة (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT ضمن `min_rtt_us + ucp_agg_factor3_qdelay_us` (افتراضي 2 ملي ثانية) من زمن الانتشار الحقيقي
4. `extra_acked` ضمن `ucp_agg_factor4_ratio_num/den` (افتراضي 1.5x) من الحد الأقصى للنافذة

**أربع حالات**: IDLE (< `ucp_agg_thresh_suspected`=256)، SUSPECTED (≥256)، CONFIRMED (≥512)، TRUSTED (≥768).

**طبقة الإشارة** (نشطة دائماً): الثقة تحدد خطياً عامل القياس R `[r_min, r_max]`. R يرتفع فوراً (استجابة سريعة)، ويتوهن بنسبة `ucp_agg_r_hysteresis`% (افتراضي 75% محتفظ به، ~4 RTTs للعودة إلى خط الأساس) لكل RTT.

**طبقة التحكم** (`agg_state ≥ CONFIRMED`): تعويض cwnd محمي بخمس طبقات أمان:
1. يمنع إذا كان تأخير الطابور > `ucp_agg_safety_qdelay_us` (افتراضي 4 ملي ثانية)
2. يمنع أثناء استرداد الخسارة
3. يمنع إذا كان cwnd > `BDP × ucp_agg_safety_bdp_mult` (افتراضي 3x)
4. يمنع إذا كانت الحزم قيد الطيران > cwnd الآمن + هدف شرائح TSO
5. مراقب: يخفض CONFIRMED → SUSPECTED بعد `ucp_agg_max_comp_duration` (افتراضي 8) RTT متتالية

### إعادة تعيين qdelay_avg للتفريغ

عند الانتقال إلى DRAIN، يتم إعادة تعيين `qdelay_avg` إلى الصفر، مما يمنع استمرار تقدير طابور STARTUP إلى PROBE_BW.

### تكييف مقسم TSO

تعدل `ucp_min_tso_segs()` مقسم عتبة المعدل بناءً على حالة كالمان:
- كالمان متقارب + `jitter_ewma < 1000 ميكروثانية`: المقسم ينخفض إلى النصف (8→4)، دفعات TSO أكبر
- `jitter_ewma > 4000 ميكروثانية`: المقسم يتضاعف (8→16)، دفعات TSO أصغر لكبت التذبذب

## معدل السرعة و Cwnd

### معدل السرعة

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // تعديل الكسوب
rate = rate × USEC_PER_SEC >> BW_SCALE            // تحويل إلى بايت/ثانية
rate = rate × margin_div / 100                    // هامش السرعة (افتراضي 1%، مطابق لـ BBR)
```

تُطبق تغييرات المعدل فوراً (بدون تمليس)، مطابقة لـ BBR (Cardwell et al. 2016). بعد `full_bw_reached`: جميع تغييرات المعدل تُكتب فوراً. في STARTUP/DRAIN: فقط الزيادات تُطبق (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP الأساسي
// حدود الحزم قيد الطيران (غير STARTUP: مشبك lo~hi؛ STARTUP: أرضية lo فقط)
target = quantization_budget(target)              // مساحة TSO + تقريب زوجي + مكافأة المرحلة-0
target += ack_agg_bonus + agg_compensation        // تعويض تجميع ACK

// تقدم cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // التقارب إلى الهدف
else (STARTUP):
    cwnd = cwnd + acked                          // نمو أسي

cwnd = max(cwnd, cwnd_min_target)                 // أرضية مطلقة 4
وضع PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // الحد الأدنى للحزم قيد الطيران
```

## معاملات الوحدة

المعاملات متاحة تحت `/proc/sys/net/ucp/`. الكتابة تستدعي `ucp_init_module_params()` (تحقق + تثبيت + حساب القيم المشتقة). كتابة معاملات المصفوفة تستدعي `ucp_rebuild_gain_table()`.

### فترات PROBE_RTT

| المعامل | الافتراضي | الأدنى | الأقصى | الوحدة | الوصف |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | ثانية | فترة PROBE_RTT الأساسية |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | ثانية | الحد الأعلى لمسارات RTT الطويلة |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | ثانية | أقصى فترة ديناميكية؛ 0 يعطّل |

### الكسوب

| المعامل | الافتراضي | الأدنى | الأقصى | الوصف |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | كسوب cwnd الأساسي لـ PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | مضاعف مكافأة تجميع ACK |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | كسوب STARTUP (≈2.885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | كسوب DRAIN (≈0.347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | الحد الأدنى للحزم قيد الطيران (1.25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | الحد الأعلى للحزم قيد الطيران (2.0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | نمط BBRv1 (256 خانة) | 0/1 | — | كسوب السرعة لكل خانة |
| `ucp_cycle_decay_mask[8]` | 0 (الكل صفر) | 0 | 0x7FFFFFFF | خريطة بت التوهين 256 بت |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | إنهاء محدود لوضع probe-up (0=إيقاف) |

### كالمان الأساسي

| المعامل | الافتراضي | الأدنى | الأقصى | الوصف |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | ضوضاء العملية الأساسية Q |
| `ucp_kalman_r` | 400 | 0 | 100k | ضوضاء القياس الأساسية R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | الحد الأقصى المطلق لـ p_est |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | عتبة التقارب |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | p_est الابتدائي |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | أرضية p_est |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | مقياس النقطة الثابتة (قوة اثنين) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | الحد الأدنى للعينات قبل الاستلام |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ملي ثانية | عتبة القيم الشاذة الأساسية |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | مضاعف تعزيز Q |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ملي ثانية | ثابت زمني لتعزيز Q |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | سقف Q |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | حد مقياس Q |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | أقصى رفض متتالٍ قبل القبول القسري |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | ميكروثانية | سقف RTT لكالمان |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | مضاعف تعزيز R الأقصى |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | مضاعف السقف الديناميكي لـ RTT |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | مقسم RTT لتكييف Q |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | مضاعف نطاق انتقال PROBE_RTT |

### كالمان الإضافيات (نوع بسط/مقام)

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | مضاعف تذبذب القيم الشاذة |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | العامل الأدنى لـ Q |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | مقسم RTT لتهيئة p_est |

### تقدير الضوضاء BBR-S

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | معدل تعلم تقدير Q |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | معدل تعلم تقدير R |
| `ucp_kalman_noise_mode` | 1 | 0-2 | وضع الدمج (0=إيقاف، 1=أقصى، 2=متوسط مرجح) |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | الحد الأعلى لتقدير Q |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | الحد الأعلى لتقدير R |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | الحد الأدنى لكل تقدير |

### توهين الكسوب (الاستكشاف)

| المعامل | الافتراضي | النطاق | الوحدة | الوصف |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | ميكروثانية | عتبة توهين تأخير الطابور |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | ميكروثانية | مقياس توهين تأخير الطابور |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | ميكروثانية | عتبة توهين التذبذب |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | ميكروثانية | مقياس توهين التذبذب |

### R التكيفية (مدفوعة بالتذبذب)

| المعامل | الافتراضي | النطاق | الوحدة | الوصف |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | ميكروثانية | عتبة التذبذب لزيادة R |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | مقسم مقياس زيادة R |

### ECN

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | المفتاح الرئيسي لـ ECN |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | كسر التراجع عن ECN |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | ميكروثانية | عتبة تأخير الطابور لـ ECN |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | أوزان EWMA لـ ECN |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | توهين ECN في الخمول |

### min_rtt

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | عدد الانخفاض السريع |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | مقسم عتبة الانخفاض السريع |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | نسبة الانخفاض اللزج |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | نسبة حماية SRTT |

### عرض النطاق طويل الأمد (LT)

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTT | الحد الأدنى لطول الفترة |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | مضاعف مهلة الفترة |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | الحد الأدنى لنسبة الخسارة |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | التسامح النسبي |
| `ucp_lt_bw_diff` | 500 | 0-100k | بايت/ثانية | التسامح المطلق |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTT | أقصى RTT نشطة لـ LT BW |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | تعزيز استكشاف LT BW |

### الاسترداد التلقائي لـ LT

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | نسبة تفعيل الاسترداد |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | عدد ACK المتتالية للتفعيل |

### ثقة تجميع ACK

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | المفتاح الرئيسي |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | عتبة ثقة تعويض cwnd |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % من BDP | سقف تعويض cwnd |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTT | مهلة المراقب |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | توهين التباطؤ لـ R |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | نطاق قياس R (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | ميكروثانية | هامش تأخير الطابور للعامل 3 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | نسبة العامل 4 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | ميكروثانية | تأخير الطابور لحماية الأمان 1 |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | مضاعف BDP لحماية الأمان |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ملي ثانية | نافذة سقف extra_acked |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | معدل توهين المراقب |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTT | فترة تدوير النافذة |
| `ucp_agg_factor_weight` | 256 | 1-1024 | درجة لكل عامل |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | أقصى ثقة |

### معاملات EWMA

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | وزن EWMA لتأخير الطابور |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | وزن EWMA للتذبذب |

### متنوعات

| المعامل | الافتراضي | النطاق | الوصف |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | مراحل دورة PROBE_BW (قوة اثنين) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | الإزاحة العشوائية لمرحلة الدورة |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | عتبة نمو الخروج من STARTUP |
| `ucp_full_bw_cnt` | 3 | 1-3 | عدد دورات عدم النمو للخروج |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | مدة البقاء في PROBE_RTT |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | هامش السرعة (0 = لا شيء) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | شريحة | مكافأة cwnd للمرحلة-0 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | دورة | طول النافذة المنزلقة لعرض النطاق |
| `ucp_cwnd_min_target` | 4 | 1-1000 | شريحة | أدنى cwnd (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | ميكروثانية | أرضية min_rtt لـ BDP |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | نانوثانية | عتبة EDT للوقت القريب |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | بايت/ثانية | عتبة المعدل المنخفض لـ TSO |
| `ucp_min_tso_rate_div` | 8 | 1-256 | مقسم معدل TSO (قاعدة تكيفية) |
| `ucp_tso_max_segs` | 127 | 1-65535 | شريحة | أقصى شرائح TSO |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | مضاعف مساحة TSO |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | عامل توسيع مخزن الإرسال |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | بايت | سقف حقبة ACK |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | أقصى نافذة تجميع ACK |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | ميكروثانية | عتبة RTT الطويلة |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | مقسم فترة RTT الطويلة |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | ميكروثانية | عتبة تأخير الطابور لتخطي التفريغ |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | جولات | الجولات قبل تفعيل وضع التدفق الفردي |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | أقصى تأخير طابور لاكتشاف التدفق الفردي |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | أقصى تقلب لاكتشاف التدفق الفردي |
| ucp_alone_agg_state_level | 1 | 0-2 | — | صرامة التجميع (0=IDLE فقط، 1=≤SUSPECTED افتراضي، 2=≤CONFIRMED شديد العدوانية) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | تخطي تراجع ECN في الوضع الفردي (1=تخطي، 0=نشط) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | تخطي شرط LT BW في الوضع الفردي (1=تخطي، 0=نشط) |

## مسار البيانات

```
وصول ACK (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► خط أنابيب ثقة تجميع ACK (عند ucp_agg_enable)
    │      قياس → تقييم → حالة → مراقب
    │      ├── طبقة الإشارة: قياس R لكالمان (نشط دائماً)
    │      └── طبقة التحكم: تعويض cwnd (CONFIRMED+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              عرض النطاق الأقصى بنافذة منزلقة
    │      ├── ucp_update_ecn_ewma()        نسبة علامات ECN-CE
    │      ├── ucp_update_ack_aggregation()  نافذة مزدوجة extra_acked
    │      ├── ucp_update_cycle_phase()     تقدم مرحلة PROBE_BW
    │      ├── ucp_check_full_bw_reached()  كشف خروج STARTUP
    │      ├── ucp_check_drain()            دخول/خروج DRAIN + تخطي التفريغ
    │      ├── ucp_update_min_rtt()         كالمان + نافذة min-RTT + PROBE_RTT
    │      └── تعيين الكسوب حسب الوضع
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            تراجع ECN (cwnd_gain فقط)
    │
    ├──► ucp_set_pacing_rate()              فوري، قاعدة BBR
    │
    └──► ucp_set_cwnd()                    BDP + حدود + تعويض التجميع
```

## التدفق الداخلي لمرشح كالمان

```
عينة RTT (rtt_us)
    │
    ├── غير صالحة (≥0 وأقل من dynamic_max)؟ لا → تجاهل
    │
    ├── بداية باردة (sample_cnt==0)؟ نعم → تهيئة: x_est=z، p_est=max(p_init, rtt_us/div)
    │                                           (يتجاوز بوابة RTT القصوى)
    │
    ├── Q التكيفية: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R التكيفية: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── الابتكار: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── التنبؤ: p_pred = p_est + Q
    │
    ├── بوابة القيم الشاذة: |innov| > dyn_thresh && p_pred ≤ converged؟
    │   ├── نعم و reject_cnt < max → رفض، ++consec_reject_cnt، إرجاع
    │   └── نعم و reject_cnt ≥ max → قبول قسري (مضاد للإغلاق)
    │
    └── تحديث كالمان:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (مثبت غير سالب)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── تحديث EWMA للتذبذب
         ├── تحديث EWMA لتأخير الطابور
         ├── تقدير الضوضاء المطابق للتباين BBR-S
         └── sample_cnt++
```

## التشخيص

واجهة تشخيص متوافقة مع BBR عبر `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: تقدير عرض النطاق 64 بت (بايت/ثانية)
bbr_min_rtt:         min_rtt_us الحالي
bbr_pacing_gain:     كسوب السرعة الحالي (BBR_UNIT، 256=1.0x)
bbr_cwnd_gain:       كسوب cwnd الحالي (BBR_UNIT)
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

## المراجع

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
