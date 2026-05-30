[🇺🇸 English](../README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Universal Communication Protocol)

Модуль управления перегрузкой TCP для VPS-сред с общей полосой пропускания, объединяющий конечный автомат BBRv1 с фильтром Калмана для оценки задержки распространения.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Обзор алгоритма

TCP UCP реализует модуль управления перегрузкой на стороне отправителя для ядра Linux в виде загружаемого `tcp_ucp.ko`. Функция управления перегрузкой `ucp_main()` вызывается при каждом ACK из `tcp_ack()`, получая структуру `rate_sample`, содержащую оценки пропускной способности и RTT на уровне ядра, а также счетчики доставки и потерь. Алгоритм работает в двух временных режимах: **быстрый путь на каждый ACK**, который обновляет состояние измерений и вычисляет мгновенные цели pacing и окна, и **медленный путь на каждый раунд**, который оценивает условия перехода состояний и пересчитывает коэффициенты усиления.

Основной конвейер измерений состоит из двух компонентов:

1. **Фильтр максимальной пропускной способности со скользящим окном** (`minmax_running_max` из `linux/win_minmax.h`): окно охватывает последние `ucp_bw_rt_cycle_len` (по умолчанию 10) кругов. Предоставляет оценку `max_bw`, совместимую с BBR.

2. **Оценщик задержки распространения на фильтре Калмана**: заменяет скользящее окно минимального RTT из BBRv1. Одномерный фильтр Калмана (Kalman 1960), работающий в единицах с фиксированной запятой `ucp_kalman_scale` × мкс, моделирующий истинную задержку распространения как случайное блуждание:
   - Состояние: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Наблюдение: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Соглашения фиксированной запятой: `BW_UNIT = 1 << 24` для пропускной способности (сегменты * 2^24 / мкс), `BBR_UNIT = 1 << 8 = 256` как безразмерная единица усиления.

## Конечный автомат

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

Четыре режима, закодированные в 2-битном поле `mode` в `struct ucp`:

- **STARTUP (0)**: Начальное состояние. pacing_gain ≈ 2.885x (`ucp_high_gain_val`), cwnd_gain также 2.885x. Экспоненциальный поиск пропускной способности.
- **DRAIN (1)**: Вход после выхода из STARTUP. pacing_gain ≈ 0.347x (`ucp_drain_gain_val`), cwnd_gain остается 2.885x. Опустошение очереди, накопленной во время STARTUP.
- **PROBE_BW (2)**: Установившееся состояние. Циклически проходит таблицу усилений из 256 слотов (повторяющийся шаблон 8 фаз: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: Периодически уменьшает количество пакетов в полете до `ucp_cwnd_min_target` (по умолчанию 4 сегмента) для получения свежего отсчета RTT.

### STARTUP → DRAIN

Срабатывает, когда установлен `full_bw_reached` — после `ucp_full_bw_cnt` (по умолчанию 3) последовательных раундов, в которых `max_bw` не увеличивается как минимум на `ucp_full_bw_thresh_val` (по умолчанию 1.25x) по сравнению с ранее наблюдавшимся пиком. BDP с коэффициентом 1.0x записывается в `snd_ssthresh`. `qdelay_avg` сбрасывается в ноль, чтобы предотвратить влияние накопленной очереди STARTUP на PROBE_BW.

### DRAIN → PROBE_BW

Срабатывает, когда расчетный объем в полете при EDT ≤ целевой объем в полете при усилении BDP 1.0x. **Оптимизация пропуска DRAIN**: когда фильтр Калмана сошелся И `qdelay_avg` ниже `ucp_drain_skip_qdelay_us` (по умолчанию 1000 мкс), фаза DRAIN пропускается — происходит ранний переход в PROBE_BW.

При входе в PROBE_BW индекс фазы цикла рандомизируется: `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (по умолчанию `len − 1 − rand(8)`), что декоррелирует параллельные потоки, разделяющие узкое место.

### PROBE_BW → PROBE_RTT

Срабатывает, когда истекает интервал фильтра PROBE_RTT — отметка времени `min_rtt_stamp` не обновлялась в течение вычисленного интервала. cwnd сохраняется в `prior_cwnd`, pacing устанавливается на опорожнение.

### PROBE_RTT → PROBE_BW

После того как объем в полете падает до `ucp_cwnd_min_target` или наблюдается граница раунда, состояние сохраняется как минимум `ucp_probe_rtt_mode_ms_val` (по умолчанию 200 мс) и после завершения как минимум одного полного раунда, затем происходит выход. cwnd восстанавливается как минимум до `prior_cwnd`, pacing временно переопределяется с помощью `ucp_high_gain_val` для быстрого заполнения трубы.

### Восстановление и потери

- При TCP_CA_Loss: `full_bw` и `full_bw_cnt` сбрасываются, `round_start` устанавливается в 1, `packet_conservation` очищается до 0. Если LT BW не активен, вводится синтетическое событие потери для запуска LT выборки.
- Вход в восстановление (TCP_CA_Recovery): `packet_conservation` включается, cwnd = inflight + acked.
- Выход из восстановления: восстанавливается до `prior_cwnd`, `packet_conservation` очищается.
- `ucp_undo_cwnd()`: сбрасывает `full_bw` и `full_bw_cnt` (сохраняя `full_bw_reached`), очищает состояние LT BW.

## Основные измерения

### Оценка пропускной способности

Фильтр максимальной пропускной способности со скользящим окном (`minmax_running_max` из `linux/win_minmax.h`) за `ucp_bw_rt_cycle_len` (по умолчанию 10) раундов. Мгновенная bw = `delivered × BW_UNIT / interval_us` вычисляется на каждый ACK. Подается в скользящее окно только когда не ограничено приложением или когда bw ≥ текущего максимума (правило BBR).

Когда активен `lt_use_bw`, оценка активной пропускной способности переключается на `lt_bw` (долгосрочная оценка пропускной способности).

### Фильтр Калмана

Одномерная скалярная рекурсия Калмана (сложность O(1)):

```
Предсказание:
  x_pred = x_est          (тождественный переход состояния)
  p_pred = p_est + Q      (предсказание ковариации)

Обновление:
  innov   = z − x_pred    (инновация)
  K       = p_pred / (p_pred + R)   (коэффициент Калмана [0,1])
  x_est   = x_pred + K × innov      (обновление состояния)
  p_est   = (1 − K) × p_pred        (апостериорная ковариация)
```

**Адаптивный шум процесса Q**:
```
Q_base   = ucp_kalman_q (по умолчанию 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Адаптивный шум измерения R**:
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: when `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Шлюз выбросов**: динамический порог `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Применяется только когда `p_pred ≤ ucp_kalman_converged_p_est_val`. После `ucp_kalman_max_consec_reject` (по умолчанию 25) последовательных отклонений следующий образец принудительно принимается, чтобы предотвратить самоблокировку.

**Оценка шума, согласованная с ковариацией (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Режим комбинации: режим 0 = только эвристический, режим 1 = максимум (по умолчанию), режим 2 = взвешенная смесь.

**Переход Калмана**: когда `x_est > 0` и `sample_cnt ≥ ucp_kalman_min_samples` (по умолчанию 5), `min_rtt_us` заменяется на `x_est / ucp_kalman_scale`. `min_rtt_stamp` не обновляется — триггер интервала PROBE_RTT остается независимым.

**Предел запаса x_est**: Полученное из Калмана `model_rtt` ограничено значением `min_rtt_us × (100 + ucp_kalman_xest_margin_pct) / 100` (по умолчанию 8%).

## Улучшения BBR

### Затухание усиления

Включено 256-битной битовой маской `ucp_cycle_decay_mask[]` для определенных фаз PROBE_BW. Формула затухания (при принятом образце Калмана):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = обратное масштабирование p_est (BBR_UNIT при полной уверенности)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Масштабирование уверенности Калмана: когда `p_est > ucp_kalman_converged_p_est`, затухание пропорционально уменьшается, избегая излишнего снижения, когда фильтр не уверен.

### ECN Backoff

Условия активации (все должны выполняться):
1. `ucp_ecn_enable_val != 0`
2. Фильтр Калмана сошелся (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (наблюдались CE-метки)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (по умолчанию 2000 мкс)
5. Режим НЕ PROBE_BW (cwnd_gain фиксирован на 2x в PROBE_BW)

Во время фаз зондирования (`pacing_gain > BBR_UNIT`), ECN backoff градуируется по `BBR_UNIT² / pacing_gain` — ~80% снижения при зондировании 1.25x, ~65% при усилении STARTUP 2.89x.

EWMA коэффициента ECN-меток: обновляется на границах раундов по `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (по умолчанию 3/4), с плавным затуханием на каждый ACK по `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (по умолчанию 31/32) на каждый ACK без новых CE-меток.

### Обнаружение Одиночного Потока

Когда UCP обнаруживает, что поток, вероятно, один на узком месте (низкая задержка очереди, низкий джиттер, отсутствие меток ECN, отсутствие агрегации ACK, отсутствие пропускной способности LT), он автоматически переходит в чистый режим BBR:

- `ucp_get_model_rtt()` возвращает `min_rtt_us` напрямую (в обход сглаженной оценки Калмана, которая имеет небольшое положительное смещение из-за одностороннего шума измерений).
- `ucp_ecn_backoff()` настраивается через `ucp_alone_bypass_ecn` (по умолчанию 1) — на пути с одним потоком метки ECN являются ложными срабатываниями AQM, поскольку нет конкурирующего отправителя. Пропуск соответствует нулевому поведению ECN в BBR. Установите 0, чтобы сохранить откат ECN даже в одиночном режиме (консервативный).
- Условие LT BW (полицер) настраивается через `ucp_alone_bypass_lt_bw` (по умолчанию 1) — на пути с одним потоком нет полицера, поэтому LT BW не может активироваться законно. Пропуск предотвращает ложные выходы из одиночного режима из-за ложных срабатываний. Установите 0 для исходного строгого поведения.

Это устраняет разрыв в пропускной способности одиночного потока между UCP и BBR, сохраняя при этом полный защитный контур UCP (Калман, откат ECN, затухание усиления, пропускная способность LT) для сценариев с несколькими потоками.

**Гистерезис**: для входа требуется `ucp_alone_confirm_rounds` (по умолчанию 3) последовательных подходящих раундов——предотвращает колебания во время коротких тихих периодов при конкуренции нескольких потоков ("консервативное ускорение"). Выход происходит немедленно——любое невыполненное условие сбрасывает флаг и счетчик подтверждения ("агрессивное торможение").

Условия квалификации (все шесть должны выполняться на границе раунда):
0. Калман сошёлся (`sample_cnt >= ucp_kalman_min_samples`) — доверять qdelay/jitter как сигналам очереди
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (по умолчанию 1000 us) — очередь почти пуста
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (по умолчанию 2000 us) — только микроджиттер тактов ACK
3. `ecn_ewma == 0` — отсутствие меток перегрузки от AQM
4. `lt_use_bw == 0` — не в режиме ограничения скорости, обнаруженном полицером
5. `agg_state <= max` согласно `ucp_alone_agg_state_level` (по умолчанию 1) — три уровня строгости агрегации ACK: 0 = только IDLE (самый строгий, нулевая агрегация), 1 = ≤ SUSPECTED (по умолчанию, допускает временную агрегацию), 2 = ≤ CONFIRMED (самый разрешительный, блокирует только постоянную агрегацию)

### Динамический интервал PROBE_RTT

Отображает `p_est` Калмана в интервал PROBE_RTT для каждого соединения:

```
p_est ≤ converged:              interval = dyn_max (по умолчанию 30 с)
p_est ≥ high (= mult × conv):   interval = base (по умолчанию 10 с)
converged < p_est < high:       линейная интерполяция
```

Уменьшает частоту PROBE_RTT при высокой уверенности (низкий `p_est`), снижая джиттер пропускной способности на стабильных путях. Возвращается к классическому 10-секундному интервалу при низкой уверенности.

**Джиттер входа для каждого потока**: Чтобы предотвратить одновременный вход всех сосуществующих потоков в PROBE_RTT (сброс до 4 пакетов ~1.8 Mbps, затем заполнение с коэффициентом 2.89×), каждый поток добавляет хэш-производный джиттер (разброс 0–845 мс) к своему интервалу PROBE_RTT. В любой момент времени в PROBE_RTT находится максимум ~1 поток, что устраняет одновременный коллапс сброса/заполнения, вызывающий RTO.

### Долгосрочная оценка пропускной способности (LT Bandwidth)

Оценщик нижней границы, запускаемый потерями. Интервал выборки охватывает [4, 16] RTT. Действителен, когда коэффициент потерь ≥ 5.9% (`ucp_lt_loss_thresh` по умолчанию 15/256). Пропускная способность `bw = delivered × BW_UNIT / interval_us`.

В отличие от простого среднего BBR (`(bw + lt_bw) >> 1`), UCP использует настраиваемую EMA (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, по умолчанию 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

Активация отличается от BBR: UCP сохраняет `lt_bw` при первом действительном интервале, но НЕ устанавливает `lt_use_bw`; требуется согласованность с предыдущим интервалом — уменьшает ложную активацию из-за шума измерений.

**Двухпороговый шлюз перегрузки**: Перед установкой `lt_use_bw = 1` оцениваются как постоянная проверка очереди EWMA (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`), ТАК и мгновенная проверка очереди на основе SRTT (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, по умолчанию 5000 мкс). При обнаружении перегрузки выборка LT BW прерывается. Проверка SRTT работает без выделения `ext`, обеспечивая защиту от сбоев выделения.

Усиление зондирования LT BW (`ucp_lt_bw_probe_pct`, по умолчанию 10%): увеличивает pacing_gain на `1 + probe_pct/100` во всех фазах PROBE_BW. Компонент нарастания: увеличение `+1% за 8 RTT`, с ограничением `2 × probe_pct`.

Автовосстановление LT BW (`ucp_lt_restore_ratio_num/den`, по умолчанию 5/4 = 1.25x): когда `max_bw > lt_bw × ratio` в течение `ucp_lt_restore_consec_acks` (по умолчанию 3) последовательных ACK, LT BW автоматически выключается и нормальное зондирование PROBE_BW возобновляется.

### Компенсация агрегации ACK на основе уверенности (вдохновлено BBRplus)

Добавляет второй уровень с порогом уверенности поверх традиционного двухслотового оценщика extra-acked.

**Четыре ортогональных фактора** (каждый вносит `ucp_agg_factor_weight` очков, по умолчанию 256):
1. Фильтр Калмана сошелся (`p_est < converged` + `sample_cnt >= min_samples`)
2. Не в состоянии восстановления потерь (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT в пределах `min_rtt_us + ucp_agg_factor3_qdelay_us` (по умолчанию 2 мс) от истинной задержки распространения
4. `extra_acked` в пределах `ucp_agg_factor4_ratio_num/den` (по умолчанию 1.5x) от максимума окна

**Четыре состояния**: IDLE (< `ucp_agg_thresh_suspected`=256), SUSPECTED (≥256), CONFIRMED (≥512), TRUSTED (≥768).

**Сигнальный уровень** (всегда активен): уверенность линейно интерполирует коэффициент масштабирования R `[r_min, r_max]`. R возрастает мгновенно (быстрый отклик), затухает со скоростью `ucp_agg_r_hysteresis`% (по умолчанию сохраняется 75%, ~4 RTT до базового уровня) за RTT.

**Уровень управления** (`agg_state ≥ CONFIRMED`): пятиуровневая компенсация cwnd с защитой:
1. Блокируется, если задержка в очереди > `ucp_agg_safety_qdelay_us` (по умолчанию 4 мс)
2. Блокируется во время восстановления потерь
3. Блокируется, если cwnd > `BDP × ucp_agg_safety_bdp_mult` (по умолчанию 3x)
4. Блокируется, если объем в полете > безопасный cwnd + цель TSO сегментов
5. Сторожевой таймер: понижает CONFIRMED→SUSPECTED после `ucp_agg_max_comp_duration` (по умолчанию 8) последовательных RTT

### Сброс qdelay_avg при DRAIN

При переходе в DRAIN, `qdelay_avg` сбрасывается в ноль, предотвращая сохранение оценки очереди из STARTUP в PROBE_BW.

### Адаптация делителя TSO

`ucp_min_tso_segs()` корректирует пороговый делитель скорости на основе состояния Калмана:
- Фильтр Калмана сошелся + `jitter_ewma < 1000 мкс`: делитель уменьшается вдвое (8→4), более крупные пакеты TSO
- `jitter_ewma > 4000 мкс`: делитель удваивается (8→16), более мелкие пакеты TSO для подавления джиттера

## Скорость пакетирования и Cwnd

### Скорость пакетирования

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // регулировка усиления
rate = rate × USEC_PER_SEC >> BW_SCALE            // преобразование в байты/с
rate = rate × margin_div / 100                    // запас пакетирования (по умолчанию 1%, matching BBR)
```

Изменения скорости применяются немедленно (без сглаживания), как в BBR (Cardwell et al. 2016). После `full_bw_reached`: все изменения скорости записываются немедленно. В STARTUP/DRAIN: применяются только увеличения (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // базовый BDP
// границы объема в полете (не STARTUP: ограничение lo~hi; STARTUP: только нижняя граница lo)
target = quantization_budget(target)              // запас TSO + выравнивание + бонус фазы 0
target += ack_agg_bonus + agg_compensation        // компенсация агрегации ACK

// прогрессия cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // сходимость к цели
else (STARTUP):
    cwnd = cwnd + acked                          // экспоненциальный рост

cwnd = max(cwnd, cwnd_min_target)                 // абсолютный минимум 4
Режим PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // минимальный объем в полете
```

## Параметры модуля

Параметры доступны через `/proc/sys/net/ucp/`. Запись вызывает `ucp_init_module_params()` (валидация + ограничение + вычисление производных значений). Запись параметров-массивов вызывает `ucp_rebuild_gain_table()`.

### Интервалы PROBE_RTT

| Параметр | По умолчанию | Мин | Макс | Ед. | Описание |
|-----------|---------|-----|-----|------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | с | Базовый интервал PROBE_RTT |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | с | Верхний предел для путей с большим RTT |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | с | Максимальный динамический интервал; 0 отключает |

### Усиления

| Параметр | По умолчанию | Мин | Макс | Описание |
|-----------|---------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Базовое усиление cwnd для PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Множитель бонуса агрегации ACK |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Усиление STARTUP (≈2.885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Усиление DRAIN (≈0.347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Нижняя граница объема в полете (1.25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Верхняя граница объема в полете (2.0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | Шаблон BBRv1 (256 слотов) | 0/1 | — | Пофазовое усиление пакетирования |
| `ucp_cycle_decay_mask[8]` | 0 (все нули) | 0 | 0x7FFFFFFF | 256-битная битовая маска затухания |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Ограниченный выход из probe-up (0=выкл) |

### Базовые параметры Калмана

| Параметр | По умолчанию | Мин | Макс | Описание |
|-----------|---------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Базовый шум процесса Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Базовый шум измерения R |
| `ucp_kalman_p_est_max` | 1,000,000 | 1 | 100M | Абсолютный максимум p_est |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1M | Порог сходимости |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10M | Начальный p_est |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | Нижняя граница p_est |
| `ucp_kalman_scale` | 1024 | 64 | 1,048,576 | Масштаб фиксированной запятой (степень двойки) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Минимальное количество образцов до перехода |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | мс | Базовый порог выбросов |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Множитель Q-boost |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | мс | Постоянная времени Q-boost |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Потолок Q |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Ограничение масштаба Q |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Максимальное количество последовательных отклонений до принудительного принятия |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10M | мкс | Потолок RTT Калмана |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | Максимальный множитель усиления R |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Множитель динамического потолка RTT |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1M | Делитель RTT для адаптации Q |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | Множитель полосы перехода PROBE_RTT |

### Дополнительные параметры Калмана (тип числитель/знаменатель)

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Множитель джиттера выбросов |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Минимальный коэффициент Q |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Делитель RTT инициализации p_est |

### Оценка шума BBR-S

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Скорость обучения оценки Q |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Скорость обучения оценки R |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Режим комбинации (0=выкл, 1=макс, 2=взвешенное среднее) |
| `ucp_kalman_q_est_max` | 1,000,000,000 | 1-2B | Верхняя граница оценки Q |
| `ucp_kalman_r_est_max` | 1,000,000,000 | 1-2B | Верхняя граница оценки R |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Нижняя граница каждой оценки |

### Затухание усиления (зондирование)

| Параметр | По умолчанию | Диапазон | Ед. | Описание |
|-----------|---------|-------|------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | мкс | Порог затухания qdelay |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | мкс | Масштаб затухания qdelay |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | мкс | Порог затухания джиттера |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | мкс | Масштаб затухания джиттера |

### Адаптивный R (управляемый джиттером)

| Параметр | По умолчанию | Диапазон | Ед. | Описание |
|-----------|---------|-------|------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | мкс | Порог джиттера для увеличения R |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | Делитель масштаба увеличения R |

### ECN

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | Главный выключатель ECN |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Доля снижения ECN |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | мкс | Порог очереди ECN |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Веса EWMA ECN |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Затухание ECN в простое |

### min_rtt

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Счетчик быстрого падения |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Делитель порога быстрого падения |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Коэффициент липкого падения |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Коэффициент защиты SRTT |

### Пропускная способность LT

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTT | Минимальная длина интервала |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Множитель тайм-аута интервала |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Минимальный коэффициент потерь |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Относительный допуск |
| `ucp_lt_bw_diff` | 500 | 0-100k | байт/с | Абсолютный допуск |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTT | Максимальное количество активных RTT LT BW |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | Усиление зондирования LT BW |

### Автовосстановление LT

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Коэффициент триггера восстановления |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Количество последовательных ACK для триггера |

### Уверенность агрегации ACK

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Главный выключатель |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | Порог уверенности для компенсации cwnd |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % от BDP | Ограничение компенсации cwnd |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTT | Тайм-аут сторожевого таймера |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | Гистерезис затухания R |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Диапазон масштабирования R (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | мкс | Запас qdelay фактора 3 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Коэффициент фактора 4 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | мкс | Защита 1: задержка очереди |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Множитель BDP для защиты |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | мс | Окно ограничения extra_acked |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Скорость затухания сторожевого таймера |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTT | Период ротации окна |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Очки за фактор |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Максимальная уверенность |

### Коэффициенты EWMA

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Вес EWMA qdelay |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Вес EWMA джиттера |

### Разное

| Параметр | По умолчанию | Диапазон | Описание |
|-----------|---------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | Фазы цикла PROBE_BW (степень двойки) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Случайное смещение фазы цикла |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Порог роста для выхода из STARTUP |
| `ucp_full_bw_cnt` | 3 | 1-3 | Количество раундов без роста для выхода |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Длительность пребывания в PROBE_RTT |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Запас пакетирования (0 = нет) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | сегм | Бонус cwnd фазы 0 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | раунды | Длина скользящего окна BW |
| `ucp_cwnd_min_target` | 4 | 1-1000 | сегм | Минимальный cwnd (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | мкс | Нижняя граница BDP min_rtt |
| `ucp_edt_near_now_ns` | 1000 | 0-10M | нс | Порог EDT "почти сейчас" |
| `ucp_min_tso_rate` | 1,200,000 | 1-1B | байт/с | Нижний порог скорости TSO |
| `ucp_min_tso_rate_div` | 8 | 1-256 | Делитель скорости TSO (адаптивная база) |
| `ucp_tso_max_segs` | 127 | 1-65535 | сегм | Максимальное количество сегментов TSO |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | Множитель запаса TSO |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Коэффициент расширения буфера отправки |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | байты | Предел эпохи ACK |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Максимальное окно агрегации ACK |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10M | мкс | Порог длинного RTT |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Делитель интервала длинного RTT |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | мкс | Порог задержки очереди для пропуска DRAIN |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | раундов | Раундов до активации режима одиночного потока |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Макс. задержка очереди для обнаружения одиночного потока |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | Макс. джиттер для обнаружения одиночного потока |
| ucp_alone_agg_state_level | 1 | 0-2 | — | Строгость агрегации (0=только IDLE, 1=≤SUSPECTED по умолч., 2=≤CONFIRMED слишком агрессивно) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | Пропустить откат ECN в одиночном режиме (1=пропустить, 0=активен) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | Пропустить условие LT BW в одиночном режиме (1=пропустить, 0=активен) |

## Путь данных

```
Прибытие ACK (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► Конвейер уверенности агрегации ACK (когда ucp_agg_enable)
    │      измерение → оценка → состояние → сторожевой таймер
    │      ├── Сигнальный уровень: масштабирование R Калмана (всегда активно)
    │      └── Уровень управления: компенсация cwnd (CONFIRMED+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              скользящее окно максимума BW
    │      ├── ucp_update_ecn_ewma()        коэффициент ECN-CE меток
    │      ├── ucp_update_ack_aggregation()  двухоконный extra_acked
    │      ├── ucp_update_cycle_phase()     продвижение фазы PROBE_BW
    │      ├── ucp_check_full_bw_reached()  обнаружение выхода из STARTUP
    │      ├── ucp_check_drain()            вход/выход DRAIN + пропуск DRAIN
    │      ├── ucp_update_min_rtt()         Калман + оконный min-RTT + PROBE_RTT
    │      └── Назначение усиления в зависимости от режима
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            снижение ECN (только cwnd_gain)
    │
    ├──► ucp_set_pacing_rate()              немедленно, правило BBR
    │
    └──► ucp_set_cwnd()                    BDP + границы + компенсация агрегации
```

## Внутренний поток фильтра Калмана

```
Отсчет RTT (rtt_us)
    │
    ├── Недействителен (≥0 и < dynamic_max)? Нет → отбросить
    │
    ├── Холодный старт (sample_cnt==0)? Да → инициализация: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           (обходит шлюз максимума RTT)
    │
    ├── Адаптивный Q: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   Адаптивный R: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Инновация: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Предсказание: p_pred = p_est + Q
    │
    ├── Шлюз выбросов: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Да и reject_cnt < max → отклонить, ++consec_reject_cnt, возврат
    │   └── Да и reject_cnt ≥ max → принудительно принять (антиблокировка)
    │
    └── Обновление Калмана:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (ограничено неотрицательным)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Обновление EWMA джиттера
         ├── Обновление EWMA qdelay
         ├── Оценка шума, согласованная с ковариацией BBR-S
         └── sample_cnt++
```

## Диагностика

Совместимый с BBR диагностический интерфейс через `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: 64-битная оценка пропускной способности (байт/с)
bbr_min_rtt:         текущий min_rtt_us
bbr_pacing_gain:     текущее усиление пакетирования (BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       текущее усиление cwnd (BBR_UNIT)
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

## Ссылки

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
