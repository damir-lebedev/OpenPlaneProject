<div dir="rtl">

# ‏AUTOPILOT / feedback — حلقة التغذية الراجعة (تمهيد)

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/feedback.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

> ⚠️ **تمهيد، غير موصول بالبرنامج الثابت.** لا `FlightController` ولا
> `Autopilot` ولا `main.cpp` يضمّن هذه الترويسات. وتُختبر بمحاكاة بحلقة مغلقة
> (`test/test_feedback`، على الحاسوب وعلى اللوحة) وباختبارات الوحدات الأصلية.
> وخطة الربط في
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#خطة-الربط).

الفكرة: بدلًا من PID على الزاوية بمعاملات مضبوطة لسرعة واحدة — منظِّم مغلق على
**استجابة الطائرة**، بنموذج للمحور يُتعلَّم في الطيران، وحماية من الانهيار،
ومراحل إقلاع وهبوط تقودها الحساسات. والمدخل الوحيد هو `FlightSnapshot`، والمخرج
الوحيد هو `FeedbackOutput`.

جميع الوحدات ترويسية فقط؛ ويضمّنها `FeedbackModules.h` كلها بسطر واحد.

---

## ‏namespace `FeedbackConfig`

**الملف:** `autopilot/feedback/FeedbackConfig.h`

جميع ثوابت الحلقة (ستنتقل إلى `Config.h` عند الربط). والقيم المعلَّمة «прикидка» («تقدير تقريبي»)
لنموذج بوزن نحو 1 kg وفتحة جناحين 1.2 m. وتُفهرَس المصفوفات `[AXIS_COUNT]` بالمحور.

| المجموعة | الثوابت |
|---|---|
| عام | `GRAVITY = 9.80665`؛ والمحاور `AXIS_ROLL = 0` و`AXIS_PITCH = 1` و`AXIS_YAW = 2` و`AXIS_COUNT = 3` |
| السرعة | `STALL_SPEED_MS = 8` و`REFERENCE_SPEED_MS = 14` و`ACCEL_FILTER_TAU_S = 0.3` |
| في الجو/على الأرض | `AIRBORNE_HEIGHT_M = 3` و`AIRBORNE_CONFIRM_MS = 500` و`GROUND_STILL_MS = 2000` و`GROUND_ACCEL_TOLERANCE_G = 0.1` |
| المنظِّم | `ANGLE_GAIN = {4, 4, 2}` 1/s و`MAX_RATE_DPS = {120, 60, 30}` و`RATE_TAU_S = {0.15, 0.20, 0.30}` و`RATE_INTEGRAL_GAIN = {2, 2, 1}` و`MAX_DEFLECTION_US = {400, 400, 400}` و`DAMPING_COMPENSATION = 0.5` |
| فعالية الأسطح | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs و`EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}` و`EFFECTIVENESS_MAX = {30, 15, 6}` و`RESPONSE_DELAY_MS = 40` و`RLS_FORGETTING = 0.995` و`ESTIMATOR_PERIOD_MS = 20` و`ESTIMATOR_PREFILTER_HZ = 2` و`MIN_EXCITATION_US = 30` |
| الانهيار | `DECEL_WARN_MS2 = 2` و`DECEL_CONFIRM_MS = 300` و`LOW_ENERGY_PITCH_DEG = 5` و`NOSE_DROP_RATE_DPS = 60` و`WING_DROP_RATE_DPS = 120` و`STALL_NOSE_UP_COMMAND_US = 50` و`LOW_EFFECTIVENESS_RATIO = 0.35` و`LOW_SPEED_MARGIN = 1.25` و`LOW_SPEED_EXIT_MARGIN = 1.5` و`LOW_ENERGY_THROTTLE_PERCENT = 80` و`LOW_ENERGY_MAX_PITCH_DEG = 5` و`STALL_THROTTLE_PERCENT = 100` و`STALL_MAX_PITCH_DEG = −5` و`STALL_MAX_BANK_DEG = 10` و`STALL_AILERON_LIMIT_US = 150` و`RECOVERY_HOLD_MS = 1000` |
| الإقلاع | `TAKEOFF_HAND_LAUNCH = false` و`TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50` و`TAKEOFF_THROTTLE_PERCENT = 100` و`LAUNCH_ACCEL_G = 1` و`LAUNCH_DETECT_MS = 50` و`ROTATE_SPEED_MS = 10` و`ROTATE_FALLBACK_MS = 1500` و`CLIMB_PITCH_DEG = 12` و`TAKEOFF_TARGET_ALTITUDE_M = 30` و`TAKEOFF_CLIMB_FALLBACK_MS = 10000` و`LAUNCH_TIMEOUT_MS = 8000` و`HEADING_HOLD_GAIN = 2` |
| الهبوط | `APPROACH_SINK_RATE_MS = 1` و`APPROACH_THROTTLE_PERCENT = 25` و`APPROACH_BASE_PITCH_DEG = −3` و`APPROACH_MIN_PITCH_DEG = −10` و`APPROACH_MAX_BANK_DEG = 20` و`GO_AROUND_THROTTLE_PERCENT = 80` و`SINK_TO_PITCH_GAIN = 4` و`FLARE_HEIGHT_M = 2` و`FLARE_SINK_RATE_MS = 0.3` و`FLARE_MAX_PITCH_DEG = 8` و`TOUCHDOWN_ACCEL_G = 0.5` و`TOUCHDOWN_HEIGHT_M = 0.3` و`TOUCHDOWN_STILL_MS = 500` و`TOUCHDOWN_STILL_RATE_DPS = 5` و`ROLLOUT_MS = 5000` |

---

## ‏namespace `FeedbackMath`

**الملف:** `autopilot/feedback/FeedbackMath.h` · **يعتمد على:** `<math.h>`

| الدالة | الوصف |
|---|---|
| `float wrap180(float deg)` | زاوية ضمن `(−180, 180]`: فرق الاتجاهين 350° و10° هو −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | التقييد إلى `[−limit, limit]` |

---

## ‏`FlightSnapshot`

**الملف:** `autopilot/feedback/FlightSnapshot.h` · **النوع:** struct

كل ما تعرفه الحلقة عن الطائرة في دورة واحدة. والإشارات إشارات طيران.

| المجموعة | الحقول |
|---|---|
| الزمن/الحالة | `timeUs` و`armed` و`linkLost` |
| الوضعية | `imuValid` و`rollDeg` و`pitchDeg` و`yawDeg` و`rollRateDps` و`pitchRateDps` و`yawRateDps` و`accelXg/Yg/Zg` |
| الارتفاع | `baroValid` و`altitudeM` (من نقطة التشغيل) و`climbRateMs` و`heightAglValid` و`heightAglM` (مقياس مدى مستقبلي) |
| السرعة | `airspeedValid` و`airspeedMs` (أنبوب بيتو مستقبلي) و`gpsValid` و`groundSpeedMs` |
| أهداف الوضع | `stabilizationActive` (القيمة false = MANUAL: تعلّم فقط) و`targetRollDeg` و`targetPitchDeg` |
| الأوامر، µs | `stick*Us` — مساهمة الطيار؛ و`command*Us` — النتيجة التي ذهبت فعلًا إلى الأسطح |
| الخانق، % | `pilotThrottlePercent` و`throttlePercent` (فعليًّا إلى ESC) |
| الفلابات | `flapsUs` و`flapsMoving` |

---

## ‏`FeedbackOutput`

**الملف:** `autopilot/feedback/FeedbackOutput.h` · **النوع:** struct

| الحقل | الوصف |
|---|---|
| `float deflectionUs[3]` | انحرافات الأسطح بحسب المحاور، µs (بإشارات `ControlCommand`) |
| `bool axisEnabled[3]` | `false` — المحور غير مُتحكَّم به، وتبقى الدفة بيد الطيار |
| `float throttleOverridePercent` | الخانق المطلق لمرحلة الطيران؛ و`< 0` — غير محدد |
| `float throttleFloorPercent` | الحد الأدنى للخانق (الحماية من الانهيار)؛ و`< 0` — لا يوجد |
| `targetRollDeg` و`targetPitchDeg` | الأهداف النهائية بعد التقييدات (للتصحيح) |
| `const char* reason` | وصف موجز للسجل/OLED |

---

## ‏`PhaseTargets`

**الملف:** `autopilot/feedback/PhaseTargets.h` · **النوع:** struct

المخرج المشترك لـ `TakeoffSequencer` و`LandingSequencer` — «ماذا»، لا «كيف».

| الحقل | الافتراضي | الوصف |
|---|---|---|
| `active` | `false` | المرحلة تقود الطائرة الآن |
| `targetRollDeg` و`targetPitchDeg` | 0 | الأهداف |
| `controlRoll` و`controlPitch` | `true` | `false` — لا يُمَسّ المحور (على العجلات يحدد جهاز الهبوط الميل) |
| `holdHeading` و`headingDeg` | `false`، 0 | الحفاظ على الاتجاه بالدفة والعجلة |
| `throttlePercent` | −1 | −1 — خانق الطيار |
| `reason` | `""` | وصف |

---

## ‏`SpeedEstimator`

**الملف:** `autopilot/feedback/SpeedEstimator.h`

السرعة (الجوية > الأرضية من GPS > مجهولة) والتسارع الطولي من IMU:
`dV/dt = g · (ax − sin θ)` عبر مرشّح تمرير منخفض `ACCEL_FILTER_TAU_S` — فتظهر عبارة
«السرعة تتناقص» حتى دون حساس سرعة.

| الدالة | الوصف |
|---|---|
| `void update(const FlightSnapshot&)` | خطوة؛ وإذا كان `dt ≤ 0` أو `> 0.5 s` منذ الاستدعاء السابق (وبالنسبة إلى الأول — منذ `timeUs = 0`) — تُتجاوَز |
| `bool hasSpeed() const` و`float getSpeed() const` و`Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const` و`float getAcceleration() const` | m/s²، و«+» — تسارع؛ ودون IMU — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`، مقيَّدًا إلى `0.05..4`؛ ودون سرعة — 1 |

---

## ‏`AirborneDetector`

**الملف:** `autopilot/feedback/AirborneDetector.h`

هل الطائرة في الجو: فلا معنى للتعلّم وتراكم التكامل والبحث عن الانهيار إلا أثناء
الطيران.

| الدالة | الوصف |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | غير مُفعَّلة — إعادة الضبط إلى «على الأرض». ويجب أن يثبت المرشَّح لتغيّر الحالة مدة `AIRBORNE_CONFIRM_MS` (الإقلاع) أو `GROUND_STILL_MS` (الهبوط) |
| `void force(bool)` | التحديد صراحةً (يعرفه الإقلاع والهبوط) |
| `void reset()` | على الأرض |
| `bool isAirborne() const` | |

«يشبه الطيران»: الارتفاع بمقياس المدى أو مقياس الضغط > `AIRBORNE_HEIGHT_M`، أو السرعة > `ROTATE_SPEED_MS`.
«يشبه الأرض»: منخفض، والسرعات الزاوية لكل المحاور < `TOUCHDOWN_STILL_RATE_DPS`، و|a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## ‏`ControlEffectivenessEstimator`

**الملف:** `autopilot/feedback/ControlEffectivenessEstimator.h`

محور واحد. النموذج: **التسارع الزاوي = b·السطح(t − التأخير) + a·ω + c**.
`b` فعالية السطح (°/s² لكل µs، والإشارة هي اتجاه الاستجابة)، و`a` هو
التخميد (1/s، وهو عادةً < 0)، و`c` عزم ثابت (ضبط تلقائي). ويُتعلَّم `b` عند سرعة مرجعية:
`b = b_ref · (V/V_ref)²` و`a = a_ref · V/V_ref`. والتقدير بطريقة المربعات الصغرى
التكرارية مع النسيان (`λ = 0.995`، وذاكرة نحو 4 s).

| الدالة | الوصف |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | يستدعي `reset()` |
| `void reset()` | θ = (prior, 0, 0)؛ والتباين المشترك: b ± prior، وa ± 5، وc ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | يُستدعى في كل دورة: يجمع المتوسطات على فترة `ESTIMATOR_PERIOD_MS`، وعند نهاية الفترة — التسارع من فرق الجيروسكوب، وتأخير الأمر، ومرشّح تمرير منخفض مشترك على الطرفين، وخطوة RLS (إن سُمح بالتعلّم وكان هناك تحريك) |
| `getEffectiveness()` | قيمة `b` عند السرعة الحالية |
| `getReferenceEffectiveness()` | قيمة `b` عند السرعة المرجعية |
| `getDamping()` | قيمة `a` عند السرعة الحالية |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (القيمة 0 إن كان `\|b\|` صغيرًا) |
| `getEffectivenessSigma()` | σ لتقدير `b` عند السرعة الحالية |
| `bool isConfident() const` | ≥ 50 خطوة RLS، و`\|b\|` ≥ الحد الأدنى وσ < 0.3·`\|b\|` |
| `getAngularAccel()` | التسارع في آخر فترة (للتصحيح) |

الخصائص:

- الفترة الأطول من `MAX_GAP_MS = 200` (توقفت الحلقة) — تبدأ البيانات من جديد
  (يُعاد ضبط سجل التأخير والمرشّح).
- لا يتعلّم إلا عند **التحريك**: مدى تأرجح متوسطات الأوامر على 16 فترة
  (نحو 0.3 s) ≥ `MIN_EXCITATION_US`؛ وإلا يتجمد التقدير.
- معالجة قيم float بعد كل خطوة: تماثل `P`، وسقف للتباينات (×10 من الابتدائية)،
  وحدّ لـ `b` (‏`±EFFECTIVENESS_MAX`) ولـ `a` (‏`−40..5`).

---

## ‏`AxisModel`

**الملف:** `autopilot/feedback/AdaptiveRateController.h` · **النوع:** struct

ما يُعرف عن استجابة المحور، من أجل المنظِّم: `effectiveness` (‏b، والافتراضي 1)
و`damping` (‏a، والقيمة 0 — دون تعويض) و`bias` (‏c، والقيمة 0).

---

## ‏`AdaptiveRateController`

**الملف:** `autopilot/feedback/AdaptiveRateController.h`

منظِّم لمحور واحد، في ثلاث مراحل:

<div dir="ltr">

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

</div>

| الدالة | الوصف |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | التكامل والتشبّع والخرج — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | المرحلة 1 (أقصر طريق للاتجاه) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | المرحلتان 2–3، وتُعيد الانحراف بوحدة µs |
| `getDesiredRate()` و`getIntegral()` و`getOutput()` و`isSaturated()` | الحالة |

الثوابت: لا يقل `|b|` عن `EFFECTIVENESS_MIN` (مع إبقاء إشارة b)؛
ويُحفظ التكامل بوحدة °/s (فيبقى صحيحًا عند تغيّر `b`) و**لا يتراكم في اتجاه
الحد** (anti-windup بحسب اتجاه تشبّع الخطوة السابقة)؛ وعند `dt ≤ 0` لا يتغير التكامل.

---

## ‏`StallGuard`

**الملف:** `autopilot/feedback/StallGuard.h`

الحماية من فقدان السرعة والانهيار. والمستويات `Level::{Normal, LowEnergy, Stall}`.

| الدالة | الوصف |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | على الأرض أو دون IMU — إعادة الضبط إلى Normal |
| `Level getLevel() const` و`const char* getLevelName() const` | `"OK"` و`"LOW_ENERGY"` و`"STALL"` |
| `const char* getReason() const` | آخر مؤشر عمل |
| `float maxPitchDeg() const` | Stall: ‏−5°، وLowEnergy: ‏5°، وفي غير ذلك 90° |
| `float maxBankDeg() const` | Stall: ‏10°، وفي غير ذلك 180° |
| `float maxAileronUs() const` | Stall: ‏150 µs، وفي غير ذلك `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall ‏100 %، وLowEnergy ‏80 %، وفي غير ذلك/دون اتصال −1 |
| `void reset()` | Normal |

‏`StallGuard::ControlState` — `pitchEffectivenessKnown` و`pitchEffectiveness`
(مقدار تقدير b على محور الميل).

مؤشرات **LowEnergy**: تباطؤ مؤكَّد (`DECEL_CONFIRM_MS`)
يفوق `DECEL_WARN_MS2` مع ميل > 5°؛ وسرعة < `1.25·Vs`؛ وفعالية موثوقة للرافعة
< 35 % من القيمة المسبقة. ومؤشرات **Stall**: سرعة < Vs؛ والأنف يهبط أسرع من 60 °/s
مع رافعة «إلى أعلى» > 50 µs؛ وعند طاقة منخفضة يهوي الجناح أسرع من 120 °/s
بعكس الجنيحات. وتُرفع التدابير بعد `RECOVERY_HOLD_MS` وفقط عندما تستعيد الطاقة
(السرعة ≥ `1.5·Vs`، ودون حساس سرعة — التسارع ≥ 0).

---

## ‏`TakeoffSequencer`

**الملف:** `autopilot/feedback/TakeoffSequencer.h`

الإقلاع على مراحل. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
والمخطط في [ARCHITECTURE.md §7](../ARCHITECTURE.md#الإقلاع-والهبوط-حلقة-التغذية-الراجعة-غير-موصولة).

| الدالة | الوصف |
|---|---|
| `void request(nowMs)` | ← `WaitThrottle` |
| `void cancel()` | مرحلة نشطة ← `Aborted`؛ وتُصفَّر الأهداف |
| `void update(snapshot, speed, nowMs)` | انتقال واحد على الأكثر في الدورة، ثم أهداف المرحلة الجديدة |
| `void reset()` | ← `Idle` |
| `getTargets()` و`getState()` و`getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

أهداف المراحل: الانتظار — خانق 0 والأسطح بيد الطيار؛ وشوط الإقلاع — خانق 100 %،
والجناحان مستويان، ولا يُمَسّ الميل، ويُحافَظ على الاتجاه المثبَّت لحظة البدء؛ والصعود —
خانق 100 %، والجناحان مستويان، والميل `CLIMB_PITCH_DEG`. والرمي باليد: التسارع الطولي
`ax − sin θ ≥ LAUNCH_ACCEL_G` لمدة أطول من `LAUNCH_DETECT_MS`.

---

## ‏`LandingSequencer`

**الملف:** `autopilot/feedback/LandingSequencer.h`

الهبوط على مراحل. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| الدالة | الوصف |
|---|---|
| `void request(nowMs)` | ← `Approach` |
| `void cancel()` و`void reset()` و`update(snapshot, nowMs)` | كما في الإقلاع |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — تُعطَّل هنا الحماية من الانهيار |
| `bool isOnGround() const` | Rollout / Complete |

يُشتق الميل أثناء النزول والتسوية من خطأ السرعة الرأسية:
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`، مقيَّدًا إلى
`[min, FLARE_MAX_PITCH_DEG]`؛ ودون مقياس ضغط — الزاوية الأساسية. والارتفاع من مقياس
المدى وإلا فمن مقياس الضغط. اللمس: طفرة |a − 1g| ≥ 0.5g أو «منخفض ولا يدور» طوال
`TOUCHDOWN_STILL_MS`. وفي شوط الهبوط يُثبَّت الاتجاه لحظة اللمس.

---

## ‏`FeedbackSupervisor`

**الملف:** `autopilot/feedback/FeedbackSupervisor.h`

الحلقة كاملةً. يملك `SpeedEstimator` و`AirborneDetector` وثلاثة
`ControlEffectivenessEstimator` وثلاثة `AdaptiveRateController` و`StallGuard`
و`TakeoffSequencer` و`LandingSequencer`.

| الدالة | الوصف |
|---|---|
| `bool requestTakeoff()` | فقط وهي مُفعَّلة، والاتصال قائم، وعلى الأرض؛ ويُلغي الهبوط |
| `bool requestLanding()` | فقط وهي مُفعَّلة، والاتصال قائم، وفي الجو؛ ويُلغي الإقلاع |
| `void cancelPhase()` | إلغاء المرحلة |
| `const FeedbackOutput& update(const FlightSnapshot&)` | دورة (والترتيب في [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-حلقة-التغذية-الراجعة-غير-موصولة)) |
| `getOutput()` و`isAirborne()` و`getSpeedEstimator()` و`getEstimator(axis)` و`getController(axis)` و`getStallGuard()` و`getTakeoff()` و`getLanding()` | الحالة للسجل والاختبارات |
| `void printStatus(Print& out) const` | سطر للحالة + سطر لكل محور (`b ± σ`، و`*` — موثوق، و`a` و`c` و`I` والخرج) |

القواعد الأساسية:

- **غير مُفعَّلة** — جميع المحاور معطَّلة، و`reason = "غير مُفعَّلة"`؛ وARM/DISARM
  (رحلة جديدة) يمسح كل ما تعلّمته.
- **فقدان الاتصال** يُلغي المراحل؛ ولا يُمَسّ الخانق (إذ يعمل
  failsafe الخاص بالبرنامج الثابت).
- **مفارقة الأرض** تصفّر التقديرات والمنظِّمات (فما «شوهد» على
  العجلات لا يصلح).
- التقدير السالب الموثوق لـ `b` لا يدخل المنظِّم **أبدًا** — فيعمل المحور على النموذج
  المسبق، ويحمل `reason` التحذير «… يستجيب للسطح بالعكس؟ تحقق على الأرض».
- التكامل مجمَّد على الأرض، عدا الاتجاه في شوط الإقلاع/الهبوط.
- الانعطاف المنسَّق (مع سرعة معلومة في الجو): يُضاف إلى معدل الميل المطلوب
  `+ g/V · sin φ · tg φ`، وإلى معدل الانعراج `g/V · sin φ` (والميلان مقيَّد بـ ±60°).
- أولوية `reason`: الانهيار > الطاقة المنخفضة > المرحلة > تحذير الإشارة >
  «التثبيت»/«يدوي (تعلّم)».

</div>
