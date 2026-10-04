<div dir="rtl">

# ‏AUTOPILOT — الأوضاع والملاحة والمفاتيح

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/autopilot.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

يتلقى الطيار الآلي عصي الطيار والمفاتيح/المقابض (`PilotInputs`) والحساسات،
ويُخرج **الأمر النهائي للأسطح** (`getCommand()`) وخانق الوضع
(`applyThrottle()`). ودون حساس لازم يتصرف الوضع بأمان (تبقى الأسطح بيد الطيار
أو في الوضع المحايد) بدل أن ينهار. وما يفعله كل وضع للطيار في
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). وفي الطيران لم يُستخدم حتى الآن سوى الوضع اليدوي؛
أما الطيار الآلي فقد اختُبر على المنصة وبالاختبارات والمحاكاة بحلقة مغلقة
(`test/native/test_sim`).

---

## ‏`AutopilotMode` و`Feature` و`Knob`

**الملف:** `autopilot/AutopilotTypes.h`

‏`enum AutopilotMode : uint8_t` (دون نطاق — فالرموز الرقمية تذهب إلى JSON الخاص بـ
`/api/setmode` و`/api/status` وإلى `MavlinkModes`):

| القيمة | الرمز | الاختصار (OLED) | الجوهر |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | الأسطح = العصي |
| `MODE_STABILIZE` | 1 | STAB | العصا هي زاوية الدوران الجانبي/الميل |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | برنامج إقلاع يقوده خانق الطيار |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + الارتفاع بالرافعة |
| `MODE_ACRO` | 4 | ACRO | العصا هي السرعة الزاوية |
| `MODE_CRUISE` | 5 | CRZ | المسار + الارتفاع + خانق تلقائي |
| `MODE_LOITER` | 6 | LOIT | دوائر فوق النقطة التي فُعِّل فيها |
| `MODE_RTH` | 7 | RTH | إلى نقطة الانطلاق، ودوائر فوقها |
| `MODE_LAUNCH` | 8 | LNCH | إطلاق باليد |
| `MODE_AUTO_LAND` | 9 | LAND | انسياب + تسوية |
| `MODE_SOARING` | 10 | SOAR | تيارات حرارية دون محرك |
| `MODE_RESCUE` | 11 | RESQ | الجناحان مستويان، والأنف إلى أعلى، والخانق |
| `MODE_COUNT` | 12 | | الحد (يتجاهل `setMode()` القيم ≥ هذا) |

‏`enum class Feature : uint8_t` — وظائف المفاتيح: `FLAPS` و`AIRBRAKE` و
`AUTO_TRIM` و`TURN_COORDINATION` و`MOTOR_KILL` و`BEEPER` و`PAYLOAD_DROP` و
`GEOFENCE` و`HOME_RESET` و`CAMERA_STAB` و`COUNT`.

‏`enum class Knob : uint8_t` — المقابض: `STAB_GAIN` و`MAX_BANK` و
`CRUISE_SPEED` و`FLAPS` و`CAMERA_TILT` و`RATES` و`LOITER_RADIUS` و`COUNT`.

‏`namespace AutopilotNames` — `mode()` و`modeShort()` (حتى 5 أحرف) و
`feature()` و`knob()`: أسماء للسجل وOLED ولوحة المعلومات وMAVLink.

### ‏`PilotInputs`

حالة المفاتيح والمقابض في دورة واحدة.

| العضو | الوصف |
|---|---|
| `bool has(Feature) const` | الوظيفة مفعَّلة |
| `float knob(Knob) const` | موضع المقبض −1…+1 |
| `bool isBound(Knob) const` | المقبض موجود في جدول الربط |
| `float knobValue(Knob, min, default, max) const` | بالوحدات: المركز هو `default` والطرفان `min`/`max`؛ وإن لم يكن مربوطًا — `default` |

---

## ‏`Binding` و`Bind` و`BindingCheck`

**الملف:** `autopilot/ControlBinding.h` · الجدول — `config/Controls.h`

‏`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`،
و`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. وسطور الجدول هي مصانع
`namespace Bind` (كلها `constexpr`):

| المصنع | المعنى |
|---|---|
| `modes(ch, up, middle, down)` و`modes(ch, up, down)` | مفتاح اختيار الوضع (المنطقة بحسب `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | وضع يُفرَض من فوق ما دامت القناة ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | وظيفة ما دامت القناة ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | مقبض، `(us − 1500) / 500`، ومقيَّد بـ ±1 |

‏`namespace BindingCheck` — دوال `constexpr` تعاودية (تُبنى نواة ESP32 بمعيار
C++11): `channelIsFree` و`channelsFree` و`channelsUnique` و
`modeSwitchCount` و`atMostOneModeSwitch`. وتُستخدم في `static_assert` داخل
`Controls.h`.

---

## ‏`PilotSwitches`

**الملف:** `autopilot/PilotSwitches.h` · **يعتمد على:** `Autopilot*` و`RcChannelState` وجدول الربط

| الدالة | الوصف |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | جدول خاص (الاختبارات والمحاكاة) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | الجدول `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | جمع `PilotInputs` وتمريرها إلى `autopilot->setInputs()`؛ و`setMode()` — **فقط عند تغيّر محصّلة المفاتيح** (فلا يُمحى وضع ضُبط من لوحة المعلومات/محطة الأرض في كل دورة). ولا يستدعيها `FlightController` إلا والاتصال قائم |
| `void printBindings() const` | التخطيط في Serial عند التشغيل: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (أعلى / وسط / أسفل)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"` و`"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | عدد المناطق 2: < 1500 / ≥ 1500؛ وعدد المناطق 3: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()` و`binding(i)` | للقياس عن بُعد والاختبارات |

يعلو `Bind::mode` على `Bind::modes`؛ ومن بين عدة `Bind::mode` مفعَّلة يفوز السطر
الأعلى.

---

## ‏`Autopilot`

**الملف:** `autopilot/Autopilot.h` · **يعتمد على:** `PidController` و`Navigation` و`AltitudeSpeedController` و`LaunchController` و`SoaringController` و`AutoTrim` والحساسات (كلها يمكن أن تكون فارغة)

### دورة الحياة

| الدالة | الوصف |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | PID الدوران الجانبي/الميل بالمعاملات Kp 5 وKi 0.5 وKd 0.5، والخرج ±500 µs |
| `bool begin()` | تحميل المعدِّل؛ و`false` مع رسالة إن لم توجد IMU أو مقياس ضغط |
| `void setInputs(const PilotInputs&)` | مفاتيح هذه الدورة ومقابضها (قبل `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | مرة في كل دورة: الحساسات (دائمًا) ← الملاحة ونقطة الانطلاق ← حفظ المعدِّل على الأرض ← failsafe ← السياج الجغرافي ← الوضع ← تنسيق الانعطاف ← الضبط التلقائي |
| `ControlCommand getCommand() const` | أوامر الأسطح النهائية (roll/pitch/yaw، µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | خانق الوضع: `PILOT` — خانق الطيار؛ و`AUTO` — الخانق الخاص؛ و`AT_LEAST` — لا يقل عن الخاص (الإقلاع التلقائي). أما ARM و`MOTOR_KILL` فيعالجهما `FlightController` |

### الأوضاع

| الدالة | الوصف |
|---|---|
| `void setMode(AutopilotMode)` | الوضع نفسه أو ≥ `MODE_COUNT` — لا شيء؛ وإلا يُعاد ضبط PID وآلات الحالات، والأهداف = المسار والارتفاع الحاليان، ومركز الدوائر = النقطة الحالية (مع GPS)، وفي RTH — ارتفاع العودة |
| `getMode()` و`getModeName()` | الاسم: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` عند فقدان الاتصال، وإلا الوضع |
| `isFailsafeActive()` و`isFailsafeGliding()` و`isFailsafeReturning()` | failsafe فوق الوضع |
| `isAutoThrottle()` و`getThrottleCorrection()` | خانق الوضع (بالنسبة المئوية، للسجل ولوحة المعلومات) |
| `getLaunchState()` و`getSoaringState()` | آلتا حالات LAUNCH وSOARING |

### المخرجات والتشخيص

| الدالة | الوصف |
|---|---|
| `getRollCorrection()` و`getPitchCorrection()` و`getYawCorrection()` | الأمر − العصي، µs |
| `getDesiredRoll()` و`getDesiredPitch()` و`getTargetAltitude()` | الأهداف |
| `const NavStatus& getNavStatus()` | GPS، ونقطة الانطلاق، والموضع، والمسافة/الاتجاه إلى نقطة الانطلاق، والمسار والمسار المستهدف، وسرعة الملاحة، والسياج الجغرافي، والانهيار |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | الارتفاع بمقياس الضغط (بالأمتار من نقطة التشغيل) |
| `getInputs()` و`getAutoTrim()` | للقياس عن بُعد |
| `getImuSensor()` … `getAirspeedSensor()` | الحساسات (قد تكون `nullptr`) |
| `getRollPid()` و`getPitchPid()` و`setPIDGains(...)` | PID (لوحة المعلومات، ومعاملات MAVLink) |

### الآلية الداخلية

- ‏`stabilize()` — PID للزاوية يؤخذ حدّ D فيه من الجيروسكوب، ويُضرب في
  `Knob::STAB_GAIN`؛ ولا يتراكم المكامِل إلا مع ARM وخطأ < `STAB_INTEGRATOR_ZONE_DEG`.
  و`stabilizeOrManual()` — دون IMU تبقى الأسطح بيد الطيار؛ و`stabilizeOrNeutral()` —
  دون IMU الوضع المحايد (الأوضاع التلقائية).
- ‏`imuReady()` = IMU موجودة ومتاحة وبلا مشكلة في فحص ما قبل الطيران.
- السرعة المستخدمة في الملاحة: أنبوب بيتو ← GPS ← `NAV_ASSUMED_SPEED_MS`.
- ‏`looksLanded()` — قرب الأرض بحسب مقياس الضغط، ودون سرعة رأسية تقريبًا،
  وأبطأ من عتبة أنبوب بيتو/GPS: عندها فقط يُكتب المعدِّل في الفلاش.
- ‏Failsafe: مع GPS ونقطة انطلاق — RTH بالمحرك، وإلا انسياب؛ ولا يُترك RTH بدأ
  بسبب فقدان قصير لـ GPS.

---

## ‏`Geo` و`Guidance` و`GeoPoint`

**الملف:** `autopilot/Navigation.h`

مستوى محلي «شمال/شرق» بالأمتار (إسقاط متساوي المسافات المستطيلة — وعلى مدى
الكيلومترات يكون الخطأ جزءًا يسيرًا من واحد بالمئة).

| الدالة | الوصف |
|---|---|
| `Geo::wrap180` و`Geo::wrap360` | تطبيع الزوايا |
| `Geo::offsetNE(a, b, north, east)` و`distance(a, b)` و`bearing(a, b)` | الإزاحة والمسافة والاتجاه 0..360 |
| `Geo::moved(a, north, east)` | نقطة بعد إزاحة |
| `Geo::fromGps(GpsData)` | `GeoPoint` من GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | الدوران الجانبي لخطأ المسار (`NAV_COURSE_GAIN`)، مقيَّد |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | مسار حقل المتجهات نحو الدائرة (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | الدوران الجانبي الاستباقي للدائرة: atan(V²/(g·R)) |

## ‏`AltitudeSpeedController`

**الملف:** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| الدالة | الوصف |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | السرعة الرأسية المطلوبة = `NAV_ALT_GAIN`·الخطأ (≤ `NAV_MAX_CLIMB/SINK`)؛ والميل = الاستباق asin(Vz/V) + PI على خطأ Vz، ضمن `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | مع أنبوب بيتو — PI على السرعة الجوية حول `cruisePct`؛ ودونه — `cruisePct`؛ + `THROTTLE_PER_CLIMB_PCT` للصعود المطلوب |
| `reset()` و`getWantedClimb()` | |

## ‏`LaunchController`

**الملف:** `autopilot/LaunchController.h`

‏`State`: ‏`IDLE → READY` (رُفع الخانق) `→ THROWN` (حمل زائد > `LAUNCH_ACCEL_G`
لمدة أطول من `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (بعد `LAUNCH_MOTOR_DELAY_MS`: المحرك،
والميل `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` أو
`LAUNCH_ALTITUDE_M`). تحريك العصي قبل الرمي — إلغاء. الدوال: `update(...)`،
و`reset()`، و`getState()`، و`motorOn()`، و`pitchTargetDeg()`، و`stateName()`.

## ‏`SoaringController`

**الملف:** `autopilot/SoaringController.h`

‏`State`: ‏`GLIDE ⇄ THERMAL` (مقياس الصعود > `SOAR_THERMAL_CLIMB_MS` لمدة أطول من
`SOAR_THERMAL_CONFIRM_MS` / المتوسط < `SOAR_EXIT_CLIMB_MS` خلال
`SOAR_EXIT_WINDOW_MS`)، `→ MOTOR_CLIMB` (دون `SOAR_MIN_ALTITUDE_M`، حتى
`SOAR_MAX_ALTITUDE_M`)، `→ RETURN` (أبعد من `SOAR_MAX_DISTANCE_M`، حتى 70 % منها).
الدوال: `update(climb, alt, distHome, dt, now)` و`reset(now)` و
`getState()` و`motorOn()` و`getAverageClimb()` و`stateName()`.

## ‏`AutoTrim`

**الملف:** `autopilot/AutoTrim.h` · التخزين — `Preferences` (‏NVS / فلاش STM32)، ومساحة الأسماء `"autotrim"`

| الدالة | الوصف |
|---|---|
| `void load()` | المعدِّل من NVS (وإن لم يوجد — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += الأمر · `AUTOTRIM_RATE` · dt، حتى ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | الكتابة إن تغيّر (يستدعيها `Autopilot` بعد DISARM على الأرض) |
| `reset()` و`getRoll()` و`getPitch()` | |

---

## ‏`PidController`

**الملف:** `autopilot/PidController.h` · **يعتمد على:** `Config` (قيمة `dt` الاسمية)

| الدالة | الوصف |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)` و`getKp/Ki/Kd()` | المعاملات |
| `setLimits(minOut, maxOut)` | حدّ الخرج (±500 افتراضيًّا) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | الخرج مقيَّدًا إلى `[min, max]` |
| `void reset()` | تصفير المكامِل، ويُحسب `dt` من «الآن» |

<div dir="ltr">

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (السرعة من الحساس!)
out = constrain(P + I + D, min, max)
```

</div>

يؤخذ D على سرعة الكمية المقيسة (الجيروسكوب °/s) لا على مشتقة الخطأ: فلا
ضجيج اشتقاق ولا قفزة عند تغيير القيمة المطلوبة. ويُؤخذ `dt` من
`micros()`؛ وفي أول استدعاء بعد `reset()` أو بعد توقف يزيد على 0.1 s تُستخدم القيمة
الاسمية `LOOP_PERIOD_MS`.

</div>
