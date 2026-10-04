# AUTOPILOT / feedback — फ़ीडबैक लूप (तैयारी)

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/feedback.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

> ⚠️ **तैयारी, फ़र्मवेयर में जुड़ा नहीं है।** न `FlightController`, न `Autopilot`, न
> `main.cpp` ये हेडर शामिल करते हैं। इन्हें बंद-लूप सिमुलेशन (`test/test_feedback`, PC पर
> और बोर्ड पर) और नेटिव यूनिट टेस्ट से जाँचा जाता है। जोड़ने की योजना
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#जोड़ने-की-योजना) में है।

विचार: एक ही गति के लिए चुने गए गुणांकों वाले कोण-PID की जगह एक रेगुलेटर, जो
**विमान की प्रतिक्रिया** पर बंद हो, जिसमें उड़ान में सीखा गया अक्ष का मॉडल, स्टॉल से
सुरक्षा और सेंसरों से चलने वाले टेकऑफ़/लैंडिंग के चरण हों। इकलौता इनपुट `FlightSnapshot`
है, इकलौता आउटपुट `FeedbackOutput`।

सभी मॉड्यूल header-only हैं; `FeedbackModules.h` उन्हें एक पंक्ति में शामिल कर लेता है।

---

## namespace `FeedbackConfig`

**फ़ाइल:** `autopilot/feedback/FeedbackConfig.h`

लूप के सभी स्थिरांक (जुड़ने पर `Config.h` में चले जाएँगे)। “прикидка” (“मोटा अनुमान”) चिह्नित मान ~1 kg और 1.2 m
पंखों के फैलाव वाले मॉडल के लिए हैं। `[AXIS_COUNT]` के ऐरे अक्ष से इंडेक्स होते हैं।

| समूह | स्थिरांक |
|---|---|
| सामान्य | `GRAVITY = 9.80665`; अक्ष `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| गति | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| हवा में/ज़मीन पर | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| रेगुलेटर | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| कंट्रोल सरफ़ेस की प्रभावशीलता | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| स्टॉल | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| टेकऑफ़ | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| लैंडिंग | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**फ़ाइल:** `autopilot/feedback/FeedbackMath.h` · **निर्भर है:** `<math.h>`

| फ़ंक्शन | विवरण |
|---|---|
| `float wrap180(float deg)` | कोण `(−180, 180]` में: हेडिंग 350° और 10° का अंतर −20° है |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | `[−limit, limit]` तक सीमित करना |

---

## `FlightSnapshot`

**फ़ाइल:** `autopilot/feedback/FlightSnapshot.h` · **प्रकार:** struct

एक चक्र में लूप को विमान के बारे में जो कुछ पता है। चिह्न विमानन के हैं।

| समूह | फ़ील्ड |
|---|---|
| समय/स्थिति | `timeUs`, `armed`, `linkLost` |
| स्थिति (एटीट्यूड) | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| ऊँचाई | `baroValid`, `altitudeM` (चालू करने के बिंदु से), `climbRateMs`, `heightAglValid`, `heightAglM` (भविष्य का रेंजफ़ाइंडर) |
| गति | `airspeedValid`, `airspeedMs` (भविष्य की पिटो ट्यूब), `gpsValid`, `groundSpeedMs` |
| मोड के लक्ष्य | `stabilizationActive` (false = MANUAL: केवल सीखना), `targetRollDeg`, `targetPitchDeg` |
| कमांड, µs | `stick*Us` — पायलट का योगदान; `command*Us` — अंतिम नतीजा जो असल में कंट्रोल सरफ़ेस पर गया |
| थ्रॉटल, % | `pilotThrottlePercent`, `throttlePercent` (असल में ESC को) |
| फ़्लैप | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**फ़ाइल:** `autopilot/feedback/FeedbackOutput.h` · **प्रकार:** struct

| फ़ील्ड | विवरण |
|---|---|
| `float deflectionUs[3]` | अक्षों के हिसाब से कंट्रोल सरफ़ेस के विक्षेपण, µs (`ControlCommand` के चिह्न) |
| `bool axisEnabled[3]` | `false` — अक्ष नियंत्रित नहीं है, सरफ़ेस पायलट के पास रहती है |
| `float throttleOverridePercent` | उड़ान के चरण का निरपेक्ष थ्रॉटल; `< 0` — तय नहीं |
| `float throttleFloorPercent` | थ्रॉटल की निचली सीमा (स्टॉल से सुरक्षा); `< 0` — नहीं |
| `targetRollDeg`, `targetPitchDeg` | सीमाओं के बाद के अंतिम लक्ष्य (डीबगिंग) |
| `const char* reason` | लॉग/OLED के लिए संक्षिप्त विवरण |

---

## `PhaseTargets`

**फ़ाइल:** `autopilot/feedback/PhaseTargets.h` · **प्रकार:** struct

`TakeoffSequencer` और `LandingSequencer` का साझा आउटपुट — “क्या”, न कि “कैसे”।

| फ़ील्ड | डिफ़ॉल्ट | विवरण |
|---|---|---|
| `active` | `false` | चरण अभी विमान को नियंत्रित कर रहा है |
| `targetRollDeg`, `targetPitchDeg` | 0 | लक्ष्य |
| `controlRoll`, `controlPitch` | `true` | `false` — अक्ष को न छुएँ (पहियों पर पिच लैंडिंग गियर तय करता है) |
| `holdHeading`, `headingDeg` | `false`, 0 | रडर और पहिये से हेडिंग थामें |
| `throttlePercent` | −1 | −1 — पायलट का थ्रॉटल |
| `reason` | `""` | विवरण |

---

## `SpeedEstimator`

**फ़ाइल:** `autopilot/feedback/SpeedEstimator.h`

गति (हवाई > GPS की ज़मीनी > अज्ञात) और IMU से अनुदैर्ध्य त्वरण:
`dV/dt = g · (ax − sin θ)`, लो-पास फ़िल्टर `ACCEL_FILTER_TAU_S` से गुज़रा हुआ — गति सेंसर
के बिना भी दिखता है कि “गति गिर रही है”।

| मेथड | विवरण |
|---|---|
| `void update(const FlightSnapshot&)` | एक क़दम; पिछली कॉल से `dt ≤ 0` या `> 0.5 s` (पहली कॉल के लिए — `timeUs = 0` से) — छोड़ दिया जाता है |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², “+” — गति बढ़ना; IMU न हो तो — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, `0.05..4` तक सीमित; गति के बिना — 1 |

---

## `AirborneDetector`

**फ़ाइल:** `autopilot/feedback/AirborneDetector.h`

विमान हवा में है या नहीं: सीखना, इंटीग्रल जमा करना और स्टॉल ढूँढना केवल उड़ान में मायने
रखता है।

| मेथड | विवरण |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | ARM नहीं — “ज़मीन पर” में रीसेट। स्थिति बदलने का उम्मीदवार `AIRBORNE_CONFIRM_MS` (टेकऑफ़) या `GROUND_STILL_MS` (लैंडिंग) तक टिकना चाहिए |
| `void force(bool)` | साफ़ तौर पर तय करना (टेकऑफ़/लैंडिंग को पता होता है) |
| `void reset()` | ज़मीन पर |
| `bool isAirborne() const` | |

“उड़ान जैसा”: रेंजफ़ाइंडर या बैरोमीटर से ऊँचाई > `AIRBORNE_HEIGHT_M`, या गति > `ROTATE_SPEED_MS`।
“ज़मीन जैसा”: नीचे, सभी अक्षों की कोणीय गति < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`)।

---

## `ControlEffectivenessEstimator`

**फ़ाइल:** `autopilot/feedback/ControlEffectivenessEstimator.h`

एक अक्ष। मॉडल: **कोणीय त्वरण = b·सरफ़ेस(t − देरी) + a·ω + c**।
`b` कंट्रोल सरफ़ेस की प्रभावशीलता है (प्रति µs °/s², चिह्न — प्रतिक्रिया की दिशा), `a`
अवमंदन है (1/s, आमतौर पर < 0), `c` स्थिर आघूर्ण है (ऑटो-ट्रिम)। `b` संदर्भ गति पर सीखा जाता है:
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`। अनुमान भूलने वाला पुनरावर्ती न्यूनतम वर्ग
(RLS) है (`λ = 0.995`, याददाश्त ~4 s)।

| मेथड | विवरण |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | `reset()` बुलाता है |
| `void reset()` | θ = (prior, 0, 0); सहप्रसरण: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | हर चक्र में बुलाएँ: `ESTIMATOR_PERIOD_MS` के अंतराल में औसत जमा करता है, अंतराल के अंत में — जाइरोस्कोप के अंतर से त्वरण, कमांड की देरी, दोनों पक्षों पर साझा लो-पास फ़िल्टर, RLS का एक क़दम (अगर सीखने की अनुमति है और हिलावट है) |
| `getEffectiveness()` | मौजूदा गति पर `b` |
| `getReferenceEffectiveness()` | संदर्भ गति पर `b` |
| `getDamping()` | मौजूदा गति पर `a` |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0, अगर `\|b\|` छोटा है) |
| `getEffectivenessSigma()` | मौजूदा गति पर `b` के अनुमान का σ |
| `bool isConfident() const` | ≥ 50 RLS क़दम, `\|b\|` ≥ न्यूनतम और σ < 0.3·`\|b\|` |
| `getAngularAccel()` | पिछले अंतराल का त्वरण (डीबगिंग) |

विशेषताएँ:

- `MAX_GAP_MS = 200` से लंबा अंतराल (चक्र रुका रहा) — डेटा नए सिरे से शुरू होता है
  (देरी का इतिहास और फ़िल्टर रीसेट हो जाते हैं)।
- सीखना केवल **हिलावट** पर होता है: 16 अंतरालों (~0.3 s) में औसत कमांड का विस्तार ≥
  `MIN_EXCITATION_US`; वरना अनुमान ठहर जाता है।
- क़दम के बाद float की सफ़ाई: `P` की सममिति, प्रसरणों की छत (शुरुआती से ×10), `b`
  (`±EFFECTIVENESS_MAX`) और `a` (`−40..5`) की सीमा।

---

## `AxisModel`

**फ़ाइल:** `autopilot/feedback/AdaptiveRateController.h` · **प्रकार:** struct

रेगुलेटर के लिए अक्ष की प्रतिक्रिया के बारे में जो पता है: `effectiveness` (b, डिफ़ॉल्ट
1), `damping` (a, 0 — भरपाई न करें), `bias` (c, 0)।

---

## `AdaptiveRateController`

**फ़ाइल:** `autopilot/feedback/AdaptiveRateController.h`

एक अक्ष का रेगुलेटर, तीन चरण:

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| मेथड | विवरण |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | इंटीग्रल, संतृप्ति और आउटपुट — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | चरण 1 (हेडिंग के लिए सबसे छोटा रास्ता) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | चरण 2–3, विक्षेपण लौटाता है, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | स्थिति |

अपरिवर्तनीय नियम: `|b|` `EFFECTIVENESS_MIN` से कम नहीं (b का चिह्न रखते हुए);
इंटीग्रल °/s में रखा जाता है (`b` बदलने पर भी सही रहता है) और **सीमा की ओर नहीं
जुड़ता** (पिछले क़दम की संतृप्ति की दिशा के अनुसार anti-windup); `dt ≤ 0` पर — इंटीग्रल नहीं बदलता।

---

## `StallGuard`

**फ़ाइल:** `autopilot/feedback/StallGuard.h`

गति खोने और स्टॉल से सुरक्षा। स्तर `Level::{Normal, LowEnergy, Stall}`।

| मेथड | विवरण |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | ज़मीन पर या IMU के बिना — Normal में रीसेट |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | आख़िरी बार जो संकेत चला |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, वरना 90° |
| `float maxBankDeg() const` | Stall: 10°, वरना 180° |
| `float maxAileronUs() const` | Stall: 150 µs, वरना `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, वरना/संपर्क के बिना −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` — `pitchEffectivenessKnown`, `pitchEffectiveness`
(पिच पर b के अनुमान का मापांक)।

**LowEnergy** के संकेत: पुष्ट (`DECEL_CONFIRM_MS`) मंदन, जो `DECEL_WARN_MS2` से
ज़्यादा हो और पिच > 5° हो; गति < `1.25·Vs`; एलिवेटर की भरोसेमंद प्रभावशीलता
पूर्व-धारणा के 35 % से कम। **Stall** के संकेत: गति < Vs; एलिवेटर “ऊपर” > 50 µs होने
पर भी नाक 60 °/s से तेज़ गिरती है; कम ऊर्जा पर पंख एलेरॉन के विरुद्ध 120 °/s से तेज़
गिरता है। उपाय `RECOVERY_HOLD_MS` के बाद हटते हैं और केवल तब जब ऊर्जा लौट आई हो (गति ≥
`1.5·Vs`, गति सेंसर के बिना — त्वरण ≥ 0)।

---

## `TakeoffSequencer`

**फ़ाइल:** `autopilot/feedback/TakeoffSequencer.h`

चरणों में टेकऑफ़। `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`।
आरेख [ARCHITECTURE.md §7](../ARCHITECTURE.md#टेकऑफ़-और-लैंडिंग-फ़ीडबैक-लूप-जुड़ा-नहीं-है) में है।

| मेथड | विवरण |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | सक्रिय चरण → `Aborted`; लक्ष्य रीसेट |
| `void update(snapshot, speed, nowMs)` | हर चक्र में अधिकतम एक बदलाव, फिर नए चरण के लक्ष्य |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

चरणों के लक्ष्य: प्रतीक्षा — थ्रॉटल 0, कंट्रोल सरफ़ेस पायलट के पास; दौड़ — थ्रॉटल 100 %,
पंख समतल, पिच को न छुएँ, शुरुआत के क्षण पर दर्ज हेडिंग थामें; चढ़ाई — थ्रॉटल 100 %,
पंख समतल, पिच `CLIMB_PITCH_DEG`। हाथ से फेंकना: अनुदैर्ध्य त्वरण
`ax − sin θ ≥ LAUNCH_ACCEL_G`, `LAUNCH_DETECT_MS` से ज़्यादा देर तक।

---

## `LandingSequencer`

**फ़ाइल:** `autopilot/feedback/LandingSequencer.h`

चरणों में लैंडिंग। `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`।

| मेथड | विवरण |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | टेकऑफ़ की तरह |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — यहाँ स्टॉल से सुरक्षा बंद की जाती है |
| `bool isOnGround() const` | Rollout / Complete |

उतरने और फ़्लेयर के समय पिच ऊर्ध्वाधर गति की त्रुटि से निकलती है:
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, `[min, FLARE_MAX_PITCH_DEG]` तक
सीमित; बैरोमीटर के बिना — बुनियादी कोण। ऊँचाई रेंजफ़ाइंडर से, वरना बैरोमीटर से।
स्पर्श: |a − 1g| ≥ 0.5g का उछाल या `TOUCHDOWN_STILL_MS` तक “नीचे और घूम नहीं रहा”।
रोलआउट में हेडिंग स्पर्श के क्षण पर दर्ज होती है।

---

## `FeedbackSupervisor`

**फ़ाइल:** `autopilot/feedback/FeedbackSupervisor.h`

पूरा लूप। इसके पास `SpeedEstimator`, `AirborneDetector`, तीन
`ControlEffectivenessEstimator`, तीन `AdaptiveRateController`, `StallGuard`,
`TakeoffSequencer` और `LandingSequencer` हैं।

| मेथड | विवरण |
|---|---|
| `bool requestTakeoff()` | केवल ARM, संपर्क और ज़मीन पर होने पर; लैंडिंग रद्द करता है |
| `bool requestLanding()` | केवल ARM, संपर्क और हवा में होने पर; टेकऑफ़ रद्द करता है |
| `void cancelPhase()` | चरण रद्द करना |
| `const FeedbackOutput& update(const FlightSnapshot&)` | एक चक्र (क्रम [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-फ़ीडबैक-लूप-जुड़ा-नहीं-है) में है) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | लॉग और टेस्ट के लिए स्थिति |
| `void printStatus(Print& out) const` | स्थिति की एक पंक्ति + हर अक्ष की एक पंक्ति (`b ± σ`, `*` — भरोसेमंद, `a`, `c`, `I`, आउटपुट) |

मुख्य नियम:

- **ARM नहीं** — सभी अक्ष बंद, `reason = "ARM नहीं"`; ARM/DISARM (नई उड़ान) सब सीखा हुआ
  मिटा देता है।
- **संपर्क टूटना** चरणों को रद्द करता है; थ्रॉटल को नहीं छुआ जाता (फ़र्मवेयर का failsafe
  काम करता है)।
- **ज़मीन छोड़ना** अनुमानों और रेगुलेटरों को रीसेट कर देता है (पहियों पर जो “देखा” गया
  वह काम का नहीं)।
- `b` का भरोसेमंद ऋणात्मक अनुमान **कभी** रेगुलेटर में नहीं जाता — अक्ष पूर्व-धारणा वाले मॉडल
  पर चलता है, और `reason` में चेतावनी होती है “… सरफ़ेस पर उल्टी प्रतिक्रिया? ज़मीन पर
  जाँचें”।
- ज़मीन पर इंटीग्रल जमा रहता है, केवल टेकऑफ़-रोल/रोलआउट में हेडिंग को छोड़कर।
- समन्वित मोड़ (हवा में गति पता होने पर): वांछित पिच-दर में `+ g/V · sin φ · tg φ`
  और यॉ-दर में `g/V · sin φ` जुड़ता है (बैंक ±60° तक सीमित)।
- `reason` की प्राथमिकता: स्टॉल > कम ऊर्जा > चरण > चिह्न की चेतावनी >
  “स्थिरीकरण”/“हाथ का (सीखना)”।
