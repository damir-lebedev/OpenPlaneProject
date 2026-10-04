# AUTOPILOT — मोड, नेविगेशन, स्विच

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/autopilot.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है। यह अनुवाद AI ने किया है और मूल भाषा के जानकारों ने इसकी जाँच नहीं की है। त्रुटियाँ मिलें तो [Damir Lebedev](https://github.com/damir-lebedev) को लिखें या [इश्यू ट्रैकर](https://github.com/damir-lebedev/OpenPlaneProject/issues) में बताएँ।

[← संदर्भ](README.md)

ऑटोपायलट पायलट के स्टिक, स्विच/नॉब (`PilotInputs`) और सेंसर लेता है, और **कंट्रोल सरफ़ेस की
अंतिम कमांड** (`getCommand()`) तथा मोड का थ्रॉटल (`applyThrottle()`) देता है। ज़रूरी सेंसर के
बिना मोड सुरक्षित व्यवहार करता है (कंट्रोल सरफ़ेस पायलट के पास या न्यूट्रल में), क्रैश नहीं
होता। हर मोड पायलट के लिए क्या करता है — [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md) में। उड़ान में अब तक केवल हाथ
का मोड रहा है; ऑटोपायलट को टेस्ट बेंच पर, टेस्ट से और बंद-लूप सिमुलेशन से
(`test/native/test_sim`) जाँचा गया है।

---

## `AutopilotMode`, `Feature`, `Knob`

**फ़ाइल:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (बिना स्कोप — संख्यात्मक कोड `/api/setmode`, `/api/status` के
JSON और `MavlinkModes` में जाते हैं):

| मान | कोड | संक्षेप (OLED) | सार |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | कंट्रोल सरफ़ेस = स्टिक |
| `MODE_STABILIZE` | 1 | STAB | स्टिक रोल/पिच का कोण है |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | पायलट के थ्रॉटल से चलने वाला टेकऑफ़ प्रोग्राम |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + एलिवेटर से ऊँचाई |
| `MODE_ACRO` | 4 | ACRO | स्टिक कोणीय गति है |
| `MODE_CRUISE` | 5 | CRZ | कोर्स + ऊँचाई + ऑटो थ्रॉटल |
| `MODE_LOITER` | 6 | LOIT | जिस बिंदु पर चालू किया उसके ऊपर घेरे |
| `MODE_RTH` | 7 | RTH | घर की ओर, होम के ऊपर घेरे |
| `MODE_LAUNCH` | 8 | LNCH | हाथ से लॉन्च |
| `MODE_AUTO_LAND` | 9 | LAND | ग्लाइड + फ़्लेयर |
| `MODE_SOARING` | 10 | SOAR | मोटर के बिना थर्मल |
| `MODE_RESCUE` | 11 | RESQ | पंख समतल, नाक ऊपर, थ्रॉटल |
| `MODE_COUNT` | 12 | | सीमा (`setMode()` ≥ को अनदेखा करता है) |

`enum class Feature : uint8_t` — स्विच के फ़ंक्शन: `FLAPS`, `AIRBRAKE`,
`AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`, `PAYLOAD_DROP`,
`GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`।

`enum class Knob : uint8_t` — नॉब: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`।

`namespace AutopilotNames` — `mode()`, `modeShort()` (≤ 5 अक्षर),
`feature()`, `knob()`: लॉग, OLED, डैशबोर्ड और MAVLink के नाम।

### `PilotInputs`

एक चक्र में स्विच और नॉब की स्थिति।

| सदस्य | विवरण |
|---|---|
| `bool has(Feature) const` | फ़ंक्शन चालू है |
| `float knob(Knob) const` | नॉब की स्थिति −1…+1 |
| `bool isBound(Knob) const` | नॉब बाइंडिंग तालिका में है |
| `float knobValue(Knob, min, default, max) const` | इकाइयों में: केंद्र `default`, छोर `min`/`max`; बँधा न हो तो — `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**फ़ाइल:** `autopilot/ControlBinding.h` · तालिका — `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`। तालिका की पंक्तियाँ `namespace Bind` की
फ़ैक्टरियाँ हैं (सभी `constexpr`):

| फ़ैक्टरी | अर्थ |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | मोड चुनने वाला स्विच (क्षेत्र `PilotSwitches::zoneOf` से) |
| `mode(ch, m)` | जब तक चैनल ≥ `SWITCH_ON_US` है, ऊपर से लगा मोड |
| `feature(ch, f)` | जब तक चैनल ≥ `SWITCH_ON_US` है, फ़ंक्शन |
| `knob(ch, k)` | नॉब, `(us − 1500) / 500`, ±1 तक सीमित |

`namespace BindingCheck` — पुनरावर्ती `constexpr` (ESP32 का कोर C++11 के रूप में
बनता है): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`। इनका उपयोग `Controls.h` के
`static_assert` में होता है।

---

## `PilotSwitches`

**फ़ाइल:** `autopilot/PilotSwitches.h` · **निर्भर है:** `Autopilot*`, `RcChannelState`, बाइंडिंग तालिका

| मेथड | विवरण |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | अपनी तालिका (टेस्ट, सिमुलेशन) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | तालिका `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | `PilotInputs` इकट्ठा करना, `autopilot->setInputs()` को देना; `setMode()` — **केवल तब जब स्विचों का नतीजा बदला हो** (डैशबोर्ड/GCS से लगाया मोड हर चक्र में नहीं मिटाया जाता)। `FlightController` इसे केवल जीवित संपर्क पर बुलाता है |
| `void printBindings() const` | चालू होते समय Serial में विन्यास: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (ऊपर / बीच / नीचे)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 क्षेत्र: < 1500 / ≥ 1500; 3 क्षेत्र: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | टेलीमेट्री और टेस्ट के लिए |

`Bind::mode`, `Bind::modes` से ऊपर है; चालू कई `Bind::mode` में से सबसे ऊपर की पंक्ति जीतती है।

---

## `Autopilot`

**फ़ाइल:** `autopilot/Autopilot.h` · **निर्भर है:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, सेंसर (सभी nullable)

### जीवन-चक्र

| मेथड | विवरण |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | रोल/पिच का PID Kp 5, Ki 0.5, Kd 0.5, आउटपुट ±500 µs |
| `bool begin()` | ट्रिमर लोड करना; IMU या बैरोमीटर न हो तो `false` और संदेश |
| `void setInputs(const PilotInputs&)` | इस चक्र के स्विच और नॉब (`update` से पहले) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | हर चक्र में एक बार: सेंसर (हमेशा) → नेविगेशन और होम → ज़मीन पर ट्रिमर सहेजना → failsafe → जियोफ़ेंस → मोड → मोड़ का समन्वय → ऑटो-ट्रिम |
| `ControlCommand getCommand() const` | अंतिम कंट्रोल सरफ़ेस कमांड (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | मोड का थ्रॉटल: `PILOT` — पायलट का; `AUTO` — अपना; `AT_LEAST` — अपने से कम नहीं (ऑटो टेकऑफ़)। ARM और `MOTOR_KILL` को `FlightController` सँभालता है |

### मोड

| मेथड | विवरण |
|---|---|
| `void setMode(AutopilotMode)` | वही या ≥ `MODE_COUNT` — कुछ नहीं; वरना PID और स्टेट मशीनें रीसेट, लक्ष्य = मौजूदा कोर्स और ऊँचाई, घेरे का केंद्र = मौजूदा बिंदु (GPS हो तो), RTH — वापसी की ऊँचाई |
| `getMode()`, `getModeName()` | नाम: संपर्क टूटने पर `FAILSAFE_GLIDE` / `FAILSAFE_RTH`, वरना मोड |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | मोड के ऊपर failsafe |
| `isAutoThrottle()`, `getThrottleCorrection()` | मोड का थ्रॉटल (%, लॉग और डैशबोर्ड के लिए) |
| `getLaunchState()`, `getSoaringState()` | LAUNCH और SOARING की स्टेट मशीनें |

### आउटपुट और निदान

| मेथड | विवरण |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | कमांड − स्टिक, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | लक्ष्य |
| `const NavStatus& getNavStatus()` | GPS, होम, स्थिति, होम की दूरी/बेयरिंग, कोर्स और लक्ष्य कोर्स, नेविगेशन के लिए गति, जियोफ़ेंस, स्टॉल |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | बैरोमीटर से ऊँचाई (चालू करने के बिंदु से m में) |
| `getInputs()`, `getAutoTrim()` | टेलीमेट्री के लिए |
| `getImuSensor()` … `getAirspeedSensor()` | सेंसर (`nullptr` हो सकते हैं) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | PID (डैशबोर्ड, MAVLink के पैरामीटर) |

### भीतरी कार्यप्रणाली

- `stabilize()` — कोण का PID, जाइरोस्कोप से D, `Knob::STAB_GAIN` से गुणा; इंटीग्रेटर केवल
  ARM पर और त्रुटि < `STAB_INTEGRATOR_ZONE_DEG` होने पर जमा होता है।
  `stabilizeOrManual()` — IMU के बिना कंट्रोल सरफ़ेस पायलट के पास;
  `stabilizeOrNeutral()` — IMU के बिना न्यूट्रल (स्वचालित मोड)।
- `imuReady()` = IMU है, उपलब्ध है और उड़ान-पूर्व जाँच की कोई समस्या नहीं।
- नेविगेशन के लिए गति: पिटो ट्यूब → GPS → `NAV_ASSUMED_SPEED_MS`।
- `looksLanded()` — बैरोमीटर के अनुसार ज़मीन के पास, लगभग शून्य ऊर्ध्वाधर गति,
  पिटो ट्यूब/GPS की सीमा से धीमी: तभी ट्रिमर फ़्लैश में लिखा जाता है।
- Failsafe: GPS और होम होने पर — मोटर के साथ RTH, वरना ग्लाइड; शुरू हो चुका RTH GPS के
  थोड़े समय के लिए खोने पर नहीं छोड़ा जाता।

---

## `Geo`, `Guidance`, `GeoPoint`

**फ़ाइल:** `autopilot/Navigation.h`

मीटर में स्थानीय “उत्तर/पूर्व” समतल (समदूरस्थ प्रक्षेपण — किलोमीटर के लिए त्रुटि
प्रतिशत का एक छोटा अंश है)।

| फ़ंक्शन | विवरण |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | कोणों का सामान्यीकरण |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | विस्थापन, दूरी, बेयरिंग 0..360 |
| `Geo::moved(a, north, east)` | विस्थापन के साथ बिंदु |
| `Geo::fromGps(GpsData)` | GPS से `GeoPoint` |
| `Guidance::rollForCourse(target, course, bankLimit)` | कोर्स की त्रुटि पर रोल (`NAV_COURSE_GAIN`), सीमित |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | घेरे की ओर ले जाने वाले वेक्टर-फ़ील्ड का कोर्स (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | घेरे का पूर्वानुमानी रोल: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**फ़ाइल:** `autopilot/AltitudeSpeedController.h` — TECS-lite।

| मेथड | विवरण |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | वांछित ऊर्ध्वाधर गति = `NAV_ALT_GAIN`·त्रुटि (≤ `NAV_MAX_CLIMB/SINK`); पिच = पूर्वानुमान asin(Vz/V) + Vz की त्रुटि पर PI, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` की सीमा में |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | पिटो ट्यूब के साथ — `cruisePct` के आसपास हवाई गति पर PI; उसके बिना — `cruisePct`; + ज़रूरी चढ़ाई के लिए `THROTTLE_PER_CLIMB_PCT` |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**फ़ाइल:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (थ्रॉटल उठाया) `→ THROWN` (`LAUNCH_ACCEL_TIME_MS` से ज़्यादा देर तक
अधिभार > `LAUNCH_ACCEL_G`) `→ CLIMB` (`LAUNCH_MOTOR_DELAY_MS` बाद: मोटर, पिच
`LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` या `LAUNCH_ALTITUDE_M`)। फेंकने से
पहले स्टिक हिलाना — रद्द। मेथड: `update(...)`, `reset()`, `getState()`, `motorOn()`,
`pitchTargetDeg()`, `stateName()`।

## `SoaringController`

**फ़ाइल:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (वेरियोमीटर > `SOAR_THERMAL_CLIMB_MS`, `SOAR_THERMAL_CONFIRM_MS`
से ज़्यादा देर तक / `SOAR_EXIT_WINDOW_MS` में औसत < `SOAR_EXIT_CLIMB_MS`),
`→ MOTOR_CLIMB` (`SOAR_MIN_ALTITUDE_M` से नीचे, `SOAR_MAX_ALTITUDE_M` तक),
`→ RETURN` (`SOAR_MAX_DISTANCE_M` से आगे, उसके 70 % तक)। मेथड:
`update(climb, alt, distHome, dt, now)`, `reset(now)`, `getState()`, `motorOn()`,
`getAverageClimb()`, `stateName()`।

## `AutoTrim`

**फ़ाइल:** `autopilot/AutoTrim.h` · भंडारण — `Preferences` (NVS / STM32 का फ़्लैश), नेमस्पेस `"autotrim"`

| मेथड | विवरण |
|---|---|
| `void load()` | NVS से ट्रिमर (न हो तो — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += कमांड · `AUTOTRIM_RATE` · dt, ±`AUTOTRIM_MAX_US` तक |
| `bool saveIfChanged()` | बदला हो तो लिखना (ज़मीन पर DISARM के बाद `Autopilot` बुलाता है) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**फ़ाइल:** `autopilot/PidController.h` · **निर्भर है:** `Config` (नाममात्र `dt`)

| मेथड | विवरण |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | गुणांक |
| `setLimits(minOut, maxOut)` | आउटपुट की सीमा (डिफ़ॉल्ट ±500) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | आउटपुट, `[min, max]` तक सीमित |
| `void reset()` | इंटीग्रेटर शून्य करना, `dt` “अभी” से गिना जाता है |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (सेंसर से गति!)
out = constrain(P + I + D, min, max)
```

D मापी जा रही राशि की गति (जाइरोस्कोप °/s) पर लिया जाता है, त्रुटि के अवकलज पर नहीं:
अवकलन का शोर नहीं और लक्ष्य बदलने पर झटका नहीं। `dt` `micros()` से आता है;
`reset()` के बाद की पहली कॉल या 0.1 s से लंबे विराम के बाद नाममात्र
`LOOP_PERIOD_MS` इस्तेमाल होता है।
