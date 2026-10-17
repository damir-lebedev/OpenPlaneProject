# CONTROL और COORDINATION — मिक्सर, थ्रॉटल, ARM, आउटपुट, संयोजक

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/control.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है। यह अनुवाद AI ने किया है और मूल भाषा के जानकारों ने इसकी जाँच नहीं की है। त्रुटियाँ मिलें तो [Damir Lebedev](https://github.com/damir-lebedev) को लिखें या [इश्यू ट्रैकर](https://github.com/damir-lebedev/OpenPlaneProject/issues) में बताएँ।

[← संदर्भ](README.md)

CONTROL परत डेटा के ऊपर का तर्क है, UART, PWM और Wi-Fi के बिना।
`FlightController` (COORDINATION) अकेली क्लास है जो सभी निचली परतों को एक चक्र में
जोड़ती है।

---

## `ControlCommand`

**फ़ाइल:** `control/ControlCommand.h` · **प्रकार:** struct

कंट्रोल सरफ़ेस की कमांड **भौतिक चिह्नों** में, विक्षेपण µs में (±500 = पूरी यात्रा)।
स्टिक, ऑटोपायलट और मिक्सर की साझा भाषा।

| फ़ील्ड | “+” का अर्थ |
|---|---|
| `int16_t roll` | दाईं ओर रोल (दायाँ एलेरॉन ऊपर, बायाँ नीचे) |
| `int16_t pitch` | नाक ऊपर (एलिवेटर ऊपर) |
| `int16_t yaw` | नाक दाईं ओर (रडर और पहिया दाईं ओर) |
| `int16_t flaps` | फ़्लैप नीचे (दोनों एलेरॉन नीचे); “−” — एयर ब्रेक (दोनों ऊपर) |

सभी फ़ील्ड डिफ़ॉल्ट रूप से 0।

---

## `FlightOutputState`

**फ़ाइल:** `control/FlightOutputState.h` · **प्रकार:** struct

आउटपुट के वांछित पल्स, PWM µs। डिफ़ॉल्ट — कंट्रोल सरफ़ेस न्यूट्रल और थ्रॉटल `PWM_MIN`।

`throttle` किसी भी दिशा में थ्रॉटल का परिमाण है (1000..2000 = 0..100%); इसे `motorReverse` के साथ ESC पल्स में `FlightOutputs` बदलता है (`EscSignal` के ज़रिए)।

| फ़ील्ड | डिफ़ॉल्ट |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` — मोटर आगे घूमती है |
| `bool escPulsesOff` | `false`; `true` — ESC को कोई पल्स नहीं (मोटर का संकेत, केवल ARM के बिना) |
| `aux1` | `PAYLOAD_CLOSED_US` — पेलोड गिराने वाला बंद है |
| `aux2` | `PWM_CENTER` — कैमरा |

---

## `FlapsController`

**फ़ाइल:** `control/FlapsController.h` · **निर्भर है:** `Config`

फ़्लैप का सहज निकलना/समेटना: स्थिति लक्ष्य की ओर बढ़ती है (कोई भी मान — स्विच से
फ़्लैप, नॉब से फ़्लैप, ऊपर की ओर एयर ब्रेक) और `FLAPS_TRANSITION_MS` में पूरी यात्रा
`FLAPS_DEPLOYED_US` से तेज़ नहीं चलती। समय पैरामीटर के रूप में दिया जाता है।

| मेथड | विवरण |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | लक्ष्य की ओर एक क़दम; मौजूदा स्थिति लौटाता है, µs (+ नीचे, − ऊपर) |
| `int16_t getPosition() const` | मौजूदा स्थिति |

अपरिवर्तनीय नियम:

- **पहली कॉल** स्थिति को सीधे लक्ष्य पर रखती है — चालू करते समय फ़्लैप मेज़ पर
  “बाहर नहीं सरकते”।
- समय का क़दम `MAX_STEP_MS = 20` तक सीमित है: लंबे विराम (failsafe, कैलिब्रेशन) के बाद
  फ़्लैप एक ही चक्र में लक्ष्य पर नहीं कूदते।

---

## `ControlMixer`

**फ़ाइल:** `control/ControlMixer.h` · **निर्भर है:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

वायुगतिकीय तर्क दो चरणों में। `FlapsController` का मालिक है।

| मेथड | विवरण |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = दाईं ओर); CH2 → `pitch` **उलटे चिह्न के साथ** (2000 = अपने से दूर = नाक नीचे); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | फ़्लैप का लक्ष्य `FlightController` चुनता है (एयर ब्रेक → फ़्लैप का स्विच → नॉब), यहाँ — सहज गति |
| `FlightOutputState mix(const ControlCommand& c) const` | कमांड → PWM। रोल/पिच/यॉ यात्रा (`*_MAX_US`) से सीमित; एलेरॉन: बायाँ = `flaps + roll`, दायाँ = `flaps − roll` (नीचे = “+”); PWM = `1500 ± विक्षेपण`, चिह्न `Config::*_REVERSED`, फ़्लैप का चिह्न — `FLAPS_REVERSED` से, 1000..2000 तक सीमित। `throttle` नहीं भरा जाता |
| `int16_t getFlaps() const` | फ़्लैप की मौजूदा स्थिति, µs |

फ़्लैपरॉन: निकलते समय दोनों एलेरॉन `FLAPS_DEPLOYED_US` नीचे आते हैं (नया
“न्यूट्रल”), रोल उसके ऊपर काम करता है। पूरे रोल पर नीचे जाने वाला एलेरॉन ऊपर जाने
वाले से पहले यात्रा के छोर पर पहुँच जाता है — यह एलेरॉन के अंतर (डिफ़रेंशियल) की तरह
काम करता है।

---

## `ThrottleManager`

**फ़ाइल:** `control/ThrottleManager.h` · **निर्भर है:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| मेथड | विवरण |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | CH3 का थ्रॉटल, 1000..2000 तक सीमित; संपर्क टूटने पर — `FAILSAFE_THROTTLE` |

इसे ARM और ऑटोपायलट के बारे में कुछ नहीं पता — उनके सुधार `FlightController` लागू करता है।

---

## `EscSignal`

**फ़ाइल:** `control/EscSignal.h` · **प्रकार:** namespace · **निर्भर है:** `Config`

थ्रॉटल (1000..2000 = 0..100%) और दिशा → ESC पिन पर पल्स।

| फ़ंक्शन | विवरण |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (3D में BLHeli 80A): 0% → `ESC_NEUTRAL_US` (1500), आगे 1500..2000, पीछे 1500..1000। एकदिश ESC: 1000..2000 जैसा है, `reverse` अनदेखा। 1000..2000 से बाहर का थ्रॉटल काट दिया जाता है |
| `uint16_t stopUs()` | “मोटर रुकी” का पल्स (द्विदिश ESC में 1500, साधारण में 1000) |

केवल रूपांतरण, सुरक्षा नहीं: दिशा कब बदलनी है यह `MotorReverse` तय करता है, पीछे की थ्रॉटल सीमा — `FlightController`।

---

## `MotorReverse`

**फ़ाइल:** `control/MotorReverse.h` · **निर्भर है:** `Config`

मोटर की दिशा `Feature::MOTOR_REVERSE` स्विच से (डिफ़ॉल्ट रूप से SwD) — **केवल ARM के बिना**।

| मेथड | विवरण |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | ARM नहीं — दिशा तुरंत स्विच के साथ बदलती है (Serial में `MotorReverse: мотор НАЗАД` / `вперёд` — “मोटर पीछे” / “आगे”)। ARM — स्विच का असर नहीं, एक बार “реверс НЕ переключён” (“रिवर्स नहीं बदला”) छपता है, नई दिशा DISARM का इंतज़ार करती है। `ESC_BIDIRECTIONAL = false` होने पर रिवर्स कभी चालू नहीं होता |
| `bool isReversed() const` | मोटर पीछे घूमती है |
| `bool isPending() const` | स्विच पहले ही दूसरी स्थिति में है, दिशा DISARM का इंतज़ार कर रही है |
| `const char* armBlockReason(uint32_t nowMs) const` | ARM अस्वीकार करने का कारण: आख़िरी दिशा-बदलाव को `REVERSE_SETTLE_MS` से कम समय हुआ (मोटर अभी घूम रही हो सकती है); वरना `nullptr` |

---

## `ArmingManager`

**फ़ाइल:** `control/ArmingManager.h` · **निर्भर है:** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

ARM अलग स्विच SwA (CH5) से। स्टेट मशीन
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager) में है।

| मेथड | विवरण |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | ऑटोपायलट के बिना केवल थ्रॉटल जाँचा जाता है |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | failsafe में — कुछ नहीं (failsafe फ़्रेम में स्विच पायलट को नहीं दर्शाता)। स्विच OFF → DISARM, `switchSeenOff = true`। OFF→ON का बदलाव → जाँचें → ARM या इनकार `blockReason` — बाहरी अस्वीकृति कारण (`FlightController` से: `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | ARM है |
| `const char* getLastRefusalReason() const` | आख़िरी इनकार का कारण या `nullptr`; स्विच बंद करने पर साफ़ हो जाता है |

`checkFailureReason(rc)` — ARM की जाँचें:

| शर्त | इनकार का कारण |
|---|---|
| थ्रॉटल ≥ `THROTTLE_LOW_US` | “थ्रॉटल न्यूनतम पर नहीं” |
| MANUAL को छोड़कर कोई भी मोड, IMU है पर जवाब नहीं देती | “IMU जवाब नहीं देती…” |
| MANUAL को छोड़कर कोई भी मोड, IMU की उड़ान-पूर्व जाँच में समस्या | `ImuSensor::getPreflightProblem()` का पाठ |
| ऊँचाई वाला मोड (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), बैरोमीटर है पर जवाब नहीं देता | “बैरोमीटर जवाब नहीं देता…” |
| बाकी सब पास, पर `blockReason` दिया गया | `blockReason` (जैसे “मोटर की दिशा अभी-अभी बदली…”) |

जो सेंसर बिल्ड में नहीं है (`nullptr`), वह ARM को नहीं रोकता; MANUAL में विमान
बिना किसी सेंसर के भी ARM हो जाता है। GPS फ़िक्स को जाँचों में जानबूझकर शामिल नहीं
किया गया: GPS के बिना नेविगेशन के मोड सुरक्षित व्यवहार करते हैं (जगह पर घेरा), और होम
तब दर्ज होगा जब GPS उपग्रह पकड़ लेगा।

अपरिवर्तनीय नियम: स्विच ON रखकर बोर्ड चालू करने पर ARM नहीं होता; हर OFF→ON बदलाव
पर एक ही कोशिश; संपर्क टूटना ARM को नहीं हटाता।

---

## `FlightOutputs`

**फ़ाइल:** `control/FlightOutputs.h` · **निर्भर है:** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

इकलौती क्लास जो PWM आउटपुट का सेट और क्रम जानती है। सभी आउटपुट एक तालिका में
वर्णित हैं; `begin()`, `write()`, स्थिति और स्व-जाँच उस पर लूप में चलते हैं।

### `FlightOutputs::OutputInfo`

| फ़ील्ड | विवरण |
|---|---|
| `const char* key` | JSON/लॉग में नाम (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | इंसान के लिए नाम |
| `int16_t pin` | पिन का नंबर; `-1` — निकाला नहीं गया। `int16_t`, क्योंकि STM32 पर एनालॉग पिनों के नंबर `0xC0 + N` हैं |
| `bool required` | इसके बिना विमान नहीं उड़ता (रडर वैकल्पिक है) |
| `uint16_t FlightOutputState::* field` | स्थिति के फ़ील्ड का पॉइंटर |

| मेथड | विवरण |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | तालिका की पंक्ति; क्रम = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | हर आउटपुट का `attach(PWM_MIN, PWM_MAX)`, स्थिति और ESC का प्रकार छापना; `true`, अगर सभी **अनिवार्य** आउटपुट को चैनल मिला |
| `bool isAttached(uint8_t ch) const` | आउटपुट जुड़ा है (सीमा से बाहर का इंडेक्स → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | आउटपुट पिन पर पल्स: तालिका से फ़ील्ड, ESC के लिए — `EscSignal::pulseUs(throttle, motorReverse)` से, `escPulsesOff` के साथ — 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | हर जुड़े पिन पर नापा हुआ पल्स बनाम अपेक्षित; अंतर ≤ 15 µs हो तो “OK” |
| `void write(const FlightOutputState&)` | सभी आउटपुट लिखना और स्थिति याद रखना; `escPulsesOff` वाला ESC — `IServoOutput::stopPulses()` |
| `void setFailsafe()` | कंट्रोल सतहें न्यूट्रल (`FAILSAFE_*`), थ्रॉटल `FAILSAFE_THROTTLE` (द्विदिश ESC के पिन पर 1500); AUX जैसे थे (संपर्क टूटने से पेलोड नहीं गिरता) |
| `void setBuzzer(bool on)` | बोर्ड का बजर (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | आख़िरी लिखी गई स्थिति |

आउटपुट जोड़ना: तालिका की एक पंक्ति + `FlightOutputState` में एक फ़ील्ड + `ServoChannel`
में एक इंडेक्स (+ `Esp32Board` में पिन और LEDC चैनल)।

---

## `FlightController`

**फ़ाइल:** `control/FlightController.h` · **परत:** COORDINATION ·
**निर्भर है:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

नियंत्रण-चक्र का इकलौता संयोजक: ख़ुद UART नहीं पार्स करता, PWM को नहीं छूता, मिक्सर
नहीं गिनता — बस बाक़ियों को सही क्रम में बुलाता है। विस्तृत आरेख
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-नियंत्रण-का-चक्र-flightcontrollerupdate) में है।

| मेथड | विवरण |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | ऑटोपायलट के बिना — शुद्ध हाथ का नियंत्रण; स्विच के बिना — केवल स्टिक |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | एक चक्र (नीचे देखें) |
| `bool isReceiverFailsafe() const` | संपर्क खो गया |
| `const IBusReceiver& getReceiver() const` | लॉग के लिए (फ़्रेमों के काउंटर, खोने का कारण) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | आउटपुट पर आख़िरी लिखा गया |
| `const RcChannelState& getRcState() const` | चैनल |
| `const FlightOutputs& getOutputs() const` | आउटपुट की तालिका और `attached` |
| `int16_t getFlapsUs() const` | फ़्लैप की स्थिति |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | इस चक्र के स्विच और नॉब |
| `bool isLostModelBeeping() const` | “मैं यहाँ हूँ” बजर चल रहा है |
| `const MotorReverse& getMotorReverse() const` | मोटर की दिशा और प्रतीक्षारत रिवर्स |
| `void toggleGroundCalibration()` | कंसोल `c`: विमान का ग्राउंड कैलिब्रेशन — शुरू करना (केवल ARM के बिना), चल रहा हो तो रद्द |
| `void playSignal(uint8_t count)` | कंसोल `e`: एक संकेत (ARM के बिना — मोटर से भी) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | OLED, लॉग और ब्लैक बॉक्स के लिए |

`update()` का क्रम:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. जीवित संपर्क पर — `switches->update(rc)` (मोड, फ़ंक्शन, नॉब);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. स्टिक `mixer.fromSticks(rc)` (जीवित संपर्क पर) × `Knob::RATES`; फ़्लैप
   `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → सहजता से, संपर्क टूटने पर — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **हमेशा**;
   रिवर्स में `pilotThrottle` की जगह `PWM_MIN` (ऑटोपायलट के लिए मोटर बंद);
6. कैलिब्रेशन: उसके दौरान ARM पलटना उसे रद्द करता है; ARM के बिना और संपर्क के साथ — `CAL_SWITCH` इशारा; `GroundCalibration::update()` का एक कदम; उसके संकेत और ऑटो-टेकऑफ़ के इनकार का संकेत `SignalPlayer` को; बजर: `Beeper::update(BEEPER, armed, failsafe, now)` या संकेत की रोशनी;
7. संपर्क खोया → `applyLinkLoss()` और चक्र से बाहर;
8. `arming.update(rc, false, कारण)` — कैलिब्रेशन चलते या संकेत बजते ARM मना (और उस चक्र में भी जिसमें ARM पलटने ने कैलिब्रेशन रद्द किया), वरना `reverse.armBlockReason(now)`; फिर `reverse.update(MOTOR_REVERSE, armed, now)` — DISARM पर दिशा उसी चक्र में बदलती है;
9. `command = autopilot->getCommand()` (या ऑटोपायलट के बिना स्टिक), फ़्लैप — अपने; संकेत का हिलना एलिवेटर और रडर पर जुड़ता है;
10. `output = mixer.mix(command)`; `output.motorReverse = reverse.isReversed()`;
    आगे — `output.throttle = autopilot->applyThrottle(pilotThrottle)`, सीमा
    `THROTTLE_LIMIT_PCT` × नॉब `Knob::THROTTLE_LIMIT`; पीछे — पायलट का थ्रॉटल, सीमा
    `min(REVERSE_THROTTLE_LIMIT_PCT, आगे की सीमा)`; अनुमत हिस्सा स्टिक की पूरी चाल पर फैलता है;
11. ARM नहीं या `MOTOR_KILL` → `throttle = PWM_MIN` (सबसे अंत में); ARM के बिना मोटर का संकेत `escPulsesOff` लगाता है;
12. AUX1 — पेलोड (`PAYLOAD_DROP`), AUX2 — कैमरा (`Knob::CAMERA_TILT`, `CAMERA_STAB` पिच घटाता है); `AUX1_REVERSED`/`AUX2_REVERSED` पल्स को उलटते हैं।
13. `outputs.write(output)`।

`applyLinkLoss()`: अगर ऑटोपायलट failsafe में है (ARM: RTH या ग्लाइड) — कंट्रोल सरफ़ेस और थ्रॉटल ऑटोपायलट की कमांड के अनुसार (फ़्लैप सहजता से समेटे जाते हैं, `MOTOR_KILL` और रिवर्स मोटर को चुप कराते हैं, AUX जैसे थे वैसे); वरना `outputs.setFailsafe()`, और जब तक संकेत बज रहा है — हिलने और ESC की पल्स रुकने के साथ न्यूट्रल (रिमोट नीचे रख दिया गया — कैलिब्रेशन चल रहा है)।

---

## `Beeper`

**फ़ाइल:** `control/Beeper.h` · **निर्भर है:** `Config`

| मेथड | विवरण |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | बजर की स्थिति: 2 Hz, अगर `Feature::BEEPER` या “मॉडल खो गया” (ARM नहीं, `LOST_MODEL_BEEP_DELAY_MS` से ज़्यादा देर से संपर्क नहीं) |
| `bool isLostModel() const` | “मुझे घास में ढूँढो” मोड |

---

## `GroundCalibration`

**फ़ाइल:** `control/GroundCalibration.h` · **निर्भर है:** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

विमान का ग्राउंड कैलिब्रेशन: चरण 1 — पूँछ ऊपर, धड़ क्षैतिज (क्षितिज, `ImuOrientation::withLevel()`), चरण 2 — पहियों पर (पार्किंग कोण)। `CAL_STILL_MS` की स्थिरता के बाद चरण अपने-आप दर्ज होता है; चरण 2 विमान के हिलाए जाने का इंतज़ार करता है, या `CAL_PARK_WAIT_MS`। सहेजना केवल पूरा एक साथ (`ImuSensor::setGroundCalibration()`), `LANDING_GEAR` के अनुसार पार्किंग कोण की जाँच के बाद। लूप को नहीं रोकता।

`Stage`: `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`।

| मेथड | विवरण |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | इशारा: `CAL_SWITCH` को `CAL_SWITCH_WINDOW_MS` के भीतर `CAL_SWITCH_FLIPS` बार नीचे-ऊपर, थ्रॉटल नीचे; पहली बार देखना और थ्रॉटल नीचे न होना गिनती फिर से शुरू करते हैं |
| `void forgetSwitch()` | ARM या संपर्क नहीं — गिनती फिर से |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | शुरू करना (संकेत 1); IMU नहीं या समर्थन नहीं — त्रुटि (संकेत 3) |
| `void cancel(const char* why)` | बिना संकेत के रद्द करना |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | एक कदम: ARM रद्द करता है; संकेत बजते समय कुछ दर्ज नहीं होता; `CAL_STEP_TIMEOUT_MS` पार — त्रुटि |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | स्थिति और नतीजा |
| `uint8_t takeSignal()` | कितने संकेत बजाने हैं: 1 — शुरू / चरण 1, 2 — पूरा, 3 — त्रुटि (एक बार लिया जाता है) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` — OLED के लिए |

---

## `SignalPlayer`

**फ़ाइल:** `control/SignalPlayer.h` · **निर्भर है:** `Config`

पायलट के लिए संकेत — शृंखला शुरू होने के बाद के समय का फ़ंक्शन। एक संकेत: एलिवेटर और रडर हिलते हैं (`SIGNAL_NOD_US` ऊपर, फिर नीचे, हर बार `SIGNAL_NOD_MS`), ARM के बिना ESC की पल्स `SIGNAL_ESC_PAUSE_MS` के लिए रुकती हैं (फिर ESC तैयार होने की धुन बजाता है), हिलने के दौरान बजर की रोशनी जलती है; अगला संकेत `SIGNAL_PERIOD_MS` बाद।

| मेथड | विवरण |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | एक शृंखला; `motor` — क्या ESC को छुआ जा सकता है (और `SIGNAL_MOTOR`) |
| `void stop()` | शांति |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | शृंखला चल रही है (इस दौरान ARM मना) |
| `bool escPulsesOff(uint32_t nowMs) const` | अभी ESC की पल्स हटानी हैं |
| `int16_t nodUs(uint32_t nowMs) const` | सरफ़ेस का हिलना, µs (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | `PIN_BUZZER` पर बजर / LED |
