# CONTROL और COORDINATION — मिक्सर, थ्रॉटल, ARM, आउटपुट, संयोजक

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/control.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

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

| फ़ील्ड | डिफ़ॉल्ट |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
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
| `FlightOutputState mix(const ControlCommand& c) const` | कमांड → PWM। रोल/पिच/यॉ यात्रा (`*_MAX_US`) से सीमित; एलेरॉन: बायाँ = `flaps + roll`, दायाँ = `flaps − roll` (नीचे = “+”); PWM = `1500 ± विक्षेपण`, चिह्न `Config::*_REVERSED` से, 1000..2000 तक सीमित। `throttle` नहीं भरा जाता |
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

## `ArmingManager`

**फ़ाइल:** `control/ArmingManager.h` · **निर्भर है:** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

ARM अलग स्विच SwA (CH5) से। स्टेट मशीन
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager) में है।

| मेथड | विवरण |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | ऑटोपायलट के बिना केवल थ्रॉटल जाँचा जाता है |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | failsafe में — कुछ नहीं (failsafe फ़्रेम में स्विच पायलट को नहीं दर्शाता)। स्विच OFF → DISARM, `switchSeenOff = true`। OFF→ON का बदलाव → जाँचें → ARM या इनकार |
| `bool isArmed() const` | ARM है |
| `const char* getLastRefusalReason() const` | आख़िरी इनकार का कारण या `nullptr`; स्विच बंद करने पर साफ़ हो जाता है |

`checkFailureReason(rc)` — ARM की जाँचें:

| शर्त | इनकार का कारण |
|---|---|
| थ्रॉटल ≥ `THROTTLE_LOW_US` | “थ्रॉटल न्यूनतम पर नहीं” |
| MANUAL को छोड़कर कोई भी मोड, IMU है पर जवाब नहीं देती | “IMU जवाब नहीं देती…” |
| MANUAL को छोड़कर कोई भी मोड, IMU की उड़ान-पूर्व जाँच में समस्या | `ImuSensor::getPreflightProblem()` का पाठ |
| ऊँचाई वाला मोड (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), बैरोमीटर है पर जवाब नहीं देता | “बैरोमीटर जवाब नहीं देता…” |

जो सेंसर बिल्ड में नहीं है (`nullptr`), वह ARM को नहीं रोकता; MANUAL में विमान
बिना किसी सेंसर के भी ARM हो जाता है। GPS फ़िक्स को जाँचों में जानबूझकर शामिल नहीं
किया गया: GPS के बिना नेविगेशन के मोड सुरक्षित व्यवहार करते हैं (जगह पर घेरा), और होम
तब दर्ज होगा जब GPS उपग्रह पकड़ लेगा।

अपरिवर्तनीय नियम: स्विच ON रखकर बोर्ड चालू करने पर ARM नहीं होता; हर OFF→ON बदलाव
पर एक ही कोशिश; संपर्क टूटना ARM को नहीं हटाता।

---

## `FlightOutputs`

**फ़ाइल:** `control/FlightOutputs.h` · **निर्भर है:** `IBoard`, `FlightOutputState`, `Config`

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
| `bool begin()` | हर आउटपुट का `attach(PWM_MIN, PWM_MAX)`, स्थिति छापना; अगर सभी **ज़रूरी** आउटपुट को चैनल मिला तो `true` |
| `bool isAttached(uint8_t ch) const` | आउटपुट जुड़ा है (सीमा से बाहर का इंडेक्स → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | तालिका के अनुसार स्थिति से आउटपुट का मान |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | हर जुड़े पिन पर नापा हुआ पल्स बनाम अपेक्षित; अंतर ≤ 15 µs हो तो “OK” |
| `void write(const FlightOutputState&)` | सभी आउटपुट लिखना और स्थिति याद रखना |
| `void setFailsafe()` | कंट्रोल सरफ़ेस न्यूट्रल (`FAILSAFE_*`), थ्रॉटल `FAILSAFE_THROTTLE`; AUX जैसे थे वैसे (संपर्क टूटने पर पेलोड नहीं गिरता) |
| `void setBuzzer(bool on)` | बोर्ड का बजर (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | आख़िरी लिखी गई स्थिति |

आउटपुट जोड़ना: तालिका की एक पंक्ति + `FlightOutputState` में एक फ़ील्ड + `ServoChannel`
में एक इंडेक्स (+ `Esp32Board` में पिन और LEDC चैनल)।

---

## `FlightController`

**फ़ाइल:** `control/FlightController.h` · **परत:** COORDINATION ·
**निर्भर है:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

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

`update()` का क्रम:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. जीवित संपर्क पर — `switches->update(rc)` (मोड, फ़ंक्शन, नॉब);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. स्टिक `mixer.fromSticks(rc)` (जीवित संपर्क पर) × `Knob::RATES`; फ़्लैप
   `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → सहजता से, संपर्क टूटने पर — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **हमेशा**;
6. बजर: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. संपर्क खोया → `applyLinkLoss()` और चक्र से बाहर;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (या ऑटोपायलट के बिना स्टिक), फ़्लैप — अपने;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. ARM नहीं या `MOTOR_KILL` → `throttle = PWM_MIN` (सबसे अंत में);
12. AUX1 — पेलोड (`PAYLOAD_DROP`), AUX2 — कैमरा (`Knob::CAMERA_TILT`, `CAMERA_STAB` पिच घटाता है);
13. `outputs.write(output)`।

`applyLinkLoss()`: अगर ऑटोपायलट failsafe में है (ARM: RTH या ग्लाइड) — कंट्रोल सरफ़ेस और
थ्रॉटल ऑटोपायलट की कमांड के अनुसार (फ़्लैप सहजता से समेटे जाते हैं, `MOTOR_KILL` अब भी
मोटर को चुप कराता है, AUX जैसे थे वैसे); वरना `outputs.setFailsafe()`।

---

## `Beeper`

**फ़ाइल:** `control/Beeper.h` · **निर्भर है:** `Config`

| मेथड | विवरण |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | बजर की स्थिति: 2 Hz, अगर `Feature::BEEPER` या “मॉडल खो गया” (ARM नहीं, `LOST_MODEL_BEEP_DELAY_MS` से ज़्यादा देर से संपर्क नहीं) |
| `bool isLostModel() const` | “मुझे घास में ढूँढो” मोड |
