# TELEMETRY — लॉग, कंसोल, वेब डैशबोर्ड, OLED, ब्लैक बॉक्स

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/telemetry.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

टेलीमेट्री उड़ान के तर्क से पूरी तरह अलग है: वह केवल `FlightController`, `Autopilot`,
सेंसरों और `LoopStats` के const गेटर पढ़ती है।
“वापस” जाने का एकमात्र रास्ता डैशबोर्ड के आदेश हैं, जो `WebDebugServer` के
मेलबॉक्स से गुज़रते हैं और उड़ान का लूप उन्हें लागू करता है।

---

## `LoopStats`

**फ़ाइल:** `telemetry/LoopStats.h` · **प्रकार:** struct

उड़ान के लूप की आवृत्ति और अवधि।

| सदस्य | विवरण |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | सेकंड में एक बार प्रकाशित होते हैं; दूसरे टास्कों से पढ़े जाते हैं (32-बिट मान — “फटी हुई” रीडिंग नहीं) |
| `void record(uint32_t durationUs)` | हर टिक पर `loop()` से बुलाएँ |
| `uint32_t takePeakUs()` | पिछली कॉल के बाद का सबसे बुरा टिक (हर 10 s पर SYS की पंक्ति के लिए); उसी टास्क से बुलाएँ जिससे `record()` |

`maxUs` केवल पिछले एक सेकंड का सबसे बुरा मान है; कभी-कभार की अटकन
`takePeakUs()` से दिखती है।

---

## `LogSettings`

**फ़ाइल:** `telemetry/LogSettings.h` · **निर्भर है:** `Preferences` (NVS, नेमस्पेस `debuglog`)

### `LogChannel` (enum class)

| चैनल | उपसर्ग | क्या छापता है | डिफ़ॉल्ट |
|---|---|---|---|
| `Status` | `STAT` | संपर्क, ARM, मोड, फ़्लैप, सेंसर | बदलने पर |
| `Rc` | `RC` | रिमोट के चैनल | बंद |
| `Outputs` | `OUT` | कंट्रोल सतहों और ESC के आउटपुट | बंद |
| `Attitude` | `ATT` | रोल, पिच, हेडिंग | बंद |
| `Autopilot` | `AP` | लक्ष्य और सुधार | बंद |
| `Altitude` | `ALT` | ऊँचाई, ऊर्ध्वाधर गति | बंद |
| `Heading` | `MAG` | कंपास की हेडिंग | बंद |
| `Gps` | `GPS` | उपग्रह, निर्देशांक | बंद |
| `Imu` | `IMU` | जायरोस्कोप और एक्सेलेरोमीटर | बंद |
| `Nav` | `NAV` | होम, हेडिंग, गति, पिटो नली, चालू की गई सुविधाएँ | बंद |
| `System` | `SYS` | लूप की आवृत्ति, मेमोरी (हर 10 s), केवल बंद/चालू | चालू |
| `Count` | — | चैनलों की संख्या | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`।

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`।

| मेथड | विवरण |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | चैनलों की तालिका की एक पंक्ति |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (चक्र में) |
| `LogSettings()`, `void setDefaults()` | डिफ़ॉल्ट मोड, 1 s की अवधि |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | चैनल का मोड |
| `void setMode(uint8_t, LogMode)` | `periodicOnly` चैनलों के लिए `OnChange` `Periodic` बन जाता है |
| `void cycleMode(uint8_t)` | बंद → बदलने पर → लगातार → बंद (SYS: बंद ↔ चालू) |
| `void setAll(LogMode)` | सभी चैनलों के लिए; “सब कुछ बदलने पर” वाला आदेश SYS को नहीं छूता |
| `uint16_t periodMs() const`, `void cyclePeriod()` | “लगातार” मोड की अवधि |
| `static const char* modeName(LogMode, bool periodicOnly)` | “बंद” / “बदलने पर” / “लगातार” (या “चालू”) |
| `void load()` | NVS से; `VERSION` या लंबाई न मिले तो डिफ़ॉल्ट मान रहते हैं; अज्ञात मोड कोड → चैनल का डिफ़ॉल्ट |
| `void save() const` | NVS में (मोड, अवधि, संस्करण) |

`VERSION` चैनलों की सूची के साथ बदलता है — पुरानी सेटिंग रीसेट हो जाती हैं
(`VERSION = 2`: NAV चैनल जोड़ा गया)। मेनू में चैनलों की कुंजियाँ: `1`..`9`, NAV —
`n`, SYS — `s`।

---

## `DebugLogger`

**फ़ाइल:** `telemetry/DebugLogger.h` · **निर्भर है:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

स्थिति को चैनलों के हिसाब से सीरियल मॉनिटर में छापना: हर चैनल की अपनी पंक्ति, अपना
मोड और अपनी सहनसीमाएँ हैं।

| मेथड | विवरण |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | हर `DEBUG_INTERVAL_MS` में एक बार चैनलों पर घूमता है (विराम में और मेनू खुला होने पर चुप रहता है) |
| `LogSettings& getSettings()`, `void saveSettings() const` | कंसोल के मेनू के लिए |
| `void suspend(bool)` | मेनू खुला है — चुप रहो; हटने पर — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | स्पेस से विराम; हटने पर — `refresh()` |
| `void refresh()` | अगला टिक सभी चालू चैनल छापेगा |

चैनल का तर्क (`updateChannel`):

- `Off` — न छापना;
- `Periodic` — हर `periodMs()` पर एक बार (SYS — हर 10 s पर), मान “जैसे हैं वैसे”;
- `OnChange` — पंक्ति **सहनसीमाओं** के साथ बनती है (भीतर का `Shown` पुराना मान तब तक रखता है
  जब तक नया सहनसीमा से आगे न निकल जाए: RC/PWM 3 µs, कोण
  0.5°, हेडिंग 1°, सुधार 2, ऊँचाई 0.3 m, त्वरण 0.03 g, निर्देशांक
  1e−5°) और केवल तब छपती है जब पिछली छपी पंक्ति से अलग हो।

भीतर के प्रकार: `LineBuffer : Print` (छापने से पहले तुलना के लिए 200 बाइट तक की पंक्ति),
`Shown` (हिस्टेरेसिस वाला मान)।

पंक्तियों के प्रारूप:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  लक्ष्य 0.0 m
MAG  हेडिंग 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (10 s में सबसे बुरा) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` में `LOST(कोई फ़्रेम नहीं)` और `LOST(रिमोट का failsafe)` का फ़र्क़ दिखता है; `IMU=` —
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`।

---

## `DebugConsole`

**फ़ाइल:** `telemetry/DebugConsole.h` · **निर्भर है:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (बसों की जाँच)

सीरियल मॉनिटर में पाठ वाला मेनू। स्क्रीनों की स्टेट मशीन `Screen::{None, Main, Log}`।

| मेथड | विवरण |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | बोर्ड के साथ — आदेश `b` और मेनू का बिंदु 7 |
| `static const char* guessI2cDevice(uint8_t address)` | पते से चिप: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | एक पंक्ति का संकेत |
| `void update()` | `Serial` के सभी बाइट संसाधित करना; अगर लॉग की सेटिंग बदली हैं, मेनू बंद है और विमान **armed नहीं** है — तो NVS में सहेजना |

हॉट कुंजियाँ (मेनू के बाहर): `h`/`?` — मुख्य मेनू; `l` — लॉग का मेनू; स्पेस —
लॉग का विराम; `s` — सेंसरों की स्थिति; `i` — जायरोस्कोप का कैलिब्रेशन; `o` —
IMU की माउंटिंग का कैलिब्रेशन; `m` — कंपास का कैलिब्रेशन; `p` — आउटपुट की स्व-जाँच;
`b` — I2C बसों की जाँच (0x08..0x7F — 0x7F तक, क्योंकि QMC6309 0x7C पर है) चिपों के
नामों के साथ; बाक़ी — संकेत। `\r`/`\n` अनदेखे किए जाते हैं।

लॉग का मेनू: `1`..`9` — चैनल 0..8 का मोड चक्र में बदलना, `n` — NAV, `s` — SYS, `p` — अवधि, `a` —
सब कुछ “बदलने पर”, `x` — सब कुछ बंद, `d` — डिफ़ॉल्ट, `0`/`q` — पीछे, `l`/`h` —
बंद करना।

रुकावट डालने वाली क्रियाएँ (`i`, `o`, `m`, `p`) **ARM होने पर वर्जित** हैं। मेनू के
खुले रहने तक लॉग रोका रहता है (`DebugLogger::suspend`)। मेनू के बिंदुओं की चौड़ाई
बाइट में नहीं, UTF-8 वर्णों में गिनी जाती है (सिरिलिक 2 बाइट की होती है)।

---

## `WebDashboardPage`

**फ़ाइल:** `telemetry/WebDashboardPage.h` · **प्रकार:** namespace

`static const char HTML[] PROGMEM` — पूरा पन्ना (HTML + CSS + JS) एक ही
लिटरल में। जो कुछ गतिशील है वह ब्राउज़र `/api/status` के JSON से बनाता है (हर
200 ms पर पूछता है): चैनलों, आउटपुट और सेंसरों की पंक्तियाँ JSON की कुंजियों से बनती हैं, इसलिए नया
आउटपुट पन्ने को बदले बिना आ जाता है। जिस PID फ़ील्ड को उपयोगकर्ता ने बदलना शुरू कर दिया हो,
उसे पूछताछ अब अधिलेखित नहीं करती।

---

## `WebDebugServer`

**फ़ाइल:** `telemetry/WebDebugServer.h` · **निर्भर है:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| मेथड | विवरण |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Wi-Fi AP (`persistent(false)` — फ़्लैश में कुछ नहीं लिखा जाता), रूट, कोर 0 पर टास्क `web`। एक्सेस पॉइंट न उठे तो `false` |
| `void applyPendingCommands()` | उड़ान के लूप से बुलाएँ: स्पिनलॉक के नीचे आदेश उठाना और ऑटोपायलट पर लागू करना |

रूट:

| रूट | उत्तर |
|---|---|
| `GET /` | डैशबोर्ड का पन्ना |
| `GET /api/status` | स्थिति का JSON (`buildStatusJson()`), प्रारूप [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) में है |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; बिना बॉडी के → 400 `no data`; ऑटोपायलट नहीं → 503; ग़लत मोड → 400 `invalid mode` |
| `POST /api/setpid` | `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch` में से कोई भी; छोड़े गए जैसे हैं वैसे रहते हैं |
| बाक़ी | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — `portMUX` के नीचे मेलबॉक्स।
`extractJsonNumber(body, key, fallback)` — ArduinoJson के बिना सपाट
JSON का न्यूनतम पार्सर: `"key"`, ख़ाली जगहें, `:`, ख़ाली जगहें, JSON की किसी भी
लिखावट में संख्या (चिह्न, भिन्न, घात `1e-7`); कुंजी या संख्या न हो तो —
`fallback`।

JSON में `attached`/`available` फ़ील्ड **हमेशा** होते हैं; सेंसर का डेटा केवल
`available: true` होने पर।

---

## `OledDisplay`

**फ़ाइल:** `telemetry/OledDisplay.h` · **निर्भर है:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

दूसरी I2C बस पर SSD1306 128×64 (I2C 0x3C); अपना टास्क `oled`
(`Rtos::startTask`: ESP32 पर कोर 0, STM32 पर कम प्राथमिकता), हर 200 ms पर।

| मेथड | विवरण |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` या स्क्रीन 0x3C पर जवाब न दे → `false`; नहीं तो U8g2 सेट करना और टास्क शुरू करना |

U8g2 `II2CBus` के ऊपर `byteCallback` से बाइट भेजता है (स्क्रीन `Wire1` के बारे में कुछ नहीं जानती)।
C का कॉलबैक संदर्भ नहीं पाता, इसलिए बस स्थिर चर `busSlot()` में रखी जाती है —
बोर्ड पर स्क्रीन एक ही है।

स्क्रीन:

```
RX ok ARM STAB FL       संपर्क (खोने पर — उलटी पंक्ति) / ARM / मोड / फ़्लैप
R  +1.2 P  -0.4         रोल / पिच, °           (IMU --)
Alt +0.3 Vz +0.1 A14    ऊँचाई / ऊर्ध्वाधर गति / वायु-गति, अगर पिटो नली है (BARO --)
H123 T1000 Y1500        हेडिंग / थ्रॉटल / रडर (H---)
L1500 R1500 E1500       एलेरॉन / एलिवेटर
Loop 500Hz max1100us    आवृत्ति और एक सेकंड में सबसे बुरा टिक
```

मोड के छोटे नाम `AutopilotNames::modeShort()` से आते हैं (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
हवा में संपर्क खोने पर — `GLIDE` या `FSRTH`।

---

## `BlackBox`

**फ़ाइल:** `telemetry/BlackBox.h` · **निर्भर है:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

उड़ान की रिकॉर्डिंग फ़्लैश (ESP32-S3) या SD कार्ड (STM32H743) पर। क्या, कब और कैसे निकालना है — [BLACKBOX.md](../BLACKBOX.md) में।

| मेथड | विवरण |
|---|---|
| `bool begin(bool startTask = true)` | माध्यम को पढ़ता है (`BlackBoxStorage::begin()`), क़तार आवंटित करता है (ESP32 पर PSRAM, STM32 पर `malloc`), मिटी हुई जगह की जाँच करता है (0.3 s तक), टास्क `bbox` शुरू करता है (`Rtos::startTask`)। रिकॉर्ड के लिए जगह नहीं है (पार्टिशन, कार्ड, फ़ाइल) — `false`, ब्लैक बॉक्स बंद |
| `void update(uint32_t workUs)` | `loop()` से हर टिक के बाद: घटनाएँ, शुरू/बंद, क़तार में स्नैपशॉट, लिखने वाले टास्क को जगाना |
| `void writerStep()` | लिखने वाले टास्क का एक क़दम: फ़्लैश में एक-दो पेज या ज़मीन पर एक बार मिटाना |
| `requestManualStart()` / `requestManualStop()` | हाथ से रिकॉर्डिंग (कंसोल `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (END लिखने से पहले क़तार को पूरा लिखता है) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | कंसोल के लिए |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [बॉड]` — `tools/blackbox.py` के लिए (USB CDC पर गति का कोई असर नहीं) |

प्लेटफ़ॉर्म के हिसाब से बदलने वाली बातें: रीबूट का कारण — `readResetCause()`; बैटरी का वोल्टेज और करंट —
ADC (S3 पर `analogReadMilliVolts`, STM32 पर 12-बिट `analogRead`); माध्यम की त्रुटियाँ
(`BlackBoxStorage::writeErrors`/`eraseErrors`) सेकंड में एक बार “माध्यम: लिखने की त्रुटियाँ …”
घटना के रूप में लॉग में जाती हैं और उड़ान में बाधा नहीं डालतीं।

## `BlackBoxStorage`

**फ़ाइल:** `telemetry/BlackBoxStorage.h` · **निर्भर है:** `IFlashRegion`

4 KB के सेक्टरों का रिंग: सिरा और उड़ानों की सूची `begin()` के समय सेक्टरों के हेडरों से मिलती है (पहला चरण हर सेक्टर का हेडर पढ़कर असली वाले याद रखता है, दूसरा केवल उन्हीं को: ख़ाली क्षेत्र एक ही बार पढ़ा जाता है); `openFlight()`/`append()`/`flush()`/`closeFlight()` — पेजों में लिखना (हर रिकॉर्ड पर CRC-8); `eraseStep(target, protect, allowErase)` — सिरे के आगे जाँचने/मिटाने का एक क़दम: कचरा — हमेशा, उड़ानें — पूरी की पूरी और केवल तब जब `target` से कम जगह ख़ाली हो; `protect` को कभी नहीं छुआ जाता।

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` टास्कों/कोरों के बीच रिकॉर्डों की बाइट-क़तार है, जो `Rtos::CriticalSection` के नीचे चलती है; भर जाने पर सबसे पुराने फेंक देती है। `BlackBoxFormat` — सेक्टर का हेडर, रिकॉर्डों के प्रकार और संरचनाएँ, योजनाओं की स्ट्रिंग (आकार `static_assert` से जाँचा जाता है), CRC-8 और CRC-32।

---

## `Mavlink` (कोडेक)

**फ़ाइल:** `telemetry/MavlinkCodec.h` · **प्रकार:** namespace · **निर्भर है:** किसी पर नहीं (पोर्टेबल)

बिना जनरेट की गई लाइब्रेरी के MAVLink 2: फ़ील्डों की MAVLink के क्रम में पैकिंग
(pymavlink से मिलाई गई), CRC-16/MCRF4XX + `CRC_EXTRA`, अंत के शून्यों की कटाई।

| इकाई | विवरण |
|---|---|
| `Msg::*` | पहचानकर्ता: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | संदेश का `CRC_EXTRA`, −1 — अज्ञात |
| `crcAccumulate`, `crcCalculate` | X.25 (जैसे mavlink का `crc_accumulate()`) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — फ़ील्ड क्रम से |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — क्रमिक `seq` के साथ v2 फ़्रेम |
| `Message` | प्राप्त संदेश: `msgid`, `sysid`, `compid`, पेलोड (शून्यों से पूरा किया हुआ), ऑफ़सेट से फ़ील्ड पढ़ना |
| `Parser` | `bool feed(byte)` → `message()`; v1 और v2, v2 का हस्ताक्षर छोड़ दिया जाता है; `goodCount()`, `badCrcCount()`; अज्ञात `CRC_EXTRA` वाले संदेश चुपचाप छोड़ दिए जाते हैं |

## `MavlinkModes`

**फ़ाइल:** `telemetry/MavlinkTelemetry.h` · **प्रकार:** namespace

| फ़ंक्शन | विवरण |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | ArduPlane का मोड-नंबर: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | उलटा, ज़मीन से आए आदेशों के लिए; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | HEARTBEAT में `AUTO_ENABLED` का झंडा |

## `MavlinkTelemetry`

**फ़ाइल:** `telemetry/MavlinkTelemetry.h` · **निर्भर है:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

QGroundControl / Mission Planner के लिए रेडियो मॉडेम पर टेलीमेट्री (वाहन
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA` है)। STM32 पर इस्तेमाल होती है
(UART4), जिसमें Wi-Fi नहीं है।

| मेथड | विवरण |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | पोर्ट खोलना, संदेश “OpenPlane online” |
| `void update()` | उड़ान के लूप से: आने वाले का विश्लेषण (हर टिक में ≤ 128 बाइट), घटनाओं के संदेश, हर टिक में 2 से अधिक फ़्रेम नहीं |
| `void statusText(severity, text)` | GCS की धारा में (4 पंक्तियों की क़तार, 50 अक्षरों तक) |
| `isGcsConnected()` | पिछले 3 s में GCS का HEARTBEAT |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | निदान |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

धाराएँ (Hz): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0.2। फ़्रेम तभी जाता है जब `availableForWrite()` में उसकी जगह हो
— नहीं तो वह अगले टिक की प्रतीक्षा करता है (लूप कभी नहीं रुकता)।

आने वाले: GCS का HEARTBEAT; PARAM_REQUEST_LIST / READ / SET (PID — सीधे
ऑटोपायलट में, मान 0..100, सहेजे नहीं जाते); SET_MODE और COMMAND_LONG
`DO_SET_MODE` (176) — अगली बार टॉगल स्विच दबाने तक का मोड; `COMPONENT_ARM_DISARM`
(400) — **DENIED**; `REQUEST_MESSAGE` (512) — धारा का बारी से बाहर भेजना;
MISSION_REQUEST_LIST — उसी `mission_type` के साथ MISSION_COUNT 0।
बाहरी डिकोडर से धारा की जाँच — `tools/check_mavlink.py` (pymavlink)।
