# DEVELOPER_GUIDE.md — OpenPlaneProject की डेवलपर मार्गदर्शिका

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../DEVELOPER_GUIDE.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

फ़र्मवेयर का तकनीकी नक़्शा: कौन-सी फ़ाइल किस काम की है, डेटा रिसीवर और सेंसरों से सर्वो तक कैसे बहता है, कौन-से चिह्न-नियम पूरी श्रृंखला को जोड़े रखते हैं, वेब API कैसे बना है और प्रोजेक्ट को कैसे बढ़ाएँ। यह उस डेवलपर के लिए है जो C++ लिखता है और इस रिपॉज़िटरी (ब्रांच `main`) में जल्दी रास्ता पाना चाहता है — भाषा या PlatformIO की बुनियादी बातें सीखने के लिए नहीं।

प्रोजेक्ट का परिचय और प्रोटोटाइप की स्थिति [`../README.md`](README.md) में है, क्या कहाँ जोड़ें और कैसे उड़ाएँ — [`PILOT_GUIDE.md`](PILOT_GUIDE.md) में, योजनाएँ — [`ROADMAP.md`](ROADMAP.md) में। यहाँ केवल कोड है। पूरा आर्किटेक्चर (परतें, टास्क, स्टेट मशीनें) [`ARCHITECTURE.md`](ARCHITECTURE.md) में है, हर क्लास का संदर्भ [`reference/`](reference/README.md) में, टेस्ट [`TESTING.md`](TESTING.md) में।

> प्रोजेक्ट सक्रिय विकास में है। ESP32-S3 वाला बेंच बना लिया गया है और सभी सेंसरों के साथ जाँचा जा चुका है, लेकिन **ऑटोपायलट उड़ान में अभी परखा नहीं गया है** — जहाँ यह किसी ख़ास मॉड्यूल से जुड़ा है, वहाँ इसे बताया गया है। अगर आपको संदेह है कि कोड क्या करता है, तो दस्तावेज़ नहीं, सोर्स दोबारा पढ़ें।

---

## विषय-सूची

1. [परतों का आर्किटेक्चर](#परतों-का-आर्किटेक्चर)
2. [FreeRTOS टास्क और नियंत्रण लूप](#freertos-टास्क-और-नियंत्रण-लूप)
3. [फ़ाइलों का संदर्भ](#फ़ाइलों-का-संदर्भ)
4. [चिह्न-नियम: IMU से सर्वो तक](#चिह्न-नियम-imu-से-सर्वो-तक)
5. [RC चैनलों का नक़्शा, ARM और failsafe](#rc-चैनलों-का-नक़्शा-arm-और-failsafe)
6. [सेंसर का डेटा](#सेंसर-का-डेटा)
7. [FlightController::update() का विश्लेषण](#flightcontrollerupdate-का-विश्लेषण)
8. [वेब डैशबोर्ड का HTTP API](#वेब-डैशबोर्ड-का-http-api)
9. [कंसोल और निदान](#कंसोल-और-निदान)
10. [बोर्ड का चुनाव और पिन-व्यवस्था](#बोर्ड-का-चुनाव-और-पिन-व्यवस्था)
11. [नया सेंसर कैसे जोड़ें](#नया-सेंसर-कैसे-जोड़ें)
12. [ऑटोपायलट का नया मोड कैसे जोड़ें](#ऑटोपायलट-का-नया-मोड-कैसे-जोड़ें)
13. [फ़ीडबैक (तैयारी, जुड़ा नहीं है)](#फ़ीडबैक-तैयारी-जुड़ा-नहीं-है)
14. [नया बोर्ड कैसे जोड़ें](#नया-बोर्ड-कैसे-जोड़ें)
15. [बिल्ड, फ़र्मवेयर डालने और मॉनिटर के कमांड](#बिल्ड-फ़र्मवेयर-डालने-और-मॉनिटर-के-कमांड)
16. [ज्ञात सीमाएँ](#ज्ञात-सीमाएँ)
17. [बदलाव कैसे करें](#बदलाव-कैसे-करें)

---

## परतों का आर्किटेक्चर

लगभग सभी क्लास हेडरों में हैं, जो `include/<परत>/` फ़ोल्डरों में रखे हैं। हर हेडर वही ख़ुद शामिल करता है जो वह इस्तेमाल करता है (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ... — रास्ते `include/` से)। `src/main.cpp` असेंबली का एकमात्र बिंदु (composition root) है: वह सभी ऑब्जेक्ट बनाता है, उन्हें जोड़ता है और `setup()`/`loop()` चलाता है। निर्भरताएँ एक दिशा में हैं — निचली परत ऊपरी परत के बारे में कुछ नहीं जानती।

```
include/
├── config/      Config.h (पिन, सभी सेटिंग), Channels.h (चैनलों के नाम),
│                Controls.h (हर स्विच क्या करता है — हर चैनल की एक पंक्ति)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (I2C/SPI के ऊपर रजिस्टर वाला डिवाइस), Rtos
│   ├── esp32/   Esp32Board + Wire/SPI/HardwareSerial/LEDC के रैपर
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (NVS की जगह फ़्लैश में सेटिंग)
├── storage/     KeyValueStore, KvPreferences — NVS के बिना सेटिंग का भंडारण
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  फ़ीडबैक की तैयारी — जुड़ा नहीं है (नीचे का खंड देखें)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (दो बैरोमीटरों वाली ट्यूब)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32 फ़र्मवेयर (S3, C3, 38-pin)
src/stm32/main.cpp  — STM32H743 फ़र्मवेयर (FreeRTOS टास्क, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — ऑब्जेक्ट की असेंबली       │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — हर चक्र में संचालन का क्रम         │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 मोड     │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ नेविगेशन, PID        │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, फ़्लैश      │
└───────────────────────────────────────────────────────────────────────┘
```

वे नियम जो आर्किटेक्चर को साफ़ रखते हैं:

- **HAL** अकेली परत है जिसे किसी ख़ास MCU को जानने की अनुमति है (`Wire`, `SPI`, `HardwareSerial`, `ledc*`)। उसके ऊपर सब कुछ केवल इंटरफ़ेस से काम करता है। दूसरे MCU पर जाने का मतलब है नया `hal/<mcu>/<Mcu>Board.h`; बाक़ी कोड नहीं बदलता (उदाहरण — STM32H743 के लिए `hal/stm32/`)।
- **सेंसर के ड्राइवर बस को नहीं जानते।** उन्हें `IRegisterDevice&` मिलता है — पते वाला I2C या CS वाला SPI `SensorSelection.h` में बनाया जाता है। एक ही `BMP388_Sensor` I2C और SPI दोनों पर चलता है।
- **साझा हिस्सा बेस क्लासों में है।** कैलिब्रेशन, अक्षों का घुमाव, चिह्न, ओरिएंटेशन फ़िल्टर, ऊँचाई और ऊर्ध्वाधर गति, कम्पास के कैलिब्रेशन का भंडारण, बस की त्रुटियों की गिनती — `ImuSensorBase`/`BarometerBase`/`MagnetometerBase` में। चिप के ड्राइवर में केवल रजिस्टर और डेटाशीट के सूत्र होते हैं।
- **RC और Outputs** विमान के बारे में कुछ नहीं जानते: iBUS के बाइट → चैनल, PWM के मान → आउटपुट।
- **Control और Autopilot** डेटा पर लॉजिक हैं, उनमें न UART है, न PWM, न Wi-Fi। जहाँ समय चाहिए (फ़्लैप), वह पैरामीटर के रूप में दिया जाता है।
- **Coordination** (`FlightController`) अकेली क्लास है जो एक साथ कई निचली परतें देखती है और संचालन का क्रम तय करती है।
- **Application** (`main.cpp`) वह अकेली जगह है जहाँ `Esp32Board`, डिवाइस और सेंसर बनते हैं और जहाँ सब कुछ हाथ से, बिना किसी DI फ़्रेमवर्क के, जुड़ता है।

---

## FreeRTOS टास्क और नियंत्रण लूप

| कहाँ | क्या | अवधि |
|---|---|---|
| कोर 1, `loop()` (Arduino का loopTask) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| कोर 0, `web` टास्क | `WebServer::handleClient()` | हर 2 ms |
| कोर 0, `oled` टास्क | दूसरी I2C बस पर SSD1306 की ड्रॉइंग | 200 ms |
| कोर 0 | ESP-IDF का Wi-Fi स्टैक | — |

- लूप की अवधि `vTaskDelayUntil` से बनी रहती है, काम के बाद `delay(2)` से नहीं — आवृत्ति इस पर निर्भर नहीं करती कि चक्र कितना लंबा चला। लंबे अवरोध (कंसोल से कैलिब्रेशन) के बाद गिनती नए सिरे से शुरू होती है, छूटे चक्र एक साथ नहीं पकड़े जाते।
- बेंच पर (ESP32-S3, सभी सेंसर): 500 Hz, हर चक्र में औसतन ~0.7 ms का काम, सबसे बुरा चक्र ~1.4 ms। यह हर 10 s पर `SYS:` पंक्ति के रूप में छपता है।
- I2C लेन-देन का टाइमआउट 5 ms है (Wire का मानक 50 ms है): व्यवधान से अटका लेन-देन लूप को देर तक नहीं रोकता।
- **टास्कों के बीच डेटा का बँटवारा।** वेब और OLED स्थिति (`FlightController`/`Autopilot`/`LoopStats`) को केवल *पढ़ते* हैं — ये अलग-अलग 16/32-बिट फ़ील्ड हैं, इसलिए सबसे बुरी हालत में पास के चक्रों के मान दिखते हैं। डैशबोर्ड की *कमांड* (`setmode`/`setpid`) वेब टास्क से सीधे लागू नहीं होतीं: वे `portMUX` के तहत “मेलबॉक्स” में रखी जाती हैं और उड़ान का लूप उन्हें `WebDebugServer::applyPendingCommands()` में उठाता है।
- 4 KB के ट्रांसमिट बफ़र वाला `Serial` (UART0 → CH343 ब्रिज → “COM” कनेक्टर): डीबग का एक फ़्रेम (~600 अक्षर) भेजे जाने के दौरान लूप को नहीं रोकता।

---

## फ़ाइलों का संदर्भ

### `config/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `Config.h` | सभी पिन (हर बोर्ड के लिए एक ब्लॉक: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) और सेटिंग: iBUS और सिग्नल खोना; कंट्रोल सरफ़ेस की यात्रा; फ़्लैप; सर्वो रिवर्स; IMU और कम्पास की माउंटिंग; ARM; failsafe (RTH या ग्लाइड); पिटो ट्यूब (`PITOT_*`); ऑटोपायलट के मोड और फ़ंक्शन की सारी संख्याएँ; लूप; Wi-Fi; MAVLink; डीबगिंग |
| `Channels.h` | चैनलों के नाम: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | तालिका `BINDINGS`: हर स्विच और नॉब क्या करता है, हर चैनल की एक पंक्ति, `static_assert` की जाँचें |

### `hal/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `IBoard.h` | हार्डवेयर का प्रवेश-बिंदु: `i2c()`, `displayI2c()` (स्क्रीन के लिए दूसरी बस, `nullptr` हो सकती है), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, `nullptr` हो सकता है), `servo(ServoChannel::*)` (AUX1/AUX2 के साथ 7 आउटपुट), `setBuzzer()` |
| `Rtos.h` | FreeRTOS टास्क, ESP32 (कोर 0) और STM32 (प्राथमिकताएँ) पर एक जैसे, ख़ाली हीप |
| `II2CBus.h` | I2C बस: `Wire` जैसे आदिम फ़ंक्शन + सहायक `writeRegister()`, `readRegisters()` (जाँचता है कि ठीक `count` बाइट आए), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, एक PWM आउटपुट (`measurePulseUs()` — असली पल्स का निदान) |
| `RegisterDevice.h` | `IRegisterDevice` — “8-बिट रजिस्टरों का सेट”; `I2cRegisterDevice` (पता), `SpiRegisterDevice` (CS, आवृत्ति, डेटा से पहले डमी बाइट) |
| `esp32/Esp32Board.h` | `IBoard` का क्रियान्वयन: `Wire` (सेंसर), `Wire1` (स्क्रीन, अगर चिप में दो I2C कंट्रोलर हैं), `SPI`, दो `HardwareSerial`, 5 LEDC चैनल |
| `esp32/Esp32I2CBus.h` | किसी भी `TwoWire` के ऊपर `II2CBus`, टाइमआउट 5 ms |
| `esp32/Esp32ServoOutput.h` | LEDC से PWM: 50 Hz, 14 बिट; पिन −1 — आउटपुट निकाला नहीं गया। ESP32Servo लाइब्रेरी इस्तेमाल नहीं होती — देखें [सीमाएँ](#ज्ञात-सीमाएँ) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | `SPI` और `HardwareSerial` के पतले रैपर |
| `stm32/*` | STM32H743: `Stm32Board` (+ रेडियो मॉडेम का UART4), बसें, PWM टाइमर, `Stm32FlashStorage` (सेटिंग फ़्लैश के एक सेक्टर में, बैकग्राउंड टास्क से लिखी जाती है), `compat/Preferences.h` |

### `storage/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `KeyValueStore.h` | RAM में CRC32 वाली “नेमस्पेस/कुंजी → बाइट” की इमेज, किसी भी माध्यम (`IFlashStorage`) के ऊपर; एक जैसा मान दोबारा नहीं लिखा जाता |
| `KvPreferences.h` | `KeyValueStore` के ऊपर ESP32 का `Preferences` API |

### `rc/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `RcChannelState.h` | 10 चैनलों का स्नैपशॉट |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → चैनल: 32 बाइट का फ़्रेम, CRC, चैनल का मान — निचले 12 बिट (`& 0x0FFF`); `isSignalLost()` = फ़्रेम नहीं हैं (या अभी तक एक भी नहीं आया) ∥ थ्रॉटल का failsafe मान; फ़्रेमों के काउंटर |

### `control/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `ControlCommand.h` | भौतिक चिह्नों में कंट्रोल सरफ़ेस की कमांड — स्टिक, ऑटोपायलट और मिक्सर की साझा भाषा |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(लक्ष्य, now)`; `mix(command)` → सर्वो रिवर्स के साथ PWM; फ़्लैपरॉन: एलेरॉन `flaps ± roll` (माइनस — एयर ब्रेक) |
| `FlapsController.h` | फ़्लैप का सहज निकलना/समेटना, समय पैरामीटर के रूप में दिया जाता है |
| `ThrottleManager.h` | स्टिक से थ्रॉटल; सिग्नल खोने पर — `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | SwA स्विच से ARM (थ्रॉटल नीचे होने पर OFF→ON का बदलाव + मोड के सेंसरों की जाँचें), तुरंत DISARM |
| `FlightOutputState.h` | वांछित PWM: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (पेलोड), `aux2` (कैमरा) |
| `Beeper.h` | बजर: फ़ंक्शन `BEEPER` से या ज़मीन पर “मॉडल खो गया” |
| `FlightOutputs.h` | आउटपुट की तालिका (`outputInfo()`: कुंजी, नाम, पिन, अनिवार्यता, स्थिति का फ़ील्ड) और उसके ऊपर लूप में सब कुछ: `begin()`, `write()`, `setFailsafe()`, स्थिति, `printPulseSelfTest()` |
| `FlightController.h` | हर चक्र में संचालन का क्रम, सिग्नल खोना (`applyLinkLoss()`), टेलीमेट्री के लिए गेटर |

### `autopilot/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 मोड), `Feature`, `Knob`, `PilotInputs`, नाम |
| `ControlBinding.h` | `Binding`, फ़ैक्टरियाँ `Bind::modes/mode/feature/knob`, `BindingCheck` की जाँचें |
| `PilotSwitches.h` | बाइंडिंग की तालिका → हर चक्र का मोड, फ़ंक्शन और नॉब; चालू होते समय का विन्यास |
| `Autopilot.h` | 12 मोड, failsafe RTH/ग्लाइड, जियोफ़ेंस, होम, मोड़ का समन्वय, ऑटो-ट्रिम; `update(armed, linkLost, थ्रॉटल, स्टिक)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (दूरी, बेयरिंग, विस्थापन), `Guidance` (हेडिंग के लिए रोल, घेरे का वेक्टर-फ़ील्ड) |
| `AltitudeSpeedController.h` | ऊँचाई के लिए पिच, हवाई गति के लिए थ्रॉटल (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | हाथ से लॉन्च और सोअरिंग की स्टेट मशीनें |
| `AutoTrim.h` | ऑटो-ट्रिम, NVS/फ़्लैश में सहेजा जाता है |
| `PidController.h` | PID: D सेंसर की गति से (जाइरोस्कोप, वेरियोमीटर), anti-windup, ARM के बिना इंटीग्रेटर जमा रहता है |
| `feedback/*` | **तैयारी, जुड़ा नहीं है:** अनुकूली फ़ीडबैक, टेकऑफ़ और लैंडिंग — देखें [फ़ीडबैक](#फ़ीडबैक-तैयारी-जुड़ा-नहीं-है) |

### `sensors/`

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `SensorInterface.h` | इंटरफ़ेस `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` और डेटा संरचनाएँ |
| `SensorSelection.h` | कौन-सी चिप कंपाइल होती है (`#define SENSOR_*`, बिल्ड फ़्लैग से बदला जा सकता है) और वह किस बस पर है (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | चिप की अक्षों को विमान की अक्षों में घुमाना (घड़ी की दिशा में 0/90/180/270°) — कम्पास के लिए और उस IMU के लिए जिसकी माउंटिंग कैलिब्रेशन नहीं हुई |
| `imu/ImuOrientation.h` | IMU की माउंटिंग “चिप की अक्ष → विमान की अक्ष” मैट्रिक्स के रूप में: `IMU_ROTATION_CW_DEG` से या तीन स्थितियों (समतल, नाक ऊपर, दायाँ पंख नीचे) से, जाँच के साथ; NVS में सहेजा जाता है |
| `imu/ImuSensorBase.h` | IMU का साझा हिस्सा: जाइरोस्कोप कैलिब्रेशन + उड़ान-पूर्व जाँच (स्थिरता, 1g, “ऊपर” माउंटिंग से मेल खाता है), माउंटिंग कैलिब्रेशन (`calibrateOrientation()`), स्केल, घुमाव, विमानन के चिह्न, बस की त्रुटियाँ |
| `imu/AttitudeEstimator.h` | रोल/पिच का कॉम्प्लिमेंटरी फ़िल्टर, यॉ का इंटीग्रल |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (चिप की पहचान WHO_AM_I से): ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz। **टेस्ट बेंच पर** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, UI फ़िल्टर 50 Hz। हार्डवेयर पर जाँचा नहीं गया |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2; I2C 0x6A/0x6B या SPI। हार्डवेयर पर जाँचा नहीं गया |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1.6 kHz, अप्रत्यक्ष रजिस्टर IPREG के ज़रिए लो-पास फ़िल्टर; I2C 0x68/0x69 या SPI। हार्डवेयर पर जाँचा नहीं गया |
| `baro/BarometerBase.h` | बैरोमीटर का साझा हिस्सा: सिर्फ़ नए सैंपल का पोलिंग, ऊँचाई, लो-पास फ़िल्टर से ऊर्ध्वाधर गति, बेस का कैलिब्रेशन, त्रुटियाँ |
| `baro/BMP388_Sensor.h` | I2C या SPI पर BMP388 (SPI के डमी बाइट के साथ), Bosch का कंपेंसेशन, तैयार होने के फ़्लैग पर पढ़ना। **टेस्ट बेंच पर (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, Bosch का कंपेंसेशन §8.1। हार्डवेयर पर जाँचा नहीं गया |
| `baro/SPL06_Sensor.h` | SPL06-001: डेटाशीट के गुणांक और सूत्र, 32 Hz ×16; I2C 0x76/0x77 या SPI। हार्डवेयर पर जाँचा नहीं गया |
| `baro/BMP581_Sensor.h` | BMP581: BMP5_SensorAPI का क्रम, 16×/2×, IIR; I2C 0x46/0x47 या SPI; मुख्य बैरोमीटर भी और पिटो ट्यूब भी। हार्डवेयर पर जाँचा नहीं गया |
| `mag/MagnetometerBase.h` | कम्पास का साझा हिस्सा: 50 Hz पोलिंग, NVS में hard-iron कैलिब्रेशन, अक्षों का घुमाव, हेडिंग, त्रुटियाँ |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C। **टेस्ट बेंच पर** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz। हार्डवेयर पर जाँचा नहीं गया |
| `gps/UbloxM10_Gps.h` | u-blox M10: CFG-VALSET से कॉन्फ़िगरेशन (115200 बॉड, 10 Hz, NAV-PVT, NMEA बंद), NAV-PVT का पार्सिंग। टेस्ट बेंच पर जुड़ा नहीं |
| `airspeed/AirspeedSensor.h` | हवाई गति सेंसर का इंटरफ़ेस: दबाव-अंतर, IAS, TAS, घनत्व |
| `airspeed/PitotDualBaroAirspeed.h` | घर में बनी पिटो ट्यूब: ट्यूब में BMP581 + फ़्यूज़लेज का बैरोमीटर; ज़मीन पर शून्य, लो-पास फ़िल्टर, स्थैतिक दबाव से घनत्व, ख़राबी की पहचान |

### `telemetry/` और एप्लिकेशन

| फ़ाइल | किसके लिए ज़िम्मेदार |
|---|---|
| `DebugLogger.h` | चैनलों के हिसाब से लॉग (`LogSettings.h`): हर चैनल की अपनी पंक्ति, अपनी डिबाउंस सहनशीलता और अपना मोड; मेन्यू खुला हो तो चुप रहता है |
| `DebugConsole.h` | पोर्ट मॉनिटर में टेक्स्ट मेन्यू (`h`) और हॉटकी (`l`/स्पेस/`s`/`i`/`o`/`m`/`p`/`b`); लॉग की सेटिंग मेन्यू से बाहर निकलते समय NVS में लिखता है और केवल ARM के बिना |
| `LogSettings.h` | लॉग के चैनल (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) और उनके मोड: बंद / बदलाव पर / लगातार; NVS में सहेजे जाते हैं |
| `WebDebugServer.h` | एक्सेस पॉइंट, रूट, JSON `/api/status`, कमांड का मेलबॉक्स; कोर 0 पर अपना टास्क |
| `WebDashboardPage.h` | डैशबोर्ड का HTML/JS एक ही लिटरल में; चैनल/आउटपुट/सेंसर की पंक्तियाँ ब्राउज़र JSON से बनाता है |
| `OledDisplay.h` | `II2CBus` के ऊपर U8g2 से SSD1306, अपना टास्क (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | QGroundControl / Mission Planner के लिए MAVLink 2: फ़्रेम, स्ट्रीम, PID पैरामीटर, ज़मीन से मोड बदलना |
| `LoopStats.h` | प्रति सेकंड आवृत्ति, चक्र का औसत और सबसे ख़राब समय (OLED) और पिछली बार पढ़ने के बाद का सबसे ख़राब (`takePeakUs()`, SYS पंक्ति) |
| `src/main.cpp` | ESP32: ऑब्जेक्ट बनाना, `setup()`, `vTaskDelayUntil` वाला `loop()` |
| `src/stm32/main.cpp` | STM32H743: वही ऑब्जेक्ट, MAVLink, SD कार्ड पर ब्लैक बॉक्स, टास्क `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: `HAL_SD_Init` के लिए SDMMC1 के पिन और क्लॉक; कंसोल की कुंजी `D` — USB DFU बूटलोडर में रीबूट |

---

## चिह्न-नियम: IMU से सर्वो तक

पूरी श्रृंखला में चिह्नों की एक ही प्रणाली है — इसलिए स्टिक और ऑटोपायलट
निश्चित रूप से कंट्रोल सरफ़ेस को एक ही दिशा में चलाते हैं, और हर सर्वो की
दिशा ठीक एक ही जगह तय होती है।

**1. सेंसर की अक्ष → विमान की अक्ष।** `ImuSensorBase` चिप की अक्षों को
मैट्रिक्स `ImuOrientation` (body = R · chip) से विमान की अक्षों में घुमाता है:
X नाक की ओर, Y बाईं ओर, Z ऊपर। मैट्रिक्स यहाँ से लिया जाता है:

- **माउंटिंग कैलिब्रेशन** से (कमांड `o`, NVS में सहेजा जाता है) — बोर्ड
  जैसे भी रखा हो। तीन स्थितियाँ: “समतल” Z अक्ष देता है (और क्षितिज भी —
  एक्सेलेरोमीटर का शून्य-ऑफ़सेट उसी में शामिल है), “नाक ऊपर” X अक्ष देता
  है (“ऊपर” का वह हिस्सा जो Z के लंबवत है), “दायाँ पंख नीचे” Y अक्ष देता है।
  चरण 2 की नाक और चरण 3 की नाक (Y × Z) लगभग 25° की सटीकता से मेल खानी
  चाहिए, वरना पायलट ने ग़लत ओर झुकाया — कैलिब्रेशन अस्वीकार कर दिया जाता
  है; नतीजा दोनों अनुमानों का औसत है। 300 यादृच्छिक माउंटिंग पर जाँचा गया
  (`test/test_imu_orientation`, त्रुटि < 0.1°);
- अन्यथा `Config::IMU_ROTATION_CW_DEG` से (बोर्ड चिप ऊपर की ओर; मान बताता है
  कि अगर नाक “12 बजे” की ओर है तो *चिप* की X अक्ष किधर देखती है), और क्षितिज
  वह स्थिति है जिसमें बोर्ड चालू हुआ।

जाइरोस्कोप के हर कैलिब्रेशन पर (चालू करना, `i`) — **उड़ान-पूर्व जाँच**:
जाइरोस्कोप का शोर < 0.5 °/s (स्थिरता; विश्राम में ~0.08), |a| ≈ 1g, “ऊपर”
सहेजे हुए से 45° के भीतर (बोर्ड को हिलाया नहीं गया)। पास नहीं हुई —
`ImuSensor::getPreflightProblem()` ≠ nullptr: `ArmingManager` स्थिरीकरण वाले
मोड ARM नहीं करता, `Autopilot::imuReady()` = false (सभी मोड में सुधार
शून्य, संपर्क टूटने पर ग्लाइड सहित)।

> मौजूदा GY-521 (MPU6500 की क्लोन) में चिप छपे हुए तीरों के सापेक्ष 90° घुमाकर
> सोल्डर की गई है: सिल्कस्क्रीन पर X का तीर = चिप की Y अक्ष। इसलिए माउंटिंग
> कैलिब्रेशन के बिना `IMU_ROTATION_CW_DEG = 90`। किसी भी फेर-बदल के बाद की जाँच:
> नाक ऊपर → P धनात्मक की ओर बढ़ता है, दायाँ पंख नीचे → R धनात्मक की ओर।

**2. कोण और गति (`ImuData`) — विमानन के चिह्न:**

| राशि | “+” का अर्थ |
|---|---|
| `roll`, `gyroX` | दायाँ पंख नीचे |
| `pitch`, `gyroY` | नाक ऊपर |
| `yaw`, `gyroZ` | नाक दाईं ओर (ऊपर से देखने पर घड़ी की दिशा में) |

**3. कमांड (`ControlCommand`, विक्षेपण µs में, ±500 = पूरी यात्रा):**

| फ़ील्ड | “+” का अर्थ | स्टिक से |
|---|---|---|
| `roll` | दाईं ओर रोल (दायाँ एलेरॉन ऊपर, बायाँ नीचे) | CH1: 2000 = दाईं ओर |
| `pitch` | नाक ऊपर (एलिवेटर ऊपर) | CH2 उलटे चिह्न के साथ: 2000 = अपने से दूर = नाक नीचे |
| `yaw` | नाक दाईं ओर (रडर और पहिया दाईं ओर) | CH4: 2000 = दाईं ओर |
| `flaps` | फ़्लैप नीचे (दोनों एलेरॉन नीचे) | SwB (CH6): 0 या `FLAPS_DEPLOYED_US`, `FLAPS_TRANSITION_MS` में सहजता से |

PID `त्रुटि = लक्ष्य − वास्तविक` गिनता है: दाईं ओर रोल (roll > 0) → रोल की
कमांड ऋणात्मक → विमान सीधा हो जाता है। ऑटोपायलट के सुधार स्टिक की कमांड में
मिक्सर से **पहले** जोड़े जाते हैं, उन्हीं चिह्नों में।

**4. कमांड → PWM।** `ControlMixer::mix()` हर सतह के पिछले किनारे का विक्षेपण
गिनता है (एलेरॉन: नीचे = “+”, बायाँ = `flaps + roll`, दायाँ = `flaps − roll`;
एलिवेटर: ऊपर = “+”; रडर: दाईं ओर = “+”) और उसे PWM `1500 ± विक्षेपण` में
बदलता है, जिन सर्वो के लिए `Config::*_REVERSED = true` है उनका चिह्न बदलकर।
डिफ़ॉल्ट मान स्टिक के लिए फ़र्मवेयर के पहले वाले व्यवहार को दोहराते हैं।
तैयार विमान पर जाँच [`PILOT_GUIDE.md`](PILOT_GUIDE.md) की उड़ान-पूर्व चेकलिस्ट में है।
रिवर्स `Config.h` में बदलना चाहिए, **ट्रांसमीटर पर नहीं** — वरना स्टिक और
ऑटोपायलट अलग-अलग दिशा में चलेंगे।

---

## RC चैनलों का नक़्शा, ARM और failsafe

स्रोत है `include/config/Channels.h`। ट्रांसमीटर FS-i6 (10 चैनल, मोड 2) +
रिसीवर FS-iA6B, iBUS 115200।

| चैनल | ट्रांसमीटर का हिस्सा | नाम | उपयोग |
|---|---|---|---|
| CH1 | दायाँ स्टिक ←→ | `AILERON` | रोल |
| CH2 | दायाँ स्टिक ↑↓ | `ELEVATOR` | पिच |
| CH3 | बायाँ स्टिक ↑↓ | `THROTTLE` | थ्रॉटल, पूरी यात्रा; < 950 = रिसीवर का failsafe |
| CH4 | बायाँ स्टिक ←→ | `RUDDER` | रडर + स्टीयरिंग पहिया (एक सर्वो) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (FS-i6 पर यह स्विच नीचे, अपनी ओर) |
| CH6 | SwB | `SWB` | डिफ़ॉल्ट रूप से फ़्लैप (≥ 1750 — निकले हुए) |
| CH7 | SwC (3 स्थितियाँ) | `SWC` | डिफ़ॉल्ट रूप से मोड: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | डिफ़ॉल्ट रूप से RTH |
| CH9 | VrA | `VRA` | डिफ़ॉल्ट रूप से स्थिरीकरण की ताक़त |
| CH10 | VrB | `VRB` | डिफ़ॉल्ट रूप से क्रूज़ की गति |

CH6–CH10 को `include/config/Controls.h` में एक ही पंक्ति से सौंपा जाता है
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#एक-पंक्ति-में-फ़ंक्शन-असाइन-करना))।

**ARM** (`ArmingManager`): स्विच का OFF→ON में जाना, थ्रॉटल < `THROTTLE_LOW_US`,
और मौजूदा मोड के लिए सेंसरों की जाँचें पास होना। वरना कारण Serial में बताकर
इनकार कर दिया जाता है, और OFF→ON का नया चक्र चाहिए। स्विच ON रखकर बोर्ड चालू
करने पर ARM नहीं होता। OFF — तुरंत DISARM। जब तक ARM नहीं है, ESC को जाने वाला
थ्रॉटल ज़बरदस्ती `PWM_MIN` रहता है।

**संपर्क टूटना** (`IBusReceiver::isSignalLost()`):

1. `RX_TIMEOUT_US` (500 ms) से ज़्यादा समय तक कोई फ़्रेम नहीं — तार टूटना या
   रिसीवर की बिजली जाना। चालू करने के बाद पहले फ़्रेम से पहले भी संपर्क खोया हुआ
   माना जाता है: चैनलों के डिफ़ॉल्ट मान (सभी 1500) को ट्रांसमीटर की कमांड नहीं
   समझा जाता।
2. थ्रॉटल < `RX_FAILSAFE_THROTTLE_US` (950) — ट्रांसमीटर में तय किया गया
   failsafe। **ट्रांसमीटर खो जाने पर FS-iA6B फ़्रेम भेजना बंद नहीं करता**, बल्कि
   आख़िरी मान दोहराता रहता है (टेस्ट बेंच पर जाँचा गया), इसलिए ट्रांसमीटर में
   failsafe सेट किए बिना संपर्क टूटना पहचाना नहीं जाता। सेटअप `PILOT_GUIDE.md` में है।

संपर्क टूटने पर क्या होता है (`FlightController::applyLinkLoss()`):

- **विमान ARM है, GPS और होम मौजूद हैं** (`FAILSAFE_RTH`) — मोटर के साथ
  **घर वापसी**, होम के ऊपर चक्कर; OLED पर — `FSRTH`, लॉग में — `FAILSAFE_RTH`;
- **विमान ARM है, GPS नहीं है** — **ग्लाइड**, मोटर `FAILSAFE_THROTTLE` पर:
  किसी भी मोड में, यहाँ तक कि MANUAL में भी, `Autopilot` रोल
  `FAILSAFE_GLIDE_ROLL_DEG` (0 — सीधा, 10–20° — पायलट के ऊपर घेरा) और पिच
  `FAILSAFE_GLIDE_PITCH_DEG` (−3°, ताकि मोटर के बिना गति न घटे) बनाए रखता है,
  फ़्लैप समेटे हुए; OLED पर — `GLIDE`, लॉग में — मोड `FAILSAFE_GLIDE`;
- **ARM नहीं है** (ज़मीन पर) या IMU जवाब नहीं देती — कंट्रोल सरफ़ेस न्यूट्रल में;
- मोड और फ़ंक्शन स्विच से नहीं बदलते, सेंसर पढ़े जाते रहते हैं। ARM रद्द नहीं
  होता — संपर्क लौटने पर विमान फिर से स्टिक और चुने हुए मोड की सुनता है
  (ऑटो टेकऑफ़ और हाथ से लॉन्च — केवल दोबारा शुरू से)।

---

## सेंसर का डेटा

संरचनाएँ `include/sensors/SensorInterface.h` में हैं।

### `ImuData`

| फ़ील्ड | इकाई | अर्थ |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | विमान की अक्षों में कोणीय गति, विमानन के चिह्न (ऊपर देखें) |
| `accelX`, `accelY`, `accelZ` | g | विमान की अक्षों में त्वरण: X नाक की ओर, Y बाईं ओर, Z ऊपर |
| `roll`, `pitch` | ° | कॉम्प्लिमेंटरी फ़िल्टर (α = 0.98, τ ≈ 0.1 s); शुरुआत सीधे एक्सेलेरोमीटर के कोण से होती है |
| `yaw` | ° | जाइरोस्कोप का इंटीग्रल, धीरे-धीरे खिसकता है; शुरुआती मान कम्पास की हेडिंग है |
| `temperature` | °C | चिप की तापमान (सूत्र MPU6050 या MPU6500 के लिए) |
| `timestamp` | µs | पढ़ने के क्षण का `micros()` |

IMU का कैलिब्रेशन (हर शुरुआत पर और कमांड `i` से): 2 s बिना हिले, जाइरोस्कोप →
शून्य-ऑफ़सेट, एक्सेलेरोमीटर → **मौजूदा स्थिति क्षितिज बन जाती है**।

### `BarometerData`

| फ़ील्ड | इकाई | अर्थ |
|---|---|---|
| `pressure` | Pa | दबाव |
| `temperature` | °C | सेंसर का तापमान |
| `altitude` | m | **कैलिब्रेशन बिंदु के सापेक्ष** (शुरुआत पर) ऊँचाई; सूत्र `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | असली सैंपल (50 Hz) पर ऊँचाई का अवकलज, τ = 0.5 s वाले लो-पास फ़िल्टर से गुज़ारा हुआ |
| `timestamp` | µs | आख़िरी नए सैंपल का क्षण |

### `MagData`

| फ़ील्ड | इकाई | अर्थ |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | hard-iron कैलिब्रेशन के बाद का क्षेत्र, विमान की अक्षों में (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, झुकाव के सुधार के बिना; माप की दिशा अभी तैयार विमान पर जाँची नहीं गई है |
| `timestamp` | µs | पढ़ने का क्षण (50 Hz) |

### `GpsData`

| फ़ील्ड | इकाई | अर्थ |
|---|---|---|
| `latitude`, `longitude` | ° | UBX-NAV-PVT से |
| `altitude` | m | समुद्र तल से ऊपर (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | ज़मीन के सापेक्ष गति और दिशा |
| `numSatellites`, `fixType` | — | 0 = फ़िक्स नहीं, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | मॉड्यूल के सटीकता-अनुमान |

**`isAvailable()` का क्या अर्थ है।** I2C सेंसरों के लिए — सेंसर ने `begin()` पर
जवाब दिया **और** हाल के पठन लगातार विफल नहीं हो रहे (MPU — ~0.1 s,
बैरोमीटर और कम्पास — बिना जवाब के ~0.5 s)। पठन विफल होने पर डेटा कचरे से
नहीं लिखा जाता: पिछले मान बने रहते हैं और त्रुटि का काउंटर बढ़ता है (कमांड
`s` से दिखता है)। GPS के लिए — कम से कम एक वैध NAV-PVT, और आख़िरी वाला
`GPS_TIMEOUT_US` से पुराना नहीं।

**अगर सेंसर नहीं है** (`nullptr` या `isAvailable() == false`), `Autopilot`
कोई सुधार नहीं देता, और विमान MANUAL की तरह चलाया जाता है। `main.cpp` केवल
उन्हीं सेंसरों को कैलिब्रेट करता है जिन्होंने जवाब दिया।

---

## FlightController::update() का विश्लेषण

`loop()` से हर 2 ms पर बुलाया जाता है। क्रम ही प्राथमिकता है:

1. **`receiver.update()`** — जमा हुए iBUS बाइटों का पार्सिंग।
2. **स्विच** — `switches->update(rc)`, केवल जीवित संपर्क पर (failsafe फ़्रेम में
   चैनल स्विच को नहीं दर्शाते): मोड (केवल बदलने पर), फ़ंक्शन, नॉब।
3. **पायलट का थ्रॉटल** — `throttle.update(rc, receiverFailsafe)`।
4. **स्टिक** — `mixer.fromSticks(rc)` × `Knob::RATES`; फ़्लैप —
   `mixer.updateFlaps(target)` (ब्रेक, स्विच, नॉब; संपर्क के बिना — 0)।
5. **सेंसर और ऑटोपायलट** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **हमेशा**, संपर्क के बिना भी: कोण के फ़िल्टर जमने नहीं चाहिए। जब तक ARM
   नहीं है, PID चलता है (कंट्रोल सरफ़ेस झुकाव पर प्रतिक्रिया देती हैं — मेज़ पर
   सुविधाजनक), पर इंटीग्रेटर शून्य पर रखा जाता है। संपर्क के बिना और ARM में —
   failsafe RTH या ग्लाइड।
6. **बीपर** — `Beeper`।
7. **संपर्क टूटना** — `applyLinkLoss()`: ARM में — कंट्रोल सरफ़ेस और थ्रॉटल
   ऑटोपायलट की failsafe कमांड के अनुसार, वरना न्यूट्रल और मोटर बंद; `return`।
   नीचे की हर चीज़ पर पूर्ण प्राथमिकता।
8. **ARM** — `arming.update(rc, false)`।
9. **कमांड** — `autopilot->getCommand()`: स्थिरीकरण वाले मोड में स्टिक
   वांछित कोण है, और अंतिम कंट्रोल सरफ़ेस कमांड ऑटोपायलट देता है।
10. **मिक्सर** — `mixer.mix(command)` → एलेरॉन (फ़्लैप + रोल), एलिवेटर और रडर
    के PWM, रिवर्स का ध्यान रखते हुए।
11. **थ्रॉटल** — `autopilot->applyThrottle(pilotThrottle)`: पायलट का थ्रॉटल,
    ऑटोपायलट का थ्रॉटल, या दोनों में से अधिकतम (ऑटो टेकऑफ़)। फिर, अगर ARM नहीं
    है या `MOTOR_KILL` है, — ज़बरदस्ती `PWM_MIN`। यह जाँच सबसे अंत में है, ताकि
    कोई मोड थ्रॉटल को ARM को बायपास करके न निकाल सके।
12. **AUX** — पेलोड (`PAYLOAD_DROP`) और कैमरा (`CAMERA_TILT`, `CAMERA_STAB`)।
13. **`outputs.write(output)`** — 7 आउटपुट पर PWM।

---

## वेब डैशबोर्ड का HTTP API

क्रियान्वयन — `include/telemetry/WebDebugServer.h`। एक्सेस पॉइंट: SSID
`OpenPlane-Debug`, पासवर्ड `12345678`, पता `http://192.168.4.1`।

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached` — ऑब्जेक्ट बिल्ड में मौजूद है; `available` — सेंसर सचमुच जवाब देता
  है। डेटा के फ़ील्ड **केवल** `available: true` पर जोड़े जाते हैं।
- `outputs.*.attached` — MCU ने LEDC चैनल और पिन आवंटित कर दिया है; भौतिक सर्वो
  जुड़ा है या नहीं, यह सॉफ़्टवेयर से नहीं दिखता (पल्स जाँचने के लिए — कंसोल,
  कमांड `p`)।
- `rollCorr`/`pitchCorr` — ऑटोपायलट की अंतिम कमांड में से स्टिक घटाकर, µs में।
  `throttleCorr` — ऑटोपायलट का थ्रॉटल, % में (0, जब तक थ्रॉटल पायलट के पास है)।
- `nav` — नेविगेशन: होम, उसकी दूरी और बेयरिंग, कोर्स और लक्ष्य कोर्स,
  नेविगेशन के लिए गति (पिटो ट्यूब / GPS), जियोफ़ेंस, स्टॉल; `features` — चालू
  किए गए स्विच-फ़ंक्शन।

### `POST /api/setmode`

`{ "mode": 1 }` — `AutopilotMode` का क्रमांक: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE। मोड तब तक बना रहता है जब तक
पायलट मोड-स्विच नहीं पलटता।

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — फ़ील्ड `kpRoll`, `kiRoll`,
`kdRoll`, `kpPitch`, `kiPitch`, `kdPitch` में से कोई भी; छोड़े गए फ़ील्ड पहले जैसे
रहते हैं।

दोनों कमांड उड़ान-चक्र अगले चक्र में लागू करता है (देखें
[FreeRTOS टास्क](#freertos-टास्क-और-नियंत्रण-लूप))।

### `GET /`

HTML-डैशबोर्ड: 10 चैनलों के बार, ARM/संपर्क, आउटपुट, सेंसर, मोड के बटन, PID का
फ़ॉर्म। हर 200 ms पर `/api/status` से पूछता है।

---

## कंसोल और निदान

पोर्ट मॉनिटर — 115200, कनेक्टर “COM”। क्रियान्वयन — `DebugConsole` और
`DebugLogger` ([संदर्भ](reference/telemetry.md))। कुंजियाँ तुरंत काम करती हैं,
Enter ज़रूरी नहीं; कैलिब्रेशन और `p` चक्र को रोक देते हैं, इसलिए केवल ARM के
बिना उपलब्ध हैं।

| कुंजी | क्या करती है |
|---|---|
| `h` / `?` | मुख्य मेन्यू |
| `l` | मेन्यू “लॉग में क्या दिखाना है” (चैनल, मोड, अवधि) |
| स्पेस | लॉग को रोकना / जारी रखना |
| `s` | सभी सेंसरों का `printStatus()`: डेटा, बस की त्रुटियों के काउंटर, कैलिब्रेशन, उड़ान-पूर्व जाँच |
| `i` | जाइरोस्कोप का कैलिब्रेशन + उड़ान-पूर्व जाँच (2 s बिना हिले) |
| `o` | तीन स्थितियों से IMU की माउंटिंग का कैलिब्रेशन, NVS में सहेजा जाता है |
| `m` | कम्पास का कैलिब्रेशन (15 s घुमाना), NVS में सहेजा जाता है |
| `p` | आउटपुट की स्व-जाँच: हर पिन पर असली पल्स बनाम अपेक्षित |

लॉग चैनलों में बँटा है (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`, `MAG`, `GPS`,
`IMU`, `SYS`), हर एक का मोड “बंद / बदलाव पर / लगातार” है; सेटिंग NVS में रहती
हैं और मेन्यू बंद करते समय लिखी जाती हैं, केवल ARM के बिना। डिफ़ॉल्ट रूप से
`STAT` (बदलाव पर) और `SYS` (हर 10 s में एक बार) चालू हैं:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (10 s में सबसे ख़राब) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

सभी चैनलों के प्रारूप [संदर्भ](reference/telemetry.md#debuglogger) में हैं।

---

## बोर्ड का चुनाव और पिन-व्यवस्था

| कमांड | `board` | मैक्रो | स्थिति |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **मुख्य, डिफ़ॉल्ट।** सभी सेंसरों के साथ टेस्ट बेंच पर जाँचा गया |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | पुराना प्रोटोटाइप, हाथ के नियंत्रण पर उड़ा था |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | टेस्ट बेंच के लिए, पिन-व्यवस्था हार्डवेयर पर जाँची नहीं गई |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: पूरा फ़र्मवेयर + MAVLink + SD पर ब्लैक बॉक्स; ख़ाली बोर्ड पर जाँचा गया ([नीचे](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | वही DevEBox H743 पर: कंसोल — USB CDC, फ़र्मवेयर DFU से |

| उपयोग | ESP32-S3 (टेस्ट बेंच) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| बायाँ / दायाँ एलेरॉन | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| एलिवेटर / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| रडर | GPIO18 | — (पिन नहीं) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| सेंसरों का I2C SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| OLED का I2C SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / नहीं (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → कनेक्टर “COM” | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** GPIO33–37 ऑक्टल PSRAM के पास हैं, 26–32 फ़्लैश के पास,
  19/20 USB के पास, 43/44 Serial के पास, 48 RGB-LED है; 0/3/45/46 — strapping।
- **ESP32-C3:** GPIO4/5 पर एलेरॉन S3 के मुक़ाबले अदल-बदल हैं। पूरे सेट के लिए
  पिन कम पड़ते हैं: BMP388 का CS और GPS का RX strapping पिनों पर हैं, GPS
  बिना TX के है (केवल प्राप्ति, UBX-CFG नहीं)। विवरण `Config.h` में।

### STM32H743

STM32H743VIT6 (Cortex-M7 480 MHz, 2 MB फ़्लैश, 1 MB RAM) — **पूरा फ़र्मवेयर**:
वही सेंसर, ऑटोपायलट, स्विच, कंसोल और स्क्रीन जो ESP32-S3 पर हैं, साथ में MAVLink
टेलीमेट्री और SD कार्ड पर ब्लैक बॉक्स। यह बनता है, cppcheck और साझा कोड के सभी
नेटिव टेस्ट पास करता है। हार्डवेयर पर **सेंसर के बिना DevEBox H743 बोर्ड**
जाँचा गया है: बूट, USB पर कंसोल, SD कार्ड, ब्लैक बॉक्स —
[TESTING.md](TESTING.md#stm32-बोर्ड-पर-टेस्ट) — और साथ ही iBUS, ARM तथा सर्वो और
मोटर पर PWM: ट्रांसमीटर से हाथ के मोड में नियंत्रण (वीडियो पर)। STM32 पर सेंसर
अभी टेस्ट बेंच का इंतज़ार कर रहे हैं। मुख्य उड़ान बोर्ड — ESP32-S3।

- **HAL** — `include/hal/stm32/`: `Stm32Board` (वही API जो `Esp32Board` का है,
  साथ में `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (`HardwareTimer` का हार्डवेयर PWM, कई आउटपुट के लिए एक
  टाइमर)। विस्तार से — [reference/hal.md](reference/hal.md#stm32h743-के-लिए-क्रियान्वयन)।
- **सेटिंग और कैलिब्रेशन** — NVS नहीं, बल्कि फ़्लैश के आख़िरी सेक्टर में
  `KeyValueStore` (`include/storage/`, `hal/stm32/Stm32FlashStorage.h`)।
  प्रोजेक्ट का कोड अब भी `#include <Preferences.h>` लिखता है: env `stm32h743`
  में डायरेक्टरी `include/hal/stm32/compat/` `-I` में है, और वहाँ उसी API वाला
  `Preferences` रखा है। इमेज CRC32 के साथ है: ख़राब इमेज (मिटाने के दौरान बिजली
  गई) ख़ाली पढ़ी जाती है। फ़्लैश में लिखना बैकग्राउंड टास्क में होता है: 128 KB के
  सेक्टर को मिटाने में सेकंड लगते हैं, पर सेक्टर बैंक 2 में है और कोड बैंक 1 से चलता
  है, और उड़ान का टास्क बैकग्राउंड टास्क को बिना रुके हटा देता है।
- **टास्क** — STM32duino FreeRTOS लाइब्रेरी का FreeRTOS, एक कोर, प्राथमिकता से
  प्रीएम्प्शन (`hal/Rtos.h`): `flight` (5) — उड़ान चक्र, MAVLink, लॉग, कंसोल;
  `oled` (1) और `storage` (1) — बैकग्राउंड में; `bbox` (2) — SD कार्ड पर ब्लैक
  बॉक्स की रिकॉर्डिंग।
- **SD कार्ड पर ब्लैक बॉक्स** — SDMMC1, 4 बिट, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, पिन — `src/stm32/sd_msp.cpp`)। कार्ड सामान्य FAT32
  ही रहता है: उस पर पहले से बनी फ़ाइल `BLACKBOX.BIN` रखी होती है, फ़र्मवेयर उसके
  भीतर कच्चे ब्लॉक लिखता है और फ़ाइल सिस्टम को ख़ुद नहीं छूता
  (`storage/Fat32File.h` — केवल पढ़ना)। कार्ड की तैयारी और डेटा निकालना —
  [BLACKBOX.md](BLACKBOX.md#sd-कार्ड-stm32h743)।
- **टेलीमेट्री** — UART4 पर MAVLink 2 (`telemetry/MavlinkTelemetry.h`), Wi-Fi
  डैशबोर्ड की जगह: QGroundControl / Mission Planner, ज़मीन से मोड और PID बदलना।
  विस्तार से —
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#ग्राउंड-स्टेशन-wi-fi-डैशबोर्ड-और-mavlink)।
- **पिन-व्यवस्था** — `Config.h` में `BOARD_STM32H743` का ब्लॉक, पिन WeAct
  MiniSTM32H743VITx पर ख़ाली पिनों में से चुने गए और STM32duino की तालिकाओं से
  मिलाए गए:

| उपयोग | STM32H743 | पेरिफ़ेरल |
|---|---|---|
| बायाँ / दायाँ एलेरॉन | PA0 / PA1 | TIM2_CH1 / CH2 |
| एलिवेटर / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| रडर | PD14 | TIM4_CH3 |
| AUX1 (पेलोड) / AUX2 (कैमरा) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX — आरक्षित) | PE7 (PE8) | UART7 |
| सेंसरों का I2C SDA / SCL | PB11 / PB10 | I2C2 |
| OLED का I2C SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / बैरोमीटर | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| रेडियो मॉडेम MAVLink RX / TX | PD0 / PD1 | UART4 |
| बजर | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)** — env `stm32h743-devebox`: वही कोड, कोर का अपना
  वेरिएंट, कंसोल USB-C से वर्चुअल COM पोर्ट (CDC) के रूप में — USB-UART की ज़रूरत
  नहीं। पहली बार फ़र्मवेयर USB से बिल्ट-इन बूटलोडर (DFU) के ज़रिए:
  1. Windows: “STM32 BOOTLOADER” के लिए WinUSB ड्राइवर एक बार इंस्टॉल करें
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver)।
  2. पिन **BT0** (BOOT0) को तार से **3V3** से जोड़ें, **RST** दबाकर छोड़ें:
     बोर्ड DFU मोड में है (DevEBox पर BOOT0 का बटन नहीं है)।
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`)।
  4. BT0 का तार हटाया जा सकता है — फ़र्मवेयर अपने आप चालू होता है।

  आगे तार की ज़रूरत नहीं: कंसोल में कुंजी **`D`** (किसी भी मेन्यू से, ARM पर नहीं)
  बोर्ड को बूटलोडर में रीबूट करती है: RAM में निशान → रीसेट → क्लॉक कॉन्फ़िगर
  करने से पहले सिस्टम मेमोरी में छलाँग (`src/stm32/bootloader.cpp`)। H7 पर चल रहे
  फ़र्मवेयर से सीधी छलाँग अटक जाती है — बोर्ड पर जाँचा गया, इसीलिए दो चरणों में।
  खुला हुआ कंसोल (USB CDC) चाहिए; अगर बोर्ड जवाब न दे — BT0 के तार के साथ RST।
- **एंट्री पॉइंट** — `src/stm32/main.cpp` (ESP32 की बिल्ड में `build_src_filter`
  से बाहर रखा गया)। ऑब्जेक्ट वही हैं जो `src/main.cpp` में हैं; `loop()` की जगह
  टास्क हैं, और `vTaskStartScheduler()` `setup()` के अंत में है।
- **बोर्ड को पहली बार चालू करना:** `pio run -e stm32h743 -t upload` (ST-Link),
  मॉनिटर LPUART1 पर USB-UART के ज़रिए; `b` — बसों पर सेंसर दिखते हैं या नहीं, `s` —
  सेंसरों की स्थिति, `p` — आउटपुट पर पल्स (प्रोपेलर हटाएँ), फिर ट्रांसमीटर और
  रेडियो मॉडेम से QGroundControl।

---

## नया सेंसर कैसे जोड़ें

### A) मौजूदा श्रेणी की एक और चिप (IMU, बैरोमीटर, कम्पास)

साझा हिस्सा बेस क्लास में पहले से लिखा है — इसलिए चिप का ड्राइवर छोटा बनता है:

1. `include/sensors/<category>/<Name>_Sensor.h` बनाएँ और `ImuSensorBase` /
   `BarometerBase` / `MagnetometerBase` से इनहेरिट करें। कंस्ट्रक्टर
   `IRegisterDevice&` लेता है — ड्राइवर को पता नहीं होता कि यह I2C है या SPI।
2. लागू करें:
   - `begin()` — `device.begin()`, चिप ID जाँचना, रजिस्टर लिखना,
     `setAvailable(true/false)` बुलाना;
   - IMU: `readSample()` (चिप की अक्षों में कच्चे accel/gyro/temp),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`;
   - बैरोमीटर: `isNewSampleReady()` (तैयार होने का फ़्लैग या सीधे `true`) और
     `readSample()` (दबाव Pa में, तापमान °C में), पोलिंग की अवधि बेस के
     कंस्ट्रक्टर में है;
   - कम्पास: `readRaw()` (चिप की अक्षों में X/Y/Z) और `lsbPerMicroTesla()`,
     कैलिब्रेशन के लिए NVS नेमस्पेस का नाम बेस के कंस्ट्रक्टर में है।
3. अगर SPI पर चिप को डेटा से पहले डमी बाइट या कोई ख़ास आवृत्ति चाहिए — स्थैतिक
   फ़ैक्टरी `spiDevice(bus, cs)` जोड़ें, जैसी `BMP388_Sensor` में है।
4. `SensorSelection.h` में एक शाखा: `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;` और `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` या SPI की फ़ैक्टरी)। सेंसर बदलने पर
   `main.cpp` को नहीं छुआ जाता।
5. फ़ाइल बदले बिना नए सेंसर के साथ बिल्ड जाँचें — फ़्लैग से:
   `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`, फिर
   तीनों एनवायरनमेंट, फिर हार्डवेयर पर।

### B) नई श्रेणी

1. डेटा की संरचना और इंटरफ़ेस — `SensorInterface.h` में, `GpsSensor`/`GpsData` के
   नमूने पर।
2. अगर श्रेणी में साझा तर्क है (फ़िल्टर, कैलिब्रेशन) — `BarometerBase` के नमूने
   पर बेस क्लास।
3. `Autopilot` के कंस्ट्रक्टर में nullable पॉइंटर (सेंसर नहीं — कोई असर नहीं, क्रैश
   नहीं) और `GET /api/status` में `attached`/`available` की जोड़ी वाले फ़ील्ड।

### नई बस या पेरिफ़ेरल

`include/hal/` में नया इंटरफ़ेस, `include/hal/esp32/` और `include/hal/stm32/`
में क्रियान्वयन, पहुँच `IBoard` के ज़रिए।

---

## ऑटोपायलट का नया मोड कैसे जोड़ें

1. `enum AutopilotMode` में एक मान (`autopilot/AutopilotTypes.h`, `MODE_COUNT` से
   पहले), `AutopilotNames::mode()` / `modeShort()` में नाम और छोटा नाम (5 अक्षर
   तक, OLED के लिए)।
2. हैंडलर `run<Mode>()` और `Autopilot::runMode()` में एक शाखा; शुरुआती लक्ष्य
   (कोर्स, ऊँचाई, घेरे का केंद्र) `initializeMode()` में। मोड `desiredRoll`/
   `desiredPitch` तय करता है और `stabilizeOrManual()` (IMU के बिना कंट्रोल सरफ़ेस
   पायलट के पास) या `stabilizeOrNeutral()` (IMU के बिना — न्यूट्रल) बुलाता है।
   ज़रूरी सेंसर के बिना — सुरक्षित व्यवहार, क्रैश नहीं। इंटीग्रेटर केवल `armed`
   पर जमा होता है।
3. थ्रॉटल: `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) और `autoThrottlePct`,
   या `autoThrottle()` — नॉब से / पिटो ट्यूब से क्रूज़ का थ्रॉटल। इसके लिए
   `FlightController` नहीं बदलता।
4. ट्रांसमीटर पर — `config/Controls.h` में एक पंक्ति
   (`Bind::mode(Channels::SWD, MODE_NEW)`)। डैशबोर्ड और MAVLink मोड को उसके क्रमांक
   से उठा लेते हैं; MAVLink के लिए — `MavlinkModes::toCustomMode()` /
   `fromCustomMode()` में ArduPlane का सबसे नज़दीकी मोड।
5. अगर मोड को ARM के लिए सेंसर चाहिए — `ArmingManager`।
6. टेस्ट: हर सेंसर पर प्रतिक्रिया — `test/native/test_autopilot_modes`, बंद-लूप
   उड़ान — `test/native/test_sim` में परिदृश्य (विमान का मॉडल
   `helpers/PlaneSim.h`, बेंच `helpers/SimHarness.h`)। फिर — प्रोपेलर के बिना मेज़:
   कंट्रोल सरफ़ेस को झुकाव पर सीधा करने की दिशा में प्रतिक्रिया देनी चाहिए।
7. [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md) में एक खंड।

---

## फ़ीडबैक (तैयारी, जुड़ा नहीं है)

`include/autopilot/feedback/` — ऑटोपायलट का अगला क़दम। **न `FlightController`,
न `Autopilot`, न `main.cpp` इन फ़ाइलों को शामिल करता है:** उड़ान परीक्षण के लिए
प्रोटोटाइप अभी नहीं है, फ़र्मवेयर इनके बिना चलता है। इन्हें बंद-लूप सिमुलेशन
(`test/test_feedback/`) से सीधे बोर्ड पर जाँचा जाता है।

### क्यों

आज `Autopilot` कोण पर PID है: त्रुटि × गुणांक = कंट्रोल सरफ़ेस। उसे नहीं पता कि
इससे विमान पर क्या हुआ, और गुणांक केवल एक गति के लिए सही हैं: कम गति पर सरफ़ेस
कमज़ोर होती है और PID कम सुधारता है, ज़्यादा गति पर ज़्यादा खींच देता है।
फ़ीडबैक लूप को **विमान की प्रतिक्रिया** पर बंद करता है:

- सरफ़ेस झुकाई, पर विमान ज़रूरत से धीमा घूम रहा है — और जोड़ें, जब तक वह
  पहुँच न जाए;
- कितनी सरफ़ेस चाहिए — यह उड़ान में नापा जाता है और गति के हिसाब से दोबारा
  गिना जाता है;
- विमान ग़लत दिशा में घूम रहा है — चिह्न उलझ गया है, पलटकर जाँचें;
- कोण सीधा कर दिया, पर गति गिर रही है — थ्रॉटल बढ़ाएँ और नाक नीचे करें, जब तक
  विमान स्टॉल न हो जाए;
- टेकऑफ़ और लैंडिंग — चरणों में, सेंसर जो दिखाते हैं उसके अनुसार।

### मॉड्यूल

| फ़ाइल | क्या करती है |
|---|---|
| `FlightSnapshot.h` | विमान के बारे में फ़ीडबैक को एक चक्र में जो कुछ पता है। इकलौता इनपुट: मॉड्यूल सेंसर और RC सीधे नहीं पढ़ते, इसलिए उन्हें सिमुलेशन और लॉग पर चलाया जा सकता है |
| `FeedbackOutput.h` | एक चक्र का आउटपुट: अक्षों के हिसाब से कंट्रोल सरफ़ेस के विक्षेपण, अक्ष चालू है या नहीं, अक्ष का चिह्न, थ्रॉटल (तय करना / इससे कम नहीं), कारण |
| `FeedbackConfig.h` | सभी स्थिरांक (जुड़ने पर `Config.h` में चले जाएँगे) |
| `SpeedEstimator.h` | गति (पिटो ट्यूब > GPS) और IMU से अनुदैर्ध्य त्वरण: `dV/dt = g·(ax − sin θ)` — गति सेंसर के बिना भी दिखता है कि “गति गिर रही है” |
| `AirborneDetector.h` | हवा में / ज़मीन पर: सीखना, इंटीग्रल जमा करना और स्टॉल ढूँढना केवल उड़ान में मायने रखता है |
| `ControlEffectivenessEstimator.h` | हर अक्ष के लिए पुनरावर्ती न्यूनतम वर्ग (RLS) से मॉडल `ε = b·u(t−delay) + a·ω + c` सीखता है |
| `AdaptiveRateController.h` | कोण → कोणीय गति → कोणीय त्वरण → सीखे हुए मॉडल से कंट्रोल सरफ़ेस की कैस्केड |
| `StallGuard.h` | गति खोने और स्टॉल से सुरक्षा |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | टेकऑफ़ (रनवे से या हाथ से) और लैंडिंग, सेंसरों के अनुसार चरणों में |
| `FeedbackSupervisor.h` | सब कुछ एक साथ: चक्र में क्रम, प्राथमिकताएँ, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, जोड़ने की योजना |
| `FeedbackModules.h` | सबके लिए एक ही include |

### यह कैसे काम करता है

**कंट्रोल सरफ़ेस की प्रभावशीलता।** अक्ष का मॉडल: कोणीय त्वरण
`ε = b·u + a·ω + c`। `b` बताता है कि 1 µs सरफ़ेस कितने °/s² देती है (चिह्न —
प्रतिक्रिया की दिशा), `a` अवमंदन है (हवा घूर्णन को रोकती है; इस पद के बिना स्थिर
घूर्णन में `b` का अनुमान शून्य की ओर चला जाता), `c` स्थिर आघूर्ण है (गुरुत्व-केंद्र,
ट्रिमर, प्रोपेलर)। सरफ़ेस का बल ∝ ρV² है, इसलिए `b` संदर्भ गति पर सीखा जाता है
और `(V/Vref)²` से गुणा किया जाता है: विमान तेज़ हुआ — सरफ़ेस तुरंत “मज़बूत हो
गई”, दोबारा सीखे बिना। पिटो ट्यूब की सूचित गति में हवा का घनत्व पहले से शामिल
है, इसलिए ऊँचाई अपने आप गिन ली जाती है; गति सेंसर के बिना पैमाना 1 रहता है, और
`b` सीधे सीखा जाता है।

डेटा 20 ms के अंतरालों पर लिया जाता है: अंतराल का औसत त्वरण सिरों पर जाइरोस्कोप
का अंतर / अवधि है, और उसके अनुरूप उसी अंतराल की औसत सरफ़ेस और औसत कोणीय गति
हैं (सरफ़ेस — `RESPONSE_DELAY_MS` की देरी के साथ)। फिर समीकरण के दोनों पक्ष एक ही
2 Hz लो-पास फ़िल्टर से गुज़रते हैं: अनुपात नहीं बदलता, जबकि वे उच्च आवृत्तियाँ हट
जाती हैं जहाँ सर्वो की जड़ता के कारण “शुद्ध देरी” का मॉडल ग़लत हो जाता है। सीखना
केवल हवा में और केवल तब संभव है जब सरफ़ेस को “हिलाया” जा रहा हो (लगभग 0.3 s में
विस्तार ≥ `MIN_EXCITATION_US`); पायलट के स्टिक भी हिलाना ही हैं, इसलिए अनुमान
MANUAL में भी सीखता है।

**रेगुलेटर।** तीन चरण, अक्ष दर अक्ष:

```
ω* = ANGLE_GAIN · (target − angle)              "नाक 10° नीचे है — 40°/s से ऊपर उठाएँ"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "ज़रूरत से धीमा घूम रहा है — सुधारें"
surface = (ε* − a·ω − c) / b                    सीखे हुए मॉडल के ज़रिए
```

इंटीग्रल `I` °/s में रखा जाता है, सरफ़ेस के µs में नहीं — इसलिए `b` का अनुमान
बदलने पर भी वह सही रहता है। ज़मीन पर इंटीग्रल जमा रहता है (टेकऑफ़-रोल / रोलआउट
में हेडिंग को छोड़कर), सरफ़ेस के सीमा पर पहुँचने पर वह सीमा की ओर नहीं जुड़ता।
समन्वित मोड़ का ध्यान रखा जाता है (अगर गति पता हो): बैंक में पिच के लिए
`g·sin φ·tg φ / V` और यॉ के लिए `g·sin φ / V` चाहिए।

**अक्षों के चिह्न — केवल ज़मीन पर।** उड़ान में अक्ष न बंद किए जाते हैं, न पलटे जाते
हैं: IMU की माउंटिंग कैलिब्रेशन `o` और चालू करते समय की जाँच से तय होती है,
सरफ़ेस की दिशाएँ पायलट की उड़ान-पूर्व जाँच से। हवा में परोक्ष संकेत (स्टॉल से
निकलना, स्पिन, कलाबाज़ियाँ, झोंके) धोखा दे सकते हैं, और ऐसे क्षण में बंद या पलटा
हुआ अक्ष विमान की क़ीमत चुकाता है। अगर किसी अक्ष का `b` का अनुमान पक्के तौर पर
ऋणात्मक है, तो यह केवल `reason` में चेतावनी है (“सरफ़ेस पर उल्टी प्रतिक्रिया? ज़मीन
पर जाँचें”); ऋणात्मक अनुमान रेगुलेटर में नहीं जाता — अक्ष पूर्व-धारणा वाले मॉडल
पर चलता है।

**स्टॉल से सुरक्षा।** दो स्तर। *LowEnergy* — नाक उठी होने पर गति तेज़ी से गिर
रही है, या स्टॉल के क़रीब है (< 1.25·Vs), या एलिवेटर की प्रभावशीलता घट गई है:
थ्रॉटल ≥ 80 %, पिच ≤ 5°। *Stall* — गति स्टॉल से नीचे है, नाक एलिवेटर के विरुद्ध
गिर रही है, कम ऊर्जा पर पंख एलेरॉन के विरुद्ध झुक रहा है: पूरा थ्रॉटल, नाक नीचे,
बैंक ≤ 10°, एलेरॉन सीमित (बड़ा एलेरॉन पंख का सिरा स्टॉल करा देता है)। उपाय
हिस्टेरिसिस के साथ हटते हैं (गति ≥ 1.5·Vs)। संपर्क टूटने पर थ्रॉटल को नहीं
छुआ जाता, ज़मीन के बिल्कुल पास (फ़्लेयर, रोलआउट) सुरक्षा बंद रहती है — लैंडिंग
ख़ुद नियंत्रित स्टॉल है।

**टेकऑफ़।** `WaitThrottle` (मोटर रुकी है) → पायलट ने थ्रॉटल ≥ 50 % दिया →
`GroundRoll` (पूरा थ्रॉटल, पंख समतल, हेडिंग रडर और पहिया थामते हैं, एलिवेटर
मुक्त) → उड़ान भरने की गति, या गति सेंसर के बिना टाइमआउट → `Climb` (12°,
पूरा थ्रॉटल) → ऊँचाई 30 m → `Complete`। हाथ से (`TAKEOFF_HAND_LAUNCH`) दौड़ की जगह
`WaitLaunch`: मोटर फेंकने के बाद ही चालू होती है (अनुदैर्ध्य त्वरण ≥ 1g)। उड़ान
भरने से पहले थ्रॉटल हटाया — रद्द।

**लैंडिंग।** `Approach` (थ्रॉटल 25 %, 1 m/s से उतरना — पिच ऊर्ध्वाधर गति की
त्रुटि से, बैंक पायलट से ≤ 20°) → ऊँचाई 2 m → `Flare` (थ्रॉटल 0, उसी नियम से
उतरने की गति घटकर 0.3 m/s) → एक्सेलेरोमीटर पर झटका या “नीचे और घूम नहीं रहा” →
`Rollout` (पहिये से हेडिंग) → `Complete`। पायलट का थ्रॉटल ≥ 80 % — दूसरा चक्कर।
फ़्लेयर के लिए रेंजफ़ाइंडर चाहिए: बैरोमीटर एक मीटर तक ग़लत हो जाता है।

**प्राथमिकताएँ** (`FeedbackSupervisor`): ARM नहीं > स्टॉल से सुरक्षा >
टेकऑफ़/लैंडिंग > मोड के लक्ष्य। संपर्क टूटने पर चरण रद्द हो जाते हैं, और स्थिरीकरण
failsafe ग्लाइड के लक्ष्य पूरे करता है।

### सिमुलेशन

`test/test_feedback/test_main.cpp` (PC पर: `pio test -e native -f test_feedback`) — विमान का मॉडल (स्वतंत्र अक्ष,
सर्वो की देरी और जड़ता, कंट्रोल सरफ़ेस की प्रभावशीलता ∝ V², अवमंदन ∝ V, स्थिर
आघूर्ण, गति से आक्रमण कोण के ज़रिए लिफ़्ट, स्टॉल, स्टीयरिंग
पहिये वाला लैंडिंग गियर) और 10 परिदृश्य:

| परिदृश्य | क्या जाँचा जाता है |
|---|---|
| स्थिर आघूर्ण के साथ 30° रोल / −15° पिच से बाहर निकलना | सीधा करना और “और सुधारो”: इंटीग्रल ट्रिमर ख़ुद ढूँढ लेता है |
| 14 और 20 m/s पर ±15° की हिलावट, गति सेंसर के बिना | `b` का अनुमान सच्चाई पर पहुँचता है और गति के अनुसार दोबारा गिना जाता है |
| अदला-बदली हुआ एलेरॉन, पायलट MANUAL में पंख हिलाता है | `b` का अनुमान ऋणात्मक → केवल चेतावनी, अक्ष बंद नहीं होता |
| 30 s की हवा की उथल-पुथल | झोंके सँभाल लिए जाते हैं, रोल 10° से आगे नहीं जाता |
| 20 % थ्रॉटल पर नाक 15° (गति सेंसर के साथ और बिना) | गति गिरकर स्टॉल तक नहीं पहुँचती |
| प्रोपेलर के प्रतिक्रिया आघूर्ण के साथ रनवे से टेकऑफ़ | चरण, ऊँचाई, दौड़ में हेडिंग |
| 15 m से लैंडिंग | चरण, ज़मीन के पास थ्रॉटल नहीं, नरम स्पर्श |
| टेकऑफ़ की दौड़ में संपर्क टूटना; ARM नहीं; MANUAL | रद्द करना, थ्रॉटल को नहीं छुआ जाता, कंट्रोल सरफ़ेस पायलट के पास रहती हैं |

मॉडल मोटा है — वह तर्क और चिह्नों को जाँचता है, किसी ख़ास एयरफ़्रेम के
लिए ट्यूनिंग को नहीं।

```bash
pio test -e native -f test_feedback      # PC पर, सेकंडों में
pio test -e esp32-s3 -f test_feedback    # टेस्ट फ़र्मवेयर डालकर चलाता है
pio run -t upload                        # सामान्य फ़र्मवेयर वापस डालें
```

### जोड़ने की योजना

1. `FlightController::update()` सेंसर पढ़ने और कमांड गिनने के बाद
   `FlightSnapshot` भरता है और `FeedbackSupervisor::update()` बुलाता है।
   पहले — **शैडो मोड**: आउटपुट केवल लॉग (`printStatus()`) और डैशबोर्ड पर जाता
   है, कंट्रोल सरफ़ेस पर नहीं। हाथ के नियंत्रण पर उड़ान में हर अक्ष के `b` का
   अनुमान धनात्मक होना चाहिए और गति के साथ बढ़ना चाहिए।
2. ज़मीन पर, विमान हाथों में, STABILIZE: झुकाएँ — कंट्रोल सरफ़ेस उसे संतुलित
   करती हैं।
3. एक-एक अक्ष करके: `Autopilot::getRollCorrection()` की जगह `deflectionUs`
   (पहले केवल रोल), फिर पिच।
4. थ्रॉटल: `throttleOverridePercent`/`throttleFloorPercent` —
   `Autopilot::applyThrottle()` के बाद, failsafe से पहले (failsafe सबसे ऊपर है)।
5. टेकऑफ़/लैंडिंग — किसी ख़ाली स्विच पर; `Autopilot` से मोड `AUTO_TAKEOFF` हटाएँ।
6. `FeedbackConfig` के स्थिरांक — `Config.h` में; हवाई गति का सेंसर —
   `AirspeedSensor` का क्रियान्वयन और `SensorSelection.h` में श्रेणी।

---

## नया बोर्ड कैसे जोड़ें

1. `platformio.ini` में `[env:<name>]` एक अनूठे `-D BOARD_ESP32_<NAME>` के साथ।
2. `Config.h` में `#elif defined(BOARD_ESP32_<NAME>)` का ब्लॉक सभी पिनों के साथ,
   `PIN_I2C2_SDA/SCL` समेत (OLED न हो तो −1)। GPIO का बजट पहले से गिन लें:
   flash/PSRAM/USB/strapping।
3. सर्वो आउटपुट को 5 LEDC चैनल चाहिए — सभी ESP32 में हैं। अगर रडर के लिए पिन
   नहीं है — `PIN_RUDDER = -1`, आउटपुट बस बंद हो जाएगा।
4. जब तक बोर्ड हार्डवेयर पर जाँचा न जाए, `default_envs` न बदलें; पिन-व्यवस्था
   न जाँची गई हो तो कमिट में साफ़ लिखें।

---

## बिल्ड, फ़र्मवेयर डालने और मॉनिटर के कमांड

```bash
pio run                        # डिफ़ॉल्ट बोर्ड की बिल्ड (esp32-s3)
pio run -t upload              # फ़र्मवेयर डालना
pio device monitor             # मॉनिटर, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # जाँचें कि सारे बोर्ड बनते हैं
```

- **ESP32-S3:** फ़र्मवेयर डालना और Serial कनेक्टर “COM” (CH343) से होता है।
  अगर ब्रिज अटक जाए (Windows जवाब देता है “डिवाइस ठीक से काम नहीं कर रहा” —
  ESC के व्यवधान से हो सकता है), तो केबल फिर से लगाना मदद करता है; “USB”
  कनेक्टर (बिल्ट-इन USB-JTAG) से भी डाला जा सकता है:
  `pio run -t upload --upload-port <USB COM port>`।
- जब तक पोर्ट मॉनिटर खुला है, उसी पोर्ट पर फ़र्मवेयर डालना नहीं चलेगा।
- `lib_deps`: `olikraus/U8g2` (OLED) — इकलौती बाहरी लाइब्रेरी।
- `test/` — विस्तार से [`TESTING.md`](TESTING.md) में:
  - `pio test -e native -e native-stm32` — PC पर 387 टेस्ट (हार्डवेयर के नक़ली
    `test/native/support/` में), कवरेज — `gcovr`;
  - `pio test -e esp32-s3` — बोर्ड पर ही `test_feedback/` (फ़ीडबैक का बंद-लूप
    सिमुलेशन) और `test_imu_orientation/`; हर एक टेस्ट फ़र्मवेयर डालता है, उसके
    बाद `pio run -t upload` से सामान्य फ़र्मवेयर डालें।
- स्थैतिक विश्लेषण: `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (`hal/stm32/` और `src/stm32/` पर
  cppcheck) और `tools/clang-tidy.sh`
  (प्रोफ़ाइल `.clang-tidy`)।

---

## ज्ञात सीमाएँ

- **ऑटोपायलट उड़ान में नहीं परखा गया।** मेज़ पर चिह्न सीधे जाँचे गए हैं
  (झुकाव → सीधा करने की दिशा में सुधार), PID के गुणांक शुरुआती हैं।
- **STABILIZE स्टिक के ऊपर सीधा करना है**, न कि “कोण मोड” (FBWA) जिसमें स्टिक
  रोल/पिच का कोण तय करता है। पायलट और ऑटोपायलट जुड़ते हैं।
- **संपर्क टूटने पर ग्लाइड उड़ान में नहीं परखा गया।** कोण `FAILSAFE_GLIDE_*`
  शुरुआती हैं; पिच −3° किसी ख़ास एयरफ़्रेम के हिसाब से चुनी जाती है (नाक न स्टॉल
  तक उठनी चाहिए, न गोता लगाना चाहिए)।
- **क्षितिज।** माउंटिंग कैलिब्रेशन (`o`) के साथ — उसी से (NVS); एक्सेलेरोमीटर का
  शून्य-ऑफ़सेट तापमान के साथ खिसकता है (हर 20 °C पर ~1–2°), अगर क्षितिज
  “खिसक गया” — `o` दोहराएँ। उसके बिना — चालू होते समय की स्थिति (समतल रखकर
  चालू करें)।
- **कम्पास की माउंटिंग** अब भी `MAG_ROTATION_CW_DEG` से तय होती है
  (स्थितियों वाला कैलिब्रेशन उसे नहीं छूता)।
- **कम्पास:** हेडिंग झुकाव के सुधार के बिना है, माप की दिशा तैयार विमान पर जाँची
  नहीं गई है, और कैलिब्रेशन विमान के भीतर ही करना होगा। अभी कोई मोड हेडिंग
  इस्तेमाल नहीं करता।
- **GPS** नेविगेशन के लिए इस्तेमाल नहीं होता; ESP32-C3 पर — केवल प्राप्ति।
- **फ़ीडबैक (`autopilot/feedback/`) जुड़ा नहीं है** और केवल विमान के मोटे मॉडल वाले
  सिमुलेशन में जाँचा गया है। `FeedbackConfig.h` की वे सभी संख्याएँ जिन पर
  “прикидка” (“मोटा अनुमान”) लिखा है, असली एयरफ़्रेम पर परिष्कृत करनी होंगी; हवाई गति का सेंसर अभी नहीं
  है (उसके बिना कंट्रोल सरफ़ेस की प्रभावशीलता धीमे सीखी जाती है, और स्टॉल
  केवल मंदन से दिखता है)।
- **हार्डवेयर पर नहीं जाँचे गए:** `ICM42688_Sensor` (`ImuSensorBase` के ज़रिए
  साझा नियम पर लाया गया), `BME280_Sensor` (Bosch का कंपेंसेशन नए सिरे से
  लिखा गया), SPI पर BMP388, `QMC5883L_Sensor`, CFG-VALSET से GPS की
  सेटिंग। जोड़ते समय — बूट का लॉग, कंसोल में `s`, झुकाकर चिह्न।
- **ब्रेडबोर्ड पर I2C में व्यवधान आता है** ESC/मोटर से (इक्का-दुक्का त्रुटियाँ `s`
  से दिखती हैं)। ड्राइवर उन्हें झेल लेते हैं, पर विमान में I2C के तार छोटे हों और
  पावर के तारों से दूर।
- **ESP32Servo इस्तेमाल नहीं होती।** ESP32-S3 पर संस्करण 3.2.1 सर्वो को MCPWM
  में बाँटता है और `attachPin()` में MCPWM ब्लॉक का नंबर टाइमर के नंबर से गड़बड़ा
  देता है: GPIO6/7 पर GPIO4/5 का सिग्नल निकलता था (ESC दाएँ स्टिक से
  चलता था)। आउटपुट LEDC पर दोबारा लिखे गए; लाइब्रेरी लौटानी हो तो `p` से जाँचकर
  ही।
- **ESC — 50 Hz PWM**, फ़र्मवेयर में थ्रॉटल की सीमा का कैलिब्रेशन मोड अभी नहीं है।
- **वेब डैशबोर्ड:** एक्सेस पॉइंट का पासवर्ड कमज़ोर है, और कमांड उड़ान में भी
  स्वीकार होती हैं। यह टेस्ट बेंच और मैदान का औज़ार है, उड़ान का नहीं।
- **प्रोटोटाइप की यांत्रिकी:** पहला प्रोटोटाइप उड़ा, मोटर का कमज़ोर माउंट और पंख की
  अपर्याप्त कठोरता सामने आई।
- **लाइसेंस — OpenPlane License** ([LICENSE](LICENSE.md)): MIT, जिसमें लेखक का नाम बताना
  अनिवार्य है, सैन्य उपयोग पर रोक है, और लोगों व संपत्ति को उनकी लिखित सहमति के
  बिना जानबूझकर नुक़सान पहुँचाने पर रोक है। फ़ाइलों में दूसरे लाइसेंस के हेडर न
  जोड़ें और लेखक का नाम न हटाएँ।

---

## बदलाव कैसे करें

- **छोटे कमिट:** एक तार्किक क़दम — एक कमिट।
- **कमिट से पहले टेस्ट और विश्लेषण:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh` —
  सब हरे ([`TESTING.md`](TESTING.md))।
- **सारे बोर्ड बनाएँ** साझा कोड में बदलाव के बाद — S3 मुख्य है, पर C3, 38-पिन
  और `stm32h743` टूटने नहीं चाहिए; रिलीज़ से पहले — `tools/build_matrix.sh`
  (सारे बोर्ड × सारे सेंसर)।
- **जो हार्डवेयर पर जाँचा जा सकता है, उसे जाँचें:** चिह्न — झुकाकर, आउटपुट —
  कमांड `p` से, संपर्क — ट्रांसमीटर बंद करके।
- **API गढ़ें नहीं।** फ़्रेमवर्क के स्रोत
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x) से
  मिलाएँ — इंटरनेट अक्सर 3.x संस्करण का वर्णन करता है जिसका API अलग है
  (जैसे LEDC)।
- **स्थिति को सजाएँ नहीं।** हार्डवेयर पर नहीं जाँचा — तो वैसा ही लिखें।
- **परत को उतना ही पता होना चाहिए जितना उसका हक़ है।** अगर निचली क्लास को अचानक
  ऊपरी क्लास चाहिए, तो तर्क `FlightController` में ऊपर उठना चाहिए।
- **डेटा का अनुबंध बदलते समय** (`FlightOutputState`, `ControlCommand`,
  `ImuData`, `/api/status` का JSON) — सभी उपभोक्ताओं को उसी कमिट में अपडेट करें।
