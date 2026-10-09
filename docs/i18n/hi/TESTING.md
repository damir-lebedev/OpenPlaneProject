# TESTING.md — टेस्ट, कवरेज और स्टैटिक विश्लेषण

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../TESTING.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है। यह अनुवाद AI ने किया है और मूल भाषा के जानकारों ने इसकी जाँच नहीं की है। त्रुटियाँ मिलें तो [Damir Lebedev](https://github.com/damir-lebedev) को लिखें या [इश्यू ट्रैकर](https://github.com/damir-lebedev/OpenPlaneProject/issues) में बताएँ।

फ़र्मवेयर की जाँच दो स्तरों पर होती है:

| कहाँ | कमांड | क्या |
|---|---|---|
| **PC (native)** | `pio test -e native` | फ़र्मवेयर के हेडर PC पर बिना बदलाव के बनते हैं, हार्डवेयर की जगह नियंत्रित फ़ेक लगे होते हैं: मॉड्यूल, ड्राइवर, क्लोज़्ड-लूप उड़ान सिमुलेशन, हर सेंसर किट के साथ पूरा ESP32 फ़र्मवेयर (S3 और 38-पिन)। कवरेज गिना जाता है |
| **PC (native-stm32)** | `pio test -e native-stm32` | पूरा STM32H743 फ़र्मवेयर (`src/stm32/main.cpp`) STM32duino के फ़ेक की परत के ऊपर: FreeRTOS के टास्क, फ़्लैश, MAVLink, I2C और SPI पर सेंसर |
| **बिल्ड मैट्रिक्स** | `tools/build_matrix.sh` | 4 बोर्ड × 6 सेंसर किट, `-Wall -Wextra (-Wshadow)` के साथ; प्रोजेक्ट के कोड में कोई भी चेतावनी त्रुटि मानी जाती है |
| **बोर्ड** | `pio test -e esp32-s3` | असली ESP32-S3 पर `test_feedback` और `test_imu_orientation` (टेस्ट वाला फ़र्मवेयर फ़्लैश होता है; बाद में सामान्य वाला वापस डालें: `pio run -t upload`) |
| **STM32 बोर्ड** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | DevEBox H743 पर **असली SD कार्ड** पर ब्लैक बॉक्स, साथ में Cortex-M7 पर `test_feedback` और `test_imu_orientation` — [नीचे](#stm32-बोर्ड-पर-टेस्ट) |

आर्किटेक्चर की पृष्ठभूमि — [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-परीक्षण-योग्यता) में।

---

## त्वरित शुरुआत

```bash
pip install platformio gcovr        # एक बार
# Windows: PATH में g++ चाहिए, जैसे WinLibs (winlibs.com, zip UCRT):
# खोलकर mingw64\bin को PATH में जोड़ें — इंस्टॉल करने की ज़रूरत नहीं
pio test -e native -e native-stm32  # सभी नेटिव टेस्ट (~1.5 मिनट)
gcovr                               # फ़ाइलों के हिसाब से कवरेज (सेटिंग — gcovr.cfg)
tools/build_matrix.sh               # सभी बोर्ड × सभी सेंसर (~25 मिनट)
gcovr --html-details -o coverage/index.html   # HTML रिपोर्ट (coverage/ .gitignore में है)

pio test -e native -f native/test_rc          # एक सेट
pio test -e native -f test_feedback           # PC पर फ़ीडबैक का सिमुलेशन

# क्लोज़्ड-लूप सिमुलेशन के रास्ते CSV में (ग्राफ़ के लिए):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# MAVLink की धारा — मानक डीकोडर से जाँच के लिए (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

टेस्ट बदलने के बाद कवरेज गिनने से पहले साफ़ बिल्ड से शुरू करना ठीक रहता है: `rm -rf .pio/build/native`, वरना पिछले रनों के काउंटर रिपोर्ट में आ जाएँगे।

---

## नेटिव बिल्ड कैसे बना है

`platformio.ini` में `[env:native]`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (ESP32-S3 की पिन-व्यवस्था), `-I test/native/support`, `-Wall -Wextra -Wshadow`, `--coverage` के साथ कवरेज और `-fkeep-inline-functions -fkeep-static-functions` — इनके बिना gcov को हेडरों के वे फ़ंक्शन नहीं दिखते जिन्हें कभी बुलाया ही नहीं गया, और कवरेज बढ़ा-चढ़ाकर दिखता है।

### हार्डवेयर के फ़ेक — `test/native/support/`

हेडर, जिनके नाम और सिग्नेचर ESP32 2.0.x के Arduino कोर, ESP-IDF और लाइब्रेरियों जैसे ही हैं, पर `namespace fake` में एक सिम्युलेटेड दुनिया के ऊपर:

| फ़ाइल | किसकी जगह | सिमुलेशन क्या कर सकता है |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | Arduino कोर | मैक्रो (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, मूल जैसा `print()` का फ़ॉर्मैट। `ARDUINO` जान-बूझकर परिभाषित **नहीं** है |
| `esp32-hal-fake.h` | समय, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | घड़ी केवल `fake::advance*()`/`delay()` से चलती है; `millis()/micros()` — `uint32_t`, जैसा ESP32 पर (ओवरफ़्लो बोर्ड जैसा व्यवहार करता है)। LEDC चैनल, असली ड्यूटी साइकल के हिसाब से `pulseIn` (तभी दिखता है जब पिन का इनपुट बफ़र चालू हो), `analogReadMilliVolts` — वोल्टेज `fake::gpio().analogMv` से। टास्क दर्ज होते हैं (हैंडल शून्य नहीं); `fake::runTask(task, n)` उसके अनंत लूप के n चक्कर चलाता है, `ulTaskNotifyTake` एक चक्कर गिना जाता है, `xTaskNotifyGive` — एक काउंटर। FreeRTOS के म्यूटेक्स — “व्यस्त” का झंडा। `psramFound()`/`ps_malloc()`। क्रिटिकल सेक्शन गिने जाते हैं |
| `HardwareSerial.h` | UART | पोर्ट नंबर से दर्ज होते हैं (`fake::uart(1)`); `pushRx()`, `txBytes()`, चलते-चलते गति बदलना (`updateBaudRate`, इतिहास — `baudChanges()`)। `Serial` = UART0 |
| `esp_partition.h` | ESP-IDF के फ़्लैश पार्टिशन | पार्टिशन NOR जैसे व्यवहार वाला बाइटों का वेक्टर है: मिटाना केवल 4 KB के सेक्टरों में, मिटा हुआ = 0xFF, लिखाई केवल बिट गिराती है (बिट चढ़ाने की कोशिश गिनी जाती है — `bitRaises`); `beforeWrite` — “बिजली चली गई”; पढ़ने, लिखने और मिटाने के काउंटर |
| `esp_system.h` | रीबूट का कारण | `fake::chip().resetReason` से `esp_reset_reason()` |
| `Wire.h` | I2C | पते के हिसाब से उपकरण; `fake::RegisterMapDevice` — ऑटो-इन्क्रीमेंट वाले रजिस्टर, लिखाई का लॉग, ख़राबियाँ (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), हुक `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | CS पिन के हिसाब से उपकरण; `fake::SpiRegisterMapDevice` — Bosch/InvenSense प्रोटोकॉल, डेटा से पहले `dummyBytes` |
| `Preferences.h` | NVS | मेमोरी में भंडार, `begin(readOnly)`/`get*`/`getBytes` का व्यवहार मूल जैसा; `failBegin` |
| `WiFi.h`, `WebServer.h` | Wi-Fi AP, HTTP | `softAP()` का नतीजा टेस्ट तय करता है; `WebServer::request(मेथड, uri, बॉडी)` दर्ज हैंडलर को बुलाता है; `fake::webServers()` — सभी इंस्टेंस |
| `U8g2lib.h` | U8g2 | पिक्सेल की जगह — बनाई गई पंक्तियों और आयतों की सूची; `begin()/sendBuffer()` बाइटों को उपयोगकर्ता के बाइट-कॉलबैक से गुज़ारते हैं; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### STM32duino की परत — `test/native/support_stm32/` (परिवेश `native-stm32`)

यह `-I` में `support/` से पहले आती है और उन्हीं फ़ेक में वह जोड़ती है जो केवल STM32duino में है। इस परिवेश में `<Preferences.h>` — **असली** `include/hal/stm32/compat/Preferences.h` है, जो `KeyValueStore` के ऊपर बना है।

| फ़ाइल | किसकी जगह | क्या कर सकती है |
|---|---|---|
| `Arduino.h` | STM32duino कोर | पिन `PA0..PE15` (पोर्ट·16 + नंबर), `pin_size_t`, सर्वो-आउटपुट पिनों के लिए `PinMap_TIM`, `HardwareTimer` (पल्स `fake::timerPulseUs(pin)` और `pulseIn()` से दिखता है), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino का FreeRTOS | `xTaskCreate` साझा टास्क-रजिस्ट्री में (स्टैक शब्दों में), `vTaskStartScheduler()` लौट आता है — टास्क टेस्ट खुद चलाता है (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM का अनुकरण | 8 KB की “फ़्लैश” (मिटी हुई = 0xFF) और एक बफ़र, `fake::eeprom()` — काउंटर और इमेज का बिगड़ना |
| `SPI.h` | | `SPIMode` |

STM32 के लिए साझा फ़ेक में जोड़ा गया: `TwoWire(sda, scl)`, `setSDA/SCL` और `fake::wireWithSda(pin)` (बोर्ड की दूसरी बस ढूँढने के लिए), `HardwareSerial(rx, tx)` और `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`।

### चिप एमुलेटर और हवाई जहाज़ का मॉडल — `test/native/helpers/`

| फ़ाइल | यह क्या है |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (IPREG के अप्रत्यक्ष रजिस्टरों के साथ), QMC6309, SPL06-001, BMP581, u-blox के NAV-PVT फ़्रेम — I2C या SPI पर रजिस्टर-मानचित्र, `World` “दुनिया” (कोण और गतियाँ, ऊँचाई, एयरस्पीड, दिशा, निर्देशांक) से डेटा, चिप की अपनी धुरियों में, `IMU_ROTATION_CW_DEG` का ध्यान रखते हुए |
| `PlaneSim.h` | ~1.2 kg के हवाई जहाज़ का मॉडल: बिंदु-द्रव्यमान + रोल/पिच में घूर्णन, स्टॉल वाला CL(α), ड्रैग, थ्रस्ट, हवा, थर्मल, ज़मीन |
| `SimHarness.h` | क्लोज़्ड लूप: रिमोट → iBUS फ़्रेम → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → सरफ़ेस के विचलन → `PlaneSim` → सेंसर (दो शोर वाले बैरोमीटर पर पिटो ट्यूब समेत)। `OPENPLANE_SIM_DIR` होने पर CSV रास्ता |

`test/native/helpers/TestSupport.h` — सेटों में साझा चीज़ें: `resetWorld()` (`setUp()` से बुलाया जाता है), `FakeUart`/`FakeServo`/`FakeBoard` और सेंसरों (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`) के प्रतिरूप, फ़्रेम बनाने वाला `ibusFrame()`, स्टैंड `I2cRig`/`SpiRig` (असली `Esp32I2CBus`/`Esp32SpiBus` और सिम्युलेटेड चिप वाले `*RegisterDevice` के ऊपर ड्राइवर)।

बोर्ड वाले टेस्ट (`test_feedback`, `test_imu_orientation`) पोर्टेबल हैं: `ARDUINO` हो तो — `setup()/loop()`, वरना `main()`। `test/native/*` के सेट बोर्ड के लिए नहीं बनते (`[esp32_common]` और `[env:stm32h743]` में `test_ignore`: पैटर्न हर एक अपनी पंक्ति में — स्पेस से अलग करने पर PlatformIO उन्हें एक मानकर पढ़ता है)। STM32 पर: `pio test -e stm32h743`।

---

## टेस्ट सेट

| सेट | टेस्ट | क्या जाँचता है |
|---|---|---|
| `native/test_hal` | 17 | `II2CBus` के सहायक (NACK, छोटी रीडिंग — बफ़र को छुआ नहीं जाता), `I2cRegisterDevice`, `SpiRegisterDevice` (रीड बिट, BMP388 का डमी बाइट), `Esp32I2CBus` (5 ms टाइमआउट), `Esp32SpiBus` (मोड 0–3), `Esp32UartPort` (8N1, पिन), `Esp32ServoOutput` (50 Hz/14 बिट, पल्स की सीमा, LEDC की विफलता, इनपुट बफ़र से माप), `Esp32Board` (बस, UART, चैनलों का क्रम, AUX, बजर) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, iBUS पार्सिंग: टुकड़ों में आने वाले फ़्रेम, CRC, 12-बिट मान, ट्रांसमीटर का फ़ेलसेफ़, 500 ms टाइमआउट (`micros()` के ओवरफ़्लो के दौरान भी), कचरा डेटा, री-सिंक्रनाइज़ेशन |
| `native/test_control` | 21 | फ़्लैप (गति, पहली कॉल, ठहराव), मिक्सर (चिह्न, रिवर्स, फ़्लैपरॉन), थ्रॉटल, ARM स्टेट मशीन और मोड के सेंसरों की जाँच, आउटपुट टेबल और पल्स की स्व-जाँच |
| `native/test_autopilot` | 22 | PID (सेंसर की गति से D, इंटीग्रल, anti-windup, `dt`), कोण-मोड के रूप में STABILIZE, समय के आधार पर ऑटो-टेकऑफ़, एलिवेटर से ALT_HOLD, सिग्नल खो जाने पर ग्लाइड |
| `native/test_autopilot_modes` | 31 | सभी 12 मोड और सेंसर न होने पर हर मोड की प्रतिक्रिया, बाइंडिंग तालिका और `static_assert`, फ़ंक्शन और नॉब, नेविगेशन (हेडिंग, घेरा, होम, जियोफ़ेंस), failsafe RTH/ग्लाइड, हाथ से लॉन्च, सोअरिंग, ऑटो-ट्रिम (केवल ज़मीन पर लिखा जाता है) |
| `native/test_flight_controller` | 12 | असली क्लासों पर `FlightController` का पूरा चक्र: प्राथमिकताएँ सिग्नल खो जाना > ARM > स्टिक/ऑटोपायलट > थ्रॉटल; AUX, `MOTOR_KILL`, बजर |
| `native/test_imu` | 21 | MPU6050/6500/9250 और ICM-42688: पहचान, रजिस्टर, स्केल, अक्षों का घुमाव और विमानन के चिह्न, बस की त्रुटियाँ, जाइरोस्कोप का कैलिब्रेशन और उड़ान-पूर्व जाँच, तीन स्थितियों से माउंटिंग का कैलिब्रेशन, NVS, ओरिएंटेशन फ़िल्टर |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, I2C और SPI पर BMP388, Bosch के संदर्भ के अनुसार BME280/BMP280, कम्पास (हेडिंग, NVS में hard-iron कैलिब्रेशन), u-blox M10 (CFG-VALSET, NAV-PVT, ख़राब फ़्रेम, टाइमआउट), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, वैकल्पिक पता, SPI), ICM-45686 (अप्रत्यक्ष रजिस्टर), QMC6309, SPL06-001 (डेटाशीट के सूत्र), BMP581 (DRDY और फ़ॉलबैक रास्ता), पिटो ट्यूब (ज़ीरो, फ़िल्टर, घनत्व, आपस में बदली हुई नलियाँ, पुराना डेटा, दो बैरोमीटरों के शोर के साथ “उड़ान”) |
| `native/test_storage` | 16 | `KeyValueStore` (रीलोड, घिसाव — एक जैसा मान दोबारा नहीं लिखा जाता, डेटा गँवाए बिना ओवरफ़्लो, CRC, मिटाते समय बिजली जाना, कचरा डेटा, फ़ॉर्मैट का संस्करण), `KvPreferences` (व्यवहार ESP32 के NVS जैसा) |
| `native/test_mavlink` | 20 | pymavlink के संदर्भ फ़्रेमों से मिलान करता कोडेक (v1, v2, हस्ताक्षरित), CRC, री-सिंक्रनाइज़ेशन; टेलीमेट्री: धाराओं की आवृत्तियाँ, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, PID पैरामीटर (सूची, पढ़ना, लिखना, ग़लत मानों का इनकार), ज़मीन से मोड बदलना, ज़मीन से ARM — अस्वीकार, मिशन — 0, भरा हुआ UART बफ़र लूप को नहीं रोकता |
| `native/test_blackbox` | 19 | ब्लैक बॉक्स: फ़ॉर्मैट और CRC, NOR फ़ेक पर सेक्टरों की रिंग (बिना मिटाए ताज़ा पार्टिशन, कचरा हमेशा मिटाया जाता है, पुरानी उड़ानें — पूरी की पूरी और केवल जगह के लिए, आख़िरी उड़ान को छुआ नहीं जाता, रिंग के सिरे को पार करना, रीबूट के बाद हेड, बिजली गुल होना, CRC से पकड़ी गई अधूरी लिखी एंट्री), असली `FlightController`/`Autopilot` पर उड़ान की रिकॉर्डिंग: ARM और थ्रॉटल पर शुरू (पूर्व-रिकॉर्डिंग के साथ), DISARM और “ज़मीन पर खड़े” होने के बाद रुकना, सिग्नल खोने पर नहीं रुकती, गड़बड़ी वाले रीबूट के बाद रिकॉर्डिंग, हाथ से शुरू करना, इवेंट, बैटरी, हवा में फ़्लैश ख़त्म, पार्टिशन से लंबी उड़ान, CRC वाले फ़्रेमों में और गति बदलकर डाउनलोड, कंसोल मेनू `k`, पार्टिशन नहीं — बंद |
| `native/test_blackbox_scan` | 3 | चालू होते समय रिंग की नमूना-आधारित जाँच बनाम पूरी जाँच: रिंग के 300 यादृच्छिक इतिहास × जाँच के 5 चरण (हेड, क्रम-संख्याएँ और उड़ानों की सूची मेल खाती हैं, और जब तस्वीर मेल न खाए — तो पूरी जाँच पर लौट जाती है) और 64 MB के SD क्षेत्र पर लागत (32,000 के बजाय ≈530 रीड) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, संस्करण), `DebugLogger` (सभी चैनल, NAV), `DebugConsole` (मेनू, हॉटकी, बसों की पड़ताल `b`, ARM के दौरान मनाही, सहेजना केवल ARM के बिना), `WebDebugServer` (रूट, JSON, मेलबॉक्स), `OledDisplay` (I2C पर बाइट, फ़्रेम, सिग्नल खोने पर इनवर्ज़न) |
| `native/test_sim` | 15 | विमान के मॉडल के साथ पूरे फ़र्मवेयर की क्लोज़्ड-लूप उड़ानें: झुकाव से बाहर निकलना, पार्श्व हवा में CRUISE, LOITER, RTH, failsafe RTH/ग्लाइड, जियोफ़ेंस, रनवे से ऑटो-टेकऑफ़, हाथ से लॉन्च, ऑटो-लैंडिंग, थर्मल, सर्पिल से RESCUE, गति बनाए रखना और स्टॉल से सुरक्षा, लूप में असली पिटो ट्यूब, “टेढ़े” विमान का ऑटो-ट्रिम, उड़ान में सेंसरों की विफलता (IMU, बैरोमीटर, पिटो ट्यूब, GPS) |
| `native/test_feedback_units` | 14 | फ़ीडबैक मॉड्यूल अलग-अलग: गति के स्रोत, हवा में/ज़मीन पर, RLS अनुमान, नियंत्रक, स्टॉल के संकेत, टेकऑफ़/लैंडिंग का रद्द होना |
| `native/test_app` | 10 | ESP32-S3 पर `src/main.cpp`, बेंच सेट MPU6500/BMP581/QMC5883P/OLED के साथ: `loop()` की अवधि, ट्रांसमीटर → सर्वो, ARM, मोड, सिग्नल खोना, कंसोल, डैशबोर्ड, स्क्रीन, ब्लैक बॉक्स (कोर 0 पर टास्क, थ्रॉटल पर रिकॉर्डिंग, DISARM के बाद उड़ान, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | ESP32-S3 पर `src/main.cpp`, उड़ान सेट के साथ: LSM6DSV + QMC6309 + SPL06 + ट्यूब में BMP581 + GPS — सभी चिप की पहचान, ट्यूब का ज़ीरो और गति, ऊँचाई, GPS से होम, चिप के कोणों से STABILIZE, होम तक RTH, बसों की पड़ताल, डैशबोर्ड |
| `native/test_app_icm45686_esp32dev` | 5 | **ESP32 38-pin** (`BOARD_ESP32_CLASSIC`) पर `src/main.cpp`, ICM-45686 + QMC6309 + SPL06 + BMP581 सेट के साथ: बोर्ड की पिन-व्यवस्था, IPREG फ़िल्टर, हाथ से लॉन्च, स्थिरीकरण और गति, एक बस की पड़ताल |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | **STM32H743** पर `src/stm32/main.cpp`, उड़ान सेट के साथ: टास्क और प्राथमिकताएँ, 2 ms की अवधि, ट्यूब, PWM टाइमर और `pulseIn`, उड़ान में MAVLink, GCS से मोड बदलना, बैकग्राउंड टास्क द्वारा सेटिंग को “फ़्लैश” में लिखना, I2C1 पर स्क्रीन, कंसोल |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743, **SPI पर** ICM-45686 और BMP581 + QMC6309 के साथ: चालू करते समय ख़राब फ़्लैश, GCS से ALT_HOLD ऊँचाई बनाए रखता है, सिग्नल खोना → RTH, MAVLink में दिखता है; ख़राब इमेज को दोबारा लिखना |
| `native_stm32/test_blackbox_sd` | 29 | SD कार्ड पर ब्लैक बॉक्स: FAT32 (MBR के साथ और बिना, दो क्लस्टर में फैली डायरेक्टरी, शोर वाली एंट्री, पराया/बिखरा हुआ/ख़ाली वॉल्यूम), `SdFileRegion` (अधूरे ब्लॉक, कैश, मिटाना, सीमाएँ, विफलताएँ), नक़ली `HAL_SD` के ऊपर असली `Stm32SdCard` ड्राइवर (4 बिट, फ़ॉलबैक गतियाँ, पुनः प्रयास, व्यस्त कार्ड, असंरेखित (unaligned) बफ़र), कार्ड पर रिंग (रीबूट, बिजली गुल होना, जाँच की लागत), “रिंग ख़ाली” का निशान, `FlightController` पर उड़ान की रिकॉर्डिंग, `RCC->RSR` से पहचाना गया गड़बड़ी वाला रीबूट, बैटरी का ADC, उड़ान में कार्ड की त्रुटियाँ, धीमा कार्ड, कंसोल से डाउनलोड, `D` कुंजी |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | कार्ड के साथ `src/stm32/main.cpp`: बूट पर कार्ड और फ़ाइल मिल जाती है, `bbox` टास्क उड़ान को लिखता है, लूप की अवधि नहीं खिंचती, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | बिना कार्ड के `src/stm32/main.cpp`: ब्लैक बॉक्स बंद रहता है और कारण बताता है, विमान उड़ता है, मेनू `k` नहीं टूटता |
| `test_feedback` | 10 | फ़ीडबैक लूप वाले विमान का क्लोज़्ड-लूप सिमुलेशन (PC पर और बोर्ड पर) |
| `test_imu_orientation` | 5 | 300 यादृच्छिक माउंटिंग पर IMU की माउंटिंग का कैलिब्रेशन (PC पर और बोर्ड पर) |
| **कुल** | **387** | `native` में 340 + `native-stm32` में 47 (साथ में 9 केवल बोर्ड पर — `test_blackbox_sd`) |

### STM32 बोर्ड पर टेस्ट

`test/test_blackbox_sd` नेटिव नहीं है: SDMMC ड्राइवर, कार्ड और समय असली हैं। टेस्ट एक FreeRTOS टास्क में चलते हैं, और उनके साथ सबसे ऊँची प्राथमिकता वाला उड़ान-चक्र की नकल करने वाला टास्क चलता है (अवधि 2 ms): वह कार्ड तक पहुँच के बीच में टेस्ट को रोक देता है (प्रीएम्प्ट करता है), ठीक वैसे ही जैसे फ़र्मवेयर में होता है। इसके बिना वह त्रुटि पकड़ में नहीं आती, जो बोर्ड पर मिली थी: प्रीएम्प्शन के समय SDMMC का FIFO ओवरफ़्लो हो जाता था (`HAL_SD_ERROR_RX_OVERRUN`), जबकि सादे लूप में ऐसा नहीं होता।

| टेस्ट | क्या जाँचता है |
|---|---|
| `reset_cause_is_a_normal_one` | रीसेट का कारण (`RCC->RSR`) न वॉचडॉग है, न वोल्टेज गिरना |
| `card_is_detected_on_four_bit_bus` | कार्ड 4-बिट बस पर और 24 MHz पर पहचाना गया |
| `file_is_found_and_contiguous` | FAT32 पर `BLACKBOX.BIN` मिल गई और वह लगातार (contiguous) पड़ी है |
| `multi_block_writes_work_at_every_length` | एक ही पहुँच में 1, 2, 4, 8 ब्लॉक लिखना |
| `pages_write_with_bounded_latency_and_read_back_intact` | 256 B के पेज: सबसे बुरी लिखाई < 250 ms (SD की सीमा), स्थिर रूप से > 40 KB/s, पढ़ना और मिटाना |
| `header_scan_cost_on_the_whole_area` | सेक्टर का हेडर पढ़ने और पूरी जाँच की लागत |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | सब कुछ मिटाना, 20,000 एंट्री की दो उड़ानें, “रीबूट”: नमूना-आधारित जाँच < 2 s, एंट्री क्रम से और सही CRC के साथ पढ़ी जाती हैं; ख़ाली रिंग निशान से < 100 ms में पहचानी जाती है |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | असली `BlackBox` 500 Hz IMU पर रियल टाइम में: एक भी एंट्री नहीं खोई, “रीबूट” के बाद उड़ान पढ़ी जा सकती है |
| `the_flight_task_was_not_disturbed` | कार्ड पर लिखने से नकल करने वाले टास्क की अवधि नहीं बिगड़ी (विचलन < 3 ms) |

चलाना (फ़ाइल वाला कार्ड — `python tools/blackbox.py sd-prepare E:`; **टेस्ट फ़ाइल की सारी उड़ानें मिटा देता है**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

बोर्ड DFU मोड में होना चाहिए (DevEBox पर — BT0→3V3 का तार और RST, Zadig के ज़रिए WinUSB ड्राइवर; विस्तार से — [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743))। STM32 का कंसोल USB CDC है: फ़र्मवेयर डालने के बाद पोर्ट तुरंत नहीं दिखता, और `pio test` कभी-कभी समय पर उसे खोल नहीं पाता (“could not open port”) — तब `pio test ... --without-testing` चलाएँ और आउटपुट किसी भी टर्मिनल प्रोग्राम में पढ़ें, जिसमें DTR चालू हो (टेस्ट पोर्ट खुलने तक 60 s तक इंतज़ार करते हैं)। टेस्ट के बाद बोर्ड **`D`** कुंजी की प्रतीक्षा करता है — वह तार के बिना उसे DFU में रीबूट कर देती है।

DevEBox H743 + 16 GB कार्ड पर नतीजे (2026-10-02): `test_blackbox_sd` — 9/9, `test_feedback` — 10/10, `test_imu_orientation` — 5/5; कार्ड की गति के आँकड़े [BLACKBOX.md](BLACKBOX.md#बोर्ड-पर-क्या-नापा-गया) में हैं।

### बेंच फ़र्मवेयर — `test/bench/`

ये टेस्ट के सेट नहीं हैं, बल्कि अलग-अलग छोटे PlatformIO प्रोजेक्ट हैं, जिन्हें उड़ान वाले फ़र्मवेयर की जगह बोर्ड पर डाला जाता है (`pio test` उन्हें नहीं देखता: फ़ोल्डरों के नाम `test_` से शुरू नहीं होते)। पिन और सीमाएँ साझा `Config.h` से ली जाती हैं।

| प्रोजेक्ट | क्या करता है |
|---|---|
| `bench/elevator_sweep` | `ControlMixer` और `FlightOutputs` के ज़रिए प्रोग्राम से एलिवेटर स्टिक (CH2) को हिलाता है, जैसे असली स्टिक: ऊपर यात्रा का 100%, नीचे 60%, धीरे-धीरे, ठहराव के साथ; 20 s काम — 20 s न्यूट्रल पर। चरम स्थितियों में आउटपुट पर पल्स मापता है। थ्रॉटल न्यूनतम पर |

डालने के लिए: `pio run -d test/bench/elevator_sweep -t upload`। उड़ान वाला फ़र्मवेयर वापस लाने के लिए: `pio run -e esp32-s3 -t upload`।

---

## कवरेज

इसे `gcovr` गिनता है — `include/` और `src/` पर (वह सब जो फ़र्मवेयर में जाता है), दोनों नेटिव परिवेशों को मिलाकर: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`।

| परत | पंक्तियाँ | शाखाएँ |
|---|---|---|
| `autopilot` | 920/943 (97.6%) | 645/731 (88.2%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 252/256 (98.4%) | 171/189 (90.5%) |
| `hal` | 98/102 (96.1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99.0%) | 21/22 (95.5%) |
| `hal/stm32` | 149/158 (94.3%) | 35/52 (67.3%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors` (सभी) | 1444/1446 (99.9%) | 716/835 (85.7%) |
| `storage` | 220/220 (100%) | 158/178 (88.8%) |
| `telemetry` | 1413/1440 (98.1%) | 1123/1269 (88.5%) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95.9%) | 20/29 (69.0%) |
| **कुल** | **5490/5584 (98.3%)** | **3457/3943 (87.7%)**; फ़ंक्शन 877/902 (97.2%) |

जो हिस्सा अब भी कवर नहीं है और क्यों:

- **हाथ से लॉन्च** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) — `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` होने पर पहुँच से बाहर; जब यह स्थिरांक कॉन्फ़िगर किया जा सकेगा (`Config.h` में जाने पर), तब टेस्ट में आ जाएगा।
- **बोर्ड पर निर्भर हिस्से:** बिना पिन का आउटपुट (`PIN_RUDDER = -1` केवल C3 पर होता है), बिना TX पिन का GPS (C3) — नेटिव टेस्ट S3, 38-pin और STM32 की पिन-व्यवस्था चलाते हैं, पर C3 की नहीं (C3 की जाँच बिल्ड मैट्रिक्स करता है)।
- **STM32:** कोर की त्रुटि वाली शाखाएँ (पिन पर टाइमर नहीं है, टाइमरों का पूल ख़त्म हो गया), संदेश `FreeRTOS не запустился` (“FreeRTOS शुरू नहीं हुआ”) — PC पर `vTaskStartScheduler()` हमेशा लौट आता है।
- **सुरक्षात्मक शाखाएँ**, जहाँ सार्वजनिक API के ज़रिए पहुँचा नहीं जा सकता: एनम पर `switch` में `default`/`Count`, `return "?"`।
- जिन फ़ाइलों में चलने वाली पंक्तियाँ नहीं हैं (`Config.h`, `Channels.h`, `FeedbackConfig.h`, स्ट्रक्चर `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, `SensorSelection.h` के मैक्रो, डैशबोर्ड का HTML) वे रिपोर्ट में नहीं आतीं — वे टेस्ट में कंपाइल होती हैं, पर gcov के पास उनमें गिनने को कुछ नहीं है।

---

## स्टैटिक विश्लेषण

| टूल | कमांड | प्रोफ़ाइल |
|---|---|---|
| GCC | `tools/build_matrix.sh` (या `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | सभी बोर्ड × सभी सेंसर सेट। नेटिव टेस्ट बिल्ड हमेशा `-Wall -Wextra -Wshadow` के साथ; `stm32h743` — `-Wall -Wextra` के साथ (`build_src_flags`; `-Wshadow` खुद STM32duino के हेडरों पर शोर करता है) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `[esp32_common]` में `check_*`: `include/` और `src/` (`stm32/` को छोड़कर), warning/style/performance/portability, इनलाइन `// cppcheck-suppress` केवल झूठे अलार्म के लिए (U8g2 का कॉलबैक, `setup/loop`)। `stm32h743` के लिए — `include/hal/stm32/` और `src/stm32/` पर वही फ़्लैग |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` आदि; बंद की गई जाँचों की व्याख्या फ़ाइल में ही है |

clang-tidy `test/native/support` के फ़ेक के साथ चलता है: clang होस्ट आर्किटेक्चर के लिए ESP-IDF के हेडर पढ़ नहीं सकता (`pio check` को `clangtidy` के साथ चलाने की कोशिश करें, तो विश्लेषण पार्सिंग त्रुटियों पर रुक जाता है और ईमानदारी से कुछ भी नहीं जाँचता)। `misc-include-cleaner` देखता है कि हर हेडर वही शामिल करे जिसका वह इस्तेमाल करता है: “छतरी” वाले हेडर (`FeedbackModules.h`, `IBoard.h`/`RegisterDevice.h` का API, `SensorSelection.h` के मैक्रो) पर `// IWYU pragma: export` लगा है। स्क्रिप्ट STM32 का कोड (`include/hal/stm32/`, `src/stm32/`) छोड़ देती है — उसकी जाँच बिल्ड, `stm32h743` परिवेश का cppcheck और `native-stm32` परिवेश के टेस्ट करते हैं।

आख़िरी बार चलाने के समय बिल्ड मैट्रिक्स — **24/24 बिना चेतावनी के**:

| बोर्ड | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) — प्रोजेक्ट के कोड पर 0 टिप्पणियाँ।

---

## नए टेस्ट कैसे लिखें

1. हार्डवेयर के बिना लॉजिक वाला मॉड्यूल — सीधा यूनिट टेस्ट: समय पैरामीटर से दिया जाता है या `fake::advanceMs()` से आगे बढ़ाया जाता है।
2. चिप का ड्राइवर — `I2cRig`/`SpiRig` के ज़रिए: सिम्युलेटेड चिप के रजिस्टर, लिखे गए मानों की जाँच (`chip.lastWrite(reg)`) और डेटा के पार्सिंग की जाँच। सूत्रों के लिए डेटाशीट का संदर्भ या स्वतंत्र गणना लें, कोड की नक़ल नहीं।
3. FreeRTOS के अनंत टास्क वाली क्लासें — `fake::findTask("नाम")` + `fake::runTask(task, n)`; STM32 का उड़ान टास्क भी इसी तरह चलाया जाता है।
4. दूसरे सेंसर सेट या दूसरे बोर्ड के साथ पूरा फ़र्मवेयर — अलग सेट, जो `#include "../../../src/main.cpp"` से पहले `SENSOR_KIT` तय करता है (या `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`); चिप — `helpers/ChipEmulators.h` से। STM32 के लिए — `test/native_stm32/`।
5. ऑटोपायलट का नया मोड — `test_sim` में क्लोज़्ड-लूप उड़ान का परिदृश्य।
6. नया सेट — फ़ोल्डर `test/native/test_<नाम>/test_main.cpp`, जिसमें `main()` हो; अगर सेट को टेस्टों के बीच स्थिति नहीं चाहिए, तो `setUp()` में `resetWorld()` बुलाएँ।
7. बग मिला — पहले ऐसा टेस्ट लिखें जो उसे पकड़े, फिर सुधार करें।
