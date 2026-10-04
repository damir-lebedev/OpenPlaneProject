# APPLICATION — `src/main.cpp` और `src/stm32/main.cpp`

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/application.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है। यह अनुवाद AI ने किया है और मूल भाषा के जानकारों ने इसकी जाँच नहीं की है। त्रुटियाँ मिलें तो [Damir Lebedev](https://github.com/damir-lebedev) को लिखें या [इश्यू ट्रैकर](https://github.com/damir-lebedev/OpenPlaneProject/issues) में बताएँ।

[← संदर्भ](README.md)

दोनों एंट्री पॉइंट अपने-अपने बोर्ड के **composition root** हैं: फ़र्मवेयर की इकलौती
ट्रांसलेशन यूनिट और इकलौती जगह जहाँ ऑब्जेक्ट बनाए जाते हैं और रेफ़रेंस से जोड़े जाते
हैं। इनमें उड़ान का तर्क नहीं है और ऑब्जेक्ट का सेट एक जैसा है; फ़र्क़ बोर्ड, टेलीमेट्री
(Wi-Fi या MAVLink) और उड़ान-चक्र के चलने के ढंग का है।

## ग्लोबल ऑब्जेक्ट (साझा)

घोषणा का क्रम = निर्माण का क्रम।

| ऑब्जेक्ट | प्रकार | संबंध |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | `SensorSelection.h` की बस |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | पिटो ट्यूब के साथ — यह स्थैतिक दबाव है |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | केवल अगर `SENSOR_MAG != NONE`, वरना `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | केवल अगर `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | केवल अगर `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | सभी सेंसर (nullable) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, तालिका `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | ऊपर का सब कुछ |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | कंट्रोलर, ऑटोपायलट, आँकड़े |
| `debugConsole` | `DebugConsole` | कंट्रोलर, आउटपुट, ऑटोपायलट, लॉग, `&board` (बसों की पूछताछ `b`) |
| `oledDisplay` | `OledDisplay` | कंट्रोलर, ऑटोपायलट, आँकड़े |
| ESP32: `webDebugServer` | `WebDebugServer` | कंट्रोलर, ऑटोपायलट |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, कंट्रोलर, ऑटोपायलट, आँकड़े |

## `src/main.cpp` — ESP32 (S3, C3, 38-पिन)

| फ़ंक्शन | विवरण |
|---|---|
| `static void printBanner()` | `Serial` में स्वागत-संदेश |
| `static void setupSensors()` | हर सेंसर का `begin()`; जवाब देने वालों का कैलिब्रेशन: IMU `calibrate()` (2 s बिना हिले + उड़ान-पूर्व जाँच), बैरोमीटर `calibrateAltitude()`, कम्पास — 25 ms बाद का पहला सैंपल IMU की हेडिंग तय करता है (`setYaw`); GPS `begin()`; पिटो ट्यूब `begin()` (शून्य — चक्र के पहले सेकंड में); `autopilot.begin()` |
| `void setup()` | `begin(115200)` से **पहले** `Serial.setTxBufferSize(4096)`; स्वागत-संदेश; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; वेब सर्वर; स्विचों का विन्यास; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; अवधि `vTaskDelayUntil(LOOP_PERIOD_MS)`; देरी > 100 ms — गिनती फिर से शुरू (“पकड़ने” के बिना) |

## `src/stm32/main.cpp` — STM32H743

env `stm32h743` का एंट्री पॉइंट (ESP32 की बिल्ड में डायरेक्टरी `src/stm32/` को
`build_src_filter` से बाहर रखा गया है)। हार्डवेयर पर सेंसर के बिना DevEBox H743 बोर्ड
जाँचा गया है (बूट, USB पर कंसोल, SD कार्ड, ब्लैक बॉक्स, iBUS और सर्वो तथा मोटर का हाथ
से नियंत्रण); पूरा PC पर टेस्ट `test/native_stm32` (env `native-stm32`) से चलाया जाता
है। पास में: `sd_msp.cpp` — SDMMC1 के पिन और क्लॉक, `bootloader.cpp` — कंसोल की कुंजी
`D` (DFU में रीबूट)।

| फ़ंक्शन | विवरण |
|---|---|
| `setup()` | `Serial.begin(115200)`; स्वागत-संदेश; `board.begin()`; आउटपुट सुरक्षित स्थिति में; `Stm32FlashStorage::store().mount()` — सेटिंग की इमेज (ख़ाली / N बाइट / ख़राब — डिफ़ॉल्ट); `setupSensors()` (जैसे ESP32 पर); `flightController.begin()`; `mavlink.begin()`; OLED; स्विचों का विन्यास; `debugLogger.begin()`; टास्क `flight` और `storage`; `vTaskStartScheduler()` (लौटता नहीं) |
| `static void flightTask(void*)` | प्राथमिकता `Rtos::PRIORITY_FLIGHT`, स्टैक 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, देरी > 100 ms — गिनती फिर से शुरू |
| `static void storageTask(void*)` | बैकग्राउंड: हर 100 ms में एक बार `Stm32FlashStorage::instance().service()` — सेटिंग के सेक्टर को मिटाना और लिखना, उड़ान का टास्क इसे हटा देता है |
| `loop()` | ख़ाली: `vTaskStartScheduler()` के बाद केवल टास्क चलते हैं |

कंसोल (`Serial`, LPUART1 PA9/PA10, 115200) वही `DebugConsole` है जो ESP32 पर है:
`h` मेन्यू, `s` सेंसर, `b` बसों की पूछताछ, `p` आउटपुट, कैलिब्रेशन।

## अपरिवर्तनीय नियम

- आउटपुट सेंसरों के आरंभ होने से **पहले** सुरक्षित स्थिति में चले जाते हैं
  (IMU का कैलिब्रेशन चक्र को ~2 s रोके रखता है)।
- ESP32: `Serial` का TX बफ़र `begin()` से पहले तय होता है। STM32: UART के बफ़र —
  `platformio.ini` में `SERIAL_RX/TX_BUFFER_SIZE`।
- कोई ऑब्जेक्ट दूसरे का मालिक नहीं है: सारे रेफ़रेंस ग़ैर-स्वामित्व वाले हैं, जीवनकाल
  पूरा प्रोग्राम है।
- स्विच क्या करता है, यह बदलना हो तो — `config/Controls.h`, न कि `main.cpp`।
