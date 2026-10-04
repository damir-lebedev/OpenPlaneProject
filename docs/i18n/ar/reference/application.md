<div dir="rtl">

# ‏APPLICATION — ‏`src/main.cpp` و`src/stm32/main.cpp`

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/application.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

نقطتا الدخول هاتان هما **composition root** للوحتيهما: الوحدة الترجمية الوحيدة
في البرنامج الثابت، والمكان الوحيد الذي تُنشأ فيه الكائنات وتُربط بالمراجع. وليس
فيهما منطق طيران، ومجموعة الكائنات واحدة؛ والاختلاف في اللوحة، والقياس عن بُعد
(Wi-Fi أو MAVLink)، وطريقة تشغيل حلقة الطيران.

## الكائنات العامة (المشتركة)

ترتيب التصريح = ترتيب الإنشاء.

| الكائن | النوع | الروابط |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice` و`imuSensor` | `SELECTED_IMU_DEVICE(board)` و`SelectedImu` | الناقل المحدد في `SensorSelection.h` |
| `baroDevice` و`baroSensor` | `SELECTED_BARO_DEVICE(board)` و`SelectedBaro` | مع أنبوب بيتو — هذا هو الضغط الساكن |
| `magDevice` و`magSensor` و`magnetometer` | … `SelectedMag` و`MagnetometerSensor* const` | فقط إذا كان `SENSOR_MAG != NONE`، وإلا `nullptr` |
| `gpsSensor` و`gpsReceiver` | `SelectedGps` و`GpsSensor* const` | فقط إذا كان `SENSOR_GPS != NONE` |
| `pitotDevice` و`pitotBaro` و`pitotSensor` و`airspeedSensor` | `SELECTED_PITOT_DEVICE(board)` و`SelectedPitotBaro` (‏`"PITOT-BMP581"`) و`PitotDualBaroAirspeed(pitotBaro, baroSensor)` | فقط إذا كان `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer` و`throttleManager` | `ControlMixer` و`ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | جميع الحساسات (يمكن أن تكون فارغة) |
| `pilotSwitches` | `PilotSwitches` | ‏`&autopilot`، وجدول `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | كل ما سبق |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | المتحكم، والطيار الآلي، والإحصاءات |
| `debugConsole` | `DebugConsole` | المتحكم، والمخارج، والطيار الآلي، والسجل، و`&board` (استطلاع النواقل `b`) |
| `oledDisplay` | `OledDisplay` | المتحكم، والطيار الآلي، والإحصاءات |
| ESP32: ‏`webDebugServer` | `WebDebugServer` | المتحكم، والطيار الآلي |
| STM32: ‏`mavlink` | `MavlinkTelemetry` | ‏`*board.telemetryUart()`، والمتحكم، والطيار الآلي، والإحصاءات |

## ‏`src/main.cpp` — ‏ESP32 (‏S3 وC3 وذات 38 طرفًا)

| الدالة | الوصف |
|---|---|
| `static void printBanner()` | شاشة الترحيب في `Serial` |
| `static void setupSensors()` | استدعاء `begin()` لكل حساس؛ ومعايرة الحساسات التي ردّت: ‏IMU ‏`calibrate()` (سكون 2 s + فحص ما قبل الطيران)، ومقياس الضغط `calibrateAltitude()`، والبوصلة — أول عيّنة بعد 25 ms تحدد اتجاه IMU (‏`setYaw`)؛ و GPS ‏`begin()`؛ وأنبوب بيتو `begin()` (الصفر — في الثانية الأولى من الحلقة)؛ و`autopilot.begin()` |
| `void setup()` | ‏`Serial.setTxBufferSize(4096)` **قبل** `begin(115200)`؛ وشاشة الترحيب؛ و`board.begin()`؛ و`flightOutputs.begin()` + `setFailsafe()`؛ و`setupSensors()`؛ و`flightController.begin()`؛ وOLED؛ وخادم الويب؛ وتخطيط المفاتيح؛ و`debugLogger.begin()` |
| `void loop()` | ‏`applyPendingCommands()` ← `flightController.update()` ← `debugLogger.update()` ← `debugConsole.update()` ← `loopStats.record()`؛ والدورة `vTaskDelayUntil(LOOP_PERIOD_MS)`؛ وعند التأخر أكثر من 100 ms — يبدأ العدّ من جديد (دون «اللحاق») |

## ‏`src/stm32/main.cpp` — ‏STM32H743

نقطة دخول env ‏`stm32h743` (وفي بناءات ESP32 يُستبعد الدليل `src/stm32/`
عبر `build_src_filter`). وعلى العتاد اختُبرت لوحة DevEBox H743 دون حساسات (الإقلاع،
والكونسول عبر USB، وبطاقة SD، والصندوق الأسود، وiBUS، والتحكم اليدوي في السيرفوات والمحرك)؛
ويجري تشغيله بكامله على الحاسوب باختبارات `test/native_stm32` (‏env ‏`native-stm32`).
وإلى جواره: `sd_msp.cpp` — أطراف SDMMC1 وساعاته، و`bootloader.cpp` — مفتاح `D` في
الكونسول (إعادة التشغيل إلى DFU).

| الدالة | الوصف |
|---|---|
| `setup()` | ‏`Serial.begin(115200)`؛ وشاشة الترحيب؛ و`board.begin()`؛ والمخارج إلى الوضع الآمن؛ و`Stm32FlashStorage::store().mount()` — صورة الإعدادات (فارغة / N بايت / تالفة — القيم الافتراضية)؛ و`setupSensors()` (كما في ESP32)؛ و`flightController.begin()`؛ و`mavlink.begin()`؛ وOLED؛ وتخطيط المفاتيح؛ و`debugLogger.begin()`؛ ومهمتا `flight` و`storage`؛ و`vTaskStartScheduler()` (لا يعود) |
| `static void flightTask(void*)` | الأولوية `Rtos::PRIORITY_FLIGHT`، والمكدّس 16 KB: ‏`flightController.update()` ← `mavlink.update()` ← `debugLogger.update()` ← `debugConsole.update()` ← `loopStats.record()`؛ و`vTaskDelayUntil(LOOP_PERIOD_MS)`، وعند التأخر أكثر من 100 ms — يبدأ العدّ من جديد |
| `static void storageTask(void*)` | في الخلفية: ‏`Stm32FlashStorage::instance().service()` مرة كل 100 ms — مسح قطاع الإعدادات وكتابته، وتزيحها مهمة الطيران |
| `loop()` | فارغة: بعد `vTaskStartScheduler()` تعمل المهام فقط |

والكونسول (`Serial`، ‏LPUART1 ‏PA9/PA10، ‏115200) هو `DebugConsole` نفسه الموجود في
ESP32: ‏`h` القائمة، و`s` الحساسات، و`b` استطلاع النواقل، و`p` المخارج، والمعايرات.

## الثوابت

- تنتقل المخارج إلى الوضع الآمن **قبل** تهيئة الحساسات
  (فمعايرة IMU تحجز الحلقة نحو 2 s).
- ‏ESP32: يُضبط مخزن TX الخاص بـ `Serial` قبل `begin()`. وSTM32: مخازن UART هي
  `SERIAL_RX/TX_BUFFER_SIZE` في `platformio.ini`.
- لا يملك أي كائن كائنًا آخر: جميع المراجع غير مالكة، وعمرها هو
  عمر البرنامج كله.
- لتغيير ما يفعله مفتاح ما — `config/Controls.h`، لا `main.cpp`.

</div>
