<div dir="rtl">

# ‏TESTING.md — الاختبارات والتغطية والتحليل الساكن

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../TESTING.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي.

يُفحص البرنامج الثابت على مستويين:

| أين | الأمر | ماذا |
|---|---|---|
| **الحاسوب (native)** | `pio test -e native` | تُبنى ترويسات البرنامج الثابت على الحاسوب دون تعديل، ويُستعاض عن العتاد ببدائل محاكية (fakes) يمكن التحكم فيها: الوحدات، والمشغّلات، ومحاكاة طيران بحلقة مغلقة، وبرنامج ESP32 الثابت كاملًا (S3 وذات 38 طرفًا) مع كل مجموعة حساسات. وتُحسب التغطية |
| **الحاسوب (native-stm32)** | `pio test -e native-stm32` | برنامج STM32H743 الثابت كاملًا (`src/stm32/main.cpp`) فوق طبقة من بدائل STM32duino المحاكية: مهام FreeRTOS، والفلاش، وMAVLink، والحساسات على I2C وSPI |
| **مصفوفة البناء** | `tools/build_matrix.sh` | 4 لوحات × 6 مجموعات حساسات بالخيارات `-Wall -Wextra (-Wshadow)`؛ وأي تحذير في شيفرة المشروع يُعدّ خطأ |
| **اللوحة** | `pio test -e esp32-s3` | `test_feedback` و`test_imu_orientation` على ESP32-S3 حقيقية (يرفع برنامجًا ثابتًا للاختبار؛ ثم أعد البرنامج المعتاد: `pio run -t upload`) |
| **لوحة STM32** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | الصندوق الأسود على **بطاقة SD حقيقية** في DevEBox H743، إضافةً إلى `test_feedback` و`test_imu_orientation` على Cortex-M7 — [أدناه](#الاختبارات-على-لوحة-stm32) |

والسياق المعماري في [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-قابلية-الاختبار).

---

## بداية سريعة

<div dir="ltr">

```bash
pip install platformio gcovr        # مرة واحدة
# ويندوز: يلزم g++ في PATH، مثل WinLibs (winlibs.com، الحزمة zip UCRT):
# فُكّ الضغط وأضف mingw64\bin إلى PATH — لا حاجة إلى التثبيت
pio test -e native -e native-stm32  # كل الاختبارات الأصلية (~1.5 دقيقة)
gcovr                               # التغطية بحسب الملفات (الإعدادات — gcovr.cfg)
tools/build_matrix.sh               # كل اللوحات × كل الحساسات (~25 دقيقة)
gcovr --html-details -o coverage/index.html   # تقرير HTML (المجلد coverage/ في .gitignore)

pio test -e native -f native/test_rc          # مجموعة واحدة
pio test -e native -f test_feedback           # محاكاة التغذية الراجعة على الحاسوب

# مسارات المحاكاة بحلقة مغلقة في CSV (للرسوم البيانية):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# تدفق MAVLink — للفحص بمفكّك مرجعي (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

</div>

قبل حساب التغطية بعد تعديل الاختبارات يحسن البدء من بناء نظيف: `rm -rf .pio/build/native`، وإلا دخلت في التقرير عدّادات التشغيلات السابقة.

---

## كيف يُبنى البناء الأصلي

البيئة `[env:native]` في `platformio.ini`: `platform = native`، وUnity، و`-std=gnu++17`، و`-D BOARD_ESP32_S3` (توزيع أطراف اللوحة الرئيسية)، و`-I test/native/support`، و`-Wall -Wextra -Wshadow`، والتغطية بـ `--coverage` مع `-fkeep-inline-functions -fkeep-static-functions` — فبدونهما لا يرى gcov دوال الترويسات التي لم تُستدعَ قط ويبالغ في التغطية.

### بدائل العتاد المحاكية — `test/native/support/`

ترويسات بالأسماء والتواقيع نفسها كما في نواة Arduino لـ ESP32 2.0.x وESP-IDF والمكتبات، لكن فوق عالم محاكى في `namespace fake`:

| الملف | يحلّ محلّ | ما تستطيعه المحاكاة |
|---|---|---|
| `Arduino.h` و`Print.h` و`WString.h` و`Stream.h` | نواة Arduino | وحدات ماكرو (`constrain` و`sq` و`DEG_TO_RAD`…)، و`map()`، و`String`، وتنسيق `print()` كما في الأصل. و`ARDUINO` **غير** معرَّف عمدًا |
| `esp32-hal-fake.h` | الزمن وGPIO وADC وLEDC وFreeRTOS وPSRAM و`ESP` | لا تتقدم الساعة إلا بـ `fake::advance*()`/`delay()`؛ و`millis()/micros()` من النوع `uint32_t` كما في ESP32 (يتصرف الفيضان كما على اللوحة). قنوات LEDC، و`pulseIn` بحسب دورة التشغيل الحقيقية (ظاهرة فقط إذا كان مخزن الدخل للطرف مفعَّلًا)، و`analogReadMilliVolts` — الجهد من `fake::gpio().analogMv`. تُسجَّل المهام (المقبض غير فارغ)؛ ويُنفّذ `fake::runTask(task, n)` عددًا n من دورات حلقتها اللانهائية، ويُحتسب `ulTaskNotifyTake` دورةً، ويُحتسب `xTaskNotifyGive` عدّادًا. وأقفال FreeRTOS (mutex) راية «مشغول». و`psramFound()`/`ps_malloc()`. وتُحصى المقاطع الحرجة |
| `HardwareSerial.h` | UART | تُسجَّل المنافذ بالرقم (`fake::uart(1)`)؛ و`pushRx()` و`txBytes()` وتغيير السرعة أثناء العمل (`updateBaudRate`، والسجل هو `baudChanges()`). و`Serial` = UART0 |
| `esp_partition.h` | أقسام الفلاش في ESP-IDF | القسم متجه من البايتات بسلوك NOR: المسح بقطاعات 4 KB فقط، والممسوح = 0xFF، والكتابة تخفض البتات فقط (ومحاولة رفع بت تُحصى — `bitRaises`)؛ و`beforeWrite` — «انقطعت الطاقة»؛ وعدّادات القراءات والكتابات والمسح |
| `esp_system.h` | سبب إعادة التشغيل | `esp_reset_reason()` من `fake::chip().resetReason` |
| `Wire.h` | I2C | أجهزة بحسب العنوان؛ و`fake::RegisterMapDevice` — سجلات بزيادة تلقائية، وسجل للكتابات، وأعطال (`present` و`failWrites` و`failReads` و`failReadIf` و`shortRead`)، وخطّافات `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | أجهزة بحسب طرف CS؛ و`fake::SpiRegisterMapDevice` — بروتوكول Bosch/InvenSense، و`dummyBytes` قبل البيانات |
| `Preferences.h` | NVS | تخزين في الذاكرة، وسلوك `begin(readOnly)`/`get*`/`getBytes` كما في الأصل؛ و`failBegin` |
| `WiFi.h` و`WebServer.h` | نقطة وصول Wi-Fi، وHTTP | يحدد الاختبارُ نتيجة `softAP()`؛ ويستدعي `WebServer::request(method, uri, body)` المعالج المسجَّل؛ و`fake::webServers()` — جميع النسخ |
| `U8g2lib.h` | U8g2 | بدل البكسلات — قائمة بالسلاسل والمستطيلات المرسومة؛ ويمرّر `begin()/sendBuffer()` البايتات عبر استدعاء راجع (callback) للبايتات يحدده المستخدم؛ و`fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2` و`GPIO_PIN_MUX_REG` و`PIN_INPUT_ENABLE` |

### طبقة STM32duino — `test/native/support_stm32/` (البيئة `native-stm32`)

تقع في `-I` قبل `support/` وتستكمل البدائل نفسها بما يخص STM32duino وحده. و`<Preferences.h>` في هذه البيئة هو **الحقيقي** `include/hal/stm32/compat/Preferences.h` فوق `KeyValueStore`.

| الملف | يحلّ محلّ | ما يستطيعه |
|---|---|---|
| `Arduino.h` | نواة STM32duino | الأطراف `PA0..PE15` (المنفذ·16 + الرقم)، و`pin_size_t`، و`PinMap_TIM` لأطراف خرج السيرفوهات، و`HardwareTimer` (النبضة تُرى بـ `fake::timerPulseUs(pin)` و`pulseIn()`)، و`Uart`، و`noInterrupts()` |
| `STM32FreeRTOS.h` | FreeRTOS في STM32duino | `xTaskCreate` إلى سجل المهام المشترك (المكدس بالكلمات)، ويعود `vTaskStartScheduler()` — فالاختبار نفسه يشغّل المهام (`fake::runTask`)، و`xPortGetFreeHeapSize` |
| `EEPROM.h` | محاكاة EEPROM | «فلاش» بحجم 8 KB (الممسوح = 0xFF) ومخزن مؤقت، و`fake::eeprom()` — العدّادات وإفساد الصورة |
| `SPI.h` | | `SPIMode` |

وأُضيف في البدائل المشتركة من أجل STM32: `TwoWire(sda, scl)` و`setSDA/SCL` و`fake::wireWithSda(pin)` (لإيجاد الناقل الثاني في اللوحة)، و`HardwareSerial(rx, tx)` و`fake::uartByRx(pin)`، و`SPIClass::setSCLK/MISO/MOSI`.

### محاكيات الشرائح ونموذج الطائرة — `test/native/helpers/`

| الملف | ما هو |
|---|---|
| `ChipEmulators.h` | LSM6DSV وICM-45686 (مع سجلات IPREG غير المباشرة) وQMC6309 وSPL06-001 وBMP581 وإطارات NAV-PVT من u-blox — خرائط سجلات على I2C أو SPI، بيانات من «العالم» `World` (الزوايا والسرعات، والارتفاع، والسرعة الجوية، والاتجاه، والإحداثيات)، وبمحاور الشريحة مع مراعاة `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | نموذج طائرة بنحو 1.2 kg: كتلة نقطية + دوران في الدوران الجانبي والميل، وCL(α) مع الانهيار الهوائي، والسحب، والدفع، والريح، والتيارات الحرارية، والأرض |
| `SimHarness.h` | حلقة مغلقة: جهاز التحكم ← إطار iBUS ← `IBusReceiver` ← `PilotSwitches` ← `Autopilot` ← `FlightController` ← PWM ← انحرافات الأسطح ← `PlaneSim` ← الحساسات (ومنها أنبوب بيتو على مقياسَي ضغط ذوَي ضجيج). ومسار CSV عند ضبط `OPENPLANE_SIM_DIR` |

والملف `test/native/helpers/TestSupport.h` هو المشترك بين المجموعات: `resetWorld()` (يُستدعى من `setUp()`)، والبدائل `FakeUart`/`FakeServo`/`FakeBoard` وبدائل الحساسات (`FakeImu` و`FakeBaro` و`FakeMag` و`FakeGps`)، وبانِي الإطارات `ibusFrame()`، ومنصات `I2cRig`/`SpiRig` (مشغّل فوق `Esp32I2CBus`/`Esp32SpiBus` الحقيقيين و`*RegisterDevice` مع شريحة محاكاة).

اختبارات اللوحة (`test_feedback` و`test_imu_orientation`) قابلة للنقل: فعند وجود `ARDUINO` تُستخدم `setup()/loop()` وإلا `main()`. ولا تُبنى مجموعات `test/native/*` للوحة (`test_ignore` في `[esp32_common]` و`[env:stm32h743]`: الأنماط كل واحد في سطر — فإن فُصلت بمسافة قرأها PlatformIO نمطًا واحدًا). وعلى STM32: `pio test -e stm32h743`.

---

## مجموعات الاختبارات

| المجموعة | الاختبارات | ما الذي تتحقق منه |
|---|---|---|
| `native/test_hal` | 17 | مساعدات `II2CBus` (NACK، قراءة قصيرة — لا يُمَسّ المخزن المؤقت)، و`I2cRegisterDevice`، و`SpiRegisterDevice` (بت القراءة، البايت الوهمي في BMP388)، و`Esp32I2CBus` (مهلة 5 ms)، و`Esp32SpiBus` (الأنماط 0–3)، و`Esp32UartPort` (8N1، الأطراف)، و`Esp32ServoOutput` (50 Hz/14 بت، تحديد النبضة، عطل LEDC، القياس عبر مخزن الإدخال المؤقت)، و`Esp32Board` (النواقل، وUART، وترتيب القنوات، وAUX، والصافرة) |
| `native/test_rc` | 16 | `RcChannelState` و`RcInput`، وتحليل iBUS: إطارات تصل على أجزاء، وCRC، وقيم 12 بت، ووضع الأمان (failsafe) لجهاز التحكم، ومهلة 500 ms (وعند تجاوز سعة `micros()` أيضًا)، وبيانات عشوائية، وإعادة المزامنة |
| `native/test_control` | 21 | الفلابات (السرعة، الاستدعاء الأول، التوقفات)، والخلّاط (الإشارات، والعكس، والفلابرون)، والخانق، وآلة حالات ARM وفحوص حساسات الأوضاع، وجدول المخارج والفحص الذاتي للنبضات |
| `native/test_autopilot` | 22 | PID (حدّ D من سرعة الحساس، والتكامل، وanti-windup، و`dt`)، وSTABILIZE بوصفه وضع زوايا، والإقلاع الآلي بحسب الزمن، وALT_HOLD بدفة الارتفاع، والانسياب عند فقدان الإشارة |
| `native/test_autopilot_modes` | 31 | الأوضاع الـ 12 كلها وردّ فعل كل منها على غياب الحساس، وجدول الربط و`static_assert`، والوظائف والمقابض، والملاحة (الاتجاه، والدائرة، ونقطة الانطلاق، والسياج الجغرافي)، وfailsafe RTH/الانسياب، والإطلاق باليد، والتحليق الشراعي، والضبط التلقائي (الكتابة على الأرض فقط) |
| `native/test_flight_controller` | 12 | دورة كاملة من `FlightController` على الفئات الحقيقية: الأولويات: فقدان الإشارة، ثم ARM، ثم عصي التحكم/الطيار الآلي، ثم الخانق؛ وAUX، و`MOTOR_KILL`، والصافرة |
| `native/test_imu` | 21 | MPU6050/6500/9250 وICM-42688: التعرّف، والسجلات، والمقاييس، ودوران المحاور وإشارات الطيران، وأخطاء الناقل، ومعايرة الجيروسكوب والفحص قبل الإقلاع، ومعايرة التركيب من ثلاثة أوضاع، وNVS، ومرشّح الاتجاه |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`، وBMP388 عبر I2C وSPI، وBME280/BMP280 وفق مرجع Bosch، والبوصلات (الاتجاه، ومعايرة hard-iron في NVS)، وu-blox M10 (CFG-VALSET وNAV-PVT، وإطارات تالفة، ومهلة)، و`SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X، عنوان بديل، SPI)، وICM-45686 (سجلات غير مباشرة)، وQMC6309، وSPL06-001 (معادلات ورقة البيانات)، وBMP581 (DRDY والمسار الاحتياطي)، وأنبوب بيتو (الصفر، المرشّح، الكثافة، خرطومان مقلوبان، بيانات قديمة، «طيران» بضجيج مقياسَي ضغط) |
| `native/test_storage` | 16 | `KeyValueStore` (إعادة التحميل، والبلى — لا تُكتب القيمة المطابقة، وتجاوز السعة دون فقدان بيانات، وCRC، وانقطاع التيار أثناء المسح، وبيانات عشوائية، وإصدار الصيغة)، و`KvPreferences` (السلوك كما في NVS في ESP32) |
| `native/test_mavlink` | 20 | الكودك مقابل إطارات pymavlink المرجعية (v1 وv2 والموقَّع)، وCRC، وإعادة المزامنة؛ والقياس عن بُعد: ترددات التدفقات، وHEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS، ومعاملات PID (القائمة، والقراءة، والكتابة، ورفض القيم الخاطئة)، وتغيير الوضع من الأرض، وARM من الأرض — مرفوض، والمهام — 0، ومخزن UART الممتلئ لا يعطّل الحلقة |
| `native/test_blackbox` | 19 | الصندوق الأسود: الصيغة وCRC، وحلقة القطاعات على بديل NOR محاكٍ (قسم جديد دون مسح، والبيانات العشوائية تُمسح دائمًا، والرحلات القديمة تُمسح كاملةً ولأجل المساحة فقط، والأخيرة لا تُمَسّ، والالتفاف عند نهاية الحلقة، والرأس بعد إعادة التشغيل، وانقطاع التيار، وسجل لم تكتمل كتابته يكشفه CRC)، وتسجيل الرحلة على `FlightController`/`Autopilot` الحقيقيين: البدء عند ARM والخانق مع التسجيل المسبق، والتوقف بعد DISARM و«الوقوف على الأرض»، وفقدان الإشارة لا يوقفه، والتسجيل بعد إعادة تشغيل معيبة، والبدء اليدوي، والأحداث، والبطارية، ونفاد الفلاش في الجو، ورحلة أطول من القسم، والتنزيل بإطارات مع CRC وتغيير السرعة، وقائمة الكونسول `k`، وغياب القسم — معطَّل |
| `native/test_blackbox_scan` | 3 | الفحص الانتقائي للحلقة عند التشغيل مقابل الفحص الكامل: 300 تاريخ عشوائي للحلقة × 5 خطوات فحص (يتطابق الرأس والأرقام وقائمة الرحلات، وحين لا تتطابق الصورة يتراجع إلى الفحص الكامل) والكلفة على منطقة SD بحجم 64 MB (≈530 قراءة بدلًا من 32 ألفًا) |
| `native/test_telemetry` | 28 | `LoopStats`، و`LogSettings` (NVS، الإصدار)، و`DebugLogger` (جميع القنوات، وNAV)، و`DebugConsole` (القائمة، والمفاتيح السريعة، وفحص النواقل `b`، والمنع أثناء ARM، والحفظ دون ARM فقط)، و`WebDebugServer` (المسارات، وJSON، وصندوق البريد)، و`OledDisplay` (البايتات عبر I2C، والإطار، والعكس عند فقدان الإشارة) |
| `native/test_sim` | 15 | رحلات بحلقة مغلقة للبرنامج الثابت كله مع نموذج الطائرة: الخروج من الميلان، وCRUISE في ريح جانبية، وLOITER، وRTH، وfailsafe RTH/الانسياب، والسياج الجغرافي، والإقلاع الآلي من المدرج، والإطلاق باليد، والهبوط الآلي، والتيار الحراري، وRESCUE من حلزون، والحفاظ على السرعة والحماية من الانهيار الهوائي، وأنبوب بيتو حقيقي في الحلقة، والضبط التلقائي لطائرة «معوجّة»، وأعطال الحساسات أثناء الطيران (IMU، ومقياس الضغط، وأنبوب بيتو، وGPS) |
| `native/test_feedback_units` | 14 | وحدات التغذية الراجعة كلٌّ على حدة: مصادر السرعة، في الجو/على الأرض، وتقدير RLS، والمنظِّم، وعلامات الانهيار الهوائي، وإلغاء الإقلاع/الهبوط |
| `native/test_app` | 10 | `src/main.cpp` على ESP32-S3 مع عدّة المنصة MPU6500/BMP388/QMC5883P/OLED: دورة `loop()`، وجهاز التحكم ← السيرفو، وARM، والأوضاع، وفقدان الإشارة، والكونسول، ولوحة المعلومات، والشاشة، والصندوق الأسود (مهمة على النواة 0، والتسجيل عند الخانق، ورحلة بعد DISARM، و`bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` على ESP32-S3 مع عدّة الطيران: LSM6DSV + QMC6309 + SPL06 + BMP581 في الأنبوب + GPS — التعرّف على جميع الشرائح، وصفر الأنبوب والسرعة، والارتفاع، ونقطة الانطلاق من GPS، وSTABILIZE بزوايا الشريحة، وRTH إلى نقطة الانطلاق، وفحص النواقل، ولوحة المعلومات |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` على **ESP32 ذات 38 طرفًا** (`BOARD_ESP32_CLASSIC`) مع عدّة ICM-45686 + QMC6309 + SPL06 + BMP581: توزيع أطراف اللوحة، ومرشّحات IPREG، والإطلاق باليد، والتثبيت والسرعة، وفحص ناقل واحد |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` على **STM32H743** مع عدّة الطيران: المهام والأولويات، ودورة 2 ms، والأنبوب، ومؤقتات PWM و`pulseIn`، وMAVLink أثناء الطيران، وتغيير الوضع من GCS، وكتابة الإعدادات بمهمة خلفية في «الفلاش»، والشاشة على I2C1، والكونسول |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 مع ICM-45686 وBMP581 **عبر SPI** + QMC6309: فلاش تالفة عند التشغيل، وALT_HOLD من GCS يحافظ على الارتفاع، وفقدان الإشارة ← RTH، ويظهر في MAVLink؛ وإعادة كتابة صورة تالفة |
| `native_stm32/test_blackbox_sd` | 29 | الصندوق الأسود على بطاقة SD: FAT32 (مع MBR وبدونه، ودليل يمتد على عنقودين، وسجلات ضجيج، وحجم غريب/مبعثر/فارغ)، و`SdFileRegion` (كتل ناقصة، وذاكرة مؤقتة، ومسح، وحدود، وأعطال)، ومشغّل `Stm32SdCard` الحقيقي فوق `HAL_SD` مزيّف (4 بتات، وسرعات احتياطية، وإعادة المحاولة، وبطاقة مشغولة، ومخازن غير محاذاة)، والحلقة على البطاقة (إعادة التشغيل، وانقطاع التيار، وكلفة الفحص)، وعلامة «الحلقة فارغة»، وتسجيل الرحلة على `FlightController`، وإعادة تشغيل معيبة تُكتشف عبر `RCC->RSR`، وADC البطارية، وأخطاء البطاقة أثناء الطيران، وبطاقة بطيئة، والتنزيل عبر الكونسول، والمفتاح `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` مع بطاقة: الإقلاع يجد البطاقة والملف، ومهمة `bbox` تكتب الرحلة، ودورة الحلقة لا تتمدد، و`bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` بلا بطاقة: الصندوق الأسود معطَّل ويشرح السبب، والطائرة تطير، وقائمة `k` لا تتعطل |
| `test_feedback` | 10 | محاكاة بحلقة مغلقة للطائرة مع حلقة التغذية الراجعة (على الحاسوب وعلى اللوحة) |
| `test_imu_orientation` | 5 | معايرة تركيب IMU على 300 تركيب عشوائي (على الحاسوب وعلى اللوحة) |
| **الإجمالي** | **387** | 340 في `native` + 47 في `native-stm32` (إضافة إلى 9 على اللوحة فقط — `test_blackbox_sd`) |

### الاختبارات على لوحة STM32

‏`test/test_blackbox_sd` ليس أصليًّا: مشغّل SDMMC والبطاقة والوقت كلها حقيقية. تعمل الاختبارات في مهمة FreeRTOS، وإلى جانبها تعمل مهمة تحاكي حلقة الطيران بأعلى أولوية (دورة 2 ms): فهي تستبق الاختبارات (preemption) في منتصف الوصول إلى البطاقة، كما في البرنامج الثابت. ومن دونها لا يمكن اصطياد الخطأ الذي اكتُشف على اللوحة: عند الاستباق كانت ذاكرة FIFO الخاصة بـ SDMMC تفيض (`HAL_SD_ERROR_RX_OVERRUN`)، وهذا لا يحدث في حلقة عارية.

| الاختبار | ما الذي يتحقق منه |
|---|---|
| `reset_cause_is_a_normal_one` | سبب إعادة التشغيل (`RCC->RSR`) ليس مؤقّت المراقبة (watchdog) ولا هبوط الجهد |
| `card_is_detected_on_four_bit_bus` | تُعرَّف البطاقة على ناقل 4 بتات وبتردد 24 MHz |
| `file_is_found_and_contiguous` | يُعثر على `BLACKBOX.BIN` في FAT32 وهو متصل (متجاور) غير مجزّأ |
| `multi_block_writes_work_at_every_length` | كتابة 1 و2 و4 و8 كتل بوصول واحد |
| `pages_write_with_bounded_latency_and_read_back_intact` | صفحات 256 B: أسوأ كتابة أقل من 250 ms (حدّ SD)، وبثبات أكثر من 40 KB/s، والقراءة والمسح |
| `header_scan_cost_on_the_whole_area` | كلفة قراءة ترويسة القطاع والفحص الكامل |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | مسح كل شيء، ورحلتان من 20 000 سجل لكل منهما، و«إعادة تشغيل»: الفحص الانتقائي أقل من 2 s، وتُقرأ السجلات بالترتيب مع CRC صحيح؛ والحلقة الفارغة تُعرف بالعلامة في أقل من 100 ms |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | `BlackBox` الحقيقي مع IMU بتردد 500 Hz في الزمن الحقيقي: لا سجل مفقود واحد، وتُقرأ الرحلة بعد «إعادة التشغيل» |
| `the_flight_task_was_not_disturbed` | لم تُخلّ الكتابة على البطاقة بدورة المهمة المحاكية (الانحراف أقل من 3 ms) |

التشغيل (بطاقة فيها الملف — `python tools/blackbox.py sd-prepare E:`؛ **الاختبار يمسح جميع الرحلات في الملف**):

<div dir="ltr">

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

</div>

يجب أن تكون اللوحة في وضع DFU (على DevEBox — سلك BT0→3V3 وRST، ومشغّل WinUSB عبر Zadig؛ والتفاصيل في [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). كونسول STM32 هو USB CDC: بعد الرفع لا يظهر المنفذ فورًا، وأحيانًا لا يلحق `pio test` بفتحه («could not open port») — عندها شغّل `pio test ... --without-testing` واقرأ المخرجات بأي برنامج طرفية مع تفعيل DTR (تنتظر الاختبارات فتح المنفذ حتى 60 s). وبعد الاختبارات تنتظر اللوحة المفتاح **`D`** — فيعيد تشغيلها في وضع DFU دون السلك.

النتائج على DevEBox H743 + بطاقة 16 GB (2026-10-02): `test_blackbox_sd` — 9/9، و`test_feedback` — 10/10، و`test_imu_orientation` — 5/5؛ أما أرقام سرعة البطاقة ففي [BLACKBOX.md](BLACKBOX.md#ما-الذي-قيس-على-اللوحة).

### برامج ثابتة لمنصة الاختبار — `test/bench/`

هذه ليست مجموعات اختبارات، بل مشاريع PlatformIO صغيرة مستقلة تُرفع إلى اللوحة بدل البرنامج الثابت الخاص بالطيران (لا يراها `pio test`: فأسماء المجلدات لا تبدأ بـ `test_`). وتؤخذ الأطراف والحدود من `Config.h` المشترك.

| المشروع | ما الذي يفعله |
|---|---|
| `bench/elevator_sweep` | يحرّك برمجيًّا عصا دفة الارتفاع (CH2) عبر `ControlMixer` و`FlightOutputs` كأنها عصا حية: إلى الأعلى 100 % من الشوط، وإلى الأسفل 60 %، بسلاسة ومع توقفات؛ 20 s عمل — 20 s في الوضع المحايد. وفي الوضعين الأقصيين يقيس النبضة على المخارج. الخانق عند الحد الأدنى |

للرفع: `pio run -d test/bench/elevator_sweep -t upload`. ولاستعادة البرنامج الثابت الخاص بالطيران: `pio run -e esp32-s3 -t upload`.

---

## التغطية

يحسبها `gcovr` على `include/` و`src/` (كل ما يدخل في البرنامج الثابت)، على البيئتين الأصليتين معًا: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| الطبقة | الأسطر | الفروع |
|---|---|---|
| `autopilot` | 920/943 (97.6 %) | 645/731 (88.2 %) |
| `autopilot/feedback` | 683/702 (97.3 %) | 501/570 (87.9 %) |
| `control` | 252/256 (98.4 %) | 171/189 (90.5 %) |
| `hal` | 98/102 (96.1 %) | 26/26 (100 %) |
| `hal/esp32` | 101/102 (99.0 %) | 21/22 (95.5 %) |
| `hal/stm32` | 149/158 (94.3 %) | 35/52 (67.3 %) |
| `rc` | 92/92 (100 %) | 41/42 (97.6 %) |
| `sensors` (الكل) | 1444/1446 (99.9 %) | 716/835 (85.7 %) |
| `storage` | 220/220 (100 %) | 158/178 (88.8 %) |
| `telemetry` | 1413/1440 (98.1 %) | 1123/1269 (88.5 %) |
| `src` (`main.cpp`، `stm32/main.cpp`) | 118/123 (95.9 %) | 20/29 (69.0 %) |
| **الإجمالي** | **5490/5584 (98.3 %)** | **3457/3943 (87.7 %)**؛ الدوال 877/902 (97.2 %) |

ما بقي دون تغطية ولماذا:

- **الإطلاق باليد** (`TakeoffSequencer`: `WaitLaunch` و`launchDetected()`) — يتعذّر الوصول إليه ما دام `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false`؛ وسيظهر في الاختبارات حين يصبح الثابت قابلًا للضبط (بانتقاله إلى `Config.h`).
- **ما يتعلق باللوحة:** مخرج بلا طرف (`PIN_RUDDER = -1` لا يحدث إلا على C3)، وGPS بلا طرف TX (C3) — الاختبارات الأصلية تجرّب توزيع الأطراف في S3 والنسخة ذات 38 طرفًا وSTM32، لكن ليس C3 (فيُفحص C3 بمصفوفة البناء).
- **‏STM32:** فروع الأخطاء في النواة (لا مؤقّت على الطرف، استُنفد مجمّع المؤقّتات)، والرسالة `FreeRTOS не запустился` («لم يبدأ FreeRTOS») — على الحاسوب تعود `vTaskStartScheduler()` دائمًا.
- **فروع الحماية** التي لا يمكن بلوغها عبر الواجهة العامة: `default`/`Count` في `switch` على التعدادات، و`return "?"`.
- الملفات التي لا تحتوي أسطرًا قابلة للتنفيذ (`Config.h` و`Channels.h` و`FeedbackConfig.h` والبُنى `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets` وماكروات `SensorSelection.h` وصفحة HTML للوحة المعلومات) لا تظهر في التقرير — فهي تُترجَم ضمن الاختبارات، لكن لا شيء فيها ليحسبه gcov.

---

## التحليل الساكن

| الأداة | الأمر | الإعداد |
|---|---|---|
| GCC | `tools/build_matrix.sh` (أو `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | كل اللوحات × كل عُدد الحساسات. بناء الاختبارات الأصلي دائمًا مع `-Wall -Wextra -Wshadow`؛ و`stm32h743` مع `-Wall -Wextra` (`build_src_flags`؛ إذ يُحدث `-Wshadow` ضجيجًا في ترويسات STM32duino نفسها) |
| cppcheck | `pio check -e esp32-s3`؛ `pio check -e stm32h743` | `check_*` في `[esp32_common]`: `include/` و`src/` (عدا `stm32/`)، وwarning/style/performance/portability، وتعليقات `// cppcheck-suppress` المضمَّنة للإنذارات الكاذبة فقط (دالة الاستدعاء الخلفي في U8g2، و`setup/loop`). أما `stm32h743` فبالأعلام نفسها على `include/hal/stm32/` و`src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone وclang-analyzer وperformance و`misc-include-cleaner` وغيرها؛ وتُشرح الفحوص المعطَّلة في الملف نفسه |

يعمل clang-tidy مع البدائل المحاكية من `test/native/support`: فلا يستطيع clang تحليل ترويسات ESP-IDF لمعمارية المضيف (فعند محاولة `pio check` مع `clangtidy` يتوقف التحليل عند أخطاء التحليل النحوي ولا يفحص شيئًا في الواقع). ويحرص `misc-include-cleaner` على أن تضمّن كل ترويسة ما تستخدمه: والترويسات «المظلّية» (`FeedbackModules.h` وواجهة `IBoard.h`/`RegisterDevice.h` وماكروات `SensorSelection.h`) موسومة بـ `// IWYU pragma: export`. ويتخطى السكربت شيفرة STM32 (`include/hal/stm32/` و`src/stm32/`) — إذ تفحصها عملية البناء، وcppcheck لبيئة `stm32h743`، واختبارات بيئة `native-stm32`.

مصفوفة البناء عند آخر تمريرة — **24/24 بلا تحذيرات**:

| اللوحة | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

‏cppcheck (`esp32-s3`، `stm32h743`) — 0 ملاحظات على شيفرة المشروع.

---

## كيف تكتب اختبارات جديدة

1. وحدة فيها منطق بلا عتاد — اختبار وحدة مباشر: يُمرَّر الوقت معاملًا أو يُقدَّم بـ `fake::advanceMs()`.
2. مشغّل شريحة — عبر `I2cRig`/`SpiRig`: سجلات شريحة محاكاة، وفحص القيم المكتوبة (`chip.lastWrite(reg)`) وتحليل البيانات. وللمعادلات استخدم مرجعًا من ورقة البيانات أو حسابًا مستقلًّا، لا نسخة من الشيفرة.
3. فئات فيها مهام FreeRTOS لا نهائية — `fake::findTask("الاسم")` + `fake::runTask(task, n)`؛ وبالطريقة نفسها تُدار مهمة الطيران في STM32.
4. البرنامج الثابت كاملًا مع عدّة حساسات أخرى أو لوحة أخرى — مجموعة مستقلة تحدّد قبل `#include "../../../src/main.cpp"` القيمة `SENSOR_KIT` (أو `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`)؛ والشرائح من `helpers/ChipEmulators.h`. وفي STM32 — `test/native_stm32/`.
5. وضع جديد للطيار الآلي — سيناريو طيران بحلقة مغلقة في `test_sim`.
6. مجموعة جديدة — مجلد `test/native/test_<الاسم>/test_main.cpp` فيه `main()`؛ وتستدعي `setUp()` الدالة `resetWorld()` إن لم تحتج المجموعة إلى حالة بين الاختبارات.
7. عثرت على علّة — اكتب أولًا اختبارًا يلتقطها، ثم أصلحها.

</div>
