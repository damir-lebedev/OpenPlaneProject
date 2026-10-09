<div dir="rtl">

# ‏DEVELOPER_GUIDE.md — دليل مطوّر OpenPlaneProject

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../DEVELOPER_GUIDE.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

خريطة تقنية للبرنامج الثابت: أي ملف مسؤول عن ماذا، وكيف تتدفق البيانات من المستقبل والحساسات إلى محركات السيرفو، وأي اتفاقيات للإشارات تُمسك السلسلة كلها معًا، وكيف صُمِّمت واجهة الويب البرمجية، وكيف يُوسَّع المشروع. وهو موجّه إلى مطوّر يكتب C++ ويريد أن يتعرف بسرعة على هذا المستودع (الفرع `main`)، لا إلى تعلّم أساسيات اللغة أو PlatformIO.

للاطلاع على نظرة عامة على المشروع وحالة النموذج الأولي انظر [`../README.md`](README.md)، وللتوصيلات وطريقة الطيران [`PILOT_GUIDE.md`](PILOT_GUIDE.md)، وللخطط [`ROADMAP.md`](ROADMAP.md). وهنا الشيفرة فقط. أما البنية كاملة (الطبقات والمهام وآلات الحالات) ففي [`ARCHITECTURE.md`](ARCHITECTURE.md)، ومرجع كل فئة في [`reference/`](reference/README.md)، والاختبارات في [`TESTING.md`](TESTING.md).

> المشروع قيد التطوير النشط. جُمِّعت منصة ESP32-S3 وفُحصت مع جميع الحساسات، لكن **الطيار الآلي لم يُجرَّب في الطيران بعد** — وقد نُبِّه على ذلك حيثما يتعلق الأمر بوحدة بعينها. وإن شككت فيما تفعله الشيفرة فأعد قراءة الشيفرة المصدرية لا الوثيقة.

---

## المحتويات

1. [بنية الطبقات](#بنية-الطبقات)
2. [مهام FreeRTOS وحلقة التحكم](#مهام-freertos-وحلقة-التحكم)
3. [مرجع الملفات](#مرجع-الملفات)
4. [اتفاقية الإشارات: من IMU إلى السيرفو](#اتفاقية-الإشارات-من-imu-إلى-السيرفو)
5. [خريطة قنوات RC وARM وfailsafe](#خريطة-قنوات-rc-وarm-وfailsafe)
6. [بيانات الحساسات](#بيانات-الحساسات)
7. [تفصيل FlightController::update()](#تفصيل-flightcontrollerupdate)
8. [واجهة HTTP البرمجية للوحة المعلومات عبر الويب](#واجهة-http-البرمجية-للوحة-المعلومات-عبر-الويب)
9. [الكونسول والتشخيص](#الكونسول-والتشخيص)
10. [اختيار اللوحة وتوزيع الأطراف](#اختيار-اللوحة-وتوزيع-الأطراف)
11. [كيف تضيف حساسًا جديدًا](#كيف-تضيف-حساسًا-جديدًا)
12. [كيف تضيف وضعًا جديدًا للطيار الآلي](#كيف-تضيف-وضعًا-جديدًا-للطيار-الآلي)
13. [التغذية الراجعة (تمهيد، غير موصولة)](#التغذية-الراجعة-تمهيد-غير-موصولة)
14. [كيف تضيف لوحة جديدة](#كيف-تضيف-لوحة-جديدة)
15. [أوامر البناء والرفع والمراقبة](#أوامر-البناء-والرفع-والمراقبة)
16. [القيود المعروفة](#القيود-المعروفة)
17. [كيف تُجري التعديلات](#كيف-تُجري-التعديلات)

---

## بنية الطبقات

تقع الفئات كلها تقريبًا في ترويسات موزّعة على المجلدات `include/<الطبقة>/`. وكل ترويسة تضمّ بنفسها ما تستخدمه (`#include "config/Config.h"` و`"hal/II2CBus.h"` و… — المسارات من `include/`). ويمثّل `src/main.cpp` نقطة التجميع الوحيدة (composition root): ينشئ جميع الكائنات ويربط بينها ويشغّل `setup()`/`loop()`. والاعتماديات أحادية الاتجاه — فالطبقة السفلى لا تعرف شيئًا عن العليا.

<div dir="ltr">

```
include/
├── config/      Config.h (الأطراف وجميع الإعدادات), Channels.h (أسماء القنوات),
│                Controls.h (ما يفعله كل مفتاح — سطر واحد لكل قناة)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (جهاز سجلات فوق I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + أغلفة فوق Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (الإعدادات في الفلاش بدل NVS)
├── storage/     KeyValueStore, KvPreferences — تخزين الإعدادات دون NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  تمهيد للتغذية الراجعة — غير موصول (انظر القسم أدناه)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (أنبوب من مقياسَي ضغط)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — البرنامج الثابت لـ ESP32 (S3 وC3 و38 طرفًا)
src/stm32/main.cpp  — البرنامج الثابت لـ STM32H743 (مهام FreeRTOS وMAVLink)
```

</div>

<div dir="ltr">

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — تجميع الكائنات       │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — ترتيب العمليات في كل دورة     │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 وضعًا    │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ الملاحة وPID       │  │ RcChannelState        │
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
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, فلاش     │
└───────────────────────────────────────────────────────────────────────┘
```

</div>

القواعد التي تحافظ على نقاء البنية:

- **‏HAL** هي الطبقة الوحيدة المسموح لها بمعرفة متحكم دقيق بعينه (`Wire` و`SPI` و`HardwareSerial` و`ledc*`). وكل ما فوقها يتعامل مع الواجهات فقط. والانتقال إلى متحكم آخر يعني ملف `hal/<mcu>/<Mcu>Board.h` جديدًا، أما بقية الشيفرة فلا تتغير (مثال ذلك `hal/stm32/` الخاص بـ STM32H743).
- **مشغّلات الحساسات لا تعرف الناقل.** تتلقى `IRegisterDevice&` — ويُنشأ جهاز I2C بعنوانه أو SPI بطرف CS في `SensorSelection.h`. ويعمل `BMP388_Sensor` نفسه عبر I2C وعبر SPI.
- **المشترك يعيش في الفئات الأساسية.** المعايرة، ودوران المحاور، والإشارات، ومرشّح الاتجاه، والارتفاع والسرعة الرأسية، وتخزين معايرة البوصلة، وعدّ أخطاء الناقل — كلها في `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. أما مشغّل الشريحة فيحوي السجلات والمعادلات من ورقة البيانات فقط.
- **‏RC وOutputs** لا يعرفان شيئًا عن الطائرة: بايتات iBUS ← قنوات، وقيم PWM ← مخارج.
- **‏Control وAutopilot** منطق فوق البيانات، بلا UART أو PWM أو Wi-Fi. والزمن، حيث يلزم (الفلابات)، يُمرَّر معاملًا.
- **‏Coordination** (`FlightController`) هي الفئة الوحيدة التي ترى عدة طبقات سفلية معًا وتقرر ترتيب العمليات.
- **‏Application** (`main.cpp`) هو المكان الوحيد الذي تُنشأ فيه `Esp32Board` والأجهزة والحساسات ويُربط فيه كل شيء يدويًّا، دون إطار لحقن الاعتماديات (DI).

---

## مهام FreeRTOS وحلقة التحكم

| أين | ماذا | الدورة |
|---|---|---|
| النواة 1، `loop()` (مهمة loopTask في Arduino) | `applyPendingCommands()` ← `FlightController::update()` ← `DebugLogger::update()` ← `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz)، `vTaskDelayUntil` |
| النواة 0، مهمة `web` | `WebServer::handleClient()` | كل 2 ms |
| النواة 0، مهمة `oled` | رسم شاشة SSD1306 عبر ناقل I2C الثاني | 200 ms |
| النواة 0 | مكدّس Wi-Fi في ESP-IDF | — |

- تُحفظ دورة الحلقة بـ `vTaskDelayUntil` لا بـ `delay(2)` بعد العمل — فالتردد لا يتوقف على طول مدة الدورة. وبعد حجب طويل (معايرة من الكونسول) يبدأ العدّ من جديد، ولا تُستدرك الدورات الفائتة دفعة واحدة.
- على المنصة (ESP32-S3، جميع الحساسات): 500 Hz، ونحو ~0.7 ms من العمل لكل دورة في المتوسط، وأسوأ دورة ~1.4 ms. ويُطبع هذا كل 10 s في سطر `SYS:`.
- مهلة معاملة I2C هي 5 ms (والمعتادة في Wire هي 50 ms): فالمعاملة التي تعلق بسبب تشويش لا توقف الحلقة طويلًا.
- **فصل البيانات بين المهام.** تقتصر مهمتا الويب وOLED على *قراءة* الحالة (`FlightController`/`Autopilot`/`LoopStats`) — وهي حقول منفصلة بعرض 16/32 بت، فتظهر في أسوأ الأحوال قيم من دورات متجاورة. أما *أوامر* لوحة المعلومات (`setmode`/`setpid`) فلا تُطبَّق مباشرة من مهمة الويب: بل توضع في «صندوق بريد» تحت `portMUX` وتلتقطها حلقة الطيران في `WebDebugServer::applyPendingCommands()`.
- ‏`Serial` (UART0 ← جسر CH343 ← موصل «COM») بمخزن إرسال 4 KB: فإطار التصحيح (نحو 600 حرف) لا يعطّل الحلقة أثناء إرساله.

---

## مرجع الملفات

### ‏`config/`

| الملف | مسؤول عن |
|---|---|
| `Config.h` | جميع الأطراف (كتلة لكل لوحة: `BOARD_ESP32_S3/C3/CLASSIC` و`BOARD_STM32H743`) والإعدادات: iBUS وفقدان الاتصال؛ أشواط أسطح التحكم؛ الفلابات؛ عكس السيرفو؛ تركيب IMU والبوصلة؛ ARM؛ failsafe (RTH أو الانسياب)؛ أنبوب بيتو (`PITOT_*`)؛ جميع أرقام أوضاع الطيار الآلي ووظائفه؛ الحلقة؛ Wi-Fi؛ MAVLink؛ التصحيح |
| `Channels.h` | أسماء القنوات: `AILERON` و`ELEVATOR` و`THROTTLE` و`RUDDER` و`ARM` و`SWB` و`SWC` و`SWD` و`VRA` و`VRB` |
| `Controls.h` | جدول `BINDINGS`: ما يفعله كل مفتاح ومقبض، سطر واحد لكل قناة، مع فحوص `static_assert` |

### ‏`hal/`

| الملف | مسؤول عن |
|---|---|
| `IBoard.h` | نقطة الدخول إلى العتاد: `i2c()` و`displayI2c()` (ناقل ثانٍ للشاشة، قد يكون `nullptr`) و`spi()` و`rcUart()` و`gpsUart()` و`telemetryUart()` (MAVLink، قد يكون `nullptr`) و`servo(ServoChannel::*)` (7 مخارج مع AUX1/AUX2) و`setBuzzer()` |
| `Rtos.h` | مهام FreeRTOS بالطريقة نفسها في ESP32 (النواة 0) وSTM32 (الأولويات)، والكومة الحرة |
| `II2CBus.h` | ناقل I2C: دوال أولية على شكل `Wire` + مساعِدات `writeRegister()` و`readRegisters()` (تتحقق من وصول `count` بايت بالضبط) و`readRegister()` و`probe()` |
| `ISpiBus.h` و`IUartPort.h` و`IServoOutput.h` | SPI وUART ومخرج PWM واحد (`measurePulseUs()` — تشخيص النبضة الفعلية) |
| `RegisterDevice.h` | `IRegisterDevice` — «مجموعة سجلات من 8 بتات»؛ `I2cRegisterDevice` (العنوان)، `SpiRegisterDevice` (CS والتردد وبايتات وهمية قبل البيانات) |
| `esp32/Esp32Board.h` | تنفيذ `IBoard`: `Wire` (الحساسات) و`Wire1` (الشاشة، إن كانت للشريحة متحكمتا I2C) و`SPI` و`HardwareSerial` اثنان و5 قنوات LEDC |
| `esp32/Esp32I2CBus.h` | `II2CBus` فوق أي `TwoWire`، بمهلة 5 ms |
| `esp32/Esp32ServoOutput.h` | PWM عبر LEDC: 50 Hz، 14 بت؛ الطرف −1 — المخرج غير موصول. ولا تُستخدم مكتبة ESP32Servo — انظر [القيود](#القيود-المعروفة) |
| `esp32/Esp32SpiBus.h` و`esp32/Esp32UartPort.h` | أغلفة رقيقة فوق `SPI` و`HardwareSerial` |
| `stm32/*` | STM32H743: `Stm32Board` (+ UART4 الخاص بمودم الراديو) والنواقل ومؤقتات PWM و`Stm32FlashStorage` (الإعدادات في قطاع من الفلاش تكتبها مهمة خلفية) و`compat/Preferences.h` |

### ‏`storage/`

| الملف | مسؤول عن |
|---|---|
| `KeyValueStore.h` | صورة «مساحة اسم/مفتاح ← بايتات» مع CRC32 في الذاكرة العشوائية فوق أي وسيط (`IFlashStorage`)؛ ولا تُعاد كتابة القيمة المطابقة |
| `KvPreferences.h` | واجهة `Preferences` الخاصة بـ ESP32 فوق `KeyValueStore` |

### ‏`rc/`

| الملف | مسؤول عن |
|---|---|
| `RcChannelState.h` | لقطة لـ 10 قنوات |
| `RcInput.h` | `clamp()` و`centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS ← قنوات: إطار من 32 بايت، وCRC، وقيمة القناة هي الـ 12 بتًّا الدنيا (`& 0x0FFF`)؛ و`isSignalLost()` = لا إطارات (أو لم يصل أي إطار بعد) ∥ قيمة الخانق في failsafe؛ وعدّادات الإطارات |

### ‏`control/`

| الملف | مسؤول عن |
|---|---|
| `ControlCommand.h` | أمر الأسطح بإشارات فيزيائية — اللغة المشتركة للعصي والطيار الآلي والخلّاط |
| `ControlMixer.h` | `fromSticks(rc)` ← `ControlCommand`؛ و`updateFlaps(الهدف, now)`؛ و`mix(command)` ← PWM مع عكس السيرفو؛ والفلابرون: الجنيحات `flaps ± roll` (السالب — مكبح هوائي) |
| `FlapsController.h` | إخراج الفلابات وسحبها بسلاسة، والزمن يُمرَّر معاملًا |
| `ThrottleManager.h` | الخانق من العصا؛ وعند فقدان الاتصال — `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM بالمفتاح SwA (انتقال OFF→ON والخانق في الأسفل + فحوص حساسات الوضع)، وDISARM فوري |
| `FlightOutputState.h` | قيم PWM المطلوبة: `aileronLeft` و`aileronRight` و`elevator` و`rudder` و`throttle` و`aux1` (الحمولة) و`aux2` (الكاميرا) |
| `Beeper.h` | الصافرة: بوظيفة `BEEPER` أو «فُقد النموذج» على الأرض |
| `FlightOutputs.h` | جدول المخارج (`outputInfo()`: المفتاح والاسم والطرف وإلزاميته وحقل الحالة) وكل ما فوقه في حلقة: `begin()` و`write()` و`setFailsafe()` والحالة و`printPulseSelfTest()` |
| `FlightController.h` | ترتيب العمليات في كل دورة، وفقدان الاتصال (`applyLinkLoss()`)، ودوال الجلب للقياس عن بُعد |

### ‏`autopilot/`

| الملف | مسؤول عن |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 وضعًا) و`Feature` و`Knob` و`PilotInputs` والأسماء |
| `ControlBinding.h` | `Binding` ومصانع `Bind::modes/mode/feature/knob` وفحوص `BindingCheck` |
| `PilotSwitches.h` | جدول الربط ← الوضع والوظائف والمقابض في كل دورة؛ والتخطيط عند التشغيل |
| `Autopilot.h` | 12 وضعًا، وfailsafe RTH/الانسياب، والسياج الجغرافي، ونقطة الانطلاق، وتنسيق الانعطاف، والضبط التلقائي؛ و`update(armed, linkLost, الخانق, العصي)` ← `getCommand()` و`applyThrottle()` |
| `Navigation.h` | `Geo` (المسافة والاتجاه والإزاحة) و`Guidance` (الدوران الجانبي لبلوغ اتجاه، وحقل المتجهات للدائرة) |
| `AltitudeSpeedController.h` | الميل للارتفاع، والخانق للسرعة الجوية (TECS-lite) |
| `LaunchController.h` و`SoaringController.h` | آلتا حالات الإطلاق باليد والتحليق الشراعي |
| `AutoTrim.h` | الضبط التلقائي، ويُخزَّن في NVS/الفلاش |
| `PidController.h` | PID: حدّ D من سرعة الحساس (الجيروسكوب، مقياس الصعود)، وanti-windup، والمكامِل مجمَّد دون ARM |
| `feedback/*` | **تمهيد، غير موصول:** تغذية راجعة تكيفية، وإقلاع وهبوط — انظر [التغذية الراجعة](#التغذية-الراجعة-تمهيد-غير-موصولة) |

### ‏`sensors/`

| الملف | مسؤول عن |
|---|---|
| `SensorInterface.h` | واجهات `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` وبنى البيانات |
| `SensorSelection.h` | أي شريحة تُترجَم (`#define SENSOR_*`، ويمكن تجاوزها بخيار بناء) وعلى أي ناقل هي (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | تدوير محاور الشريحة إلى محاور الطائرة (0/90/180/270° باتجاه عقارب الساعة) — للبوصلة ولـ IMU غير المعايَرة للتركيب |
| `imu/ImuOrientation.h` | تركيب IMU بمصفوفة «محاور الشريحة ← محاور الطائرة»: من `IMU_ROTATION_CW_DEG` أو من ثلاث وضعيات (أفقية، الأنف إلى أعلى، الجناح الأيمن إلى أسفل) مع تحقق؛ وتُخزَّن في NVS |
| `imu/ImuSensorBase.h` | القاسم المشترك بين وحدات IMU: معايرة الجيروسكوب + فحص ما قبل الطيران (السكون، 1g، تطابق «الأعلى» مع التركيب)، ومعايرة التركيب (`calibrateOrientation()`)، والمقياس، والتدوير، وإشارات الطيران، وأخطاء الناقل |
| `imu/AttitudeEstimator.h` | مرشّح تكميلي للدوران الجانبي/الميل، وتكامل الانعراج |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (تُعرَف الشريحة بـ WHO_AM_I): ‏±2000°/s، ‏±16g، ‏DLPF نحو 41 Hz، ‏1 kHz. **على المنصة** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ‏±2000°/s، ‏±16g، ‏1 kHz، مرشّح UI عند 50 Hz. لم يُختبر على العتاد |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ‏±2000°/s، ‏±16g، ‏960 Hz، ‏LPF1/LPF2؛ ‏I2C بالعنوان 0x6A/0x6B أو SPI. لم يُختبر على العتاد |
| `imu/ICM45686_Sensor.h` | ICM-45686: ‏±2000°/s، ‏±16g، ‏1.6 kHz، مرشّح تمرير منخفض عبر السجلات غير المباشرة IPREG؛ ‏I2C بالعنوان 0x68/0x69 أو SPI. لم يُختبر على العتاد |
| `baro/BarometerBase.h` | القاسم المشترك بين مقاييس الضغط: استطلاع العيّنات الجديدة فقط، والارتفاع، والسرعة الرأسية عبر مرشّح تمرير منخفض، ومعايرة القاعدة، والأخطاء |
| `baro/BMP388_Sensor.h` | ‏BMP388 عبر I2C أو SPI (مع بايت SPI الوهمي)، وتعويض Bosch، والقراءة بحسب علَم الجاهزية. **على المنصة (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280، وتعويض Bosch §8.1. لم يُختبر على العتاد |
| `baro/SPL06_Sensor.h` | SPL06-001: المعاملات والصيغ من ورقة البيانات، ‏32 Hz ×16؛ ‏I2C بالعنوان 0x76/0x77 أو SPI. لم يُختبر على العتاد |
| `baro/BMP581_Sensor.h` | BMP581: تسلسل BMP5_SensorAPI، ‏16×/2×، ‏IIR؛ ‏I2C بالعنوان 0x46/0x47 أو SPI؛ ويصلح مقياس ضغط رئيسيًّا (طقم المنصة الافتراضي) وأنبوب بيتو معًا. لم يُختبر على العتاد |
| `mag/MagnetometerBase.h` | القاسم المشترك بين البوصلات: استطلاع 50 Hz، ومعايرة hard-iron في NVS، وتدوير المحاور، والاتجاه، والأخطاء |
| `mag/QMC5883P_Sensor.h` | QMC5883P، بالعنوان 0x2C. **على المنصة** |
| `mag/QMC5883L_Sensor.h` | QMC5883L، بالعنوان 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309، بالعنوان 0x7C: ‏±8 G، ‏200 Hz. لم يُختبر على العتاد |
| `gps/UbloxM10_Gps.h` | u-blox M10: الضبط عبر CFG-VALSET (115200 باود، ‏10 Hz، ‏NAV-PVT، دون NMEA)، وتحليل NAV-PVT. غير موصول على المنصة |
| `airspeed/AirspeedSensor.h` | واجهة حساس السرعة الجوية: فرق الضغط، وIAS، وTAS، والكثافة |
| `airspeed/PitotDualBaroAirspeed.h` | أنبوب بيتو مصنوع منزليًّا: BMP581 داخل الأنبوب + مقياس ضغط في جسم الطائرة؛ الصفر على الأرض، ومرشّح تمرير منخفض، والكثافة من الضغط الساكن، وكشف العطل |

### ‏`telemetry/` والتطبيق

| الملف | مسؤول عن |
|---|---|
| `DebugLogger.h` | سجل بحسب القنوات (`LogSettings.h`): لكل قناة سطرها وهامش ارتداد خاص بها ووضعها؛ ويصمت ما دامت القائمة مفتوحة |
| `DebugConsole.h` | قائمة نصية في مراقِب المنفذ (`h`) ومفاتيح اختصار (`l`/المسافة/`s`/`i`/`o`/`m`/`p`/`b`)؛ ويكتب إعدادات السجل في NVS عند الخروج من القائمة وفقط دون ARM |
| `LogSettings.h` | قنوات السجل (STAT وRC وOUT وATT وAP وALT وMAG وGPS وIMU وNAV وSYS) وأوضاعها: مُعطَّل / عند التغيّر / دائم؛ وتُخزَّن في NVS |
| `WebDebugServer.h` | نقطة الوصول، والمسارات، وJSON ‏`/api/status`، وصندوق الأوامر؛ ومهمته الخاصة على النواة 0 |
| `WebDashboardPage.h` | HTML/JS للوحة المعلومات في ثابت نصي واحد؛ وتبني المتصفحُ صفوف القنوات والمخارج والحساسات من JSON |
| `OledDisplay.h` | SSD1306 عبر U8g2 فوق `II2CBus`، ومهمته الخاصة (`Rtos`) |
| `MavlinkCodec.h` و`MavlinkTelemetry.h` | MAVLink 2 لـ QGroundControl / Mission Planner: الإطارات، والتدفقات، ومعاملات PID، وتغيير الوضع من الأرض |
| `LoopStats.h` | التردد ومتوسط زمن الدورة وأسوؤه في الثانية (OLED) والأسوأ منذ آخر قراءة (`takePeakUs()`، سطر SYS) |
| `src/main.cpp` | ESP32: إنشاء الكائنات، و`setup()`، و`loop()` مع `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743: الكائنات نفسها، وMAVLink، والصندوق الأسود على بطاقة SD، ومهام `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp` و`src/stm32/bootloader.cpp` | STM32H743: أطراف SDMMC1 وساعاته من أجل `HAL_SD_Init`؛ ومفتاح `D` في الطرفية — إعادة التشغيل إلى محمِّل الإقلاع USB DFU |

---

## اتفاقية الإشارات: من IMU إلى السيرفو

نظام إشارات واحد للسلسلة كلها — ولذلك تحرّك العصا والطيار الآلي أسطح التحكم
حتمًا في الاتجاه نفسه، ويُحدَّد اتجاه كل سيرفو في مكان واحد فقط.

**1. محاور الحساس ← محاور الطائرة.** يدوّر `ImuSensorBase` محاور الشريحة
بالمصفوفة `ImuOrientation` (body = R · chip) إلى محاور الطائرة: X نحو الأنف،
وY إلى اليسار، وZ إلى أعلى. وتؤخذ المصفوفة:

- من **معايرة التركيب** (الأمر `o`، وتُخزَّن في NVS) — فيمكن وضع اللوحة بأي
  اتجاه. ثلاث وضعيات: «أفقية» تعطي المحور Z (والأفق أيضًا — إذ تدخل فيه إزاحة
  الصفر لمقياس التسارع)، و«الأنف إلى أعلى» تعطي المحور X (الجزء من «الأعلى»
  العمودي على Z)، و«الجناح الأيمن إلى أسفل» تعطي المحور Y. ويجب أن يتطابق الأنف
  من الخطوة 2 مع الأنف من الخطوة 3 (Y × Z) بدقة نحو 25°، وإلا فقد مال الطيار
  إلى الجهة الخطأ — فتُرفَض المعايرة؛ والنتيجة هي متوسط التقديرين. جرى التحقق
  على 300 تركيب عشوائي (`test/test_imu_orientation`، والخطأ أقل من 0.1°)؛
- وإلا فمن `Config::IMU_ROTATION_CW_DEG` (اللوحة والشريحة إلى أعلى؛ والقيمة هي
  الجهة التي يشير إليها المحور X *للشريحة* إذا كان الأنف عند «الساعة 12»)،
  ويكون الأفق هو الوضع عند التشغيل.

عند كل معايرة للجيروسكوب (التشغيل، `i`) يجري **فحص ما قبل الطيران**:
ضجيج الجيروسكوب أقل من 0.5 °/s (السكون؛ وفي السكون نحو 0.08)، و|a| ≈ 1g،
و«الأعلى» ضمن 45° من المحفوظ (أي لم تُحرَّك اللوحة). وإن لم ينجح الفحص —
`ImuSensor::getPreflightProblem()` ≠ nullptr: لا يفعّل `ArmingManager` الأوضاع
المستقرة بالتثبيت، و`Autopilot::imuReady()` = false (تصحيحات صفرية في
جميع الأوضاع، بما فيها الانسياب عند فقدان الاتصال).

> في GY-521 الحالية (نسخة مقلَّدة بشريحة MPU6500) لُحمت الشريحة مدارةً بزاوية 90°
> بالنسبة إلى الأسهم المطبوعة: سهم X على الطباعة الحريرية = المحور Y للشريحة.
> لذلك تكون القيمة `IMU_ROTATION_CW_DEG = 90` عند غياب معايرة التركيب. والفحص
> بعد أي تغيير في الوضع: الأنف إلى أعلى ← يزداد P نحو الموجب، والجناح الأيمن
> إلى أسفل ← يزداد R نحو الموجب.

**2. الزوايا والسرعات (`ImuData`) — إشارات الطيران:**

| الكمية | معنى «+» |
|---|---|
| `roll` و`gyroX` | الجناح الأيمن إلى أسفل |
| `pitch` و`gyroY` | الأنف إلى أعلى |
| `yaw` و`gyroZ` | الأنف إلى اليمين (باتجاه عقارب الساعة عند النظر من أعلى) |

**3. الأمر (`ControlCommand`، الانحراف بوحدة µs، و±500 = الشوط الكامل):**

| الحقل | معنى «+» | من العصا |
|---|---|---|
| `roll` | دوران جانبي إلى اليمين (الجنيح الأيمن إلى أعلى والأيسر إلى أسفل) | CH1: ‏2000 = إلى اليمين |
| `pitch` | الأنف إلى أعلى (الرافعة إلى أعلى) | CH2 بإشارة معكوسة: ‏2000 = بعيدًا عنك = الأنف إلى أسفل |
| `yaw` | الأنف إلى اليمين (الدفة وعجلة الأنف إلى اليمين) | CH4: ‏2000 = إلى اليمين |
| `flaps` | الفلابات إلى أسفل (كلا الجنيحين إلى أسفل) | SwB ‏(CH6): ‏0 أو `FLAPS_DEPLOYED_US`، بسلاسة خلال `FLAPS_TRANSITION_MS` |

يحسب PID القيمة `الخطأ = الهدف − الفعلي`: دوران جانبي إلى اليمين (roll أكبر
من 0) ← أمر دوران سالب ← تعتدل الطائرة. وتُضاف تصحيحات الطيار الآلي إلى أمر
العصي **قبل** الخلّاط، وبالإشارات نفسها.

**4. الأمر ← PWM.** تحسب `ControlMixer::mix()` انحراف الحافة الخلفية لكل سطح
(الجنيحات: إلى أسفل = «+»، واليسار = `flaps + roll`، واليمين = `flaps − roll`؛
الرافعة: إلى أعلى = «+»؛ الدفة: إلى اليمين = «+») وتحوّله إلى PWM بالصيغة
`1500 ± الانحراف`، مع قلب الإشارة للسيرفوات التي فيها
`Config::*_REVERSED = true`. وتكرر القيم الافتراضية سلوك البرنامج الثابت
السابق للعصي. أما الفحص على الطائرة المجمَّعة فهو في قائمة ما قبل الطيران في
[`PILOT_GUIDE.md`](PILOT_GUIDE.md). ويجب تغيير العكس في `Config.h`، **لا في جهاز
التحكم** — وإلا اختلفت العصا والطيار الآلي.

---

## خريطة قنوات RC وARM وfailsafe

المصدر هو `include/config/Channels.h`. جهاز التحكم FS-i6 (10 قنوات، النمط 2) +
المستقبِل FS-iA6B، ‏iBUS 115200.

| القناة | عنصر جهاز التحكم | الاسم | الغرض |
|---|---|---|---|
| CH1 | العصا اليمنى ←→ | `AILERON` | الدوران الجانبي |
| CH2 | العصا اليمنى ↑↓ | `ELEVATOR` | الميل |
| CH3 | العصا اليسرى ↑↓ | `THROTTLE` | الخانق، الشوط الكامل؛ أقل من 950 = failsafe المستقبِل |
| CH4 | العصا اليسرى ←→ | `RUDDER` | الدفة + عجلة التوجيه (سيرفو واحد) |
| CH5 | SwA | `ARM` | ‏≥ 1750 = ARM (في FS-i6 هو المفتاح إلى أسفل، نحوك) |
| CH6 | SwB | `SWB` | افتراضيًّا الفلابات (≥ 1750 — مُخرَجة) |
| CH7 | SwC (3 أوضاع) | `SWC` | افتراضيًّا الوضع: أقل من 1250 MANUAL، و1250–1749 STABILIZE، و≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | افتراضيًّا RTH |
| CH9 | VrA | `VRA` | افتراضيًّا قوة التثبيت |
| CH10 | VrB | `VRB` | افتراضيًّا سرعة الانطلاق |

تُسنَد القنوات CH6–CH10 بسطر واحد في `include/config/Controls.h`
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#إسناد-وظيفة-بسطر-واحد)).

**‏ARM** (`ArmingManager`): انتقال المفتاح من OFF إلى ON، والخانق أقل من
`THROTTLE_LOW_US`، واجتياز فحوص الحساسات للوضع الحالي. وإلا يُرفَض الأمر مع
ذكر السبب في Serial، ويلزم دورة OFF→ON جديدة. تشغيل اللوحة والمفتاح في
وضع ON لا يفعّل ARM. وOFF — DISARM فورًا. وما دام الجهاز غير مُفعَّل،
يُفرَض على الخانق المتجه إلى ESC القيمة `PWM_MIN`.

**فقدان الاتصال** (`IBusReceiver::isSignalLost()`):

1. لا إطارات لمدة تزيد على `RX_TIMEOUT_US` (500 ms) — انقطاع سلك أو
   انقطاع التغذية عن المستقبِل. وقبل أول إطار بعد التشغيل يُعدّ الاتصال
   مفقودًا أيضًا: فلا تُؤخذ قيم القنوات الافتراضية (كلها 1500) على أنها
   أوامر من جهاز التحكم.
2. الخانق أقل من `RX_FAILSAFE_THROTTLE_US` (950) — وهو failsafe المضبوط في
   جهاز التحكم. **لا يتوقف FS-iA6B عن إرسال الإطارات عند فقدان جهاز
   التحكم**، بل يكرر آخر القيم (جرى التحقق على المنصة)، ولهذا لا يُكتشف فقدان
   الاتصال دون ضبط failsafe في جهاز التحكم. والضبط موصوف في `PILOT_GUIDE.md`.

ما يحدث عند فقدان الاتصال (`FlightController::applyLinkLoss()`):

- **الطائرة مفعَّلة ويتوفر GPS ونقطة الانطلاق** (`FAILSAFE_RTH`) — **العودة
  إلى نقطة الانطلاق** بالمحرك، والدوران فوقها؛ وعلى OLED — `FSRTH`، وفي
  السجل — `FAILSAFE_RTH`؛
- **الطائرة مفعَّلة ولا يتوفر GPS** — **انسياب**، والمحرك عند
  `FAILSAFE_THROTTLE`: يحافظ `Autopilot` في أي وضع، حتى في MANUAL، على
  الدوران الجانبي `FAILSAFE_GLIDE_ROLL_DEG` (0 — مستقيمًا، و10–20° — دائرة
  فوق الطيار) وعلى الميل `FAILSAFE_GLIDE_PITCH_DEG` (‏−3°، كي لا تفقد
  السرعة دون محرك)، مع سحب الفلابات؛ وعلى OLED — `GLIDE`، وفي السجل — الوضع
  `FAILSAFE_GLIDE`؛
- **غير مفعَّلة** (على الأرض) أو IMU لا تستجيب — تعود الأسطح إلى الوضع
  المحايد؛
- لا يتغير الوضع والوظائف بالمفاتيح، وتستمر قراءة الحساسات. ولا يُلغى ARM —
  فبعد عودة الاتصال تطيع الطائرة العصي والوضع المختار من جديد (أما الإقلاع
  التلقائي والإطلاق باليد فيبدآن من الصفر فقط).

---

## بيانات الحساسات

البنى موجودة في `include/sensors/SensorInterface.h`.

### ‏`ImuData`

| الحقل | الوحدة | المعنى |
|---|---|---|
| `gyroX` و`gyroY` و`gyroZ` | °/s | السرعات الزاوية في محاور الطائرة، بإشارات الطيران (انظر أعلاه) |
| `accelX` و`accelY` و`accelZ` | g | التسارع في محاور الطائرة: X نحو الأنف، وY إلى اليسار، وZ إلى أعلى |
| `roll` و`pitch` | ° | مرشّح تكميلي (α = 0.98، وτ ≈ 0.1 s)؛ ويبدآن مباشرةً من زاوية مقياس التسارع |
| `yaw` | ° | تكامل الجيروسكوب، ينجرف ببطء؛ والقيمة الابتدائية هي اتجاه البوصلة |
| `temperature` | °C | حرارة الشريحة (الصيغة خاصة بـ MPU6050 أو MPU6500) |
| `timestamp` | µs | قيمة `micros()` لحظة القراءة |

معايرة IMU (عند كل تشغيل وبالأمر `i`): سكون لمدة 2 s، الجيروسكوب ←
إزاحة الصفر، ومقياس التسارع ← **يصبح الوضع الحالي هو الأفق**.

### ‏`BarometerData`

| الحقل | الوحدة | المعنى |
|---|---|---|
| `pressure` | Pa | الضغط |
| `temperature` | °C | حرارة الحساس |
| `altitude` | m | الارتفاع **نسبةً إلى نقطة المعايرة** (عند التشغيل)؛ والصيغة `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | مشتقة الارتفاع على العيّنات الفعلية (50 Hz) عبر مرشّح تمرير منخفض بثابت τ = 0.5 s |
| `timestamp` | µs | لحظة آخر عيّنة جديدة |

### ‏`MagData`

| الحقل | الوحدة | المعنى |
|---|---|---|
| `magX` و`magY` و`magZ` | µT | المجال بعد معايرة hard-iron، في محاور الطائرة (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`، دون تعويض الميل؛ ولم يُتحقَّق بعدُ من اتجاه العدّ على طائرة مجمَّعة |
| `timestamp` | µs | لحظة القراءة (50 Hz) |

### ‏`GpsData`

| الحقل | الوحدة | المعنى |
|---|---|---|
| `latitude` و`longitude` | ° | من UBX-NAV-PVT |
| `altitude` | m | فوق سطح البحر (hMSL) |
| `groundSpeed` و`heading` | m/s، ° | السرعة الأرضية والمسار فوق الأرض |
| `numSatellites` و`fixType` | — | 0 = لا تثبيت، و2 = 2D، و3 = 3D |
| `horizontalAccuracy` و`verticalAccuracy` | m | تقديرات الدقة التي تعطيها الوحدة |

**ما معنى `isAvailable()`.** بالنسبة إلى حساسات I2C — أن يكون الحساس قد
ردّ عند `begin()` **وأن** تكون القراءات الأخيرة لا تفشل تباعًا (MPU — نحو 0.1 s،
ومقياس الضغط والبوصلة — نحو 0.5 s دون رد). وعند فشل القراءة لا تُمحى البيانات
بقيم عشوائية: تبقى القيم السابقة ويزداد عدّاد الأخطاء (ويظهر بالأمر `s`).
أما GPS فيلزم NAV-PVT صالح واحد على الأقل، وألا يكون الأخير أقدم من
`GPS_TIMEOUT_US`.

**إذا لم يوجد الحساس** (`nullptr` أو `isAvailable() == false`) فلا يعطي
`Autopilot` أي تصحيحات، وتُقاد الطائرة كما في MANUAL. ولا يعاير `main.cpp`
سوى الحساسات التي ردّت.

---

## تفصيل FlightController::update()

تُستدعى من `loop()` كل 2 ms. والترتيب هو الأولوية:

1. **‏`receiver.update()`** — تحليل بايتات iBUS المتراكمة.
2. **المفاتيح** — `switches->update(rc)`، فقط والاتصال قائم (في إطار failsafe
   لا تعكس القنوات حالة المفاتيح): الوضع (عند التغيّر فقط)، والوظائف،
   والمقابض.
3. **خانق الطيار** — `throttle.update(rc, receiverFailsafe)`.
4. **العصي** — `mixer.fromSticks(rc)` × `Knob::RATES`؛ والفلابات —
   `mixer.updateFlaps(target)` (المكبح، والمفتاح، والمقبض؛ وعند انعدام الاتصال
   — 0).
5. **الحساسات والطيار الآلي** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **دائمًا**، حتى دون اتصال: فلا ينبغي أن تتجمد مرشّحات الزوايا. وما دام
   الجهاز غير مُفعَّل يعمل PID (تستجيب الأسطح للميلان — وهذا مفيد على
   المكتب)، لكن المكامِل يبقى عند الصفر. ودون اتصال مع ARM — failsafe RTH أو
   الانسياب.
6. **الصافرة** — `Beeper`.
7. **فقدان الاتصال** — `applyLinkLoss()`: مع ARM — تتبع الأسطح والخانق أمر
   failsafe الصادر عن الطيار الآلي، وإلا فالوضع المحايد والمحرك مطفأ؛ ثم
   `return`. أولوية مطلقة على كل ما يلي.
8. **‏ARM** — `arming.update(rc, false)`.
9. **الأمر** — `autopilot->getCommand()`: في الأوضاع المستقرة بالتثبيت تمثّل
   العصا الزاوية المطلوبة، ويُصدر الطيار الآلي أوامر الأسطح النهائية.
10. **الخلّاط** — `mixer.mix(command)` ← PWM للجنيحات (الفلابات + الدوران
    الجانبي) والرافعة والدفة، مع مراعاة العكس.
11. **الخانق** — `autopilot->applyThrottle(pilotThrottle)`: خانق الطيار، أو
    خانق الطيار الآلي، أو الأكبر من الاثنين (الإقلاع التلقائي). ثم إذا لم يكن
    الجهاز مُفعَّلًا أو كان `MOTOR_KILL` — يُفرَض `PWM_MIN`. ويأتي هذا الفحص
    في الآخر كي لا يتمكن أي وضع من تمرير الخانق متجاوزًا ARM.
12. **‏AUX** — الحمولة (`PAYLOAD_DROP`) والكاميرا (`CAMERA_TILT` و`CAMERA_STAB`).
13. **‏`outputs.write(output)`** — PWM على 7 مخارج.

---

## واجهة HTTP البرمجية للوحة المعلومات عبر الويب

التنفيذ في `include/telemetry/WebDebugServer.h`. نقطة الوصول: SSID
بالاسم `OpenPlane-Debug`، وكلمة المرور `12345678`، والعنوان `http://192.168.4.1`.

### ‏`GET /api/status`

<div dir="ltr">

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

</div>

- ‏`attached` — الكائن موجود في البناء؛ و`available` — الحساس يستجيب فعلًا.
  وتُضاف حقول البيانات **فقط** عندما تكون `available: true`.
- ‏`outputs.*.attached` — خصّصت وحدة التحكم قناة LEDC وطرفًا؛ أما هل
  السيرفو الفعلي موصول فلا يظهر من البرنامج (ولفحص النبضة — الكونسول،
  الأمر `p`).
- ‏`rollCorr`/`pitchCorr` — الأمر النهائي للطيار الآلي مطروحًا منه العصي، بوحدة
  µs. و`throttleCorr` — خانق الطيار الآلي، بالنسبة المئوية (0 ما دام الخانق
  بيد الطيار).
- ‏`nav` — الملاحة: نقطة الانطلاق، والمسافة إليها واتجاهها، والمسار والمسار
  المستهدف، والسرعة المستخدمة في الملاحة (أنبوب بيتو / GPS)، والسياج
  الجغرافي، والانهيار؛ و`features` — وظائف المفاتيح المفعّلة.

### ‏`POST /api/setmode`

‏`{ "mode": 1 }` — رقم `AutopilotMode`: ‏`0` MANUAL، و`1` STABILIZE، و`2`
AUTO_TAKEOFF، و`3` ALT_HOLD، و`4` ACRO، و`5` CRUISE، و`6` LOITER، و`7` RTH، و`8`
LAUNCH، و`9` AUTO_LAND، و`10` SOARING، و`11` RESCUE. ويبقى الوضع قائمًا
حتى يقلب الطيار مفتاح الوضع.

### ‏`POST /api/setpid`

‏`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — أي من الحقول `kpRoll` و
`kiRoll` و`kdRoll` و`kpPitch` و`kiPitch` و`kdPitch`؛ وتبقى المحذوفة على
قيمها السابقة.

يطبّق كلا الأمرين حلقةُ الطيران في الدورة التالية (انظر
[مهام FreeRTOS](#مهام-freertos-وحلقة-التحكم)).

### ‏`GET /`

لوحة المعلومات بصيغة HTML: أشرطة القنوات الـ 10، وARM/الاتصال، والمخارج،
والحساسات، وأزرار الأوضاع، ونموذج PID. وتستطلع `/api/status` كل 200 ms.

---

## الكونسول والتشخيص

مراقِب المنفذ — 115200، الموصل «COM». التنفيذ في `DebugConsole` و
`DebugLogger` ([المرجع](reference/telemetry.md)). تعمل المفاتيح فورًا، ولا
حاجة إلى Enter؛ أما المعايرات والأمر `p` فتحجز الحلقة، ولذلك لا تتاح إلا دون ARM.

| المفتاح | ماذا يفعل |
|---|---|
| `h` / `?` | القائمة الرئيسية |
| `l` | قائمة «ما الذي يُكتب في السجل» (القنوات، والأوضاع، والدورة) |
| المسافة | إيقاف السجل مؤقتًا / استئنافه |
| `s` | `printStatus()` لجميع الحساسات: البيانات، وعدّادات أخطاء الناقل، والمعايرات، وفحص ما قبل الطيران |
| `i` | معايرة الجيروسكوب + فحص ما قبل الطيران (2 s سكون) |
| `o` | معايرة تركيب IMU بثلاث وضعيات، وتُحفظ في NVS |
| `m` | معايرة البوصلة (15 s من التدوير)، وتُحفظ في NVS |
| `p` | الفحص الذاتي للمخارج: النبضة الفعلية على كل طرف مقابل المتوقعة |

يُقسَّم السجل إلى قنوات (`STAT` و`RC` و`OUT` و`ATT` و`AP` و`ALT` و`MAG` و`GPS` و
`IMU` و`SYS`)، ولكل قناة وضع «مُعطَّل / عند التغيّر / دائم»؛ وتُخزَّن الإعدادات في
NVS وتُكتب عند إغلاق القائمة، ودون ARM فقط. ويُفعَّل افتراضيًّا `STAT` (عند
التغيّر) و`SYS` (مرة كل 10 s):

<div dir="ltr">

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (الأسوأ خلال 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

</div>

صيغ جميع القنوات في [المرجع](reference/telemetry.md#debuglogger).

---

## اختيار اللوحة وتوزيع الأطراف

| الأمر | `board` | الماكرو | الحالة |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (‏`qio_opi`، ‏16 MB) | `BOARD_ESP32_S3` | **الرئيسية، وهي الافتراضية.** جرى اختبارها على المنصة مع جميع الحساسات |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | النموذج الأولي القديم، طار بالتحكم اليدوي |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | للمنصة، ولم يُختبر توزيع الأطراف على العتاد |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: البرنامج الثابت الكامل + MAVLink + الصندوق الأسود على SD؛ اختُبرت على لوحة عارية ([أدناه](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | الشيء نفسه على DevEBox H743: الكونسول عبر USB CDC، ورفع البرنامج الثابت عبر DFU |

| الغرض | ESP32-S3 (المنصة) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| الجنيح الأيسر / الأيمن | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| الرافعة / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| الدفة | GPIO18 | — (لا طرف) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| I2C الحساسات SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| I2C شاشة OLED ‏SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI ‏SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI ‏CS ‏ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS ‏RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / لا يوجد (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 ← الموصل «COM» | USB-CDC | UART0 |

- **‏ESP32-S3 N16R8:** الأطراف GPIO33–37 تشغلها PSRAM الثمانية، و26–32 تشغلها
  الفلاش، و19/20 يشغلها USB، و43/44 يشغلها Serial، والطرف 48 هو الصمام الثنائي
  RGB؛ أما 0/3/45/46 فأطراف strapping.
- **‏ESP32-C3:** الجنيحان على GPIO4/5 مبدَّلان قياسًا إلى S3. ولا تكفي الأطراف
  للمجموعة الكاملة: طرف CS لـ BMP388 وطرف RX لـ GPS على أطراف strapping،
  وGPS بلا TX (استقبال فقط، دون UBX-CFG). والتفاصيل في `Config.h`.

### ‏STM32H743

تشغّل STM32H743VIT6 (النواة Cortex-M7 بتردد 480 MHz، وفلاش 2 MB، وذاكرة
RAM بسعة 1 MB) **البرنامج الثابت الكامل**: الحساسات نفسها والطيار الآلي
والمفاتيح والكونسول والشاشة كما في ESP32-S3، إضافةً إلى القياس عن بُعد
MAVLink وصندوق أسود على بطاقة SD. وهو يُبنى، ويجتاز cppcheck وجميع
الاختبارات الأصلية للشيفرة المشتركة. وعلى العتاد اختُبرت **لوحة DevEBox H743
دون حساسات**: الإقلاع، والكونسول عبر USB، وبطاقة SD، والصندوق الأسود —
[TESTING.md](TESTING.md#الاختبارات-على-لوحة-stm32) — وكذلك iBUS وARM وPWM إلى
السيرفوات والمحرك: التحكم من جهاز التحكم في الوضع اليدوي (موثَّق في فيديو).
أما الحساسات على STM32 فما زالت تنتظر منصة. واللوحة الرئيسية للطيران هي
ESP32-S3.

- **‏HAL** — `include/hal/stm32/`: ‏`Stm32Board` (واجهة API نفسها التي لـ
  `Esp32Board`، إضافةً إلى `telemetryUart()`) و`Stm32I2CBus` و`Stm32SpiBus` و
  `Stm32UartPort` و`Stm32ServoOutput` (‏PWM عتادي عبر `HardwareTimer`، ومؤقت
  واحد لعدة مخارج). بالتفصيل — [reference/hal.md](reference/hal.md#التنفيذ-لمعالج-stm32h743).
- **الإعدادات والمعايرات** — لا NVS، بل `KeyValueStore` في آخر قطاع من الفلاش
  (`include/storage/`، و`hal/stm32/Stm32FlashStorage.h`). وما زالت شيفرة
  المشروع تكتب `#include <Preferences.h>`: ففي env ‏`stm32h743` يقع الدليل
  `include/hal/stm32/compat/` ضمن `-I`، وفيه `Preferences` بواجهة API نفسها.
  والصورة مزوَّدة بـ CRC32: فالصورة التالفة (انقطعت الطاقة أثناء المسح) تُقرأ
  فارغة. وتجري الكتابة في الفلاش في مهمة خلفية: يستغرق مسح قطاع بحجم 128 KB
  ثواني، لكن القطاع في المصرف 2 والشيفرة تُنفَّذ من المصرف 1، وتزيح مهمة الطيران
  المهمة الخلفية دون توقف.
- **المهام** — FreeRTOS من مكتبة STM32duino FreeRTOS، نواة واحدة، مع الاستباق
  بحسب الأولوية (`hal/Rtos.h`): ‏`flight` ‏(5) — حلقة الطيران وMAVLink والسجل
  والكونسول؛ و`oled` ‏(1) و`storage` ‏(1) — في الخلفية؛ و`bbox` ‏(2) — كتابة
  الصندوق الأسود على بطاقة SD.
- **الصندوق الأسود على بطاقة SD** — SDMMC1، ‏4 بت، ‏24 MHz
  (`hal/stm32/Stm32SdCard.h`، والأطراف في `src/stm32/sd_msp.cpp`). تبقى
  البطاقة FAT32 عادية: عليها ملف `BLACKBOX.BIN` أُنشئ مسبقًا، ويكتب البرنامج
  الثابت داخله كتلًا خامًّا ولا يمس نظام الملفات نفسه (`storage/Fat32File.h` —
  للقراءة فقط). تجهيز البطاقة وتنزيل البيانات — [BLACKBOX.md](BLACKBOX.md#بطاقة-sd-stm32h743).
- **القياس عن بُعد** — MAVLink 2 على UART4 (`telemetry/MavlinkTelemetry.h`)
  بدلًا من لوحة Wi-Fi: QGroundControl / Mission Planner، وتغيير الوضع وPID من
  الأرض. بالتفصيل —
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#محطة-التحكم-الأرضية-لوحة-wi-fi-وmavlink).
- **توزيع الأطراف** — كتلة `BOARD_STM32H743` في `Config.h`، والأطراف مختارة من
  الأطراف الحرة على WeAct MiniSTM32H743VITx ومطابَقة مع جداول STM32duino:

| الغرض | STM32H743 | الطرفية |
|---|---|---|
| الجنيح الأيسر / الأيمن | PA0 / PA1 | TIM2_CH1 / CH2 |
| الرافعة / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| الدفة | PD14 | TIM4_CH3 |
| AUX1 (الحمولة) / AUX2 (الكاميرا) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (وTX احتياطي) | PE7 (PE8) | UART7 |
| I2C الحساسات SDA / SCL | PB11 / PB10 | I2C2 |
| I2C شاشة OLED ‏SDA / SCL | PB9 / PB8 | I2C1 |
| SPI ‏SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI ‏CS ‏IMU / مقياس الضغط | PB12 / PD10 | GPIO |
| GPS ‏RX / TX | PD9 / PD8 | USART3 |
| مودم الراديو MAVLink ‏RX / TX | PD0 / PD1 | UART4 |
| الصافرة | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **‏DevEBox H743 (MCUDEV)** — env ‏`stm32h743-devebox`: الشيفرة نفسها، ولها
  نسختها الخاصة من النواة، والكونسول عبر USB-C كمنفذ COM افتراضي (CDC) — فلا
  حاجة إلى USB-UART. ويجري رفع البرنامج الثابت أول مرة عبر USB بمحمِّل الإقلاع
  المدمج (DFU):
  1. ‏Windows: ثبّت مرة واحدة برنامج تشغيل WinUSB لـ «STM32 BOOTLOADER»
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver).
  2. صِل الطرف **BT0** (‏BOOT0) بالطرف **3V3** بسلك، واضغط **RST** ثم أفلته:
     تصبح اللوحة في وضع DFU (لا يوجد على DevEBox زر BOOT0).
  3. ‏`pio run -e stm32h743-devebox -t upload` (‏`upload_protocol = dfu`).
  4. يمكن نزع سلك BT0 — يبدأ البرنامج الثابت من تلقاء نفسه.

  بعد ذلك لا حاجة إلى السلك: المفتاح **`D`** في الكونسول (من أي قائمة، وليس أثناء
  ARM) يعيد تشغيل اللوحة إلى محمِّل الإقلاع: علامة في RAM ← إعادة ضبط ← قفزة
  إلى ذاكرة النظام قبل ضبط الساعات (`src/stm32/bootloader.cpp`). والقفز مباشرةً من
  البرنامج الثابت العامل يتجمد على H7 — تحققنا من ذلك على اللوحة، ولهذا جاءت
  الخطوتان. ويلزم كونسول مفتوح (USB CDC)؛ وإن لم تستجب اللوحة — فاضغط RST
  مع بقاء سلك BT0.
- **نقطة الدخول** — `src/stm32/main.cpp` (مستبعَد من بناءات ESP32 عبر
  `build_src_filter`). والكائنات هي نفسها الموجودة في `src/main.cpp`؛ وبدلًا من
  `loop()` توجد مهام، ويقع `vTaskStartScheduler()` في آخر `setup()`.
- **أول تشغيل للوحة:** `pio run -e stm32h743 -t upload` (ST-Link)، والمراقِب
  على LPUART1 عبر USB-UART؛ و`b` — هل تظهر الحساسات على النواقل، و`s` — حالة
  الحساسات، و`p` — النبضات على المخارج (انزع المروحة)، ثم جهاز التحكم
  وQGroundControl عبر مودم الراديو.

---

## كيف تضيف حساسًا جديدًا

### ‏A) شريحة أخرى من فئة موجودة (IMU أو مقياس ضغط أو بوصلة)

الجزء المشترك مكتوب أصلًا في الأصناف الأساسية — فيخرج مشغِّل الشريحة صغيرًا:

1. أنشئ `include/sensors/<category>/<Name>_Sensor.h` وارِث من
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. يتلقى المُنشئ
   `IRegisterDevice&` — فالمشغِّل لا يعرف أهو I2C أم SPI.
2. نفِّذ:
   - ‏`begin()` — ‏`device.begin()`، والتحقق من معرّف الشريحة، وكتابة السجلات،
     واستدعاء `setAvailable(true/false)`؛
   - ‏IMU: ‏`readSample()` (قيم accel/gyro/temp الخام في محاور الشريحة)،
     و`accelLsbPerG()` و`gyroLsbPerDps()` و`temperatureC()`؛
   - مقياس الضغط: ‏`isNewSampleReady()` (علَم الجاهزية أو `true` ببساطة)
     و`readSample()` (الضغط بوحدة Pa والحرارة بوحدة °C)، وفترة الاستطلاع في
     مُنشئ الأساس؛
   - البوصلة: ‏`readRaw()` (‏X/Y/Z في محاور الشريحة) و`lsbPerMicroTesla()`،
     واسم مساحة أسماء NVS الخاصة بالمعايرة في مُنشئ الأساس.
3. إذا احتاجت الشريحة عبر SPI إلى بايت وهمي قبل البيانات أو إلى تردد خاص —
   أضف مصنعًا ساكنًا `spiDevice(bus, cs)`، كما في `BMP388_Sensor`.
4. فرع في `SensorSelection.h`: ‏`#define SENSOR_<CATEGORY>_<NAME>`،
   و`using Selected... = ...;` و`#define SELECTED_..._DEVICE(board) ...`
   (‏`I2cRegisterDevice(board.i2c(), address)` أو مصنع SPI). ولا يُمَسّ `main.cpp`
   عند تغيير الحساس.
5. تحقق من البناء مع الحساس الجديد دون تعديل الملف — بخيار بناء:
   `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`، ثم
   البيئات الثلاث جميعها، ثم على العتاد.

### ‏B) فئة جديدة

1. بنية البيانات والواجهة في `SensorInterface.h` على غرار `GpsSensor`/`GpsData`.
2. إذا كان للفئة منطق مشترك (مرشّحات، معايرة) — صنف أساسي على غرار
   `BarometerBase`.
3. مؤشر يقبل القيمة الفارغة في مُنشئ `Autopilot` (بلا حساس — بلا أي أثر، لا
   انهيار) وحقول في `GET /api/status` مع الزوج `attached`/`available`.

### ناقل أو طرفية جديدة

واجهة جديدة في `include/hal/`، وتنفيذ في `include/hal/esp32/` وفي
`include/hal/stm32/`، والوصول عبر `IBoard`.

---

## كيف تضيف وضعًا جديدًا للطيار الآلي

1. قيمة في `enum AutopilotMode` (‏`autopilot/AutopilotTypes.h`، قبل
   `MODE_COUNT`)، واسم واسم قصير (حتى 5 أحرف، لشاشة OLED) في
   `AutopilotNames::mode()` / `modeShort()`.
2. معالج `run<Mode>()` وفرع في `Autopilot::runMode()`؛ والأهداف الابتدائية
   (المسار، والارتفاع، ومركز الدوائر) في `initializeMode()`. يضبط الوضع
   `desiredRoll`/`desiredPitch` ويستدعي `stabilizeOrManual()` (دون IMU تكون
   الأسطح بيد الطيار) أو `stabilizeOrNeutral()` (دون IMU — الوضع المحايد). ودون
   الحساس اللازم — سلوك آمن، لا انهيار. ولا يتراكم المكامِل إلا عند `armed`.
3. الخانق: ‏`throttleMode` (‏`PILOT` / `AUTO` / `AT_LEAST`) و`autoThrottlePct`،
   أو `autoThrottle()` — خانق الانطلاق من المقبض / من أنبوب بيتو. ولا يتغير
   `FlightController` في ذلك.
4. على جهاز التحكم — سطر واحد في `config/Controls.h`
   (‏`Bind::mode(Channels::SWD, MODE_NEW)`). تلتقط لوحة المعلومات وMAVLink الوضع
   برقمه؛ وبالنسبة إلى MAVLink — أقرب وضع في ArduPlane في
   `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. إذا احتاج الوضع إلى حساسات من أجل ARM — `ArmingManager`.
6. الاختبارات: الاستجابة لكل حساس — `test/native/test_autopilot_modes`،
   والطيران بحلقة مغلقة — سيناريو في `test/native/test_sim` (نموذج الطائرة
   `helpers/PlaneSim.h`، والمنصة `helpers/SimHarness.h`). ثم — المكتب دون
   مروحة: يجب أن تستجيب الأسطح للميلان في اتجاه التسوية.
7. قسم في [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## التغذية الراجعة (تمهيد، غير موصولة)

‏`include/autopilot/feedback/` هي الخطوة التالية للطيار الآلي. **لا
`FlightController` ولا `Autopilot` ولا `main.cpp` يضمّن هذه الملفات:**
لا يوجد بعدُ نموذج أولي للاختبارات الطيرانية، ويعمل البرنامج الثابت من دونها.
وتُختبر بمحاكاة بحلقة مغلقة (`test/test_feedback/`) على اللوحة مباشرةً.

### لماذا

‏`Autopilot` اليوم هو PID على الزاوية: الخطأ × المعامل = السطح. وهو لا يعرف
ما الذي نتج عن ذلك في الطائرة، والمعاملات صحيحة لسرعة واحدة فقط: عند السرعة
المنخفضة يكون السطح أضعف فيصحّح PID أقل من اللازم، وعند السرعة العالية يفرط
في التصحيح. وتغلق التغذية الراجعة الحلقة على **استجابة الطائرة**:

- انحرف السطح لكن الطائرة تدور أبطأ من المطلوب — فيُزاد حتى تبلغ المطلوب؛
- مقدار السطح المطلوب يُقاس في الطيران ويُعاد حسابه بحسب السرعة؛
- الطائرة تدور في الاتجاه الخطأ — اختلطت الإشارة، فتُعكس ويُتحقق؛
- سُوّيت الزاوية لكن السرعة تتناقص — يُزاد الخانق ويُخفض الأنف حتى لا تنهار
  الطائرة؛
- الإقلاع والهبوط — على مراحل بحسب ما تُظهره الحساسات.

### الوحدات

| الملف | ماذا يفعل |
|---|---|
| `FlightSnapshot.h` | كل ما تعرفه التغذية الراجعة عن الطائرة في دورة واحدة. وهو المدخل الوحيد: فالوحدات لا تقرأ الحساسات وRC مباشرةً، ولذلك يمكن تشغيلها على المحاكاة وعلى السجلات |
| `FeedbackOutput.h` | مخرج الدورة الواحدة: انحرافات الأسطح بحسب المحاور، وهل المحور مفعَّل، وإشارة المحور، والخانق (تحديد / ليس أقل من)، والسبب |
| `FeedbackConfig.h` | جميع الثوابت (ستنتقل إلى `Config.h` عند الربط) |
| `SpeedEstimator.h` | السرعة (أنبوب بيتو > GPS) والتسارع الطولي من IMU: `dV/dt = g·(ax − sin θ)` — فتظهر عبارة «السرعة تتناقص» حتى دون حساس سرعة |
| `AirborneDetector.h` | في الجو / على الأرض: فلا معنى للتعلّم وتراكم التكامل والبحث عن الانهيار إلا أثناء الطيران |
| `ControlEffectivenessEstimator.h` | يتعلّم لكل محور النموذج `ε = b·u(t−delay) + a·ω + c` بطريقة المربعات الصغرى التكرارية |
| `AdaptiveRateController.h` | سلسلة متعاقبة: الزاوية ← السرعة الزاوية ← التسارع الزاوي ← السطح عبر النموذج المتعلَّم |
| `StallGuard.h` | الحماية من فقدان السرعة والانهيار |
| `TakeoffSequencer.h` و`LandingSequencer.h` و`PhaseTargets.h` | الإقلاع (من مدرج أو باليد) والهبوط على مراحل بحسب الحساسات |
| `FeedbackSupervisor.h` | كل شيء معًا: الترتيب في الدورة، والأولويات، و`requestTakeoff()`/`requestLanding()`/`cancelPhase()`، و`printStatus()`، وخطة الربط |
| `FeedbackModules.h` | ‏include واحد لكل شيء |

### كيف يعمل

**فعالية الأسطح.** نموذج المحور: التسارع الزاوي `ε = b·u + a·ω + c`. القيمة
`b` هي عدد °/s² الذي يعطيه 1 µs من السطح (والإشارة هي اتجاه الاستجابة)، و`a`
هو التخميد (يكبح الهواء الدوران؛ ومن دون هذا الحد كان تقدير `b` سيتجه إلى
الصفر عند الدوران المستقر)، و`c` عزم ثابت (موضع مركز الثقل، والمعدِّل،
والمروحة). وقوة السطح ∝ ρV²، لذلك يُتعلَّم `b` عند سرعة مرجعية ويُضرب في
`(V/Vref)²`: تسارعت الطائرة — فـ«يقوى» السطح فورًا دون إعادة تعلّم. والسرعة
المؤشَّرة من أنبوب بيتو تتضمن كثافة الهواء أصلًا، فيؤخذ الارتفاع في الحسبان
تلقائيًّا؛ ودون حساس سرعة يكون المقياس 1 ويُتعلَّم `b` مباشرةً.

تؤخذ البيانات على فترات مدة كل منها 20 ms: متوسط التسارع في الفترة هو فرق
الجيروسكوب عند الطرفين / المدة، ويقابله متوسط السطح ومتوسط السرعة الزاوية
في الفترة نفسها (السطح — مع التأخير `RESPONSE_DELAY_MS`). ثم يمر طرفا
المعادلة عبر مرشّح التمرير المنخفض نفسه بتردد 2 Hz: فلا تتغير النسبة، بينما
تزول الترددات العالية التي يخطئ فيها نموذج «التأخير الصرف» بسبب قصور السيرفو.
ولا يمكن التعلّم إلا في الجو وإلا عند «تحريك» السطح (سعة ≥
`MIN_EXCITATION_US` خلال نحو 0.3 s)؛ وعصي الطيار تحرّك أيضًا، ولذلك يتعلّم
التقدير حتى في MANUAL.

**المنظِّم.** ثلاث مراحل، محورًا محورًا:

<div dir="ltr">

```
ω* = ANGLE_GAIN · (target − angle)              "الأنف منخفض 10° — ارفعه بسرعة 40°/s"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "يدور أبطأ من المطلوب — صحّح"
surface = (ε* − a·ω − c) / b                    عبر النموذج المتعلَّم
```

</div>

يُحفظ التكامل `I` بوحدة °/s لا بوحدة µs للسطح — ولذلك يبقى صحيحًا عندما يتغير
تقدير `b`. وعلى الأرض يكون التكامل مجمَّدًا (عدا المسار في شوط الإقلاع
والهبوط)، وعند بلوغ السطح حدَّه لا يتراكم في اتجاه الحد. ويؤخذ الانعطاف
المنسَّق في الحسبان (إذا كانت السرعة معلومة): ففي الميلان يلزم ميل مقداره
`g·sin φ·tg φ / V` وانعراج مقداره `g·sin φ / V`.

**إشارات المحاور — على الأرض فقط.** في الطيران لا تُعطَّل المحاور ولا
تُعكس: فتركيب IMU تحدده المعايرة `o` والفحص عند التشغيل، واتجاهات الأسطح يحددها
فحص الطيار قبل الطيران. والمؤشرات غير المباشرة في الجو (الانهيار، والتدوّم،
والحركات البهلوانية، والهبّات) قد تخدع، وتعطيل محور أو عكسه في لحظة كهذه
يكلّف الطائرة. وإذا كان تقدير `b` لمحور ما سالبًا بثقة، فهذا مجرد تحذير في
`reason` («تستجيب للسطح بالعكس؟ تحقق على الأرض»)؛ ولا يدخل التقدير السالب
في المنظِّم — ويعمل المحور وفق النموذج المسبق.

**الحماية من الانهيار.** مستويان. *LowEnergy* — السرعة تتناقص بسرعة مع رفع
الأنف، أو قريبة من الانهيار (< 1.25·Vs)، أو فقدت الرافعة فعاليتها: الخانق
≥ 80 %، والميل ≤ 5°. *Stall* — السرعة أدنى من سرعة الانهيار، والأنف يهبط
ضد الرافعة، والجناح يهوي ضد الجنيحات عند طاقة منخفضة: خانق كامل، وأنف إلى
أسفل، وميلان ≤ 10°، وجنيحات محدودة (فالجنيح الكبير يسبب انهيار طرف الجناح).
وتُرفع التدابير بتخلّف (سرعة ≥ 1.5·Vs). وعند فقدان الاتصال لا يُمَسّ
الخانق، وقرب الأرض مباشرةً (التسوية، شوط الهبوط) تكون الحماية معطّلة —
فالهبوط نفسه انهيار مُتحكَّم به.

**الإقلاع.** `WaitThrottle` (المحرك متوقف) ← أعطى الطيار خانقًا ≥ 50 % ←
`GroundRoll` (خانق كامل، والجناحان مستويان، ويحافظ على المسار الدفة
والعجلة، والرافعة حرة) ← سرعة الإقلاع أو انتهاء المهلة دون حساس سرعة ←
`Climb` (‏12°، خانق كامل) ← ارتفاع 30 m ← `Complete`. وباليد
(`TAKEOFF_HAND_LAUNCH`) يحلّ `WaitLaunch` محل شوط الإقلاع: يبدأ المحرك بعد
الرمي فقط (تسارع طولي ≥ 1g). وإذا سُحب الخانق قبل الإقلاع — إلغاء.

**الهبوط.** `Approach` (خانق 25 %، ونزول 1 m/s — الميل من خطأ السرعة الرأسية،
والميلان من الطيار ≤ 20°) ← ارتفاع 2 m ← `Flare` (خانق 0، ويُخمَّد النزول إلى
0.3 m/s بالقاعدة نفسها) ← صدمة على مقياس التسارع أو «منخفض ولا يدور» ←
`Rollout` (المسار بالعجلة) ← `Complete`. خانق الطيار ≥ 80 % — إعادة المحاولة
بالإقلاع من جديد. وتحتاج التسوية إلى مقياس مدى: فمقياس الضغط يخطئ بمتر.

**الأولويات** (`FeedbackSupervisor`): غير مُفعَّل > الحماية من الانهيار >
الإقلاع/الهبوط > أهداف الوضع. وعند فقدان الاتصال تُلغى المراحل، وينفذ التثبيت
أهداف الانسياب في failsafe.

### المحاكاة

‏`test/test_feedback/test_main.cpp` (على الحاسوب: `pio test -e native -f test_feedback`) — نموذج لطائرة (محاور مستقلة،
وتأخير السيرفو وقصوره، وفعالية الأسطح ∝ V²، والتخميد ∝ V، وعزوم
ثابتة، والرفع عبر زاوية الهجوم بحسب السرعة، والانهيار، وعجلات الهبوط ذات
عجلة التوجيه) و10 سيناريوهات:

| السيناريو | ما الذي يُفحص |
|---|---|
| الخروج من ميلان 30° / ميل −15° مع عزم ثابت | التسوية و«واصل التصحيح»: يجد التكامل المعدِّل بنفسه |
| اهتزاز ±15° عند 14 و20 m/s، دون حساس سرعة | يتقارب تقدير `b` من الحقيقة ويُعاد حسابه بحسب السرعة |
| جنيح معكوس، والطيار يهزّ الجناحين في MANUAL | تقدير `b` سالب ← تحذير فقط، ولا يُعطَّل المحور |
| اضطراب جوي لمدة 30 s | تُصدّ الهبّات ولا يتجاوز الميلان 10° |
| أنف 15° عند خانق 20 % (مع حساس سرعة ودونه) | لا تنخفض السرعة إلى حد الانهيار |
| إقلاع من مدرج مع عزم رد فعل المروحة | المراحل، والارتفاع، والمسار في شوط الإقلاع |
| هبوط من 15 m | المراحل، ولا خانق قرب الأرض، ولمسة ناعمة |
| فقدان الاتصال في شوط الإقلاع؛ غير مفعَّل؛ MANUAL | الإلغاء، ولا يُمَسّ الخانق، وتبقى الأسطح بيد الطيار |

النموذج تقريبي — فهو يفحص المنطق والإشارات، لا الضبط لهيكل
طائرة بعينه.

<div dir="ltr">

```bash
pio test -e native -f test_feedback      # على الحاسوب، في ثوانٍ
pio test -e esp32-s3 -f test_feedback    # يرفع برنامج الاختبار الثابت ويشغّله
pio run -t upload                        # إعادة البرنامج الثابت المعتاد
```

</div>

### خطة الربط

1. تملأ `FlightController::update()` بعد قراءة الحساسات وحساب الأوامر بنية
   `FlightSnapshot` وتستدعي `FeedbackSupervisor::update()`. في البداية —
   **الوضع الظلي**: يذهب المخرج إلى السجل (`printStatus()`) ولوحة المعلومات
   فقط، ولا يصل إلى الأسطح. وفي الطيران بالتحكم اليدوي يجب أن يكون تقدير `b`
   لكل محور موجبًا ويزداد مع السرعة.
2. على الأرض، والطائرة في اليدين، STABILIZE: أمِلها — فتقاوم الأسطح.
3. محورًا واحدًا في كل مرة: `deflectionUs` بدلًا من
   `Autopilot::getRollCorrection()` (الدوران الجانبي أولًا فقط)، ثم الميل.
4. الخانق: `throttleOverridePercent`/`throttleFloorPercent` — بعد
   `Autopilot::applyThrottle()` وقبل failsafe (فـfailsafe فوق كل شيء).
5. الإقلاع/الهبوط — على مفتاح حر؛ وأزِل الوضع `AUTO_TAKEOFF` من
   `Autopilot`.
6. ثوابت `FeedbackConfig` — إلى `Config.h`؛ وحساس السرعة الجوية — تنفيذ
   `AirspeedSensor` وفئة في `SensorSelection.h`.

---

## كيف تضيف لوحة جديدة

1. ‏`[env:<name>]` في `platformio.ini` مع `-D BOARD_ESP32_<NAME>` فريد.
2. كتلة `#elif defined(BOARD_ESP32_<NAME>)` في `Config.h` بجميع الأطراف،
   بما فيها `PIN_I2C2_SDA/SCL` (القيمة −1 إن لم توجد شاشة OLED). احسب ميزانية
   GPIO مسبقًا: flash/PSRAM/USB/strapping.
3. تحتاج مخارج السيرفو إلى 5 قنوات LEDC — وهي موجودة في كل ESP32. وإن لم يكن
   للدفة طرف — `PIN_RUDDER = -1`، ويُعطَّل المخرج ببساطة.
4. لا تغيّر `default_envs` قبل اختبار اللوحة على العتاد؛ ونصّ صراحةً في
   الـ commit إذا لم يُختبر توزيع الأطراف.

---

## أوامر البناء والرفع والمراقبة

<div dir="ltr">

```bash
pio run                        # بناء اللوحة الافتراضية (esp32-s3)
pio run -t upload              # الرفع
pio device monitor             # المراقِب، 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # التحقق من أن جميع اللوحات تُبنى
```

</div>

- **‏ESP32-S3:** الرفع وSerial عبر الموصل «COM» (‏CH343). وإذا تعطّل الجسر
  (يردّ Windows بعبارة «الجهاز لا يعمل» — وقد يحدث ذلك بسبب تشويش من ESC)
  فإعادة توصيل الكبل تفيد؛ ويمكن الرفع أيضًا عبر الموصل «USB» (‏USB-JTAG
  المدمج): `pio run -t upload --upload-port <USB COM port>`.
- ما دام مراقِب المنفذ مفتوحًا فلن ينجح الرفع إلى المنفذ نفسه.
- ‏`lib_deps`: ‏`olikraus/U8g2` (شاشة OLED) هي المكتبة الخارجية الوحيدة.
- ‏`test/` — بالتفصيل في [`TESTING.md`](TESTING.md):
  - ‏`pio test -e native -e native-stm32` — 387 اختبارًا على الحاسوب (بدائل
    العتاد في `test/native/support/`)، والتغطية بأداة `gcovr`؛
  - ‏`pio test -e esp32-s3` — ‏`test_feedback/` (محاكاة التغذية الراجعة بحلقة
    مغلقة) و`test_imu_orientation/` على اللوحة نفسها؛ ويرفع كلٌّ منهما برنامجًا
    ثابتًا للاختبار، وبعده ارفع البرنامج المعتاد بالأمر `pio run -t upload`.
- التحليل الساكن: `pio check -e esp32-s3` (‏cppcheck)، و`pio check -e stm32h743` (‏cppcheck على
  `hal/stm32/` و`src/stm32/`)، و`tools/clang-tidy.sh`
  (الملف التعريفي `.clang-tidy`).

---

## القيود المعروفة

- **لم يُختبر الطيار الآلي في الطيران.** على المكتب جرى التحقق من الإشارات
  مباشرةً (الميلان ← تصحيح في اتجاه التسوية)، ومعاملات PID قيم ابتدائية.
- **‏STABILIZE تسوية فوق العصي**، وليس «وضعًا زاويًّا» (FBWA) تحدد فيه العصا
  زاوية الدوران الجانبي/الميل. فيجتمع الطيار والطيار الآلي.
- **لم يُختبر الانسياب عند فقدان الاتصال في الطيران.** زوايا `FAILSAFE_GLIDE_*`
  قيم ابتدائية؛ والميل −3° يُختار بحسب هيكل الطائرة (فلا ينبغي أن يرتفع الأنف
  حتى الانهيار ولا أن ينقضّ).
- **الأفق.** مع معايرة التركيب (`o`) — منها (NVS)؛ وإزاحة الصفر لمقياس
  التسارع تنجرف مع الحرارة (نحو 1–2° لكل 20 °C)، فإذا «انجرف» الأفق — كرّر `o`.
  ودونها — الوضع عند التشغيل (شغّل والطائرة مستوية).
- **تركيب البوصلة** ما زال يُحدَّد بـ `MAG_ROTATION_CW_DEG`
  (ولا تمسّه المعايرة بالوضعيات).
- **البوصلة:** الاتجاه دون تعويض الميل، ولم يُتحقق من اتجاه العدّ على طائرة
  مجمَّعة، ويجب إجراء المعايرة داخل الطائرة نفسها. ولا يستخدم أي وضع الاتجاه حتى الآن.
- **‏GPS** لا يُستخدم في الملاحة؛ وفي ESP32-C3 هو للاستقبال فقط.
- **التغذية الراجعة (`autopilot/feedback/`) غير موصولة** ولم تُختبر إلا في
  محاكاة بنموذج فظّ للطائرة. وكل الأرقام في `FeedbackConfig.h` المعلَّمة
  «прикидка» («تقدير تقريبي») يلزم تدقيقها على هيكل طائرة حقيقي؛ ولا يوجد بعدُ
  حساس للسرعة الجوية (ودونه تُتعلَّم فعالية الأسطح ببطء أكبر، ولا يظهر
  الانهيار إلا بالتباطؤ).
- **لم تُختبر على العتاد:** `ICM42688_Sensor` (جرى توحيده مع الاتفاقية العامة
  عبر `ImuSensorBase`)، و`BME280_Sensor` (أُعيد تنفيذ تعويض Bosch)، وBMP388
  عبر SPI، و`QMC5883L_Sensor`، وضبط GPS عبر CFG-VALSET. عند التوصيل — سجل
  الإقلاع، و`s` في الكونسول، والإشارات بالميلان.
- **يلتقط I2C على لوحة التجارب تشويشًا** من ESC/المحرك (تظهر الأخطاء المفردة
  بالأمر `s`). والمشغِّلات تتحمله، لكن في الطائرة يجب أن تكون أسلاك I2C
  قصيرة وبعيدة عن أسلاك القدرة.
- **لا تُستخدم ESP32Servo.** يوزّع الإصدار 3.2.1 على ESP32-S3 السيرفوات على
  MCPWM ويخلط في `attachPin()` بين رقم وحدة MCPWM ورقم المؤقت: كان GPIO6/7
  يُخرجان إشارة GPIO4/5 (وكان ESC يُتحكَّم به من العصا اليمنى). وأُعيدت
  كتابة المخارج على LEDC؛ ولا تُعَد المكتبة إلا بعد التحقق بالأمر `p`.
- **‏ESC يعمل بـ PWM عند 50 Hz**، ولا يوجد في البرنامج الثابت بعدُ وضع لمعايرة
  مدى الخانق.
- **لوحة المعلومات عبر الويب:** كلمة مرور نقطة الوصول ضعيفة، وتُقبل الأوامر
  حتى أثناء الطيران. وهي أداة للمنصة والميدان، لا للطيران.
- **ميكانيكا النموذج الأولي:** طار النموذج الأول، وظهر ضعف في تثبيت المحرك
  وعدم كفاية في صلابة الجناح.
- **الترخيص هو OpenPlane License** ([LICENSE](LICENSE.md)): ‏MIT مع إلزامية ذكر
  المؤلف، وحظر الاستخدام العسكري، وحظر إلحاق الضرر المتعمد بالأشخاص والممتلكات
  دون موافقتهم الكتابية. لا تضف إلى الملفات ترويسات ترخيص أخرى ولا تحذف
  اسم المؤلف.

---

## كيف تُجري التعديلات

- **‏commits صغيرة:** خطوة منطقية واحدة — commit واحد.
- **الاختبارات والتحليل قبل الـ commit:** `pio test -e native -e native-stm32`،
  و`pio check -e esp32-s3`، و`pio check -e stm32h743`، و`tools/clang-tidy.sh` —
  جميعها خضراء ([`TESTING.md`](TESTING.md)).
- **ابنِ جميع اللوحات** بعد التعديلات في الشيفرة المشتركة — فـS3 هي الرئيسية،
  لكن C3 واللوحة ذات 38 طرفًا و`stm32h743` يجب ألا تنكسر؛ وقبل الإصدار —
  `tools/build_matrix.sh` (جميع اللوحات × جميع الحساسات).
- **تحقق على العتاد مما يمكن التحقق منه:** الإشارات — بالميلان، والمخارج —
  بالأمر `p`، والاتصال — بإطفاء جهاز التحكم.
- **لا تختلق واجهات API.** راجع مصادر الإطار في
  `~/.platformio/packages/framework-arduinoespressif32/` (‏Arduino core 2.0.x)
  — فالإنترنت كثيرًا ما يصف الإصدار 3.x ذا الواجهة المختلفة (مثل LEDC).
- **لا تجمّل الحالة.** ما لم يُختبر على العتاد — اكتب ذلك كما هو.
- **ينبغي ألا تعرف الطبقة أكثر مما يحق لها.** إذا احتاج صنف سفلي فجأةً إلى
  صنف علوي، فيجب أن يصعد المنطق إلى `FlightController`.
- **عند تغيير عقد البيانات** (`FlightOutputState` و`ControlCommand` و
  `ImuData` وJSON الخاص بـ `/api/status`) — حدّث جميع المستهلكين في الـ commit نفسه.

</div>
