<div dir="rtl">

# ‏TELEMETRY — السجل والوحدة الطرفية ولوحة الويب وOLED والصندوق الأسود

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/telemetry.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

القياس عن بُعد منفصل تمامًا عن منطق الطيران: فهو يقرأ فقط
الدوال الثابتة (const getters) في `FlightController` و`Autopilot` والمستشعرات و`LoopStats`.
والطريق الوحيد «للعودة» هو أوامر اللوحة، وهي تمرّ عبر صندوق بريد
`WebDebugServer` وتطبّقها حلقة الطيران.

---

## ‏`LoopStats`

**الملف:** `telemetry/LoopStats.h` · **النوع:** struct

تردد حلقة الطيران ومدتها.

| العضو | الوصف |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | تُنشر مرة كل ثانية؛ وتُقرأ من مهام أخرى (قيم من 32 بتًّا — بلا قراءات «ممزّقة») |
| `void record(uint32_t durationUs)` | تُستدعى في كل نبضة من `loop()` |
| `uint32_t takePeakUs()` | أسوأ نبضة منذ الاستدعاء السابق (لسطر SYS كل 10 s)؛ وتُستدعى من المهمة نفسها التي تستدعي `record()` |

و`maxUs` هو الأسوأ خلال آخر ثانية فقط؛ أما التعثّر النادر فيظهر عبر
`takePeakUs()`.

---

## ‏`LogSettings`

**الملف:** `telemetry/LogSettings.h` · **يعتمد على:** `Preferences` (NVS، مساحة الأسماء `debuglog`)

### ‏`LogChannel` (enum class)

| القناة | البادئة | ما تطبعه | الافتراضي |
|---|---|---|---|
| `Status` | `STAT` | الاتصال وARM والنمط والسدائف والمستشعرات | عند التغيّر |
| `Rc` | `RC` | قنوات جهاز التحكم | مُطفأ |
| `Outputs` | `OUT` | المخارج إلى أسطح التحكم وESC | مُطفأ |
| `Attitude` | `ATT` | الدحرجة والانحدار والاتجاه | مُطفأ |
| `Autopilot` | `AP` | الأهداف والتصحيحات | مُطفأ |
| `Altitude` | `ALT` | الارتفاع والسرعة الرأسية | مُطفأ |
| `Heading` | `MAG` | اتجاه البوصلة | مُطفأ |
| `Gps` | `GPS` | الأقمار والإحداثيات | مُطفأ |
| `Imu` | `IMU` | الجيروسكوب ومقياس التسارع | مُطفأ |
| `Nav` | `NAV` | نقطة الانطلاق والاتجاه والسرعة وأنبوب بيتو والميزات المفعّلة | مُطفأ |
| `System` | `SYS` | تردد الحلقة والذاكرة (كل 10 s)، وإطفاء/تشغيل فقط | مُشغَّل |
| `Count` | — | عدد القنوات | — |

‏`LogMode` (enum class): ‏`Off` و`OnChange` و`Periodic`.

‏`LogChannelInfo`: ‏`tag` و`title` و`periodicOnly` و`defaultMode`.

| الدالة | الوصف |
|---|---|
| `static constexpr uint8_t COUNT`، `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | صف من جدول القنوات |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (بالتناوب الدوري) |
| `LogSettings()`، `void setDefaults()` | الأنماط الافتراضية، وفترة 1 s |
| `LogMode mode(uint8_t)`، `mode(LogChannel)` | نمط القناة |
| `void setMode(uint8_t, LogMode)` | في قنوات `periodicOnly` يتحوّل `OnChange` إلى `Periodic` |
| `void cycleMode(uint8_t)` | مُطفأ ← عند التغيّر ← مستمر ← مُطفأ (SYS: مُطفأ ↔ مُشغَّل) |
| `void setAll(LogMode)` | لكل القنوات؛ ولا يمسّ أمر «الكل عند التغيّر» قناة SYS |
| `uint16_t periodMs() const`، `void cyclePeriod()` | فترة النمط «المستمر» |
| `static const char* modeName(LogMode, bool periodicOnly)` | «مُطفأ» / «عند التغيّر» / «مستمر» (أو «مُشغَّل») |
| `void load()` | من NVS؛ وإذا لم تتطابق `VERSION` أو الطول بقيت القيم الافتراضية؛ ورمز النمط المجهول ← الافتراضي للقناة |
| `void save() const` | إلى NVS (الأنماط والفترة والإصدار) |

تتغيّر `VERSION` مع قائمة القنوات — فتُعاد الإعدادات القديمة إلى وضعها الأصلي
(`VERSION = 2`: أُضيفت القناة NAV). ومفاتيح القنوات في القائمة: `1`..`9`، وNAV —
`n`، وSYS — `s`.

---

## ‏`DebugLogger`

**الملف:** `telemetry/DebugLogger.h` · **يعتمد على:** `FlightController` و`Autopilot*` و`LoopStats*` و`LogSettings` و`Config`

طباعة الحالة في مراقب المنفذ التسلسلي بحسب القنوات: لكل قناة سطرها الخاص ونمطها
الخاص وحدود تسامحها.

| الدالة | الوصف |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | مرة كل `DEBUG_INTERVAL_MS` يمرّ على القنوات (ويصمت في الإيقاف المؤقت وما دامت القائمة مفتوحة) |
| `LogSettings& getSettings()`، `void saveSettings() const` | لقائمة الوحدة الطرفية |
| `void suspend(bool)` | القائمة مفتوحة — يصمت؛ وعند الرفع — `refresh()` |
| `void setPaused(bool)`، `bool isPaused() const` | إيقاف مؤقت بمفتاح المسافة؛ وعند الرفع — `refresh()` |
| `void refresh()` | ستطبع النبضة التالية كل القنوات المفعّلة |

منطق القناة (`updateChannel`):

- ‏`Off` — لا طباعة؛
- ‏`Periodic` — مرة كل `periodMs()` (وفي SYS — مرة كل 10 s)، بالقيم «كما هي»؛
- ‏`OnChange` — يُجمَع السطر مع **حدود تسامح** (الكائن المتداخل `Shown` يحتفظ
  بالقيمة القديمة حتى تتجاوز الجديدة حدّ التسامح: RC/PWM 3 µs، والزوايا
  0.5°، والاتجاه 1°، والتصحيحات 2، والارتفاع 0.3 m، والتسارع 0.03 g، والإحداثيات
  1e−5°) ولا يُطبع إلا إذا اختلف عن آخر سطر طُبع.

الأنواع المتداخلة: `LineBuffer : Print` (سطر حتى 200 بايت للمقارنة قبل
الطباعة) و`Shown` (قيمة ذات تخلّف).

صيغ الأسطر:

<div dir="ltr">

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  الهدف 0.0 m
MAG  الاتجاه 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (الأسوأ خلال 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

</div>

يميّز `RX=` بين `LOST(لا إطارات)` و`LOST(failsafe جهاز التحكم)`؛ وقيم `IMU=` هي
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## ‏`DebugConsole`

**الملف:** `telemetry/DebugConsole.h` · **يعتمد على:** `FlightController` و`FlightOutputs` و`Autopilot` و`DebugLogger` و`LogSettings` و`IBoard*` (مسح الناقلات)

قائمة نصية في مراقب المنفذ التسلسلي. آلة حالات للشاشات `Screen::{None, Main, Log}`.

| الدالة | الوصف |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | مع اللوحة — الأمر `b` والبند 7 في القائمة |
| `static const char* guessI2cDevice(uint8_t address)` | الرقاقة بحسب العنوان: 0x6A LSM6DSV، و0x68 MPU/ICM، و0x76 BME280/BMP388/SPL06، و0x46/0x47 BMP581، و0x7C QMC6309، و0x2C QMC5883P، و0x0D QMC5883L، و0x3C OLED |
| `void printHint() const` | تلميح من سطر واحد |
| `void update()` | معالجة كل البايتات الواردة من `Serial`؛ وإذا تغيّرت إعدادات السجل وكانت القائمة مغلقة والطائرة **ليست armed** — فتُحفظ في NVS |

المفاتيح السريعة (خارج القائمة): `h`/`?` — القائمة الرئيسية؛ `l` — قائمة السجل؛ المسافة —
إيقاف السجل مؤقتًا؛ `s` — حالة المستشعرات؛ `i` — معايرة الجيروسكوب؛ `o` —
معايرة تركيب IMU؛ `m` — معايرة البوصلة؛ `p` — الفحص الذاتي للمخارج؛
`b` — مسح ناقلات I2C (من 0x08 إلى 0x7F — حتى 0x7F لأن QMC6309 على العنوان 0x7C) مع
أسماء الرقاقات؛ وأي مفتاح آخر — تلميح. ويُتجاهل `\r`/`\n`.

قائمة السجل: `1`..`9` — تدوير نمط القنوات من 0 إلى 8، و`n` — NAV، و`s` — SYS، و`p` — الفترة، و`a` —
الكل «عند التغيّر»، و`x` — إطفاء الكل، و`d` — الافتراضي، و`0`/`q` — رجوع، و`l`/`h` —
إغلاق.

الإجراءات الحاجزة (`i` و`o` و`m` و`p`) **ممنوعة أثناء ARM**. وما دامت القائمة
مفتوحة يكون السجل معلَّقًا (`DebugLogger::suspend`). ويُحسب عرض بنود القائمة
بعدد محارف UTF-8 لا بالبايتات (الحرف السيريلي يشغل 2 بايت).

---

## ‏`WebDashboardPage`

**الملف:** `telemetry/WebDashboardPage.h` · **النوع:** namespace

‏`static const char HTML[] PROGMEM` — الصفحة كلها (HTML + CSS + JS) في
نص حرفي واحد. وكل ما هو ديناميكي يبنيه المتصفح من JSON الخاص بـ `/api/status` (يُستطلَع
كل 200 ms): تُنشأ صفوف القنوات والمخارج والمستشعرات من مفاتيح JSON، فيظهر مخرج
جديد دون تعديل الصفحة. وحقل PID الذي بدأ المستخدم
تحريره لا يعيد الاستطلاع الكتابة فوقه.

---

## ‏`WebDebugServer`

**الملف:** `telemetry/WebDebugServer.h` · **يعتمد على:** `WebServer` و`WiFi` و`FlightController` و`Autopilot*` و`WebDashboardPage` و`Config`

| الدالة | الوصف |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | نقطة وصول Wi-Fi (`persistent(false)` — دون كتابة في الفلاش) والمسارات ومهمة `web` على النواة 0. وتُرجع `false` إذا لم ترتفع نقطة الوصول |
| `void applyPendingCommands()` | تُستدعى من حلقة الطيران: تأخذ الأوامر تحت قفل دوراني وتطبّقها على الطيار الآلي |

المسارات:

| المسار | الاستجابة |
|---|---|
| `GET /` | صفحة اللوحة |
| `GET /api/status` | JSON الحالة (`buildStatusJson()`)، وصيغته في [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` ← 200 ‏`{"status":"ok"}`؛ بلا جسم ← 400 ‏`no data`؛ بلا طيار آلي ← 503؛ نمط خاطئ ← 400 ‏`invalid mode` |
| `POST /api/setpid` | أي من `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`؛ وما أُغفل يبقى على حاله |
| غير ذلك | 404 |

‏`PendingCommands { hasMode, mode, hasPid, pid[6] }` — صندوق بريد تحت
`portMUX`. و`extractJsonNumber(body, key, fallback)` — تحليل أدنى
لـ JSON مسطّح دون ArduinoJson: `"key"` ثم فراغات ثم `:` ثم فراغات ثم رقم بأي
كتابة JSON (إشارة، كسر، أُس `1e-7`)؛ وإن غاب المفتاح أو الرقم —
`fallback`.

في JSON يوجد الحقلان `attached`/`available` **دائمًا**؛ أما بيانات المستشعر فعند
`available: true` فقط.

---

## ‏`OledDisplay`

**الملف:** `telemetry/OledDisplay.h` · **يعتمد على:** U8g2 و`II2CBus` و`FlightController` و`Autopilot*` و`LoopStats`

شاشة SSD1306 بحجم 128×64 (I2C بالعنوان 0x3C) على ناقل I2C الثاني؛ ولها مهمتها الخاصة `oled`
(`Rtos::startTask`: النواة 0 في ESP32، وأولوية منخفضة في STM32)، كل 200 ms.

| الدالة | الوصف |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` أو الشاشة لا تستجيب عند 0x3C ← `false`؛ وإلا ضبط U8g2 وتشغيل المهمة |

يرسل U8g2 البايتات عبر `byteCallback` فوق `II2CBus` (والشاشة لا تعرف شيئًا عن
`Wire1`). ولا يتلقى رد النداء بلغة C سياقًا، فيُحفظ الناقل في متغير ساكن
`busSlot()` — إذ توجد شاشة واحدة على اللوحة.

الشاشة:

<div dir="ltr">

```
RX ok ARM STAB FL       الاتصال (الفقدان — سطر معكوس) / ARM / النمط / السدائف
R  +1.2 P  -0.4         الدحرجة / الانحدار، °           (IMU --)
Alt +0.3 Vz +0.1 A14    الارتفاع / السرعة الرأسية / سرعة الهواء، إن وُجد أنبوب بيتو (BARO --)
H123 T1000 Y1500        الاتجاه / الدفع / دفة الاتجاه (H---)
L1500 R1500 E1500       الجنيحات / دفة الارتفاع
Loop 500Hz max1100us    التردد وأسوأ نبضة خلال الثانية
```

</div>

الأسماء المختصرة للأنماط من `AutopilotNames::modeShort()` (`MAN` و`STAB` و
`TKOFF` و`ALT` و`ACRO` و`CRZ` و`LOIT` و`RTH` و`LNCH` و`LAND` و`SOAR` و`RESQ`)؛
وعند فقدان الاتصال في الجو — `GLIDE` أو `FSRTH`.

---

## ‏`BlackBox`

**الملف:** `telemetry/BlackBox.h` · **يعتمد على:** `FlightController` و`Autopilot` و`LoopStats` و`BlackBoxStorage` و`PilotSwitches*`

تسجيل الرحلة على الفلاش (ESP32-S3) أو على بطاقة SD (STM32H743). وما الذي يُنزَّل ومتى وكيف — في [BLACKBOX.md](../BLACKBOX.md).

| الدالة | الوصف |
|---|---|
| `bool begin(bool startTask = true)` | يقرأ الوسيط (`BlackBoxStorage::begin()`)، ويحجز الطابور (PSRAM في ESP32، و`malloc` في STM32)، ويطابق المساحة الممسوحة (حتى 0.3 s)، ويشغّل مهمة `bbox` (`Rtos::startTask`). وإن لم يوجد مكان للتسجيل (قسم أو بطاقة أو ملف) — `false` والصندوق الأسود معطّل |
| `void update(uint32_t workUs)` | من `loop()` بعد كل نبضة: الأحداث والبدء/الإيقاف ولقطات تُدفع إلى الطابور، وإيقاظ مهمة الكتابة |
| `void writerStep()` | خطوة لمهمة الكتابة: صفحة أو صفحتان إلى الفلاش، أو مسح واحد على الأرض |
| `requestManualStart()` / `requestManualStop()` | التسجيل اليدوي (الوحدة الطرفية `k` ← `r`) |
| `State getState()` / `bool isRecording()` | `Off` و`Idle` و`Recording` و`Stopping` (يكتب بقية الطابور قبل تسجيل END) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | للوحدة الطرفية |
| `void handleHostCommand(const char*)` | `bb list` و`bb get <n> [الباود]` — من أجل `tools/blackbox.py` (وعلى USB CDC لا تؤثر السرعة في شيء) |

الأجزاء الخاصة بكل منصة: سبب إعادة التشغيل — `readResetCause()`؛ وجهد البطارية وتيارها —
عبر ADC (الدالة `analogReadMilliVolts` في S3، و`analogRead` بدقة 12 بتًّا في STM32)؛ وأخطاء الوسيط
(`BlackBoxStorage::writeErrors`/`eraseErrors`) تدخل السجل مرة كل ثانية
كحدث «الوسيط: أخطاء كتابة …» ولا تعيق الطيران.

## ‏`BlackBoxStorage`

**الملف:** `telemetry/BlackBoxStorage.h` · **يعتمد على:** `IFlashRegion`

حلقة من قطاعات حجم كل منها 4 KB: الرأس وقائمة الرحلات من رؤوس القطاعات عند `begin()` (المرور الأول يقرأ رأس كل قطاع ويحفظ الصحيحة منها، والثاني يمرّ عليها وحدها: فالمنطقة الفارغة تُقرأ مرة واحدة)؛ و`openFlight()`/`append()`/`flush()`/`closeFlight()` — كتابة بالصفحات (مع CRC-8 لكل سجل)؛ و`eraseStep(target, protect, allowErase)` — خطوة مطابقة/مسح أمام الرأس: القمامة — دائمًا، والرحلات — كاملة ولا تُمسح إلا ما دامت المساحة الحرة أقل من `target`؛ ولا يُمَسّ `protect` أبدًا.

## ‏`BlackBoxRing` و`BlackBoxFormat`

‏`BlackBoxRing` طابور بايتات للسجلات بين المهام/النوى تحت `Rtos::CriticalSection`؛ وعند امتلائه يُسقط الأقدم. و`BlackBoxFormat` — رأس القطاع وأنواع السجلات وبناها وسلاسل المخططات (يُتحقق من الحجم بـ `static_assert`) وCRC-8 وCRC-32.

---

## ‏`Mavlink` (المرمِّز)

**الملف:** `telemetry/MavlinkCodec.h` · **النوع:** namespace · **يعتمد على:** لا شيء (قابل للنقل)

‏MAVLink 2 دون المكتبة المولَّدة: تعبئة الحقول بترتيب MAVLink
(مطابَقة مع pymavlink)، وCRC-16/MCRF4XX مع `CRC_EXTRA`، وقصّ الأصفار الذيلية.

| الكيان | الوصف |
|---|---|
| `Msg::*` | المعرِّفات: HEARTBEAT وSYS_STATUS وSET_MODE وPARAM_* وGPS_RAW_INT وATTITUDE وGLOBAL_POSITION_INT وSERVO_OUTPUT_RAW وMISSION_REQUEST_LIST/COUNT وNAV_CONTROLLER_OUTPUT وRC_CHANNELS وREQUEST_DATA_STREAM وVFR_HUD وCOMMAND_LONG/ACK وHOME_POSITION وSTATUSTEXT |
| `int crcExtraOf(uint32_t id)` | قيمة `CRC_EXTRA` للرسالة، و−1 تعني مجهولة |
| `crcAccumulate`، `crcCalculate` | X.25 (مثل `crc_accumulate()` في mavlink) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — الحقول بالترتيب |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — إطار v2 مع `seq` متتابع |
| `Message` | رسالة مستلَمة: `msgid` و`sysid` و`compid` والحمولة (مكمَّلة بالأصفار)، وقراءة الحقول بالإزاحة |
| `Parser` | `bool feed(byte)` ← `message()`؛ v1 وv2، ويُتخطّى توقيع v2؛ و`goodCount()` و`badCrcCount()`؛ والرسائل ذات `CRC_EXTRA` المجهول تُتخطّى بصمت |

## ‏`MavlinkModes`

**الملف:** `telemetry/MavlinkTelemetry.h` · **النوع:** namespace

| الدالة | الوصف |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | رقم نمط ArduPlane: MANUAL 0، وSTABILIZE←FBWA 5، وALT_HOLD←FBWB 6، وACRO 4، وCRUISE 7، وLOITER 12، وRTH←RTL 11، وAUTO_TAKEOFF/LAUNCH←TAKEOFF 13، وAUTO_LAND←AUTO 10، وSOARING←THERMAL 24، وRESCUE←STABILIZE 2؛ وfailsafe ← RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | العكس، لأوامر الأرض؛ وAUTO وCIRCLE وGUIDED — `false` |
| `isAutonomous(mode)` | الراية `AUTO_ENABLED` في HEARTBEAT |

## ‏`MavlinkTelemetry`

**الملف:** `telemetry/MavlinkTelemetry.h` · **يعتمد على:** `IUartPort` و`FlightController` و`Autopilot*` و`LoopStats*`

قياس عن بُعد عبر مودم راديو لبرنامجَي QGroundControl / Mission Planner (المركبة هي
`MAV_TYPE_FIXED_WING` و`MAV_AUTOPILOT_ARDUPILOTMEGA`). ويُستخدم في STM32
(UART4) الذي لا يملك Wi-Fi.

| الدالة | الوصف |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | فتح المنفذ، والرسالة «OpenPlane online» |
| `void update()` | من حلقة الطيران: تحليل الوارد (حتى 128 بايت في النبضة)، ورسائل الأحداث، وبما لا يزيد على 2 إطار في النبضة |
| `void statusText(severity, text)` | إلى سجل GCS (طابور من 4 أسطر، حتى 50 محرفًا) |
| `isGcsConnected()` | وصول HEARTBEAT من GCS خلال آخر 3 s |
| `getSentFrames()`، `getDeferredFrames()`، `getParser()` | تشخيص |
| `static const char* paramName(uint8_t)` | `RLL_KP` و`RLL_KI` و`RLL_KD` و`PTCH_KP` و`PTCH_KI` و`PTCH_KD` |

التدفقات (Hz): ATTITUDE 10؛ وGLOBAL_POSITION_INT وVFR_HUD 5؛ وGPS_RAW_INT
وRC_CHANNELS وSERVO_OUTPUT_RAW وNAV_CONTROLLER_OUTPUT 2؛ وHEARTBEAT وSYS_STATUS 1؛
وHOME_POSITION 0.2. ولا يُرسل الإطار إلا إذا كان في `availableForWrite()` متسع له
— وإلا انتظر النبضة التالية (فالحلقة لا تُحجَب أبدًا).

الوارد: HEARTBEAT من GCS؛ وPARAM_REQUEST_LIST / READ / SET (قيم PID — مباشرة إلى
الطيار الآلي، من 0 إلى 100، ولا تُحفظ)؛ وSET_MODE وCOMMAND_LONG
`DO_SET_MODE` (176) — النمط حتى النقرة التالية للمفتاح؛ و`COMPONENT_ARM_DISARM`
(400) — **DENIED**؛ و`REQUEST_MESSAGE` (512) — إرسال تدفق خارج الدور؛
وMISSION_REQUEST_LIST — MISSION_COUNT 0 بالقيمة نفسها لـ `mission_type`.
والتحقق من التدفق بمحلّل خارجي — `tools/check_mavlink.py` (pymavlink).

</div>
