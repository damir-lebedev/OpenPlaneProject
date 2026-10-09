<div dir="rtl">

# ‏HAL — تجريد العتاد

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/hal.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

طبقة HAL هي الطبقة الوحيدة المسموح لها بمعرفة معالج MCU بعينه. الواجهات
موجودة في `include/hal/`، أما التنفيذات فهي:

- ‏`include/hal/esp32/` — ESP32 (Arduino core 2.0.x)؛
- ‏`include/hal/stm32/` — STM32H743 (STM32duino 3.x)، **وهو الأساسي**: يُبنى البرنامج الثابت الكامل (`pio run -e stm32h743-devebox`) ويعمل على الحاسوب (`pio test -e native-stm32`)؛ وعلى لوحة DevEBox جرى التحقق من بطاقة SD والصندوق الأسود وiBUS والمؤازِرات، أما الحساسات فلم يُتحقق منها بعد؛
- ‏`hal/Rtos.h` — مهام FreeRTOS، متطابقة على المنصتين.

كل ما فوق هذه الطبقة يتعامل مع الواجهات فقط، ولذلك فالانتقال إلى معالج MCU آخر
يعني تنفيذًا جديدًا لـ `IBoard`، لا إعادة كتابة المستشعرات.

---

## ‏namespace `ServoChannel`

**الملف:** `hal/IBoard.h`

فهارس المخارج في `IBoard::servo(channel)`. هي قائمة مسطحة وليست دوالّ مسمّاة،
فإضافة مخرج لا تغيّر واجهة `IBoard`. ويطابق الترتيبُ صفوفَ جدول
`FlightOutputs::outputInfo()`.

| الثابت | القيمة |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — إسقاط الحمولة (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — الكاميرا (`Knob::CAMERA_TILT`، `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## ‏`IBoard`

**الملف:** `hal/IBoard.h` · **النوع:** واجهة · **التنفيذات:** `Esp32Board`، `Stm32Board`

نقطة الدخول الوحيدة إلى العتاد. لا شيء فوقها يضمّ `<Wire.h>` أو
`<SPI.h>` أو `HardwareSerial`، ولا شيء يستدعي LEDC مباشرة.

| الدالة | الوصف |
|---|---|
| `virtual void begin()` | تهيئة لمرة واحدة لناقلَي I2C وSPI. تفتح منافذ UART أصحابُها (`IBusReceiver`، وGPS) بسرعتها الخاصة، وتفتح PWM الدالةُ `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | ناقل المستشعرات |
| `virtual ISpiBus& spi()` | ناقل SPI |
| `virtual II2CBus* displayI2c()` | ناقل I2C ثانٍ للشاشة فقط؛ و`nullptr` إن لم يوجد |
| `virtual IUartPort& rcUart()` | منفذ UART لمستقبِل iBUS |
| `virtual IUartPort& gpsUart()` | منفذ UART لـ GPS |
| `virtual IUartPort* telemetryUart()` | منفذ UART لمودم الراديو MAVLink؛ والقيمة الافتراضية `nullptr` (لا يوجد في ESP32 منفذ UART حرّ) |
| `virtual IServoOutput& servo(uint8_t channel)` | مخرج PWM بحسب الفهرس `ServoChannel::*` |
| `virtual void setBuzzer(bool on)` | الجرس `PIN_BUZZER`؛ ولا يفعل شيئًا افتراضيًا |

---

## ‏`II2CBus`

**الملف:** `hal/II2CBus.h` · **النوع:** واجهة مع دوالّ مساعدة غير افتراضية ·
**التنفيذات:** `Esp32I2CBus`، `Stm32I2CBus`

تجريد لناقل I2C على شكل `Wire`. يثبّت التنفيذُ الأطراف والتردد في المُنشئ،
ولذلك لا تأخذ `begin()` و`setClock()` أطرافًا — فالناقل يُهيَّأ مرة واحدة تمامًا،
حتى لو كان عليه عدة أجهزة.

| الدالة | الوصف |
|---|---|
| `begin()`، `setClock(hz)` | التهيئة والتردد |
| `beginTransmission(addr)`، `write(byte)`، `write(data, len)`، `endTransmission(sendStop = true)` | الكتابة؛ وتُرجع `endTransmission` القيمة 0 عند النجاح (مثل `Wire`) |
| `requestFrom(addr, n)`، `available()`، `read()` | القراءة |
| `bool writeRegister(addr, reg, value)` | دالة مساعدة: كتابة سجل واحد؛ و`false` تعني NACK |
| `bool readRegisters(addr, reg, buf, count)` | دالة مساعدة: بدء مكرَّر ثم قراءة `count` بايت. تُرجع `false` عند NACK **أو إذا وصل أقل من `count` بايت**؛ ولا يُمَسّ المخزن المؤقت عندئذٍ |
| `int readRegister(addr, reg)` | قيمة السجل أو `-1` |
| `bool probe(addr)` | الجهاز يردّ بـ ACK على العنوان |

ثابت لا يتغير: عند الإخفاق لا تكتب الدوالّ المساعدة في المخزن المؤقت — فيحتفظ
المشغّل بالبيانات السابقة بدل القيم العشوائية (القيمة `0xFF` التي يعيدها `read()`
على مخزن فارغ).

---

## ‏`ISpiBus`

**الملف:** `hal/ISpiBus.h` · **النوع:** واجهة · **التنفيذات:** `Esp32SpiBus`، `Stm32SpiBus`

ناقل SPI **دون إدارة CS**: تتشارك عدة أجهزة ناقلًا واحدًا، ويبدّل `SpiRegisterDevice`
خطّ CS.

| الدالة | الوصف |
|---|---|
| `begin()` | ضبط SCK/MISO/MOSI (الأطراف في مُنشئ التنفيذ) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` من 0 إلى 3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | تبادل بايت بنمط الإرسال والاستقبال المتزامنين |
| `endTransaction()` | نهاية المعاملة |

---

## ‏`IUartPort`

**الملف:** `hal/IUartPort.h` · **النوع:** واجهة · **التنفيذات:** `Esp32UartPort`، `Stm32UartPort`

منفذ UART على شكل `HardwareSerial`، لكن `begin()` تأخذ السرعة فقط:
الأطراف والصيغة (8N1) يثبّتها التنفيذ.

| الدالة | الوصف |
|---|---|
| `begin(baud)` | فتح المنفذ |
| `int available()`، `int read()` | الاستقبال |
| `size_t write(byte)`، `size_t write(buffer, size)` | الإرسال |
| `virtual int availableForWrite()` | المساحة الحرة في مخزن الإرسال؛ و`-1` تعني غير معروفة (الافتراضي). تؤجّل القياسات عن بُعد الإطار بالاعتماد عليها بدل الانتظار |

---

## ‏`IServoOutput`

**الملف:** `hal/IServoOutput.h` · **النوع:** واجهة · **التنفيذات:** `Esp32ServoOutput`، `Stm32ServoOutput`

مخرج PWM واحد. يثبّت التنفيذُ الطرف.

| الدالة | الوصف |
|---|---|
| `bool attach(minUs, maxUs)` | حجز القناة/المؤقّت وضبط الطرف؛ ومدى تحديد النبضة. تعني `true` فقط أن المعالج MCU حجز الموارد، **لا** أن المؤازِر موصول |
| `writeMicroseconds(us)` | عرض النبضة، µs (محدود بمدى `attach`) |
| `bool isAttached() const` | نتيجة `attach()` |
| `virtual int32_t measurePulseUs()` | تشخيص: العرض الفعلي للنبضة على الطرف أو `-1`. والتنفيذ الافتراضي يُرجع `-1` |

---

## ‏`IFlashRegion`

**الملف:** `hal/IFlashRegion.h` · **النوع:** واجهة · **التنفيذات:** `Esp32FlashPartition`، `SdFileRegion`

منطقة في فلاش NOR مخصّصة للسجل (الصندوق الأسود): المسح يكون بقطاعات من 4 KB فقط
(وما مُسح يُقرأ `0xFF`)، والكتابة تُصفّر البتّات فقط — يمكن الكتابة في البايتات الممسوحة،
وعلى دفعات في الصفحة الواحدة أيضًا. في ESP32 تُوقف الكتابة والمسح كلاهما النواتين معًا —
ومتى يكون ذلك مقبولًا يقرّره المستدعي.

| الدالة | الوصف |
|---|---|
| `uint32_t size() const` | حجم المنطقة بالبايت؛ 0 يعني لا منطقة |
| `bool read(offset, data, length)` | قراءة |
| `bool write(offset, data, length)` | كتابة (في بايتات ممسوحة) |
| `bool erase(offset, length)` | مسح؛ العنوان والطول من مضاعفات 4096 |

‏`Esp32FlashPartition(const char* name)` — قسم بيانات بحسب اسمه في جدول الأقسام
(`esp_partition_*`)؛ و`begin()` تجد القسم (بعد إقلاع النواة)،
وإن لم يوجد فالنتيجة `false` و`size() == 0`.

---

## ‏`IBlockDevice`

**الملف:** `hal/IBlockDevice.h` · **النوع:** واجهة · **التنفيذات:** `Stm32SdCard` (وفي الاختبارات — `fake::SdCardModel`)

بطاقة SD على هيئة مصفوفة من كتل حجم كل منها 512 بايت. لا يوجد مسح: يمكن إعادة كتابة الكتلة.

| الدالة | الوصف |
|---|---|
| `uint32_t blockCount() const` | الحجم بالكتل؛ 0 يعني لا بطاقة |
| `bool read(block, data, count)` / `write(...)` | `count` كتلة متتالية، و`data` أي عنوان |

## ‏`SdFileRegion`

**الملف:** `hal/SdFileRegion.h` · **يرث من:** `IFlashRegion` · **يعتمد على:** `IBlockDevice`، `Fat32::locate`

منطقة الصندوق الأسود على بطاقة SD: ملف في جذر FAT32 (افتراضيًا
`BLACKBOX.BIN`)، يُنشأ مسبقًا على الحاسوب قطعةً واحدة متّصلة (`tools/blackbox.py
sd-prepare`) ويُملأ بالقيمة `0xFF`. يُعثر على الملف فقط (**لا** تُمَسّ جداول FAT
ولا الدليل)، ثم تُكتب الكتل الخام داخله. وبالنسبة لـ `BlackBoxStorage` فهي
نفس `IFlashRegion` الذي هو قسم فلاش ESP32.

| الدالة | الوصف |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` هو سقف المنطقة: يزداد وقت مطابقة القطاعات عند التشغيل بازدياده |
| `Fat32::Result begin()` | البحث عن الملف. `Ok` تعني `size() > 0`؛ وإلا فالسبب (`Fat32::describe()`): لا بطاقة، أو ليست FAT32، أو لا ملف، أو مجزّأ، أو فارغ |
| `size()` | الملف (بما لا يزيد على `maxBytes`) مقرَّبًا إلى الأسفل إلى قطاع 4 KB؛ و0 تعني لا منطقة |
| `read` / `write` | أي إزاحة وأي طول. الكتلة الناقصة تُقرأ وتُكمَل وتُكتب كاملة؛ والكتلة التي كُتبت للتو تُتذكَّر (ذاكرة مؤقتة بالكتابة المباشرة): فالصفحات المتتالية من 256 بايت لا تقرأ البطاقة من جديد. وانقطاع الطاقة لا يُفقد شيئًا عادت به `write()` |
| `erase(offset, length)` | من مضاعفات 4096؛ يكتب `0xFF` (للبطاقة مسحها الخاص من الداخل ولا حاجة إليه من الخارج) |

## ‏`IRegisterDevice`

**الملف:** `hal/RegisterDevice.h` · **النوع:** واجهة ·
**التنفيذات:** `I2cRegisterDevice`، `SpiRegisterDevice`

«مجموعة من السجلات ذات 8 بتّات». يُكتب مشغّل المستشعر مرة واحدة،
ويُختار الناقل عند إنشاء الكائن في `SensorSelection.h`.

| الدالة | الوصف |
|---|---|
| `virtual void begin()` | تجهيز خطوط الجهاز (لـ SPI — الخط CS). ولا شيء افتراضيًا |
| `virtual bool probe()` | الجهاز استجاب (في SPI دائمًا `true` — لا يوجد ACK، ويُفحص سجل المعرِّف ID) |
| `virtual bool writeRegister(reg, value)` | كتابة سجل |
| `virtual bool writeRegisters(reg, data, count)` | كتابة متتالية (زيادة تلقائية للعنوان) |
| `virtual bool readRegisters(reg, buffer, count)` | قراءة `count` بايت متتالية؛ وعند `false` لا يُمَسّ المخزن المؤقت |
| `int readRegister(reg)` | القيمة أو `-1` (دالة مساعدة غير افتراضية) |

---

## ‏`I2cRegisterDevice`

**الملف:** `hal/RegisterDevice.h` · **يرث من:** `IRegisterDevice`

جهاز على `II2CBus` بعنوان من 7 بتّات. تُفوَّض كل العمليات إلى الدوالّ المساعدة في `II2CBus`.

| الدالة | الوصف |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` هو العنوان الثاني للرقاقة (الطرف SDO/SA0): LSM6DSV 0x6A/0x6B، وICM-45686 0x68/0x69، وSPL06 0x76/0x77، وBMP581 0x46/0x47 |
| `begin()` | إذا لم يستجب العنوان الأساسي واستجاب الاحتياطي — يُتابَع العمل بالاحتياطي |
| `probe()`، `writeRegister()`، `writeRegisters()`، `readRegisters()` | ← الدوالّ المساعدة في `II2CBus(address, …)` |
| `uint8_t getAddress() const` | العنوان الحالي للجهاز |

---

## ‏`SpiRegisterDevice`

**الملف:** `hal/RegisterDevice.h` · **يرث من:** `IRegisterDevice`

جهاز على `ISpiBus` له طرف CS خاص. بروتوكول Bosch/InvenSense: القراءة —
العنوان مع البت `0x80`، والكتابة — مع تصفير البت 7.

| الدالة | الوصف |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` — كم بايتًا «عشوائيًا» تخرجها الرقاقة بعد العنوان وقبل البيانات (BMP388 — 1، وICM42688 — 0)؛ و`mode` — نمط SPI من 0 إلى 3 |
| `begin()` | `pinMode(cs, OUTPUT)`، وCS = HIGH |
| `probe()` | دائمًا `true` |
| `writeRegister(reg, value)` | CS↓، ثم `reg & 0x7F`، ثم `value`، ثم CS↑؛ ودائمًا `true` |
| `readRegisters(reg, buf, n)` | CS↓، ثم `reg \| 0x80`، وتخطّي `dummyReadBytes`، ثم `n` بايت، ثم CS↑؛ ودائمًا `true` |

كل عملية معاملة مستقلة `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## ‏`Esp32Board`

**الملف:** `hal/esp32/Esp32Board.h` · **يرث من:** `IBoard`

المكان الوحيد الذي ينشئ كائنات الطرفيات الملموسة في ESP32 ويعرف الأطراف الواردة في `Config.h`.

| الحقل | النوع | ما هو |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` على `PIN_I2C_SDA/SCL`، بتردد 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` على `PIN_I2C2_SDA/SCL` — فقط إذا كان `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | الكائن العام `SPI` |
| `rcSerial`، `rcPort` | `HardwareSerial(1)`، `Esp32UartPort` | iBUS على `PIN_IBUS`، استقبال RX فقط |
| `gpsSerial`، `gpsPort` | `HardwareSerial(UART_NUM_GPS)`، `Esp32UartPort` | GPS على `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | قنوات LEDC من 0 إلى 6 بترتيب `ServoChannel` (AUX1/AUX2 — `PIN_AUX1/2`، إذا كانت موصولة) |

| الدالة | الوصف |
|---|---|
| `begin()` | `i2cBus.begin()`، ثم `spiBus.begin()`، ثم `displayBus.begin()` إن وُجد الناقل الثاني؛ وطرف الجرس |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` إذا كان الطرف موصولًا |
| `displayI2c()` | `&displayBus` إذا كانت `hasDisplayBus()` صحيحة، وإلا `nullptr` |
| `static constexpr bool hasDisplayBus()` | طرفا الناقل الثاني كلاهما ≥ 0. وهي موجودة (مثل الحقل `displayBus`) فقط عندما `SOC_I2C_NUM > 1` — ففي C3 متحكم I2C واحد |
| الباقي | تُرجع الحقول المقابلة |

---

## ‏`Esp32I2CBus`

**الملف:** `hal/esp32/Esp32I2CBus.h` · **يرث من:** `II2CBus`

غلاف رقيق فوق `TwoWire` (`Wire` أو `Wire1`).

| الدالة | الوصف |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | تحفظ المعاملات |
| `begin()` | `wire.begin(sda, scl, hz)` و`wire.setTimeOut(TIMEOUT_MS)` — وهذا الاستدعاء الوحيد لـ `wire.begin()` |
| الباقي | تفويض مباشر إلى `TwoWire` |

‏`TIMEOUT_MS = 5`: قراءة 14 بايت من IMU عند 400 kHz تستغرق نحو 0.4 ms؛ وكانت المعاملة
التي تعلق بسبب التشويش ستوقف الحلقة بمقدار 50 ms المعتادة.

---

## ‏`Esp32SpiBus`

**الملف:** `hal/esp32/Esp32SpiBus.h` · **يرث من:** `ISpiBus`

غلاف فوق الكائن العام `SPI`. `begin()` ← `SPI.begin(sck, miso, mosi, -1)` (الخط CS
تمسكه الأجهزة). وتبني `beginTransaction()` الكائن `SPISettings(hz, MSBFIRST,
SPI_MODEn)`؛ وتحوّل `spiModeOf()` القيم 0..3 إلى ثوابت Arduino، والقيمة
المجهولة ← `SPI_MODE0`.

---

## ‏`Esp32UartPort`

**الملف:** `hal/esp32/Esp32UartPort.h` · **يرث من:** `IUartPort`

غلاف فوق `HardwareSerial`: `begin(baud)` ← `serial.begin(baud, SERIAL_8N1,
rx, tx)`؛ و`tx = -1` تعني الاستقبال فقط. والباقي تفويض.

---

## ‏`Esp32ServoOutput`

**الملف:** `hal/esp32/Esp32ServoOutput.h` · **يرث من:** `IServoOutput`

‏PWM مباشرة عبر LEDC (الدوال `ledcSetup/ledcAttachPin/ledcWrite` في Arduino core 2.x).
مكتبة ESP32Servo **غير مستخدمة**: فالإصدار 3.2.1 على S3 كان يخلط بين كتل MCPWM
(إذ كان GPIO6/7 يكرّران GPIO4/5).

| الثابت | القيمة |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (نحو 1.2 µs للخطوة) |

| الدالة | الوصف |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | الطرف `< 0` يعني أن المخرج غير موصول |
| `attach(minUs, maxUs)` | تحفظ المدى؛ الطرف < 0 ← `false`؛ وإلا `ledcSetup() != 0` ← `ledcAttachPin()` |
| `writeMicroseconds(us)` | إن لم يكن attached فلا شيء؛ وإلا `constrain(us, min, max) * MAX_DUTY / PERIOD_US` ← `ledcWrite` |
| `measurePulseUs()` | تفعّل مخزن الإدخال لنفس GPIO (`PIN_INPUT_ENABLE`، ولا يُمَسّ المخرج) وتقيس `pulseIn(pin, HIGH, 30 ms)`؛ وإن لم توجد نبضة ← `-1` |

القناتان 2n و2n+1 تتشاركان مؤقّت LEDC واحدًا — وجميع المخارج تعمل بتردد 50 Hz فلا تعارض.

---

# التنفيذ لمعالج STM32H743

اللوحة من الجيل التالي هي STM32H743VIT6 (نواة Cortex-M7 بتردد 480 MHz، وفلاش 2 MB،
وذاكرة RAM بحجم 1 MB). يُبنى البرنامج الثابت الكامل (البيئة `stm32h743` — لوحة PlatformIO
`weact_mini_h743vitx`، والبيئة `stm32h743-devebox` — لوحة DevEBox H743، والوحدة الطرفية عبر
USB CDC)، ويجتاز cppcheck والاختبارات على الحاسوب (البيئة `native-stm32` مع طبقة
محاكاة STM32duino). وعلى لوحة DevEBox **من دون مستشعرات** جرى التحقق من: الإقلاع، وبطاقة SD، والصندوق الأسود —
[اختبارات على اللوحة](../TESTING.md#الاختبارات-على-لوحة-stm32) — واستقبال iBUS، وARM، وPWM إلى
المؤازِرات والمحرك: تُقاد الطائرة من جهاز التحكم في الوضع اليدوي (وقد صُوِّر الإطلاق بالفيديو).
أما المستشعرات فلم توصل باللوحة بعد.
توزيع الأطراف في كتلة `BOARD_STM32H743` من [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

الفروق العامة عن ESP32 التي تخفيها هذه الطبقة:

- **النواة هي التي تختار الطرفيات.** تجد STM32duino المتحكم بنفسها (I2C1/I2C2،
  وSPI2، وUSART3، وUART4، وUART7، وTIMx) من أرقام الأطراف في جداول `PeripheralPins`
  الخاصة بالإصدار، ولذلك لا توجد أرقام UART أو القنوات في `Config.h`.
- **أرقام الأطراف** هي «أطراف Arduino» الخاصة بالإصدار (`PA0` و`PD14`…)، لا GPIO؛ وفي
  الأطراف التمثيلية تكون `0xC0 + N`، ولذلك فالأطراف في كتلة STM32 من النوع `int16_t`.
- **أطراف UART** تُحدَّد عند إنشاء الكائن `Uart(rx, tx)`، لا في `begin()`.

## ‏`Stm32Board`

**الملف:** `hal/stm32/Stm32Board.h` · **يرث من:** `IBoard`

هي نفسها `Esp32Board`، لكن فوق STM32duino.

| الحقل | النوع | ما هو |
|---|---|---|
| `displayWire` | `TwoWire` | متحكم I2C الثاني (فالكائن العام `Wire` تشغله المستشعرات). مُعلَن قبل `displayBus` الذي يحتفظ بمرجع إليه |
| `i2cBus` | `Stm32I2CBus` | `Wire` على `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10)، بتردد 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` على `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) — الناقل الثاني موجود دائمًا |
| `spiBus` | `Stm32SpiBus` | الكائن العام `SPI` على `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`، `rcPort` | `Uart`، `Stm32UartPort` | iBUS: UART7، والاستقبال RX على `PIN_IBUS` (PE7)، والإرسال TX على `PIN_IBUS_TX` (PE8، محجوز لـ iBUS-SENS) |
| `gpsSerial`، `gpsPort` | `Uart`، `Stm32UartPort` | GPS: USART3، `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`، `telemetryPort` | `Uart`، `Stm32UartPort` | مودم الراديو MAVLink: UART4، `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | بترتيب `ServoChannel` (AUX1 — PD15/TIM4، وAUX2 — PE9/TIM1) |

| الدالة | الوصف |
|---|---|
| `begin()` | `i2cBus.begin()`، ثم `spiBus.begin()`، ثم `displayBus.begin()`، وطرف الجرس |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | دائمًا `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | تحويل طرف من `Config.h` إلى نوع واجهة النواة البرمجية |
| الباقي | تُرجع الحقول المقابلة |

## ‏`Stm32I2CBus`

**الملف:** `hal/stm32/Stm32I2CBus.h` · **يرث من:** `II2CBus`

| الدالة | الوصف |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | تحفظ المعاملات |
| `begin()` | `setSDA()`/`setSCL()` (تؤثران قبل `begin()` فقط)، ثم `wire.begin()`، ثم `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`؛ ونتيجتها من النوع `size_t` تُحوَّل إلى `uint8_t` |
| الباقي | تفويض مباشر إلى `TwoWire` |

مهلة المعاملة في STM32duino ليست دالة بل الماكرو `I2C_TIMEOUT_TICK` (بالميلي ثانية،
والافتراضي 100). وفي البيئة `stm32h743` يُحدَّد بالراية `-D I2C_TIMEOUT_TICK=5` —
للسبب نفسه الذي من أجله `TIMEOUT_MS` في `Esp32I2CBus`.

## ‏`Stm32SpiBus`

**الملف:** `hal/stm32/Stm32SpiBus.h` · **يرث من:** `ISpiBus`

غلاف فوق `SPIClass&`. `begin()` ← `setSCLK/setMISO/setMOSI` ثم `spi.begin()`؛
ولا يُستخدم NSS العتادي — فالخط CS يبدّله `SpiRegisterDevice`، كما في ESP32.
وتبني `beginTransaction()` الكائن `SPISettings(hz, MSBFIRST, SPIMode)`؛ وتحوّل `spiModeOf()`
القيم 0..3 إلى `SPI_MODEn`، والقيمة المجهولة ← `SPI_MODE0`.

## ‏`Stm32UartPort`

**الملف:** `hal/stm32/Stm32UartPort.h` · **يرث من:** `IUartPort`

غلاف فوق `HardwareSerial&` (وهو في STM32duino 3.x الصنف الأساسي المجرد
`arduino::HardwareSerial`، أما الكائن الملموس `Uart` فتنشئه `Stm32Board`).
`begin(baud)` ← `serial.begin(baud, SERIAL_8N1)`؛ و`availableForWrite()` تأتي
من `HardwareSerial`. المخازن المؤقتة (`SERIAL_RX/TX_BUFFER_SIZE` في البيئة): الاستقبال 256
بايت (إطار NAV-PVT حجمه 100، والـ 64 القياسية قليلة)، والإرسال 1024 (أسطر السجل
وإطارات MAVLink دون انتظار).

## ‏`Stm32ServoOutput`

**الملف:** `hal/stm32/Stm32ServoOutput.h` · **يرث من:** `IServoOutput`

‏PWM عتادي للمؤقّت عبر `HardwareTimer`، بتردد 50 Hz. تُولَّد النبضة بواسطة المؤقّت
دون مقاطعات ودون المعالج — بخلاف مكتبة `Servo` الخاصة بـ STM32 التي
تقلب الأطراف من مقاطعة مؤقّت واحد وتُحدث اهتزازًا في التوقيت.

| الثابت | القيمة |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — كم مؤقّتًا مختلفًا يمكن أن تشغله المخارج (المشغول الآن TIM2 وTIM4) |

| الدالة | الوصف |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | الطرف `< 0` يعني أن المخرج غير موصول |
| `attach(minUs, maxUs)` | يؤخذ المؤقّت والقناة من `PinMap_TIM` بحسب الطرف (`pinmap_peripheral`، `STM_PIN_CHANNEL`)، كما في `analogWrite()`. إن لم يكن على الطرف مؤقّت أو نفد المخزون ← `false`. وإلا `setMode(PWM1)`، والمقارنة 0 (لا نبضة حتى أول كتابة)، ثم `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` ← `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. سجل المقارنة مُسبَق التحميل — فتسري القيمة من الدورة التالية |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` دون إعادة ضبط الطرف: ففي STM32 يرى السجل IDR المستوى حتى في وضع الوظيفة البديلة |
| `static acquireTimer(TIM_TypeDef*)` | مخزون مشترك: **مؤقّت `HardwareTimer` واحد لكل TIMx**. فكائن ثانٍ على المؤقّت نفسه سيكتب فوق معالج النواة (`HardwareTimer_Handle[index]`). وتُحدَّد الدورة عند أول مخرج على المؤقّت؛ وتختار `setOverflow(MICROSEC_FORMAT)` المقسّم — بخطوة نحو 0.3 µs عند ساعة مؤقّت 240 MHz |

## ‏`Stm32FlashStorage`

**الملف:** `hal/stm32/Stm32FlashStorage.h` · **يرث من:** `IFlashStorage` ([storage.md](storage.md))

وسيط `KeyValueStore` في STM32: آخر قطاع في الفلاش (البنك 2) عبر محاكاة
EEPROM في STM32duino (`eeprom_buffer_fill/flush`، ومخزن مؤقت حجمه 8 KB تُستخدم
منه أول `KeyValueStore::CAPACITY` بايت).

| الدالة | الوصف |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` ثم قراءة المخزن المؤقت بايتًا بايتًا |
| `write(src, n)` | **سريعة**: نسخ الصورة إلى مخزنها المؤقت تحت `noInterrupts()`، ورفع راية «توجد كتابة معلّقة». تُستدعى من `KvPreferences::end()` في مهمة الطيران |
| `bool service()` | **بطيئة**: لقطة إلى مخزن المحاكاة المؤقت (تحت `noInterrupts()`) ثم `eeprom_buffer_flush()` — مسح قطاع حجمه 128 KB (ثوانٍ) والكتابة. من مهمة الخلفية `storage` فقط |
| `hasPending()`، `flushCount()` | تشخيص |
| `static instance()`، `static store()` | الوسيط و`KeyValueStore` المشترك للبرنامج الثابت |

لماذا لا يتجمد الطيران: قطاع الإعدادات في البنك 2 والشيفرة في البنك 1، وفلاش H7
يقرأ من بنك واحد بينما يُكتب في الآخر؛ وتستبق مهمةُ الطيران مهمةَ الخلفية.

## ‏`compat/Preferences.h`

**الملف:** `hal/stm32/compat/Preferences.h` — في البيئة `stm32h743` (وفي
`native-stm32`) يقع الدليل `compat/` في `-I` قبل المكتبات، فيجده
`#include <Preferences.h>` في مشغّلات المستشعرات والمعدِّل التلقائي وإعدادات السجل.
`class Preferences : public KvPreferences` فوق
`Stm32FlashStorage::store()` — بواجهة برمجية مماثلة لـ NVS في ESP32 ([storage.md](storage.md#kvpreferences)).

## ‏`Stm32SdCard`

**الملف:** `hal/stm32/Stm32SdCard.h` · **يرث من:** `IBlockDevice` · **الأطراف:** `src/stm32/sd_msp.cpp`

بطاقة SD على SDMMC1: ناقل من 4 بتّات، و`HAL_SD` بنمط الاستطلاع (دون DMA ودون
مقاطعات) **مع تحكم عتادي في التدفق**: فمهمة الطيران تستبق مهمة الكتابة في منتصف كتلة،
ومن دونه كان FIFO يفيض (`HAL_SD_ERROR_RX_OVERRUN`، 0x20) — وعلى اللوحة ظهر ذلك في صورة وحدة طرفية
وكتابة تتجمدان لثوانٍ. الأطراف PC8..PC11 (D0..D3) وPC12 (CK) وPD2 (CMD) هي فتحة µSD في
DevEBox وWeAct. وتُغذّى نواة SDMMC بالساعة من PLL1Q = 48 MHz، و`ClockDiv = 1` ←
**24 MHz**؛ وإن فشلت أول قراءة عند 24 MHz جُرّبت 12 ثم 6.

| العضو | الوصف |
|---|---|
| `bool begin()` | تشغيل الناقل وتعرّف البطاقة وقراءة تجريبية. `false` تعني لا بطاقة؛ و`initError()` — الرمز |
| `read` / `write` | بقطع لا تزيد على 4 KB (مع توقفات قصيرة)؛ والعنوان غير المضاعف للعدد 4 يُنسخ عبر مخزن مؤقت محاذى (تقرأ HAL الـ FIFO بالكلمات). وعند الفشل — محاولة واحدة أخرى |
| الانتظار | قبل أي وصول بعد الكتابة ينتظر عودة البطاقة إلى حالة النقل (`Rtos::sleepMs(1)`: فلا تجوع مهام الخلفية)، حتى 1 s. وبعد القراءة لا يُرسل طلب حالة زائد — فالمطابقة عند التشغيل تقرأ عشرات الآلاف من القطاعات |
| `blockCount()`، `cardType()`، `clockDivider()`، `lastErrorCode()` | لسطر الحالة |
| `readOps`، `writeOps`، `errors`، `retries` | العدّادات |

## ‏`ResetCause`

**الملف:** `hal/ResetCause.h` · `readResetCause()`، `isCrashReset()`، `resetCauseName()`

سبب إعادة التشغيل، متطابق على اللوحتين. في ESP32 — `esp_reset_reason()`؛
وفي STM32 — رايات `RCC->RSR` (تُقرأ مرة واحدة ثم تُمسح؛ وفي H7 تُضبط `PINRSTF`
عند أي إعادة ضبط، ولذلك تُفحص الأسباب الأكثر تحديدًا أولًا:
مؤقّت المراقبة ← التشغيل ← هبوط الجهد ← إعادة الضبط البرمجية).
والذعر البرمجي (panic) ومؤقّتات المراقبة وهبوط الطاقة تُعدّ «عطلًا»: فيبدأ الصندوق الأسود التسجيل فورًا عند حدوثها.

## ‏`Rtos`

**الملف:** `hal/Rtos.h` · namespace

| العضو | الوصف |
|---|---|
| `PRIORITY_BACKGROUND` (1)، `PRIORITY_TELEMETRY` (2)، `PRIORITY_FLIGHT` (5) | أولويات المهام |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | في ESP32 — `xTaskCreatePinnedToCore(..., النواة 0)` والمكدّس بالبايت؛ وفي STM32 — `xTaskCreate` ويُحوَّل المكدّس إلى كلمات؛ و`handle` من أجل `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`؛ وقبل بدء المجدول (الدالة `setup()` في STM32) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: في ESP32 — قفل الدوران `portMUX`، وفي STM32 — `taskENTER_CRITICAL()`. وفي داخله نسخ بايتات فقط (طابور الصندوق الأسود) |
| `uint32_t freeHeapBytes()` | في ESP32 — `ESP.getFreeHeap()`؛ وفي STM32 — `xPortGetFreeHeapSize()` |

## نقطة الدخول `src/stm32/main.cpp`

البرنامج الثابت الكامل: الكائنات نفسها الموجودة في `src/main.cpp`، مع قياسات MAVLink عن بُعد بدل
Wi-Fi، ومهام FreeRTOS بدل `loop()` — [application.md](application.md#srcstm32maincpp--stm32h743).

</div>
