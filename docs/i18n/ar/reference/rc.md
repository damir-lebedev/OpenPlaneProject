<div dir="rtl">

# ‏RC — استقبال أوامر جهاز التحكم

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/rc.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

تحوّل طبقة RC بايتات UART إلى قيم القنوات وإلى علامة «لا اتصال». وهي لا تعرف
شيئًا عن الطائرة وARM وسلوك failsafe والسيرفوات. يحدّد `Config::RX_PROTOCOL`
بروتوكول المستقبِل — iBUS أو SBUS أو CRSF؛ والبروتوكول الجديد ليس سوى محلّل
إطارات آخر بجانب المحلّلات الثلاثة الموجودة.

---

## ‏`RcChannelState`

**الملف:** `rc/RcChannelState.h` · **يعتمد على:** `Config` و`Channels`

لقطة للقنوات الـ 10 للمستقبِل (µs)، دون منطق تحكم.

| الدالة | الوصف |
|---|---|
| `RcChannelState()` | يستدعي `reset()` |
| `void reset()` | قيم آمنة: جميع القنوات `PWM_CENTER`، والخانق — `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | قيمة القناة؛ فهرس خارج المدى ← `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | كتابة قناة؛ ويُتجاهل الفهرس الخارج عن المدى |
| `const uint16_t* data() const` | المصفوفة كاملة (للتصحيح) |

---

## ‏`RcInput`

**الملف:** `rc/RcInput.h` · **النوع:** مجموعة دوال ساكنة · **يعتمد على:** `Config`

تحويلات مشتركة لإشارات RC.

| الدالة | الوصف |
|---|---|
| `static uint16_t clamp(uint16_t value)` | التقييد بين `PWM_MIN..PWM_MAX` |
| `static uint16_t reversed(uint16_t value)` | القناة بالعكس: `3000 − clamp(value)` ‏(1000 ↔ 2000) — لأجل `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | خطي: 1000 ← `−max`، و1500 ← 0، و2000 ← `+max` (يُقيَّد المدخل أولًا)؛ و`reverse` يقلب الإشارة. والنتيجة مقيَّدة بـ ±`max` |

مثال: `centered(1750, 500) == 250`، و`centered(1750, 500, true) == -250`.

---

## ‏`RcReceiver`

**الملف:** `rc/RcReceiver.h` · **يعتمد على:** `IUartPort` و`IBusParser` و`SbusParser` و`CrsfParser` و`RcChannelState` و`RcInput` و`Config` و`Channels`

مستقبِل جهاز التحكم: يفتح UART بصيغة البروتوكول، ويسلّم البايتات إلى المحلّل،
ويتولّى ما تشترك فيه كل البروتوكولات — أول `RC_CHANNELS` = 10 قنوات، وعكس
القنوات، ومهلة الإطارات، وfailsafe، والعدّادات.

| البروتوكول (`Config::RX_PROTOCOL`) | المستقبِلات | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B وFS-iA10B | 115200، 8N1 |
| `SBUS` | FrSky وFutaba وRadioMaster وJumper ومعظم الأنواع الأخرى | 100000، 8E2، إشارة معكوسة — يعكسها UART نفسه |
| `CRSF` | ExpressLRS وTBS Crossfire وTracer | 420000، 8N1 |

| الدالة | الوصف |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | لا يُفتح المنفذ في المُنشئ؛ ويمكن في الاختبارات تمرير أي بروتوكول |
| `void begin()` | فتح UART بصيغة البروتوكول (`IUartPort::begin(baud, format)`)، وبدء عدّ المهلة من «الآن» |
| `void update()` | قراءة كل ما تراكم في UART؛ يُستدعى في كل دورة |
| `const RcChannelState& getState() const` | آخر القنوات المستلمة |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | لم يصل أي إطار بعد **أو** أن الأخير أقدم من `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | أبلغ المستقبِل عن فقدان الاتصال: علامة failsafe في إطار SBUS، أو جودة اتصال 0 % في CRSF، أو خانق < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | قيمة `micros()` لآخر إطار صحيح |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | عدّادا الإطارات المقبولة والمرفوضة (CRC، نهاية الإطار) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | البروتوكول؛ واسمه `"iBUS"` أو `"SBUS"` أو `"CRSF"` |
| `int16_t getLinkQuality() const` | جودة الاتصال اللاسلكي، % — في CRSF فقط، وإلا −1 |
| `bool isPortReady() const` | فُتح UART بالصيغة المطلوبة؛ `false` — المنفذ لا يستطيع ذلك (SBUS دون عكس)، فلن يكون هناك اتصال |

الدالة الحرة `rxProtocolName(Config::RxProtocol)` تعطي اسم البروتوكول؛ وهو نفسه ما
يطبعه جواز البرنامج الثابت (`@id`، الحقل `rx`).

الثوابت:

- حتى أول إطار صحيح تكون `isSignalLost() == true` — فلا تُؤخذ القيم الافتراضية
  (كلها 1500) على أنها أوامر من جهاز التحكم.
- تُعاد حسابات علامة failsafe عند **كل** إطار قنوات — فيعود الاتصال مع أول إطار
  بخانق طبيعي (وفي CRSF أيضًا مع إطار إحصاءات جودة الاتصال فيه أكبر من 0).
- تُحفظ القنوات ذات `Config::CHANNEL_REVERSED` معكوسة (`RcInput::reversed`)؛
  ويُكتشف failsafe قبل ذلك من الخانق **الخام**، ويمنع `static_assert` عكس CH3.

---

## ‏`IBusParser`

**الملف:** `rc/IBusParser.h` · **يعتمد على:** `Config`

محلّل FlySky iBUS بايتًا بايتًا: تُرجع `bool feed(uint8_t)` القيمة `true` عند اكتمال إطار
بـCRC صحيح؛ والقنوات في `values()`. **الإطار** (32 بايتًا): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`،
و`CRC = 0xFFFF − Σ(first 30 bytes)`؛ وقيمة القناة هي **الـ 12 بتًا الدنيا** (في العليا
يرسل FS-iA6B بيانات خدمية، مثلًا في failsafe ‏`0x2384` ← 900 µs). ينتظر `0x20`، ويجب
أن يكون البايت التالي `0x40`، وإلا يبدأ البحث من جديد (فقد يبدأ `0x20` ثانٍ إطارًا
بنفسه). لا توجد في iBUS علامة failsafe: يبلغ FS-iA6B عن فقدان الاتصال بخانق أقل
من الطبيعي.

---

## ‏`SbusParser`

**الملف:** `rc/SbusParser.h` · **يعتمد على:** `Packed11`

**الإطار** (25 بايتًا): `0x0F | 16 قناة كل منها 11 بتًا (22 بايتًا) | العلامات | النهاية`؛
العلامات: البت 2 — ضاع إطار واحد عبر الراديو (وهذا ليس فقدانًا للاتصال بعد)، والبت 3
— failsafe؛ والنهاية — `0x00` في SBUS، و`0x04`/`0x14`/`0x24`/`0x34` في SBUS2. يظهر
البايت `0x0F` في البيانات أيضًا، لذا لا يُقبل الإطار إلا إذا كانت بدايته ونهايته في
مكانهما؛ وإلا يستمر التحليل من `0x0F` التالي داخل ما استُلم — فيتزامن التدفق خلال
إطار أو اثنين.

---

## ‏`CrsfParser`

**الملف:** `rc/CrsfParser.h` · **يعتمد على:** `Packed11`

**الإطار:** `0xC8 | الطول | النوع | البيانات | CRC8`؛ الطول = النوع + البيانات + CRC،
والإطار لا يتجاوز 64 بايتًا؛ وCRC8 بكثيرة الحدود `0xD5` ‏(DVB-S2) على النوع والبيانات
(`static uint8_t crc8(...)`). يلزم نوعان: `0x16` — القنوات (16 قناة من 11 بتًا كما في
SBUS) و`0x14` — إحصاءات الاتصال (البايت الثالث هو جودة الاتصال، %). عند فقدان الاتصال
يتوقف ExpressLRS افتراضيًا عن إرسال القنوات — وهذه مهلة الإطارات؛ وفي وضع «آخر
المواضع» تستمر القنوات في الوصول لكن جودة الاتصال 0 — وهذا هو `failsafe()`.

---

## ‏`Packed11`

**الملف:** `rc/Packed11.h` · **النوع:** فضاء أسماء

16 قناة كل منها 11 بتًا في 22 بايتًا، البتات الدنيا أولًا — هكذا يحزمها SBUS
وCRSF كلاهما. `toMicros(raw)`: ‏172…1811 ← 988…2012 µs، و992 ← 1500 (`5/8 · value + 880`،
كما في Betaflight وINAV)؛ و`unpack(data, out)` يحوّل 22 بايتًا إلى 16 قيمة بالـµs.

</div>
