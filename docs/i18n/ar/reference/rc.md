<div dir="rtl">

# ‏RC — استقبال أوامر جهاز التحكم

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/rc.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

تحوّل طبقة RC بايتات UART إلى قيم القنوات وإلى علامة «لا اتصال». وهي لا تعرف
شيئًا عن الطائرة وARM وسلوك failsafe والسيرفوات — فاستبدال البروتوكول
(S-Bus أو PPM) لا يمس غير هذه الطبقة.

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | خطي: 1000 ← `−max`، و1500 ← 0، و2000 ← `+max` (يُقيَّد المدخل أولًا)؛ و`reverse` يقلب الإشارة. والنتيجة مقيَّدة بـ ±`max` |

مثال: `centered(1750, 500) == 250`، و`centered(1750, 500, true) == -250`.

---

## ‏`IBusReceiver`

**الملف:** `rc/IBusReceiver.h` · **يعتمد على:** `IUartPort` و`RcChannelState` و`Config` و`Channels`

محلل بايتًا بايتًا لبروتوكول FlySky iBUS.

**صيغة الإطار** (32 بايت): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`،
و`CRC = 0xFFFF − Σ(first 30 bytes)`. تُؤخذ أول `IBUS_CHANNELS` = 10 قنوات؛
وقيمة القناة هي **الـ 12 بتًّا الدنيا** (وفي البتات العليا يرسل FS-iA6B بيانات
خدمة، مثلًا في failsafe ‏`0x2384` ← 900 µs).

| الدالة | الوصف |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | لا يُفتح المنفذ في المُنشئ |
| `void begin()` | ‏`serial.begin(IBUS_BAUDRATE)`، ويبدأ عدّ المهلة من «الآن» |
| `void update()` | قراءة كل ما تراكم في UART؛ يُستدعى في كل دورة |
| `const RcChannelState& getState() const` | آخر القنوات المستلمة |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | لم يصل أي إطار بعد **أو** أن الأخير أقدم من `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | في الإطار الأخير الخانق < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | قيمة `micros()` لآخر إطار صحيح |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | عدّادا الإطارات الصحيحة وأخطاء CRC |

آلة حالات التحليل (`processByte`): تنتظر `0x20`؛ ويجب أن يكون البايت التالي
`0x40`، وإلا يبدأ البحث من جديد؛ ثم تجمع 32 بايتًا وتستدعي
`processFrame()`. ويُهمَل الإطار ذو CRC الخاطئ كاملًا (لا تتغير القنوات،
و`badFrames++`).

الثوابت:

- حتى أول إطار صحيح تكون `isSignalLost() == true` — فلا تُؤخذ القيم الافتراضية
  (كلها 1500) على أنها أوامر من جهاز التحكم.
- تُعاد حسابات علامة failsafe عند **كل** إطار صحيح — فيعود الاتصال بأول
  إطار يحمل خانقًا طبيعيًّا.

</div>
