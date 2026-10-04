<div dir="rtl">

# ‏STORAGE — إعدادات دون NVS

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/storage.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي.

[→ المرجع](README.md)

تُحفظ معايرات الحساسات وتركيب IMU والضبط التلقائي وإعدادات السجل عبر واجهة
`Preferences`. وهي في ESP32 NVS؛ أما في اللوحات التي لا NVS فيها (STM32) فهي صورة
«مفتاح ← بايتات» في قطاع من الفلاش. وشيفرة هذه الطبقة قابلة للنقل (دون Arduino)
وتُختبر على الحاسوب (`test/native/test_storage`)، وفي STM32 أيضًا ضمن البرنامج
الثابت كله (`test/native_stm32`).

---

## ‏`IFlashStorage`

**الملف:** `storage/KeyValueStore.h` · **النوع:** واجهة · **التنفيذات:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage))، ووسائط RAM في الاختبارات

| الدالة | الوصف |
|---|---|
| `size_t capacity() const` | عدد البايتات على الوسيط |
| `void read(uint8_t* dst, size_t n)` | القراءة من الإزاحة صفر |
| `bool write(const uint8_t* src, size_t n)` | كتابة الصورة كاملة |

## ‏`KeyValueStore`

**الملف:** `storage/KeyValueStore.h`

الصورة كلها (`CAPACITY` = 2048 بايت) في الذاكرة العشوائية؛ وتُقرأ الفلاش مرة
واحدة (`mount()`، عند الحاجة) وتُعاد كتابتها كاملة في `commit()`.

<div dir="ltr">

```
الترويسة 12 بايت:  "OPKV" | الإصدار u16 | المستخدم u16 | CRC32 للسجلات u32
السجلات تباعًا:     [len ns u8][len key u8][len value u16][ns][key][value]
```

</div>

| الدالة | الوصف |
|---|---|
| `void mount()` | قراءة الصورة والتحقق منها: فلاش نظيفة (0xFF) — فارغة؛ لا علامة سحرية، أو إصدار غريب، أو طول أكبر من الوسيط، أو CRC أو بنية السجلات لا تتطابق — `wasCorrupt()`، فارغة |
| `const uint8_t* get(ns, key, size_t* len)` | القيمة أو `nullptr` |
| `contains(ns, key)` و`hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | القيمة نفسها — لا تُمَسّ الفلاش (فلا يزداد التآكل)؛ ضاق المكان — `false`، وتبقى القيمة القديمة سليمة |
| `remove(ns, key)` و`clear(ns)` | |
| `bool commit()` | الكتابة إن تغيّرت؛ خطأ في الوسيط — يبقى `isDirty()` |
| `bytesUsed()` و`isDirty()` و`wasCorrupt()` و`commitCount()` | التشخيص |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

أسماء المساحات والمفاتيح — من 1 إلى 15 حرفًا، كما في NVS.

## ‏`KvPreferences`

**الملف:** `storage/KvPreferences.h`

واجهة الصنف `Preferences` الخاصة بـ ESP32 فوق `KeyValueStore` — المجموعة الجزئية
التي يستخدمها المشروع: `begin(name, readOnly)`، و`end()` (تكتب إن تغيّرت)،
و`clear` و`remove` و`isKey`، و`put/get` للأنواع `Bool` و`UChar` و`Int` و`UInt`
و`Float` و`Bytes`، و`getBytesLength`.

السلوك كما في الأصل: ‏`begin(name, true)` لمساحة غير موجودة — `false`؛ ولا مفتاح
أو حجم مختلف — القيمة الافتراضية (‏`getFloat` — `NAN`)؛ و`getBytes` إلى مخزن أصغر — 0؛
ودون `begin()` لا يُقرأ شيء ولا يُكتب.

وفي STM32 ‏`class Preferences : public KvPreferences` موجود في
`hal/stm32/compat/Preferences.h`.

---

## ‏`Fat32::locate`

**الملف:** `storage/Fat32File.h` · مساحة الأسماء `Fat32`

‏FAT32 **للقراءة فقط**: إيجاد ملف في جذر البطاقة وتحديد موضعه.
ولا ينشئ البرنامج الثابت شيئًا في نظام الملفات ولا يغيّر فيه شيئًا.

| الدالة | الوصف |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | MBR ذو قسم FAT32 (النوع 0x0B/0x0C) أو FAT32 دون جدول أقسام؛ والقطاع 512 بايت؛ والاسم 8.3. يمشي على سلسلة عناقيد الجذر متجاوزًا تسمية القرص والأدلة وLFN والمدخلات المحذوفة، ويتحقق من أن عناقيد الملف **متتابعة** |
| `Extent` | ‏`firstBlock` — كتلة البطاقة التي تبدأ منها البيانات؛ و`bytes` — حجم الملف |
| `Result` | `Ok` و`NoCard` و`ReadError` و`NotFat32` و`NotFound` و`Fragmented` و`Empty` |
| `const char* describe(Result)` | السبب بالروسية، مع تلميح بما ينبغي فعله |
| `bool shortName(name, out[11])` | `blackbox.bin` ← `BLACKBOXBIN ` |

</div>
