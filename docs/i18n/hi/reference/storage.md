# STORAGE — NVS के बिना सेटिंग

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/storage.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

सेंसरों के कैलिब्रेशन, IMU की माउंटिंग, ऑटो-ट्रिम और लॉग की सेटिंग `Preferences` API
के ज़रिए सहेजे जाते हैं। ESP32 पर यह NVS है; NVS के बिना बोर्डों (STM32) पर —
फ़्लैश के सेक्टर में “कुंजी → बाइट” की इमेज। परत का कोड पोर्टेबल है (Arduino के बिना)
और PC पर जाँचा जाता है (`test/native/test_storage`), और STM32 पर — पूरे फ़र्मवेयर
के हिस्से के रूप में भी (`test/native_stm32`)।

---

## `IFlashStorage`

**फ़ाइल:** `storage/KeyValueStore.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), टेस्ट में RAM के माध्यम

| मेथड | विवरण |
|---|---|
| `size_t capacity() const` | माध्यम पर बाइट |
| `void read(uint8_t* dst, size_t n)` | शून्य ऑफ़सेट से पढ़ना |
| `bool write(const uint8_t* src, size_t n)` | पूरी इमेज लिखना |

## `KeyValueStore`

**फ़ाइल:** `storage/KeyValueStore.h`

पूरी इमेज (`CAPACITY` = 2048 बाइट) RAM में रहती है; फ़्लैश एक बार पढ़ा जाता है
(`mount()`, आलसी ढंग से) और `commit()` में पूरा दोबारा लिखा जाता है।

```
हेडर 12 बाइट:      "OPKV" | संस्करण u16 | प्रयुक्त u16 | रिकॉर्डों का CRC32 u32
रिकॉर्ड लगातार:    [len ns u8][len key u8][len value u16][ns][key][value]
```

| मेथड | विवरण |
|---|---|
| `void mount()` | इमेज पढ़ना और जाँचना: साफ़ फ़्लैश (0xFF) — ख़ाली; मैजिक नहीं, पराया संस्करण, माध्यम से बड़ी लंबाई, CRC या रिकॉर्डों की संरचना मेल नहीं खाती — `wasCorrupt()`, ख़ाली |
| `const uint8_t* get(ns, key, size_t* len)` | मान या `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | वही मान — फ़्लैश को नहीं छुआ जाता (घिसाव नहीं बढ़ता); जगह कम पड़ी — `false`, पुराना मान सुरक्षित |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | बदला हो तो लिखना; माध्यम की त्रुटि — `isDirty()` बना रहता है |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | निदान |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

नेमस्पेस और कुंजियों के नाम — 1..15 अक्षर, जैसे NVS में।

## `KvPreferences`

**फ़ाइल:** `storage/KvPreferences.h`

`KeyValueStore` के ऊपर ESP32 की `Preferences` क्लास का API — वह उपसमुच्चय जो
प्रोजेक्ट इस्तेमाल करता है: `begin(name, readOnly)`, `end()` (बदला हो तो लिखता है),
`clear`, `remove`, `isKey`, `Bool`, `UChar`, `Int`, `UInt`, `Float`, `Bytes` के लिए
`put/get`, `getBytesLength`।

व्यवहार मूल जैसा है: मौजूद न होने वाले नेमस्पेस के लिए `begin(name, true)` —
`false`; कुंजी नहीं या आकार अलग — डिफ़ॉल्ट मान (`getFloat` — `NAN`); छोटे बफ़र में
`getBytes` — 0; `begin()` के बिना कुछ नहीं पढ़ा जाता और कुछ नहीं लिखा जाता।

STM32 पर `class Preferences : public KvPreferences` —
`hal/stm32/compat/Preferences.h`।

---

## `Fat32::locate`

**फ़ाइल:** `storage/Fat32File.h` · नेमस्पेस `Fat32`

FAT32 **केवल पढ़ने के लिए**: कार्ड की रूट में फ़ाइल ढूँढना और बताना कि वह कहाँ है।
फ़र्मवेयर फ़ाइल सिस्टम में कुछ नहीं बनाता और कुछ नहीं बदलता।

| फ़ंक्शन | विवरण |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | FAT32 पार्टीशन (प्रकार 0x0B/0x0C) वाला MBR या पार्टीशन तालिका के बिना FAT32; सेक्टर 512 बाइट; नाम 8.3। रूट की क्लस्टर-शृंखला पर चलता है, वॉल्यूम लेबल, डायरेक्टरी, LFN और मिटाई गई प्रविष्टियों को छोड़ते हुए, और जाँचता है कि फ़ाइल के क्लस्टर **लगातार** हैं |
| `Extent` | `firstBlock` — कार्ड का वह ब्लॉक जहाँ से डेटा शुरू होता है; `bytes` — फ़ाइल का आकार |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | कारण रूसी में, क्या करना है इसके संकेत के साथ |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
