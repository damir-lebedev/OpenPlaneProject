# RC — ट्रांसमीटर की कमांड प्राप्त करना

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/rc.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है। यह अनुवाद AI ने किया है और मूल भाषा के जानकारों ने इसकी जाँच नहीं की है। त्रुटियाँ मिलें तो [Damir Lebedev](https://github.com/damir-lebedev) को लिखें या [इश्यू ट्रैकर](https://github.com/damir-lebedev/OpenPlaneProject/issues) में बताएँ।

[← संदर्भ](README.md)

RC परत UART के बाइटों को चैनल के मानों और “संपर्क नहीं है” के संकेत में बदलती है।
उसे विमान, ARM, failsafe के व्यवहार और सर्वो के बारे में कुछ नहीं पता। रिसीवर का
प्रोटोकॉल — iBUS, SBUS या CRSF — `Config::RX_PROTOCOL` तय करता है; नया प्रोटोकॉल
मौजूदा तीन के पास बस एक और फ़्रेम पार्सर है।

---

## `RcChannelState`

**फ़ाइल:** `rc/RcChannelState.h` · **निर्भर है:** `Config`, `Channels`

रिसीवर के 10 चैनलों का स्नैपशॉट (µs), नियंत्रण के तर्क के बिना।

| मेथड | विवरण |
|---|---|
| `RcChannelState()` | `reset()` बुलाता है |
| `void reset()` | सुरक्षित मान: सभी चैनल `PWM_CENTER`, थ्रॉटल — `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | चैनल का मान; सीमा से बाहर का इंडेक्स → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | चैनल लिखना; सीमा से बाहर का इंडेक्स अनदेखा किया जाता है |
| `const uint16_t* data() const` | पूरा ऐरे (डीबगिंग के लिए) |

---

## `RcInput`

**फ़ाइल:** `rc/RcInput.h` · **प्रकार:** स्थैतिक फ़ंक्शनों का समूह · **निर्भर है:** `Config`

RC सिग्नलों के साझा रूपांतरण।

| मेथड | विवरण |
|---|---|
| `static uint16_t clamp(uint16_t value)` | `PWM_MIN..PWM_MAX` तक सीमित करना |
| `static uint16_t reversed(uint16_t value)` | चैनल उल्टा: `3000 − clamp(value)` (1000 ↔ 2000) — `Config::CHANNEL_REVERSED` के लिए |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | रैखिक: 1000 → `−max`, 1500 → 0, 2000 → `+max` (इनपुट पहले सीमित होता है); `reverse` चिह्न बदलता है। नतीजा ±`max` तक सीमित |

उदाहरण: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`।

---

## `RcReceiver`

**फ़ाइल:** `rc/RcReceiver.h` · **निर्भर है:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

रिमोट का रिसीवर: प्रोटोकॉल के फ़ॉर्मैट में UART खोलता है, बाइट पार्सर को देता है और
वह काम करता है जो सभी प्रोटोकॉल में साझा है — पहले `RC_CHANNELS` = 10 चैनल, चैनल
रिवर्स, फ़्रेम टाइमआउट, failsafe, काउंटर।

| प्रोटोकॉल (`Config::RX_PROTOCOL`) | रिसीवर | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper और अधिकांश अन्य | 100000, 8E2, उलटा सिग्नल — उलटना UART ख़ुद करता है |
| `CRSF` | ExpressLRS, TBS Crossfire और Tracer | 420000, 8N1 |

| मेथड | विवरण |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | कंस्ट्रक्टर में पोर्ट नहीं खुलता; टेस्ट में कोई भी प्रोटोकॉल दिया जा सकता है |
| `void begin()` | प्रोटोकॉल के फ़ॉर्मैट में UART खोलना (`IUartPort::begin(baud, format)`), टाइमआउट “अभी” से गिनना |
| `void update()` | UART में जमा सब कुछ पढ़ना; हर टिक पर बुलाएँ |
| `const RcChannelState& getState() const` | आख़िरी मिले चैनल |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | अभी तक एक भी फ़्रेम नहीं आया **या** आख़िरी फ़्रेम `RX_TIMEOUT_US` से पुराना है |
| `bool isFailsafeReported() const` | रिसीवर ने संपर्क टूटने की सूचना दी: SBUS फ़्रेम में failsafe फ़्लैग, CRSF में लिंक गुणवत्ता 0 %, या थ्रॉटल < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | आख़िरी सही फ़्रेम का `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | स्वीकार और अस्वीकार (CRC, फ़्रेम का अंत) फ़्रेमों के काउंटर |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | प्रोटोकॉल; नाम — `"iBUS"`, `"SBUS"`, `"CRSF"` |
| `int16_t getLinkQuality() const` | रेडियो लिंक की गुणवत्ता, % — केवल CRSF, वरना −1 |
| `bool isPortReady() const` | UART ज़रूरी फ़ॉर्मैट में खुला; `false` — पोर्ट ऐसा नहीं कर सकता (बिना उलटने के SBUS), संपर्क नहीं होगा |

स्वतंत्र फ़ंक्शन `rxProtocolName(Config::RxProtocol)` प्रोटोकॉल का नाम देता है; फ़र्मवेयर
का पासपोर्ट (`@id`, फ़ील्ड `rx`) यही छापता है।

अपरिवर्तनीय नियम:

- पहले सही फ़्रेम तक `isSignalLost() == true` — डिफ़ॉल्ट मान (सब 1500) रिमोट के
  आदेश नहीं माने जाते।
- failsafe का संकेत चैनलों के **हर** फ़्रेम पर दोबारा गिना जाता है — सामान्य थ्रॉटल
  वाले पहले ही फ़्रेम से संपर्क लौट आता है (CRSF में साथ ही 0 से अधिक लिंक गुणवत्ता
  वाले आँकड़ों से भी)।
- `Config::CHANNEL_REVERSED` वाले चैनल उलटे रूप में सहेजे जाते हैं
  (`RcInput::reversed`); failsafe उससे पहले **कच्चे** थ्रॉटल से पहचाना जाता है, और
  CH3 का रिवर्स `static_assert` से मना है।

---

## `IBusParser`

**फ़ाइल:** `rc/IBusParser.h` · **निर्भर है:** `Config`

FlySky iBUS का बाइट-दर-बाइट पार्सर: `bool feed(uint8_t)` सही CRC वाला फ़्रेम पूरा होने पर
`true` देता है; चैनल `values()` में। **फ़्रेम** (32 बाइट): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(पहले 30 बाइट)`; चैनल का मान — **निचले 12 बिट** (ऊपरी बिटों में FS-iA6B
सेवा डेटा भेजता है, जैसे failsafe में `0x2384` → 900 µs)। `0x20` का इंतज़ार करता है;
अगला बाइट `0x40` होना चाहिए, वरना खोज फिर से शुरू (दूसरा `0x20` ख़ुद फ़्रेम शुरू कर सकता
है)। iBUS में failsafe फ़्लैग नहीं है: FS-iA6B सामान्य से कम थ्रॉटल से संपर्क टूटने की सूचना
देता है।

---

## `SbusParser`

**फ़ाइल:** `rc/SbusParser.h` · **निर्भर है:** `Packed11`

**फ़्रेम** (25 बाइट): `0x0F | 11 बिट के 16 चैनल (22 बाइट) | फ़्लैग | अंत`; फ़्लैग: बिट 2 —
रेडियो पर एक फ़्रेम खोया (यह अभी संपर्क टूटना नहीं), बिट 3 — failsafe; अंत — SBUS में
`0x00`, SBUS2 में `0x04`/`0x14`/`0x24`/`0x34`। बाइट `0x0F` डेटा में भी आता है, इसलिए फ़्रेम
तभी स्वीकार होता है जब शुरुआत और अंत दोनों अपनी जगह हों; वरना पार्सिंग मिले डेटा के
अंदर अगले `0x0F` से जारी रहती है — धारा एक-दो फ़्रेम में तालमेल पा लेती है।

---

## `CrsfParser`

**फ़ाइल:** `rc/CrsfParser.h` · **निर्भर है:** `Packed11`

**फ़्रेम:** `0xC8 | लंबाई | प्रकार | डेटा | CRC8`; लंबाई = प्रकार + डेटा + CRC, फ़्रेम 64 बाइट
से लंबा नहीं; CRC8 बहुपद `0xD5` (DVB-S2) के साथ प्रकार और डेटा पर (`static uint8_t crc8(...)`)।
दो प्रकार चाहिए: `0x16` — चैनल (SBUS की तरह 11 बिट के 16) और `0x14` — लिंक के आँकड़े
(तीसरा बाइट — लिंक गुणवत्ता, %)। संपर्क टूटने पर ExpressLRS डिफ़ॉल्ट रूप से चैनल भेजना
बंद कर देता है — यह फ़्रेम टाइमआउट है; “आख़िरी स्थिति” मोड में चैनल आते रहते हैं, पर लिंक
गुणवत्ता 0 होती है — यह `failsafe()` है।

---

## `Packed11`

**फ़ाइल:** `rc/Packed11.h` · **प्रकार:** नेमस्पेस

22 बाइट में 11 बिट के 16 चैनल, निचले बिट पहले — SBUS और CRSF दोनों ऐसे पैक करते हैं।
`toMicros(raw)`: 172…1811 → 988…2012 µs, 992 → 1500 (`5/8 · मान + 880`, Betaflight और INAV
की तरह); `unpack(data, out)` — 22 बाइट → µs में 16 मान।
