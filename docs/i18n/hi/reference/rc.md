# RC — ट्रांसमीटर की कमांड प्राप्त करना

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/rc.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

RC परत UART के बाइटों को चैनल के मानों और “संपर्क नहीं है” के संकेत में बदलती है।
उसे विमान, ARM, failsafe के व्यवहार और सर्वो के बारे में कुछ नहीं पता — प्रोटोकॉल
बदलना (S-Bus, PPM) केवल इसी परत को प्रभावित करता है।

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | रैखिक: 1000 → `−max`, 1500 → 0, 2000 → `+max` (इनपुट पहले सीमित होता है); `reverse` चिह्न बदलता है। नतीजा ±`max` तक सीमित |

उदाहरण: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`।

---

## `IBusReceiver`

**फ़ाइल:** `rc/IBusReceiver.h` · **निर्भर है:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

FlySky iBUS प्रोटोकॉल का बाइट-दर-बाइट पार्सर।

**फ़्रेम का प्रारूप** (32 बाइट): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`। पहले `IBUS_CHANNELS` = 10 चैनल लिए जाते हैं;
चैनल का मान — **निचले 12 बिट** (ऊपरी बिटों में FS-iA6B सेवा-डेटा भेजता है,
जैसे failsafe में `0x2384` → 900 µs)।

| मेथड | विवरण |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | पोर्ट कंस्ट्रक्टर में नहीं खुलता |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, टाइमआउट की गिनती “अभी” से |
| `void update()` | UART में जमा सब कुछ पढ़ लेना; हर चक्र में बुलाएँ |
| `const RcChannelState& getState() const` | आख़िरी प्राप्त चैनल |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | अभी तक एक भी फ़्रेम नहीं आया **या** आख़िरी `RX_TIMEOUT_US` से पुराना है |
| `bool isFailsafeReported() const` | आख़िरी फ़्रेम में थ्रॉटल < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | आख़िरी सही फ़्रेम का `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | सही फ़्रेमों और CRC त्रुटियों के काउंटर |

पार्सिंग की स्टेट मशीन (`processByte`): `0x20` की प्रतीक्षा करती है; अगला बाइट
`0x40` होना चाहिए, वरना खोज फिर से शुरू; फिर 32 बाइट जमा करती है और
`processFrame()` बुलाती है। ग़लत CRC वाला फ़्रेम पूरा फेंक दिया जाता है (चैनल नहीं
बदलते, `badFrames++`)।

अपरिवर्तनीय नियम:

- पहले सही फ़्रेम तक `isSignalLost() == true` — डिफ़ॉल्ट मान (सभी 1500)
  ट्रांसमीटर की कमांड नहीं समझे जाते।
- failsafe का संकेत **हर** सही फ़्रेम पर दोबारा गिना जाता है — संपर्क सामान्य
  थ्रॉटल वाले पहले ही फ़्रेम से बहाल हो जाता है।
