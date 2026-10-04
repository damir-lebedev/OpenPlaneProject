# HAL — हार्डवेयर एब्स्ट्रैक्शन

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/hal.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

HAL अकेली ऐसी परत है जिसे किसी ख़ास MCU के बारे में जानने की अनुमति है। इंटरफ़ेस
`include/hal/` में हैं, और क्रियान्वयन ये हैं:

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x), **मुख्य**;
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x): पूरा फ़र्मवेयर
  बिल्ड होता है (`pio run -e stm32h743`) और PC पर चलता है (`pio test -e
  native-stm32`); हार्डवेयर पर अकेला DevEBox बोर्ड जाँचा गया है (SD कार्ड,
  ब्लैक बॉक्स), सेंसर और सर्वो अभी नहीं;
- `hal/Rtos.h` — FreeRTOS टास्क, दोनों प्लेटफ़ॉर्म पर एक जैसे।

इसके ऊपर का सब कुछ केवल इंटरफ़ेस के साथ काम करता है, इसलिए किसी दूसरे MCU पर जाने का
मतलब `IBoard` का नया क्रियान्वयन लिखना है, सेंसर दोबारा लिखना नहीं।

---

## namespace `ServoChannel`

**फ़ाइल:** `hal/IBoard.h`

`IBoard::servo(channel)` के लिए आउटपुट के इंडेक्स। यह नाम वाले मेथड की जगह एक सपाट सूची है —
आउटपुट जोड़ने से `IBoard` इंटरफ़ेस नहीं बदलता। क्रम `FlightOutputs::outputInfo()` तालिका की
पंक्तियों से मेल खाता है।

| स्थिरांक | मान |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — माल गिराना (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — कैमरा (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**फ़ाइल:** `hal/IBoard.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Esp32Board`, `Stm32Board`

हार्डवेयर का एकमात्र प्रवेश-द्वार। इसके ऊपर कोई भी कोड `<Wire.h>`,
`<SPI.h>`, `HardwareSerial` को शामिल नहीं करता और LEDC को सीधे नहीं बुलाता।

| मेथड | विवरण |
|---|---|
| `virtual void begin()` | I2C/SPI बसों का एक बार का आरंभीकरण। UART को उनके मालिक (`IBusReceiver`, GPS) अपनी गति से खोलते हैं, PWM को — `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | सेंसर बस |
| `virtual ISpiBus& spi()` | SPI बस |
| `virtual II2CBus* displayI2c()` | केवल स्क्रीन के लिए दूसरी I2C बस; न हो तो `nullptr` |
| `virtual IUartPort& rcUart()` | iBUS रिसीवर का UART |
| `virtual IUartPort& gpsUart()` | GPS का UART |
| `virtual IUartPort* telemetryUart()` | MAVLink रेडियो मॉडेम का UART; डिफ़ॉल्ट रूप से `nullptr` (ESP32 में कोई ख़ाली UART नहीं है) |
| `virtual IServoOutput& servo(uint8_t channel)` | `ServoChannel::*` इंडेक्स से PWM आउटपुट |
| `virtual void setBuzzer(bool on)` | बज़र `PIN_BUZZER`; डिफ़ॉल्ट रूप से कुछ नहीं करता |

---

## `II2CBus`

**फ़ाइल:** `hal/II2CBus.h` · **प्रकार:** ग़ैर-वर्चुअल सहायकों वाला इंटरफ़ेस ·
**क्रियान्वयन:** `Esp32I2CBus`, `Stm32I2CBus`

`Wire` जैसे रूप में I2C बस का एब्स्ट्रैक्शन। पिन और आवृत्ति क्रियान्वयन अपने कंस्ट्रक्टर में तय करता है,
इसलिए `begin()`/`setClock()` में पिन नहीं होते — बस का आरंभीकरण ठीक एक बार होता है, चाहे
उस पर कई डिवाइस हों।

| मेथड | विवरण |
|---|---|
| `begin()`, `setClock(hz)` | आरंभीकरण, आवृत्ति |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | लिखना; सफल होने पर `endTransmission` 0 लौटाता है (`Wire` की तरह) |
| `requestFrom(addr, n)`, `available()`, `read()` | पढ़ना |
| `bool writeRegister(addr, reg, value)` | सहायक: एक रजिस्टर लिखना; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | सहायक: रिपीटेड स्टार्ट + `count` बाइट पढ़ना। NACK होने पर **या `count` से कम बाइट आने पर** `false`; बफ़र को छुआ नहीं जाता |
| `int readRegister(addr, reg)` | रजिस्टर का मान या `-1` |
| `bool probe(addr)` | डिवाइस पते पर ACK से जवाब देता है |

अपरिवर्तनीय नियम: विफल होने पर सहायक बफ़र में नहीं लिखते — ड्राइवर के पास पिछला डेटा रहता है,
कचरा नहीं (ख़ाली बफ़र पर `read()` से मिलने वाला `0xFF`)।

---

## `ISpiBus`

**फ़ाइल:** `hal/ISpiBus.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Esp32SpiBus`, `Stm32SpiBus`

**CS के प्रबंधन के बिना** SPI बस: एक बस पर कई डिवाइस होते हैं, और CS को
`SpiRegisterDevice` बदलता है।

| मेथड | विवरण |
|---|---|
| `begin()` | SCK/MISO/MOSI सेट करना (पिन क्रियान्वयन के कंस्ट्रक्टर में हैं) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 0..3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | एक बाइट का फ़ुल-डुप्लेक्स आदान-प्रदान |
| `endTransaction()` | ट्रांज़ैक्शन का अंत |

---

## `IUartPort`

**फ़ाइल:** `hal/IUartPort.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Esp32UartPort`, `Stm32UartPort`

`HardwareSerial` जैसा UART, लेकिन `begin()` केवल गति लेता है: पिन और
फ़ॉर्मैट (8N1) क्रियान्वयन तय करता है।

| मेथड | विवरण |
|---|---|
| `begin(baud)` | पोर्ट खोलना |
| `int available()`, `int read()` | प्राप्त करना |
| `size_t write(byte)`, `size_t write(buffer, size)` | भेजना |
| `virtual int availableForWrite()` | भेजने वाले बफ़र में ख़ाली जगह; `-1` — अज्ञात (डिफ़ॉल्ट)। टेलीमेट्री इसी से फ़्रेम को टालती है, इंतज़ार नहीं करती |

---

## `IServoOutput`

**फ़ाइल:** `hal/IServoOutput.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Esp32ServoOutput`, `Stm32ServoOutput`

एक अकेला PWM आउटपुट। पिन क्रियान्वयन तय करता है।

| मेथड | विवरण |
|---|---|
| `bool attach(minUs, maxUs)` | चैनल/टाइमर आवंटित करना और पिन सेट करना; पल्स की सीमा का दायरा। `true` केवल यह दिखाता है कि MCU ने संसाधन आवंटित किए, यह **नहीं** कि सर्वो जुड़ा है |
| `writeMicroseconds(us)` | पल्स की चौड़ाई, µs (`attach` के दायरे में सीमित) |
| `bool isAttached() const` | `attach()` का नतीजा |
| `virtual int32_t measurePulseUs()` | निदान: पिन पर पल्स की असली चौड़ाई या `-1`। डिफ़ॉल्ट क्रियान्वयन `-1` लौटाता है |

---

## `IFlashRegion`

**फ़ाइल:** `hal/IFlashRegion.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Esp32FlashPartition`, `SdFileRegion`

लॉग (ब्लैक बॉक्स) के लिए NOR-फ़्लैश का क्षेत्र: मिटाना केवल 4 KB के सेक्टरों में होता है
(मिटा हुआ `0xFF` पढ़ा जाता है), लिखना केवल बिट गिराता है — मिटे हुए बाइटों में लिखा जा सकता है,
एक ही पेज में टुकड़ों में भी। ESP32 पर लिखना और मिटाना दोनों दोनों कोर को रोक देते हैं —
कब यह स्वीकार्य है, यह बुलाने वाला तय करता है।

| मेथड | विवरण |
|---|---|
| `uint32_t size() const` | क्षेत्र का आकार, बाइट; 0 — क्षेत्र नहीं है |
| `bool read(offset, data, length)` | पढ़ना |
| `bool write(offset, data, length)` | लिखना (मिटे हुए बाइटों में) |
| `bool erase(offset, length)` | मिटाना; पता और लंबाई 4096 के गुणज हों |

`Esp32FlashPartition(const char* name)` — पार्टिशन तालिका से नाम के आधार पर डेटा पार्टिशन
(`esp_partition_*`); `begin()` पार्टिशन ढूँढता है (कोर शुरू होने के बाद),
पार्टिशन न हो तो — `false` और `size() == 0`।

---

## `IBlockDevice`

**फ़ाइल:** `hal/IBlockDevice.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `Stm32SdCard` (टेस्ट में — `fake::SdCardModel`)

SD कार्ड, 512 बाइट के ब्लॉकों की सारणी के रूप में। मिटाना नहीं है: ब्लॉक को दोबारा लिखा जा सकता है।

| मेथड | विवरण |
|---|---|
| `uint32_t blockCount() const` | ब्लॉकों में आकार; 0 — कार्ड नहीं है |
| `bool read(block, data, count)` / `write(...)` | लगातार `count` ब्लॉक, `data` — कोई भी पता |

## `SdFileRegion`

**फ़ाइल:** `hal/SdFileRegion.h` · **इनहेरिट करता है:** `IFlashRegion` · **निर्भर है:** `IBlockDevice`, `Fat32::locate`

SD कार्ड पर ब्लैक बॉक्स का क्षेत्र: FAT32 की रूट में एक फ़ाइल (डिफ़ॉल्ट रूप से
`BLACKBOX.BIN`), जो पहले से PC पर एक ही टुकड़े में बनाई गई है (`tools/blackbox.py
sd-prepare`) और `0xFF` से भरी है। फ़ाइल को केवल **ढूँढा** जाता है (FAT तालिकाओं और
निर्देशिका को छुआ नहीं जाता), उसके बाद उसके भीतर कच्चे ब्लॉक लिखे जाते हैं।
`BlackBoxStorage` के लिए यह वही `IFlashRegion` है जो ESP32 का फ़्लैश पार्टिशन।

| मेथड | विवरण |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` क्षेत्र की ऊपरी सीमा है: चालू करते समय सेक्टरों की जाँच का समय उसके साथ बढ़ता है |
| `Fat32::Result begin()` | फ़ाइल ढूँढना। `Ok` — `size() > 0`; नहीं तो कारण (`Fat32::describe()`): कार्ड नहीं है, FAT32 नहीं है, फ़ाइल नहीं है, बिखरी हुई है, ख़ाली है |
| `size()` | फ़ाइल (`maxBytes` से अधिक नहीं), 4 KB के सेक्टर तक नीचे पूर्णांकित; 0 — क्षेत्र नहीं है |
| `read` / `write` | कोई भी ऑफ़सेट और लंबाई। अधूरा ब्लॉक पढ़ा जाता है, पूरा किया जाता है और पूरा लिखा जाता है; जो ब्लॉक अभी लिखा गया उसे याद रखा जाता है (राइट-थ्रू कैश): 256 बाइट के लगातार पेज कार्ड को दोबारा नहीं पढ़ते। बिजली जाने से वह कुछ नहीं खोता जो `write()` से लौट चुका है |
| `erase(offset, length)` | 4096 का गुणज; `0xFF` लिखता है (कार्ड के अंदर अपना मिटाना है, बाहर से उसकी ज़रूरत नहीं) |

## `IRegisterDevice`

**फ़ाइल:** `hal/RegisterDevice.h` · **प्रकार:** इंटरफ़ेस ·
**क्रियान्वयन:** `I2cRegisterDevice`, `SpiRegisterDevice`

“8-बिट रजिस्टरों का समूह”। सेंसर का ड्राइवर एक बार लिखा जाता है, और बस का चुनाव
`SensorSelection.h` में ऑब्जेक्ट बनाते समय होता है।

| मेथड | विवरण |
|---|---|
| `virtual void begin()` | डिवाइस की लाइनें तैयार करना (SPI के लिए — CS)। डिफ़ॉल्ट रूप से कुछ नहीं |
| `virtual bool probe()` | डिवाइस ने जवाब दिया (SPI के लिए हमेशा `true` — ACK नहीं होता, ID रजिस्टर जाँचा जाता है) |
| `virtual bool writeRegister(reg, value)` | रजिस्टर लिखना |
| `virtual bool writeRegisters(reg, data, count)` | लगातार लिखना (पते का ऑटो-इंक्रीमेंट) |
| `virtual bool readRegisters(reg, buffer, count)` | लगातार `count` बाइट पढ़ना; `false` होने पर बफ़र को छुआ नहीं जाता |
| `int readRegister(reg)` | मान या `-1` (ग़ैर-वर्चुअल सहायक) |

---

## `I2cRegisterDevice`

**फ़ाइल:** `hal/RegisterDevice.h` · **इनहेरिट करता है:** `IRegisterDevice`

`II2CBus` पर 7-बिट पते वाला डिवाइस। सभी ऑपरेशन `II2CBus` के सहायकों को सौंप दिए जाते हैं।

| मेथड | विवरण |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` चिप का दूसरा पता है (SDO/SA0 पिन): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | मुख्य पता जवाब नहीं देता पर वैकल्पिक देता है — आगे वैकल्पिक पते से काम करना |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → `II2CBus(address, …)` के सहायक |
| `uint8_t getAddress() const` | डिवाइस का मौजूदा पता |

---

## `SpiRegisterDevice`

**फ़ाइल:** `hal/RegisterDevice.h` · **इनहेरिट करता है:** `IRegisterDevice`

`ISpiBus` पर अपने CS पिन वाला डिवाइस। Bosch/InvenSense प्रोटोकॉल: पढ़ने के लिए —
पता बिट `0x80` के साथ, लिखने के लिए — बिट 7 साफ़ करके।

| मेथड | विवरण |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` — पते के बाद, डेटा से पहले चिप कितने “फ़ालतू” बाइट देती है (BMP388 — 1, ICM42688 — 0); `mode` — SPI मोड 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | हमेशा `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; हमेशा `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, `dummyReadBytes` छोड़ना, `n` बाइट, CS↑; हमेशा `true` |

हर ऑपरेशन अलग ट्रांज़ैक्शन है: `beginTransaction(clockHz, spiMode)` …
`endTransaction()`।

---

## `Esp32Board`

**फ़ाइल:** `hal/esp32/Esp32Board.h` · **इनहेरिट करता है:** `IBoard`

एकमात्र जगह जो ESP32 के ठोस पेरिफ़ेरल ऑब्जेक्ट बनाती है और `Config.h` के पिन जानती है।

| फ़ील्ड | प्रकार | यह क्या है |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `PIN_I2C_SDA/SCL` पर `Wire`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `PIN_I2C2_SDA/SCL` पर `Wire1` — केवल तब जब `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | ग्लोबल `SPI` |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | `PIN_IBUS` पर iBUS, केवल RX |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | `PIN_GPS_RX/TX` पर GPS |
| `servos[7]` | `Esp32ServoOutput` | `ServoChannel` के क्रम में LEDC चैनल 0..6 (AUX1/AUX2 — `PIN_AUX1/2`, अगर निकाले गए हों) |

| मेथड | विवरण |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, फिर दूसरी बस हो तो `displayBus.begin()`; बज़र का पिन |
| `setBuzzer(on)` | पिन निकाला गया हो तो `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | `hasDisplayBus()` होने पर `&displayBus`, नहीं तो `nullptr` |
| `static constexpr bool hasDisplayBus()` | दूसरी बस के दोनों पिन ≥ 0 हैं। यह (फ़ील्ड `displayBus` की तरह) केवल `SOC_I2C_NUM > 1` होने पर मौजूद है — C3 में एक ही I2C कंट्रोलर है |
| बाक़ी | संबंधित फ़ील्ड लौटाते हैं |

---

## `Esp32I2CBus`

**फ़ाइल:** `hal/esp32/Esp32I2CBus.h` · **इनहेरिट करता है:** `II2CBus`

`TwoWire` (`Wire` या `Wire1`) पर पतला आवरण।

| मेथड | विवरण |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | पैरामीटर याद रखता है |
| `begin()` | `wire.begin(sda, scl, hz)` और `wire.setTimeOut(TIMEOUT_MS)` — `wire.begin()` की इकलौती कॉल |
| बाक़ी | सीधे `TwoWire` को सौंपना |

`TIMEOUT_MS = 5`: 400 kHz पर IMU के 14 बाइट पढ़ने में ~0.4 ms लगता है; वरना शोर से अटका ट्रांज़ैक्शन
लूप को मानक 50 ms तक रोके रखता।

---

## `Esp32SpiBus`

**फ़ाइल:** `hal/esp32/Esp32SpiBus.h` · **इनहेरिट करता है:** `ISpiBus`

ग्लोबल `SPI` पर आवरण। `begin()` → `SPI.begin(sck, miso, mosi, -1)` (CS
डिवाइस ख़ुद संभालते हैं)। `beginTransaction()` `SPISettings(hz, MSBFIRST,
SPI_MODEn)` बनाता है; `spiModeOf()` 0..3 को Arduino के स्थिरांकों में बदलता है, अज्ञात
मान → `SPI_MODE0`।

---

## `Esp32UartPort`

**फ़ाइल:** `hal/esp32/Esp32UartPort.h` · **इनहेरिट करता है:** `IUartPort`

`HardwareSerial` पर आवरण: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` — केवल प्राप्त करना। बाक़ी सौंपना है।

---

## `Esp32ServoOutput`

**फ़ाइल:** `hal/esp32/Esp32ServoOutput.h` · **इनहेरिट करता है:** `IServoOutput`

PWM सीधे LEDC से (Arduino core 2.x के `ledcSetup/ledcAttachPin/ledcWrite`)।
ESP32Servo लाइब्रेरी **इस्तेमाल नहीं होती**: S3 पर संस्करण 3.2.1 MCPWM ब्लॉकों को गड्डमड्ड कर देता था
(GPIO6/7, GPIO4/5 को दोहराते थे)।

| स्थिरांक | मान |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (प्रति चरण ≈1.2 µs) |

| मेथड | विवरण |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | पिन `< 0` — आउटपुट निकाला नहीं गया |
| `attach(minUs, maxUs)` | दायरा याद रखता है; पिन < 0 → `false`; नहीं तो `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | attached न हो तो कुछ नहीं; नहीं तो `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | उसी GPIO का इनपुट बफ़र चालू करता है (`PIN_INPUT_ENABLE`, आउटपुट को छुआ नहीं जाता) और `pulseIn(pin, HIGH, 30 ms)` नापता है; पल्स नहीं → `-1` |

चैनल 2n और 2n+1 एक LEDC टाइमर साझा करते हैं — सभी आउटपुट 50 Hz पर हैं, इसलिए टकराव नहीं होता।

---

# STM32H743 के लिए क्रियान्वयन

अगली पीढ़ी का बोर्ड STM32H743VIT6 है (Cortex-M7 480 MHz, 2 MB फ़्लैश,
1 MB RAM)। पूरा फ़र्मवेयर बिल्ड होता है (env `stm32h743` — PlatformIO बोर्ड
`weact_mini_h743vitx`, और `stm32h743-devebox` — DevEBox H743, कंसोल
USB CDC से), cppcheck और PC पर टेस्ट पास करता है (env `native-stm32`, STM32duino की
नक़ली परत के साथ)। **सेंसर के बिना** DevEBox बोर्ड पर यह जाँचा गया है: बूट, SD कार्ड, ब्लैक बॉक्स —
[बोर्ड पर टेस्ट](../TESTING.md#stm32-बोर्ड-पर-टेस्ट) — और iBUS का प्राप्त होना, ARM, सर्वो और
मोटर तक PWM: विमान रिमोट से मैनुअल मोड में चलता है (शुरुआत वीडियो पर रिकॉर्ड है)।
सेंसर अभी बोर्ड से नहीं जोड़े गए हैं।
पिन का बँटवारा [`Config.h`](config.md#stm32h743vit6-board_stm32h743) के `BOARD_STM32H743` ब्लॉक में है।

ESP32 से वे सामान्य अंतर जिन्हें यह परत छिपाती है:

- **पेरिफ़ेरल कोर चुनता है।** STM32duino वेरिएंट की `PeripheralPins` तालिकाओं में पिन के नंबरों से
  ख़ुद कंट्रोलर (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx) ढूँढ लेता है,
  इसलिए `Config.h` में UART/चैनलों के नंबर नहीं हैं।
- **पिन के नंबर** वेरिएंट के “Arduino-पिन” हैं (`PA0`, `PD14`...), GPIO नहीं; एनालॉग
  पिनों के लिए वे `0xC0 + N` हैं, इसलिए STM32 ब्लॉक में पिन `int16_t` हैं।
- **UART के पिन** `Uart(rx, tx)` ऑब्जेक्ट बनाते समय तय होते हैं, `begin()` में नहीं।

## `Stm32Board`

**फ़ाइल:** `hal/stm32/Stm32Board.h` · **इनहेरिट करता है:** `IBoard`

वही जो `Esp32Board`, पर STM32duino के ऊपर।

| फ़ील्ड | प्रकार | यह क्या है |
|---|---|---|
| `displayWire` | `TwoWire` | दूसरा I2C कंट्रोलर (ग्लोबल `Wire` को सेंसर घेरे हैं)। `displayBus` से पहले घोषित है, जो उसका संदर्भ रखता है |
| `i2cBus` | `Stm32I2CBus` | `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10) पर `Wire`, 400 kHz |
| `displayBus` | `Stm32I2CBus` | `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) पर `displayWire` — दूसरी बस हमेशा होती है |
| `spiBus` | `Stm32SpiBus` | `PIN_SENSOR_SPI_*` (SPI2) पर ग्लोबल `SPI` |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, iBUS-SENS के लिए आरक्षित) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | MAVLink रेडियो मॉडेम: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | `ServoChannel` के क्रम में (AUX1 — PD15/TIM4, AUX2 — PE9/TIM1) |

| मेथड | विवरण |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, बज़र का पिन |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | हमेशा `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | `Config.h` के पिन को कोर API के प्रकार में बदलना |
| बाक़ी | संबंधित फ़ील्ड लौटाते हैं |

## `Stm32I2CBus`

**फ़ाइल:** `hal/stm32/Stm32I2CBus.h` · **इनहेरिट करता है:** `II2CBus`

| मेथड | विवरण |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | पैरामीटर याद रखता है |
| `begin()` | `setSDA()`/`setSCL()` (केवल `begin()` से पहले असर करते हैं), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`; `size_t` नतीजे को `uint8_t` में बदला जाता है |
| बाक़ी | सीधे `TwoWire` को सौंपना |

STM32duino में ट्रांज़ैक्शन का टाइमआउट मेथड नहीं, बल्कि मैक्रो `I2C_TIMEOUT_TICK` है (ms में,
डिफ़ॉल्ट 100)। env `stm32h743` में यह फ़्लैग `-D I2C_TIMEOUT_TICK=5` से तय है —
उसी कारण से जिससे `Esp32I2CBus` में `TIMEOUT_MS`।

## `Stm32SpiBus`

**फ़ाइल:** `hal/stm32/Stm32SpiBus.h` · **इनहेरिट करता है:** `ISpiBus`

`SPIClass&` पर आवरण। `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`;
हार्डवेयर NSS इस्तेमाल नहीं होता — CS को `SpiRegisterDevice` बदलता है, जैसे ESP32 पर।
`beginTransaction()` `SPISettings(hz, MSBFIRST, SPIMode)` बनाता है; `spiModeOf()`
0..3 को `SPI_MODEn` में बदलता है, अज्ञात मान → `SPI_MODE0`।

## `Stm32UartPort`

**फ़ाइल:** `hal/stm32/Stm32UartPort.h` · **इनहेरिट करता है:** `IUartPort`

`HardwareSerial&` पर आवरण (STM32duino 3.x में यह अमूर्त बेस
`arduino::HardwareSerial` है, ठोस ऑब्जेक्ट `Uart` को `Stm32Board` बनाता है)।
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`; `availableForWrite()` —
`HardwareSerial` से। बफ़र (env में `SERIAL_RX/TX_BUFFER_SIZE`): प्राप्त करने के लिए 256
बाइट (NAV-PVT का फ़्रेम 100 का है, मानक 64 कम पड़ते हैं), भेजने के लिए 1024 (लॉग की पंक्तियाँ और
MAVLink के फ़्रेम बिना इंतज़ार के)।

## `Stm32ServoOutput`

**फ़ाइल:** `hal/stm32/Stm32ServoOutput.h` · **इनहेरिट करता है:** `IServoOutput`

`HardwareTimer` के ज़रिए टाइमर का हार्डवेयर PWM, 50 Hz। पल्स को टाइमर
बिना इंटरप्ट और बिना CPU के बनाता है — STM32 की `Servo` लाइब्रेरी के उलट, जो
एक टाइमर के इंटरप्ट से पिन हिलाती है और जिटर देती है।

| स्थिरांक | मान |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — आउटपुट कितने अलग-अलग टाइमर घेर सकते हैं (अभी TIM2 और TIM4 घिरे हैं) |

| मेथड | विवरण |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | पिन `< 0` — आउटपुट निकाला नहीं गया |
| `attach(minUs, maxUs)` | टाइमर और चैनल पिन से `PinMap_TIM` से लिए जाते हैं (`pinmap_peripheral`, `STM_PIN_CHANNEL`), जैसे `analogWrite()` में। पिन पर टाइमर नहीं या पूल ख़त्म → `false`। नहीं तो `setMode(PWM1)`, तुलना 0 (पहली लिखाई तक पल्स नहीं), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`। तुलना रजिस्टर प्रीलोड वाला है — मान अगली अवधि से लागू होता है |
| `measurePulseUs()` | पिन को दोबारा सेट किए बिना `pulseIn(pin, HIGH, 30 ms)`: STM32 पर IDR रजिस्टर वैकल्पिक फ़ंक्शन के मोड में भी स्तर देखता है |
| `static acquireTimer(TIM_TypeDef*)` | साझा पूल: **हर TIMx पर एक `HardwareTimer`**। उसी टाइमर पर दूसरा ऑब्जेक्ट कोर का हैंडलर (`HardwareTimer_Handle[index]`) अधिलेखित कर देता। अवधि टाइमर के पहले आउटपुट पर तय होती है; `setOverflow(MICROSEC_FORMAT)` भाजक चुनता है — टाइमर की घड़ी 240 MHz होने पर चरण ~0.3 µs |

## `Stm32FlashStorage`

**फ़ाइल:** `hal/stm32/Stm32FlashStorage.h` · **इनहेरिट करता है:** `IFlashStorage` ([storage.md](storage.md))

STM32 पर `KeyValueStore` का माध्यम: फ़्लैश (बैंक 2) का आख़िरी सेक्टर, STM32duino के
EEPROM-अनुकरण (`eeprom_buffer_fill/flush`, 8 KB का बफ़र, जिसमें से
पहले `KeyValueStore::CAPACITY` बाइट इस्तेमाल होते हैं) के ज़रिए।

| मेथड | विवरण |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + बफ़र को बाइट-दर-बाइट पढ़ना |
| `write(src, n)` | **तेज़**: `noInterrupts()` में इमेज की प्रति अपने बफ़र में, “लिखना बाक़ी है” का झंडा। उड़ान वाले टास्क में `KvPreferences::end()` से बुलाया जाता है |
| `bool service()` | **धीमा**: (`noInterrupts()` में) अनुकरण वाले बफ़र में स्नैपशॉट और `eeprom_buffer_flush()` — 128 KB के सेक्टर को मिटाना (सेकंड) और लिखना। केवल बैकग्राउंड टास्क `storage` से |
| `hasPending()`, `flushCount()` | निदान |
| `static instance()`, `static store()` | माध्यम और फ़र्मवेयर का साझा `KeyValueStore` |

उड़ान क्यों नहीं जमती: सेटिंग वाला सेक्टर बैंक 2 में है, कोड — बैंक 1 में; H7 का फ़्लैश
एक बैंक को तब पढ़ सकता है जब दूसरे में लिखा जा रहा हो; उड़ान वाला टास्क बैकग्राउंड वाले को हटा देता है।

## `compat/Preferences.h`

**फ़ाइल:** `hal/stm32/compat/Preferences.h` — env `stm32h743` (और
`native-stm32`) में `compat/` निर्देशिका `-I` में लाइब्रेरियों से पहले आती है, और
सेंसर ड्राइवरों, ऑटो-ट्रिमर और लॉग की सेटिंग का `#include <Preferences.h>`
उसे ढूँढ लेता है। `class Preferences : public KvPreferences`, `Stm32FlashStorage::store()` के
ऊपर — API वही जो ESP32 के NVS का ([storage.md](storage.md#kvpreferences))।

## `Stm32SdCard`

**फ़ाइल:** `hal/stm32/Stm32SdCard.h` · **इनहेरिट करता है:** `IBlockDevice` · **पिन:** `src/stm32/sd_msp.cpp`

SDMMC1 पर SD कार्ड: 4-बिट बस, पोलिंग मोड में `HAL_SD` (DMA और इंटरप्ट के बिना)
**हार्डवेयर फ़्लो कंट्रोल के साथ**: उड़ान वाला टास्क लिखने वाले टास्क को ब्लॉक के बीच में हटा देता है,
और इसके बिना FIFO ओवरफ़्लो हो जाता था (`HAL_SD_ERROR_RX_OVERRUN`, 0x20) — बोर्ड पर यह कंसोल और
लिखने के सेकंडों तक जम जाने के रूप में दिखता था। पिन PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) —
DevEBox और WeAct का µSD स्लॉट। SDMMC कोर की घड़ी PLL1Q = 48 MHz से आती है, `ClockDiv = 1` →
**24 MHz**; अगर 24 MHz पर पहली रीडिंग नहीं हुई, तो 12 और 6 आज़माए जाते हैं।

| सदस्य | विवरण |
|---|---|
| `bool begin()` | बस चालू करना, कार्ड पहचानना, परीक्षण रीडिंग। `false` — कार्ड नहीं है; `initError()` — कोड |
| `read` / `write` | 4 KB से बड़े टुकड़े नहीं (छोटे ठहराव); 4 का गुणज न होने वाला पता संरेखित बफ़र से कॉपी होता है (HAL FIFO को शब्दों में पढ़ता है)। विफलता पर — एक बार दोबारा कोशिश |
| प्रतीक्षा | लिखने के बाद अगली पहुँच से पहले कार्ड के ट्रांसफ़र की अवस्था में लौटने की प्रतीक्षा करता है (`Rtos::sleepMs(1)`: बैकग्राउंड टास्क भूखे नहीं रहते), 1 s तक। पढ़ने के बाद अवस्था का फ़ालतू अनुरोध नहीं भेजा जाता — चालू करते समय की जाँच दसियों हज़ार सेक्टर पढ़ती है |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | स्थिति की पंक्ति के लिए |
| `readOps`, `writeOps`, `errors`, `retries` | काउंटर |

## `ResetCause`

**फ़ाइल:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

रीबूट का कारण, दोनों बोर्डों पर एक जैसा। ESP32 — `esp_reset_reason()`;
STM32 — `RCC->RSR` के झंडे (एक बार पढ़े जाते और साफ़ कर दिए जाते हैं; H7 में `PINRSTF`
हर रीसेट पर सेट होता है, इसलिए पहले अधिक ख़ास कारण जाँचे जाते हैं:
वॉचडॉग → चालू करना → वोल्टेज गिरना → सॉफ़्टवेयर रीसेट)।
पैनिक, वॉचडॉग और बिजली का गिरना “विफलता” हैं: इन पर ब्लैक बॉक्स तुरंत रिकॉर्डिंग शुरू कर देता है।

## `Rtos`

**फ़ाइल:** `hal/Rtos.h` · namespace

| सदस्य | विवरण |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | टास्क की प्राथमिकताएँ |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., कोर 0)`, स्टैक बाइट में; STM32 — `xTaskCreate`, स्टैक शब्दों में बदला जाता है; `handle` — `xTaskNotifyGive` के लिए |
| `void sleepMs(ms)` | `vTaskDelay`; शेड्यूलर के चलने से पहले (STM32 का `setup()`) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 — स्पिनलॉक `portMUX`, STM32 — `taskENTER_CRITICAL()`। अंदर केवल बाइट की नक़ल (ब्लैक बॉक्स की क़तार) |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()`; STM32 — `xPortGetFreeHeapSize()` |

## प्रवेश-बिंदु `src/stm32/main.cpp`

पूरा फ़र्मवेयर: वही ऑब्जेक्ट जो `src/main.cpp` में हैं, Wi-Fi की जगह MAVLink टेलीमेट्री,
`loop()` की जगह FreeRTOS के टास्क — [application.md](application.md#srcstm32maincpp--stm32h743)।
