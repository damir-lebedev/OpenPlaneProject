# SENSORS — सेंसर

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/sensors.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

सेंसर तीन स्तरों में बने हैं:

1. **श्रेणी के इंटरफ़ेस** (`SensorInterface.h`) — जो `Autopilot`,
   `ArmingManager` और टेलीमेट्री को दिखता है।
2. **श्रेणी के बेस क्लास** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — सब कुछ जो साझा है: कैलिब्रेशन, फ़िल्टर, अक्षों का घुमाव, चिह्न,
   त्रुटियों की गिनती, NVS में भंडारण। Template Method पैटर्न।
3. **चिप के ड्राइवर** — केवल डेटाशीट के रजिस्टर और सूत्र। इन्हें
   `IRegisterDevice&` (बस में फ़र्क़ नहीं करते) या `IUartPort&` मिलता है।

कौन-सी चिप कंपाइल होगी, यह `SensorSelection.h` तय करता है।

---

## इंटरफ़ेस और डेटा की संरचनाएँ

**फ़ाइल:** `sensors/SensorInterface.h`

### संरचनाएँ

| संरचना | फ़ील्ड |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (रोल + दायाँ पंख नीचे, पिच + नाक ऊपर, यॉ + नाक दाएँ); `accelX/Y/Z` g (विमान की धुरियाँ: X नाक की ओर, Y बाएँ, Z ऊपर); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, जायरोस्कोप का समाकल) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m, **कैलिब्रेशन बिंदु के सापेक्ष**; `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | hard-iron कैलिब्रेशन के बाद `magX/Y/Z` µT, विमान की धुरियों में; `headingDegrees` 0..360 (झुकाव के संशोधन के बिना); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 नहीं, 2 — 2D, 3 — 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (इंटरफ़ेस)

| मेथड | विवरण |
|---|---|
| `bool begin()` | चिप को पहचानना और सेट करना; `true` — सेंसर काम कर रहा है |
| `bool isAvailable() const` | जुड़ा है और **अभी** जवाब दे रहा है |
| `void update()` | हर टिक पर बुलाएँ; पढ़ने का समय हुआ या नहीं, यह वह ख़ुद तय करता है |
| `const char* getSensorType() const` | लॉग के लिए नाम |
| `void printStatus() const` | निदान की पंक्ति (कंसोल `s`) |

### `ImuSensor : Sensor`

| मेथड | विवरण |
|---|---|
| `const ImuData& getImuData() const` | ताज़ा डेटा |
| `void calibrate()` | जायरोस्कोप का कैलिब्रेशन (स्थिर रखकर) + उड़ान-पूर्व जाँच |
| `void setYaw(float)` | हेडिंग तय करना (जैसे शुरू में कंपास से) |
| `virtual void calibrateOrientation()` | बोर्ड के लगने का कैलिब्रेशन (डिफ़ॉल्ट रूप से समर्थित नहीं) |
| `virtual const char* getPreflightProblem() const` | उड़ान-पूर्व जाँच की समस्या या `nullptr` (डिफ़ॉल्ट `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (मौजूदा ऊँचाई = 0),
`setSeaLevelPressure(Pa)`।

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: 15 s घुमाएँ)।

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (कम से कम 2D फ़िक्स है)।

---

## `AirspeedSensor`

**फ़ाइल:** `sensors/airspeed/AirspeedSensor.h` · **प्रकार:** इंटरफ़ेस · **क्रियान्वयन:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
शून्य करने और फ़िल्टर के बाद का दाब-अंतर, सूचित गति (ρ0 = 1.225), वास्तविक
गति (ρ स्थिर दाब और तापमान से), घनत्व। मेथड: `getAirspeedData()`,
`calibrateZero()` (शून्य करना फिर से शुरू करना), `isZeroing()`।

## `PitotDualBaroAirspeed`

**फ़ाइल:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **इनहेरिट करता है:** `AirspeedSensor` · **स्थिति:** शोर वाले बंद-लूप सिमुलेशन में जाँची गई, उड़ान में नहीं आज़माई गई

**दो निरपेक्ष बैरोमीटरों** पर घर में बनाई पिटो नली: `total` — नली के अंदर का
BMP581 (कुल दाब), `stat` — धड़ का मुख्य बैरोमीटर
(स्थिर दाब)। बनाने की मार्गदर्शिका [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#खुद-बनाई-पिटो-ट्यूब) में है।

| मेथड | विवरण |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | नली के बैरोमीटर का `begin()` (स्थिर वाला पहले से चालू है), शून्य करना शुरू |
| `update()` | नली का नया नमूना → दाब-अंतर − शून्य, लो-पास फ़िल्टर `PITOT_FILTER_TAU_S`; पहले `PITOT_ZERO_SAMPLES` नमूनों से शून्य का औसत; गति `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | दोनों बैरोमीटर जीवित हैं, शून्य इकट्ठा हो गया, कोई ख़राबी नहीं, नमूना `PITOT_STALE_US` से ताज़ा है |
| `hasFault()` | दाब-अंतर −`PITOT_NEGATIVE_FAULT_PA` से नीचे `PITOT_NEGATIVE_FAULT_MS` से अधिक समय तक रहे (नलियाँ, पानी) |
| `getZeroOffset()`, `printStatus()` | निदान |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | सूत्र |

---

## namespace `SensorMounting`

**फ़ाइल:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
चिप की धुरियों को ऊर्ध्वाधर (चिप ऊपर की ओर) के गिर्द विमान की धुरियों (X नाक की ओर,
Y बाएँ) में घुमाता है। `rotationCwDeg` बताता है कि चिप की X धुरी किधर देखती है, ऊपर से देखने पर घड़ी की दिशा में:

| मान | bodyX | bodyY |
|---|---|---|
| 0 (और कोई भी अज्ञात) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

कंपास (`MAG_ROTATION_CW_DEG`) और माउंटिंग कैलिब्रेशन के बिना IMU इसका इस्तेमाल करते हैं।

---

## `SensorSelection.h`

**फ़ाइल:** `sensors/SensorSelection.h` · **प्रकार:** प्रीप्रोसेसर कॉन्फ़िगरेशन

भौतिक सेंसर बदलने की इकलौती जगह: तैयार सेट एक पंक्ति में
(`SENSOR_KIT`) या हर सेंसर अलग से। किसी भी चुनाव को बिल्ड के फ़्लैग से बदला जा सकता है
(`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`)।

| `SENSOR_KIT` सेट | IMU | बैरोमीटर | कंपास | वायु-गति | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, डिफ़ॉल्ट) | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (धड़) | QMC6309 | नली में BMP581 | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (धड़) | QMC6309 | नली में BMP581 | M10 |
| `SENSOR_KIT_CUSTOM` (0) | नीचे के पाँचों मैक्रो तय करें | | | | |

| चुनाव का मैक्रो | विकल्प |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — नली में BMP581 I2C 0x47 |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

सभी संयोजन सभी बोर्डों पर बिल्ड होते हैं — `tools/build_matrix.sh`।

नतीजा टाइप के उपनाम और डिवाइस की फ़ैक्टरियाँ हैं:

| नाम | MPU6050 / ICM42688 आदि |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (NONE के लिए परिभाषित नहीं) |
| नए IMU | `LSM6DSV_Sensor` + I2C 0x6A (आरक्षित 0x6B) या SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) या SPI |
| नए बैरोमीटर | `SPL06_Sensor` + I2C 0x76 (0x77) या SPI; `BMP581_Sensor` + I2C 0x46 (0x47) या SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (NONE के लिए परिभाषित नहीं) |
| `SelectedGps` | `UbloxM10_Gps` (NONE के लिए परिभाषित नहीं) |

`main.cpp` कंपास, GPS और नली के बनाने को `#if SENSOR_* != SENSOR_*_NONE` में लपेटता है।

---

## `ImuOrientation`

**फ़ाइल:** `sensors/imu/ImuOrientation.h` · **निर्भर है:** `SensorMounting`, `Preferences` (NVS)

चिप की धुरियों से विमान की धुरियों का घुमाव-मैट्रिक्स `R`: `body = R · chip`;
`R` की पंक्तियाँ चिप की धुरियों में विमान की धुरियाँ हैं।

| मेथड | विवरण |
|---|---|
| `ImuOrientation()` | इकाई मैट्रिक्स (`fromYawSteps(0)` के बराबर) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | ऊर्ध्वाधर के गिर्द 90° के क़दमों में घुमाव, बोर्ड चिप ऊपर की ओर |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | तीन मुद्राओं से कैलिब्रेशन (चिप की धुरियों में एक्सेलेरोमीटर का “ऊपर” वाला पाठ्यांक)। `nullptr` — सफल, नहीं तो इनकार का कारण |
| `void apply(const float chip[3], float body[3]) const` | घुमाव लागू करना |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | नापे गए “ऊपर” और विमान की Z धुरी के बीच का कोण (वेक्टर शून्य हो तो 180°) |
| `bool load(const char* ns)` | NVS से लोड करना; ऑर्थोनॉर्मल न होने वाली या बाएँ हाथ की तिकड़ी को ठुकराता है |
| `void save(const char* ns) const` | NVS में सहेजना |
| `void describe(Print&) const` | “नाक = चिप का +Y, ऊपर = चिप का +Z” (कोण के साथ, अगर कोई धुरी चिप की धुरी से ±14° के भीतर नहीं मिलती) |

`fromPoses` का एल्गोरिदम: Z = norm(level); X₁ = noseUp का ⟂ Z हिस्सा; Y = 
rightWingDown का ⟂ Z हिस्सा, X₂ = Y × Z; X = norm(X₁ + X₂), Y = Z × X। इनकार के मामले:

| शर्त | संदेश |
|---|---|
| शून्य वेक्टर | “एक्सेलेरोमीटर का कोई पाठ्यांक नहीं” |
| क़दम 2 या 3 का झुकाव 20..80° से बाहर | “क़दम N में 30-60° का झुकाव चाहिए” |
| cos(X₁, X₂) < −0.5 | “क़दम 2 और 3 एक-दूसरे के विरोधी हैं…” (नाक नीचे कर दी गई या ग़लत पंख उठाया गया) |
| cos(X₁, X₂) < 0.9 (≈25°) | “…ग़लत धुरियाँ झुकाई गईं…” |

---

## `AttitudeEstimator`

**फ़ाइल:** `sensors/imu/AttitudeEstimator.h`

रोल/पिच का पूरक फ़िल्टर और यॉ का समाकल, चिप पर निर्भर नहीं।

| मेथड | विवरण |
|---|---|
| `void reset()` | अगला `update()` तुरंत एक्सेलेरोमीटर के कोण से शुरू होगा |
| `void setYaw(float deg)` | हेडिंग तय करना (±180 में लाई जाती है) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | विमान की धुरियों में त्वरण g में, गतियाँ °/s में |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (2 ms पर τ ≈ 0.1 s)।
`reset()` के बाद पहली कॉल सीधे एक्सेलेरोमीटर के कोण देती है; `dt ≤ 0` या 0.1 s से अधिक होने पर
क़दम छोड़ दिया जाता है (विराम, बस का अटकना)।

---

## `ImuSensorBase`

**फ़ाइल:** `sensors/imu/ImuSensorBase.h` · **इनहेरिट करता है:** `ImuSensor` · **प्रकार:** अमूर्त

`RawImuSample` — चिप की धुरियों में ADC इकाइयों में एक नमूना:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`)।

`update()` → `process()` की पाइपलाइन:

```
कच्चे नमूने − ऑफ़सेट (ADC) → स्केल (g, °/s) → ImuOrientation (विमान की धुरियाँ)
→ विमानन के चिह्न (gyroY, gyroZ का चिह्न बदलकर) → AttitudeEstimator
```

| मेथड | विवरण |
|---|---|
| `bool isAvailable() const` | `begin()` सफल है और लगातार 50 से कम पढ़ने की त्रुटियाँ |
| `void update()` | एक बार पढ़ना; त्रुटि होने पर डेटा नहीं बदलता, काउंटर बढ़ते हैं |
| `void calibrate()` | 200 नमूने × 10 ms: जायरोस्कोप का ऑफ़सेट, शोर, “ऊपर”; माउंटिंग कैलिब्रेशन के बिना — क्षितिज = अभी की स्थिति। फिर उड़ान-पूर्व जाँच |
| `void calibrateOrientation()` | तीन मुद्राएँ (`capturePose`: ~1 s स्थिर, मुद्रा पिछली से ≥ 20° अलग, 30 s का टाइमआउट), `ImuOrientation::fromPoses`, NVS में सहेजना। उड़ान-पूर्व जाँच की माउंटिंग की समस्याएँ हटाता है |
| `const char* getPreflightProblem() const` | समस्या का पाठ या `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

ड्राइवरों के लिए सुरक्षित API:

| मेथड | विवरण |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | हर ड्राइवर का अपना NVS नेमस्पेस |
| `virtual bool readSample(RawImuSample&) = 0` | एक नमूना; `false` — चिप ने जवाब नहीं दिया |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | स्केल |
| `virtual float temperatureC(int16_t raw) const = 0` | तापमान का सूत्र |
| `void setAvailable(bool)` | `begin()` का नतीजा; `true` होने पर माउंटिंग NVS या `Config::IMU_ROTATION_CW_DEG` से लोड होती है |
| `void setName(const char*)` | पहचान के बाद नाम को सटीक बनाना |

उड़ान-पूर्व जाँच (`runPreflightCheck`), क्रम से:

| समस्या | शर्त |
|---|---|
| `NotResponding` | कैलिब्रेशन के आधे से कम नमूने पढ़े गए |
| `Moved` | जायरोस्कोप का शोर > 0.5 °/s |
| `NotOneG` | \|a\| 1g से 0.2g से अधिक अलग है |
| `MountingMismatch` | (माउंटिंग कैलिब्रेट है) “ऊपर” सहेजे गए से 45° से अधिक दूर है |
| `NotChipUp` | (कैलिब्रेट नहीं) बोर्ड चिप ऊपर की ओर करके नहीं रखा है (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**फ़ाइल:** `sensors/imu/MPU6050_Sensor.h` · **इनहेरिट करता है:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **स्थिति:** बेंच पर (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 और क्लोन (GY-521 बोर्ड), I2C 0x68/0x69 या
SPI। चिप `WHO_AM_I` से पहचानी जाती है (0x68 — MPU6050, नहीं तो 6500 परिवार)।

- `begin()`: WHO_AM_I (जवाब नहीं → अनुपलब्ध), ID से नाम, रीसेट, स्लीप से बाहर
  (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz; 6500 में एक्सेलेरोमीटर का अलग
  लो-पास फ़िल्टर `ACCEL_CONFIG2` है।
- `readSample()`: `0x3B` से 14 बाइट, big-endian: accel XYZ, temp, gyro XYZ।
- स्केल: 2048 LSB/g, 16.4 LSB/(°/s)।
- तापमान: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`।

---

## `ICM42688_Sensor`

**फ़ाइल:** `sensors/imu/ICM42688_Sensor.h` · **इनहेरिट करता है:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz, बिना
  नक़ली बाइट के।
- `begin()`: बैंक 0, सॉफ़्टवेयर रीसेट, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, UI फ़िल्टर 50 Hz।
- `readSample()`: `0x1D` से 14 बाइट, big-endian: temp, accel XYZ, gyro XYZ।
- तापमान: `raw/132.48 + 25`।

---

## `LSM6DSV_Sensor`

**फ़ाइल:** `sensors/imu/LSM6DSV_Sensor.h` · **इनहेरिट करता है:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST)। रजिस्टर ST के `lsm6dsv-pid` और ArduPilot से मिलाए गए हैं।

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (CTRL3 का बिट 0) और प्रतीक्षा;
  32X को CTRL8 के संस्करण-बिट से पहचाना जाता है (उसका अपना ±16 g का कोड है); BDU + ऑटो-इंक्रीमेंट,
  LPF1 के साथ ±2000 °/s, LPF2 के साथ ±16 g, 960 Hz हाई-परफ़ॉर्मेंस।
- `readSample()`: 0x20 से 14 बाइट, little-endian: temp, gyro XYZ, accel XYZ।
- स्केल: 1000/0.488 LSB/g, 1000/70 LSB/(°/s); तापमान `raw/256 + 25`।
- `static spiDevice(bus, cs)` — SPI मोड 0, बिना नक़ली बाइट के।

## `ICM45686_Sensor`

**फ़ाइल:** `sensors/imu/ICM45686_Sensor.h` · **इनहेरिट करता है:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

ICM-45686 (TDK)। रजिस्टर TDK के ड्राइवर, Zephyr और ArduPilot से मिलाए गए हैं।

- `begin()`: `REG_MISC2` (0x7F) से रीसेट, `WHO_AM_I` (0x72) = 0xE9; ±2000 °/s और ±16 g
  1.6 kHz पर (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); ODR/32
  लो-पास फ़िल्टर — **अप्रत्यक्ष** रजिस्टर IPREG (0xA4AC, 0xA583) का
  पढ़ना-बदलना-लिखना, 0x7C..0x7E की खिड़की से; जायरोस्कोप के चालू होने के लिए 45 ms।
- `readSample()`: 0x00 से 14 बाइट, little-endian: accel XYZ, gyro XYZ, temp।
- स्केल: 2048 LSB/g, 16.4 LSB/(°/s); तापमान `raw/132.48 + 25`।

---

## `BarometerBase`

**फ़ाइल:** `sensors/baro/BarometerBase.h` · **इनहेरिट करता है:** `BarometerSensor` · **प्रकार:** अमूर्त

| मेथड | विवरण |
|---|---|
| `bool isAvailable() const` | `begin()` सफल है और लगातार 100 से कम त्रुटियाँ |
| `void update()` | `pollPeriodUs` से अधिक बार नहीं: `isNewSampleReady()` → `readSample()` → ऊँचाई और ऊर्ध्वाधर गति |
| `void calibrateAltitude()` | 20 नमूने × 50 ms: औसत निरपेक्ष ऊँचाई = आधार; ऊँचाई और गति शून्य |
| `void setSeaLevelPressure(Pa)` | P₀ (डिफ़ॉल्ट 101325) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

सुरक्षित API: कंस्ट्रक्टर `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`।

सूत्र: `h = 44330 · (1 − (P/P₀)^0.1903) − base`; ऊर्ध्वाधर गति ऊँचाई का
**असली नए** नमूनों पर अवकलज है, τ = 0.5 s के लो-पास फ़िल्टर से गुज़रकर
(`dt` के `(0, 0.5 s)` से बाहर होने पर गति नहीं बदलती)। केवल नए नमूने पढ़ने से
“सीढ़ीनुमा” शोर (0 m/s के बीच-बीच में Δh/2 ms की छलाँगें) ख़त्म हो जाता है।

---

## `BMP388_Sensor`

**फ़ाइल:** `sensors/baro/BMP388_Sensor.h` · **इनहेरिट करता है:** `BarometerBase` · **स्थिति:** बेंच पर (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz,
  **1 नक़ली बाइट** (डेटाशीट §5.3.2)।
- `begin()`: चिप ID `0x50`, सॉफ़्टवेयर रीसेट, `0x31` से NVM के 21 बाइट गुणांक
  (स्केल §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, normal मोड।
- `isNewSampleReady()`: STATUS रजिस्टर का `drdy_press` झंडा (बिट 0x20); हर 5 ms पर
  पूछा जाता है।
- `readSample()`: `0x04` से 6 बाइट; Bosch का संशोधन §9.3 (double):
  पहले तापमान (`tLin`), फिर दाब।

---

## `BME280_Sensor`

**फ़ाइल:** `sensors/baro/BME280_Sensor.h` · **इनहेरिट करता है:** `BarometerBase` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

BME280 (ID 0x60) और BMP280 (ID 0x58), I2C 0x76/0x77 या SPI बिना नक़ली
बाइट के। नमी नहीं पढ़ी जाती।

- `begin()`: चिप ID, रीसेट, `0x88` से कैलिब्रेशन के 24 बाइट, `CTRL_HUM` (केवल
  BME280, `CTRL_MEAS` से **पहले** लिखा जाता है), `CONFIG` = IIR 4 + 0.5 ms, `CTRL_MEAS`
  = T×2, P×8, normal।
- तैयारी का झंडा नहीं है — `ready = true`, हर 25 ms पर पूछना।
- संशोधन — Bosch के §8.1 के सूत्र (double); शून्य से भाग से सुरक्षा।

---

## `SPL06_Sensor`

**फ़ाइल:** `sensors/baro/SPL06_Sensor.h` · **इनहेरिट करता है:** `BarometerBase` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

SPL06-001 (Goertek)। सूत्र डेटाशीट §4.9 से हैं।

- `begin()`: `ID` (0x0D) = 0x10 (0x11 SPA06 है, गुणांकों का अलग सेट —
  ठुकराया जाता है); रीसेट, `COEF_RDY | SENSOR_RDY` की प्रतीक्षा; गुणांकों के 18 बाइट
  (चिह्नित 12/20/16-बिट फ़ील्ड); तापमान का स्रोत — `COEF_SRCE` से;
  दाब 16× (32 Hz), तापमान 1×, सतत मोड।
- `isNewSampleReady()` — `PRS_RDY` बिट; `readSample()` — 24-बिट
  big-endian नमूने, `kP = 253952`, `kT = 524288`।

## `BMP581_Sensor`

**फ़ाइल:** `sensors/baro/BMP581_Sensor.h` · **इनहेरिट करता है:** `BarometerBase` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

BMP581 (Bosch)। क्रम आधिकारिक BMP5_SensorAPI का है।

- `begin()`: नक़ली रीडिंग (SPI के लिए), `CHIP_ID` (0x01) = 0x50/0x51;
  सॉफ़्टवेयर रीसेट, `INT_STATUS` का POR और `STATUS` में NVM तैयार, त्रुटि के बिना;
  standby → OSR (दाब 16×, तापमान 2×), IIR, DRDY; ODR की
  संभाव्यता की जाँच (`OSR_EFF`); सतत मोड।
- नमूना — DRDY पर, या हर 40 ms अगर झंडा खो जाए; तापमान
  `int24/65536`, दाब `uint24/64`।
- नली वाले बिल्ड में दो इंस्टेंस: मुख्य बैरोमीटर और `PITOT-BMP581`।

---

## `MagnetometerBase`

**फ़ाइल:** `sensors/mag/MagnetometerBase.h` · **इनहेरिट करता है:** `MagnetometerSensor` · **प्रकार:** अमूर्त

| मेथड | विवरण |
|---|---|
| `bool isAvailable() const` | `begin()` सफल है और लगातार 25 से कम त्रुटियाँ |
| `void update()` | 50 Hz: `readRaw()` → ऑफ़सेट घटाना → स्केल → `MAG_ROTATION_CW_DEG` से घुमाव → हेडिंग `atan2(Y, X)` 0..360 में |
| `void calibrate()` | 15 s घुमाना: ऑफ़सेट = हर धुरी पर (min + max)/2 (hard-iron), NVS में सहेजना। एक भी सफल रीडिंग न हो तो कैलिब्रेशन ठुकरा दिया जाता है, NVS का पुराना वाला नहीं छुआ जाता |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

सुरक्षित API: कंस्ट्रक्टर `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (`true` होने पर कैलिब्रेशन NVS से लोड करता है)।

झुकाव के संशोधन के बिना हेडिंग: तब तक सही है जब तक विमान लगभग क्षितिज पर है। नाक उत्तर
की ओर → क्षेत्र +X के साथ → 0°; नाक पूर्व की ओर → 90°।

---

## `QMC5883P_Sensor`

**फ़ाइल:** `sensors/mag/QMC5883P_Sensor.h` · **इनहेरिट करता है:** `MagnetometerBase` · **NVS:** `qmc5883p` · **स्थिति:** बेंच पर

`DEFAULT_ADDRESS = 0x2C`। `begin()`: चिप ID `0x80` (रजिस्टर 0x00), सॉफ़्टवेयर
रीसेट, धुरियों के चिह्न `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8)। डेटा — `0x01` से 6 बाइट,
little-endian। 37.5 LSB/µT।

---

## `QMC5883L_Sensor`

**फ़ाइल:** `sensors/mag/QMC5883L_Sensor.h` · **इनहेरिट करता है:** `MagnetometerBase` · **NVS:** `qmc5883l` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

`DEFAULT_ADDRESS = 0x0D`। `begin()`: `probe()` (चिप में भरोसेमंद ID नहीं है),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512)।
डेटा — `0x00` से 6 बाइट, little-endian। 30 LSB/µT। रजिस्टर QMC5883P से
**संगत नहीं** हैं।

---

## `QMC6309_Sensor`

**फ़ाइल:** `sensors/mag/QMC6309_Sensor.h` · **इनहेरिट करता है:** `MagnetometerBase` · **NVS:** `qmc6309` · **स्थिति:** हार्डवेयर पर जाँचा नहीं गया

QMC6309 (QST), I2C 0x7C — सामान्य दायरे 0x08..0x77 से बाहर का पता
(कंसोल `b` से बसों की जाँच 0x7F तक जाती है)।

- `begin()`: `CHIP_ID` (0x00) = 0x90; रीसेट (CTRL2 0x80 → 0x00), `NVM_RDY | NVM_LOAD_DONE`
  की प्रतीक्षा; ±8 G, 200 Hz, set/reset; LPF 16, OSR 8, normal।
- `readRaw()`: 0x01 से X/Y/Z little-endian; 40.96 LSB/µT।

---

## `UbloxM10_Gps`

**फ़ाइल:** `sensors/gps/UbloxM10_Gps.h` · **इनहेरिट करता है:** `GpsSensor` · **निर्भर है:** `IUartPort`, `Config` · **स्थिति:** बेंच पर जुड़ा नहीं है

UART पर u-blox M10, UBX प्रोटोकॉल। केवल **NAV-PVT** पार्स होता है (क्लास 0x01,
id 0x07, 92 बाइट)।

| मेथड | विवरण |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 बॉड → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 बॉड → CFG-VALSET: 10 Hz, UART1 पर NAV-PVT, UBX चालू, NMEA बंद। TX पिन के बिना (`PIN_GPS_TX < 0`) केवल 9600 पर सुनता है। हमेशा `true` (ACK नहीं होता) |
| `bool isAvailable() const` | वैध NAV-PVT आया था और आख़िरी वाला `GPS_TIMEOUT_US` से पुराना नहीं है |
| `void update()` | UART में जो कुछ है वह पार्सर को खिला देना |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = उपलब्ध और `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` `available` को `isAvailable()` के अनुसार दिखाता है (टाइमआउट का ध्यान रखकर) |

पार्सर बाइट-दर-बाइट की स्टेट मशीन है `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, class+id+len+payload पर Fletcher-8 चेकसम।
लंबाई > 512 सिंक्रनाइज़ेशन का टूटना है, खोज फिर से शुरू होती है। NAV-PVT के फ़ील्ड ऐसे पार्स होते हैं: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, 0..360 में लाया जाता है)।

भीतर का `ValsetBuilder` UBX-CFG-VALSET का पेलोड है (version 0, layer RAM, U4 LE की कुंजी
और 1/2/4 बाइट LE के मान के जोड़े, 64 बाइट का बफ़र)।
