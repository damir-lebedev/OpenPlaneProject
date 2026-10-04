<div dir="rtl">

# ‏SENSORS — المستشعرات

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/sensors.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

بُنيت المستشعرات على ثلاثة مستويات:

1. **واجهات الفئات** (`SensorInterface.h`) — ما يراه `Autopilot`
   و`ArmingManager` والقياس عن بُعد.
2. **الأصناف الأساسية للفئات** (`ImuSensorBase` و`BarometerBase` و
   `MagnetometerBase`) — كل ما هو مشترك: المعايرات والمرشّحات ودوران المحاور والإشارات
   وعدّ الأخطاء والحفظ في NVS. نمط Template Method.
3. **مشغّلات الرقاقات** — السجلات والصيغ من ورقة البيانات فقط. تستقبل
   `IRegisterDevice&` (ولا تميّز بين الناقلات) أو `IUartPort&`.

أيّ رقاقة تُترجَم يقرّره `SensorSelection.h`.

---

## الواجهات وبنى البيانات

**الملف:** `sensors/SensorInterface.h`

### البنى

| البنية | الحقول |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (الدحرجة + الجناح الأيمن لأسفل، والانحدار + الأنف لأعلى، والانعراج + الأنف لليمين)؛ `accelX/Y/Z` g (محاور الطائرة: X نحو الأنف، وY لليسار، وZ لأعلى)؛ `roll` (−180..180) و`pitch` (−90..90) و`yaw` (−180..180، وهو تكامل الجيروسكوب) °؛ `temperature` °C؛ `timestamp` µs |
| `BarometerData` | `pressure` Pa؛ `temperature` °C؛ `altitude` m **نسبةً إلى نقطة المعايرة**؛ `verticalSpeed` m/s؛ `timestamp` µs |
| `MagData` | `magX/Y/Z` µT بعد معايرة hard-iron، في محاور الطائرة؛ `headingDegrees` من 0 إلى 360 (دون تعويض الميلان)؛ `timestamp` |
| `GpsData` | `latitude` و`longitude` (double، °)؛ `altitude` m MSL؛ `groundSpeed` m/s؛ `heading` من 0 إلى 360؛ `numSatellites`؛ `fixType` (0 لا يوجد، و2 — 2D، و3 — 3D)؛ `horizontalAccuracy` و`verticalAccuracy` m؛ `timestamp` |

### ‏`Sensor` (واجهة)

| الدالة | الوصف |
|---|---|
| `bool begin()` | التعرّف على الرقاقة وضبطها؛ `true` — المستشعر يعمل |
| `bool isAvailable() const` | موصول ويستجيب **الآن** |
| `void update()` | تُستدعى في كل نبضة؛ وهو يقرّر بنفسه هل حان وقت القراءة |
| `const char* getSensorType() const` | اسم للسجل |
| `void printStatus() const` | سطر تشخيص (أمر الوحدة الطرفية `s`) |

### ‏`ImuSensor : Sensor`

| الدالة | الوصف |
|---|---|
| `const ImuData& getImuData() const` | آخر البيانات |
| `void calibrate()` | معايرة الجيروسكوب (والجهاز ساكن) + الفحص قبل الإقلاع |
| `void setYaw(float)` | ضبط الاتجاه (مثلًا من البوصلة عند التشغيل) |
| `virtual void calibrateOrientation()` | معايرة تركيب اللوحة (غير مدعومة افتراضيًا) |
| `virtual const char* getPreflightProblem() const` | مشكلة الفحص قبل الإقلاع أو `nullptr` (والافتراضي `nullptr`) |

### ‏`BarometerSensor : Sensor`

‏`getBarometerData()` و`calibrateAltitude()` (الارتفاع الحالي = 0) و
`setSeaLevelPressure(Pa)`.

### ‏`MagnetometerSensor : Sensor`

‏`getMagData()` و`calibrate()` (hard-iron: التدوير لمدة 15 s).

### ‏`GpsSensor : Sensor`

‏`getGpsData()` و`hasFix()` (يوجد تثبيت 2D على الأقل).

---

## ‏`AirspeedSensor`

**الملف:** `sensors/airspeed/AirspeedSensor.h` · **النوع:** واجهة · **التنفيذ:** `PitotDualBaroAirspeed`

‏`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
فرق الضغط بعد التصفير والترشيح، والسرعة المبيّنة (ρ0 = 1.225)، والسرعة الحقيقية
(ρ من الضغط الساكن ودرجة الحرارة)، والكثافة. الدوال: `getAirspeedData()` و
`calibrateZero()` (إعادة بدء التصفير) و`isZeroing()`.

## ‏`PitotDualBaroAirspeed`

**الملف:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **يرث من:** `AirspeedSensor` · **الحالة:** جرى التحقق منها في محاكاة حلقة مغلقة مع ضجيج، ولم تُجرَّب في الطيران

أنبوب بيتو مصنوع منزليًا على **مقياسَي ضغط جوي مطلقَين**: `total` هو
BMP581 داخل الأنبوب (الضغط الكلي)، و`stat` هو مقياس الضغط الرئيسي في جسم الطائرة
(الضغط الساكن). دليل التصنيع في [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#أنبوب-بيتو-من-صنع-يديك).

| الدالة | الوصف |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | الدالة `begin()` لمقياس ضغط الأنبوب (والساكن يعمل بالفعل)، وبدء التصفير |
| `update()` | عيّنة جديدة من الأنبوب ← الفرق − الصفر، ومرشّح تمرير منخفض `PITOT_FILTER_TAU_S`؛ أول `PITOT_ZERO_SAMPLES` عيّنة تُستخدم لمتوسط الصفر؛ والسرعة `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | المقياسان كلاهما حيّ، والصفر اكتمل جمعه، ولا عطل، والعيّنة أحدث من `PITOT_STALE_US` |
| `hasFault()` | الفرق دون −`PITOT_NEGATIVE_FAULT_PA` لمدة أطول من `PITOT_NEGATIVE_FAULT_MS` (الخراطيم أو الماء) |
| `getZeroOffset()`، `printStatus()` | تشخيص |
| `static speedFrom(Δp, ρ)`، `static densityOf(p, T)` | الصيغ |

---

## ‏namespace `SensorMounting`

**الملف:** `sensors/SensorMounting.h`

‏`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
تدوّر محاور الرقاقة حول الرأسي (الرقاقة لأعلى) إلى محاور الطائرة (X نحو الأنف،
وY لليسار). و`rotationCwDeg` هو الجهة التي يشير إليها محور X للرقاقة، مع عقارب الساعة من الأعلى:

| القيمة | bodyX | bodyY |
|---|---|---|
| 0 (وأي قيمة مجهولة) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

تستخدمه البوصلة (`MAG_ROTATION_CW_DEG`) ووحدة IMU التي لم تُعايَر لها زاوية التركيب.

---

## ‏`SensorSelection.h`

**الملف:** `sensors/SensorSelection.h` · **النوع:** إعداد للمعالج المسبق

المكان الوحيد لتغيير المستشعر المادي: طقم جاهز بسطر واحد
(`SENSOR_KIT`) أو كل مستشعر على حدة. ويمكن تجاوز أي اختيار
بعلامة بناء (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`،
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| الطقم `SENSOR_KIT` | IMU | مقياس الضغط | البوصلة | سرعة الهواء | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1، الافتراضي) | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (الجسم) | QMC6309 | BMP581 في الأنبوب | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (الجسم) | QMC6309 | BMP581 في الأنبوب | M10 |
| `SENSOR_KIT_CUSTOM` (0) | حدّد الماكرو الخمسة التالية كلها | | | | |

| ماكرو الاختيار | الخيارات |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1)، `ICM42688` (2، SPI)، `LSM6DSV` (3)، `LSM6DSV_SPI` (4)، `ICM45686` (5)، `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1)، `BMP388` (2، SPI)، `BMP388_I2C` (3)، `SPL06` (4)، `SPL06_SPI` (5)، `BMP581` (6)، `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0)، `QMC5883P` (1)، `QMC5883L` (2)، `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0)، `PITOT_BMP581` (1) — BMP581 على I2C بالعنوان 0x47 في الأنبوب |
| `SENSOR_GPS` | `NONE` (0)، `UBLOX_M10` (1) |

تُبنى كل التوليفات على كل اللوحات — `tools/build_matrix.sh`.

والناتج أسماء بديلة للأنواع ومصانع للأجهزة:

| الاسم | MPU6050 / ICM42688 وغيرها |
|---|---|
| `SelectedImu`، `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`، `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (الخط CS هو `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`، `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (غير معرَّفة عند NONE) |
| وحدات IMU الجديدة | `LSM6DSV_Sensor` + I2C 0x6A (الاحتياطي 0x6B) أو SPI؛ `ICM45686_Sensor` + I2C 0x68 (0x69) أو SPI |
| مقاييس الضغط الجديدة | `SPL06_Sensor` + I2C 0x76 (0x77) أو SPI؛ `BMP581_Sensor` + I2C 0x46 (0x47) أو SPI |
| `SelectedPitotBaro`، `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (غير معرَّفة عند NONE) |
| `SelectedGps` | `UbloxM10_Gps` (غير معرَّف عند NONE) |

يلفّ `main.cpp` إنشاء البوصلة وGPS والأنبوب داخل `#if SENSOR_* != SENSOR_*_NONE`.

---

## ‏`ImuOrientation`

**الملف:** `sensors/imu/ImuOrientation.h` · **يعتمد على:** `SensorMounting`، `Preferences` (NVS)

مصفوفة الدوران `R` من محاور الرقاقة إلى محاور الطائرة: `body = R · chip`؛ وصفوف
`R` هي محاور الطائرة معبَّرًا عنها في محاور الرقاقة.

| الدالة | الوصف |
|---|---|
| `ImuOrientation()` | مصفوفة الوحدة (تكافئ `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | دوران حول الرأسي بخطوات من 90°، واللوحة والرقاقة لأعلى |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | معايرة من ثلاثة أوضاع (قراءة «الأعلى» من مقياس التسارع في محاور الرقاقة). `nullptr` — نجاح، وإلا فسبب الرفض |
| `void apply(const float chip[3], float body[3]) const` | تطبيق الدوران |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | الزاوية بين «الأعلى» المقيس ومحور Z للطائرة (180° إذا كان المتجه صفرًا) |
| `bool load(const char* ns)` | التحميل من NVS؛ ويرفض ثلاثية غير متعامدة أحادية أو يسارية |
| `void save(const char* ns) const` | الحفظ في NVS |
| `void describe(Print&) const` | «الأنف = +Y للرقاقة، والأعلى = +Z للرقاقة» (مع الزاوية إذا لم ينطبق المحور على محور الرقاقة ضمن ±14°) |

خوارزمية `fromPoses`: Z = norm(level)؛ X₁ = مركّبة noseUp العمودية على Z؛ Y = مركّبة
rightWingDown العمودية على Z، وX₂ = Y × Z؛ X = norm(X₁ + X₂)، وY = Z × X. حالات الرفض:

| الشرط | الرسالة |
|---|---|
| متجه صفري | «لا توجد قراءات من مقياس التسارع» |
| ميلان الخطوة 2 أو 3 خارج المدى 20..80° | «الخطوة N تحتاج إلى ميلان 30-60°» |
| cos(X₁, X₂) أقل من −0.5 | «الخطوتان 2 و3 متناقضتان…» (خُفض الأنف أو استُخدم الجناح الخطأ) |
| cos(X₁, X₂) أقل من 0.9 (≈25°) | «…مِيلت محاور خاطئة…» |

---

## ‏`AttitudeEstimator`

**الملف:** `sensors/imu/AttitudeEstimator.h`

مرشّح تكميلي للدحرجة والانحدار وتكامل للانعراج، لا يعتمد على الرقاقة.

| الدالة | الوصف |
|---|---|
| `void reset()` | تبدأ `update()` التالية فورًا من زاوية مقياس التسارع |
| `void setYaw(float deg)` | ضبط الاتجاه (يُختزل إلى ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | التسارع بوحدة g في محاور الطائرة، والمعدلات بوحدة °/s |
| `getRoll()`، `getPitch()`، `getYaw()` | ° |

‏`roll_acc = atan2(ay, az)`، و`pitch_acc = atan2(ax, √(ay² + az²))`؛
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0.1 s عند 2 ms).
أول استدعاء بعد `reset()` يعطي زوايا مقياس التسارع مباشرة؛ وإذا كان `dt ≤ 0` أو أكبر من 0.1 s
فتُتخطّى الخطوة (توقف، أو تعليق في الناقل).

---

## ‏`ImuSensorBase`

**الملف:** `sensors/imu/ImuSensorBase.h` · **يرث من:** `ImuSensor` · **النوع:** مجرَّد

‏`RawImuSample` — عيّنة واحدة بوحدات ADC في محاور الرقاقة:
`accelX/Y/Z` و`gyroX/Y/Z` و`temperature` (`int16_t`).

مسار المعالجة `update()` ← `process()`:

<div dir="ltr">

```
العيّنات الخام − الإزاحات (ADC) → المقياس (g، °/s) → ImuOrientation (محاور الطائرة)
→ إشارات الطيران (gyroY وgyroZ بعد عكس الإشارة) → AttitudeEstimator
```

</div>

| الدالة | الوصف |
|---|---|
| `bool isAvailable() const` | نجحت `begin()` وأخطاء القراءة المتتالية أقل من 50 |
| `void update()` | قراءة واحدة؛ وعند الخطأ لا تتغير البيانات وتزداد العدّادات |
| `void calibrate()` | 200 عيّنة × 10 ms: إزاحة الجيروسكوب والضجيج و«الأعلى»؛ ودون معايرة التركيب — الأفق = الوضع الحالي. ثم الفحص قبل الإقلاع |
| `void calibrateOrientation()` | ثلاثة أوضاع (`capturePose`: ساكنة ~1 s، والوضع يختلف عن السابقة بما لا يقل عن 20°، ومهلة 30 s)، ثم `ImuOrientation::fromPoses`، والحفظ في NVS. تُزيل مشكلات التركيب في الفحص قبل الإقلاع |
| `const char* getPreflightProblem() const` | نص المشكلة أو `nullptr` |
| `void setYaw(float)`، `getSensorType()`، `printStatus()` | |

الواجهة البرمجية المحمية للمشغّلات:

| الدالة | الوصف |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | لكل مشغّل مساحة أسماء خاصة في NVS |
| `virtual bool readSample(RawImuSample&) = 0` | عيّنة واحدة؛ و`false` — الرقاقة لم تستجب |
| `virtual float accelLsbPerG() const = 0`، `gyroLsbPerDps() const = 0` | المقاييس |
| `virtual float temperatureC(int16_t raw) const = 0` | صيغة درجة الحرارة |
| `void setAvailable(bool)` | نتيجة `begin()`؛ وعند `true` تحمّل التركيب من NVS أو من `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | تدقيق الاسم بعد التعرّف |

الفحص قبل الإقلاع (`runPreflightCheck`)، بالترتيب:

| المشكلة | الشرط |
|---|---|
| `NotResponding` | قُرئ أقل من نصف عيّنات المعايرة |
| `Moved` | ضجيج الجيروسكوب أكبر من 0.5 °/s |
| `NotOneG` | \|a\| يختلف عن 1g بأكثر من 0.2g |
| `MountingMismatch` | (التركيب معايَر) «الأعلى» يبعد أكثر من 45° عن المحفوظ |
| `NotChipUp` | (غير معايَر) اللوحة ليست موضوعة والرقاقة لأعلى (`z < 0.5g`) |

---

## ‏`MPU6050_Sensor`

**الملف:** `sensors/imu/MPU6050_Sensor.h` · **يرث من:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **الحالة:** على منضدة الاختبار (MPU6500)

‏MPU6050 / MPU6500 / MPU9250 / MPU9255 ونسخها المقلَّدة (لوحات GY-521)، على I2C بالعنوان 0x68/0x69 أو
على SPI. تُعرَف الرقاقة بالسجل `WHO_AM_I` (0x68 — MPU6050، وإلا فعائلة 6500).

- ‏`begin()`: WHO_AM_I (لا استجابة ← غير متاح)، والاسم بحسب المعرِّف، وإعادة الضبط، والخروج من
  السكون (PLL)، و±2000 °/s، و±16 g، ومرشّح DLPF بنحو 41 Hz، و1 kHz؛ وفي 6500 مرشّح
  تمرير منخفض مستقل لمقياس التسارع `ACCEL_CONFIG2`.
- ‏`readSample()`: 14 بايت من `0x3B`، big-endian: accel XYZ وtemp وgyro XYZ.
- المقاييس: 2048 LSB/g و16.4 LSB/(°/s).
- درجة الحرارة: في MPU6050 `raw/340 + 36.53`، وفي MPU6500 `raw/333.87 + 21`.

---

## ‏`ICM42688_Sensor`

**الملف:** `sensors/imu/ICM42688_Sensor.h` · **يرث من:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **الحالة:** لم يُتحقق منه على العتاد

- ‏`static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI بتردد 8 MHz، دون
  بايت وهمي.
- ‏`begin()`: المصرف 0، وإعادة ضبط برمجية، و`WHO_AM_I == 0x47`، وLow Noise،
  و±2000 °/s / ±16 g، و1 kHz، ومرشّح UI عند 50 Hz.
- ‏`readSample()`: 14 بايت من `0x1D`، big-endian: temp وaccel XYZ وgyro XYZ.
- درجة الحرارة: `raw/132.48 + 25`.

---

## ‏`LSM6DSV_Sensor`

**الملف:** `sensors/imu/LSM6DSV_Sensor.h` · **يرث من:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **الحالة:** لم يُتحقق منه على العتاد

‏LSM6DSV / LSM6DSV16X / LSM6DSV32X (من ST). السجلات مطابَقة مع `lsm6dsv-pid` من ST ومع ArduPilot.

- ‏`begin()`: `WHO_AM_I` (0x0F) = 0x70؛ و`SW_RESET` (البت 0 في CTRL3) مع انتظار؛
  ويُميَّز 32X بواسطة بت الإصدار في CTRL8 (وله رمزه الخاص للمدى ±16 g)؛ BDU مع الزيادة التلقائية،
  و±2000 °/s مع LPF1، و±16 g مع LPF2، و960 Hz بالأداء العالي.
- ‏`readSample()`: 14 بايت من 0x20، little-endian: temp وgyro XYZ وaccel XYZ.
- المقاييس: 1000/0.488 LSB/g و1000/70 LSB/(°/s)؛ ودرجة الحرارة `raw/256 + 25`.
- ‏`static spiDevice(bus, cs)` — نمط SPI رقم 0، دون بايت وهمي.

## ‏`ICM45686_Sensor`

**الملف:** `sensors/imu/ICM45686_Sensor.h` · **يرث من:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **الحالة:** لم يُتحقق منه على العتاد

‏ICM-45686 (من TDK). السجلات مطابَقة مع مشغّل TDK ومع Zephyr وArduPilot.

- ‏`begin()`: إعادة ضبط عبر `REG_MISC2` (0x7F)، و`WHO_AM_I` (0x72) = 0xE9؛ و±2000 °/s و±16 g
  عند 1.6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15)، وLow Noise (`PWR_MGMT0` = 0x0F)؛ ومرشّح
  التمرير المنخفض ODR/32 — قراءة ثم تعديل ثم كتابة للسجلات **غير المباشرة** IPREG
  (0xA4AC و0xA583) عبر النافذة 0x7C..0x7E؛ و45 ms لإقلاع الجيروسكوب.
- ‏`readSample()`: 14 بايت من 0x00، little-endian: accel XYZ وgyro XYZ وtemp.
- المقاييس: 2048 LSB/g و16.4 LSB/(°/s)؛ ودرجة الحرارة `raw/132.48 + 25`.

---

## ‏`BarometerBase`

**الملف:** `sensors/baro/BarometerBase.h` · **يرث من:** `BarometerSensor` · **النوع:** مجرَّد

| الدالة | الوصف |
|---|---|
| `bool isAvailable() const` | نجحت `begin()` والأخطاء المتتالية أقل من 100 |
| `void update()` | لا أكثر من مرة كل `pollPeriodUs`: `isNewSampleReady()` ← `readSample()` ← الارتفاع والسرعة الرأسية |
| `void calibrateAltitude()` | 20 عيّنة × 50 ms: متوسط الارتفاع المطلق = القاعدة؛ وتصفير الارتفاع والسرعة |
| `void setSeaLevelPressure(Pa)` | P₀ (القيمة الافتراضية 101325) |
| `getBarometerData()`، `getSensorType()`، `printStatus()` | |

الواجهة البرمجية المحمية: المُنشئ `(name, pollPeriodUs)`،
و`virtual bool isNewSampleReady(bool& ready) = 0`،
و`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`،
و`setAvailable(bool)`.

الصيغ: `h = 44330 · (1 − (P/P₀)^0.1903) − base`؛ والسرعة الرأسية هي
مشتقة الارتفاع على العيّنات **الجديدة الفعلية** عبر مرشّح تمرير منخفض بثابت τ = 0.5 s
(وإذا خرج `dt` عن `(0, 0.5 s)` فلا تُحدَّث السرعة). وقراءة العيّنات الجديدة فقط
تزيل الضجيج «الدرجي» (0 m/s تتخللها قفزات Δh/2 ms).

---

## ‏`BMP388_Sensor`

**الملف:** `sensors/baro/BMP388_Sensor.h` · **يرث من:** `BarometerBase` · **الحالة:** على منضدة الاختبار (I2C)

- ‏`static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI بتردد 8 MHz،
  و**بايت وهمي واحد** (ورقة البيانات §5.3.2).
- ‏`begin()`: معرّف الرقاقة `0x50`، وإعادة ضبط برمجية، و21 بايت من معاملات NVM بدءًا من
  `0x31` (المقاييس §9.1)، وOSR ×8/×1، وODR بتردد 50 Hz، وIIR 3، والنمط العادي.
- ‏`isNewSampleReady()`: راية `drdy_press` (البت 0x20) في سجل STATUS؛ ويُستطلَع
  كل 5 ms.
- ‏`readSample()`: 6 بايت من `0x04`؛ وتعويض Bosch §9.3 (double):
  درجة الحرارة أولًا (`tLin`) ثم الضغط.

---

## ‏`BME280_Sensor`

**الملف:** `sensors/baro/BME280_Sensor.h` · **يرث من:** `BarometerBase` · **الحالة:** لم يُتحقق منه على العتاد

‏BME280 (المعرِّف 0x60) وBMP280 (المعرِّف 0x58)، على I2C بالعنوان 0x76/0x77 أو على SPI دون بايت
وهمي. ولا تُقرأ الرطوبة.

- ‏`begin()`: معرّف الرقاقة، وإعادة الضبط، و24 بايت من المعايرة بدءًا من `0x88`، و`CTRL_HUM` (في
  BME280 فقط، ويُكتب **قبل** `CTRL_MEAS`)، و`CONFIG` = IIR 4 + 0.5 ms، و`CTRL_MEAS`
  = T×2 وP×8 والنمط العادي.
- لا توجد راية جاهزية — `ready = true`، والاستطلاع كل 25 ms.
- التعويض — صيغ Bosch §8.1 (double)؛ مع حماية من القسمة على صفر.

---

## ‏`SPL06_Sensor`

**الملف:** `sensors/baro/SPL06_Sensor.h` · **يرث من:** `BarometerBase` · **الحالة:** لم يُتحقق منه على العتاد

‏SPL06-001 (من Goertek). الصيغ من ورقة البيانات §4.9.

- ‏`begin()`: `ID` (0x0D) = 0x10 (القيمة 0x11 هي SPA06 ذات مجموعة معاملات مختلفة —
  فتُرفض)؛ وإعادة ضبط وانتظار `COEF_RDY | SENSOR_RDY`؛ و18 بايت من المعاملات
  (حقول ذات إشارة بأطوال 12/20/16 بتًّا)؛ ومصدر درجة الحرارة بحسب `COEF_SRCE`؛
  والضغط 16× (32 Hz) ودرجة الحرارة 1× والنمط المستمر.
- ‏`isNewSampleReady()` — البت `PRS_RDY`؛ و`readSample()` — عيّنات من 24 بتًّا
  big-endian، مع `kP = 253952` و`kT = 524288`.

## ‏`BMP581_Sensor`

**الملف:** `sensors/baro/BMP581_Sensor.h` · **يرث من:** `BarometerBase` · **الحالة:** لم يُتحقق منه على العتاد

‏BMP581 (من Bosch). يتبع التسلسل مكتبة BMP5_SensorAPI الرسمية.

- ‏`begin()`: قراءة وهمية (من أجل SPI)، و`CHIP_ID` (0x01) = 0x50/0x51؛
  وإعادة ضبط برمجية، وPOR في `INT_STATUS` وجاهزية NVM في `STATUS` دون أخطاء؛
  ثم standby ← OSR (الضغط 16× ودرجة الحرارة 2×) وIIR وDRDY؛ وفحص
  جدوى ODR (`OSR_EFF`)؛ والنمط المستمر.
- العيّنة — عند DRDY، أو كل 40 ms إذا ضاعت الراية؛ ودرجة الحرارة
  `int24/65536` والضغط `uint24/64`.
- نسختان في البناء الذي فيه الأنبوب: مقياس الضغط الرئيسي و`PITOT-BMP581`.

---

## ‏`MagnetometerBase`

**الملف:** `sensors/mag/MagnetometerBase.h` · **يرث من:** `MagnetometerSensor` · **النوع:** مجرَّد

| الدالة | الوصف |
|---|---|
| `bool isAvailable() const` | نجحت `begin()` والأخطاء المتتالية أقل من 25 |
| `void update()` | 50 Hz: `readRaw()` ← طرح الإزاحات ← المقياس ← الدوران بحسب `MAG_ROTATION_CW_DEG` ← الاتجاه `atan2(Y, X)` في المدى 0..360 |
| `void calibrate()` | 15 s من التدوير: الإزاحة = (min + max)/2 لكل محور (hard-iron)، والحفظ في NVS. وإن لم تنجح قراءة واحدة تُرفض المعايرة وتبقى السابقة في NVS دون مساس |
| `getMagData()`، `getSensorType()`، `printStatus()` | |

الواجهة البرمجية المحمية: المُنشئ `(name, nvsNamespace)`،
و`virtual bool readRaw(int16_t raw[3]) = 0`، و`virtual float lsbPerMicroTesla() const = 0`،
و`setAvailable(bool)` (وعند `true` تحمّل المعايرة من NVS).

الاتجاه دون تعويض الميلان: صحيح ما دامت الطائرة قريبة من الأفق. الأنف نحو
الشمال ← المجال على امتداد +X ← 0°؛ والأنف نحو الشرق ← 90°.

---

## ‏`QMC5883P_Sensor`

**الملف:** `sensors/mag/QMC5883P_Sensor.h` · **يرث من:** `MagnetometerBase` · **NVS:** `qmc5883p` · **الحالة:** على منضدة الاختبار

‏`DEFAULT_ADDRESS = 0x2C`. `begin()`: معرّف الرقاقة `0x80` (السجل 0x00)، وإعادة ضبط
برمجية، وإشارات المحاور `0x29 = 0x06`، و`CONTROL2 = 0x08` (SET/RESET، ±8 G)،
و`CONTROL1 = 0xCD` (عادي، 200 Hz، OSR 8/8). البيانات — 6 بايت من `0x01`،
little-endian. و37.5 LSB/µT.

---

## ‏`QMC5883L_Sensor`

**الملف:** `sensors/mag/QMC5883L_Sensor.h` · **يرث من:** `MagnetometerBase` · **NVS:** `qmc5883l` · **الحالة:** لم يُتحقق منه على العتاد

‏`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (فليس للرقاقة معرّف موثوق)،
و`SET/RESET = 0x01`، و`CONTROL1 = 0x1D` (continuous، 200 Hz، ±8 G، OSR 512).
البيانات — 6 بايت من `0x00`، little-endian. و30 LSB/µT. والسجلات **غير متوافقة**
مع QMC5883P.

---

## ‏`QMC6309_Sensor`

**الملف:** `sensors/mag/QMC6309_Sensor.h` · **يرث من:** `MagnetometerBase` · **NVS:** `qmc6309` · **الحالة:** لم يُتحقق منه على العتاد

‏QMC6309 (من QST)، على I2C بالعنوان 0x7C — وهو عنوان خارج المدى المعتاد 0x08..0x77
(ومسح الناقلات بأمر الوحدة الطرفية `b` يصل إلى 0x7F).

- ‏`begin()`: `CHIP_ID` (0x00) = 0x90؛ وإعادة ضبط (CTRL2 من 0x80 إلى 0x00)، وانتظار
  `NVM_RDY | NVM_LOAD_DONE`؛ و±8 G و200 Hz وset/reset؛ وLPF 16 وOSR 8 والنمط العادي.
- ‏`readRaw()`: X/Y/Z بنظام little-endian من 0x01؛ و40.96 LSB/µT.

---

## ‏`UbloxM10_Gps`

**الملف:** `sensors/gps/UbloxM10_Gps.h` · **يرث من:** `GpsSensor` · **يعتمد على:** `IUartPort`، `Config` · **الحالة:** غير موصول على منضدة الاختبار

مستقبِل u-blox M10 عبر UART، بروتوكول UBX. لا يُحلَّل سوى **NAV-PVT** (الفئة 0x01،
والمعرِّف 0x07، و92 بايت).

| الدالة | الوصف |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 باود ← CFG-VALSET `UART1_BAUDRATE = 115200` ← 115200 باود ← CFG-VALSET: 10 Hz، وNAV-PVT على UART1، وUBX مفعّل، وNMEA معطّل. وبلا طرف TX (`PIN_GPS_TX < 0`) يكتفي بالاستماع عند 9600. وتُرجع دائمًا `true` (لا يوجد ACK) |
| `bool isAvailable() const` | وصل NAV-PVT صالح وآخر واحد منها ليس أقدم من `GPS_TIMEOUT_US` |
| `void update()` | تمرير كل ما في UART إلى المحلّل |
| `const GpsData& getGpsData() const`، `bool hasFix() const` | `hasFix` = متاح و`fixType ≥ 2` |
| `getSensorType()`، `printStatus()` | تعرض `printStatus()` القيمة `available` بحسب `isAvailable()` (مع مراعاة المهلة) |

المحلّل آلة حالات بايتًا ببايت `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`، ومجموع التحقق Fletcher-8 على class+id+len+payload.
وإذا زاد الطول على 512 فهذا فقدان للتزامن ويبدأ البحث من جديد. وتُحلَّل حقول NAV-PVT هكذا: `fixType`
(20) و`numSV` (23) و`lon`/`lat` (24/28، ×1e−7) و`hMSL` (36، mm) و`hAcc`/`vAcc`
(40/44، mm) و`gSpeed` (60، mm/s) و`headMot` (64، ×1e−5 °، ويُختزل إلى 0..360).

والكائن المتداخل `ValsetBuilder` هو حمولة UBX-CFG-VALSET (الإصدار 0، الطبقة RAM، أزواج
مفتاح U4 LE — قيمة من 1/2/4 بايت LE، ومخزن مؤقت حجمه 64 بايت).

</div>
