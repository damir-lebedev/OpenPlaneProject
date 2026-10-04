# SENSORS — 센서

> 🌐 이 문서는 [러시아어 원문](../../../reference/sensors.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

[← 참조](README.md)

센서는 세 단계로 구성됩니다.

1. **카테고리 인터페이스** (`SensorInterface.h`) — `Autopilot`, `ArmingManager`,
   텔레메트리가 보는 부분입니다.
2. **카테고리 기반 클래스** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — 공통 부분 전부, 즉 보정, 필터, 축 회전, 부호,
   오류 횟수 세기, NVS 저장을 맡습니다. Template Method 패턴입니다.
3. **칩 드라이버** — 데이터시트의 레지스터와 공식만 담습니다.
   `IRegisterDevice&` (버스를 구분하지 않음) 또는 `IUartPort&`를 받습니다.

어떤 칩을 컴파일할지는 `SensorSelection.h`가 결정합니다.

---

## 인터페이스와 데이터 구조

**파일:** `sensors/SensorInterface.h`

### 구조체

| 구조체 | 필드 |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (롤 + 오른쪽 날개 아래, 피치 + 기수 위, 요 + 기수 오른쪽); `accelX/Y/Z` g (기체의 축: X는 기수 방향, Y는 왼쪽, Z는 위쪽); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, 자이로의 적분값) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m (**보정 지점 기준 상대값**); `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | hard-iron 보정 후 기체의 축에서의 `magX/Y/Z` µT; `headingDegrees` 0..360 (기울기 보정 없음); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 없음, 2는 2D, 3은 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (인터페이스)

| 메서드 | 설명 |
|---|---|
| `bool begin()` | 칩을 식별하고 설정합니다. `true`이면 센서가 동작 중입니다 |
| `bool isAvailable() const` | 연결되어 있고 **지금** 응답함 |
| `void update()` | 매 틱마다 호출합니다. 읽을 때가 되었는지는 스스로 판단합니다 |
| `const char* getSensorType() const` | 로그에 쓸 이름 |
| `void printStatus() const` | 진단 한 줄 (콘솔의 `s`) |

### `ImuSensor : Sensor`

| 메서드 | 설명 |
|---|---|
| `const ImuData& getImuData() const` | 최신 데이터 |
| `void calibrate()` | 자이로 보정 (정지 상태에서) + 비행 전 점검 |
| `void setYaw(float)` | 방위를 설정합니다 (예를 들어 시작할 때 나침반으로) |
| `virtual void calibrateOrientation()` | 기판 장착 방향의 보정 (기본적으로는 지원하지 않음) |
| `virtual const char* getPreflightProblem() const` | 비행 전 점검에서 발견된 문제 또는 `nullptr` (기본값은 `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (현재 고도를 0으로),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: 15초 동안 회전시킵니다).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (적어도 2D 픽스가 있음).

---

## `AirspeedSensor`

**파일:** `sensors/airspeed/AirspeedSensor.h` · **종류:** 인터페이스 · **구현:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
영점 조정과 필터를 거친 차압, 지시 대기속도 (ρ0 = 1.225), 진대기속도 (ρ는
정압과 온도로 구함), 공기 밀도입니다. 메서드: `getAirspeedData()`,
`calibrateZero()` (영점 조정을 다시 시작), `isZeroing()`.

## `PitotDualBaroAirspeed`

**파일:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **상속:** `AirspeedSensor` · **상태:** 노이즈를 넣은 폐루프 시뮬레이션에서 검증했으며, 비행 시험은 하지 않았습니다

**절대압 기압계 두 개**로 만든 자작 피토관입니다. `total`은 관 안의
BMP581 (전압), `stat`는 동체의 주 기압계 (정압)입니다.
제작 안내는 [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#직접-만드는-피토관)에 있습니다.

| 메서드 | 설명 |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | 관 기압계의 `begin()` (정압 쪽은 이미 가동 중), 영점 조정 시작 |
| `update()` | 관의 새 샘플 → 차압 − 영점, 저역 통과 필터 `PITOT_FILTER_TAU_S`. 처음 `PITOT_ZERO_SAMPLES`개의 샘플로 영점을 평균합니다. 속도는 `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | 두 기압계가 모두 살아 있고, 영점이 모였고, 고장이 없고, 샘플이 `PITOT_STALE_US`보다 최신임 |
| `hasFault()` | 차압이 −`PITOT_NEGATIVE_FAULT_PA`보다 낮은 상태가 `PITOT_NEGATIVE_FAULT_MS`보다 오래 이어짐 (호스, 물) |
| `getZeroOffset()`, `printStatus()` | 진단 |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | 공식 |

---

## namespace `SensorMounting`

**파일:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
칩의 축을 수직축(칩이 위를 향함)을 중심으로 회전시켜 기체의 축(X는 기수 방향,
Y는 왼쪽)으로 변환합니다. `rotationCwDeg`는 칩의 X축이 향하는 방향이며, 위에서 보아 시계 방향입니다.

| 값 | bodyX | bodyY |
|---|---|---|
| 0 (그리고 알 수 없는 모든 값) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

나침반 (`MAG_ROTATION_CW_DEG`)과 장착 방향 보정이 없는 IMU가 사용합니다.

---

## `SensorSelection.h`

**파일:** `sensors/SensorSelection.h` · **종류:** 전처리기 구성

물리적인 센서를 바꾸는 유일한 곳입니다. 한 줄로 고르는 완성된 세트
(`SENSOR_KIT`)를 쓰거나 센서를 하나씩 따로 지정합니다. 어떤 선택이든
빌드 플래그로 덮어쓸 수 있습니다 (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| `SENSOR_KIT` 세트 | IMU | 기압계 | 나침반 | 대기속도 | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, 기본값) | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (동체) | QMC6309 | 관 안의 BMP581 | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (동체) | QMC6309 | 관 안의 BMP581 | M10 |
| `SENSOR_KIT_CUSTOM` (0) | 아래 다섯 매크로를 모두 지정 | | | | |

| 선택 매크로 | 선택지 |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — 관 안의 BMP581, I2C 0x47 |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

모든 조합이 모든 보드에서 빌드됩니다 — `tools/build_matrix.sh`.

그 결과물은 타입 별칭과 장치 팩토리입니다.

| 이름 | MPU6050 / ICM42688 등 |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS는 `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (NONE에서는 정의되지 않음) |
| 새 IMU | `LSM6DSV_Sensor` + I2C 0x6A (예비는 0x6B) 또는 SPI, `ICM45686_Sensor` + I2C 0x68 (0x69) 또는 SPI |
| 새 기압계 | `SPL06_Sensor` + I2C 0x76 (0x77) 또는 SPI, `BMP581_Sensor` + I2C 0x46 (0x47) 또는 SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (NONE에서는 정의되지 않음) |
| `SelectedGps` | `UbloxM10_Gps` (NONE에서는 정의되지 않음) |

`main.cpp`는 나침반, GPS, 피토관의 생성을 `#if SENSOR_* != SENSOR_*_NONE`으로 감쌉니다.

---

## `ImuOrientation`

**파일:** `sensors/imu/ImuOrientation.h` · **의존:** `SensorMounting`, `Preferences` (NVS)

칩의 축에서 기체의 축으로 가는 회전 행렬 `R`입니다. `body = R · chip`이며, `R`의 행은
칩의 축으로 나타낸 기체의 축입니다.

| 메서드 | 설명 |
|---|---|
| `ImuOrientation()` | 단위 행렬 (`fromYawSteps(0)`과 같음) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | 수직축을 중심으로 90° 단위로 회전합니다. 기판은 칩이 위를 향함 |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | 세 가지 자세로 하는 보정 (칩의 축에서 가속도계가 읽은 “위” 값). `nullptr`이면 성공, 아니면 거부 사유 |
| `void apply(const float chip[3], float body[3]) const` | 회전을 적용합니다 |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | 측정한 “위”와 기체의 Z축 사이의 각도 (벡터가 0이면 180°) |
| `bool load(const char* ns)` | NVS에서 불러옵니다. 정규직교가 아니거나 왼손 좌표계인 세 벡터는 거부합니다 |
| `void save(const char* ns) const` | NVS에 저장합니다 |
| `void describe(Print&) const` | “기수 = 칩의 +Y, 위 = 칩의 +Z” (축이 칩의 축과 ±14° 이내로 일치하지 않으면 각도와 함께) |

`fromPoses`의 알고리즘: Z = norm(level), X₁ = noseUp의 ⟂ Z 성분, Y =
rightWingDown의 ⟂ Z 성분, X₂ = Y × Z, X = norm(X₁ + X₂), Y = Z × X. 거부되는 경우는 다음과 같습니다.

| 조건 | 메시지 |
|---|---|
| 영벡터 | “가속도계 값이 없습니다” |
| 2단계 또는 3단계의 기울기가 20..80°를 벗어남 | “N단계에서는 30-60°의 기울기가 필요합니다” |
| cos(X₁, X₂) < −0.5 | “2단계와 3단계가 서로 모순됩니다…” (기수를 내렸거나 다른 날개를 썼음) |
| cos(X₁, X₂) < 0.9 (≈25°) | “…잘못된 축을 기울였습니다…” |

---

## `AttitudeEstimator`

**파일:** `sensors/imu/AttitudeEstimator.h`

롤/피치의 상보 필터와 요의 적분으로, 칩에 의존하지 않습니다.

| 메서드 | 설명 |
|---|---|
| `void reset()` | 다음 `update()`는 곧바로 가속도계의 각도에서 시작합니다 |
| `void setYaw(float deg)` | 방위를 설정합니다 (±180으로 정규화됨) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | 기체의 축에서의 가속도(g), 각속도(°/s) |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`,
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (2 ms일 때 τ ≈ 0.1초).
`reset()` 이후 첫 호출은 곧바로 가속도계의 각도를 줍니다. `dt ≤ 0`이거나 0.1초보다 크면
그 단계를 건너뜁니다 (일시 정지, 버스 멈춤).

---

## `ImuSensorBase`

**파일:** `sensors/imu/ImuSensorBase.h` · **상속:** `ImuSensor` · **종류:** 추상 클래스

`RawImuSample` — 칩의 축에서 ADC 단위로 나타낸 샘플 하나입니다.
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

`update()` → `process()` 파이프라인:

```
원시 샘플 − 오프셋 (ADC) → 스케일 (g, °/s) → ImuOrientation (기체의 축)
→ 항공 부호 (gyroY, gyroZ는 부호 반전) → AttitudeEstimator
```

| 메서드 | 설명 |
|---|---|
| `bool isAvailable() const` | `begin()`이 성공했고 읽기 오류가 연속 50회 미만 |
| `void update()` | 한 번 읽습니다. 오류가 나면 데이터는 바뀌지 않고 카운터가 올라갑니다 |
| `void calibrate()` | 샘플 200개 × 10 ms: 자이로 오프셋, 노이즈, “위”. 장착 방향 보정이 없으면 수평 = 현재 자세입니다. 이어서 비행 전 점검을 합니다 |
| `void calibrateOrientation()` | 세 가지 자세 (`capturePose`: 약 1초 정지, 이전 자세와 20° 이상 달라야 함, 제한 시간 30초), `ImuOrientation::fromPoses`, NVS에 저장. 비행 전 점검의 장착 관련 문제를 해소합니다 |
| `const char* getPreflightProblem() const` | 문제의 텍스트 또는 `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

드라이버용 protected API:

| 메서드 | 설명 |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | 드라이버마다 고유한 NVS 네임스페이스를 가집니다 |
| `virtual bool readSample(RawImuSample&) = 0` | 샘플 하나. `false`는 칩이 응답하지 않았다는 뜻 |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | 스케일 |
| `virtual float temperatureC(int16_t raw) const = 0` | 온도 공식 |
| `void setAvailable(bool)` | `begin()`의 결과. `true`이면 장착 방향을 NVS 또는 `Config::IMU_ROTATION_CW_DEG`에서 불러옵니다 |
| `void setName(const char*)` | 식별 후 이름을 구체화합니다 |

비행 전 점검 (`runPreflightCheck`)은 다음 순서로 진행합니다.

| 문제 | 조건 |
|---|---|
| `NotResponding` | 보정 샘플의 절반 미만만 읽힘 |
| `Moved` | 자이로 노이즈가 0.5 °/s를 넘음 |
| `NotOneG` | \|a\|가 1g와 0.2g를 넘게 다름 |
| `MountingMismatch` | (장착 방향이 보정되어 있음) “위”가 저장된 값에서 45°를 넘게 벗어남 |
| `NotChipUp` | (보정되지 않음) 기판이 칩을 위로 하고 놓여 있지 않음 (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**파일:** `sensors/imu/MPU6050_Sensor.h` · **상속:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **상태:** 벤치에서 확인 (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255와 클론 (GY-521 기판), I2C 0x68/0x69 또는
SPI입니다. 칩은 `WHO_AM_I`로 식별합니다 (0x68은 MPU6050, 그 밖에는 6500 계열).

- `begin()`: WHO_AM_I (응답 없음 → 사용 불가), ID에 따른 이름, 리셋, 슬립에서 깨우기
  (PLL), ±2000 °/s, ±16 g, DLPF 약 41 Hz, 1 kHz. 6500에는 가속도계용 별도
  저역 통과 필터 `ACCEL_CONFIG2`가 있습니다.
- `readSample()`: `0x3B`부터 14바이트, 빅 엔디언: accel XYZ, temp, gyro XYZ.
- 스케일: 2048 LSB/g, 16.4 LSB/(°/s).
- 온도: MPU6050은 `raw/340 + 36.53`, MPU6500은 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**파일:** `sensors/imu/ICM42688_Sensor.h` · **상속:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **상태:** 하드웨어에서는 확인하지 않음

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz, 더미
  바이트 없음.
- `begin()`: 뱅크 0, 소프트웨어 리셋, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, UI 필터 50 Hz.
- `readSample()`: `0x1D`부터 14바이트, 빅 엔디언: temp, accel XYZ, gyro XYZ.
- 온도: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**파일:** `sensors/imu/LSM6DSV_Sensor.h` · **상속:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **상태:** 하드웨어에서는 확인하지 않음

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). 레지스터는 ST의 `lsm6dsv-pid` 및 ArduPilot과 대조해 확인했습니다.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70, `SW_RESET` (CTRL3의 비트 0)과 대기.
  32X는 CTRL8의 변형 비트로 구분합니다 (±16 g에 대한 고유한 코드가 있음). BDU + 자동 증가,
  LPF1을 쓴 ±2000 °/s, LPF2를 쓴 ±16 g, 960 Hz 고성능 모드.
- `readSample()`: 0x20부터 14바이트, 리틀 엔디언: temp, gyro XYZ, accel XYZ.
- 스케일: 1000/0.488 LSB/g, 1000/70 LSB/(°/s). 온도는 `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI 모드 0, 더미 바이트 없음.

## `ICM45686_Sensor`

**파일:** `sensors/imu/ICM45686_Sensor.h` · **상속:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **상태:** 하드웨어에서는 확인하지 않음

ICM-45686 (TDK). 레지스터는 TDK의 드라이버, Zephyr, ArduPilot과 대조해 확인했습니다.

- `begin()`: `REG_MISC2` (0x7F)로 리셋, `WHO_AM_I` (0x72) = 0xE9, ±2000 °/s와 ±16 g를
  1.6 kHz로 (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F). ODR/32
  저역 통과 필터는 **간접** 레지스터 IPREG (0xA4AC, 0xA583)의
  읽기-수정-쓰기를 0x7C..0x7E 창으로 처리합니다. 자이로 기동에 45 ms.
- `readSample()`: 0x00부터 14바이트, 리틀 엔디언: accel XYZ, gyro XYZ, temp.
- 스케일: 2048 LSB/g, 16.4 LSB/(°/s). 온도는 `raw/132.48 + 25`.

---

## `BarometerBase`

**파일:** `sensors/baro/BarometerBase.h` · **상속:** `BarometerSensor` · **종류:** 추상 클래스

| 메서드 | 설명 |
|---|---|
| `bool isAvailable() const` | `begin()`이 성공했고 연속 오류가 100회 미만 |
| `void update()` | `pollPeriodUs`보다 자주 실행하지 않습니다: `isNewSampleReady()` → `readSample()` → 고도와 수직 속도 |
| `void calibrateAltitude()` | 샘플 20개 × 50 ms: 평균 절대 고도 = 기준. 고도와 속도를 초기화합니다 |
| `void setSeaLevelPressure(Pa)` | P₀ (기본값 101325) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

protected API: 생성자 `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

공식: `h = 44330 · (1 − (P/P₀)^0.1903) − base`. 수직 속도는 고도를 **실제로 새로운**
샘플에 대해 미분하고 τ = 0.5초의 저역 통과 필터를 거친 값입니다
(`dt`가 `(0, 0.5 s)` 밖이면 속도를 갱신하지 않음). 새 샘플만 읽으면
“계단식” 노이즈 (0 m/s 사이사이에 Δh/2 ms의 도약이 섞이는 현상)가 사라집니다.

---

## `BMP388_Sensor`

**파일:** `sensors/baro/BMP388_Sensor.h` · **상속:** `BarometerBase` · **상태:** 벤치에서 확인 (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz,
  **더미 바이트 1개** (데이터시트 §5.3.2).
- `begin()`: 칩 ID `0x50`, 소프트웨어 리셋, `0x31`부터 NVM 계수 21바이트
  (스케일은 §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, 노멀 모드.
- `isNewSampleReady()`: STATUS 레지스터의 `drdy_press` 플래그 (비트 0x20).
  5 ms마다 폴링합니다.
- `readSample()`: `0x04`부터 6바이트. Bosch의 보정 §9.3 (double):
  온도 (`tLin`)를 먼저, 그다음 기압.

---

## `BME280_Sensor`

**파일:** `sensors/baro/BME280_Sensor.h` · **상속:** `BarometerBase` · **상태:** 하드웨어에서는 확인하지 않음

BME280 (ID 0x60)과 BMP280 (ID 0x58), I2C 0x76/0x77 또는 SPI (더미
바이트 없음). 습도는 읽지 않습니다.

- `begin()`: 칩 ID, 리셋, `0x88`부터 보정 데이터 24바이트, `CTRL_HUM` (BME280에서만,
  `CTRL_MEAS`보다 **먼저** 씀), `CONFIG` = IIR 4 + 0.5 ms, `CTRL_MEAS`
  = T×2, P×8, 노멀.
- 준비 완료 플래그가 없습니다. `ready = true`로 두고 25 ms마다 폴링합니다.
- 보정은 Bosch §8.1의 공식 (double)이며, 0으로 나누기를 방지합니다.

---

## `SPL06_Sensor`

**파일:** `sensors/baro/SPL06_Sensor.h` · **상속:** `BarometerBase` · **상태:** 하드웨어에서는 확인하지 않음

SPL06-001 (Goertek). 공식은 데이터시트 §4.9에 따랐습니다.

- `begin()`: `ID` (0x0D) = 0x10 (0x11은 SPA06으로 계수 집합이 달라
  거부합니다), 리셋, `COEF_RDY | SENSOR_RDY` 대기, 계수 18바이트
  (부호 있는 12/20/16비트 필드), 온도의 출처는 `COEF_SRCE`에 따라 결정,
  기압 16× (32 Hz), 온도 1×, 연속 모드.
- `isNewSampleReady()`는 `PRS_RDY` 비트이고, `readSample()`은 24비트
  빅 엔디언 샘플이며 `kP = 253952`, `kT = 524288`입니다.

## `BMP581_Sensor`

**파일:** `sensors/baro/BMP581_Sensor.h` · **상속:** `BarometerBase` · **상태:** 하드웨어에서는 확인하지 않음

BMP581 (Bosch). 순서는 공식 BMP5_SensorAPI를 따랐습니다.

- `begin()`: 더미 읽기 (SPI용), `CHIP_ID` (0x01) = 0x50/0x51,
  소프트웨어 리셋, `INT_STATUS`의 POR과 `STATUS`의 NVM 준비 완료 (오류 없음),
  standby → OSR (기압 16×, 온도 2×), IIR, DRDY, ODR을 실현할 수 있는지
  확인 (`OSR_EFF`), 연속 모드.
- 샘플은 DRDY가 뜰 때, 플래그가 사라진 경우에는 40 ms마다 읽습니다. 온도는
  `int24/65536`, 기압은 `uint24/64`입니다.
- 피토관이 있는 빌드에는 인스턴스가 두 개 있습니다: 주 기압계와 `PITOT-BMP581`.

---

## `MagnetometerBase`

**파일:** `sensors/mag/MagnetometerBase.h` · **상속:** `MagnetometerSensor` · **종류:** 추상 클래스

| 메서드 | 설명 |
|---|---|
| `bool isAvailable() const` | `begin()`이 성공했고 연속 오류가 25회 미만 |
| `void update()` | 50 Hz: `readRaw()` → 오프셋 빼기 → 스케일 → `MAG_ROTATION_CW_DEG`만큼 회전 → 방위 `atan2(Y, X)`를 0..360으로 |
| `void calibrate()` | 15초 동안 회전: 오프셋 = 각 축의 (min + max)/2 (hard-iron), NVS에 저장합니다. 한 번도 읽기에 성공하지 못했다면 보정을 거부하고 NVS의 이전 값은 건드리지 않습니다 |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

protected API: 생성자 `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (`true`이면 보정값을 NVS에서 불러옵니다).

기울기 보정이 없는 방위: 기체가 거의 수평인 동안에는 정확합니다. 기수가
북쪽이면 자기장은 +X 방향 → 0°, 기수가 동쪽이면 → 90°입니다.

---

## `QMC5883P_Sensor`

**파일:** `sensors/mag/QMC5883P_Sensor.h` · **상속:** `MagnetometerBase` · **NVS:** `qmc5883p` · **상태:** 벤치에서 확인

`DEFAULT_ADDRESS = 0x2C`. `begin()`: 칩 ID `0x80` (레지스터 0x00), 소프트웨어
리셋, 축 부호 `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (노멀, 200 Hz, OSR 8/8). 데이터는 `0x01`부터 6바이트,
리틀 엔디언입니다. 37.5 LSB/µT.

---

## `QMC5883L_Sensor`

**파일:** `sensors/mag/QMC5883L_Sensor.h` · **상속:** `MagnetometerBase` · **NVS:** `qmc5883l` · **상태:** 하드웨어에서는 확인하지 않음

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (이 칩에는 믿을 만한 ID가 없습니다),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512).
데이터는 `0x00`부터 6바이트, 리틀 엔디언입니다. 30 LSB/µT. 레지스터는 QMC5883P와
**호환되지 않습니다**.

---

## `QMC6309_Sensor`

**파일:** `sensors/mag/QMC6309_Sensor.h` · **상속:** `MagnetometerBase` · **NVS:** `qmc6309` · **상태:** 하드웨어에서는 확인하지 않음

QMC6309 (QST), I2C 0x7C — 일반적인 범위 0x08..0x77을 벗어난 주소입니다
(콘솔의 `b`로 하는 버스 스캔은 0x7F까지 갑니다).

- `begin()`: `CHIP_ID` (0x00) = 0x90, 리셋 (CTRL2를 0x80 → 0x00), `NVM_RDY | NVM_LOAD_DONE`
  대기, ±8 G, 200 Hz, set/reset, LPF 16, OSR 8, 노멀.
- `readRaw()`: 0x01부터 X/Y/Z를 리틀 엔디언으로. 40.96 LSB/µT.

---

## `UbloxM10_Gps`

**파일:** `sensors/gps/UbloxM10_Gps.h` · **상속:** `GpsSensor` · **의존:** `IUartPort`, `Config` · **상태:** 벤치에는 연결하지 않음

UART로 연결하는 u-blox M10이며 프로토콜은 UBX입니다. 해석하는 것은 **NAV-PVT** (클래스 0x01,
id 0x07, 92바이트)뿐입니다.

| 메서드 | 설명 |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 보 → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 보 → CFG-VALSET: 10 Hz, UART1에 NAV-PVT, UBX 켬, NMEA 끔. TX 핀이 없으면 (`PIN_GPS_TX < 0`) 9600에서 듣기만 합니다. 항상 `true` (ACK가 없음) |
| `bool isAvailable() const` | 유효한 NAV-PVT가 있었고 마지막 것이 `GPS_TIMEOUT_US`보다 오래되지 않음 |
| `void update()` | UART에 있는 모든 것을 파서에 넘깁니다 |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = 사용 가능하고 `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()`는 `isAvailable()`에 따라 `available`을 표시합니다 (타임아웃 반영) |

파서는 바이트 단위 상태 기계 `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`이며, class+id+len+payload에 대해 Fletcher-8 체크섬을 계산합니다.
길이가 512를 넘으면 동기가 깨진 것으로 보고 탐색을 처음부터 다시 합니다. NAV-PVT의 필드는 다음과 같이 해석합니다: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, 0..360으로 정규화).

내부의 `ValsetBuilder`는 UBX-CFG-VALSET의 페이로드입니다 (version 0, layer RAM,
U4 LE 키와 1/2/4바이트 LE 값의 쌍, 64바이트 버퍼).
