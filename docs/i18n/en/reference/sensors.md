# SENSORS — sensors

> 🌐 This page is a translation of the [Russian original](../../../reference/sensors.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Reference](README.md)

The sensors are built in three levels:

1. **Category interfaces** (`SensorInterface.h`) — what `Autopilot`,
   `ArmingManager` and the telemetry see.
2. **Category base classes** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — everything common: calibrations, filters, axis rotation, signs,
   error counting, storage in NVS. The Template Method pattern.
3. **Chip drivers** — only the registers and formulas from the datasheet. They receive
   `IRegisterDevice&` (they do not tell the buses apart) or `IUartPort&`.

Which chip is compiled in is decided by `SensorSelection.h`.

---

## Interfaces and data structures

**File:** `sensors/SensorInterface.h`

### Structures

| Structure | Fields |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (roll + right wing down, pitch + nose up, yaw + nose right); `accelX/Y/Z` g (airframe axes: X toward the nose, Y to the left, Z up); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, the gyro integral) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m **relative to the calibration point**; `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | `magX/Y/Z` µT after the hard-iron calibration, in airframe axes; `headingDegrees` 0..360 (without tilt compensation); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 none, 2 — 2D, 3 — 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (interface)

| Method | Description |
|---|---|
| `bool begin()` | Identify and configure the chip; `true` — the sensor works |
| `bool isAvailable() const` | Connected and responding **right now** |
| `void update()` | Call every tick; it decides for itself whether it is time to read |
| `const char* getSensorType() const` | A name for the log |
| `void printStatus() const` | A diagnostics line (console `s`) |

### `ImuSensor : Sensor`

| Method | Description |
|---|---|
| `const ImuData& getImuData() const` | The latest data |
| `void calibrate()` | Gyroscope calibration (keep still) + the preflight check |
| `void setYaw(float)` | Set the heading (for example from the compass at start-up) |
| `virtual void calibrateOrientation()` | Board mounting calibration (not supported by default) |
| `virtual const char* getPreflightProblem() const` | The preflight-check problem or `nullptr` (`nullptr` by default) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (the current altitude = 0),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: rotate for 15 s).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (there is at least a 2D fix).

---

## `AirspeedSensor`

**File:** `sensors/airspeed/AirspeedSensor.h` · **Kind:** interface · **Implementation:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
the differential pressure after zeroing and filtering, the indicated airspeed (ρ0 = 1.225), the true
airspeed (ρ from static pressure and temperature), the density. Methods: `getAirspeedData()`,
`calibrateZero()` (restart the zeroing), `isZeroing()`.

## `PitotDualBaroAirspeed`

**File:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Inherits:** `AirspeedSensor` · **Status:** verified in a closed-loop simulation with noise, not flight-tested

A home-made pitot tube on **two absolute barometers**: `total` — the
BMP581 inside the tube (total pressure), `stat` — the fuselage's main barometer
(static pressure). The build guide is in [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#a-diy-pitot-tube).

| Method | Description |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` of the tube barometer (the static one is already running), start the zeroing |
| `update()` | a new tube sample → differential − zero, low-pass filter `PITOT_FILTER_TAU_S`; the first `PITOT_ZERO_SAMPLES` samples average the zero; the speed is `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | both barometers are alive, the zero is collected, there is no fault, the sample is fresher than `PITOT_STALE_US` |
| `hasFault()` | the differential stays below −`PITOT_NEGATIVE_FAULT_PA` for longer than `PITOT_NEGATIVE_FAULT_MS` (hoses, water) |
| `getZeroOffset()`, `printStatus()` | diagnostics |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | formulas |

---

## namespace `SensorMounting`

**File:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
rotates the chip axes about the vertical (chip up) into the airframe axes (X toward the nose,
Y to the left). `rotationCwDeg` is where the chip's X axis points, clockwise seen from above:

| Value | bodyX | bodyY |
|---|---|---|
| 0 (and any unknown value) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Used by the compass (`MAG_ROTATION_CW_DEG`) and by an IMU without mounting calibration.

---

## `SensorSelection.h`

**File:** `sensors/SensorSelection.h` · **Kind:** preprocessor configuration

The single place for changing a physical sensor: a ready-made kit in one line
(`SENSOR_KIT`) or each sensor separately. Any choice can be overridden
with a build flag (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| `SENSOR_KIT` kit | IMU | Barometer | Compass | Airspeed | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, the default) | MPU6500 | BMP581 I2C (0x46/0x47) | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (fuselage) | QMC6309 | BMP581 in the tube | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (fuselage) | QMC6309 | BMP581 in the tube | M10 |
| `SENSOR_KIT_CUSTOM` (0) | set all five macros below | | | | |

| Selection macro | Options |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — a BMP581 I2C 0x47 in the tube |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

All combinations build on all boards — `tools/build_matrix.sh`.

The result is type aliases and device factories:

| Name | MPU6050 / ICM42688 etc. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (not defined for NONE) |
| the new IMUs | `LSM6DSV_Sensor` + I2C 0x6A (spare 0x6B) or SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) or SPI |
| the new barometers | `SPL06_Sensor` + I2C 0x76 (0x77) or SPI; `BMP581_Sensor` + I2C 0x46 (0x47) or SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (not defined for NONE) |
| `SelectedGps` | `UbloxM10_Gps` (not defined for NONE) |

`main.cpp` wraps the creation of the compass, the GPS and the tube in `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**File:** `sensors/imu/ImuOrientation.h` · **Depends on:** `SensorMounting`, `Preferences` (NVS)

The rotation matrix `R` from the chip axes to the airframe axes: `body = R · chip`; the rows of
`R` are the airframe axes in the chip axes.

| Method | Description |
|---|---|
| `ImuOrientation()` | The identity (equivalent to `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | A rotation about the vertical in steps of 90°, the board chip-up |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Calibration from three poses (the accelerometer's "up" reading in the chip axes). `nullptr` — success, otherwise the reason for the refusal |
| `void apply(const float chip[3], float body[3]) const` | Apply the rotation |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | The angle between the measured "up" and the airframe Z axis (180° if the vector is zero) |
| `bool load(const char* ns)` | Load from NVS; rejects a non-orthonormal or left-handed triple |
| `void save(const char* ns) const` | Save to NVS |
| `void describe(Print&) const` | "nose = +Y of the chip, up = +Z of the chip" (with an angle if an axis does not coincide with a chip axis to within ±14°) |

The `fromPoses` algorithm: Z = norm(level); X₁ = the part of noseUp ⟂ Z; Y = the part of
rightWingDown ⟂ Z, X₂ = Y × Z; X = norm(X₁ + X₂), Y = Z × X. The refusals:

| Condition | Message |
|---|---|
| zero vector | "no accelerometer readings" |
| the tilt in step 2 or 3 is outside 20..80° | "step N needs a tilt of 30-60°" |
| cos(X₁, X₂) < −0.5 | "steps 2 and 3 contradict each other…" (the nose was lowered or the wrong wing was used) |
| cos(X₁, X₂) < 0.9 (≈25°) | "…the wrong axes were tilted…" |

---

## `AttitudeEstimator`

**File:** `sensors/imu/AttitudeEstimator.h`

A complementary filter for roll/pitch and a yaw integral, independent of the chip.

| Method | Description |
|---|---|
| `void reset()` | The next `update()` starts at once from the angle given by the accelerometer |
| `void setYaw(float deg)` | Set the heading (wrapped to ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | The acceleration in g in the airframe axes, the rates in °/s |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0.1 s at 2 ms).
The first call after `reset()` gives the accelerometer angles at once; `dt ≤ 0` or > 0.1 s —
the step is skipped (a pause, a hung bus).

---

## `ImuSensorBase`

**File:** `sensors/imu/ImuSensorBase.h` · **Inherits:** `ImuSensor` · **Kind:** abstract

`RawImuSample` — one sample in ADC units in the chip axes:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

The `update()` → `process()` pipeline:

```
raw samples − offsets (ADC) → scale (g, °/s) → ImuOrientation (airframe axes)
→ aviation signs (gyroY, gyroZ with the sign changed) → AttitudeEstimator
```

| Method | Description |
|---|---|
| `bool isAvailable() const` | `begin()` succeeded and fewer than 50 consecutive read errors |
| `void update()` | One read; on an error the data does not change and the counters grow |
| `void calibrate()` | 200 samples × 10 ms: the gyroscope offset, the noise, "up"; without mounting calibration — the horizon = the current position. Then the preflight check |
| `void calibrateOrientation()` | Three poses (`capturePose`: still for ~1 s, the pose differs from the previous ones by ≥ 20°, a 30 s timeout), `ImuOrientation::fromPoses`, saving to NVS. Clears the mounting problems of the preflight check |
| `const char* getPreflightProblem() const` | The text of the problem or `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

The protected API for drivers:

| Method | Description |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Each driver has its own NVS namespace |
| `virtual bool readSample(RawImuSample&) = 0` | One sample; `false` — the chip did not answer |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | The scales |
| `virtual float temperatureC(int16_t raw) const = 0` | The temperature formula |
| `void setAvailable(bool)` | The result of `begin()`; on `true` it loads the mounting from NVS or `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Refine the name after identification |

The preflight check (`runPreflightCheck`), in order:

| Problem | Condition |
|---|---|
| `NotResponding` | fewer than half of the calibration samples were read |
| `Moved` | the gyroscope noise > 0.5 °/s |
| `NotOneG` | \|a\| differs from 1g by more than 0.2g |
| `MountingMismatch` | (the mounting is calibrated) "up" is more than 45° away from the saved one |
| `NotChipUp` | (not calibrated) the board is not lying chip-up (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**File:** `sensors/imu/MPU6050_Sensor.h` · **Inherits:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **Status:** on the bench (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 and clones (GY-521 boards), I2C 0x68/0x69 or
SPI. The chip is identified by `WHO_AM_I` (0x68 — MPU6050, otherwise the 6500 family).

- `begin()`: WHO_AM_I (no answer → unavailable), the name by ID, reset, leave
  sleep (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz; the 6500 has a separate
  accelerometer low-pass filter `ACCEL_CONFIG2`.
- `readSample()`: 14 bytes from `0x3B`, big-endian: accel XYZ, temp, gyro XYZ.
- Scales: 2048 LSB/g, 16.4 LSB/(°/s).
- Temperature: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**File:** `sensors/imu/ICM42688_Sensor.h` · **Inherits:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **Status:** not verified on hardware

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz, without
  a dummy byte.
- `begin()`: bank 0, software reset, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, UI filter 50 Hz.
- `readSample()`: 14 bytes from `0x1D`, big-endian: temp, accel XYZ, gyro XYZ.
- Temperature: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**File:** `sensors/imu/LSM6DSV_Sensor.h` · **Inherits:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **Status:** not verified on hardware

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). The registers were checked against ST's `lsm6dsv-pid` and ArduPilot.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (CTRL3 bit 0) and a wait;
  the 32X is told apart by the variant bit in CTRL8 (it has its own ±16 g code); BDU + auto-increment,
  ±2000 °/s with LPF1, ±16 g with LPF2, 960 Hz high-performance.
- `readSample()`: 14 bytes from 0x20, little-endian: temp, gyro XYZ, accel XYZ.
- Scales: 1000/0.488 LSB/g, 1000/70 LSB/(°/s); temperature `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI mode 0, without a dummy byte.

## `ICM45686_Sensor`

**File:** `sensors/imu/ICM45686_Sensor.h` · **Inherits:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **Status:** not verified on hardware

ICM-45686 (TDK). The registers were checked against the TDK driver, Zephyr and ArduPilot.

- `begin()`: reset via `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9; ±2000 °/s and ±16 g
  at 1.6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); the ODR/32
  low-pass filter — a read-modify-write of the **indirect** IPREG registers
  (0xA4AC, 0xA583) through the window 0x7C..0x7E; 45 ms for the gyroscope to start.
- `readSample()`: 14 bytes from 0x00, little-endian: accel XYZ, gyro XYZ, temp.
- Scales: 2048 LSB/g, 16.4 LSB/(°/s); temperature `raw/132.48 + 25`.

---

## `BarometerBase`

**File:** `sensors/baro/BarometerBase.h` · **Inherits:** `BarometerSensor` · **Kind:** abstract

| Method | Description |
|---|---|
| `bool isAvailable() const` | `begin()` succeeded and fewer than 100 consecutive errors |
| `void update()` | No more often than `pollPeriodUs`: `isNewSampleReady()` → `readSample()` → altitude and vertical speed |
| `void calibrateAltitude()` | 20 samples × 50 ms: the mean absolute altitude = the base; resets the altitude and the speed |
| `void setSeaLevelPressure(Pa)` | P₀ (101325 by default) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

The protected API: the constructor `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Formulas: `h = 44330 · (1 − (P/P₀)^0.1903) − base`; the vertical speed is the
derivative of the altitude over the **real new** samples through a low-pass filter with τ = 0.5 s
(`dt` outside `(0, 0.5 s)` — the speed is not updated). Reading only new samples
removes the "stair-step" noise (0 m/s interleaved with Δh/2 ms jumps).

---

## `BMP388_Sensor`

**File:** `sensors/baro/BMP388_Sensor.h` · **Inherits:** `BarometerBase` · **Status:** on the bench (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz,
  **1 dummy byte** (datasheet §5.3.2).
- `begin()`: chip ID `0x50`, software reset, 21 bytes of NVM coefficients from
  `0x31` (scales §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, normal mode.
- `isNewSampleReady()`: the `drdy_press` flag (bit 0x20) of the STATUS register; polled
  every 5 ms.
- `readSample()`: 6 bytes from `0x04`; Bosch compensation §9.3 (double):
  the temperature first (`tLin`), then the pressure.

---

## `BME280_Sensor`

**File:** `sensors/baro/BME280_Sensor.h` · **Inherits:** `BarometerBase` · **Status:** not verified on hardware

BME280 (ID 0x60) and BMP280 (ID 0x58), I2C 0x76/0x77 or SPI without a dummy
byte. Humidity is not read.

- `begin()`: chip ID, reset, 24 bytes of calibration from `0x88`, `CTRL_HUM` (BME280
  only, written **before** `CTRL_MEAS`), `CONFIG` = IIR 4 + 0.5 ms, `CTRL_MEAS`
  = T×2, P×8, normal.
- There is no ready flag — `ready = true`, polled every 25 ms.
- Compensation — the Bosch §8.1 formulas (double); protection against division by zero.

---

## `SPL06_Sensor`

**File:** `sensors/baro/SPL06_Sensor.h` · **Inherits:** `BarometerBase` · **Status:** not verified on hardware

SPL06-001 (Goertek). The formulas are from datasheet §4.9.

- `begin()`: `ID` (0x0D) = 0x10 (0x11 is the SPA06, which has a different set of coefficients —
  it is rejected); reset, wait for `COEF_RDY | SENSOR_RDY`; 18 bytes of coefficients
  (signed 12/20/16-bit fields); the temperature source — by `COEF_SRCE`;
  pressure 16× (32 Hz), temperature 1×, continuous mode.
- `isNewSampleReady()` — the `PRS_RDY` bit; `readSample()` — 24-bit
  big-endian samples, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**File:** `sensors/baro/BMP581_Sensor.h` · **Inherits:** `BarometerBase` · **Status:** the main barometer of the default kit, not verified on hardware

BMP581 (Bosch). The sequence follows the official BMP5_SensorAPI.

- `begin()`: a dummy read (for SPI), `CHIP_ID` (0x01) = 0x50/0x51;
  software reset, `INT_STATUS` POR and `STATUS` NVM ready without errors;
  standby → OSR (pressure 16×, temperature 2×), IIR, DRDY; a check that
  the ODR is feasible (`OSR_EFF`); continuous mode.
- A sample — on DRDY, or every 40 ms if the flag is lost; temperature
  `int24/65536`, pressure `uint24/64`.
- Two instances in the build with the tube: the main barometer and `PITOT-BMP581`.

---

## `MagnetometerBase`

**File:** `sensors/mag/MagnetometerBase.h` · **Inherits:** `MagnetometerSensor` · **Kind:** abstract

| Method | Description |
|---|---|
| `bool isAvailable() const` | `begin()` succeeded and fewer than 25 consecutive errors |
| `void update()` | 50 Hz: `readRaw()` → subtract the offsets → scale → rotate by `MAG_ROTATION_CW_DEG` → heading `atan2(Y, X)` in 0..360 |
| `void calibrate()` | 15 s of rotation: offset = (min + max)/2 per axis (hard-iron), saved to NVS. If there was not a single successful read, the calibration is rejected and the previous one in NVS is left untouched |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

The protected API: the constructor `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (on `true` it loads the calibration from NVS).

The heading without tilt compensation: correct while the aircraft is nearly level. Nose to the
north → the field along +X → 0°; nose to the east → 90°.

---

## `QMC5883P_Sensor`

**File:** `sensors/mag/QMC5883P_Sensor.h` · **Inherits:** `MagnetometerBase` · **NVS:** `qmc5883p` · **Status:** on the bench

`DEFAULT_ADDRESS = 0x2C`. `begin()`: chip ID `0x80` (register 0x00), software
reset, axis signs `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8). The data — 6 bytes from `0x01`,
little-endian. 37.5 LSB/µT.

---

## `QMC5883L_Sensor`

**File:** `sensors/mag/QMC5883L_Sensor.h` · **Inherits:** `MagnetometerBase` · **NVS:** `qmc5883l` · **Status:** not verified on hardware

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (the chip has no reliable ID),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512).
The data — 6 bytes from `0x00`, little-endian. 30 LSB/µT. The registers are **not compatible**
with the QMC5883P.

---

## `QMC6309_Sensor`

**File:** `sensors/mag/QMC6309_Sensor.h` · **Inherits:** `MagnetometerBase` · **NVS:** `qmc6309` · **Status:** not verified on hardware

QMC6309 (QST), I2C 0x7C — an address outside the usual range 0x08..0x77
(the console's bus scan `b` goes up to 0x7F).

- `begin()`: `CHIP_ID` (0x00) = 0x90; reset (CTRL2 0x80 → 0x00), wait for
  `NVM_RDY | NVM_LOAD_DONE`; ±8 G, 200 Hz, set/reset; LPF 16, OSR 8, normal.
- `readRaw()`: X/Y/Z little-endian from 0x01; 40.96 LSB/µT.

---

## `UbloxM10_Gps`

**File:** `sensors/gps/UbloxM10_Gps.h` · **Inherits:** `GpsSensor` · **Depends on:** `IUartPort`, `Config` · **Status:** not connected on the bench

A u-blox M10 over UART, the UBX protocol. Only **NAV-PVT** is parsed (class 0x01,
id 0x07, 92 bytes).

| Method | Description |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 baud → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 baud → CFG-VALSET: 10 Hz, NAV-PVT on UART1, UBX on, NMEA off. Without a TX pin (`PIN_GPS_TX < 0`) it only listens at 9600. Always `true` (there is no ACK) |
| `bool isAvailable() const` | There was a valid NAV-PVT and the last one is not older than `GPS_TIMEOUT_US` |
| `void update()` | Feed everything from the UART to the parser |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = available and `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` shows `available` according to `isAvailable()` (taking the timeout into account) |

The parser is a byte-by-byte state machine `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, the Fletcher-8 checksum over class+id+len+payload.
A length > 512 is a loss of synchronization, and the search starts over. The NAV-PVT fields are parsed as: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, wrapped to 0..360).

The nested `ValsetBuilder` is the UBX-CFG-VALSET payload (version 0, layer RAM, pairs
of a U4 LE key — a 1/2/4-byte LE value, a 64-byte buffer).
