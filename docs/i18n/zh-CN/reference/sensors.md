# SENSORS — 传感器

> 🌐 本页是[俄语原文](../../../reference/sensors.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

传感器分为三个层次：

1. **类别接口**（`SensorInterface.h`）——`Autopilot`、`ArmingManager` 和遥测所看到的内容。
2. **类别基类**（`ImuSensorBase`、`BarometerBase`、
   `MagnetometerBase`）——所有共性的部分：校准、滤波器、坐标轴旋转、符号、错误计数、NVS 存储。采用模板方法（Template Method）模式。
3. **芯片驱动**——只包含数据手册中的寄存器和公式。它们接收
   `IRegisterDevice&`（不区分总线）或 `IUartPort&`。

具体编译哪一款芯片，由 `SensorSelection.h` 决定。

---

## 接口与数据结构

**文件：** `sensors/SensorInterface.h`

### 结构体

| 结构体 | 字段 |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s（滚转 + 右翼下沉，俯仰 + 机头抬起，偏航 + 机头向右）；`accelX/Y/Z` g（机体坐标轴：X 指向机头，Y 向左，Z 向上）；`roll`（−180..180）、`pitch`（−90..90）、`yaw`（−180..180，陀螺仪积分）°；`temperature` °C；`timestamp` µs |
| `BarometerData` | `pressure` Pa；`temperature` °C；`altitude` m，**相对于校准点**；`verticalSpeed` m/s；`timestamp` µs |
| `MagData` | `magX/Y/Z` µT，经过 hard-iron 校准，在机体坐标轴下；`headingDegrees` 0..360（未做倾斜补偿）；`timestamp` |
| `GpsData` | `latitude`、`longitude`（double，°）；`altitude` m MSL；`groundSpeed` m/s；`heading` 0..360；`numSatellites`；`fixType`（0 无，2 — 2D，3 — 3D）；`horizontalAccuracy`、`verticalAccuracy` m；`timestamp` |

### `Sensor`（接口）

| 方法 | 说明 |
|---|---|
| `bool begin()` | 识别并配置芯片；`true` 表示传感器正常工作 |
| `bool isAvailable() const` | 已连接且**此刻**有应答 |
| `void update()` | 每个节拍调用；由它自己判断是否该读取了 |
| `const char* getSensorType() const` | 用于日志的名称 |
| `void printStatus() const` | 一行诊断信息（控制台命令 `s`） |

### `ImuSensor : Sensor`

| 方法 | 说明 |
|---|---|
| `const ImuData& getImuData() const` | 最新数据 |
| `void calibrate()` | 陀螺仪校准（保持静止）+ 飞行前检查 |
| `void setYaw(float)` | 设定航向（例如启动时根据指南针设定） |
| `virtual void calibrateOrientation()` | 电路板安装方向校准（默认不支持） |
| `virtual const char* getPreflightProblem() const` | 飞行前检查发现的问题，或 `nullptr`（默认为 `nullptr`） |

### `BarometerSensor : Sensor`

`getBarometerData()`、`calibrateAltitude()`（把当前高度设为 0）、
`setSeaLevelPressure(Pa)`。

### `MagnetometerSensor : Sensor`

`getMagData()`、`calibrate()`（hard-iron：旋转 15 s）。

### `GpsSensor : Sensor`

`getGpsData()`、`hasFix()`（至少有 2D 定位）。

---

## `AirspeedSensor`

**文件：** `sensors/airspeed/AirspeedSensor.h` · **类别：** 接口 · **实现：** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`：调零和滤波之后的压差、指示空速（ρ0 = 1.225）、真空速（ρ
由静压和温度求得）、空气密度。方法：`getAirspeedData()`、
`calibrateZero()`（重新开始调零）、`isZeroing()`。

## `PitotDualBaroAirspeed`

**文件：** `sensors/airspeed/PitotDualBaroAirspeed.h` · **继承：** `AirspeedSensor` · **状态：** 已在带噪声的闭环仿真中验证，尚未试飞

用**两个绝对气压计**自制的皮托管：`total` 是管内的
BMP581（全压），`stat` 是机身的主气压计（静压）。制作指南见 [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#自制皮托管)。

| 方法 | 说明 |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | 皮托管气压计的 `begin()`（静压气压计已经启动），开始调零 |
| `update()` | 皮托管的新采样 → 压差 − 零点，低通滤波 `PITOT_FILTER_TAU_S`；前 `PITOT_ZERO_SAMPLES` 个采样用于平均出零点；速度为 `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | 两个气压计都正常，零点已采集完，没有故障，采样比 `PITOT_STALE_US` 更新 |
| `hasFault()` | 压差低于 −`PITOT_NEGATIVE_FAULT_PA` 的时间超过 `PITOT_NEGATIVE_FAULT_MS`（软管、进水） |
| `getZeroOffset()`、`printStatus()` | 诊断 |
| `static speedFrom(Δp, ρ)`、`static densityOf(p, T)` | 公式 |

---

## namespace `SensorMounting`

**文件：** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)`——
把芯片坐标轴绕垂直方向（芯片朝上）旋转到机体坐标轴（X 指向机头，
Y 向左）。`rotationCwDeg` 是芯片 X 轴的指向，从上方看顺时针计：

| 值 | bodyX | bodyY |
|---|---|---|
| 0（以及任何未知值） | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

由指南针（`MAG_ROTATION_CW_DEG`）和未做安装校准的 IMU 使用。

---

## `SensorSelection.h`

**文件：** `sensors/SensorSelection.h` · **类别：** 预处理器配置

更换物理传感器的唯一位置：用一行（`SENSOR_KIT`）指定现成的套装，或者分别指定每个传感器。任何选择都可以用构建标志覆盖（`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`、
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`）。

| `SENSOR_KIT` 套装 | IMU | 气压计 | 指南针 | 空速 | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521`（1，默认） | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT`（2） | LSM6DSV | SPL06（机身） | QMC6309 | 管内的 BMP581 | M10 |
| `SENSOR_KIT_ICM45686_PITOT`（3） | ICM-45686 | SPL06（机身） | QMC6309 | 管内的 BMP581 | M10 |
| `SENSOR_KIT_CUSTOM`（0） | 设定下面全部五个宏 | | | | |

| 选择宏 | 可选项 |
|---|---|
| `SENSOR_IMU` | `MPU6050`（1）、`ICM42688`（2，SPI）、`LSM6DSV`（3）、`LSM6DSV_SPI`（4）、`ICM45686`（5）、`ICM45686_SPI`（6） |
| `SENSOR_BARO` | `BME280`（1）、`BMP388`（2，SPI）、`BMP388_I2C`（3）、`SPL06`（4）、`SPL06_SPI`（5）、`BMP581`（6）、`BMP581_SPI`（7） |
| `SENSOR_MAG` | `NONE`（0）、`QMC5883P`（1）、`QMC5883L`（2）、`QMC6309`（3） |
| `SENSOR_AIRSPEED` | `NONE`（0）、`PITOT_BMP581`（1）——管内的 BMP581，I2C 0x47 |
| `SENSOR_GPS` | `NONE`（0）、`UBLOX_M10`（1） |

所有组合都能在所有电路板上构建——见 `tools/build_matrix.sh`。

其结果是类型别名和设备工厂：

| 名称 | MPU6050 / ICM42688 等 |
|---|---|
| `SelectedImu`、`SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`、`SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI（CS 为 `PIN_SPI_CS_BARO`）/ `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`、`SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C（NONE 时未定义） |
| 新的 IMU | `LSM6DSV_Sensor` + I2C 0x6A（备用 0x6B）或 SPI；`ICM45686_Sensor` + I2C 0x68（0x69）或 SPI |
| 新的气压计 | `SPL06_Sensor` + I2C 0x76（0x77）或 SPI；`BMP581_Sensor` + I2C 0x46（0x47）或 SPI |
| `SelectedPitotBaro`、`SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47（NONE 时未定义） |
| `SelectedGps` | `UbloxM10_Gps`（NONE 时未定义） |

`main.cpp` 把指南针、GPS 和皮托管的创建包在 `#if SENSOR_* != SENSOR_*_NONE` 之中。

---

## `ImuOrientation`

**文件：** `sensors/imu/ImuOrientation.h` · **依赖：** `SensorMounting`、`Preferences`（NVS）

从芯片坐标轴到机体坐标轴的旋转矩阵 `R`：`body = R · chip`；
`R` 的各行是芯片坐标轴下的机体坐标轴。

| 方法 | 说明 |
|---|---|
| `ImuOrientation()` | 单位矩阵（等价于 `fromYawSteps(0)`） |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | 绕垂直方向以 90° 为步长旋转，电路板芯片朝上 |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | 根据三个姿态校准（加速度计在芯片坐标轴下的“向上”读数）。`nullptr` 表示成功，否则为拒绝的原因 |
| `void apply(const float chip[3], float body[3]) const` | 应用旋转 |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | 测得的“向上”与机体 Z 轴之间的夹角（向量为零时为 180°） |
| `bool load(const char* ns)` | 从 NVS 加载；拒绝非正交归一或左手系的三元组 |
| `void save(const char* ns) const` | 保存到 NVS |
| `void describe(Print&) const` | “机头 = 芯片的 +Y，向上 = 芯片的 +Z”（如果某个轴与芯片轴的偏差超过 ±14°，则同时给出角度） |

`fromPoses` 算法：Z = norm(level)；X₁ = noseUp 中 ⟂ Z 的部分；Y = 
rightWingDown 中 ⟂ Z 的部分，X₂ = Y × Z；X = norm(X₁ + X₂)，Y = Z × X。拒绝的情形：

| 条件 | 消息 |
|---|---|
| 零向量 | “没有加速度计读数” |
| 第 2 步或第 3 步的倾斜角在 20..80° 之外 | “第 N 步需要 30-60° 的倾斜” |
| cos(X₁, X₂) < −0.5 | “第 2 步和第 3 步互相矛盾……”（机头压低了，或者抬错了机翼） |
| cos(X₁, X₂) < 0.9（≈25°） | “……倾斜的坐标轴不对……” |

---

## `AttitudeEstimator`

**文件：** `sensors/imu/AttitudeEstimator.h`

滚转/俯仰的互补滤波器加上偏航积分，与具体芯片无关。

| 方法 | 说明 |
|---|---|
| `void reset()` | 下一次 `update()` 立刻从加速度计给出的角度开始 |
| `void setYaw(float deg)` | 设定航向（归一化到 ±180） |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | 加速度为机体坐标轴下的 g，角速度为 °/s |
| `getRoll()`、`getPitch()`、`getYaw()` | ° |

`roll_acc = atan2(ay, az)`，`pitch_acc = atan2(ax, √(ay² + az²))`；
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc`（2 ms 步长下 τ ≈ 0.1 s）。
`reset()` 之后的第一次调用直接给出加速度计的角度；`dt ≤ 0` 或大于 0.1 s 时跳过该步（暂停、总线卡死）。

---

## `ImuSensorBase`

**文件：** `sensors/imu/ImuSensorBase.h` · **继承：** `ImuSensor` · **类别：** 抽象类

`RawImuSample`——芯片坐标轴下、以 ADC 单位表示的一个采样：
`accelX/Y/Z`、`gyroX/Y/Z`、`temperature`（`int16_t`）。

`update()` → `process()` 流水线：

```
原始采样 − 偏移（ADC）→ 缩放（g、°/s）→ ImuOrientation（机体坐标轴）
→ 航空符号（gyroY、gyroZ 变号）→ AttitudeEstimator
```

| 方法 | 说明 |
|---|---|
| `bool isAvailable() const` | `begin()` 成功，且连续读取错误少于 50 次 |
| `void update()` | 读取一次；出错时数据不变，计数器增加 |
| `void calibrate()` | 200 个采样 × 10 ms：陀螺仪偏移、噪声、“向上”方向；没有安装校准时，地平线 = 当前位置。然后做飞行前检查 |
| `void calibrateOrientation()` | 三个姿态（`capturePose`：静止约 1 s，姿态与之前的相差 ≥ 20°，超时 30 s）、`ImuOrientation::fromPoses`，并保存到 NVS。清除飞行前检查中与安装相关的问题 |
| `const char* getPreflightProblem() const` | 问题的文字，或 `nullptr` |
| `void setYaw(float)`、`getSensorType()`、`printStatus()` | |

供驱动使用的受保护 API：

| 方法 | 说明 |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | 每个驱动有自己的 NVS 命名空间 |
| `virtual bool readSample(RawImuSample&) = 0` | 一个采样；`false` 表示芯片没有应答 |
| `virtual float accelLsbPerG() const = 0`、`gyroLsbPerDps() const = 0` | 缩放系数 |
| `virtual float temperatureC(int16_t raw) const = 0` | 温度公式 |
| `void setAvailable(bool)` | `begin()` 的结果；为 `true` 时从 NVS 或 `Config::IMU_ROTATION_CW_DEG` 加载安装方向 |
| `void setName(const char*)` | 识别之后更精确的名称 |

飞行前检查（`runPreflightCheck`），按顺序：

| 问题 | 条件 |
|---|---|
| `NotResponding` | 读到的校准采样不足一半 |
| `Moved` | 陀螺仪噪声 > 0.5 °/s |
| `NotOneG` | \|a\| 与 1g 相差超过 0.2g |
| `MountingMismatch` | （已做安装校准）“向上”方向与保存的方向相差超过 45° |
| `NotChipUp` | （未校准）电路板没有芯片朝上放置（`z < 0.5g`） |

---

## `MPU6050_Sensor`

**文件：** `sensors/imu/MPU6050_Sensor.h` · **继承：** `ImuSensorBase` · **NVS：** `imu_mpu6050` · **状态：** 台架测试（MPU6500）

MPU6050 / MPU6500 / MPU9250 / MPU9255 及其仿制品（GY-521 板），I2C 0x68/0x69 或
SPI。芯片通过 `WHO_AM_I` 识别（0x68 为 MPU6050，否则为 6500 系列）。

- `begin()`：读取 WHO_AM_I（无应答 → 不可用），按 ID 给出名称，复位，退出睡眠（PLL），±2000 °/s，±16 g，DLPF 约 41 Hz，1 kHz；6500 另有独立的加速度计低通滤波器 `ACCEL_CONFIG2`。
- `readSample()`：从 `0x3B` 起读 14 字节，大端序：accel XYZ、temp、gyro XYZ。
- 缩放系数：2048 LSB/g，16.4 LSB/(°/s)。
- 温度：MPU6050 为 `raw/340 + 36.53`，MPU6500 为 `raw/333.87 + 21`。

---

## `ICM42688_Sensor`

**文件：** `sensors/imu/ICM42688_Sensor.h` · **继承：** `ImuSensorBase` · **NVS：** `imu_icm42688` · **状态：** 未在硬件上验证

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)`——SPI 8 MHz，没有虚拟字节。
- `begin()`：bank 0，软件复位，`WHO_AM_I == 0x47`，Low Noise，
  ±2000 °/s / ±16 g，1 kHz，UI 滤波器 50 Hz。
- `readSample()`：从 `0x1D` 起读 14 字节，大端序：temp、accel XYZ、gyro XYZ。
- 温度：`raw/132.48 + 25`。

---

## `LSM6DSV_Sensor`

**文件：** `sensors/imu/LSM6DSV_Sensor.h` · **继承：** `ImuSensorBase` · **NVS：** `imu_lsm6dsv` · **状态：** 未在硬件上验证

LSM6DSV / LSM6DSV16X / LSM6DSV32X（ST）。寄存器已对照 ST 的 `lsm6dsv-pid` 和 ArduPilot 核对过。

- `begin()`：`WHO_AM_I`（0x0F）= 0x70；`SW_RESET`（CTRL3 的第 0 位）并等待；
  32X 通过 CTRL8 中的型号位区分（它有自己的 ±16 g 代码）；BDU + 地址自增，
  ±2000 °/s 带 LPF1，±16 g 带 LPF2，960 Hz 高性能模式。
- `readSample()`：从 0x20 起读 14 字节，小端序：temp、gyro XYZ、accel XYZ。
- 缩放系数：1000/0.488 LSB/g，1000/70 LSB/(°/s)；温度 `raw/256 + 25`。
- `static spiDevice(bus, cs)`——SPI 模式 0，没有虚拟字节。

## `ICM45686_Sensor`

**文件：** `sensors/imu/ICM45686_Sensor.h` · **继承：** `ImuSensorBase` · **NVS：** `imu_icm45686` · **状态：** 未在硬件上验证

ICM-45686（TDK）。寄存器已对照 TDK 驱动、Zephyr 和 ArduPilot 核对过。

- `begin()`：通过 `REG_MISC2`（0x7F）复位，`WHO_AM_I`（0x72）= 0xE9；±2000 °/s 和 ±16 g，
  1.6 kHz（`GYRO/ACCEL_CONFIG0` = 0x15），Low Noise（`PWR_MGMT0` = 0x0F）；ODR/32
  低通滤波器——通过 0x7C..0x7E 窗口对**间接**寄存器 IPREG
  （0xA4AC、0xA583）做读-改-写；陀螺仪启动需 45 ms。
- `readSample()`：从 0x00 起读 14 字节，小端序：accel XYZ、gyro XYZ、temp。
- 缩放系数：2048 LSB/g，16.4 LSB/(°/s)；温度 `raw/132.48 + 25`。

---

## `BarometerBase`

**文件：** `sensors/baro/BarometerBase.h` · **继承：** `BarometerSensor` · **类别：** 抽象类

| 方法 | 说明 |
|---|---|
| `bool isAvailable() const` | `begin()` 成功，且连续错误少于 100 次 |
| `void update()` | 不会比 `pollPeriodUs` 更频繁：`isNewSampleReady()` → `readSample()` → 高度和垂直速度 |
| `void calibrateAltitude()` | 20 个采样 × 50 ms：平均绝对高度 = 基准；高度和速度归零 |
| `void setSeaLevelPressure(Pa)` | P₀（默认 101325） |
| `getBarometerData()`、`getSensorType()`、`printStatus()` | |

受保护 API：构造函数 `(name, pollPeriodUs)`、
`virtual bool isNewSampleReady(bool& ready) = 0`、
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`、
`setAvailable(bool)`。

公式：`h = 44330 · (1 − (P/P₀)^0.1903) − base`；垂直速度是高度对
**真实的新**采样求导，再经过 τ = 0.5 s 的低通滤波（`dt` 在 `(0, 0.5 s)` 之外时不更新速度）。只读取新的采样，可以消除“阶梯状”噪声（0 m/s 与 Δh/2 ms 的跳变交替出现）。

---

## `BMP388_Sensor`

**文件：** `sensors/baro/BMP388_Sensor.h` · **继承：** `BarometerBase` · **状态：** 台架测试（I2C）

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)`——SPI 8 MHz，
  **1 个虚拟字节**（数据手册 §5.3.2）。
- `begin()`：芯片 ID `0x50`，软件复位，从 `0x31` 起读 21 字节 NVM 系数（缩放见 §9.1），OSR ×8/×1，ODR 50 Hz，IIR 3，normal 模式。
- `isNewSampleReady()`：STATUS 寄存器的 `drdy_press` 标志（位 0x20）；每 5 ms
  轮询一次。
- `readSample()`：从 `0x04` 起读 6 字节；Bosch 补偿 §9.3（double）：先算温度（`tLin`），再算压力。

---

## `BME280_Sensor`

**文件：** `sensors/baro/BME280_Sensor.h` · **继承：** `BarometerBase` · **状态：** 未在硬件上验证

BME280（ID 0x60）和 BMP280（ID 0x58），I2C 0x76/0x77 或 SPI，没有虚拟字节。不读取湿度。

- `begin()`：芯片 ID，复位，从 `0x88` 起读 24 字节校准数据，`CTRL_HUM`（仅
  BME280，要在 `CTRL_MEAS` **之前**写入），`CONFIG` = IIR 4 + 0.5 ms，`CTRL_MEAS`
  = T×2、P×8、normal。
- 没有就绪标志——`ready = true`，每 25 ms 轮询一次。
- 补偿采用 Bosch §8.1 的公式（double）；防止除以零。

---

## `SPL06_Sensor`

**文件：** `sensors/baro/SPL06_Sensor.h` · **继承：** `BarometerBase` · **状态：** 未在硬件上验证

SPL06-001（Goertek）。公式出自数据手册 §4.9。

- `begin()`：`ID`（0x0D）= 0x10（0x11 是 SPA06，系数集不同——
  会被拒绝）；复位，等待 `COEF_RDY | SENSOR_RDY`；读取 18 字节系数（有符号的 12/20/16 位字段）；温度源由 `COEF_SRCE` 决定；压力 16×（32 Hz），温度 1×，连续模式。
- `isNewSampleReady()`——`PRS_RDY` 位；`readSample()`——24 位大端序采样，`kP = 253952`，`kT = 524288`。

## `BMP581_Sensor`

**文件：** `sensors/baro/BMP581_Sensor.h` · **继承：** `BarometerBase` · **状态：** 未在硬件上验证

BMP581（Bosch）。流程遵循官方的 BMP5_SensorAPI。

- `begin()`：一次虚拟读取（针对 SPI），`CHIP_ID`（0x01）= 0x50/0x51；软件复位，`INT_STATUS` 的 POR 和 `STATUS` 中 NVM 就绪且无错误；
  standby → OSR（压力 16×，温度 2×），IIR，DRDY；检查 ODR
  是否可行（`OSR_EFF`）；连续模式。
- 采样——在 DRDY 时读取，如果标志丢失则每 40 ms 读取一次；温度
  `int24/65536`，压力 `uint24/64`。
- 带皮托管的构建中有两个实例：主气压计和 `PITOT-BMP581`。

---

## `MagnetometerBase`

**文件：** `sensors/mag/MagnetometerBase.h` · **继承：** `MagnetometerSensor` · **类别：** 抽象类

| 方法 | 说明 |
|---|---|
| `bool isAvailable() const` | `begin()` 成功，且连续错误少于 25 次 |
| `void update()` | 50 Hz：`readRaw()` → 减去偏移 → 缩放 → 按 `MAG_ROTATION_CW_DEG` 旋转 → 航向 `atan2(Y, X)`，范围 0..360 |
| `void calibrate()` | 旋转 15 s：每个轴的偏移 = (min + max)/2（hard-iron），保存到 NVS。如果一次成功的读取都没有，校准会被拒绝，NVS 中原有的校准保持不变 |
| `getMagData()`、`getSensorType()`、`printStatus()` | |

受保护 API：构造函数 `(name, nvsNamespace)`、
`virtual bool readRaw(int16_t raw[3]) = 0`、`virtual float lsbPerMicroTesla() const = 0`、
`setAvailable(bool)`（为 `true` 时从 NVS 加载校准）。

未做倾斜补偿的航向：只要飞机基本保持水平就是准确的。机头朝北 → 磁场沿 +X → 0°；机头朝东 → 90°。

---

## `QMC5883P_Sensor`

**文件：** `sensors/mag/QMC5883P_Sensor.h` · **继承：** `MagnetometerBase` · **NVS：** `qmc5883p` · **状态：** 台架测试

`DEFAULT_ADDRESS = 0x2C`。`begin()`：芯片 ID `0x80`（寄存器 0x00），软件复位，坐标轴符号 `0x29 = 0x06`，`CONTROL2 = 0x08`（SET/RESET，±8 G），
`CONTROL1 = 0xCD`（normal，200 Hz，OSR 8/8）。数据为从 `0x01` 起的 6 字节，小端序。37.5 LSB/µT。

---

## `QMC5883L_Sensor`

**文件：** `sensors/mag/QMC5883L_Sensor.h` · **继承：** `MagnetometerBase` · **NVS：** `qmc5883l` · **状态：** 未在硬件上验证

`DEFAULT_ADDRESS = 0x0D`。`begin()`：`probe()`（该芯片没有可靠的 ID），
`SET/RESET = 0x01`，`CONTROL1 = 0x1D`（continuous，200 Hz，±8 G，OSR 512）。数据为从 `0x00` 起的 6 字节，小端序。30 LSB/µT。寄存器与
QMC5883P **不兼容**。

---

## `QMC6309_Sensor`

**文件：** `sensors/mag/QMC6309_Sensor.h` · **继承：** `MagnetometerBase` · **NVS：** `qmc6309` · **状态：** 未在硬件上验证

QMC6309（QST），I2C 0x7C——这个地址超出了通常的 0x08..0x77 范围（控制台命令 `b` 的总线扫描会一直扫到 0x7F）。

- `begin()`：`CHIP_ID`（0x00）= 0x90；复位（CTRL2 0x80 → 0x00），等待
  `NVM_RDY | NVM_LOAD_DONE`；±8 G，200 Hz，set/reset；LPF 16，OSR 8，normal。
- `readRaw()`：从 0x01 起 X/Y/Z 小端序；40.96 LSB/µT。

---

## `UbloxM10_Gps`

**文件：** `sensors/gps/UbloxM10_Gps.h` · **继承：** `GpsSensor` · **依赖：** `IUartPort`、`Config` · **状态：** 台架上未连接

通过 UART 连接的 u-blox M10，使用 UBX 协议。只解析 **NAV-PVT**（class 0x01，
id 0x07，92 字节）。

| 方法 | 说明 |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 波特 → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 波特 → CFG-VALSET：10 Hz，UART1 上输出 NAV-PVT，开启 UBX，关闭 NMEA。没有 TX 引脚（`PIN_GPS_TX < 0`）时只在 9600 下监听。始终返回 `true`（没有 ACK） |
| `bool isAvailable() const` | 收到过有效的 NAV-PVT，且最后一帧不早于 `GPS_TIMEOUT_US` |
| `void update()` | 把 UART 中的全部数据喂给解析器 |
| `const GpsData& getGpsData() const`、`bool hasFix() const` | `hasFix` = 可用且 `fixType ≥ 2` |
| `getSensorType()`、`printStatus()` | `printStatus()` 根据 `isAvailable()` 显示 `available`（考虑了超时） |

解析器是逐字节的状态机 `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`，对 class+id+len+payload 计算 Fletcher-8 校验和。长度 > 512 视为同步丢失，重新搜索。NAV-PVT 的字段解析为：`fixType`
（20）、`numSV`（23）、`lon`/`lat`（24/28，×1e−7）、`hMSL`（36，mm）、`hAcc`/`vAcc`
（40/44，mm）、`gSpeed`（60，mm/s）、`headMot`（64，×1e−5 °，归一化到 0..360）。

嵌套的 `ValsetBuilder` 是 UBX-CFG-VALSET 的负载（version 0，layer RAM，由
U4 LE 键与 1/2/4 字节 LE 值组成的键值对，64 字节缓冲区）。
