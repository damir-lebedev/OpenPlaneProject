# DEVELOPER_GUIDE.md——OpenPlaneProject 开发者指南

> 🌐 本页是[俄语原文](../../DEVELOPER_GUIDE.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

固件的技术地图：每个文件负责什么，数据如何从接收机和传感器流向舵机，哪些符号约定把整条链路连在一起，Web API 是怎样组织的，以及如何扩展项目。本文面向会写 C++、想在这个仓库（`main` 分支）里快速找到方向的开发者，而不是用来学习语言或 PlatformIO 的基础知识。

项目概览和原型状态见 [`../README.md`](README.md)，接线和飞行方法见 [`PILOT_GUIDE.md`](PILOT_GUIDE.md)，规划见 [`ROADMAP.md`](ROADMAP.md)。这里只讲代码。完整的架构（分层、任务、状态机）见 [`ARCHITECTURE.md`](ARCHITECTURE.md)，每个类的参考见 [`reference/`](reference/README.md)，测试见 [`TESTING.md`](TESTING.md)。

> 项目正在积极开发中。ESP32-S3 台架已搭建好，并连同全部传感器验证过，但**自动驾驶仪尚未在飞行中试过**——涉及具体模块的地方都有标注。如果你不确定某段代码做了什么，请重读源码，而不是这份文档。

---

## 目录

1. [分层架构](#分层架构)
2. [FreeRTOS 任务与控制循环](#freertos-任务与控制循环)
3. [文件参考](#文件参考)
4. [符号约定：从 IMU 到舵机](#符号约定从-imu-到舵机)
5. [RC 通道图、ARM 与 failsafe](#rc-通道图arm-与-failsafe)
6. [传感器数据](#传感器数据)
7. [FlightController::update() 详解](#flightcontrollerupdate-详解)
8. [网页仪表盘的 HTTP API](#网页仪表盘的-http-api)
9. [控制台与诊断](#控制台与诊断)
10. [开发板选择与引脚分配](#开发板选择与引脚分配)
11. [如何添加新传感器](#如何添加新传感器)
12. [如何添加新的自动驾驶仪模式](#如何添加新的自动驾驶仪模式)
13. [反馈（雏形，未接入）](#反馈雏形未接入)
14. [如何添加新开发板](#如何添加新开发板)
15. [构建、烧录与监视命令](#构建烧录与监视命令)
16. [已知局限](#已知局限)
17. [如何提交修改](#如何提交修改)

---

## 分层架构

几乎所有类都定义在按文件夹 `include/<层>/` 组织的头文件中。每个头文件自己包含它所用到的内容（`#include "config/Config.h"`、`"hal/II2CBus.h"` 等——路径从 `include/` 起算）。`src/main.cpp` 是唯一的装配点（composition root）：它创建所有对象、把它们连接起来，并运行 `setup()`/`loop()`。依赖是单向的——下层对上层一无所知。

```
include/
├── config/      Config.h (引脚、所有设置), Channels.h (通道名称),
│                Controls.h (每个拨杆的作用——每个通道一行)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (基于 I2C/SPI 的寄存器设备), Rtos
│   ├── esp32/   Esp32Board + 对 Wire/SPI/HardwareSerial/LEDC 的封装
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (设置存放在闪存而非 NVS)
├── storage/     KeyValueStore, KvPreferences — 不依赖 NVS 的设置存储
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  反馈回路的雏形——未接入（见下文）
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (由两个气压计组成的皮托管)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32 固件（S3、C3、38 针）
src/stm32/main.cpp  — STM32H743 固件（FreeRTOS 任务、MAVLink）
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — 对象的装配           │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — 每个周期内的操作顺序          │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 种模式  │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ 导航、PID          │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, 闪存     │
└───────────────────────────────────────────────────────────────────────┘
```

保持架构清晰的规则：

- **HAL** 是唯一允许了解具体 MCU 的层（`Wire`、`SPI`、`HardwareSerial`、`ledc*`）。上面的所有层都只和接口打交道。换成另一种 MCU，只需新增一个 `hal/<mcu>/<Mcu>Board.h`，其余代码不变（例子是用于 STM32H743 的 `hal/stm32/`）。
- **传感器驱动不了解总线。** 它们拿到 `IRegisterDevice&`——带地址的 I2C 或带 CS 的 SPI 在 `SensorSelection.h` 中创建。同一个 `BMP388_Sensor` 既能走 I2C 也能走 SPI。
- **共用的部分放在基类里。** 校准、轴旋转、符号、姿态滤波器、高度与垂直速度、罗盘校准的存储、总线错误计数——都在 `ImuSensorBase`/`BarometerBase`/`MagnetometerBase` 中。芯片驱动只包含寄存器和数据手册里的公式。
- **RC 和 Outputs** 对飞机一无所知：iBUS 字节 → 通道，PWM 值 → 输出。
- **Control 和 Autopilot** 是针对数据的逻辑，没有 UART、PWM 和 Wi-Fi。需要时间的地方（襟翼），时间以参数传入。
- **Coordination**（`FlightController`）是唯一能同时看到多个下层并决定操作顺序的类。
- **Application**（`main.cpp`）是唯一创建 `Esp32Board`、各设备和传感器，并且手工把一切连接起来的地方，不使用 DI 框架。

---

## FreeRTOS 任务与控制循环

| 位置 | 内容 | 周期 |
|---|---|---|
| 核心 1，`loop()`（Arduino loopTask） | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms（500 Hz），`vTaskDelayUntil` |
| 核心 0，`web` 任务 | `WebServer::handleClient()` | 每 2 ms |
| 核心 0，`oled` 任务 | 通过第二条 I2C 总线绘制 SSD1306 | 200 ms |
| 核心 0 | ESP-IDF 的 Wi-Fi 协议栈 | — |

- 循环周期由 `vTaskDelayUntil` 保持，而不是在工作之后 `delay(2)`——频率与一个周期持续了多久无关。在长时间阻塞（从控制台发起的校准）之后，计时会重新开始，错过的周期不会成批追赶。
- 在台架上（ESP32-S3，全部传感器）：500 Hz，每个周期平均约 0.7 ms 的工作量，最差的周期约 1.4 ms。每 10 s 会以 `SYS:` 行的形式打印一次。
- I2C 事务的超时为 5 ms（Wire 默认是 50 ms）：被干扰卡住的事务不会让循环长时间停下来。
- **任务之间的数据隔离。** 网页和 OLED 只*读取*状态（`FlightController`/`Autopilot`/`LoopStats`）——它们都是独立的 16/32 位字段，最坏的情况下看到的是相邻周期的数值。仪表盘的*命令*（`setmode`/`setpid`）不会在 `web` 任务中直接执行：它们在 `portMUX` 保护下放入“邮箱”，由飞行循环在 `WebDebugServer::applyPendingCommands()` 中取走。
- `Serial`（UART0 → CH343 桥接芯片 → “COM”接口）带有 4 KB 的发送缓冲：一帧调试输出（约 600 个字符）在发送期间不会阻塞循环。

---

## 文件参考

### `config/`

| 文件 | 负责内容 |
|---|---|
| `Config.h` | 所有引脚（每块开发板一个块：`BOARD_ESP32_S3/C3/CLASSIC`、`BOARD_STM32H743`）和设置：iBUS 与信号丢失；舵面行程；襟翼；舵机反向；IMU 与电子罗盘的安装；ARM；failsafe（RTH 或滑翔）；皮托管（`PITOT_*`）；自动驾驶仪各模式和功能的所有数值；循环；Wi-Fi；MAVLink；调试 |
| `Channels.h` | 通道名称：`AILERON`、`ELEVATOR`、`THROTTLE`、`RUDDER`、`ARM`、`SWB`、`SWC`、`SWD`、`VRA`、`VRB` |
| `Controls.h` | `BINDINGS` 表：每个拨杆和旋钮的作用，每个通道一行，带 `static_assert` 检查 |

### `hal/`

| 文件 | 负责内容 |
|---|---|
| `IBoard.h` | 通向硬件的入口：`i2c()`、`displayI2c()`（给屏幕用的第二条总线，可以是 `nullptr`）、`spi()`、`rcUart()`、`gpsUart()`、`telemetryUart()`（MAVLink，可以是 `nullptr`）、`servo(ServoChannel::*)`（7 路输出，含 AUX1/AUX2）、`setBuzzer()` |
| `Rtos.h` | FreeRTOS 任务，在 ESP32（核心 0）和 STM32（优先级）上用法一致，以及空闲堆 |
| `II2CBus.h` | I2C 总线：与 `Wire` 形式一致的基本操作 + 辅助函数 `writeRegister()`、`readRegisters()`（检查恰好收到了 `count` 个字节）、`readRegister()`、`probe()` |
| `ISpiBus.h`、`IUartPort.h`、`IServoOutput.h` | SPI、UART、单路 PWM 输出（`measurePulseUs()`——诊断实际脉冲） |
| `RegisterDevice.h` | `IRegisterDevice`——“一组 8 位寄存器”；`I2cRegisterDevice`（地址）、`SpiRegisterDevice`（CS、频率、数据前的哑字节） |
| `esp32/Esp32Board.h` | `IBoard` 的实现：`Wire`（传感器）、`Wire1`（屏幕，如果芯片有两个 I2C 控制器）、`SPI`、两个 `HardwareSerial`、5 个 LEDC 通道 |
| `esp32/Esp32I2CBus.h` | 基于任意 `TwoWire` 的 `II2CBus`，超时 5 ms |
| `esp32/Esp32ServoOutput.h` | 通过 LEDC 输出 PWM：50 Hz，14 位；引脚 −1——该输出没有引出。不使用 ESP32Servo 库——见[局限](#已知局限) |
| `esp32/Esp32SpiBus.h`、`esp32/Esp32UartPort.h` | 对 `SPI` 和 `HardwareSerial` 的薄封装 |
| `stm32/*` | STM32H743：`Stm32Board`（+ 数传电台的 UART4）、各总线、PWM 定时器、`Stm32FlashStorage`（设置存于闪存扇区，由后台任务写入）、`compat/Preferences.h` |

### `storage/`

| 文件 | 负责内容 |
|---|---|
| `KeyValueStore.h` | 位于 RAM 中、带 CRC32 的“命名空间/键 → 字节”映像，可建立在任意存储介质（`IFlashStorage`）之上；相同的值不会重复写入 |
| `KvPreferences.h` | 基于 `KeyValueStore` 的 ESP32 `Preferences` API |

### `rc/`

| 文件 | 负责内容 |
|---|---|
| `RcChannelState.h` | 10 个通道的快照 |
| `RcInput.h` | `clamp()`、`centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → 通道：32 字节的帧、CRC，通道值取低 12 位（`& 0x0FFF`）；`isSignalLost()` = 没有帧（或者还一帧都没收到）∥ 油门的 failsafe 值；帧计数器 |

### `control/`

| 文件 | 负责内容 |
|---|---|
| `ControlCommand.h` | 以物理符号表示的舵面指令——摇杆、自动驾驶仪和混控器共用的“语言” |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`；`updateFlaps(目标, now)`；`mix(command)` → 带舵机反向的 PWM；襟副翼：副翼为 `flaps ± roll`（减号表示减速板） |
| `FlapsController.h` | 襟翼平滑放出/收起，时间以参数传入 |
| `ThrottleManager.h` | 来自摇杆的油门；信号丢失时为 `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | 用 SwA 拨杆 ARM（油门在底时 OFF→ON 的切换 + 该模式所需的传感器检查），DISARM 立即生效 |
| `FlightOutputState.h` | 期望的 PWM：`aileronLeft`、`aileronRight`、`elevator`、`rudder`、`throttle`、`aux1`（载荷）、`aux2`（相机） |
| `Beeper.h` | 蜂鸣器：由 `BEEPER` 功能触发，或在地面时“模型丢失” |
| `FlightOutputs.h` | 输出表（`outputInfo()`：键、名称、引脚、是否必需、状态字段），以及在它之上循环完成的所有工作：`begin()`、`write()`、`setFailsafe()`、状态、`printPulseSelfTest()` |
| `FlightController.h` | 每个周期内的操作顺序、信号丢失（`applyLinkLoss()`）、供遥测使用的取值函数 |

### `autopilot/`

| 文件 | 负责内容 |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode`（12 种模式）、`Feature`、`Knob`、`PilotInputs`、名称 |
| `ControlBinding.h` | `Binding`、工厂函数 `Bind::modes/mode/feature/knob`、`BindingCheck` 检查 |
| `PilotSwitches.h` | 绑定表 → 每个周期的模式、功能和旋钮；开机时的布局 |
| `Autopilot.h` | 12 种模式、failsafe RTH/滑翔、地理围栏、返航点、转弯协调、自动配平；`update(armed, linkLost, 油门, 摇杆)` → `getCommand()`、`applyThrottle()` |
| `Navigation.h` | `Geo`（距离、方位、偏移）、`Guidance`（按航向给出横滚、圆周的向量场） |
| `AltitudeSpeedController.h` | 用俯仰控制高度，用油门控制空速（TECS-lite） |
| `LaunchController.h`、`SoaringController.h` | 手抛起飞和翱翔的状态机 |
| `AutoTrim.h` | 自动配平，存于 NVS/闪存 |
| `PidController.h` | PID：D 项取自传感器的速度（陀螺仪、升降速度计），抗积分饱和，未 ARM 时积分器冻结 |
| `feedback/*` | **雏形，未接入**：自适应反馈、起飞与降落——见[反馈](#反馈雏形未接入) |

### `sensors/`

| 文件 | 负责内容 |
|---|---|
| `SensorInterface.h` | `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` 接口和数据结构 |
| `SensorSelection.h` | 编译哪一种芯片（`#define SENSOR_*`，可用编译参数覆盖）以及它接在哪条总线上（`SELECTED_*_DEVICE(board)`） |
| `SensorMounting.h` | 把芯片坐标轴旋转到飞机坐标轴（顺时针 0/90/180/270°）——用于电子罗盘，以及未做安装校准的 IMU |
| `imu/ImuOrientation.h` | 用“芯片轴 → 飞机轴”矩阵描述 IMU 的安装：来自 `IMU_ROTATION_CW_DEG`，或来自三种姿态（水平、机头朝上、右翼朝下）并带校验；存于 NVS |
| `imu/ImuSensorBase.h` | IMU 的通用部分：陀螺仪校准 + 飞行前检查（静止、1g、“上方”与安装方向一致）、安装校准（`calibrateOrientation()`）、量程、旋转、航空符号、总线错误 |
| `imu/AttitudeEstimator.h` | 横滚/俯仰的互补滤波器，偏航积分 |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500（按 WHO_AM_I 识别芯片）：±2000°/s，±16g，DLPF 约 41 Hz，1 kHz。**已在试验台上验证** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P：±2000°/s，±16g，1 kHz，UI 滤波器 50 Hz。未在硬件上验证 |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X：±2000°/s，±16g，960 Hz，LPF1/LPF2；I2C 0x6A/0x6B 或 SPI。未在硬件上验证 |
| `imu/ICM45686_Sensor.h` | ICM-45686：±2000°/s，±16g，1.6 kHz，低通滤波器经间接寄存器 IPREG 配置；I2C 0x68/0x69 或 SPI。未在硬件上验证 |
| `baro/BarometerBase.h` | 气压计的通用部分：只读取新采样、高度、经低通滤波器得到的垂直速度、基准校准、错误 |
| `baro/BMP388_Sensor.h` | 通过 I2C 或 SPI 连接的 BMP388（SPI 带哑字节），Bosch 补偿，按就绪标志读取。**已在试验台上验证（I2C）** |
| `baro/BME280_Sensor.h` | BME280/BMP280，Bosch 补偿（§8.1）。未在硬件上验证 |
| `baro/SPL06_Sensor.h` | SPL06-001：系数和公式取自数据手册，32 Hz ×16；I2C 0x76/0x77 或 SPI。未在硬件上验证 |
| `baro/BMP581_Sensor.h` | BMP581：遵循 BMP5_SensorAPI 的流程，16×/2×，IIR；I2C 0x46/0x47 或 SPI；既可作主气压计，也可用作皮托管。未在硬件上验证 |
| `mag/MagnetometerBase.h` | 电子罗盘的通用部分：50 Hz 轮询，硬铁校准存于 NVS，坐标轴旋转，航向，错误 |
| `mag/QMC5883P_Sensor.h` | QMC5883P，0x2C。**已在试验台上验证** |
| `mag/QMC5883L_Sensor.h` | QMC5883L，0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309，0x7C：±8 G，200 Hz。未在硬件上验证 |
| `gps/UbloxM10_Gps.h` | u-blox M10：通过 CFG-VALSET 配置（115200 波特率，10 Hz，NAV-PVT，关闭 NMEA），解析 NAV-PVT。试验台上未接入 |
| `airspeed/AirspeedSensor.h` | 空速传感器接口：压差、IAS、TAS、密度 |
| `airspeed/PitotDualBaroAirspeed.h` | 自制皮托管：管内一个 BMP581 + 机身气压计；地面归零，低通滤波器，由静压推算密度，故障检测 |

### `telemetry/` 与应用程序

| 文件 | 负责内容 |
|---|---|
| `DebugLogger.h` | 按通道记录日志（`LogSettings.h`）：每个通道有自己的行、自己的抖动容限和模式；菜单打开时保持静默 |
| `DebugConsole.h` | 串口监视器中的文本菜单（`h`）和快捷键（`l`/空格/`s`/`i`/`o`/`m`/`p`/`b`）；日志设置在退出菜单时写入 NVS，且仅在未 ARM 时才写 |
| `LogSettings.h` | 日志通道（STAT、RC、OUT、ATT、AP、ALT、MAG、GPS、IMU、NAV、SYS）及其模式：关闭 / 变化时 / 持续；存于 NVS |
| `WebDebugServer.h` | 接入点、路由、JSON `/api/status`、命令邮箱；在核心 0 上有自己的任务 |
| `WebDashboardPage.h` | 仪表盘的 HTML/JS 写成单个字面量；通道/输出/传感器的行由浏览器根据 JSON 构建 |
| `OledDisplay.h` | 基于 `II2CBus`，通过 U8g2 驱动 SSD1306，有自己的任务（`Rtos`） |
| `MavlinkCodec.h`、`MavlinkTelemetry.h` | 面向 QGroundControl / Mission Planner 的 MAVLink 2：帧、数据流、PID 参数、从地面切换模式 |
| `LoopStats.h` | 每秒的频率、平均周期时间和最差周期时间（OLED），以及自上次读取以来的最差值（`takePeakUs()`，SYS 行） |
| `src/main.cpp` | ESP32：创建对象、`setup()`、带 `vTaskDelayUntil` 的 `loop()` |
| `src/stm32/main.cpp` | STM32H743：同样的对象、MAVLink、SD 卡黑匣子、`flight`/`storage`/`oled`/`bbox` 任务 |
| `src/stm32/sd_msp.cpp`、`src/stm32/bootloader.cpp` | STM32H743：为 `HAL_SD_Init` 配置 SDMMC1 的引脚和时钟；控制台的 `D` 键——重启进入 USB DFU 引导程序 |

---

## 符号约定：从 IMU 到舵机

整条链路使用同一套符号系统——因此摇杆和自动驾驶仪必然把舵面推向同一方向，而每个舵机的方向只在一个地方设定。

**1. 传感器坐标轴 → 飞机坐标轴。**`ImuSensorBase` 用 `ImuOrientation`
矩阵（body = R · chip）把芯片坐标轴旋转到飞机坐标轴：X 指向机头，Y 指向左侧，Z 向上。这个矩阵来自：

- **安装校准**（命令 `o`，存于 NVS）——电路板可以任意摆放。共三种姿态：
  “水平”给出 Z 轴（同时给出地平线——加速度计的零偏也包含在内），“机头朝上”
  给出 X 轴（“上方”中与 Z 垂直的那部分），“右翼朝下”给出 Y 轴。第 2 步得到的机头方向与第 3 步得到的机头方向（Y × Z）必须在约 25° 以内吻合，否则说明飞行员倾斜错了方向——校准会被拒绝；最终结果取两次估计的平均值。已在
  300 种随机安装方式上验证（`test/test_imu_orientation`，误差 < 0.1°）；
- 否则来自 `Config::IMU_ROTATION_CW_DEG`（电路板芯片朝上；其值表示当机头指向
  “12 点钟”时，*芯片* X 轴指向哪里），地平线取开机时的姿态。

每次校准陀螺仪（开机、`i`）时都会进行**飞行前检查**：陀螺仪噪声
< 0.5 °/s（静止；静止时约 0.08），|a| ≈ 1g，“上方”与保存的方向相差不超过
45°（电路板没有被挪动过）。未通过——`ImuSensor::getPreflightProblem()` ≠
nullptr：`ArmingManager` 不会为带增稳的模式执行 ARM，
`Autopilot::imuReady()` = false（所有模式下修正量均为零，包括信号丢失时的滑翔）。

> 目前的 GY-521（MPU6500 的仿制品）上，芯片焊接时相对于印刷的箭头旋转了
> 90°：丝印上的 X 箭头 = 芯片的 Y 轴。因此在没有安装校准时，
> `IMU_ROTATION_CW_DEG = 90`。任何一次重新摆放之后的检查方法：机头朝上 →
> P 向正方向增大，右翼朝下 → R 向正方向增大。

**2. 角度与角速度（`ImuData`）——航空符号：**

| 物理量 | “+”表示 |
|---|---|
| `roll`、`gyroX` | 右翼朝下 |
| `pitch`、`gyroY` | 机头朝上 |
| `yaw`、`gyroZ` | 机头向右（从上方看为顺时针） |

**3. 指令（`ControlCommand`，偏转量，单位 µs，±500 = 全行程）：**

| 字段 | “+”表示 | 来自摇杆 |
|---|---|---|
| `roll` | 向右横滚（右副翼向上，左副翼向下） | CH1：2000 = 向右 |
| `pitch` | 机头朝上（升降舵向上） | CH2 符号相反：2000 = 推离自己 = 机头朝下 |
| `yaw` | 机头向右（方向舵和前轮向右） | CH4：2000 = 向右 |
| `flaps` | 襟翼向下（两侧副翼都向下） | SwB（CH6）：0 或 `FLAPS_DEPLOYED_US`，在 `FLAPS_TRANSITION_MS` 内平滑过渡 |

PID 计算 `误差 = 目标 − 实际`：向右横滚（roll > 0）→ 横滚指令为负 →
飞机恢复水平。自动驾驶仪的修正量在混控器**之前**以相同的符号叠加到摇杆指令上。

**4. 指令 → PWM。**`ControlMixer::mix()` 计算每个舵面后缘的偏转量（副翼：向下 = “+”，左侧 = `flaps + roll`，右侧 = `flaps − roll`；升降舵：向上 =
“+”；方向舵：向右 = “+”），并换算为 PWM `1500 ± 偏转量`，对
`Config::*_REVERSED = true` 的舵机改变符号。默认值保持了固件此前对摇杆的行为。在组装好的飞机上的检查见 [`PILOT_GUIDE.md`](PILOT_GUIDE.md) 的飞行前检查清单。舵机反向必须在 `Config.h` 中修改，**而不是在遥控器上**——否则摇杆与自动驾驶仪的方向会不一致。

---

## RC 通道图、ARM 与 failsafe

来源是 `include/config/Channels.h`。遥控器 FS-i6（10 个通道，模式 2）+
接收机 FS-iA6B，iBUS 115200。

| 通道 | 遥控器上的部件 | 名称 | 用途 |
|---|---|---|---|
| CH1 | 右摇杆 ←→ | `AILERON` | 横滚 |
| CH2 | 右摇杆 ↑↓ | `ELEVATOR` | 俯仰 |
| CH3 | 左摇杆 ↑↓ | `THROTTLE` | 油门，全行程；< 950 = 接收机 failsafe |
| CH4 | 左摇杆 ←→ | `RUDDER` | 方向舵 + 转向轮（共用一个舵机） |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM（在 FS-i6 上是拨杆向下，朝自己一侧） |
| CH6 | SwB | `SWB` | 默认是襟翼（≥ 1750 表示放出） |
| CH7 | SwC（3 档） | `SWC` | 默认是模式：< 1250 MANUAL，1250–1749 STABILIZE，≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | 默认是 RTH |
| CH9 | VrA | `VRA` | 默认是增稳强度 |
| CH10 | VrB | `VRB` | 默认是巡航速度 |

CH6–CH10 只需在 `include/config/Controls.h` 中用一行代码即可分配（[AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#一行代码分配功能)）。

**ARM**（`ArmingManager`）：拨杆由 OFF 切到 ON，油门 < `THROTTLE_LOW_US`，并且已通过当前模式所需的传感器检查。否则拒绝，并在 Serial 中给出原因，需要重新执行一次 OFF→ON。开机时拨杆已在 ON 位置不会 ARM。OFF——立即
DISARM。未 ARM 时，送往电调的油门被强制为 `PWM_MIN`。

**信号丢失**（`IBusReceiver::isSignalLost()`）：

1. 超过 `RX_TIMEOUT_US`（500 ms）没有帧——接收机导线断开或断电。开机后收到第一帧之前也视为信号丢失：通道的默认值（全部为 1500）不会被当作遥控器的指令。
2. 油门 < `RX_FAILSAFE_THROTTLE_US`（950）——在遥控器中设定的 failsafe。
   **FS-iA6B 在遥控器丢失时不会停止发送帧**，而是重复最后的数值（已在试验台上验证），因此如果没有在遥控器中设置 failsafe，就无法识别信号丢失。设置方法见 `PILOT_GUIDE.md`。

信号丢失时会发生什么（`FlightController::applyLinkLoss()`）：

- **飞机已 ARM，且有 GPS 和返航点**（`FAILSAFE_RTH`）——**返航**，带动力飞行，到达返航点上空后盘旋；OLED 上显示 `FSRTH`，日志中为 `FAILSAFE_RTH`；
- **飞机已 ARM，没有 GPS**——**滑翔**，电机油门为 `FAILSAFE_THROTTLE`：无论处于何种模式，甚至 MANUAL，`Autopilot` 都会保持横滚角
  `FAILSAFE_GLIDE_ROLL_DEG`（0——直飞，10–20°——在飞手上空盘旋）和俯仰角
  `FAILSAFE_GLIDE_PITCH_DEG`（−3°，以免没有动力时损失速度），收起襟翼；
  OLED 上显示 `GLIDE`，日志中为模式 `FAILSAFE_GLIDE`；
- **未 ARM**（在地面上）或 IMU 无响应——舵面回到中立位；
- 模式和功能不再随拨杆切换，传感器继续读取。ARM 不会被清除——信号恢复后，飞机重新听从摇杆和所选模式（自动起飞和手抛起飞只能重新开始）。

---

## 传感器数据

数据结构位于 `include/sensors/SensorInterface.h`。

### `ImuData`

| 字段 | 单位 | 含义 |
|---|---|---|
| `gyroX`、`gyroY`、`gyroZ` | °/s | 飞机坐标轴下的角速度，采用航空符号（见上文） |
| `accelX`、`accelY`、`accelZ` | g | 飞机坐标轴下的加速度：X 指向机头，Y 指向左侧，Z 向上 |
| `roll`、`pitch` | ° | 互补滤波器（α = 0.98，τ ≈ 0.1 s）；启动时直接取加速度计给出的角度 |
| `yaw` | ° | 陀螺仪积分，会缓慢漂移；初始值取电子罗盘的航向 |
| `temperature` | °C | 芯片裸片温度（公式适用于 MPU6050 或 MPU6500） |
| `timestamp` | µs | 读取时刻的 `micros()` |

IMU 校准（每次启动时以及通过 `i` 命令）：静止 2 s，陀螺仪 → 零偏，加速度计 →
**当前姿态成为地平线**。

### `BarometerData`

| 字段 | 单位 | 含义 |
|---|---|---|
| `pressure` | Pa | 气压 |
| `temperature` | °C | 传感器温度 |
| `altitude` | m | **相对于校准点**（启动时）的高度；公式 `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | 高度对真实采样（50 Hz）的导数，经 τ = 0.5 s 的低通滤波器 |
| `timestamp` | µs | 最近一次新采样的时刻 |

### `MagData`

| 字段 | 单位 | 含义 |
|---|---|---|
| `magX`、`magY`、`magZ` | µT | 硬铁校准后的磁场，位于飞机坐标轴下（`MAG_ROTATION_CW_DEG`） |
| `headingDegrees` | °（0..360） | `atan2(magY, magX)`，不含倾斜补偿；读数的计数方向尚未在组装好的飞机上验证 |
| `timestamp` | µs | 读取时刻（50 Hz） |

### `GpsData`

| 字段 | 单位 | 含义 |
|---|---|---|
| `latitude`、`longitude` | ° | 来自 UBX-NAV-PVT |
| `altitude` | m | 海拔高度（hMSL） |
| `groundSpeed`、`heading` | m/s、° | 地速与对地航向 |
| `numSatellites`、`fixType` | — | 0 = 无定位，2 = 2D，3 = 3D |
| `horizontalAccuracy`、`verticalAccuracy` | m | 模块给出的精度估计 |

**`isAvailable()` 的含义**。对于 I2C 传感器：传感器在 `begin()` 时有应答，
**并且**最近的读取没有连续失败（MPU——约 0.1 s，气压计和电子罗盘——约 0.5 s
无应答）。读取失败时，数据不会被垃圾值覆盖：保留之前的数值，错误计数器递增（可用 `s` 命令查看）。对于 GPS：至少有一条有效的 NAV-PVT，且最近一条不早于 `GPS_TIMEOUT_US`。

**如果没有该传感器**（`nullptr` 或 `isAvailable() == false`），`Autopilot`
不给出任何修正，飞机按 MANUAL 方式操纵。`main.cpp` 只校准有应答的传感器。

---

## FlightController::update() 详解

由 `loop()` 每 2 ms 调用一次。顺序就是优先级：

1. **`receiver.update()`**——解析累积的 iBUS 字节。
2. **拨杆**——`switches->update(rc)`，仅在链路正常时执行（failsafe 帧中的通道并不反映拨杆状态）：模式（仅在变化时）、功能、旋钮。
3. **飞手油门**——`throttle.update(rc, receiverFailsafe)`。
4. **摇杆**——`mixer.fromSticks(rc)` × `Knob::RATES`；襟翼——
   `mixer.updateFlaps(target)`（刹车、拨杆、旋钮；无链路时为 0）。
5. **传感器与自动驾驶仪**——`autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **始终**执行，即使没有链路：角度滤波器不能停滞。未 ARM 时 PID 仍在运行（舵面会随倾斜而响应——放在桌上调试很方便），但积分器保持为零。无链路且已 ARM 时——failsafe RTH 或滑翔。
6. **蜂鸣器**——`Beeper`。
7. **信号丢失**——`applyLinkLoss()`：已 ARM 时，舵面和油门按自动驾驶仪的
   failsafe 指令输出，否则舵面回中立位、电机关闭；然后 `return`。优先级高于下面的一切。
8. **ARM**——`arming.update(rc, false)`。
9. **指令**——`autopilot->getCommand()`：在带增稳的模式下，摇杆表示期望角度，最终舵面指令由自动驾驶仪给出。
10. **混控器**——`mixer.mix(command)` → 副翼（襟翼 + 横滚）、升降舵和方向舵的
    PWM，已考虑舵机反向。
11. **油门**——`autopilot->applyThrottle(pilotThrottle)`：飞手油门、自动驾驶仪油门，或两者中的较大者（自动起飞）。然后，如果未 ARM 或处于 `MOTOR_KILL`，
    ——强制为 `PWM_MIN`。这项检查放在最后，以保证任何模式都无法让油门绕过 ARM。
12. **AUX**——载荷（`PAYLOAD_DROP`）和相机（`CAMERA_TILT`、`CAMERA_STAB`）。
13. **`outputs.write(output)`**——向 7 路输出写入 PWM。

---

## 网页仪表盘的 HTTP API

实现位于 `include/telemetry/WebDebugServer.h`。接入点：SSID
`OpenPlane-Debug`，密码 `12345678`，地址 `http://192.168.4.1`。

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached`——该对象存在于当前构建中；`available`——传感器确实有应答。数据字段**仅**在 `available: true` 时才会添加。
- `outputs.*.attached`——MCU 已分配 LEDC 通道和引脚；物理舵机是否接上，软件无法得知（要检查脉冲，请使用控制台的 `p` 命令）。
- `rollCorr`/`pitchCorr`——自动驾驶仪的最终指令减去摇杆量，单位 µs。
  `throttleCorr`——自动驾驶仪的油门，单位 %（油门在飞手手中时为 0）。
- `nav`——导航信息：返航点、到返航点的距离与方位、航向与目标航向、导航所用的速度（皮托管 / GPS）、地理围栏、失速；`features`——已启用的拨杆功能。

### `POST /api/setmode`

`{ "mode": 1 }`——`AutopilotMode` 的编号：`0` MANUAL，`1` STABILIZE，`2`
AUTO_TAKEOFF，`3` ALT_HOLD，`4` ACRO，`5` CRUISE，`6` LOITER，`7` RTH，`8`
LAUNCH，`9` AUTO_LAND，`10` SOARING，`11` RESCUE。该模式会一直保持，直到飞手拨动模式拨杆。

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }`——可以只含 `kpRoll`、
`kiRoll`、`kdRoll`、`kpPitch`、`kiPitch`、`kdPitch` 中的任意字段；省略的字段保持原值。

这两条命令都由飞行循环在下一个周期应用（见
[FreeRTOS 任务](#freertos-任务与控制循环)）。

### `GET /`

HTML 仪表盘：10 个通道的条形图、ARM/链路、各路输出、传感器、模式按钮、
PID 表单。每 200 ms 轮询一次 `/api/status`。

---

## 控制台与诊断

串口监视器——115200，接口为“COM”。实现位于 `DebugConsole` 和
`DebugLogger`（[参考](reference/telemetry.md)）。按键立即生效，无需按 Enter；校准和 `p` 会阻塞循环，因此仅在未 ARM 时可用。

| 按键 | 作用 |
|---|---|
| `h` / `?` | 主菜单 |
| `l` | “日志输出什么”菜单（通道、模式、周期） |
| 空格 | 暂停 / 继续日志 |
| `s` | 所有传感器的 `printStatus()`：数据、总线错误计数、校准、飞行前检查 |
| `i` | 陀螺仪校准 + 飞行前检查（静止 2 s） |
| `o` | 按三种姿态校准 IMU 的安装，保存到 NVS |
| `m` | 电子罗盘校准（旋转 15 s），保存到 NVS |
| `p` | 输出自检：把每个引脚上的实际脉冲与预期值对比 |

日志分为多个通道（`STAT`、`RC`、`OUT`、`ATT`、`AP`、`ALT`、`MAG`、`GPS`、
`IMU`、`SYS`），每个通道都有“关闭 / 变化时 / 持续”三种模式；设置保存在 NVS 中，并在关闭菜单时写入，且仅在未 ARM 时才写。默认启用 `STAT`（变化时）和 `SYS`
（每 10 s 一次）：

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (10 s 内最差) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

所有通道的格式见[参考](reference/telemetry.md#debuglogger)。

---

## 开发板选择与引脚分配

| 命令 | `board` | 宏 | 状态 |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8（`qio_opi`，16 MB） | `BOARD_ESP32_S3` | **主力板，默认选项**。已在试验台上连同全部传感器验证 |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | 早期原型，曾在手动控制下飞行 |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | 用于试验台，引脚分配尚未在硬件上验证 |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6：完整固件 + MAVLink + SD 卡黑匣子；已在裸板上验证（[见下文](#stm32h743)） |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | 同样适用于 DevEBox H743：控制台为 USB CDC，通过 DFU 烧录固件 |

| 用途 | ESP32-S3（试验台） | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| 左 / 右副翼 | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| 升降舵 / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| 方向舵 | GPIO18 | —（无引脚） | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| 传感器 I2C SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| OLED I2C SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40（UART2） | GPIO9 ⚠️ / 无（UART0） | GPIO4 / GPIO17（UART2） |
| Serial | UART0 → “COM”接口 | USB-CDC | UART0 |

- **ESP32-S3 N16R8**：GPIO33–37 被八线 PSRAM 占用，26–32 被闪存占用，19/20 被
  USB 占用，43/44 被 Serial 占用，48 是 RGB LED；0/3/45/46 是 strapping 引脚。
- **ESP32-C3**：GPIO4/5 上的副翼与 S3 相比互换了位置。引脚不够用于完整配置：
  BMP388 的 CS 和 GPS 的 RX 位于 strapping 引脚上，GPS 没有 TX（只接收，无法发送 UBX-CFG）。详情见 `Config.h`。

### STM32H743

STM32H743VIT6（Cortex-M7 480 MHz，2 MB 闪存，1 MB RAM）运行**完整固件**：传感器、自动驾驶仪、拨杆、控制台和显示屏与 ESP32-S3 相同，另外还有 MAVLink
遥测和 SD 卡上的黑匣子。固件可以编译，通过 cppcheck 以及通用代码的全部原生测试。在硬件上验证过的是**没有接传感器的 DevEBox H743 板**：启动、通过 USB 的控制台、SD 卡、黑匣子——见 [TESTING.md](TESTING.md#stm32-开发板上的测试)——
以及 iBUS、ARM 和送往舵机与电机的 PWM：用遥控器在手动模式下操控（有视频）。
STM32 上的传感器仍在等待试验台。主力飞控板是 ESP32-S3。

- **HAL**——`include/hal/stm32/`：`Stm32Board`（与 `Esp32Board` 的 API 相同，另加 `telemetryUart()`）、`Stm32I2CBus`、`Stm32SpiBus`、`Stm32UartPort`、
  `Stm32ServoOutput`（`HardwareTimer` 提供的硬件 PWM，一个定时器带多路输出）。详见 [reference/hal.md](reference/hal.md#stm32h743-的实现)。
- **设置与校准**——不用 NVS，而是放在闪存最后一个扇区里的 `KeyValueStore`
  （`include/storage/`、`hal/stm32/Stm32FlashStorage.h`）。项目代码仍然写
  `#include <Preferences.h>`：在 env `stm32h743` 中，目录
  `include/hal/stm32/compat/` 位于 `-I` 中，那里放着 API 相同的 `Preferences`。映像带有 CRC32：损坏的映像（擦除期间断电）会被当作空映像读取。写闪存在后台任务中进行：擦除一个 128 KB 的扇区需要数秒，但该扇区在存储体 2 中，而代码从存储体 1 执行，飞行任务会抢占后台任务而不会停顿。
- **任务**——使用 STM32duino FreeRTOS 库中的 FreeRTOS，单核，按优先级抢占（`hal/Rtos.h`）：`flight`（5）——飞行循环、MAVLink、日志、控制台；
  `oled`（1）和 `storage`（1）——后台运行；`bbox`（2）——把黑匣子数据写入
  SD 卡。
- **SD 卡上的黑匣子**——SDMMC1，4 位，24 MHz（`hal/stm32/Stm32SdCard.h`，引脚见 `src/stm32/sd_msp.cpp`）。这张卡仍是普通的 FAT32：上面放着预先创建的 `BLACKBOX.BIN` 文件，固件在它内部写入原始数据块，不碰文件系统本身（`storage/Fat32File.h` 只读）。卡的准备与数据导出见
  [BLACKBOX.md](BLACKBOX.md#sd-卡stm32h743)。
- **遥测**——UART4 上的 MAVLink 2（`telemetry/MavlinkTelemetry.h`），取代
  Wi-Fi 仪表盘：QGroundControl / Mission Planner，从地面切换模式和调整 PID。详见 [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#地面站wi-fi-仪表盘和-mavlink)。
- **引脚分配**——`Config.h` 中的 `BOARD_STM32H743` 块，引脚选自 WeAct
  MiniSTM32H743VITx 上空闲的引脚，并与 STM32duino 的表格核对过：

| 用途 | STM32H743 | 外设 |
|---|---|---|
| 左 / 右副翼 | PA0 / PA1 | TIM2_CH1 / CH2 |
| 升降舵 / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| 方向舵 | PD14 | TIM4_CH3 |
| AUX1（载荷）/ AUX2（相机） | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX（TX 备用） | PE7（PE8） | UART7 |
| 传感器 I2C SDA / SCL | PB11 / PB10 | I2C2 |
| OLED I2C SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / 气压计 | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| 数传电台 MAVLink RX / TX | PD0 / PD1 | UART4 |
| 蜂鸣器 | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743（MCUDEV）**——env `stm32h743-devebox`：代码相同，使用自己的内核变体，控制台通过 USB-C 作为虚拟 COM 口（CDC）——不需要 USB-UART。首次烧录通过 USB 使用内置引导程序（DFU）：
  1. Windows：为“STM32 BOOTLOADER”安装一次 WinUSB 驱动（[Zadig](https://zadig.akeo.ie)：DFU in FS Mode → WinUSB → Install Driver）。
  2. 用导线把 **BT0**（BOOT0）引脚连到 **3V3**，按下再松开 **RST**：板子进入
     DFU 模式（DevEBox 上没有 BOOT0 按键）。
  3. `pio run -e stm32h743-devebox -t upload`（`upload_protocol = dfu`）。
  4. 可以拿掉 BT0 导线——固件会自行启动。

  之后就不再需要导线：在控制台中按 **`D`** 键（在任何菜单里都可以，ARM 时除外）会让板子重启进入引导程序：在 RAM 中留下标记 → 复位 → 在配置时钟之前跳转到系统存储器（`src/stm32/bootloader.cpp`）。在 H7 上直接从正在运行的固件跳转会死机——已在板子上验证，所以才分成两步。需要打开控制台（USB CDC）；如果板子没有响应——接上 BT0 导线后按 RST。
- **入口点**——`src/stm32/main.cpp`（在 ESP32 构建中通过 `build_src_filter`
  排除）。对象与 `src/main.cpp` 中的相同；取代 `loop()` 的是各个任务，
  `vTaskStartScheduler()` 位于 `setup()` 末尾。
- **板子首次上电**：`pio run -e stm32h743 -t upload`（ST-Link），通过 USB-UART
  在 LPUART1 上看监视器；`b`——总线上能否看到传感器，`s`——传感器状态，
  `p`——输出端的脉冲（先拆下螺旋桨），然后接遥控器，并通过数传电台连接
  QGroundControl。

---

## 如何添加新传感器

### A) 现有类别中的另一种芯片（IMU、气压计、电子罗盘）

通用部分已经写在基类里了——所以芯片驱动会很小：

1. 创建 `include/sensors/<category>/<Name>_Sensor.h`，并继承自
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`。构造函数接收
   `IRegisterDevice&`——驱动并不知道它是 I2C 还是 SPI。
2. 需要实现：
   - `begin()`——`device.begin()`，检查芯片 ID，写寄存器，调用
     `setAvailable(true/false)`；
   - IMU：`readSample()`（芯片坐标轴下的原始加速度/陀螺仪/温度）、
     `accelLsbPerG()`、`gyroLsbPerDps()`、`temperatureC()`；
   - 气压计：`isNewSampleReady()`（就绪标志，或者直接返回 `true`）和
     `readSample()`（气压单位为 Pa，温度单位为 °C），轮询周期在基类构造函数中；
   - 电子罗盘：`readRaw()`（芯片坐标轴下的 X/Y/Z）和 `lsbPerMicroTesla()`，用于校准的 NVS 命名空间名称在基类构造函数中。
3. 如果芯片通过 SPI 通信时在数据前需要一个哑字节，或需要特殊的频率——添加静态工厂函数 `spiDevice(bus, cs)`，就像 `BMP388_Sensor` 那样。
4. 在 `SensorSelection.h` 中增加一个分支：`#define SENSOR_<CATEGORY>_<NAME>`、
   `using Selected... = ...;` 以及 `#define SELECTED_..._DEVICE(board) ...`
   （`I2cRegisterDevice(board.i2c(), address)` 或 SPI 工厂函数）。更换传感器时不需要改动 `main.cpp`。
5. 不改文件，用编译参数检查换用新传感器后的构建：
   `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`，然后构建全部三个环境，最后在硬件上验证。

### B) 新类别

1. 数据结构和接口放在 `SensorInterface.h` 中，参照 `GpsSensor`/`GpsData`。
2. 如果该类别有通用逻辑（滤波器、校准）——参照 `BarometerBase` 写一个基类。
3. 在 `Autopilot` 的构造函数里使用可为空的指针（没有传感器时——没有任何影响，而不是崩溃），并在 `GET /api/status` 中加入带 `attached`/`available`
   这一对字段的内容。

### 新的总线或外设

在 `include/hal/` 中新增接口，在 `include/hal/esp32/` 和
`include/hal/stm32/` 中实现，通过 `IBoard` 访问。

---

## 如何添加新的自动驾驶仪模式

1. 在 `enum AutopilotMode` 中增加一个值（`autopilot/AutopilotTypes.h`，位于
   `MODE_COUNT` 之前），并在 `AutopilotNames::mode()` / `modeShort()` 中给出名称和简称（最多 5 个字符，用于 OLED）。
2. 增加处理函数 `run<Mode>()`，并在 `Autopilot::runMode()` 中加一个分支；初始目标（航向、高度、盘旋圆心）写在 `initializeMode()` 中。该模式设定
   `desiredRoll`/`desiredPitch`，并调用 `stabilizeOrManual()`（没有 IMU 时舵面交给飞手）或 `stabilizeOrNeutral()`（没有 IMU 时——中立位）。缺少所需传感器时应表现为安全行为，而不是崩溃。积分器只在 `armed` 时累积。
3. 油门：`throttleMode`（`PILOT` / `AUTO` / `AT_LEAST`）和 `autoThrottlePct`，或者 `autoThrottle()`——来自旋钮或皮托管的巡航油门。这时
   `FlightController` 无需改动。
4. 在遥控器上——只需在 `config/Controls.h` 中加一行（`Bind::mode(Channels::SWD, MODE_NEW)`）。仪表盘和 MAVLink 会按编号识别该模式；对于 MAVLink——在 `MavlinkModes::toCustomMode()` /
   `fromCustomMode()` 中映射到最接近的 ArduPlane 模式。
5. 如果该模式需要传感器才能 ARM——修改 `ArmingManager`。
6. 测试：对每个传感器的响应——`test/native/test_autopilot_modes`；闭环飞行——
   `test/native/test_sim` 中的场景（飞机模型 `helpers/PlaneSim.h`，测试台架
   `helpers/SimHarness.h`）。然后是不装螺旋桨的桌面测试：舵面应当朝着使飞机恢复水平的方向响应倾斜。
7. 在 [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md) 中补充一节。

---

## 反馈（雏形，未接入）

`include/autopilot/feedback/` 是自动驾驶仪的下一步。**`FlightController`、
`Autopilot` 和 `main.cpp` 都没有包含这些文件**：目前还没有用于飞行测试的原型机，固件不依赖它们也能工作。它们靠闭环仿真（`test/test_feedback/`）直接在板子上验证。

### 为什么需要

如今的 `Autopilot` 是基于角度的 PID：误差 × 系数 = 舵量。它不知道这在飞机上产生了什么结果，而且系数只对某一个速度正确：低速时舵面更弱，PID 修正不足；高速时则修正过度。反馈把回路闭合在**飞机的响应**上：

- 舵面偏转了，但飞机转得比需要的慢——再加一些，直到转到位；
- 需要多大的舵量，在飞行中测量并随速度重新换算；
- 飞机朝错误的方向转——符号弄反了，翻转并检查；
- 角度已经改平，但速度在下降——加油门、压机头，直到飞机不再失速；
- 起飞和降落——根据传感器的读数分阶段进行。

### 模块

| 文件 | 作用 |
|---|---|
| `FlightSnapshot.h` | 反馈在每个周期内了解到的关于飞机的全部信息。它是唯一的输入：各模块不直接读取传感器和 RC，因此可以放在仿真和日志上运行 |
| `FeedbackOutput.h` | 每个周期的输出：各轴的舵面偏转量、该轴是否启用、轴的符号、油门（设定 / 不低于）、原因 |
| `FeedbackConfig.h` | 所有常量（接入时会迁移到 `Config.h`） |
| `SpeedEstimator.h` | 速度（皮托管 > GPS）以及由 IMU 得出的纵向加速度：`dV/dt = g·(ax − sin θ)`——即使没有空速传感器，也能看出“速度在下降” |
| `AirborneDetector.h` | 在空中 / 在地面：只有在飞行中，学习、累积积分和检测失速才有意义 |
| `ControlEffectivenessEstimator.h` | 对每个轴用递归最小二乘法学习模型 `ε = b·u(t−delay) + a·ω + c` |
| `AdaptiveRateController.h` | 级联：角度 → 角速度 → 角加速度 → 通过学到的模型得到舵量 |
| `StallGuard.h` | 防止速度丢失和失速 |
| `TakeoffSequencer.h`、`LandingSequencer.h`、`PhaseTargets.h` | 起飞（从跑道或手抛）和降落，根据传感器分阶段进行 |
| `FeedbackSupervisor.h` | 把一切整合在一起：每个周期的顺序、优先级、`requestTakeoff()`/`requestLanding()`/`cancelPhase()`、`printStatus()`、接入计划 |
| `FeedbackModules.h` | 一次 include 包含全部 |

### 工作原理

**舵面有效性**。轴的模型：角加速度 `ε = b·u + a·ω + c`。`b` 表示 1 µs 的舵量能产生多少 °/s²（符号表示响应的方向），`a` 是阻尼（空气会减慢旋转；没有这一项，在稳定旋转时对 `b` 的估计会趋于零），`c` 是恒定力矩（重心位置、配平、螺旋桨）。舵面的力 ∝ ρV²，所以 `b` 在参考速度下学习，并乘以 `(V/Vref)²`：飞机加速后，舵面立刻“变强”，无需重新学习。皮托管给出的表速本身已经包含了空气密度，因此高度已被自动考虑；没有空速传感器时比例为 1，`b` 直接学习。

数据按 20 ms 的间隔采集：一个间隔内的平均加速度，等于两端陀螺仪读数之差除以时长，与之对应的是同一间隔内的平均舵量和平均角速度（舵量要加上
`RESPONSE_DELAY_MS` 的延迟）。随后方程两边都通过同一个 2 Hz 低通滤波器：二者的比例关系不变，而“纯延迟”模型因舵机惯性而失真的高频成分则被滤除。只有在空中，并且舵面被“激励”时才能学习（在约 0.3 s 内摆幅 ≥
`MIN_EXCITATION_US`）；飞手的摇杆操作同样是激励，所以在 MANUAL 下估计值也在学习。

**控制器**。三级结构，逐轴进行：

```
ω* = ANGLE_GAIN · (target − angle)              "机头低了 10° — 以 40°/s 抬起"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "转得比需要的慢 — 修正"
surface = (ε* − a·ω − c) / b                    通过学到的模型
```

积分项 `I` 以 °/s 为单位保存，而不是以舵量 µs 为单位——因此在 `b` 的估计改变时它仍然正确。在地面上积分被冻结（起飞滑跑 / 着陆滑跑时的航向除外），舵面到达限位时，积分不会朝限位方向继续累积。协调转弯也被考虑在内（前提是已知速度）：在坡度中，俯仰需要 `g·sin φ·tg φ / V`，偏航需要 `g·sin φ / V`。

**轴的符号——只在地面确定**。飞行中不会关闭任何轴，也不会翻转轴的符号：
IMU 的安装由校准 `o` 和开机时的检查确定，舵面方向由飞手的飞行前检查确定。空中的间接迹象（失速改出、尾旋、特技动作、阵风）可能产生误导，而在这种时刻关闭或翻转某个轴，代价是整架飞机。如果某个轴的 `b` 估计值明确为负，也只是在
`reason` 中给出警告（“对舵面的响应相反？请在地面上检查”）；负的估计值不会送入控制器——该轴按先验模型工作。

**失速保护**。分两级。*LowEnergy*——抬头时速度迅速下降，或接近失速速度（< 1.25·Vs），或升降舵失去效力：油门 ≥ 80 %，俯仰 ≤ 5°。*Stall*——速度低于失速速度，机头在升降舵的作用下仍在下沉，机翼在能量不足时违背副翼指令而下坠：全油门、机头朝下、坡度 ≤ 10°、副翼受限（大幅度的副翼会使翼尖失速）。这些措施带滞回地解除（速度 ≥ 1.5·Vs）。信号丢失时不干预油门，贴近地面时（拉平、着陆滑跑）保护关闭——降落本身就是一次受控的失速。

**起飞**。`WaitThrottle`（电机停转）→ 飞手给油门 ≥ 50 % → `GroundRoll`
（全油门，机翼水平，方向舵和前轮保持航向，升降舵自由）→ 离地速度，或没有速度传感器时超时 → `Climb`（12°，全油门）→ 高度 30 m → `Complete`。手抛时（`TAKEOFF_HAND_LAUNCH`）取代滑跑的是 `WaitLaunch`：只有在抛出之后电机才启动（纵向加速度 ≥ 1g）。离地前收回油门——取消。

**降落**。`Approach`（油门 25 %，下降率 1 m/s——俯仰由垂直速度误差决定，坡度由飞手给出，≤ 20°）→ 高度 2 m → `Flare`（油门 0，下降率按同样的规则减小到 0.3 m/s）→ 加速度计检测到撞击，或“高度很低且不再旋转”→ `Rollout`
（用前轮保持航向）→ `Complete`。飞手油门 ≥ 80 %——复飞。拉平需要测距仪：气压计会有一米的误差。

**优先级**（`FeedbackSupervisor`）：未 ARM > 失速保护 > 起飞/降落 > 模式目标。信号丢失时取消各阶段，由增稳去执行 failsafe 滑翔的目标。

### 仿真

`test/test_feedback/test_main.cpp`（在 PC 上：`pio test -e native -f test_feedback`）——包含一个飞机模型（各轴相互独立、舵机延迟与惯性、舵面有效性 ∝ V²、阻尼 ∝ V、恒定力矩、由速度决定迎角进而决定升力、失速、带转向轮的起落架）以及 10 个场景：

| 场景 | 检验内容 |
|---|---|
| 在恒定力矩下，从 30° 坡度 / −15° 俯仰中改出 | 改平以及“再修正一些”：积分项自己找到配平量 |
| 在 14 和 20 m/s 下 ±15° 摆动，无速度传感器 | `b` 的估计值收敛到真值，并随速度重新换算 |
| 副翼接反，飞手在 MANUAL 下摇晃机翼 | `b` 的估计值为负 → 只给出警告，不关闭该轴 |
| 30 s 的颠簸 | 阵风被抵消，坡度不超过 10° |
| 油门 20 % 时机头 15°（有速度传感器和无速度传感器） | 速度不会降到失速 |
| 从跑道起飞，带螺旋桨的反扭矩 | 各阶段、高度、滑跑时的航向 |
| 从 15 m 降落 | 各阶段，近地时不给油门，轻柔着地 |
| 起飞滑跑时信号丢失；未 ARM；MANUAL | 取消，不干预油门，舵面仍由飞手控制 |

该模型比较粗糙——它检验的是逻辑和符号，而不是针对某个具体机体的调参。

```bash
pio test -e native -f test_feedback      # 在 PC 上，几秒钟
pio test -e esp32-s3 -f test_feedback    # 烧录测试固件并运行
pio run -t upload                        # 恢复正常固件
```

### 接入计划

1. `FlightController::update()` 在读取传感器并计算指令之后，填写 `FlightSnapshot` 并调用 `FeedbackSupervisor::update()`。首先是**影子模式**：输出只写入日志（`printStatus()`）和仪表盘，不送往舵面。在手动控制下飞行时，每个轴的 `b`
   估计值都应为正，并随速度增大。
2. 在地面上，手持飞机，STABILIZE：倾斜它——舵面会反向抵消。
3. 逐个轴启用：用 `deflectionUs` 取代 `Autopilot::getRollCorrection()`
   （先只做横滚），然后是俯仰。
4. 油门：`throttleOverridePercent`/`throttleFloorPercent`——放在
   `Autopilot::applyThrottle()` 之后、failsafe 之前（failsafe 的优先级高于一切）。
5. 起飞/降落——接到一个空闲的拨杆上；把 `AUTO_TAKEOFF` 模式从
   `Autopilot` 中移除。
6. `FeedbackConfig` 的常量——移入 `Config.h`；空速传感器——
   实现 `AirspeedSensor`，并在 `SensorSelection.h` 中增加该类别。

---

## 如何添加新开发板

1. 在 `platformio.ini` 中添加 `[env:<name>]`，并带上唯一的 `-D BOARD_ESP32_<NAME>`。
2. 在 `Config.h` 中添加 `#elif defined(BOARD_ESP32_<NAME>)` 块，写明所有引脚，包括 `PIN_I2C2_SDA/SCL`（没有 OLED 就写 −1）。要提前核算
   GPIO 预算：flash/PSRAM/USB/strapping。
3. 舵机输出需要 5 个 LEDC 通道——所有 ESP32 都有。如果方向舵没有可用引脚——`PIN_RUDDER = -1`，该输出会被直接关闭。
4. 在板子没有经过硬件验证之前，不要修改 `default_envs`；如果引脚分配尚未验证，要在提交说明中明确写出来。

---

## 构建、烧录与监视命令

```bash
pio run                        # 构建默认开发板（esp32-s3）
pio run -t upload              # 烧录
pio device monitor             # 监视器，115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # 检查所有开发板都能构建
```

- **ESP32-S3**：烧录和 Serial 都通过“COM”接口（CH343）。如果桥接芯片卡死（Windows 提示“设备无法正常工作”——可能是电调的干扰引起的），重新插拔线缆即可；也可以通过“USB”接口（内置 USB-JTAG）烧录：
  `pio run -t upload --upload-port <USB COM port>`。
- 只要串口监视器还开着，就无法向同一个端口烧录。
- `lib_deps`：`olikraus/U8g2`（OLED）是唯一的外部库。
- `test/`——详见 [`TESTING.md`](TESTING.md)：
  - `pio test -e native -e native-stm32`——在 PC 上运行 387 个测试（硬件的模拟替身位于 `test/native/support/`），覆盖率用 `gcovr`；
  - `pio test -e esp32-s3`——在板子上运行 `test_feedback/`（反馈闭环仿真）和 `test_imu_orientation/`；每一项都会烧录测试固件，之后请用
    `pio run -t upload` 烧回正常固件。
- 静态分析：`pio check -e esp32-s3`（cppcheck）、`pio check -e stm32h743`（对 `hal/stm32/` 和 `src/stm32/`
  运行 cppcheck）以及 `tools/clang-tidy.sh`
  （配置文件 `.clang-tidy`）。

---

## 已知局限

- **自动驾驶仪尚未经过飞行验证**。在桌面上已实时验证了符号（倾斜 → 朝改平方向修正），PID 系数只是初始值。
- **STABILIZE 是叠加在摇杆之上的改平**，而不是由摇杆设定横滚/俯仰角度的
  “角度模式”（FBWA）。飞手和自动驾驶仪的作用会叠加。
- **信号丢失时的滑翔尚未经过飞行验证**。`FAILSAFE_GLIDE_*` 角度只是初始值；
  −3° 的俯仰要针对具体机体来调整（机头既不能仰到失速，也不能俯冲）。
- **地平线**。有安装校准（`o`）时——取自校准结果（NVS）；加速度计的零偏会随温度漂移（每 20 °C 约 1–2°），如果地平线“漂走”了——重新执行 `o`。没有校准时——取开机时的姿态（开机时要放平）。
- **电子罗盘的安装方向**仍由 `MAG_ROTATION_CW_DEG` 设定（按姿态校准不涉及它）。
- **电子罗盘**：航向没有倾斜补偿，读数的计数方向尚未在组装好的飞机上验证，而且校准必须在飞机上进行。目前还没有任何模式使用航向。
- **GPS** 尚未用于导航；在 ESP32-C3 上只能接收。
- **反馈（`autopilot/feedback/`）尚未接入**，仅在带有粗糙飞机模型的仿真中验证过。`FeedbackConfig.h` 中所有标注为“прикидка”（“粗略估计”）的数字都需要在真实机体上进一步确认；目前还没有空速传感器（没有它，舵面有效性学得更慢，失速也只能通过减速来察觉）。
- **尚未在硬件上验证**：`ICM42688_Sensor`（已通过 `ImuSensorBase` 统一到通用约定）、`BME280_Sensor`（Bosch 补偿是重新实现的）、SPI 接口的 BMP388、
  `QMC5883L_Sensor`、通过 CFG-VALSET 配置 GPS。接入时——查看启动日志、控制台中的 `s`，并通过倾斜检查符号。
- **面包板上的 I2C 会受到干扰**，来自电调/电机（偶发错误可通过 `s` 看到）。驱动能够承受这些错误，但在飞机上，I2C 导线要短，并远离动力线。
- **不使用 ESP32Servo**。3.2.1 版在 ESP32-S3 上把舵机分配给 MCPWM，并在
  `attachPin()` 中把 MCPWM 单元编号与定时器编号搞混：GPIO6/7 输出了
  GPIO4/5 的信号（电调被右摇杆控制）。输出已改写为基于 LEDC；要换回这个库，必须先用 `p` 检查。
- **电调使用 50 Hz PWM**，固件中暂时没有油门行程校准模式。
- **网页仪表盘**：接入点的密码很弱，而且飞行中也会接受命令。它是用于试验台和野外的工具，不是用于飞行的。
- **原型机的机械结构**：第一架原型机飞过，发现电机固定不牢，机翼刚度不足。
- **许可证为 OpenPlane License**（[LICENSE](LICENSE.md)）：MIT，并强制署名作者，禁止军事用途，禁止在未经当事人书面同意的情况下故意伤害人员和损坏财产。请不要在文件中添加其他许可证头，也不要删除作者姓名。

---

## 如何提交修改

- **小步提交**：一个逻辑步骤对应一次提交。
- **提交前先测试和分析**：`pio test -e native -e native-stm32`、
  `pio check -e esp32-s3`、`pio check -e stm32h743`、`tools/clang-tidy.sh`——
  全部通过（[`TESTING.md`](TESTING.md)）。
- **修改通用代码后要构建所有开发板**——S3 是主力，但 C3、
  38 针板和 `stm32h743` 不能被破坏；发布前运行
  `tools/build_matrix.sh`（所有开发板 × 所有传感器）。
- **能在硬件上验证的就在硬件上验证**：符号——靠倾斜，输出——用 `p` 命令，链路——靠关闭遥控器。
- **不要凭空编造 API**。请对照 `~/.platformio/packages/framework-arduinoespressif32/`
  中的框架源码（Arduino core 2.0.x）——网上的资料常常描述的是 API 不同的
  3.x 版本（例如 LEDC）。
- **不要美化状态**。没有在硬件上验证过——就如实写明。
- **各层不应知道超出其职责的内容**。如果下层的类突然需要用到上层的类，就应当把逻辑上移到 `FlightController`。
- **修改数据契约时**（`FlightOutputState`、`ControlCommand`、
  `ImuData`、`/api/status` 的 JSON）——要在同一次提交中更新所有使用方。
