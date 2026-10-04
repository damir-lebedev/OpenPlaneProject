# CONFIG — `Config`、`Channels`、`Controls`

> 🌐 本页是[俄语原文](../../../reference/config.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

[← 参考](README.md)

配置层只有 `constexpr` 常量，不含代码。各个类的逻辑中不应出现“魔法数字”式的引脚、超时和阈值：凡是可能需要针对具体飞机或开发板修改的内容，都放在这里。

---

## namespace `Config`

**文件：** `include/config/Config.h` · **依赖：** `<stdint.h>` ·
**使用者：** 几乎所有层。

### 引脚（取决于开发板）

引脚块由 `platformio.ini` 中 `[env:*]` 设置的宏来选择（`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`）。没有定义宏时——`#error`。STM32 的引脚块见
[下文](#stm32h743vit6board_stm32h743)。

| 常量 | 类型 | 用途 | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | 副翼 | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | 升降舵 | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | 电机调速器 | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | 方向舵 + 前轮；`-1`——该输出被关闭 | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | iBUS 接收机的 RX（UART1） | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | 传感器总线（`Wire`） | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | OLED 总线（`Wire1`）；`-1`——没有 | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | 共用的 SPI 总线 | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | SPI 接口 IMU 的 CS | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | SPI 接口气压计的 CS | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | GPS 的 UART；TX 为 `-1`——只接收 | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | GPS 使用的硬件 UART 编号 | 2 | 0 | 2 |
| `PIN_AUX1`、`PIN_AUX2` | `int8_t` | 舵机输出：抛投载荷、襟翼；`-1`——没有 | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | 通过三极管驱动的蜂鸣器；`-1`——没有 | 38 | −1 | 2 |
| `PIN_AUX3`、`PIN_LIGHT`、`PIN_VBAT_ADC`、`PIN_CURRENT_ADC`、`PIN_TELEM_TX/RX` | `int8_t` | **仅 S3**：为飞控板预留（[FC_BOARD.md](../FC_BOARD.md)） | 47, 21, 8, 3, 9/10 | — | — |

传感器的 SPI 总线命名为 `PIN_SENSOR_SPI_*`，而不是 `PIN_SPI_*`：在
STM32duino 内核（以及其他 Arduino 内核）中，`PIN_SPI_SCK/MISO/MOSI` 是变体宏，它们会替换掉 `Config` 中的常量。

<a id="stm32h743"></a>

#### STM32H743VIT6（`BOARD_STM32H743`）

目前还没有这块板子：引脚分配**尚未在硬件上验证**（固件在 PC 上运行，env `native-stm32`）。引脚选自 WeAct MiniSTM32H743VITx（PlatformIO 的 env `stm32h743` 所用板）上空闲的引脚，并与 STM32duino 变体的 `PeripheralPins` 表核对过。取值是变体宏（`PA0`……），因此在 `Config.h` 开头的
`#if defined(BOARD_STM32H743)` 下引入了 `<Arduino.h>`。所有引脚的类型都是
`int16_t`（模拟引脚的编号为 `0xC0 + N`）。没有 UART 编号——内核根据引脚选择外设。

| 常量 | 引脚 | 外设 |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7（TX——为 iBUS-SENS 预留） |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2——传感器 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1——显示屏（在 WeAct 上是摄像头接口） |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1——载荷 / 相机 |
| `PIN_BUZZER` | PE15 | GPIO——蜂鸣器 |
| `PIN_VBAT_ADC`、`PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11——预留 |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4——MAVLink 数传电台（同样的引脚也是 FDCAN1） |

控制台 `Serial` 是 LPUART1（PA9 TX / PA10 RX），即该变体的默认设置。

### iBUS 与信号丢失

| 常量 | 值 | 含义 |
|---|---|---|
| `IBUS_CHANNELS` | 10 | 使用帧中的多少个通道 |
| `IBUS_FRAME_LENGTH` | 32 | 帧长度，字节 |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | 帧头 |
| `IBUS_BAUDRATE` | 115200 | UART 速率 |
| `RX_TIMEOUT_US` | 500 000 | 超过该时间没有正确的帧——信号丢失 |
| `RX_FAILSAFE_THROTTLE_US` | 950 | 油门低于该值——接收机报告遥控器的 failsafe |

### GPS

| 常量 | 值 | 含义 |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | NAV-PVT 比这更旧——`UbloxM10_Gps::isAvailable() == false` |

### PWM 范围与舵面行程

| 常量 | 值 | 含义 |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | 标准的 RC 脉冲，µs |
| `AILERON_MAX_US`、`ELEVATOR_MAX_US`、`RUDDER_MAX_US` | 500 / 500 / 300 | 摇杆满行程时相对中心的偏转量，µs。方向舵较小：同一个舵机上还带着起落架的前轮 |
| `THROTTLE_LIMIT_PCT` | 100 | 送往电调的油门上限，%，对摇杆和自动驾驶仪相同（`FlightController::capThrottle`）。在较弱的 3S1P 电池上做台架测试时曾设为 50；测试会按该值计算预期输出 |

### 襟翼（襟副翼）

| 常量 | 值 | 含义 |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 高于该值——襟翼放出（不用 1500：在第一帧到来之前通道值 = 1500） |
| `FLAPS_DEPLOYED_US` | 220 | 每个副翼向下的偏转量，µs（MG90S 摇臂约 20°） |
| `FLAPS_TRANSITION_MS` | 1000 | 完全放出/收起所需的时间 |

### 舵机方向

`AILERON_LEFT_REVERSED`、`AILERON_RIGHT_REVERSED`（`true`——副翼舵机是镜像安装的）、`ELEVATOR_REVERSED`（`true`）、
`RUDDER_REVERSED`——这是设置舵机反向的唯一位置。`ControlMixer`
按物理符号计算，只在这里改变符号，因此摇杆和自动驾驶仪不会出现方向不一致。**不能**在遥控器上设置反向。

### 传感器的安装

| 常量 | 值 | 含义 |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | IMU 芯片坐标轴绕垂直方向的旋转角度（0/90/180/270），即芯片 X 轴的朝向。仅在 NVS 中没有安装校准 `o` 时使用 |
| `MAG_ROTATION_CW_DEG` | 0 | 电子罗盘的同类设置（电子罗盘没有安装校准） |

### ARM

| 常量 | 值 | 含义 |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 高于该值——ARM 拨杆已打开 |
| `THROTTLE_LOW_US` | 1050 | 油门低于该值——视为“油门在底”，可以执行 ARM |

### Failsafe

| 常量 | 值 | 含义 |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | 舵面中立位 |
| `FAILSAFE_THROTTLE` | 1000 | 电机关闭 |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | 空中信号丢失时滑翔的坡度 |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | 滑翔的俯仰角（略低于地平线） |
| `FAILSAFE_RTH` | `true` | 有 GPS 和返航点时，空中信号丢失——带动力返航，而不是滑翔 |

### 拨杆、皮托管、自动驾驶仪

所有模式和功能的数值都在 `Config.h` 中，旁边有详细的注释；它们对飞手意味着什么，见 [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md)。

| 分组 | 常量 |
|---|---|
| 拨杆 | `SWITCH_ON_US` = 1750（通道高于该值——拨杆打开；不用 1500，以免在第一帧到来之前触发任何功能） |
| 皮托管 | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| 增稳 | `MAX_BANK_DEG` 45（旋钮 15…60）, `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| 导航 | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| 高度 | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| 油门与速度 | `CRUISE_THROTTLE_PCT` 55（30…85）, `CRUISE_AIRSPEED_MS` 14（10…22）, `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| 失速 | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| 盘旋与返航点 | `LOITER_RADIUS_M` 50（25…150）, `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| 地理围栏 | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| 手抛起飞 | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| 降落 | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| 翱翔 | `SOAR_*`：滑翔 −3°，热气流 > 0.5 m/s 持续 1.5 s，盘旋 25°，退出条件 < −0.2 m/s 持续 8 s，30 m 以下开电机直到 100 m，距离超过 400 m 时返航 |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| 自动配平 | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, 在地面保存：`AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| 转弯协调 | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| 功能 | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### 循环、Wi-Fi、调试

| 常量 | 值 | 含义 |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | 飞行循环的周期（500 Hz）；同时也是 `PidController` 的名义 `dt` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | 仪表盘的接入点（密码很弱——这是试验台用的工具） |
| `WEB_SERVER_PORT` | 80 | HTTP 端口 |
| `TELEM_BAUDRATE` | 57600 | MAVLink 数传电台的速率（SiK 的默认值） |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | 飞机在 MAVLink 中的地址 |
| `DEBUG_INTERVAL_MS` | 100 | `DebugLogger` 检查日志通道的频率 |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | “变化时”模式下对 RC/PWM 抖动的容限 |

### 黑匣子

详见 [BLACKBOX.md](../BLACKBOX.md)。

| 常量 | 值 | 含义 |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | 位于 PSRAM 的记录队列（没有 PSRAM 时——放在内部存储器中） |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743：位于 RAM 的队列——10 s 的预录以及应对存储卡延迟的余量 |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743：SD 卡上的文件及其可使用部分的上限（开机时的核对时间随该区域增大而变长） |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | 启动前（ARM + 油门）和 DISARM 之后的记录时长 |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | 已 ARM、电机停转、飞机静止达到该时长——停止记录 |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | 什么情况算“静止” |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | 异常重启之后——记录时长不得短于该值 |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | 随时备好的已擦除空间；旧的飞行记录在地面上整段擦除 |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | 两次擦除之间的停顿 |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU 每隔 N 个周期记录一次（1——500 Hz） |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | 飞控板上电池（56k/10k）和电流传感器（10k/15k）的分压比 |

---

## namespace `Channels`

**文件：** `include/config/Channels.h` · **依赖：** `<stdint.h>`

物理通道号与用途唯一对应的地方。取值是 `RcChannelState` 中的**索引**
（从 0 开始）。

| 常量 | 索引 | 通道 | FS-i6 上的部件 | 用途 |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | 右摇杆 ←→ | 横滚 |
| `ELEVATOR` | 1 | CH2 | 右摇杆 ↑↓ | 俯仰（2000 = 推离自己 = 机头朝下） |
| `THROTTLE` | 2 | CH3 | 左摇杆 ↑↓ | 油门 |
| `RUDDER` | 3 | CH4 | 左摇杆 ←→ | 方向舵 + 前轮 |
| `ARM` | 4 | CH5 | SwA | ARM 拨杆（不能重新分配） |
| `SWB` | 5 | CH6 | SwB | 按 `Controls.h` 表（默认是襟翼） |
| `SWC` | 6 | CH7 | SwC（3 档） | 默认是模式 MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | 默认是 RTH |
| `VRA` | 8 | CH9 | VrA | 默认是 `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | 默认是 `CRUISE_SPEED` |
| `COUNT` | 10 | | | 通道数量 |

---

## namespace `Controls`

**文件：** `include/config/Controls.h` · **依赖：** `ControlBinding.h`、`Channels`

`constexpr Binding BINDINGS[]`——每个拨杆和旋钮的作用，**每个通道一行**
（`Bind::modes/mode/feature/knob`，参见
[autopilot.md](autopilot.md#bindingbindbindingcheck)）。旁边是被注释掉的现成想法。三个 `static_assert` 会在构建时捕获表中的错误：表中出现摇杆或
ARM、通道越界、通道重复、不止一个模式选择拨杆。
