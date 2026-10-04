# TELEMETRY — 日志、控制台、网页仪表盘、OLED、黑匣子

> 🌐 本页是[俄语原文](../../../reference/telemetry.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

[← 参考](README.md)

遥测与飞行逻辑完全分离：它只读取 `FlightController`、`Autopilot`、传感器和 `LoopStats` 的 const 访问器。唯一的“回传”通道是仪表盘命令，经由 `WebDebugServer` 的邮箱传递，再由飞行循环应用。

---

## `LoopStats`

**文件：** `telemetry/LoopStats.h` · **类别：** struct

飞行循环的频率和耗时。

| 成员 | 说明 |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | 每秒发布一次；由其他任务读取（32 位数值——不会出现“撕裂”的读取） |
| `void record(uint32_t durationUs)` | 在 `loop()` 中每个节拍调用 |
| `uint32_t takePeakUs()` | 自上次调用以来最差的节拍（用于每 10 s 一次的 SYS 行）；必须在与 `record()` 相同的任务中调用 |

`maxUs` 只是最近一秒内的最差值；偶发的卡顿可以通过
`takePeakUs()` 看到。

---

## `LogSettings`

**文件：** `telemetry/LogSettings.h` · **依赖：** `Preferences`（NVS，命名空间 `debuglog`）

### `LogChannel`（enum class）

| 通道 | 前缀 | 输出内容 | 默认 |
|---|---|---|---|
| `Status` | `STAT` | 链路、ARM、模式、襟翼、传感器 | 变化时 |
| `Rc` | `RC` | 遥控器通道 | 关 |
| `Outputs` | `OUT` | 舵面和电调（ESC）的输出 | 关 |
| `Attitude` | `ATT` | 滚转、俯仰、航向 | 关 |
| `Autopilot` | `AP` | 目标和修正量 | 关 |
| `Altitude` | `ALT` | 高度、垂直速度 | 关 |
| `Heading` | `MAG` | 指南针航向 | 关 |
| `Gps` | `GPS` | 卫星、坐标 | 关 |
| `Imu` | `IMU` | 陀螺仪和加速度计 | 关 |
| `Nav` | `NAV` | 返航点、航向、速度、皮托管、已启用的功能 | 关 |
| `System` | `SYS` | 循环频率、内存（每 10 s），只有关/开 | 开 |
| `Count` | — | 通道的数量 | — |

`LogMode`（enum class）：`Off`、`OnChange`、`Periodic`。

`LogChannelInfo`：`tag`、`title`、`periodicOnly`、`defaultMode`。

| 方法 | 说明 |
|---|---|
| `static constexpr uint8_t COUNT`、`PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | 通道表中的一行 |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms（循环切换） |
| `LogSettings()`、`void setDefaults()` | 默认模式，周期 1 s |
| `LogMode mode(uint8_t)`、`mode(LogChannel)` | 通道的模式 |
| `void setMode(uint8_t, LogMode)` | 对 `periodicOnly` 通道，`OnChange` 会变成 `Periodic` |
| `void cycleMode(uint8_t)` | 关 → 变化时 → 持续 → 关（SYS：关 ↔ 开） |
| `void setAll(LogMode)` | 作用于所有通道；“全部变化时”命令不会改动 SYS |
| `uint16_t periodMs() const`、`void cyclePeriod()` | “持续”模式的周期 |
| `static const char* modeName(LogMode, bool periodicOnly)` | “关” / “变化时” / “持续”（或“开”） |
| `void load()` | 从 NVS 读取；如果 `VERSION` 或长度不匹配，则保留默认值；未知的模式代码 → 该通道的默认值 |
| `void save() const` | 写入 NVS（模式、周期、版本） |

`VERSION` 随通道列表一起变化——旧的设置会被重置（`VERSION = 2`：增加了 NAV 通道）。菜单中的通道按键：`1`..`9`，NAV 为
`n`，SYS 为 `s`。

---

## `DebugLogger`

**文件：** `telemetry/DebugLogger.h` · **依赖：** `FlightController`、`Autopilot*`、`LoopStats*`、`LogSettings`、`Config`

按通道向串口监视器输出状态：每个通道都有自己的一行、自己的模式和自己的容差。

| 方法 | 说明 |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | 每个 `DEBUG_INTERVAL_MS` 遍历一次通道（暂停时以及菜单打开时保持安静） |
| `LogSettings& getSettings()`、`void saveSettings() const` | 供控制台菜单使用 |
| `void suspend(bool)` | 菜单打开——保持安静；解除时——`refresh()` |
| `void setPaused(bool)`、`bool isPaused() const` | 用空格键暂停；解除时——`refresh()` |
| `void refresh()` | 下一个节拍会打印所有已启用的通道 |

通道的逻辑（`updateChannel`）：

- `Off` ——不打印；
- `Periodic` ——每 `periodMs()` 一次（SYS 为每 10 s 一次），数值“原样”输出；
- `OnChange` ——按**容差**拼出这一行（内嵌的 `Shown` 保存旧值，直到新值超出容差：RC/PWM 3 µs，角度
  0.5°，航向 1°，修正量 2，高度 0.3 m，加速度 0.03 g，坐标
  1e−5°），只有与上次打印的内容不同才会打印。

内嵌的类型：`LineBuffer : Print`（最长 200 字节的一行，用于打印前比较）、`Shown`（带迟滞的数值）。

各行的格式：

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  目标 0.0 m
MAG  航向 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (10 s 内最差) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` 区分 `LOST(无帧)` 和 `LOST(遥控器失效保护)`；`IMU=` 为
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`。

---

## `DebugConsole`

**文件：** `telemetry/DebugConsole.h` · **依赖：** `FlightController`、`FlightOutputs`、`Autopilot`、`DebugLogger`、`LogSettings`、`IBoard*`（总线扫描）

串口监视器中的文本菜单。屏幕状态机 `Screen::{None, Main, Log}`。

| 方法 | 说明 |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | 带电路板时——提供 `b` 命令和菜单第 7 项 |
| `static const char* guessI2cDevice(uint8_t address)` | 按地址推断芯片：0x6A LSM6DSV，0x68 MPU/ICM，0x76 BME280/BMP388/SPL06，0x46/0x47 BMP581，0x7C QMC6309，0x2C QMC5883P，0x0D QMC5883L，0x3C OLED |
| `void printHint() const` | 一行提示 |
| `void update()` | 处理来自 `Serial` 的所有字节；如果日志设置有改动，菜单已关闭且飞机**未处于 armed 状态**，则保存到 NVS |

快捷键（菜单之外）：`h`/`?`——主菜单；`l`——日志菜单；空格——
暂停日志；`s`——传感器状态；`i`——陀螺仪校准；`o`——
IMU 安装方向校准；`m`——指南针校准；`p`——输出自检；
`b`——扫描 I2C 总线（0x08..0x7F——一直扫到 0x7F，因为 QMC6309 位于 0x7C），并给出芯片名称；其他——提示。`\r`/`\n` 会被忽略。

日志菜单：`1`..`9`——循环切换通道 0..8 的模式，`n`——NAV，`s`——SYS，`p`——周期，`a`——
全部“变化时”，`x`——全部关，`d`——恢复默认，`0`/`q`——返回，`l`/`h`——
关闭。

会阻塞的操作（`i`、`o`、`m`、`p`）**在 ARM 时被禁止**。菜单打开期间，日志被挂起（`DebugLogger::suspend`）。菜单项的宽度按 UTF-8 字符而不是字节计算（西里尔字母占 2 字节）。

---

## `WebDashboardPage`

**文件：** `telemetry/WebDashboardPage.h` · **类别：** namespace

`static const char HTML[] PROGMEM`——整个页面（HTML + CSS + JS）放在同一个字面量里。所有动态内容都由浏览器根据 `/api/status` 的 JSON 构建（每
200 ms 轮询一次）：通道、输出和传感器的行按 JSON 的键创建，所以新增一个输出无需修改页面。用户已经开始编辑的 PID 字段，轮询不会再覆盖。

---

## `WebDebugServer`

**文件：** `telemetry/WebDebugServer.h` · **依赖：** `WebServer`、`WiFi`、`FlightController`、`Autopilot*`、`WebDashboardPage`、`Config`

| 方法 | 说明 |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Wi-Fi AP（`persistent(false)`——不写入闪存）、路由、核心 0 上的任务 `web`。如果接入点没有启动则返回 `false` |
| `void applyPendingCommands()` | 在飞行循环中调用：在自旋锁保护下取出命令并应用到自动驾驶仪 |

路由：

| 路由 | 响应 |
|---|---|
| `GET /` | 仪表盘页面 |
| `GET /api/status` | 状态 JSON（`buildStatusJson()`），格式见 [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`；没有消息体 → 400 `no data`；没有自动驾驶仪 → 503；模式错误 → 400 `invalid mode` |
| `POST /api/setpid` | `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch` 中的任意几个；省略的保持当前值 |
| 其他 | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }`——受 `portMUX`
保护的邮箱。`extractJsonNumber(body, key, fallback)`——不依赖 ArduinoJson 的扁平 JSON 最小解析：`"key"`、空格、`:`、空格、任意
JSON 写法的数字（正负号、小数、指数 `1e-7`）；缺少键或数字时返回
`fallback`。

在 JSON 中，`attached`/`available` 字段**始终**存在；传感器数据只在
`available: true` 时才有。

---

## `OledDisplay`

**文件：** `telemetry/OledDisplay.h` · **依赖：** U8g2、`II2CBus`、`FlightController`、`Autopilot*`、`LoopStats`

第二条 I2C 总线上的 SSD1306 128×64（I2C 0x3C）；有自己的任务 `oled`
（`Rtos::startTask`：ESP32 上在核心 0，STM32 上为低优先级），每 200 ms 一次。

| 方法 | 说明 |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` 或屏幕在 0x3C 无应答 → `false`；否则配置 U8g2 并启动任务 |

U8g2 通过建立在 `II2CBus` 之上的 `byteCallback` 发送字节（屏幕并不知道
`Wire1`）。C 回调拿不到上下文，所以总线保存在静态变量
`busSlot()` 中——板上只有一块屏幕。

屏幕：

```
RX ok ARM STAB FL       链路（丢失——反色行）/ ARM / 模式 / 襟翼
R  +1.2 P  -0.4         滚转 / 俯仰，°           (IMU --)
Alt +0.3 Vz +0.1 A14    高度 / 垂直速度 / 空速（如果有皮托管） (BARO --)
H123 T1000 Y1500        航向 / 油门 / 方向舵 (H---)
L1500 R1500 E1500       副翼 / 升降舵
Loop 500Hz max1100us    频率和一秒内最差的节拍
```

模式的简称来自 `AutopilotNames::modeShort()`（`MAN`、`STAB`、
`TKOFF`、`ALT`、`ACRO`、`CRZ`、`LOIT`、`RTH`、`LNCH`、`LAND`、`SOAR`、`RESQ`）；在空中丢失链路时——`GLIDE` 或 `FSRTH`。

---

## `BlackBox`

**文件：** `telemetry/BlackBox.h` · **依赖：** `FlightController`、`Autopilot`、`LoopStats`、`BlackBoxStorage`、`PilotSwitches*`

把飞行过程记录到闪存（ESP32-S3）或 SD 卡（STM32H743）。导出什么、何时导出、如何导出——见 [BLACKBOX.md](../BLACKBOX.md)。

| 方法 | 说明 |
|---|---|
| `bool begin(bool startTask = true)` | 读取存储介质（`BlackBoxStorage::begin()`），分配队列（ESP32 用 PSRAM，STM32 用 `malloc`），核对已擦除的空间（最多 0.3 s），启动任务 `bbox`（`Rtos::startTask`）。没有可记录的位置（没有分区、卡或文件）——返回 `false`，黑匣子关闭 |
| `void update(uint32_t workUs)` | 在 `loop()` 中每个节拍之后调用：事件、开始/停止、把快照放入队列、唤醒写入任务 |
| `void writerStep()` | 写入任务的一步：向闪存写一到两页，或在地面上擦除一次 |
| `requestManualStart()` / `requestManualStop()` | 手动记录（控制台 `k` → `r`） |
| `State getState()` / `bool isRecording()` | `Off`、`Idle`、`Recording`、`Stopping`（先把队列写完，再记录 END） |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | 供控制台使用 |
| `void handleHostCommand(const char*)` | `bb list`、`bb get <n> [波特率]`——供 `tools/blackbox.py` 使用（在 USB CDC 上速率没有任何影响） |

各平台不同的部分：重启原因——`readResetCause()`；电池电压和电流——
ADC（S3 用 `analogReadMilliVolts`，STM32 用 12 位的 `analogRead`）；存储介质错误（`BlackBoxStorage::writeErrors`/`eraseErrors`）每秒一次以“存储介质：写入错误 …”
事件的形式进入日志，不会影响飞行。

## `BlackBoxStorage`

**文件：** `telemetry/BlackBoxStorage.h` · **依赖：** `IFlashRegion`

由 4 KB 扇区组成的环：头部和飞行列表在 `begin()` 时根据扇区头得到（第一遍读取每个扇区的头并记住有效的，第二遍只处理这些有效的：空白区域只读一次）；`openFlight()`/`append()`/`flush()`/`closeFlight()`——按页写入（每条记录带 CRC-8）；`eraseStep(target, protect, allowErase)`——在头部之前核对/擦除的一步：垃圾数据——总是擦除，飞行记录——整段擦除，并且只在空闲空间小于 `target` 时；`protect` 永远不会被触碰。

## `BlackBoxRing`、`BlackBoxFormat`

`BlackBoxRing` 是任务/核心之间受 `Rtos::CriticalSection` 保护的记录字节队列；溢出时丢弃最旧的。`BlackBoxFormat`——扇区头、记录的类型和结构、模式字符串（大小由 `static_assert` 核对）、CRC-8 和 CRC-32。

---

## `Mavlink`（编解码器）

**文件：** `telemetry/MavlinkCodec.h` · **类别：** namespace · **依赖：** 无（可移植）

不依赖生成库的 MAVLink 2：按 MAVLink 的顺序打包字段（已与 pymavlink 核对）、CRC-16/MCRF4XX + `CRC_EXTRA`、去除末尾的零。

| 实体 | 说明 |
|---|---|
| `Msg::*` | 标识符：HEARTBEAT、SYS_STATUS、SET_MODE、PARAM_*、GPS_RAW_INT、ATTITUDE、GLOBAL_POSITION_INT、SERVO_OUTPUT_RAW、MISSION_REQUEST_LIST/COUNT、NAV_CONTROLLER_OUTPUT、RC_CHANNELS、REQUEST_DATA_STREAM、VFR_HUD、COMMAND_LONG/ACK、HOME_POSITION、STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | 消息的 `CRC_EXTRA`，−1 表示未知 |
| `crcAccumulate`、`crcCalculate` | X.25（与 mavlink 的 `crc_accumulate()` 相同） |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)`——按顺序写入字段 |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)`——带连续 `seq` 的 v2 帧 |
| `Message` | 接收到的消息：`msgid`、`sysid`、`compid`、负载（用零补全）、按偏移读取字段 |
| `Parser` | `bool feed(byte)` → `message()`；支持 v1 和 v2，v2 的签名会被跳过；`goodCount()`、`badCrcCount()`；`CRC_EXTRA` 未知的消息会被静默跳过 |

## `MavlinkModes`

**文件：** `telemetry/MavlinkTelemetry.h` · **类别：** namespace

| 函数 | 说明 |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | ArduPlane 的模式编号：MANUAL 0，STABILIZE→FBWA 5，ALT_HOLD→FBWB 6，ACRO 4，CRUISE 7，LOITER 12，RTH→RTL 11，AUTO_TAKEOFF/LAUNCH→TAKEOFF 13，AUTO_LAND→AUTO 10，SOARING→THERMAL 24，RESCUE→STABILIZE 2；failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | 反向转换，用于地面站发来的命令；AUTO、CIRCLE、GUIDED——`false` |
| `isAutonomous(mode)` | HEARTBEAT 中的 `AUTO_ENABLED` 标志 |

## `MavlinkTelemetry`

**文件：** `telemetry/MavlinkTelemetry.h` · **依赖：** `IUartPort`、`FlightController`、`Autopilot*`、`LoopStats*`

通过无线电模块向 QGroundControl / Mission Planner 提供遥测（飞行器为
`MAV_TYPE_FIXED_WING`、`MAV_AUTOPILOT_ARDUPILOTMEGA`）。用于 STM32
（UART4），它没有 Wi-Fi。

| 方法 | 说明 |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | 打开端口，发出消息“OpenPlane online” |
| `void update()` | 在飞行循环中调用：解析收到的数据（每个节拍 ≤ 128 字节）、事件消息，每个节拍最多 2 帧 |
| `void statusText(severity, text)` | 发送到地面站的消息流（4 行的队列，每行最多 50 个字符） |
| `isGcsConnected()` | 最近 3 s 内收到过地面站的 HEARTBEAT |
| `getSentFrames()`、`getDeferredFrames()`、`getParser()` | 诊断 |
| `static const char* paramName(uint8_t)` | `RLL_KP`、`RLL_KI`、`RLL_KD`、`PTCH_KP`、`PTCH_KI`、`PTCH_KD` |

各数据流（Hz）：ATTITUDE 10；GLOBAL_POSITION_INT、VFR_HUD 5；GPS_RAW_INT、
RC_CHANNELS、SERVO_OUTPUT_RAW、NAV_CONTROLLER_OUTPUT 2；HEARTBEAT、SYS_STATUS 1；
HOME_POSITION 0.2。只有当 `availableForWrite()` 容得下一帧时才会发送
——否则等到下一个节拍（循环从不被阻塞）。

收到的内容：地面站的 HEARTBEAT；PARAM_REQUEST_LIST / READ / SET（PID 直接进入自动驾驶仪，取值 0..100，不保存）；SET_MODE 和 COMMAND_LONG
`DO_SET_MODE`（176）——模式保持到下一次拨动开关；`COMPONENT_ARM_DISARM`
（400）——**DENIED**；`REQUEST_MESSAGE`（512）——插队发送一个数据流；
MISSION_REQUEST_LIST——返回 MISSION_COUNT 0，`mission_type` 与请求相同。用外部解码器检查数据流——`tools/check_mavlink.py`（pymavlink）。
