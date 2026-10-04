# AUTOPILOT — 模式、导航、拨杆

> 🌐 本页是[俄语原文](../../../reference/autopilot.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

[← 参考](README.md)

自动驾驶仪接收飞手的摇杆、拨杆/旋钮（`PilotInputs`）和传感器，输出
**最终的舵面指令**（`getCommand()`）以及该模式的油门（`applyThrottle()`）。缺少所需的传感器时，模式会采取安全行为（舵面交给飞手或回到中立位），而不是崩溃。每种模式对飞手意味着什么，见 [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md)。目前飞行中只用过手动模式；自动驾驶仪已通过试验台、测试和闭环仿真（`test/native/test_sim`）验证。

---

## `AutopilotMode`、`Feature`、`Knob`

**文件：** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t`（无作用域——数值编码会出现在
`/api/setmode`、`/api/status` 的 JSON 以及 `MavlinkModes` 中）：

| 取值 | 编码 | 简称（OLED） | 要点 |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | 舵面 = 摇杆 |
| `MODE_STABILIZE` | 1 | STAB | 摇杆给出横滚/俯仰角度 |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | 由飞手油门触发的起飞程序 |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + 用升降舵控制高度 |
| `MODE_ACRO` | 4 | ACRO | 摇杆给出角速度 |
| `MODE_CRUISE` | 5 | CRZ | 航向 + 高度 + 自动油门 |
| `MODE_LOITER` | 6 | LOIT | 在开启模式的点上空盘旋 |
| `MODE_RTH` | 7 | RTH | 返航，在返航点上空盘旋 |
| `MODE_LAUNCH` | 8 | LNCH | 手抛起飞 |
| `MODE_AUTO_LAND` | 9 | LAND | 滑翔 + 拉平 |
| `MODE_SOARING` | 10 | SOAR | 不开电机，利用热气流 |
| `MODE_RESCUE` | 11 | RESQ | 机翼水平、机头朝上、给油门 |
| `MODE_COUNT` | 12 | | 边界（`setMode()` 会忽略 ≥ 该值的编码） |

`enum class Feature : uint8_t`——拨杆的功能：`FLAPS`、`AIRBRAKE`、
`AUTO_TRIM`、`TURN_COORDINATION`、`MOTOR_KILL`、`BEEPER`、`PAYLOAD_DROP`、
`GEOFENCE`、`HOME_RESET`、`CAMERA_STAB`、`COUNT`。

`enum class Knob : uint8_t`——旋钮：`STAB_GAIN`、`MAX_BANK`、
`CRUISE_SPEED`、`FLAPS`、`CAMERA_TILT`、`RATES`、`LOITER_RADIUS`、`COUNT`。

`namespace AutopilotNames`——`mode()`、`modeShort()`（≤ 5 个字符）、
`feature()`、`knob()`：用于日志、OLED、仪表盘和 MAVLink 的名称。

### `PilotInputs`

一个周期内拨杆和旋钮的状态。

| 成员 | 说明 |
|---|---|
| `bool has(Feature) const` | 该功能已打开 |
| `float knob(Knob) const` | 旋钮位置 −1…+1 |
| `bool isBound(Knob) const` | 该旋钮在绑定表中 |
| `float knobValue(Knob, min, default, max) const` | 换算成实际单位：中点为 `default`，两端为 `min`/`max`；未绑定时——`default` |

---

## `Binding`、`Bind`、`BindingCheck`

**文件：** `autopilot/ControlBinding.h` · 表——`config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`，
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`。表中的每一行都是
`namespace Bind` 的工厂函数（全部为 `constexpr`）：

| 工厂函数 | 含义 |
|---|---|
| `modes(ch, up, middle, down)`、`modes(ch, up, down)` | 模式选择拨杆（档位由 `PilotSwitches::zoneOf` 判定） |
| `mode(ch, m)` | 在通道 ≥ `SWITCH_ON_US` 期间叠加在上层的模式 |
| `feature(ch, f)` | 在通道 ≥ `SWITCH_ON_US` 期间生效的功能 |
| `knob(ch, k)` | 旋钮，`(us − 1500) / 500`，限制在 ±1 |

`namespace BindingCheck`——递归的 `constexpr` 函数（ESP32 内核按
C++11 编译）：`channelIsFree`、`channelsFree`、`channelsUnique`、
`modeSwitchCount`、`atMostOneModeSwitch`。它们用于 `Controls.h` 的
`static_assert`。

---

## `PilotSwitches`

**文件：** `autopilot/PilotSwitches.h` · **依赖：** `Autopilot*`、`RcChannelState`、绑定表

| 方法 | 说明 |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | 自定义的表（用于测试、仿真） |
| `explicit PilotSwitches(Autopilot* = nullptr)` | 使用 `Controls::BINDINGS` 表 |
| `void update(const RcChannelState&)` | 收集 `PilotInputs`，传给 `autopilot->setInputs()`；`setMode()`——**仅当拨杆的结果发生变化时**调用（从仪表盘/地面站设置的模式不会在每个周期被覆盖）。`FlightController` 只在链路正常时调用它 |
| `void printBindings() const` | 开机时向 Serial 输出布局：`SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (向上 / 中间 / 向下)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`、`"VrA (CH9)"`…… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 档：< 1500 / ≥ 1500；3 档：< 1250 / < 1750 / ≥ 1750 |
| `getInputs()`、`binding(i)` | 供遥测和测试使用 |

`Bind::mode` 的优先级高于 `Bind::modes`；在多个同时打开的 `Bind::mode` 中，最上面一行生效。

---

## `Autopilot`

**文件：** `autopilot/Autopilot.h` · **依赖：** `PidController`、`Navigation`、`AltitudeSpeedController`、`LaunchController`、`SoaringController`、`AutoTrim`、各传感器（均可为空）

### 生命周期

| 方法 | 说明 |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | 横滚/俯仰 PID：Kp 5、Ki 0.5、Kd 0.5，输出 ±500 µs |
| `bool begin()` | 加载配平；如果没有 IMU 或气压计，则返回 `false` 并给出提示 |
| `void setInputs(const PilotInputs&)` | 本周期的拨杆和旋钮（在 `update` 之前调用） |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | 每个周期一次：传感器（始终）→ 导航与返航点 → 在地面保存配平 → failsafe → 地理围栏 → 模式 → 转弯协调 → 自动配平 |
| `ControlCommand getCommand() const` | 最终的舵面指令（roll/pitch/yaw，µs） |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | 该模式的油门：`PILOT`——飞手油门；`AUTO`——自己的油门；`AT_LEAST`——不低于自己的油门（自动起飞）。ARM 和 `MOTOR_KILL` 由 `FlightController` 处理 |

### 模式

| 方法 | 说明 |
|---|---|
| `void setMode(AutopilotMode)` | 相同模式或 ≥ `MODE_COUNT`——不做任何事；否则重置 PID 和各状态机，目标 = 当前航向和高度，盘旋圆心 = 当前点（有 GPS 时），RTH——返航高度 |
| `getMode()`、`getModeName()` | 名称：信号丢失时为 `FAILSAFE_GLIDE` / `FAILSAFE_RTH`，否则为模式名 |
| `isFailsafeActive()`、`isFailsafeGliding()`、`isFailsafeReturning()` | 叠加在模式之上的 failsafe |
| `isAutoThrottle()`、`getThrottleCorrection()` | 该模式的油门（%，用于日志和仪表盘） |
| `getLaunchState()`、`getSoaringState()` | LAUNCH 和 SOARING 的状态机 |

### 输出与诊断

| 方法 | 说明 |
|---|---|
| `getRollCorrection()`、`getPitchCorrection()`、`getYawCorrection()` | 指令 − 摇杆，µs |
| `getDesiredRoll()`、`getDesiredPitch()`、`getTargetAltitude()` | 各目标值 |
| `const NavStatus& getNavStatus()` | GPS、返航点、位置、到返航点的距离/方位、航向与目标航向、导航所用速度、地理围栏、失速 |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | 气压计高度（相对开机点的米数） |
| `getInputs()`、`getAutoTrim()` | 供遥测使用 |
| `getImuSensor()` … `getAirspeedSensor()` | 各传感器（可能为 `nullptr`） |
| `getRollPid()`、`getPitchPid()`、`setPIDGains(...)` | PID（仪表盘、MAVLink 参数） |

### 内部机制

- `stabilize()`——角度 PID，D 项取自陀螺仪，乘以 `Knob::STAB_GAIN`；积分器只在 ARM 且误差 < `STAB_INTEGRATOR_ZONE_DEG` 时累积。
  `stabilizeOrManual()`——没有 IMU 时舵面交给飞手；`stabilizeOrNeutral()`——没有
  IMU 时回中立位（自动模式）。
- `imuReady()` = IMU 存在、可用且没有飞行前检查的问题。
- 导航所用速度：皮托管 → GPS → `NAV_ASSUMED_SPEED_MS`。
- `looksLanded()`——根据气压计判断贴近地面，几乎没有垂直速度，速度低于皮托管/GPS 的阈值：只有此时才会把配平写入闪存。
- Failsafe：有 GPS 和返航点时——带动力 RTH，否则滑翔；已经开始的 RTH
  不会因 GPS 短暂丢失而放弃。

---

## `Geo`、`Guidance`、`GeoPoint`

**文件：** `autopilot/Navigation.h`

以米为单位的局部“北/东”平面（等距圆柱投影——在千米量级上，误差只是百分之几以下的一小部分）。

| 函数 | 说明 |
|---|---|
| `Geo::wrap180`、`Geo::wrap360` | 角度归一化 |
| `Geo::offsetNE(a, b, north, east)`、`distance(a, b)`、`bearing(a, b)` | 偏移量、距离、方位 0..360 |
| `Geo::moved(a, north, east)` | 带偏移量的点 |
| `Geo::fromGps(GpsData)` | 由 GPS 得到 `GeoPoint` |
| `Guidance::rollForCourse(target, course, bankLimit)` | 针对航向误差的坡度（`NAV_COURSE_GAIN`），带限幅 |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | 引向圆周的向量场航向（`LOITER_CONVERGENCE`） |
| `Guidance::orbitBankDeg(speed, radius)` | 盘旋时的前馈坡度：atan(V²/(g·R)) |

## `AltitudeSpeedController`

**文件：** `autopilot/AltitudeSpeedController.h`——TECS-lite。

| 方法 | 说明 |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | 期望垂直速度 = `NAV_ALT_GAIN`·误差（≤ `NAV_MAX_CLIMB/SINK`）；俯仰 = 前馈 asin(Vz/V) + 针对 Vz 误差的 PI，限制在 `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 之内 |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | 有皮托管时——围绕 `cruisePct` 对空速做 PI；没有时——`cruisePct`；再加上为所需爬升量增加的 `THROTTLE_PER_CLIMB_PCT` |
| `reset()`、`getWantedClimb()` | |

## `LaunchController`

**文件：** `autopilot/LaunchController.h`

`State`：`IDLE → READY`（油门已推起）`→ THROWN`（过载 > `LAUNCH_ACCEL_G`
且持续超过 `LAUNCH_ACCEL_TIME_MS`）`→ CLIMB`（经过 `LAUNCH_MOTOR_DELAY_MS`：电机启动，俯仰 `LAUNCH_CLIMB_PITCH_DEG`）`→ DONE`（`LAUNCH_CLIMB_MS` 或
`LAUNCH_ALTITUDE_M`）。抛出之前动摇杆——取消。方法：`update(...)`、
`reset()`、`getState()`、`motorOn()`、`pitchTargetDeg()`、`stateName()`。

## `SoaringController`

**文件：** `autopilot/SoaringController.h`

`State`：`GLIDE ⇄ THERMAL`（升降速度计 > `SOAR_THERMAL_CLIMB_MS` 且持续超过
`SOAR_THERMAL_CONFIRM_MS` / 在 `SOAR_EXIT_WINDOW_MS` 内的平均值 < `SOAR_EXIT_CLIMB_MS`），
`→ MOTOR_CLIMB`（低于 `SOAR_MIN_ALTITUDE_M`，直到
`SOAR_MAX_ALTITUDE_M`），`→ RETURN`（超出 `SOAR_MAX_DISTANCE_M`，直到该距离的 70 %）。方法：`update(climb, alt, distHome, dt, now)`、`reset(now)`、
`getState()`、`motorOn()`、`getAverageClimb()`、`stateName()`。

## `AutoTrim`

**文件：** `autopilot/AutoTrim.h` · 存储——`Preferences`（NVS / STM32 闪存），命名空间 `"autotrim"`

| 方法 | 说明 |
|---|---|
| `void load()` | 从 NVS 读取配平（没有则为 0） |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += 指令 · `AUTOTRIM_RATE` · dt，上限为 ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | 有变化时写入（由 `Autopilot` 在地面 DISARM 之后调用） |
| `reset()`、`getRoll()`、`getPitch()` | |

---

## `PidController`

**文件：** `autopilot/PidController.h` · **依赖：** `Config`（名义 `dt`）

| 方法 | 说明 |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`、`getKp/Ki/Kd()` | 各系数 |
| `setLimits(minOut, maxOut)` | 输出限幅（默认 ±500） |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | 输出，限制在 `[min, max]` 内 |
| `void reset()` | 积分器清零，`dt` 从“现在”开始计时 |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (来自传感器的速率！)
out = constrain(P + I + D, min, max)
```

D 项取自被测量量的变化率（陀螺仪 °/s），而不是误差的导数：没有微分噪声，设定值改变时也不会出现跳变。`dt` 取自
`micros()`；`reset()` 之后的第一次调用，或停顿超过 0.1 s 时，使用名义值
`LOOP_PERIOD_MS`。
