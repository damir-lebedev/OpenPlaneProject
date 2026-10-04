# AUTOPILOT / feedback — 反馈回路（雏形）

> 🌐 本页是[俄语原文](../../../reference/feedback.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

> ⚠️ **雏形，未接入固件。**`FlightController`、`Autopilot` 和 `main.cpp`
> 都没有包含这些头文件。它们靠闭环仿真（`test/test_feedback`，在 PC 和板子上都能运行）以及原生单元测试来验证。接入计划见
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#接入计划)。

思路：不再使用系数只适用于一种速度的角度 PID，而是使用一个闭合在
**飞机响应**上的控制器：它带有在飞行中学习的轴模型、失速保护，以及由传感器驱动的起飞/降落阶段。唯一的输入是 `FlightSnapshot`，唯一的输出是
`FeedbackOutput`。

所有模块都是纯头文件；`FeedbackModules.h` 用一行 include 就能全部引入。

---

## namespace `FeedbackConfig`

**文件：** `autopilot/feedback/FeedbackConfig.h`

回路的所有常量（接入时会迁移到 `Config.h`）。标有“прикидка”（“粗略估计”）的取值针对约 1 kg、翼展 1.2 m 的模型。`[AXIS_COUNT]` 数组按轴索引。

| 分组 | 常量 |
|---|---|
| 通用 | `GRAVITY = 9.80665`；各轴 `AXIS_ROLL = 0`、`AXIS_PITCH = 1`、`AXIS_YAW = 2`、`AXIS_COUNT = 3` |
| 速度 | `STALL_SPEED_MS = 8`、`REFERENCE_SPEED_MS = 14`、`ACCEL_FILTER_TAU_S = 0.3` |
| 空中/地面 | `AIRBORNE_HEIGHT_M = 3`、`AIRBORNE_CONFIRM_MS = 500`、`GROUND_STILL_MS = 2000`、`GROUND_ACCEL_TOLERANCE_G = 0.1` |
| 控制器 | `ANGLE_GAIN = {4, 4, 2}` 1/s、`MAX_RATE_DPS = {120, 60, 30}`、`RATE_TAU_S = {0.15, 0.20, 0.30}`、`RATE_INTEGRAL_GAIN = {2, 2, 1}`、`MAX_DEFLECTION_US = {400, 400, 400}`、`DAMPING_COMPENSATION = 0.5` |
| 舵面有效性 | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs、`EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`、`EFFECTIVENESS_MAX = {30, 15, 6}`、`RESPONSE_DELAY_MS = 40`、`RLS_FORGETTING = 0.995`、`ESTIMATOR_PERIOD_MS = 20`、`ESTIMATOR_PREFILTER_HZ = 2`、`MIN_EXCITATION_US = 30` |
| 失速 | `DECEL_WARN_MS2 = 2`、`DECEL_CONFIRM_MS = 300`、`LOW_ENERGY_PITCH_DEG = 5`、`NOSE_DROP_RATE_DPS = 60`、`WING_DROP_RATE_DPS = 120`、`STALL_NOSE_UP_COMMAND_US = 50`、`LOW_EFFECTIVENESS_RATIO = 0.35`、`LOW_SPEED_MARGIN = 1.25`、`LOW_SPEED_EXIT_MARGIN = 1.5`、`LOW_ENERGY_THROTTLE_PERCENT = 80`、`LOW_ENERGY_MAX_PITCH_DEG = 5`、`STALL_THROTTLE_PERCENT = 100`、`STALL_MAX_PITCH_DEG = −5`、`STALL_MAX_BANK_DEG = 10`、`STALL_AILERON_LIMIT_US = 150`、`RECOVERY_HOLD_MS = 1000` |
| 起飞 | `TAKEOFF_HAND_LAUNCH = false`、`TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`、`TAKEOFF_THROTTLE_PERCENT = 100`、`LAUNCH_ACCEL_G = 1`、`LAUNCH_DETECT_MS = 50`、`ROTATE_SPEED_MS = 10`、`ROTATE_FALLBACK_MS = 1500`、`CLIMB_PITCH_DEG = 12`、`TAKEOFF_TARGET_ALTITUDE_M = 30`、`TAKEOFF_CLIMB_FALLBACK_MS = 10000`、`LAUNCH_TIMEOUT_MS = 8000`、`HEADING_HOLD_GAIN = 2` |
| 降落 | `APPROACH_SINK_RATE_MS = 1`、`APPROACH_THROTTLE_PERCENT = 25`、`APPROACH_BASE_PITCH_DEG = −3`、`APPROACH_MIN_PITCH_DEG = −10`、`APPROACH_MAX_BANK_DEG = 20`、`GO_AROUND_THROTTLE_PERCENT = 80`、`SINK_TO_PITCH_GAIN = 4`、`FLARE_HEIGHT_M = 2`、`FLARE_SINK_RATE_MS = 0.3`、`FLARE_MAX_PITCH_DEG = 8`、`TOUCHDOWN_ACCEL_G = 0.5`、`TOUCHDOWN_HEIGHT_M = 0.3`、`TOUCHDOWN_STILL_MS = 500`、`TOUCHDOWN_STILL_RATE_DPS = 5`、`ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**文件：** `autopilot/feedback/FeedbackMath.h` · **依赖：** `<math.h>`

| 函数 | 说明 |
|---|---|
| `float wrap180(float deg)` | 把角度归入 `(−180, 180]`：航向 350° 与 10° 的差是 −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | 限制在 `[−limit, limit]` |

---

## `FlightSnapshot`

**文件：** `autopilot/feedback/FlightSnapshot.h` · **类别：** struct

回路在一个周期内了解到的关于飞机的全部信息。符号采用航空符号。

| 分组 | 字段 |
|---|---|
| 时间/状态 | `timeUs`、`armed`、`linkLost` |
| 姿态 | `imuValid`、`rollDeg`、`pitchDeg`、`yawDeg`、`rollRateDps`、`pitchRateDps`、`yawRateDps`、`accelXg/Yg/Zg` |
| 高度 | `baroValid`、`altitudeM`（相对开机点）、`climbRateMs`、`heightAglValid`、`heightAglM`（未来的测距仪） |
| 速度 | `airspeedValid`、`airspeedMs`（未来的皮托管）、`gpsValid`、`groundSpeedMs` |
| 模式目标 | `stabilizationActive`（false = MANUAL：仅学习）、`targetRollDeg`、`targetPitchDeg` |
| 指令，µs | `stick*Us`——飞手的贡献；`command*Us`——实际送往舵面的最终结果 |
| 油门，% | `pilotThrottlePercent`、`throttlePercent`（实际送往电调） |
| 襟翼 | `flapsUs`、`flapsMoving` |

---

## `FeedbackOutput`

**文件：** `autopilot/feedback/FeedbackOutput.h` · **类别：** struct

| 字段 | 说明 |
|---|---|
| `float deflectionUs[3]` | 各轴的舵面偏转量，µs（符号同 `ControlCommand`） |
| `bool axisEnabled[3]` | `false`——该轴不受控制，舵面留给飞手 |
| `float throttleOverridePercent` | 飞行阶段给出的绝对油门；`< 0`——未设定 |
| `float throttleFloorPercent` | 油门下限（失速保护）；`< 0`——无 |
| `targetRollDeg`、`targetPitchDeg` | 经过限幅后的最终目标（用于调试） |
| `const char* reason` | 给日志/OLED 用的简短说明 |

---

## `PhaseTargets`

**文件：** `autopilot/feedback/PhaseTargets.h` · **类别：** struct

`TakeoffSequencer` 和 `LandingSequencer` 共用的输出——“做什么”，而不是“怎么做”。

| 字段 | 默认值 | 说明 |
|---|---|---|
| `active` | `false` | 该阶段当前在控制飞机 |
| `targetRollDeg`、`targetPitchDeg` | 0 | 目标 |
| `controlRoll`、`controlPitch` | `true` | `false`——不去动该轴（在轮子上时，俯仰由起落架决定） |
| `holdHeading`、`headingDeg` | `false`、0 | 用方向舵和前轮保持航向 |
| `throttlePercent` | −1 | −1——使用飞手的油门 |
| `reason` | `""` | 说明 |

---

## `SpeedEstimator`

**文件：** `autopilot/feedback/SpeedEstimator.h`

速度（空速 > GPS 地速 > 未知）以及由 IMU 得到的纵向加速度：
`dV/dt = g · (ax − sin θ)`，经低通滤波器 `ACCEL_FILTER_TAU_S` 处理——即使没有空速传感器，也能看出“速度在下降”。

| 方法 | 说明 |
|---|---|
| `void update(const FlightSnapshot&)` | 执行一步；距上一次调用 `dt ≤ 0` 或 `> 0.5 s`（第一次调用则相对于 `timeUs = 0`）——跳过 |
| `bool hasSpeed() const`、`float getSpeed() const`、`Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`、`float getAcceleration() const` | m/s²，“+”表示加速；没有 IMU 时——`hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`，限制在 `0.05..4`；没有速度时为 1 |

---

## `AirborneDetector`

**文件：** `autopilot/feedback/AirborneDetector.h`

判断飞机是否在空中：学习、累积积分和检测失速，只有在飞行中才有意义。

| 方法 | 说明 |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | 未 ARM——重置为“在地面”。状态改变的候选条件必须持续 `AIRBORNE_CONFIRM_MS`（起飞）或 `GROUND_STILL_MS`（降落） |
| `void force(bool)` | 显式设定（起飞/降落阶段了解实际情况） |
| `void reset()` | 在地面 |
| `bool isAirborne() const` | |

“像是在飞行”：测距仪或气压计测得的高度 > `AIRBORNE_HEIGHT_M`，或者速度 > `ROTATE_SPEED_MS`。
“像是在地面”：高度低，所有轴的角速度 < `TOUCHDOWN_STILL_RATE_DPS`，|a| ≈ 1g（± `GROUND_ACCEL_TOLERANCE_G`）。

---

## `ControlEffectivenessEstimator`

**文件：** `autopilot/feedback/ControlEffectivenessEstimator.h`

单个轴。模型：**角加速度 = b·舵量(t − 延迟) + a·ω + c**。
`b` 是舵面有效性（每 µs 产生的 °/s²，符号表示响应方向），`a` 是阻尼（1/s，通常 < 0），`c` 是恒定力矩（自动配平）。`b` 在参考速度下学习：
`b = b_ref · (V/V_ref)²`，`a = a_ref · V/V_ref`。估计采用带遗忘因子的递归最小二乘法（`λ = 0.995`，记忆约 4 s）。

| 方法 | 说明 |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | 调用 `reset()` |
| `void reset()` | θ = (prior, 0, 0)；协方差：b ± prior，a ± 5，c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | 每个周期调用：在 `ESTIMATOR_PERIOD_MS` 的区间内累积平均值，区间结束时——由陀螺仪读数之差求加速度、对指令做延迟处理、对两侧施加同一个低通滤波器、执行一步 RLS（前提是允许学习且存在激励） |
| `getEffectiveness()` | 当前速度下的 `b` |
| `getReferenceEffectiveness()` | 参考速度下的 `b` |
| `getDamping()` | 当前速度下的 `a` |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b`（`\|b\|` 很小时为 0） |
| `getEffectivenessSigma()` | 当前速度下 `b` 估计的 σ |
| `bool isConfident() const` | ≥ 50 步 RLS，`\|b\|` ≥ 最小值且 σ < 0.3·`\|b\|` |
| `getAngularAccel()` | 上一个区间内的加速度（调试用） |

特点：

- 区间长于 `MAX_GAP_MS = 200`（循环停顿过）——数据重新开始（延迟历史和滤波器被重置）。
- 只有在存在**激励**时才学习：16 个区间（约 0.3 s）内平均指令的摆幅 ≥
  `MIN_EXCITATION_US`；否则估计值保持不变。
- 每步之后的浮点数处理：保持 `P` 对称、对方差设上限（初始值的 ×10）、限制 `b`（`±EFFECTIVENESS_MAX`）和 `a`（`−40..5`）。

---

## `AxisModel`

**文件：** `autopilot/feedback/AdaptiveRateController.h` · **类别：** struct

供控制器使用的、关于某个轴响应特性的已知信息：`effectiveness`（b，默认 1）、`damping`（a，0——不补偿）、`bias`（c，0）。

---

## `AdaptiveRateController`

**文件：** `autopilot/feedback/AdaptiveRateController.h`

单轴控制器，分三级：

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| 方法 | 说明 |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | 积分、饱和状态和输出清零 |
| `float angleToRate(targetDeg, angleDeg) const` | 第 1 级（航向走最短路径） |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | 第 2–3 级，返回偏转量，µs |
| `getDesiredRate()`、`getIntegral()`、`getOutput()`、`isSaturated()` | 状态 |

不变量：`|b|` 不小于 `EFFECTIVENESS_MIN`（保留 b 的符号）；积分以 °/s 保存（`b` 改变时仍然正确），并且**不会朝限位方向累积**
（按上一步饱和的方向做 anti-windup）；`dt ≤ 0`——积分不变。

---

## `StallGuard`

**文件：** `autopilot/feedback/StallGuard.h`

防止速度丢失和失速的保护。等级为 `Level::{Normal, LowEnergy, Stall}`。

| 方法 | 说明 |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | 在地面或没有 IMU 时——重置为 Normal |
| `Level getLevel() const`、`const char* getLevelName() const` | `"OK"`、`"LOW_ENERGY"`、`"STALL"` |
| `const char* getReason() const` | 最近一次触发的迹象 |
| `float maxPitchDeg() const` | Stall：−5°，LowEnergy：5°，其他情况 90° |
| `float maxBankDeg() const` | Stall：10°，其他情况 180° |
| `float maxAileronUs() const` | Stall：150 µs，其他情况为 `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 为 100 %，LowEnergy 为 80 %，其他情况/无链路为 −1 |
| `void reset()` | Normal |

`StallGuard::ControlState`——`pitchEffectivenessKnown`、`pitchEffectiveness`
（俯仰轴 b 估计值的模）。

**LowEnergy** 的迹象：经确认（`DECEL_CONFIRM_MS`）的减速大于 `DECEL_WARN_MS2` 且俯仰 > 5°；速度 < `1.25·Vs`；对升降舵有效性的可靠估计 < 先验值的 35 %。**Stall** 的迹象：速度 < Vs；机头下沉速度超过 60 °/s 而升降舵“向上”> 50 µs；能量不足时机翼下坠速度超过
120 °/s，与副翼的作用相反。解除措施要在 `RECOVERY_HOLD_MS` 之后，并且只有在能量已经恢复时才进行（速度 ≥ `1.5·Vs`，没有速度传感器时——加速度 ≥ 0）。

---

## `TakeoffSequencer`

**文件：** `autopilot/feedback/TakeoffSequencer.h`

分阶段起飞。`State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`。示意图见 [ARCHITECTURE.md §7](../ARCHITECTURE.md#起飞与降落反馈回路未接入)。

| 方法 | 说明 |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | 活动阶段 → `Aborted`；目标被重置 |
| `void update(snapshot, speed, nowMs)` | 每个周期最多一次状态转换，然后给出新阶段的目标 |
| `void reset()` | → `Idle` |
| `getTargets()`、`getState()`、`getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

各阶段的目标：等待——油门 0，舵面留给飞手；滑跑——油门 100 %，机翼水平，不去动俯仰，保持起飞开始时锁定的航向；爬升——油门
100 %，机翼水平，俯仰 `CLIMB_PITCH_DEG`。手抛：纵向加速度 `ax − sin θ ≥ LAUNCH_ACCEL_G` 持续超过 `LAUNCH_DETECT_MS`。

---

## `LandingSequencer`

**文件：** `autopilot/feedback/LandingSequencer.h`

分阶段降落。`State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`。

| 方法 | 说明 |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`、`void reset()`、`update(snapshot, nowMs)` | 与起飞相同 |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout——失速保护在此阶段关闭 |
| `bool isOnGround() const` | Rollout / Complete |

下降和拉平时的俯仰由垂直速度误差决定：
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`，限制在
`[min, FLARE_MAX_PITCH_DEG]`；没有气压计时——使用基础角度。高度取自测距仪，否则取自气压计。接地判定：|a − 1g| 出现尖峰 ≥ 0.5g，或“高度很低且在
`TOUCHDOWN_STILL_MS` 内不再旋转”。着陆滑跑时，航向在接地的瞬间锁定。

---

## `FeedbackSupervisor`

**文件：** `autopilot/feedback/FeedbackSupervisor.h`

整个回路。拥有 `SpeedEstimator`、`AirborneDetector`、三个
`ControlEffectivenessEstimator`、三个 `AdaptiveRateController`、`StallGuard`、
`TakeoffSequencer` 和 `LandingSequencer`。

| 方法 | 说明 |
|---|---|
| `bool requestTakeoff()` | 仅限已 ARM、有链路、在地面；会取消降落 |
| `bool requestLanding()` | 仅限已 ARM、有链路、在空中；会取消起飞 |
| `void cancelPhase()` | 取消当前阶段 |
| `const FeedbackOutput& update(const FlightSnapshot&)` | 一个周期（顺序见 [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-反馈回路未接入)） |
| `getOutput()`、`isAirborne()`、`getSpeedEstimator()`、`getEstimator(axis)`、`getController(axis)`、`getStallGuard()`、`getTakeoff()`、`getLanding()` | 供日志和测试使用的状态 |
| `void printStatus(Print& out) const` | 一行总体状态 + 每个轴一行（`b ± σ`，`*`——可靠，`a`、`c`、`I`、输出） |

关键规则：

- **未 ARM**——所有轴关闭，`reason = "未 ARM"`；ARM/DISARM（新的一次飞行）会清除所有已学到的内容。
- **信号丢失**会取消各阶段；不干预油门（由固件的
  failsafe 起作用）。
- **离地**会重置各估计值和控制器（在轮子上“看到”的数据不能用）。
- 可靠的负 `b` 估计值**绝不**会进入控制器——该轴按先验模型工作，
  `reason` 中会给出警告“……对舵面的响应相反？请在地面上检查”。
- 地面上积分被冻结，起飞滑跑/着陆滑跑时的航向除外。
- 协调转弯（空中速度已知时）：在期望的俯仰角速度上加
  `+ g/V · sin φ · tg φ`，在偏航角速度上加 `g/V · sin φ`（坡度限制在 ±60°）。
- `reason` 的优先级：失速 > 能量不足 > 阶段 > 符号警告 >
  “增稳”/“手动（学习）”。
