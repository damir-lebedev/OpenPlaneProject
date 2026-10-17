# CONTROL 与 COORDINATION — 混控器、油门、ARM、输出、协调器

> 🌐 本页是[俄语原文](../../../reference/control.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

CONTROL 层是基于数据的逻辑，不涉及 UART、PWM 或 Wi-Fi。
`FlightController`（COORDINATION）是唯一把所有下层汇总到一个周期里的类。

---

## `ControlCommand`

**文件：** `control/ControlCommand.h` · **类别：** struct

以**物理符号**表示的舵面指令，单位为偏转量 µs（±500 = 全行程）。是摇杆、自动驾驶仪和混控器共用的语言。

| 字段 | “+”表示 |
|---|---|
| `int16_t roll` | 向右横滚（右副翼向上，左副翼向下） |
| `int16_t pitch` | 机头朝上（升降舵向上） |
| `int16_t yaw` | 机头向右（方向舵和前轮向右） |
| `int16_t flaps` | 襟翼向下（两侧副翼都向下）；“−”——减速板（两侧都向上） |

所有字段默认为 0。

---

## `FlightOutputState`

**文件：** `control/FlightOutputState.h` · **类别：** struct

期望的输出脉冲，PWM µs。默认值：舵面中立位，油门为 `PWM_MIN`。

`throttle` 是任一方向上的油门大小（1000..2000 = 0..100%）；由 `FlightOutputs` 把它和 `motorReverse` 一起换算成电调脉冲（通过 `EscSignal`）。

| 字段 | 默认值 |
|---|---|
| `aileronLeft`、`aileronRight`、`elevator`、`rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false`——电机正转 |
| `bool escPulsesOff` | `false`；`true`——完全不给电调脉冲（电机提示，仅在未解锁时） |
| `aux1` | `PAYLOAD_CLOSED_US`——载荷抛投装置关闭 |
| `aux2` | `PWM_CENTER`——相机 |

---

## `FlapsController`

**文件：** `control/FlapsController.h` · **依赖：** `Config`

襟翼的平滑放出/收起：位置向目标移动（目标可以是任意值——来自拨杆的襟翼、来自旋钮的襟翼、向上的减速板），速度不超过在 `FLAPS_TRANSITION_MS` 内走完全行程 `FLAPS_DEPLOYED_US`。时间以参数传入。

| 方法 | 说明 |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | 朝目标前进一步；返回当前位置，µs（+ 向下，− 向上） |
| `int16_t getPosition() const` | 当前位置 |

不变量：

- **第一次调用**会把位置直接设为目标——开机时襟翼不会在桌面上
  “伸出来”。
- 时间步长被限制为 `MAX_STEP_MS = 20`：长时间停顿（failsafe、校准）之后，襟翼不会在一个周期内跳到目标位置。

---

## `ControlMixer`

**文件：** `control/ControlMixer.h` · **依赖：** `RcInput`、`RcChannelState`、`FlapsController`、`ControlCommand`、`FlightOutputState`、`Config`、`Channels`

分两步实现的空气动力学逻辑。拥有 `FlapsController`。

| 方法 | 说明 |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll`（2000 = 向右）；CH2 → `pitch`，**符号相反**（2000 = 推离自己 = 机头朝下）；CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | 襟翼的目标由 `FlightController` 选择（减速板 → 襟翼拨杆 → 旋钮），这里负责平滑的行程 |
| `FlightOutputState mix(const ControlCommand& c) const` | 指令 → PWM。横滚/俯仰/偏航受行程（`*_MAX_US`）限制；副翼：左 = `flaps + roll`，右 = `flaps − roll`（向下 = “+”）；PWM = `1500 ± 偏转量`，符号取自 `Config::*_REVERSED`，襟翼的符号取自 `FLAPS_REVERSED`，限制在 1000..2000。`throttle` 不在此填写 |
| `int16_t getFlaps() const` | 当前的襟翼位置，µs |

襟副翼：放出时两侧副翼各下垂 `FLAPS_DEPLOYED_US`（新的“中立位”），横滚在此基础上叠加。满横滚时，下垂一侧的副翼比上抬一侧更早到达行程末端——
这相当于副翼差动。

---

## `ThrottleManager`

**文件：** `control/ThrottleManager.h` · **依赖：** `RcInput`、`RcChannelState`、`Config`、`Channels`

| 方法 | 说明 |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | CH3 的油门，限制在 1000..2000；信号丢失时——`FAILSAFE_THROTTLE` |

它不了解 ARM 和自动驾驶仪——它们的修正由 `FlightController` 施加。

---

## `EscSignal`

**文件：** `control/EscSignal.h` · **类别：** namespace · **依赖：** `Config`

油门（1000..2000 = 0..100%）和方向 → 电调引脚上的脉冲。

| 函数 | 说明 |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL`（3D 模式的 BLHeli 80A）：0% → `ESC_NEUTRAL_US`（1500），正转 1500..2000，反转 1500..1000。单向电调：1000..2000 原样输出，忽略 `reverse`。超出 1000..2000 的油门会被截断 |
| `uint16_t stopUs()` | “电机停止”的脉冲（双向电调为 1500，普通电调为 1000） |

只做换算，不做保护：什么时候可以改变方向由 `MotorReverse` 决定，反转油门上限由 `FlightController` 负责。

---

## `MotorReverse`

**文件：** `control/MotorReverse.h` · **依赖：** `Config`

电机方向跟随 `Feature::MOTOR_REVERSE` 开关（默认 SwD）——**只在未解锁时**。

| 方法 | 说明 |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | 未解锁——方向立即跟随开关（串口打印 `MotorReverse: мотор НАЗАД` / `вперёд`——“电机反转”/“正转”）。已解锁——开关不起作用，只打印一次“реверс НЕ переключён”（“反转未切换”），新方向等到 DISARM。`ESC_BIDIRECTIONAL = false` 时永远不会启用反转 |
| `bool isReversed() const` | 电机反转 |
| `bool isPending() const` | 开关已在另一位置，方向等待 DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | 拒绝 ARM 的原因：距上次改变方向不到 `REVERSE_SETTLE_MS`（电机可能还在转）；否则为 `nullptr` |

---

## `ArmingManager`

**文件：** `control/ArmingManager.h` · **依赖：** `Autopilot`（可为空）、`RcChannelState`、`Config`、`Channels`

ARM 由单独的拨杆 SwA（CH5）完成。状态机见
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager)。

| 方法 | 说明 |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | 没有自动驾驶仪时只检查油门 |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | failsafe 时——什么也不做（failsafe 帧中的拨杆并不反映飞手的操作）。拨杆 OFF → DISARM，`switchSeenOff = true`。OFF→ON 的切换 → 检查 → ARM 或拒绝。`blockReason`——外部的拒绝原因（来自 `FlightController`：`MotorReverse::armBlockReason()`） |
| `bool isArmed() const` | 是否已 ARM |
| `const char* getLastRefusalReason() const` | 上一次拒绝的原因，或 `nullptr`；关闭拨杆时清除 |

`checkFailureReason(rc)`——ARM 的检查项：

| 条件 | 拒绝原因 |
|---|---|
| 油门 ≥ `THROTTLE_LOW_US` | “油门不在最低位” |
| 除 MANUAL 外的任何模式，IMU 存在但无应答 | “IMU 无应答……” |
| 除 MANUAL 外的任何模式，IMU 的飞行前检查有问题 | `ImuSensor::getPreflightProblem()` 的文字 |
| 需要高度的模式（`needsAltitude`：ALT_HOLD、CRUISE、LOITER、RTH、AUTO_LAND、SOARING），气压计存在但无应答 | “气压计无应答……” |
| 其他检查都通过，但传入了 `blockReason` | `blockReason`（例如“电机方向刚刚改变……”） |

构建中不存在的传感器（`nullptr`）不会阻止 ARM；在 MANUAL 下，飞机即使完全没有传感器也能 ARM。GPS 定位有意不列入检查项：没有 GPS 时，导航模式的行为是安全的（原地盘旋），而 GPS 搜到卫星之后会记录返航点。

不变量：开机时拨杆已在 ON 位置不会 ARM；每次 OFF→ON 切换只尝试一次；信号丢失不会解除 ARM。

---

## `FlightOutputs`

**文件：** `control/FlightOutputs.h` · **依赖：** `IBoard`、`FlightOutputState`、`EscSignal`、`Config`

唯一了解 PWM 输出集合及其顺序的类。所有输出都在一张表中描述；
`begin()`、`write()`、状态和自检都循环遍历这张表。

### `FlightOutputs::OutputInfo`

| 字段 | 说明 |
|---|---|
| `const char* key` | JSON/日志中的名称（`aileronLeft`、……、`esc`、`rudder`、`aux1`、`aux2`） |
| `const char* label` | 给人看的名称 |
| `int16_t pin` | 引脚编号；`-1`——未引出。使用 `int16_t` 是因为 STM32 上模拟引脚的编号是 `0xC0 + N` |
| `bool required` | 没有它飞机就不能飞（方向舵是可选的） |
| `uint16_t FlightOutputState::* field` | 指向状态字段的指针 |

| 方法 | 说明 |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | 表中的一行；顺序 = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | 对每个输出执行 `attach(PWM_MIN, PWM_MAX)`，打印状态和电调类型；所有**必需**输出都拿到通道时返回 `true` |
| `bool isAttached(uint8_t ch) const` | 该输出已连接（索引越界 → `false`） |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | 输出引脚上的脉冲：按表取字段，电调则经过 `EscSignal::pulseUs(throttle, motorReverse)`，`escPulsesOff` 时为 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | 在每个已引出的引脚上，把测得的脉冲与预期值比较；偏差 ≤ 15 µs 时为“OK” |
| `void write(const FlightOutputState&)` | 写入所有输出并记住该状态；`escPulsesOff` 时电调调用 `IServoOutput::stopPulses()` |
| `void setFailsafe()` | 舵面回中（`FAILSAFE_*`），油门 `FAILSAFE_THROTTLE`（双向电调引脚上为 1500）；AUX 保持原样（失去信号不会投放载荷） |
| `void setBuzzer(bool on)` | 板载蜂鸣器（`IBoard::setBuzzer`） |
| `const FlightOutputState& getLastState() const` | 最近一次写入的状态 |

添加一个输出：表中增加一行 + 在 `FlightOutputState` 中增加一个字段 + 在
`ServoChannel` 中增加一个索引（+ 在 `Esp32Board` 中增加引脚和 LEDC 通道）。

---

## `FlightController`

**文件：** `control/FlightController.h` · **层：** COORDINATION ·
**依赖：** `IBusReceiver`、`ControlMixer`、`ThrottleManager`、`ArmingManager`、`FlightOutputs`、`Autopilot*`、`PilotSwitches*`、`Beeper`、`MotorReverse`、`GroundCalibration`、`SignalPlayer`

控制循环唯一的协调者：它自己不解析 UART，不碰 PWM，也不计算混控——
只是按正确的顺序调用其他部分。详细的图见 [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-控制周期flightcontrollerupdate)。

| 方法 | 说明 |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | 没有自动驾驶仪——纯手动控制；没有拨杆——只有摇杆 |
| `void begin()` | `outputs.setFailsafe()`、`receiver.begin()` |
| `void update()` | 一个周期（见下文） |
| `bool isReceiverFailsafe() const` | 信号已丢失 |
| `const IBusReceiver& getReceiver() const` | 用于日志（帧计数器、信号丢失的原因） |
| `bool isArmed() const`、`const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | 最近一次写入输出的状态 |
| `const RcChannelState& getRcState() const` | 各通道 |
| `const FlightOutputs& getOutputs() const` | 输出表和 `attached` |
| `int16_t getFlapsUs() const` | 襟翼位置 |
| `const PilotSwitches* getSwitches() const`、`const PilotInputs& getInputs() const` | 本周期的拨杆和旋钮 |
| `bool isLostModelBeeping() const` | “我在这里”蜂鸣器正在工作 |
| `const MotorReverse& getMotorReverse() const` | 电机方向和等待中的反转 |
| `void toggleGroundCalibration()` | 控制台 `c`：飞机地面校准——开始（仅在未解锁时），正在进行时——取消 |
| `void playSignal(uint8_t count)` | 控制台 `e`：一次提示（未解锁时也用电机） |
| `const GroundCalibration& getCalibration() const`、`const SignalPlayer& getSignals() const` | 供 OLED、日志和黑匣子使用 |

`update()` 的顺序：

1. `receiver.update()`；`failsafe = receiver.isSignalLost()`；
2. 链路正常时——`switches->update(rc)`（模式、功能、旋钮）；
3. `pilotThrottle = throttle.update(rc, failsafe)`；
4. 摇杆 `mixer.fromSticks(rc)`（链路正常时）× `Knob::RATES`；襟翼
   `mixer.updateFlaps(target)`：`AIRBRAKE` → −`AIRBRAKE_US`，`FLAPS` →
   `FLAPS_DEPLOYED_US`，`Knob::FLAPS` → 平滑变化，信号丢失时——0；
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)`——**始终**执行；
   反转时传入 `PWM_MIN` 代替 `pilotThrottle`（对自动驾驶仪来说电机是关着的）；
6. 校准：校准期间拨一下 ARM 即取消；未解锁且有信号时——`CAL_SWITCH` 手势；执行一步 `GroundCalibration::update()`；它的提示和自动起飞拒绝的提示交给 `SignalPlayer`；蜂鸣器：`Beeper::update(BEEPER, armed, failsafe, now)` 或提示灯；
7. 信号丢失 → `applyLinkLoss()` 并退出本周期；
8. `arming.update(rc, false, 原因)`——校准或提示进行中时拒绝 ARM（拨 ARM 取消校准的那个周期也拒绝），否则用 `reverse.armBlockReason(now)`；然后 `reverse.update(MOTOR_REVERSE, armed, now)`——DISARM 时方向在同一个周期内改变；
9. `command = autopilot->getCommand()`（没有自动驾驶仪时用摇杆），襟翼——使用自己的；提示的点头加到升降舵和方向舵上；
10. `output = mixer.mix(command)`；`output.motorReverse = reverse.isReversed()`；正转——`output.throttle = autopilot->applyThrottle(pilotThrottle)`，上限 `THROTTLE_LIMIT_PCT` × 旋钮 `Knob::THROTTLE_LIMIT`；反转——飞手油门，上限 `min(REVERSE_THROTTLE_LIMIT_PCT, 正转上限)`；允许的范围拉伸到整个摇杆行程；
11. 未 ARM 或 `MOTOR_KILL` → `throttle = PWM_MIN`（最后执行）；未解锁时电机提示设置 `escPulsesOff`；
12. AUX1——载荷（`PAYLOAD_DROP`），AUX2——相机（`Knob::CAMERA_TILT`，`CAMERA_STAB` 会减去俯仰）；`AUX1_REVERSED`/`AUX2_REVERSED` 把脉冲镜像。
13. `outputs.write(output)`。

`applyLinkLoss()`：如果自动驾驶仪处于 failsafe（已 ARM：RTH 或滑翔）——舵面和油门按自动驾驶仪的指令输出（襟翼平滑收起，`MOTOR_KILL` 和反转让电机停转，AUX 保持原样）；否则调用 `outputs.setFailsafe()`，提示进行中时——回中并点头、暂停电调脉冲（遥控器被放下了——正在校准）。

---

## `Beeper`

**文件：** `control/Beeper.h` · **依赖：** `Config`

| 方法 | 说明 |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | 蜂鸣器状态：如果有 `Feature::BEEPER` 或者“模型丢失”（未 ARM，且无信号的时间超过 `LOST_MODEL_BEEP_DELAY_MS`），则以 2 Hz 鸣叫 |
| `bool isLostModel() const` | “到草丛里找我”模式 |

---

## `GroundCalibration`

**文件：** `control/GroundCalibration.h` · **依赖：** `Config`、`RcChannelState`、`ImuSensor`、`ImuOrientation`

飞机地面校准：第 1 步——抬起机尾，机身水平（水平基准，`ImuOrientation::withLevel()`），第 2 步——停在轮子上（停机姿态）。静止 `CAL_STILL_MS` 后自动记录该步骤；第 2 步等待飞机被挪动，或等 `CAL_PARK_WAIT_MS`。只整体保存（`ImuSensor::setGroundCalibration()`），保存前按 `LANDING_GEAR` 检查停机姿态。不阻塞循环。

`Stage`：`IDLE`、`LEVEL`、`PARK`、`DONE`、`FAILED`。

| 方法 | 说明 |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | 手势：`CAL_SWITCH` 在 `CAL_SWITCH_WINDOW_MS` 内上下拨 `CAL_SWITCH_FLIPS` 次，油门在最低；第一次读取和油门不在最低都会重新计数 |
| `void forgetSwitch()` | ARM 或没有信号——重新计数 |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | 开始（提示 1）；没有 IMU 或不支持——出错（提示 3） |
| `void cancel(const char* why)` | 取消，不提示 |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | 一步：ARM 会取消；提示进行中时不记录任何步骤；超过 `CAL_STEP_TIMEOUT_MS`——出错 |
| `isActive()`、`getStage()`、`getStageSinceMs()`、`getFailure()`、`getParkPitch()`、`getParkRoll()` | 状态和结果 |
| `uint8_t takeSignal()` | 要播放几次提示：1——已开始 / 第 1 步，2——完成，3——出错（只取一次） |
| `static const char* stageShort(Stage)` | `CAL1`、`CAL2`、`CALOK`、`CALER`——供 OLED 使用 |

---

## `SignalPlayer`

**文件：** `control/SignalPlayer.h` · **依赖：** `Config`

给飞手的提示——是从一组提示开始时起算的时间函数。一次提示：升降舵和方向舵点头（先向上 `SIGNAL_NOD_US`，再向下，各 `SIGNAL_NOD_MS`），未解锁时电调脉冲暂停 `SIGNAL_ESC_PAUSE_MS`（之后电调播放就绪提示音），点头期间蜂鸣器灯亮；下一次提示在 `SIGNAL_PERIOD_MS` 之后。

| 方法 | 说明 |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | 一组提示；`motor`——是否允许动电调（还要看 `SIGNAL_MOTOR`） |
| `void stop()` | 静默 |
| `bool isBusy(uint32_t nowMs) const`、`uint8_t getCount(uint32_t nowMs) const` | 一组提示正在进行（期间拒绝 ARM） |
| `bool escPulsesOff(uint32_t nowMs) const` | 现在是否停掉电调脉冲 |
| `int16_t nodUs(uint32_t nowMs) const` | 舵面点头，µs（`SIGNAL_SURFACES`） |
| `bool light(uint32_t nowMs) const` | `PIN_BUZZER` 上的蜂鸣器 / LED |
