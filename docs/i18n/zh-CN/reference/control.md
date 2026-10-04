# CONTROL 与 COORDINATION — 混控器、油门、ARM、输出、协调器

> 🌐 本页是[俄语原文](../../../reference/control.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

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

| 字段 | 默认值 |
|---|---|
| `aileronLeft`、`aileronRight`、`elevator`、`rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
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
| `FlightOutputState mix(const ControlCommand& c) const` | 指令 → PWM。横滚/俯仰/偏航受行程（`*_MAX_US`）限制；副翼：左 = `flaps + roll`，右 = `flaps − roll`（向下 = “+”）；PWM = `1500 ± 偏转量`，符号取自 `Config::*_REVERSED`，限制在 1000..2000。`throttle` 不在此填写 |
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

## `ArmingManager`

**文件：** `control/ArmingManager.h` · **依赖：** `Autopilot`（可为空）、`RcChannelState`、`Config`、`Channels`

ARM 由单独的拨杆 SwA（CH5）完成。状态机见
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager)。

| 方法 | 说明 |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | 没有自动驾驶仪时只检查油门 |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | failsafe 时——什么也不做（failsafe 帧中的拨杆并不反映飞手的操作）。拨杆 OFF → DISARM，`switchSeenOff = true`。OFF→ON 的切换 → 检查 → ARM 或拒绝 |
| `bool isArmed() const` | 是否已 ARM |
| `const char* getLastRefusalReason() const` | 上一次拒绝的原因，或 `nullptr`；关闭拨杆时清除 |

`checkFailureReason(rc)`——ARM 的检查项：

| 条件 | 拒绝原因 |
|---|---|
| 油门 ≥ `THROTTLE_LOW_US` | “油门不在最低位” |
| 除 MANUAL 外的任何模式，IMU 存在但无应答 | “IMU 无应答……” |
| 除 MANUAL 外的任何模式，IMU 的飞行前检查有问题 | `ImuSensor::getPreflightProblem()` 的文字 |
| 需要高度的模式（`needsAltitude`：ALT_HOLD、CRUISE、LOITER、RTH、AUTO_LAND、SOARING），气压计存在但无应答 | “气压计无应答……” |

构建中不存在的传感器（`nullptr`）不会阻止 ARM；在 MANUAL 下，飞机即使完全没有传感器也能 ARM。GPS 定位有意不列入检查项：没有 GPS 时，导航模式的行为是安全的（原地盘旋），而 GPS 搜到卫星之后会记录返航点。

不变量：开机时拨杆已在 ON 位置不会 ARM；每次 OFF→ON 切换只尝试一次；信号丢失不会解除 ARM。

---

## `FlightOutputs`

**文件：** `control/FlightOutputs.h` · **依赖：** `IBoard`、`FlightOutputState`、`Config`

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
| `bool begin()` | 对每个输出调用 `attach(PWM_MIN, PWM_MAX)`，打印状态；所有**必需**输出都拿到通道时返回 `true` |
| `bool isAttached(uint8_t ch) const` | 该输出已连接（索引越界 → `false`） |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | 按表取出状态中该输出的值 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | 在每个已引出的引脚上，把测得的脉冲与预期值比较；偏差 ≤ 15 µs 时为“OK” |
| `void write(const FlightOutputState&)` | 写入所有输出并记住该状态 |
| `void setFailsafe()` | 舵面中立位（`FAILSAFE_*`），油门 `FAILSAFE_THROTTLE`；AUX 保持原样（信号丢失时不会抛投载荷） |
| `void setBuzzer(bool on)` | 板载蜂鸣器（`IBoard::setBuzzer`） |
| `const FlightOutputState& getLastState() const` | 最近一次写入的状态 |

添加一个输出：表中增加一行 + 在 `FlightOutputState` 中增加一个字段 + 在
`ServoChannel` 中增加一个索引（+ 在 `Esp32Board` 中增加引脚和 LEDC 通道）。

---

## `FlightController`

**文件：** `control/FlightController.h` · **层：** COORDINATION ·
**依赖：** `IBusReceiver`、`ControlMixer`、`ThrottleManager`、`ArmingManager`、`FlightOutputs`、`Autopilot*`、`PilotSwitches*`、`Beeper`

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

`update()` 的顺序：

1. `receiver.update()`；`failsafe = receiver.isSignalLost()`；
2. 链路正常时——`switches->update(rc)`（模式、功能、旋钮）；
3. `pilotThrottle = throttle.update(rc, failsafe)`；
4. 摇杆 `mixer.fromSticks(rc)`（链路正常时）× `Knob::RATES`；襟翼
   `mixer.updateFlaps(target)`：`AIRBRAKE` → −`AIRBRAKE_US`，`FLAPS` →
   `FLAPS_DEPLOYED_US`，`Knob::FLAPS` → 平滑变化，信号丢失时——0；
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)`——**始终**执行；
6. 蜂鸣器：`Beeper::update(BEEPER, armed, failsafe, now)`；
7. 信号丢失 → `applyLinkLoss()` 并退出本周期；
8. `arming.update(rc, false)`；
9. `command = autopilot->getCommand()`（没有自动驾驶仪时用摇杆），襟翼——使用自己的；
10. `output = mixer.mix(command)`；`output.throttle = autopilot->applyThrottle(pilotThrottle)`；
11. 未 ARM 或 `MOTOR_KILL` → `throttle = PWM_MIN`（最后执行）；
12. AUX1——载荷（`PAYLOAD_DROP`），AUX2——相机（`Knob::CAMERA_TILT`，`CAMERA_STAB` 会减去俯仰）；
13. `outputs.write(output)`。

`applyLinkLoss()`：如果自动驾驶仪处于 failsafe（已 ARM：RTH 或滑翔）——
舵面和油门按自动驾驶仪的指令输出（襟翼平滑收起，`MOTOR_KILL` 依然会让电机停转，AUX 保持原样）；否则调用 `outputs.setFailsafe()`。

---

## `Beeper`

**文件：** `control/Beeper.h` · **依赖：** `Config`

| 方法 | 说明 |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | 蜂鸣器状态：如果有 `Feature::BEEPER` 或者“模型丢失”（未 ARM，且无信号的时间超过 `LOST_MODEL_BEEP_DELAY_MS`），则以 2 Hz 鸣叫 |
| `bool isLostModel() const` | “到草丛里找我”模式 |
