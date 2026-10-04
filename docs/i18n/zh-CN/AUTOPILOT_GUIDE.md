# OpenPlane 自动驾驶仪参考手册

> 🌐 本页是[俄语原文](../../AUTOPILOT_GUIDE.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

自动驾驶仪能做什么、如何启用每项功能，以及如何**只用一行代码**把它挂到遥控器的任意开关或旋钮上。

> 先说实话。所有模式都经过单元测试和闭环飞行仿真验证（`test/native/test_sim`：整套固件驾驶一个飞机模型）。模型是简化的，`Config.h` 中的系数只是起始值：**每个模式都要先在 50 m 以上的高度试飞，手指放在 MANUAL 开关上**。目前实际飞过的只有手动模式（第一架原型机，使用 ESP32-C3）；增稳已在台架上验证——舵面对倾斜的响应方向正确。STM32H743 开发板可以编译，并通过同样的测试；在实际硬件上，DevEBox 开发板已在没有传感器的情况下验证过：SD 卡、黑匣子，以及用遥控器手动控制舵机和电机（已拍成视频）；传感器还没有接到它上面，自动驾驶仪的各个模式也还没有在它上面试过。

---

## 目录

1. [一分钟了解整体结构](#一分钟了解整体结构)
2. [遥控器默认布局](#遥控器默认布局)
3. [一行代码分配功能](#一行代码分配功能)
4. [模式](#模式)
5. [功能（开关）](#功能开关)
6. [旋钮](#旋钮)
7. [信号丢失、地理围栏、返航点](#信号丢失地理围栏返航点)
8. [自制皮托管](#自制皮托管)
9. [地面站：Wi-Fi 仪表盘和 MAVLink](#地面站wi-fi-仪表盘和-mavlink)
10. [新飞机的调试顺序](#新飞机的调试顺序)
11. [自动驾驶仪飞行前检查](#自动驾驶仪飞行前检查)
12. [每个模式需要什么](#每个模式需要什么)

---

## 一分钟了解整体结构

```
摇杆 ───┐
        ├─► PilotSwitches (config/Controls.h) ─► 模式、功能、旋钮
开关 ───┘                                              │
                                                       ▼
传感器（IMU、气压计、罗盘、GPS、皮托管） ──────────► Autopilot ─► 舵面和油门指令
                                                       │
                          FlightController：襟翼、载荷、相机、蜂鸣器、failsafe
                                                       ▼
                                          副翼 · 升降舵 · 方向舵 · ESC · AUX1 · AUX2
```

- **模式**决定由谁来操纵：飞手（MANUAL）、飞手加助手（STABILIZE、ALT_HOLD、ACRO）、带飞手修正的自动驾驶仪（CRUISE、LOITER、RTH……）。
- **功能**叠加在任何模式之上：襟翼、减速板、载荷投放、地理围栏……
- **旋钮**平滑地调节一个数值：增稳强度、巡航速度、盘旋半径……
- 在带增稳的模式下，**摇杆给出的是角度**，而不是舵面偏转量：松开摇杆，飞机自己回到水平。
- 传感器故障绝不会让飞机“抽动”：没有 IMU——舵面留给飞手；没有气压计——高度由飞手保持；没有 GPS——就没有导航，而需要它的模式会以安全的方式运行（见[表格](#每个模式需要什么)）。

---

## 遥控器默认布局

FS-i6 + FS-iA6B，iBUS，10 个通道（`config/Channels.h`）。

| 通道 | 遥控器部件 | 默认功能 |
|---|---|---|
| CH1–CH4 | 摇杆 | 横滚、俯仰、油门、方向舵（不能重新分配） |
| CH5 | **SwA** | **ARM**（向下 = 解锁，仅在油门拉到最低时有效；不能重新分配） |
| CH6 | SwB | 襟翼（`Feature::FLAPS`） |
| CH7 | **SwC**（3 档） | 向上 **MANUAL** · 中间 **STABILIZE** · 向下 **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH**——返航，开关接通期间有效 |
| CH9 | VrA | 增稳强度（`Knob::STAB_GAIN`） |
| CH10 | VrB | 巡航速度（`Knob::CRUISE_SPEED`） |

开机时，串口监视器会打印实际的布局——也就是真正烧录进去的内容（固件输出的是俄语，其中“вверх / середина / вниз”的意思是向上 / 中间 / 向下）：

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> 在 FS-i6 上，通道 7–10 默认没有输出。遥控器菜单：**Functions setup → Aux. channels**，把 SwC、SwD、VrA、VrB 分配上去。

---

## 一行代码分配功能

全部都在一个文件里——`include/config/Controls.h`：

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| 写法 | 作用 |
|---|---|
| `Bind::modes(通道, 上, 中, 下)` | 三档开关选择模式 |
| `Bind::modes(通道, 上, 下)` | 两档开关——两种模式 |
| `Bind::mode(通道, 模式)` | 开关接通期间，该模式**优先于**其他模式；关闭后恢复为模式开关选定的模式 |
| `Bind::feature(通道, 功能)` | 开关接通期间，该功能生效 |
| `Bind::knob(通道, 旋钮)` | 旋钮：中点 = `Config.h` 中的值，两端 = 最小值和最大值 |

“接通”指通道高于 1750 µs（在 FS-i6 上是开关向下，拨向自己）。在收到接收机第一帧数据之前，所有通道都视为关闭：上电时不会放出或投放任何东西。

### 现成的配方

```cpp
// 热气流滑翔机：SwD——翱翔，SwB——自动配平，VrB——盘旋半径
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// 学员：只用增稳，RESCUE 放在“紧急按钮”上，柔和的摇杆
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// 摄影与投送：带增稳的相机，载荷投放，在某点上空盘旋
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// 无起落架的手抛起飞：SwD——LAUNCH，用旋钮平滑控制襟翼
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### 编译器会拦住错误

表格在编译时检查（`static_assert`），在固件装上飞机之前就会发现问题：

- 摇杆和 SwA（ARM）不能绑定，通道编号必须小于 `Channels::COUNT`；
- 每个通道只能有一个绑定；
- 选择模式的开关（`Bind::modes`）最多只能有一个。

如果同时接通了多个 `Bind::mode` 开关，则表格中靠上的那一行生效。

---

## 模式

模式会在开关被**拨动**时切换。从仪表盘或地面站启用的模式一直有效，直到飞手再次拨动模式开关。OLED 上显示简短名称（括号中的部分）。

### MANUAL (MAN)
舵面 = 摇杆，就像没有飞控一样。只有功能（襟翼、减速板、载荷）和自动配平在工作。**最主要的安全模式**：始终把它放在手指下的开关上。

### STABILIZE (STAB) — “摇杆给出角度”
横滚摇杆给出的是横滚角，最大为 `MAX_BANK_DEG`（45°，旋钮 `MAX_BANK` 为 15…60°），俯仰摇杆给出的是俯仰角，最大为 `STAB_MAX_PITCH_DEG`（25°）。松开摇杆——飞机自己回到水平。油门由飞手控制。PID 的积分项只在接近目标时（±10°）累积，所以猛烈机动之后飞机不会“冲过”水平线。
**需要：** IMU。没有 IMU——和 MANUAL 一样。

### ALT_HOLD (ALT) — “保持高度”
横滚方面与 STABILIZE 相同，而升降舵根据气压计保持高度。推动俯仰摇杆——由你自己操纵；松开——飞机保持**新的**高度。油门由飞手控制（要加油门，否则爬升时速度不够）。
**需要：** IMU + 气压计。

### ACRO — “摇杆给出旋转速率”
摇杆打满——180°/s。松开——飞机**保持当时的姿态**（哪怕是倒飞），陀螺仪抑制阵风。用于特技动作。
**需要：** IMU。没有 IMU——舵面 = 摇杆。

### CRUISE (CRZ) — “保持航向、高度和速度”
飞机在当前高度上直线飞行，油门自动（旋钮 `CRUISE_SPEED`：油门 30…55…85 %，装有皮托管时是**空速** 10…14…22 m/s）。横滚摇杆用来转弯（松开后保持新航向），俯仰摇杆用来改变高度。装有皮托管时，失速保护生效。航向取自 GPS（地速 > 3 m/s 时；低于 2 m/s 时切回罗盘），否则取自罗盘，再否则取自陀螺仪。在仿真中，4 m/s 的侧风下，地面航迹保持在 ±5° 以内，高度保持在 ±3 m 以内。
**需要：** IMU + 气压计；要让航向不漂移，还需要 GPS 或罗盘。

### LOITER (LOIT) — “在这里盘旋”
在开启该模式的那一点上空顺时针盘旋，半径 50 m（旋钮 `LOITER_RADIUS` 为 25…150 m），保持当前高度，油门自动。导引采用向量场：从远处，飞机沿切线切入圆周，在圆周上保持前馈坡度。没有 GPS——就原地以恒定坡度画圆。
**需要：** IMU + 气压计 + GPS。

### RTH — “返航”
朝返航点（ARM 的位置）飞去，高度为 `RTH_ALTITUDE_M` = 40 m（低于这个高度——沿途爬升，高于——保持不变）。到了返航点上空——以 LOITER 半径盘旋，直到飞手接管。没有 GPS 或返航点——原地盘旋。信号丢失和地理围栏触发的也是这个模式。
**需要：** IMU + 气压计 + 已记录返航点的 GPS。

### AUTO_TAKEOFF (TKOFF) — “按油门起飞”
ARM 之后什么也不会发生，直到飞手把油门推过中点。然后按程序执行：1 s 加速到 100 % 油门，机翼保持水平；2 s 抬头离地，俯仰 15°；然后以 10° 俯仰爬升，直到飞手切换模式。摇杆会叠加在程序之上——滑跑时可以修正航向。
**需要：** IMU。

### LAUNCH (LNCH) — “手抛起飞”
1. 已解锁，油门高于中点——发射处于**待发状态**，电机不转。
2. 投掷：向前的过载 > 1.5 g 且持续超过 40 ms。
3. 0.3 s 后（手已离开螺旋桨）——油门 100 %，以 15° 俯仰爬升，机翼保持水平——持续 6 s 或爬到 30 m。
4. 之后——和 CRUISE 一样，在已达到的高度上飞行。

投掷之前，摇杆的任何动作（> 150 µs）都会取消发射：飞机交还到飞手手中。
**需要：** IMU（加速度计）。仿真中：以 9 m/s 从手的高度投出——飞机一次也没有触地，15 s 内爬升超过 15 m。

### AUTO_LAND (LAND) — “降落”
电机关闭，沿航向滑翔，俯仰 −4°；按气压计低于 3 m 后——拉平（+4°）。横滚摇杆用来修正进近航向。要在直线航段上、迎风、在 20–40 m 高度时启用，并留足跑道余量。仿真中，接地时垂直速度小于 1.5 m/s，机翼水平，不是机头先着地。
**需要：** IMU + 气压计（开机时在地面归零）。

### SOARING (SOAR) — “在热气流中翱翔”
电机关闭，滑翔。升降速度计（装有皮托管时用总能量，不会因为拉杆而误判出“热气流”）高于 0.5 m/s 且超过 1.5 s——判定为热气流：以 25° 坡度盘旋。8 s 内的平均爬升率降到 −0.2 m/s 以下——退出热气流。低于 30 m——开电机，直到 100 m；距返航点超过 400 m——滑翔返航。仿真中，它找到了热气流（核心 3 m/s），并在不开电机的情况下爬升超过 50 m。
**需要：** IMU + 气压计；GPS——用于返回返航点。

### RESCUE (RESQ) — “救命”
机翼水平，机头 +8°，油门 70 %——从任何盘旋下降中改出。失去方位感——拨一下开关，然后松口气。仿真中，从坡度 70°、机头 −40° 的盘旋下降开始，4 s 内——机翼水平并开始爬升。
**需要：** IMU。

---

## 功能（开关）

| 功能 | 作用 | 细节和数值（`Config.h`） |
|---|---|---|
| `FLAPS` | 两个副翼同时下偏——襟副翼 | `FLAPS_DEPLOYED_US` = 220 µs，1 s 内平滑展开；横滚叠加在其上起作用 |
| `AIRBRAKE` | 两个副翼同时上偏——减速板，下滑道更陡 | `AIRBRAKE_US` = 250；优先级高于襟翼 |
| `AUTO_TRIM` | 学会在不碰摇杆的情况下让飞机保持直飞：平飞时持续的舵面指令会“流入”配平 | 20 %/s，最多 ±120 µs；DISARM 后在**地面上**保存到闪存 |
| `TURN_COORDINATION` | 转弯时方向舵跟进，坡度中抬头 | 在导航模式中始终开启 |
| `MOTOR_KILL` | 任何模式下都关闭电机，自动模式也一样 | 优先于任何模式和油门 |
| `BEEPER` | “我在这里”蜂鸣器 | 不接开关也会自己鸣叫：在地面上，信号丢失超过 10 s |
| `PAYLOAD_DROP` | 开关接通期间 AUX1 舵机打开 | 1000 µs 关闭，2000 打开 |
| `GEOFENCE` | 距返航点超过 500 m 或高度超过 120 m——RTH | `GEOFENCE_ALWAYS_ON`——不需要开关 |
| `HOME_RESET` | 返航点 = 当前位置（开关接通的那一刻） | 仅在 GPS 良好时 |
| `CAMERA_STAB` | AUX2 上的相机保持相对地平线的角度 | 减去飞机的俯仰角 |

## 旋钮

旋钮的中点 = `Config.h` 中的默认值；两端是最小值和最大值。没有绑定时，使用默认值。

| 旋钮 | 最小值 … 中点 … 最大值 | 作用范围 |
|---|---|---|
| `STAB_GAIN` | ×0.25 … ×1 … ×2 | 所有带增稳的模式和 ACRO——“更柔和/更强硬” |
| `MAX_BANK` | 15° … 45° … 60° | 摇杆和导航给出的最大坡度 |
| `CRUISE_SPEED` | 油门 30 … 55 … 85 %（有皮托管时：10 … 14 … 22 m/s） | CRUISE、LOITER、RTH，以及 SOARING 中的电机 |
| `FLAPS` | 襟翼 0 … 50 … 100 % | 用平滑的襟翼代替开关 |
| `CAMERA_TILT` | −90° … 0° … +30° | 相机角度（AUX2） |
| `RATES` | 摇杆行程的 30 … 65 … 100 % | 所有模式：摇杆灵敏度 |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER 以及返航点上空的盘旋 |

> 一个实用的技巧：把旋钮 `STAB_GAIN` 放在 VrA 上，就是在飞行中“实时”整定系数。飞机摇摆——调小；发软——调大；然后把这个倍数写进 `Config.h`。

---

## 信号丢失、地理围栏、返航点

**返航点**在 ARM 时记录，前提是 GPS 良好（3D 定位，≥ 6 颗卫星，精度 ≤ 5 m）。如果 ARM 时 GPS 还没有定位——一旦定位就会记录返航点。在野外更改——使用 `HOME_RESET` 功能。

**信号丢失**（超过 0.5 s 没有 iBUS 帧，或者接收机发来低于 950 µs 的油门——这是在遥控器中设置的 failsafe；见 `docs/PILOT_GUIDE.md`）：

| 情况 | 飞机的行为 |
|---|---|
| 在地面（未解锁） | 电机 0，舵面回中；10 s 后——蜂鸣器 |
| 在空中，有 GPS 和返航点 | 带电机的 **RTH**，到返航点上空——在 40 m 高度盘旋 |
| 在空中，没有 GPS | **滑翔**：电机关闭，机翼水平，机头 −3° |
| 信号恢复 | 立即切回飞手开关所选的模式 |

已经开始的返航不会因为 GPS 短暂丢失而转入滑翔。`FAILSAFE_RTH = false`——只滑翔。

**地理围栏**（`GEOFENCE` 或 `GEOFENCE_ALWAYS_ON`）：飞出 `FENCE_RADIUS_M`（500 m）或高于 `FENCE_ALTITUDE_M`（120 m）——RTH。要接管控制，把模式开关拨到任意其他位置（模式是靠位置变化来启用的）。飞机回到围栏内、并留出 10 % 余量后，围栏会再次生效。

---

## 自制皮托管

不用买差压传感器也能得到空速：**两个气压计**。

```
               迎面气流 ─►  ┌──────────── 管（PVC/黄铜，Ø4–6 mm） ──┐
                            │  BMP581 (I2C 0x47) — 总压             │  密封
                            └───────────────────────────────────────┘
   机身：主气压计（BMP581 0x46 / SPL06 / BMP388）— 静压

   速度  V = √(2·(P_tube − P_static − zero) / ρ),   ρ — 由静压和温度算出
```

**组装**。BMP581（地址为 0x47 的模块：SDO 引脚接 VCC）封在一根只朝前开口的管子里——电路板位于密封的腔体内，导线穿过密封胶引出。管子朝前，位于螺旋桨气流之外（在机翼上或机头上方）。第二个气压计在机身内部，用海绵挡住直吹的气流。

**在固件中启用**——`sensors/SensorSelection.h`：`SENSOR_KIT_LSM6DSV_PITOT` 或 `SENSOR_KIT_ICM45686_PITOT`（现成的套件），或者在自己的套件中设置 `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581`。

**零点**。两个气压计总会有一点偏差：每个的绝对精度是几十帕，而这正是低速时的全部压差（10 m/s ≈ 60 Pa）。上电后的第一秒，固件会对差值取平均，并把它当作零点。**上电时飞机静止，皮托管用手指或套帽盖住，或者朝向迎风方向**。归零之后，OLED、仪表盘和遥测中才会出现空速。

**校准 `PITOT_SCALE`**。机身内的压力并不是严格的静压。在无风时，用 CRUISE 沿直线往返飞一趟，把 GPS 的平均地速与皮托管测得的速度比较：`PITOT_SCALE = V_GPS / V_tube`。

**保护**。压差为明显的负值超过 2 s（管子接反、进水），或者皮托管的采样超过 0.2 s 没有更新——不再输出空速，自动驾驶仪改用旋钮设定的油门，并在没有空速的情况下保持航向。已在对两个气压计都加噪声的闭环仿真中验证：飞行中的空速误差 < 0.5 m/s。

皮托管带来的好处：CRUISE 保持的是**空速**而不是油门；失速保护；SOARING 用的总能量升降速度计；遥测中真实的速度。

---

## 地面站：Wi-Fi 仪表盘和 MAVLink

**ESP32——Wi-Fi 仪表盘**：热点 `OpenPlane-Debug`，密码 `12345678`，地址会显示在串口监视器中。可以看到通道、输出、所有传感器、模式、导航和已启用的功能；还可以切换模式和调整 PID。详见 `docs/PILOT_GUIDE.md`。

**STM32H743——通过数传电台使用 MAVLink**（UART4：PD0 RX，PD1 TX，57600 波特——SiK 的默认值）。适用 SiK 433/868/915 MHz、MAVLink 模式的 ELRS，以及作为 Wi-Fi 桥接的 ESP-01。**QGroundControl** 和 **Mission Planner** 会把这架飞机当作 ArduPilot 飞机：

- 地平仪、带返航点的地图、速度（有皮托管时取自皮托管）、高度、升降速度计、油门；
- 模式使用 ArduPlane 的名称：STABILIZE → FBWA，ALT_HOLD → FBWB，CRUISE → CRUISE，LOITER → LOITER，RTH → RTL，AUTO_TAKEOFF/LAUNCH → TAKEOFF，SOARING → THERMAL，RESCUE → STABILIZE，AUTO_LAND → AUTO；failsafe 显示为 RTL 或 CIRCLE；
- 消息栏：ARM/DISARM、模式切换（使用我们自己的名称）、信号丢失、地理围栏；
- **从地面切换模式**——用地面站里的模式按钮（AUTO 除外：OpenPlane 没有任务）；
- **参数** `RLL_KP … PTCH_KD`——横滚和俯仰的 PID，可以直接在飞行中从地面站的参数窗口读取和修改（重启后不会保存——把效果好的数值写进 `Config.h`）。

从地面发出的 ARM/DISARM 会被**拒绝**——只能用遥控器上的开关。不接硬件检查数据流：`OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin`（需要 `pip install pymavlink`）。

---

## 新飞机的调试顺序

1. **在地面上用 MANUAL**：检查舵面方向（`Config.h` 中的 `*_REVERSED`）、襟翼向下、AUX。用控制台里的 `p` 检查输出（先拆下螺旋桨！）。
2. **在地面上检查传感器**：`b`——扫描总线，`s`——状态；`o`——IMU 安装校准（3 个姿态，只做一次），`m`——罗盘。
3. **在地面上用 STABILIZE**：把飞机向右倾斜——右副翼应当**向下**（把飞机扳平）。机头抬起——升降舵向下。如果不是这样——说明方向反转设置或 IMU 安装弄错了。
4. **第一次飞行用 MANUAL**，爬升到 50 m 以上 → STABILIZE。摇摆——调小 `STAB_GAIN`；发软——调大。
5. 平飞时用 **AUTO_TRIM** 飞 20–30 s，降落，DISARM——配平值会被保存。
6. **ALT_HOLD**，然后 **CRUISE**——检查高度和航向；装有皮托管的话——校准 `PITOT_SCALE`。
7. **LOITER** 和 **RTH**——在一定高度、在视距范围内，手指放在 MANUAL 上。
8. 只有在这之后——才做 **failsafe 测试**（在高空关闭遥控器，飞机应当飞回返航点）以及自动起飞/降落。

## 自动驾驶仪飞行前检查

- [ ] 开机时打印出的布局，就是你预期的布局。
- [ ] 控制台/OLED：IMU ok，IMU 飞行前检查已通过（开机时飞机是静止的）。
- [ ] 气压计已在地面归零（OLED 上高度约为 0）。
- [ ] 装有皮托管时：地面上速度约为 0，向管子里吹气——数值上升。
- [ ] GPS：**ARM 之前**要有 3D 定位、≥ 6 颗卫星——否则不会有返航点，也没有 RTH。
- [ ] 在地面上用 STABILIZE：副翼和升降舵是把飞机扳平，而不是把它推倒。
- [ ] 遥控器中已设置 failsafe（信号丢失时油门低于 950），并且已在**地面上**、不装螺旋桨的情况下通过关闭遥控器验证过。
- [ ] MANUAL——放在手指下的开关上。

---

## 每个模式需要什么

| 模式 | IMU | 气压计 | GPS | 罗盘 | 皮托管 | 油门 | 缺少所需传感器时 |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | 飞手 | — |
| STABILIZE | ● | | | | | 飞手 | 舵面 = 摇杆 |
| ALT_HOLD | ● | ● | | | | 飞手 | 高度由飞手保持 |
| ACRO | ● | | | | | 飞手 | 舵面 = 摇杆 |
| CRUISE | ● | ● | ○ | ○ | ○ | 自动 | 按陀螺仪保持航向（会漂移），高度由飞手控制 |
| LOITER | ● | ● | ● | | ○ | 自动 | 原地以一定坡度画圆 |
| RTH | ● | ● | ● | | ○ | 自动 | 原地盘旋 |
| AUTO_TAKEOFF | ● | | | | | 程序 | 舵面 = 摇杆 + 飞手控制油门 |
| LAUNCH | ● | ○ | | | | 程序 | 舵面回中 |
| AUTO_LAND | ● | ● | | ○ | | 0 | 不拉平 |
| SOARING | ● | ● | ○ | | ○ | 0 / 电机 | 没有热气流——滑翔 |
| RESCUE | ● | | | | | 70 % | 舵面回中 |

● 表示必需，○ 表示能改善效果。代码中的检查：`Autopilot.h`（`imuReady`、`baroReady`、`nav.gpsGood`）；测试——`test/native/test_autopilot_modes`（各模式对每个传感器的反应）和 `test/native/test_sim`（闭环飞行）。
