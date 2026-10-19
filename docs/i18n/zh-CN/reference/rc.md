# RC — 接收遥控器指令

> 🌐 本页是[俄语原文](../../../reference/rc.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

RC 层把 UART 字节转换成通道值和“无链路”标志。它对飞机、ARM、failsafe 行为和舵机一无所知。接收机协议——iBUS、SBUS 或 CRSF——由 `Config::RX_PROTOCOL` 设定；新协议只需在现有三个帧解析器旁再加一个。

---

## `RcChannelState`

**文件：** `rc/RcChannelState.h` · **依赖：** `Config`、`Channels`

接收机 10 个通道（µs）的快照，不含控制逻辑。

| 方法 | 说明 |
|---|---|
| `RcChannelState()` | 调用 `reset()` |
| `void reset()` | 安全值：所有通道为 `PWM_CENTER`，油门为 `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | 通道值；索引越界 → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | 写入通道；索引越界则忽略 |
| `const uint16_t* data() const` | 整个数组（用于调试） |

---

## `RcInput`

**文件：** `rc/RcInput.h` · **类别：** 一组静态函数 · **依赖：** `Config`

RC 信号的通用转换。

| 方法 | 说明 |
|---|---|
| `static uint16_t clamp(uint16_t value)` | 限制在 `PWM_MIN..PWM_MAX` 内 |
| `static uint16_t reversed(uint16_t value)` | 通道反向：`3000 − clamp(value)`（1000 ↔ 2000）——用于 `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | 线性映射：1000 → `−max`，1500 → 0，2000 → `+max`（输入先被限幅）；`reverse` 会改变符号。结果被限制在 ±`max` 内 |

示例：`centered(1750, 500) == 250`，`centered(1750, 500, true) == -250`。

---

## `RcReceiver`

**文件：**`rc/RcReceiver.h` · **依赖：**`IUartPort`、`IBusParser`、`SbusParser`、`CrsfParser`、`RcChannelState`、`RcInput`、`Config`、`Channels`

遥控接收机：按协议格式打开 UART，把字节交给解析器，并完成所有协议共有的工作——前 `RC_CHANNELS` = 10 个通道、通道反向、帧超时、failsafe、计数器。

| 协议（`Config::RX_PROTOCOL`） | 接收机 | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B、FS-iA10B | 115200，8N1 |
| `SBUS` | FrSky、Futaba、RadioMaster、Jumper 及大多数其他品牌 | 100000，8E2，反相——由 UART 自身反相 |
| `CRSF` | ExpressLRS、TBS Crossfire 和 Tracer | 420000，8N1 |

| 方法 | 说明 |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | 构造函数中不打开端口；测试中可传入任意协议 |
| `void begin()` | 按协议格式打开 UART（`IUartPort::begin(baud, format)`），超时从“现在”开始计 |
| `void update()` | 读出 UART 中积累的全部数据；每个周期调用 |
| `const RcChannelState& getState() const` | 最近收到的通道 |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | 尚未收到任何帧，**或**最后一帧早于 `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | 接收机报告失去链路：SBUS 帧中的 failsafe 标志、CRSF 链路质量为 0 %，或油门 < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | 最后一个正确帧的 `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | 已接收帧与被丢弃帧（CRC、帧尾）的计数器 |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | 协议；名称为 `"iBUS"`、`"SBUS"`、`"CRSF"` |
| `int16_t getLinkQuality() const` | 无线链路质量，%——仅 CRSF，否则为 −1 |
| `bool isPortReady() const` | UART 已按所需格式打开；`false`——端口不支持（SBUS 无反相），不会有链路 |

自由函数 `rxProtocolName(Config::RxProtocol)` 返回协议名称；固件身份信息（`@id` 的 `rx` 字段）打印的也是它。

不变量：

- 在第一个正确帧之前 `isSignalLost() == true`——默认值（全部为 1500）不会被当作遥控器指令。
- failsafe 标志在**每个**通道帧上重新计算——第一个油门正常的帧就会恢复链路（CRSF 还需要链路质量大于 0 的统计帧）。
- 设置了 `Config::CHANNEL_REVERSED` 的通道以反向值保存（`RcInput::reversed`）；failsafe 在此之前根据**原始**油门判断，而 CH3 的反向被 `static_assert` 禁止。

---

## `IBusParser`

**文件：**`rc/IBusParser.h` · **依赖：**`Config`

逐字节解析 FlySky iBUS：`bool feed(uint8_t)` 在收齐 CRC 正确的帧时返回 `true`；通道在 `values()` 中。**帧**（32 字节）：`0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`，`CRC = 0xFFFF − Σ(前 30 字节)`；通道值为**低 12 位**（FS-iA6B 在高位传送服务数据，例如 failsafe 时 `0x2384` → 900 µs）。等待 `0x20`，下一个字节必须是 `0x40`，否则重新搜索（第二个 `0x20` 本身可能是帧头）。iBUS 没有 failsafe 标志：FS-iA6B 通过低于正常范围的油门报告失去链路。

---

## `SbusParser`

**文件：**`rc/SbusParser.h` · **依赖：**`Packed11`

**帧**（25 字节）：`0x0F | 16 个 11 位通道（22 字节） | 标志 | 帧尾`；标志：位 2——无线丢失一帧（还不算失去链路），位 3——failsafe；帧尾——SBUS 为 `0x00`，SBUS2 为 `0x04`/`0x14`/`0x24`/`0x34`。数据中也会出现 `0x0F`，所以只有帧头和帧尾都在正确位置时才接受该帧；否则从已接收数据中的下一个 `0x0F` 继续解析——一两帧内即可同步。

---

## `CrsfParser`

**文件：**`rc/CrsfParser.h` · **依赖：**`Packed11`

**帧：**`0xC8 | 长度 | 类型 | 数据 | CRC8`；长度 = 类型 + 数据 + CRC，整帧不超过 64 字节；CRC8 采用多项式 `0xD5`（DVB-S2），覆盖类型和数据（`static uint8_t crc8(...)`）。需要两种类型：`0x16`——通道（16 个 11 位，与 SBUS 相同）和 `0x14`——链路统计（第三个字节为链路质量，%）。ExpressLRS 默认在失去链路时直接停止发送通道——这就是帧超时；在“保持最后位置”模式下通道继续发送，但链路质量为 0——这就是 `failsafe()`。

---

## `Packed11`

**文件：**`rc/Packed11.h` · **类别：**命名空间

16 个 11 位通道装在 22 字节中，低位在前——SBUS 和 CRSF 都这样打包。`toMicros(raw)`：172…1811 → 988…2012 µs，992 → 1500（`5/8 · 值 + 880`，与 Betaflight 和 INAV 相同）；`unpack(data, out)` 把 22 字节转换为 16 个 µs 值。
