# RC — 接收遥控器指令

> 🌐 本页是[俄语原文](../../../reference/rc.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

RC 层把 UART 字节转换成通道值和“无链路”标志。它对飞机、ARM、failsafe 行为和舵机一无所知——更换协议（S-Bus、PPM）只会影响这一层。

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | 线性映射：1000 → `−max`，1500 → 0，2000 → `+max`（输入先被限幅）；`reverse` 会改变符号。结果被限制在 ±`max` 内 |

示例：`centered(1750, 500) == 250`，`centered(1750, 500, true) == -250`。

---

## `IBusReceiver`

**文件：** `rc/IBusReceiver.h` · **依赖：** `IUartPort`、`RcChannelState`、`Config`、`Channels`

FlySky iBUS 协议的逐字节解析器。

**帧格式**（32 字节）：`0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`，
`CRC = 0xFFFF − Σ(first 30 bytes)`。只取前 `IBUS_CHANNELS` = 10 个通道；通道值取**低 12 位**（高位中 FS-iA6B 传送的是服务数据，例如在 failsafe 时 `0x2384` → 900 µs）。

| 方法 | 说明 |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | 构造函数中不打开端口 |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`，超时从“现在”开始计时 |
| `void update()` | 读出 UART 中累积的全部数据；每个周期调用一次 |
| `const RcChannelState& getState() const` | 最近一次收到的通道值 |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | 还没有收到过任何一帧，**或者**最后一帧比 `RX_TIMEOUT_US` 更旧 |
| `bool isFailsafeReported() const` | 最后一帧中油门 < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | 最后一个正确帧的 `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | 正确帧计数器和 CRC 错误计数器 |

解析状态机（`processByte`）：等待 `0x20`；下一个字节必须是 `0x40`，否则重新开始查找；然后收集 32 个字节并调用
`processFrame()`。CRC 错误的帧会被整体丢弃（通道不变，`badFrames++`）。

不变量：

- 在收到第一个正确帧之前，`isSignalLost() == true`——默认值（全部为 1500）不会被当作遥控器的指令。
- failsafe 标志在**每个**正确帧上重新计算——收到第一个油门正常的帧，链路就恢复了。
