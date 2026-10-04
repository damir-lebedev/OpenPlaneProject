# RC — receiving transmitter commands

> 🌐 This page is a translation of the [Russian original](../../../reference/rc.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

The RC layer turns UART bytes into channel values and a "no link" flag. It
knows nothing about the airplane, ARM, failsafe behavior or servos — replacing
the protocol (S-Bus, PPM) affects only this layer.

---

## `RcChannelState`

**File:** `rc/RcChannelState.h` · **Depends on:** `Config`, `Channels`

A snapshot of the receiver's 10 channels (µs), with no control logic.

| Method | Description |
|---|---|
| `RcChannelState()` | Calls `reset()` |
| `void reset()` | Safe values: all channels `PWM_CENTER`, throttle — `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | The channel value; an index out of range → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Write a channel; an index out of range is ignored |
| `const uint16_t* data() const` | The whole array (for debugging) |

---

## `RcInput`

**File:** `rc/RcInput.h` · **Kind:** a set of static functions · **Depends on:** `Config`

Common conversions of RC signals.

| Method | Description |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Limits to `PWM_MIN..PWM_MAX` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linear: 1000 → `−max`, 1500 → 0, 2000 → `+max` (the input is limited first); `reverse` flips the sign. The result is limited to ±`max` |

Example: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**File:** `rc/IBusReceiver.h` · **Depends on:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

A byte-by-byte parser of the FlySky iBUS protocol.

**Frame format** (32 bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`. The first `IBUS_CHANNELS` = 10 channels are
taken; the channel value is the **low 12 bits** (in the high bits the FS-iA6B
carries service data, for example in failsafe `0x2384` → 900 µs).

| Method | Description |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | The port is not opened in the constructor |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, the timeout counts from "now" |
| `void update()` | Read out everything accumulated in the UART; call it every cycle |
| `const RcChannelState& getState() const` | The last received channels |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | There has not been a single frame yet **or** the last one is older than `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | In the last frame the throttle < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` of the last correct frame |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Counters of correct frames and CRC errors |

The parsing state machine (`processByte`): waits for `0x20`; the next byte must
be `0x40`, otherwise the search starts over; then it collects 32 bytes and calls
`processFrame()`. A frame with a wrong CRC is discarded entirely (the channels
do not change, `badFrames++`).

Invariants:

- Until the first correct frame `isSignalLost() == true` — the default values
  (all 1500) are not taken for transmitter commands.
- The failsafe flag is recomputed on **every** correct frame — the link is
  restored by the very first frame with a normal throttle.
