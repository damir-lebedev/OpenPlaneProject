# RC — receiving transmitter commands

> 🌐 This page is a translation of the [Russian original](../../../reference/rc.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Reference](README.md)

The RC layer turns UART bytes into channel values and a "no link" flag. It
knows nothing about the airplane, ARM, failsafe behavior or servos. The receiver
protocol — iBUS, SBUS or CRSF — is set by `Config::RX_PROTOCOL`; a new protocol
is one more frame parser next to the three that exist.

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
| `static uint16_t reversed(uint16_t value)` | A channel the other way round: `3000 − clamp(value)` (1000 ↔ 2000) — for `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linear: 1000 → `−max`, 1500 → 0, 2000 → `+max` (the input is limited first); `reverse` flips the sign. The result is limited to ±`max` |

Example: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**File:** `rc/RcReceiver.h` · **Depends on:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

The RC receiver: opens the UART in the protocol's format, hands the bytes to the
parser and does what all protocols share — the first `RC_CHANNELS` = 10
channels, channel reversing, the frame timeout, failsafe, counters.

| Protocol (`Config::RX_PROTOCOL`) | Receivers | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper and most others | 100000, 8E2, inverted — the UART itself inverts |
| `CRSF` | ExpressLRS, TBS Crossfire and Tracer | 420000, 8N1 |

| Method | Description |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | The port is not opened in the constructor; tests may pass any protocol |
| `void begin()` | Open the UART in the protocol's format (`IUartPort::begin(baud, format)`), start the timeout from "now" |
| `void update()` | Read everything that has accumulated in the UART; call every tick |
| `const RcChannelState& getState() const` | The last received channels |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | No frame has arrived yet **or** the last one is older than `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | The receiver reported link loss: the failsafe flag in an SBUS frame, 0 % link quality on CRSF, or throttle < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` of the last valid frame |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Counters of accepted frames and discarded ones (CRC, frame end) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | The protocol; its name is `"iBUS"`, `"SBUS"` or `"CRSF"` |
| `int16_t getLinkQuality() const` | Radio link quality, % — CRSF only, otherwise −1 |
| `bool isPortReady() const` | The UART opened in the needed format; `false` — the port can't do it (SBUS without inversion), there will be no link |

The free function `rxProtocolName(Config::RxProtocol)` gives the protocol's name;
the firmware passport prints the same (`@id`, field `rx`).

Invariants:

- Until the first valid frame `isSignalLost() == true` — the default values
  (all 1500) are not taken as transmitter commands.
- The failsafe flag is recomputed on **every** channels frame — the link
  comes back with the very first frame with normal throttle (on CRSF also with
  a statistics frame whose link quality is above 0).
- Channels with `Config::CHANNEL_REVERSED` are stored reversed
  (`RcInput::reversed`); failsafe is detected from the **raw** throttle before
  that, and reversing CH3 is forbidden by `static_assert`.

---

## `IBusParser`

**File:** `rc/IBusParser.h` · **Depends on:** `Config`

Byte-by-byte FlySky iBUS parser: `bool feed(uint8_t)` returns `true` when a frame
with a valid CRC is complete; the channels are in `values()`. **Frame** (32 bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`; a channel value is the **low 12 bits** (in
the high ones the FS-iA6B sends service data, e.g. in failsafe `0x2384` → 900 µs).
It waits for `0x20`; the next byte must be `0x40`, otherwise the search restarts
(a second `0x20` may itself start a frame). iBUS has no failsafe flag: the FS-iA6B
reports link loss with throttle below normal.

---

## `SbusParser`

**File:** `rc/SbusParser.h` · **Depends on:** `Packed11`

**Frame** (25 bytes): `0x0F | 16 channels of 11 bits (22 bytes) | flags | end`;
flags: bit 2 — one frame lost over the radio (not link loss yet), bit 3 —
failsafe; end — `0x00` for SBUS, `0x04`/`0x14`/`0x24`/`0x34` for SBUS2. The byte
`0x0F` also occurs in the data, so a frame is accepted only when both its start
and its end are in place; otherwise parsing continues from the next `0x0F`
inside what was received — the stream syncs within a frame or two.

---

## `CrsfParser`

**File:** `rc/CrsfParser.h` · **Depends on:** `Packed11`

**Frame:** `0xC8 | length | type | data | CRC8`; length = type + data + CRC, a
frame is at most 64 bytes; CRC8 with polynomial `0xD5` (DVB-S2) over type and
data (`static uint8_t crc8(...)`). Two types are needed: `0x16` — channels (16
of 11 bits, as in SBUS) and `0x14` — link statistics (the third byte is the link
quality, %). On link loss ExpressLRS by default just stops sending channels —
that is the frame timeout; in "last position" mode channels keep coming but the
link quality is 0 — that is `failsafe()`.

---

## `Packed11`

**File:** `rc/Packed11.h` · **Kind:** namespace

16 channels of 11 bits in 22 bytes, least significant bits first — that's how
both SBUS and CRSF pack them. `toMicros(raw)`: 172…1811 → 988…2012 µs, 992 → 1500
(`5/8 · value + 880`, as in Betaflight and INAV); `unpack(data, out)` turns 22
bytes into 16 values in µs.
