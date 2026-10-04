# TELEMETRY — log, console, web dashboard, OLED, black box

> 🌐 This page is a translation of the [Russian original](../../../reference/telemetry.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Reference](README.md)

The telemetry is completely separated from the flight logic: it only reads the
const getters of `FlightController`, `Autopilot`, the sensors and `LoopStats`.
The only path "back" is the dashboard commands, passed through the `WebDebugServer`
mailbox and applied by the flight loop.

---

## `LoopStats`

**File:** `telemetry/LoopStats.h` · **Kind:** struct

The frequency and duration of the flight loop.

| Member | Description |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Published once a second; read from other tasks (32-bit values — no "torn" reads) |
| `void record(uint32_t durationUs)` | Call every tick from `loop()` |
| `uint32_t takePeakUs()` | The worst tick since the previous call (for the SYS line every 10 s); call it from the same task as `record()` |

`maxUs` is the worst only over the last second; a rare hiccup is visible through
`takePeakUs()`.

---

## `LogSettings`

**File:** `telemetry/LogSettings.h` · **Depends on:** `Preferences` (NVS, the `debuglog` namespace)

### `LogChannel` (enum class)

| Channel | Prefix | What it prints | Default |
|---|---|---|---|
| `Status` | `STAT` | link, ARM, mode, flaps, sensors | on change |
| `Rc` | `RC` | transmitter channels | off |
| `Outputs` | `OUT` | outputs to the control surfaces and the ESC | off |
| `Attitude` | `ATT` | roll, pitch, heading | off |
| `Autopilot` | `AP` | targets and corrections | off |
| `Altitude` | `ALT` | altitude, vertical speed | off |
| `Heading` | `MAG` | compass heading | off |
| `Gps` | `GPS` | satellites, coordinates | off |
| `Imu` | `IMU` | gyroscope and accelerometer | off |
| `Nav` | `NAV` | home, heading, speed, pitot tube, enabled features | off |
| `System` | `SYS` | loop frequency, memory (every 10 s), only off/on | on |
| `Count` | — | the number of channels | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| Method | Description |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | A row of the channel table |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (cyclically) |
| `LogSettings()`, `void setDefaults()` | The default modes, a period of 1 s |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | The mode of a channel |
| `void setMode(uint8_t, LogMode)` | For `periodicOnly` channels `OnChange` becomes `Periodic` |
| `void cycleMode(uint8_t)` | off → on change → continuous → off (SYS: off ↔ on) |
| `void setAll(LogMode)` | For all channels; SYS is not touched by the "all on change" command |
| `uint16_t periodMs() const`, `void cyclePeriod()` | The period of the "continuous" mode |
| `static const char* modeName(LogMode, bool periodicOnly)` | "off" / "on change" / "continuous" (or "on") |
| `void load()` | From NVS; if `VERSION` or the length does not match, the defaults remain; an unknown mode code → the channel's default |
| `void save() const` | To NVS (the modes, the period, the version) |

`VERSION` changes together with the list of channels — the old settings are reset
(`VERSION = 2`: the NAV channel was added). The channel keys in the menu: `1`..`9`, NAV —
`n`, SYS — `s`.

---

## `DebugLogger`

**File:** `telemetry/DebugLogger.h` · **Depends on:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Printing of the state to the serial monitor by channel: each has its own line, its own
mode and its own tolerances.

| Method | Description |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | Once per `DEBUG_INTERVAL_MS` it walks over the channels (silent while paused and while the menu is open) |
| `LogSettings& getSettings()`, `void saveSettings() const` | For the console menu |
| `void suspend(bool)` | The menu is open — stay silent; on release — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Pause with the space bar; on release — `refresh()` |
| `void refresh()` | The next tick will print all enabled channels |

The channel logic (`updateChannel`):

- `Off` — do not print;
- `Periodic` — once per `periodMs()` (SYS — once per 10 s), values "as is";
- `OnChange` — the line is assembled with **tolerances** (the nested `Shown` keeps
  the old value until the new one moves beyond the tolerance: RC/PWM 3 µs, angles
  0.5°, heading 1°, corrections 2, altitude 0.3 m, acceleration 0.03 g, coordinates
  1e−5°) and is printed only if it differs from the last printed one.

The nested types: `LineBuffer : Print` (a line of up to 200 bytes for comparison before
printing), `Shown` (a value with hysteresis).

The line formats:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  target 0.0 m
MAG  heading 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (worst in 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` tells `LOST(no frames)` from `LOST(transmitter failsafe)`; `IMU=` is
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**File:** `telemetry/DebugConsole.h` · **Depends on:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (bus scan)

A text menu in the serial monitor. A state machine of screens `Screen::{None, Main, Log}`.

| Method | Description |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | with a board — the `b` command and menu item 7 |
| `static const char* guessI2cDevice(uint8_t address)` | the chip by address: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | A one-line hint |
| `void update()` | Process all bytes from `Serial`; if the log settings were changed, the menu is closed and the aircraft is **not armed** — save to NVS |

Hot keys (outside the menu): `h`/`?` — the main menu; `l` — the log menu; space —
pause the log; `s` — sensor status; `i` — gyroscope calibration; `o` —
IMU mounting calibration; `m` — compass calibration; `p` — output self-test;
`b` — scan of the I2C buses (0x08..0x7F — up to 0x7F, because the QMC6309 sits at 0x7C) with
chip names; anything else — a hint. `\r`/`\n` are ignored.

The log menu: `1`..`9` — cycle the mode of channels 0..8, `n` — NAV, `s` — SYS, `p` — period, `a` —
everything "on change", `x` — everything off, `d` — defaults, `0`/`q` — back, `l`/`h` —
close.

The blocking actions (`i`, `o`, `m`, `p`) are **forbidden while ARMed**. While the menu
is open, the log is suspended (`DebugLogger::suspend`). The width of the menu items
is counted in UTF-8 characters rather than bytes (Cyrillic takes 2 bytes).

---

## `WebDashboardPage`

**File:** `telemetry/WebDashboardPage.h` · **Kind:** namespace

`static const char HTML[] PROGMEM` — the whole page (HTML + CSS + JS) as a single
literal. Everything dynamic is built by the browser from the JSON of `/api/status` (polled
every 200 ms): the rows of channels, outputs and sensors are created from the JSON keys, so a new
output appears without editing the page. A PID field that the user has started
to edit is no longer overwritten by the polling.

---

## `WebDebugServer`

**File:** `telemetry/WebDebugServer.h` · **Depends on:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Method | Description |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | A Wi-Fi AP (`persistent(false)` — nothing is written to flash), the routes, the `web` task on core 0. `false` if the access point did not come up |
| `void applyPendingCommands()` | Call it from the flight loop: take the commands under a spinlock and apply them to the autopilot |

The routes:

| Route | Response |
|---|---|
| `GET /` | The dashboard page |
| `GET /api/status` | The state JSON (`buildStatusJson()`), the format is in the [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; no body → 400 `no data`; no autopilot → 503; a wrong mode → 400 `invalid mode` |
| `POST /api/setpid` | Any of `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`; the omitted ones stay as they are |
| anything else | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — a mailbox under a
`portMUX`. `extractJsonNumber(body, key, fallback)` — a minimal parser of
flat JSON without ArduinoJson: `"key"`, spaces, `:`, spaces, a number in
any JSON notation (a sign, a fraction, an exponent `1e-7`); if the key or the number is missing —
`fallback`.

In the JSON the fields `attached`/`available` are present **always**; the sensor data only
when `available: true`.

---

## `OledDisplay`

**File:** `telemetry/OledDisplay.h` · **Depends on:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

An SSD1306 128×64 (I2C 0x3C) on the second I2C bus; its own `oled` task
(`Rtos::startTask`: core 0 on the ESP32, a low priority on the STM32), every 200 ms.

| Method | Description |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` or the display does not answer at 0x3C → `false`; otherwise it sets up U8g2 and starts the task |

U8g2 sends bytes through a `byteCallback` on top of `II2CBus` (the display knows nothing about
`Wire1`). A C callback gets no context, so the bus is kept in a static
variable `busSlot()` — there is only one display on board.

The display:

```
RX ok ARM STAB FL       link (a loss — an inverted line) / ARM / mode / flaps
R  +1.2 P  -0.4         roll / pitch, °           (IMU --)
Alt +0.3 Vz +0.1 A14    altitude / vertical speed / airspeed, if there is a pitot tube (BARO --)
H123 T1000 Y1500        heading / throttle / rudder (H---)
L1500 R1500 E1500       ailerons / elevator
Loop 500Hz max1100us    frequency and the worst tick over the second
```

The short mode names are `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
on a loss of the link in the air — `GLIDE` or `FSRTH`.

---

## `BlackBox`

**File:** `telemetry/BlackBox.h` · **Depends on:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

Recording of the flight to flash (ESP32-S3) or an SD card (STM32H743). What to download, when and how — see [BLACKBOX.md](../BLACKBOX.md).

| Method | Description |
|---|---|
| `bool begin(bool startTask = true)` | Reads the medium (`BlackBoxStorage::begin()`), allocates the queue (PSRAM on the ESP32, `malloc` on the STM32), checks the erased space (up to 0.3 s), starts the `bbox` task (`Rtos::startTask`). If there is no room to record (no partition, card or file) — `false`, the black box is off |
| `void update(uint32_t workUs)` | From `loop()` after every tick: events, start/stop, snapshots into the queue, wakes the writer task |
| `void writerStep()` | A step of the writer task: one or two pages into flash, or one erase on the ground |
| `requestManualStart()` / `requestManualStop()` | Recording by hand (console `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (writes out the queue before recording END) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | For the console |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [baud]` — for `tools/blackbox.py` (over USB CDC the speed affects nothing) |

The platform-specific parts: the reset cause — `readResetCause()`; the battery voltage and current —
the ADC (`analogReadMilliVolts` on the S3, a 12-bit `analogRead` on the STM32); the medium errors
(`BlackBoxStorage::writeErrors`/`eraseErrors`) go into the log once a second
as an event "medium: write errors …" and do not interfere with the flight.

## `BlackBoxStorage`

**File:** `telemetry/BlackBoxStorage.h` · **Depends on:** `IFlashRegion`

A ring of 4 KB sectors: the head and the list of flights come from the sector headers at `begin()` (the first pass reads the header of every sector and remembers the genuine ones, the second only those: an empty area is read once); `openFlight()`/`append()`/`flush()`/`closeFlight()` — writing by pages (a CRC-8 on every record); `eraseStep(target, protect, allowErase)` — one step of checking/erasing ahead of the head: garbage — always, flights — whole and only while less than `target` is free; `protect` is never touched.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` is a byte queue of records between tasks/cores under `Rtos::CriticalSection`; when it overflows, it drops the oldest. `BlackBoxFormat` — the sector header, the types and structures of records, the schema strings (the size is checked by `static_assert`), CRC-8 and CRC-32.

---

## `Mavlink` (codec)

**File:** `telemetry/MavlinkCodec.h` · **Kind:** namespace · **Depends on:** nothing (portable)

MAVLink 2 without the generated library: packing fields in MAVLink order
(checked against pymavlink), CRC-16/MCRF4XX + `CRC_EXTRA`, trimming of trailing zeros.

| Entity | Description |
|---|---|
| `Msg::*` | identifiers: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | the `CRC_EXTRA` of a message, −1 — unknown |
| `crcAccumulate`, `crcCalculate` | X.25 (like mavlink's `crc_accumulate()`) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — the fields in order |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — a v2 frame with a sequential `seq` |
| `Message` | a received message: `msgid`, `sysid`, `compid`, the payload (padded with zeros), reading fields by offset |
| `Parser` | `bool feed(byte)` → `message()`; v1 and v2, the v2 signature is skipped; `goodCount()`, `badCrcCount()`; messages with an unknown `CRC_EXTRA` are silently skipped |

## `MavlinkModes`

**File:** `telemetry/MavlinkTelemetry.h` · **Kind:** namespace

| Function | Description |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | the ArduPlane mode number: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | the reverse, for commands from the ground; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | the `AUTO_ENABLED` flag in HEARTBEAT |

## `MavlinkTelemetry`

**File:** `telemetry/MavlinkTelemetry.h` · **Depends on:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Telemetry over a radio modem for QGroundControl / Mission Planner (the vehicle is
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Used on the STM32
(UART4), which has no Wi-Fi.

| Method | Description |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | open the port, the message "OpenPlane online" |
| `void update()` | from the flight loop: parse the incoming data (≤ 128 bytes per tick), event messages, no more than 2 frames per tick |
| `void statusText(severity, text)` | to the GCS feed (a queue of 4 lines, up to 50 characters) |
| `isGcsConnected()` | a HEARTBEAT from the GCS within the last 3 s |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | diagnostics |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

The streams (Hz): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0.2. A frame is sent only if `availableForWrite()` has room for it
— otherwise it waits for the next tick (the loop is never blocked).

Incoming: the GCS HEARTBEAT; PARAM_REQUEST_LIST / READ / SET (the PID — straight into
the autopilot, values 0..100, not saved); SET_MODE and COMMAND_LONG
`DO_SET_MODE` (176) — the mode until the next flick of the switch; `COMPONENT_ARM_DISARM`
(400) — **DENIED**; `REQUEST_MESSAGE` (512) — an out-of-turn send of a stream;
MISSION_REQUEST_LIST — MISSION_COUNT 0 with the same `mission_type`.
Checking the stream with an external decoder — `tools/check_mavlink.py` (pymavlink).
