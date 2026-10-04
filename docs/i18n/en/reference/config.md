# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 This page is a translation of the [Russian original](../../../reference/config.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

The configuration layer is only `constexpr` constants, with no code. The class
logic must not contain "magic" pins, timeouts and thresholds: everything that
may need to change for a particular airplane or board lives here.

---

## namespace `Config`

**File:** `include/config/Config.h` · **Depends on:** `<stdint.h>` ·
**Used by:** almost all layers.

### Pins (board-dependent)

The pin block is selected by the macro that `[env:*]` sets in `platformio.ini`
(`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Without a macro — `#error`. The STM32 block is described
[below](#stm32h743vit6-board_stm32h743).

| Constant | Type | Purpose | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Ailerons | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Elevator | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Motor controller | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Rudder + wheel; `-1` — the output is disabled | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | iBUS receiver RX (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | The sensor bus (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | The OLED bus (`Wire1`); `-1` — none | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | The shared SPI bus | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | The IMU CS over SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | The barometer CS over SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | The GPS UART; TX `-1` — receive only | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | The hardware UART number for the GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | servo outputs: payload drop, flaps; `-1` — none | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | the beeper through a transistor; `-1` — none | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **S3 only:** reserved for the flight-controller board ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

The sensor SPI bus is called `PIN_SENSOR_SPI_*`, not `PIN_SPI_*`: in the
STM32duino core (and other Arduino cores) `PIN_SPI_SCK/MISO/MOSI` are variant
macros, and they would replace the `Config` constants.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

There is no board yet: the pinout is **not tested on hardware** (the firmware is run on a PC, env `native-stm32`). The pins are chosen from the free ones
on the WeAct MiniSTM32H743VITx (the PlatformIO env `stm32h743` board) and
cross-checked against the `PeripheralPins` tables of the STM32duino variant.
The values are variant macros (`PA0`…), so at the top of `Config.h`, under
`#if defined(BOARD_STM32H743)`, `<Arduino.h>` is included. The type of all
pins is `int16_t` (analog pins are numbered `0xC0 + N`). There are no UART
numbers — the core picks the peripheral by the pins.

| Constant | Pin | Peripheral |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX — reserved for iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — sensors |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — the display (on the WeAct — the camera connector) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — payload / camera |
| `PIN_BUZZER` | PE15 | GPIO — the beeper |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — reserved |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — the MAVLink radio modem (the same pins are FDCAN1) |

The `Serial` console is LPUART1 (PA9 TX / PA10 RX), the variant's default.

### iBUS and link loss

| Constant | Value | Meaning |
|---|---|---|
| `IBUS_CHANNELS` | 10 | How many channels of the frame are used |
| `IBUS_FRAME_LENGTH` | 32 | Frame length, bytes |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | The frame header |
| `IBUS_BAUDRATE` | 115200 | UART speed |
| `RX_TIMEOUT_US` | 500 000 | No correct frame for longer than this — the link is lost |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Throttle below this — the receiver reports the transmitter's failsafe |

### GPS

| Constant | Value | Meaning |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | A NAV-PVT older than this — `UbloxM10_Gps::isAvailable() == false` |

### PWM range and control surface travel

| Constant | Value | Meaning |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | The standard RC pulse, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Deflection from center at full stick travel, µs. The rudder is smaller: the landing-gear wheel is on the same servo |
| `THROTTLE_LIMIT_PCT` | 100 | The throttle ceiling to the ESC, %, the same for the stick and the autopilot (`FlightController::capThrottle`). For bench tests on a weak 3S1P pack it was set to 50; the tests compute the expected output from this value |

### Flaps (flaperons)

| Constant | Value | Meaning |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 above this — the flaps are deployed (not 1500: until the first frame the channels = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Downward deflection of each aileron, µs (~20° of the MG90S horn) |
| `FLAPS_TRANSITION_MS` | 1000 | The time for a full deploy/retract |

### Servo direction

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true` — the aileron servos are mirrored), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` — the only place where reversal is set. `ControlMixer`
computes in physical signs and flips the sign only here, so the sticks and the
autopilot cannot diverge. Reversal on the transmitter **must not** be used.

### Sensor mounting

| Constant | Value | Meaning |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | The rotation of the IMU chip axes about the vertical (0/90/180/270), where the chip's X axis points. Used only while there is no mounting calibration `o` in NVS |
| `MAG_ROTATION_CW_DEG` | 0 | The same for the compass (the compass has no mounting calibration) |

### ARM

| Constant | Value | Meaning |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 above this — the ARM switch is on |
| `THROTTLE_LOW_US` | 1050 | Throttle below this — "throttle down", arming is allowed |

### Failsafe

| Constant | Value | Meaning |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Control surfaces neutral |
| `FAILSAFE_THROTTLE` | 1000 | Motor off |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | The gliding bank on link loss in the air |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | The gliding pitch (slightly below the horizon) |
| `FAILSAFE_RTH` | `true` | With GPS and a home point, link loss in the air — return home with the motor, rather than gliding |

### Switches, pitot tube, autopilot

The numbers for all the modes and functions are in `Config.h` next to detailed
comments; what they mean for the pilot is in [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Group | Constants |
|---|---|
| Switches | `SWITCH_ON_US` = 1750 (channel above this — the switch is on; not 1500, so that nothing turns on before the first frame) |
| Pitot tube | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Stabilization | `MAX_BANK_DEG` 45 (knob 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Navigation | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Altitude | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Throttle and speed | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Stall | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Circles and home | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Geofence | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Hand launch | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Landing | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Soaring | `SOAR_*`: gliding −3°, a thermal > 0.5 m/s for 1.5 s, a 25° circle, exit < −0.2 m/s for 8 s, motor below 30 m up to 100 m, home beyond 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Auto-trim | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, saving on the ground: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Coordination | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Functions | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Loop, Wi-Fi, debugging

| Constant | Value | Meaning |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | The flight loop period (500 Hz); also the nominal `dt` for `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | The dashboard access point (the password is weak — a bench tool) |
| `WEB_SERVER_PORT` | 80 | HTTP port |
| `TELEM_BAUDRATE` | 57600 | The MAVLink radio modem speed (the SiK default) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | The aircraft's address in MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | How often `DebugLogger` checks the log channels |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Tolerance for RC/PWM jitter in the "on change" mode |

### Black box

In detail — [BLACKBOX.md](../BLACKBOX.md).

| Constant | Value | Meaning |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | The record queue in PSRAM (without PSRAM — in internal memory) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: the queue in RAM — 10 s of pre-recording and headroom for card delays |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: the file on the SD card and the ceiling for its used part (the reconciliation time at power-on grows with the area) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Recording before the start (ARM + throttle) and after DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Armed, the motor stopped, the airplane motionless for this long — stop |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | What counts as "motionless" |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | After a faulty reboot — recording for no less than this |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Erased space kept ready; old flights are erased whole on the ground |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | The pause between erases |
| `BLACKBOX_IMU_DIVIDER` | 1 | The IMU every Nth cycle (1–500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | The battery (56k/10k) and current sensor (10k/15k) dividers on the flight-controller board |

---

## namespace `Channels`

**File:** `include/config/Channels.h` · **Depends on:** `<stdint.h>`

The only place where a physical channel number is tied to a purpose. The
values are **indices** (0-based) in `RcChannelState`.

| Constant | Index | Channel | FS-i6 control | Purpose |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | right stick ←→ | Roll |
| `ELEVATOR` | 1 | CH2 | right stick ↑↓ | Pitch (2000 = away from you = nose down) |
| `THROTTLE` | 2 | CH3 | left stick ↑↓ | Throttle |
| `RUDDER` | 3 | CH4 | left stick ←→ | Rudder + wheel |
| `ARM` | 4 | CH5 | SwA | The ARM switch (cannot be reassigned) |
| `SWB` | 5 | CH6 | SwB | per the `Controls.h` table (flaps by default) |
| `SWC` | 6 | CH7 | SwC (3 pos.) | mode by default MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | RTH by default |
| `VRA` | 8 | CH9 | VrA | `STAB_GAIN` by default |
| `VRB` | 9 | CH10 | VrB | `CRUISE_SPEED` by default |
| `COUNT` | 10 | | | the number of channels |

---

## namespace `Controls`

**File:** `include/config/Controls.h` · **Depends on:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` — what each switch and knob does, **one line
per channel** (`Bind::modes/mode/feature/knob`, see
[autopilot.md](autopilot.md#binding-bind-bindingcheck)). Next to it are
commented-out ready-made ideas. Three `static_assert`s catch table errors at
build time: a stick or ARM in the table, a channel out of range, a repeated
channel, more than one mode-selection switch.
