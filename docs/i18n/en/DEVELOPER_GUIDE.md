# DEVELOPER_GUIDE.md — the OpenPlaneProject developer guide

> 🌐 This page is a translation of the [Russian original](../../DEVELOPER_GUIDE.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

A technical map of the firmware: which file is responsible for what, how data flows from the receiver and sensors to the servos, which sign conventions hold the whole chain together, how the web API is organized and how to extend the project. It is meant for a developer who writes C++ and wants to get oriented quickly in this repository (the `main` branch), not for learning the basics of the language or PlatformIO.

For an overview of the project and the prototype status, see [`../README.md`](README.md); for what to connect where and how to fly, see [`PILOT_GUIDE.md`](PILOT_GUIDE.md); for the plans, see [`ROADMAP.md`](ROADMAP.md). Here there is only code. The whole architecture (layers, tasks, state machines) is in [`ARCHITECTURE.md`](ARCHITECTURE.md), a reference for each class is in [`reference/`](reference/README.md), and the tests are in [`TESTING.md`](TESTING.md).

> The project is under active development. The ESP32-S3 bench has been assembled and verified with all the sensors, but **the autopilot has not yet been tested in flight** — this is noted wherever it concerns a specific module. If you doubt what the code does, re-read the source, not the document.

---

## Contents

1. [Architecture of the layers](#architecture-of-the-layers)
2. [FreeRTOS tasks and the control loop](#freertos-tasks-and-the-control-loop)
3. [File reference](#file-reference)
4. [Sign convention: from the IMU to the servo](#sign-convention-from-the-imu-to-the-servo)
5. [RC channel map, ARM and failsafe](#rc-channel-map-arm-and-failsafe)
6. [Sensor data](#sensor-data)
7. [Breakdown of FlightController::update()](#breakdown-of-flightcontrollerupdate)
8. [HTTP API of the web dashboard](#http-api-of-the-web-dashboard)
9. [Console and diagnostics](#console-and-diagnostics)
10. [Board selection and pinout](#board-selection-and-pinout)
11. [How to add a new sensor](#how-to-add-a-new-sensor)
12. [How to add a new autopilot mode](#how-to-add-a-new-autopilot-mode)
13. [Feedback (groundwork, not connected)](#feedback-groundwork-not-connected)
14. [How to add a new board](#how-to-add-a-new-board)
15. [Build, upload and monitor commands](#build-upload-and-monitor-commands)
16. [Known limitations](#known-limitations)
17. [How to make changes](#how-to-make-changes)

---

## Architecture of the layers

Almost all classes live in headers laid out in the folders `include/<layer>/`. Each header includes what it uses itself (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ... — paths from `include/`). `src/main.cpp` is the single assembly point (the composition root): it creates all the objects, links them and runs `setup()`/`loop()`. The dependencies are one-directional — a lower layer knows nothing about an upper one.

```
include/
├── config/      Config.h (pins, all settings), Channels.h (channel names),
│                Controls.h (what each switch does — one line per channel)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (a register device on top of I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + wrappers over Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (settings in flash instead of NVS)
├── storage/     KeyValueStore, KvPreferences — settings storage without NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  groundwork for the feedback loop — NOT connected (see the section below)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (a tube made of two barometers)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32 firmware (S3, C3, 38-pin)
src/stm32/main.cpp  — STM32H743 firmware (FreeRTOS tasks, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — object assembly      │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — order of operations per tick  │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 modes   │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ navigation, PID    │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, flash    │
└───────────────────────────────────────────────────────────────────────┘
```

The rules that keep the architecture clean:

- **HAL** is the only layer allowed to know a specific MCU (`Wire`, `SPI`, `HardwareSerial`, `ledc*`). Everything above works only with interfaces. Moving to another MCU means a new `hal/<mcu>/<Mcu>Board.h`; the rest of the code does not change (an example is `hal/stm32/` for the STM32H743).
- **Sensor drivers do not know about the bus.** They receive an `IRegisterDevice&` — an I2C device with an address or an SPI device with a CS is created in `SensorSelection.h`. The same `BMP388_Sensor` works over both I2C and SPI.
- **What is shared lives in the base classes.** Calibration, axis rotation, signs, the orientation filter, altitude and vertical speed, compass calibration storage, bus error counting — in `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. A chip driver contains only the registers and the formulas from the datasheet.
- **RC and Outputs** know nothing about the airplane: iBUS bytes → channels, PWM values → outputs.
- **Control and Autopilot** are logic over data, with no UART, PWM or Wi-Fi. Time, where it is needed (flaps), is passed as a parameter.
- **Coordination** (`FlightController`) is the only class that sees several lower layers at once and decides the order of operations.
- **Application** (`main.cpp`) is the only place where `Esp32Board`, the devices and the sensors are created and where everything is wired together by hand, without a DI framework.

---

## FreeRTOS tasks and the control loop

| Where | What | Period |
|---|---|---|
| Core 1, `loop()` (the Arduino loopTask) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| Core 0, the `web` task | `WebServer::handleClient()` | every 2 ms |
| Core 0, the `oled` task | drawing the SSD1306 over the second I2C bus | 200 ms |
| Core 0 | the ESP-IDF Wi-Fi stack | — |

- The loop period is held by `vTaskDelayUntil`, not by `delay(2)` after the work —
  the frequency does not depend on how long the tick lasted. After a long block
  (a calibration from the console) the timing starts anew, and the missed ticks are
  not caught up in a burst.
- On the bench (ESP32-S3, all sensors): 500 Hz, about ~0.7 ms of work per
  tick on average, the worst tick ~1.4 ms. This is printed every 10 s as a `SYS:` line.
- The I2C transaction timeout is 5 ms (the stock one in Wire is 50 ms): a transaction
  hung by interference does not stop the loop for long.
- **Data separation between tasks.** The web and the OLED only *read* the
  state (`FlightController`/`Autopilot`/`LoopStats`) — these are separate
  16/32-bit fields, so in the worst case the values of adjacent ticks are visible.
  The dashboard *commands* (`setmode`/`setpid`) are not applied directly from the
  web task: they are put into a “mailbox” under `portMUX` and picked up by the
  flight loop in `WebDebugServer::applyPendingCommands()`.
- `Serial` (UART0 → the CH343 bridge → the “COM” connector) with a 4 KB transmit buffer:
  a debug frame (~600 characters) does not block the loop while it is being sent.

---

## File reference

### `config/`

| File | Responsible for |
|---|---|
| `Config.h` | All the pins (a block per board: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) and the settings: iBUS and link loss; control-surface travel; flaps; servo reverse; IMU and compass mounting; ARM; failsafe (RTH or gliding); the pitot tube (`PITOT_*`); all the numbers of the autopilot modes and functions; the loop; Wi-Fi; MAVLink; debugging |
| `Channels.h` | Channel names: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | The `BINDINGS` table: what each switch and knob does, one line per channel, `static_assert` checks |

### `hal/`

| File | Responsible for |
|---|---|
| `IBoard.h` | The entry point to the hardware: `i2c()`, `displayI2c()` (a second bus for the display, may be `nullptr`), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, may be `nullptr`), `servo(ServoChannel::*)` (7 outputs with AUX1/AUX2), `setBuzzer()` |
| `Rtos.h` | FreeRTOS tasks in the same way on the ESP32 (core 0) and the STM32 (priorities), the free heap |
| `II2CBus.h` | The I2C bus: primitives shaped like `Wire` + the helpers `writeRegister()`, `readRegisters()` (checks that exactly `count` bytes arrived), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, one PWM output (`measurePulseUs()` — diagnostics of the real pulse) |
| `RegisterDevice.h` | `IRegisterDevice` — “a set of 8-bit registers”; `I2cRegisterDevice` (address), `SpiRegisterDevice` (CS, frequency, dummy bytes before the data) |
| `esp32/Esp32Board.h` | The `IBoard` implementation: `Wire` (sensors), `Wire1` (the display, if the chip has two I2C controllers), `SPI`, two `HardwareSerial`, 5 LEDC channels |
| `esp32/Esp32I2CBus.h` | `II2CBus` on top of any `TwoWire`, 5 ms timeout |
| `esp32/Esp32ServoOutput.h` | PWM through LEDC: 50 Hz, 14 bits; pin −1 — the output is not routed. The ESP32Servo library is not used — see [limitations](#known-limitations) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | Thin wrappers over `SPI` and `HardwareSerial` |
| `stm32/*` | STM32H743: `Stm32Board` (+ the radio modem's UART4), buses, PWM timers, `Stm32FlashStorage` (settings in a flash sector, written by a background task), `compat/Preferences.h` |

### `storage/`

| File | Responsible for |
|---|---|
| `KeyValueStore.h` | An image “namespace/key → bytes” with CRC32 in RAM on top of any medium (`IFlashStorage`); an identical value is not rewritten |
| `KvPreferences.h` | The ESP32 `Preferences` API on top of `KeyValueStore` |

### `rc/`

| File | Responsible for |
|---|---|
| `RcChannelState.h` | A snapshot of the 10 channels |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → channels: a 32-byte frame, CRC, the channel value is the low 12 bits (`& 0x0FFF`); `isSignalLost()` = no frames (or none yet) ∥ the failsafe throttle value; frame counters |

### `control/`

| File | Responsible for |
|---|---|
| `ControlCommand.h` | The surface command in physical signs — the common language of the sticks, the autopilot and the mixer |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(target, now)`; `mix(command)` → PWM with servo reverse; flaperons: the ailerons `flaps ± roll` (minus — air brake) |
| `FlapsController.h` | Smooth extension/retraction of the flaps, with time passed as a parameter |
| `ThrottleManager.h` | Throttle from the stick; on link loss — `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM with the SwA switch (an OFF→ON transition with the throttle down + the mode's sensor checks), instant DISARM |
| `FlightOutputState.h` | The desired PWM: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (payload), `aux2` (camera) |
| `Beeper.h` | The buzzer: by the `BEEPER` function or “model lost” on the ground |
| `FlightOutputs.h` | The output table (`outputInfo()`: key, name, pin, whether it is mandatory, the state field) and everything on top of it in a loop: `begin()`, `write()`, `setFailsafe()`, status, `printPulseSelfTest()` |
| `FlightController.h` | The order of operations per tick, link loss (`applyLinkLoss()`), getters for telemetry |

### `autopilot/`

| File | Responsible for |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 modes), `Feature`, `Knob`, `PilotInputs`, names |
| `ControlBinding.h` | `Binding`, the factories `Bind::modes/mode/feature/knob`, the `BindingCheck` checks |
| `PilotSwitches.h` | The bindings table → the mode, functions and knobs of each tick; the layout at power-on |
| `Autopilot.h` | 12 modes, failsafe RTH/gliding, geofence, home, turn coordination, auto-trim; `update(armed, linkLost, throttle, sticks)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (distance, bearing, offset), `Guidance` (roll for a course, the circle vector field) |
| `AltitudeSpeedController.h` | Pitch for altitude, throttle for airspeed (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | The hand-launch and soaring state machines |
| `AutoTrim.h` | Auto-trim, stored in NVS/flash |
| `PidController.h` | PID: D on the sensor rate (gyroscope, variometer), anti-windup, the integrator frozen without ARM |
| `feedback/*` | **Groundwork, not connected:** adaptive feedback, takeoff and landing — see [Feedback](#feedback-groundwork-not-connected) |

### `sensors/`

| File | Responsible for |
|---|---|
| `SensorInterface.h` | The `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` interfaces and the data structures |
| `SensorSelection.h` | Which chip is compiled in (`#define SENSOR_*`, can be overridden with a build flag) and on which bus (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | Rotating the chip axes into the airframe axes (0/90/180/270° clockwise) — for the compass and for an IMU without mounting calibration |
| `imu/ImuOrientation.h` | IMU mounting as a "chip axes → airframe axes" matrix: from `IMU_ROTATION_CW_DEG` or from three poses (level, nose up, right wing down) with a sanity check; stored in NVS |
| `imu/ImuSensorBase.h` | Common IMU code: gyro calibration + preflight check (stillness, 1g, "up" matches the mounting), mounting calibration (`calibrateOrientation()`), scale, rotation, aviation signs, bus errors |
| `imu/AttitudeEstimator.h` | Complementary filter for roll/pitch, yaw integral |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (chip detected by WHO_AM_I): ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz. **On the bench** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, UI filter 50 Hz. Not tested on hardware |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2; I2C 0x6A/0x6B or SPI. Not tested on hardware |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1.6 kHz, low-pass filter through the indirect IPREG registers; I2C 0x68/0x69 or SPI. Not tested on hardware |
| `baro/BarometerBase.h` | Common barometer code: polling only for new samples, altitude, vertical speed through a low-pass filter, base calibration, errors |
| `baro/BMP388_Sensor.h` | BMP388 over I2C or SPI (with the SPI dummy byte), Bosch compensation, reading on the data-ready flag. **On the bench (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, Bosch compensation §8.1. Not tested on hardware |
| `baro/SPL06_Sensor.h` | SPL06-001: coefficients and formulas from the datasheet, 32 Hz ×16; I2C 0x76/0x77 or SPI. Not tested on hardware |
| `baro/BMP581_Sensor.h` | BMP581: the BMP5_SensorAPI sequence, 16×/2×, IIR; I2C 0x46/0x47 or SPI; works as both the main barometer and the pitot tube. Not tested on hardware |
| `mag/MagnetometerBase.h` | Common compass code: 50 Hz polling, hard-iron calibration in NVS, axis rotation, heading, errors |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **On the bench** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz. Not tested on hardware |
| `gps/UbloxM10_Gps.h` | u-blox M10: CFG-VALSET setup (115200 baud, 10 Hz, NAV-PVT, no NMEA), NAV-PVT parsing. Not connected on the bench |
| `airspeed/AirspeedSensor.h` | The airspeed sensor interface: differential pressure, IAS, TAS, density |
| `airspeed/PitotDualBaroAirspeed.h` | The home-made pitot tube: a BMP581 in the tube + a fuselage barometer; zero on the ground, low-pass filter, density from static pressure, fault detection |

### `telemetry/` and the application

| File | Responsible for |
|---|---|
| `DebugLogger.h` | Per-channel log (`LogSettings.h`): each channel has its own line, its own debounce tolerance and mode; stays silent while the menu is open |
| `DebugConsole.h` | A text menu in the port monitor (`h`) and hotkeys (`l`/space/`s`/`i`/`o`/`m`/`p`/`b`); saves the log settings to NVS when you leave the menu and only without ARM |
| `LogSettings.h` | The log channels (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) and their modes: off / on change / continuous; stored in NVS |
| `WebDebugServer.h` | The access point, the routes, the `/api/status` JSON, the command mailbox; runs in its own task on core 0 |
| `WebDashboardPage.h` | The dashboard HTML/JS as a single literal; the browser builds the channel/output/sensor rows from the JSON |
| `OledDisplay.h` | SSD1306 through U8g2 on top of `II2CBus`, its own task (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | MAVLink 2 for QGroundControl / Mission Planner: frames, streams, PID parameters, changing the mode from the ground |
| `LoopStats.h` | Frequency, mean and worst cycle time per second (OLED) and the worst since the last read (`takePeakUs()`, the SYS line) |
| `src/main.cpp` | ESP32: creating the objects, `setup()`, `loop()` with `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743: the same objects, MAVLink, the SD-card black box, the `flight`/`storage`/`oled`/`bbox` tasks |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: SDMMC1 pins and clocks for `HAL_SD_Init`; the console `D` key — reboot into the USB DFU bootloader |

---

## Sign convention: from the IMU to the servo

One sign system for the whole chain — so the stick and the autopilot are
guaranteed to move the control surfaces in the same direction, and the
direction of each servo is set in exactly one place.

**1. Sensor axes → airframe axes.** `ImuSensorBase` rotates the chip axes
with the `ImuOrientation` matrix (body = R · chip) into the airframe axes: X
toward the nose, Y to the left, Z up. The matrix is taken:

- from the **mounting calibration** (the `o` command, stored in NVS) — the
  board can sit in any orientation. Three poses: "level" gives the Z axis
  (and the horizon — the accelerometer zero offset is folded into it), "nose
  up" gives the X axis (the part of "up" perpendicular to Z), "right wing
  down" gives the Y axis. The nose from step 2 and the nose from step 3
  (Y × Z) must agree to within ~25°, otherwise the pilot tilted the wrong
  way — the calibration is rejected; the result is the average of the two
  estimates. Verified on 300 random mountings
  (`test/test_imu_orientation`, error < 0.1°);
- otherwise — from `Config::IMU_ROTATION_CW_DEG` (board with the chip facing
  up; the value is where the *chip's* X axis points if the nose is "12
  o'clock"), and the horizon is the pose at power-on.

On every gyro calibration (power-on, `i`) there is a **preflight check**:
gyro noise < 0.5 °/s (stillness; at rest ~0.08), |a| ≈ 1g, "up" within 45°
of the stored one (the board has not been moved). If it fails —
`ImuSensor::getPreflightProblem()` ≠ nullptr: `ArmingManager` does not arm
the stabilized modes, and `Autopilot::imuReady()` = false (zero corrections
in all modes, including gliding on link loss).

> On the current GY-521 (an MPU6500 clone) the chip is soldered rotated by 90°
> relative to the printed arrows: the X arrow on the silkscreen = the chip's
> Y axis. That is why, without mounting calibration, `IMU_ROTATION_CW_DEG = 90`.
> The check after any rearrangement: nose up → P increases to the positive
> side, right wing down → R to the positive side.

**2. Angles and rates (`ImuData`) — aviation signs:**

| Quantity | "+" means |
|---|---|
| `roll`, `gyroX` | right wing down |
| `pitch`, `gyroY` | nose up |
| `yaw`, `gyroZ` | nose right (clockwise seen from above) |

**3. The command (`ControlCommand`, µs of deflection, ±500 = full travel):**

| Field | "+" means | From the stick |
|---|---|---|
| `roll` | roll right (right aileron up, left down) | CH1: 2000 = right |
| `pitch` | nose up (elevator up) | CH2 with the opposite sign: 2000 = away from you = nose down |
| `yaw` | nose right (rudder and nose wheel to the right) | CH4: 2000 = right |
| `flaps` | flaps down (both ailerons down) | SwB (CH6): 0 or `FLAPS_DEPLOYED_US`, smoothly over `FLAPS_TRANSITION_MS` |

The PID computes `error = target − actual`: roll right (roll > 0) → a
negative roll command → the airplane levels out. The autopilot corrections
are added to the stick command **before** the mixer, in the same signs.

**4. Command → PWM.** `ControlMixer::mix()` computes the trailing-edge
deflection of each surface (ailerons: down = "+", left = `flaps + roll`,
right = `flaps − roll`; elevator: up = "+"; rudder: right = "+") and
converts it to PWM `1500 ± deflection`, flipping the sign for servos with
`Config::*_REVERSED = true`. The default values reproduce the firmware's
earlier behavior for the sticks. The check on the assembled airplane is in
the preflight checklist of [`PILOT_GUIDE.md`](PILOT_GUIDE.md). The reversal
must be changed in `Config.h`, **not on the transmitter** — otherwise the
stick and the autopilot will diverge.

---

## RC channel map, ARM and failsafe

The source is `include/config/Channels.h`. An FS-i6 transmitter (10 channels,
mode 2) + an FS-iA6B receiver, iBUS 115200.

| Channel | Transmitter control | Name | Purpose |
|---|---|---|---|
| CH1 | right stick ←→ | `AILERON` | Roll |
| CH2 | right stick ↑↓ | `ELEVATOR` | Pitch |
| CH3 | left stick ↑↓ | `THROTTLE` | Throttle, full travel; < 950 = receiver failsafe |
| CH4 | left stick ←→ | `RUDDER` | Rudder + steering wheel (one servo) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (on the FS-i6 this is the switch down, toward you) |
| CH6 | SwB | `SWB` | flaps by default (≥ 1750 — deployed) |
| CH7 | SwC (3 positions) | `SWC` | mode by default: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | RTH by default |
| CH9 | VrA | `VRA` | stabilization strength by default |
| CH10 | VrB | `VRB` | cruise speed by default |

CH6–CH10 are assigned with a single line in `include/config/Controls.h`
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#assigning-a-function-with-a-single-line)).

**ARM** (`ArmingManager`): the switch goes OFF→ON, throttle < `THROTTLE_LOW_US`,
and the sensor checks for the current mode have passed. Otherwise, it refuses,
with the reason printed to Serial, and a new OFF→ON cycle is needed. Powering
the board on with the switch already ON does not arm. OFF — DISARM at once.
While not armed, the throttle to the ESC is forced to `PWM_MIN`.

**Link loss** (`IBusReceiver::isSignalLost()`):

1. No frames for longer than `RX_TIMEOUT_US` (500 ms) — a broken wire or
   lost power to the receiver. Before the first frame after power-on the link
   is also considered lost: the default channel values (all 1500) are not
   taken for transmitter commands.
2. Throttle < `RX_FAILSAFE_THROTTLE_US` (950) — the failsafe set in the
   transmitter. **When the transmitter is lost, the FS-iA6B does not stop
   sending frames**, it repeats the last values (verified on the bench), so
   without a failsafe configured in the transmitter the link loss is not
   detected. The setup is described in `PILOT_GUIDE.md`.

What happens on link loss (`FlightController::applyLinkLoss()`):

- **the aircraft is armed, GPS and a home point are available**
  (`FAILSAFE_RTH`) — **return home** with the motor, circling over home; on
  the OLED — `FSRTH`, in the log — `FAILSAFE_RTH`;
- **the aircraft is armed, no GPS** — **gliding**, motor at
  `FAILSAFE_THROTTLE`: in any mode, even MANUAL, `Autopilot` holds the roll
  `FAILSAFE_GLIDE_ROLL_DEG` (0 — straight, 10–20° — a circle over the pilot)
  and the pitch `FAILSAFE_GLIDE_PITCH_DEG` (−3°, so as not to lose speed
  without the motor), the flaps are retracted; on the OLED — `GLIDE`, in the
  log — the `FAILSAFE_GLIDE` mode;
- **not armed** (on the ground) or the IMU does not respond — the control
  surfaces go to neutral;
- the mode and functions are not switched from the switches, the sensors
  keep being read. ARM is not cleared — once the link is restored the
  airplane obeys the sticks and the selected mode again (automatic takeoff
  and hand launch only start over).

---

## Sensor data

The structures are in `include/sensors/SensorInterface.h`.

### `ImuData`

| Field | Unit | Meaning |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | Angular rates in the airframe axes, aviation signs (see above) |
| `accelX`, `accelY`, `accelZ` | g | Acceleration in the airframe axes: X toward the nose, Y to the left, Z up |
| `roll`, `pitch` | ° | Complementary filter (α = 0.98, τ ≈ 0.1 s); they start straight from the accelerometer angle |
| `yaw` | ° | Gyro integral, drifts slowly; the initial value is the compass heading |
| `temperature` | °C | Die temperature (formula for the MPU6050 or MPU6500) |
| `timestamp` | µs | `micros()` at the moment of the read |

IMU calibration (at every start and with the `i` command): 2 s motionless, the
gyro → zero offset, the accelerometer → **the current position becomes the
horizon**.

### `BarometerData`

| Field | Unit | Meaning |
|---|---|---|
| `pressure` | Pa | Pressure |
| `temperature` | °C | Sensor temperature |
| `altitude` | m | Altitude **relative to the calibration point** (at start); formula `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | Derivative of altitude over the real samples (50 Hz) through a low-pass filter with τ = 0.5 s |
| `timestamp` | µs | Moment of the last new sample |

### `MagData`

| Field | Unit | Meaning |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | The field after hard-iron calibration, in the airframe axes (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, without tilt compensation; the direction of the reading has not yet been verified on an assembled airplane |
| `timestamp` | µs | Moment of the read (50 Hz) |

### `GpsData`

| Field | Unit | Meaning |
|---|---|---|
| `latitude`, `longitude` | ° | From UBX-NAV-PVT |
| `altitude` | m | Above sea level (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | Ground speed and course over ground |
| `numSatellites`, `fixType` | — | 0 = no fix, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | The module's accuracy estimates |

**What `isAvailable()` means.** For I2C sensors — the sensor answered at
`begin()` **and** the latest reads are not failing in a row (MPU — ~0.1 s,
barometer and compass — ~0.5 s without a response). When a read fails the
data is not overwritten with garbage: the previous values stay and the error
counter grows (visible with the `s` command). For GPS — at least one valid
NAV-PVT, and the latest one is not older than `GPS_TIMEOUT_US`.

**If a sensor is absent** (`nullptr` or `isAvailable() == false`), `Autopilot`
gives no corrections, and the airplane is flown as in MANUAL. `main.cpp`
calibrates only the sensors that responded.

---

## Breakdown of FlightController::update()

Called from `loop()` every 2 ms. The order is the priority:

1. **`receiver.update()`** — parsing the accumulated iBUS bytes.
2. **Switches** — `switches->update(rc)`, only while the link is alive (in a
   failsafe frame the channels do not reflect the switches): the mode (only
   on a change), functions, knobs.
3. **Pilot throttle** — `throttle.update(rc, receiverFailsafe)`.
4. **Sticks** — `mixer.fromSticks(rc)` × `Knob::RATES`; flaps —
   `mixer.updateFlaps(target)` (brake, switch, knob; without a link — 0).
5. **Sensors and autopilot** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **always**, even without a link: the angle filters must not freeze. While
   not armed, the PID runs (the control surfaces respond to tilting — handy
   on the desk), but the integrator is held at zero. Without a link and
   armed — failsafe RTH or gliding.
6. **Beeper** — `Beeper`.
7. **Link loss** — `applyLinkLoss()`: when armed — the control surfaces and
   throttle follow the autopilot's failsafe command, otherwise neutral and
   the motor off; `return`. Absolute priority over everything below.
8. **ARM** — `arming.update(rc, false)`.
9. **Command** — `autopilot->getCommand()`: in the stabilized modes the stick
   is the desired angle, and the autopilot issues the final control surface
   commands.
10. **Mixer** — `mixer.mix(command)` → PWM for the ailerons (flaps + roll),
    the elevator and the rudder, with reversal taken into account.
11. **Throttle** — `autopilot->applyThrottle(pilotThrottle)`: the pilot's
    throttle, the autopilot's throttle, or the max of the two (automatic
    takeoff). Then, if not armed or `MOTOR_KILL`, — forced to `PWM_MIN`. This
    check comes last so that no mode can slip the throttle past ARM.
12. **AUX** — payload (`PAYLOAD_DROP`) and camera (`CAMERA_TILT`, `CAMERA_STAB`).
13. **`outputs.write(output)`** — PWM to the 7 outputs.

---

## HTTP API of the web dashboard

The implementation is `include/telemetry/WebDebugServer.h`. The access point:
SSID `OpenPlane-Debug`, password `12345678`, address `http://192.168.4.1`.

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached` — the object exists in the build; `available` — the sensor
  really responds. The data fields are added **only** when `available: true`.
- `outputs.*.attached` — the MCU has allocated an LEDC channel and a pin;
  whether a physical servo is connected cannot be seen from software (to
  check the pulse use the console, command `p`).
- `rollCorr`/`pitchCorr` — the autopilot's final command minus the sticks,
  µs. `throttleCorr` — the autopilot's throttle, % (0 while the throttle is
  with the pilot).
- `nav` — navigation: home, the distance and bearing to it, the course and
  target course, the speed used for navigation (pitot tube / GPS), the
  geofence, stall; `features` — the switch functions that are enabled.

### `POST /api/setmode`

`{ "mode": 1 }` — the `AutopilotMode` number: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. The mode holds until the
pilot flips the mode switch.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — any of the fields `kpRoll`,
`kiRoll`, `kdRoll`, `kpPitch`, `kiPitch`, `kdPitch`; omitted ones keep their
previous values.

Both commands are applied by the flight loop on the next cycle (see
[FreeRTOS tasks](#freertos-tasks-and-the-control-loop)).

### `GET /`

The HTML dashboard: bars for the 10 channels, ARM/link, the outputs, the
sensors, mode buttons, the PID form. It polls `/api/status` every 200 ms.

---

## Console and diagnostics

The port monitor — 115200, the "COM" connector. The implementation is
`DebugConsole` and `DebugLogger` ([reference](reference/telemetry.md)). Keys
work immediately, Enter is not required; the calibrations and `p` block the
loop and are therefore available only without ARM.

| Key | What it does |
|---|---|
| `h` / `?` | Main menu |
| `l` | The "what to print to the log" menu (channels, modes, period) |
| space | Pause the log / resume |
| `s` | `printStatus()` of all sensors: data, bus error counters, calibrations, the preflight check |
| `i` | Gyro calibration + preflight check (2 s motionless) |
| `o` | IMU mounting calibration by three poses, saved to NVS |
| `m` | Compass calibration (15 s of rotation), saved to NVS |
| `p` | Output self-test: the real pulse on each pin against the expected one |

The log is split into channels (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`, `MAG`,
`GPS`, `IMU`, `SYS`), each with a mode "off / on change / continuous"; the
settings are stored in NVS and written when the menu is closed, only without
ARM. By default, `STAT` (on change) and `SYS` (once every 10 s) are enabled:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (worst over 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

The formats of all the channels are in the [reference](reference/telemetry.md#debuglogger).

---

## Board selection and pinout

| Command | `board` | Macro | Status |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **Main, the default.** Tested on the bench with all the sensors |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | The old prototype, flown under manual control |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | For the bench, the pinout has not been tested on hardware |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: the full firmware + MAVLink + an SD-card black box; tested on a bare board ([below](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | The same on the DevEBox H743: the console is USB CDC, flashing over DFU |

| Purpose | ESP32-S3 (bench) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| Aileron left / right | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| Elevator / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| Rudder | GPIO18 | — (no pin) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| Sensor I2C SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| OLED I2C SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / none (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → the "COM" connector | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** GPIO33–37 are taken by the octal PSRAM, 26–32 by the
  flash, 19/20 by USB, 43/44 by Serial, 48 is the RGB LED; 0/3/45/46 are
  strapping pins.
- **ESP32-C3:** the ailerons on GPIO4/5 are swapped relative to the S3. There
  are not enough pins for the full set: the BMP388 CS and the GPS RX are on
  strapping pins, the GPS has no TX (receive only, no UBX-CFG). Details are
  in `Config.h`.

### STM32H743

The STM32H743VIT6 (Cortex-M7 480 MHz, 2 MB of flash, 1 MB of RAM) runs the
**full firmware**: the same sensors, autopilot, switches, console and display
as on the ESP32-S3, plus MAVLink telemetry and an SD-card black box. It
builds, passes cppcheck and all the native tests of the shared code. On
hardware, the **DevEBox H743 board without sensors** has been tested: boot,
the console over USB, the SD card, the black box —
[TESTING.md](TESTING.md#tests-on-the-stm32-board) — as well as iBUS, ARM and PWM
to the servos and the motor: control from the transmitter in manual mode (on
video). The sensors on the STM32 are still waiting for a bench. The main
flight board is the ESP32-S3.

- **HAL** — `include/hal/stm32/`: `Stm32Board` (the same API as `Esp32Board`,
  plus `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (hardware PWM from `HardwareTimer`, one timer for
  several outputs). In detail — [reference/hal.md](reference/hal.md#implementation-for-the-stm32h743).
- **Settings and calibrations** — not NVS but a `KeyValueStore` in the last
  flash sector (`include/storage/`, `hal/stm32/Stm32FlashStorage.h`). The
  project code still writes `#include <Preferences.h>`: in the `stm32h743`
  env the `include/hal/stm32/compat/` directory is on `-I`, and a
  `Preferences` with the same API lives there. The image carries a CRC32: a
  corrupted one (power lost during the erase) reads as empty. The flash
  write happens in a background task: erasing a 128 KB sector takes seconds,
  but the sector is in bank 2 while the code executes from bank 1, and the
  flight task preempts the background one without stopping.
- **Tasks** — FreeRTOS from the STM32duino FreeRTOS library, a single core,
  priority preemption (`hal/Rtos.h`): `flight` (5) — the flight loop,
  MAVLink, log, console; `oled` (1) and `storage` (1) — in the background;
  `bbox` (2) — writing the black box to the SD card.
- **The black box on the SD card** — SDMMC1, 4 bits, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, pins in `src/stm32/sd_msp.cpp`). The card
  stays an ordinary FAT32: a pre-created `BLACKBOX.BIN` file sits on it, the
  firmware writes raw blocks inside it and does not touch the file system
  itself (`storage/Fat32File.h` is read-only). Preparing the card and
  downloading — [BLACKBOX.md](BLACKBOX.md#sd-card-stm32h743).
- **Telemetry** — MAVLink 2 on UART4 (`telemetry/MavlinkTelemetry.h`) instead
  of the Wi-Fi dashboard: QGroundControl / Mission Planner, changing the mode
  and PID from the ground. In detail —
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#ground-station-wi-fi-dashboard-and-mavlink).
- **Pinout** — the `BOARD_STM32H743` block in `Config.h`, the pins chosen
  from the free ones on the WeAct MiniSTM32H743VITx and cross-checked against
  the STM32duino tables:

| Purpose | STM32H743 | Peripheral |
|---|---|---|
| Aileron left / right | PA0 / PA1 | TIM2_CH1 / CH2 |
| Elevator / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| Rudder | PD14 | TIM4_CH3 |
| AUX1 (payload) / AUX2 (camera) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX — spare) | PE7 (PE8) | UART7 |
| Sensor I2C SDA / SCL | PB11 / PB10 | I2C2 |
| OLED I2C SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / barometer | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| Radio modem MAVLink RX / TX | PD0 / PD1 | UART4 |
| Beeper | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)** — the `stm32h743-devebox` env: the same code, its
  own core variant, the console over USB-C as a virtual COM port (CDC) — no
  USB-UART needed. The first flashing is over USB through the built-in
  bootloader (DFU):
  1. Windows: install the WinUSB driver for "STM32 BOOTLOADER" once
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver).
  2. Connect the **BT0** (BOOT0) pin to **3V3** with a wire, press and
     release **RST**: the board is in DFU mode (the DevEBox has no BOOT0
     button).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. The BT0 wire can be removed — the firmware starts by itself.

  After that the wire is not needed: the **`D`** key in the console (from any
  menu, not while ARMed) reboots the board into the bootloader: a marker in
  RAM → reset → a jump into system memory before the clocks are configured
  (`src/stm32/bootloader.cpp`). Jumping straight from running firmware hangs
  on the H7 — verified on the board, hence the two steps. An open console
  (USB CDC) is needed; if the board does not respond — RST with the BT0 wire
  in place.
- **Entry point** — `src/stm32/main.cpp` (excluded from the ESP32 builds via
  `build_src_filter`). The objects are the same as in `src/main.cpp`; instead
  of `loop()` there are tasks, and `vTaskStartScheduler()` is at the end of
  `setup()`.
- **First power-up of the board:** `pio run -e stm32h743 -t upload` (ST-Link),
  the monitor on LPUART1 through a USB-UART; `b` — whether the sensors are
  visible on the buses, `s` — sensor status, `p` — pulses on the outputs
  (take the propeller off), then the transmitter and QGroundControl through
  the radio modem.

---

## How to add a new sensor

### A) Another chip of an existing category (IMU, barometer, compass)

The common code is already written in the base classes — a chip driver comes
out small:

1. Create `include/sensors/<category>/<Name>_Sensor.h` and inherit from
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. The constructor
   takes an `IRegisterDevice&` — the driver does not know whether it is I2C
   or SPI.
2. Implement:
   - `begin()` — `device.begin()`, check the chip ID, write the registers,
     call `setAvailable(true/false)`;
   - IMU: `readSample()` (raw accel/gyro/temp in the chip axes),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`;
   - barometer: `isNewSampleReady()` (a ready flag or simply `true`) and
     `readSample()` (pressure in Pa, temperature in °C), the polling period
     is in the base constructor;
   - compass: `readRaw()` (X/Y/Z in the chip axes) and
     `lsbPerMicroTesla()`, the NVS namespace name for the calibration is in
     the base constructor.
3. If the chip needs a dummy byte before the data over SPI or a special
   frequency — add a static factory `spiDevice(bus, cs)`, like the one in
   `BMP388_Sensor`.
4. A branch in `SensorSelection.h`: `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;` and `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` or the SPI factory). `main.cpp`
   is not touched when the sensor changes.
5. Check the build with the new sensor without editing the file — with a
   flag: `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`,
   then all three environments, then on hardware.

### B) A new category

1. The data structure and the interface go in `SensorInterface.h`, modeled on
   `GpsSensor`/`GpsData`.
2. If the category has common logic (filters, calibration) — a base class
   modeled on `BarometerBase`.
3. A nullable pointer in the `Autopilot` constructor (no sensor — no effects,
   rather than a crash) and fields in `GET /api/status` with an
   `attached`/`available` pair.

### A new bus or peripheral

A new interface in `include/hal/`, an implementation in `include/hal/esp32/`
and in `include/hal/stm32/`, access through `IBoard`.

---

## How to add a new autopilot mode

1. A value in `enum AutopilotMode` (`autopilot/AutopilotTypes.h`, before
   `MODE_COUNT`), a name and a short name (up to 5 characters, for the OLED)
   in `AutopilotNames::mode()` / `modeShort()`.
2. A handler `run<Mode>()` and a branch in `Autopilot::runMode()`; the initial
   targets (course, altitude, circle center) go in `initializeMode()`. The
   mode sets `desiredRoll`/`desiredPitch` and calls `stabilizeOrManual()`
   (without an IMU the pilot has the control surfaces) or
   `stabilizeOrNeutral()` (without an IMU — neutral). Without the required
   sensor — safe behavior, not a crash. The integrator accumulates only when
   `armed`.
3. Throttle: `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) and
   `autoThrottlePct`, or `autoThrottle()` — the cruise throttle from the
   knob / from the pitot tube. `FlightController` does not change.
4. On the transmitter — a single line in `config/Controls.h`
   (`Bind::mode(Channels::SWD, MODE_NEW)`). The dashboard and MAVLink pick up
   the mode by its number; for MAVLink — the nearest ArduPlane mode in
   `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. If the mode needs sensors for ARM — `ArmingManager`.
6. Tests: the reaction to each sensor — `test/native/test_autopilot_modes`,
   closed-loop flight — a scenario in `test/native/test_sim` (the airplane
   model `helpers/PlaneSim.h`, the harness `helpers/SimHarness.h`). Then — the
   desk without the propeller: the control surfaces must respond to tilting
   in the direction of leveling.
7. A section in [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## Feedback (groundwork, not connected)

`include/autopilot/feedback/` is the next step for the autopilot. **Neither
`FlightController` nor `Autopilot` nor `main.cpp` includes these files:**
there is no prototype for flight tests yet, and the firmware works without
them. They are verified by a closed-loop simulation (`test/test_feedback/`)
right on the board.

### Why

Today `Autopilot` is a PID on the angle: error × gain = control surface. It
does not know what came of that on the airplane, and the gains are correct
only for one speed: at low speed the surface is weaker and the PID
under-corrects, at high speed it over-corrects. Feedback closes the loop on
**the airplane's response**:

- the surface was deflected but the airplane rotates slower than needed — add
  more, until it gets there;
- how much surface is needed is measured in flight and recalculated with
  speed;
- the airplane rotates the wrong way — the sign is mixed up, flip it and
  check;
- the angle was leveled but the speed is dropping — throttle up and nose
  down, until the airplane stalls;
- takeoff and landing — in phases, by what the sensors show.

### Modules

| File | What it does |
|---|---|
| `FlightSnapshot.h` | Everything the feedback knows about the airplane for one cycle. The only input: the modules do not read the sensors and RC directly, so they can be run on a simulation and on logs |
| `FeedbackOutput.h` | The output for one cycle: control surface deflections per axis, whether the axis is enabled, the axis sign, throttle (set / not below), the reason |
| `FeedbackConfig.h` | All the constants (they will move into `Config.h` when it is connected) |
| `SpeedEstimator.h` | Speed (pitot tube > GPS) and longitudinal acceleration from the IMU: `dV/dt = g·(ax − sin θ)` — "speed is dropping" is visible even without an airspeed sensor |
| `AirborneDetector.h` | In the air / on the ground: learning, accumulating the integral and looking for a stall only make sense in flight |
| `ControlEffectivenessEstimator.h` | For each axis it learns the model `ε = b·u(t−delay) + a·ω + c` with recursive least squares |
| `AdaptiveRateController.h` | A cascade angle → angular rate → angular acceleration → control surface through the learned model |
| `StallGuard.h` | Protection against loss of speed and stall |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | Takeoff (from a runway or by hand) and landing in phases, by the sensors |
| `FeedbackSupervisor.h` | Everything together: the order within a cycle, the priorities, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, the connection plan |
| `FeedbackModules.h` | A single include for everything |

### How it works

**Control surface effectiveness.** The axis model: angular acceleration
`ε = b·u + a·ω + c`. `b` is how many °/s² 1 µs of surface gives (the sign is
the direction of the response), `a` is damping (the air brakes the rotation;
without this term the estimate of `b` would go to zero during a steady
rotation), `c` is a constant moment (center of gravity, trim, propeller). The
force of a surface ∝ ρV², so `b` is learned at a reference speed and
multiplied by `(V/Vref)²`: the airplane speeds up — the surface instantly
"becomes stronger" without relearning. The indicated airspeed from the pitot
tube already contains the air density, so altitude is accounted for by
itself; without an airspeed sensor the scale is 1, and `b` is learned
directly.

The data is taken over 20 ms intervals: the mean acceleration over an
interval is the difference of the gyro at the ends / the duration, and it
corresponds to the mean surface deflection and angular rate over the same
interval (the surface — with the delay `RESPONSE_DELAY_MS`). Then both sides
of the equation pass through the same 2 Hz low-pass filter: the relation does
not change, while the high frequencies, where the "pure delay" model lies
because of servo inertia, are removed. Learning is possible only in the air
and only while the surface is being "shaken" (a swing ≥ `MIN_EXCITATION_US`
over ~0.3 s); the pilot's sticks are shaking too, so the estimate learns in
MANUAL as well.

**The controller.** Three stages, axis by axis:

```
ω* = ANGLE_GAIN · (target − angle)              "nose is 10° down — raise it at 40°/s"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "rotating slower than needed — correct it"
surface = (ε* − a·ω − c) / b                    through the learned model
```

The integral `I` is stored in °/s, not in µs of surface — so it stays
correct when the estimate of `b` changes. On the ground the integral is
frozen (except for the heading during the takeoff roll / rollout), and at the
surface limit it does not accumulate toward the limit. A coordinated turn is
taken into account (if the speed is known): in a bank the pitch needs
`g·sin φ·tg φ / V` and the yaw `g·sin φ / V`.

**Axis signs — on the ground only.** In flight the axes are neither disabled
nor flipped: the IMU mounting is determined by the `o` calibration and the
check at power-on, and the surface directions by the pilot's preflight
check. Indirect signs in the air (a departure, a spin, maneuvers, gusts) can
deceive, and a disabled or flipped axis at such a moment costs the airplane.
If the estimate of `b` for an axis is confidently negative, it is only a
warning in `reason` ("responds to the surface backwards? check on the
ground"); a negative estimate does not go into the controller — the axis
works on the a priori model.

**Stall protection.** Two levels. *LowEnergy* — the speed is dropping fast
with the nose raised, or is close to the stall (< 1.25·Vs), or the elevator
has lost effectiveness: throttle ≥ 80 %, pitch ≤ 5°. *Stall* — the speed is
below the stall, the nose drops against the elevator, the wing drops against
the ailerons at low energy: full throttle, nose down, bank ≤ 10°, ailerons
limited (a large aileron stalls the wingtip). The measures are released with
hysteresis (speed ≥ 1.5·Vs). On link loss the throttle is not touched, and
right near the ground (the flare, the rollout) the protection is off —
landing is itself a controlled stall.

**Takeoff.** `WaitThrottle` (the motor is stopped) → the pilot gave throttle
≥ 50 % → `GroundRoll` (full throttle, wings level, the heading held by the
rudder and the wheel, the elevator free) → liftoff speed or a timeout
without a speed sensor → `Climb` (12°, full throttle) → altitude 30 m →
`Complete`. By hand (`TAKEOFF_HAND_LAUNCH`), instead of the roll — `WaitLaunch`:
the motor starts only after the throw (longitudinal acceleration ≥ 1g). The
throttle pulled back before liftoff — cancel.

**Landing.** `Approach` (throttle 25 %, descent 1 m/s — the pitch from the
vertical speed error, the bank from the pilot ≤ 20°) → altitude 2 m →
`Flare` (throttle 0, the descent is damped to 0.3 m/s by the same rule) →
impact on the accelerometer or "low and not rotating" → `Rollout` (heading
with the wheel) → `Complete`. Pilot throttle ≥ 80 % — go around. The flare
needs a rangefinder: the barometer is off by a meter.

**Priorities** (`FeedbackSupervisor`): not armed > stall protection >
takeoff/landing > mode targets. On link loss the phases are cancelled, and
stabilization carries out the failsafe gliding targets.

### Simulation

`test/test_feedback/test_main.cpp` (on a PC: `pio test -e native -f test_feedback`) — an airplane model (independent axes,
servo delay and inertia, control surface effectiveness ∝ V², damping ∝ V, constant
moments, lift through the angle of attack from speed, stall, landing gear with
a steering wheel) and 10 scenarios:

| Scenario | What is checked |
|---|---|
| Recovery from a 30° bank / −15° pitch with a constant moment | Leveling and "keep correcting": the integral finds the trim by itself |
| A ±15° shake at 14 and 20 m/s, without a speed sensor | The estimate of `b` converges to the truth and is rescaled with speed |
| A swapped aileron, the pilot rocks the wings in MANUAL | The estimate of `b` is negative → only a warning, the axis is not disabled |
| 30 s of turbulence | Gusts are countered, the bank does not go beyond 10° |
| Nose 15° at 20 % throttle (with a speed sensor and without) | The speed does not drop to a stall |
| Takeoff from a runway with the propeller's reaction torque | Phases, altitude, heading on the roll |
| Landing from 15 m | Phases, no throttle near the ground, a soft touchdown |
| Link loss on the takeoff roll; not armed; MANUAL | Cancellation, the throttle is not touched, the control surfaces stay with the pilot |

The model is crude — it checks the logic and the signs, not the tuning for a
specific airframe.

```bash
pio test -e native -f test_feedback      # on a PC, in seconds
pio test -e esp32-s3 -f test_feedback    # flashes the test firmware and runs it
pio run -t upload                        # restore the normal firmware
```

### Connection plan

1. `FlightController::update()`, after reading the sensors and computing the
   commands, fills in a `FlightSnapshot` and calls
   `FeedbackSupervisor::update()`. At first — **shadow mode**: the output goes
   only to the log (`printStatus()`) and to the dashboard, not to the control
   surfaces. In flight under manual control the estimate of `b` for each axis
   must be positive and grow with speed.
2. On the ground, the airplane in your hands, STABILIZE: tilt it — the
   control surfaces counter.
3. One axis at a time: `deflectionUs` instead of
   `Autopilot::getRollCorrection()` (roll only at first), then pitch.
4. Throttle: `throttleOverridePercent`/`throttleFloorPercent` — after
   `Autopilot::applyThrottle()`, before the failsafe (the failsafe outranks
   everything).
5. Takeoff/landing — onto a free switch; remove the `AUTO_TAKEOFF` mode from
   `Autopilot`.
6. The `FeedbackConfig` constants — into `Config.h`; the airspeed sensor — an
   `AirspeedSensor` implementation and a category in `SensorSelection.h`.

---

## How to add a new board

1. `[env:<name>]` in `platformio.ini` with a unique `-D BOARD_ESP32_<NAME>`.
2. A `#elif defined(BOARD_ESP32_<NAME>)` block in `Config.h` with all the
   pins, including `PIN_I2C2_SDA/SCL` (−1 if there is no OLED). Work out the
   GPIO budget in advance: flash/PSRAM/USB/strapping.
3. The servo outputs need 5 LEDC channels — every ESP32 has them. If there is
   no pin for the rudder — `PIN_RUDDER = -1`, and the output simply turns
   off.
4. Do not change `default_envs` until the board has been tested on hardware;
   state explicitly in the commit if the pinout has not been tested.

---

## Build, upload and monitor commands

```bash
pio run                        # build the default board (esp32-s3)
pio run -t upload              # upload
pio device monitor             # monitor, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # check that all the boards build
```

- **ESP32-S3:** uploading and Serial go through the "COM" connector (CH343).
  If the bridge hangs (Windows answers "the device is not functioning" —
  this can happen because of interference from the ESC), reconnecting the
  cable helps; you can also upload through the "USB" connector (the built-in
  USB-JTAG): `pio run -t upload --upload-port <USB COM port>`.
- While the port monitor is open, uploading to the same port will not work.
- `lib_deps`: `olikraus/U8g2` (OLED) is the only external library.
- `test/` — in detail in [`TESTING.md`](TESTING.md):
  - `pio test -e native -e native-stm32` — 387 tests on a PC (hardware fakes
    in `test/native/support/`), coverage — `gcovr`;
  - `pio test -e esp32-s3` — `test_feedback/` (the closed-loop feedback
    simulation) and `test_imu_orientation/` on the board itself; each one
    flashes a test firmware, afterward upload the normal one with
    `pio run -t upload`.
- Static analysis: `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (cppcheck over
  `hal/stm32/` and `src/stm32/`) and `tools/clang-tidy.sh`
  (the `.clang-tidy` profile).

---

## Known limitations

- **The autopilot has not been tested in flight.** On the desk the signs have
  been verified live (tilt → correction toward leveling), the PID gains are
  starting values.
- **STABILIZE is leveling on top of the sticks**, not an "angle mode"
  (FBWA) where the stick sets the roll/pitch angle. The pilot and the
  autopilot add up.
- **Gliding on link loss has not been tested in flight.** The
  `FAILSAFE_GLIDE_*` angles are starting values; the −3° pitch is chosen for
  a specific airframe (the nose must neither rise to a stall nor dive).
- **The horizon.** With the mounting calibration (`o`) — from it (NVS); the
  accelerometer zero offset drifts with temperature (~1–2° per 20 °C), if the
  horizon has "drifted" — repeat `o`. Without it — the pose at power-on
  (power on while level).
- **The compass mounting** is still set by `MAG_ROTATION_CW_DEG` (the
  pose-based calibration does not affect it).
- **Compass:** the heading is without tilt compensation, the direction of the
  reading has not been verified on an assembled airplane, and the calibration
  has to be done in the airplane itself. No mode uses the heading yet.
- **GPS** is not used for navigation; on the ESP32-C3 it is receive-only.
- **Feedback (`autopilot/feedback/`) is not connected** and has been verified
  only in simulation with a crude airplane model. All the numbers in
  `FeedbackConfig.h` marked "прикидка" ("rough estimate") need refining on a real
  airframe; there is no airspeed sensor yet (without it the control surface
  effectiveness is learned more slowly, and a stall is visible only by the
  deceleration).
- **Not tested on hardware:** `ICM42688_Sensor` (brought to the common
  convention through `ImuSensorBase`), `BME280_Sensor` (the Bosch
  compensation was reimplemented), BMP388 over SPI, `QMC5883L_Sensor`, the GPS
  setup through CFG-VALSET. When connecting — the boot log, `s` in the
  console, signs by tilting.
- **I2C on a breadboard picks up interference** from the ESC/motor (isolated
  errors are visible via `s`). The drivers survive them, but in the airplane
  the I2C wires should be short and kept away from the power wiring.
- **ESP32Servo is not used.** Version 3.2.1 on the ESP32-S3 distributes the
  servos across MCPWM and in `attachPin()` confuses the MCPWM unit number with
  the timer number: GPIO6/7 produced the signal of GPIO4/5 (the ESC was
  controlled by the right stick). The outputs were rewritten on LEDC; bring
  the library back only after checking with `p`.
- **The ESC is 50 Hz PWM**, the firmware has no throttle range calibration
  mode yet.
- **Web dashboard:** the access point password is weak, and commands are
  accepted in flight too. It is a tool for the bench and the field, not for
  flight.
- **Prototype mechanics:** the first prototype flew, a weak motor mount and
  insufficient wing stiffness were found.
- **The license is the OpenPlane License** ([LICENSE](../../../LICENSE)): MIT with mandatory attribution of the
  author, a ban on military use and a ban on intentional harm to people and
  property without their written consent. Do not add other license headers
  to the files and do not remove the author's name.

---

## How to make changes

- **Small commits:** one logical step — one commit.
- **Tests and analysis before a commit:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh` —
  all green ([`TESTING.md`](TESTING.md)).
- **Build all the boards** after changes in shared code — the S3 is the main
  one, but the C3, the 38-pin and `stm32h743` must not break; before a
  release — `tools/build_matrix.sh` (all boards × all sensors).
- **Check on hardware what can be checked:** signs — by tilting, outputs — with
  the `p` command, the link — by switching the transmitter off.
- **Do not invent APIs.** Consult the framework sources in
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x)
  — the internet often describes version 3.x with a different API (for example,
  LEDC).
- **Do not embellish the status.** Not tested on hardware — say so.
- **A layer must not know more than it should.** If a lower class suddenly
  needs a higher one, the logic must move up into `FlightController`.
- **When changing a data contract** (`FlightOutputState`, `ControlCommand`,
  `ImuData`, the JSON of `/api/status`) — update all the consumers in the same
  commit.
