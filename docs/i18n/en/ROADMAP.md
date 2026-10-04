# OpenPlaneProject — roadmap and pitch for investors and partners

> 🌐 This page is a translation of the [Russian original](../../ROADMAP.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

> Repository: [github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject), branch `main`.
> This document is a deeper version of the README teaser, addressed to those
> who are considering investing money, time or partnership in the project. It describes
> where the project stands now, where and why it is heading, and what exactly in the code
> that has already been written makes this trajectory realistic rather than merely declared.

## 1. Current state — honestly

OpenPlaneProject is currently a prototype glider that has flown but is immature, with its
own firmware on the ESP32 — not a finished product and not an autonomous drone.
Hardware: 1200 mm wingspan, 250 mm chord, NACA 4412 airfoil, a PETG structure
(3D-printed), MG90S servos (a separate one for each aileron), 3S LiPo power. The first prototype
(ESP32-C3, D2212 1000KV motor, 40A ESC) has already flown under manual control and, after the
flight, revealed specific problems: insufficient strength of the motor and wing mounts
(they need carbon reinforcement) and the need to tune the servos.

The current build has moved to the ESP32-S3 (N16R8) with a D3548 1100KV motor and a
60–80A ESC, and all the autopilot sensors are connected to it on the bench: an IMU (MPU6500),
a BMP388 barometer, a QMC5883P compass, plus an OLED status display. Verified live:
manual RC control over iBUS with a mixer for ailerons, elevator, rudder
(with a steerable wheel) and flaps (flaperons);
ARM with a separate switch; failsafe with the transmitter switched off; a 500 Hz control
loop; a live web dashboard over Wi-Fi. On the table, stabilization responds to tilts
in the right direction.

Since then the firmware has gained 12 autopilot modes (stabilization, altitude hold,
cruise, circles and return home by GPS, hand launch, auto-landing, soaring
in thermals, “rescue”), a geofence, return home on link loss, payload drop,
a pitot tube made of two barometers, support for new sensors (LSM6DSV, ICM-45686,
QMC6309, SPL06, BMP581), and a complete firmware for the STM32H743 with MAVLink telemetry
for QGroundControl. All of this has been checked by 387 automated tests and closed-loop
flight simulations (the whole firmware flies an airplane model), but **it has not yet been
tested in the air**. In other words: so far only manual control has flown; the autopilot has been
written, checked by everything that can be used to check it without flying, and is waiting for
flight tests.

## 2. Why this matters

Autonomous small aviation with a low barrier to entry covers tasks where what decides
is not maximum payload or range but reaction speed and low operating cost:

- **Delivery of medicines and blood to hard-to-reach and post-disaster
  areas** — washed-out roads, no infrastructure, destruction after natural
  disasters or conflicts. What decides here is not payload capacity
  (the parcel is small) but reaction time: minutes and hours instead of a day by
  car or on foot.
- **Search-and-rescue operations** — pinpoint drops of equipment, first-aid kits,
  communication gear and flotation devices to victims before the ground team arrives, in zones
  where a helicopter is excessively expensive or unavailable because of weather or terrain.
- **Precision agriculture** — crop monitoring and spot spraying/application of products
  where closed drone platforms for these tasks cost from thousands of dollars, which is
  unprofitable for small and medium farms.

In all three categories the economics are the same: the difference between “a solution exists,
but it is expensive and closed” and “there is effectively no solution, because it is expensive” — and it is exactly
this niche that a cheap open platform targets. This is a description of the application area
and the market problem, not a claim that OpenPlaneProject can already
deliver cargo — at the current stage it is a statement about the goal and about why the
goal is worth pursuing.

## 3. Investment thesis: why an open ESP32 architecture is an asymmetry

Commercial autonomous delivery/monitoring drone platforms are usually built
on closed flight controllers and closed software, cost from hundreds to thousands of
dollars per aircraft, and require license fees or service contracts for
fleet operation. OpenPlaneProject proceeds from a different assumption:

- **A cheap base.** An ESP32 module costs on the order of $5–15, and the surrounding hardware (MG90S
  servos, an ESC, an iBUS receiver) consists of standard hobby-grade components. That is an
  order of magnitude cheaper than the entry ticket to closed commercial platforms, which is
  critical for pilot deployments in budget-constrained settings (NGOs,
  small-scale agriculture, regional rescue services).
- **Open code changes the economics of trust.** An organization deploying
  a fleet for medical delivery can audit the safety logic (failsafe,
  ARM) and adapt the firmware to its own sensors and regulations instead of
  depending on a single vendor and its roadmap.
- **The architecture is already designed for expansion today, not only for the
  current glider.** This is not a declaration but a direct consequence of how the code is structured:
  - A new sensor is added as a class implementing the existing
    `Sensor` → `ImuSensor`/`BarometerSensor` interface (`include/sensors/SensorInterface.h`),
    without changing the core. This is how the drivers for IMUs (MPU6050/MPU6500,
    ICM-42688), barometers (BMP388, BME280), compasses (QMC5883P/L) and GPS
    (u-blox M10) were already made — all written directly through the bus registers (the
    `II2CBus`/`ISpiBus`/`IUartPort` interfaces), without third-party libraries, that is, without
    hidden dependencies on a specific vendor SDK.
  - A new board is added with a single `#elif` block in `include/config/Config.h` plus
    a single `[env:...]` block in `platformio.ini` — switching the board already
    works today for four targets (see the table below); this is not a hypothetical
    possibility.
  - `Autopilot.h` already accepts `ImuSensor*`/`BarometerSensor*` as
    parameters (they may be `nullptr`) — that is, the contract between the autopilot and the
    hardware assumes that the set of sensors will change (the next
    step is GPS as one more class with the same pattern, see Phase 3).
  - `WebDebugServer.h` already serves a single aggregated JSON (`GET
    /api/status`) and accepts commands (`POST /api/setmode`, `/api/setpid`)
    — that is, the protocol “the aircraft serves telemetry and
    accepts commands” already exists, and the ground station should grow out of
    it rather than be written from scratch.

The asymmetry is that the entry barrier (money, time to adapt to a new
task) of this platform is an order of magnitude lower than that of closed analogues, while
the path to autonomy does not require rewriting the core — only adding new
classes on top of existing interfaces. This is an open engineering platform, not
a finished commercial product — accordingly, the thesis for an investor/partner is
not “buy a ready-made solution” but “come in at a stage when the foundation has already been
verified and the next steps are technically clear”.

## 4. The architecture today — the foundation for the phases below

### 4.1 Supported boards

Choosing the board is a single PlatformIO build option; switching the board does not require
touching the logic (`include/config/Config.h` + `platformio.ini`):

| Environment (`pio run -e ...`) | Board | Status | aileron L / R | elevator | esc | ibus_rx | i2c sda / scl |
|---|---|---|---|---|---|---|---|
| `esp32-s3` (default) | ESP32-S3 N16R8 (DevKitC-1) | **Primary, verified on the bench with all the sensors** | GPIO4 / GPIO5 | GPIO6 | GPIO7 | GPIO17 | GPIO41 / GPIO42 |
| `esp32-c3` | ESP32-C3 SuperMini | First prototype, flew under manual control | GPIO5 / GPIO4 | GPIO6 | GPIO7 | GPIO8 | GPIO1 / GPIO3 |
| `esp32-dev` | classic ESP32 38-pin | For the bench, **not verified on hardware** (the whole firmware is covered by tests) | GPIO13 / GPIO14 | GPIO27 | GPIO26 | GPIO16 | GPIO21 / GPIO22 |
| `stm32h743` | STM32H743VIT6 (WeAct Mini) | Full firmware + MAVLink, **no board yet** (the whole firmware runs in tests on a PC) | PA0 / PA1 | PA2 | PA3 | PE7 | PB11 / PB10 |

Upload the firmware: `pio run -t upload`. Monitor: `pio device monitor` (115200).

### 4.2 RC channel map (FS-i6 + FS-iA6B, iBUS, 10 channels, 1000–2000 µs)

| Channel | Name | Default purpose |
|---|---|---|
| CH1–CH4 | sticks | roll, pitch, throttle, rudder |
| CH5 | ARM | switch SwA: ARM with the throttle down, DISARM instantly |
| CH6 | SWB | flaps |
| CH7 | SWC | mode: MANUAL / STABILIZE / AUTO_TAKEOFF |
| CH8 | SWD | RTH — home |
| CH9 | VRA | stabilization strength |
| CH10 | VRB | cruise speed |

CH6–CH10 are assigned with a single line in `include/config/Controls.h`: any of the 12
modes, 10 functions (flaps, brake, payload drop, geofence…) and 7 knobs —
[AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

### 4.3 Control flow (`FlightController::update()`)

A single orchestrator with a fixed order; the loop runs at 500 Hz with a fixed
period: iBUS reception → pilot throttle → mode from CH7 → **sensor reading and autopilot
calculation (always, even without a link)** → **link-loss check with absolute
priority** (in the air — home by GPS with the motor on, or gliding with wings level
when there is no GPS; on the ground — control surfaces to neutral) → ARM → the sticks' command +
the autopilot's corrections in unified aviation sign conventions → mixer with servo reverse →
the mode's throttle → throttle blocked without ARM → writing to the servos/ESC. It is exactly this
discipline of ordering (safety first, then manual control, then the autopilot as a
superstructure) that makes it possible to safely build ever more
autonomous behavior into it without rewriting the base loop. Wi-Fi, the dashboard and the display
run on the second core and do not delay the control.

### 4.4 The web dashboard as the seed of a ground station

`WebDebugServer.h` already today brings up an access point (SSID
`OpenPlane-Debug`, IP `192.168.4.1`) and serves and accepts JSON:

| Method and path | What it does |
|---|---|
| `GET /api/status` | A single aggregated JSON: RC (10 channels), armed/failsafe, 7 outputs (`us`, `attached`), IMU, barometer, compass, GPS, pitot tube (each with `attached`/`available` + data), autopilot (mode, corrections, PID, navigation, enabled functions) |
| `POST /api/setmode` | `{mode: 0-11}` — switch the autopilot mode |
| `POST /api/setpid` | `{kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch}` — any field is optional |
| `GET /` | The HTML dashboard: live bars for the 10 channels, the status of each output/sensor, mode buttons, a PID form |

The `attached`/`available` fields are always present in the JSON — the dashboard honestly
shows “not in the setup” separately from “in the setup but not responding”, instead of
staying silent about a missing sensor. This is the same principle of honesty
that underlies this whole document: do not pass off the desired as the actual.

## 5. A phase-by-phase technical roadmap

Below are eight phases, each described as the next logical step on top of the
classes that already exist, with no invented dates or sums.

### Phase 1 — Manual control and a safe base [done]

**Goal:** a reliable, predictable radio-controlled glider with transparent debugging. **What already exists technically:** iBUS parsing with link-loss detection in `IBusReceiver.h`, the `ControlMixer.h` mixer (sticks → roll/pitch/flaps command → PWM with servo reverse, with no knowledge of UART/PWM), `ThrottleManager.h`, `ArmingManager.h` (ARM with a separate switch with the throttle down, instant DISARM), failsafe with absolute priority in `FlightController.h`, output to Serial (`DebugLogger.h`), the web dashboard (`WebDebugServer.h`) and the OLED display (`OledDisplay.h`). **Why this is the base for everything else:** it is the only layer that must always work, even if all the other phases have not yet been implemented or their sensors are disconnected — which is exactly why failsafe and ARM were written first and verified on hardware (including the real behavior of the FS-iA6B receiver with the transmitter switched off).

### Phase 2 — IMU + barometer → autopilot [written and verified by tests and simulation, awaiting flight tests]

**Goal:** the first autonomous flight mode — horizon stabilization, auto-takeoff, altitude hold. **What already exists technically:** `imu/MPU6050_Sensor.h` (MPU6050 and MPU6500, registers accessed directly, axis rotation to match the board's mounting, aviation sign conventions, a complementary filter in `ImuSensorBase`), `baro/BMP388_Sensor.h` (I2C or SPI, full Bosch compensation, reading on the data-ready flag, a filtered vertical speed), `Autopilot.h` with `PidController` (the D term on the gyroscope, the integrator accumulates only after ARM) and twelve modes (from MANUAL to SOARING and RESCUE), switched by switches according to the `Controls.h` table, from the web dashboard and from QGroundControl, plus failsafe (home or gliding). Every mode flies in a closed-loop simulation of the whole firmware with an airplane model (`test/native/test_sim`). On the bench with the ESP32-S3 all the sensors respond and the signs have been verified live: tilt → the control surfaces correct toward leveling. **What is needed to close the phase:** move the electronics into the glider, check the control-surface directions on the assembled aircraft and carry out the first flight tests — starting with STABILIZE at a safe altitude.

### Phase 2.5 — Feedback from the real airplane [groundwork, verified in simulation]

**Goal:** for the autopilot to depend not on coefficients tuned for a single speed, but on how the real airplane responds to the control surface right now. The PID from Phase 2 deflects the surface “by a formula” and does not check the result; at low speed it under-corrects, at high speed it over-corrects. **What already exists technically** (`include/autopilot/feedback/`, **not connected** to the firmware): in-flight estimation of control-surface effectiveness (recursive least squares, rescaled with speed ∝ V²), an “angle → rotation rate → control surface” controller with follow-up correction (“the surface did not turn all the way — turn it further”), protection against loss of airspeed and stall (throttle, nose down, wings level), takeoff from a runway or by hand and landing in stages driven by the sensors. All of it has been verified by a closed-loop airplane simulation on the board itself (`pio test -e esp32-s3 -f test_feedback`, 10 scenarios) — including a swapped aileron, turbulence, a raised nose at low throttle, takeoff and landing. **What is needed to close the phase:** after the first flights of Phase 2 — “shadow mode” (the feedback only writes to the log what it would have done), then connecting it one axis at a time, an airspeed sensor (pitot tube) and a rangefinder for leveling off before touchdown. Details — the “Feedback” section in [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

### Phase 3 — GPS and compass [in the firmware, verified by simulation]

**Status (updated): navigation works** — heading from GPS/compass/gyroscope with hysteresis, home set at ARM, CRUISE, LOITER (a vector field onto a circle), RTH, geofence; verified by closed-loop simulations. Below is the original entry for the phase. GPS (u-blox M10, `sensors/gps/UbloxM10_Gps.h`, the UBX-NAV-PVT protocol) and a magnetometer (two variants of the “GY-273” board: QMC5883P — `sensors/mag/QMC5883P_Sensor.h`, installed on the current bench and verified live; QMC5883L — `sensors/mag/QMC5883L_Sensor.h`) were added as new classes implementing the `GpsSensor`/`MagnetometerSensor` interfaces (`include/sensors/SensorInterface.h`) on the same principle as the IMU and barometer — `Autopilot.h` and `FlightController.h` were not rewritten; they gained two more data sources the same way (a nullable pointer in the constructor). Along the way a HAL layer (`include/hal/`) appeared, through which the sensors access the buses — I2C/SPI/UART are no longer tied directly to the ESP32-specific `Wire`/`SPI`/`HardwareSerial`.

For now the GPS/compass data are available through `Autopilot::getGpsSensor()`/`getMagnetometerSensor()` and in `GET /api/status`, plus a one-time setting of the initial yaw from the compass at startup — **but they do not take part in control**. Open questions for finishing the phase: connect the GPS module to the ESP32-S3 (the UART2 pins are already reserved), calibrate the compass on the assembled aircraft and add tilt compensation to the heading. On the ESP32-C3 a full-fledged GPS is impossible — there are not enough GPIOs for TX (see the pinout in `DEVELOPER_GUIDE.md`).

### Phase 4 — Flying along waypoints (waypoint navigation) [next step]

**Status:** the primitives are ready — `Guidance::rollForCourse`, a circle around a point, return to a point (RTH), a MAVLink channel for uploading a mission (right now, when asked for a mission, the aircraft honestly replies “0 points”). What remains: route storage, transitions between points, the MAVLink MISSION_* protocol.

**Goal:** the aircraft flies a given set of coordinates without an operator's involvement on each leg of the route. **How this fits into the architecture:** it is a new `AutopilotMode` in `Autopilot.h`, alongside the existing MANUAL/STABILIZE/AUTO_TAKEOFF/ALT_HOLD — that is, the mode-switching mechanism (through RC slots and through `POST /api/setmode`) does not change; a fifth mode is added, which takes the heading and distance from the GPS (Phase 3) instead of manual input through the RC. **What is needed technically:** an algorithm for computing the heading to a point and the logic for moving between route points, plus a way to upload the route itself to the aircraft (the natural candidate is an extension of the same HTTP API that is already used to control the modes and the PID).

### Phase 5 — Long-range telemetry [done in the STM32 firmware]

**Status:** on the STM32H743 — MAVLink 2 over a radio modem (SiK, ELRS in MAVLink mode): attitude, position, speed, mode, PID parameters, mode change from the ground. The frames have been checked against the reference pymavlink. The ESP32 has no free UART — it uses the Wi-Fi dashboard. Below is the original entry for the phase.

**Goal:** an aircraft↔ground link at distances relevant to real delivery, not to the bench. **An honest assessment of the current state:** the `WebDebugServer` Wi-Fi access point already today transmits the full aggregated status and control commands, but the range of an ordinary Wi-Fi AP is tens of meters, which is enough for debugging on a table or at an airfield, but not for an autonomous route beyond line of sight. **What is needed technically:** a separate longer-range radio channel (for example, a LoRa module or a dedicated telemetry radio modem) as the transport for the same data format that is already defined in `GET /api/status` — that is, replacing or supplementing the transport layer, not rewriting the telemetry format.

### Phase 6 — A full-fledged ground-control GUI [partly: QGroundControl / Mission Planner]

**Status:** thanks to MAVLink, standard ground stations already see the aircraft (map, home, instruments, modes under ArduPlane names). A ground station of our own for a fleet is still a goal. Below is the original entry for the phase.

**Goal:** a mission-planning station with a map, live telemetry and fleet management, rather than a debugging page for a single aircraft. **How this fits into the architecture:** `WebDebugServer.h` is already today not a stub but a working web server with an aggregated JSON status and a command API (see the table in section 4.4); it is the starting point, not something that will have to be thrown away. The next steps are a map with the current position (after Phase 3), displaying and uploading a route (after Phase 4), working over a long-range radio channel (after Phase 5), and scaling the interface from one aircraft to several. More in section 6.

### Phase 7 — A payload-drop mechanism and protective measures for delivery [done in the firmware, not yet flown]

**Status:** payload drop (the AUX1 servo, the `PAYLOAD_DROP` function on any switch), geofence (radius and ceiling → RTH), return home on link loss, the “model lost” buzzer. Below is the original entry for the phase.

**Goal:** to turn the platform from “an airplane that flies autonomously” into “an airplane that delivers autonomously”. **What is needed technically:** an additional servo for the payload release/dispensing mechanism, controlled on the same principle as the other outputs in `FlightOutputs.h`; and protective measures specific to delivery rather than to neutral flight — geofences (a limit on the flight area) and automatic return to the starting point on link loss (today, when the signal is lost in the air, `FlightController` cuts the motor and switches to gliding with wings level, which is right for a manually flown glider, but for autonomous delivery the logical next step is returning to base by GPS instead of simply gliding).

### Phase 8 — Scaling to a fleet

**Goal:** managing several aircraft at the same time — task dispatching, an operations panel, flight history. This is the level at which the project stops being only an engineering hobby prototype and becomes an operational tool, interesting as a business problem: route planning for several aircraft, a task queue, the status of each aircraft in real time. Technically this is a superstructure over Phases 3–6 (GPS, telemetry, GUI) — in effect the same `WebDebugServer` API, but extended to many telemetry sources instead of one.

## 6. GUI: from a debugging page to a ground station

The key thesis of this section: the GUI does not need to be built from scratch — it already partly exists and works. Today `WebDebugServer.h`:

- serves a single aggregated JSON snapshot of the aircraft's state (`GET /api/status`) — the RC channels, the armed/failsafe flags, the state of each output, the state of each sensor (honestly, with separate `attached` and `available` flags) and the state of the autopilot;
- accepts control commands in real time (mode change, PID editing) without reflashing;
- serves a ready-made HTML dashboard with live channel bars and control buttons.

The path to a full-fledged ground station is a sequential extension of the protocol that already works, not a change of architecture:

1. Add a map and the current position to the dashboard — this requires GPS (Phase 3) as one more field in the same JSON status.
2. Add route building and uploading — this requires a waypoint mode (Phase 4) and an extension of the POST API similar to `/api/setmode`/`/api/setpid`.
3. Move the transport from the Wi-Fi AP to a long-range link (Phase 5), keeping the same message format so that the existing frontend does not have to be rewritten.
4. Scale the interface from one aircraft to several telemetry sources (Phase 8).

In other words, the element of the future ground station that is the riskiest in terms of “will it have to be written from scratch” — the serialization of the aircraft's state and the command API — has already been implemented and verified live on the board.

## 7. Honest limitations — what does not work yet

So that the roadmap does not read like marketing, we record separately what has not been done yet or has been done only partially:

- The autopilot has been verified on the bench, by 387 automated tests and by closed-loop simulations, but it has never been tested in flight — so far only manual control has flown (on the first prototype). The airplane model in the simulations is simplified, and the coefficients are starting values.
- The new sensors (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) have been verified by register emulators built from the datasheets; on hardware, not yet.
- STM32H743: the whole firmware runs on a PC on top of STM32duino fakes; there is no physical board yet.
- The horizon for stabilization is the attitude of the aircraft at power-on (the IMU calibrates at every start), or the mounting calibration from three poses.
- The physical connection of a servo is not visible to the software; all that can be seen is that the pulse really comes out on the pin (a self-check from the console).
- The pinout of the ordinary 38-pin ESP32 was chosen from the chip's documentation and has not been verified on hardware.
- The license is the [OpenPlane License](../../../LICENSE): MIT with mandatory attribution of the author and prohibitions on military use and on intentional harm to people and property without their consent. Because of these prohibitions it is not considered “open source” in the OSI sense.

## 8. Open questions — an invitation to discuss

Below are the points on which the project does not yet have an answer, deliberately worded as questions for a potential partner or investor rather than as settled facts:

- The funding model and its size — open to discussion; there are no specific sums or dates at the moment, and none will be invented in this document.
- The legal form of the project (a company, a foundation, a purely open-source community) — open for discussion with those interested in partnership.
- The composition of the team — for now the project is run publicly and is open to participation; no specific roles or obligations are postulated in advance.

If any of this matters to you as a potential partner, the right place to talk about it is the project's GitHub Discussions (see section 9), not assumptions in this document.

## 9. How to get in touch and take part

The only official channel of the project today is the GitHub repository:
[github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject)
(branch `main`). There are no other contacts (email, social networks, a legal entity) at the moment, and they are deliberately not listed here so as not to mislead.

- **Issues** — report a bug, propose a specific technical change, report the results of a flight test on your own copy of the prototype.
- **Discussions** — discuss the roadmap, partnership, use in a specific task (delivery, search and rescue, agriculture), and the licensing and funding questions from section 8.
- **Pull requests** — add a new sensor through the `Sensor` interface, a new board through a block in `Config.h`, a new `AutopilotMode`, improvements to the web dashboard — the architecture is designed so that this can be done without touching the core.

If you are reading this document as a potential investor or partner: the next meaningful step is not signing anything, but opening a Discussion in the repository with a specific question or proposal. The roadmap above is an invitation to discuss it phase by phase, with full access to the code on which it is based.
