# OpenPlane autopilot reference

> 🌐 This page is a translation of the [Russian original](../../AUTOPILOT_GUIDE.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

What the autopilot can do, how to enable each feature, and how to hang it on any switch or knob of the transmitter **with a single line**.

> Honesty first. All the modes have been verified with unit tests and closed-loop flight simulations (`test/native/test_sim`: the entire firmware flies an airplane model). The model is simplified, and the coefficients in `Config.h` are starting values: **try each mode first at an altitude of 50+ m with your finger on the MANUAL switch**. So far only manual mode has been flown (the first prototype, on an ESP32-C3); stabilization has been verified on the bench — the surfaces respond to tilts in the right direction. The STM32H743 board builds and passes the same tests; on real hardware the DevEBox board has been verified without sensors: the SD card, the black box and manual control of the servos and motor from the transmitter (recorded on video); no sensors have been connected to it yet, and the autopilot modes have not been tried on it.

---

## Contents

1. [How it works in a minute](#how-it-works-in-a-minute)
2. [Default transmitter layout](#default-transmitter-layout)
3. [Assigning a function with a single line](#assigning-a-function-with-a-single-line)
4. [Modes](#modes)
5. [Features (switches)](#features-switches)
6. [Knobs](#knobs)
7. [Link loss, geofence, home](#link-loss-geofence-home)
8. [A DIY pitot tube](#a-diy-pitot-tube)
9. [Ground station: Wi-Fi dashboard and MAVLink](#ground-station-wi-fi-dashboard-and-mavlink)
10. [Setting up a new airplane, step by step](#setting-up-a-new-airplane-step-by-step)
11. [Autopilot pre-flight check](#autopilot-pre-flight-check)
12. [What each mode needs](#what-each-mode-needs)

---

## How it works in a minute

```
sticks ─┐
        ├─► PilotSwitches (config/Controls.h) ─► mode, features, knobs
switches┘                                               │
                                                        ▼
sensors (IMU, barometer, compass, GPS, pitot tube) ─► Autopilot ─► surface and throttle commands
                                                        │
                           FlightController: flaps, payload, camera, buzzer, failsafe
                                                        ▼
                                           ailerons · elevator · rudder · ESC · AUX1 · AUX2
```

- The **mode** decides who flies: the pilot (MANUAL), the pilot with a helper (STABILIZE, ALT_HOLD, ACRO), the autopilot with the pilot's corrections (CRUISE, LOITER, RTH…).
- **Features** are switched on top of any mode: flaps, brake, payload drop, geofence…
- **Knobs** smoothly change a number: stabilization strength, cruise speed, circle radius…
- In the modes with stabilization **the stick sets the angle**, not the surface deflection: let go of the stick, and the airplane levels itself.
- A sensor failure never "jerks" the airplane: no IMU — the surfaces stay with the pilot; no barometer — the pilot holds the altitude; no GPS — no navigation, and the modes that need it behave safely (see [the table](#what-each-mode-needs)).

---

## Default transmitter layout

FS-i6 + FS-iA6B, iBUS, 10 channels (`config/Channels.h`).

| Channel | Transmitter control | Default |
|---|---|---|
| CH1–CH4 | sticks | roll, pitch, throttle, rudder (cannot be reassigned) |
| CH5 | **SwA** | **ARM** (down = armed, only with the throttle down; cannot be reassigned) |
| CH6 | SwB | flaps (`Feature::FLAPS`) |
| CH7 | **SwC** (3 positions) | up **MANUAL** · middle **STABILIZE** · down **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH** — home, while switched on |
| CH9 | VrA | stabilization strength (`Knob::STAB_GAIN`) |
| CH10 | VrB | cruise speed (`Knob::CRUISE_SPEED`) |

When the board is powered up, the serial monitor prints the actual layout — what is really flashed (the firmware prints it in Russian; "вверх / середина / вниз" means up / middle / down):

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> Channels 7–10 are not enabled on the FS-i6 by default. In the transmitter menu: **Functions setup → Aux. channels**, assign SwC, SwD, VrA, VrB.

---

## Assigning a function with a single line

Everything is in one file — `include/config/Controls.h`:

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| Form of the line | What it does |
|---|---|
| `Bind::modes(channel, up, middle, down)` | a three-position switch selects the mode |
| `Bind::modes(channel, up, down)` | a two-position switch — two modes |
| `Bind::mode(channel, mode)` | a mode **on top of** the others while the switch is on; switch it off and the mode from the mode switch returns |
| `Bind::feature(channel, feature)` | the feature works while the switch is on |
| `Bind::knob(channel, knob)` | a knob: center = the value from `Config.h`, edges = minimum and maximum |

"On" means the channel is above 1750 µs (on the FS-i6, the switch down, toward you). Until the first receiver frame arrives, all channels are considered off: nothing will deploy or drop at power-up.

### Ready-made recipes

```cpp
// Thermal glider: SwD — soaring, SwB — auto-trim, VrB — circle radius
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Student: stabilization only, RESCUE on a "panic button", soft sticks
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// Filming and delivery: camera with stabilization, payload drop, circling over a point
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Hand launch without landing gear: SwD — LAUNCH, flaps smoothly on a knob
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### The compiler catches mistakes

The table is checked at build time (`static_assert`), before the firmware gets into the airplane:

- the sticks and SwA (ARM) cannot be bound, and the channel number must be less than `Channels::COUNT`;
- a channel has only one binding;
- there can be no more than one mode-selection switch (`Bind::modes`).

If several `Bind::mode` switches are on at once, the topmost row of the table wins.

---

## Modes

The mode changes when a switch is **flipped**. A mode switched on from the dashboard or the ground station stays in effect until the pilot flips a mode switch again. The OLED shows a short name (in parentheses).

### MANUAL (MAN)
Surfaces = sticks, as without a flight controller. Only the features (flaps, brake, payload) and the auto-trim work. **The main safety mode**: always keep it on a switch under your finger.

### STABILIZE (STAB) — "the stick sets the angle"
The roll stick sets the bank angle up to `MAX_BANK_DEG` (45°, the `MAX_BANK` knob 15…60°), the pitch stick sets the angle up to `STAB_MAX_PITCH_DEG` (25°). Let go — the airplane levels itself. The throttle is the pilot's. The PID integrator accumulates only near the target (±10°), so after a sharp maneuver the airplane does not "overshoot" the horizon.
**Needs:** IMU. Without an IMU — same as MANUAL.

### ALT_HOLD (ALT) — "hold the altitude"
Like STABILIZE in roll, while the elevator holds the altitude by the barometer. Move the pitch stick — you fly it yourself; let go — the airplane holds the **new** altitude. The throttle is the pilot's (add throttle, otherwise there will not be enough speed to climb).
**Needs:** IMU + barometer.

### ACRO — "the stick sets the rotation rate"
Full stick — 180°/s. Let go — the airplane **holds the attitude it was in** (even upside down), and the gyroscope damps gusts. For aerobatics.
**Needs:** IMU. Without an IMU — surfaces = sticks.

### CRUISE (CRZ) — "hold course, altitude and speed"
The airplane flies straight at the current altitude, with the throttle automatic (the `CRUISE_SPEED` knob: 30…55…85 % throttle, and with a pitot tube — **airspeed** 10…14…22 m/s). The roll stick turns (let go — it holds the new course), the pitch stick changes the altitude. With a pitot tube, stall protection works. The course comes from GPS (at a ground speed > 3 m/s, back to the compass below 2 m/s), otherwise from the compass, otherwise from the gyroscope. In simulation, with a 4 m/s crosswind, the ground track holds within ±5° and the altitude within ±3 m.
**Needs:** IMU + barometer; GPS or a compass for a course without drift.

### LOITER (LOIT) — "circle here"
Clockwise circles over the point where the mode was switched on, radius 50 m (the `LOITER_RADIUS` knob 25…150 m), at the current altitude, with the throttle automatic. Guidance is a vector field: from far away the airplane joins the circle along a tangent, and on the circle it holds a feed-forward bank. Without GPS — just a circle with a constant bank in place.
**Needs:** IMU + barometer + GPS.

### RTH — "home"
Course to home (the ARM point), altitude `RTH_ALTITUDE_M` = 40 m (below it — climbs along the way, above it — stays). Over home — circles of the LOITER radius until the pilot takes control. No GPS or no home — circles in place. The same mode is engaged by link loss and by the geofence.
**Needs:** IMU + barometer + GPS with a home point.

### AUTO_TAKEOFF (TKOFF) — "takeoff on throttle"
After ARM nothing happens until the pilot raises the throttle above the middle. Then the program: 1 s of acceleration to 100 % throttle with the wings level, 2 s of rotation with a pitch of 15°, then a climb with a pitch of 10° until the pilot switches the mode. The sticks are added on top of the program — you can straighten the course on the takeoff run.
**Needs:** IMU.

### LAUNCH (LNCH) — "hand launch"
1. Armed, throttle above the middle — the launch is **primed**, the motor is stopped.
2. The throw: forward acceleration > 1.5 g for longer than 40 ms.
3. After 0.3 s (the hand is away from the propeller) — throttle 100 %, a climb with a pitch of 15°, wings level — for 6 s or until 30 m.
4. Then — like CRUISE at the altitude reached.

Any stick movement (> 150 µs) before the throw cancels it: the airplane is in the pilot's hands.
**Needs:** IMU (accelerometer). In simulation: a throw at 9 m/s from hand height — the airplane never touches the ground and climbs more than 15 m in 15 s.

### AUTO_LAND (LAND) — "landing"
The motor is off, gliding on course with a pitch of −4°; below 3 m by the barometer — flare (+4°). The roll stick adjusts the approach course. Engage it on a straight leg, into the wind, at 20–40 m, with runway to spare. In simulation, touchdown is at a vertical speed below 1.5 m/s, wings level, not on the nose.
**Needs:** IMU + barometer (zeroed on the ground at power-up).

### SOARING (SOAR) — "soaring in thermals"
The motor is off, gliding. A variometer (with a pitot tube — total energy, with no false "thermals" from pulling back on the stick) above 0.5 m/s for longer than 1.5 s — a thermal: circles with a bank of 25°. An average climb over 8 s that drops below −0.2 m/s — exit from the thermal. Below 30 m — the motor until 100 m; farther than 400 m from home — it glides home. In simulation it finds a thermal (a 3 m/s core) and climbs more than 50 m without the motor.
**Needs:** IMU + barometer; GPS — for returning home.

### RESCUE (RESQ) — "save me"
Wings level, nose +8°, throttle 70 % — from any spiral. Lost your orientation? Flip the switch and exhale. In simulation, from a spiral with a bank of 70° and a nose of −40°, in 4 s — wings level and a climb.
**Needs:** IMU.

---

## Features (switches)

| Feature | What it does | Details and numbers (`Config.h`) |
|---|---|---|
| `FLAPS` | both ailerons down — flaperons | `FLAPS_DEPLOYED_US` = 220 µs, smoothly over 1 s; roll works on top |
| `AIRBRAKE` | both ailerons up — an air brake, a steeper glide slope | `AIRBRAKE_US` = 250; takes priority over flaps |
| `AUTO_TRIM` | learns to hold the airplane straight without the sticks: the steady surface command in level flight "flows" into the trim | 20 %/s, up to ±120 µs; saved to flash after DISARM **on the ground** |
| `TURN_COORDINATION` | rudder into the turn, nose up in the bank | always on in the navigation modes |
| `MOTOR_KILL` | the motor is off in any mode, even the automatic ones | overrides any mode and the throttle |
| `BEEPER` | the "I'm here" buzzer | without a switch it beeps by itself: on the ground, link lost > 10 s |
| `PAYLOAD_DROP` | the AUX1 servo is open while the switch is on | 1000 µs closed, 2000 open |
| `GEOFENCE` | farther than 500 m from home or higher than 120 m — RTH | `GEOFENCE_ALWAYS_ON` — without a switch |
| `HOME_RESET` | home = the current point (at the moment the switch is turned on) | only with a good GPS |
| `CAMERA_STAB` | the camera on AUX2 holds its angle to the horizon | the airplane's pitch is subtracted |

## Knobs

The center of a knob = the default value from `Config.h`; the edges are the minimum and maximum. If a knob is not bound, the default value applies.

| Knob | Minimum … center … maximum | Where it acts |
|---|---|---|
| `STAB_GAIN` | ×0.25 … ×1 … ×2 | all modes with stabilization, and ACRO — "softer/stiffer" |
| `MAX_BANK` | 15° … 45° … 60° | the maximum bank from the stick and from navigation |
| `CRUISE_SPEED` | throttle 30 … 55 … 85 % (with a pitot tube: 10 … 14 … 22 m/s) | CRUISE, LOITER, RTH, the motor in SOARING |
| `FLAPS` | 0 … 50 … 100 % flaps | smooth flaps instead of a switch |
| `CAMERA_TILT` | −90° … 0° … +30° | the camera angle (AUX2) |
| `RATES` | 30 … 65 … 100 % of stick travel | all modes: stick sensitivity |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER and circles over home |

> A useful trick: the `STAB_GAIN` knob on VrA is "live" tuning of the coefficients in flight. If it oscillates — turn it down; if it is sluggish — turn it up; then move the multiplier into `Config.h`.

---

## Link loss, geofence, home

**Home** is recorded at ARM if the GPS is good (a 3D fix, ≥ 6 satellites, accuracy ≤ 5 m). If the GPS has not locked on by ARM — home is recorded as soon as it does. To change it in the field — the `HOME_RESET` feature.

**Link loss** (no iBUS frames for > 0.5 s, or the receiver sent a throttle below 950 µs — that is the failsafe configured in the transmitter; see `docs/PILOT_GUIDE.md`):

| Situation | What the aircraft does |
|---|---|
| on the ground (not armed) | motor 0, surfaces to neutral; after 10 s — the buzzer |
| in the air, GPS and home are available | **RTH** with the motor, over home — circles at 40 m |
| in the air, no GPS | **gliding**: motor off, wings level, nose −3° |
| the link is back | immediately the mode from the pilot's switch |

A return that has already begun does not drop into gliding because of a short GPS loss. `FAILSAFE_RTH = false` — gliding only.

**Geofence** (`GEOFENCE` or `GEOFENCE_ALWAYS_ON`): flying farther than `FENCE_RADIUS_M` (500 m) or higher than `FENCE_ALTITUDE_M` (120 m) — RTH. To take back control, flip a mode switch to any other position (a mode is switched on by changing the position). It triggers again when the airplane has come back inside with a 10 % margin.

---

## A DIY pitot tube

Airspeed without a bought differential pressure sensor: **two barometers**.

```
       incoming airflow ─►  ┌──────────── tube (PVC/brass, Ø4–6 mm) ──┐
                            │  BMP581 (I2C 0x47) — total pressure     │  airtight
                            └─────────────────────────────────────────┘
   fuselage: main barometer (BMP581 0x46 / SPL06 / BMP388) — static pressure

   speed  V = √(2·(P_tube − P_static − zero) / ρ),   ρ — from static pressure and temperature
```

**Assembly.** The BMP581 (a module with address 0x47: the SDO pin to VCC) is glued into a tube that is open only at the front — the board is inside an airtight cavity, with the wires brought out through sealant. The tube points forward, outside the propeller wash (on the wing or above the nose). The second barometer is inside the fuselage, shielded from the direct airflow (foam).

**Switching it on in the firmware** — `sensors/SensorSelection.h`: `SENSOR_KIT_LSM6DSV_PITOT` or `SENSOR_KIT_ICM45686_PITOT` (ready-made kits), or `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581` in your own kit.

**Zero.** Two barometers always differ slightly: the absolute accuracy of each is tens of pascals, and that is the entire pressure difference at low speed (10 m/s ≈ 60 Pa). During the first second after power-up the firmware averages the difference and takes it as zero. **The airplane stands still at power-up, and the tube is covered with a finger or a cap, or faces into the wind.** On the OLED, in the dashboard and in the telemetry the speed appears after the zeroing.

**`PITOT_SCALE` calibration.** The pressure inside the fuselage is not strictly static. In calm air, fly a straight line there and back in CRUISE and compare the average GPS ground speed with the tube's speed: `PITOT_SCALE = V_GPS / V_tube`.

**Protection.** A strongly negative pressure difference for longer than 2 s (hoses mixed up, water) or tube samples older than 0.2 s — no speed is output, and the autopilot falls back to the throttle set by the knob and flies the course without airspeed. Verified in a closed-loop simulation with noise on both barometers: the in-flight speed error is < 0.5 m/s.

What the tube gives you: CRUISE holds the **airspeed** rather than the throttle; stall protection; a total-energy variometer for SOARING; an honest speed in the telemetry.

---

## Ground station: Wi-Fi dashboard and MAVLink

**ESP32 — Wi-Fi dashboard.** The `OpenPlane-Debug` access point, password `12345678`, the address is in the serial monitor. Channels, outputs, all the sensors, the mode, navigation, the enabled features; you can change the mode and the PID. Details — `docs/PILOT_GUIDE.md`.

**STM32H743 — MAVLink over a radio modem** (UART4: PD0 RX, PD1 TX, 57600 baud — the SiK default). SiK 433/868/915 MHz, ELRS in MAVLink mode, and an ESP-01 as a Wi-Fi bridge all work. **QGroundControl** and **Mission Planner** see the aircraft as an ArduPilot airplane:

- the horizon, a map with home, speed (from the pitot tube, if there is one), altitude, variometer, throttle;
- modes under the ArduPlane names: STABILIZE → FBWA, ALT_HOLD → FBWB, CRUISE → CRUISE, LOITER → LOITER, RTH → RTL, AUTO_TAKEOFF/LAUNCH → TAKEOFF, SOARING → THERMAL, RESCUE → STABILIZE, AUTO_LAND → AUTO; failsafe is shown as RTL or CIRCLE;
- a message feed: ARM/DISARM, mode changes (under our own name), link loss, geofence;
- **changing the mode from the ground** — with the mode button in the GCS (except AUTO: OpenPlane has no missions);
- **parameters** `RLL_KP … PTCH_KD` — the roll and pitch PID, read and changed from the GCS parameter window right in flight (they are not saved across a reboot — move good values into `Config.h`).

ARM/DISARM from the ground is **rejected** — only with the transmitter switch. To check the stream without hardware: `OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin` (needs `pip install pymavlink`).

---

## Setting up a new airplane, step by step

1. **MANUAL on the ground:** the surface directions (`*_REVERSED` in `Config.h`), flaps down, AUX. Check the outputs with `p` in the console (propeller off!).
2. **Sensors on the ground:** `b` — bus scan, `s` — status; `o` — IMU mounting calibration (3 poses, once), `m` — compass.
3. **STABILIZE on the ground:** tilt the airplane to the right — the right aileron should go **down** (to level it). Nose up — elevator down. If not — the reverses or the IMU mounting are mixed up.
4. **The first flight in MANUAL**, climb 50+ m → STABILIZE. If it oscillates — lower `STAB_GAIN`; if it is sluggish — raise it.
5. **AUTO_TRIM** in level flight for 20–30 s, land, DISARM — the trim will be saved.
6. **ALT_HOLD**, then **CRUISE** — check the altitude and course; with a pitot tube — calibrate `PITOT_SCALE`.
7. **LOITER** and **RTH** — at altitude, within line of sight, finger on MANUAL.
8. Only after that — the **failsafe test** (switch the transmitter off at altitude, the airplane should head home) and automatic takeoff/landing.

## Autopilot pre-flight check

- [ ] The layout printed at power-up is the one you expect.
- [ ] Console/OLED: IMU ok, the IMU pre-flight check passed (the airplane stood still at power-up).
- [ ] The barometer is zeroed on the ground (altitude ~0 on the OLED).
- [ ] With a pitot tube: speed ~0 on the ground, blow into the tube — it rises.
- [ ] GPS: 3D fix, ≥ 6 satellites **before ARM** — otherwise there will be no home and no RTH.
- [ ] STABILIZE on the ground: the ailerons and elevator level the airplane rather than tip it over.
- [ ] Failsafe is configured in the transmitter (throttle below 950 on link loss) and verified by switching the transmitter off **on the ground** without a propeller.
- [ ] MANUAL — under your finger.

---

## What each mode needs

| Mode | IMU | Barometer | GPS | Compass | Pitot | Throttle | Without the required sensor |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | pilot | — |
| STABILIZE | ● | | | | | pilot | surfaces = sticks |
| ALT_HOLD | ● | ● | | | | pilot | the pilot holds the altitude |
| ACRO | ● | | | | | pilot | surfaces = sticks |
| CRUISE | ● | ● | ○ | ○ | ○ | auto | course by the gyroscope (drifts), altitude with the pilot |
| LOITER | ● | ● | ● | | ○ | auto | a circle with a bank in place |
| RTH | ● | ● | ● | | ○ | auto | circles in place |
| AUTO_TAKEOFF | ● | | | | | program | surfaces = sticks + the pilot's throttle |
| LAUNCH | ● | ○ | | | | program | surfaces to neutral |
| AUTO_LAND | ● | ● | | ○ | | 0 | no flare |
| SOARING | ● | ● | ○ | | ○ | 0 / motor | no thermals — gliding |
| RESCUE | ● | | | | | 70 % | surfaces to neutral |

● — required, ○ — improves. Checks in the code: `Autopilot.h` (`imuReady`, `baroReady`, `nav.gpsGood`); tests — `test/native/test_autopilot_modes` (how the modes react to each sensor) and `test/native/test_sim` (closed-loop flights).
