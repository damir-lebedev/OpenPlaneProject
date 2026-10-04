# AUTOPILOT — modes, navigation, switches

> 🌐 This page is a translation of the [Russian original](../../../reference/autopilot.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

The autopilot receives the pilot's sticks, the switches/knobs (`PilotInputs`)
and the sensors, and outputs **the final control surface command**
(`getCommand()`) and the mode's throttle (`applyThrottle()`). Without a
required sensor, a mode behaves safely (the control surfaces stay with the
pilot or go neutral) rather than crashing. What each mode does for the pilot is
in [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). So far only manual mode has
been flown; the autopilot has been tested on the bench, with tests and with
closed-loop simulations (`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**File:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (unscoped — the numeric codes go into the JSON
of `/api/setmode`, `/api/status` and into `MavlinkModes`):

| Value | Code | Short (OLED) | Essence |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | control surfaces = sticks |
| `MODE_STABILIZE` | 1 | STAB | the stick is the roll/pitch angle |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | a takeoff program driven by the pilot's throttle |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + altitude with the elevator |
| `MODE_ACRO` | 4 | ACRO | the stick is the angular rate |
| `MODE_CRUISE` | 5 | CRZ | course + altitude + auto throttle |
| `MODE_LOITER` | 6 | LOIT | circles over the point where it was switched on |
| `MODE_RTH` | 7 | RTH | home, circles over home |
| `MODE_LAUNCH` | 8 | LNCH | hand launch |
| `MODE_AUTO_LAND` | 9 | LAND | gliding + flare |
| `MODE_SOARING` | 10 | SOAR | thermals without the motor |
| `MODE_RESCUE` | 11 | RESQ | wings level, nose up, throttle |
| `MODE_COUNT` | 12 | | the boundary (`setMode()` ignores ≥) |

`enum class Feature : uint8_t` — the switch functions: `FLAPS`, `AIRBRAKE`,
`AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`, `PAYLOAD_DROP`,
`GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`.

`enum class Knob : uint8_t` — the knobs: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`.

`namespace AutopilotNames` — `mode()`, `modeShort()` (≤ 5 characters),
`feature()`, `knob()`: names for the log, OLED, dashboard, MAVLink.

### `PilotInputs`

The state of the switches and knobs for one cycle.

| Member | Description |
|---|---|
| `bool has(Feature) const` | the function is on |
| `float knob(Knob) const` | the knob position −1…+1 |
| `bool isBound(Knob) const` | the knob is in the binding table |
| `float knobValue(Knob, min, default, max) const` | in units: the center is `default`, the ends are `min`/`max`; not bound — `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**File:** `autopilot/ControlBinding.h` · the table — `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. The table rows are the
factories of `namespace Bind` (all `constexpr`):

| Factory | Meaning |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | a mode-selection switch (the zone by `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | a mode on top while the channel ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | a function while the channel ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | a knob, `(us − 1500) / 500`, limited to ±1 |

`namespace BindingCheck` — recursive `constexpr` functions (the ESP32 core is
built as C++11): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. They are used in the `static_assert`s
of `Controls.h`.

---

## `PilotSwitches`

**File:** `autopilot/PilotSwitches.h` · **Depends on:** `Autopilot*`, `RcChannelState`, the binding table

| Method | Description |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | your own table (tests, simulations) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | the `Controls::BINDINGS` table |
| `void update(const RcChannelState&)` | collect `PilotInputs`, pass them to `autopilot->setInputs()`; `setMode()` — **only when the switches' result has changed** (a mode set from the dashboard/GCS is not overwritten every cycle). `FlightController` calls it only while the link is alive |
| `void printBindings() const` | the layout to Serial at power-on: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 zones: < 1500 / ≥ 1500; 3 zones: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | for telemetry and tests |

`Bind::mode` outranks `Bind::modes`; of several `Bind::mode` entries that are on,
the top row wins.

---

## `Autopilot`

**File:** `autopilot/Autopilot.h` · **Depends on:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, the sensors (all nullable)

### Lifecycle

| Method | Description |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | The roll/pitch PID Kp 5, Ki 0.5, Kd 0.5, output ±500 µs |
| `bool begin()` | load the trim; `false` and a message if there is no IMU or barometer |
| `void setInputs(const PilotInputs&)` | this cycle's switches and knobs (before `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | once per cycle: sensors (always) → navigation and home → saving the trim on the ground → failsafe → geofence → mode → turn coordination → auto-trim |
| `ControlCommand getCommand() const` | the final control surface commands (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | the mode's throttle: `PILOT` — the pilot's throttle; `AUTO` — its own; `AT_LEAST` — not less than its own (automatic takeoff). ARM and `MOTOR_KILL` are handled by `FlightController` |

### Modes

| Method | Description |
|---|---|
| `void setMode(AutopilotMode)` | the same or ≥ `MODE_COUNT` — nothing; otherwise reset the PID and the state machines, the targets = the current course and altitude, the circle center = the current point (with GPS), RTH — the return altitude |
| `getMode()`, `getModeName()` | the name: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` on link loss, otherwise the mode |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | failsafe on top of the mode |
| `isAutoThrottle()`, `getThrottleCorrection()` | the mode's throttle (%, for the log and dashboard) |
| `getLaunchState()`, `getSoaringState()` | the LAUNCH and SOARING state machines |

### Outputs and diagnostics

| Method | Description |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | command − sticks, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | the targets |
| `const NavStatus& getNavStatus()` | GPS, home, position, distance/bearing to home, course and target course, speed for navigation, geofence, stall |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | the barometric altitude (m from the power-on point) |
| `getInputs()`, `getAutoTrim()` | for telemetry |
| `getImuSensor()` … `getAirspeedSensor()` | the sensors (may be `nullptr`) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | the PID (dashboard, MAVLink parameters) |

### Internal mechanics

- `stabilize()` — an angle PID with D from the gyro, multiplied by
  `Knob::STAB_GAIN`; the integrator accumulates only when ARMed and the error
  is < `STAB_INTEGRATOR_ZONE_DEG`. `stabilizeOrManual()` — without an IMU the
  control surfaces stay with the pilot; `stabilizeOrNeutral()` — without an
  IMU, neutral (automatic modes).
- `imuReady()` = the IMU exists, is available and has no preflight problem.
- The speed for navigation: pitot tube → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` — near the ground by the barometer, with almost no vertical
  speed, slower than the pitot/GPS threshold: only then is the trim written to
  flash.
- Failsafe: with GPS and a home point — RTH with the motor, otherwise gliding;
  an RTH that has begun is not abandoned because of a short GPS loss.

---

## `Geo`, `Guidance`, `GeoPoint`

**File:** `autopilot/Navigation.h`

A local "north/east" plane in meters (an equirectangular projection — for
kilometers the error is a fraction of a percent).

| Function | Description |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | angle normalization |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | offset, distance, bearing 0..360 |
| `Geo::moved(a, north, east)` | a point with an offset |
| `Geo::fromGps(GpsData)` | a `GeoPoint` from the GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | the bank for a course error (`NAV_COURSE_GAIN`), limited |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | the vector-field course onto a circle (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | the feed-forward bank for a circle: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**File:** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| Method | Description |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | the wanted vertical speed = `NAV_ALT_GAIN`·error (≤ `NAV_MAX_CLIMB/SINK`); pitch = the feed-forward asin(Vz/V) + a PI on the Vz error, within `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | with a pitot tube — a PI on the airspeed around `cruisePct`; without — `cruisePct`; + `THROTTLE_PER_CLIMB_PCT` for the required climb |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**File:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (throttle raised) `→ THROWN` (an overload > `LAUNCH_ACCEL_G`
for longer than `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (after `LAUNCH_MOTOR_DELAY_MS`: the
motor, pitch `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` or
`LAUNCH_ALTITUDE_M`). Moving the sticks before the throw — cancel. Methods:
`update(...)`, `reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`,
`stateName()`.

## `SoaringController`

**File:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (the variometer > `SOAR_THERMAL_CLIMB_MS` for longer than
`SOAR_THERMAL_CONFIRM_MS` / the average < `SOAR_EXIT_CLIMB_MS` over
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (below `SOAR_MIN_ALTITUDE_M`, up to
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (beyond `SOAR_MAX_DISTANCE_M`, down to 70 % of
it). Methods: `update(climb, alt, distHome, dt, now)`, `reset(now)`,
`getState()`, `motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**File:** `autopilot/AutoTrim.h` · storage — `Preferences` (NVS / STM32 flash), namespace `"autotrim"`

| Method | Description |
|---|---|
| `void load()` | the trim from NVS (none — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += command · `AUTOTRIM_RATE` · dt, up to ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | write if it changed (called by `Autopilot` after DISARM on the ground) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**File:** `autopilot/PidController.h` · **Depends on:** `Config` (the nominal `dt`)

| Method | Description |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | the gains |
| `setLimits(minOut, maxOut)` | the output limit (±500 by default) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | the output, limited to `[min, max]` |
| `void reset()` | zero the integrator, `dt` — counted from "now" |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (the rate from the sensor!)
out = constrain(P + I + D, min, max)
```

D is taken on the rate of the measured quantity (gyro °/s), not on the
derivative of the error: no differentiation noise and no jump when the setpoint
changes. `dt` comes from `micros()`; the first call after `reset()` or after a
pause of more than 0.1 s uses the nominal `LOOP_PERIOD_MS`.
