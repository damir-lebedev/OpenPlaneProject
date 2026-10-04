# AUTOPILOT / feedback — the feedback loop (groundwork)

> 🌐 This page is a translation of the [Russian original](../../../reference/feedback.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

> ⚠️ **Groundwork, not connected to the firmware.** Neither `FlightController`
> nor `Autopilot` nor `main.cpp` includes these headers. They are verified by
> a closed-loop simulation (`test/test_feedback`, on a PC and on the board) and
> by native unit tests. The connection plan is in
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#connection-plan).

The idea: instead of a PID on the angle with gains tuned for one speed — a
controller closed on **the airplane's response**, with an axis model learned in
flight, stall protection and takeoff/landing phases driven by the sensors. The
only input is `FlightSnapshot`, the only output is `FeedbackOutput`.

All the modules are header-only; `FeedbackModules.h` includes them in one line.

---

## namespace `FeedbackConfig`

**File:** `autopilot/feedback/FeedbackConfig.h`

All the loop's constants (they will move into `Config.h` when it is connected).
The values marked "прикидка" ("rough estimate") are for a model of ~1 kg and a
1.2 m wingspan. The `[AXIS_COUNT]` arrays are indexed by axis.

| Group | Constants |
|---|---|
| General | `GRAVITY = 9.80665`; the axes `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Speed | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| In the air/on the ground | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Controller | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Control surface effectiveness | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Stall | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Takeoff | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Landing | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**File:** `autopilot/feedback/FeedbackMath.h` · **Depends on:** `<math.h>`

| Function | Description |
|---|---|
| `float wrap180(float deg)` | An angle in `(−180, 180]`: the difference between headings 350° and 10° is −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Limits to `[−limit, limit]` |

---

## `FlightSnapshot`

**File:** `autopilot/feedback/FlightSnapshot.h` · **Kind:** struct

Everything the loop knows about the airplane for one cycle. The signs are
aviation signs.

| Group | Fields |
|---|---|
| Time/status | `timeUs`, `armed`, `linkLost` |
| Attitude | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Altitude | `baroValid`, `altitudeM` (from the power-on point), `climbRateMs`, `heightAglValid`, `heightAglM` (a future rangefinder) |
| Speed | `airspeedValid`, `airspeedMs` (a future pitot tube), `gpsValid`, `groundSpeedMs` |
| Mode targets | `stabilizationActive` (false = MANUAL: learning only), `targetRollDeg`, `targetPitchDeg` |
| Commands, µs | `stick*Us` — the pilot's contribution; `command*Us` — the result that actually went to the control surfaces |
| Throttle, % | `pilotThrottlePercent`, `throttlePercent` (actually to the ESC) |
| Flaps | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**File:** `autopilot/feedback/FeedbackOutput.h` · **Kind:** struct

| Field | Description |
|---|---|
| `float deflectionUs[3]` | Control surface deflections per axis, µs (the signs of `ControlCommand`) |
| `bool axisEnabled[3]` | `false` — the axis is not controlled, the surface stays with the pilot |
| `float throttleOverridePercent` | The absolute throttle of a flight phase; `< 0` — not set |
| `float throttleFloorPercent` | The lower bound of the throttle (stall protection); `< 0` — none |
| `targetRollDeg`, `targetPitchDeg` | The final targets after the limits (debugging) |
| `const char* reason` | A short description for the log/OLED |

---

## `PhaseTargets`

**File:** `autopilot/feedback/PhaseTargets.h` · **Kind:** struct

The common output of `TakeoffSequencer` and `LandingSequencer` — "what", not
"how".

| Field | Default | Description |
|---|---|---|
| `active` | `false` | The phase is now controlling the airplane |
| `targetRollDeg`, `targetPitchDeg` | 0 | The targets |
| `controlRoll`, `controlPitch` | `true` | `false` — leave the axis alone (on wheels the pitch is set by the landing gear) |
| `holdHeading`, `headingDeg` | `false`, 0 | Hold the heading with the rudder and the wheel |
| `throttlePercent` | −1 | −1 — the pilot's throttle |
| `reason` | `""` | A description |

---

## `SpeedEstimator`

**File:** `autopilot/feedback/SpeedEstimator.h`

The speed (airspeed > GPS ground speed > unknown) and the longitudinal
acceleration from the IMU: `dV/dt = g · (ax − sin θ)` through a low-pass filter
`ACCEL_FILTER_TAU_S` — "speed is dropping" is visible even without an airspeed
sensor.

| Method | Description |
|---|---|
| `void update(const FlightSnapshot&)` | A step; `dt ≤ 0` or `> 0.5 s` since the previous call (for the first — since `timeUs = 0`) — skipped |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², "+" — speeding up; no IMU — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, limited to `0.05..4`; without a speed — 1 |

---

## `AirborneDetector`

**File:** `autopilot/feedback/AirborneDetector.h`

Whether the airplane is in the air: learning, accumulating the integral and
looking for a stall only make sense in flight.

| Method | Description |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | Not armed — reset to "on the ground". A candidate for a state change must hold for `AIRBORNE_CONFIRM_MS` (takeoff) or `GROUND_STILL_MS` (landing) |
| `void force(bool)` | Set explicitly (known by takeoff/landing) |
| `void reset()` | On the ground |
| `bool isAirborne() const` | |

"Looks like flight": the rangefinder or barometer height > `AIRBORNE_HEIGHT_M`,
or the speed > `ROTATE_SPEED_MS`. "Looks like the ground": low, the angular
rates of all axes < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**File:** `autopilot/feedback/ControlEffectivenessEstimator.h`

One axis. The model: **angular acceleration = b·surface(t − delay) + a·ω + c**.
`b` is the control surface effectiveness (°/s² per µs, the sign is the
direction of the response), `a` is damping (1/s, usually < 0), `c` is a
constant moment (auto-trim). `b` is learned at a reference speed:
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`. The estimate is recursive least
squares with forgetting (`λ = 0.995`, a memory of ~4 s).

| Method | Description |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Calls `reset()` |
| `void reset()` | θ = (prior, 0, 0); covariance: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | Call it every cycle: accumulates means over an interval of `ESTIMATOR_PERIOD_MS`, at the end of the interval — the acceleration from the gyro difference, the command delay, a common low-pass filter on both sides, an RLS step (if learning is allowed and there is excitation) |
| `getEffectiveness()` | `b` at the current speed |
| `getReferenceEffectiveness()` | `b` at the reference speed |
| `getDamping()` | `a` at the current speed |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0 if `\|b\|` is small) |
| `getEffectivenessSigma()` | σ of the `b` estimate at the current speed |
| `bool isConfident() const` | ≥ 50 RLS steps, `\|b\|` ≥ the minimum and σ < 0.3·`\|b\|` |
| `getAngularAccel()` | The acceleration over the last interval (debugging) |

Features:

- An interval longer than `MAX_GAP_MS = 200` (the loop stood still) — the data
  starts over (the delay history and the filter are reset).
- It learns only under **excitation**: the swing of the mean commands over 16
  intervals (~0.3 s) ≥ `MIN_EXCITATION_US`; otherwise the estimate freezes.
- Float hygiene after a step: the symmetry of `P`, a ceiling on the variances
  (×10 of the initial ones), a limit on `b` (`±EFFECTIVENESS_MAX`) and `a`
  (`−40..5`).

---

## `AxisModel`

**File:** `autopilot/feedback/AdaptiveRateController.h` · **Kind:** struct

What is known about an axis's response, for the controller: `effectiveness` (b,
default 1), `damping` (a, 0 — do not compensate), `bias` (c, 0).

---

## `AdaptiveRateController`

**File:** `autopilot/feedback/AdaptiveRateController.h`

A single-axis controller, three stages:

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Method | Description |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | The integral, the saturation and the output — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Stage 1 (the shortest path for the heading) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Stages 2–3, returns the deflection, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | State |

Invariants: `|b|` is not less than `EFFECTIVENESS_MIN` (keeping the sign of b);
the integral is stored in °/s (it stays correct when `b` changes) and **does not
accumulate toward the limit** (anti-windup by the direction of the previous
step's saturation); `dt ≤ 0` — the integral does not change.

---

## `StallGuard`

**File:** `autopilot/feedback/StallGuard.h`

Protection against loss of speed and stall. The levels are `Level::{Normal, LowEnergy, Stall}`.

| Method | Description |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | On the ground or without an IMU — reset to Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | The last indication that fired |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, otherwise 90° |
| `float maxBankDeg() const` | Stall: 10°, otherwise 180° |
| `float maxAileronUs() const` | Stall: 150 µs, otherwise `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, otherwise/without a link −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` — `pitchEffectivenessKnown`, `pitchEffectiveness`
(the magnitude of the pitch b estimate).

**LowEnergy** indications: a confirmed (`DECEL_CONFIRM_MS`) deceleration
above `DECEL_WARN_MS2` at a pitch > 5°; speed < `1.25·Vs`; a confident elevator
effectiveness < 35 % of the a priori one. **Stall** indications: speed < Vs; the
nose drops faster than 60 °/s with the elevator "up" > 50 µs; at low energy the
wing drops faster than 120 °/s against the ailerons. The measures are released
after `RECOVERY_HOLD_MS` and only once the energy has recovered (speed ≥
`1.5·Vs`, without a speed sensor — acceleration ≥ 0).

---

## `TakeoffSequencer`

**File:** `autopilot/feedback/TakeoffSequencer.h`

Takeoff in phases. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
The diagram is in [ARCHITECTURE.md §7](../ARCHITECTURE.md#takeoff-and-landing-feedback-loop-not-connected).

| Method | Description |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | An active phase → `Aborted`; the targets are reset |
| `void update(snapshot, speed, nowMs)` | At most one transition per cycle, then the new phase's targets |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

The phase targets: waiting — throttle 0, the control surfaces with the pilot;
the roll — throttle 100 %, wings level, pitch left alone, hold the heading
captured at the moment of the start; the climb — throttle 100 %, wings level,
pitch `CLIMB_PITCH_DEG`. A hand throw: the longitudinal acceleration
`ax − sin θ ≥ LAUNCH_ACCEL_G` for longer than `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**File:** `autopilot/feedback/LandingSequencer.h`

Landing in phases. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Method | Description |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | As for takeoff |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — stall protection is switched off here |
| `bool isOnGround() const` | Rollout / Complete |

The pitch during the descent and flare comes from the vertical speed error:
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, limited to
`[min, FLARE_MAX_PITCH_DEG]`; without a barometer — the base angle. The
height is from the rangefinder, otherwise the barometer. Touchdown: a spike of
|a − 1g| ≥ 0.5g or "low and not rotating" for `TOUCHDOWN_STILL_MS`. During the
rollout the heading is captured at the moment of touchdown.

---

## `FeedbackSupervisor`

**File:** `autopilot/feedback/FeedbackSupervisor.h`

The loop as a whole. Owns the `SpeedEstimator`, `AirborneDetector`, three
`ControlEffectivenessEstimator`s, three `AdaptiveRateController`s, `StallGuard`,
`TakeoffSequencer` and `LandingSequencer`.

| Method | Description |
|---|---|
| `bool requestTakeoff()` | Only armed, with a link, on the ground; cancels a landing |
| `bool requestLanding()` | Only armed, with a link, in the air; cancels a takeoff |
| `void cancelPhase()` | Cancel the phase |
| `const FeedbackOutput& update(const FlightSnapshot&)` | A cycle (the order is in [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-the-feedback-loop-not-connected)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | State for the log and tests |
| `void printStatus(Print& out) const` | A status line + a line per axis (`b ± σ`, `*` — confident, `a`, `c`, `I`, the output) |

Key rules:

- **Not armed** — all the axes are off, `reason = "not armed"`; ARM/DISARM (a
  new flight) resets everything learned.
- **Link loss** cancels the phases; the throttle is not touched (the firmware's
  failsafe acts).
- **Liftoff** resets the estimates and controllers (what was "seen" on the
  wheels is no good).
- A negative confident estimate of `b` **never** goes into the controller — the
  axis works on the a priori model, and `reason` carries the warning "…
  responds to the surface backwards? check on the ground".
- The integral is frozen on the ground, except for the heading on the
  takeoff roll/rollout.
- Coordinated turn (with a known speed in the air): to the desired pitch rate
  `+ g/V · sin φ · tg φ`, to the yaw rate `g/V · sin φ` (the bank is limited to ±60°).
- The priority of `reason`: stall > low energy > phase > the sign warning >
  "stabilization"/"manual (learning)".
