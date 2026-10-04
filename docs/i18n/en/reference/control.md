# CONTROL and COORDINATION — the mixer, throttle, ARM, outputs, the orchestrator

> 🌐 This page is a translation of the [Russian original](../../../reference/control.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

The CONTROL layer is logic over data, with no UART, PWM or Wi-Fi.
`FlightController` (COORDINATION) is the only class that brings all the lower
layers together into a single cycle.

---

## `ControlCommand`

**File:** `control/ControlCommand.h` · **Kind:** struct

A command to the control surfaces in **physical signs**, µs of deflection (±500
= full travel). The common language of the sticks, the autopilot and the mixer.

| Field | "+" means |
|---|---|
| `int16_t roll` | roll right (right aileron up, left down) |
| `int16_t pitch` | nose up (elevator up) |
| `int16_t yaw` | nose right (rudder and wheel to the right) |
| `int16_t flaps` | flaps down (both ailerons down); "−" — airbrake (both up) |

All fields default to 0.

---

## `FlightOutputState`

**File:** `control/FlightOutputState.h` · **Kind:** struct

The desired output pulses, PWM µs. By default — control surfaces neutral and
throttle `PWM_MIN`.

| Field | Default |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `aux1` | `PAYLOAD_CLOSED_US` — the payload drop is closed |
| `aux2` | `PWM_CENTER` — the camera |

---

## `FlapsController`

**File:** `control/FlapsController.h` · **Depends on:** `Config`

Smooth deploying/retracting of the flaps: the position moves toward the target
(any value — flaps from the switch, from the knob, an airbrake upward) no
faster than the full travel `FLAPS_DEPLOYED_US` in `FLAPS_TRANSITION_MS`. The
time is passed as a parameter.

| Method | Description |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | A step toward the target; returns the current position, µs (+ down, − up) |
| `int16_t getPosition() const` | The current position |

Invariants:

- **The first call** sets the position straight to the target — the flaps do
  not "slide out" on the desk at power-on.
- The time step is limited to `MAX_STEP_MS = 20`: after a long pause (failsafe,
  calibration) the flaps do not jump to the target in a single cycle.

---

## `ControlMixer`

**File:** `control/ControlMixer.h` · **Depends on:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

The aerodynamic logic in two steps. Owns the `FlapsController`.

| Method | Description |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = right); CH2 → `pitch` **with the opposite sign** (2000 = away from you = nose down); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | the flaps target is chosen by `FlightController` (airbrake → flaps switch → knob), the smooth travel is here |
| `FlightOutputState mix(const ControlCommand& c) const` | Command → PWM. Roll/pitch/yaw are limited by the travel (`*_MAX_US`); ailerons: left = `flaps + roll`, right = `flaps − roll` (down = "+"); PWM = `1500 ± deflection` with the sign from `Config::*_REVERSED`, limited to 1000..2000. `throttle` is not filled in |
| `int16_t getFlaps() const` | The current flaps position, µs |

Flaperons: on deploying, both ailerons drop by `FLAPS_DEPLOYED_US` (the new
"neutral"), and roll works on top of that. At full roll the descending aileron
hits the end of its travel earlier than the rising one — this works as aileron
differential.

---

## `ThrottleManager`

**File:** `control/ThrottleManager.h` · **Depends on:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Method | Description |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | The CH3 throttle, limited to 1000..2000; on link loss — `FAILSAFE_THROTTLE` |

It knows nothing about ARM and the autopilot — their corrections are applied by
`FlightController`.

---

## `ArmingManager`

**File:** `control/ArmingManager.h` · **Depends on:** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

ARM by a separate switch SwA (CH5). The state machine is in
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Method | Description |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Without an autopilot only the throttle is checked |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | On failsafe — nothing (the switch in a failsafe frame does not reflect the pilot). Switch OFF → DISARM, `switchSeenOff = true`. An OFF→ON transition → checks → ARM or a refusal |
| `bool isArmed() const` | Armed |
| `const char* getLastRefusalReason() const` | The reason for the last refusal or `nullptr`; reset when the switch is turned off |

`checkFailureReason(rc)` — the ARM checks:

| Condition | Reason for refusal |
|---|---|
| Throttle ≥ `THROTTLE_LOW_US` | "throttle not at minimum" |
| Any mode except MANUAL, the IMU exists but does not respond | "the IMU does not respond…" |
| Any mode except MANUAL, the IMU has a preflight check problem | the text of `ImuSensor::getPreflightProblem()` |
| A mode with altitude (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), the barometer exists but does not respond | "the barometer does not respond…" |

A sensor that is absent from the build (`nullptr`) does not block ARM; in MANUAL
the aircraft arms with no sensors at all. A GPS fix is deliberately not among
the checks: without GPS the navigation modes behave safely (a circle in
place), and home is recorded once the GPS catches satellites.

Invariants: powering the board on with the switch ON does not arm; one attempt
per OFF→ON transition; link loss does not clear ARM.

---

## `FlightOutputs`

**File:** `control/FlightOutputs.h` · **Depends on:** `IBoard`, `FlightOutputState`, `Config`

The only class that knows the set and order of the PWM outputs. All the outputs
are described in one table; `begin()`, `write()`, the status and the self-test
walk through it in a loop.

### `FlightOutputs::OutputInfo`

| Field | Description |
|---|---|
| `const char* key` | The name in the JSON/log (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | The human-readable name |
| `int16_t pin` | The pin number; `-1` — not wired. `int16_t`, because on the STM32 analog pin numbers are `0xC0 + N` |
| `bool required` | Without it the aircraft does not fly (the rudder is optional) |
| `uint16_t FlightOutputState::* field` | A pointer to the state field |

| Method | Description |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | A table row; the order = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` of each output, prints the status; `true` if all the **required** ones got a channel |
| `bool isAttached(uint8_t ch) const` | The output is connected (an index out of range → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | The output's value from the state, by the table |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | The measured pulse against the expected one on each wired pin; "OK" when the discrepancy is ≤ 15 µs |
| `void write(const FlightOutputState&)` | Write all the outputs and remember the state |
| `void setFailsafe()` | Control surfaces neutral (`FAILSAFE_*`), throttle `FAILSAFE_THROTTLE`; AUX as they were (the payload is not dropped on link loss) |
| `void setBuzzer(bool on)` | the board's beeper (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | The last written state |

To add an output: a table row + a field in `FlightOutputState` + an index in
`ServoChannel` (+ a pin and an LEDC channel in `Esp32Board`).

---

## `FlightController`

**File:** `control/FlightController.h` · **Layer:** COORDINATION ·
**Depends on:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

The sole coordinator of the control loop: it does not parse the UART, touch
the PWM or compute the mixer itself — it only calls the others in the right
order. A detailed diagram is in [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-the-control-tick-flightcontrollerupdate).

| Method | Description |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Without an autopilot — pure manual control; without switches — only the sticks |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | One cycle (see below) |
| `bool isReceiverFailsafe() const` | The link is lost |
| `const IBusReceiver& getReceiver() const` | For the log (frame counters, the cause of the loss) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | The last one written to the outputs |
| `const RcChannelState& getRcState() const` | The channels |
| `const FlightOutputs& getOutputs() const` | The output table and `attached` |
| `int16_t getFlapsUs() const` | The flaps position |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | This cycle's switches and knobs |
| `bool isLostModelBeeping() const` | The "I am here" beeper is running |

The order of `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. with a live link — `switches->update(rc)` (the mode, functions, knobs);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. the sticks `mixer.fromSticks(rc)` (with a live link) × `Knob::RATES`; the
   flaps `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → smoothly, on link loss — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **always**;
6. the beeper: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. link lost → `applyLinkLoss()` and leaving the cycle;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (or the sticks without an autopilot), the flaps — its own;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. not armed or `MOTOR_KILL` → `throttle = PWM_MIN` (last);
12. AUX1 — the payload (`PAYLOAD_DROP`), AUX2 — the camera (`Knob::CAMERA_TILT`, `CAMERA_STAB` subtracts the pitch);
13. `outputs.write(output)`.

`applyLinkLoss()`: if the autopilot is in failsafe (armed: RTH or gliding) —
the control surfaces and throttle follow the autopilot's command (the flaps
retract smoothly, `MOTOR_KILL` still silences the motor, AUX as they were);
otherwise `outputs.setFailsafe()`.

---

## `Beeper`

**File:** `control/Beeper.h` · **Depends on:** `Config`

| Method | Description |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | the beeper state: 2 Hz if `Feature::BEEPER` or "model lost" (not armed, no link for longer than `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | the "look for me in the grass" mode |
