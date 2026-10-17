# CONTROL and COORDINATION — the mixer, throttle, ARM, outputs, the orchestrator

> 🌐 This page is a translation of the [Russian original](../../../reference/control.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

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

`throttle` is the throttle magnitude (1000..2000 = 0..100%) in either direction; `FlightOutputs` turns it, together with `motorReverse`, into the ESC pulse (via `EscSignal`).

| Field | Default |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` — the motor runs forwards |
| `bool escPulsesOff` | `false`; `true` — no pulses to the ESC at all (motor signal, only without ARM) |
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
| `FlightOutputState mix(const ControlCommand& c) const` | Command → PWM. Roll/pitch/yaw are limited by the travel (`*_MAX_US`); ailerons: left = `flaps + roll`, right = `flaps − roll` (down = "+"); PWM = `1500 ± deflection` with the sign from `Config::*_REVERSED`, the flaps sign — `FLAPS_REVERSED`, limited to 1000..2000. `throttle` is not filled in |
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

## `EscSignal`

**File:** `control/EscSignal.h` · **Kind:** namespace · **Depends on:** `Config`

Throttle (1000..2000 = 0..100%) and direction → the pulse on the ESC pin.

| Function | Description |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (BLHeli 80A in 3D): 0% → `ESC_NEUTRAL_US` (1500), forwards 1500..2000, backwards 1500..1000. A unidirectional ESC: 1000..2000 as is, `reverse` is ignored. Throttle outside 1000..2000 is clamped |
| `uint16_t stopUs()` | The “motor stopped” pulse (1500 for a bidirectional ESC, 1000 for an ordinary one) |

Conversion only, no protections: `MotorReverse` decides when the direction may change, and `FlightController` sets the backwards throttle limit.

---

## `MotorReverse`

**File:** `control/MotorReverse.h` · **Depends on:** `Config`

The motor direction from the `Feature::MOTOR_REVERSE` switch (SwD by default) — **only when disarmed**.

| Method | Description |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | Not armed — the direction follows the switch at once (Serial prints `MotorReverse: мотор НАЗАД` / `вперёд` — “motor BACKWARDS” / “forwards”). Armed — the switch has no effect, “реверс НЕ переключён” (“reverse NOT switched”) is printed once, and the new direction waits for DISARM. With `ESC_BIDIRECTIONAL = false` reverse is never engaged |
| `bool isReversed() const` | The motor runs backwards |
| `bool isPending() const` | The switch is already in the other position; the direction waits for DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | A reason to refuse ARM: less than `REVERSE_SETTLE_MS` has passed since the last direction change (the motor may still be turning); otherwise `nullptr` |

---

## `ArmingManager`

**File:** `control/ArmingManager.h` · **Depends on:** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

ARM by a separate switch SwA (CH5). The state machine is in
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Method | Description |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Without an autopilot only the throttle is checked |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | On failsafe — nothing (the switch in a failsafe frame does not reflect the pilot). Switch OFF → DISARM, `switchSeenOff = true`. An OFF→ON transition → checks → ARM or a refusal `blockReason` is an external refusal reason (from `FlightController`: `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | Armed |
| `const char* getLastRefusalReason() const` | The reason for the last refusal or `nullptr`; reset when the switch is turned off |

`checkFailureReason(rc)` — the ARM checks:

| Condition | Reason for refusal |
|---|---|
| Throttle ≥ `THROTTLE_LOW_US` | "throttle not at minimum" |
| Any mode except MANUAL, the IMU exists but does not respond | "the IMU does not respond…" |
| Any mode except MANUAL, the IMU has a preflight check problem | the text of `ImuSensor::getPreflightProblem()` |
| A mode with altitude (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), the barometer exists but does not respond | "the barometer does not respond…" |
| Everything else passed, but `blockReason` was given | `blockReason` (e.g. “the motor direction has just changed…”) |

A sensor that is absent from the build (`nullptr`) does not block ARM; in MANUAL
the aircraft arms with no sensors at all. A GPS fix is deliberately not among
the checks: without GPS the navigation modes behave safely (a circle in
place), and home is recorded once the GPS catches satellites.

Invariants: powering the board on with the switch ON does not arm; one attempt
per OFF→ON transition; link loss does not clear ARM.

---

## `FlightOutputs`

**File:** `control/FlightOutputs.h` · **Depends on:** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

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
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` of each output, prints the status and the ESC type; `true` if all the **required** ones got a channel |
| `bool isAttached(uint8_t ch) const` | The output is connected (an index out of range → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | The pulse on the output pin: the field from the table, for the ESC via `EscSignal::pulseUs(throttle, motorReverse)`, 0 with `escPulsesOff` |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | The measured pulse against the expected one on each wired pin; "OK" when the discrepancy is ≤ 15 µs |
| `void write(const FlightOutputState&)` | Write all the outputs and remember the state; the ESC with `escPulsesOff` — `IServoOutput::stopPulses()` |
| `void setFailsafe()` | Control surfaces neutral (`FAILSAFE_*`), throttle `FAILSAFE_THROTTLE` (1500 on the pin of a bidirectional ESC); AUX as they were (the payload is not dropped on link loss) |
| `void setBuzzer(bool on)` | the board's beeper (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | The last written state |

To add an output: a table row + a field in `FlightOutputState` + an index in
`ServoChannel` (+ a pin and an LEDC channel in `Esp32Board`).

---

## `FlightController`

**File:** `control/FlightController.h` · **Layer:** COORDINATION ·
**Depends on:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

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
| `const MotorReverse& getMotorReverse() const` | The motor direction and a pending reverse |
| `void toggleGroundCalibration()` | Console `c`: ground calibration of the airplane — start (only without ARM), or cancel while it runs |
| `void playSignal(uint8_t count)` | Console `e`: a signal (without ARM — by the motor too) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | For the OLED, the log and the black box |

The order of `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. with a live link — `switches->update(rc)` (the mode, functions, knobs);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. the sticks `mixer.fromSticks(rc)` (with a live link) × `Knob::RATES`; the
   flaps `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → smoothly, on link loss — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **always**;
   in reverse `PWM_MIN` is passed instead of `pilotThrottle` (for the autopilot the motor is off);
6. calibration: an ARM flick during it cancels it; without ARM and with a link — the `CAL_SWITCH` gesture; a `GroundCalibration::update()` step; its signals and the auto-takeoff refusal signal go to `SignalPlayer`; the beeper: `Beeper::update(BEEPER, armed, failsafe, now)` or the signal light;
7. link lost → `applyLinkLoss()` and leaving the cycle;
8. `arming.update(rc, false, reason)` — ARM is refused while a calibration runs or a signal plays (and in the tick when an ARM flick cancelled a calibration), otherwise `reverse.armBlockReason(now)`; then `reverse.update(MOTOR_REVERSE, armed, now)` — on DISARM the direction changes in the same tick;
9. `command = autopilot->getCommand()` (or the sticks without an autopilot), the flaps — its own; the signal nod goes to the elevator and the rudder;
10. `output = mixer.mix(command)`; `output.motorReverse = reverse.isReversed()`;
    forwards — `output.throttle = autopilot->applyThrottle(pilotThrottle)` limited by
    `THROTTLE_LIMIT_PCT` × the `Knob::THROTTLE_LIMIT` knob, backwards — the pilot's throttle limited by
    `min(REVERSE_THROTTLE_LIMIT_PCT, the forward limit)`; the allowed range is stretched over the whole stick travel;
11. not armed or `MOTOR_KILL` → `throttle = PWM_MIN` (last); without ARM the motor signal sets `escPulsesOff`;
12. AUX1 — the payload (`PAYLOAD_DROP`), AUX2 — the camera (`Knob::CAMERA_TILT`, `CAMERA_STAB` subtracts the pitch); `AUX1_REVERSED`/`AUX2_REVERSED` mirror the pulse.
13. `outputs.write(output)`.

`applyLinkLoss()`: if the autopilot is in failsafe (armed: RTH or gliding) — surfaces and throttle per the autopilot command (the flaps retract smoothly, `MOTOR_KILL` and reverse stop the motor, AUX as they were); otherwise `outputs.setFailsafe()`, and while a signal plays — neutral with the nod and the ESC pulse pause (the transmitter was put down while the calibration runs).

---

## `Beeper`

**File:** `control/Beeper.h` · **Depends on:** `Config`

| Method | Description |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | the beeper state: 2 Hz if `Feature::BEEPER` or "model lost" (not armed, no link for longer than `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | the "look for me in the grass" mode |

---

## `GroundCalibration`

**File:** `control/GroundCalibration.h` · **Depends on:** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

Ground calibration of the airplane: step 1 — tail up, fuselage horizontal (the horizon, `ImuOrientation::withLevel()`), step 2 — on the wheels (the parking attitude). A step is accepted by itself after `CAL_STILL_MS` of stillness; step 2 waits until the airplane is moved, or `CAL_PARK_WAIT_MS`. It is saved only as a whole (`ImuSensor::setGroundCalibration()`), after the parking attitude is checked against `LANDING_GEAR`. It does not block the loop.

`Stage`: `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`.

| Method | Description |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | the gesture: `CAL_SWITCH` down and up `CAL_SWITCH_FLIPS` times within `CAL_SWITCH_WINDOW_MS`, throttle down; the first look and a raised throttle restart the count |
| `void forgetSwitch()` | ARM or no link — the count starts over |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | start (signal 1); no IMU or unsupported — error (signal 3) |
| `void cancel(const char* why)` | cancel without a signal |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | a step: ARM cancels; nothing is accepted while a signal plays; `CAL_STEP_TIMEOUT_MS` timeout — error |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | state and result |
| `uint8_t takeSignal()` | how many signals to play: 1 — started / step 1, 2 — done, 3 — error (taken once) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` — for the OLED |

---

## `SignalPlayer`

**File:** `control/SignalPlayer.h` · **Depends on:** `Config`

Signals to the pilot — a function of the time since the series started. One signal: the elevator and the rudder nod (`SIGNAL_NOD_US` up, then down, `SIGNAL_NOD_MS` each); without ARM the ESC pulses pause for `SIGNAL_ESC_PAUSE_MS` (afterwards the ESC plays its ready tune); the buzzer light is on during the nod; the next signal follows after `SIGNAL_PERIOD_MS`.

| Method | Description |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | a series; `motor` — whether the ESC may be touched (and `SIGNAL_MOTOR`) |
| `void stop()` | silence |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | a series is playing (ARM is refused meanwhile) |
| `bool escPulsesOff(uint32_t nowMs) const` | drop the ESC pulses now |
| `int16_t nodUs(uint32_t nowMs) const` | the control-surface nod, µs (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | buzzer / LED on `PIN_BUZZER` |
