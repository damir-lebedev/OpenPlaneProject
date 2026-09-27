// Реализация control/FlightController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/FlightController.h"


FlightController::FlightController(
        IBusReceiver& rcReceiver,
        ControlMixer& controlMixer,
        ThrottleManager& throttleManager,
        ArmingManager& armingManager,
        FlightOutputs& flightOutputs,
        Autopilot* ap,
        PilotSwitches* pilotSwitches
    )
: receiver(rcReceiver),
      mixer(controlMixer),
      throttle(throttleManager),
      arming(armingManager),
      outputs(flightOutputs),
      autopilot(ap),
      switches(pilotSwitches)
{
}

auto FlightController::begin() -> void
{
    outputs.setFailsafe();
    receiver.begin();
}

auto FlightController::update() -> void
{
    receiver.update();

    const bool receiverFailsafe = receiver.isSignalLost();
    const RcChannelState& rc = receiver.getState();
    const uint32_t nowMs = millis();

    // Режим и функции с тумблеров — только при живой связи: в
    // failsafe-кадре каналы содержат значения failsafe.
    if (switches && !receiverFailsafe) switches->update(rc);
    const PilotInputs& in = inputs();

    const uint16_t pilotThrottle = throttle.update(rc, receiverFailsafe);

    ControlCommand sticks;
    if (!receiverFailsafe)
    {
        sticks = mixer.fromSticks(rc);
        applyRates(sticks, in);
    }
    sticks.flaps = mixer.updateFlaps(receiverFailsafe ? 0.0f : flapsTarget(in), nowMs);

    if (autopilot) autopilot->update(arming.isArmed(), receiverFailsafe, pilotThrottle, sticks);

    outputs.setBuzzer(beeper.update(in.has(Feature::BEEPER), arming.isArmed(), receiverFailsafe, nowMs));

    if (receiverFailsafe)
    {
        applyLinkLoss(sticks.flaps, in);
        return;  // стики и тумблеры в этом цикле не участвуют
    }

    arming.update(rc, false);

    ControlCommand command = autopilot ? autopilot->getCommand() : sticks;
    command.flaps = sticks.flaps;

    FlightOutputState output = mixer.mix(command);
    output.throttle = autopilot ? autopilot->applyThrottle(pilotThrottle) : pilotThrottle;

    // ARM реально блокирует газ (см. ArmingManager) — ставим ПОСЛЕ
    // автопилота, чтобы ни один режим не мог протащить газ мимо
    // этой проверки. MOTOR_KILL — то же, по тумблеру.
    if (!arming.isArmed() || in.has(Feature::MOTOR_KILL))
    {
        output.throttle = Config::PWM_MIN;
    }

    applyAuxOutputs(output, in);
    outputs.write(output);
}

auto FlightController::inputs() const -> const PilotInputs&
{
    return switches ? switches->getInputs() : noInputs;
}

auto FlightController::applyRates(ControlCommand& sticks, const PilotInputs& in) -> void
{
    if (!in.isBound(Knob::RATES)) return;

    const float centre = (Config::RATES_MIN_PCT + Config::RATES_MAX_PCT) * 0.5f;
    const float scale = in.knobValue(Knob::RATES, Config::RATES_MIN_PCT, centre, Config::RATES_MAX_PCT) / 100.0f;
    sticks.roll = static_cast<int16_t>(sticks.roll * scale);
    sticks.pitch = static_cast<int16_t>(sticks.pitch * scale);
    sticks.yaw = static_cast<int16_t>(sticks.yaw * scale);
}

auto FlightController::flapsTarget(const PilotInputs& in) -> float
{
    if (in.has(Feature::AIRBRAKE)) return -static_cast<float>(Config::AIRBRAKE_US);
    if (in.has(Feature::FLAPS)) return Config::FLAPS_DEPLOYED_US;
    if (in.isBound(Knob::FLAPS)) return (in.knob(Knob::FLAPS) + 1.0f) * 0.5f * Config::FLAPS_DEPLOYED_US;
    return 0.0f;
}

auto FlightController::applyAuxOutputs(FlightOutputState& output, const PilotInputs& in) const -> void
{
    output.aux1 = in.has(Feature::PAYLOAD_DROP) ? Config::PAYLOAD_OPEN_US : Config::PAYLOAD_CLOSED_US;

    float tilt = in.knobValue(Knob::CAMERA_TILT, Config::CAMERA_TILT_MIN_DEG, 0.0f, Config::CAMERA_TILT_MAX_DEG);
    const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
    if (in.has(Feature::CAMERA_STAB) && imu && imu->isAvailable())
    {
        tilt -= imu->getImuData().pitch;
    }
    const float pwm = Config::PWM_CENTER + tilt * Config::CAMERA_US_PER_DEG;
    output.aux2 = static_cast<uint16_t>(constrain(pwm, static_cast<float>(Config::PWM_MIN),
                                                  static_cast<float>(Config::PWM_MAX)));
}

auto FlightController::applyLinkLoss(int16_t flapsUs, const PilotInputs& in) -> void
{
    if (!autopilot || !autopilot->isFailsafeActive())
    {
        outputs.setFailsafe();
        return;
    }

    ControlCommand command = autopilot->getCommand();
    command.flaps = flapsUs;

    FlightOutputState output = mixer.mix(command);
    output.throttle = autopilot->applyThrottle(Config::FAILSAFE_THROTTLE);
    if (in.has(Feature::MOTOR_KILL)) output.throttle = Config::PWM_MIN;
    output.aux1 = outputs.getLastState().aux1;
    output.aux2 = outputs.getLastState().aux2;
    outputs.write(output);
}
