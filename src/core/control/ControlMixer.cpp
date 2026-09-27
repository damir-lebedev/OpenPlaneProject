// Реализация control/ControlMixer.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/ControlMixer.h"


auto ControlMixer::fromSticks(const RcChannelState& rc) const -> ControlCommand
{
    ControlCommand command;
    command.roll = RcInput::centered(rc.get(Channels::AILERON), Config::AILERON_MAX_US, false);
    command.pitch = RcInput::centered(rc.get(Channels::ELEVATOR), Config::ELEVATOR_MAX_US, true);
    command.yaw = RcInput::centered(rc.get(Channels::RUDDER), Config::RUDDER_MAX_US, false);
    return command;
}

auto ControlMixer::updateFlaps(float targetUs, uint32_t nowMs) -> int16_t
{
    return flaps.update(targetUs, nowMs);
}

auto ControlMixer::mix(const ControlCommand& command) const -> FlightOutputState
{
    const int32_t roll = constrain(command.roll, -Config::AILERON_MAX_US, Config::AILERON_MAX_US);
    const int32_t pitch = constrain(command.pitch, -Config::ELEVATOR_MAX_US, Config::ELEVATOR_MAX_US);
    const int32_t yaw = constrain(command.yaw, -Config::RUDDER_MAX_US, Config::RUDDER_MAX_US);

    // Отклонение задней кромки вниз (+) для каждого элерона:
    // общая "нейтраль" закрылков + крен в разные стороны. Если
    // сумма выходит за ход серво, toPwm() обрежет опускающийся
    // элерон, а поднимающийся продолжит отклоняться — это
    // работает как дифференциал элеронов.
    const int32_t leftDown = command.flaps + roll;
    const int32_t rightDown = command.flaps - roll;

    FlightOutputState output;

    output.aileronLeft = toPwm(leftDown, Config::AILERON_LEFT_REVERSED);
    output.aileronRight = toPwm(rightDown, Config::AILERON_RIGHT_REVERSED);
    output.elevator = toPwm(pitch, Config::ELEVATOR_REVERSED);
    output.rudder = toPwm(yaw, Config::RUDDER_REVERSED);

    return output;
}

auto ControlMixer::getFlaps() const -> int16_t
{
    return flaps.getPosition();
}

auto ControlMixer::toPwm(int32_t deflection, bool reversed) -> uint16_t
{
    const int32_t pwm = Config::PWM_CENTER + (reversed ? -deflection : deflection);
    return static_cast<uint16_t>(constrain(pwm, Config::PWM_MIN, Config::PWM_MAX));
}
