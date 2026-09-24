#pragma once
// ============================================================
// CONTROL MIXER
//
// Чистая аэродинамическая логика в два шага, без UART, Servo,
// failsafe и millis():
//
//   1. fromSticks(): RC-каналы -> ControlCommand — команда крена/
//      тангажа/закрылков в "физических" знаках (см. ниже).
//      FlightController прибавляет к ней коррекции автопилота —
//      в тех же знаках, поэтому автопилот и стики гарантированно
//      крутят рули в одну сторону.
//   2. mix(): ControlCommand -> FlightOutputState (PWM на каждый
//      серво) с учётом реверса серво из Config.h.
//
// Знаки ControlCommand (мкс отклонения, ±500 = полный ход):
//   roll  > 0 — крен вправо   (правый элерон вверх, левый вниз)
//   pitch > 0 — нос вверх     (руль высоты вверх)
//   flaps > 0 — закрылки вниз (оба элерона вниз)
//
// Раньше крен со стика и закрылки подавались на элероны по одной
// и той же схеме (левый +x, правый -x), то есть физически двигали
// рули одинаково — одно из двух было неверно при любой установке
// серво. А коррекция тангажа автопилота шла ещё и на элероны.
// ============================================================

struct ControlCommand
{
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t flaps = 0;
};

class ControlMixer
{
public:

    ControlCommand fromSticks(
        const RcChannelState& rc
    ) const
    {
        ControlCommand command;

        // CH1: 2000 = стик вправо = крен вправо.
        command.roll = RcInput::centered(
            rc.get(Channels::AILERON),
            Config::AILERON_MAX_US,
            false
        );

        // CH2: 2000 = стик от себя = нос вниз, поэтому знак обратный.
        command.pitch = RcInput::centered(
            rc.get(Channels::ELEVATOR),
            Config::ELEVATOR_MAX_US,
            true
        );

        // CH9: линейно 1000..2000 -> 0..FLAPS_MAX_US (крутилка на
        // пульте, а не 3-позиционный переключатель).
        command.flaps = static_cast<int16_t>(
            map(
                RcInput::clamp(rc.get(Channels::FLAPS)),
                Config::PWM_MIN,
                Config::PWM_MAX,
                0,
                Config::FLAPS_MAX_US
            )
        );

        return command;
    }

    FlightOutputState mix(
        const ControlCommand& command
    ) const
    {
        const int32_t roll = constrain(command.roll, -Config::AILERON_MAX_US, Config::AILERON_MAX_US);
        const int32_t pitch = constrain(command.pitch, -Config::ELEVATOR_MAX_US, Config::ELEVATOR_MAX_US);

        // Отклонение задней кромки вниз (+) для каждого элерона.
        const int32_t leftDown = roll + command.flaps;
        const int32_t rightDown = -roll + command.flaps;

        FlightOutputState output;

        output.aileronLeft = toPwm(leftDown, Config::AILERON_LEFT_REVERSED);
        output.aileronRight = toPwm(rightDown, Config::AILERON_RIGHT_REVERSED);
        output.elevator = toPwm(pitch, Config::ELEVATOR_REVERSED);

        return output;
    }


private:

    static uint16_t toPwm(int32_t deflection, bool reversed)
    {
        const int32_t pwm = Config::PWM_CENTER + (reversed ? -deflection : deflection);
        return static_cast<uint16_t>(constrain(pwm, Config::PWM_MIN, Config::PWM_MAX));
    }
};
