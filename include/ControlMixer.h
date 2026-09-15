#pragma once
// ============================================================
// CONTROL MIXER
//
// Чистая аэродинамическая логика: CH1/CH2/CH5 -> положения
// поверхностей. Не знает про UART, Servo, failsafe или millis() —
// поэтому один и тот же mixer сможет использовать и ручное
// управление, и будущий автопилот.
// ============================================================

class ControlMixer
{
public:

    FlightOutputState calculate(
        const RcChannelState& rc
    ) const
    {
        FlightOutputState output;

        const uint16_t aileronInput = rc.get(Channels::AILERON);
        const uint16_t elevatorInput = rc.get(Channels::ELEVATOR);

        const int16_t aileron = RcInput::centered(
            aileronInput,
            Config::AILERON_MAX_US,
            false
        );

        const int16_t elevator = RcInput::centered(
            elevatorInput,
            Config::ELEVATOR_MAX_US,
            false
        );

        // CH5: < 1250 -> убраны (0), 1250..1749 -> половина (50), >= 1750 -> выпущены (100).
        const uint16_t flapOffset =
            calculateFlapOffset(
                rc.get(Channels::FLAPS)
            );

        // Элероны работают синхронно (зеркально), флапы добавляют
        // общий offset поверх стика.
        int32_t left =
            Config::PWM_CENTER +
            aileron +
            flapOffset;

        int32_t right =
            Config::PWM_CENTER -
            aileron -
            flapOffset;

        left = constrain(left, Config::PWM_MIN, Config::PWM_MAX);
        right = constrain(right, Config::PWM_MIN, Config::PWM_MAX);

        const int32_t elevatorOutput =
            constrain(
                Config::PWM_CENTER + elevator,
                Config::PWM_MIN,
                Config::PWM_MAX
            );

        output.aileronLeft = static_cast<uint16_t>(left);
        output.aileronRight = static_cast<uint16_t>(right);
        output.elevator = static_cast<uint16_t>(elevatorOutput);

        return output;
    }


private:

    uint16_t calculateFlapOffset(uint16_t input) const
    {
        if (input >= 1750)
        {
            return 100;
        }

        if (input >= 1250)
        {
            return 50;
        }

        return 0;
    }
};
