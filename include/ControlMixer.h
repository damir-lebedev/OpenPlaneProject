#pragma once
// ============================================================
// 7. CONTROL MIXER
//
// Здесь находится только аэродинамическая логика.
//
// RC:
//
// CH1 → Aileron
// CH2 → Elevator
// CH5 → Flaps
//
// На выходе:
//
// Left Aileron
// Right Aileron
// Elevator
//
// Никакого UART.
// Никаких Servo.
// Никакого failsafe.
// Никакого millis().
//
// Это особенно важно для будущего автопилота:
//
// manual input и autopilot output смогут использовать
// один и тот же mixer.
// ============================================================

class ControlMixer
{
public:

    // --------------------------------------------------------
    // Расчёт управляющих поверхностей.
    // --------------------------------------------------------

    FlightOutputState calculate(
        const RcChannelState& rc
    ) const
    {
        FlightOutputState output;


        // ----------------------------------------------------
        // Получаем основные RC inputs.
        // ----------------------------------------------------

        const uint16_t aileronInput =
            rc.get(Channels::AILERON);

        const uint16_t elevatorInput =
            rc.get(Channels::ELEVATOR);


        // ----------------------------------------------------
        // Преобразуем Aileron.
        // ----------------------------------------------------

        const int16_t aileron =
            RcInput::centered(
                aileronInput,
                Config::AILERON_MAX_US,
                false
            );


        // ----------------------------------------------------
        // Преобразуем Elevator.
        // ----------------------------------------------------

        const int16_t elevator =
            RcInput::centered(
                elevatorInput,
                Config::ELEVATOR_MAX_US,
                false
            );


        // ----------------------------------------------------
        // Рассчитываем положение закрылков.
        //
        // CH5:
        //
        // < 1250 → 0 us
        // 1250..1749 → 50 us
        // >= 1750 → 100 us
        // ----------------------------------------------------

        const uint16_t flapOffset =
            calculateFlapOffset(
                rc.get(Channels::FLAPS)
            );


        // ----------------------------------------------------
        // LEFT AILERON
        //
        // Элерон + flap offset.
        // ----------------------------------------------------

        int32_t left =
            Config::PWM_CENTER +
            aileron +
            flapOffset;


        // ----------------------------------------------------
        // RIGHT AILERON
        //
        // Элерон зеркальный.
        //
        // Flap offset остаётся физически направленным вниз
        // относительно соответствующего крыла.
        // ----------------------------------------------------

        int32_t right =
            Config::PWM_CENTER -
            aileron -
            flapOffset;


        // ----------------------------------------------------
        // Ограничиваем выходы стандартным PWM диапазоном.
        // ----------------------------------------------------

        left = constrain(
            left,
            Config::PWM_MIN,
            Config::PWM_MAX
        );

        right = constrain(
            right,
            Config::PWM_MIN,
            Config::PWM_MAX
        );


        // ----------------------------------------------------
        // Elevator.
        // ----------------------------------------------------

        const int32_t elevatorOutput =
            constrain(
                Config::PWM_CENTER + elevator,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Формируем итоговое состояние поверхностей.
        // ----------------------------------------------------

        output.aileronLeft =
            static_cast<uint16_t>(left);

        output.aileronRight =
            static_cast<uint16_t>(right);

        output.elevator =
            static_cast<uint16_t>(elevatorOutput);


        return output;
    }


private:

    // --------------------------------------------------------
    // Преобразование положения CH5 в flap offset.
    // --------------------------------------------------------

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