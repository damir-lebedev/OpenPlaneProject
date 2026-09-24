#pragma once
// ============================================================
// CONTROL MIXER
//
// Аэродинамическая логика в два шага, без UART, Servo и failsafe:
//
//   1. fromSticks(): RC-каналы -> ControlCommand — команда крена/
//      тангажа/рысканья/закрылков в "физических" знаках (см. ниже).
//      FlightController прибавляет к ней коррекции автопилота —
//      в тех же знаках, поэтому автопилот и стики гарантированно
//      крутят рули в одну сторону.
//   2. mix(): ControlCommand -> FlightOutputState (PWM на каждый
//      серво) с учётом реверса серво из Config.h.
//
// Знаки ControlCommand (мкс отклонения, ±500 = полный ход):
//   roll  > 0 — крен вправо   (правый элерон вверх, левый вниз)
//   pitch > 0 — нос вверх     (руль высоты вверх)
//   yaw   > 0 — нос вправо    (руль направления и колесо вправо)
//   flaps > 0 — закрылки вниз (оба элерона вниз)
//
// ЗАКРЫЛКИ (флапероны). Отдельных закрылков нет — их роль играют
// элероны: при выпуске оба опускаются на одинаковый угол (это новая
// "нейтраль" элеронов, растёт подъёмная сила), а крен от стика и
// автопилота добавляется поверх неё, как обычно, в разные стороны.
// Выпуск — тумблером SwB, до Config::FLAPS_DEPLOYED_US, плавно за
// Config::FLAPS_TRANSITION_MS, чтобы не было клевка по тангажу.
//
// Раньше крен со стика и закрылки подавались на элероны по одной
// и той же схеме (левый +x, правый -x), то есть закрылки физически
// работали как триммер крена, а не опускали оба элерона.
// ============================================================

struct ControlCommand
{
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;
    int16_t flaps = 0;
};

class ControlMixer
{
public:

    // nowMs — текущее время (millis()), нужно только для плавного
    // выпуска закрылков; передаётся снаружи, чтобы микшер не зависел
    // от системных часов.
    ControlCommand fromSticks(
        const RcChannelState& rc,
        uint32_t nowMs
    )
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

        // CH4: 2000 = левый стик вправо = нос вправо.
        command.yaw = RcInput::centered(
            rc.get(Channels::RUDDER),
            Config::RUDDER_MAX_US,
            false
        );

        const bool flapsDeployed =
            rc.get(Channels::FLAPS) >= Config::FLAPS_SWITCH_ON_US;

        command.flaps = updateFlaps(flapsDeployed ? Config::FLAPS_DEPLOYED_US : 0, nowMs);

        return command;
    }

    FlightOutputState mix(
        const ControlCommand& command
    ) const
    {
        const int32_t roll = constrain(command.roll, -Config::AILERON_MAX_US, Config::AILERON_MAX_US);
        const int32_t pitch = constrain(command.pitch, -Config::ELEVATOR_MAX_US, Config::ELEVATOR_MAX_US);
        const int32_t yaw = constrain(command.yaw, -Config::RUDDER_MAX_US, Config::RUDDER_MAX_US);

        // Отклонение задней кромки вниз (+) для каждого элерона:
        // общая "нейтраль" закрылков + крен в разные стороны. Если
        // сумма выходит за ход серво, toPwm() обрежет опускающийся
        // элерон, поднимающийся продолжит отклоняться — это работает
        // как дифференциал элеронов.
        const int32_t leftDown = command.flaps + roll;
        const int32_t rightDown = command.flaps - roll;

        FlightOutputState output;

        output.aileronLeft = toPwm(leftDown, Config::AILERON_LEFT_REVERSED);
        output.aileronRight = toPwm(rightDown, Config::AILERON_RIGHT_REVERSED);
        output.elevator = toPwm(pitch, Config::ELEVATOR_REVERSED);
        output.rudder = toPwm(yaw, Config::RUDDER_REVERSED);

        return output;
    }

    // Текущее (плавно меняющееся) положение закрылков, мкс.
    int16_t getFlaps() const
    {
        return static_cast<int16_t>(flapsUs);
    }


private:

    float flapsUs = 0;
    uint32_t lastFlapsUpdateMs = 0;
    bool flapsInitialized = false;

    // Двигает закрылки к цели не быстрее, чем FLAPS_DEPLOYED_US за
    // FLAPS_TRANSITION_MS. При первом вызове (включение платы) —
    // сразу в целевое положение, без "выезда" на столе.
    int16_t updateFlaps(int16_t targetUs, uint32_t nowMs)
    {
        if (!flapsInitialized)
        {
            flapsUs = targetUs;
            lastFlapsUpdateMs = nowMs;
            flapsInitialized = true;
            return targetUs;
        }

        const float maxStep =
            static_cast<float>(Config::FLAPS_DEPLOYED_US) * (nowMs - lastFlapsUpdateMs) /
            Config::FLAPS_TRANSITION_MS;
        lastFlapsUpdateMs = nowMs;

        const float delta = targetUs - flapsUs;
        flapsUs += constrain(delta, -maxStep, maxStep);

        return static_cast<int16_t>(flapsUs);
    }

    static uint16_t toPwm(int32_t deflection, bool reversed)
    {
        const int32_t pwm = Config::PWM_CENTER + (reversed ? -deflection : deflection);
        return static_cast<uint16_t>(constrain(pwm, Config::PWM_MIN, Config::PWM_MAX));
    }
};
