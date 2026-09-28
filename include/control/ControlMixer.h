#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "control/ControlCommand.h"
#include "control/FlapsController.h"
#include "control/FlightOutputState.h"
#include "rc/RcChannelState.h"
#include "rc/RcInput.h"

// ============================================================
// CONTROL MIXER
//
// Аэродинамическая логика в два шага, без UART, Servo и failsafe:
//
//   1. fromSticks(): RC-каналы стиков -> ControlCommand (знаки — см.
//      ControlCommand.h). Автопилот получает её и отдаёт итоговую
//      команду в тех же знаках, поэтому автопилот и стики
//      гарантированно крутят рули в одну сторону.
//   2. mix(): ControlCommand -> FlightOutputState (PWM на каждый
//      серво) с учётом реверса серво из Config.h.
//
// ЗАКРЫЛКИ (флапероны). Отдельных закрылков нет — их роль играют
// элероны: при выпуске оба опускаются на одинаковый угол (новая
// "нейтраль" элеронов, растёт подъёмная сила), а крен от стика и
// автопилота добавляется поверх неё, как обычно, в разные стороны.
// Отрицательное положение — воздушный тормоз (оба элерона вверх).
// Цель задаёт FlightController по привязкам, микшер ведёт к ней
// плавно (FlapsController.h).
// ============================================================

class ControlMixer
{
public:

    ControlCommand fromSticks(const RcChannelState& rc) const
    {
        ControlCommand command;
        command.roll = RcInput::centered(rc.get(Channels::AILERON), Config::AILERON_MAX_US, false);
        command.pitch = RcInput::centered(rc.get(Channels::ELEVATOR), Config::ELEVATOR_MAX_US, true);
        command.yaw = RcInput::centered(rc.get(Channels::RUDDER), Config::RUDDER_MAX_US, false);
        return command;
    }

    // Закрылки к цели targetUs (мкс вниз; < 0 — тормоз), плавно.
    // nowMs передаётся снаружи, чтобы микшер не зависел от часов.
    int16_t updateFlaps(float targetUs, uint32_t nowMs)
    {
        return flaps.update(targetUs, nowMs);
    }

    FlightOutputState mix(const ControlCommand& command) const
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

    // Текущее (плавно меняющееся) положение закрылков, мкс.
    int16_t getFlaps() const
    {
        return flaps.getPosition();
    }


private:

    FlapsController flaps;

    static uint16_t toPwm(int32_t deflection, bool reversed)
    {
        const int32_t pwm = Config::PWM_CENTER + (reversed ? -deflection : deflection);
        return static_cast<uint16_t>(constrain(pwm, Config::PWM_MIN, Config::PWM_MAX));
    }
};
