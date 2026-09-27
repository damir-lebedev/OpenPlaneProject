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

    ControlCommand fromSticks(const RcChannelState& rc) const;

    // Закрылки к цели targetUs (мкс вниз; < 0 — тормоз), плавно.
    // nowMs передаётся снаружи, чтобы микшер не зависел от часов.
    int16_t updateFlaps(float targetUs, uint32_t nowMs);

    FlightOutputState mix(const ControlCommand& command) const;

    // Текущее (плавно меняющееся) положение закрылков, мкс.
    int16_t getFlaps() const;


private:

    FlapsController flaps;

    static uint16_t toPwm(int32_t deflection, bool reversed);
};
