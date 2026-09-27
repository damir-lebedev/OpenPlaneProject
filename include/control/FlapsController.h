#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// FLAPS CONTROLLER
//
// Плавный выпуск/уборка закрылков: положение идёт к цели (мкс; 0 —
// убраны, Config::FLAPS_DEPLOYED_US — выпущены, отрицательное —
// воздушный тормоз, элероны вверх) не быстрее, чем ход
// FLAPS_DEPLOYED_US за Config::FLAPS_TRANSITION_MS. Резкий выпуск
// закрылков даёт клевок по тангажу, поэтому тумблер на пульте не
// переводится в ступеньку на сервах. Цель задают привязки
// (config/Controls.h): Feature::FLAPS, Knob::FLAPS, Feature::AIRBRAKE.
//
// Время передаётся снаружи (nowMs) — класс не зависит от
// системных часов и проверяется без железа.
// ============================================================

class FlapsController
{
public:

    // Возвращает текущее положение закрылков, мкс отклонения вниз.
    int16_t update(float target, uint32_t nowMs);

    int16_t getPosition() const;


private:

    static constexpr uint32_t MAX_STEP_MS = 20;

    float positionUs = 0;
    uint32_t lastUpdateMs = 0;
    bool initialized = false;
};
