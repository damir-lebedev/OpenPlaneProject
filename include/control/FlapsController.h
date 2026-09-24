#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// FLAPS CONTROLLER
//
// Плавный выпуск/уборка закрылков: положение идёт к цели (0 или
// Config::FLAPS_DEPLOYED_US) не быстрее, чем полный ход за
// Config::FLAPS_TRANSITION_MS. Резкий выпуск закрылков даёт клевок
// по тангажу, поэтому переключатель на пульте не переводится в
// ступеньку на сервах.
//
// Время передаётся снаружи (nowMs) — класс не зависит от
// системных часов и проверяется без железа.
// ============================================================

class FlapsController
{
public:

    // Возвращает текущее положение закрылков, мкс отклонения вниз.
    int16_t update(bool deployed, uint32_t nowMs)
    {
        const float target = deployed ? Config::FLAPS_DEPLOYED_US : 0;

        // Первый вызов (включение платы) — сразу в целевое положение,
        // без "выезда" закрылков на столе.
        if (!initialized)
        {
            positionUs = target;
            lastUpdateMs = nowMs;
            initialized = true;
            return getPosition();
        }

        // Шаг по времени ограничен: если update() долго не вызывался
        // (failsafe, калибровка из консоли), закрылки не должны
        // прыгнуть к цели за один цикл.
        const uint32_t elapsedMs = min<uint32_t>(nowMs - lastUpdateMs, MAX_STEP_MS);
        lastUpdateMs = nowMs;

        const float maxStep =
            static_cast<float>(Config::FLAPS_DEPLOYED_US) * elapsedMs / Config::FLAPS_TRANSITION_MS;

        positionUs += constrain(target - positionUs, -maxStep, maxStep);

        return getPosition();
    }

    int16_t getPosition() const
    {
        return static_cast<int16_t>(positionUs);
    }


private:

    static constexpr uint32_t MAX_STEP_MS = 20;

    float positionUs = 0;
    uint32_t lastUpdateMs = 0;
    bool initialized = false;
};
