#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// ПАРЕНИЕ (MODE_SOARING) — поиск и отработка термиков
//
// Мотор выключен, самолёт планирует. Термик (восходящий поток)
// виден по вариометру: самолёт без мотора вдруг перестаёт снижаться
// или набирает. Дальше — как делают планеристы: круги с постоянным
// креном, пока в круге в среднем набираем.
//
//   GLIDE ──набор > SOAR_THERMAL_CLIMB_MS дольше CONFIRM──► THERMAL
//   THERMAL ──средний набор < SOAR_EXIT_CLIMB_MS (не раньше чем через
//             SOAR_EXIT_WINDOW_MS)──► GLIDE
//   любое ──высота < SOAR_MIN_ALTITUDE_M──► MOTOR_CLIMB (с мотором до
//             SOAR_MAX_ALTITUDE_M) ──► GLIDE
//   GLIDE ──дальше SOAR_MAX_DISTANCE_M от дома──► RETURN (планирование
//             к дому) ──ближе 70% этого──► GLIDE
//
// Вариометр — с поправкой на полную энергию (если есть трубка Пито,
// см. Autopilot): иначе самолёт, задравший нос и теряющий скорость,
// "видит" набор и ищет термик там, где его нет.
// ============================================================

class SoaringController
{
public:

    enum class State : uint8_t { GLIDE, THERMAL, MOTOR_CLIMB, RETURN };

    void reset(uint32_t nowMs);

    // climbMs — вариометр (м/с); distanceHomeM < 0 — дом неизвестен.
    void update(float climbMs, float altitudeM, float distanceHomeM, float dtS, uint32_t nowMs);

    State getState() const { return state; }
    bool motorOn() const { return state == State::MOTOR_CLIMB; }
    float getAverageClimb() const { return averageClimb; }

    static const char* stateName(State s);


private:

    State state = State::GLIDE;
    uint32_t stateSinceMs = 0;
    uint32_t liftSinceMs = 0;
    float averageClimb = 0;

    void enter(State next, uint32_t nowMs, const char* why);
};
