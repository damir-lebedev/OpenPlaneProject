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

    void reset(uint32_t nowMs)
    {
        state = State::GLIDE;
        stateSinceMs = nowMs;
        liftSinceMs = 0;
        averageClimb = 0;
    }

    // climbMs — вариометр (м/с); distanceHomeM < 0 — дом неизвестен.
    void update(float climbMs, float altitudeM, float distanceHomeM, float dtS, uint32_t nowMs)
    {
        if (state != State::MOTOR_CLIMB && altitudeM < Config::SOAR_MIN_ALTITUDE_M)
        {
            enter(State::MOTOR_CLIMB, nowMs, "низко — набор с мотором");
            return;
        }

        switch (state)
        {
            case State::MOTOR_CLIMB:
                if (altitudeM >= Config::SOAR_MAX_ALTITUDE_M)
                {
                    enter(State::GLIDE, nowMs, "высота набрана — планирую");
                }
                break;

            case State::RETURN:
                if (distanceHomeM >= 0 && distanceHomeM < Config::SOAR_MAX_DISTANCE_M * 0.7f)
                {
                    enter(State::GLIDE, nowMs, "снова рядом с домом");
                }
                break;

            case State::GLIDE:
                if (distanceHomeM > Config::SOAR_MAX_DISTANCE_M)
                {
                    enter(State::RETURN, nowMs, "далеко — к дому");
                    break;
                }
                if (climbMs > Config::SOAR_THERMAL_CLIMB_MS)
                {
                    if (liftSinceMs == 0) liftSinceMs = nowMs == 0 ? 1 : nowMs;
                    if (nowMs - liftSinceMs >= Config::SOAR_THERMAL_CONFIRM_MS)
                    {
                        averageClimb = climbMs;
                        enter(State::THERMAL, nowMs, "термик! круги");
                    }
                }
                else
                {
                    liftSinceMs = 0;
                }
                break;

            case State::THERMAL:
            {
                const float tau = Config::SOAR_EXIT_WINDOW_MS / 3000.0f;
                averageClimb += dtS / (tau + dtS) * (climbMs - averageClimb);
                if (nowMs - stateSinceMs >= Config::SOAR_EXIT_WINDOW_MS &&
                    averageClimb < Config::SOAR_EXIT_CLIMB_MS)
                {
                    enter(State::GLIDE, nowMs, "термик кончился — планирую дальше");
                }
                break;
            }
        }
    }

    State getState() const { return state; }
    bool motorOn() const { return state == State::MOTOR_CLIMB; }
    float getAverageClimb() const { return averageClimb; }

    static const char* stateName(State s)
    {
        switch (s)
        {
            case State::GLIDE:       return "GLIDE";
            case State::THERMAL:     return "THERMAL";
            case State::MOTOR_CLIMB: return "MOTOR_CLIMB";
            case State::RETURN:      return "RETURN";
            default:                 return "?";
        }
    }


private:

    State state = State::GLIDE;
    uint32_t stateSinceMs = 0;
    uint32_t liftSinceMs = 0;
    float averageClimb = 0;

    void enter(State next, uint32_t nowMs, const char* why)
    {
        state = next;
        stateSinceMs = nowMs;
        liftSinceMs = 0;
        Serial.print("Soaring: ");
        Serial.println(why);
    }
};
