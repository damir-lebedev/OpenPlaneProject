#pragma once
#include <Arduino.h>
#include <math.h>

#include "config/Config.h"

// ============================================================
// ВЫСОТА И СКОРОСТЬ ДЛЯ АВТОМАТИЧЕСКИХ РЕЖИМОВ
//
// Самолёт — не коптер: высоту быстро меняет руль высоты, а газ
// отвечает за энергию (скорость + набор). Поэтому:
//
//   тангаж: желаемая вертикальная скорость = NAV_ALT_GAIN · ошибка
//           высоты (не быстрее NAV_MAX_CLIMB/SINK), тангаж =
//           упреждение asin(Vy / V) + ПИ по ошибке вертикальной
//           скорости, в пределах NAV_MAX_DIVE ... NAV_MAX_CLIMB_PITCH;
//   газ:    с трубкой Пито — ПИ по воздушной скорости вокруг
//           CRUISE_THROTTLE_PCT; без неё — газ круиза (крутилка).
//           Плюс THROTTLE_PER_CLIMB_PCT на каждый м/с желаемого
//           набора: подъём требует энергии.
//
// Это упрощённый вариант TECS из ArduPilot: без полной энергетической
// модели, но с тем же разделением ролей руля высоты и газа.
// ============================================================

class AltitudeSpeedController
{
public:

    void reset()
    {
        climbIntegral = 0;
        speedIntegral = 0;
        lastWantedClimb = 0;
    }

    // Тангаж (°), чтобы держать targetAltitude. integrate = false —
    // интеграторы не копятся (не заармлен, на земле).
    float pitchFor(float targetAltitudeM, float altitudeM, float climbMs, float speedMs, float dtS, bool integrate)
    {
        lastWantedClimb = constrain((targetAltitudeM - altitudeM) * Config::NAV_ALT_GAIN,
                                    -Config::NAV_MAX_SINK_MS, Config::NAV_MAX_CLIMB_MS);

        const float error = lastWantedClimb - climbMs;
        if (integrate)
        {
            climbIntegral = constrain(climbIntegral + error * Config::NAV_CLIMB_KI_DEG * dtS,
                                      -CLIMB_INTEGRAL_LIMIT_DEG, CLIMB_INTEGRAL_LIMIT_DEG);
        }

        const float ratio = constrain(lastWantedClimb / max(speedMs, MIN_SPEED_MS), -1.0f, 1.0f);
        const float feedForward = asinf(ratio) * RAD_PER_DEG_INV;

        return constrain(feedForward + Config::NAV_CLIMB_KP_DEG * error + climbIntegral,
                         Config::NAV_MAX_DIVE_PITCH_DEG, Config::NAV_MAX_CLIMB_PITCH_DEG);
    }

    // Газ (%) для автоматических режимов. hasAirspeed — трубка Пито
    // жива; иначе cruisePct задаёт газ напрямую.
    float throttleFor(bool hasAirspeed, float airspeedMs, float targetAirspeedMs, float cruisePct,
                      float dtS, bool integrate)
    {
        float throttle;
        if (hasAirspeed)
        {
            const float error = targetAirspeedMs - airspeedMs;
            if (integrate)
            {
                speedIntegral = constrain(speedIntegral + error * Config::AIRSPEED_THROTTLE_KI * dtS,
                                          -SPEED_INTEGRAL_LIMIT_PCT, SPEED_INTEGRAL_LIMIT_PCT);
            }
            throttle = Config::CRUISE_THROTTLE_PCT + Config::AIRSPEED_THROTTLE_KP * error + speedIntegral;
        }
        else
        {
            throttle = cruisePct;
        }

        throttle += Config::THROTTLE_PER_CLIMB_PCT * lastWantedClimb;
        return constrain(throttle, Config::AUTO_THROTTLE_MIN_PCT, Config::AUTO_THROTTLE_MAX_PCT);
    }

    float getWantedClimb() const { return lastWantedClimb; }


private:

    static constexpr float MIN_SPEED_MS = 5.0f;
    static constexpr float CLIMB_INTEGRAL_LIMIT_DEG = 10.0f;
    static constexpr float SPEED_INTEGRAL_LIMIT_PCT = 30.0f;
    static constexpr float RAD_PER_DEG_INV = 57.2957795f;

    float climbIntegral = 0;
    float speedIntegral = 0;
    float lastWantedClimb = 0;
};
