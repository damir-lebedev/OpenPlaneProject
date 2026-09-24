#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/SpeedEstimator.h"

// ============================================================
// AIRBORNE DETECTOR — самолёт в воздухе или на земле
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// От этого зависит почти всё: учить эффективность рулей, копить
// интеграл, искать сваливание и перепутанный знак имеет смысл только
// в полёте — на колёсах рули самолёт не вращают.
//
// В воздухе: высота над точкой включения (или над землёй по
// дальномеру) выше AIRBORNE_HEIGHT_M, или скорость выше скорости
// отрыва — дольше AIRBORNE_CONFIRM_MS.
// На земле: низко, не вращается и перегрузка ≈ 1g дольше
// GROUND_STILL_MS (самолёт стоит или катится ровно).
//
// Взлёт и посадка знают лучше — FeedbackSupervisor может сказать
// явно через force(): после отрыва — в воздухе, после касания — на
// земле.
// ============================================================

class AirborneDetector
{
public:

    void reset()
    {
        airborne = false;
        pending = false;
    }

    void force(bool isAirborne)
    {
        airborne = isAirborne;
        pending = false;
    }

    void update(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs)
    {
        if (!s.armed)
        {
            reset();
            return;
        }

        const bool candidate = airborne ? looksLanded(s) : looksAirborne(s, speed);
        if (!candidate)
        {
            pending = false;
            return;
        }
        if (!pending)
        {
            pending = true;
            pendingSinceMs = nowMs;
        }

        const uint32_t confirmMs = airborne ? FeedbackConfig::GROUND_STILL_MS
                                            : FeedbackConfig::AIRBORNE_CONFIRM_MS;
        if (nowMs - pendingSinceMs >= confirmMs)
        {
            airborne = !airborne;
            pending = false;
        }
    }

    bool isAirborne() const { return airborne; }


private:

    bool airborne = false;
    bool pending = false;
    uint32_t pendingSinceMs = 0;

    static bool looksAirborne(const FlightSnapshot& s, const SpeedEstimator& speed)
    {
        if (s.heightAglValid && s.heightAglM > FeedbackConfig::AIRBORNE_HEIGHT_M) return true;
        if (s.baroValid && s.altitudeM > FeedbackConfig::AIRBORNE_HEIGHT_M) return true;
        return speed.hasSpeed() && speed.getSpeed() > FeedbackConfig::ROTATE_SPEED_MS;
    }

    static bool looksLanded(const FlightSnapshot& s)
    {
        const bool low = s.heightAglValid ? s.heightAglM < FeedbackConfig::TOUCHDOWN_HEIGHT_M
                                          : (!s.baroValid || s.altitudeM < FeedbackConfig::AIRBORNE_HEIGHT_M);
        const bool still = fabsf(s.rollRateDps) < FeedbackConfig::TOUCHDOWN_STILL_RATE_DPS &&
                           fabsf(s.pitchRateDps) < FeedbackConfig::TOUCHDOWN_STILL_RATE_DPS &&
                           fabsf(s.yawRateDps) < FeedbackConfig::TOUCHDOWN_STILL_RATE_DPS;
        const float gLoad = sqrtf(s.accelXg * s.accelXg + s.accelYg * s.accelYg + s.accelZg * s.accelZg);
        const bool oneG = fabsf(gLoad - 1.0f) < FeedbackConfig::GROUND_ACCEL_TOLERANCE_G;
        return low && still && oneG;
    }
};
