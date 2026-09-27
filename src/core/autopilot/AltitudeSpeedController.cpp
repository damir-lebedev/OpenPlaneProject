// Реализация autopilot/AltitudeSpeedController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/AltitudeSpeedController.h"


auto AltitudeSpeedController::reset() -> void
{
    climbIntegral = 0;
    speedIntegral = 0;
    lastWantedClimb = 0;
}

auto AltitudeSpeedController::pitchFor(float targetAltitudeM, float altitudeM, float climbMs, float speedMs, float dtS, bool integrate) -> float
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

auto AltitudeSpeedController::throttleFor(bool hasAirspeed, float airspeedMs, float targetAirspeedMs, float cruisePct,
                      float dtS, bool integrate) -> float
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
