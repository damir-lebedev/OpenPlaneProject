// Реализация autopilot/feedback/AirborneDetector.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/AirborneDetector.h"


auto AirborneDetector::reset() -> void
{
    airborne = false;
    pending = false;
}

auto AirborneDetector::force(bool isAirborne) -> void
{
    airborne = isAirborne;
    pending = false;
}

auto AirborneDetector::update(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs) -> void
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

auto AirborneDetector::looksAirborne(const FlightSnapshot& s, const SpeedEstimator& speed) -> bool
{
    if (s.heightAglValid && s.heightAglM > FeedbackConfig::AIRBORNE_HEIGHT_M) return true;
    if (s.baroValid && s.altitudeM > FeedbackConfig::AIRBORNE_HEIGHT_M) return true;
    return speed.hasSpeed() && speed.getSpeed() > FeedbackConfig::ROTATE_SPEED_MS;
}

auto AirborneDetector::looksLanded(const FlightSnapshot& s) -> bool
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
