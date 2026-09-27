// Реализация autopilot/feedback/SpeedEstimator.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/SpeedEstimator.h"


auto SpeedEstimator::update(const FlightSnapshot& s) -> void
{
    const float dt = (s.timeUs - lastUs) / 1000000.0f;
    lastUs = s.timeUs;
    if (dt <= 0.0f || dt > 0.5f) return;

    // Продольное ускорение по IMU.
    if (s.imuValid)
    {
        const float kinematic =
            FeedbackConfig::GRAVITY * (s.accelXg - sinf(s.pitchDeg * static_cast<float>(DEG_TO_RAD)));
        const float alpha = dt / (FeedbackConfig::ACCEL_FILTER_TAU_S + dt);
        accelMs2 += alpha * (kinematic - accelMs2);
        accelValid = true;
    }
    else
    {
        accelValid = false;
    }

    // Скорость: лучший доступный источник.
    if (s.airspeedValid)
    {
        speedMs = s.airspeedMs;
        source = Source::Airspeed;
    }
    else if (s.gpsValid)
    {
        speedMs = s.groundSpeedMs;
        source = Source::Gps;
    }
    else
    {
        source = Source::None;
    }
}

auto SpeedEstimator::effectivenessScale() const -> float
{
    if (!hasSpeed()) return 1.0f;
    const float ratio = speedMs / FeedbackConfig::REFERENCE_SPEED_MS;
    return constrain(ratio * ratio, 0.05f, 4.0f);
}
