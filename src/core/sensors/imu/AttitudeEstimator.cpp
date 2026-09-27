// Реализация sensors/imu/AttitudeEstimator.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/imu/AttitudeEstimator.h"


auto AttitudeEstimator::reset() -> void
{
    initialized = false;
}

auto AttitudeEstimator::setYaw(float yawDegrees) -> void
{
    yaw = wrap180(yawDegrees);
}

auto AttitudeEstimator::update(float ax, float ay, float az,
                float rollRate, float pitchRate, float yawRate,
                uint32_t nowUs) -> void
{
    // Нос вверх -> проекция "верха" на X положительна -> pitch > 0.
    // Правое крыло вниз -> проекция "верха" на Y (влево) > 0 -> roll > 0.
    const float accelRoll = atan2f(ay, az) * RAD_TO_DEG_F;
    const float accelPitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

    const float dt = (nowUs - lastUpdateUs) / 1000000.0f;
    lastUpdateUs = nowUs;

    if (!initialized)
    {
        roll = accelRoll;
        pitch = accelPitch;
        initialized = true;
        return;
    }

    // Пауза (калибровка, зависание шины) — не интегрируем скачок dt.
    if (dt <= 0.0f || dt > MAX_DT_S)
    {
        return;
    }

    roll = wrap180(ALPHA * (roll + rollRate * dt) + (1.0f - ALPHA) * accelRoll);
    pitch = wrap180(ALPHA * (pitch + pitchRate * dt) + (1.0f - ALPHA) * accelPitch);
    yaw = wrap180(yaw + yawRate * dt);
}

auto AttitudeEstimator::wrap180(float angle) -> float
{
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}
