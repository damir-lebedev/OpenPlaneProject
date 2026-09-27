// Реализация autopilot/feedback/AdaptiveRateController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/AdaptiveRateController.h"


AdaptiveRateController::AdaptiveRateController(uint8_t axisIndex)
: axis(axisIndex) {}

auto AdaptiveRateController::reset() -> void
{
    integral = 0;
    saturatedDirection = 0;
    desiredRate = 0;
    output = 0;
}

auto AdaptiveRateController::angleToRate(float targetDeg, float angleDeg) const -> float
{
    const float error = FeedbackMath::wrap180(targetDeg - angleDeg);
    return FeedbackMath::clampAbs(FeedbackConfig::ANGLE_GAIN[axis] * error,
                                  FeedbackConfig::MAX_RATE_DPS[axis]);
}

auto AdaptiveRateController::update(float desiredRateDps, float rateDps, const AxisModel& model,
                 bool allowIntegral, float dt) -> float
{
    desiredRate = FeedbackMath::clampAbs(desiredRateDps, FeedbackConfig::MAX_RATE_DPS[axis]);
    const float rateError = desiredRate - rateDps;

    // Модуль b не меньше минимума: иначе на почти нулевой
    // эффективности руль улетел бы в бесконечность.
    const float bMin = FeedbackConfig::EFFECTIVENESS_MIN[axis];
    float b = model.effectiveness;
    if (fabsf(b) < bMin) b = (b < 0) ? -bMin : bMin;

    // Интеграл. Не копить в сторону, куда руль уже упёрся
    // (anti-windup): иначе после упора он долго "отматывается".
    if (allowIntegral && dt > 0)
    {
        const float step = FeedbackConfig::RATE_INTEGRAL_GAIN[axis] * rateError * dt;
        const int8_t pushDirection = static_cast<int8_t>(FeedbackMath::signOf(step) * FeedbackMath::signOf(b));
        if (saturatedDirection == 0 || pushDirection != saturatedDirection)
        {
            integral = FeedbackMath::clampAbs(integral + step, FeedbackConfig::MAX_RATE_DPS[axis]);
        }
    }

    const float desiredAccel = (rateError + integral) / FeedbackConfig::RATE_TAU_S[axis];
    const float deflection = (desiredAccel - model.damping * rateDps - model.bias) / b;

    const float limit = FeedbackConfig::MAX_DEFLECTION_US[axis];
    saturatedDirection = static_cast<int8_t>(deflection > limit ? 1 : (deflection < -limit ? -1 : 0));
    output = FeedbackMath::clampAbs(deflection, limit);
    return output;
}
