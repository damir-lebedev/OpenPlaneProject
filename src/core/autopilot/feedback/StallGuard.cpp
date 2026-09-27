// Реализация autopilot/feedback/StallGuard.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/StallGuard.h"


auto StallGuard::reset() -> void
{
    level = Level::Normal;
    decelerating = false;
    reason = "";
}

auto StallGuard::update(const FlightSnapshot& s, const SpeedEstimator& speed,
                const ControlState& control, bool airborne, uint32_t nowMs) -> void
{
    if (!airborne || !s.imuValid)
    {
        reset();
        return;
    }

    const bool lowEnergy = detectLowEnergy(s, speed, control, nowMs);
    const bool stall = detectStall(s, speed, lowEnergy);

    if (stall)
    {
        level = Level::Stall;
        lastStallMs = nowMs;
        lastLowEnergyMs = nowMs;
    }
    else if (level == Level::Stall && nowMs - lastStallMs < FeedbackConfig::RECOVERY_HOLD_MS)
    {
        // держим меры сваливания
    }
    else if (lowEnergy)
    {
        level = Level::LowEnergy;
        lastLowEnergyMs = nowMs;
    }
    else if (level != Level::Normal &&
             (nowMs - lastLowEnergyMs < FeedbackConfig::RECOVERY_HOLD_MS || !energyRecovered(speed)))
    {
        level = Level::LowEnergy;   // признаки ушли, но энергия ещё не вернулась
    }
    else
    {
        level = Level::Normal;
        reason = "";
    }
}

auto StallGuard::getLevelName() const -> const char*
{
    switch (level)
    {
        case Level::Normal:    return "OK";
        case Level::LowEnergy: return "LOW_ENERGY";
        case Level::Stall:     return "STALL";
    }
    return "?";
}

auto StallGuard::maxPitchDeg() const -> float
{
    if (level == Level::Stall) return FeedbackConfig::STALL_MAX_PITCH_DEG;
    if (level == Level::LowEnergy) return FeedbackConfig::LOW_ENERGY_MAX_PITCH_DEG;
    return 90.0f;
}

auto StallGuard::maxBankDeg() const -> float
{
    return level == Level::Stall ? FeedbackConfig::STALL_MAX_BANK_DEG : 180.0f;
}

auto StallGuard::maxAileronUs() const -> float
{
    return level == Level::Stall ? FeedbackConfig::STALL_AILERON_LIMIT_US
                                 : FeedbackConfig::MAX_DEFLECTION_US[FeedbackConfig::AXIS_ROLL];
}

auto StallGuard::throttleFloorPercent(bool linkLost) const -> float
{
    if (linkLost) return -1.0f;
    if (level == Level::Stall) return FeedbackConfig::STALL_THROTTLE_PERCENT;
    if (level == Level::LowEnergy) return FeedbackConfig::LOW_ENERGY_THROTTLE_PERCENT;
    return -1.0f;
}

auto StallGuard::detectLowEnergy(const FlightSnapshot& s, const SpeedEstimator& speed,
                         const ControlState& control, uint32_t nowMs) -> bool
{
    // Скорость быстро падает — подтверждаем, чтобы не реагировать
    // на тряску.
    const bool decelNow = speed.hasAcceleration() &&
                          speed.getAcceleration() < -FeedbackConfig::DECEL_WARN_MS2;
    if (decelNow && !decelerating) decelSinceMs = nowMs;
    decelerating = decelNow;
    const bool decelConfirmed = decelerating &&
                                nowMs - decelSinceMs >= FeedbackConfig::DECEL_CONFIRM_MS;

    if (decelConfirmed && s.pitchDeg > FeedbackConfig::LOW_ENERGY_PITCH_DEG)
    {
        reason = "скорость падает при поднятом носе";
        return true;
    }

    if (speed.hasSpeed() &&
        speed.getSpeed() < FeedbackConfig::STALL_SPEED_MS * FeedbackConfig::LOW_SPEED_MARGIN)
    {
        reason = "скорость близка к сваливанию";
        return true;
    }

    if (control.pitchEffectivenessKnown &&
        control.pitchEffectiveness < FeedbackConfig::LOW_EFFECTIVENESS_RATIO *
                                     FeedbackConfig::EFFECTIVENESS_PRIOR[FeedbackConfig::AXIS_PITCH])
    {
        reason = "руль высоты потерял эффективность";
        return true;
    }

    return false;
}

auto StallGuard::energyRecovered(const SpeedEstimator& speed) -> bool
{
    if (speed.hasSpeed())
    {
        return speed.getSpeed() >= FeedbackConfig::STALL_SPEED_MS * FeedbackConfig::LOW_SPEED_EXIT_MARGIN;
    }
    return !speed.hasAcceleration() || speed.getAcceleration() >= 0.0f;
}

auto StallGuard::detectStall(const FlightSnapshot& s, const SpeedEstimator& speed, bool lowEnergy) -> bool
{
    if (speed.hasSpeed() && speed.getSpeed() < FeedbackConfig::STALL_SPEED_MS)
    {
        reason = "скорость ниже сваливания";
        return true;
    }

    // Нос резко падает, хотя руль высоты тянет вверх.
    const bool elevatorUp = s.commandPitchUs > FeedbackConfig::STALL_NOSE_UP_COMMAND_US;
    if (elevatorUp && s.pitchRateDps < -FeedbackConfig::NOSE_DROP_RATE_DPS)
    {
        reason = "нос падает против руля высоты";
        return true;
    }

    // Крыло резко валится против элеронов — только при малой
    // энергии: на скорости это скорее порыв.
    if (lowEnergy && fabsf(s.rollRateDps) > FeedbackConfig::WING_DROP_RATE_DPS &&
        s.rollRateDps * s.commandRollUs < 0)
    {
        reason = "сваливание на крыло";
        return true;
    }

    return false;
}
