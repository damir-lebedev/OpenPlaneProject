// Реализация autopilot/feedback/FeedbackSupervisor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/FeedbackSupervisor.h"


FeedbackSupervisor::FeedbackSupervisor()
: estimators{ ControlEffectivenessEstimator(FeedbackConfig::AXIS_ROLL),
                  ControlEffectivenessEstimator(FeedbackConfig::AXIS_PITCH),
                  ControlEffectivenessEstimator(FeedbackConfig::AXIS_YAW) },
      controllers{ AdaptiveRateController(FeedbackConfig::AXIS_ROLL),
                   AdaptiveRateController(FeedbackConfig::AXIS_PITCH),
                   AdaptiveRateController(FeedbackConfig::AXIS_YAW) }
{
    resetFlight();
}

auto FeedbackSupervisor::requestTakeoff() -> bool
{
    if (!last.armed || last.linkLost || airborne.isAirborne()) return false;
    landing.reset();
    takeoff.request(nowMs);
    return true;
}

auto FeedbackSupervisor::requestLanding() -> bool
{
    if (!last.armed || last.linkLost || !airborne.isAirborne()) return false;
    takeoff.cancel();
    landing.request(nowMs);
    return true;
}

auto FeedbackSupervisor::cancelPhase() -> void
{
    takeoff.cancel();
    landing.cancel();
}

auto FeedbackSupervisor::update(const FlightSnapshot& s) -> const FeedbackOutput&
{
    const float dt = timeStep(s.timeUs);
    nowMs = s.timeUs / 1000;
    last = s;
    output = FeedbackOutput();

    if (!s.armed)
    {
        if (wasArmed) resetFlight();
        wasArmed = false;
        disableAllAxes();
        output.reason = "не заармлен";
        return output;
    }
    if (!wasArmed)
    {
        resetFlight();
        wasArmed = true;
    }

    if (s.linkLost) cancelPhase();

    // 1. Скорость, в воздухе ли.
    speed.update(s);
    updateAirborne(s);
    const bool inAir = airborne.isAirborne();

    // 2. Обучение: только в воздухе, не на сваливании (там модель
    //    неверна), не пока выпускаются закрылки (меняется
    //    балансировка — это не реакция на руль).
    const bool learningAllowed = inAir && s.imuValid && !s.flapsMoving &&
                                 stall.getLevel() != StallGuard::Level::Stall;
    const float commands[] = { s.commandRollUs, s.commandPitchUs, s.commandYawUs };
    const float rates[] = { s.rollRateDps, s.pitchRateDps, s.yawRateDps };
    for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
    {
        estimators[axis].update(commands[axis], rates[axis], speed.effectivenessScale(),
                                learningAllowed, nowMs);
    }

    // 3. Защита от сваливания. У самой земли (выравнивание, пробег)
    //    не работает: посадка — это и есть управляемое сваливание.
    StallGuard::ControlState control;
    control.pitchEffectivenessKnown = estimators[FeedbackConfig::AXIS_PITCH].isConfident();
    control.pitchEffectiveness = fabsf(estimators[FeedbackConfig::AXIS_PITCH].getEffectiveness());
    stall.update(s, speed, control, inAir && !landing.isNearGround(), nowMs);

    // 4. Взлёт / посадка.
    takeoff.update(s, speed, nowMs);
    landing.update(s, nowMs);
    const PhaseTargets phase = takeoff.isActive() ? takeoff.getTargets()
                             : landing.isActive() ? landing.getTargets()
                             : PhaseTargets();

    // 5. Цели и ограничения.
    float targetRoll = phase.active ? phase.targetRollDeg : s.targetRollDeg;
    float targetPitch = phase.active ? phase.targetPitchDeg : s.targetPitchDeg;
    targetPitch = min(targetPitch, stall.maxPitchDeg());
    targetRoll = FeedbackMath::clampAbs(targetRoll, stall.maxBankDeg());
    output.targetRollDeg = targetRoll;
    output.targetPitchDeg = targetPitch;

    const bool stabilizing = s.stabilizationActive || phase.active;
    const bool holdHeading = phase.active && phase.holdHeading;

    bool controlled[FeedbackConfig::AXIS_COUNT];
    controlled[FeedbackConfig::AXIS_ROLL] = stabilizing && (!phase.active || phase.controlRoll);
    controlled[FeedbackConfig::AXIS_PITCH] = stabilizing && (!phase.active || phase.controlPitch);
    controlled[FeedbackConfig::AXIS_YAW] = stabilizing && (holdHeading || (inAir && speed.hasSpeed()));

    float desiredRate[FeedbackConfig::AXIS_COUNT];
    desiredRates(s, targetRoll, targetPitch, phase, inAir, desiredRate);

    // 6. Рули.
    for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
    {
        if (!controlled[axis] || !s.imuValid)
        {
            controllers[axis].reset();
            output.axisEnabled[axis] = false;
            continue;
        }

        // Интеграл на земле копил бы то, что держит шасси, — кроме
        // курса на разбеге/пробеге: снос от винта и ветра колесо и
        // руль направления как раз должны "доправлять".
        const bool allowIntegral = inAir || (axis == FeedbackConfig::AXIS_YAW && holdHeading);
        float deflection = controllers[axis].update(desiredRate[axis], rates[axis],
                                                    axisModel(axis), allowIntegral, dt);
        if (axis == FeedbackConfig::AXIS_ROLL)
        {
            deflection = FeedbackMath::clampAbs(deflection, stall.maxAileronUs());
        }
        output.deflectionUs[axis] = deflection;
    }

    // Газ: этап полёта задаёт, защита от сваливания не даёт
    // опуститься ниже своего минимума. Без связи не трогаем.
    if (!s.linkLost)
    {
        if (phase.active && phase.throttlePercent >= 0)
        {
            output.throttleOverridePercent = phase.throttlePercent;
        }
        const float throttleFloor = stall.throttleFloorPercent(s.linkLost);
        if (throttleFloor >= 0)
        {
            output.throttleFloorPercent = throttleFloor;
            if (output.throttleOverridePercent >= 0)
            {
                output.throttleOverridePercent = max(output.throttleOverridePercent, throttleFloor);
            }
        }
    }

    output.reason = describe(phase, stabilizing);
    return output;
}

auto FeedbackSupervisor::printStatus(Print& out) const -> void
{
    out.printf("FEEDBACK: %s | air=%d speed=", output.reason, airborne.isAirborne());
    if (speed.hasSpeed()) out.printf("%.1f", speed.getSpeed());
    else out.print("?");
    out.printf(" accel=%.1f stall=%s takeoff=%s landing=%s\n",
               speed.getAcceleration(), stall.getLevelName(),
               takeoff.getStateName(), landing.getStateName());

    for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
    {
        const ControlEffectivenessEstimator& e = estimators[axis];
        out.printf("  %-5s b=%6.2f±%.2f%s a=%6.1f c=%6.0f I=%6.1f out=%5.0f%s\n",
                   axisName(axis), e.getEffectiveness(), e.getEffectivenessSigma(),
                   e.isConfident() ? "*" : " ", e.getDamping(), e.getBias(),
                   controllers[axis].getIntegral(), output.deflectionUs[axis],
                   output.axisEnabled[axis] ? "" : " (off)");
    }
}

auto FeedbackSupervisor::axisName(uint8_t axis) -> const char*
{
    switch (axis)
    {
        case FeedbackConfig::AXIS_ROLL:  return "ROLL";
        case FeedbackConfig::AXIS_PITCH: return "PITCH";
        default:                         return "YAW";
    }
}

auto FeedbackSupervisor::resetFlight() -> void
{
    airborne.reset();
    wasAirborne = false;
    for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
    {
        estimators[axis].reset();
        controllers[axis].reset();
    }
    stall.reset();
    takeoff.reset();
    landing.reset();
}

auto FeedbackSupervisor::timeStep(uint32_t timeUs) -> float
{
    const float dt = timeInitialized ? (timeUs - lastTimeUs) / 1000000.0f : 0.0f;
    lastTimeUs = timeUs;
    timeInitialized = true;
    return (dt > 0.0f && dt < 0.1f) ? dt : 0.0f;
}

auto FeedbackSupervisor::disableAllAxes() -> void
{
    for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
    {
        output.axisEnabled[axis] = false;
    }
}

auto FeedbackSupervisor::updateAirborne(const FlightSnapshot& s) -> void
{
    airborne.update(s, speed, nowMs);

    // Взлёт и посадка знают точнее эвристики.
    if (takeoff.getState() == TakeoffSequencer::State::Climb && !airborne.isAirborne())
    {
        airborne.force(true);
    }
    if (landing.isOnGround() && airborne.isAirborne())
    {
        airborne.force(false);
    }

    // Только что оторвались — то, что оценка "видела" на земле, не
    // годится (там рули самолёт не вращали), начинаем с априорной.
    const bool inAir = airborne.isAirborne();
    if (inAir && !wasAirborne)
    {
        for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
        {
            estimators[axis].reset();
            controllers[axis].reset();
        }
    }
    wasAirborne = inAir;
}

auto FeedbackSupervisor::desiredRates(const FlightSnapshot& s, float targetRoll, float targetPitch,
                      const PhaseTargets& phase, bool inAir, float desiredRate[]) const -> void
{
    using namespace FeedbackConfig;

    desiredRate[AXIS_ROLL] = controllers[AXIS_ROLL].angleToRate(targetRoll, s.rollDeg);
    desiredRate[AXIS_PITCH] = controllers[AXIS_PITCH].angleToRate(targetPitch, s.pitchDeg);
    desiredRate[AXIS_YAW] = 0;

    // Координированный разворот: в крене самолёт поворачивает с
    // угловой скоростью g·tg(крен)/V; в связанных осях это даёт
    // рысканье g·sin(крен)/V и тангаж g·sin(крен)·tg(крен)/V. Без
    // этого регулятор тангажа "не понимает", почему в вираже нос
    // приходится всё время тянуть.
    if (inAir && speed.hasSpeed() && speed.getSpeed() > 1.0f)
    {
        const float bank = constrain(s.rollDeg, -60.0f, 60.0f) * static_cast<float>(DEG_TO_RAD);
        const float turnRate = GRAVITY / speed.getSpeed() * static_cast<float>(RAD_TO_DEG);
        desiredRate[AXIS_PITCH] += turnRate * sinf(bank) * tanf(bank);
        desiredRate[AXIS_YAW] = turnRate * sinf(bank);
    }

    // Удержание курса на разбеге/пробеге — рулём направления и колесом.
    if (phase.active && phase.holdHeading)
    {
        desiredRate[AXIS_YAW] = HEADING_HOLD_GAIN * FeedbackMath::wrap180(phase.headingDeg - s.yawDeg);
    }
}

auto FeedbackSupervisor::expectedEffectiveness(uint8_t axis) const -> float
{
    const float scale = airborne.isAirborne() ? speed.effectivenessScale() : 1.0f;
    return FeedbackConfig::EFFECTIVENESS_PRIOR[axis] * scale;
}

auto FeedbackSupervisor::axisModel(uint8_t axis) const -> AxisModel
{
    const ControlEffectivenessEstimator& e = estimators[axis];

    AxisModel model;
    if (e.isConfident() && e.getEffectiveness() > 0)
    {
        model.effectiveness = e.getEffectiveness();
        model.damping = FeedbackConfig::DAMPING_COMPENSATION * e.getDamping();
        model.bias = e.getBias();
    }
    else
    {
        model.effectiveness = expectedEffectiveness(axis);
    }
    return model;
}

auto FeedbackSupervisor::describe(const PhaseTargets& phase, bool stabilizing) -> const char*
{
    if (stall.getLevel() == StallGuard::Level::Stall)
    {
        snprintf(reasonBuffer, sizeof(reasonBuffer), "СВАЛИВАНИЕ: %s", stall.getReason());
        return reasonBuffer;
    }
    if (stall.getLevel() == StallGuard::Level::LowEnergy)
    {
        snprintf(reasonBuffer, sizeof(reasonBuffer), "мало энергии: %s", stall.getReason());
        return reasonBuffer;
    }
    if (phase.active) return phase.reason;
    for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
    {
        // Только сообщение: ось продолжает работать по априорной
        // модели. Причина — проверить на земле (Config, установка IMU).
        const ControlEffectivenessEstimator& e = estimators[axis];
        if (e.isConfident() && e.getEffectiveness() < 0)
        {
            snprintf(reasonBuffer, sizeof(reasonBuffer),
                     "ВНИМАНИЕ: %s реагирует на руль наоборот? проверить на земле", axisName(axis));
            return reasonBuffer;
        }
    }
    return stabilizing ? "стабилизация" : "ручное (обучение)";
}
