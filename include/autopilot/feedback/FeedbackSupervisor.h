#pragma once
#include <Arduino.h>

#include "autopilot/feedback/AdaptiveRateController.h"
#include "autopilot/feedback/AirborneDetector.h"
#include "autopilot/feedback/ControlEffectivenessEstimator.h"
#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FeedbackMath.h"
#include "autopilot/feedback/FeedbackOutput.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/LandingSequencer.h"
#include "autopilot/feedback/PhaseTargets.h"
#include "autopilot/feedback/SpeedEstimator.h"
#include "autopilot/feedback/StallGuard.h"
#include "autopilot/feedback/TakeoffSequencer.h"

// ============================================================
// FEEDBACK SUPERVISOR — контур обратной связи целиком
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА. Прототипа для лётных тестов
// пока нет, поэтому ни FlightController, ни Autopilot, ни main.cpp
// эти файлы не включают — прошивка собирается и работает без них.
//
// Зачем. Сейчас Autopilot — ПИД по углу с коэффициентами под одну
// скорость: он отклоняет руль "по формуле" и не проверяет, что из
// этого вышло на самом самолёте. Обратная связь замыкает контур на
// реальный самолёт с его реальными скоростью и высотой:
//   • руль отклонили, а самолёт вращается медленнее нужного —
//     добавить ещё, пока не довернёт (AdaptiveRateController);
//   • сколько руля нужно, не константа: на малой скорости и на
//     высоте руль слабее. Это измеряется прямо в полёте
//     (ControlEffectivenessEstimator);
//   • знаки осей и установка датчиков определяются на земле
//     (калибровка установки IMU, предполётная проверка, проверка
//     рулей пилотом). В полёте оси не выключаются и не
//     переворачиваются: отрицательная оценка b — только
//     предупреждение в логе, в регулятор она не идёт;
//   • нос выровняли, а скорость падает — газ, нос вниз
//     (StallGuard);
//   • взлёт и посадка — этапами по датчикам (Takeoff/LandingSequencer).
//
// Порядок за такт (update):
//   1. скорость и ускорение, в воздухе ли;
//   2. обучение эффективности рулей по каждой оси;
//   3. защита от сваливания;
//   4. взлёт/посадка — цели по углам и газ;
//   5. цели ← ограничения защиты от сваливания (у неё приоритет);
//   6. регуляторы осей → отклонения рулей; газ.
//
// Приоритеты: не заармлен > защита от сваливания > взлёт/посадка >
// цели режима автопилота (FlightSnapshot.target*). При потере связи
// взлёт и посадка отменяются, газ не трогается (failsafe выключил
// мотор), а стабилизация выполняет цели планирования failsafe.
//
// ПЛАН ПОДКЛЮЧЕНИЯ (когда будет прототип):
//   1. FlightController::update() после чтения датчиков и расчёта
//      команд заполняет FlightSnapshot и вызывает update(). Сначала —
//      "теневой режим": выход только в лог (printStatus в DebugConsole,
//      строка на веб-странице), на рули не идёт. Так на реальном
//      самолёте проверяются оценки и знаки без риска: в полёте на
//      ручном управлении оценка b каждой оси должна быть
//      положительной и расти со скоростью.
//   2. На земле: самолёт в руках, STABILIZE — наклонить, рули должны
//      парировать, как сейчас с ПИД.
//   3. По одной оси: deflectionUs вместо коррекции ПИД из
//      Autopilot::getRollCorrection()/getPitchCorrection() — сначала
//      только крен.
//   4. Газ: throttleOverridePercent/throttleFloorPercent применять
//      после Autopilot::applyThrottle(), до failsafe (failsafe
//      главнее всего).
//   5. Взлёт/посадка — на свободный тумблер: requestTakeoff(),
//      requestLanding(), cancelPhase(). Режим AUTO_TAKEOFF из
//      Autopilot — убрать.
//   6. Константы FeedbackConfig — в config/Config.h.
//
// Проверка без подключения — замкнутая симуляция на плате:
//   pio test -e esp32-s3 -f test_feedback
// ============================================================

class FeedbackSupervisor
{
public:

    FeedbackSupervisor()
        : estimators{ ControlEffectivenessEstimator(FeedbackConfig::AXIS_ROLL),
                      ControlEffectivenessEstimator(FeedbackConfig::AXIS_PITCH),
                      ControlEffectivenessEstimator(FeedbackConfig::AXIS_YAW) },
          controllers{ AdaptiveRateController(FeedbackConfig::AXIS_ROLL),
                       AdaptiveRateController(FeedbackConfig::AXIS_PITCH),
                       AdaptiveRateController(FeedbackConfig::AXIS_YAW) }
    {
        resetFlight();
    }

    // --- Этапы полёта (будущий тумблер) ---

    // Взлёт: заармлен, связь есть, стоим на земле.
    bool requestTakeoff()
    {
        if (!last.armed || last.linkLost || airborne.isAirborne()) return false;
        landing.reset();
        takeoff.request(nowMs);
        return true;
    }

    // Посадка: в воздухе, связь есть.
    bool requestLanding()
    {
        if (!last.armed || last.linkLost || !airborne.isAirborne()) return false;
        takeoff.cancel();
        landing.request(nowMs);
        return true;
    }

    void cancelPhase()
    {
        takeoff.cancel();
        landing.cancel();
    }

    // --- Такт ---

    const FeedbackOutput& update(const FlightSnapshot& s)
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

    const FeedbackOutput& getOutput() const { return output; }

    // --- Состояние (лог, тесты) ---

    bool isAirborne() const { return airborne.isAirborne(); }
    const SpeedEstimator& getSpeedEstimator() const { return speed; }
    const ControlEffectivenessEstimator& getEstimator(uint8_t axis) const { return estimators[axis]; }
    const AdaptiveRateController& getController(uint8_t axis) const { return controllers[axis]; }
    const StallGuard& getStallGuard() const { return stall; }
    const TakeoffSequencer& getTakeoff() const { return takeoff; }
    const LandingSequencer& getLanding() const { return landing; }

    void printStatus(Print& out) const
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


private:

    static const char* axisName(uint8_t axis)
    {
        switch (axis)
        {
            case FeedbackConfig::AXIS_ROLL:  return "ROLL";
            case FeedbackConfig::AXIS_PITCH: return "PITCH";
            default:                         return "YAW";
        }
    }

    SpeedEstimator speed;
    AirborneDetector airborne;
    ControlEffectivenessEstimator estimators[FeedbackConfig::AXIS_COUNT];
    AdaptiveRateController controllers[FeedbackConfig::AXIS_COUNT];
    StallGuard stall;
    TakeoffSequencer takeoff;
    LandingSequencer landing;

    FeedbackOutput output;
    FlightSnapshot last;
    uint32_t nowMs = 0;
    uint32_t lastTimeUs = 0;
    bool timeInitialized = false;
    bool wasArmed = false;
    bool wasAirborne = false;
    char reasonBuffer[128] = {};   // кириллица в UTF-8 — 2 байта на букву

    // Новый полёт (арминг/дизарм): всё, что выучено, — с нуля.
    void resetFlight()
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

    float timeStep(uint32_t timeUs)
    {
        const float dt = timeInitialized ? (timeUs - lastTimeUs) / 1000000.0f : 0.0f;
        lastTimeUs = timeUs;
        timeInitialized = true;
        return (dt > 0.0f && dt < 0.1f) ? dt : 0.0f;
    }

    void disableAllAxes()
    {
        for (uint8_t axis = 0; axis < FeedbackConfig::AXIS_COUNT; ++axis)
        {
            output.axisEnabled[axis] = false;
        }
    }

    void updateAirborne(const FlightSnapshot& s)
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

    // Желаемые угловые скорости осей.
    void desiredRates(const FlightSnapshot& s, float targetRoll, float targetPitch,
                      const PhaseTargets& phase, bool inAir, float desiredRate[]) const
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
            const float bank = constrain(s.rollDeg, -60.0f, 60.0f) * DEG_TO_RAD;
            const float turnRate = GRAVITY / speed.getSpeed() * RAD_TO_DEG;
            desiredRate[AXIS_PITCH] += turnRate * sinf(bank) * tanf(bank);
            desiredRate[AXIS_YAW] = turnRate * sinf(bank);
        }

        // Удержание курса на разбеге/пробеге — рулём направления и колесом.
        if (phase.active && phase.holdHeading)
        {
            desiredRate[AXIS_YAW] = HEADING_HOLD_GAIN * FeedbackMath::wrap180(phase.headingDeg - s.yawDeg);
        }
    }

    // Априорная эффективность на текущей скорости. На земле — без
    // поправки на V²: там курс держит в основном колесо, и на малой
    // скорости поправка загнала бы руль в упор от любой ошибки.
    float expectedEffectiveness(uint8_t axis) const
    {
        const float scale = airborne.isAirborne() ? speed.effectivenessScale() : 1.0f;
        return FeedbackConfig::EFFECTIVENESS_PRIOR[axis] * scale;
    }

    // Модель оси для регулятора: изученная, если ей можно верить и ось
    // работает в правильную сторону; иначе — априорная на текущей
    // скорости. Отрицательная оценка в регулятор не идёт никогда —
    // только предупреждение в describe().
    AxisModel axisModel(uint8_t axis) const
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

    // Короткое описание для лога и OLED, по приоритету.
    const char* describe(const PhaseTargets& phase, bool stabilizing)
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
};
