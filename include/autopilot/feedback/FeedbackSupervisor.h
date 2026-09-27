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

    FeedbackSupervisor();

    // --- Этапы полёта (будущий тумблер) ---

    // Взлёт: заармлен, связь есть, стоим на земле.
    bool requestTakeoff();

    // Посадка: в воздухе, связь есть.
    bool requestLanding();

    void cancelPhase();

    // --- Такт ---

    const FeedbackOutput& update(const FlightSnapshot& s);

    const FeedbackOutput& getOutput() const { return output; }

    // --- Состояние (лог, тесты) ---

    bool isAirborne() const { return airborne.isAirborne(); }
    const SpeedEstimator& getSpeedEstimator() const { return speed; }
    const ControlEffectivenessEstimator& getEstimator(uint8_t axis) const { return estimators[axis]; }
    const AdaptiveRateController& getController(uint8_t axis) const { return controllers[axis]; }
    const StallGuard& getStallGuard() const { return stall; }
    const TakeoffSequencer& getTakeoff() const { return takeoff; }
    const LandingSequencer& getLanding() const { return landing; }

    void printStatus(Print& out) const;


private:

    static const char* axisName(uint8_t axis);

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
    void resetFlight();

    float timeStep(uint32_t timeUs);

    void disableAllAxes();

    void updateAirborne(const FlightSnapshot& s);

    // Желаемые угловые скорости осей.
    void desiredRates(const FlightSnapshot& s, float targetRoll, float targetPitch,
                      const PhaseTargets& phase, bool inAir, float desiredRate[]) const;

    // Априорная эффективность на текущей скорости. На земле — без
    // поправки на V²: там курс держит в основном колесо, и на малой
    // скорости поправка загнала бы руль в упор от любой ошибки.
    float expectedEffectiveness(uint8_t axis) const;

    // Модель оси для регулятора: изученная, если ей можно верить и ось
    // работает в правильную сторону; иначе — априорная на текущей
    // скорости. Отрицательная оценка в регулятор не идёт никогда —
    // только предупреждение в describe().
    AxisModel axisModel(uint8_t axis) const;

    // Короткое описание для лога и OLED, по приоритету.
    const char* describe(const PhaseTargets& phase, bool stabilizing);
};
