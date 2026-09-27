#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/PilotSwitches.h"
#include "config/Config.h"
#include "control/ArmingManager.h"
#include "control/Beeper.h"
#include "control/ControlCommand.h"
#include "control/ControlMixer.h"
#include "control/FlightOutputState.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"
#include "sensors/SensorInterface.h"
#include "rc/RcChannelState.h"

// ============================================================
// FLIGHT CONTROLLER
//
// Единственный координатор всего цикла управления. Сам не
// парсит UART, не трогает Servo и не считает mixer/throttle —
// только вызывает остальные классы в правильном порядке.
//
// Порядок в update() и есть приоритет управления:
//   1. ТУМБЛЕРЫ   — PilotSwitches: режим, функции, крутилки по
//                   таблице config/Controls.h (при связи)
//   2. СТИКИ      — стики -> команда крена/тангажа/рысканья
//                   (× чувствительность Knob::RATES), закрылки/тормоз
//                   плавно к цели по привязкам
//   3. АВТОПИЛОТ  — Autopilot::update() читает датчики всегда, даже
//                   без связи, и считает итоговую команду и газ режима
//   4. FAILSAFE   — потеря связи важнее всего: в воздухе (armed)
//                   автопилот возвращает домой или планирует (мотор
//                   выключен), на земле — рули в нейтраль; дальше цикл
//                   не идёт
//   5. ARMING     — тумблер ARM (SwA)
//   6. ВЫХОДЫ     — команда -> PWM каждого серво; газ -> 0, если не
//                   armed или включён Feature::MOTOR_KILL; AUX1 —
//                   груз, AUX2 — камера; пищалка
// ============================================================

class FlightController
{
public:

    // Autopilot/PilotSwitches опциональны (nullptr = чистое ручное
    // управление: рули = стики, закрылки не выпускаются).
    FlightController(
        IBusReceiver& rcReceiver,
        ControlMixer& controlMixer,
        ThrottleManager& throttleManager,
        ArmingManager& armingManager,
        FlightOutputs& flightOutputs,
        Autopilot* ap = nullptr,
        PilotSwitches* pilotSwitches = nullptr
    );

    void begin();

    void update();


    // Для DebugLogger/WebDebugServer/OledDisplay.
    bool isReceiverFailsafe() const { return receiver.isSignalLost(); }
    const IBusReceiver& getReceiver() const { return receiver; }
    bool isArmed() const { return arming.isArmed(); }
    const ArmingManager& getArming() const { return arming; }
    const FlightOutputState& getOutputState() const { return outputs.getLastState(); }
    const RcChannelState& getRcState() const { return receiver.getState(); }
    const FlightOutputs& getOutputs() const { return outputs; }
    int16_t getFlapsUs() const { return mixer.getFlaps(); }
    const PilotSwitches* getSwitches() const { return switches; }
    const PilotInputs& getInputs() const { return inputs(); }
    bool isLostModelBeeping() const { return beeper.isLostModel(); }


private:

    IBusReceiver& receiver;
    ControlMixer& mixer;
    ThrottleManager& throttle;
    ArmingManager& arming;
    FlightOutputs& outputs;

    Autopilot* autopilot;
    PilotSwitches* switches;

    Beeper beeper;
    PilotInputs noInputs;

    const PilotInputs& inputs() const;

    // Чувствительность стиков (Knob::RATES), если крутилка привязана.
    static void applyRates(ControlCommand& sticks, const PilotInputs& in);

    // Куда ехать закрылкам: тормоз важнее закрылков, тумблер — крутилки.
    static float flapsTarget(const PilotInputs& in);

    // AUX1 — груз (закрыт, пока функция не включена), AUX2 — камера:
    // угол с крутилки, со стабилизацией — минус тангаж самолёта.
    void applyAuxOutputs(FlightOutputState& output, const PilotInputs& in) const;

    // Потеря связи. В воздухе (armed) автопилот ведёт самолёт сам:
    // домой с мотором или планирование с выключенным мотором (см.
    // Autopilot::setFailsafe()); закрылки плавно убираются, груз и
    // камера — как были. На земле или без автопилота — нейтраль.
    void applyLinkLoss(int16_t flapsUs, const PilotInputs& in);
};
