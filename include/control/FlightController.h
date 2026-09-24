#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotModeSelector.h"
#include "config/Config.h"
#include "control/ArmingManager.h"
#include "control/ControlCommand.h"
#include "control/ControlMixer.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"

// ============================================================
// FLIGHT CONTROLLER
//
// Единственный координатор всего цикла управления. Сам не
// парсит UART, не трогает Servo и не считает mixer/throttle —
// только вызывает остальные классы в правильном порядке.
//
// Порядок в update() и есть приоритет управления:
//   1. ДАТЧИКИ    — Autopilot::update() читает IMU/баро всегда, даже
//                   без связи, чтобы фильтры углов не застывали
//   2. FAILSAFE   — потеря связи важнее всего: мотор в ноль; в
//                   воздухе (armed) — планирование с ровными крыльями,
//                   на земле — рули в нейтраль; дальше цикл не идёт
//   3. ARMING     — тумблер ARM (SwA)
//   4. MIXER      — стики -> команда крена/тангажа/рысканья/закрылков, к ней
//                   прибавляются коррекции автопилота (в тех же
//                   физических знаках), затем -> PWM каждого серво
//   5. THROTTLE   — газ пилота -> газ режима автопилота
//                   (AUTO_TAKEOFF/ALT_HOLD) -> 0, если не armed
//   6. OUTPUTS    — PWM на GPIO
// ============================================================

class FlightController
{
public:

    // Autopilot/AutopilotModeSelector опциональны (nullptr = чистое
    // ручное управление, как раньше).
    FlightController(
        IBusReceiver& receiver,
        ControlMixer& mixer,
        ThrottleManager& throttle,
        ArmingManager& arming,
        FlightOutputs& outputs,
        Autopilot* autopilot = nullptr,
        AutopilotModeSelector* modeSelector = nullptr
    )
        : receiver(receiver),
          mixer(mixer),
          throttle(throttle),
          arming(arming),
          outputs(outputs),
          autopilot(autopilot),
          modeSelector(modeSelector)
    {
    }

    void begin()
    {
        outputs.setFailsafe();
        receiver.begin();
    }

    void update()
    {
        receiver.update();

        const bool receiverFailsafe = receiver.isSignalLost();
        const RcChannelState& rc = receiver.getState();

        const uint16_t pilotThrottle = throttle.update(rc, receiverFailsafe);

        // Режим с RC переключаем только при живой связи: в failsafe-кадре
        // CH7 содержит не положение тумблера, а значение failsafe.
        if (modeSelector && !receiverFailsafe)
        {
            modeSelector->update(rc);
        }

        if (autopilot)
        {
            autopilot->update(arming.isArmed(), receiverFailsafe, pilotThrottle);
        }

        if (receiverFailsafe)
        {
            applyLinkLoss();
            return;  // стики и режимы в этом цикле не участвуют
        }

        arming.update(rc, false);

        ControlCommand command = mixer.fromSticks(rc, millis());

        if (autopilot)
        {
            command.roll = clampCommand(command.roll + autopilot->getRollCorrection());
            command.pitch = clampCommand(command.pitch + autopilot->getPitchCorrection());
        }

        FlightOutputState output = mixer.mix(command);

        output.throttle = autopilot ? autopilot->applyThrottle(pilotThrottle) : pilotThrottle;

        // ARM реально блокирует газ (см. ArmingManager) — ставим ПОСЛЕ
        // автопилота, чтобы ни один режим не мог протащить газ мимо
        // этой проверки.
        if (!arming.isArmed())
        {
            output.throttle = Config::PWM_MIN;
        }

        outputs.write(output);
    }


    // Для DebugLogger/WebDebugServer/OledDisplay.
    bool isReceiverFailsafe() const { return receiver.isSignalLost(); }
    const IBusReceiver& getReceiver() const { return receiver; }
    bool isArmed() const { return arming.isArmed(); }
    const ArmingManager& getArming() const { return arming; }
    const FlightOutputState& getOutputState() const { return outputs.getLastState(); }
    const RcChannelState& getRcState() const { return receiver.getState(); }
    const FlightOutputs& getOutputs() const { return outputs; }
    int16_t getFlapsUs() const { return mixer.getFlaps(); }


private:

    IBusReceiver& receiver;
    ControlMixer& mixer;
    ThrottleManager& throttle;
    ArmingManager& arming;
    FlightOutputs& outputs;

    Autopilot* autopilot;
    AutopilotModeSelector* modeSelector;

    static int16_t clampCommand(float value)
    {
        return static_cast<int16_t>(constrain(value, -500.0f, 500.0f));
    }

    // Потеря связи. Мотор выключен всегда. В воздухе (armed) —
    // планирование: рули по коррекциям автопилота (крылья ровно, нос
    // чуть ниже горизонта, см. Autopilot::handleFailsafeGlide()),
    // закрылки убраны. На земле или без IMU — рули в нейтраль.
    void applyLinkLoss()
    {
        if (!autopilot || !autopilot->isFailsafeGliding())
        {
            outputs.setFailsafe();
            return;
        }

        ControlCommand glide;
        glide.roll = clampCommand(autopilot->getRollCorrection());
        glide.pitch = clampCommand(autopilot->getPitchCorrection());

        FlightOutputState output = mixer.mix(glide);
        output.throttle = Config::FAILSAFE_THROTTLE;
        outputs.write(output);
    }
};
