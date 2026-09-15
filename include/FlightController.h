#pragma once

// ============================================================
// FLIGHT CONTROLLER
//
// Единственный координатор всего цикла управления. Сам не
// парсит UART, не трогает Servo и не считает mixer/throttle —
// только вызывает остальные классы в правильном порядке.
//
// Порядок в update() и есть приоритет управления:
//   1. FAILSAFE   — потеря сигнала важнее всего, обрывает cycle
//   2. ARMING     — обновляем состояние ARM
//   3. MIXER      — RC -> положения поверхностей
//   4. AUTOPILOT  — коррекции поверх mixer (только если armed)
//   5. THROTTLE   — газ + boost
//   6. OUTPUTS    — PWM на GPIO
// ============================================================

class FlightController
{
public:

    // Autopilot/FeatureManager опциональны (nullptr = чистое
    // ручное управление, как раньше).
    FlightController(
        IBusReceiver& receiver,
        ControlMixer& mixer,
        ThrottleManager& throttle,
        ArmingManager& arming,
        FlightOutputs& outputs,
        Autopilot* autopilot = nullptr,
        FeatureManager* featureManager = nullptr
    )
        : receiver(receiver),
          mixer(mixer),
          throttle(throttle),
          arming(arming),
          outputs(outputs),
          autopilot(autopilot),
          featureManager(featureManager)
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

        // FeatureManager/Autopilot обновляются до failsafe-проверки:
        // так их внутренние таймеры/фильтры продолжают идти даже
        // если этот конкретный цикл потом обрывается по failsafe.
        if (featureManager && !receiverFailsafe)
        {
            featureManager->update(receiver.getState());
        }

        if (autopilot && !receiverFailsafe)
        {
            autopilot->update();
        }

        const RcChannelState& rc = receiver.getState();

        if (receiverFailsafe)
        {
            arming.update(Config::PWM_MIN, true);
            throttle.update(rc, true);
            outputs.setFailsafe();
            return;  // ничего больше не делаем — самолёт в безопасном режиме
        }

        arming.update(rc.get(Channels::THROTTLE), false);

        FlightOutputState output = mixer.calculate(rc);

        // Коррекции автопилота применяются только когда armed —
        // иначе на земле до вооружения система могла бы дёргать
        // поверхности сама.
        if (autopilot && arming.isArmed())
        {
            const float rollCorr = autopilot->getRollCorrection();
            const float pitchCorr = autopilot->getPitchCorrection();

            output.aileronLeft += pitchCorr - rollCorr;
            output.aileronRight += pitchCorr + rollCorr;
            output.elevator += pitchCorr;

            output.aileronLeft = constrain(output.aileronLeft, Config::PWM_MIN, Config::PWM_MAX);
            output.aileronRight = constrain(output.aileronRight, Config::PWM_MIN, Config::PWM_MAX);
            output.elevator = constrain(output.elevator, Config::PWM_MIN, Config::PWM_MAX);
        }

        output.throttle = throttle.update(rc, false);

        outputs.write(output);
    }


    // Для DebugLogger/WebDebugServer.
    bool isReceiverFailsafe() const { return receiver.isSignalLost(); }
    bool isArmed() const { return arming.isArmed(); }
    bool isBoostActive() const { return throttle.isBoostActive(); }
    const FlightOutputState& getOutputState() const { return outputs.getLastState(); }
    const RcChannelState& getRcState() const { return receiver.getState(); }
    const FlightOutputs& getOutputs() const { return outputs; }


private:

    IBusReceiver& receiver;
    ControlMixer& mixer;
    ThrottleManager& throttle;
    ArmingManager& arming;
    FlightOutputs& outputs;

    Autopilot* autopilot;
    FeatureManager* featureManager;
};
