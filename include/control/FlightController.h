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
    )
        : receiver(rcReceiver),
          mixer(controlMixer),
          throttle(throttleManager),
          arming(armingManager),
          outputs(flightOutputs),
          autopilot(ap),
          switches(pilotSwitches)
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
        const uint32_t nowMs = millis();

        // Режим и функции с тумблеров — только при живой связи: в
        // failsafe-кадре каналы содержат значения failsafe.
        if (switches && !receiverFailsafe) switches->update(rc);
        const PilotInputs& in = inputs();

        const uint16_t pilotThrottle = throttle.update(rc, receiverFailsafe);

        ControlCommand sticks;
        if (!receiverFailsafe)
        {
            sticks = mixer.fromSticks(rc);
            applyRates(sticks, in);
        }
        sticks.flaps = mixer.updateFlaps(receiverFailsafe ? 0.0f : flapsTarget(in), nowMs);

        if (autopilot) autopilot->update(arming.isArmed(), receiverFailsafe, pilotThrottle, sticks);

        outputs.setBuzzer(beeper.update(in.has(Feature::BEEPER), arming.isArmed(), receiverFailsafe, nowMs));

        if (receiverFailsafe)
        {
            applyLinkLoss(sticks.flaps, in);
            return;  // стики и тумблеры в этом цикле не участвуют
        }

        arming.update(rc, false);

        ControlCommand command = autopilot ? autopilot->getCommand() : sticks;
        command.flaps = sticks.flaps;

        FlightOutputState output = mixer.mix(command);
        output.throttle = autopilot ? autopilot->applyThrottle(pilotThrottle) : pilotThrottle;

        // ARM реально блокирует газ (см. ArmingManager) — ставим ПОСЛЕ
        // автопилота, чтобы ни один режим не мог протащить газ мимо
        // этой проверки. MOTOR_KILL — то же, по тумблеру.
        if (!arming.isArmed() || in.has(Feature::MOTOR_KILL))
        {
            output.throttle = Config::PWM_MIN;
        }

        applyAuxOutputs(output, in);
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

    const PilotInputs& inputs() const
    {
        return switches ? switches->getInputs() : noInputs;
    }

    // Чувствительность стиков (Knob::RATES), если крутилка привязана.
    static void applyRates(ControlCommand& sticks, const PilotInputs& in)
    {
        if (!in.isBound(Knob::RATES)) return;

        const float centre = (Config::RATES_MIN_PCT + Config::RATES_MAX_PCT) * 0.5f;
        const float scale = in.knobValue(Knob::RATES, Config::RATES_MIN_PCT, centre, Config::RATES_MAX_PCT) / 100.0f;
        sticks.roll = static_cast<int16_t>(sticks.roll * scale);
        sticks.pitch = static_cast<int16_t>(sticks.pitch * scale);
        sticks.yaw = static_cast<int16_t>(sticks.yaw * scale);
    }

    // Куда ехать закрылкам: тормоз важнее закрылков, тумблер — крутилки.
    static float flapsTarget(const PilotInputs& in)
    {
        if (in.has(Feature::AIRBRAKE)) return -static_cast<float>(Config::AIRBRAKE_US);
        if (in.has(Feature::FLAPS)) return Config::FLAPS_DEPLOYED_US;
        if (in.isBound(Knob::FLAPS)) return (in.knob(Knob::FLAPS) + 1.0f) * 0.5f * Config::FLAPS_DEPLOYED_US;
        return 0.0f;
    }

    // AUX1 — груз (закрыт, пока функция не включена), AUX2 — камера:
    // угол с крутилки, со стабилизацией — минус тангаж самолёта.
    void applyAuxOutputs(FlightOutputState& output, const PilotInputs& in) const
    {
        output.aux1 = in.has(Feature::PAYLOAD_DROP) ? Config::PAYLOAD_OPEN_US : Config::PAYLOAD_CLOSED_US;

        float tilt = in.knobValue(Knob::CAMERA_TILT, Config::CAMERA_TILT_MIN_DEG, 0.0f, Config::CAMERA_TILT_MAX_DEG);
        const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        if (in.has(Feature::CAMERA_STAB) && imu && imu->isAvailable())
        {
            tilt -= imu->getImuData().pitch;
        }
        const float pwm = Config::PWM_CENTER + tilt * Config::CAMERA_US_PER_DEG;
        output.aux2 = static_cast<uint16_t>(constrain(pwm, static_cast<float>(Config::PWM_MIN),
                                                      static_cast<float>(Config::PWM_MAX)));
    }

    // Потеря связи. В воздухе (armed) автопилот ведёт самолёт сам:
    // домой с мотором или планирование с выключенным мотором (см.
    // Autopilot::setFailsafe()); закрылки плавно убираются, груз и
    // камера — как были. На земле или без автопилота — нейтраль.
    void applyLinkLoss(int16_t flapsUs, const PilotInputs& in)
    {
        if (!autopilot || !autopilot->isFailsafeActive())
        {
            outputs.setFailsafe();
            return;
        }

        ControlCommand command = autopilot->getCommand();
        command.flaps = flapsUs;

        FlightOutputState output = mixer.mix(command);
        output.throttle = autopilot->applyThrottle(Config::FAILSAFE_THROTTLE);
        if (in.has(Feature::MOTOR_KILL)) output.throttle = Config::PWM_MIN;
        output.aux1 = outputs.getLastState().aux1;
        output.aux2 = outputs.getLastState().aux2;
        outputs.write(output);
    }
};
