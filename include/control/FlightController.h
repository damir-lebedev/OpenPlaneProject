#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/PilotSwitches.h"
#include "config/Channels.h"
#include "config/Config.h"
#include "control/ArmingManager.h"
#include "control/Beeper.h"
#include "control/ControlCommand.h"
#include "control/ControlMixer.h"
#include "control/FlightOutputState.h"
#include "control/FlightOutputs.h"
#include "control/GroundCalibration.h"
#include "control/MotorReverse.h"
#include "control/SignalPlayer.h"
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
//   5. ARMING     — тумблер ARM (SwA); сразу после смены направления
//                   ARM запрещён (мотор ещё крутится), а также пока идёт
//                   наземная калибровка или звучит сигнал
//   6. РЕВЕРС     — MotorReverse: направление мотора по тумблеру
//                   Feature::MOTOR_REVERSE, но только пока не armed
//   7. ВЫХОДЫ     — команда -> PWM каждого серво; газ ограничен
//                   Config::THROTTLE_LIMIT_PCT и крутилкой
//                   Knob::THROTTLE_LIMIT (стик и автопилот одинаково),
//                   назад — ещё и REVERSE_THROTTLE_LIMIT_PCT,
//                   затем -> 0, если не armed или включён
//                   Feature::MOTOR_KILL; AUX1 — груз, AUX2 — камера;
//                   пищалка
//
// В реверсе газом управляет только пилот: автопилот получает газ
// 0 (для него мотор выключен — ни автовзлёт, ни круиз не стартуют),
// его газ не применяется, при потере связи мотор глохнет.
//
// НАЗЕМНАЯ КАЛИБРОВКА (GroundCalibration) и СИГНАЛЫ (SignalPlayer):
// калибровку запускает жест тумблером без ARM (Config::CAL_SWITCH) или
// консоль ('c'), шаги идут в этом же цикле. Сигнал — поверх выходов:
// рули высоты и направления кивают, без ARM ещё и пропадают импульсы
// ESC (регулятор потом играет мелодию готовности), пищалка/светодиод.
// Сигналы звучат и без связи (пульт положили — калибровка идёт).
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
        const bool reversed = reverse.isReversed();

        ControlCommand sticks;
        if (!receiverFailsafe)
        {
            sticks = mixer.fromSticks(rc);
            applyRates(sticks, in);
        }
        sticks.flaps = mixer.updateFlaps(receiverFailsafe ? 0.0f : flapsTarget(in), nowMs);

        if (autopilot) autopilot->update(arming.isArmed(), receiverFailsafe, reversed ? Config::PWM_MIN : pilotThrottle, sticks);

        updateCalibration(rc, receiverFailsafe, nowMs);
        outputs.setBuzzer(beeper.update(in.has(Feature::BEEPER), arming.isArmed(), receiverFailsafe, nowMs) ||
                          signals.light(nowMs));

        if (receiverFailsafe)
        {
            applyLinkLoss(sticks.flaps, in, nowMs);
            return;  // стики и тумблеры в этом цикле не участвуют
        }

        // ARM — раньше реверса: на DISARM направление меняется в том же
        // такте, а щелчок обоих тумблеров разом армит в старом
        // направлении (новое ждёт следующего DISARM).
        arming.update(rc, false, armBlockReason(nowMs));
        reverse.update(in.has(Feature::MOTOR_REVERSE), arming.isArmed(), nowMs);

        ControlCommand command = autopilot ? autopilot->getCommand() : sticks;
        command.flaps = sticks.flaps;
        addNod(command, nowMs);

        FlightOutputState output = mixer.mix(command);
        output.motorReverse = reverse.isReversed();
        if (output.motorReverse)
        {
            output.throttle = capThrottle(pilotThrottle, reverseLimitPct(in));
        }
        else
        {
            output.throttle = autopilot ? autopilot->applyThrottle(pilotThrottle) : pilotThrottle;
            output.throttle = capThrottle(output.throttle, throttleLimitPct(in));
        }

        // ARM реально блокирует газ (см. ArmingManager) — ставим ПОСЛЕ
        // автопилота, чтобы ни один режим не мог протащить газ мимо
        // этой проверки. MOTOR_KILL — то же, по тумблеру.
        if (!arming.isArmed() || in.has(Feature::MOTOR_KILL))
        {
            output.throttle = Config::PWM_MIN;
        }
        output.escPulsesOff = !arming.isArmed() && signals.escPulsesOff(nowMs);

        applyAuxOutputs(output, in);
        outputs.write(output);
    }

    // Наземная калибровка из консоли ('c'): запустить, а идёт — отменить.
    void toggleGroundCalibration()
    {
        const uint32_t nowMs = millis();
        if (calibration.isActive())
        {
            calibration.cancel("из консоли");
            return;
        }
        if (arming.isArmed())
        {
            Serial.println("Калибровка самолёта: только без ARM");
            return;
        }
        calibration.start(autopilot ? autopilot->getImuSensor() : nullptr, nowMs, "консоль");
    }

    // Сигнал пилоту (консоль 'e' — проверить, поёт ли ESC после паузы).
    void playSignal(uint8_t count)
    {
        signals.play(count, millis(), !arming.isArmed());
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
    const MotorReverse& getMotorReverse() const { return reverse; }
    const GroundCalibration& getCalibration() const { return calibration; }
    const SignalPlayer& getSignals() const { return signals; }


private:

    IBusReceiver& receiver;
    ControlMixer& mixer;
    ThrottleManager& throttle;
    ArmingManager& arming;
    FlightOutputs& outputs;

    Autopilot* autopilot;
    PilotSwitches* switches;

    Beeper beeper;
    MotorReverse reverse;
    GroundCalibration calibration;
    SignalPlayer signals;
    bool armSwitchWasOn = false;
    bool cancelledByArmSwitch = false;
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

    // Предел газа вперёд, %: Config::THROTTLE_LIMIT_PCT, и поверх —
    // крутилка Knob::THROTTLE_LIMIT (THROTTLE_KNOB_MIN_PCT ... 100 %),
    // если привязана. Один выбор на всех — и ручной газ, и автопилотный
    // (CRUISE/RTH/LAUNCH и т.п.).
    static float throttleLimitPct(const PilotInputs& in)
    {
        float pct = Config::THROTTLE_LIMIT_PCT;
        if (in.isBound(Knob::THROTTLE_LIMIT))
        {
            const float centre = (Config::THROTTLE_KNOB_MIN_PCT + 100.0f) * 0.5f;
            pct *= in.knobValue(Knob::THROTTLE_LIMIT, Config::THROTTLE_KNOB_MIN_PCT, centre, 100.0f) / 100.0f;
        }
        return pct;
    }

    // Назад — свой предел REVERSE_THROTTLE_LIMIT_PCT, но не выше переднего.
    static float reverseLimitPct(const PilotInputs& in)
    {
        return min(static_cast<float>(Config::REVERSE_THROTTLE_LIMIT_PCT), throttleLimitPct(in));
    }

    // Разрешённое растягивается на весь ход: PWM_MIN..PWM_MAX ->
    // PWM_MIN..(PWM_MIN + pct% хода), стик остаётся пропорциональным —
    // просто верх ниже.
    static uint16_t capThrottle(uint16_t us, float pct)
    {
        const float span = Config::PWM_MAX - Config::PWM_MIN;
        const float x = constrain(static_cast<float>(us) - Config::PWM_MIN, 0.0f, span);
        return static_cast<uint16_t>(lroundf(Config::PWM_MIN + x * pct / 100.0f));
    }

    // Куда ехать закрылкам: тормоз важнее закрылков, тумблер — крутилки.
    static float flapsTarget(const PilotInputs& in)
    {
        if (in.has(Feature::AIRBRAKE)) return -static_cast<float>(Config::AIRBRAKE_US);
        if (in.has(Feature::FLAPS)) return Config::FLAPS_DEPLOYED_US;
        if (in.isBound(Knob::FLAPS)) return (in.knob(Knob::FLAPS) + 1.0f) * 0.5f * Config::FLAPS_DEPLOYED_US;
        return 0.0f;
    }

    // Калибровка: жест тумблером (только связь есть и без ARM), шаги,
    // сигналы её и отказа автовзлёта.
    void updateCalibration(const RcChannelState& rc, bool receiverFailsafe, uint32_t nowMs)
    {
        ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        const bool armed = arming.isArmed();

        // Щелчок ARM во время калибровки — отмена (и в этот раз без ARM).
        cancelledByArmSwitch = false;
        if (!receiverFailsafe)
        {
            const bool armSwitch = rc.get(Channels::ARM) >= Config::ARM_SWITCH_ON_US;
            if (armSwitch && !armSwitchWasOn && calibration.isActive())
            {
                calibration.cancel("тумблер ARM");
                cancelledByArmSwitch = true;
            }
            armSwitchWasOn = armSwitch;
        }

        if (receiverFailsafe || armed)
        {
            calibration.forgetSwitch();
        }
        else if (calibration.watchSwitch(rc, nowMs))
        {
            if (calibration.isActive()) calibration.cancel("с пульта");
            else calibration.start(imu, nowMs, "пульт");
        }

        calibration.update(imu, armed, signals.isBusy(nowMs), nowMs);
        if (const uint8_t count = calibration.takeSignal()) signals.play(count, nowMs, !armed);
        if (autopilot && autopilot->takeTakeoffRefusal()) signals.play(3, nowMs, false);
    }

    // Почему ARM сейчас нельзя (nullptr — можно).
    const char* armBlockReason(uint32_t nowMs) const
    {
        if (cancelledByArmSwitch) return "этим щелчком отменена калибровка — для ARM выключите и включите тумблер снова";
        if (calibration.isActive()) return "идёт калибровка самолёта — дождитесь сигнала или отмените";
        if (signals.isBusy(nowMs)) return "звучит сигнал — подождите пару секунд";
        return reverse.armBlockReason(nowMs);
    }

    // Кивок рулей высоты и направления (сигнал пилоту).
    void addNod(ControlCommand& command, uint32_t nowMs) const
    {
        const int16_t nod = signals.nodUs(nowMs);
        if (nod == 0) return;
        command.pitch = static_cast<int16_t>(constrain(command.pitch + nod, -Config::ELEVATOR_MAX_US, Config::ELEVATOR_MAX_US));
        command.yaw = static_cast<int16_t>(constrain(command.yaw + nod, -Config::RUDDER_MAX_US, Config::RUDDER_MAX_US));
    }

    // AUX1 — груз (закрыт, пока функция не включена), AUX2 — камера:
    // угол с крутилки, со стабилизацией — минус тангаж самолёта.
    void applyAuxOutputs(FlightOutputState& output, const PilotInputs& in) const
    {
        output.aux1 = Config::directedUs(in.has(Feature::PAYLOAD_DROP) ? Config::PAYLOAD_OPEN_US : Config::PAYLOAD_CLOSED_US,
                                         Config::AUX1_REVERSED);

        float tilt = in.knobValue(Knob::CAMERA_TILT, Config::CAMERA_TILT_MIN_DEG, 0.0f, Config::CAMERA_TILT_MAX_DEG);
        const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        if (in.has(Feature::CAMERA_STAB) && imu && imu->isAvailable())
        {
            tilt -= imu->getImuData().pitch;
        }
        const float pwm = Config::PWM_CENTER + tilt * Config::CAMERA_US_PER_DEG;
        output.aux2 = Config::directedUs(static_cast<uint16_t>(constrain(pwm, static_cast<float>(Config::PWM_MIN),
                                                                         static_cast<float>(Config::PWM_MAX))),
                                         Config::AUX2_REVERSED);
    }

    // Потеря связи. В воздухе (armed) автопилот ведёт самолёт сам:
    // домой с мотором или планирование с выключенным мотором (см.
    // Autopilot::setFailsafe()); в реверсе мотор всегда стоит;
    // закрылки плавно убираются, груз и камера — как были. На земле
    // или без автопилота — нейтраль (и сигнал калибровки поверх: пульт
    // могли положить).
    void applyLinkLoss(int16_t flapsUs, const PilotInputs& in, uint32_t nowMs)
    {
        if (!autopilot || !autopilot->isFailsafeActive())
        {
            if (!signals.isBusy(nowMs))
            {
                outputs.setFailsafe();
                return;
            }
            ControlCommand nod;
            addNod(nod, nowMs);
            FlightOutputState safe = mixer.mix(nod);
            safe.throttle = Config::FAILSAFE_THROTTLE;
            safe.escPulsesOff = !arming.isArmed() && signals.escPulsesOff(nowMs);
            safe.aux1 = outputs.getLastState().aux1;
            safe.aux2 = outputs.getLastState().aux2;
            outputs.write(safe);
            return;
        }

        ControlCommand command = autopilot->getCommand();
        command.flaps = flapsUs;

        FlightOutputState output = mixer.mix(command);
        output.throttle = autopilot->applyThrottle(Config::FAILSAFE_THROTTLE);
        output.throttle = capThrottle(output.throttle, throttleLimitPct(in));
        output.motorReverse = reverse.isReversed();
        // Назад автопилот мотором не ведёт — домой только планированием.
        if (in.has(Feature::MOTOR_KILL) || output.motorReverse) output.throttle = Config::PWM_MIN;
        output.aux1 = outputs.getLastState().aux1;
        output.aux2 = outputs.getLastState().aux2;
        outputs.write(output);
    }
};
