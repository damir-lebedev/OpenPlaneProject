#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "config/Channels.h"
#include "config/Config.h"
#include "rc/RcChannelState.h"

// ============================================================
// ARMING MANAGER
//
// ARM — отдельным тумблером SwA (Channels::ARM, CH5):
//   • ARM: тумблер переведён из OFF в ON, газ в этот момент внизу
//     (< THROTTLE_LOW_US) и пройдены предполётные проверки
//     (checkFailureReason()). Если газ не внизу или проверки не
//     пройдены — ARM не происходит, нужно выключить тумблер, убрать
//     газ и включить снова (нельзя "случайно" заармиться, подняв
//     тумблер с газом вверху и потом убрав газ).
//   • DISARM: тумблер в OFF — сразу, в любой момент.
//   • При включении платы тумблер уже в ON не армит: нужен именно
//     переход OFF -> ON, увиденный прошивкой.
//
// Раньше ARM наступал сам, если газ пролежал внизу 1.5 секунды,
// и не снимался ничем, кроме потери связи — то есть после каждого
// включения мотор был заармлен без действий пилота.
//
// Потеря связи (failsafe) ARM НЕ снимает: FlightController на время
// failsafe сам глушит мотор и ставит рули в нейтраль, а когда связь
// вернулась, самолёт продолжает лететь по стикам без повторного
// ARM — для самолёта это безопаснее, чем требовать в воздухе
// переключать тумблер с газом в ноль. Пока связи нет, тумблер
// не читается (его значение в failsafe-кадре не отражает пилота).
//
// armed реально блокирует газ — см. FlightController::update(),
// где output.throttle принудительно ставится в PWM_MIN, пока !armed.
//
// Предполётные проверки требуют только те датчики, которые реально
// нужны ТЕКУЩЕМУ выбранному режиму автопилота (см. Autopilot::
// handle*Mode()) — если пилот держит переключатель в MANUAL, борт
// без единого датчика по-прежнему спокойно армится. Датчик
// становится обязательным ровно в тот момент, когда пилот реально
// включает режим, которому он нужен.
//
// GPS-фикс в проверки намеренно НЕ включён: сегодня ни один
// AutopilotMode фактически не использует GPS (см. Фазу 3/4 в
// docs/ROADMAP.md) — добавить её нужно вместе с первым режимом,
// которому GPS реально нужен (waypoint/RTH).
// ============================================================

class ArmingManager
{
public:

    explicit ArmingManager(Autopilot* autopilot = nullptr)
        : autopilot(autopilot)
    {
    }

    void update(
        const RcChannelState& rc,
        bool receiverFailsafe
    )
    {
        if (receiverFailsafe)
        {
            return;
        }

        const bool switchOn = rc.get(Channels::ARM) >= Config::ARM_SWITCH_ON_US;

        if (!switchOn)
        {
            if (armed)
            {
                Serial.println("ArmingManager: DISARM (тумблер ARM выключен)");
            }

            armed = false;
            switchSeenOff = true;
            lastPrintedReason = nullptr;
            return;
        }

        // Тумблер ON. Армимся только на переходе OFF -> ON.
        if (armed || !switchSeenOff)
        {
            return;
        }

        switchSeenOff = false;  // эта попытка использована, дальше — только через OFF

        const char* reason = checkFailureReason(rc);

        if (reason)
        {
            Serial.print("ArmingManager: ARM отклонён — ");
            Serial.print(reason);
            Serial.println(" (выключите тумблер ARM и попробуйте снова)");
            lastPrintedReason = reason;
            return;
        }

        armed = true;
        Serial.println("ArmingManager: ARM");
    }

    bool isArmed() const
    {
        return armed;
    }

    // Причина последнего отказа в ARM (nullptr — отказа не было) —
    // для отладочного вывода/дашборда.
    const char* getLastRefusalReason() const
    {
        return lastPrintedReason;
    }


private:

    Autopilot* autopilot;

    bool armed = false;

    // false до тех пор, пока прошивка не увидела тумблер в OFF — так
    // включение платы с уже поднятым тумблером не армит мотор.
    bool switchSeenOff = false;

    const char* lastPrintedReason = nullptr;

    const char* checkFailureReason(const RcChannelState& rc) const
    {
        if (rc.get(Channels::THROTTLE) >= Config::THROTTLE_LOW_US)
        {
            return "газ не на минимуме";
        }

        if (!autopilot) return nullptr;

        switch (autopilot->getMode())
        {
            case MODE_STABILIZE:
            case MODE_AUTO_TAKEOFF:
            {
                ImuSensor* imu = autopilot->getImuSensor();
                if (imu && !imu->isAvailable())
                {
                    return "IMU не отвечает, а выбранному режиму нужен гироскоп";
                }
                break;
            }

            case MODE_ALT_HOLD:
            {
                BarometerSensor* baro = autopilot->getBarometerSensor();
                if (baro && !baro->isAvailable())
                {
                    return "барометр не отвечает, а ALT_HOLD нужна высота";
                }
                break;
            }

            case MODE_MANUAL:
            default:
                break;
        }

        return nullptr;
    }
};
