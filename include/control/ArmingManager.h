#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "config/Channels.h"
#include "config/Config.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"

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
// включает режим, которому он нужен. Для режимов со стабилизацией
// IMU ещё и должен пройти предполётную проверку
// (ImuSensor::getPreflightProblem(): неподвижность при калибровке,
// установка платы совпадает с сохранённой).
//
// GPS-фикс в проверки намеренно НЕ включён: LOITER и RTH без GPS
// не опасны — они кружат на месте с удержанием высоты (см.
// Autopilot::orbit()), а дом записывается при первом хорошем фиксе.
// ============================================================

class ArmingManager
{
public:

    explicit ArmingManager(Autopilot* ap = nullptr);

    void update(
        const RcChannelState& rc,
        bool receiverFailsafe
    );

    bool isArmed() const;

    // Причина последнего отказа в ARM (nullptr — отказа не было) —
    // для отладочного вывода/дашборда.
    const char* getLastRefusalReason() const;


private:

    Autopilot* autopilot;

    bool armed = false;

    // false до тех пор, пока прошивка не увидела тумблер в OFF — так
    // включение платы с уже поднятым тумблером не армит мотор.
    bool switchSeenOff = false;

    const char* lastPrintedReason = nullptr;

    const char* checkFailureReason(const RcChannelState& rc) const;

    static bool needsAltitude(AutopilotMode mode);
};
