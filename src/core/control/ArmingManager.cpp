// Реализация control/ArmingManager.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/ArmingManager.h"


ArmingManager::ArmingManager(Autopilot* ap)
: autopilot(ap)
{
}

auto ArmingManager::update(
        const RcChannelState& rc,
        bool receiverFailsafe
    ) -> void
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

auto ArmingManager::isArmed() const -> bool
{
    return armed;
}

auto ArmingManager::getLastRefusalReason() const -> const char*
{
    return lastPrintedReason;
}

auto ArmingManager::checkFailureReason(const RcChannelState& rc) const -> const char*
{
    if (rc.get(Channels::THROTTLE) >= Config::THROTTLE_LOW_US)
    {
        return "газ не на минимуме";
    }

    if (!autopilot) return nullptr;

    const AutopilotMode mode = autopilot->getMode();

    // Всем режимам, кроме MANUAL, нужны углы.
    if (mode != MODE_MANUAL)
    {
        const ImuSensor* imu = autopilot->getImuSensor();
        if (imu && !imu->isAvailable())
        {
            return "IMU не отвечает, а выбранному режиму нужен гироскоп";
        }
        if (imu && imu->getPreflightProblem())
        {
            return imu->getPreflightProblem();
        }
    }

    // Режимам с удержанием высоты — барометр.
    if (needsAltitude(mode))
    {
        const BarometerSensor* baro = autopilot->getBarometerSensor();
        if (baro && !baro->isAvailable())
        {
            return "барометр не отвечает, а выбранному режиму нужна высота";
        }
    }

    return nullptr;
}

auto ArmingManager::needsAltitude(AutopilotMode mode) -> bool
{
    return mode == MODE_ALT_HOLD || mode == MODE_CRUISE || mode == MODE_LOITER || mode == MODE_RTH ||
           mode == MODE_AUTO_LAND || mode == MODE_SOARING;
}
