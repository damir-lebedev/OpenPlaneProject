#pragma once

// ============================================================
// ARMING MANAGER
//
// Держит throttle на LOW дольше Config::ARM_LOW_TIME_MS -> armed.
//
// ВАЖНО: armed сейчас НЕ блокирует throttle/outputs — это только
// отслеживаемое состояние. Как только появится реальная arm/disarm
// логика (запрет газа до ARM), её нужно менять только здесь, не
// трогая receiver/mixer/outputs.
// ============================================================

class ArmingManager
{
public:

    void update(
        uint16_t throttle,
        bool receiverFailsafe
    )
    {
        if (receiverFailsafe)
        {
            armed = false;
            throttleLowSince = 0;
            return;
        }

        if (throttle < Config::THROTTLE_LOW_US)
        {
            if (throttleLowSince == 0)
            {
                throttleLowSince = millis();
            }

            if (
                millis() - throttleLowSince >=
                Config::ARM_LOW_TIME_MS
            )
            {
                armed = true;
            }

            return;
        }

        // Газ поднят: не делаем автоматический disarm, чтобы не
        // менять поведение оригинальной прошивки.
        throttleLowSince = 0;
    }

    bool isArmed() const
    {
        return armed;
    }


private:

    bool armed = false;

    uint32_t throttleLowSince = 0;
};
