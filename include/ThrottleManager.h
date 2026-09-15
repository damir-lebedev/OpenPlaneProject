#pragma once
// ============================================================
// THROTTLE MANAGER
//
// Читает throttle (CH3) и boost switch (Channels::BOOST, CH8),
// ограничивает обычный газ до THROTTLE_LIMIT_PERCENT и разрешает
// временный полный газ через boost. Не знает про Servo — просто
// возвращает нужный PWM.
// ============================================================

class ThrottleManager
{
public:

    uint16_t update(
        const RcChannelState& rc,
        bool receiverFailsafe
    )
    {
        const uint32_t now = millis();

        if (receiverFailsafe)
        {
            boostActive = false;
            return Config::FAILSAFE_THROTTLE;
        }

        const uint16_t throttle =
            RcInput::clamp(
                rc.get(Channels::THROTTLE)
            );

        const uint16_t boostSwitch =
            rc.get(Channels::BOOST);

        // LOW на переключателе снова разрешает следующий boost.
        if (boostSwitch < 1250)
        {
            boostReady = true;
        }

        // HIGH запускает boost один раз, до возврата переключателя в LOW.
        if (
            boostSwitch >= 1750 &&
            boostReady &&
            !boostActive
        )
        {
            boostActive = true;
            boostReady = false;
            boostStartTime = now;
        }

        if (boostActive)
        {
            if (
                now - boostStartTime >=
                Config::THROTTLE_BOOST_TIME_MS
            )
            {
                boostActive = false;
            }
        }

        if (boostActive)
        {
            return Config::PWM_MAX;
        }

        // Весь ход стика сохраняется, но верхняя граница
        // ограничена THROTTLE_LIMIT_PERCENT (1000->1000, 2000->1400 при 40%).
        const uint16_t maximumThrottle =
            Config::PWM_MIN +
            (
                (Config::PWM_MAX - Config::PWM_MIN) *
                Config::THROTTLE_LIMIT_PERCENT
            ) / 100;

        uint16_t limitedThrottle =
            map(
                throttle,
                Config::PWM_MIN,
                Config::PWM_MAX,
                Config::PWM_MIN,
                maximumThrottle
            );

        limitedThrottle =
            constrain(
                limitedThrottle,
                Config::PWM_MIN,
                maximumThrottle
            );

        return limitedThrottle;
    }

    bool isBoostActive() const
    {
        return boostActive;
    }

    bool isBoostReady() const
    {
        return boostReady;
    }


private:

    bool boostActive = false;

    bool boostReady = true;

    uint32_t boostStartTime = 0;
};
