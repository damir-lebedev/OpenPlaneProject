#pragma once
// ============================================================
// 8. THROTTLE MANAGER
//
// Вся логика двигателя находится здесь.
//
// Ответственность:
//
// - чтение throttle;
// - ограничение мощности;
// - boost;
// - boost timer;
// - повторная активация boost.
//
// Этот класс ничего не знает о Servo.
// Он просто возвращает требуемый PWM.
// ============================================================

class ThrottleManager
{
public:

    // --------------------------------------------------------
    // Обновление throttle.
    // --------------------------------------------------------

    uint16_t update(
        const RcChannelState& rc,
        bool receiverFailsafe
    )
    {
        const uint32_t now = millis();


        // ----------------------------------------------------
        // Приёмник потерян → немедленно выключаем boost.
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // LOW на boost switch снова разрешает следующий boost.
        // ----------------------------------------------------

        if (boostSwitch < 1250)
        {
            boostReady = true;
        }


        // ----------------------------------------------------
        // HIGH на boost switch запускает boost.
        //
        // Boost запускается только один раз до тех пор,
        // пока переключатель не вернётся в LOW.
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // Проверяем таймер boost.
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // Во время boost разрешаем полный газ.
        // ----------------------------------------------------

        if (boostActive)
        {
            return Config::PWM_MAX;
        }


        // ----------------------------------------------------
        // Обычный режим.
        //
        // 1000 → 1000
        // 1500 → 1200
        // 2000 → 1400
        //
        // То есть весь ход стика сохраняется,
        // но максимум ограничивается 40%.
        // ----------------------------------------------------

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


        // ----------------------------------------------------
        // Дополнительная защита результата.
        // ----------------------------------------------------

        limitedThrottle =
            constrain(
                limitedThrottle,
                Config::PWM_MIN,
                maximumThrottle
            );


        return limitedThrottle;
    }


    // --------------------------------------------------------
    // Активен ли boost прямо сейчас.
    // --------------------------------------------------------

    bool isBoostActive() const
    {
        return boostActive;
    }


    // --------------------------------------------------------
    // Можно ли снова запустить boost.
    // --------------------------------------------------------

    bool isBoostReady() const
    {
        return boostReady;
    }


private:

    bool boostActive = false;

    bool boostReady = true;

    uint32_t boostStartTime = 0;
};