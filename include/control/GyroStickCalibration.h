#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "control/ControlCommand.h"
#include "rc/RcChannelState.h"

// ============================================================
// КАЛИБРОВКА ГИРОСКОПА С ПУЛЬТА (как в Betaflight)
//
// Без ARM: газ вниз, рысканье влево, тангаж вниз (стик на себя),
// элероны в центре — подержать STICK_GYRO_CAL_HOLD_MS. Ноль гироскопа
// меряется заново (~2 с, самолёт неподвижен), ответ — кивки рулей:
// 2 — готово, 3 — самолёт двигали, ноль не принят (FlightController).
// Компьютер на поле не нужен.
//
// Стики читаются в знаках ControlCommand (ControlMixer::fromSticks()):
// «тангаж вниз» — это команда «нос вверх», «рысканье влево» — «нос
// влево», так что реверс каналов в пульте и в Config.h учтён. Сработав,
// команда ждёт, пока стики отпустят — повтора от удержания нет.
// ============================================================

class GyroStickCalibration
{
public:

    // true — жест удержан нужное время (один раз на удержание).
    // Вызывать каждый такт при живой связи и без ARM.
    bool update(const RcChannelState& rc, const ControlCommand& sticks, uint32_t nowMs)
    {
        if (!Config::STICK_GYRO_CAL)
        {
            return false;
        }

        const bool held = rc.get(Channels::THROTTLE) < Config::THROTTLE_LOW_US &&
                          sticks.yaw <= -FULL * Config::RUDDER_MAX_US &&
                          sticks.pitch >= FULL * Config::ELEVATOR_MAX_US &&
                          abs(sticks.roll) <= CENTER * Config::AILERON_MAX_US;
        if (!held)
        {
            holding = false;
            fired = false;
            return false;
        }
        if (!holding)
        {
            holding = true;
            sinceMs = nowMs;
        }
        if (fired || nowMs - sinceMs < Config::STICK_GYRO_CAL_HOLD_MS)
        {
            return false;
        }
        fired = true;
        return true;
    }

    // Связь пропала или ARM — жест начинается заново.
    void forget()
    {
        holding = false;
        fired = false;
    }


private:

    static constexpr float FULL = 0.8f;     // «до упора» — 80 % хода
    static constexpr float CENTER = 0.3f;   // «в центре» — не дальше 30 %

    bool holding = false;
    bool fired = false;
    uint32_t sinceMs = 0;
};
