#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
// ============================================================
// RC CHANNEL STATE
//
// Снимок всех 10 каналов приёмника, без какой-либо логики
// управления самолётом.
// ============================================================

class RcChannelState
{
public:

    RcChannelState()
    {
        reset();
    }

    // Безопасные значения: все каналы в центре, кроме throttle (в минимум).
    void reset()
    {
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            channels[i] = Config::PWM_CENTER;
        }

        channels[Channels::THROTTLE] = Config::PWM_MIN;
    }

    uint16_t get(uint8_t index) const
    {
        if (index >= Config::IBUS_CHANNELS)
        {
            return Config::PWM_CENTER;
        }

        return channels[index];
    }

    void set(uint8_t index, uint16_t value)
    {
        if (index >= Config::IBUS_CHANNELS)
        {
            return;
        }

        channels[index] = value;
    }

    // Доступ ко всему массиву сразу — только для кода, которому
    // действительно нужен весь набор каналов (например, отладка).
    const uint16_t* data() const
    {
        return channels;
    }


private:

    uint16_t channels[Config::IBUS_CHANNELS];
};