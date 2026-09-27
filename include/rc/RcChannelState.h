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

    RcChannelState();

    // Безопасные значения: все каналы в центре, кроме throttle (в минимум).
    void reset();

    uint16_t get(uint8_t index) const;

    void set(uint8_t index, uint16_t value);

    // Доступ ко всему массиву сразу — только для кода, которому
    // действительно нужен весь набор каналов (например, отладка).
    const uint16_t* data() const;


private:

    uint16_t channels[Config::IBUS_CHANNELS] = {};
};