// Реализация rc/RcChannelState.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "rc/RcChannelState.h"


RcChannelState::RcChannelState()
{
    reset();
}

auto RcChannelState::reset() -> void
{
    for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
    {
        channels[i] = Config::PWM_CENTER;
    }

    channels[Channels::THROTTLE] = Config::PWM_MIN;
}

auto RcChannelState::get(uint8_t index) const -> uint16_t
{
    if (index >= Config::IBUS_CHANNELS)
    {
        return Config::PWM_CENTER;
    }

    return channels[index];
}

auto RcChannelState::set(uint8_t index, uint16_t value) -> void
{
    if (index >= Config::IBUS_CHANNELS)
    {
        return;
    }

    channels[index] = value;
}

auto RcChannelState::data() const -> const uint16_t*
{
    return channels;
}
