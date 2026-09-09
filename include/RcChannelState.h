#pragma once
// ============================================================
// 3. RC CHANNEL STATE
//
// Этот класс представляет состояние одного набора каналов.
// Никакой логики управления самолётом здесь нет.
//
// В будущем сюда можно будет добавить:
// - timestamp;
// - quality;
// - signal strength;
// - channel validity;
// - failsafe information.
// ============================================================

class RcChannelState
{
public:

    RcChannelState()
    {
        reset();
    }


    // --------------------------------------------------------
    // Сброс каналов в безопасное состояние.
    // --------------------------------------------------------

    void reset()
    {
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            channels[i] = Config::PWM_CENTER;
        }

        channels[Channels::THROTTLE] = Config::PWM_MIN;
    }


    // --------------------------------------------------------
    // Получить значение конкретного канала.
    // --------------------------------------------------------

    uint16_t get(uint8_t index) const
    {
        if (index >= Config::IBUS_CHANNELS)
        {
            return Config::PWM_CENTER;
        }

        return channels[index];
    }


    // --------------------------------------------------------
    // Установить значение конкретного канала.
    // --------------------------------------------------------

    void set(uint8_t index, uint16_t value)
    {
        if (index >= Config::IBUS_CHANNELS)
        {
            return;
        }

        channels[index] = value;
    }


    // --------------------------------------------------------
    // Доступ к массиву каналов.
    //
    // Используется только для систем, которым действительно
    // нужен весь набор каналов.
    // --------------------------------------------------------

    const uint16_t* data() const
    {
        return channels;
    }


private:

    uint16_t channels[Config::IBUS_CHANNELS];
};