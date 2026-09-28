#pragma once
#include <stddef.h>
#include <stdint.h>

#include "autopilot/AutopilotTypes.h"
#include "config/Channels.h"

// ============================================================
// ПРИВЯЗКА КАНАЛА ПУЛЬТА К РЕЖИМУ / ФУНКЦИИ / КРУТИЛКЕ
//
// Одна привязка = одна строка таблицы в config/Controls.h:
//
//   Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE)
//   Bind::mode   (Channels::SWD, MODE_RTH)
//   Bind::feature(Channels::SWB, Feature::FLAPS)
//   Bind::knob   (Channels::VRA, Knob::STAB_GAIN)
//
// • modes  — тумблер на 2 или 3 положения выбирает режим: вверх
//   (<1250) — первый, середина — второй, вниз (>=1750) — третий. У
//   двухпозиционного — до/после 1500.
// • mode   — "поверх": пока тумблер включён (>=1750), режим
//   принудительный, выключили — снова режим с тумблера modes. Если
//   таких тумблеров несколько, приоритет у того, что выше в таблице.
// • feature — функция включена, пока тумблер включён (>=1750).
// • knob   — крутилка, плавно.
//
// Таблица проверяется при компиляции (static_assert в Controls.h):
// каналы стиков и ARM заняты, один канал — одна привязка, тумблер
// режимов — не больше одного.
// ============================================================

struct Binding
{
    enum class Kind : uint8_t { MODES, MODE, FEATURE, KNOB };

    Kind kind;
    uint8_t channel;
    AutopilotMode modes[3];
    uint8_t modeCount;
    Feature feature;
    Knob knob;
};

namespace Bind
{
    constexpr Binding modes(uint8_t channel, AutopilotMode up, AutopilotMode middle, AutopilotMode down)
    {
        return Binding{ Binding::Kind::MODES, channel, { up, middle, down }, 3, Feature::COUNT, Knob::COUNT };
    }

    constexpr Binding modes(uint8_t channel, AutopilotMode up, AutopilotMode down)
    {
        return Binding{ Binding::Kind::MODES, channel, { up, down, down }, 2, Feature::COUNT, Knob::COUNT };
    }

    constexpr Binding mode(uint8_t channel, AutopilotMode whenOn)
    {
        return Binding{ Binding::Kind::MODE, channel, { whenOn, whenOn, whenOn }, 1, Feature::COUNT, Knob::COUNT };
    }

    constexpr Binding feature(uint8_t channel, Feature f)
    {
        return Binding{ Binding::Kind::FEATURE, channel, { MODE_MANUAL, MODE_MANUAL, MODE_MANUAL }, 0, f, Knob::COUNT };
    }

    constexpr Binding knob(uint8_t channel, Knob k)
    {
        return Binding{ Binding::Kind::KNOB, channel, { MODE_MANUAL, MODE_MANUAL, MODE_MANUAL }, 0, Feature::COUNT, k };
    }
}

// Проверки таблицы при компиляции. Рекурсией, а не циклами: ядро
// ESP32 собирается в C++11, где constexpr-функция — один return.
namespace BindingCheck
{
    constexpr bool channelIsFree(uint8_t channel)
    {
        return channel != Channels::AILERON && channel != Channels::ELEVATOR &&
               channel != Channels::THROTTLE && channel != Channels::RUDDER &&
               channel != Channels::ARM && channel < Channels::COUNT;
    }

    template <size_t N>
    constexpr bool channelsFree(const Binding (&table)[N], size_t i = 0)
    {
        return i >= N || (channelIsFree(table[i].channel) && channelsFree(table, i + 1));
    }

    template <size_t N>
    constexpr bool notRepeatedAfter(const Binding (&table)[N], size_t i, size_t j)
    {
        return j >= N || (table[i].channel != table[j].channel && notRepeatedAfter(table, i, j + 1));
    }

    template <size_t N>
    constexpr bool channelsUnique(const Binding (&table)[N], size_t i = 0)
    {
        return i >= N || (notRepeatedAfter(table, i, i + 1) && channelsUnique(table, i + 1));
    }

    template <size_t N>
    constexpr size_t modeSwitchCount(const Binding (&table)[N], size_t i = 0)
    {
        return i >= N ? 0 : (table[i].kind == Binding::Kind::MODES ? 1 : 0) + modeSwitchCount(table, i + 1);
    }

    template <size_t N>
    constexpr bool atMostOneModeSwitch(const Binding (&table)[N])
    {
        return modeSwitchCount(table) <= 1;
    }
}
