#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// ГАЗ -> ИМПУЛЬС НА ESC
//
// Газ в прошивке — PWM_MIN..PWM_MAX (1000..2000 мкс = 0..100%)
// плюс направление мотора. Импульс на пине ESC зависит от того,
// какой регулятор стоит (Config::ESC_BIDIRECTIONAL):
//
//   • двунаправленный (BLHeli 80A в режиме Bidirectional/3D):
//       0%        -> ESC_NEUTRAL_US (1500) — мотор стоит;
//       вперёд    -> 1500..2000;
//       назад     -> 1500..1000.
//     Мёртвую зону вокруг нейтрали держит сам ESC (настройка
//     BLHeliSuite), здесь — линейно.
//   • однонаправленный: 1000..2000 как есть, направление
//     игнорируется (крутить назад такой ESC не умеет).
//
// Только перевод — без защит. Когда направление вообще можно менять,
// решает MotorReverse, предел газа назад — FlightController.
// ============================================================

namespace EscSignal
{
    inline uint16_t pulseUs(uint16_t throttleUs, bool reverse)
    {
        const uint32_t t = constrain(throttleUs, Config::PWM_MIN, Config::PWM_MAX) - Config::PWM_MIN;
        const uint32_t span = Config::PWM_MAX - Config::PWM_MIN;

        if (!Config::ESC_BIDIRECTIONAL) return static_cast<uint16_t>(Config::PWM_MIN + t);

        if (reverse)
        {
            return static_cast<uint16_t>(Config::ESC_NEUTRAL_US - t * (Config::ESC_NEUTRAL_US - Config::PWM_MIN) / span);
        }
        return static_cast<uint16_t>(Config::ESC_NEUTRAL_US + t * (Config::PWM_MAX - Config::ESC_NEUTRAL_US) / span);
    }

    // Импульс "мотор стоит".
    inline uint16_t stopUs()
    {
        return pulseUs(Config::PWM_MIN, false);
    }
}
