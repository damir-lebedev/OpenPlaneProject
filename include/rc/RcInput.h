#pragma once
#include <Arduino.h>

#include "config/Config.h"
// ============================================================
// RC INPUT UTILITIES
//
// Общие преобразования RC-сигналов, без знания о конкретном
// самолёте.
// ============================================================

class RcInput
{
public:

    static uint16_t clamp(uint16_t value)
    {
        return constrain(
            value,
            Config::PWM_MIN,
            Config::PWM_MAX
        );
    }

    // 1000 -> -maximumDeflection, 1500 -> 0, 2000 -> +maximumDeflection.
    // reverse инвертирует направление канала.
    static int16_t centered(
        uint16_t input,
        int16_t maximumDeflection,
        bool reverse = false
    )
    {
        input = clamp(input);

        int32_t output = static_cast<int32_t>(map(
            input,
            Config::PWM_MIN,
            Config::PWM_MAX,
            -maximumDeflection,
            maximumDeflection
        ));

        if (reverse)
        {
            output = -output;
        }

        return static_cast<int16_t>(
            constrain(
                output,
                -maximumDeflection,
                maximumDeflection
            )
        );
    }
};