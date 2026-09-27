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

    static uint16_t clamp(uint16_t value);

    // 1000 -> -maximumDeflection, 1500 -> 0, 2000 -> +maximumDeflection.
    // reverse инвертирует направление канала.
    static int16_t centered(
        uint16_t input,
        int16_t maximumDeflection,
        bool reverse = false
    );
};