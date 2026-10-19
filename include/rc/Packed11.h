#pragma once
#include <Arduino.h>

// ============================================================
// 16 КАНАЛОВ ПО 11 БИТ
//
// Так каналы пакуют и SBUS, и CRSF: 22 байта подряд, младшими битами
// вперёд (CH1 — биты 0–10, CH2 — 11–21, ...). Значение 172…1811
// соответствует 988…2012 мкс, 992 — центру 1500:
//   мкс = 5/8 · значение + 880   (как в Betaflight и INAV).
// ============================================================

namespace Packed11
{
    constexpr uint8_t CHANNELS = 16;
    constexpr uint8_t BYTES = 22;

    inline uint16_t toMicros(uint16_t raw)
    {
        return static_cast<uint16_t>(880 + (static_cast<uint32_t>(raw) * 5 + 4) / 8);
    }

    // data — BYTES байт; out — CHANNELS значений в мкс.
    inline void unpack(const uint8_t* data, uint16_t* out)
    {
        uint32_t bits = 0;
        uint8_t count = 0;
        uint8_t next = 0;
        for (uint8_t ch = 0; ch < CHANNELS; ++ch)
        {
            while (count < 11)
            {
                bits |= static_cast<uint32_t>(data[next++]) << count;
                count += 8;
            }
            out[ch] = toMicros(static_cast<uint16_t>(bits & 0x7FF));
            bits >>= 11;
            count -= 11;
        }
    }
}
