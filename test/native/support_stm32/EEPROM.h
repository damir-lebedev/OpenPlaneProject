#pragma once

// ============================================================
// Нативная замена EEPROM-эмуляции STM32duino (utility/stm32_eeprom.h):
// "флеш" (8 КБ, стёртый = 0xFF) и буфер в ОЗУ с тем же API.
// fake::eeprom() — чтобы тест видел, что и сколько раз записано, и
// мог испортить образ ("питание пропало во время стирания").
// ============================================================

#include <cstdint>
#include <cstring>

#define E2END 0x1FFF

namespace fake
{
    struct EepromState
    {
        uint8_t flash[E2END + 1];
        uint8_t buffer[E2END + 1];
        unsigned fills = 0;
        unsigned flushes = 0;

        EepromState()
        {
            memset(flash, 0xFF, sizeof(flash));
            memset(buffer, 0xFF, sizeof(buffer));
        }
    };

    inline EepromState& eeprom()
    {
        static EepromState state;
        return state;
    }

    inline void resetEeprom() { eeprom() = EepromState(); }
}

inline void eeprom_buffer_fill()
{
    memcpy(fake::eeprom().buffer, fake::eeprom().flash, sizeof(fake::eeprom().flash));
    fake::eeprom().fills++;
}

inline void eeprom_buffer_flush()
{
    memcpy(fake::eeprom().flash, fake::eeprom().buffer, sizeof(fake::eeprom().flash));
    fake::eeprom().flushes++;
}

inline uint8_t eeprom_buffered_read_byte(const uint32_t pos)
{
    return pos <= E2END ? fake::eeprom().buffer[pos] : 0;
}

inline void eeprom_buffered_write_byte(uint32_t pos, uint8_t value)
{
    if (pos <= E2END) fake::eeprom().buffer[pos] = value;
}
