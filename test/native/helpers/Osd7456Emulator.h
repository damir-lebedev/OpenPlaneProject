#pragma once
#include <stdint.h>
#include <string.h>

#include <string>

#include "hal/ISpiBus.h"

// ============================================================
// Чип OSD AT7456E/MAX7456 на шине SPI — для тестов: регистры VM0, DMM,
// DMAH/DMAL/DMDI, OSDBL, STAT; память экрана 30 × 16 символов. Каждая
// транзакция — пара байтов «адрес, данные» (чтение — адрес | 0x80).
// ============================================================

struct Osd7456Emulator : public ISpiBus
{
    bool connected = true;
    bool palSignal = true;          // какая камера: PAL или NTSC
    uint8_t vm0 = 0, dmm = 0, dmah = 0, dmal = 0, osdbl = 0x1F;
    uint8_t memory[480] = {};
    int charWrites = 0;
    int resets = 0;

    void begin() override {}

    void beginTransaction(uint32_t, uint8_t) override
    {
        step = 0;
        address = 0;
    }

    uint8_t transfer(uint8_t data) override
    {
        if (!connected) return 0xFF;
        if (step++ == 0)
        {
            address = data;
            return 0;
        }
        if (address & 0x80) return readRegister(address);
        writeRegister(address, data);
        return 0;
    }

    void endTransaction() override {}

    // Строка экрана как текст (символы шрифта BF для латиницы = ASCII).
    std::string row(uint8_t r) const
    {
        std::string s;
        for (uint8_t c = 0; c < 30; ++c) s += static_cast<char>(memory[r * 30 + c] ? memory[r * 30 + c] : ' ');
        return s;
    }

private:
    uint8_t step = 0;
    uint8_t address = 0;

    uint8_t readRegister(uint8_t a) const
    {
        switch (a)
        {
            case 0x80: return vm0;
            case 0x84: return dmm;
            case 0xEC: return osdbl;
            case 0xA0: return palSignal ? 0x01 : 0x02;
            default: return 0;
        }
    }

    void writeRegister(uint8_t a, uint8_t v)
    {
        switch (a)
        {
            case 0x00:
                if (v & 0x02)
                {
                    ++resets;
                    vm0 = 0;
                    memset(memory, 0, sizeof(memory));
                }
                else
                {
                    vm0 = v;
                }
                break;
            case 0x04:
                dmm = v;
                if (v & 0x04) memset(memory, 0, sizeof(memory));
                break;
            case 0x05: dmah = v; break;
            case 0x06: dmal = v; break;
            case 0x07:
            {
                const uint16_t pos = static_cast<uint16_t>(((dmah & 0x01) << 8) | dmal);
                if (pos < sizeof(memory)) memory[pos] = v;
                ++charWrites;
                break;
            }
            case 0x6C: osdbl = v; break;
            default: break;
        }
    }
};
