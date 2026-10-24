#pragma once

// ============================================================
// Эмулятор SPI-NOR флеша (W25Q128 и родня) на фейковой шине SPI — для
// hal/SpiNorFlash.h и чёрного ящика F405. Как настоящая микросхема:
// JEDEC ID, состояние (бит «занята» держится busyPolls опросов после
// записи/стирания), разрешение записи (WEL сбрасывается после команды),
// запись страницы только опускает биты и заворачивается внутри страницы,
// стирание сектора 4 КБ / блока 64 КБ / всей микросхемы — в 0xFF.
// Команда выполняется, когда CS поднят (deselect), — как у железа.
// ============================================================

#include <stdint.h>

#include <vector>

#include <SPI.h>

namespace chips
{
    class SpiNorFlashChip : public fake::SpiDevice
    {
    public:
        explicit SpiNorFlashChip(uint32_t jedecId = 0xEF4018, uint32_t sizeBytes = 16u * 1024 * 1024)
            : jedec(jedecId), mem(sizeBytes, 0xFF)
        {
        }

        void attach(SPIClass& bus, uint8_t csPin) { bus.attach(csPin, this); }

        void select() override
        {
            op = 0;
            pos = 0;
            address = 0;
            data.clear();
        }

        uint8_t transfer(uint8_t mosi) override
        {
            if (pos++ == 0)
            {
                op = mosi;
                return 0xFF;
            }
            const size_t k = pos - 2;   // номер байта после команды
            switch (op)
            {
                case 0x9F: return k < 3 ? static_cast<uint8_t>(jedec >> (16 - 8 * k)) : 0xFF;
                case 0x05:
                {
                    const bool busy = busyLeft > 0;
                    if (busyLeft > 0) --busyLeft;
                    ++statusReads;
                    return static_cast<uint8_t>((busy ? 0x01 : 0x00) | (wel ? 0x02 : 0x00));
                }
                case 0x03:
                    if (k < 3)
                    {
                        address = (address << 8) | mosi;
                        return 0xFF;
                    }
                    if (busyLeft > 0) ++readsWhileBusy;
                    return mem[(address + (k - 3)) % mem.size()];
                case 0x02:
                case 0x20:
                case 0xD8:
                    if (k < 3) address = (address << 8) | mosi;
                    else data.push_back(mosi);
                    return 0xFF;
                default:
                    return 0xFF;
            }
        }

        void deselect() override
        {
            const bool writeOp = op == 0x02 || op == 0x20 || op == 0xD8 || op == 0xC7 || op == 0x60;
            if (op == 0x06) wel = true;
            if (op == 0x04) wel = false;
            if (!writeOp) return;
            if (!wel || busyLeft > 0)
            {
                ++rejected;   // без WREN или пока занята — микросхема команду не выполнит
                return;
            }
            wel = false;
            busyLeft = busyPolls;
            if (op == 0x02)
            {
                const uint32_t page = address & ~0xFFu;
                for (size_t i = 0; i < data.size(); ++i)
                {
                    const uint32_t a = page | ((address + static_cast<uint32_t>(i)) & 0xFFu);   // заворот в странице
                    if (a < mem.size()) mem[a] &= data[i];
                }
                ++programs;
            }
            else if (op == 0x20 || op == 0xD8)
            {
                const uint32_t span = op == 0x20 ? 4096u : 65536u;
                const uint32_t from = address & ~(span - 1);
                for (uint32_t a = from; a < from + span && a < mem.size(); ++a) mem[a] = 0xFF;
                ++(op == 0x20 ? sectorErases : blockErases);
            }
            else
            {
                std::fill(mem.begin(), mem.end(), 0xFF);
                ++chipErases;
            }
        }

        uint32_t jedec;
        std::vector<uint8_t> mem;
        int busyPolls = 3;
        int busyLeft = 0;
        bool wel = false;
        uint32_t programs = 0, sectorErases = 0, blockErases = 0, chipErases = 0, rejected = 0;
        uint32_t statusReads = 0, readsWhileBusy = 0;

    private:
        uint8_t op = 0;
        size_t pos = 0;
        uint32_t address = 0;
        std::vector<uint8_t> data;
    };
}
