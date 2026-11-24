#pragma once

// ============================================================
// Эмулятор SD-карты в режиме SPI на фейковой шине — для hal/SpiSdCard.h и
// чёрного ящика F405 со слотом microSD на SPI (SpeedyBee F405 V3/V4).
// Как настоящая карта: кадр команды из 6 байт (0x40 | номер), ответ R1
// через байт ожидания, CMD8 с эхом у v2, ACMD41 — «занята» readyPolls раз,
// CMD58 с битом CCS у SDHC, CSD v2 (SDHC) или v1 (SDSC), адреса блоками у
// SDHC и байтами у SDSC; чтение CMD17/CMD18 (+CMD12) — маркер 0xFE через
// readLatency байт 0xFF; запись CMD24/CMD25 — ответ 0x05 и busyBytes нулей.
// Данные карты — `data` (под fat_image::format).
// ============================================================

#include <stdint.h>

#include <deque>
#include <vector>

#include <SPI.h>

namespace chips
{
    class SpiSdCardChip : public fake::SpiDevice
    {
    public:
        explicit SpiSdCardChip(uint32_t blocks = 32768, bool highCapacity = true)
            : data(static_cast<size_t>(blocks) * 512, 0), sdhc(highCapacity)
        {
        }

        std::vector<uint8_t> data;
        bool sdhc;
        bool v2 = true;                // ответ на CMD8 (у SDSC v1 — «нет такой команды»)
        bool present = true;           // нет карты — MISO всегда 0xFF
        uint32_t readyPolls = 3;       // сколько ACMD41 карта отвечает «ещё занята»
        uint32_t readLatency = 2;      // байт 0xFF до маркера данных
        uint32_t busyBytes = 4;        // нулей после записанного блока
        bool rejectWrites = false;     // ответ 0x0D (ошибка записи) на каждый блок

        // Статистика для тестов.
        uint32_t blockReads = 0, blockWrites = 0, multiWrites = 0, multiReads = 0, commands = 0;
        uint32_t lastWriteBlock = 0;
        bool sawCmd12 = false;

        void attach(SPIClass& bus, uint8_t csPin) { bus.attach(csPin, this); }

        void select() override { selected = true; }
        void deselect() override { selected = false; }

        uint8_t transfer(uint8_t mosi) override
        {
            if (!present) return 0xFF;
            uint8_t miso = 0xFF;
            if (!out.empty())
            {
                miso = out.front();
                out.pop_front();
                if (out.empty() && streaming) queueReadBlock();   // CMD18: следующий блок
            }
            if (!selected) return 0xFF;
            input(mosi);
            return miso;
        }

    private:
        enum class Mode
        {
            Command,
            WaitToken,   // запись: ждём маркер 0xFE / 0xFC / 0xFD
            Data,        // запись: 512 байт + 2 CRC
        };

        bool selected = false;
        Mode mode = Mode::Command;
        uint8_t frame[6] = {};
        uint8_t framePos = 0;
        bool idle = true;
        bool app = false;
        uint32_t acmd41Count = 0;
        std::deque<uint8_t> out;
        bool streaming = false;   // CMD18
        uint32_t streamBlock = 0;
        bool multiWrite = false;
        uint32_t writeBlock = 0;
        uint32_t dataPos = 0;
        uint8_t buffer[514] = {};

        uint32_t blockOf(uint32_t arg) const { return sdhc ? arg : arg / 512; }
        uint32_t blocks() const { return static_cast<uint32_t>(data.size() / 512); }

        void r1(uint8_t value)
        {
            out.push_back(0xFF);   // Ncr
            out.push_back(value);
        }

        void queueReadBlock()
        {
            if (streamBlock >= blocks())
            {
                streaming = false;
                out.push_back(0x08);   // маркер ошибки: вне диапазона
                return;
            }
            for (uint32_t i = 0; i < readLatency; ++i) out.push_back(0xFF);
            out.push_back(0xFE);
            const size_t base = static_cast<size_t>(streamBlock) * 512;
            for (size_t i = 0; i < 512; ++i) out.push_back(data[base + i]);
            out.push_back(0x00);
            out.push_back(0x00);
            ++streamBlock;
            ++blockReads;
        }

        void input(uint8_t mosi)
        {
            switch (mode)
            {
                case Mode::WaitToken:
                    if (mosi == 0xFE || mosi == 0xFC)
                    {
                        mode = Mode::Data;
                        dataPos = 0;
                    }
                    else if (mosi == 0xFD && multiWrite)
                    {
                        multiWrite = false;
                        mode = Mode::Command;
                        out.push_back(0xFF);
                        for (uint32_t i = 0; i < busyBytes; ++i) out.push_back(0x00);
                    }
                    return;
                case Mode::Data:
                    buffer[dataPos++] = mosi;
                    if (dataPos == 514) finishWriteBlock();
                    return;
                case Mode::Command:
                    break;
            }
            if (framePos == 0 && (mosi & 0xC0) != 0x40) return;
            frame[framePos++] = mosi;
            if (framePos == 6)
            {
                framePos = 0;
                execute();
            }
        }

        void finishWriteBlock()
        {
            if (rejectWrites || writeBlock >= blocks())
            {
                out.push_back(0x0D);
                multiWrite = false;
                mode = Mode::Command;
                return;
            }
            std::copy(buffer, buffer + 512, data.begin() + static_cast<std::ptrdiff_t>(writeBlock) * 512);
            lastWriteBlock = writeBlock;
            ++writeBlock;
            ++blockWrites;
            out.push_back(0x05);
            for (uint32_t i = 0; i < busyBytes; ++i) out.push_back(0x00);
            mode = multiWrite ? Mode::WaitToken : Mode::Command;
        }

        void execute()
        {
            ++commands;
            const uint8_t cmd = frame[0] & 0x3F;
            const uint32_t arg = (static_cast<uint32_t>(frame[1]) << 24) | (static_cast<uint32_t>(frame[2]) << 16) |
                                 (static_cast<uint32_t>(frame[3]) << 8) | frame[4];
            const uint8_t st = idle ? 0x01 : 0x00;
            const bool wasApp = app;
            app = false;
            if (cmd == 12)
            {
                sawCmd12 = true;
                streaming = false;
                out.clear();
                out.push_back(0xFF);
                out.push_back(0x00);   // байт после CMD12
                out.push_back(0x00);   // R1
                for (uint32_t i = 0; i < busyBytes; ++i) out.push_back(0x00);
                return;
            }
            if (wasApp && cmd == 41)
            {
                if (++acmd41Count > readyPolls) idle = false;
                r1(idle ? 0x01 : 0x00);
                return;
            }
            switch (cmd)
            {
                case 0:
                    if (frame[5] != 0x95) return r1(0x09);   // CRC обязателен
                    idle = true;
                    acmd41Count = 0;
                    return r1(0x01);
                case 8:
                    if (!v2) return r1(0x05);   // IDLE | ILLEGAL
                    r1(st);
                    out.push_back(0x00);
                    out.push_back(0x00);
                    out.push_back(static_cast<uint8_t>((arg >> 8) & 0x0F));
                    out.push_back(static_cast<uint8_t>(arg & 0xFF));
                    return;
                case 55:
                    app = true;
                    return r1(st);
                case 58:
                    r1(st);
                    out.push_back(static_cast<uint8_t>(0x80 | (sdhc && !idle ? 0x40 : 0x00)));
                    out.push_back(0xFF);
                    out.push_back(0x80);
                    out.push_back(0x00);
                    return;
                case 16:
                    return r1(arg == 512 ? st : 0x40);
                case 9:
                {
                    r1(st);
                    uint8_t csd[16] = {};
                    if (sdhc)
                    {
                        const uint32_t cSize = blocks() / 1024 - 1;
                        csd[0] = 0x40;
                        csd[7] = static_cast<uint8_t>((cSize >> 16) & 0x3F);
                        csd[8] = static_cast<uint8_t>(cSize >> 8);
                        csd[9] = static_cast<uint8_t>(cSize);
                    }
                    else
                    {
                        // READ_BL_LEN = 9 (512), C_SIZE_MULT = 7 (×512): блоков = (C_SIZE+1) × 512
                        const uint32_t cSize = blocks() / 512 - 1;
                        csd[0] = 0x00;
                        csd[5] = 0x09;
                        csd[6] = static_cast<uint8_t>((cSize >> 10) & 0x03);
                        csd[7] = static_cast<uint8_t>(cSize >> 2);
                        csd[8] = static_cast<uint8_t>((cSize & 0x03) << 6);
                        csd[9] = 0x03;
                        csd[10] = 0x80;
                    }
                    out.push_back(0xFF);
                    out.push_back(0xFE);
                    for (uint8_t b : csd) out.push_back(b);
                    out.push_back(0x00);
                    out.push_back(0x00);
                    return;
                }
                case 17:
                case 18:
                    if (idle) return r1(0x05);
                    r1(0x00);
                    streamBlock = blockOf(arg);
                    if (cmd == 18) ++multiReads;
                    queueReadBlock();
                    streaming = cmd == 18;
                    return;
                case 24:
                case 25:
                    if (idle) return r1(0x05);
                    r1(0x00);
                    writeBlock = blockOf(arg);
                    multiWrite = cmd == 25;
                    if (multiWrite) ++multiWrites;
                    mode = Mode::WaitToken;
                    return;
                default:
                    return r1(static_cast<uint8_t>(st | 0x04));
            }
        }
    };
}
