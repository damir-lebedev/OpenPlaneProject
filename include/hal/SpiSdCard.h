#pragma once
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/IBlockDevice.h"
#include "hal/ISpiBus.h"
#include "hal/Rtos.h"

// ============================================================
// SD-карта по SPI (режим SPI из стандарта SD Physical Layer, упрощённая
// спецификация) как IBlockDevice — для чёрного ящика F405-полётников, у
// которых слот microSD висит на SPI (SpeedyBee F405 V3/V4 и т. п., у BF —
// `sdcard_mode = SPI`). Поверх — тот же SdFileRegion (файл BLACKBOX.BIN на
// FAT32), что и у H743 с SDMMC.
//
// Запуск: CS поднят, 80 тактов на 400 кГц → CMD0 (в SPI-режим) → CMD8
// (карта v2? — эхо 0x1AA) → ACMD41 до готовности (с HCS у v2) → CMD58
// (бит CCS: SDHC/SDXC адресуются блоками, SDSC — байтами) → CMD16 512 у
// SDSC → CMD9 (CSD: объём) → рабочая частота.
// Чтение: CMD17 (блок) / CMD18 + CMD12 (подряд). Запись: CMD24 / CMD25 с
// маркерами 0xFC и 0xFD; после каждого блока карта держит MISO в нуле,
// пока пишет, — драйвер ждёт 0xFF (задача ящика, не полётный цикл).
// CRC не проверяются (в SPI-режиме выключены по умолчанию), кроме
// обязательных CMD0/CMD8.
// Без железа — тесты на ПК с эмулятором карты (test/native/helpers/SpiSdCardChip.h).
// ============================================================

class SpiSdCard : public IBlockDevice
{
public:
    // Коды initError(): на каком шаге запуска карта не ответила.
    enum InitError : uint8_t
    {
        INIT_OK = 0,
        INIT_NO_PIN = 1,
        INIT_NO_IDLE = 2,      // CMD0 — карты нет
        INIT_BAD_ECHO = 3,     // CMD8 — эхо не то
        INIT_NOT_READY = 4,    // ACMD41 — не вышла из ожидания за секунду
        INIT_NO_OCR = 5,       // CMD58
        INIT_NO_CSD = 6,       // CMD9
        INIT_BLOCK_LEN = 7,    // CMD16 у SDSC
    };

    SpiSdCard(ISpiBus& spiBus, int16_t csPin, uint32_t clockHz = 21000000)
        : bus(spiBus), cs(csPin), clock(clockHz)
    {
    }

    // false — карты нет или она не опозналась; код — initError().
    bool begin()
    {
        blocks = 0;
        sdhc = false;
        error = INIT_OK;
        if (cs < 0) return fail(INIT_NO_PIN);
        pinMode(static_cast<uint32_t>(cs), OUTPUT);
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        speed = INIT_CLOCK;

        // ≥ 74 тактов при поднятом CS — карта переходит в рабочее состояние
        bus.beginTransaction(speed, 0);
        for (int i = 0; i < 10; ++i) bus.transfer(0xFF);
        bus.endTransaction();

        uint8_t r = 0xFF;
        for (int attempt = 0; attempt < 10 && r != R1_IDLE; ++attempt) r = command(0, 0);
        if (r != R1_IDLE) return fail(INIT_NO_IDLE);

        bool v2 = false;
        {
            uint8_t echo[4] = {};
            r = command(8, 0x1AA, echo, 4);
            if (r == R1_IDLE)
            {
                if (echo[2] != 0x01 || echo[3] != 0xAA) return fail(INIT_BAD_ECHO);
                v2 = true;
            }
            else if (!(r & R1_ILLEGAL))
                return fail(INIT_BAD_ECHO);
        }

        const uint32_t start = millis();
        uint32_t polls = 0;
        for (;;)
        {
            r = appCommand(41, v2 ? 0x40000000UL : 0);
            if (r == 0) break;
            if (r != R1_IDLE || millis() - start > READY_TIMEOUT_MS) return fail(INIT_NOT_READY);
            if (++polls > 10) Rtos::sleepMs(1);
        }

        if (v2)
        {
            uint8_t ocr[4] = {};
            if (command(58, 0, ocr, 4) != 0) return fail(INIT_NO_OCR);
            sdhc = (ocr[0] & 0x40) != 0;
        }
        if (!sdhc && command(16, BLOCK_SIZE) != 0) return fail(INIT_BLOCK_LEN);

        uint8_t csd[16] = {};
        if (!readRegister(9, csd)) return fail(INIT_NO_CSD);
        blocks = capacityBlocks(csd);
        if (blocks == 0) return fail(INIT_NO_CSD);
        speed = clock;
        return true;
    }

    bool highCapacity() const { return sdhc; }
    uint8_t initError() const { return error; }
    uint32_t timeoutCount() const { return timeouts; }

    // ---- IBlockDevice ----

    uint32_t blockCount() const override { return blocks; }

    bool read(uint32_t block, void* data, uint32_t count) override
    {
        if (!inRange(block, count)) return false;
        uint8_t* out = static_cast<uint8_t*>(data);
        if (count == 1) return readSingle(block, out);
        bool ok = true;
        transaction([&]() {
            if (send(18, address(block)) != 0)
            {
                ok = false;
                return;
            }
            for (uint32_t i = 0; i < count && ok; ++i) ok = receiveData(out + i * BLOCK_SIZE, BLOCK_SIZE);
            send(12, 0);
            bus.transfer(0xFF);    // байт после CMD12 — мусор
            response();
            ok = waitIdle(WRITE_TIMEOUT_MS) && ok;
        });
        return ok;
    }

    bool write(uint32_t block, const void* data, uint32_t count) override
    {
        if (!inRange(block, count)) return false;
        const uint8_t* in = static_cast<const uint8_t*>(data);
        bool ok = true;
        transaction([&]() {
            if (!waitIdle(WRITE_TIMEOUT_MS) || send(count == 1 ? 24 : 25, address(block)) != 0)
            {
                ok = false;
                return;
            }
            const uint8_t token = count == 1 ? TOKEN_SINGLE : TOKEN_MULTI;
            for (uint32_t i = 0; i < count && ok; ++i) ok = sendData(token, in + i * BLOCK_SIZE);
            if (count > 1)
            {
                bus.transfer(TOKEN_STOP);
                bus.transfer(0xFF);
                ok = waitIdle(WRITE_TIMEOUT_MS) && ok;
            }
        });
        return ok;
    }

private:
    static constexpr uint32_t INIT_CLOCK = 400000;
    static constexpr uint32_t READY_TIMEOUT_MS = 1000;
    static constexpr uint32_t READ_TIMEOUT_MS = 200;
    static constexpr uint32_t WRITE_TIMEOUT_MS = 500;
    static constexpr uint8_t R1_IDLE = 0x01;
    static constexpr uint8_t R1_ILLEGAL = 0x04;
    static constexpr uint8_t TOKEN_SINGLE = 0xFE;   // и маркер начала данных при чтении
    static constexpr uint8_t TOKEN_MULTI = 0xFC;
    static constexpr uint8_t TOKEN_STOP = 0xFD;

    bool fail(uint8_t code)
    {
        error = code;
        blocks = 0;
        return false;
    }

    bool inRange(uint32_t block, uint32_t count) const
    {
        return blocks > 0 && count > 0 && block < blocks && count <= blocks - block;
    }

    uint32_t address(uint32_t block) const { return sdhc ? block : block * BLOCK_SIZE; }

    template <typename F> void transaction(F body)
    {
        bus.beginTransaction(speed, 0);
        digitalWrite(static_cast<uint32_t>(cs), LOW);
        body();
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        bus.transfer(0xFF);   // карта отпускает MISO только с тактом после CS
        bus.endTransaction();
    }

    // Команда целиком: CS, кадр, R1 и (если надо) ещё `extra` байт ответа (R3/R7).
    uint8_t command(uint8_t cmd, uint32_t arg, uint8_t* extra = nullptr, size_t extraLen = 0)
    {
        uint8_t r = 0xFF;
        transaction([&]() {
            r = send(cmd, arg);
            for (size_t i = 0; i < extraLen; ++i) extra[i] = bus.transfer(0xFF);
        });
        return r;
    }

    uint8_t appCommand(uint8_t cmd, uint32_t arg)
    {
        command(55, 0);
        return command(cmd, arg);
    }

    // Кадр команды внутри открытой транзакции; ответ R1 (старший бит 0) — до 8 байт ожидания.
    uint8_t send(uint8_t cmd, uint32_t arg)
    {
        bus.transfer(static_cast<uint8_t>(0x40 | cmd));
        bus.transfer(static_cast<uint8_t>(arg >> 24));
        bus.transfer(static_cast<uint8_t>(arg >> 16));
        bus.transfer(static_cast<uint8_t>(arg >> 8));
        bus.transfer(static_cast<uint8_t>(arg));
        bus.transfer(cmd == 0 ? 0x95 : cmd == 8 ? 0x87 : 0x01);
        return response();
    }

    uint8_t response()
    {
        uint8_t r = 0xFF;
        for (int i = 0; i < 8 && (r & 0x80); ++i) r = bus.transfer(0xFF);
        return r;
    }

    // Ждать, пока карта отпустит MISO (0xFF) после записи. Долгая запись (карта
    // занята до сотен мс) — шину на время сна отпускаем: CS вверх, транзакция
    // закрыта, другие на этой SPI (OSD у SpeedyBee F405 V3) работают; занятость
    // карта держит и без CS и снова покажет её, когда CS опустится.
    bool waitIdle(uint32_t timeoutMs)
    {
        const uint32_t start = millis();
        uint32_t polls = 0;
        while (bus.transfer(0xFF) != 0xFF)
        {
            if (++polls > 512)
            {
                digitalWrite(static_cast<uint32_t>(cs), HIGH);
                bus.transfer(0xFF);
                bus.endTransaction();
                Rtos::sleepMs(1);
                bus.beginTransaction(speed, 0);
                digitalWrite(static_cast<uint32_t>(cs), LOW);
            }
            if (millis() - start > timeoutMs)
            {
                ++timeouts;
                return false;
            }
        }
        return true;
    }

    // Блок данных от карты: маркер 0xFE, n байт, CRC (2 байта, не проверяется).
    bool receiveData(uint8_t* out, size_t n)
    {
        const uint32_t start = millis();
        uint32_t polls = 0;
        uint8_t t = 0xFF;
        while ((t = bus.transfer(0xFF)) == 0xFF)
        {
            if (++polls > 512) Rtos::sleepMs(1);
            if (millis() - start > READ_TIMEOUT_MS)
            {
                ++timeouts;
                return false;
            }
        }
        if (t != TOKEN_SINGLE) return false;   // маркер ошибки чтения (0000xxxx)
        for (size_t i = 0; i < n; ++i) out[i] = bus.transfer(0xFF);
        bus.transfer(0xFF);
        bus.transfer(0xFF);
        return true;
    }

    // Блок данных карте; ответ xxx0_0101 — принят; потом ждать, пока запишет.
    bool sendData(uint8_t token, const uint8_t* in)
    {
        bus.transfer(0xFF);
        bus.transfer(token);
        for (uint32_t i = 0; i < BLOCK_SIZE; ++i) bus.transfer(in[i]);
        bus.transfer(0xFF);
        bus.transfer(0xFF);
        const uint8_t resp = bus.transfer(0xFF);
        if ((resp & 0x1F) != 0x05) return false;
        return waitIdle(WRITE_TIMEOUT_MS);
    }

    bool readSingle(uint32_t block, uint8_t* out)
    {
        bool ok = false;
        transaction([&]() { ok = send(17, address(block)) == 0 && receiveData(out, BLOCK_SIZE); });
        return ok;
    }

    bool readRegister(uint8_t cmd, uint8_t* out16)
    {
        bool ok = false;
        transaction([&]() { ok = send(cmd, 0) == 0 && receiveData(out16, 16); });
        return ok;
    }

    // Объём по CSD: версия 2 (SDHC/SDXC) — (C_SIZE + 1) × 512 КБ; версия 1 —
    // (C_SIZE + 1) × 2^(C_SIZE_MULT + 2) × 2^READ_BL_LEN байт.
    static uint32_t capacityBlocks(const uint8_t* csd)
    {
        const uint8_t structure = csd[0] >> 6;
        if (structure == 1)
        {
            const uint32_t cSize = (static_cast<uint32_t>(csd[7] & 0x3F) << 16) | (static_cast<uint32_t>(csd[8]) << 8) | csd[9];
            return (cSize + 1) * 1024;
        }
        if (structure == 0)
        {
            const uint32_t readBlLen = csd[5] & 0x0F;
            const uint32_t cSize = (static_cast<uint32_t>(csd[6] & 0x03) << 10) | (static_cast<uint32_t>(csd[7]) << 2) | (csd[8] >> 6);
            const uint32_t mult = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
            const uint64_t bytes = static_cast<uint64_t>(cSize + 1) << (mult + 2 + readBlLen);
            return static_cast<uint32_t>(bytes / BLOCK_SIZE);
        }
        return 0;
    }

    ISpiBus& bus;
    int16_t cs;
    uint32_t clock;
    uint32_t speed = INIT_CLOCK;
    uint32_t blocks = 0;
    bool sdhc = false;
    uint8_t error = INIT_OK;
    uint32_t timeouts = 0;
};
