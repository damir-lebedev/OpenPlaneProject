#pragma once
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/IFlashRegion.h"
#include "hal/ISpiBus.h"
#include "hal/Rtos.h"

// ============================================================
// SPI-NOR флеш полётника (W25Q128, W25Q64, M25P16, GD25Q… — у F405 под
// Betaflight это «flash» чёрного ящика) как IFlashRegion для
// BlackBoxStorage: кольцо секторов 4 КБ.
//
// Микросхема опознаётся по JEDEC ID (команда 0x9F): производитель, тип,
// объём 2^N байт. Берётся до 16 МБ — трёхбайтовые адреса (W25Q256 и
// больше работает как 16 МБ). Команды — общие для всех: 0x03 чтение,
// 0x06 разрешить запись, 0x02 страница 256 байт, 0x20 сектор 4 КБ, 0xD8
// блок 64 КБ, 0xC7 вся микросхема, 0x05 состояние (бит 0 — занята).
//
// Запись страницы — до 3 мс, сектор — до 0.4 с, блок — до 2 с, вся
// микросхема W25Q128 — 40–200 с: драйвер ждёт по биту «занята»
// (задача ящика, не полётный цикл; стирание — только на земле, как и для
// SD). erase() целой области — одной командой 0xC7.
//
// Шина может быть общей с OSD: transaction-методы ISpiBus у общей шины
// закрыты мьютексом (LockedSpiBus в main_copter.cpp). CS — свой пин.
// Без Arduino-железа — тесты на ПК с эмулятором микросхемы.
// ============================================================

class SpiNorFlash : public IFlashRegion
{
public:
    static constexpr uint32_t PAGE = 256;
    static constexpr uint32_t SECTOR = 4096;
    static constexpr uint32_t BLOCK = 65536;
    static constexpr uint32_t MAX_BYTES = 16UL * 1024 * 1024;

    SpiNorFlash(ISpiBus& spiBus, int16_t csPin, uint32_t clockHz = 20000000)
        : bus(spiBus), cs(csPin), clock(clockHz)
    {
    }

    // false — пина нет или микросхема не ответила внятным JEDEC ID.
    bool begin()
    {
        bytes = 0;
        jedec = 0;
        if (cs < 0) return false;
        pinMode(static_cast<uint32_t>(cs), OUTPUT);
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        uint8_t id[3] = {};
        command(0x9F, nullptr, 0, id, 3);
        jedec = (static_cast<uint32_t>(id[0]) << 16) | (static_cast<uint32_t>(id[1]) << 8) | id[2];
        if (id[0] == 0x00 || id[0] == 0xFF || id[2] < 0x10 || id[2] > 0x20) return false;
        const uint32_t full = 1UL << (id[2] > 24 ? 24 : id[2]);
        bytes = full > MAX_BYTES ? MAX_BYTES : full;
        return true;
    }

    uint32_t jedecId() const { return jedec; }
    bool present() const { return bytes > 0; }

    // Производитель по JEDEC: для flash_info.
    const char* vendor() const
    {
        switch (jedec >> 16)
        {
            case 0xEF: return "Winbond";
            case 0x20: return "Micron/ST";
            case 0xC8: return "GigaDevice";
            case 0x68: return "Boya";
            case 0x0B: return "XTX";
            case 0x5E: return "Zbit";
            case 0xA1: return "Fudan";
            case 0x85: return "Puya";
            default: return "?";
        }
    }

    // ---- IFlashRegion ----

    uint32_t size() const override { return bytes; }
    uint32_t writeUnit() const override { return PAGE; }

    bool read(uint32_t offset, void* data, size_t length) override
    {
        if (!inRange(offset, length)) return false;
        if (!waitReady(READY_TIMEOUT_MS)) return false;
        const uint8_t a[3] = { addr(offset, 16), addr(offset, 8), addr(offset, 0) };
        command(0x03, a, 3, static_cast<uint8_t*>(data), length);
        return true;
    }

    // Страницами: кусок, что переходит границу страницы, — отдельной командой
    // (иначе микросхема завернула бы его в начало той же страницы).
    bool write(uint32_t offset, const void* data, size_t length) override
    {
        if (!inRange(offset, length)) return false;
        const uint8_t* p = static_cast<const uint8_t*>(data);
        while (length > 0)
        {
            const uint32_t room = PAGE - (offset % PAGE);
            const uint32_t n = length < room ? static_cast<uint32_t>(length) : room;
            if (!waitReady(READY_TIMEOUT_MS)) return false;
            writeEnable();
            const uint8_t a[3] = { addr(offset, 16), addr(offset, 8), addr(offset, 0) };
            transaction([&]() {
                bus.transfer(0x02);
                for (uint8_t b : a) bus.transfer(b);
                for (uint32_t i = 0; i < n; ++i) bus.transfer(p[i]);
            });
            ++pageWrites;
            if (!waitReady(PAGE_TIMEOUT_MS)) return false;
            offset += n;
            p += n;
            length -= n;
        }
        return true;
    }

    // Сектора 4 КБ; выровненные 64 КБ — блоком; вся область — одной командой.
    bool erase(uint32_t offset, uint32_t length) override
    {
        if (offset % SECTOR || length % SECTOR || !inRange(offset, length)) return false;
        if (offset == 0 && length == bytes)
        {
            if (!waitReady(READY_TIMEOUT_MS)) return false;
            writeEnable();
            command(0xC7, nullptr, 0, nullptr, 0);
            ++chipErases;
            return waitReady(CHIP_TIMEOUT_MS);
        }
        while (length > 0)
        {
            const bool block = offset % BLOCK == 0 && length >= BLOCK;
            if (!waitReady(READY_TIMEOUT_MS)) return false;
            writeEnable();
            const uint8_t a[3] = { addr(offset, 16), addr(offset, 8), addr(offset, 0) };
            command(block ? 0xD8 : 0x20, a, 3, nullptr, 0);
            ++eraseCommands;
            if (!waitReady(block ? BLOCK_TIMEOUT_MS : SECTOR_TIMEOUT_MS)) return false;
            const uint32_t step = block ? BLOCK : SECTOR;
            offset += step;
            length -= step;
        }
        return true;
    }

    uint32_t pageWrites = 0, eraseCommands = 0, chipErases = 0, timeouts = 0;

private:
    static constexpr uint32_t READY_TIMEOUT_MS = 3000;
    static constexpr uint32_t PAGE_TIMEOUT_MS = 10;
    static constexpr uint32_t SECTOR_TIMEOUT_MS = 1000;
    static constexpr uint32_t BLOCK_TIMEOUT_MS = 4000;
    static constexpr uint32_t CHIP_TIMEOUT_MS = 400000;

    static uint8_t addr(uint32_t offset, int shift) { return static_cast<uint8_t>(offset >> shift); }

    bool inRange(uint32_t offset, size_t length) const
    {
        return bytes > 0 && offset <= bytes && length <= bytes - offset;
    }

    template <typename F> void transaction(F body)
    {
        bus.beginTransaction(clock, 0);
        digitalWrite(static_cast<uint32_t>(cs), LOW);
        body();
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        bus.endTransaction();
    }

    void command(uint8_t op, const uint8_t* out, size_t outLen, uint8_t* in, size_t inLen)
    {
        transaction([&]() {
            bus.transfer(op);
            for (size_t i = 0; i < outLen; ++i) bus.transfer(out[i]);
            for (size_t i = 0; i < inLen; ++i) in[i] = bus.transfer(0xFF);
        });
    }

    void writeEnable() { command(0x06, nullptr, 0, nullptr, 0); }

    uint8_t status()
    {
        uint8_t s = 0;
        command(0x05, nullptr, 0, &s, 1);
        return s;
    }

    // Ждать, пока микросхема не закончит (бит 0 состояния). Долгое — с
    // паузами по 1 мс, чтобы не забирать процессор у других задач.
    bool waitReady(uint32_t timeoutMs)
    {
        const uint32_t start = millis();
        uint32_t polls = 0;
        while (status() & 0x01)
        {
            if (++polls > 50) Rtos::sleepMs(1);
            if (millis() - start > timeoutMs)
            {
                ++timeouts;
                return false;
            }
        }
        return true;
    }

    ISpiBus& bus;
    int16_t cs;
    uint32_t clock;
    uint32_t bytes = 0;
    uint32_t jedec = 0;
};
