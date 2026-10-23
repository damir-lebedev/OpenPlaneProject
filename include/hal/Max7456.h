#pragma once
#include <Arduino.h>

#include "copter/Osd.h"
#include "hal/ISpiBus.h"

// ============================================================
// Чип аналогового OSD — MAX7456 и его клон AT7456E (стоит на большинстве
// полётников F4/F7 под Betaflight). По открытому описанию регистров:
//   VM0 0x00 — режим: бит 6 PAL, бит 3 включить OSD, бит 1 сброс;
//   DMM 0x04 — память экрана: бит 2 очистить;
//   DMAH 0x05 / DMAL 0x06 — адрес символа (строка × 30 + столбец);
//   DMDI 0x07 — номер символа шрифта;
//   OSDBL 0x6C — уровень чёрного: бит 4 = 0 — автоматически;
//   STAT 0xA0 (чтение) — бит 0 PAL, бит 1 NTSC.
// Чтение регистра — адрес | 0x80.
//
// Картинка камеры проходит сквозь чип всегда; OSD — поверх. Чип пишется
// из фоновой задачи (своя шина SPI, не гироскопа): за раз — не больше
// maxChars изменившихся символов, весь экран перерисовывать не нужно.
// Камеру подключили позже или сменили PAL на NTSC — check() раз в секунду
// это заметит и перенастроит чип.
// ============================================================

class Max7456
{
public:
    static constexpr uint32_t SPI_HZ = 8000000;   // чип — до 10 МГц

    Max7456(ISpiBus& spiBus, int16_t csPin) : bus(spiBus), cs(csPin) {}

    // false — чипа нет (регистр не читается обратно).
    bool begin()
    {
        if (cs < 0) return false;
        pinMode(static_cast<uint32_t>(cs), OUTPUT);
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        writeRegister(VM0, 0x02);   // программный сброс
        delay(1);
        pal = detectPal(true);
        configure();
        present = readRegister(VM0) == vm0();
        if (!present) return false;
        const uint8_t black = readRegister(OSDBL_READ);
        writeRegister(OSDBL, static_cast<uint8_t>(black & ~0x10u));   // уровень чёрного — сам
        clearChip();
        return true;
    }

    bool isPresent() const { return present; }
    bool isPal() const { return pal; }
    uint8_t rows() const { return pal ? 16 : 13; }

    // Раз в секунду: стандарт видео поменялся — перенастроить и стереть.
    void check()
    {
        if (!present) return;
        const bool nowPal = detectPal(pal);
        if (nowPal == pal && readRegister(VM0) == vm0()) return;
        pal = nowPal;
        configure();
        clearChip();
    }

    // Изменившиеся символы экрана — в чип; не больше maxChars за вызов.
    // Возвращает, сколько записал.
    uint16_t flush(const copter::OsdScreen& screen, uint16_t maxChars)
    {
        if (!present) return 0;
        uint16_t written = 0;
        for (uint16_t i = 0; i < CELLS && written < maxChars; ++i)
        {
            const uint16_t pos = static_cast<uint16_t>((cursor + i) % CELLS);
            const uint8_t row = static_cast<uint8_t>(pos / copter::OsdScreen::COLS);
            const uint8_t col = static_cast<uint8_t>(pos % copter::OsdScreen::COLS);
            if (row >= rows()) continue;
            const uint8_t c = screen.at(row, col);
            if (shadow[pos] == c) continue;
            writeRegister(DMAH, static_cast<uint8_t>(pos >> 8));
            writeRegister(DMAL, static_cast<uint8_t>(pos));
            writeRegister(DMDI, c);
            shadow[pos] = c;
            ++written;
        }
        cursor = static_cast<uint16_t>((cursor + 37) % CELLS);   // в следующий раз — с другого места
        return written;
    }

private:
    static constexpr uint8_t VM0 = 0x00, DMM = 0x04, DMAH = 0x05, DMAL = 0x06, DMDI = 0x07, OSDBL = 0x6C;
    static constexpr uint8_t OSDBL_READ = 0xEC, STAT = 0xA0;
    static constexpr uint16_t CELLS = copter::OsdScreen::COLS * copter::OsdScreen::MAX_ROWS;

    uint8_t vm0() const { return static_cast<uint8_t>((pal ? 0x40 : 0x00) | 0x08); }

    void configure() { writeRegister(VM0, vm0()); }

    // Нет сигнала — оставить, что было (по умолчанию PAL: у Ratel и большинства камер так).
    bool detectPal(bool fallback)
    {
        const uint8_t stat = readRegister(STAT);
        if (stat & 0x01) return true;
        if (stat & 0x02) return false;
        return fallback;
    }

    void clearChip()
    {
        writeRegister(DMM, 0x04);
        delayMicroseconds(30);
        // после очистки в памяти чипа — символ 0x00, а пусто в шрифте BF — пробел
        // 0x20: flush() допишет пробелы сам (тень «не совпадает» со всем экраном)
        memset(shadow, 0x00, sizeof(shadow));
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        bus.beginTransaction(SPI_HZ, 0);
        digitalWrite(static_cast<uint32_t>(cs), LOW);
        bus.transfer(reg);
        bus.transfer(value);
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        bus.endTransaction();
    }

    uint8_t readRegister(uint8_t reg)
    {
        bus.beginTransaction(SPI_HZ, 0);
        digitalWrite(static_cast<uint32_t>(cs), LOW);
        bus.transfer(static_cast<uint8_t>(reg | 0x80));
        const uint8_t v = bus.transfer(0x00);
        digitalWrite(static_cast<uint32_t>(cs), HIGH);
        bus.endTransaction();
        return v;
    }

    ISpiBus& bus;
    int16_t cs;
    bool present = false;
    bool pal = true;
    uint16_t cursor = 0;
    uint8_t shadow[CELLS] = {};
};
