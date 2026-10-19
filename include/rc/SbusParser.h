#pragma once
#include <Arduino.h>
#include <string.h>

#include "rc/Packed11.h"

// ============================================================
// РАЗБОР SBUS (FrSky, Futaba, RadioMaster, Jumper и многие другие)
//
// 100000 бод, 8E2, сигнал инвертирован — инверсию делает сам UART
// (UartFormat::SBUS_8E2_INVERTED), внешний инвертор не нужен.
// Кадр 25 байт, каждые 7–14 мс:
//   [0x0F] [22 байта: 16 каналов по 11 бит] [флаги] [конец]
//   флаги: бит 2 — потерян один кадр по радио, бит 3 — FAILSAFE
//          (связи с пультом нет, в каналах — failsafe-значения приёмника);
//   конец: 0x00 у SBUS, 0x04/0x14/0x24/0x34 у SBUS2.
//
// Байт 0x0F встречается и внутри данных, поэтому кадр принимается,
// только когда на своём месте и начало, и конец. Иначе разбор
// продолжается со следующего 0x0F внутри принятого — так поток
// синхронизируется за один-два кадра, даже если подключились посреди.
// ============================================================

class SbusParser
{
public:
    static constexpr uint8_t FRAME_LENGTH = 25;
    static constexpr uint8_t HEADER = 0x0F;
    static constexpr uint8_t FLAG_FRAME_LOST = 0x04;
    static constexpr uint8_t FLAG_FAILSAFE = 0x08;

    // Байт из UART; true — принят кадр (каналы — в values()).
    bool feed(uint8_t value)
    {
        if (index == 0 && value != HEADER) return false;

        frame[index++] = value;
        if (index < FRAME_LENGTH) return false;

        if (validEnd(frame[FRAME_LENGTH - 1]))
        {
            Packed11::unpack(frame + 1, channels);
            flags = frame[FRAME_LENGTH - 2];
            index = 0;
            return true;
        }

        bad++;
        resync();
        return false;
    }

    const uint16_t* values() const { return channels; }
    bool failsafe() const { return (flags & FLAG_FAILSAFE) != 0; }
    bool frameLost() const { return (flags & FLAG_FRAME_LOST) != 0; }
    uint32_t badFrames() const { return bad; }


private:

    uint8_t frame[FRAME_LENGTH] = {};
    uint8_t index = 0;
    uint16_t channels[Packed11::CHANNELS] = {};
    uint8_t flags = 0;
    uint32_t bad = 0;

    static bool validEnd(uint8_t value)
    {
        return value == 0x00 || (value & 0x0F) == 0x04;
    }

    // Начало оказалось байтом данных: продолжаем со следующего 0x0F.
    void resync()
    {
        uint8_t start = 1;
        while (start < FRAME_LENGTH && frame[start] != HEADER) ++start;
        index = static_cast<uint8_t>(FRAME_LENGTH - start);
        memmove(frame, frame + start, index);
    }
};
