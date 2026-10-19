#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// РАЗБОР iBUS (FlySky: FS-iA6B, FS-iA10B)
//
// 115200 бод 8N1. Кадр 32 байта:
//   [0x20][0x40] [CH1 low][CH1 high] ... [CH14 low][CH14 high] [CRC low][CRC high]
//   CRC = 0xFFFF − (сумма первых 30 байт).
// Значение канала — сразу микросекунды, младшие 12 бит.
//
// Флага failsafe в iBUS нет: FS-iA6B при потере связи продолжает слать
// кадры, а о потере сообщает газом ниже нормы — это проверяет
// RcReceiver (RX_FAILSAFE_THROTTLE_US).
// ============================================================

class IBusParser
{
public:
    static constexpr uint8_t CHANNELS = 14;

    // Байт из UART; true — собран кадр с верной CRC (каналы — в values()).
    // Байты приходят порциями, поэтому разбор побайтовый.
    bool feed(uint8_t value)
    {
        if (index == 0)
        {
            if (value == Config::IBUS_HEADER_0) frame[index++] = value;
            return false;
        }

        if (index == 1)
        {
            if (value != Config::IBUS_HEADER_1)
            {
                // 0x20 был случайным, начинаем заново — но этот байт сам
                // может быть началом кадра (0x20 0x20 0x40 ...).
                index = (value == Config::IBUS_HEADER_0) ? 1 : 0;
                return false;
            }
            frame[index++] = value;
            return false;
        }

        frame[index++] = value;
        if (index < Config::IBUS_FRAME_LENGTH) return false;

        index = 0;
        return decode();
    }

    const uint16_t* values() const { return channels; }
    bool failsafe() const { return false; }
    uint32_t badFrames() const { return bad; }


private:

    uint8_t frame[Config::IBUS_FRAME_LENGTH] = {};
    uint8_t index = 0;
    uint16_t channels[CHANNELS] = {};
    uint32_t bad = 0;

    bool decode()
    {
        uint16_t checksum = 0xFFFF;
        for (uint8_t i = 0; i < Config::IBUS_FRAME_LENGTH - 2; ++i)
        {
            checksum -= frame[i];
        }

        const uint16_t received = static_cast<uint16_t>(frame[Config::IBUS_FRAME_LENGTH - 2]) |
                                  (static_cast<uint16_t>(frame[Config::IBUS_FRAME_LENGTH - 1]) << 8);
        if (checksum != received)
        {
            bad++;
            return false;   // помехи на линии — кадр не берём
        }

        for (uint8_t ch = 0; ch < CHANNELS; ++ch)
        {
            // Значение канала — только младшие 12 бит. В старших 4 битах
            // FS-iA6B передаёт служебные данные (так кодируются каналы
            // 15-18), и они не нулевые, например, в failsafe: на стенде
            // CH3 = 0x2384 -> 900 мкс, а без маски читалось 9092, и
            // failsafe по газу (< 950) не срабатывал.
            channels[ch] = static_cast<uint16_t>((frame[2 + ch * 2] | (frame[3 + ch * 2] << 8)) & 0x0FFF);
        }
        return true;
    }
};
