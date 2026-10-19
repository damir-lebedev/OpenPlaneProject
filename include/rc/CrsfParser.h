#pragma once
#include <Arduino.h>

#include "rc/Packed11.h"

// ============================================================
// РАЗБОР CRSF (ExpressLRS, TBS Crossfire / Tracer)
//
// 420000 бод 8N1. Кадр: [адрес][длина][тип][данные][CRC8]
//   адрес — 0xC8 (полётнику); длина — тип + данные + CRC, кадр целиком
//   не длиннее 64 байт; CRC8 (полином 0xD5, DVB-S2) — по типу и данным.
// Нужны два типа:
//   0x16 — каналы: 16 по 11 бит, как в SBUS (Packed11);
//   0x14 — статистика связи: третий байт — качество связи (LQ), %.
//
// Потеря связи: ExpressLRS по умолчанию просто перестаёт слать каналы
// (RcReceiver видит таймаут кадров); в режиме «последние значения» или
// «свои значения» каналы идут, но LQ падает до 0 — это failsafe().
// ============================================================

class CrsfParser
{
public:
    static constexpr uint8_t ADDRESS_FLIGHT_CONTROLLER = 0xC8;
    static constexpr uint8_t TYPE_LINK_STATISTICS = 0x14;
    static constexpr uint8_t TYPE_RC_CHANNELS = 0x16;
    static constexpr uint8_t MAX_FRAME = 64;

    // Байт из UART; true — пришли каналы (кадр 0x16 с верной CRC).
    bool feed(uint8_t value)
    {
        if (index == 0)
        {
            if (value == ADDRESS_FLIGHT_CONTROLLER) frame[index++] = value;
            return false;
        }

        if (index == 1)
        {
            // тип + CRC — минимум 2; адрес и длина + остальное — не больше 64
            if (value < 2 || value > MAX_FRAME - 2)
            {
                index = (value == ADDRESS_FLIGHT_CONTROLLER) ? 1 : 0;
                return false;
            }
            frame[index++] = value;
            return false;
        }

        frame[index++] = value;
        if (index < frame[1] + 2) return false;

        index = 0;
        return decode();
    }

    const uint16_t* values() const { return channels; }

    // Приёмник сообщил: связи с пультом нет (качество связи 0 %).
    bool failsafe() const { return statsSeen && quality == 0; }

    // Качество связи, % (−1 — статистики ещё не было).
    int16_t linkQuality() const { return statsSeen ? quality : -1; }

    uint32_t badFrames() const { return bad; }

    static uint8_t crc8(const uint8_t* data, uint8_t length)
    {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
            }
        }
        return crc;
    }


private:

    uint8_t frame[MAX_FRAME] = {};
    uint8_t index = 0;
    uint16_t channels[Packed11::CHANNELS] = {};
    uint8_t quality = 0;
    bool statsSeen = false;
    uint32_t bad = 0;

    bool decode()
    {
        const uint8_t length = frame[1];                 // тип + данные + CRC
        if (crc8(frame + 2, static_cast<uint8_t>(length - 1)) != frame[length + 1])
        {
            bad++;
            return false;
        }

        const uint8_t type = frame[2];
        const uint8_t* payload = frame + 3;
        const uint8_t payloadLength = static_cast<uint8_t>(length - 2);

        if (type == TYPE_RC_CHANNELS && payloadLength == Packed11::BYTES)
        {
            Packed11::unpack(payload, channels);
            return true;
        }
        if (type == TYPE_LINK_STATISTICS && payloadLength >= 3)
        {
            quality = payload[2];                        // uplink LQ, %
            statsSeen = true;
        }
        return false;
    }
};
