#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// MAVLINK 2 — КОДИРОВАНИЕ И РАЗБОР КАДРОВ (без генерируемых библиотек)
//
// MAVLink — протокол телеметрии ArduPilot/PX4: его понимают
// QGroundControl, Mission Planner, радиомодемы SiK, ELRS в режиме
// MAVLink. Проекту нужна дюжина сообщений, поэтому вместо
// сгенерированной библиотеки (сотни КБ заголовков) — упаковка руками.
//
// Кадр v2: FD | len | incompat | compat | seq | sysid | compid |
//          msgid (3 байта LE) | payload | CRC16 (X.25, + CRC_EXTRA)
// Поля payload идут не в порядке XML, а отсортированы по размеру
// типа (8-байтные первыми) — порядок каждой упаковки ниже сверен с
// pymavlink (test_mavlink проверяет кадры им же, если он установлен).
// Хвостовые нули payload в v2 обрезаются — так требует спецификация.
//
// Принимаются кадры v1 (FE) и v2 (FD); подписанные v2 (incompat 0x01)
// тоже, подпись (13 байт) пропускается — не проверяется. CRC можно
// проверить только у сообщений с известным CRC_EXTRA (crcExtraOf()):
// остальные парсер молча пропускает.
// ============================================================

namespace Mavlink
{
    constexpr uint8_t STX_V1 = 0xFE;
    constexpr uint8_t STX_V2 = 0xFD;
    constexpr size_t MAX_PAYLOAD = 255;
    constexpr size_t MAX_FRAME = 10 + MAX_PAYLOAD + 2 + 13;

    // Идентификаторы и CRC_EXTRA (common.xml).
    namespace Msg
    {
        constexpr uint32_t HEARTBEAT = 0;               // extra 50
        constexpr uint32_t SYS_STATUS = 1;              // 124
        constexpr uint32_t SET_MODE = 11;               // 89
        constexpr uint32_t PARAM_REQUEST_READ = 20;     // 214
        constexpr uint32_t PARAM_REQUEST_LIST = 21;     // 159
        constexpr uint32_t PARAM_VALUE = 22;            // 220
        constexpr uint32_t PARAM_SET = 23;              // 168
        constexpr uint32_t GPS_RAW_INT = 24;            // 24
        constexpr uint32_t ATTITUDE = 30;               // 39
        constexpr uint32_t GLOBAL_POSITION_INT = 33;    // 104
        constexpr uint32_t SERVO_OUTPUT_RAW = 36;       // 222
        constexpr uint32_t MISSION_REQUEST_LIST = 43;   // 132
        constexpr uint32_t MISSION_COUNT = 44;          // 221
        constexpr uint32_t NAV_CONTROLLER_OUTPUT = 62;  // 183
        constexpr uint32_t RC_CHANNELS = 65;            // 118
        constexpr uint32_t REQUEST_DATA_STREAM = 66;    // 148
        constexpr uint32_t VFR_HUD = 74;                // 20
        constexpr uint32_t COMMAND_LONG = 76;           // 152
        constexpr uint32_t COMMAND_ACK = 77;            // 143
        constexpr uint32_t HOME_POSITION = 242;         // 104
        constexpr uint32_t STATUSTEXT = 253;            // 83
    }

    // -1 — сообщение неизвестно (CRC не проверить).
    inline int crcExtraOf(uint32_t id)
    {
        switch (id)
        {
            case Msg::HEARTBEAT: return 50;
            case Msg::SYS_STATUS: return 124;
            case Msg::SET_MODE: return 89;
            case Msg::PARAM_REQUEST_READ: return 214;
            case Msg::PARAM_REQUEST_LIST: return 159;
            case Msg::PARAM_VALUE: return 220;
            case Msg::PARAM_SET: return 168;
            case Msg::GPS_RAW_INT: return 24;
            case Msg::ATTITUDE: return 39;
            case Msg::GLOBAL_POSITION_INT: return 104;
            case Msg::SERVO_OUTPUT_RAW: return 222;
            case Msg::MISSION_REQUEST_LIST: return 132;
            case Msg::MISSION_COUNT: return 221;
            case Msg::NAV_CONTROLLER_OUTPUT: return 183;
            case Msg::RC_CHANNELS: return 118;
            case Msg::REQUEST_DATA_STREAM: return 148;
            case Msg::VFR_HUD: return 20;
            case Msg::COMMAND_LONG: return 152;
            case Msg::COMMAND_ACK: return 143;
            case Msg::HOME_POSITION: return 104;
            case Msg::STATUSTEXT: return 83;
            default: return -1;
        }
    }

    // CRC-16/MCRF4XX (X.25), как crc_accumulate() в mavlink/checksum.h.
    inline uint16_t crcAccumulate(uint8_t data, uint16_t crc)
    {
        uint8_t tmp = static_cast<uint8_t>(data ^ static_cast<uint8_t>(crc & 0xFF));
        tmp = static_cast<uint8_t>(tmp ^ static_cast<uint8_t>(tmp << 4));
        return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                     (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
    }

    inline uint16_t crcCalculate(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF)
    {
        for (size_t i = 0; i < length; ++i) crc = crcAccumulate(data[i], crc);
        return crc;
    }


    // Сборка payload: поля добавляются в порядке упаковки MAVLink.
    class Payload
    {
    public:
        Payload& u8(uint8_t v) { return raw(&v, 1); }
        Payload& i8(int8_t v) { return u8(static_cast<uint8_t>(v)); }
        Payload& u16(uint16_t v) { return le(v, 2); }
        Payload& i16(int16_t v) { return u16(static_cast<uint16_t>(v)); }
        Payload& u32(uint32_t v) { return le(v, 4); }
        Payload& i32(int32_t v) { return u32(static_cast<uint32_t>(v)); }
        Payload& u64(uint64_t v) { return le(v, 8); }

        Payload& f32(float v)
        {
            uint32_t bits;
            memcpy(&bits, &v, 4);
            return u32(bits);
        }

        // Строка фиксированной длины: без '\0', если заполнена целиком.
        Payload& chars(const char* text, size_t size)
        {
            for (size_t i = 0; i < size; ++i)
            {
                const char c = (text && *text) ? *text++ : '\0';
                u8(static_cast<uint8_t>(c));
            }
            return *this;
        }

        const uint8_t* data() const { return bytes; }
        size_t size() const { return length; }

    private:
        uint8_t bytes[MAX_PAYLOAD] = {};
        size_t length = 0;

        Payload& raw(const uint8_t* src, size_t n)
        {
            for (size_t i = 0; i < n && length < MAX_PAYLOAD; ++i) bytes[length++] = src[i];
            return *this;
        }

        Payload& le(uint64_t v, size_t n)
        {
            for (size_t i = 0; i < n && length < MAX_PAYLOAD; ++i) bytes[length++] = static_cast<uint8_t>(v >> (8 * i));
            return *this;
        }
    };


    // Кодировщик кадров одной системы/компонента (sysid/compid).
    class Encoder
    {
    public:
        Encoder(uint8_t systemId, uint8_t componentId)
            : sysid(systemId), compid(componentId)
        {
        }

        // Кадр в out (не меньше MAX_FRAME байт), возвращает длину.
        size_t encode(uint8_t* out, uint32_t msgid, const Payload& payload)
        {
            const int extra = crcExtraOf(msgid);
            if (extra < 0) return 0;

            // v2: хвостовые нули не передаются, но хотя бы один байт остаётся.
            size_t length = payload.size();
            while (length > 1 && payload.data()[length - 1] == 0) --length;

            out[0] = STX_V2;
            out[1] = static_cast<uint8_t>(length);
            out[2] = 0;   // incompat_flags
            out[3] = 0;   // compat_flags
            out[4] = sequence++;
            out[5] = sysid;
            out[6] = compid;
            out[7] = static_cast<uint8_t>(msgid);
            out[8] = static_cast<uint8_t>(msgid >> 8);
            out[9] = static_cast<uint8_t>(msgid >> 16);
            memcpy(out + 10, payload.data(), length);

            uint16_t crc = crcCalculate(out + 1, 9 + length);
            crc = crcAccumulate(static_cast<uint8_t>(extra), crc);
            out[10 + length] = static_cast<uint8_t>(crc);
            out[11 + length] = static_cast<uint8_t>(crc >> 8);
            return 12 + length;
        }

        uint8_t systemId() const { return sysid; }
        uint8_t componentId() const { return compid; }

    private:
        uint8_t sysid;
        uint8_t compid;
        uint8_t sequence = 0;
    };


    // Принятое сообщение: payload дополнен нулями до MAX_PAYLOAD
    // (v2 обрезает хвостовые нули — читать можно любые поля).
    struct Message
    {
        uint32_t msgid = 0;
        uint8_t sysid = 0;
        uint8_t compid = 0;
        uint8_t length = 0;
        uint8_t payload[MAX_PAYLOAD] = {};

        uint8_t u8(size_t at) const { return payload[at]; }
        uint16_t u16(size_t at) const { return static_cast<uint16_t>(payload[at] | (payload[at + 1] << 8)); }
        int16_t i16(size_t at) const { return static_cast<int16_t>(u16(at)); }
        uint32_t u32(size_t at) const
        {
            return static_cast<uint32_t>(payload[at]) | (static_cast<uint32_t>(payload[at + 1]) << 8) |
                   (static_cast<uint32_t>(payload[at + 2]) << 16) | (static_cast<uint32_t>(payload[at + 3]) << 24);
        }
        float f32(size_t at) const
        {
            const uint32_t bits = u32(at);
            float v;
            memcpy(&v, &bits, 4);
            return v;
        }
        // Строка фиксированной длины size -> out (size + 1 байт, с '\0').
        void chars(size_t at, size_t size, char* out) const
        {
            memcpy(out, payload + at, size);
            out[size] = '\0';
        }
    };


    // Потоковый разбор: feed() по байту, true — готово сообщение (message()).
    class Parser
    {
    public:
        bool feed(uint8_t byte)
        {
            switch (state)
            {
                case State::IDLE:
                    if (byte == STX_V2 || byte == STX_V1)
                    {
                        v2 = byte == STX_V2;
                        headerLength = v2 ? 9 : 5;
                        position = 0;
                        state = State::HEADER;
                    }
                    return false;

                case State::HEADER:
                    header[position++] = byte;
                    if (position < headerLength) return false;
                    startPayload();
                    return false;

                case State::PAYLOAD:
                    current.payload[position++] = byte;
                    if (position >= current.length)
                    {
                        position = 0;
                        state = State::CHECKSUM;
                    }
                    return false;

                case State::CHECKSUM:
                    crcBytes[position++] = byte;
                    if (position < 2) return false;
                    if (signatureLeft > 0)
                    {
                        state = State::SIGNATURE;
                        return false;
                    }
                    return finish();

                case State::SIGNATURE:
                    if (--signatureLeft == 0) return finish();
                    return false;
            }
            return false;
        }

        const Message& message() const { return ready; }
        uint32_t goodCount() const { return good; }
        uint32_t badCrcCount() const { return badCrc; }


    private:

        enum class State : uint8_t { IDLE, HEADER, PAYLOAD, CHECKSUM, SIGNATURE };

        State state = State::IDLE;
        bool v2 = true;
        uint8_t header[9] = {};
        uint8_t headerLength = 9;
        size_t position = 0;
        uint8_t crcBytes[2] = {};
        uint8_t signatureLeft = 0;
        Message current;
        Message ready;
        uint32_t good = 0;
        uint32_t badCrc = 0;

        void startPayload()
        {
            current = Message();
            current.length = header[0];
            bool signedFrame = false;
            if (v2)
            {
                signedFrame = (header[1] & 0x01) != 0;   // MAVLINK_IFLAG_SIGNED
                current.sysid = header[4];
                current.compid = header[5];
                current.msgid = static_cast<uint32_t>(header[6]) | (static_cast<uint32_t>(header[7]) << 8) |
                                (static_cast<uint32_t>(header[8]) << 16);
            }
            else
            {
                current.sysid = header[2];
                current.compid = header[3];
                current.msgid = header[4];
            }
            signatureLeft = signedFrame ? 13 : 0;
            position = 0;
            state = current.length > 0 ? State::PAYLOAD : State::CHECKSUM;
        }

        bool finish()
        {
            state = State::IDLE;
            position = 0;

            const int extra = crcExtraOf(current.msgid);
            if (extra < 0) return false;   // неизвестное сообщение — не проверить, пропускаем

            uint16_t crc = crcCalculate(header, headerLength);
            crc = crcCalculate(current.payload, current.length, crc);
            crc = crcAccumulate(static_cast<uint8_t>(extra), crc);
            if (crcBytes[0] != static_cast<uint8_t>(crc) || crcBytes[1] != static_cast<uint8_t>(crc >> 8))
            {
                ++badCrc;
                return false;
            }

            ready = current;
            ++good;
            return true;
        }
    };
}
