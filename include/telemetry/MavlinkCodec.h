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
    int crcExtraOf(uint32_t id);

    // CRC-16/MCRF4XX (X.25), как crc_accumulate() в mavlink/checksum.h.
    uint16_t crcAccumulate(uint8_t data, uint16_t crc);

    uint16_t crcCalculate(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF);


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

        Payload& f32(float v);

        // Строка фиксированной длины: без '\0', если заполнена целиком.
        Payload& chars(const char* text, size_t size);

        const uint8_t* data() const { return bytes; }
        size_t size() const { return length; }

    private:
        uint8_t bytes[MAX_PAYLOAD] = {};
        size_t length = 0;

        Payload& raw(const uint8_t* src, size_t n);

        Payload& le(uint64_t v, size_t n);
    };


    // Кодировщик кадров одной системы/компонента (sysid/compid).
    class Encoder
    {
    public:
        Encoder(uint8_t systemId, uint8_t componentId);

        // Кадр в out (не меньше MAX_FRAME байт), возвращает длину.
        size_t encode(uint8_t* out, uint32_t msgid, const Payload& payload);

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
        uint32_t u32(size_t at) const;
        float f32(size_t at) const;
        // Строка фиксированной длины size -> out (size + 1 байт, с '\0').
        void chars(size_t at, size_t size, char* out) const;
    };


    // Потоковый разбор: feed() по байту, true — готово сообщение (message()).
    class Parser
    {
    public:
        bool feed(uint8_t byte);

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

        void startPayload();

        bool finish();
    };
}
