// Реализация telemetry/MavlinkCodec.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "telemetry/MavlinkCodec.h"

namespace Mavlink
{

int crcExtraOf(uint32_t id)
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

uint16_t crcAccumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = static_cast<uint8_t>(data ^ static_cast<uint8_t>(crc & 0xFF));
    tmp = static_cast<uint8_t>(tmp ^ static_cast<uint8_t>(tmp << 4));
    return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                 (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

uint16_t crcCalculate(const uint8_t* data, size_t length, uint16_t crc)
{
    for (size_t i = 0; i < length; ++i) crc = crcAccumulate(data[i], crc);
    return crc;
}

auto Payload::f32(float v) -> Payload&
{
    uint32_t bits;
    memcpy(&bits, &v, 4);
    return u32(bits);
}

auto Payload::chars(const char* text, size_t size) -> Payload&
{
    for (size_t i = 0; i < size; ++i)
    {
        const char c = (text && *text) ? *text++ : '\0';
        u8(static_cast<uint8_t>(c));
    }
    return *this;
}

auto Payload::raw(const uint8_t* src, size_t n) -> Payload&
{
    for (size_t i = 0; i < n && length < MAX_PAYLOAD; ++i) bytes[length++] = src[i];
    return *this;
}

auto Payload::le(uint64_t v, size_t n) -> Payload&
{
    for (size_t i = 0; i < n && length < MAX_PAYLOAD; ++i) bytes[length++] = static_cast<uint8_t>(v >> (8 * i));
    return *this;
}

Encoder::Encoder(uint8_t systemId, uint8_t componentId)
: sysid(systemId), compid(componentId)
{
}

auto Encoder::encode(uint8_t* out, uint32_t msgid, const Payload& payload) -> size_t
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

auto Message::u32(size_t at) const -> uint32_t
{
    return static_cast<uint32_t>(payload[at]) | (static_cast<uint32_t>(payload[at + 1]) << 8) |
           (static_cast<uint32_t>(payload[at + 2]) << 16) | (static_cast<uint32_t>(payload[at + 3]) << 24);
}

auto Message::f32(size_t at) const -> float
{
    const uint32_t bits = u32(at);
    float v;
    memcpy(&v, &bits, 4);
    return v;
}

auto Message::chars(size_t at, size_t size, char* out) const -> void
{
    memcpy(out, payload + at, size);
    out[size] = '\0';
}

auto Parser::feed(uint8_t byte) -> bool
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

auto Parser::startPayload() -> void
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

auto Parser::finish() -> bool
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

}  // namespace Mavlink
