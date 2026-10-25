#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// MSP (MultiWii Serial Protocol) — протокол конфигуратора Betaflight.
// Своя реализация по открытому описанию протокола.
//
//   v1:  '$' 'M' '<' | размер u8 | команда u8 | данные | xor(размер, команда, данные)
//   v2:  '$' 'X' '<' | флаг u8 | команда u16 | размер u16 | данные | CRC-8 DVB-S2
//        (полином 0xD5, по флагу, команде, размеру и данным)
// Ответ — тот же вид с '>' (или '!' — ошибка: команда не понята или
// отклонена). Числа — младший байт первым.
//
// Здесь — только кадры: разбор входящих (Parser), сборка ответа (frame),
// чтение и запись полей (Reader, Writer). Что отвечать — CopterMsp.h.
// Без Arduino — тесты на ПК.
// ============================================================

namespace msp
{
    constexpr uint16_t MAX_PAYLOAD = 512;

    inline uint8_t crc8DvbS2(uint8_t crc, uint8_t b)
    {
        crc ^= b;
        for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
        return crc;
    }

    // Входящие кадры по байту. feed() -> true — кадр целиком и верен.
    class Parser
    {
    public:
        bool feed(uint8_t b)
        {
            switch (state)
            {
                case State::Idle:
                    if (b == '$') state = State::Proto;
                    return false;
                case State::Proto:
                    if (b == 'M') version = 1;
                    else if (b == 'X') version = 2;
                    else
                    {
                        state = State::Idle;
                        return false;
                    }
                    state = State::Direction;
                    return false;
                case State::Direction:
                    state = b == '<' ? (version == 1 ? State::V1Size : State::V2Flag) : State::Idle;
                    return false;
                case State::V1Size:
                    size = b;
                    check = b;
                    state = State::V1Cmd;
                    return false;
                case State::V1Cmd:
                    cmd = b;
                    check ^= b;
                    received = 0;
                    state = size ? State::V1Data : State::V1Check;
                    return false;
                case State::V1Data:
                    payload[received++] = b;
                    check ^= b;
                    if (received >= size) state = State::V1Check;
                    return false;
                case State::V1Check:
                    state = State::Idle;
                    if (b != check)
                    {
                        ++errors;
                        return false;
                    }
                    return true;
                case State::V2Flag:
                    flag = b;
                    crc = crc8DvbS2(0, b);
                    head = 0;
                    state = State::V2Head;
                    return false;
                case State::V2Head:
                    headBytes[head++] = b;
                    crc = crc8DvbS2(crc, b);
                    if (head < 4) return false;
                    cmd = static_cast<uint16_t>(headBytes[0] | headBytes[1] << 8);
                    size = static_cast<uint16_t>(headBytes[2] | headBytes[3] << 8);
                    received = 0;
                    if (size > MAX_PAYLOAD)
                    {
                        ++errors;
                        state = State::Idle;
                        return false;
                    }
                    state = size ? State::V2Data : State::V2Check;
                    return false;
                case State::V2Data:
                    payload[received++] = b;
                    crc = crc8DvbS2(crc, b);
                    if (received >= size) state = State::V2Check;
                    return false;
                case State::V2Check:
                    state = State::Idle;
                    if (b != crc)
                    {
                        ++errors;
                        return false;
                    }
                    return true;
            }
            return false;
        }

        bool busy() const { return state != State::Idle; }
        void reset() { state = State::Idle; }

        uint8_t version = 1;
        uint16_t cmd = 0;
        uint16_t size = 0;
        uint8_t flag = 0;
        uint8_t payload[MAX_PAYLOAD] = {};
        uint32_t errors = 0;

    private:
        enum class State : uint8_t { Idle, Proto, Direction, V1Size, V1Cmd, V1Data, V1Check, V2Flag, V2Head, V2Data, V2Check };
        State state = State::Idle;
        uint8_t check = 0, crc = 0;
        uint8_t headBytes[4] = {};
        uint8_t head = 0;
        uint16_t received = 0;
    };

    // Поля ответа (младший байт первым). Не влезло — overflow.
    class Writer
    {
    public:
        void u8(uint32_t v) { put(static_cast<uint8_t>(v)); }
        void u16(uint32_t v)
        {
            put(static_cast<uint8_t>(v));
            put(static_cast<uint8_t>(v >> 8));
        }
        void u32(uint32_t v)
        {
            for (int i = 0; i < 4; ++i) put(static_cast<uint8_t>(v >> (8 * i)));
        }
        void data(const void* d, size_t n)
        {
            for (size_t i = 0; i < n; ++i) put(static_cast<const uint8_t*>(d)[i]);
        }
        void text(const char* s) { data(s, strlen(s)); }
        // Строка с длиной впереди (u8), как в BOARD_INFO.
        void lenText(const char* s)
        {
            const size_t n = strlen(s) > 255 ? 255 : strlen(s);
            u8(n);
            data(s, n);
        }

        uint8_t buf[MAX_PAYLOAD] = {};
        size_t length = 0;
        bool overflow = false;

    private:
        void put(uint8_t b)
        {
            if (length < MAX_PAYLOAD) buf[length++] = b;
            else overflow = true;
        }
    };

    // Поля запроса: за концом — нули, remaining() — сколько осталось.
    class Reader
    {
    public:
        Reader(const uint8_t* d, size_t n) : p(d), size(n) {}
        size_t remaining() const { return at < size ? size - at : 0; }
        uint8_t u8() { return at < size ? p[at++] : (at++, 0); }
        uint16_t u16()
        {
            const uint16_t lo = u8();
            return static_cast<uint16_t>(lo | (u8() << 8));
        }
        uint32_t u32()
        {
            uint32_t v = 0;
            for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(u8()) << (8 * i);
            return v;
        }

    private:
        const uint8_t* p;
        size_t size;
        size_t at = 0;
    };

    // Кадр ответа той же версии, что запрос. Возвращает длину (0 — не влезло).
    inline size_t frame(uint8_t* out, size_t capacity, uint8_t version, uint16_t cmd, bool error, const uint8_t* data, size_t len)
    {
        const char dir = error ? '!' : '>';
        if (version == 1)
        {
            if (len > 255 || capacity < len + 6) return 0;
            size_t n = 0;
            out[n++] = '$';
            out[n++] = 'M';
            out[n++] = static_cast<uint8_t>(dir);
            out[n++] = static_cast<uint8_t>(len);
            out[n++] = static_cast<uint8_t>(cmd);
            uint8_t check = static_cast<uint8_t>(len ^ static_cast<uint8_t>(cmd));
            for (size_t i = 0; i < len; ++i)
            {
                out[n++] = data[i];
                check ^= data[i];
            }
            out[n++] = check;
            return n;
        }
        if (capacity < len + 9) return 0;
        size_t n = 0;
        out[n++] = '$';
        out[n++] = 'X';
        out[n++] = static_cast<uint8_t>(dir);
        const uint8_t head[5] = { 0, static_cast<uint8_t>(cmd), static_cast<uint8_t>(cmd >> 8), static_cast<uint8_t>(len),
                                  static_cast<uint8_t>(len >> 8) };
        uint8_t crc = 0;
        for (uint8_t b : head)
        {
            out[n++] = b;
            crc = crc8DvbS2(crc, b);
        }
        for (size_t i = 0; i < len; ++i)
        {
            out[n++] = data[i];
            crc = crc8DvbS2(crc, data[i]);
        }
        out[n++] = crc;
        return n;
    }
}
