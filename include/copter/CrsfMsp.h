#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "copter/CrsfTelemetry.h"
#include "telemetry/Msp.h"

// ============================================================
// MSP через CRSF — настройка с пульта: Lua-скрипты Betaflight на EdgeTX
// (PID, расходы, VTX) и VTX Admin ExpressLRS. Запрос идёт с пульта через
// приёмник кадрами CRSF, ответ — обратно телеметрией; разбирает и отвечает
// тот же CopterMsp, что и по USB.
//
// Кадры (по открытому описанию CRSF, «расширенный» заголовок):
//   0xC8 длина тип куда откуда | MSP-кусок | CRC8
//   тип 0x7A — запрос (ждут ответ), 0x7C — запись (без ответа),
//   0x7B — ответ; куда/откуда: 0xC8 — полётник, 0xEA — пульт.
// MSP-кусок: байт состояния, дальше данные:
//   биты 0–3 — номер куска (по кругу 0..15), бит 4 — первый кусок,
//   биты 5–6 — версия MSP (1 или 2), бит 7 — ошибка (в ответе).
// Первый кусок v1: размер u8, команда u8, данные…; v2: флаг u8, команда
// u16, размер u16, данные… (младший байт первым). Длинное сообщение —
// несколькими кусками подряд; v1-ответ кончается xor (размер, команда,
// данные), как в MSP по проводу.
//
// Ответ отдаётся по куску за раз вместо очередного кадра телеметрии
// (main_copter.cpp, sendTelemetry): линия к пульту узкая.
//
// Без Arduino — тесты на ПК (test_copter).
// ============================================================

namespace copter
{
    constexpr uint8_t CRSF_FRAME_MSP_REQ = 0x7A;
    constexpr uint8_t CRSF_FRAME_MSP_RESP = 0x7B;
    constexpr uint8_t CRSF_FRAME_MSP_WRITE = 0x7C;
    constexpr uint8_t CRSF_ADDRESS_RADIO = 0xEA;

    class CrsfMsp
    {
    public:
        static constexpr uint8_t MAX_FRAME = 64;
        static constexpr uint8_t CHUNK = MAX_FRAME - 6;   // без 0xC8, длины, типа, куда, откуда, CRC

        // Кадр CRSF целиком (CRC уже проверена приёмником). true — запрос собран:
        // request() отдать в CopterMsp, ответ (если wantsReply()) — в reply().
        bool feed(const uint8_t* frame, uint8_t length)
        {
            if (length < 7) return false;
            const uint8_t type = frame[2];
            if (type != CRSF_FRAME_MSP_REQ && type != CRSF_FRAME_MSP_WRITE) return false;
            if (frame[3] != CRSF_ADDRESS_FC) return false;
            const uint8_t* p = frame + 5;
            const uint8_t n = static_cast<uint8_t>(length - 6);   // кусок без CRC
            const uint8_t status = p[0];
            const uint8_t seq = status & 0x0F;
            if (status & 0x10)
            {
                const uint8_t version = static_cast<uint8_t>((status >> 5) & 0x03);
                uint8_t head = 0;
                uint16_t size = 0, cmd = 0;
                if (version == 1 && n >= 3)
                {
                    size = p[1];
                    cmd = p[2];
                    head = 3;
                }
                else if (version == 2 && n >= 6)
                {
                    req.flag = p[1];
                    cmd = static_cast<uint16_t>(p[2] | (p[3] << 8));
                    size = static_cast<uint16_t>(p[4] | (p[5] << 8));
                    head = 6;
                }
                if (!head || size > msp::MAX_PAYLOAD)
                {
                    collecting = false;
                    ++dropped;
                    return false;
                }
                req.version = version;
                req.cmd = cmd;
                req.size = size;
                received = 0;
                origin = frame[4];
                write = type == CRSF_FRAME_MSP_WRITE;
                collecting = true;
                return take(p + head, static_cast<uint8_t>(n - head), seq);
            }
            if (!collecting || seq != ((lastSeq + 1) & 0x0F))
            {
                if (collecting) ++dropped;
                collecting = false;   // кусок пропал — ждём новый первый
                return false;
            }
            return take(p + 1, static_cast<uint8_t>(n - 1), seq);
        }

        const msp::Parser& request() const { return req; }
        bool wantsReply() const { return !write; }

        // Ответ на собранный запрос: данные без обёртки MSP (CopterMsp::process).
        void reply(bool ok, const uint8_t* data, size_t length)
        {
            if (!ok) length = 0;
            if (length > msp::MAX_PAYLOAD) length = msp::MAX_PAYLOAD;
            size_t n = 0;
            if (req.version == 1)
            {
                // длиннее 254 — «jumbo», как по проводу: 255, команда, размер u16
                resp[n++] = static_cast<uint8_t>(length < 255 ? length : 255);
                resp[n++] = static_cast<uint8_t>(req.cmd);
                if (length >= 255)
                {
                    resp[n++] = static_cast<uint8_t>(length);
                    resp[n++] = static_cast<uint8_t>(length >> 8);
                }
                memcpy(resp + n, data, length);
                n += length;
                uint8_t x = 0;
                for (size_t i = 0; i < n; ++i) x ^= resp[i];
                resp[n++] = x;
            }
            else
            {
                resp[n++] = 0;
                resp[n++] = static_cast<uint8_t>(req.cmd);
                resp[n++] = static_cast<uint8_t>(req.cmd >> 8);
                resp[n++] = static_cast<uint8_t>(length);
                resp[n++] = static_cast<uint8_t>(length >> 8);
                memcpy(resp + n, data, length);
                n += length;
            }
            respLength = n;
            respAt = 0;
            respError = !ok;
            respVersion = req.version;
            respFirst = true;
        }

        bool pending() const { return respAt < respLength; }

        // Следующий кусок ответа — кадр CRSF 0x7B в out; длина (0 — нечего слать).
        uint8_t next(uint8_t* out)
        {
            if (!pending()) return 0;
            size_t chunk = respLength - respAt;
            if (chunk > CHUNK - 1u) chunk = CHUNK - 1u;
            out[0] = CRSF_ADDRESS_FC;
            out[1] = static_cast<uint8_t>(chunk + 5);   // тип, куда, откуда, состояние, кусок, CRC
            out[2] = CRSF_FRAME_MSP_RESP;
            out[3] = origin;
            out[4] = CRSF_ADDRESS_FC;
            out[5] = static_cast<uint8_t>((txSeq & 0x0F) | (respFirst ? 0x10 : 0) | (respVersion << 5) | (respError ? 0x80 : 0));
            memcpy(out + 6, resp + respAt, chunk);
            respAt += chunk;
            respFirst = false;
            txSeq = static_cast<uint8_t>((txSeq + 1) & 0x0F);
            const uint8_t length = static_cast<uint8_t>(chunk + 6);
            out[length] = crsfCrc(out + 2, static_cast<uint8_t>(length - 2));
            return static_cast<uint8_t>(length + 1);
        }

        uint32_t dropped = 0;   // запросов потеряно (кусок пропал, слишком длинный)

    private:
        bool take(const uint8_t* data, uint8_t n, uint8_t seq)
        {
            lastSeq = seq;
            const uint16_t need = static_cast<uint16_t>(req.size - received);
            const uint16_t copy = n < need ? n : need;   // лишнее в конце (xor v1) — пропустить
            memcpy(req.payload + received, data, copy);
            received = static_cast<uint16_t>(received + copy);
            if (received < req.size) return false;
            collecting = false;
            return true;
        }

        msp::Parser req;
        uint16_t received = 0;
        uint8_t lastSeq = 0, origin = CRSF_ADDRESS_RADIO;
        bool collecting = false, write = false;

        uint8_t resp[msp::MAX_PAYLOAD + 6] = {};
        size_t respLength = 0, respAt = 0;
        uint8_t respVersion = 1, txSeq = 0;
        bool respError = false, respFirst = false;
    };
}
