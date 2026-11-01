#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry/BlackBoxFormat.h"

// ============================================================
// Кадры камеры (OV5640, JPEG): между задачей камеры, приложением и
// чёрным ящиком.
//
// Приложение просит снимок командой ПК "cam snap" (STX, строка, '\n');
// задача камеры снимает кадр и отдаёт его сюда (frameReady), полётная
// задача отправляет его строками
//   @cam {"n":12,"w":640,"h":480,"len":10234,"off":0,"d":"<base64>"}
// по CHUNK байт кадра в строке — когда в порту есть место под строку
// целиком (полётный цикл ПК не ждёт). Пока кадр уходит, задача камеры
// буфер не трогает.
//
// В чёрный ящик кадр ложится записями REC_CAM: t_us, номер кадра,
// смещение, длина кадра, байты JPEG (до CAM_DATA на запись) —
// tools/blackbox.py собирает их обратно в файлы .jpg.
//
// Без Arduino — тестируется на ПК (test_camera).
// ============================================================

namespace CameraFormat
{
    // Данные в записи ящика: 255 − t_us(4) − кадр(2) − смещение(4) − длина(4)
    constexpr size_t CAM_HEADER = 4 + 2 + 4 + 4;
    constexpr size_t CAM_DATA = BlackBoxFormat::MAX_PAYLOAD - CAM_HEADER;

    // Запись REC_CAM в record (MAX_RECORD байт): [тип][длина][t_us][кадр][смещение][длина кадра][байты].
    inline size_t record(uint8_t* record, uint32_t tUs, uint16_t frame, uint32_t offset, uint32_t size, const uint8_t* data,
                         size_t n)
    {
        if (n > CAM_DATA) n = CAM_DATA;
        record[0] = BlackBoxFormat::REC_CAM;
        record[1] = static_cast<uint8_t>(CAM_HEADER + n);
        uint8_t* p = record + BlackBoxFormat::RECORD_HEADER;
        memcpy(p, &tUs, 4);
        memcpy(p + 4, &frame, 2);
        memcpy(p + 6, &offset, 4);
        memcpy(p + 10, &size, 4);
        memcpy(p + 14, data, n);
        return BlackBoxFormat::RECORD_HEADER + CAM_HEADER + n;
    }

    inline size_t base64(const uint8_t* in, size_t n, char* out, size_t cap)
    {
        static const char* abc = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        size_t o = 0;
        for (size_t i = 0; i < n; i += 3)
        {
            if (o + 4 >= cap) return 0;
            const uint32_t v = static_cast<uint32_t>(in[i]) << 16 | (i + 1 < n ? in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);
            out[o++] = abc[(v >> 18) & 63];
            out[o++] = abc[(v >> 12) & 63];
            out[o++] = i + 1 < n ? abc[(v >> 6) & 63] : '=';
            out[o++] = i + 2 < n ? abc[v & 63] : '=';
        }
        out[o] = '\0';
        return o;
    }
}

class CameraStream
{
public:
    static constexpr size_t CHUNK = 384;                 // байт кадра в строке "@cam"
    static constexpr size_t LINE_CAPACITY = 640;         // строка целиком: заголовок + base64(CHUNK) = 512

    enum class State : uint8_t { NONE, NORESP, OK, ERROR };

    // ---- полётная задача (команды ПК) ----

    void request() { requested = true; }

    // Следующая строка "@cam …" в buf; 0 — отправлять нечего.
    size_t nextLine(char* buf, size_t cap)
    {
        if (!ready) return 0;
        const uint32_t n = sendLength - sendOffset > CHUNK ? CHUNK : sendLength - sendOffset;
        int head = snprintf(buf, cap, "@cam {\"n\":%u,\"w\":%u,\"h\":%u,\"len\":%lu,\"off\":%lu,\"d\":\"",
                            static_cast<unsigned>(sendFrame), static_cast<unsigned>(sendW), static_cast<unsigned>(sendH),
                            static_cast<unsigned long>(sendLength), static_cast<unsigned long>(sendOffset));
        if (head <= 0 || static_cast<size_t>(head) >= cap) return 0;
        const size_t b = CameraFormat::base64(sendData + sendOffset, n, buf + head, cap - head - 4);
        if (b == 0 && n > 0) return 0;
        size_t len = static_cast<size_t>(head) + b;
        memcpy(buf + len, "\"}\n", 4);
        len += 3;
        return len;
    }

    // Строка ушла — следующий кусок; кадр кончился — буфер свободен.
    void lineSent()
    {
        const uint32_t n = sendLength - sendOffset > CHUNK ? CHUNK : sendLength - sendOffset;
        sendOffset += n;
        if (sendOffset >= sendLength) ready = false;
    }

    // Ошибка снимка — строкой, а не молчанием (приложение ждёт).
    size_t errorLine(char* buf, size_t cap)
    {
        if (!failed) return 0;
        failed = false;
        const int n = snprintf(buf, cap, "@cam {\"error\":\"%s\"}\n", failText ? failText : "?");
        return n > 0 && static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : 0;
    }

    // ---- задача камеры ----

    // Буфер занят отправкой — не снимать в него.
    bool busy() const { return ready; }
    bool snapshotWanted() const { return requested; }

    // Кадр снят (data живёт до конца отправки). forHost — по заявке приложения.
    void frameReady(const uint8_t* data, uint32_t length, uint16_t frame, uint16_t w, uint16_t h, bool forHost)
    {
        frames++;
        lastBytes = length;
        lastFrame = frame;
        if (!forHost) return;
        requested = false;
        sendData = data;
        sendLength = length;
        sendOffset = 0;
        sendFrame = frame;
        sendW = w;
        sendH = h;
        ready = true;
    }

    void snapshotFailed(const char* why)
    {
        requested = false;
        failText = why;
        failed = true;
        errors++;
    }

    // ---- состояние для @live и status ----
    State state = State::NONE;
    uint16_t chip = 0;
    uint16_t width = 0, height = 0;
    uint32_t frames = 0, errors = 0, lastBytes = 0;
    uint32_t recordedFrames = 0;
    uint16_t lastFrame = 0;

    const char* stateName() const
    {
        switch (state)
        {
            case State::OK: return "ok";
            case State::NORESP: return "noresp";
            case State::ERROR: return "error";
            case State::NONE:
            default: return "none";
        }
    }

private:
    volatile bool requested = false;
    volatile bool ready = false;
    volatile bool failed = false;
    const char* failText = nullptr;
    const uint8_t* sendData = nullptr;
    uint32_t sendLength = 0, sendOffset = 0;
    uint16_t sendFrame = 0, sendW = 0, sendH = 0;
};
