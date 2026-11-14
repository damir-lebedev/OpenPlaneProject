#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hal/IUartPort.h"
#include "telemetry/Msp.h"

// ============================================================
// Порт MSP на UART — для того, что подключают к полётнику проводом:
// цифровые очки и видеопередатчики (DJI, Walksnail, HDZero — они сами
// опрашивают полётник: ARM, батарея, канал VTX), Bluetooth-модуль с
// приложением, второй конфигуратор. 115200 8N1, как у всех.
//
// Работает из полётного цикла и никогда не ждёт: входящие байты — в
// разборщик кадров, ответ — в UART, только если он помещается в его буфер
// передачи целиком (availableForWrite); не помещается — ответ пропадает,
// спросят ещё раз. Так же, целыми кадрами, сюда кладётся OSD для очков
// (copter/DisplayPort.h) — поэтому кадры на проводе не рвутся и не
// перемешиваются. Своей очереди нет: очередью служит буфер UART (на F405
// каждый килобайт ОЗУ на счету).
//
// Запросы очков короткие — разборщик держит до REQUEST_MAX байт данных;
// длиннее — кадр проглатывается и считается ошибкой. Что отвечать, решает
// вызывающий (CopterMsp): сюда он передаёт функцию.
// ============================================================

class MspPort
{
public:
    static constexpr uint32_t BAUD = 115200;
    static constexpr uint16_t REQUEST_MAX = 128;   // данные запроса: SET_VTX_CONFIG — 15, SET_PID_ADVANCED — ~60
    static constexpr uint32_t ALIVE_MS = 3000;     // запросов нет дольше — «на проводе тихо»
    static constexpr uint8_t RX_PER_POLL = 96;     // байт за один такт: цикл не должен застрять на приёме
    static constexpr size_t UNLIMITED = 0xFFFF;    // порт не говорит, сколько места (тесты, ESP32)

    explicit MspPort(IUartPort& uart) : port(uart) {}

    void begin()
    {
        port.begin(BAUD);
        opened = true;
    }

    bool isOpen() const { return opened; }

    // Каждый такт: принять и ответить. onRequest(parser, out, capacity) ->
    // длина готового кадра ответа (0 — отвечать нечем).
    template <typename Handler>
    void poll(uint32_t nowMs, Handler&& onRequest)
    {
        if (!opened) return;
        for (uint8_t i = 0; i < RX_PER_POLL && port.available() > 0; ++i)
        {
            const int b = port.read();
            if (b < 0) break;
            if (!parser.feed(static_cast<uint8_t>(b))) continue;
            ++requests;
            lastRequestMs = nowMs;
            lastCommand = parser.cmd;
            // обработчику — обычный разборщик (на стеке, только пока идёт ответ)
            msp::Parser request;
            request.version = parser.version;
            request.cmd = parser.cmd;
            request.size = parser.size;
            request.flag = parser.flag;
            memcpy(request.payload, parser.payload, parser.size);
            uint8_t reply[msp::MAX_PAYLOAD + 16];
            const size_t n = onRequest(request, reply, sizeof(reply));
            if (n && !queue(reply, n)) ++dropped;   // UART занят — ответ пропал, спросят ещё раз
        }
    }

    // Сколько сейчас поместится в буфер передачи UART.
    size_t room()
    {
        if (!opened) return 0;
        const int space = port.availableForWrite();
        return space < 0 ? UNLIMITED : static_cast<size_t>(space);
    }

    // Кадр — в UART целиком или никак (не ждёт).
    bool queue(const uint8_t* frame, size_t n)
    {
        if (!opened || n == 0 || n > room()) return false;
        port.write(frame, n);
        sentBytes += static_cast<uint32_t>(n);
        return true;
    }

    // На проводе кто-то есть: хоть раз спросил / спрашивал недавно.
    bool seen() const { return requests > 0; }
    bool alive(uint32_t nowMs) const { return requests > 0 && nowMs - lastRequestMs < ALIVE_MS; }

    uint32_t requests = 0;        // принятых запросов
    uint32_t dropped = 0;         // ответов, не поместившихся в UART
    uint32_t sentBytes = 0;
    uint16_t lastCommand = 0;
    uint32_t lastRequestMs = 0;

    uint32_t parseErrors() const { return parser.errors; }

private:
    IUartPort& port;
    msp::BasicParser<REQUEST_MAX> parser;
    bool opened = false;
};
