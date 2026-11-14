#pragma once
#include <stdint.h>
#include <string.h>

#include "copter/Osd.h"
#include "copter/Vtx.h"
#include "telemetry/Msp.h"

// ============================================================
// OSD в цифровых очках — MSP DisplayPort: DJI O3/O4, Walksnail Avatar,
// HDZero, DJI с WTFOS, OpenIPC. Очки ничего не считают сами: полётник шлёт
// им символы шрифта Betaflight с координатами — как чипу AT7456E, только
// по UART. Своя реализация по открытому описанию протокола (документация
// Betaflight, «DisplayPort MSP Extensions»):
//
//   кадр MSP v1 от полётника: '$' 'M' '>' | размер | 182 | данные | xor
//   данные — первый байт говорит, что делать:
//     0 HEARTBEAT     «я здесь» — очки не возвращаются к своему экрану
//     1 RELEASE       экран отдан очкам
//     2 CLEAR_SCREEN  стереть набранное
//     3 WRITE_STRING  строка, столбец, атрибут, символы (до 30 за кадр)
//     4 DRAW_SCREEN   показать набранное после CLEAR
//   атрибут: биты 0–1 — страница шрифта, бит 6 — «мигай сам» (не просим:
//   мигание делает Osd.h, одинаково для чипа и очков), бит 7 — версия, 0.
//   Символы — без завершающего нуля: длину строки даёт размер кадра.
//
// Размер экрана очки сообщают сами — MSP_SET_OSD_CANVAS (188: столбцы,
// строки), тогда экран HD. Молчат — берётся из настроек: vcd_video_system
// (AUTO/PAL/NTSC — 30 × 16 или 13, HD — osd_canvas_width × osd_canvas_height).
//
// Куда рисовать — osd_displayport_device, как в BF: NONE, AUTO, MAX7456,
// MSP. AUTO у нас умнее: чип на плате — в него, а как только на порту MSP
// кто-то начал спрашивать (очки и передатчики опрашивают полётник: ARM,
// батарея, канал) — ещё и туда. Ставить ничего не нужно: припаял — работает.
//
// Экран уходит целиком osd_framerate_hz раз в секунду: CLEAR, строки, DRAW
// (так делают все полётники, очки на это рассчитаны), раз в секунду —
// HEARTBEAT. Экран собирает фоновая задача OSD, а в UART пишет полётный
// цикл (там же ответы на запросы очков): render() оставляет готовую пачку
// кадров, drain() отдаёт её порту по одному целому кадру — сколько влезает.
// Пока пачка не ушла, следующая не собирается: медленный порт сам
// снижает частоту, а кадры не рвутся и не перемешиваются с ответами.
//
// Без Arduino — тесты на ПК (test_copter, очки — helpers/DisplayPortGoggles.h).
// ============================================================

namespace copter
{
    namespace dp
    {
        constexpr uint8_t MSP_DISPLAYPORT = 182;
        constexpr uint8_t MSP_SET_OSD_CANVAS = 188;
        constexpr uint8_t MSP_OSD_CANVAS = 189;

        constexpr uint8_t HEARTBEAT = 0, RELEASE = 1, CLEAR_SCREEN = 2, WRITE_STRING = 3, DRAW_SCREEN = 4;
        constexpr uint8_t OPTIONS = 5, SYS = 6;   // шлют INAV и ArduPilot — нам не нужны
        constexpr uint8_t MAX_STRING = 30;
        constexpr uint8_t ATTR_BLINK = 0x40, ATTR_FONT_MASK = 0x03;
    }

    // osd_displayport_device — порядок как в BF (FRSKYOSD у нас нет).
    enum class OsdDevice : uint8_t
    {
        NONE,
        AUTO,
        MAX7456,
        MSP,
    };

    // vcd_video_system — как в BF.
    enum class VideoSystem : uint8_t
    {
        AUTO,
        PAL,
        NTSC,
        HD,
    };

    struct OsdCanvas
    {
        uint8_t cols = OsdScreen::COLS;
        uint8_t rows = OsdScreen::SD_ROWS;
        bool hd = false;
    };

    // Что известно: настройки, плата и что сказали очки.
    struct OsdRouteInput
    {
        OsdDevice device = OsdDevice::AUTO;
        VideoSystem video = VideoSystem::AUTO;
        uint8_t canvasCols = 53, canvasRows = 20;   // osd_canvas_width / osd_canvas_height
        bool chipPresent = false;                   // AT7456E ответил
        uint8_t chipRows = OsdScreen::SD_ROWS;      // 16 PAL, 13 NTSC — по камере
        bool portPresent = false;                   // на плате есть UART для очков
        bool deviceSeen = false;                    // на нём кто-то спрашивал нас по MSP
        uint8_t deviceCols = 0, deviceRows = 0;     // MSP_SET_OSD_CANVAS от очков; 0 — не сообщали
    };

    // Куда рисовать и какого размера экран.
    struct OsdRoute
    {
        bool chip = false;
        bool msp = false;
        OsdCanvas canvas;
    };

    inline OsdRoute osdRoute(const OsdRouteInput& in)
    {
        OsdRoute r;
        r.chip = in.chipPresent && (in.device == OsdDevice::AUTO || in.device == OsdDevice::MAX7456);
        r.msp = in.portPresent && (in.device == OsdDevice::MSP || (in.device == OsdDevice::AUTO && in.deviceSeen));
        auto fit = [](uint8_t v, uint8_t lo, uint8_t hi) { return v < lo ? lo : v > hi ? hi : v; };
        if (r.msp && in.deviceCols && in.deviceRows)
        {
            // очки сказали сами — как у BF: приём MSP_SET_OSD_CANVAS включает HD
            r.canvas.cols = fit(in.deviceCols, 1, OsdScreen::MAX_COLS);
            r.canvas.rows = fit(in.deviceRows, 1, OsdScreen::MAX_ROWS);
            r.canvas.hd = true;
        }
        else if (r.msp && in.video == VideoSystem::HD)
        {
            r.canvas.cols = fit(in.canvasCols, OsdScreen::COLS, OsdScreen::MAX_COLS);
            r.canvas.rows = fit(in.canvasRows, 13, OsdScreen::MAX_ROWS);
            r.canvas.hd = true;
        }
        else if (in.video == VideoSystem::NTSC) r.canvas.rows = 13;
        else if (in.video == VideoSystem::PAL) r.canvas.rows = OsdScreen::SD_ROWS;
        else if (r.chip) r.canvas.rows = in.chipRows;
        return r;
    }

    // Для приложения и консоли: куда рисуется OSD и кто на порту очков.
    struct OsdOutputStatus
    {
        bool port = false;           // UART для очков на плате есть
        bool chip = false;           // рисуем в чип AT7456E
        bool msp = false;            // шлём в очки по MSP
        uint8_t cols = OsdScreen::COLS, rows = OsdScreen::SD_ROWS;
        bool hd = false;
        bool deviceAlive = false;    // на порту спрашивали за последние секунды
        bool deviceCanvas = false;   // размер экрана сказали сами очки
        bool mspVtx = false;         // на порту — передатчик, читающий MSP_VTX_CONFIG
        uint32_t requests = 0;       // запросов с порта всего
    };

    class DisplayPort
    {
    public:
        // весь экран 60 × 22 без пробелов: 22 строки × 2 кадра по 40 байт + HEARTBEAT, CLEAR, DRAW
        static constexpr size_t BATCH_SIZE = OsdScreen::MAX_ROWS * 2 * (dp::MAX_STRING + 10) + 3 * 7;
        static constexpr uint32_t HEARTBEAT_MS = 1000;
        // Столько пробелов внутри строки дешевле передать, чем начать новый кадр (10 байт обёртки).
        static constexpr uint8_t BRIDGE = 8;

        // displayport_msp_col_adjust / _row_adjust (сдвиг влево и вверх, как в BF), osd_framerate_hz.
        void configure(int8_t colAdjustment, int8_t rowAdjustment, uint8_t framerateHz)
        {
            colAdjust = colAdjustment;
            rowAdjust = rowAdjustment;
            const uint8_t hz = framerateHz < 1 ? 1 : framerateHz;
            periodMs = static_cast<uint16_t>(1000u / hz);
        }

        // Задача OSD: экран — в пачку кадров. false — рано (osd_framerate_hz)
        // или прошлая пачка ещё не ушла в порт.
        bool render(const OsdScreen& screen, uint32_t nowMs)
        {
            if (ready) return false;
            if (drawn && nowMs - lastMs + TOLERANCE_MS < periodMs) return false;
            lastMs = nowMs;
            drawn = true;
            length = 0;
            if (!beat || nowMs - beatMs >= HEARTBEAT_MS)
            {
                beatMs = nowMs;
                beat = true;
                command(dp::HEARTBEAT);
            }
            command(dp::CLEAR_SCREEN);
            for (uint8_t row = 0; row < screen.rowCount(); ++row) renderRow(screen, row);
            command(dp::DRAW_SCREEN);
            publish();
            return true;
        }

        // Задача OSD: экран больше не наш (osd_displayport_device сменили) — RELEASE, один раз.
        bool release()
        {
            if (!drawn) return true;
            if (ready) return false;
            length = 0;
            command(dp::RELEASE);
            drawn = false;
            beat = false;
            publish();
            return true;
        }

        // Полётный цикл: пачка ждёт отправки.
        bool pending() const { return ready; }

        // Полётный цикл: целые кадры пачки — в порт, сколько влезает. Порт —
        // с методами room() и queue(данные, длина) (telemetry/MspPort.h).
        template <typename Port>
        void drain(Port& port)
        {
            if (!ready) return;
            while (sent < length)
            {
                const size_t n = static_cast<size_t>(out[sent + 3]) + 6u;   // '$M>' размер команда … xor
                if (sent + n > length)
                {
                    sent = length;   // пачка испорчена — бросить остаток, не зависать
                    break;
                }
                if (port.room() < n || !port.queue(out + sent, n)) break;
                sent += n;
                ++frames;
            }
            if (sent >= length)
            {
                sent = 0;
                ++batches;
                ready = false;
            }
        }

        // Порт пропал или его очередь сбросили — не ждать вечно.
        void drop()
        {
            sent = 0;
            ready = false;
        }

        // Для тестов и статуса.
        const uint8_t* batchData() const { return out; }
        size_t batchLength() const { return length; }
        uint32_t frames = 0, batches = 0;

    private:
        static constexpr uint32_t TOLERANCE_MS = 5;   // задача OSD будит не точно в срок

        void publish()
        {
            sent = 0;
            ready = length > 0;
        }

        void frame(const uint8_t* payload, size_t n)
        {
            const size_t written = msp::frame(out + length, BATCH_SIZE - length, 1, dp::MSP_DISPLAYPORT, false, payload, n);
            length += written;   // не влезло — 0: кадр пропущен целиком
        }

        void command(uint8_t sub) { frame(&sub, 1); }

        // Строка экрана — кусками до 30 символов: от первого непустого, хвостовые
        // пробелы не передаются; короткий пропуск внутри — передаётся как есть.
        void renderRow(const OsdScreen& screen, uint8_t row)
        {
            const int outRow = row + rowAdjust;
            if (outRow < 0) return;
            const uint8_t cols = screen.colCount();
            uint8_t col = 0;
            while (col < cols)
            {
                while (col < cols && blank(screen.at(row, col))) ++col;
                if (col >= cols) break;
                const uint8_t start = col;
                uint8_t end = start;   // за последним непустым
                while (col < cols && col - start < dp::MAX_STRING)
                {
                    if (!blank(screen.at(row, col))) end = static_cast<uint8_t>(col + 1);
                    else if (col - end >= BRIDGE) break;
                    ++col;
                }
                int outCol = start + colAdjust;
                uint8_t from = start;
                if (outCol < 0)
                {
                    // сдвиг вывел начало за левый край — эти символы не видны
                    from = static_cast<uint8_t>(start - outCol);
                    outCol = 0;
                }
                if (from >= end) continue;
                uint8_t payload[4 + dp::MAX_STRING];
                payload[0] = dp::WRITE_STRING;
                payload[1] = static_cast<uint8_t>(outRow);
                payload[2] = static_cast<uint8_t>(outCol);
                payload[3] = 0;   // страница шрифта 0, без мигания
                uint8_t n = 0;
                for (uint8_t c = from; c < end; ++c)
                {
                    const uint8_t ch = screen.at(row, c);
                    payload[4 + n++] = ch ? ch : ' ';
                }
                frame(payload, static_cast<size_t>(4 + n));
            }
        }

        static bool blank(uint8_t ch) { return ch == ' ' || ch == 0; }

        uint8_t out[BATCH_SIZE] = {};
        size_t length = 0;
        size_t sent = 0;               // трогает только полётный цикл
        volatile bool ready = false;   // true ставит задача OSD, false — полётный цикл
        int8_t colAdjust = 0, rowAdjust = 0;
        uint16_t periodMs = 83;        // 12 Гц, как osd_framerate_hz у BF
        uint32_t lastMs = 0, beatMs = 0;
        bool drawn = false, beat = false;
    };

    // ---------------- видеопередатчик на MSP ----------------

    // Цифровой передатчик (HDZero, Walksnail) канал и мощность берёт у
    // полётника сам: читает MSP_VTX_CONFIG (88) с порта очков. SmartAudio и
    // Tramp ему не нужны. Здесь — только «он есть» и «настройки поменялись»:
    // тогда полётник шлёт тот же MSP_VTX_CONFIG сам, не дожидаясь опроса
    // (как BF с VTX типа MSP). Так канал, выбранный с пульта через VTX Admin
    // ExpressLRS (он же переключает очки через Backpack), доходит до
    // передатчика сразу.
    class MspVtxLink
    {
    public:
        // С порта очков спросили MSP_VTX_CONFIG или прислали MSP_SET_VTX_CONFIG.
        void polled() { seen = true; }
        bool online() const { return seen; }

        // true — передатчик есть и то, что от него хотим, поменялось: послать MSP_VTX_CONFIG.
        bool changed(const VtxRequest& r, uint8_t lowPowerDisarm)
        {
            const uint32_t key = static_cast<uint32_t>(r.frequencyMhz) | static_cast<uint32_t>(r.powerLevel & 0x0F) << 16 |
                                 static_cast<uint32_t>(r.band & 0x0F) << 20 | static_cast<uint32_t>(r.channel & 0x0F) << 24 |
                                 static_cast<uint32_t>(r.pit ? 1u : 0u) << 28 | static_cast<uint32_t>(lowPowerDisarm & 0x03) << 29;
            if (!seen || !r.control) return false;
            if (known && key == lastKey) return false;
            known = true;
            lastKey = key;
            return true;
        }

    private:
        bool seen = false, known = false;
        uint32_t lastKey = 0;
    };
}
