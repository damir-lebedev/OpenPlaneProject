#pragma once
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// OSD коптера — что рисуется поверх картинки камеры (чип AT7456E/MAX7456,
// hal/Max7456.h): экран 30 столбцов × 16 строк (PAL) или 13 (NTSC).
//
// Шрифт в чипе — шрифт Betaflight (его зашивает производитель полётника):
// латиница, цифры и значки (батарея, спутники, стрелки). Поэтому надписи —
// латиницей и короткие, как в BF: LOW BATTERY, RXLOSS, DISARMED…
//
//   ┌──────────────────────────────┐
//   │LQ100          ANGL      ⊲12⊳ │  связь · режим · спутники
//   │                       400MW  │  мощность VTX
//   │                              │
//   │              +               │  прицел
//   │           LOW BATTERY        │  предупреждение (одно, главное)
//   │ ↑125M                 12M   │  домой: стрелка и расстояние · высота
//   │25.2V 4.20V   12.3A 1234MAH 02:13│ батарея · ток · мА·ч · время в полёте
//   └──────────────────────────────┘
//
// Без Arduino: собирает экран в буфер, чип пишет драйвер — только то, что
// поменялось.
// ============================================================

namespace copter
{
    // Значки шрифта Betaflight (номера символов в чипе).
    namespace osdsym
    {
        constexpr uint8_t RSSI = 0x01;
        constexpr uint8_t VOLT = 0x06;
        constexpr uint8_t MAH = 0x07;
        constexpr uint8_t METRE = 0x0C;
        constexpr uint8_t HOME = 0x11;
        constexpr uint8_t SAT_L = 0x1E;
        constexpr uint8_t SAT_R = 0x1F;
        constexpr uint8_t ARROW_SOUTH = 0x60;   // 16 стрелок: 0x60 юг, 0x64 восток, 0x68 север, 0x6C запад
        constexpr uint8_t CROSSHAIR = 0x73;
        constexpr uint8_t ALTITUDE = 0x7F;
        constexpr uint8_t LINK_QUALITY = 0x7B;
        constexpr uint8_t AMP = 0x9A;
        constexpr uint8_t FLY_TIME = 0x9C;
    }

    // Стрелка шрифта BF на угол относительно носа (0 — вперёд/вверх, по часовой).
    inline uint8_t osdArrow(float relativeDeg)
    {
        int k = static_cast<int>(lroundf(relativeDeg / 22.5f));
        k = ((8 - k) % 16 + 16) % 16;
        return static_cast<uint8_t>(osdsym::ARROW_SOUTH + k);
    }

    // Что показать.
    struct OsdData
    {
        bool armed = false;
        bool failsafe = false;
        const char* mode = "ACRO";          // ACRO, ANGL, HOR, ALTH
        int16_t linkQuality = -1;           // %, −1 — неизвестно
        bool linkLost = false;
        float volts = 0.0f, cellVolts = 0.0f;
        bool hasCurrent = false;
        float amps = 0.0f, mahDrawn = 0.0f;
        uint8_t batteryState = 1;           // BatteryState: 0 нет, 1 OK, 2 WARNING, 3 CRITICAL
        bool batteryNotFull = false;        // подключили не полную (vbat_full_cell_voltage)
        uint32_t flightSeconds = 0;
        float altitudeM = 0.0f;
        bool gpsPresent = false;
        uint8_t satellites = 0;
        bool home = false;
        float homeDistanceM = 0.0f, homeRelativeDeg = 0.0f;
        uint16_t vtxMw = 0;                 // 0 — нет VTX
        bool vtxPit = false;
        const char* armBlock = nullptr;     // код BF, почему не армится (только без ARM)
    };

    class OsdScreen
    {
    public:
        static constexpr uint8_t COLS = 30;
        static constexpr uint8_t MAX_ROWS = 16;

        OsdScreen() { clear(); }

        void setRows(uint8_t n) { rows = n > MAX_ROWS ? MAX_ROWS : n; }
        uint8_t rowCount() const { return rows; }
        uint8_t at(uint8_t row, uint8_t col) const { return row < MAX_ROWS && col < COLS ? cells[row][col] : ' '; }

        void clear() { memset(cells, ' ', sizeof(cells)); }

        // Текст — заглавными (в шрифте только они).
        void print(uint8_t row, uint8_t col, const char* text)
        {
            for (; *text && col < COLS && row < rows; ++text, ++col)
            {
                cells[row][col] = static_cast<uint8_t>(toupper(static_cast<unsigned char>(*text)));
            }
        }

        void put(uint8_t row, uint8_t col, uint8_t symbol)
        {
            if (row < rows && col < COLS) cells[row][col] = symbol;
        }

        // Весь экран из данных.
        void compose(const OsdData& d)
        {
            clear();
            char s[24];
            const uint8_t bottom = static_cast<uint8_t>(rows - 2);

            // связь
            if (d.linkQuality >= 0)
            {
                put(0, 1, osdsym::LINK_QUALITY);
                snprintf(s, sizeof(s), "%d", d.linkQuality);
                print(0, 2, s);
            }
            // режим
            snprintf(s, sizeof(s), "%s", d.failsafe ? "!FS!" : d.mode);
            print(0, static_cast<uint8_t>(COLS / 2 - strlen(s) / 2), s);
            // спутники
            if (d.gpsPresent)
            {
                put(0, 24, osdsym::SAT_L);
                put(0, 25, osdsym::SAT_R);
                snprintf(s, sizeof(s), "%u", d.satellites);
                print(0, 26, s);
            }
            // видеопередатчик
            if (d.vtxMw)
            {
                snprintf(s, sizeof(s), d.vtxPit ? "PIT" : "%uMW", d.vtxMw);
                print(1, static_cast<uint8_t>(COLS - 1 - strlen(s)), s);
            }
            // прицел
            put(static_cast<uint8_t>(rows / 2), COLS / 2 - 1, osdsym::CROSSHAIR);
            // домой
            if (d.home)
            {
                put(static_cast<uint8_t>(bottom - 1), 1, osdArrow(d.homeRelativeDeg));
                formatDistance(s, sizeof(s), d.homeDistanceM);
                print(static_cast<uint8_t>(bottom - 1), 2, s);
            }
            // высота
            put(static_cast<uint8_t>(bottom - 1), 23, osdsym::ALTITUDE);
            snprintf(s, sizeof(s), "%d", static_cast<int>(lroundf(d.altitudeM)));
            print(static_cast<uint8_t>(bottom - 1), 24, s);
            put(static_cast<uint8_t>(bottom - 1), static_cast<uint8_t>(24 + strlen(s)), osdsym::METRE);
            // батарея
            if (d.batteryState != 0)
            {
                snprintf(s, sizeof(s), "%.1f", static_cast<double>(d.volts));
                print(bottom, 1, s);
                put(bottom, static_cast<uint8_t>(1 + strlen(s)), osdsym::VOLT);
                snprintf(s, sizeof(s), "%.2f", static_cast<double>(d.cellVolts));
                print(bottom, 8, s);
                put(bottom, static_cast<uint8_t>(8 + strlen(s)), osdsym::VOLT);
            }
            if (d.hasCurrent)
            {
                snprintf(s, sizeof(s), "%.1f", static_cast<double>(d.amps));
                print(static_cast<uint8_t>(bottom + 1), 1, s);
                put(static_cast<uint8_t>(bottom + 1), static_cast<uint8_t>(1 + strlen(s)), osdsym::AMP);
                snprintf(s, sizeof(s), "%d", static_cast<int>(d.mahDrawn));
                print(static_cast<uint8_t>(bottom + 1), 8, s);
                put(static_cast<uint8_t>(bottom + 1), static_cast<uint8_t>(8 + strlen(s)), osdsym::MAH);
            }
            // время в полёте
            put(bottom, 22, osdsym::FLY_TIME);
            snprintf(s, sizeof(s), "%02lu:%02lu", static_cast<unsigned long>(d.flightSeconds / 60 % 100),
                     static_cast<unsigned long>(d.flightSeconds % 60));
            print(bottom, 23, s);
            // одно главное предупреждение — в центре, под прицелом
            const char* warning = warningText(d);
            if (warning) print(static_cast<uint8_t>(rows / 2 + 2), static_cast<uint8_t>((COLS - strlen(warning)) / 2), warning);
        }

        // Самое важное, что сейчас не так (или nullptr).
        static const char* warningText(const OsdData& d)
        {
            if (d.failsafe) return "FAILSAFE";
            if (d.linkLost) return "RXLOSS";
            if (d.batteryState == 3) return "BATT CRITICAL";
            if (d.batteryState == 2) return "LOW BATTERY";
            if (!d.armed && d.batteryNotFull) return "BATT < FULL";
            if (!d.armed && d.armBlock) return d.armBlock;
            if (!d.armed) return "DISARMED";
            return nullptr;
        }

    private:
        static void formatDistance(char* out, size_t size, float m)
        {
            if (m < 1000.0f) snprintf(out, size, "%dM", static_cast<int>(lroundf(m)));
            else snprintf(out, size, "%.1fKM", static_cast<double>(m / 1000.0f));
        }

        uint8_t cells[MAX_ROWS][COLS];
        uint8_t rows = MAX_ROWS;
    };
}
