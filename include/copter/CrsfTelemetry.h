#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// Телеметрия в пульт по CRSF (ExpressLRS, Crossfire): полётник пишет кадры
// в тот же UART, откуда берёт каналы, приёмник отправляет их на пульт —
// EdgeTX (RadioMaster TX12) показывает их на экране и в «Телеметрии»:
// напряжение, ток, мА·ч и остаток (Bat), GPS (координаты, спутники,
// скорость, высота), ориентацию (крен, тангаж, курс), режим полёта и
// вертикальную скорость.
//
// Кадр (по открытому описанию CRSF): 0xC8 длина тип данные… CRC8 —
// длина считает тип, данные и CRC; CRC8 (полином 0xD5) — по типу и
// данным; числа — старшим байтом вперёд.
//
// Кадры по кругу, по одному за раз (send() — не чаще раза в 20 мс): так
// линия не забивается, а каждый кадр приходит в пульт несколько раз в
// секунду. GPS — только когда он есть.
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    constexpr uint8_t CRSF_ADDRESS_FC = 0xC8;
    constexpr uint8_t CRSF_FRAME_GPS = 0x02;
    constexpr uint8_t CRSF_FRAME_VARIO = 0x07;
    constexpr uint8_t CRSF_FRAME_BATTERY = 0x08;
    constexpr uint8_t CRSF_FRAME_ATTITUDE = 0x1E;
    constexpr uint8_t CRSF_FRAME_FLIGHT_MODE = 0x21;

    inline uint8_t crsfCrc(const uint8_t* data, uint8_t length)
    {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; ++b) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
        }
        return crc;
    }

    // Что показать в пульте (единицы — человеческие, перевод — здесь).
    struct CrsfTelemetryData
    {
        float volts = 0.0f, amps = 0.0f, mahDrawn = 0.0f, percentLeft = 0.0f;
        float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
        float climbMs = 0.0f;
        bool gps = false;
        double latitude = 0.0, longitude = 0.0;
        float speedMs = 0.0f, courseDeg = 0.0f, altitudeM = 0.0f;
        uint8_t satellites = 0;
        char mode[16] = "ACRO";   // как в BF: ACRO, ANGL, HOR, ALTH, !FS!; без ARM — с «*»
    };

    class CrsfTelemetry
    {
    public:
        static constexpr uint8_t MAX_FRAME = 64;

        // Следующий кадр по кругу в out; длина (0 — нечего слать).
        uint8_t next(const CrsfTelemetryData& d, uint8_t* out)
        {
            for (uint8_t tries = 0; tries < KINDS; ++tries)
            {
                const uint8_t kind = turn;
                turn = static_cast<uint8_t>((turn + 1) % KINDS);
                const uint8_t n = build(kind, d, out);
                if (n) return n;
            }
            return 0;
        }

        static uint8_t build(uint8_t kind, const CrsfTelemetryData& d, uint8_t* out)
        {
            uint8_t p[MAX_FRAME];
            uint8_t n = 0;
            uint8_t type = 0;
            switch (kind)
            {
                case 0:   // батарея: 0.1 В, 0.1 А, мА·ч (24 бита), %
                {
                    type = CRSF_FRAME_BATTERY;
                    n = put16(p, n, clampU16(lroundf(d.volts * 10.0f)));
                    n = put16(p, n, clampU16(lroundf(d.amps * 10.0f)));
                    const uint32_t mah = static_cast<uint32_t>(fmaxf(0.0f, d.mahDrawn)) & 0xFFFFFFu;
                    p[n++] = static_cast<uint8_t>(mah >> 16);
                    p[n++] = static_cast<uint8_t>(mah >> 8);
                    p[n++] = static_cast<uint8_t>(mah);
                    p[n++] = static_cast<uint8_t>(fminf(100.0f, fmaxf(0.0f, d.percentLeft)));
                    break;
                }
                case 1:   // ориентация: тангаж, крен, курс — радианы × 10000
                {
                    type = CRSF_FRAME_ATTITUDE;
                    n = put16(p, n, static_cast<uint16_t>(radians10000(d.pitchDeg)));
                    n = put16(p, n, static_cast<uint16_t>(radians10000(d.rollDeg)));
                    n = put16(p, n, static_cast<uint16_t>(radians10000(wrap180(d.yawDeg))));
                    break;
                }
                case 2:   // режим полёта — строка с нулём
                {
                    type = CRSF_FRAME_FLIGHT_MODE;
                    const size_t len = strnlen(d.mode, sizeof(d.mode) - 1);
                    memcpy(p, d.mode, len);
                    n = static_cast<uint8_t>(len);
                    p[n++] = 0;
                    break;
                }
                case 3:   // вариометр: см/с
                {
                    type = CRSF_FRAME_VARIO;
                    n = put16(p, n, static_cast<uint16_t>(static_cast<int16_t>(lroundf(fminf(320.0f, fmaxf(-320.0f, d.climbMs)) * 100.0f))));
                    break;
                }
                case 4:   // GPS: широта/долгота × 1e7, км/ч × 10, курс × 100, высота + 1000 м, спутники
                {
                    if (!d.gps) return 0;
                    type = CRSF_FRAME_GPS;
                    n = put32(p, n, static_cast<uint32_t>(static_cast<int32_t>(llround(d.latitude * 1e7))));
                    n = put32(p, n, static_cast<uint32_t>(static_cast<int32_t>(llround(d.longitude * 1e7))));
                    n = put16(p, n, clampU16(lroundf(d.speedMs * 3.6f * 10.0f)));
                    float course = fmodf(d.courseDeg, 360.0f);
                    if (course < 0.0f) course += 360.0f;
                    n = put16(p, n, clampU16(lroundf(course * 100.0f)));
                    n = put16(p, n, clampU16(lroundf(d.altitudeM + 1000.0f)));
                    p[n++] = d.satellites;
                    break;
                }
                default: return 0;
            }
            out[0] = CRSF_ADDRESS_FC;
            out[1] = static_cast<uint8_t>(n + 2);   // тип + данные + CRC
            out[2] = type;
            memcpy(out + 3, p, n);
            out[3 + n] = crsfCrc(out + 2, static_cast<uint8_t>(n + 1));
            return static_cast<uint8_t>(n + 4);
        }

        // Режим как в BF (4 буквы); без ARM — «*» в конце.
        static void modeName(char* out, size_t size, const char* mode, bool armed, bool failsafe)
        {
            if (failsafe)
            {
                snprintf(out, size, "!FS!");
                return;
            }
            snprintf(out, size, "%s%s", mode, armed ? "" : "*");
        }

    private:
        static constexpr uint8_t KINDS = 5;

        static uint8_t put16(uint8_t* p, uint8_t n, uint16_t v)
        {
            p[n++] = static_cast<uint8_t>(v >> 8);
            p[n++] = static_cast<uint8_t>(v);
            return n;
        }
        static uint8_t put32(uint8_t* p, uint8_t n, uint32_t v)
        {
            p[n++] = static_cast<uint8_t>(v >> 24);
            p[n++] = static_cast<uint8_t>(v >> 16);
            p[n++] = static_cast<uint8_t>(v >> 8);
            p[n++] = static_cast<uint8_t>(v);
            return n;
        }
        static uint16_t clampU16(long v) { return static_cast<uint16_t>(v < 0 ? 0 : v > 65535 ? 65535 : v); }
        static int16_t radians10000(float deg) { return static_cast<int16_t>(lroundf(deg * 0.0174532925f * 10000.0f)); }
        static float wrap180(float deg)
        {
            while (deg > 180.0f) deg -= 360.0f;
            while (deg < -180.0f) deg += 360.0f;
            return deg;
        }

        uint8_t turn = 0;
    };
}
