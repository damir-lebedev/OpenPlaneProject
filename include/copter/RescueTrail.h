#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// След полёта для возврата (P45 а, своё; в BF нет): точки пути от дома
// с высотой — там коптер уже пролетел, значит, путь свободен. Возврат
// (copter/GpsRescue.h, этап TRAIL) идёт по ним назад, пока коптер ниже,
// чем летал на ближайших rth_path_m метрах пути (залетел под кроны, во
// двор, под крышу), или пока прямая домой пересекает запретную зону
// (P44 б2) — а не поднимается вертикально в ветки.
//
// Точка — когда коптер сдвинулся на STEP_M (по земле или по высоте), по
// свежему достоверному фиксу и только в воздухе, не во время возврата.
// Хранение: SIZE точек по 6 байт (север/восток по 0.5 м, высота по 0.1 м).
// Заполнился — старшая половина прореживается через одну: новый путь
// всегда подробный, старый — грубее, но след всегда доходит до дома.
//
// Петли срезаются: на каждой точке возврат ищет самую старую точку следа
// рядом (CUT_M по земле, CUT_ALT_M по высоте) и идёт сразу к ней.
//
// Без Arduino — тестируется на ПК (test_copter, test_copter_sim).
// ============================================================

namespace copter
{
    class RescueTrail
    {
    public:
        static constexpr uint8_t SIZE = 128;
        static constexpr float STEP_M = 3.0f;
        static constexpr float CUT_M = 4.0f;
        static constexpr float CUT_ALT_M = 3.0f;
        static constexpr float POS_UNIT = 0.5f;   // м на единицу севера/востока
        static constexpr float ALT_UNIT = 0.1f;   // м на единицу высоты

        void reset() { count = 0; }
        uint8_t size() const { return count; }

        // Положение от дома и высота над точкой ARM; ближе STEP_M к последней — не пишется.
        void record(float n, float e, float alt)
        {
            if (count)
            {
                float ln, le, la;
                point(static_cast<uint8_t>(count - 1), ln, le, la);
                if (hypotf(n - ln, e - le) < STEP_M && fabsf(alt - la) < STEP_M) return;
            }
            if (count == SIZE) thin();
            pts[count].n = pack(n, POS_UNIT);
            pts[count].e = pack(e, POS_UNIT);
            pts[count].alt = pack(alt, ALT_UNIT);
            ++count;
        }

        void point(uint8_t i, float& n, float& e, float& alt) const
        {
            n = pts[i].n * POS_UNIT;
            e = pts[i].e * POS_UNIT;
            alt = pts[i].alt * ALT_UNIT;
        }

        // Наибольшая высота следа от точки i к дому на pathM метрах пути (по земле).
        float maxAltAhead(uint8_t i, float pathM) const
        {
            if (!count) return -1e9f;
            if (i >= count) i = static_cast<uint8_t>(count - 1);
            float n, e, a;
            point(i, n, e, a);
            float best = a, path = 0.0f;
            for (int j = i - 1; j >= 0 && path < pathM; --j)
            {
                float n2, e2, a2;
                point(static_cast<uint8_t>(j), n2, e2, a2);
                path += hypotf(n2 - n, e2 - e);
                n = n2;
                e = e2;
                if (a2 > best) best = a2;
            }
            return best;
        }

        // Наибольшая высота на последних pathM метрах полёта (от новой точки к дому).
        float maxAltRecent(float pathM) const { return count ? maxAltAhead(static_cast<uint8_t>(count - 1), pathM) : -1e9f; }

        // Срезать петлю: самая старая точка не новее i рядом с (n, e, alt), иначе i.
        uint8_t cutLoop(uint8_t i, float n, float e, float alt) const
        {
            for (uint8_t j = 0; j < i && j < count; ++j)
            {
                float pn, pe, pa;
                point(j, pn, pe, pa);
                if (hypotf(pn - n, pe - e) < CUT_M && fabsf(pa - alt) < CUT_ALT_M) return j;
            }
            return i;
        }

    private:
        struct Crumb
        {
            int16_t n, e, alt;
        };

        static int16_t pack(float v, float unit)
        {
            const float q = roundf(v / unit);
            return static_cast<int16_t>(fmaxf(-32767.0f, fminf(32767.0f, q)));
        }

        // Старшая половина — через одну (первая, у дома, остаётся): освобождается четверть.
        void thin()
        {
            const uint8_t half = SIZE / 2;
            uint8_t w = 1;
            for (uint8_t r = 2; r < half; r = static_cast<uint8_t>(r + 2)) pts[w++] = pts[r];
            for (uint8_t r = half; r < count; ++r) pts[w++] = pts[r];
            count = w;
        }

        Crumb pts[SIZE];
        uint8_t count = 0;
    };
}
