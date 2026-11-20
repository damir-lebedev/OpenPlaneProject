#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/DeclinationTable.h"

// ============================================================
// Магнитное склонение по координатам GPS — чтобы не вводить mag_declination
// руками (как inav_auto_mag_decl в INAV).
//
// Источник — World Magnetic Model WMM-2025 (NOAA/BGS): таблица 10° x 10° на
// середину срока модели (copter/DeclinationTable.h, генерирует
// tools/gen_declination.py) и билинейная интерполяция между узлами. Ошибка
// против полной модели на 2025…2030: медиана 0.15°, 95 % точек — до 0.8°
// (замерено генератором), хуже — только у магнитных полюсов, где компасу и
// так не верим.
//
// AutoDeclination решает, когда пересчитывать: первый 3D-фикс — сразу, потом —
// когда коптер уехал больше чем на 10 км от точки прошлого расчёта (склонение
// меняется медленно, тригонометрия на каждый отсчёт GPS не нужна). Фикс
// пропал — значение остаётся последним посчитанным.
//
// Без Arduino — тестируется на ПК.
// ============================================================

namespace copter
{
    // Склонение в точке, градусы: + — магнитный север восточнее истинного.
    inline float declinationAt(double latDeg, double lonDeg)
    {
        using namespace declination_table;
        float lat = static_cast<float>(latDeg), lon = static_cast<float>(lonDeg);
        if (!(lat == lat) || !(lon == lon)) return 0.0f;   // NaN
        if (lat < LAT_MIN) lat = static_cast<float>(LAT_MIN);
        if (lat > LAT_MAX) lat = static_cast<float>(LAT_MAX);
        while (lon < LON_MIN) lon += 360.0f;
        while (lon >= LON_MAX) lon -= 360.0f;
        const float fy = (lat - LAT_MIN) / STEP, fx = (lon - LON_MIN) / STEP;
        int i = static_cast<int>(fy), j = static_cast<int>(fx);
        if (i > ROWS - 2) i = ROWS - 2;
        if (j > COLS - 2) j = COLS - 2;
        const float ty = fy - i, tx = fx - j;
        // углы клетки — относительно первого, чтобы не прыгать через ±180 у полюсов
        const float c0 = DECI_DEG[i][j] * 0.1f;
        auto rel = [c0](int16_t v) {
            float d = v * 0.1f - c0;
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        };
        const float c1 = rel(DECI_DEG[i][j + 1]), c2 = rel(DECI_DEG[i + 1][j]), c3 = rel(DECI_DEG[i + 1][j + 1]);
        const float top = c1 * tx, bottom = c2 + (c3 - c2) * tx;
        float d = c0 + top + (bottom - top) * ty;
        while (d > 180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return d;
    }

    class AutoDeclination
    {
    public:
        static constexpr float REFRESH_KM = 10.0f;

        // Новый отсчёт GPS. true — склонение пересчитано (можно отдать компасу).
        bool update(bool fix, double latDeg, double lonDeg)
        {
            if (!fix) return false;
            if (known && distanceKm(latDeg, lonDeg) < REFRESH_KM) return false;
            lat = latDeg;
            lon = lonDeg;
            deg = declinationAt(latDeg, lonDeg);
            known = true;
            return true;
        }

        bool valid() const { return known; }
        float value() const { return deg; }

    private:
        float distanceKm(double la, double lo) const
        {
            const float dLat = static_cast<float>(la - lat);
            float dLon = static_cast<float>(lo - lon);
            if (dLon > 180.0f) dLon -= 360.0f;
            if (dLon < -180.0f) dLon += 360.0f;
            const float k = cosf(static_cast<float>(lat) * 0.017453293f);
            return 111.2f * sqrtf(dLat * dLat + dLon * dLon * k * k);
        }

        bool known = false;
        double lat = 0.0, lon = 0.0;
        float deg = 0.0f;
    };
}
