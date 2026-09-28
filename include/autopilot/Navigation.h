#pragma once
#include <Arduino.h>
#include <math.h>

#include "config/Config.h"
#include "sensors/SensorInterface.h"

// ============================================================
// НАВИГАЦИЯ: координаты, дом, наведение по курсу и по окружности
//
// Геометрия — в локальной плоскости "север/восток" в метрах
// (равнопромежуточная проекция вокруг средней широты). Для полётов
// модели в пределах нескольких километров ошибка — доли процента,
// а считать проще и быстрее, чем по большому кругу.
//
// Курсы — в градусах 0..360 по часовой от севера (как у GPS), крен —
// авиационный знак (+ правое крыло вниз = поворот вправо).
// ============================================================

struct GeoPoint
{
    double lat = 0;  // градусы, + север
    double lon = 0;  // градусы, + восток
};

namespace Geo
{
    constexpr double EARTH_RADIUS_M = 6371000.0;
    constexpr double DEG = 3.14159265358979323846 / 180.0;
    constexpr float GRAVITY = 9.80665f;

    inline float wrap180(float degrees)
    {
        float d = fmodf(degrees + 180.0f, 360.0f);
        if (d < 0) d += 360.0f;
        return d - 180.0f;
    }

    inline float wrap360(float degrees)
    {
        float d = fmodf(degrees, 360.0f);
        if (d < 0) d += 360.0f;
        return d;
    }

    // Смещение точки b относительно a, метры на север и восток.
    inline void offsetNE(const GeoPoint& a, const GeoPoint& b, float& north, float& east)
    {
        const double midLat = (a.lat + b.lat) * 0.5 * DEG;
        north = static_cast<float>((b.lat - a.lat) * DEG * EARTH_RADIUS_M);
        east = static_cast<float>((b.lon - a.lon) * DEG * EARTH_RADIUS_M * cos(midLat));
    }

    inline float distance(const GeoPoint& a, const GeoPoint& b)
    {
        float n, e;
        offsetNE(a, b, n, e);
        return sqrtf(n * n + e * e);
    }

    // Направление из a на b, 0..360.
    inline float bearing(const GeoPoint& a, const GeoPoint& b)
    {
        float n, e;
        offsetNE(a, b, n, e);
        return wrap360(atan2f(e, n) / static_cast<float>(DEG));
    }

    // Точка в north/east метрах от a.
    inline GeoPoint moved(const GeoPoint& a, float north, float east)
    {
        GeoPoint p;
        p.lat = a.lat + north / EARTH_RADIUS_M / DEG;
        p.lon = a.lon + east / (EARTH_RADIUS_M * cos(a.lat * DEG)) / DEG;
        return p;
    }

    inline GeoPoint fromGps(const GpsData& gps)
    {
        GeoPoint p;
        p.lat = gps.latitude;
        p.lon = gps.longitude;
        return p;
    }
}

namespace Guidance
{
    // Крен, чтобы довернуть с курса course на курс target: пропорционально
    // ошибке курса, но не больше bankLimit.
    inline float rollForCourse(float targetDeg, float courseDeg, float bankLimitDeg)
    {
        const float error = Geo::wrap180(targetDeg - courseDeg);
        return constrain(error * Config::NAV_COURSE_GAIN, -bankLimitDeg, bankLimitDeg);
    }

    // Наведение на окружность радиусом radius вокруг центра ("векторное
    // поле"): bearingFromCenter — направление от центра на самолёт,
    // distance — расстояние до центра. На окружности — касательная,
    // далеко снаружи — прямо на центр, внутри — по касательной наружу.
    inline float orbitCourse(float bearingFromCenterDeg, float distanceM, float radiusM, bool clockwise)
    {
        const float radius = max(radiusM, 1.0f);
        const float offset = 90.0f + atanf(Config::LOITER_CONVERGENCE * (distanceM - radius) / radius) /
                                         static_cast<float>(Geo::DEG);
        return Geo::wrap360(bearingFromCenterDeg + (clockwise ? offset : -offset));
    }

    // Крен установившегося круга радиусом radius на скорости speed:
    // tg φ = V² / (g·R).
    inline float orbitBankDeg(float speedMs, float radiusM)
    {
        return atanf(speedMs * speedMs / (Geo::GRAVITY * max(radiusM, 1.0f))) / static_cast<float>(Geo::DEG);
    }
}
