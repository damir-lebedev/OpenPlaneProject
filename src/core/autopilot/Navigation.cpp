// Реализация autopilot/Navigation.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/Navigation.h"

namespace Geo
{

float wrap180(float degrees)
{
    float d = fmodf(degrees + 180.0f, 360.0f);
    if (d < 0) d += 360.0f;
    return d - 180.0f;
}

float wrap360(float degrees)
{
    float d = fmodf(degrees, 360.0f);
    if (d < 0) d += 360.0f;
    return d;
}

void offsetNE(const GeoPoint& a, const GeoPoint& b, float& north, float& east)
{
    const double midLat = (a.lat + b.lat) * 0.5 * DEG;
    north = static_cast<float>((b.lat - a.lat) * DEG * EARTH_RADIUS_M);
    east = static_cast<float>((b.lon - a.lon) * DEG * EARTH_RADIUS_M * cos(midLat));
}

float distance(const GeoPoint& a, const GeoPoint& b)
{
    float n, e;
    offsetNE(a, b, n, e);
    return sqrtf(n * n + e * e);
}

float bearing(const GeoPoint& a, const GeoPoint& b)
{
    float n, e;
    offsetNE(a, b, n, e);
    return wrap360(atan2f(e, n) / static_cast<float>(DEG));
}

GeoPoint moved(const GeoPoint& a, float north, float east)
{
    GeoPoint p;
    p.lat = a.lat + north / EARTH_RADIUS_M / DEG;
    p.lon = a.lon + east / (EARTH_RADIUS_M * cos(a.lat * DEG)) / DEG;
    return p;
}

GeoPoint fromGps(const GpsData& gps)
{
    GeoPoint p;
    p.lat = gps.latitude;
    p.lon = gps.longitude;
    return p;
}

}  // namespace Geo

namespace Guidance
{

float rollForCourse(float targetDeg, float courseDeg, float bankLimitDeg)
{
    const float error = Geo::wrap180(targetDeg - courseDeg);
    return constrain(error * Config::NAV_COURSE_GAIN, -bankLimitDeg, bankLimitDeg);
}

float orbitCourse(float bearingFromCenterDeg, float distanceM, float radiusM, bool clockwise)
{
    const float radius = max(radiusM, 1.0f);
    const float offset = 90.0f + atanf(Config::LOITER_CONVERGENCE * (distanceM - radius) / radius) /
                                     static_cast<float>(Geo::DEG);
    return Geo::wrap360(bearingFromCenterDeg + (clockwise ? offset : -offset));
}

float orbitBankDeg(float speedMs, float radiusM)
{
    return atanf(speedMs * speedMs / (Geo::GRAVITY * max(radiusM, 1.0f))) / static_cast<float>(Geo::DEG);
}

}  // namespace Guidance
