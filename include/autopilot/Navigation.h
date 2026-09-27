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

    float wrap180(float degrees);

    float wrap360(float degrees);

    // Смещение точки b относительно a, метры на север и восток.
    void offsetNE(const GeoPoint& a, const GeoPoint& b, float& north, float& east);

    float distance(const GeoPoint& a, const GeoPoint& b);

    // Направление из a на b, 0..360.
    float bearing(const GeoPoint& a, const GeoPoint& b);

    // Точка в north/east метрах от a.
    GeoPoint moved(const GeoPoint& a, float north, float east);

    GeoPoint fromGps(const GpsData& gps);
}

namespace Guidance
{
    // Крен, чтобы довернуть с курса course на курс target: пропорционально
    // ошибке курса, но не больше bankLimit.
    float rollForCourse(float targetDeg, float courseDeg, float bankLimitDeg);

    // Наведение на окружность радиусом radius вокруг центра ("векторное
    // поле"): bearingFromCenter — направление от центра на самолёт,
    // distance — расстояние до центра. На окружности — касательная,
    // далеко снаружи — прямо на центр, внутри — по касательной наружу.
    float orbitCourse(float bearingFromCenterDeg, float distanceM, float radiusM, bool clockwise);

    // Крен установившегося круга радиусом radius на скорости speed:
    // tg φ = V² / (g·R).
    float orbitBankDeg(float speedMs, float radiusM);
}
