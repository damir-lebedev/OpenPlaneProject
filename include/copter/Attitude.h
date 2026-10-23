#pragma once
#include <math.h>

#include "copter/Filters.h"

// ============================================================
// Ориентация мультикоптера: кватернион + поправка по акселерометру
// (фильтр Махони).
//
// Самолётный AttitudeEstimator складывает угловые скорости прямо с
// углами Эйлера — для самолёта, который не крутит сальто, этого
// хватает. Коптер в акро делает флипы и висит на любом крене, поэтому
// здесь — кватернион: повороты складываются точно при любой
// ориентации, без «замка» на тангаже 90°.
//
// Акселерометр на коптере — плохая опора: тяга всегда вдоль оси Z
// рамы, и при разгоне он «видит» горизонт там, где его нет. Верен он
// только в среднем (в установившемся полёте сопротивление уравновешивает
// тягу). Поэтому поправка медленная — постоянная времени ~2 с в полёте
// (на земле до взлёта — 0.2 с, чтобы быстро встать в горизонт), и
// выключается совсем, когда модуль ускорения далёк от 1 g (вираж,
// рывок газом, удар).
//
// Оси — авиационные, как во всей прошивке: гироскоп — крен (+ правый
// борт вниз), тангаж (+ нос вверх), рысканье (+ нос вправо), °/с;
// акселерометр — g, X к носу, Y влево, Z вверх. Внутри — связанная
// система FRD (X вперёд, Y вправо, Z вниз): угловые скорости в ней
// совпадают с авиационными знаками.
// ============================================================

namespace copter
{
    struct Vec3
    {
        float x = 0, y = 0, z = 0;
    };

    inline Vec3 cross(const Vec3& a, const Vec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float length(const Vec3& a) { return sqrtf(dot(a, a)); }

    class AttitudeQuat
    {
    public:
        // Постоянные времени поправки по акселерометру, с.
        void configure(float flightTauS, float groundTauS)
        {
            kFlight = flightTauS > 0.0f ? 1.0f / flightTauS : 0.0f;
            kGround = groundTauS > 0.0f ? 1.0f / groundTauS : 0.0f;
        }

        void reset()
        {
            w = 1.0f;
            x = y = z = 0.0f;
            initialized = false;
        }

        // На земле до взлёта поправка быстрая.
        void setGrounded(bool g) { grounded = g; }

        // gyroDps — крен/тангаж/рысканье, °/с; accelG — X нос, Y влево, Z вверх.
        void update(const float gyroDps[3], const float accelG[3], float dtS)
        {
            const Vec3 a = { accelG[0], -accelG[1], -accelG[2] };   // FRD
            const float aLen = length(a);
            if (!initialized)
            {
                if (aLen > 0.5f) levelFrom(a);
                initialized = true;
                return;
            }

            const float d2r = PI_F / 180.0f;
            float p = gyroDps[0] * d2r, q = gyroDps[1] * d2r, r = gyroDps[2] * d2r;

            // поправка: измеренный «верх» (удельная сила) против оценки. В полёте
            // акселерометр видит направление тяги — с тяжестью оно совпадает
            // только в спокойном полёте: не при рывке (модуль не 1 g), не во
            // вращении (флип на газе висения — ровно 1 g, но «вверх» крутится
            // вместе с рамой) и не тогда, когда расходится с гироскопом на
            // десятки градусов (так далеко гироскоп не уплывает).
            float trust = fmaxf(0.0f, 1.0f - fabsf(aLen - 1.0f) / 0.15f);
            if (!grounded)
            {
                const float spinDps = sqrtf(gyroDps[0] * gyroDps[0] + gyroDps[1] * gyroDps[1] + gyroDps[2] * gyroDps[2]);
                trust *= fminf(1.0f, fmaxf(0.0f, 1.0f - (spinDps - 30.0f) / 70.0f));
            }
            const float k = (grounded ? kGround : kFlight) * trust;
            if (k > 0.0f && aLen > 0.1f)
            {
                const Vec3 measured = { a.x / aLen, a.y / aLen, a.z / aLen };
                const Vec3 estimated = upBody();
                const float gain = (!grounded && dot(measured, estimated) < COS_25) ? 0.0f : k;
                const Vec3 e = cross(measured, estimated);
                p += gain * e.x;
                q += gain * e.y;
                r += gain * e.z;
            }

            // q̇ = ½·q ⊗ (0, ω)
            const float hw = 0.5f * dtS;
            const float nw = w + hw * (-x * p - y * q - z * r);
            const float nx = x + hw * (w * p + y * r - z * q);
            const float ny = y + hw * (w * q - x * r + z * p);
            const float nz = z + hw * (w * r + x * q - y * p);
            const float n = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
            w = nw / n;
            x = nx / n;
            y = ny / n;
            z = nz / n;
        }

        // Направление «вниз» Земли в осях рамы (FRD), единичный вектор.
        Vec3 downBody() const
        {
            return { 2.0f * (x * z - w * y), 2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y) };
        }

        Vec3 upBody() const
        {
            const Vec3 d = downBody();
            return { -d.x, -d.y, -d.z };
        }

        // Ось Z рамы (вниз), спроецированная на вертикаль: 1 — ровно, 0 — на боку, −1 — вверх ногами.
        float cosTilt() const { return 1.0f - 2.0f * (x * x + y * y); }
        float tiltDeg() const { return acosf(fminf(1.0f, fmaxf(-1.0f, cosTilt()))) * 180.0f / PI_F; }

        float rollDeg() const { return atan2f(2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y)) * 180.0f / PI_F; }
        float pitchDeg() const
        {
            const float s = fminf(1.0f, fmaxf(-1.0f, 2.0f * (w * y - x * z)));
            return asinf(s) * 180.0f / PI_F;
        }
        float yawDeg() const { return atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / PI_F; }

        // Вектор из рамы (FRD) в земные оси (NED).
        Vec3 toEarth(const Vec3& v) const
        {
            return {
                (1.0f - 2.0f * (y * y + z * z)) * v.x + 2.0f * (x * y - w * z) * v.y + 2.0f * (x * z + w * y) * v.z,
                2.0f * (x * y + w * z) * v.x + (1.0f - 2.0f * (x * x + z * z)) * v.y + 2.0f * (y * z - w * x) * v.z,
                2.0f * (x * z - w * y) * v.x + 2.0f * (y * z + w * x) * v.y + (1.0f - 2.0f * (x * x + y * y)) * v.z,
            };
        }

        // Вертикальное ускорение, м/с² (+ вверх), из показаний акселерометра (g).
        float verticalAccel(const float accelG[3]) const
        {
            const Vec3 f = toEarth({ accelG[0], -accelG[1], -accelG[2] });
            return (-f.z - 1.0f) * 9.80665f;
        }

        bool isInitialized() const { return initialized; }

        float qw() const { return w; }
        float qx() const { return x; }
        float qy() const { return y; }
        float qz() const { return z; }

    private:
        // Начальная ориентация — сразу по акселерометру (рысканье 0).
        void levelFrom(const Vec3& a)
        {
            const float roll = atan2f(-a.y, -a.z);
            const float pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z));
            const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
            const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
            w = cr * cp;
            x = sr * cp;
            y = cr * sp;
            z = -sr * sp;
        }

        static constexpr float COS_25 = 0.9063078f;

        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
        float kFlight = 0.5f, kGround = 5.0f;
        bool grounded = true;
        bool initialized = false;
    };
}
