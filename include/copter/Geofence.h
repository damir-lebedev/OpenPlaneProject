#pragma once

// Геозабор (P44, своё; в BF нет): граница, за которую коптер не вылетит,
// что бы ни делал пилот, — в отличие от учебных ограничений (TrainingLimits.h)
// работает в любом режиме, включая ACRO и failsafe, и пилот его в полёте не
// выключит (настройки при ARM не меняются).
//   круг   — geofence_radius от дома (точка ARM): летит наружу быстрее
//            тормозного пути — в ANGLE/HORIZON/POS HOLD наклон уменьшается;
//            точка остановки (с запасом на реакцию) за краем — автомат
//            (GPS Rescue) сразу тормозит и ведёт назад до 80 % радиуса, стики
//            не слушаются; внутри и не летит наружу — снова пилот;
//   потолок — geofence_alt над точкой ARM: газ (и набор в ALT_HOLD) срезается
//            так, чтобы успеть затормозить; нужен барометр;
//   без GPS — с включённым забором не армится; в полёте место потеряно
//            (GPS пропал, подменён): наклон стиком — наименьший; худший случай
//            (последняя точка, скорость и разгон пилота LOST_ACCEL) может
//            достать до края или прошло NO_GPS_GRACE_S — по geofence_no_gps:
//            WARN — только предупреждение, RETURN — домой (без GPS — вслепую
//            по своим датчикам, DeadReckonHome.h; нечем — сесть), LAND — сесть.
// Показ: в очках — радар (круг забора вокруг коптера, нос вверх) и
// расстояние до края со стрелкой (Osd.h), предупреждения FENCE…

#include <math.h>
#include <stdint.h>

#include "copter/TrainingLimits.h"

namespace copter
{
    enum class GeofenceNoGps : uint8_t { WARN, RETURN, LAND };

    struct GeofenceSettings
    {
        bool enabled = false;                       // geofence
        float radiusM = 200.0f;                     // geofence_radius, м от дома; 0 — круга нет
        float ceilingM = 120.0f;                    // geofence_alt, м над точкой ARM; 0 — потолка нет
        GeofenceNoGps noGps = GeofenceNoGps::RETURN;   // geofence_no_gps

        bool circle() const { return enabled && radiusM > 0.0f; }
    };

    // Биты: что сейчас делает забор (OSD, живые кадры, Studio).
    enum GeofenceFlag : uint8_t
    {
        FENCE_NEAR = 1u << 0,     // у края (дальше 80 % радиуса или ближе 20 м)
        FENCE_RETURN = 1u << 1,   // автомат ведёт назад, стики не слушаются
        FENCE_CEILING = 1u << 2,  // у потолка: газ срезан
        FENCE_NO_GPS = 1u << 3,   // место неизвестно (GPS нет или подменён)
        FENCE_LAND = 1u << 4,     // садится: место неизвестно, вернуться нечем
        FENCE_OUTSIDE = 1u << 5,  // за краем (ветер, пилот обогнал автомат)
    };

    // Предупреждение в очках (или nullptr).
    inline const char* geofenceText(uint8_t flags)
    {
        if (flags & FENCE_LAND) return "FENCE LAND";
        if (flags & FENCE_RETURN) return "FENCE RETURN";
        if (flags & FENCE_NO_GPS) return "FENCE NO GPS";
        if (flags & FENCE_NEAR) return "FENCE";
        if (flags & FENCE_CEILING) return "FENCE ALT";
        return nullptr;
    }

    class Geofence
    {
    public:
        static constexpr float WARN_FRACTION = 0.8f;    // дальше этой доли радиуса — FENCE
        static constexpr float WARN_MIN_M = 20.0f;      // или ближе к краю, чем столько
        static constexpr float RELEASE_FRACTION = 0.8f; // вернулся сюда и не летит наружу — управление пилоту
        static constexpr float RELEASE_OUTWARD = 0.5f;  // м/с: «не летит наружу»
        static constexpr float RETURN_BRAKE = 3.0f;     // м/с²: на такое торможение автомата рассчитываем
        static constexpr float REACTION_S = 0.4f;       // с: пока автомат перехватит и наклонится
        static constexpr float SOFT_BRAKE = 2.0f;       // м/с²: мягкое торможение наклоном к краю
        static constexpr float SOFT_SPAN = 3.0f;        // м/с: на столько быстрее — наклон к минимуму
        static constexpr float MIN_ANGLE_K = 0.15f;     // наклон не меньше этой доли
        static constexpr float ANGLE_SLEW = 3.0f;       // 1/с: как быстро меняется доля наклона
        static constexpr float CEILING_ZONE_M = 5.0f;   // ниже потолка на столько — FENCE ALT
        static constexpr float NO_GPS_GRACE_S = 3.0f;   // столько без места — действие geofence_no_gps
        static constexpr float GPS_BACK_S = 2.0f;       // место снова есть столько подряд — ему можно верить
        static constexpr float GPS_STALE_S = 0.5f;      // столько без нового фикса — места не знаем (фикс 5–10 Гц)
        static constexpr float LOST_ACCEL = 8.0f;       // м/с²: так может разгоняться пилот (ACRO), пока места не знаем

        void configure(const GeofenceSettings& s) { set = s; }
        const GeofenceSettings& settings() const { return set; }
        bool enabled() const { return set.enabled; }

        void reset()
        {
            flags_ = 0;
            returning = false;
            landing = false;
            noGpsTimer = 0.0f;
            gpsBackTimer = GPS_BACK_S;
            angleK = 1.0f;
            dist = 0.0f;
            lastDist = lastSpeed = 0.0f;
        }

        // Раз в цикл, пока заармлен: altM — над точкой ARM (baroOk — с барометром),
        // distM — от дома, outwardMs — скорость от дома, speedMs — по земле
        // (posOk — место известно: дом есть, GPS свежий и не подменён), airborne — в воздухе.
        void update(float altM, bool baroOk, float distM, float outwardMs, float speedMs, bool posOk, bool airborne, float dt)
        {
            flags_ = 0;
            alt = altM;
            dist = distM;
            ceilingValid = set.enabled && set.ceilingM > 0.0f && baroOk;
            if (ceilingValid && altM > set.ceilingM - CEILING_ZONE_M) flags_ |= FENCE_CEILING;
            if (!set.circle())
            {
                returning = landing = false;
                angleK = 1.0f;
                return;
            }

            // место потеряно — уже GPS_STALE_S без фикса
            noGpsTimer = posOk ? 0.0f : noGpsTimer > 0.0f ? noGpsTimer + dt : GPS_STALE_S;
            gpsBackTimer = posOk ? gpsBackTimer + dt : 0.0f;
            if (!posOk) flags_ |= FENCE_NO_GPS;

            float k = 1.0f;
            bool lost = false;
            if (!posOk)
            {
                // худший случай: летит от дома с последней скоростью и ещё разгоняется
                const float t = noGpsTimer;
                const float v = lastSpeed + LOST_ACCEL * t;
                const float d = lastDist + lastSpeed * t + 0.5f * LOST_ACCEL * t * t;
                lost = t > NO_GPS_GRACE_S || d + v * REACTION_S + v * v / (2.0f * RETURN_BRAKE) > set.radiusM;
                k = MIN_ANGLE_K;   // стиком не разогнаться
            }
            else
            {
                lastDist = distM;
                lastSpeed = speedMs;
            }
            if (posOk)
            {
                const float out = fmaxf(0.0f, outwardMs);
                const float stop = distM + out * REACTION_S + out * out / (2.0f * RETURN_BRAKE);
                if (stop > set.radiusM && airborne) returning = true;
                if (distM > fminf(WARN_FRACTION * set.radiusM, set.radiusM - WARN_MIN_M)) flags_ |= FENCE_NEAR;
                if (distM > set.radiusM) flags_ |= FENCE_OUTSIDE;
                const float excess = outwardMs - TrainingLimits::brakeSpeed(set.radiusM - distM, SOFT_BRAKE);
                if (excess > 0.0f) k = fmaxf(MIN_ANGLE_K, 1.0f - excess / SOFT_SPAN * (1.0f - MIN_ANGLE_K));
                // месту снова можно верить, внутри и не летит наружу — пилоту
                if (returning && gpsBackTimer >= GPS_BACK_S && distM < RELEASE_FRACTION * set.radiusM &&
                    outwardMs < RELEASE_OUTWARD)
                {
                    returning = false;
                }
                if (gpsBackTimer >= GPS_BACK_S) landing = false;
            }
            else if (lost && airborne)
            {
                if (set.noGps == GeofenceNoGps::RETURN) returning = true;
                if (set.noGps == GeofenceNoGps::LAND) landing = true;
            }
            if (!airborne) returning = landing = false;
            if (returning) flags_ |= FENCE_RETURN;
            if (landing) flags_ |= FENCE_LAND;

            const float step = ANGLE_SLEW * dt;
            angleK += fmaxf(-step, fminf(step, k - angleK));
        }

        uint8_t flags() const { return flags_; }

        // Вести назад автоматом (GPS Rescue, без GPS — вслепую).
        bool wantReturn() const { return returning; }
        // Сесть на месте: geofence_no_gps LAND или вернуться нечем (решает контроллер).
        bool wantLand() const { return landing; }
        // Контроллер: вернуть нечем (нет GPS и модели для полёта вслепую) — садиться.
        void forceLand() { if (set.circle()) landing = true; }

        // Потолок для автомата возврата; без потолка — 0.
        float rescueCeilingM() const { return set.enabled && set.ceilingM > 0.0f ? set.ceilingM : 0.0f; }

        // Наибольший наклон в ANGLE/HORIZON/POS HOLD: к краю — меньше.
        float angleCap(float maxAngleDeg) const { return set.circle() ? maxAngleDeg * angleK : maxAngleDeg; }

        // ALT_HOLD: заданный набор, м/с, — не выше допустимого у потолка.
        float limitClimb(float climb) const
        {
            if (!ceilingValid) return climb;
            return fminf(climb, climbCap());
        }

        // Ручной газ: тяга не больше той, что гасит подъём к потолку.
        float limitThrust(float thrust, float climbRate, float hover, float cosTilt) const
        {
            if (!ceilingValid) return thrust;
            const float hold = hover / fmaxf(0.5f, cosTilt);
            float cap = hold * (1.0f + TrainingLimits::THRUST_GAIN * (climbCap() - climbRate));
            cap = fmaxf(0.25f * hover, fminf(1.0f, cap));
            return fminf(thrust, cap);
        }

        // До края, м (< 0 — за ним); без круга — 0.
        float marginM() const { return set.circle() ? set.radiusM - dist : 0.0f; }

    private:
        float climbCap() const { return TrainingLimits::brakeSpeed(set.ceilingM - alt, TrainingLimits::ALT_BRAKE); }

        GeofenceSettings set;
        uint8_t flags_ = 0;
        bool returning = false;
        bool landing = false;
        bool ceilingValid = false;
        float alt = 0.0f, dist = 0.0f;
        float noGpsTimer = 0.0f;
        float lastDist = 0.0f, lastSpeed = 0.0f;   // последнее известное место: от дома и скорость, м и м/с
        float gpsBackTimer = GPS_BACK_S;
        float angleK = 1.0f;
    };
}
