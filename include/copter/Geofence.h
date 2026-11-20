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
//   поле   — многоугольники с карты (GeoZones.h, P44 б): FIELD — летать
//            внутри, NOFLY — запретные зоны; то же торможение и автомат,
//            «до края» — до ближайшей стороны; ARM — только внутри поля;
//            круг и поле вместе — действует то, что ближе;
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

#include "copter/GeoZones.h"
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
        const GeoZoneTable* zones = nullptr;        // поле и запретные зоны (geozone), владелец — CopterParams

        bool circle() const { return enabled && radiusM > 0.0f; }
        bool polygons() const { return enabled && zones && zones->anyValid(); }
        // Есть граница по земле (круг или поле): без GPS не армится.
        bool area() const { return circle() || polygons(); }
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
        static constexpr float POLY_RELEASE_K = 0.5f;   // поле: отпустить, когда до края больше половины того, что было при ARM

        void configure(const GeofenceSettings& s)
        {
            set = s;
            buildZones();
        }
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
            lastPolyMargin = 1e9f;
            armMargin = -1.0f;
            armSpan = 0.0f;
        }

        // На каждый новый фикс GPS (и без ARM — для проверки перед взлётом):
        // где коптер относительно поля и запретных зон.
        void updatePosition(double lat, double lon, float velN, float velE)
        {
            if (!polys) return;
            GeoZoneTable::offsetM(ref, lat, lon, posN, posE);
            float field = -1e9f, fieldUN = 0.0f, fieldUE = 0.0f;
            bool anyField = false;
            float best = 1e9f, uN = 0.0f, uE = 0.0f;
            for (uint8_t i = 0; i < zoneCount; ++i)
            {
                const LocalZone& z = local[i];
                const PolygonNearest r = polygonNearest(z.n, z.e, z.count, posN, posE);
                if (z.type == GeoZoneType::FIELD)
                {
                    // полей несколько — внутри любого: берём самое «глубокое»
                    const float m = r.inside ? r.distM : -r.distM;
                    anyField = true;
                    if (m > field)
                    {
                        field = m;
                        fieldUN = r.inside ? r.towardN : -r.towardN;
                        fieldUE = r.inside ? r.towardE : -r.towardE;
                    }
                }
                else
                {
                    const float m = r.inside ? -r.distM : r.distM;
                    if (m < best)
                    {
                        best = m;
                        uN = r.inside ? -r.towardN : r.towardN;
                        uE = r.inside ? -r.towardE : r.towardE;
                    }
                }
            }
            if (anyField && field < best)
            {
                best = field;
                uN = fieldUN;
                uE = fieldUE;
            }
            polyMargin = best;
            polyOutward = velN * uN + velE * uE;
            float deg = atan2f(uE, uN) * 57.2957795f;
            if (deg < 0.0f) deg += 360.0f;
            edgeDeg = deg;
            posKnown = true;
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
            if (!set.circle() && !polys)
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
                const float s = lastSpeed * t + 0.5f * LOST_ACCEL * t * t;
                const float lastMargin = fminf(set.circle() ? set.radiusM - lastDist : 1e9f, polys ? lastPolyMargin : 1e9f);
                lost = t > NO_GPS_GRACE_S || s + v * REACTION_S + v * v / (2.0f * RETURN_BRAKE) > lastMargin;
                k = MIN_ANGLE_K;   // стиком не разогнаться
            }
            else
            {
                lastDist = distM;
                lastSpeed = speedMs;
                if (polys && posKnown) lastPolyMargin = polyMargin;
            }
            if (posOk)
            {
                bool release = true;
                // одна граница: до края margin, к краю летит со скоростью outward
                auto edge = [&](float margin, float outward, float warnMargin, float releaseMargin) {
                    const float out = fmaxf(0.0f, outward);
                    const float stop = out * REACTION_S + out * out / (2.0f * RETURN_BRAKE);
                    if (stop > margin && airborne) returning = true;
                    if (margin < warnMargin) flags_ |= FENCE_NEAR;
                    if (margin < 0.0f) flags_ |= FENCE_OUTSIDE;
                    const float excess = outward - TrainingLimits::brakeSpeed(margin, SOFT_BRAKE);
                    if (excess > 0.0f) k = fminf(k, fmaxf(MIN_ANGLE_K, 1.0f - excess / SOFT_SPAN * (1.0f - MIN_ANGLE_K)));
                    if (margin <= releaseMargin || outward >= RELEASE_OUTWARD) release = false;
                };
                if (set.circle())
                {
                    const float m = set.radiusM - distM;
                    edge(m, outwardMs, fmaxf((1.0f - WARN_FRACTION) * set.radiusM, WARN_MIN_M),
                         (1.0f - RELEASE_FRACTION) * set.radiusM);
                }
                if (polys && posKnown)
                {
                    // запас при ARM: в маленьком поле «у края» и «отпустить» — от него
                    if (armMargin < 0.0f)
                    {
                        armMargin = fmaxf(0.0f, polyMargin);
                        armSpan = polygonSpanM();
                    }
                    const float rel = fminf(WARN_MIN_M, POLY_RELEASE_K * armMargin);
                    edge(polyMargin, polyOutward, rel, rel);
                }
                // месту снова можно верить, внутри и не летит наружу — пилоту
                if (returning && gpsBackTimer >= GPS_BACK_S && release) returning = false;
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
        void forceLand() { if (set.circle() || polys) landing = true; }

        // Потолок для автомата возврата; без потолка — 0.
        float rescueCeilingM() const { return set.enabled && set.ceilingM > 0.0f ? set.ceilingM : 0.0f; }

        // Наибольший наклон в ANGLE/HORIZON/POS HOLD: к краю — меньше.
        float angleCap(float maxAngleDeg) const { return set.circle() || polys ? maxAngleDeg * angleK : maxAngleDeg; }

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

        // До края, м (< 0 — за ним); без границы — 0. Круг и поле — что ближе.
        float marginM() const
        {
            float m = set.circle() ? set.radiusM - dist : 1e9f;
            if (polys && posKnown) m = fminf(m, polyMargin);
            return m < 1e8f ? m : 0.0f;
        }
        // Ближний край — сторона поля (стрелка в очках — edgeBearingDeg), иначе круг (прочь от дома).
        bool polygonNearer() const
        {
            return polys && posKnown && (!set.circle() || polyMargin < set.radiusM - dist);
        }
        // Куда ближняя сторона поля, градусы от севера.
        float edgeBearingDeg() const { return edgeDeg; }

        // ARM: поле есть — только внутри и вне запретных зон (место известно).
        bool armOutside() const { return polys && (!posKnown || polyMargin <= 0.0f); }

        // Поле в метрах (север/восток) от общей точки и коптер в тех же осях — для радара.
        bool hasPolygons() const { return polys; }
        uint8_t polygonCount() const { return zoneCount; }
        uint8_t polygonPoints(uint8_t i) const { return i < zoneCount ? local[i].count : 0; }
        bool polygonNoFly(uint8_t i) const { return i < zoneCount && local[i].type == GeoZoneType::NOFLY; }
        void polygonPoint(uint8_t i, uint8_t k, float& n, float& e) const
        {
            n = local[i].n[k];
            e = local[i].e[k];
        }
        bool positionKnown() const { return posKnown; }
        void positionM(float& n, float& e) const
        {
            n = posN;
            e = posE;
        }
        // Масштаб радара: дальняя точка поля от места ARM (постоянный весь полёт), м.
        float radarSpanM() const { return armSpan > 0.0f ? armSpan : polygonSpanM(); }
        // Размер поля для масштаба радара: от коптера до дальней точки, м.
        float polygonSpanM() const
        {
            float m = 0.0f;
            for (uint8_t i = 0; i < zoneCount; ++i)
            {
                for (uint8_t k = 0; k < local[i].count; ++k)
                    m = fmaxf(m, sqrtf((local[i].n[k] - posN) * (local[i].n[k] - posN) + (local[i].e[k] - posE) * (local[i].e[k] - posE)));
            }
            return m;
        }

    private:
        float climbCap() const { return TrainingLimits::brakeSpeed(set.ceilingM - alt, TrainingLimits::ALT_BRAKE); }

        struct LocalZone
        {
            GeoZoneType type = GeoZoneType::NONE;
            uint8_t count = 0;
            float n[GeoZoneTable::MAX_POINTS], e[GeoZoneTable::MAX_POINTS];
        };

        // Годные зоны -> метры от первой точки первой годной зоны (общие оси).
        void buildZones()
        {
            zoneCount = 0;
            polys = false;
            posKnown = false;
            if (!set.polygons()) return;
            const GeoZoneTable& t = *set.zones;
            bool haveRef = false;
            for (uint8_t i = 0; i < GeoZoneTable::MAX_ZONES; ++i)
            {
                if (t.problem(i)) continue;
                const GeoZoneTable::Zone& z = t.zone(i);
                if (!haveRef)
                {
                    ref = z.pts[0];
                    haveRef = true;
                }
                LocalZone& l = local[zoneCount++];
                l.type = z.type;
                l.count = z.count;
                for (uint8_t k = 0; k < z.count; ++k) GeoZoneTable::offsetM(ref, z.pts[k], l.n[k], l.e[k]);
            }
            polys = zoneCount > 0;
        }

        GeofenceSettings set;
        LocalZone local[GeoZoneTable::MAX_ZONES];
        uint8_t zoneCount = 0;
        bool polys = false;
        GeoPoint ref;
        bool posKnown = false;
        float posN = 0.0f, posE = 0.0f;
        float polyMargin = 1e9f, polyOutward = 0.0f, edgeDeg = 0.0f;
        float lastPolyMargin = 1e9f;
        float armMargin = -1.0f;
        float armSpan = 0.0f;
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
