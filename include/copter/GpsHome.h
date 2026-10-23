#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// Дом коптера по GPS: точка взлёта, расстояние и направление до неё — для
// status, OSD (стрелка домой) и телеметрии в пульт. Режимов по GPS пока
// нет (возврат домой — пункт плана коптера), только знание, где дом.
//
// Дом ставится при ARM, если есть 3D-фикс и хватает спутников
// (gps_minimum_sats, как в BF); не было — при первом хорошем фиксе уже в
// полёте (status об этом скажет). DISARM дом не стирает — до следующего ARM.
//
// Расстояния — плоская Земля вокруг дома (до десятков километров ошибка
// меньше метра на сотни метров), этого хватает для стрелки и цифр.
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    struct GpsFix
    {
        bool valid = false;        // 3D-фикс
        uint8_t satellites = 0;
        double latitude = 0.0, longitude = 0.0;   // градусы
        float altitudeM = 0.0f;    // над уровнем моря
        float speedMs = 0.0f;      // по земле
        float courseDeg = 0.0f;    // куда едем, 0 — север
        float accuracyM = 99.0f;   // по горизонтали
    };

    class GpsHome
    {
    public:
        static constexpr float EARTH_RADIUS_M = 6371000.0f;

        void setMinimumSats(uint8_t n) { minSats = n; }

        bool good(const GpsFix& f) const { return f.valid && f.satellites >= minSats; }

        // Каждый раз, когда пришёл новый фикс; armed — заармлен ли коптер.
        void update(const GpsFix& f, bool armed)
        {
            last = f;
            if (armed && !wasArmed)
            {
                homeSet = false;   // новый ARM — новый дом
                setInFlight = false;
            }
            if (armed && !homeSet && good(f))
            {
                homeLat = f.latitude;
                homeLon = f.longitude;
                homeAlt = f.altitudeM;
                homeSet = true;
                setInFlight = armedTimeFixes > 0;
            }
            if (armed && !homeSet) ++armedTimeFixes;
            if (!armed) armedTimeFixes = 0;
            wasArmed = armed;
        }

        bool hasHome() const { return homeSet; }
        bool homeSetInFlight() const { return setInFlight; }
        const GpsFix& fix() const { return last; }

        // Расстояние до дома, м (0 — дома нет).
        float distanceM() const
        {
            if (!homeSet) return 0.0f;
            float north, east;
            offset(north, east);
            return sqrtf(north * north + east * east);
        }

        // Направление ОТ коптера НА дом, градусы от севера 0..360.
        float bearingHomeDeg() const
        {
            if (!homeSet) return 0.0f;
            float north, east;
            offset(north, east);
            float deg = atan2f(-east, -north) * 57.2957795f;
            if (deg < 0.0f) deg += 360.0f;
            return deg;
        }

        // Куда повернуть до дома относительно носа, −180..180 (для стрелки OSD).
        float relativeHomeDeg(float headingDeg) const
        {
            float d = bearingHomeDeg() - headingDeg;
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        }

        // Высота над домом по GPS, м.
        float altitudeAboveHomeM() const { return homeSet ? last.altitudeM - homeAlt : 0.0f; }

    private:
        // Смещение коптера от дома: на север и на восток, м.
        void offset(float& north, float& east) const
        {
            const double toRad = 3.14159265358979323846 / 180.0;
            north = static_cast<float>((last.latitude - homeLat) * toRad * EARTH_RADIUS_M);
            east = static_cast<float>((last.longitude - homeLon) * toRad * EARTH_RADIUS_M * cos(homeLat * toRad));
        }

        GpsFix last;
        uint8_t minSats = 8;
        bool homeSet = false;
        bool setInFlight = false;
        bool wasArmed = false;
        uint32_t armedTimeFixes = 0;
        double homeLat = 0.0, homeLon = 0.0;
        float homeAlt = 0.0f;
    };
}
