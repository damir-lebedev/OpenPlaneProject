#pragma once

// Учебные ограничения (своё, в BF нет): для новичка — предел высоты,
// расстояния от дома, наклона и скорости в ANGLE. За пределом коптер не
// падает и не выключается — мягко не пускает дальше:
//   высота   — подъём не быстрее, чем успеет затормозить к пределу
//              (√(2·a·d)): газ (или набор в ALT_HOLD) срезается; вниз —
//              всегда можно; только с барометром (без него высота «плывёт»);
//   дальность — так же по скорости от дома (курс носа не нужен): летит
//              наружу быстрее тормозного пути — наклон уменьшается; у края
//              (90 %) — предупреждение FENCE; не успеет остановиться до края
//              (или уже за ним) — возврат домой автоматом GPS Rescue: он
//              тормозит и летит назад до 80 % радиуса, дальше снова пилот;
//              нужен дом (GPS при ARM);
//   наклон   — в ANGLE стик даёт не больше training_max_angle;
//   скорость — быстрее training_max_speed по GPS — наклон в ANGLE плавно
//              уменьшается, сопротивление воздуха тормозит.
// Ноль в пределе — этот предел выключен; training_limits OFF — все.

#include <math.h>
#include <stdint.h>

namespace copter
{
    struct TrainingSettings
    {
        bool enabled = false;      // training_limits
        float maxAltM = 50.0f;     // training_max_alt, м над точкой ARM; 0 — нет
        float maxDistM = 150.0f;   // training_max_dist, м от дома; 0 — нет
        float maxAngleDeg = 30.0f; // training_max_angle, ° в ANGLE; 0 — нет
        float maxSpeedMs = 11.0f;  // training_max_speed (км/ч в настройке), м/с; 0 — нет
    };

    // Биты: какой предел сейчас держит коптер (OSD, живые кадры, Studio).
    enum TrainingFlag : uint8_t
    {
        TRAINING_ALT = 1u << 0,     // у потолка: газ срезан
        TRAINING_FENCE = 1u << 1,   // у края круга (90 %) или за ним
        TRAINING_RETURN = 1u << 2,  // за краем: летит назад сам
        TRAINING_SPEED = 1u << 3,   // быстрее предела: наклон уменьшен
    };

    // Строка предупреждения в очках (или nullptr).
    inline const char* trainingText(uint8_t flags)
    {
        if (flags & TRAINING_RETURN) return "FENCE RETURN";
        if (flags & TRAINING_FENCE) return "FENCE";
        if (flags & TRAINING_ALT) return "ALT LIMIT";
        if (flags & TRAINING_SPEED) return "SPEED LIMIT";
        return nullptr;
    }

    class TrainingLimits
    {
    public:
        static constexpr float ALT_ZONE_M = 3.0f;      // ниже потолка на столько — предупреждение ALT LIMIT
        static constexpr float ALT_BRAKE = 4.0f;       // м/с²: торможение подъёма к потолку
        static constexpr float ALT_GAIN = 0.7f;        // 1/с: у самого потолка допустимый подъём = до него × это
        static constexpr float FENCE_BRAKE = 1.5f;     // м/с²: торможение к краю круга (сопротивлением воздуха)
        static constexpr float FENCE_SPAN = 3.0f;      // м/с: на столько быстрее тормозного пути — наклон к минимуму
        static constexpr float THRUST_GAIN = 0.25f;    // доля висения на 1 м/с лишнего подъёма
        static constexpr float FENCE_WARN = 0.9f;      // доля радиуса: предупреждение
        static constexpr float FENCE_RELEASE = 0.8f;   // вернулся сюда и не летит наружу — управление пилоту
        static constexpr float RELEASE_OUTWARD = 0.5f; // м/с: «не летит наружу»
        static constexpr float RETURN_BRAKE = 4.0f;    // м/с²: так тормозит автомат возврата
        static constexpr float SPEED_SPAN = 0.5f;      // на столько (доля предела) быстрее — наклон к минимуму
        static constexpr float SPEED_MIN_K = 0.15f;    // наклон не меньше этой доли
        static constexpr float SPEED_SLEW = 2.0f;      // 1/с: как быстро меняется доля наклона

        void configure(const TrainingSettings& s) { set = s; }
        const TrainingSettings& settings() const { return set; }
        bool enabled() const { return set.enabled; }

        void reset()
        {
            flags_ = 0;
            returning = false;
            speedK = 1.0f;
        }

        // Раз в цикл, пока заармлен: altM — над точкой ARM (baroOk — с
        // барометром), distM — от дома и outwardMs — скорость от дома
        // (homeOk — дом есть и GPS свежий), speedMs — скорость по земле
        // (gpsOk — ей можно верить).
        void update(float altM, bool baroOk, float distM, float outwardMs, bool homeOk, float speedMs, bool gpsOk, float dt)
        {
            flags_ = 0;
            alt = altM;
            altValid = set.enabled && set.maxAltM > 0.0f && baroOk;
            if (altValid && altM > set.maxAltM - ALT_ZONE_M) flags_ |= TRAINING_ALT;

            if (!set.enabled || set.maxDistM <= 0.0f)
            {
                returning = false;
            }
            else if (homeOk)
            {
                // точка остановки (автомат тормозит с RETURN_BRAKE) за краем — возврат сразу
                const float out = fmaxf(0.0f, outwardMs);
                if (distM + out * out / (2.0f * RETURN_BRAKE) > set.maxDistM) returning = true;
                else if (distM < FENCE_RELEASE * set.maxDistM && outwardMs < RELEASE_OUTWARD) returning = false;
                if (distM > FENCE_WARN * set.maxDistM) flags_ |= TRAINING_FENCE;
            }
            float fenceK = 1.0f;
            if (set.enabled && set.maxDistM > 0.0f && homeOk && !returning)
            {
                const float excess = outwardMs - brakeSpeed(set.maxDistM - distM, FENCE_BRAKE);
                if (excess > 0.0f) fenceK = fmaxf(SPEED_MIN_K, 1.0f - excess / FENCE_SPAN * (1.0f - SPEED_MIN_K));
            }
            // GPS пропал за краем — возврат остаётся (автомат сам решит, что можно)
            if (returning) flags_ |= TRAINING_FENCE | TRAINING_RETURN;

            float k = 1.0f;
            if (set.enabled && set.maxSpeedMs > 0.0f && gpsOk && speedMs > set.maxSpeedMs)
            {
                k = 1.0f - (speedMs - set.maxSpeedMs) / (SPEED_SPAN * set.maxSpeedMs) * (1.0f - SPEED_MIN_K);
                k = fmaxf(SPEED_MIN_K, k);
                flags_ |= TRAINING_SPEED;
            }
            k = fminf(k, fenceK);
            const float step = SPEED_SLEW * dt;
            speedK += fmaxf(-step, fminf(step, k - speedK));
        }

        uint8_t flags() const { return flags_; }

        // За краем круга — вернуть домой (GPS Rescue).
        bool wantReturn() const { return returning; }

        // Потолок для автомата возврата (GPS Rescue не поднимается выше); без предела — 0.
        float rescueCeilingM() const { return set.enabled && set.maxAltM > 0.0f ? set.maxAltM : 0.0f; }

        // Наибольший наклон стиком в ANGLE.
        float angleCap(float maxAngleDeg) const
        {
            if (!set.enabled) return maxAngleDeg;
            float a = set.maxAngleDeg > 0.0f ? fminf(maxAngleDeg, set.maxAngleDeg) : maxAngleDeg;
            return a * speedK;
        }

        // ALT_HOLD: заданный набор высоты, м/с, — не выше допустимого у потолка.
        float limitClimb(float climb) const
        {
            if (!altValid) return climb;
            return fminf(climb, climbCap());
        }

        // Ручной газ (ANGLE, HORIZON, ACRO): тяга не больше той, что гасит
        // подъём к потолку; hover — тяга висения, cosTilt — косинус наклона.
        float limitThrust(float thrust, float climbRate, float hover, float cosTilt) const
        {
            if (!altValid) return thrust;
            const float hold = hover / fmaxf(0.5f, cosTilt);
            float cap = hold * (1.0f + THRUST_GAIN * (climbCap() - climbRate));
            cap = fmaxf(0.25f * hover, fminf(1.0f, cap));
            return fminf(thrust, cap);
        }

    private:
        float climbCap() const { return brakeSpeed(set.maxAltM - alt, ALT_BRAKE); }

        // Скорость к пределу, с которой ещё успеть остановиться за d м
        // (у самого предела — линейно, за ним — назад).
        static float brakeSpeed(float d, float brake)
        {
            if (d <= 0.0f) return d * ALT_GAIN;
            return fminf(sqrtf(2.0f * brake * d), d * ALT_GAIN + 0.5f);
        }

        TrainingSettings set;
        uint8_t flags_ = 0;
        bool returning = false;
        bool altValid = false;
        float alt = 0.0f;
        float speedK = 1.0f;
    };
}
