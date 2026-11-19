#pragma once
#include <math.h>
#include <stdint.h>
#include "copter/GpsRescue.h"

// ============================================================
// Домой вслепую (P43 в): GPS подменяют или глушат, а возвращаться надо.
// Своё, у BF такого нет (у него GPS пропал — DISARM, у нас до этого —
// посадка на месте).
//
// Где мы: последняя достоверная точка GPS плюс путь, посчитанный своими
// датчиками. Скорость относительно воздуха даёт модель фильтра ориентации
// (copter/Attitude.h, MEKF): горизонтальная часть тяги разгоняет коптер,
// сопротивление винтов −k·v тормозит — по наклону скорость известна без
// GPS, ошибка ограничена (сопротивление гасит её за 1/k секунд). К ней —
// ветер, выученный тем же фильтром, пока GPS был. Оси модели — рысканье
// гироскопа; в земные переводит поправка курса (компас или курс по полёту).
//
// Путь считается всегда, и пока GPS есть, — с отметками каждую секунду:
// медленный увод ловят не сразу, поэтому тогда точка и ветер берутся за
// LOOKBACK_S до обнаружения, а путь — с той же отметки. При глушении и
// скачке (виден сразу) последняя точка честная — берётся она.
//
// Честно: ошибка растёт на 5–15 % пройденного (ветер меняется, k известен
// неточно), поэтому — неуверенность errorM() и предел по времени
// deadreck_max_s. Этапы:
//   CLIMB    — высота возврата (как у GPS Rescue), нос уже к дому, место
//              держится против ветра;
//   FLY_HOME — нос туда, куда надо лететь относительно воздуха (с поправкой
//              на ветер), наклон — под скорость gps_rescue_ground_speed по
//              модели tg θ = k·v/g; ближе descent_dist — медленнее и ниже;
//   LAND     — по оценке дом ближе LAND_RADIUS_M или время вышло:
//              снижение descend_rate, против ветра — стоим на месте;
//              касание — DISARM (контроллер).
//
// Без Arduino — тестируется на ПК (test_copter, test_copter_sim).
// ============================================================

namespace copter
{
    enum class DeadReckonPhase : uint8_t { IDLE, CLIMB, FLY_HOME, LAND };

    inline const char* deadReckonPhaseName(DeadReckonPhase p)
    {
        switch (p)
        {
            case DeadReckonPhase::CLIMB: return "CLIMB";
            case DeadReckonPhase::FLY_HOME: return "FLY_HOME";
            case DeadReckonPhase::LAND: return "LAND";
            case DeadReckonPhase::IDLE:
            default: return "IDLE";
        }
    }

    // Что нужно на каждом цикле (контроллер собирает из фильтра и GPS).
    struct DeadReckonInput
    {
        bool gpsGood = false;          // в этом цикле — достоверный фикс с домом (не подменён)
        float gpsN = 0.0f, gpsE = 0.0f;   // от дома, м
        bool modelOk = false;          // MEKF работает и коптер в воздухе
        float airX = 0.0f, airY = 0.0f;   // скорость относительно воздуха, оси рысканья гироскопа, м/с
        bool headingOk = false;        // поправка курса известна (компас или курс по полёту)
        float headingOffsetDeg = 0.0f; // курс носа минус рысканье гироскопа
        float windN = 0.0f, windE = 0.0f;   // ветер (куда дует), м/с
        float windSigma = 0.0f;        // его неуверенность, м/с
        float dragCoef = 0.0f;         // сопротивление винтов, 1/с
        float dragSigma = 0.0f;        // его неуверенность, доля
    };

    struct DeadReckonSettings
    {
        float maxS = 120.0f;           // deadreck_max_s: дольше — посадка где есть; 0 — выключено
    };

    class DeadReckonHome
    {
    public:
        static constexpr float LOOKBACK_S = 10.0f;      // подмена: точка — за столько до неё
        static constexpr uint8_t MARKS = 12;            // отметки раз в секунду (больше LOOKBACK_S)
        static constexpr float GPS_ERR_M = 3.0f;        // ошибка самой точки GPS
        static constexpr float HEADING_ERR = 0.03f;     // ≈ 2° курса — доля пути вбок
        static constexpr float MODEL_ERR = 0.05f;       // наименьшая ошибка скорости по модели, доля
        static constexpr float GUST_MS = 0.3f;          // порывы, которых фильтр не видит, м/с
        static constexpr float WIND_ERR = 0.5f;         // доля σ ветра фильтра, что копится в пути (σ у него с запасом)
        static constexpr float LAND_RADIUS_M = 6.0f;
        static constexpr float MAX_YAW_RATE = 90.0f;
        static constexpr float G = 9.80665f;
        static constexpr uint8_t PATH_WINDOW_S = 8;     // сверка пути: окно (отметок назад)
        static constexpr float PATH_TOL_M = 6.0f;       // допуск: шум GPS и порывы
        static constexpr float PATH_SHARE = 0.15f;      // и доля пути (k, курс)
        static constexpr float PATH_DRAG_SIGMA = 0.15f; // сопротивление выучено не хуже — сверять

        void configure(const DeadReckonSettings& s, const GpsRescueSettings& r)
        {
            set = s;
            rescue = r;
        }
        const DeadReckonSettings& settings() const { return set; }

        void resetFlight()
        {
            phase_ = DeadReckonPhase::IDLE;
            anchored = false;
            pathN = pathE = 0.0f;
            airPathN = airPathE = 0.0f;
            clock = 0.0f;
            markCount = 0;
            markTimer = 0.0f;
            modelFor = 0.0f;
            pathRatio_ = -1.0f;
        }

        // Каждый цикл, пока заармлен: путь по модели и отметки с GPS.
        void track(const DeadReckonInput& in, float dt)
        {
            last = in;
            if (in.headingOk) offsetDeg = in.headingOffsetDeg;
            float vn = 0.0f, ve = 0.0f, an = 0.0f, ae = 0.0f;
            groundVelocity(vn, ve, an, ae);
            clock += dt;
            if (in.modelOk && headingKnown())
            {
                pathN += vn * dt;
                pathE += ve * dt;
                airPathN += an * dt;
                airPathE += ae * dt;
                modelFor += dt;
            }
            else
            {
                modelFor = 0.0f;
            }
            markTimer += dt;
            if (phase_ != DeadReckonPhase::IDLE)
            {
                // в пути: место — по модели, неуверенность растёт
                if (in.modelOk && headingKnown())
                {
                    posN += vn * dt;
                    posE += ve * dt;
                }
                const float air = sqrtf(in.airX * in.airX + in.airY * in.airY);
                const float ground = sqrtf(vn * vn + ve * ve);
                err += (ground * HEADING_ERR + air * MODEL_ERR + WIND_ERR * windSigma_) * dt;
                elapsed_ += dt;
                return;
            }
            if (in.gpsGood)
            {
                anchored = true;
                fixN = in.gpsN;
                fixE = in.gpsE;
                fixPathN = pathN;
                fixPathE = pathE;
                if (markTimer >= 1.0f || markCount == 0)
                {
                    markTimer = 0.0f;
                    for (uint8_t i = MARKS - 1; i > 0; --i) marks[i] = marks[i - 1];
                    marks[0] = { in.gpsN, in.gpsE, pathN, pathE, in.windN, in.windE, airPathN, airPathE, clock, in.windSigma };
                    if (markCount < MARKS) ++markCount;
                    comparePaths(in);
                }
            }
            else if (!in.modelOk)
            {
                pathRatio_ = -1.0f;
            }
        }

        // Можно ли лететь вслепую: включено, есть точка, модель и курс.
        bool available() const { return set.maxS > 0.0f && anchored && last.modelOk && headingKnown(); }

        // Начать: spoofed — GPS медленно уводили (точка и ветер — за LOOKBACK_S
        // до обнаружения; скачок виден сразу — его так не начинают);
        // targetAlt — высота возврата (GpsRescue::returnAltitude).
        void start(bool spoofed, float targetAlt)
        {
            windN_ = last.windN;
            windE_ = last.windE;
            windSigma_ = last.windSigma;
            float dN = pathN - fixPathN, dE = pathE - fixPathE;   // путь от последнего фикса
            posN = fixN;
            posE = fixE;
            float since = 0.0f;   // сколько секунд путь считала модель
            if (spoofed && markCount > 0)
            {
                const uint8_t i = static_cast<uint8_t>(fminf(static_cast<float>(markCount - 1), LOOKBACK_S));
                const Mark& m = marks[i];
                // ветер фильтр учил и по подменённому GPS — берём, каким он был
                // тогда, и путь с той отметки — относительно воздуха плюс он
                windN_ = m.windN;
                windE_ = m.windE;
                windSigma_ = m.windSigma;
                since = clock - m.t;
                posN = m.n;
                posE = m.e;
                dN = airPathN - m.airN + windN_ * since;
                dE = airPathE - m.airE + windE_ * since;
            }
            posN += dN;
            posE += dE;
            const float dist = sqrtf(dN * dN + dE * dE);
            err = GPS_ERR_M + dist * (HEADING_ERR + MODEL_ERR) + since * WIND_ERR * windSigma_;
            elapsed_ = 0.0f;
            returnAlt = targetAlt;
            altTarget = targetAlt;
            phase_ = distanceHome() < rescue.minStartDistM ? DeadReckonPhase::LAND : DeadReckonPhase::CLIMB;
        }

        void stop() { phase_ = DeadReckonPhase::IDLE; }

        bool active() const { return phase_ != DeadReckonPhase::IDLE; }
        bool landing() const { return phase_ == DeadReckonPhase::LAND; }
        DeadReckonPhase phase() const { return phase_; }
        float northM() const { return posN; }
        float eastM() const { return posE; }
        float errorM() const { return err; }
        float elapsedS() const { return elapsed_; }
        float distanceHome() const { return sqrtf(posN * posN + posE * posE); }
        bool timedOut() const { return elapsed_ > set.maxS; }

        // Сверка для доверия GPS (copter/GpsIntegrity.h): путь по GPS против
        // пути по модели за PATH_WINDOW_S — отношение невязки к допуску (< 0 —
        // сравнить нельзя: модель не выучена или не работала всё окно). Модель
        // от GPS не зависит (наклон и сопротивление): увод, который GPS Rescue
        // наполовину гасит сам, здесь виден — но так же выглядит и резкая
        // смена ветра, поэтому только доверие, не защёлка.
        float pathRatio() const { return pathRatio_; }

        // Один цикл: altM — высота над точкой ARM, yawDeg — рысканье гироскопа.
        RescueCommand update(float altM, float yawDeg, float dt)
        {
            (void)dt;
            RescueCommand c;
            if (phase_ == DeadReckonPhase::IDLE) return c;
            const float dist = distanceHome();
            if (phase_ != DeadReckonPhase::LAND && (timedOut() || dist <= LAND_RADIUS_M))
                phase_ = DeadReckonPhase::LAND;

            // какая скорость по земле нужна
            float wantN = 0.0f, wantE = 0.0f;
            if (phase_ == DeadReckonPhase::FLY_HOME && dist > 0.1f)
            {
                float speed = rescue.groundSpeed;
                if (dist < rescue.descentDistM)
                {
                    const float k = fmaxf(0.0f, fminf(1.0f, dist / fmaxf(1.0f, rescue.descentDistM)));
                    speed = fmaxf(1.0f, speed * k);
                    altTarget = rescue.landingAltM + (returnAlt - rescue.landingAltM) * k;
                }
                wantN = -posN / dist * speed;
                wantE = -posE / dist * speed;
            }
            // относительно воздуха — с поправкой на ветер
            const float airN = wantN - windN_, airE = wantE - windE_;
            const float air = sqrtf(airN * airN + airE * airE);
            const float heading = yawDeg + offsetDeg;
            // нос — на дом (на подъёме и в пути); на посадке не крутим
            if (phase_ != DeadReckonPhase::LAND)
            {
                const float bearing = phase_ == DeadReckonPhase::FLY_HOME && air > 0.5f ? atan2f(airE, airN) * RAD2DEG
                                                                                        : atan2f(-posE, -posN) * RAD2DEG;
                c.yawRateDps = fmaxf(-MAX_YAW_RATE, fminf(MAX_YAW_RATE, wrap180(bearing - heading) * rescue.yawP * 0.1f));
            }
            // наклон под скорость: в установившемся полёте g·tg θ = k·v
            const float h = heading * DEG2RAD;
            const float fwd = airN * cosf(h) + airE * sinf(h);
            const float side = -airN * sinf(h) + airE * cosf(h);
            const float k = fmaxf(0.05f, last.dragCoef);
            const float maxTilt = rescue.maxAngleDeg;
            c.pitchDeg = -clampDeg(atanf(k * fwd / G) * RAD2DEG, maxTilt);
            c.rollDeg = clampDeg(atanf(k * side / G) * RAD2DEG, maxTilt);

            switch (phase_)
            {
                case DeadReckonPhase::CLIMB:
                    c.climbRate = rescue.ascendRate;
                    if (altM >= returnAlt - 1.0f) phase_ = DeadReckonPhase::FLY_HOME;
                    break;
                case DeadReckonPhase::FLY_HOME:
                {
                    const float e = altTarget - altM;
                    c.climbRate = fmaxf(-rescue.descendRate, fminf(rescue.ascendRate, e));
                    break;
                }
                case DeadReckonPhase::LAND:
                default:
                    c.climbRate = altM > rescue.landingAltM ? -rescue.descendRate : -0.5f * rescue.descendRate;
                    break;
            }
            return c;
        }

    private:
        static constexpr float DEG2RAD = 0.017453293f;
        static constexpr float RAD2DEG = 57.29578f;

        struct Mark
        {
            float n, e, pathN, pathE, windN, windE;
            float airN, airE, t, windSigma;   // путь относительно воздуха, время, неуверенность ветра
        };

        static float wrap180(float d)
        {
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        }
        static float clampDeg(float v, float lim) { return fmaxf(-lim, fminf(lim, v)); }

        bool headingKnown() const { return last.headingOk || headingSeen; }

        void comparePaths(const DeadReckonInput& in)
        {
            if (markCount <= PATH_WINDOW_S || modelFor < PATH_WINDOW_S + 1.0f || in.dragSigma > PATH_DRAG_SIGMA)
            {
                pathRatio_ = -1.0f;
                return;
            }
            // ветер — каким он был в начале окна: фильтр учит его по тому же
            // GPS и подменённую скорость за пару секунд принимает за ветер
            const Mark& a = marks[PATH_WINDOW_S];
            const Mark& b = marks[0];
            const float w = b.t - a.t;
            const float gN = b.n - a.n, gE = b.e - a.e;
            const float mN = b.airN - a.airN + a.windN * w, mE = b.airE - a.airE + a.windE * w;
            const float e = sqrtf((gN - mN) * (gN - mN) + (gE - mE) * (gE - mE));
            const float tol = PATH_TOL_M + PATH_SHARE * sqrtf(mN * mN + mE * mE) + w * (a.windSigma + GUST_MS);
            pathRatio_ = e / tol;
        }

        // скорость по земле: воздух (оси рысканья → земные: an, ae) плюс ветер
        void groundVelocity(float& vn, float& ve, float& an, float& ae)
        {
            if (last.headingOk) headingSeen = true;
            const float d = offsetDeg * DEG2RAD, c = cosf(d), s = sinf(d);
            const bool frozen = phase_ != DeadReckonPhase::IDLE;
            an = c * last.airX - s * last.airY;
            ae = s * last.airX + c * last.airY;
            vn = an + (frozen ? windN_ : last.windN);
            ve = ae + (frozen ? windE_ : last.windE);
        }

        DeadReckonSettings set;
        GpsRescueSettings rescue;
        DeadReckonInput last;
        DeadReckonPhase phase_ = DeadReckonPhase::IDLE;
        float offsetDeg = 0.0f;
        bool headingSeen = false;
        bool anchored = false;
        float fixN = 0.0f, fixE = 0.0f, fixPathN = 0.0f, fixPathE = 0.0f;
        float pathN = 0.0f, pathE = 0.0f;   // путь по модели с ARM
        Mark marks[MARKS] = {};
        uint8_t markCount = 0;
        float markTimer = 0.0f;
        float posN = 0.0f, posE = 0.0f, err = 0.0f, elapsed_ = 0.0f;
        float returnAlt = 0.0f, altTarget = 0.0f;
        float windN_ = 0.0f, windE_ = 0.0f;   // ветер в пути — замороженный на старте
        float windSigma_ = 0.0f;
        float modelFor = 0.0f;                 // модель работает подряд, с
        float clock = 0.0f;                    // с ARM, с
        float airPathN = 0.0f, airPathE = 0.0f;   // путь относительно воздуха (без ветра)
        float pathRatio_ = -1.0f;
    };
}
