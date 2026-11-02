#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// GPS Rescue — возврат домой по GPS, как в Betaflight: по потере связи
// (failsafe_procedure = GPS-RESCUE, этап 2) или режимом GPS RESCUE с
// тумблера. Своя реализация по описанию поведения BF, код BF не копирован.
//
// Этапы:
//   CLIMB     — набор высоты возврата (gps_rescue_alt_mode: MAX_ALT —
//               наибольшая высота полёта + initial_climb, FIXED_ALT —
//               return_alt, CURRENT_ALT — текущая + initial_climb) со
//               скоростью ascend_rate; нос уже поворачивает к дому;
//   FLY_HOME  — нос на дом (yaw_p), скорость по земле ground_speed:
//               тангаж — PID по скорости вдоль носа (velocity_p/i/d), крен
//               гасит снос вбок (ветер), наклон не больше max_angle; пока нос
//               не на доме — скорость меньше;
//   DESCEND   — ближе descent_dist: скорость и высота убывают к дому,
//               внизу — landing_alt;
//   LAND      — над домом: удержание места, снижение descend_rate, ниже
//               landing_alt — вдвое медленнее; касание — DISARM (контроллер);
//   ABORT     — проверка не прошла: возврат невозможен, контроллер садит на
//               месте (AUTO-LAND). У BF здесь DISARM — мы выбираем посадку.
//
// Ближе min_start_dist — сразу LAND (BF: «слишком близко»).
//
// Проверки (gps_rescue_sanity_checks: OFF / ON / FS_ONLY — только при потере
// связи): нет дома или фикса, спутников меньше min_sats, GPS молчит 2 с,
// дом не приближается 15 с, коптер улетел от дома дальше, чем был, на 50 м.
//
// Курс: с компасом — по компасу; без него — гироскоп с поправкой по курсу
// GPS в полёте домой: крен тогда не трогаем, и на скорости курс GPS — это
// нос (снос ветром — его ошибка). С компасом крен гасит снос вбок.
//
// Знаки — как во всей прошивке: крен > 0 — правый борт вниз (влево — < 0),
// тангаж > 0 — нос вверх (вперёд — < 0), рысканье > 0 — нос вправо, курс
// 0 — север, по часовой.
//
// Без Arduino — тестируется на ПК (test_copter, test_copter_sim).
// ============================================================

namespace copter
{
    enum class RescueAltMode : uint8_t { MAX_ALT, FIXED_ALT, CURRENT_ALT };
    enum class RescueSanity : uint8_t { OFF, ON, FS_ONLY };

    struct GpsRescueSettings
    {
        float minStartDistM = 15.0f;    // gps_rescue_min_start_dist
        RescueAltMode altMode = RescueAltMode::MAX_ALT;
        float initialClimbM = 10.0f;    // gps_rescue_initial_climb
        float ascendRate = 7.5f;        // gps_rescue_ascend_rate, м/с
        float returnAltM = 30.0f;       // gps_rescue_return_alt
        float groundSpeed = 7.5f;       // gps_rescue_ground_speed, м/с
        float maxAngleDeg = 45.0f;      // gps_rescue_max_angle
        float yawP = 20.0f;             // gps_rescue_yaw_p: десятые °/с на градус
        float descentDistM = 20.0f;     // gps_rescue_descent_dist
        float descendRate = 1.5f;       // gps_rescue_descend_rate, м/с
        float landingAltM = 4.0f;       // gps_rescue_landing_alt
        uint8_t minSats = 8;            // gps_rescue_min_sats
        RescueSanity sanity = RescueSanity::FS_ONLY;
        bool allowArmingWithoutFix = false;
        float velocityP = 8.0f, velocityI = 40.0f, velocityD = 12.0f;   // gps_rescue_velocity_*
    };

    // Что знает коптер о GPS и курсе (main_copter.cpp, симулятор — тестам).
    struct GpsInput
    {
        bool fresh = false;             // в этом цикле — новый фикс
        bool fix = false;               // 3D
        uint8_t sats = 0;
        bool home = false;
        float northM = 0.0f, eastM = 0.0f;   // от дома
        float velN = 0.0f, velE = 0.0f;      // м/с по земле
        float headingDeg = 0.0f;        // курс носа, 0 — север (компас или оценка)
        bool headingTrusted = false;    // по компасу
    };

    enum class RescuePhase : uint8_t { IDLE, CLIMB, FLY_HOME, DESCEND, LAND, ABORT };

    inline const char* rescuePhaseName(RescuePhase p)
    {
        switch (p)
        {
            case RescuePhase::CLIMB: return "CLIMB";
            case RescuePhase::FLY_HOME: return "FLY_HOME";
            case RescuePhase::DESCEND: return "DESCEND";
            case RescuePhase::LAND: return "LAND";
            case RescuePhase::ABORT: return "ABORT";
            case RescuePhase::IDLE:
            default: return "IDLE";
        }
    }

    // Почему возврат прервали (или не начали).
    enum class RescueFail : uint8_t { NONE, NO_HOME, NO_FIX, LOW_SATS, GPS_LOST, NO_PROGRESS, FLYAWAY };

    inline const char* rescueFailName(RescueFail f)
    {
        switch (f)
        {
            case RescueFail::NO_HOME: return "NO_HOME";
            case RescueFail::NO_FIX: return "NO_FIX";
            case RescueFail::LOW_SATS: return "LOW_SATS";
            case RescueFail::GPS_LOST: return "GPS_LOST";
            case RescueFail::NO_PROGRESS: return "NO_PROGRESS";
            case RescueFail::FLYAWAY: return "FLYAWAY";
            case RescueFail::NONE:
            default: return "NONE";
        }
    }

    // Команда контроллеру на этот цикл.
    struct RescueCommand
    {
        float rollDeg = 0.0f, pitchDeg = 0.0f;   // целевой наклон (как стик ANGLE)
        float yawRateDps = 0.0f;
        float climbRate = 0.0f;                  // м/с, > 0 — вверх
    };

    class GpsRescue
    {
    public:
        static constexpr float GPS_TIMEOUT_S = 2.0f;
        static constexpr float PROGRESS_WINDOW_S = 15.0f;
        static constexpr float FLYAWAY_M = 50.0f;
        static constexpr float ALIGN_SPEED = 2.5f;     // м/с: курс GPS — это курс носа
        static constexpr float ALIGN_TAU_S = 4.0f;           // потом — медленно следить за уходом гироскопа
        static constexpr float ALIGN_MAX_YAW_RATE = 10.0f;   // °/с
        static constexpr float ALIGN_STRAIGHT_S = 1.0f;
        static constexpr float ALIGN_PITCH_DEG = 3.0f;   // летим вперёд, а не тормозим
        static constexpr float GROUND_LEVEL_M = 1.0f;
        static constexpr float MAX_YAW_RATE = 90.0f;
        static constexpr float MAX_ROLL_DEG = 25.0f;
        static constexpr float LAND_RADIUS_M = 6.0f;      // ближе — посадка с удержанием точки
        static constexpr float NO_YAW_DIST_M = 12.0f;
        static constexpr float LAND_GAIN = 0.5f;    // 1/с: скорость к точке дома на посадке
        static constexpr float LAND_SPEED = 2.0f;   // м/с, не быстрее

        void configure(const GpsRescueSettings& s) { set = s; }
        const GpsRescueSettings& settings() const { return set; }

        // Каждый цикл, пока заармлен (и вне возврата): наибольшая высота
        // полёта (для MAX_ALT) и последний фикс.
        void track(const GpsInput& g, float altM, float dt)
        {
            if (altM > maxAltM) maxAltM = altM;
            sinceFix = g.fresh ? 0.0f : sinceFix + dt;
            if (g.fresh) last = g;
        }

        void resetFlight()
        {
            maxAltM = 0.0f;
            phase_ = RescuePhase::IDLE;
            fail_ = RescueFail::NONE;
            headingOffset = 0.0f;
            headingAligned = false;
        }

        // GPS годится для возврата (до ARM дома ещё нет): фикс, спутники, свежий.
        bool gpsOk() const { return last.fix && last.sats >= set.minSats && sinceFix <= GPS_TIMEOUT_S; }

        // Можно ли вернуться прямо сейчас.
        RescueFail ready() const
        {
            if (!last.fix) return RescueFail::NO_FIX;
            if (last.sats < set.minSats) return RescueFail::LOW_SATS;
            if (!last.home) return RescueFail::NO_HOME;
            if (sinceFix > GPS_TIMEOUT_S) return RescueFail::GPS_LOST;
            return RescueFail::NONE;
        }

        // Начать возврат; failsafe — по потере связи (для sanity FS_ONLY);
        // ceilingM — выше не подниматься (учебный потолок; 0 — нет).
        void start(float altM, bool failsafe, float ceilingM = 0.0f)
        {
            byFailsafe = failsafe;
            integral = 0.0f;
            sideIntegral = 0.0f;
            straightTimer = 0.0f;
            prevSpeedErr = 0.0f;
            progressTimer = 0.0f;
            fail_ = ready();
            if (fail_ != RescueFail::NONE)
            {
                phase_ = RescuePhase::ABORT;
                return;
            }
            const float dist = distance(last);
            bestDist = dist;
            switch (set.altMode)
            {
                case RescueAltMode::FIXED_ALT: targetAlt = set.returnAltM; break;
                case RescueAltMode::CURRENT_ALT: targetAlt = altM + set.initialClimbM; break;
                case RescueAltMode::MAX_ALT:
                default: targetAlt = fmaxf(maxAltM, altM) + set.initialClimbM; break;
            }
            if (ceilingM > 0.0f) targetAlt = fminf(targetAlt, ceilingM);
            returnAlt = targetAlt;
            phase_ = dist < set.minStartDistM ? RescuePhase::LAND : RescuePhase::CLIMB;
        }

        void stop() { phase_ = RescuePhase::IDLE; }

        bool active() const { return phase_ != RescuePhase::IDLE; }
        RescuePhase phase() const { return phase_; }
        RescueFail failure() const { return fail_; }
        bool landing() const { return phase_ == RescuePhase::LAND; }
        float targetAltitude() const { return targetAlt; }
        float distanceHome() const { return distance(last); }
        float groundSpeed() const { return sqrtf(last.velN * last.velN + last.velE * last.velE); }
        // скорость от дома (по радиусу), м/с; < 0 — к дому
        float outwardSpeed() const
        {
            const float d = distance(last);
            return d > 1.0f ? (last.northM * last.velN + last.eastM * last.velE) / d : 0.0f;
        }
        float headingCorrection() const { return headingOffset; }

        // Один цикл возврата: altM и climb — высота над точкой ARM и скорость
        // подъёма (барометр с акселерометром); yawDeg — рысканье гироскопа.
        // track() в этом цикле уже позван.
        RescueCommand update(float altM, float yawDeg, float dt)
        {
            RescueCommand c;
            if (phase_ == RescuePhase::IDLE || phase_ == RescuePhase::ABORT) return c;

            const bool checks = set.sanity == RescueSanity::ON || (set.sanity == RescueSanity::FS_ONLY && byFailsafe);
            if (checks)
            {
                const RescueFail f = sinceFix > GPS_TIMEOUT_S ? RescueFail::GPS_LOST
                                     : !last.fix                 ? RescueFail::NO_FIX
                                     : last.sats < set.minSats   ? RescueFail::LOW_SATS
                                                                 : RescueFail::NONE;
                if (f != RescueFail::NONE)
                {
                    abort(f);
                    return c;
                }
            }

            const float dist = distance(last);
            const float heading = headingNow(yawDeg);
            // направление на дом и ошибка носа
            const float homeBearing = atan2f(-last.eastM, -last.northM) * RAD2DEG;
            const float groundSpeed = sqrtf(last.velN * last.velN + last.velE * last.velE);
            const float err = wrap180(homeBearing - heading);
            // скорость в осях носа
            const float h = heading * DEG2RAD;
            const float fwd = last.velN * cosf(h) + last.velE * sinf(h);
            const float side = -last.velN * sinf(h) + last.velE * cosf(h);

            // прогресс: дом приближается
            if (phase_ == RescuePhase::FLY_HOME || phase_ == RescuePhase::DESCEND)
            {
                if (dist < bestDist - 1.0f)
                {
                    bestDist = dist;
                    progressTimer = 0.0f;
                }
                else
                {
                    progressTimer += dt;
                }
                if (checks && progressTimer > PROGRESS_WINDOW_S)
                {
                    abort(RescueFail::NO_PROGRESS);
                    return c;
                }
                if (checks && dist > bestDist + FLYAWAY_M)
                {
                    abort(RescueFail::FLYAWAY);
                    return c;
                }
            }

            switch (phase_)
            {
                case RescuePhase::CLIMB:
                {
                    c.yawRateDps = yawTowards(err);
                    c.climbRate = set.ascendRate;
                    holdPosition(fwd, side, 0.0f, 0.0f, c, dt);
                    if (altM >= targetAlt - 1.0f) phase_ = RescuePhase::FLY_HOME;
                    break;
                }
                case RescuePhase::FLY_HOME:
                case RescuePhase::DESCEND:
                {
                    if (dist < set.descentDistM) phase_ = RescuePhase::DESCEND;
                    float speed = set.groundSpeed;
                    if (phase_ == RescuePhase::DESCEND)
                    {
                        const float k = fmaxf(0.0f, fminf(1.0f, dist / fmaxf(1.0f, set.descentDistM)));
                        speed = fmaxf(1.0f, set.groundSpeed * k);
                        targetAlt = set.landingAltM + (returnAlt - set.landingAltM) * k;
                    }
                    c.climbRate = climbTo(altM);
                    if (!last.headingTrusted && !headingAligned)
                    {
                        // без компаса: сначала прямо, без рысканья, — на скорости
                        // курс GPS и есть нос: поправка гироскопа ставится сразу
                        c.pitchDeg = -speedControl(set.groundSpeed, groundSpeed, dt);
                        straightTimer = groundSpeed > ALIGN_SPEED ? straightTimer + dt : 0.0f;
                        if (straightTimer > ALIGN_STRAIGHT_S)
                        {
                            headingOffset = wrap180(atan2f(last.velE, last.velN) * RAD2DEG - yawDeg);
                            headingAligned = true;
                            straightTimer = 0.0f;
                        }
                        break;
                    }
                    // нос не на доме — медленнее (на 60° и больше — стоим)
                    speed *= fmaxf(0.0f, 1.0f - fabsf(err) / 60.0f);
                    // у самого дома пеленг на него скачет — нос больше не крутим
                    c.yawRateDps = yawTowards(err) * fminf(1.0f, dist / NO_YAW_DIST_M);
                    // тангаж держит скорость именно к дому
                    const float toward = dist > 1.0f ? -(last.velN * last.northM + last.velE * last.eastM) / dist : fwd;
                    c.pitchDeg = -speedControl(speed, toward, dt);
                    // снос вбок гасим только с компасом: без него крен спрятал
                    // бы ошибку курса (коптер шёл бы боком туда, куда «смотрит»)
                    c.rollDeg = last.headingTrusted ? sideControl(0.0f, side, dt) : 0.0f;
                    // курс GPS отстаёт от носа на поворотах — учиться только на прямой
                    straightTimer = fabsf(c.yawRateDps) < ALIGN_MAX_YAW_RATE && c.pitchDeg < -ALIGN_PITCH_DEG ? straightTimer + dt : 0.0f;
                    if (straightTimer > ALIGN_STRAIGHT_S) alignHeading(yawDeg, dt);
                    if (dist < LAND_RADIUS_M)
                    {
                        phase_ = RescuePhase::LAND;
                        integral = 0.0f;
                    }
                    break;
                }
                case RescuePhase::LAND:
                {
                    // над домом: скорость — к точке дома (ветер сносит — возвращает)
                    const float vN = fmaxf(-LAND_SPEED, fminf(LAND_SPEED, -LAND_GAIN * last.northM));
                    const float vE = fmaxf(-LAND_SPEED, fminf(LAND_SPEED, -LAND_GAIN * last.eastM));
                    holdPosition(fwd, side, vN * cosf(h) + vE * sinf(h), -vN * sinf(h) + vE * cosf(h), c, dt);
                    c.climbRate = -(altM > set.landingAltM ? set.descendRate : 0.5f * set.descendRate);
                    // у самой земли — ровно: наклон в опору мешает понять, что сел
                    if (altM < GROUND_LEVEL_M)
                    {
                        c.rollDeg = c.pitchDeg = 0.0f;
                        integral = sideIntegral = 0.0f;
                    }
                    break;
                }
                default: break;
            }
            return c;
        }

    private:
        static constexpr float DEG2RAD = 0.017453293f;
        static constexpr float RAD2DEG = 57.29578f;

        static float wrap180(float d)
        {
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        }

        static float distance(const GpsInput& g) { return sqrtf(g.northM * g.northM + g.eastM * g.eastM); }

        void abort(RescueFail f)
        {
            fail_ = f;
            phase_ = RescuePhase::ABORT;
        }

        float headingNow(float yawDeg) const
        {
            return last.headingTrusted ? last.headingDeg : wrap180(yawDeg + headingOffset);
        }

        // Без компаса: летим вперёд одним тангажом — курс GPS и есть курс носа
        // (с точностью до сноса ветром), поправка гироскопа тянется к нему.
        void alignHeading(float yawDeg, float dt)
        {
            if (last.headingTrusted) return;
            const float speed = sqrtf(last.velN * last.velN + last.velE * last.velE);
            if (speed < ALIGN_SPEED) return;
            const float course = atan2f(last.velE, last.velN) * RAD2DEG;
            const float e = wrap180(course - (yawDeg + headingOffset));
            headingOffset = wrap180(headingOffset + e * fminf(1.0f, dt / ALIGN_TAU_S));
        }

        float yawTowards(float err) const
        {
            const float r = set.yawP / 10.0f * err;
            return fmaxf(-MAX_YAW_RATE, fminf(MAX_YAW_RATE, r));
        }

        float climbTo(float altM) const
        {
            const float r = 0.6f * (targetAlt - altM);
            return fmaxf(-set.descendRate, fminf(set.ascendRate, r));
        }

        // Наклон вперёд (°) по ошибке скорости вдоль носа: velocity_p/i/d —
        // единицы BF, у нас P 8 — 2.4° на м/с, I 40 — 1.2 °/с на м/с.
        float speedControl(float target, float fwd, float dt)
        {
            const float e = target - fwd;
            const float kp = set.velocityP * 0.3f, ki = set.velocityI * 0.03f, kd = set.velocityD * 0.02f;
            integral += ki * e * dt;
            integral = fmaxf(-set.maxAngleDeg * 0.5f, fminf(set.maxAngleDeg * 0.5f, integral));
            const float d = dt > 0.0f ? (e - prevSpeedErr) / dt : 0.0f;
            prevSpeedErr = e;
            const float out = kp * e + integral + kd * fmaxf(-5.0f, fminf(5.0f, d));
            return fmaxf(-set.maxAngleDeg, fminf(set.maxAngleDeg, out));
        }

        // Скорость вбок (снос ветром): правый борт вниз — коптер идёт вправо;
        // интеграл держит крен против постоянного ветра.
        float sideControl(float target, float side, float dt)
        {
            const float e = target - side;
            const float kp = set.velocityP * 0.3f, ki = set.velocityI * 0.03f;
            sideIntegral = fmaxf(-MAX_ROLL_DEG * 0.6f, fminf(MAX_ROLL_DEG * 0.6f, sideIntegral + ki * e * dt));
            return fmaxf(-MAX_ROLL_DEG, fminf(MAX_ROLL_DEG, kp * e + sideIntegral));
        }

        // Скорости вдоль носа и вбок — к заданным (0 — стоять на месте).
        void holdPosition(float fwd, float side, float fwdTarget, float sideTarget, RescueCommand& c, float dt)
        {
            c.pitchDeg = -speedControl(fwdTarget, fwd, dt);
            c.rollDeg = sideControl(sideTarget, side, dt);
        }

        GpsRescueSettings set;
        GpsInput last;
        RescuePhase phase_ = RescuePhase::IDLE;
        RescueFail fail_ = RescueFail::NONE;
        bool byFailsafe = false;
        float sinceFix = 1e6f;
        float maxAltM = 0.0f;
        float targetAlt = 0.0f, returnAlt = 0.0f;
        float bestDist = 0.0f, progressTimer = 0.0f;
        float integral = 0.0f, prevSpeedErr = 0.0f, sideIntegral = 0.0f;
        float straightTimer = 0.0f;
        bool headingAligned = false;   // без компаса: поправка курса уже поставлена по GPS
        float headingOffset = 0.0f;
    };
}
