#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Filters.h"
#include "copter/PositionEstimator.h"

// ============================================================
// Удержание позиции (POS HOLD), как режим Betaflight 2025: коптер стоит над
// точкой, стики двигают его, отпустил — тормозит и держит новое место.
// Высоту в это время держит AltitudeController (POS HOLD = ALT HOLD + место).
// Своя реализация по описанию поведения, код BF не копирован.
//
// Каскад в осях север/восток (оценка — copter/PositionEstimator.h):
//   ошибка положения × P -> желаемая скорость;
//   ошибка скорости × D + её интеграл × I (ветер) -> ускорение;
//   A — поправка по ускорению: измеренное акселерометром минус заданное —
//       это и есть порыв ветра; он гасится наклоном раньше, чем успеет
//       набежать скорость;
//   ускорение -> наклон (g·tg угла), не больше autopilot_max_angle.
// Настройки autopilot_position_P/I/D/A — в единицах BF: 30 — наши расчётные
// (1 м/с на метр, 2.5 м/с² на м/с, ветер учится за 2–3 с, порыв гасится
// наполовину); autopilot_position_cutoff — срез сглаживания A (и 4× — D).
//
// Стики (как крен/тангаж ANGLE, но задают не наклон, а скорость): дальше
// pos_hold_deadband — летит со скоростью до pos_hold_adjust_rate в сторону
// стика (в осях носа); в центре — тормозит (BRAKE) и, остановившись,
// запоминает точку (HOLD). Разгон и торможение — не резче половины
// autopilot_max_angle (как предел пилота в BF): отпущенный на скорости стик
// не ставит коптер на дыбы; весь угол — запас против ветра.
// pos_hold_deadband = 0 — стики не действуют (как в BF).
//
// Отказы (check): нет годного GPS — NO_GPS; курс неизвестен или компаса нет /
// он врёт при pos_hold_without_mag = OFF — NO_HEADING; унесло от точки дальше
// 10 м — DRIFT (до выключения режима). Что делать при отказе, решает
// контроллер: ALT HOLD и предупреждение в OSD.
//
// Знаки — как во всей прошивке: крен > 0 — вправо, тангаж > 0 — нос вверх
// (вперёд — < 0). Без Arduino — тестируется на ПК.
// ============================================================

namespace copter
{
    struct PosHoldSettings
    {
        float adjustRate = 5.0f;     // pos_hold_adjust_rate: м/с при полном стике
        float deadband = 0.05f;      // pos_hold_deadband: доля хода стика; 0 — стики не действуют
        bool withoutMag = false;     // pos_hold_without_mag: можно без компаса (курс — по полёту)
        float maxAngleDeg = 50.0f;   // autopilot_max_angle
        float p = 30.0f, i = 30.0f, d = 30.0f, a = 30.0f;   // autopilot_position_P/I/D/A
        float cutoffHz = 0.8f;       // autopilot_position_cutoff, Гц
    };

    enum class PosHoldFail : uint8_t { NONE, NO_GPS, NO_HEADING, DRIFT };
    enum class PosHoldPhase : uint8_t { OFF, MOVE, BRAKE, HOLD };

    inline const char* posHoldFailName(PosHoldFail f)
    {
        switch (f)
        {
            case PosHoldFail::NO_GPS: return "NO_GPS";
            case PosHoldFail::NO_HEADING: return "NO_HEADING";
            case PosHoldFail::DRIFT: return "DRIFT";
            case PosHoldFail::NONE:
            default: return "NONE";
        }
    }

    inline const char* posHoldPhaseName(PosHoldPhase p)
    {
        switch (p)
        {
            case PosHoldPhase::MOVE: return "MOVE";
            case PosHoldPhase::BRAKE: return "BRAKE";
            case PosHoldPhase::HOLD: return "HOLD";
            case PosHoldPhase::OFF:
            default: return "OFF";
        }
    }

    // Слово для очков: почему позиция не держится.
    inline const char* posHoldFailText(PosHoldFail f)
    {
        switch (f)
        {
            case PosHoldFail::NO_GPS: return "POSHOLD NO GPS";
            case PosHoldFail::NO_HEADING: return "POSHOLD NO MAG";
            case PosHoldFail::DRIFT: return "POSHOLD FAIL";
            case PosHoldFail::NONE:
            default: return nullptr;
        }
    }

    struct PosHoldCommand
    {
        float rollDeg = 0.0f, pitchDeg = 0.0f;   // целевой наклон (как стик ANGLE)
    };

    class PositionHold
    {
    public:
        static constexpr float DRIFT_M = 10.0f;       // дальше от точки — отказ (как в BF)
        static constexpr float STOP_SPEED = 0.7f;     // м/с: остановился — точка запоминается
        static constexpr float BRAKE_TIMEOUT_S = 3.0f;
        static constexpr float SLEW_TAU_S = 0.15f;
        static constexpr float G = 9.80665f;

        void configure(const PosHoldSettings& s, float dtS)
        {
            set = s;
            dt = dtS;
            for (auto& f : accelLpf) f.configure(s.cutoffHz, dtS);
            for (auto& f : cmdLpf) f.configure(s.cutoffHz, dtS);
            for (auto& f : velLpf) f.configure(4.0f * s.cutoffHz, dtS);
        }
        const PosHoldSettings& settings() const { return set; }

        // Можно ли держать позицию прямо сейчас.
        PosHoldFail check(const PositionEstimator& est) const
        {
            if (drift) return PosHoldFail::DRIFT;
            if (!est.valid()) return PosHoldFail::NO_GPS;
            const HeadingSource h = est.headingSource();
            if (h == HeadingSource::NONE) return PosHoldFail::NO_HEADING;
            if (h == HeadingSource::GPS && !set.withoutMag) return PosHoldFail::NO_HEADING;
            return PosHoldFail::NONE;
        }

        // Начать: скорость гасится плавно, точка — где остановится.
        // rollDeg/pitchDeg — наклон сейчас: по нему и акселерометру видно ветер,
        // который пилот уже держал стиком, — интеграл начинает с него, без рывка.
        void start(const PositionEstimator& est, float rollDeg, float pitchDeg)
        {
            const float h = est.headingDeg() * DEG2RAD, c = cosf(h), s = sinf(h);
            const float af = -G * tanf(clampAngle(pitchDeg) * DEG2RAD), ar = G * tanf(clampAngle(rollDeg) * DEG2RAD);
            const float tiltN = af * c - ar * s, tiltE = af * s + ar * c;
            const float kA = gainA();
            // ветер = измеренное ускорение − ускорение от наклона
            const float windN = est.accelN() - tiltN, windE = est.accelE() - tiltE;
            accelLpf[0].reset(est.accelN());
            accelLpf[1].reset(est.accelE());
            cmdLpf[0].reset(tiltN);
            cmdLpf[1].reset(tiltE);
            integral[0] = clampI(-(1.0f - kA) * windN);
            integral[1] = clampI(-(1.0f - kA) * windE);
            vT[0] = est.velN();
            vT[1] = est.velE();
            velLpf[0].reset(0.0f);
            velLpf[1].reset(0.0f);
            target[0] = est.north();
            target[1] = est.east();
            phase_ = est.speed() > STOP_SPEED ? PosHoldPhase::BRAKE : PosHoldPhase::HOLD;
            brakeTimer = 0.0f;
        }

        void stop() { phase_ = PosHoldPhase::OFF; }
        // Режим выключили — отказ по сносу забыт.
        void clearFail() { drift = false; }

        bool active() const { return phase_ != PosHoldPhase::OFF; }
        PosHoldPhase phase() const { return phase_; }
        float targetNorth() const { return target[0]; }
        float targetEast() const { return target[1]; }
        // расстояние до точки, м
        float errorM(const PositionEstimator& est) const
        {
            const float n = target[0] - est.north(), e = target[1] - est.east();
            return sqrtf(n * n + e * e);
        }

        // Один цикл. Стики −1…1 (крен + вправо, тангаж + на себя); angleCapDeg —
        // предел наклона сверху (учебный; ≤ 0 — нет), speedCap — предел скорости
        // по стику, м/с (≤ 0 — нет).
        PosHoldCommand update(const PositionEstimator& est, float stickRoll, float stickPitch, float angleCapDeg,
                              float speedCap, float dtS)
        {
            PosHoldCommand out;
            if (phase_ == PosHoldPhase::OFF) return out;
            dt = dtS;
            const float h = est.headingDeg() * DEG2RAD, c = cosf(h), s = sinf(h);
            float maxAngle = fminf(70.0f, fmaxf(10.0f, set.maxAngleDeg));
            if (angleCapDeg > 0.0f) maxAngle = fminf(maxAngle, angleCapDeg);
            const float aMax = G * tanf(maxAngle * DEG2RAD);
            // разгон и торможение по стику — половина угла (как предел пилота в BF)
            const float aSlew = G * tanf(fminf(0.5f * set.maxAngleDeg, maxAngle) * DEG2RAD);
            float rate = fmaxf(0.1f, set.adjustRate);
            if (speedCap > 0.0f) rate = fminf(rate, speedCap);

            // ---- желаемая скорость ----
            const float fwd = -dead(stickPitch), right = dead(stickRoll);
            float want[2] = { 0.0f, 0.0f };
            if (set.deadband > 0.0f && (fwd != 0.0f || right != 0.0f))
            {
                phase_ = PosHoldPhase::MOVE;
                float vf = fwd * rate, vr = right * rate;
                const float v = sqrtf(vf * vf + vr * vr);
                if (v > rate)
                {
                    vf *= rate / v;
                    vr *= rate / v;
                }
                want[0] = vf * c - vr * s;
                want[1] = vf * s + vr * c;
            }
            else if (phase_ == PosHoldPhase::MOVE)
            {
                phase_ = PosHoldPhase::BRAKE;
                brakeTimer = 0.0f;
            }
            if (phase_ == PosHoldPhase::BRAKE)
            {
                const bool commandDone = fabsf(vT[0]) < 0.05f && fabsf(vT[1]) < 0.05f;
                if (commandDone) brakeTimer += dt;
                if (commandDone && (est.speed() < STOP_SPEED || brakeTimer > BRAKE_TIMEOUT_S)) phase_ = PosHoldPhase::HOLD;
            }
            if (phase_ != PosHoldPhase::HOLD)
            {
                target[0] = est.north();
                target[1] = est.east();
            }
            else
            {
                const float eN = target[0] - est.north(), eE = target[1] - est.east();
                const float dist = sqrtf(eN * eN + eE * eE);
                if (dist > DRIFT_M)
                {
                    drift = true;
                    phase_ = PosHoldPhase::OFF;
                    return out;
                }
                const float kPos = set.p / 30.0f * POS_GAIN;
                want[0] = kPos * eN;
                want[1] = kPos * eE;
                const float v = sqrtf(want[0] * want[0] + want[1] * want[1]);
                if (v > rate)
                {
                    want[0] *= rate / v;
                    want[1] *= rate / v;
                }
            }

            // ---- уставка скорости: не резче aSlew ----
            float ff[2];
            {
                // к желаемой — за SLEW_TAU_S, но не резче aSlew (без рывка на последних см/с)
                const float dN = want[0] - vT[0], dE = want[1] - vT[1];
                const float d = sqrtf(dN * dN + dE * dE);
                const float a = fminf(aSlew, d / SLEW_TAU_S);
                const float k = d > 1e-6f ? a / d : 0.0f;
                ff[0] = dN * k;
                ff[1] = dE * k;
                vT[0] += ff[0] * dt;
                vT[1] += ff[1] * dt;
            }

            // ---- ускорение ----
            const float kVel = set.d / 30.0f * VEL_GAIN, kI = set.i / 30.0f * I_GAIN, kA = gainA();
            const float vel[2] = { est.velN(), est.velE() }, acc[2] = { est.accelN(), est.accelE() };
            float cmd[2];
            for (uint8_t k = 0; k < 2; ++k)
            {
                const float err = velLpf[k].apply(vT[k] - vel[k]);
                // порыв: измеренное ускорение минус то, что просили (оба — через один фильтр)
                const float gust = accelLpf[k].apply(acc[k]) - cmdLpf[k].value();
                cmd[k] = ff[k] + kVel * err + integral[k] - kA * gust;
                pendingI[k] = kI * err * dt;
            }
            const float mag = sqrtf(cmd[0] * cmd[0] + cmd[1] * cmd[1]);
            const bool saturated = mag > aMax;
            if (saturated)
            {
                cmd[0] *= aMax / mag;
                cmd[1] *= aMax / mag;
            }
            for (uint8_t k = 0; k < 2; ++k)
            {
                // в упоре интеграл не копится дальше в ту же сторону
                if (!saturated || pendingI[k] * cmd[k] < 0.0f) integral[k] = clampI(integral[k] + pendingI[k]);
                cmdLpf[k].apply(cmd[k]);
            }

            // ---- наклон в осях носа ----
            const float af = cmd[0] * c + cmd[1] * s, ar = -cmd[0] * s + cmd[1] * c;
            out.pitchDeg = -atanf(af / G) * RAD2DEG;
            out.rollDeg = atanf(ar / G) * RAD2DEG;
            return out;
        }

    private:
        static constexpr float DEG2RAD = 0.017453293f;
        static constexpr float RAD2DEG = 57.29578f;
        // что значат 30 единиц BF
        static constexpr float POS_GAIN = 1.0f;    // (м/с) на метр
        static constexpr float VEL_GAIN = 2.5f;    // (м/с²) на м/с
        static constexpr float I_GAIN = 1.0f;      // (м/с²) на (м/с)·с
        static constexpr float A_GAIN = 0.5f;      // доля порыва, которая гасится сразу
        static constexpr float I_MAX = 6.0f;       // м/с² — наклон 31° против ветра

        float gainA() const { return fminf(0.95f, fmaxf(0.0f, set.a / 30.0f * A_GAIN)); }
        static float clampI(float v) { return fminf(I_MAX, fmaxf(-I_MAX, v)); }
        static float clampAngle(float deg) { return fminf(60.0f, fmaxf(-60.0f, deg)); }

        float dead(float stick) const
        {
            const float db = fminf(0.9f, fmaxf(0.0f, set.deadband));
            const float a = fabsf(stick);
            if (a <= db) return 0.0f;
            const float v = fminf(1.0f, (a - db) / (1.0f - db));
            return stick > 0.0f ? v : -v;
        }

        PosHoldSettings set;
        float dt = 0.001f;
        PosHoldPhase phase_ = PosHoldPhase::OFF;
        bool drift = false;
        float target[2] = {};     // север, восток
        float vT[2] = {};         // уставка скорости
        float integral[2] = {};   // м/с²
        float pendingI[2] = {};
        float brakeTimer = 0.0f;
        Pt1 accelLpf[2], cmdLpf[2], velLpf[2];
    };
}
