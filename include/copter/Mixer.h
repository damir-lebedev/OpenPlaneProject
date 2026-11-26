#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Frames.h"

// ============================================================
// Смеситель: газ и команды крена/тангажа/рысканья -> доля тяги
// каждого мотора 0..1.
//
// Моторы не могут дать меньше нуля и больше полного газа. Когда
// разница между моторами (то, чем держится ориентация) не влезает:
//   1. крен и тангаж важнее рысканья — сначала урезается рысканье;
//   2. не влезают и крен с тангажом — они урезаются вместе, без
//      искажения направления;
//   3. газ сдвигается так, чтобы разница уместилась целиком
//      (airmode): на нулевом газе коптер всё равно управляется — в
//      сальто и при сбросе газа на спуске, а на полном газе мотор,
//      которому надо сильнее, получает место за счёт остальных.
// Без airmode на газе ниже 15 % команды ослабляются, и на земле
// коптер не прыгает от интеграла.
//
// Линеаризация тяги: тяга винта ~ квадрат оборотов ~ квадрат сигнала,
// поэтому одна и та же команда на малом газе слабее, чем на большом.
// thrustLinear (0..0.5) выпрямляет зависимость: сигнал = доля тяги,
// подтянутая к корню, — коэффициент контура меньше гуляет с газом.
//
// mixer_type (имена как в BF, поведение — своё):
//   LEGACY    — как описано выше (по умолчанию);
//   LINEAR    — разница моторов не урезается: газ встаёт в середину,
//               края обрезаются; управления больше, но средняя тяга и
//               направление команды на упоре гуляют;
//   DYNAMIC   — у середины газа LEGACY, к краям (0 и 100 %) плавно LINEAR;
//   EZLANDING — LEGACY, но при стиках у центра и газе внизу разница
//               моторов не больше ez_landing_limit: у земли airmode не
//               подбрасывает коптер. Стик дальше ez_landing_threshold или
//               скорость по GPS выше ez_landing_speed — полный ход.
// ============================================================

namespace copter
{
    struct MixInput
    {
        float throttle = 0.0f;   // 0..1, доля тяги
        float roll = 0.0f;       // команды регуляторов, ~ -1..1
        float pitch = 0.0f;
        float yaw = 0.0f;
        float stick = 1.0f;      // EZLANDING: max(газ, |отклонение стиков|) 0..1
        float speedMs = -1.0f;   // EZLANDING: скорость по GPS, < 0 — неизвестна
    };

    enum class MixerType : uint8_t { LEGACY, LINEAR, DYNAMIC, EZLANDING };

    struct EzLanding
    {
        float threshold = 0.25f;   // ez_landing_threshold: стик, с которого ход полный
        float limit = 0.15f;       // ez_landing_limit: разница моторов при стиках в нуле
        float speedMs = 5.0f;      // ez_landing_speed: быстрее — ограничения нет
    };

    struct MixResult
    {
        float motor[MAX_MOTORS] = {};   // доля тяги 0..1
        float throttle = 0.0f;          // газ после сдвига airmode
        bool saturated = false;         // разница не влезла: команды урезаны
    };

    class Mixer
    {
    public:
        void setAirmode(bool on) { airmode = on; }
        bool isAirmode() const { return airmode; }
        void setType(MixerType t) { type = t; }
        MixerType getType() const { return type; }
        void setEzLanding(const EzLanding& e) { ez = e; }

        // EZLANDING: допустимый размах разницы моторов 0..1
        float ezLimit(const MixInput& in) const
        {
            if (ez.speedMs > 0.0f && in.speedMs >= ez.speedMs) return 1.0f;
            const float th = fmaxf(0.01f, ez.threshold);
            const float k = fminf(1.0f, fmaxf(0.0f, in.stick / th));
            return ez.limit + (1.0f - ez.limit) * k;
        }

        MixResult mix(const Frame& frame, MixInput in) const
        {
            if (type == MixerType::LINEAR) return mixLinear(frame, in);
            if (type == MixerType::DYNAMIC)
            {
                const MixResult a = mixLegacy(frame, in);
                const MixResult b = mixLinear(frame, in);
                const float w = fminf(1.0f, fabsf(2.0f * fminf(1.0f, fmaxf(0.0f, in.throttle)) - 1.0f));
                MixResult r;
                for (uint8_t i = 0; i < frame.count; ++i) r.motor[i] = a.motor[i] + w * (b.motor[i] - a.motor[i]);
                r.throttle = a.throttle + w * (b.throttle - a.throttle);
                r.saturated = a.saturated || b.saturated;
                return r;
            }
            return mixLegacy(frame, in, type == MixerType::EZLANDING ? ezLimit(in) : 1.0f);
        }

    private:
        // spanLimit < 1 — EZLANDING: размах разницы моторов не больше
        MixResult mixLegacy(const Frame& frame, MixInput in, float spanLimit = 1.0f) const
        {
            MixResult r;
            const uint8_t n = frame.count;
            if (n == 0) return r;

            if (!airmode)
            {
                const float k = fminf(1.0f, fmaxf(0.0f, in.throttle / 0.15f));
                in.roll *= k;
                in.pitch *= k;
                in.yaw *= k;
            }

            // 1–2: размах крена и тангажа не больше 1
            float rp[MAX_MOTORS], yw[MAX_MOTORS];
            float rpMin = 0.0f, rpMax = 0.0f;
            for (uint8_t i = 0; i < n; ++i)
            {
                rp[i] = in.roll * frame.mix[i].roll + in.pitch * frame.mix[i].pitch;
                yw[i] = in.yaw * frame.mix[i].yaw;
                rpMin = i ? fminf(rpMin, rp[i]) : rp[i];
                rpMax = i ? fmaxf(rpMax, rp[i]) : rp[i];
            }
            const float rpSpan = rpMax - rpMin;
            if (rpSpan > 1.0f)
            {
                for (uint8_t i = 0; i < n; ++i) rp[i] /= rpSpan;
                r.saturated = true;
            }

            // рысканье — сколько осталось: размах (rp + k·yaw) <= 1
            float k = 1.0f;
            if (span(rp, yw, n, 1.0f) > 1.0f)
            {
                float lo = 0.0f, hi = 1.0f;
                for (int it = 0; it < 12; ++it)
                {
                    const float mid = 0.5f * (lo + hi);
                    (span(rp, yw, n, mid) > 1.0f ? hi : lo) = mid;
                }
                k = lo;
                r.saturated = true;
            }

            float dMin = 0.0f, dMax = 0.0f, d[MAX_MOTORS];
            for (uint8_t i = 0; i < n; ++i)
            {
                d[i] = rp[i] + k * yw[i];
                dMin = i ? fminf(dMin, d[i]) : d[i];
                dMax = i ? fmaxf(dMax, d[i]) : d[i];
            }
            if (spanLimit < 1.0f && dMax - dMin > spanLimit)
            {
                const float s = spanLimit / (dMax - dMin);
                for (uint8_t i = 0; i < n; ++i) d[i] *= s;
                dMin *= s;
                dMax *= s;
            }

            // 3: газ — так, чтобы разница уместилась
            float t = fminf(1.0f, fmaxf(0.0f, in.throttle));
            if (airmode)
            {
                t = fmaxf(t, -dMin);
                t = fminf(t, 1.0f - dMax);
            }
            r.throttle = t;
            for (uint8_t i = 0; i < n; ++i)
            {
                r.motor[i] = fminf(1.0f, fmaxf(0.0f, t + d[i]));
            }
            return r;
        }

        // LINEAR: сумма PID без урезания; газ — чтобы разница влезла, а
        // не влезает — в середину размаха, края обрезаются
        MixResult mixLinear(const Frame& frame, MixInput in) const
        {
            MixResult r;
            const uint8_t n = frame.count;
            if (n == 0) return r;
            if (!airmode)
            {
                const float k = fminf(1.0f, fmaxf(0.0f, in.throttle / 0.15f));
                in.roll *= k;
                in.pitch *= k;
                in.yaw *= k;
            }
            float d[MAX_MOTORS], dMin = 0.0f, dMax = 0.0f;
            for (uint8_t i = 0; i < n; ++i)
            {
                d[i] = in.roll * frame.mix[i].roll + in.pitch * frame.mix[i].pitch + in.yaw * frame.mix[i].yaw;
                dMin = i ? fminf(dMin, d[i]) : d[i];
                dMax = i ? fmaxf(dMax, d[i]) : d[i];
            }
            float t = fminf(1.0f, fmaxf(0.0f, in.throttle));
            if (dMax - dMin > 1.0f)
            {
                r.saturated = true;
                if (airmode) t = 0.5f - 0.5f * (dMin + dMax);
            }
            else if (airmode)
            {
                t = fmaxf(t, -dMin);
                t = fminf(t, 1.0f - dMax);
            }
            r.throttle = t;
            for (uint8_t i = 0; i < n; ++i)
            {
                const float m = t + d[i];
                if (m < 0.0f || m > 1.0f) r.saturated = true;
                r.motor[i] = fminf(1.0f, fmaxf(0.0f, m));
            }
            return r;
        }

    public:
        // Доля тяги -> сигнал мотору 0..1 (до холостых оборотов).
        static float linearize(float thrust, float thrustLinear)
        {
            thrust = fminf(1.0f, fmaxf(0.0f, thrust));
            return (1.0f - thrustLinear) * thrust + thrustLinear * sqrtf(thrust);
        }

        // Сигнал 0..1 -> команда регулятору 0..1 с холостыми оборотами
        // idle: armed-моторы не останавливаются. limit (motor_output_limit) —
        // какая доля хода выше холостых доступна: батарея 6S на моторах под
        // 4S — 0.66, полный газ тогда и есть предел.
        static float withIdle(float signal, float idle, float limit = 1.0f)
        {
            return idle + (1.0f - idle) * limit * fminf(1.0f, fmaxf(0.0f, signal));
        }

    private:
        static float span(const float* rp, const float* yw, uint8_t n, float k)
        {
            float lo = rp[0] + k * yw[0], hi = lo;
            for (uint8_t i = 1; i < n; ++i)
            {
                const float v = rp[i] + k * yw[i];
                lo = fminf(lo, v);
                hi = fmaxf(hi, v);
            }
            return hi - lo;
        }

        bool airmode = true;
        MixerType type = MixerType::LEGACY;
        EzLanding ez;
    };
}
