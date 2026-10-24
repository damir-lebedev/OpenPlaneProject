#pragma once
#include <math.h>

#include "copter/Filters.h"
#include "copter/Presets.h"

// ============================================================
// Регулятор угловой скорости одной оси — внутренний контур, на нём
// держится любой полёт мультикоптера (и акро, и все режимы с
// автоматикой — они только задают ему уставку).
//
//   P  — ошибка «уставка − гироскоп»;
//   I  — интеграл ошибки: снос ветром, кривой центр тяжести. Не копится,
//        пока коптер на земле и пока смеситель или сумма PID упёрлись
//        (anti-windup), не выходит за предел (iterm_limit); на резком
//        манёвре придерживается (I-relax: после флипа нет «отскока»); при
//        рывке газом временно сильнее (anti-gravity — нос не клюёт, P —
//        тоже, множителем снаружи);
//   D  — по гироскопу, не по ошибке: движение стика не даёт пинка;
//        через свой ФНЧ — в D больше всего шума. На резком движении
//        (пропвош, флип) растёт до d_max — в покое меньше, и моторы не
//        греются от шума;
//   FF — упреждение от скорости изменения уставки: стик начал движение —
//        моторы уже отвечают, не дожидаясь ошибки.
// P и D умножаются на поправку по газу (TPA, Presets.h); в режиме
// TpaMode::D — только D, как в Betaflight. Сумма — не больше своего
// предела (pidsum_limit, у рысканья — pidsum_limit_yaw).
// Тонкости I и D — настройки PID-профиля, как в BF (PidExtras).
// ============================================================

namespace copter
{
    struct AxisTerms
    {
        float p = 0, i = 0, d = 0, ff = 0, sum = 0;
    };

    // d_max: при каком угловом ускорении D дорастает до d_max целиком.
    // d_max_gain 37 — 5000 °/с² (резкий манёвр, пропвош; шум гироскопа на
    // висении после ФНЧ D — в разы меньше); d_max_advance 20 — уставка
    // меняется со скоростью 2500 °/с² (стик рывком).
    constexpr float D_MAX_GAIN_SCALE = 185000.0f;
    constexpr float D_MAX_ADVANCE_SCALE = 50000.0f;
    constexpr float D_MAX_BOOST_HZ = 20.0f;      // сглаживание «насколько резко»
    constexpr float ITERM_RELAX_DPS = 40.0f;     // уставка убежала от сглаженной на столько — I стоит
    // relax по ошибке: куда уставка шла последние ~0.2 с (флип у «вялой»
    // гексы выходит на расход за 0.13–0.15 с — память длиннее, иначе
    // перелёт растёт с 11.7 до 12–18 %)
    constexpr float RELAX_ERROR_TRACK_HZ = 0.7f;

    class RateAxis
    {
    public:
        void configure(const AxisGains& g, float dtermLpfHz, float outLpfHz, float iLimit, float dtS)
        {
            gains = g;
            limit = iLimit;
            dt = dtS;
            dLpf.configure(dtermLpfHz, dtS);
            outLpf.configure(outLpfHz, dtS);
            ffLpf.configure(25.0f, dtS);
            boostLpf.configure(D_MAX_BOOST_HZ, dtS);
            relaxLpf.configure(relaxCutoffHz, dtS);
            spTrack.configure(RELAX_ERROR_TRACK_HZ, dtS);
            reset();
        }

        // Тонкости PID-профиля для этой оси (0 крен, 1 тангаж, 2 рысканье); на
        // ходу — интеграл и фильтры не сбрасываются.
        void setExtras(const PidExtras& x, uint8_t axis)
        {
            dMax = x.dMax[axis < 3 ? axis : 2];
            dMaxGain = x.dMaxGain;
            dMaxAdvance = x.dMaxAdvance;
            const bool yaw = axis == 2;
            relaxOn = x.relax == ItermRelax::RPY || x.relax == ItermRelax::RPY_INC ||
                      (!yaw && (x.relax == ItermRelax::RP || x.relax == ItermRelax::RP_INC));
            relaxIncOnly = x.relax == ItermRelax::RP_INC || x.relax == ItermRelax::RPY_INC;
            relaxGyro = x.relaxType == ItermRelaxType::GYRO;
            relaxError = x.relaxError;
            relaxCutoffHz = x.relaxCutoffHz;
            relaxLpf.configure(relaxCutoffHz, dt);
            sumLimit = yaw ? x.pidSumLimitYaw : x.pidSumLimit;
            tpaOnP = x.tpaMode != TpaMode::D;
        }

        void reset()
        {
            terms = AxisTerms();
            iTerm = 0.0f;
            boost = 0.0f;
            sumClamped = false;
            prevMeasured = 0.0f;
            prevSetpoint = 0.0f;
            first = true;
            dLpf.reset();
            outLpf.reset();
            ffLpf.reset();
            boostLpf.reset();
            relaxLpf.reset();
            spTrack.reset();
        }

        void resetIntegral() { iTerm = 0.0f; }

        // Интеграл как есть — для поворота вместе с рамой (iterm_rotation).
        float integral() const { return iTerm; }
        void setIntegral(float v) { iTerm = fminf(limit, fmaxf(-limit, v)); }

        // Другие коэффициенты на ходу (смена профиля): интеграл и фильтры не сбрасываются.
        void setGains(const AxisGains& g) { gains = g; }
        void setLimit(float iLimit) { limit = iLimit; }
        void setTpaMode(TpaMode mode) { tpaOnP = mode != TpaMode::D; }

        // setpoint, measured — °/с. tpa — множитель D (и P, кроме TpaMode::D); iBoost —
        // множитель I (anti-gravity, iterm_windup); holdI — интеграл не копить
        // (земля); saturated — смеситель упёрся: интеграл не растёт в сторону упора;
        // pBoost — множитель P (anti-gravity).
        float update(float setpoint, float measured, float tpa, float iBoost, bool holdI, bool saturated,
                     float pBoost = 1.0f)
        {
            if (first)
            {
                prevMeasured = measured;
                prevSetpoint = setpoint;
                relaxLpf.reset(setpoint);
                spTrack.reset(setpoint);
                first = false;
            }
            const float error = setpoint - measured;
            const float smooth = relaxLpf.apply(setpoint);
            const float moved = fabsf(setpoint - spTrack.apply(setpoint));   // уставка недавно сдвинулась на столько

            terms.p = gains.p * error * (tpaOnP ? tpa : 1.0f) * pBoost;

            if (!holdI)
            {
                const float raw = gains.i * error * dt * iBoost;
                float delta = gains.i * relaxedError(setpoint, measured, error, smooth, moved) * dt * iBoost;
                // *_INC: уменьшаться интеграл может всегда, придерживается только рост
                if (relaxIncOnly && iTerm != 0.0f && (raw > 0.0f) != (iTerm > 0.0f)) delta = raw;
                const bool pushesIntoLimit = (saturated || sumClamped) && ((delta > 0.0f) == (terms.sum > 0.0f));
                if (!pushesIntoLimit) iTerm = fminf(limit, fmaxf(-limit, iTerm + delta));
            }
            terms.i = iTerm;

            const float spRate = ffLpf.apply((setpoint - prevSetpoint) / dt);   // °/с² уставки
            prevSetpoint = setpoint;

            const float rate = dLpf.apply((measured - prevMeasured) / dt);       // °/с² гироскопа
            prevMeasured = measured;
            float d = gains.d;
            if (dMax > gains.d)
            {
                const float byGyro = fabsf(rate) * dMaxGain / D_MAX_GAIN_SCALE;
                const float bySetpoint = fabsf(spRate) * dMaxAdvance / D_MAX_ADVANCE_SCALE;
                boost = boostLpf.apply(fminf(1.0f, fmaxf(byGyro, bySetpoint)));
                d = gains.d + (dMax - gains.d) * boost;
            }
            else
            {
                boost = 0.0f;
            }
            terms.d = -d * rate * tpa;

            terms.ff = gains.ff * spRate;

            const float sum = terms.p + terms.i + terms.d + terms.ff;
            sumClamped = fabsf(sum) > sumLimit;
            terms.sum = outLpf.apply(fminf(sumLimit, fmaxf(-sumLimit, sum)));
            return terms.sum;
        }

        const AxisTerms& lastTerms() const { return terms; }
        const AxisGains& getGains() const { return gains; }
        // Насколько D сейчас поднят к d_max: 0 — базовый D, 1 — d_max.
        float dBoost() const { return boost; }

    private:
        // Ошибка, по которой копится интеграл, с I-relax. smooth — уставка через
        // ФНЧ relax, moved — насколько уставка сдвинулась за последние ~0.2 с.
        float relaxedError(float setpoint, float measured, float error, float smooth, float moved) const
        {
            const float away = fabsf(setpoint - smooth);   // уставка быстро меняется
            float e = error;
            if (relaxOn)
            {
                if (relaxGyro)
                {
                    // к сглаженной уставке, в полосе ±away — ноль
                    const float toSmooth = smooth - measured;
                    e = fabsf(toSmooth) <= away ? 0.0f : toSmooth - copysignf(away, toSmooth);
                }
                else
                {
                    e *= fmaxf(0.0f, 1.0f - away / ITERM_RELAX_DPS);
                }
            }
            // наш relax по ошибке: коптер ещё разгоняется к новой уставке
            // (флип) — копить большую ошибку разгона значит проскочить
            // уставку. Придерживается только та доля ошибки, которую
            // объясняет недавнее движение уставки: ошибку без него (порыв,
            // перекос тяги на рывке газом — это тоже десятки °/с) интеграл
            // должен догнать, иначе её держит один P и коптер уходит на
            // десятки градусов
            if (relaxError)
            {
                const float f = fminf(1.0f, fmaxf(0.0f, 1.0f - (fabsf(error) - 20.0f) / 60.0f));
                const float w = fminf(1.0f, moved / fmaxf(1.0f, fabsf(error)));
                e *= 1.0f - w * (1.0f - f);
            }
            return e;
        }

        AxisGains gains;
        float limit = 0.3f;
        float sumLimit = 1.0f;           // pidsum_limit: доля смесителя
        bool sumClamped = false;         // сумма упёрлась в предел (прошлый цикл)
        float dt = 0.001f;
        bool tpaOnP = true;
        float dMax = 0.0f, dMaxGain = 37.0f, dMaxAdvance = 20.0f;
        bool relaxOn = true, relaxIncOnly = false, relaxGyro = false, relaxError = true;
        float relaxCutoffHz = 10.0f;
        Pt1 dLpf, outLpf, ffLpf, boostLpf, relaxLpf, spTrack;
        AxisTerms terms;
        float iTerm = 0.0f;
        float boost = 0.0f;
        float prevMeasured = 0.0f;
        float prevSetpoint = 0.0f;
        bool first = true;
    };

    // iterm_rotation: рама повернулась на rates·dt — интеграл, который держит
    // неподвижное в мире (ветер), поворачивается вместе с ней: в пируэте
    // накопленное на крене переходит в тангаж. Оси FRD (крен, тангаж,
    // рысканье — правая тройка, Attitude.h): v' = v − ω × v · dt.
    inline void rotateIterm(float v[3], const float ratesDps[3], float dtS)
    {
        const float k = dtS * 0.017453293f;
        const float w0 = ratesDps[0] * k, w1 = ratesDps[1] * k, w2 = ratesDps[2] * k;
        const float c0 = w1 * v[2] - w2 * v[1];
        const float c1 = w2 * v[0] - w0 * v[2];
        const float c2 = w0 * v[1] - w1 * v[0];
        v[0] -= c0;
        v[1] -= c1;
        v[2] -= c2;
    }

    // iterm_windup: моторы разошлись больше порога (доля хода) — интеграл
    // копится слабее, на полном разбросе стоит. windup 1 — выключено.
    inline float itermWindupScale(float spread, float windup)
    {
        if (windup >= 1.0f || spread <= windup) return 1.0f;
        return fminf(1.0f, fmaxf(0.0f, (1.0f - spread) / (1.0f - windup)));
    }
}
