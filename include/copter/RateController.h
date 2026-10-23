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
//        пока коптер на земле и пока смеситель упёрся (anti-windup);
//        на резком манёвре придерживается (I-relax: после флипа нет
//        «отскока»); при рывке газом временно сильнее (anti-gravity —
//        нос не клюёт);
//   D  — по гироскопу, не по ошибке: движение стика не даёт пинка;
//        через свой ФНЧ — в D больше всего шума;
//   FF — упреждение от скорости изменения уставки: стик начал движение —
//        моторы уже отвечают, не дожидаясь ошибки.
// P и D умножаются на поправку по газу (TPA из модели винтомоторной
// группы, Presets.h).
// ============================================================

namespace copter
{
    struct AxisTerms
    {
        float p = 0, i = 0, d = 0, ff = 0, sum = 0;
    };

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
            relaxLpf.configure(10.0f, dtS);
            reset();
        }

        void reset()
        {
            terms = AxisTerms();
            iTerm = 0.0f;
            prevMeasured = 0.0f;
            prevSetpoint = 0.0f;
            first = true;
            dLpf.reset();
            outLpf.reset();
            ffLpf.reset();
            relaxLpf.reset();
        }

        void resetIntegral() { iTerm = 0.0f; }

        // setpoint, measured — °/с. tpa — множитель P и D; iBoost —
        // множитель I (anti-gravity); holdI — интеграл не копить (земля);
        // saturated — смеситель упёрся: интеграл не растёт в сторону упора.
        float update(float setpoint, float measured, float tpa, float iBoost, bool holdI, bool saturated)
        {
            if (first)
            {
                prevMeasured = measured;
                prevSetpoint = setpoint;
                relaxLpf.reset(setpoint);
                first = false;
            }
            const float error = setpoint - measured;

            terms.p = gains.p * error * tpa;

            // I-relax: уставка быстро меняется (флип, рывок стиком) или коптер
            // ещё разгоняется к ней (большая ошибка) — интеграл ждёт: ветер и
            // кривой центр тяжести дают ошибку в единицы °/с, а не в десятки,
            // и копить большую ошибку разгона — значит проскочить уставку
            const float fast = fabsf(setpoint - relaxLpf.apply(setpoint));
            const float relaxSetpoint = fmaxf(0.0f, 1.0f - fast / 40.0f);
            const float relaxError = fminf(1.0f, fmaxf(0.0f, 1.0f - (fabsf(error) - 20.0f) / 60.0f));
            const float relax = relaxSetpoint * relaxError;
            if (!holdI)
            {
                const float delta = gains.i * error * dt * relax * iBoost;
                const bool pushesIntoLimit = saturated && ((delta > 0.0f) == (terms.sum > 0.0f));
                if (!pushesIntoLimit) iTerm = fminf(limit, fmaxf(-limit, iTerm + delta));
            }
            terms.i = iTerm;

            const float rate = (measured - prevMeasured) / dt;
            terms.d = -gains.d * dLpf.apply(rate) * tpa;
            prevMeasured = measured;

            terms.ff = gains.ff * ffLpf.apply((setpoint - prevSetpoint) / dt);
            prevSetpoint = setpoint;

            terms.sum = outLpf.apply(fminf(1.0f, fmaxf(-1.0f, terms.p + terms.i + terms.d + terms.ff)));
            return terms.sum;
        }

        const AxisTerms& lastTerms() const { return terms; }
        const AxisGains& getGains() const { return gains; }

    private:
        AxisGains gains;
        float limit = 0.3f;
        float dt = 0.001f;
        Pt1 dLpf, outLpf, ffLpf, relaxLpf;
        AxisTerms terms;
        float iTerm = 0.0f;
        float prevMeasured = 0.0f;
        float prevSetpoint = 0.0f;
        bool first = true;
    };
}
