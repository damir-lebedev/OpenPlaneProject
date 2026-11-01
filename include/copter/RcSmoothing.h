#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Filters.h"

// ============================================================
// Сглаживание стиков и мёртвые зоны — как rc_smoothing_* и deadband в
// Betaflight (по описанию поведения; код BF не копирован).
//
// Приёмник присылает кадр раз в 4 мс (ELRS 250 Гц) или в 20 мс (iBUS):
// между кадрами стик стоит, на кадре прыгает — моторы дёргались бы
// ступеньками. Поэтому стики идут через ФНЧ PT3, частота которого — от
// измеренной частоты кадров:
//
//   срез = частота кадров × 1.5 / (1 + factor / 10), не ниже 15 Гц
//
// (rc_smoothing_auto_factor = 30: ELRS 250 Гц → 94 Гц, 150 Гц → 56 Гц,
// iBUS 50 Гц → 19 Гц). Газ — свой коэффициент (…_auto_factor_throttle).
// Частоту можно задать и числом (rc_smoothing_setpoint_cutoff,
// rc_smoothing_throttle_cutoff; 0 — авто). rc_smoothing = OFF — без
// фильтра, ступеньками, как пришло.
//
// Мёртвая зона (deadband, yaw_deadband — в единицах BF, 500 — полстика):
// около центра стик — ноль, дальше растянут так, что упор — по-прежнему
// полный (как в BF).
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    // Три PT1 подряд; частота каждого поднята так, что у тройки −3 дБ ровно на cutoffHz.
    class Pt3
    {
    public:
        void configure(float cutoffHz, float dtS)
        {
            const float corrected = cutoffHz > 0.0f ? cutoffHz * 1.961459177f : 0.0f;
            a.configure(corrected, dtS);
            b.configure(corrected, dtS);
            c.configure(corrected, dtS);
        }

        float apply(float x) { return c.apply(b.apply(a.apply(x))); }

        void reset(float value = 0.0f)
        {
            a.reset(value);
            b.reset(value);
            c.reset(value);
        }

    private:
        Pt1 a, b, c;
    };

    struct RcSmoothingSettings
    {
        bool enabled = true;              // rc_smoothing
        uint8_t autoFactor = 30;          // rc_smoothing_auto_factor
        uint8_t autoFactorThrottle = 30;  // rc_smoothing_auto_factor_throttle
        uint16_t setpointCutoffHz = 0;    // rc_smoothing_setpoint_cutoff: 0 — авто
        uint16_t throttleCutoffHz = 0;    // rc_smoothing_throttle_cutoff: 0 — авто
        uint8_t deadband = 0;             // deadband: крен и тангаж, единицы BF (500 — полстика)
        uint8_t yawDeadband = 0;          // yaw_deadband
    };

    // Мёртвая зона: |x| ≤ доли — ноль, дальше растянуто до прежнего упора.
    inline float applyDeadband(float x, uint8_t units)
    {
        if (units == 0) return x;
        const float d = units / 500.0f;
        const float m = fabsf(x);
        if (m <= d) return 0.0f;
        const float y = (m - d) / (1.0f - d);
        return x < 0.0f ? -fminf(1.0f, y) : fminf(1.0f, y);
    }

    class RcSmoothing
    {
    public:
        static constexpr float MIN_CUTOFF_HZ = 15.0f;   // ниже — стики «плывут»
        static constexpr float RETUNE_SHARE = 0.1f;     // частота кадров ушла больше — пересчитать

        void configure(const RcSmoothingSettings& s, float dtS)
        {
            set = s;
            dt = dtS;
            setpointHz = throttleHz = 0.0f;
            retune(frameHz > 0.0f ? frameHz : NOMINAL_FRAME_HZ, true);
        }

        // Срез по частоте кадров (авто) или из настройки.
        static float autoCutoff(float frameRateHz, uint8_t factor)
        {
            if (frameRateHz <= 0.0f) return MIN_CUTOFF_HZ;
            return fmaxf(MIN_CUTOFF_HZ, frameRateHz * 1.5f / (1.0f + factor / 10.0f));
        }

        // Каждый цикл: измеренный период кадров (0 — неизвестен) — фильтры подстраиваются.
        void track(float framePeriodS)
        {
            if (framePeriodS <= 0.0f) return;
            retune(1.0f / framePeriodS, false);
        }

        // Стики: крен, тангаж, рысканье −1…1, газ 0…1 — на месте.
        void apply(float sticks[4])
        {
            sticks[0] = applyDeadband(sticks[0], set.deadband);
            sticks[1] = applyDeadband(sticks[1], set.deadband);
            sticks[2] = applyDeadband(sticks[2], set.yawDeadband);
            if (!set.enabled) return;
            for (uint8_t a = 0; a < 3; ++a) sticks[a] = axes[a].apply(sticks[a]);
            sticks[3] = throttle.apply(sticks[3]);
        }

        void reset()
        {
            for (auto& f : axes) f.reset();
            throttle.reset();
        }

        float frameRateHz() const { return frameHz; }
        float setpointCutoffHz() const { return set.enabled ? setpointHz : 0.0f; }
        float throttleCutoffHz() const { return set.enabled ? throttleHz : 0.0f; }
        const RcSmoothingSettings& settings() const { return set; }

    private:
        static constexpr float NOMINAL_FRAME_HZ = 250.0f;   // пока кадры не измерены — ELRS 250

        void retune(float rateHz, bool force)
        {
            if (!force && frameHz > 0.0f && fabsf(rateHz - frameHz) < RETUNE_SHARE * frameHz) return;
            frameHz = rateHz;
            const float sp = set.setpointCutoffHz ? static_cast<float>(set.setpointCutoffHz) : autoCutoff(rateHz, set.autoFactor);
            const float th = set.throttleCutoffHz ? static_cast<float>(set.throttleCutoffHz) : autoCutoff(rateHz, set.autoFactorThrottle);
            if (force || sp != setpointHz)
            {
                setpointHz = sp;
                for (auto& f : axes) f.configure(sp, dt);   // состояние не сбрасывается
            }
            if (force || th != throttleHz)
            {
                throttleHz = th;
                throttle.configure(th, dt);
            }
        }

        RcSmoothingSettings set;
        float dt = 0.001f;
        float frameHz = 0.0f;
        float setpointHz = 0.0f, throttleHz = 0.0f;
        Pt3 axes[3];
        Pt3 throttle;
    };
}
