#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Filters.h"

// ============================================================
// Цепочки фильтров как в Betaflight (пункты 054–059): две ступени ФНЧ
// гироскопа и D любого из типов BF, динамический срез первой ступени по
// газу, статические режекторы (гироскоп — два, D — один). Пункты 060–063:
// динамический режектор по спектру (`dyn_notch_*`, copter/Spectrum.h) и
// RPM-фильтр (`rpm_filter_*`) — режекторы по оборотам каждого мотора.
//
// Имена и смысл — как настройки BF: `gyro_lpf1_type` …, частота 0 —
// ступень выключена; режектор — центр и «cutoff» (нижний край полосы,
// Q = центр·cutoff / (центр² − cutoff²)); динамический срез —
// min + (max − min)·(газ + газ·(1 − газ)·expo/10), как в описании BF.
// Код свой. Без Arduino — тесты на ПК (test_copter).
// ============================================================

namespace copter
{
    // Порядок — как в BF (номер значения в CLI).
    enum class LpfType : uint8_t
    {
        PT1,
        BIQUAD,
        PT2,
        PT3,
    };

    struct LpfStage
    {
        LpfType type = LpfType::PT1;
        float hz = 0.0f;   // 0 — выключена
    };

    struct DynLpf
    {
        float minHz = 0.0f, maxHz = 0.0f;
        uint8_t expo = 5;
        bool on() const { return minHz > 0.0f && maxHz > minHz; }
    };

    struct NotchSpec
    {
        float hz = 0.0f, cutoffHz = 0.0f;
        bool on() const { return hz > 0.0f && cutoffHz > 0.0f && cutoffHz < hz; }
        float q() const { return hz * cutoffHz / (hz * hz - cutoffHz * cutoffHz); }
    };

    constexpr uint8_t DYN_NOTCH_MAX = 5;      // dyn_notch_count — до 5, как в BF
    constexpr uint8_t RPM_HARMONICS_MAX = 3;  // rpm_filter_harmonics — до 3

    // Динамический режектор: count режекторов на оси едут за пиками спектра
    // гироскопа в полосе minHz..maxHz; добротность q (dyn_notch_q / 100).
    struct DynNotchSpec
    {
        uint8_t count = 0;   // 0 — выключен
        float q = 3.0f;
        float minHz = 100.0f, maxHz = 600.0f;
        bool on() const { return count > 0 && maxHz > minHz; }
    };

    // RPM-фильтр: на каждый мотор — режекторы на гармониках его оборотов.
    // Ниже minHz режектор стоит на minHz с весом 0, выше — вес растёт до
    // weights[h] на полосе fadeHz; обороты сглаживаются ФНЧ lpfHz.
    struct RpmFilterSpec
    {
        uint8_t harmonics = RPM_HARMONICS_MAX;   // 0 — выключен
        float weights[RPM_HARMONICS_MAX] = { 1.0f, 1.0f, 1.0f };
        float q = 5.0f;
        float minHz = 100.0f;
        float fadeHz = 50.0f;
        float lpfHz = 150.0f;   // 0 — без сглаживания

        // Вес гармоники на частоте hz (0 — режектор не работает).
        float weightAt(uint8_t h, float hz) const
        {
            if (h >= harmonics || hz <= 0.0f || hz < minHz) return 0.0f;
            const float fade = fadeHz > 0.0f ? fminf(1.0f, (hz - minHz) / fadeHz) : 1.0f;
            return weights[h] * fade;
        }
    };

    struct FilterSpec
    {
        bool valid = false;   // false — цепочка не задана (старый путь: PT2 гироскопа и PT1 D)
        LpfStage gyro1, gyro2;
        DynLpf gyroDyn;
        NotchSpec gyroNotch[2];
        LpfStage dterm1, dterm2;
        DynLpf dtermDyn;
        NotchSpec dtermNotch;
        DynNotchSpec dynNotch;
        RpmFilterSpec rpm;
    };

    inline bool sameStage(const LpfStage& a, const LpfStage& b) { return a.type == b.type && a.hz == b.hz; }
    inline bool sameDyn(const DynLpf& a, const DynLpf& b) { return a.minHz == b.minHz && a.maxHz == b.maxHz && a.expo == b.expo; }
    inline bool sameNotch(const NotchSpec& a, const NotchSpec& b) { return a.hz == b.hz && a.cutoffHz == b.cutoffHz; }

    inline bool sameDynNotch(const DynNotchSpec& a, const DynNotchSpec& b)
    {
        return a.count == b.count && a.q == b.q && a.minHz == b.minHz && a.maxHz == b.maxHz;
    }
    inline bool sameRpm(const RpmFilterSpec& a, const RpmFilterSpec& b)
    {
        for (uint8_t h = 0; h < RPM_HARMONICS_MAX; ++h)
        {
            if (a.weights[h] != b.weights[h]) return false;
        }
        return a.harmonics == b.harmonics && a.q == b.q && a.minHz == b.minHz && a.fadeHz == b.fadeHz && a.lpfHz == b.lpfHz;
    }

    inline bool sameFilters(const FilterSpec& a, const FilterSpec& b)
    {
        return sameDynNotch(a.dynNotch, b.dynNotch) && sameRpm(a.rpm, b.rpm) && a.valid == b.valid && sameStage(a.gyro1, b.gyro1) && sameStage(a.gyro2, b.gyro2) && sameDyn(a.gyroDyn, b.gyroDyn) &&
               sameNotch(a.gyroNotch[0], b.gyroNotch[0]) && sameNotch(a.gyroNotch[1], b.gyroNotch[1]) &&
               sameStage(a.dterm1, b.dterm1) && sameStage(a.dterm2, b.dterm2) && sameDyn(a.dtermDyn, b.dtermDyn) &&
               sameNotch(a.dtermNotch, b.dtermNotch);
    }

    // Срез динамического ФНЧ при газе 0..1.
    inline float dynLpfHz(const DynLpf& d, float throttle)
    {
        const float t = fminf(1.0f, fmaxf(0.0f, throttle));
        const float curve = t + t * (1.0f - t) * d.expo / 10.0f;
        return d.minHz + (d.maxHz - d.minHz) * curve;
    }

    // Поправка частоты одного звена PTn, чтобы у n звеньев −3 дБ было на срезе.
    inline float ptnCorrection(uint8_t order)
    {
        return order == 2 ? 1.553773974f : order == 3 ? 1.961459177f : 1.0f;
    }

    // Одна ступень ФНЧ любого типа; срез можно менять на ходу.
    class Lowpass
    {
    public:
        void configure(LpfType t, float hz, float dtS)
        {
            type = t;
            dt = dtS;
            cutoff = -1.0f;
            setCutoff(hz);
            reset();
        }

        // Пересчитать коэффициенты (состояние не трогается); 0 — выключить.
        void setCutoff(float hz)
        {
            if (fabsf(hz - cutoff) < 0.5f) return;
            cutoff = hz;
            on = hz > 0.0f && dt > 0.0f && hz < 0.49f / dt;
            if (!on) return;
            if (type == LpfType::BIQUAD)
            {
                bq.lowpass(hz, 1.0f / dt);
                return;
            }
            const uint8_t n = order();
            const float f = hz * ptnCorrection(n);
            for (uint8_t i = 0; i < 3; ++i) pt[i].configure(i < n ? f : 0.0f, dt);
        }

        float apply(float x)
        {
            if (!on) return x;
            if (type == LpfType::BIQUAD) return bq.apply(x);
            const uint8_t n = order();
            for (uint8_t i = 0; i < n; ++i) x = pt[i].apply(x);
            return x;
        }

        void reset()
        {
            for (Pt1& p : pt) p.reset();
            bq.reset();
        }

        bool enabled() const { return on; }
        float cutoffHz() const { return on ? cutoff : 0.0f; }

    private:
        uint8_t order() const { return type == LpfType::PT3 ? 3 : type == LpfType::PT2 ? 2 : 1; }

        LpfType type = LpfType::PT1;
        float dt = 0.001f;
        float cutoff = 0.0f;
        bool on = false;
        Pt1 pt[3];
        Biquad bq;
    };

    // Цепочка одной оси: ФНЧ 1 (может быть динамическим), ФНЧ 2, режекторы.
    template <uint8_t NOTCHES>
    class AxisChain
    {
    public:
        void configure(const LpfStage& a, const LpfStage& b, const DynLpf& dyn, const NotchSpec* notches, float dtS)
        {
            dynamic = dyn;
            lpf1.configure(a.type, dynamic.on() ? dynamic.minHz : a.hz, dtS);
            lpf2.configure(b.type, b.hz, dtS);
            for (uint8_t i = 0; i < NOTCHES; ++i)
            {
                const NotchSpec& n = notches[i];
                notch[i].notch(n.on() ? n.hz : 0.0f, 1.0f / dtS, n.on() ? n.q() : 1.0f);
                notch[i].reset();
            }
        }

        // Газ 0..1 — срез динамического ФНЧ.
        void setThrottle(float throttle)
        {
            if (dynamic.on()) lpf1.setCutoff(dynLpfHz(dynamic, throttle));
        }

        float apply(float x) { return applyLowpass(applyNotches(x)); }
        float applyNotches(float x)
        {
            for (uint8_t i = 0; i < NOTCHES; ++i) x = notch[i].apply(x);
            return x;
        }
        float applyLowpass(float x) { return lpf2.apply(lpf1.apply(x)); }

        void reset()
        {
            lpf1.reset();
            lpf2.reset();
            for (uint8_t i = 0; i < NOTCHES; ++i) notch[i].reset();
        }

        float lpf1Hz() const { return lpf1.cutoffHz(); }

    private:
        DynLpf dynamic;
        Lowpass lpf1, lpf2;
        Biquad notch[NOTCHES];
    };
}
