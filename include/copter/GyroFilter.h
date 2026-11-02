#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/FilterChain.h"
#include "copter/Filters.h"
#include "copter/Frames.h"
#include "copter/Spectrum.h"

// ============================================================
// Фильтр гироскопа мультикоптера.
//
// Главный шум гироскопа на коптере — вибрация от моторов: на частоте
// вращения, на её удвоенной и на частоте прохода лопастей (обороты ×
// число лопастей). Режекторы ставятся ровно туда и едут вслед за
// оборотами; обороты — по модели винтомоторной группы (Presets.h:
// доля тяги и напряжение -> частота вращения). Датчика оборотов не
// нужно (двунаправленный DShot даст точные — тогда центр берётся из
// него, setRotationHz). Дальше — ФНЧ второго порядка: всё, что выше
// полосы контура, но мимо режекторов. Настройки BF (gyro_lpf1/2_*,
// динамический срез по газу, gyro_notch1/2_*) — configureChain.
//
// Режектор не опускается ниже minNotchHz: у больших винтов вращение
// медленное, и режектор у полосы контура съел бы запас по фазе.
//
// Пункты 060–063: режекторы по оборотам — на каждый мотор свои
// (rpm_filter_*: гармоники, веса, плавное включение выше min, ФНЧ
// оборотов; обороты — модель по команде мотора или телеметрия DShot), за
// ними статические режекторы, спектр (copter/Spectrum.h) и динамические
// режекторы по его пикам (dyn_notch_*), в конце ФНЧ.
// ============================================================

namespace copter
{
    constexpr uint8_t NOTCH_HARMONICS = 3;

    // Где стоят режекторы при частоте вращения rotHz: 1×, 2× и проход
    // лопастей (у двухлопастных — 3×). Ниже minHz — выключен (0).
    inline void notchCenters(float rotHz, uint8_t blades, float minHz, float out[NOTCH_HARMONICS])
    {
        const float mult[NOTCH_HARMONICS] = { 1.0f, 2.0f, blades > 2 ? static_cast<float>(blades) : 3.0f };
        for (uint8_t h = 0; h < NOTCH_HARMONICS; ++h)
        {
            const float f = rotHz * mult[h];
            out[h] = rotHz > 0.0f && f >= minHz ? f : 0.0f;
        }
    }

    // Старый путь (цепочка не задана): PT2 гироскопа, режекторы оборотов
    // на 1×, 2×, 3× без плавного включения, без сглаживания оборотов.
    inline FilterSpec legacyFilters(float gyroLpfHz, float dtermLpfHz, float notchQ, float minNotchHz)
    {
        FilterSpec f;
        f.gyro1 = { LpfType::PT2, gyroLpfHz };
        f.dterm1 = { LpfType::PT1, dtermLpfHz };
        f.rpm.q = notchQ;
        f.rpm.minHz = minNotchHz;
        f.rpm.fadeHz = 0.0f;
        f.rpm.lpfHz = 0.0f;
        return f;
    }

    class GyroFilter
    {
    public:
        void configure(float loopHz, float lpfHz, float notchQ, uint8_t blades, float minNotchHz)
        {
            fs = loopHz;
            bladeCount = blades;
            configureChain(legacyFilters(lpfHz, 0.0f, notchQ, minNotchHz));
        }

        // ФНЧ, статические и динамические режекторы, RPM-фильтр как в BF — после configure().
        void configureChain(const FilterSpec& f)
        {
            for (int a = 0; a < 3; ++a) chain[a].configure(f.gyro1, f.gyro2, f.gyroDyn, f.gyroNotch, 1.0f / fs);
            rpm = f.rpm;
            if (rpm.harmonics > RPM_HARMONICS_MAX) rpm.harmonics = RPM_HARMONICS_MAX;
            dyn = f.dynNotch;
            if (dyn.count > DYN_NOTCH_MAX) dyn.count = DYN_NOTCH_MAX;
            spectrum.configure(fs, dyn);
            for (int a = 0; a < 3; ++a)
            {
                for (int i = 0; i < DYN_NOTCH_MAX; ++i) dynNotch[a][i].notch(0.0f, fs, dyn.q);
            }
            const float lpf = rpm.lpfHz > 0.0f && rpm.lpfHz < 0.49f * fs ? rpm.lpfHz : 0.0f;
            const float rc = lpf > 0.0f ? 1.0f / (2.0f * PI_F * lpf) : 0.0f;
            motorAlpha = lpf > 0.0f ? (1.0f / fs) / (1.0f / fs + rc) : 1.0f;
            reset();
        }

        // Газ 0..1 — срез динамического ФНЧ (gyro_lpf1_dyn_*).
        void setThrottle(float throttle)
        {
            for (int a = 0; a < 3; ++a) chain[a].setThrottle(throttle);
        }

        // Срез первой ступени сейчас (динамический — по газу), Гц.
        float lpf1Hz() const { return chain[0].lpf1Hz(); }

        void reset()
        {
            for (int a = 0; a < 3; ++a)
            {
                chain[a].reset();
                for (int i = 0; i < DYN_NOTCH_MAX; ++i) dynNotch[a][i].reset();
            }
            for (uint8_t m = 0; m < MAX_MOTORS; ++m)
            {
                motorHz[m] = 0.0f;
                for (uint8_t h = 0; h < RPM_HARMONICS_MAX; ++h) rpmNotch[m][h] = RpmNotch{};
            }
            spectrum.reset();
        }

        // Частота вращения всех моторов одна, Гц (0 — стоят, режекторы выключены).
        void setRotationHz(float hz)
        {
            const float one[1] = { hz };
            setMotorHz(one, 1);
        }

        // Частота вращения каждого мотора, Гц: count моторов (режекторы на каждый).
        void setMotorHz(const float* hz, uint8_t count)
        {
            motors = count < MAX_MOTORS ? count : MAX_MOTORS;
            for (uint8_t m = 0; m < motors; ++m)
            {
                const float target = hz[m] > 0.0f ? hz[m] : 0.0f;
                // мотор встал или только пошёл — сразу; иначе — ФНЧ оборотов (rpm_filter_lpf_hz)
                motorHz[m] = target <= 0.0f || motorHz[m] <= 0.0f ? target : motorHz[m] + (target - motorHz[m]) * motorAlpha;
                float centers[NOTCH_HARMONICS];
                notchCenters(motorHz[m], bladeCount, 0.0f, centers);
                for (uint8_t h = 0; h < RPM_HARMONICS_MAX; ++h) setRpmNotch(rpmNotch[m][h], centers[h], h);
            }
        }

        // rates — °/с по трём осям; фильтруется на месте.
        void apply(float rates[3])
        {
            float v[3];
            for (int a = 0; a < 3; ++a)
            {
                float x = rates[a];
                for (uint8_t m = 0; m < motors; ++m)
                {
                    for (uint8_t h = 0; h < RPM_HARMONICS_MAX; ++h) x = rpmNotch[m][h].apply(x, a);
                }
                v[a] = chain[a].applyNotches(x);
            }
            const int updated = spectrum.push(v);
            if (updated >= 0)
            {
                for (uint8_t i = 0; i < dyn.count; ++i) dynNotch[updated][i].setCenter(spectrum.center(updated, i));
            }
            for (int a = 0; a < 3; ++a)
            {
                float x = v[a];
                for (uint8_t i = 0; i < dyn.count; ++i) x = dynNotch[a][i].apply(x);
                rates[a] = chain[a].applyLowpass(x);
            }
        }

        // Центр режектора гармоники у мотора 0 (0 — выключен).
        float notchCenter(uint8_t harmonic) const { return motorNotchHz(0, harmonic); }
        float motorNotchHz(uint8_t motor, uint8_t harmonic) const
        {
            if (motor >= motors || harmonic >= RPM_HARMONICS_MAX) return 0.0f;
            const RpmNotch& n = rpmNotch[motor][harmonic];
            return n.weight > 0.0f ? n.center : 0.0f;
        }
        // Обороты мотора после ФНЧ, Гц.
        float motorRotationHz(uint8_t motor) const { return motor < motors ? motorHz[motor] : 0.0f; }
        uint8_t motorCount() const { return motors; }
        // Динамический режектор: центр i на оси (0 — ещё не встал).
        float dynNotchHz(uint8_t axis, uint8_t i) const { return i < dyn.count ? spectrum.center(axis, i) : 0.0f; }
        uint8_t dynNotchCount() const { return dyn.count; }
        const GyroSpectrum& getSpectrum() const { return spectrum; }

    private:
        // Режектор с общими для трёх осей коэффициентами и весом:
        // y = x + w·(режектор(x) − x).
        struct RpmNotch
        {
            float b0 = 1.0f, b1 = 0.0f, a2 = 0.0f;   // у режектора b2 = b0, a1 = b1
            float weight = 0.0f, center = 0.0f;
            float z1[3] = {}, z2[3] = {};

            float apply(float x, int a)
            {
                if (weight <= 0.0f) return x;
                const float y = b0 * x + z1[a];
                z1[a] = b1 * (x - y) + z2[a];
                z2[a] = b0 * x - a2 * y;
                return x + weight * (y - x);
            }
        };

        void setRpmNotch(RpmNotch& n, float hz, uint8_t h)
        {
            float w = rpm.weightAt(h, hz);
            const float c = fmaxf(hz, rpm.minHz);
            if (c >= 0.48f * fs) w = 0.0f;
            if (w <= 0.0f)
            {
                n.weight = 0.0f;   // состояние остаётся: включится снова — без скачка
                return;
            }
            n.weight = w;
            if (fabsf(c - n.center) < 0.05f) return;
            n.center = c;
            const float omega = 2.0f * PI_F * c / fs;
            const float alpha = sinf(omega) / (2.0f * rpm.q);
            const float a0 = 1.0f + alpha;
            n.b0 = 1.0f / a0;
            n.b1 = -2.0f * cosf(omega) / a0;
            n.a2 = (1.0f - alpha) / a0;
        }

        AxisChain<2> chain[3];
        RpmFilterSpec rpm;
        DynNotchSpec dyn;
        GyroSpectrum spectrum;
        Biquad dynNotch[3][DYN_NOTCH_MAX];
        RpmNotch rpmNotch[MAX_MOTORS][RPM_HARMONICS_MAX];
        float motorHz[MAX_MOTORS] = {};
        float motorAlpha = 1.0f;
        float fs = 1000.0f;
        uint8_t motors = 0;
        uint8_t bladeCount = 3;
    };
}
