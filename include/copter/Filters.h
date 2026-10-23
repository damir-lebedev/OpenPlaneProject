#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// Фильтры мультикоптера: PT1, PT2, биквад (ФНЧ и режектор).
//
// Всё — с шагом dt, заданным при настройке (цикл стабилизации идёт с
// постоянной частотой). Биквад — по «кулинарной книге» RBJ, прямая
// форма II транспонированная; центр режектора можно двигать на ходу
// (оценка оборотов моторов), состояние при этом не сбрасывается.
// ============================================================

namespace copter
{
    constexpr float PI_F = 3.14159265358979f;

    class Pt1
    {
    public:
        // cutoffHz <= 0 — фильтр выключен (проходит как есть).
        void configure(float cutoffHz, float dtS)
        {
            if (cutoffHz <= 0.0f || dtS <= 0.0f)
            {
                k = 1.0f;
                return;
            }
            // точный дискретный аналог RC-цепочки: частота среза та же, что
            // заложена в модель контура (Presets.h), а не ниже на десятки %
            k = 1.0f - expf(-2.0f * PI_F * cutoffHz * dtS);
        }

        float apply(float x)
        {
            y += k * (x - y);
            return y;
        }

        void reset(float value = 0.0f) { y = value; }
        float value() const { return y; }

    private:
        float k = 1.0f;
        float y = 0.0f;
    };

    // Два PT1 подряд; частота каждого поднята так, что у пары −3 дБ ровно на cutoffHz.
    class Pt2
    {
    public:
        void configure(float cutoffHz, float dtS)
        {
            const float corrected = cutoffHz > 0.0f ? cutoffHz * 1.553773974f : 0.0f;
            a.configure(corrected, dtS);
            b.configure(corrected, dtS);
        }

        float apply(float x) { return b.apply(a.apply(x)); }

        void reset(float value = 0.0f)
        {
            a.reset(value);
            b.reset(value);
        }

    private:
        Pt1 a, b;
    };

    class Biquad
    {
    public:
        void lowpass(float cutoffHz, float sampleHz, float quality = 0.70710678f)
        {
            kind = LOWPASS;
            fs = sampleHz;
            q = quality;
            enabled = cutoffHz > 0.0f && cutoffHz < 0.49f * sampleHz;
            if (enabled) compute(cutoffHz);
        }

        // Режектор: центр и добротность Q = центр / ширина полосы.
        void notch(float centerHz, float sampleHz, float quality)
        {
            kind = NOTCH;
            fs = sampleHz;
            q = quality;
            setCenter(centerHz);
        }

        // Сдвинуть центр режектора (или частоту ФНЧ), не трогая состояние.
        void setCenter(float hz)
        {
            enabled = hz > 0.0f && hz < 0.49f * fs;
            if (enabled) compute(hz);
        }

        float apply(float x)
        {
            if (!enabled) return x;
            const float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }

        void reset()
        {
            z1 = 0.0f;
            z2 = 0.0f;
        }

        bool isEnabled() const { return enabled; }

    private:
        enum Kind : uint8_t { LOWPASS, NOTCH };

        void compute(float hz)
        {
            const float omega = 2.0f * PI_F * hz / fs;
            const float sn = sinf(omega);
            const float cs = cosf(omega);
            const float alpha = sn / (2.0f * q);
            const float a0 = 1.0f + alpha;
            if (kind == LOWPASS)
            {
                b0 = (1.0f - cs) * 0.5f / a0;
                b1 = (1.0f - cs) / a0;
                b2 = b0;
            }
            else
            {
                b0 = 1.0f / a0;
                b1 = -2.0f * cs / a0;
                b2 = b0;
            }
            a1 = -2.0f * cs / a0;
            a2 = (1.0f - alpha) / a0;
        }

        Kind kind = LOWPASS;
        bool enabled = false;
        float fs = 1000.0f;
        float q = 0.70710678f;
        float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
        float z1 = 0.0f, z2 = 0.0f;
    };
}
