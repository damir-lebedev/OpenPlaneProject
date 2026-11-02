#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/FilterChain.h"
#include "copter/Filters.h"

// ============================================================
// Спектр гироскопа для динамического режектора (пункт 060, BF
// `dyn_notch_*`). Скользящее ДПФ (SDFT) окном SDFT_SIZE отсчётов по
// каждой оси: на новый отсчёт — одно комплексное умножение на полосу,
// без БПФ. Окно Ханна — свёрткой соседних полос. Пики ищутся в полосе
// minHz..maxHz, частота уточняется параболой по логарифму мощности
// (точнее ширины полосы), центры режекторов сглаживаются.
//
// Цикл быстрее, чем нужно для полосы, — отсчёты усредняются по
// нескольку (частота анализа не ниже 2.2 верха полосы). За отсчёт
// анализа пики ищутся только на одной оси (по кругу).
// Код свой. Без Arduino — тесты на ПК (test_copter).
// ============================================================

namespace copter
{
    constexpr uint8_t SDFT_SIZE = 64;              // окно, отсчётов анализа
    constexpr uint8_t SDFT_BINS = SDFT_SIZE / 2;   // полосы 0..31 (Найквист не нужен)

    class GyroSpectrum
    {
    public:
        // sampleHz — частота отсчётов на входе (цикл); полоса — из d
        // (выключенный режектор: спектр всё равно считается — для приложения).
        void configure(float sampleHz, const DynNotchSpec& d)
        {
            spec = d;
            const float top = d.on() ? d.maxHz : 600.0f;
            decimation = static_cast<uint8_t>(fmaxf(1.0f, fminf(16.0f, floorf(sampleHz / (2.2f * top)))));
            rate = sampleHz / decimation;
            resolution = rate / SDFT_SIZE;
            const float maxHz = fminf(d.maxHz, 0.48f * rate);
            binLo = static_cast<uint8_t>(fmaxf(1.0f, ceilf(d.minHz / resolution)));
            binHi = static_cast<uint8_t>(fminf(SDFT_BINS - 2.0f, floorf(maxHz / resolution)));
            hiHz = maxHz;
            for (uint8_t k = 0; k <= SDFT_BINS; ++k)
            {
                const float w = 2.0f * PI_F * k / SDFT_SIZE;
                twCos[k] = cosf(w);
                twSin[k] = sinf(w);
            }
            rN = powf(DAMPING, SDFT_SIZE);
            // центры: ФНЧ ~20 Гц по обновлениям оси (раз в 3 отсчёта анализа)
            const float dtUpd = 3.0f / rate;
            alpha = dtUpd / (dtUpd + 1.0f / (2.0f * PI_F * 20.0f));
            reset();
        }

        void reset()
        {
            for (uint8_t a = 0; a < 3; ++a)
            {
                acc[a] = 0.0f;
                for (uint8_t i = 0; i < SDFT_SIZE; ++i) ring[a][i] = 0.0f;
                for (uint8_t k = 0; k <= SDFT_BINS; ++k) re[a][k] = im[a][k] = 0.0f;
                for (uint8_t k = 0; k < SDFT_BINS; ++k) power[a][k] = 0.0f;
                for (uint8_t i = 0; i < DYN_NOTCH_MAX; ++i) centers[a][i] = 0.0f;
            }
            accN = 0;
            pos = 0;
            turn = 0;
            filled = 0;
        }

        // Отсчёт трёх осей (°/с). Вернёт ось, у которой обновились пики, или −1.
        int push(const float x[3])
        {
            for (uint8_t a = 0; a < 3; ++a) acc[a] += x[a];
            if (++accN < decimation) return -1;
            for (uint8_t a = 0; a < 3; ++a)
            {
                const float v = acc[a] / accN;
                acc[a] = 0.0f;
                const float d = v - rN * ring[a][pos];
                ring[a][pos] = v;
                // S_k ← W_k·(r·S_k + x_new − r^N·x_old), W_k = e^{j2πk/N}
                for (uint8_t k = 0; k <= SDFT_BINS; ++k)
                {
                    const float tr = DAMPING * re[a][k] + d;
                    const float ti = DAMPING * im[a][k];
                    re[a][k] = tr * twCos[k] - ti * twSin[k];
                    im[a][k] = tr * twSin[k] + ti * twCos[k];
                }
            }
            accN = 0;
            pos = static_cast<uint8_t>((pos + 1) % SDFT_SIZE);
            if (filled < SDFT_SIZE)
            {
                ++filled;
                return -1;   // окно ещё не заполнено
            }
            const uint8_t a = turn;
            turn = static_cast<uint8_t>((turn + 1) % 3);
            findPeaks(a);
            return a;
        }

        // Центр i-го режектора оси (по возрастанию частоты), Гц; 0 — пика ещё не было.
        float center(uint8_t axis, uint8_t i) const { return axis < 3 && i < DYN_NOTCH_MAX ? centers[axis][i] : 0.0f; }
        uint8_t notchCount() const { return spec.on() ? spec.count : 0; }

        // Для приложения: мощность полосы k (окно Ханна, сумма осей, (°/с)²) и её частота.
        float binPower(uint8_t k) const { return k < SDFT_BINS ? power[0][k] + power[1][k] + power[2][k] : 0.0f; }
        float binHz(uint8_t k) const { return k * resolution; }
        float binWidthHz() const { return resolution; }
        float analysisHz() const { return rate; }
        // Мощность полосы на оси: отсчёт окна — амплитуда²·N²/16 у синуса (Ханн).
        float axisPower(uint8_t axis, uint8_t k) const { return axis < 3 && k < SDFT_BINS ? power[axis][k] : 0.0f; }

    private:
        static constexpr float DAMPING = 0.9999f;   // устойчивость SDFT к ошибкам округления
        static constexpr float PEAK_OVER_MEAN = 2.0f;   // пик — мощнее среднего по полосе во столько раз

        void findPeaks(uint8_t a)
        {
            // окно Ханна: Y_k = ½X_k − ¼(X_{k−1} + X_{k+1}); постоянная (X_0 —
            // вращение коптера) в соседнюю полосу не протекает
            for (uint8_t k = 1; k < SDFT_BINS; ++k)
            {
                const float lr = k > 1 ? re[a][k - 1] : 0.0f, li = k > 1 ? im[a][k - 1] : 0.0f;
                const float yr = 0.5f * re[a][k] - 0.25f * (lr + re[a][k + 1]);
                const float yi = 0.5f * im[a][k] - 0.25f * (li + im[a][k + 1]);
                power[a][k] = yr * yr + yi * yi;
            }
            power[a][0] = 0.0f;
            const uint8_t want = notchCount();
            if (want == 0 || binHi <= binLo) return;
            float mean = 0.0f;
            for (uint8_t k = binLo; k <= binHi; ++k) mean += power[a][k];
            mean /= (binHi - binLo + 1);

            // самые сильные локальные максимумы
            uint8_t bins[DYN_NOTCH_MAX];
            uint8_t found = 0;
            for (uint8_t k = binLo; k <= binHi; ++k)
            {
                const float p = power[a][k];
                if (p <= PEAK_OVER_MEAN * mean || p <= power[a][k - 1] || p < power[a][k + 1]) continue;
                uint8_t at = found < want ? found : want;
                while (at > 0 && power[a][bins[at - 1]] < p) --at;
                if (at >= want) continue;
                const uint8_t last = found < want ? found : static_cast<uint8_t>(want - 1);
                for (uint8_t j = last; j > at; --j) bins[j] = bins[j - 1];
                bins[at] = k;
                if (found < want) ++found;
            }
            float hz[DYN_NOTCH_MAX];
            for (uint8_t i = 0; i < found; ++i) hz[i] = peakHz(a, bins[i]);
            // по возрастанию частоты — режекторы не прыгают между пиками
            for (uint8_t i = 1; i < found; ++i)
            {
                for (uint8_t j = i; j > 0 && hz[j] < hz[j - 1]; --j)
                {
                    const float t = hz[j];
                    hz[j] = hz[j - 1];
                    hz[j - 1] = t;
                }
            }
            for (uint8_t i = 0; i < found; ++i)
            {
                float& c = centers[a][i];
                c = c > 0.0f ? c + (hz[i] - c) * alpha : hz[i];
            }
        }

        // Частота пика по трём полосам: парабола по логарифму (у окна Ханна пик почти гауссов).
        float peakHz(uint8_t a, uint8_t k) const
        {
            const float p0 = power[a][k], pm = power[a][k - 1], pp = power[a][k + 1];
            float delta = 0.0f;
            if (pm > 0.0f && pp > 0.0f)
            {
                const float lm = logf(pm), l0 = logf(p0), lp = logf(pp);
                const float den = 2.0f * (2.0f * l0 - lm - lp);
                if (den > 1e-6f) delta = fminf(0.5f, fmaxf(-0.5f, (lp - lm) / den));
            }
            return fminf(hiHz, fmaxf(spec.minHz, (k + delta) * resolution));
        }

        DynNotchSpec spec;
        uint8_t decimation = 1;
        float rate = 1000.0f, resolution = 1000.0f / SDFT_SIZE, hiHz = 480.0f;
        uint8_t binLo = 1, binHi = SDFT_BINS - 2;
        float twCos[SDFT_BINS + 1] = {}, twSin[SDFT_BINS + 1] = {};
        float rN = 1.0f, alpha = 0.3f;
        float acc[3] = {};
        uint8_t accN = 0, pos = 0, turn = 0, filled = 0;
        float ring[3][SDFT_SIZE] = {};
        float re[3][SDFT_BINS + 1] = {}, im[3][SDFT_BINS + 1] = {};
        float power[3][SDFT_BINS] = {};
        float centers[3][DYN_NOTCH_MAX] = {};
    };
}
