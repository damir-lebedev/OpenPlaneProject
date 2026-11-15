// ============================================================
// Математика идентификации и автонастройки (copter/SysId.h, AutoTune.h) на
// известных системах — без модели коптера. Те же опыты, с теми же числами,
// повторяет OpenPlane Studio (src/core/sysid.test.ts, autotune.test.ts):
// генератор — целочисленный, поэтому данные в обоих языках совпадают до
// последнего знака, и результаты сверяются напрямую.
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <vector>

#include "copter/AutoTune.h"

using namespace copter;

namespace
{
    // Замкнутый контур с известным объектом: K/(s(1 + sτ)), задержка — целое
    // число циклов; регулятор — PI; возбуждение — chirp в команду; шум гироскопа.
    struct Synthetic
    {
        std::vector<double> r, u, y, yf;
    };

    constexpr double SYN_DT = 0.001;
    constexpr double SYN_K = 120000.0, SYN_TAU = 0.03;
    constexpr int SYN_DELAY = 2;
    constexpr double SYN_P = 0.0008, SYN_I = 0.012;
    constexpr double SYN_FILTER = 0.6;   // гироскоп после фильтра: PT1 с этим коэффициентом

    Synthetic synthetic(int samples, double noiseDps)
    {
        Synthetic s;
        uint32_t seed = 12345u;
        const auto noise = [&seed]() {
            double sum = 0.0;
            for (int i = 0; i < 4; ++i)
            {
                seed = seed * 1664525u + 1013904223u;
                sum += static_cast<double>(seed) / 4294967296.0 - 0.5;
            }
            return sum;
        };
        const double f0 = 1.0, f1 = 200.0, total = samples * SYN_DT;
        const double lag = 1.0 - std::exp(-SYN_DT / SYN_TAU);
        double motor = 0.0, rate = 0.0, integral = 0.0, filtered = 0.0, phase = 0.0;
        double queue[SYN_DELAY + 1] = {};
        for (int k = 0; k < samples; ++k)
        {
            const double t = k * SYN_DT;
            phase += 2.0 * sysid::kPi * f0 * std::pow(f1 / f0, t / total) * SYN_DT;
            const double excite = 0.05 * std::sin(phase);
            const double gyro = rate + noiseDps * noise();
            filtered += SYN_FILTER * (gyro - filtered);
            integral += SYN_I * (0.0 - filtered) * SYN_DT;
            const double command = SYN_P * (0.0 - filtered) + integral + excite;
            s.r.push_back(excite);
            s.u.push_back(command);
            s.y.push_back(gyro);
            s.yf.push_back(filtered);
            for (int i = SYN_DELAY; i > 0; --i) queue[i] = queue[i - 1];
            queue[0] = command;
            motor += lag * (queue[SYN_DELAY] - motor);
            rate += SYN_K * motor * SYN_DT;
        }
        return s;
    }
}

void test_sysid_fft_finds_a_sine()
{
    std::vector<sysid::Cx> x(64);
    for (int i = 0; i < 64; ++i) x[i] = 3.0 * std::cos(2.0 * sysid::kPi * 5.0 * i / 64.0) + 1.0;
    sysid::fft(x);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 64.0, std::abs(x[0]));        // постоянная
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 96.0, std::abs(x[5]));        // амплитуда 3 — N·3/2
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 96.0, std::abs(x[59]));
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, std::abs(x[7]));
}

// Частотная характеристика известной системы: объект восстановлен в замкнутом
// контуре, с шумом гироскопа; фильтр гироскопа и шум — тоже.
void test_sysid_recovers_a_known_plant_in_closed_loop()
{
    const Synthetic s = synthetic(16000, 0.6);
    const sysid::Response resp = sysid::estimate(s.r, s.u, s.y, s.yf, 1.0 / SYN_DT);
    TEST_ASSERT_TRUE(resp.bands.size() > 80);
    const sysid::Plant p = sysid::fit(resp, false);
    char line[220];
    snprintf(line, sizeof(line), "synthetic: bands %d, K %.9g, tau %.9g, delay %.9g, error %.9g, points %d", static_cast<int>(resp.bands.size()),
             p.gain, p.tau, p.delay, p.error, p.points);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(p.valid(), line);
    // объект здесь посчитан с шагом цикла (а модель — непрерывная): несколько процентов — от этого
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05 * SYN_K, SYN_K, p.gain, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.07 * SYN_TAU, SYN_TAU, p.tau, line);
    // задержка: очередь в 2 цикла + шаг счёта объекта и удержание команды
    TEST_ASSERT_TRUE_MESSAGE(p.delay > SYN_DELAY * SYN_DT && p.delay < (SYN_DELAY + 1.6) * SYN_DT, line);
    TEST_ASSERT_TRUE_MESSAGE(p.error < 0.08, line);

    // в полосе 5–80 Гц: измеренная ЧХ против точной модели — доли дБ и градусы
    sysid::Plant truth = p;
    for (const sysid::Band& b : resp.bands)
    {
        if (b.hz < 5.0 || b.hz > 80.0) continue;
        TEST_ASSERT_TRUE_MESSAGE(b.coherence > 0.6, line);
        const sysid::Cx ratio = b.plant / truth.at(b.hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.12, 1.0, std::abs(ratio), line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(6.0, 0.0, std::arg(ratio) * 180.0 / sysid::kPi, line);
        // фильтр гироскопа — PT1 с коэффициентом SYN_FILTER
        const sysid::Cx z1 = std::exp(sysid::Cx(0.0, -2.0 * sysid::kPi * b.hz * SYN_DT));
        const sysid::Cx filter = SYN_FILTER / (1.0 - (1.0 - SYN_FILTER) * z1);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.03, std::abs(filter), std::abs(b.gyroFilter), line);
        // контур как летал: L = C·F·G
        const sysid::Cx c = SYN_P + SYN_I * SYN_DT / (1.0 - z1);
        const sysid::Cx loop = c * filter * truth.at(b.hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15 * std::abs(loop), std::abs(loop), std::abs(b.loop), line);
    }
    // шум гироскопа: сумма четырёх равномерных — дисперсия 4/12·σ², белый: СПМ = дисперсия / (fs/2)
    const double psd = 0.6 * 0.6 / 3.0 / 500.0;
    double sum = 0.0;
    int n = 0;
    for (const sysid::Band& b : resp.bands)
    {
        if (b.hz < 250.0) continue;
        sum += b.noise;
        ++n;
    }
    snprintf(line, sizeof(line), "synthetic noise PSD above 250 Hz: %.9g (true %.9g)", sum / n, psd);
    TEST_MESSAGE(line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.2 * psd, psd, sum / n, line);
}

// Запасы эталонного контура L = K·e^(−sT)/s: срез ω = K, запас по фазе
// 90° − K·T, по усилению — π/(2·T·K); пик чувствительности известен численно.
void test_autotune_margins_of_a_reference_loop()
{
    const double K = 60.0, T = 0.008;
    std::vector<double> hz;
    std::vector<sysid::Cx> L;
    for (double f = 0.2; f < 490.0; f *= 1.02)
    {
        const double w = 2.0 * sysid::kPi * f;
        hz.push_back(f);
        L.push_back(K / sysid::Cx(0.0, w) * std::exp(sysid::Cx(0.0, -w * T)));
    }
    const autotune::Margins m = autotune::margins(hz, L);
    char line[200];
    snprintf(line, sizeof(line), "reference loop: crossover %.9g Hz, pm %.9g, gm %.9g dB, ms %.9g dB at %.9g Hz", m.crossoverHz, m.phaseMarginDeg,
             m.gainMarginDb, m.sensitivityDb, m.sensitivityHz);
    TEST_MESSAGE(line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02, K / (2.0 * sysid::kPi), m.crossoverHz, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.2, 90.0 - K * T * 180.0 / sysid::kPi, m.phaseMarginDeg, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05, 20.0 * std::log10(sysid::kPi / (2.0 * T * K)), m.gainMarginDb, line);
    // |S| наибольший между срезом и −180°: 1/|1 + L| больше 1 и меньше 1/(1 − 1/запас)
    TEST_ASSERT_TRUE_MESSAGE(m.sensitivityDb > 2.0 && m.sensitivityDb < 5.0, line);
    TEST_ASSERT_TRUE_MESSAGE(m.sensitivityHz > m.crossoverHz, line);

    // усиление вдвое выше запаса — неустойчив: запас по фазе меньше нуля
    for (sysid::Cx& v : L) v *= 4.0;
    TEST_ASSERT_TRUE(autotune::margins(hz, L).phaseMarginDeg < 0.0);
}

// Синтез на известной модели: запасы выдержаны, шум в бюджете, ступенька не
// хуже, чем у «текущей» настройки; числа — для сверки с расчётом в Studio.
void test_autotune_synthesis_on_a_known_plant()
{
    autotune::AxisData data[3];
    autotune::Controller base[3];
    for (int a = 0; a < 3; ++a)
    {
        data[a].plant.gain = a == 2 ? 18000.0 : 150000.0;
        data[a].plant.tau = 0.03;
        data[a].plant.delay = 0.002;
        data[a].plant.zero = a == 2 ? 0.1 : 0.0;
        data[a].plant.points = 60;
        // фильтр гироскопа — PT2 150 Гц, шум — белый 0.3 °/с на отсчёт плюс вибрация у 180 Гц
        for (double f = 1.0; f < 500.0; f *= 1.06)
        {
            data[a].gyroFilter.add(f, autotune::lowpassAt({ autotune::PT2, 150.0 }, f, 0.001));
            data[a].noise.hz.push_back(f);
            data[a].noise.psd.push_back(0.09 / 500.0 + 0.02 * std::exp(-0.5 * (f - 180.0) * (f - 180.0) / 100.0));
        }
        data[a].stepDps = a == 2 ? 120.0 : 200.0;
        base[a].dt = 0.001;
        base[a].p = a == 2 ? 0.0007 : 0.0008;
        base[a].i = a == 2 ? 0.0015 : 0.011;
        base[a].d = a == 2 ? 0.0 : 0.000015;
        base[a].ff = a == 2 ? 0.00004 : 0.000005;
        base[a].dLpf1 = { autotune::PT1, 110.0 };
        base[a].outputLpfHz = a == 2 ? 75.0 : 0.0;
        base[a].levelGain = a == 2 ? 0.0 : 10.0;
    }
    const autotune::Limits lim;
    const autotune::Proposal p = autotune::synthesize(data, base, lim);
    TEST_ASSERT_TRUE(p.found);
    for (int a = 0; a < 3; ++a)
    {
        const autotune::AxisResult& r = p.axis[a];
        const autotune::Margins was = autotune::marginsOf(data[a], base[a]);
        char line[320];
        snprintf(line, sizeof(line),
                 "known plant axis %d: P %.9g I %.9g D %.9g FF %.9g, D lpf %.0f | crossover %.9g (was %.9g) pm %.9g gm %.9g ms %.9g | noise %.9g | step %.9g s %.9g (was %.9g s %.9g)",
                 a, r.controller.p, r.controller.i, r.controller.d, r.controller.ff, p.dtermLpfHz, r.margins.crossoverHz, was.crossoverHz,
                 r.margins.phaseMarginDeg, r.margins.gainMarginDb, r.margins.sensitivityDb, r.noise, r.step.rise, r.step.overshoot,
                 p.before[a].rise, p.before[a].overshoot);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(r.found, line);
        TEST_ASSERT_TRUE_MESSAGE(r.margins.ok(lim.phaseMarginDeg, lim.gainMarginDb, lim.sensitivityDb + 0.01), line);
        TEST_ASSERT_TRUE_MESSAGE(r.angleMargins.ok(lim.phaseMarginDeg - 0.5, lim.gainMarginDb - 0.1, lim.sensitivityDb + 0.1), line);
        TEST_ASSERT_TRUE_MESSAGE(r.noise <= r.noiseLimit, line);
        TEST_ASSERT_TRUE_MESSAGE(r.margins.crossoverHz > was.crossoverHz, line);        // полоса шире, чем была
        TEST_ASSERT_TRUE_MESSAGE(r.step.stable && r.step.error <= p.before[a].error * 1.02, line);
        // рысканье «текущей» настройки здесь — с рывком и перелётом 76 %: быстрее него
        // по времени до 90 % не сделать, берётся та, где ошибка меньше всего
        if (a < 2) TEST_ASSERT_TRUE_MESSAGE(r.step.rise <= p.before[a].rise + 0.001, line);
        else TEST_ASSERT_TRUE_MESSAGE(r.step.overshoot < 0.2 * p.before[a].overshoot, line);
        TEST_ASSERT_TRUE_MESSAGE(r.controller.p > 0 && r.controller.i > 0 && r.controller.ff > 0, line);
        // срез — на заданной частоте: |L| = 1 там, где сказано
        const autotune::Margins again = autotune::marginsOf(data[a], r.controller);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02 * r.margins.crossoverHz, r.margins.crossoverHz, again.crossoverHz, line);
    }
    TEST_ASSERT_TRUE(p.axis[2].controller.d == 0.0);                                    // у рысканья D нет
    TEST_ASSERT_TRUE(p.axis[2].margins.crossoverHz <= lim.yawShare * p.axis[0].margins.crossoverHz * 1.01);
    // бюджет шума работает: шумнее гироскоп — тише настройка
    for (int a = 0; a < 3; ++a)
    {
        for (double& v : data[a].noise.psd) v *= 400.0;
    }
    const autotune::Proposal noisy = autotune::synthesize(data, base, lim);
    TEST_ASSERT_TRUE(noisy.found);
    TEST_ASSERT_TRUE(noisy.axis[0].margins.crossoverHz < p.axis[0].margins.crossoverHz);
    TEST_ASSERT_TRUE(noisy.axis[0].noise <= noisy.axis[0].noiseLimit);
    TEST_ASSERT_TRUE(noisy.dtermLpfHz <= p.dtermLpfHz);
}
