#pragma once
#include <cmath>
#include <complex>
#include <vector>

#include "copter/SysId.h"

// ============================================================
// Автонастройка контура угловой скорости по измеренной модели (пункт 169).
//
// Расчёт для ПК, как и copter/SysId.h: тесты на модели и эталон для
// OpenPlane Studio (src/core/autotune.ts). Результат — предложение, а не
// запись: коэффициенты и срез ФНЧ D, запасы «было → стало», переходная.
//
// Контур одной оси (в прошивке — copter/RateController.h, дискретный, шаг dt):
//
//   команда = −C(z)·F(z)·гироскоп,   C = [P + I·dt/(1 − z⁻¹) + D·(1 − z⁻¹)/dt·Fd(z)]·Fвых(z)
//   L(jω) = C·F·G   — разомкнутый контур
//
//   G  — объект: модель из sysid::fit (K, τ, задержка, у рысканья τz);
//   F  — фильтры гироскопа, как они работали в полёте: измерены по той же
//        записи (гироскоп до фильтров -> после), вместе с режекторами оборотов;
//   Fd — ФНЧ D-части (dterm_lpf*), Fвых — ФНЧ выхода рысканья (yaw_lowpass_hz).
//
// Синтез — задача оптимизации на сетке частот: найти наибольшую частоту
// среза ωc (полосу контура), при которой выполнены ограничения
//   • запас по фазе ≥ 45°,
//   • запас по усилению ≥ 6 дБ,
//   • чувствительность |S| = |1/(1 + L)| ≤ 6 дБ на всех частотах,
//   • шум гироскопа на моторах — в основном через D-часть — в бюджете: СКЗ не
//     больше 2 % хода моторов от одной оси (по спектру шума гироскопа из той
//     же записи, от 20 Гц до половины частоты цикла); если текущая настройка
//     уже шумит сильнее — не больше, чем она (и не больше 6 %);
//   • ступенька стиком на модели — не хуже, чем у настройки, которая стоит
//     сейчас (интеграл модуля ошибки и время до 90 %): полоса не покупается
//     ценой отклика.
// Свободны: отношение D/P (доля запаздывания мотора τ), срез ФНЧ D и сама ωc;
// P — такое, чтобы |L(ωc)| = 1; I = P·ωc/6 (излом интеграла вшестеро ниже
// среза стоит ~9° фазы). У рысканья D нет, срез — не выше 0.6 среза крена и
// тангажа (моторам рысканье даётся тяжелее всего).
// Упреждение: D-часть работает по гироскопу и на движении стика тормозит
// раму (−D·ускорение), а объекту на ускорение нужна команда ускорение/K;
// поэтому FF = c·D + доля/K (у рысканья K — «быстрое», K·τz/τ: момент
// разгона ротора). c и доля подбираются по ступеньке на модели — самая
// быстрая с перелётом не больше 5 %.
//
// ANGLE. Там уставка — kL·(угол цели − угол), и упреждение от её скорости
// замыкается обратно: −FF·kL·угловая скорость. К обратной связи добавляются
// (P + I/jω)·kL/jω (сам контур горизонта) и FF·kL·H25(jω). Запасы проверяются
// для обоих контуров — в ACRO и в ANGLE; упреждение, с которым ANGLE их не
// держит, не берётся.
//
// Почему эти пороги. 45° и 6 дБ — классические «хорошие» запасы: коптер
// может отличаться от измеренного вдвое по усилению (другая батарея, винты
// после удара) или на 45° по фазе (лишние ~4 мс задержки на срезе 30 Гц) и
// не раскачается. |S| ≤ 6 дБ — возмущение (ветер, пропвош) нигде не
// усиливается больше чем вдвое; это же даёт расстояние от точки −1 не
// меньше 0.5 и закрывает случаи, которых отдельные запасы не видят.
// Запасы по фазе и усилению у настройки «из описания» (Presets.h) — 50° и
// 12 дБ: там модель известна на ±30 %, здесь — измерена, запас на незнание
// не нужен.
// ============================================================

namespace copter
{
    namespace autotune
    {
        using sysid::Cx;
        using sysid::PI;

        constexpr double FF_LPF_HZ = 25.0;   // ФНЧ скорости уставки в упреждении (RateController.h)

        // Тип ступени ФНЧ — номера как у copter::LpfType (и в CLI Betaflight).
        enum Lowpass : int { PT1 = 0, BIQUAD = 1, PT2 = 2, PT3 = 3 };

        struct Stage
        {
            int type = PT1;
            double hz = 0.0;   // 0 — выключена
        };

        // ЧХ дискретного ФНЧ прошивки (copter/Filters.h, FilterChain.h) на частоте hz.
        inline Cx lowpassAt(const Stage& s, double hz, double dt)
        {
            if (s.hz <= 0.0 || s.hz >= 0.49 / dt) return Cx(1.0, 0.0);
            const Cx z1 = std::exp(Cx(0.0, -2.0 * PI * hz * dt));   // z⁻¹
            if (s.type == BIQUAD)
            {
                const double omega = 2.0 * PI * s.hz * dt;
                const double sn = std::sin(omega), cs = std::cos(omega);
                const double alpha = sn / (2.0 * 0.70710678);
                const double a0 = 1.0 + alpha;
                const double b0 = (1.0 - cs) * 0.5 / a0, b1 = (1.0 - cs) / a0;
                const double a1 = -2.0 * cs / a0, a2 = (1.0 - alpha) / a0;
                return (b0 + b1 * z1 + b0 * z1 * z1) / (1.0 + a1 * z1 + a2 * z1 * z1);
            }
            const int order = s.type == PT3 ? 3 : s.type == PT2 ? 2 : 1;
            const double correction = order == 2 ? 1.553773974 : order == 3 ? 1.961459177 : 1.0;
            const double k = 1.0 - std::exp(-2.0 * PI * s.hz * correction * dt);
            const Cx one = k / (1.0 - (1.0 - k) * z1);
            Cx r = one;
            for (int i = 1; i < order; ++i) r *= one;
            return r;
        }

        // Режектор (биквад): центр и «cutoff» — нижний край полосы, как в BF.
        inline Cx notchAt(double centerHz, double cutoffHz, double hz, double dt)
        {
            if (centerHz <= 0.0 || cutoffHz <= 0.0 || cutoffHz >= centerHz || centerHz >= 0.49 / dt) return Cx(1.0, 0.0);
            const double q = centerHz * cutoffHz / (centerHz * centerHz - cutoffHz * cutoffHz);
            const double omega = 2.0 * PI * centerHz * dt;
            const double alpha = std::sin(omega) / (2.0 * q), cs = std::cos(omega);
            const double a0 = 1.0 + alpha;
            const Cx z1 = std::exp(Cx(0.0, -2.0 * PI * hz * dt));
            return (1.0 / a0 - 2.0 * cs / a0 * z1 + 1.0 / a0 * z1 * z1) / (1.0 - 2.0 * cs / a0 * z1 + (1.0 - alpha) / a0 * z1 * z1);
        }

        // Регулятор одной оси, коэффициенты — как в прошивке (команда смесителя на °/с).
        struct Controller
        {
            double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
            Stage dLpf1, dLpf2;
            double dNotchHz = 0.0, dNotchCutoffHz = 0.0;
            double outputLpfHz = 0.0;   // рысканье: yaw_lowpass_hz (PT1)
            double levelGain = 0.0;     // ANGLE: (°/с) на градус ошибки угла; 0 — оси в ANGLE нет (рысканье)
            double dt = 0.001;

            Cx dFilterAt(double hz) const
            {
                return lowpassAt(dLpf1, hz, dt) * lowpassAt(dLpf2, hz, dt) * notchAt(dNotchHz, dNotchCutoffHz, hz, dt);
            }
            // D-часть: от гироскопа (после фильтров) к команде, без знака
            Cx dPathAt(double hz) const
            {
                const Cx z1 = std::exp(Cx(0.0, -2.0 * PI * hz * dt));
                return d * (1.0 - z1) / dt * dFilterAt(hz);
            }
            // C(z): от гироскопа после фильтров к команде (обратная связь, без знака)
            Cx at(double hz) const
            {
                const Cx z1 = std::exp(Cx(0.0, -2.0 * PI * hz * dt));
                Cx c = p + i * dt / (1.0 - z1) + dPathAt(hz);
                if (outputLpfHz > 0.0) c *= lowpassAt({ PT1, outputLpfHz }, hz, dt);
                return c;
            }
            // Добавка к обратной связи в ANGLE — от гироскопа до фильтров (угол
            // считается по нему): контур горизонта и упреждение от скорости уставки
            Cx angleAt(double hz) const
            {
                if (levelGain <= 0.0) return Cx(0.0, 0.0);
                const Cx z1 = std::exp(Cx(0.0, -2.0 * PI * hz * dt));
                const Cx integrator = dt / (1.0 - z1);
                return (p + i * integrator) * levelGain * integrator + ff * levelGain * lowpassAt({ PT1, FF_LPF_HZ }, hz, dt);
            }
        };

        // Кривая по частоте (измеренная): значение между точками — по логарифму
        // частоты: модуль — в логарифме, фаза — линейно (развёрнутая).
        struct Curve
        {
            std::vector<double> hz, lnMag, phase;

            void add(double f, Cx v)
            {
                const double m = std::abs(v);
                if (!(f > 0.0) || !(m > 0.0)) return;
                double ph = std::arg(v);
                if (!phase.empty())
                {
                    while (ph - phase.back() > PI) ph -= 2.0 * PI;
                    while (ph - phase.back() < -PI) ph += 2.0 * PI;
                }
                hz.push_back(f);
                lnMag.push_back(std::log(m));
                phase.push_back(ph);
            }
            bool empty() const { return hz.empty(); }
            Cx at(double f) const
            {
                if (hz.empty()) return Cx(1.0, 0.0);
                if (f <= hz.front()) return std::polar(std::exp(lnMag.front()), phase.front());
                if (f >= hz.back()) return std::polar(std::exp(lnMag.back()), phase.back());
                size_t k = 1;
                while (hz[k] < f) ++k;
                const double t = std::log(f / hz[k - 1]) / std::log(hz[k] / hz[k - 1]);
                return std::polar(std::exp(lnMag[k - 1] + t * (lnMag[k] - lnMag[k - 1])), phase[k - 1] + t * (phase[k] - phase[k - 1]));
            }
        };

        // Фильтр гироскопа и шум по записи свипа.
        inline Curve gyroFilterCurve(const sysid::Response& r)
        {
            Curve c;
            for (const sysid::Band& b : r.bands) c.add(b.hz, b.gyroFilter);
            return c;
        }

        struct Noise
        {
            std::vector<double> hz, psd;   // СПМ шума гироскопа до фильтров, (°/с)²/Гц
        };

        inline Noise noiseCurve(const sysid::Response& r)
        {
            Noise n;
            for (const sysid::Band& b : r.bands)
            {
                n.hz.push_back(b.hz);
                n.psd.push_back(b.noise);
            }
            return n;
        }

        constexpr double NOISE_FROM_HZ = 20.0;   // ниже — не шум, а движение коптера


        struct Margins
        {
            double crossoverHz = 0.0;        // срез: |L| = 1
            double phaseMarginDeg = 180.0;
            double gainMarginDb = 60.0;
            double sensitivityDb = 0.0;      // пик |S|
            double sensitivityHz = 0.0;
            bool ok(double pm, double gm, double ms) const
            {
                return crossoverHz > 0.0 && phaseMarginDeg >= pm && gainMarginDb >= gm && sensitivityDb <= ms;
            }
        };

        // Сетка частот: от GRID_FROM_HZ до 0.49 частоты цикла, GRID_POINTS точек по логарифму.
        constexpr double GRID_FROM_HZ = 0.2;
        constexpr int GRID_POINTS = 240;

        inline std::vector<double> frequencyGrid(double dt)
        {
            std::vector<double> g(GRID_POINTS);
            const double hi = 0.49 / dt;
            for (int i = 0; i < GRID_POINTS; ++i) g[static_cast<size_t>(i)] = GRID_FROM_HZ * std::pow(hi / GRID_FROM_HZ, static_cast<double>(i) / (GRID_POINTS - 1));
            return g;
        }

        // Запасы по значениям L на сетке. Срез — первое пересечение |L| = 1
        // сверху вниз; запас по фазе — худший из всех пересечений единицы
        // (фаза развёрнута от нижней частоты: меньше нуля — контур неустойчив);
        // по усилению — худший из всех проходов фазы через −180° − 360°·n:
        // на сколько дБ усиление можно поднять (|L| < 1) или опустить (|L| > 1).
        inline Margins margins(const std::vector<double>& hz, const std::vector<Cx>& L)
        {
            Margins m;
            if (hz.size() < 2) return m;
            double prevMag = std::abs(L[0]), prevPhase = std::arg(L[0]);
            // далеко внизу контур с интегралом — двойной интегратор: фаза у −180°, а не у +180°
            if (prevPhase > 0.0) prevPhase -= 2.0 * PI;
            double peak = 1.0 / std::abs(1.0 + L[0]);
            m.sensitivityHz = hz[0];
            for (size_t k = 1; k < hz.size(); ++k)
            {
                const double mag = std::abs(L[k]);
                double phase = std::arg(L[k]);
                while (phase - prevPhase > PI) phase -= 2.0 * PI;
                while (phase - prevPhase < -PI) phase += 2.0 * PI;
                const double s = 1.0 / std::abs(1.0 + L[k]);
                if (s > peak)
                {
                    peak = s;
                    m.sensitivityHz = hz[k];
                }
                if ((prevMag - 1.0) * (mag - 1.0) < 0.0)
                {
                    const double t = std::log(prevMag) / (std::log(prevMag) - std::log(mag));
                    const double at = hz[k - 1] * std::pow(hz[k] / hz[k - 1], t);
                    const double pm = 180.0 + (prevPhase + t * (phase - prevPhase)) * 180.0 / PI;
                    if (m.crossoverHz == 0.0 && prevMag > mag) m.crossoverHz = at;
                    m.phaseMarginDeg = std::fmin(m.phaseMarginDeg, pm);
                }
                // фаза прошла −180° − 360°·n
                const double a = (prevPhase + PI) / (2.0 * PI), b = (phase + PI) / (2.0 * PI);
                if (std::floor(a) != std::floor(b))
                {
                    const double edge = (prevPhase > phase ? std::floor(a) : std::floor(b)) * 2.0 * PI - PI;
                    const double t = (edge - prevPhase) / (phase - prevPhase);
                    const double at = std::exp(std::log(prevMag) + t * (std::log(mag) - std::log(prevMag)));
                    m.gainMarginDb = std::fmin(m.gainMarginDb, std::fabs(20.0 * std::log10(at)));
                }
                prevMag = mag;
                prevPhase = phase;
            }
            m.sensitivityDb = 20.0 * std::log10(peak);
            if (m.crossoverHz == 0.0) m.phaseMarginDeg = 0.0;   // усиление нигде не падает ниже 1 — контура нет
            return m;
        }

        // Объект и фильтры гироскопа одной оси — то, что измерено.
        struct AxisData
        {
            sysid::Plant plant;
            Curve gyroFilter;
            Noise noise;
            double stepDps = 200.0;   // обычная ступенька стиком по этой оси, °/с (расход на 40 % стика)
        };

        // СКЗ шума гироскопа на входе смесителя (доля хода моторов): шум и
        // вибрация до фильтров, прошедшие всю обратную связь — фильтры и
        // регулятор, а в ANGLE ещё контур горизонта и упреждение (туда гироскоп
        // идёт мимо фильтров). Почти всё — D-часть (растёт с частотой); у
        // рысканья D нет, зато P велико — считается вместе.
        inline double noiseRms(const AxisData& a, const Controller& c)
        {
            const Noise& n = a.noise;
            double sum = 0.0, prev = 0.0;
            for (size_t k = 0; k < n.hz.size(); ++k)
            {
                const double f = n.hz[k];
                const double v = n.psd[k] * std::norm(c.at(f) * a.gyroFilter.at(f) + c.angleAt(f));
                if (k > 0 && n.hz[k - 1] >= NOISE_FROM_HZ) sum += 0.5 * (prev + v) * (f - n.hz[k - 1]);
                prev = v;
            }
            return std::sqrt(sum);
        }

        // Контур оси на сетке частот.
        inline std::vector<Cx> loop(const AxisData& a, const Controller& c, const std::vector<double>& hz)
        {
            std::vector<Cx> L(hz.size());
            for (size_t k = 0; k < hz.size(); ++k) L[k] = c.at(hz[k]) * a.gyroFilter.at(hz[k]) * a.plant.at(hz[k]);
            return L;
        }

        // То же в ANGLE: с контуром горизонта и упреждением от скорости уставки.
        inline std::vector<Cx> angleLoop(const AxisData& a, const Controller& c, const std::vector<double>& hz)
        {
            std::vector<Cx> L(hz.size());
            for (size_t k = 0; k < hz.size(); ++k) L[k] = (c.at(hz[k]) * a.gyroFilter.at(hz[k]) + c.angleAt(hz[k])) * a.plant.at(hz[k]);
            return L;
        }

        inline Margins marginsOf(const AxisData& a, const Controller& c)
        {
            const std::vector<double> hz = frequencyGrid(c.dt);
            return margins(hz, loop(a, c, hz));
        }

        inline Margins angleMarginsOf(const AxisData& a, const Controller& c)
        {
            const std::vector<double> hz = frequencyGrid(c.dt);
            return margins(hz, angleLoop(a, c, hz));
        }

        // ---------------- переходная на модели ----------------

        struct Step
        {
            double rise = 0.0;        // до 90 % уставки, с
            double overshoot = 0.0;   // перелёт, доля уставки
            double error = 0.0;       // интеграл |уставка − отклик|, доля уставки × с
            bool stable = true;
        };

        constexpr double STEP_SECONDS = 0.45;
        constexpr double STEP_SETPOINT_HZ = 90.0;   // сглаживание стика (PT3), как у пульта 250 Гц
        // Наибольшая команда оси: смеситель делит её между моторами с весами ±1 и
        // размах больше хода мотора ужимает — оси достаётся не больше ±0.5
        constexpr double MIXER_LIMIT = 0.5;
        constexpr double RELAX_DPS = 40.0;          // I-relax: уставка убежала от сглаженной — I стоит
        constexpr double RELAX_HZ = 10.0;

        // Ступенька уставки угловой скорости на линейной модели: тот же дискретный
        // регулятор, объект — интегратор с запаздыванием мотора и задержкой
        // (к задержке объекта добавлена задержка фильтров гироскопа на срезе).
        // Высота ступеньки — обычная для этой оси (AxisData::stepDps), команда — не
        // больше того, что пропустит смеситель: упреждение, которое в него
        // упирается, отклик не ускоряет, а D-часть, которой мотор не поспевает
        // помочь, его тормозит. trace — отклик по циклам (для графика), доля уставки.
        // Наружу — ещё и интеграл модуля ошибки (доля уставки × с).
        inline Step simulateStep(const AxisData& a, const Controller& c, std::vector<double>* trace = nullptr)
        {
            Step out;
            const sysid::Plant& g = a.plant;
            if (!g.valid()) return out;
            const double dt = c.dt;
            const double STEP_DPS = a.stepDps > 0.0 ? a.stepDps : 200.0;
            const int SUB = 4;
            const double h = dt / SUB;
            // задержка фильтров гироскопа: наклон фазы у 20 Гц
            const double f0 = 20.0;
            const double gyroDelay = std::fmax(0.0, -std::arg(a.gyroFilter.at(f0)) / (2.0 * PI * f0));
            // задержка объекта меряна «в том же цикле»: полцикла из неё — удержание
            // команды, оно здесь получается само
            const double delay = std::fmax(0.0, g.delay - 0.5 * dt) + gyroDelay;
            const int line = static_cast<int>(std::lround(delay / h)) + 1;
            std::vector<double> queue(static_cast<size_t>(line), 0.0);
            size_t head = 0;

            const auto pt1 = [dt](double hz) { return 1.0 - std::exp(-2.0 * PI * hz * dt); };
            const double kSp = pt1(STEP_SETPOINT_HZ * 1.961459177), kFf = pt1(FF_LPF_HZ), kRelax = pt1(RELAX_HZ);
            const double kOut = c.outputLpfHz > 0.0 ? pt1(c.outputLpfHz) : 1.0;
            // ФНЧ D: до двух ступеней PT1…PT3 (биквад — как PT2 на той же частоте)
            double dState[6] = {};
            double dK[6] = {};
            int dStages = 0;
            for (const Stage* s : { &c.dLpf1, &c.dLpf2 })
            {
                if (s->hz <= 0.0 || s->hz >= 0.49 / dt) continue;
                const int order = s->type == PT3 ? 3 : (s->type == PT2 || s->type == BIQUAD) ? 2 : 1;
                const double correction = order == 2 ? 1.553773974 : order == 3 ? 1.961459177 : 1.0;
                for (int k = 0; k < order; ++k) dK[dStages++] = pt1(s->hz * correction);
            }

            double motor = 0.0, rate = 0.0;
            double sp1 = 0.0, sp2 = 0.0, sp3 = 0.0, spPrev = 0.0, spSlow = 0.0;
            double integral = 0.0, ffRate = 0.0, yPrev = 0.0, outState = 0.0;
            const double ratio = g.tau > 0.0 ? g.zero / g.tau : 0.0;
            const int steps = static_cast<int>(STEP_SECONDS / dt);
            double peak = 0.0;
            for (int k = 0; k < steps; ++k)
            {
                // уставка: ступенька через PT3
                sp1 += kSp * (STEP_DPS - sp1);
                sp2 += kSp * (sp1 - sp2);
                sp3 += kSp * (sp2 - sp3);
                const double sp = sp3;
                const double y = rate;
                const double error = sp - y;
                spSlow += kRelax * (sp - spSlow);
                const double relax = std::fmax(0.0, 1.0 - std::fabs(sp - spSlow) / RELAX_DPS);
                integral += c.i * error * dt * relax;
                ffRate += kFf * ((sp - spPrev) / dt - ffRate);
                spPrev = sp;
                double dRate = (y - yPrev) / dt;
                yPrev = y;
                for (int s = 0; s < dStages; ++s)
                {
                    dState[s] += dK[s] * (dRate - dState[s]);
                    dRate = dState[s];
                }
                double u = c.p * error + integral - c.d * dRate + c.ff * ffRate;
                outState += kOut * (u - outState);
                u = outState;
                if (u > MIXER_LIMIT) u = MIXER_LIMIT;
                if (u < -MIXER_LIMIT) u = -MIXER_LIMIT;
                for (int s = 0; s < SUB; ++s)
                {
                    queue[head] = u;
                    head = (head + 1) % queue.size();
                    const double ud = queue[head];   // самое старое: line шагов назад
                    motor += (ud - motor) * h / g.tau;
                    rate += g.gain * (ratio * ud + (1.0 - ratio) * motor) * h;
                }
                const double norm = rate / STEP_DPS;
                out.error += std::fabs(sp / STEP_DPS - norm) * dt;
                if (trace) trace->push_back(norm);
                if (out.rise == 0.0 && norm >= 0.9) out.rise = (k + 1) * dt;
                peak = std::fmax(peak, norm);
                if (!(std::fabs(norm) < 20.0)) out.stable = false;
            }
            out.overshoot = std::fmax(0.0, peak - 1.0);
            if (out.rise == 0.0) out.rise = STEP_SECONDS;
            // не затухло к концу — неустойчив
            if (std::fabs(rate / STEP_DPS - 1.0) > 0.2) out.stable = false;
            return out;
        }

        // ---------------- синтез ----------------

        struct Limits
        {
            double phaseMarginDeg = 45.0;
            double gainMarginDb = 6.0;
            double sensitivityDb = 6.0;
            double noise = 0.02;           // СКЗ шума гироскопа на входе смесителя от одной оси, доля хода моторов
            double noiseCeiling = 0.06;    // текущая настройка шумит больше бюджета — не больше, чем она, и не больше этого
            double yawOvershoot = 0.10;    // рысканье: рывок ротора модель знает хуже — перелёт до
            double iRatio = 1.0 / 6.0;     // I = P·ωc·iRatio
            double yawShare = 0.6;         // срез рысканья — не выше этой доли среза крена и тангажа
            double ffOvershoot = 0.05;     // упреждение: перелёт на модели не больше
            double maxCrossoverShare = 0.12;   // срез не выше этой доли частоты цикла
        };

        struct AxisResult
        {
            Controller controller;
            Margins margins;          // контур угловой скорости (ACRO)
            Margins angleMargins;     // он же в ANGLE (у рысканья — тот же)
            Step step;
            double noise = 0.0;       // шум гироскопа на входе смесителя, СКЗ
            double noiseLimit = 0.0;  // и предел, с которым шёл поиск
            bool found = false;
        };

        namespace detail
        {
            // «Быстрое» усиление объекта: у рысканья — с моментом разгона ротора (K·τz/τ).
            inline double fastGain(const sysid::Plant& g)
            {
                return g.gain * std::fmax(1.0, g.tau > 0.0 ? g.zero / g.tau : 1.0);
            }

            // Всё, что на частотах не зависит от искомых P, I, D, FF: считается
            // один раз на ось и срез ФНЧ D. Две сетки: контура (запасы) и полос
            // записи (шум).
            struct Table
            {
                std::vector<double> hz;
                std::vector<Cx> integrator, dPath, output, plant, filtered, ffPath;   // dt/(1−z⁻¹); (1−z⁻¹)/dt·Fd; Fвых; G; F·G (или F); kL·H25·G (или kL·H25)
            };

            inline void fill(Table& t, const AxisData& a, const Controller& base, const std::vector<double>& hz, bool withPlant)
            {
                Controller unit = base;
                unit.p = unit.i = unit.ff = 0.0;
                unit.d = 1.0;
                const size_t n = hz.size();
                t.hz = hz;
                t.integrator.resize(n);
                t.dPath.resize(n);
                t.output.resize(n);
                t.plant.resize(n);
                t.filtered.resize(n);
                t.ffPath.resize(n);
                for (size_t k = 0; k < n; ++k)
                {
                    const double f = hz[k];
                    const Cx z1 = std::exp(Cx(0.0, -2.0 * PI * f * base.dt));
                    const Cx g = withPlant ? a.plant.at(f) : Cx(1.0, 0.0);
                    t.integrator[k] = base.dt / (1.0 - z1);
                    t.dPath[k] = unit.dPathAt(f);
                    t.output[k] = base.outputLpfHz > 0.0 ? lowpassAt({ PT1, base.outputLpfHz }, f, base.dt) : Cx(1.0, 0.0);
                    t.plant[k] = g;
                    t.filtered[k] = a.gyroFilter.at(f) * g;
                    t.ffPath[k] = base.levelGain > 0.0 ? base.levelGain * lowpassAt({ PT1, FF_LPF_HZ }, f, base.dt) * g : Cx(0.0, 0.0);
                }
            }

            struct Tables
            {
                Table loop, noise;
                std::vector<double> psd;
                std::vector<Cx> buffer;
                double levelGain = 0.0;
            };

            inline Tables makeTables(const AxisData& a, const Controller& base)
            {
                Tables t;
                fill(t.loop, a, base, frequencyGrid(base.dt), true);
                fill(t.noise, a, base, a.noise.hz, false);
                t.psd = a.noise.psd;
                t.buffer.resize(t.loop.hz.size());
                t.levelGain = base.levelGain;
                return t;
            }

            // Обратная связь в точке k: по угловой скорости и добавка ANGLE (без объекта и фильтра гироскопа).
            inline Cx rateFeedback(const Table& t, size_t k, const Controller& c)
            {
                return (c.p + c.i * t.integrator[k] + c.d * t.dPath[k]) * t.output[k];
            }
            inline Cx angleFeedback(const Table& t, size_t k, const Controller& c, double levelGain)
            {
                return (c.p + c.i * t.integrator[k]) * levelGain * t.integrator[k] * t.plant[k] + c.ff * t.ffPath[k];
            }

            inline Margins rateMargins(Tables& t, const Controller& c)
            {
                for (size_t k = 0; k < t.loop.hz.size(); ++k) t.buffer[k] = rateFeedback(t.loop, k, c) * t.loop.filtered[k];
                return margins(t.loop.hz, t.buffer);
            }
            inline Margins angleMargins(Tables& t, const Controller& c)
            {
                for (size_t k = 0; k < t.loop.hz.size(); ++k)
                    t.buffer[k] = rateFeedback(t.loop, k, c) * t.loop.filtered[k] + angleFeedback(t.loop, k, c, t.levelGain);
                return margins(t.loop.hz, t.buffer);
            }
            inline double noise(const Tables& t, const Controller& c)
            {
                double sum = 0.0, prev = 0.0;
                for (size_t k = 0; k < t.noise.hz.size(); ++k)
                {
                    const double v = t.psd[k] * std::norm(rateFeedback(t.noise, k, c) * t.noise.filtered[k] + angleFeedback(t.noise, k, c, t.levelGain));
                    if (k > 0 && t.noise.hz[k - 1] >= NOISE_FROM_HZ) sum += 0.5 * (prev + v) * (t.noise.hz[k] - t.noise.hz[k - 1]);
                    prev = v;
                }
                return std::sqrt(sum);
            }

            // Регулятор с заданными срезом, долей D и ФНЧ D: P — из |L(ωc)| = 1.
            // Упреждение — наибольшее, какое может выбрать подбор: запасы в ANGLE
            // и шум считаются с ним.
            inline Controller shaped(const AxisData& a, const Controller& base, double crossoverHz, double dOverP, double iRatio)
            {
                Controller c = base;
                c.ff = 0.0;
                c.p = 1.0;
                c.i = 2.0 * PI * crossoverHz * iRatio;
                c.d = dOverP;
                const double mag = std::abs(c.at(crossoverHz) * a.gyroFilter.at(crossoverHz) * a.plant.at(crossoverHz));
                const double p = mag > 0.0 ? 1.0 / mag : 0.0;
                c.p = p;
                c.i *= p;
                c.d *= p;
                c.ff = c.d + 1.0 / fastGain(a.plant);
                return c;
            }

            inline bool feasible(Tables& t, const Controller& c, const Limits& lim, double noiseLimit, Margins& m, double& n)
            {
                m = rateMargins(t, c);
                if (!m.ok(lim.phaseMarginDeg, lim.gainMarginDb, lim.sensitivityDb)) return false;
                n = noise(t, c);
                if (n > noiseLimit) return false;
                if (t.levelGain <= 0.0) return true;
                return angleMargins(t, c).ok(lim.phaseMarginDeg, lim.gainMarginDb, lim.sensitivityDb);
            }

            // Наибольший срез для заданной доли D (и ФНЧ D в base): перебор сверху вниз, затем деление пополам.
            inline bool bestCrossover(const AxisData& a, Tables& t, const Controller& base, double dOverP, const Limits& lim,
                                      double noiseLimit, double maxHz, AxisResult& out)
            {
                const double minHz = 0.3;
                const int SCAN = 48;
                double okHz = 0.0, badHz = maxHz;
                Margins m;
                double n = 0.0;
                for (int i = 0; i <= SCAN; ++i)
                {
                    const double f = maxHz * std::pow(minHz / maxHz, static_cast<double>(i) / SCAN);
                    if (feasible(t, shaped(a, base, f, dOverP, lim.iRatio), lim, noiseLimit, m, n))
                    {
                        okHz = f;
                        break;
                    }
                    badHz = f;
                }
                if (okHz == 0.0) return false;
                for (int it = 0; it < 10 && badHz > okHz * 1.002; ++it)
                {
                    const double mid = std::sqrt(okHz * badHz);
                    (feasible(t, shaped(a, base, mid, dOverP, lim.iRatio), lim, noiseLimit, m, n) ? okHz : badHz) = mid;
                }
                out.controller = shaped(a, base, okHz, dOverP, lim.iRatio);
                feasible(t, out.controller, lim, noiseLimit, out.margins, out.noise);
                out.noiseLimit = noiseLimit;
                out.found = true;
                return true;
            }

            // Упреждение FF = c·D + доля/K: с наименьшим интегралом ошибки ступеньки
            // на модели среди тех, где перелёт в пределах, запасы в ANGLE держатся и
            // шум в бюджете. Перелёт в пределы не лезет — самое спокойное.
            inline void tuneFeedforward(const AxisData& a, Tables& t, AxisResult& r, const Limits& lim);
        }

        // Доли D/P в единицах запаздывания мотора τ и срезы ФНЧ D, среди которых идёт поиск.
        constexpr double D_SHARES[] = { 0.0, 0.15, 0.25, 0.35, 0.5, 0.65, 0.8, 1.0, 1.25, 1.5 };
        constexpr double D_LPF_HZ[] = { 30.0, 40.0, 50.0, 65.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0 };
        constexpr double FF_SHARES[] = { 0.2, 0.6, 1.0 };
        constexpr double FF_D_SHARES[] = { 0.0, 0.5, 0.75, 0.9, 1.0 };   // какая доля D возвращается упреждением
        // «не хуже текущей»: интеграл ошибки ступеньки — в пределах 2 %, время до 90 % — не больше
        constexpr double STEP_ERROR_TOLERANCE = 1.02, STEP_RISE_TOLERANCE = 1.0;

        inline void detail::tuneFeedforward(const AxisData& a, Tables& t, AxisResult& r, const Limits& lim)
        {
            if (!r.found) return;
            const double fast = fastGain(a.plant);
            const double limit = t.levelGain > 0.0 ? lim.ffOvershoot : lim.yawOvershoot;
            double bestError = 1e9, lowest = 1e9;
            Controller pick = r.controller, calm = r.controller;
            pick.ff = calm.ff = FF_SHARES[0] / fast;
            bool any = false;
            for (double comp : FF_D_SHARES)
            {
                if (comp > 0.0 && r.controller.d <= 0.0) break;
                for (double share : FF_SHARES)
                {
                    Controller c = r.controller;
                    c.ff = comp * c.d + share / fast;
                    const Step s = simulateStep(a, c);
                    if (!s.stable) continue;
                    if (s.overshoot < lowest)
                    {
                        lowest = s.overshoot;
                        calm = c;
                    }
                    if (s.overshoot <= limit && s.error < bestError)
                    {
                        bestError = s.error;
                        pick = c;
                        any = true;
                    }
                }
            }
            r.controller = any ? pick : calm;
            r.step = simulateStep(a, r.controller);
            r.angleMargins = t.levelGain > 0.0 ? angleMargins(t, r.controller) : r.margins;
            r.noise = noise(t, r.controller);
        }

        // Одна ось при заданном ФНЧ D (в base): для каждой доли D — наибольший
        // срез; от наибольшего среза к меньшему подбирается упреждение, и берётся
        // первая настройка, где ступенька на модели не хуже опорной (reference —
        // у текущей настройки; nullptr — без этого условия). Ни одна не дотянула —
        // та, где ступенька лучше всех. withD = false — рысканье (без D).
        // noiseNow — шум текущей настройки (предел шума — не строже, чем уже есть).
        inline AxisResult tuneAxis(const AxisData& a, const Controller& base, const Limits& lim, bool withD, double maxHz,
                                   const Step* reference = nullptr, double noiseNow = 0.0)
        {
            AxisResult none;
            if (!a.plant.valid()) return none;
            detail::Tables t = detail::makeTables(a, base);
            const double noiseLimit = std::fmax(lim.noise, std::fmin(noiseNow, lim.noiseCeiling));
            const int count = withD ? static_cast<int>(sizeof(D_SHARES) / sizeof(D_SHARES[0])) : 1;
            std::vector<AxisResult> found;
            for (int k = 0; k < count; ++k)
            {
                AxisResult r;
                if (detail::bestCrossover(a, t, base, D_SHARES[k] * a.plant.tau, lim, noiseLimit, maxHz, r)) found.push_back(r);
            }
            // от большего среза к меньшему (почти равные — с меньшей долей D вперёд)
            std::vector<size_t> order(found.size());
            for (size_t i = 0; i < order.size(); ++i) order[i] = i;
            for (size_t i = 1; i < order.size(); ++i)
            {
                for (size_t j = i; j > 0 && found[order[j]].margins.crossoverHz > found[order[j - 1]].margins.crossoverHz * 1.02; --j)
                    std::swap(order[j], order[j - 1]);
            }
            const bool bounded = reference && reference->stable;
            AxisResult closest;
            for (size_t index : order)
            {
                AxisResult r = found[index];
                detail::tuneFeedforward(a, t, r, lim);
                if (!r.step.stable) continue;
                if (!bounded || (r.step.error <= reference->error * STEP_ERROR_TOLERANCE &&
                                 r.step.rise <= reference->rise * STEP_RISE_TOLERANCE + base.dt))
                    return r;
                if (!closest.found || r.step.error < closest.step.error) closest = r;
            }
            return closest;
        }

        struct Proposal
        {
            AxisResult axis[3];
            Step before[3];            // ступенька текущей настройки на той же модели
            double noiseBefore[3] = {};   // и её шум
            double dtermLpfHz = 0.0;   // срез первой ступени ФНЧ D (общий для осей)
            bool found = false;
        };

        // Настройка трёх осей. base[ось] — регулятор как сейчас: из него берутся
        // шаг, тип ФНЧ D, вторая ступень, режектор D, ФНЧ выхода рысканья и
        // усиление горизонта; P, I, D, FF и срез первой ступени ФНЧ D — ищутся.
        // Срез ФНЧ D общий: берётся тот, при котором меньший из срезов крена и
        // тангажа наибольший.
        inline Proposal synthesize(const AxisData data[3], const Controller base[3], const Limits& lim = Limits())
        {
            Proposal out;
            for (int a = 0; a < 3; ++a)
            {
                out.before[a] = simulateStep(data[a], base[a]);
                out.noiseBefore[a] = noiseRms(data[a], base[a]);
            }
            const double maxHz = lim.maxCrossoverShare / base[0].dt;
            double bestScore = 0.0;
            for (double lpf : D_LPF_HZ)
            {
                if (lpf >= 0.3 / base[0].dt) break;
                AxisResult rp[2];
                double score = 1e9;
                for (int a = 0; a < 2; ++a)
                {
                    Controller b = base[a];
                    b.dLpf1.hz = lpf;
                    rp[a] = tuneAxis(data[a], b, lim, true, maxHz, &out.before[a], out.noiseBefore[a]);
                    score = std::fmin(score, rp[a].found ? rp[a].margins.crossoverHz : 0.0);
                }
                if (score > bestScore * 1.02)
                {
                    bestScore = score;
                    out.axis[0] = rp[0];
                    out.axis[1] = rp[1];
                    out.dtermLpfHz = lpf;
                }
            }
            if (bestScore <= 0.0) return out;
            Controller yaw = base[2];
            yaw.dLpf1.hz = out.dtermLpfHz;
            out.axis[2] = tuneAxis(data[2], yaw, lim, false, lim.yawShare * bestScore, &out.before[2], out.noiseBefore[2]);
            out.found = out.axis[0].found && out.axis[1].found && out.axis[2].found;
            return out;
        }
    }
}
