#pragma once
#include <cmath>
#include <complex>
#include <vector>

// ============================================================
// Идентификация коптера по записи chirp (пункт 168): частотная
// характеристика «команда оси → угловая скорость» и модель по ней.
//
// Расчёт для ПК: тесты на модели (test_copter_sim) и эталон для того же
// расчёта в OpenPlane Studio (src/core/sysid.ts — те же формулы, сверка по
// общим данным). В прошивку не входит.
//
// Что записано (запись QCHIRP чёрного ящика, один цикл — одна строка):
//   r  — возбуждение chirp, доля хода смесителя;
//   u  — команда оси на входе смесителя: сумма PID + r;
//   y  — гироскоп до фильтров, °/с;
//   yf — гироскоп после фильтров (его видит PID).
//
// 1. Спектры по Уэлчу: запись режется на отрезки с перекрытием 75 %, окно
//    Ханна, БПФ; копятся взаимные спектры Sab = Σ conj(A)·B.
// 2. Измерение идёт в замкнутом контуре, поэтому y/u напрямую (H1 = Suy/Suu)
//    врёт: шум гироскопа через PID попадает в u и «сам с собой» согласован.
//    Опора — возбуждение r, оно известно точно и с шумом не связано:
//       чувствительность  S = u/r  = Sru/Srr     (H1 от r к u),
//       замкнутый ответ   y/r      = Sry/Srr     (H1 от r к y),
//       объект            G = y/u  = Sry/Sru,
//       контур как летал  L = 1/S − 1 = Srr/Sru − 1,
//       фильтр гироскопа  F = yf/y = Syf/Syy     (тут опора — сам гироскоп).
// 3. Усреднение по частоте: бины собираются в полосы, равные по логарифму
//    (12 на октаву). У chirp каждая частота звучит один раз — без этого
//    когерентность была бы всегда 1. Когерентность γ² = |Sry|²/(Srr·Syy)
//    (и то же для u): доля мощности, объяснённая возбуждением; с поправкой
//    на число усреднений n: (n·γ² − 1)/(n − 1).
// 4. Модель: G(jω) = K·(1 + jωτz) / (jω·(1 + jωτ)) · e^(−jωT)
//       K  — угловое ускорение на единицу команды, (°/с²): тяга и инерция рамы;
//       τ  — запаздывание мотора с винтом;
//       T  — транспортная задержка: регулятор, цикл, фильтр чипа гироскопа;
//       τz — только рысканье: разгон ротора даёт момент сразу (τz > τ).
//    Подгонка — взвешенный МНК по логарифму комплексной ЧХ: ошибка —
//    (Δln|G|)² + (Δфазы)², вес полосы — γ²/(1 − γ²) (обратная дисперсия
//    оценки), не больше WEIGHT_CAP. При заданных τ и τz ln K и T входят
//    линейно и находятся сразу; τ и τz — перебором по логарифмической сетке
//    и уточнением золотым сечением. Локальных минимумов поэтому нет.
// ============================================================

namespace copter
{
    namespace sysid
    {
        using Cx = std::complex<double>;
        constexpr double kPi = 3.14159265358979323846;

        // БПФ на месте, длина — степень двойки.
        inline void fft(std::vector<Cx>& x)
        {
            const size_t n = x.size();
            for (size_t i = 1, j = 0; i < n; ++i)
            {
                size_t bit = n >> 1;
                for (; j & bit; bit >>= 1) j ^= bit;
                j ^= bit;
                if (i < j) std::swap(x[i], x[j]);
            }
            for (size_t len = 2; len <= n; len <<= 1)
            {
                const double ang = -2.0 * kPi / static_cast<double>(len);
                const Cx wl(std::cos(ang), std::sin(ang));
                for (size_t i = 0; i < n; i += len)
                {
                    Cx w(1.0, 0.0);
                    for (size_t k = 0; k < len / 2; ++k)
                    {
                        const Cx a = x[i + k], b = x[i + k + len / 2] * w;
                        x[i + k] = a + b;
                        x[i + k + len / 2] = a - b;
                        w *= wl;
                    }
                }
            }
        }

        struct Band
        {
            double hz = 0.0;          // где в полосе было возбуждение (среднее с весом его мощности)
            Cx plant;                 // G: команда оси -> гироскоп, (°/с) на единицу хода
            Cx loop;                  // L: разомкнутый контур, как он летал
            Cx gyroFilter;            // F: гироскоп до фильтров -> после
            double coherence = 0.0;   // γ²(r→y)·γ²(r→u), 0…1
            double noise = 0.0;       // СПМ гироскопа до фильтров мимо возбуждения (шум, вибрация), (°/с)²/Гц
            int count = 0;            // усреднено спектральных отсчётов
        };

        struct Response
        {
            std::vector<Band> bands;
            double sampleHz = 0.0;
            double seconds = 0.0;
        };

        struct Options
        {
            int segment = 0;            // длина отрезка, степень двойки; 0 — сама (около 4 с, не больше четверти записи)
            int bandsPerOctave = 12;
            double minHz = 0.0;         // 0 — от первого бина
            double maxHz = 0.0;         // 0 — до половины частоты выборки (шум у неё — самый громкий для D-части)
        };

        constexpr double OVERLAP = 0.75;

        // Частотные характеристики по записи одного свипа.
        inline Response estimate(const std::vector<double>& r, const std::vector<double>& u, const std::vector<double>& y,
                                 const std::vector<double>& yf, double sampleHz, const Options& opt = Options())
        {
            Response out;
            out.sampleHz = sampleHz;
            const size_t n = r.size();
            out.seconds = sampleHz > 0.0 ? static_cast<double>(n) / sampleHz : 0.0;
            if (n < 256 || u.size() != n || y.size() != n || yf.size() != n || sampleHz <= 0.0) return out;

            size_t N = 256;
            if (opt.segment > 0)
            {
                while (N * 2 <= static_cast<size_t>(opt.segment)) N *= 2;
            }
            else
            {
                while (N * 2 <= n / 4 && static_cast<double>(N * 2) <= 4.2 * sampleHz) N *= 2;
            }
            if (N > n) return out;
            const size_t hop = static_cast<size_t>(static_cast<double>(N) * (1.0 - OVERLAP));
            const size_t half = N / 2;

            std::vector<double> window(N);
            double wPower = 0.0;
            for (size_t i = 0; i < N; ++i)
            {
                window[i] = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(i) / static_cast<double>(N));
                wPower += window[i] * window[i];
            }

            std::vector<double> srr(half, 0.0), suu(half, 0.0), syy(half, 0.0), sff(half, 0.0);
            std::vector<Cx> sru(half), sry(half), srf(half), syf(half);
            std::vector<Cx> R(N), U(N), Y(N), F(N);
            int segments = 0;
            const std::vector<double>* src[4] = { &r, &u, &y, &yf };
            std::vector<Cx>* dst[4] = { &R, &U, &Y, &F };
            for (size_t start = 0; start + N <= n; start += hop)
            {
                for (int c = 0; c < 4; ++c)
                {
                    const std::vector<double>& s = *src[c];
                    double mean = 0.0;
                    for (size_t i = 0; i < N; ++i) mean += s[start + i];
                    mean /= static_cast<double>(N);
                    std::vector<Cx>& d = *dst[c];
                    for (size_t i = 0; i < N; ++i) d[i] = Cx((s[start + i] - mean) * window[i], 0.0);
                    fft(d);
                }
                for (size_t k = 1; k < half; ++k)
                {
                    srr[k] += std::norm(R[k]);
                    suu[k] += std::norm(U[k]);
                    syy[k] += std::norm(Y[k]);
                    sff[k] += std::norm(F[k]);
                    sru[k] += std::conj(R[k]) * U[k];
                    sry[k] += std::conj(R[k]) * Y[k];
                    srf[k] += std::conj(R[k]) * F[k];
                    syf[k] += std::conj(Y[k]) * F[k];
                }
                ++segments;
            }
            if (segments == 0) return out;

            const double df = sampleHz / static_cast<double>(N);
            const double lo = opt.minHz > df ? opt.minHz : df;
            const double hi = opt.maxHz > 0.0 ? opt.maxHz : 0.5 * sampleHz;
            const int total = static_cast<int>(std::floor(std::log2(hi / lo) * opt.bandsPerOctave)) + 1;
            if (total <= 0) return out;

            struct Sum
            {
                double rr = 0, uu = 0, yy = 0, ff = 0, fhz = 0;
                Cx ru, ry, rf, yf;
                int bins = 0;
            };
            std::vector<Sum> sums(static_cast<size_t>(total));
            for (size_t k = 1; k < half; ++k)
            {
                const double f = static_cast<double>(k) * df;
                if (f < lo || f > hi) continue;
                const int b = static_cast<int>(std::floor(std::log2(f / lo) * opt.bandsPerOctave));
                if (b < 0 || b >= total) continue;
                Sum& s = sums[static_cast<size_t>(b)];
                s.rr += srr[k];
                s.uu += suu[k];
                s.yy += syy[k];
                s.ff += sff[k];
                s.fhz += f * srr[k];
                s.ru += sru[k];
                s.ry += sry[k];
                s.rf += srf[k];
                s.yf += syf[k];
                ++s.bins;
            }

            // отрезки перекрыты на 75 %: независимых усреднений примерно вдвое меньше
            const double psdScale = 2.0 / (sampleHz * wPower);
            for (const Sum& s : sums)
            {
                if (s.bins == 0 || s.rr <= 0.0 || s.uu <= 0.0 || s.yy <= 0.0 || std::abs(s.ru) <= 0.0) continue;
                Band b;
                b.count = s.bins * segments;
                b.hz = s.fhz / s.rr;
                b.plant = s.ry / s.ru;
                b.loop = Cx(s.rr, 0.0) / s.ru - 1.0;
                b.gyroFilter = s.yf / s.yy;
                const double nEff = 0.5 * b.count;
                auto fair = [nEff](double g2) {
                    if (nEff <= 1.5) return 0.0;
                    const double v = (nEff * g2 - 1.0) / (nEff - 1.0);
                    return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
                };
                const double gRy = fair(std::norm(s.ry) / (s.rr * s.yy));
                const double gRu = fair(std::norm(s.ru) / (s.rr * s.uu));
                b.coherence = gRy * gRu;
                const double coherent = std::norm(s.ry) / s.rr;
                b.noise = (s.yy > coherent ? s.yy - coherent : 0.0) * psdScale / static_cast<double>(b.count);
                out.bands.push_back(b);
            }
            return out;
        }

        // Модель объекта: G = K·(1 + jωτz)/(jω(1 + jωτ))·e^(−jωT), выход — °/с.
        struct Plant
        {
            double gain = 0.0;     // K, (°/с²) на единицу команды оси
            double tau = 0.0;      // τ, с
            double delay = 0.0;    // T, с
            double zero = 0.0;     // τz, с (0 — нет)
            double error = 0.0;    // скз остатка подгонки: ln модуля и фаза (рад) вместе
            int points = 0;        // полос в подгонке
            bool valid() const { return gain > 0.0 && tau > 0.0 && points >= 6; }

            Cx at(double hz) const
            {
                const double w = 2.0 * kPi * hz;
                const Cx jw(0.0, w);
                return gain * (1.0 + jw * zero) / (jw * (1.0 + jw * tau)) * std::exp(Cx(0.0, -w * delay));
            }
        };

        constexpr double WEIGHT_CAP = 20.0;      // γ² = 0.95 и выше — один вес
        constexpr double MIN_COHERENCE = 0.5;    // полосы хуже в подгонку не идут
        constexpr double MAX_DELAY_S = 0.02;

        inline double wrapPi(double a)
        {
            while (a > kPi) a -= 2.0 * kPi;
            while (a < -kPi) a += 2.0 * kPi;
            return a;
        }

        namespace detail
        {
            struct Point
            {
                double w, lnMag, phase, weight;
            };

            // При заданных τ и τz: ln K и T — сразу, наружу — взвешенный средний квадрат остатка.
            inline double solveLinear(const std::vector<Point>& pts, double tau, double zero, double& lnK, double& delay)
            {
                double sw = 0.0, sm = 0.0;
                for (const Point& p : pts)
                {
                    const double shape = -std::log(p.w) - 0.5 * std::log1p(p.w * p.w * tau * tau) + 0.5 * std::log1p(p.w * p.w * zero * zero);
                    sw += p.weight;
                    sm += p.weight * (p.lnMag - shape);
                }
                lnK = sm / sw;
                // фаза без задержки; остаток должен быть −ωT (по модулю 2π): грубо по сетке, затем МНК
                std::vector<double> rho(pts.size());
                for (size_t i = 0; i < pts.size(); ++i)
                {
                    const Point& p = pts[i];
                    rho[i] = p.phase + 0.5 * kPi + std::atan(p.w * tau) - std::atan(p.w * zero);
                }
                double best = 1e300, bestT = 0.0;
                for (double T = 0.0; T <= MAX_DELAY_S; T += 0.0001)
                {
                    double c = 0.0;
                    for (size_t i = 0; i < pts.size(); ++i)
                    {
                        const double e = wrapPi(rho[i] + pts[i].w * T);
                        c += pts[i].weight * e * e;
                    }
                    if (c < best)
                    {
                        best = c;
                        bestT = T;
                    }
                }
                double num = 0.0, den = 0.0;
                for (size_t i = 0; i < pts.size(); ++i)
                {
                    const double e = wrapPi(rho[i] + pts[i].w * bestT);
                    num += pts[i].weight * pts[i].w * e;
                    den += pts[i].weight * pts[i].w * pts[i].w;
                }
                delay = bestT - (den > 0.0 ? num / den : 0.0);
                if (delay < 0.0) delay = 0.0;
                double cost = 0.0;
                for (size_t i = 0; i < pts.size(); ++i)
                {
                    const Point& p = pts[i];
                    const double shape = -std::log(p.w) - 0.5 * std::log1p(p.w * p.w * tau * tau) + 0.5 * std::log1p(p.w * p.w * zero * zero);
                    const double em = p.lnMag - shape - lnK;
                    const double ep = wrapPi(rho[i] + p.w * delay);
                    cost += p.weight * (em * em + ep * ep);
                }
                return cost / sw;
            }

            // Минимум функции одной переменной на отрезке (золотое сечение).
            template <typename F>
            double golden(F f, double a, double b, int iterations = 40)
            {
                const double g = 0.6180339887498949;
                double c = b - g * (b - a), d = a + g * (b - a);
                double fc = f(c), fd = f(d);
                for (int i = 0; i < iterations; ++i)
                {
                    if (fc < fd)
                    {
                        b = d;
                        d = c;
                        fd = fc;
                        c = b - g * (b - a);
                        fc = f(c);
                    }
                    else
                    {
                        a = c;
                        c = d;
                        fc = fd;
                        d = a + g * (b - a);
                        fd = f(d);
                    }
                }
                return 0.5 * (a + b);
            }
        }

        // Подгонка модели к измеренной ЧХ. withZero — искать и τz (рысканье).
        inline Plant fit(const Response& resp, bool withZero, double minHz = 0.0, double maxHz = 0.0)
        {
            Plant out;
            std::vector<detail::Point> pts;
            for (const Band& b : resp.bands)
            {
                if (b.coherence < MIN_COHERENCE || b.hz <= 0.0) continue;
                if (minHz > 0.0 && b.hz < minHz) continue;
                if (maxHz > 0.0 && b.hz > maxHz) continue;
                const double mag = std::abs(b.plant);
                if (!(mag > 0.0)) continue;
                const double weight = std::fmin(WEIGHT_CAP, b.coherence / (1.0 - b.coherence + 1e-6));
                pts.push_back({ 2.0 * kPi * b.hz, std::log(mag), std::arg(b.plant), weight });
            }
            out.points = static_cast<int>(pts.size());
            if (pts.size() < 6) return out;

            const double tauLo = std::log(0.002), tauHi = std::log(0.4);
            double bestCost = 1e300, bestTau = 0.03, bestZero = 0.0;
            double lnK = 0.0, delay = 0.0;
            const int GRID = 48;
            for (int i = 0; i <= GRID; ++i)
            {
                const double tau = std::exp(tauLo + (tauHi - tauLo) * i / GRID);
                const int zeros = withZero ? GRID : 0;
                for (int j = -1; j < zeros; ++j)
                {
                    // рысканье: τz от 0.3τ до 30τ (и 0 — без нуля)
                    const double zero = j < 0 ? 0.0 : tau * std::exp(std::log(0.3) + (std::log(30.0) - std::log(0.3)) * j / (GRID - 1));
                    const double c = detail::solveLinear(pts, tau, zero, lnK, delay);
                    if (c < bestCost)
                    {
                        bestCost = c;
                        bestTau = tau;
                        bestZero = zero;
                    }
                }
            }
            // уточнение: по очереди τ и τz
            const double step = (tauHi - tauLo) / GRID;
            for (int round = 0; round < (withZero ? 4 : 1); ++round)
            {
                bestTau = std::exp(detail::golden(
                    [&](double lt) { return detail::solveLinear(pts, std::exp(lt), bestZero, lnK, delay); }, std::log(bestTau) - step,
                    std::log(bestTau) + step));
                if (withZero && bestZero > 0.0)
                {
                    bestZero = std::exp(detail::golden(
                        [&](double lz) { return detail::solveLinear(pts, bestTau, std::exp(lz), lnK, delay); }, std::log(bestZero) - 0.15,
                        std::log(bestZero) + 0.15));
                }
            }
            bestCost = detail::solveLinear(pts, bestTau, bestZero, lnK, delay);
            out.gain = std::exp(lnK);
            out.tau = bestTau;
            out.zero = bestZero;
            out.delay = delay;
            out.error = std::sqrt(bestCost);
            return out;
        }
    }
}
