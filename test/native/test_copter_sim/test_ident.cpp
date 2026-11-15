// ============================================================
// Chirp и идентификация (пункт 168) в замкнутом полёте: свип по тумблеру
// поверх стабилизации, запись как в чёрном ящике, частотная характеристика
// и модель «команда оси → угловая скорость» против параметров модели
// коптера (helpers/ChirpFlight.h: truePlant).
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "helpers/ChirpFlight.h"
#include "helpers/QuadVariants.h"

using namespace copter;

namespace
{
    const BuildSpec& five() { return *findBuild("freestyle5"); }

    // Взлёт и висение в ANGLE на газе висения.
    bool hoverInAngle(QuadHarness& h, double height)
    {
        if (!h.takeoffTo(height)) return false;
        h.rc.mode = CopterMode::ANGLE;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(1.0);
        return h.ctl.isArmed();
    }

    CopterSettings shortChirp(float seconds)
    {
        CopterSettings s;
        s.chirp.seconds = seconds;
        return s;
    }
}

// Свип начинается только по фронту тумблера, только в воздухе и только в
// ANGLE/ACRO; оси идут по очереди; тумблер, смена режима и DISARM его
// прерывают; коптер при этом висит, где висел.
void test_chirp_runs_by_switch_in_flight_only()
{
    QuadHarness h(five(), QuadPerturb(), 210, EscProtocol::DSHOT600, 1000.0f, shortChirp(4.0f));
    const CopterStatus& st = h.ctl.getStatus();
    h.rc.chirp = true;   // включён ещё до ARM
    TEST_ASSERT_TRUE(hoverInAngle(h, 10.0));
    h.run(0.5);
    TEST_ASSERT_FALSE(st.chirpActive);   // «включён заранее» — не команда
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.chirpExcitation);

    // удержание высоты — не тот режим: фронт пропадает
    h.rc.mode = CopterMode::ALT_HOLD;
    h.rc.throttle = 0.5f;
    h.rc.chirp = false;
    h.run(0.2);
    h.rc.chirp = true;
    h.run(0.3);
    TEST_ASSERT_FALSE(st.chirpActive);
    h.rc.mode = CopterMode::ANGLE;
    h.rc.throttle = st.hoverLearned;
    h.run(0.5);
    TEST_ASSERT_FALSE(st.chirpActive);   // и «дослать» его сменой режима нельзя

    // ANGLE: выключил-включил — крен качается
    h.rc.chirp = false;
    h.run(0.1);
    h.rc.chirp = true;
    h.run(1.0);
    TEST_ASSERT_TRUE(st.chirpActive);
    TEST_ASSERT_EQUAL(0, st.chirpAxis);
    TEST_ASSERT_TRUE(st.chirpHz > 1.0f && st.chirpHz < 10.0f);
    // пилот выключил — конец, та же ось пойдёт снова
    h.rc.chirp = false;
    h.run(0.05);
    TEST_ASSERT_FALSE(st.chirpActive);
    TEST_ASSERT_EQUAL(static_cast<int>(ChirpEnd::SWITCH), static_cast<int>(st.chirpEnd));
    TEST_ASSERT_EQUAL(0, st.chirpAxis);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.chirpExcitation);

    // три свипа целиком: крен, тангаж, рысканье; коптер на месте
    for (int axis = 0; axis < 3; ++axis)
    {
        const ChirpLog log = flyChirp(h);
        char line[160];
        snprintf(line, sizeof(line), "chirp axis %d: %s, %zu samples, tilt up to %.1f deg, rate up to %.0f deg/s, height %.1f..%.1f m", log.axis,
                 chirpEndName(log.end), log.r.size(), log.maxTilt, log.maxRate, log.minHeight, log.maxHeight);
        TEST_MESSAGE(line);
        TEST_ASSERT_EQUAL_MESSAGE(axis, log.axis, line);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(ChirpEnd::DONE), static_cast<int>(log.end), line);
        TEST_ASSERT_INT_WITHIN_MESSAGE(3, 4000, static_cast<int>(log.r.size()), line);
        TEST_ASSERT_TRUE_MESSAGE(log.maxTilt < 12.0, line);
        TEST_ASSERT_TRUE_MESSAGE(log.maxHeight - log.minHeight < 1.5, line);
        TEST_ASSERT_FALSE_MESSAGE(log.saturated, line);
        TEST_ASSERT_TRUE_MESSAGE(log.maxRate > 20.0 && log.maxRate < 300.0, line);
    }
    TEST_ASSERT_EQUAL(0, st.chirpAxis);   // по кругу: снова крен

    // режим сменили посреди свипа — конец; DISARM — тем более
    h.rc.chirp = false;
    h.run(0.1);
    h.rc.chirp = true;
    h.run(0.5);
    TEST_ASSERT_TRUE(st.chirpActive);
    h.rc.mode = CopterMode::ALT_HOLD;
    h.rc.throttle = 0.5f;
    h.run(0.05);
    TEST_ASSERT_FALSE(st.chirpActive);
    TEST_ASSERT_EQUAL(static_cast<int>(ChirpEnd::MODE), static_cast<int>(st.chirpEnd));
    TEST_ASSERT_TRUE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(h.sim.tiltDeg() < 10.0);
}

// Параметры модели коптера восстановлены по свипу: K, τ и задержка крена и
// тангажа — с ошибкой меньше 10 % при шуме и вибрации гироскопа; рысканье —
// по самой частотной характеристике (K и τz порознь ниже 1.5 Гц не видны,
// их произведение — видно).
void test_chirp_identifies_the_plant_within_10_percent()
{
    struct Case
    {
        const char* id;
        QuadPerturb p;
        const char* name;
    };
    const Case cases[] = {
        { "freestyle5", QuadPerturb(), "nominal" }, { "freestyle5", sluggish(), "sluggish" }, { "freestyle5", twitchy(), "twitchy" },
        { "longrange7", QuadPerturb(), "nominal" }, { "hex13", QuadPerturb(), "nominal" },    { "whoop65", QuadPerturb(), "nominal" },
    };
    uint32_t seed = 220;
    for (const Case& c : cases)
    {
        QuadHarness h(*findBuild(c.id), c.p, seed++);
        TEST_ASSERT_TRUE_MESSAGE(hoverInAngle(h, 15.0), c.id);
        for (int axis = 0; axis < 3; ++axis)
        {
            const TruePlant truth = truePlant(h, axis);
            const ChirpLog log = flyChirp(h);
            const sysid::Response resp = chirpResponse(log, h.tune.loopHz);
            const sysid::Plant p = sysid::fit(resp, axis == 2);
            char line[260];
            snprintf(line, sizeof(line),
                     "identified %-10s %-8s axis %d: K %.0f (true %.0f, %+.1f%%), tau %.1f ms (%.1f, %+.1f%%), delay %.2f ms (%.2f, %+.1f%%), tau_z %.0f ms (%.0f), fit rms %.3f on %d bands",
                     c.id, c.name, axis, p.gain, truth.gain, 100 * (p.gain / truth.gain - 1), p.tau * 1e3, truth.tau * 1e3,
                     100 * (p.tau / truth.tau - 1), p.delay * 1e3, truth.delay * 1e3, 100 * (p.delay / truth.delay - 1), p.zero * 1e3,
                     truth.zero * 1e3, p.error, p.points);
            TEST_MESSAGE(line);
            TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(ChirpEnd::DONE), static_cast<int>(log.end), line);
            TEST_ASSERT_TRUE_MESSAGE(p.valid() && p.points >= 40, line);
            TEST_ASSERT_TRUE_MESSAGE(p.error < 0.12, line);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10 * truth.delay, truth.delay, p.delay, line);
            if (axis < 2)
            {
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10 * truth.gain, truth.gain, p.gain, line);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10 * truth.tau, truth.tau, p.tau, line);
            }
            else
            {
                // рысканье: модель и «правда» как частотные характеристики, 3–100 Гц
                sysid::Plant t;
                t.gain = truth.gain;
                t.tau = truth.tau;
                t.delay = truth.delay;
                t.zero = truth.zero;
                for (double hz = 3.0; hz <= 100.0; hz *= 1.3)
                {
                    const sysid::Cx ratio = p.at(hz) / t.at(hz);
                    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10, 1.0, std::abs(ratio), line);
                    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(6.0, 0.0, std::arg(ratio) * 57.29578, line);
                }
            }
        }
    }
}

// ---------------- автонастройка (169) ----------------

namespace
{
    // Запасы контура, как он летает: по записи свипа, без модели.
    autotune::Margins flownMargins(const sysid::Response& r)
    {
        std::vector<double> hz;
        std::vector<sysid::Cx> loop;
        for (const sysid::Band& b : r.bands)
        {
            if (b.coherence < 0.6) continue;
            hz.push_back(b.hz);
            loop.push_back(b.loop);
        }
        return autotune::margins(hz, loop);
    }

    struct TuneRun
    {
        Measured measured;
        autotune::Controller base[3];
        autotune::Proposal proposal;
        RateStep before[3], after[3];
        autotune::Margins flown[3];
        bool flew = false, reflown = false;
        double hoverNoiseBefore = 0, hoverNoiseAfter = 0;
    };

    double hoverNoise(QuadHarness& h)
    {
        settleAt(h, 15.0);
        h.stats.reset();
        h.run(2.0);
        return h.stats.motorNoiseRms();
    }

    // Свипы на настройке «из описания» -> предложение -> тот же коптер с ним.
    // recheck — ещё три свипа уже на новой настройке: запасы, как она летает.
    TuneRun tuneByChirp(const BuildSpec& b, const QuadPerturb& p, uint32_t seed, bool recheck)
    {
        TuneRun run;
        const CopterSettings cs = shortChirp(12.0f);
        QuadHarness h(b, p, seed, EscProtocol::DSHOT600, 1000.0f, cs);
        if (!hoverInAngle(h, 15.0)) return run;
        run.measured = measureByChirp(h);
        for (int a = 0; a < 3; ++a) run.base[a] = controllerOf(h.tune, a);
        run.proposal = autotune::synthesize(run.measured.data, run.base);
        run.hoverNoiseBefore = hoverNoise(h);
        for (int a = 0; a < 3; ++a) run.before[a] = acroStep(h, a);
        if (!run.measured.ok || !run.proposal.found) return run;

        QuadHarness tuned(b, p, seed, EscProtocol::DSHOT600, 1000.0f, cs);
        tuned.useTune(applyProposal(tuned.tune, run.proposal), cs);
        if (!hoverInAngle(tuned, 15.0)) return run;
        run.hoverNoiseAfter = hoverNoise(tuned);
        for (int a = 0; a < 3; ++a) run.after[a] = acroStep(tuned, a);
        run.flew = tuned.ctl.isArmed();
        if (recheck)
        {
            settleAt(tuned, 15.0);
            tuned.rc.mode = CopterMode::ANGLE;
            tuned.rc.throttle = tuned.ctl.getStatus().hoverLearned;
            tuned.run(1.0);
            const Measured again = measureByChirp(tuned);
            run.reflown = again.ok;
            for (int a = 0; a < 3; ++a) run.flown[a] = flownMargins(again.response[a]);
        }
        return run;
    }
}

// Автонастройка по chirp на всём каталоге и на «не той» физике: запасы
// выдержаны (и по модели, и в повторном свипе — как летает), шум в бюджете,
// ступенька стиком не хуже, чем на настройке «из описания» (Presets.h).
void test_autotune_by_chirp_on_every_build()
{
    uint8_t count = 0;
    const BuildSpec* cat = buildCatalog(count);
    const QuadPerturb variants[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    const autotune::Limits lim;
    for (uint8_t b = 0; b < count; ++b)
    {
        for (int v = 0; v < 3; ++v)
        {
            const bool recheck = strcmp(cat[b].id, "freestyle5") == 0 ||
                                 (v == 0 && (strcmp(cat[b].id, "whoop65") == 0 || strcmp(cat[b].id, "longrange7") == 0 ||
                                             strcmp(cat[b].id, "hex13") == 0));
            const TuneRun r = tuneByChirp(cat[b], variants[v], 300 + v, recheck);
            const autotune::Proposal& p = r.proposal;
            char line[420];
            snprintf(line, sizeof(line),
                     "autotune %-12s %-8s: crossover %4.1f>%4.1f Hz (yaw %4.1f>%4.1f), D lpf %3.0f Hz | step roll %2.0f ms %2.0f%% > %2.0f ms %2.0f%%, pitch %2.0f ms %2.0f%% > %2.0f ms %2.0f%%, yaw overshoot %3.0f%% > %2.0f%% | error x%.2f x%.2f x%.2f | motor noise %.3f > %.3f",
                     cat[b].id, names[v], autotune::marginsOf(r.measured.data[0], r.base[0]).crossoverHz, p.axis[0].margins.crossoverHz,
                     autotune::marginsOf(r.measured.data[2], r.base[2]).crossoverHz, p.axis[2].margins.crossoverHz, p.dtermLpfHz,
                     r.before[0].t90 * 1e3, r.before[0].overshoot * 100, r.after[0].t90 * 1e3, r.after[0].overshoot * 100,
                     r.before[1].t90 * 1e3, r.before[1].overshoot * 100, r.after[1].t90 * 1e3, r.after[1].overshoot * 100,
                     r.before[2].overshoot * 100, r.after[2].overshoot * 100, r.after[0].error / r.before[0].error,
                     r.after[1].error / r.before[1].error, r.after[2].error / r.before[2].error, r.hoverNoiseBefore, r.hoverNoiseAfter);
            TEST_MESSAGE(line);
            TEST_ASSERT_TRUE_MESSAGE(r.measured.ok && p.found && r.flew, line);
            for (int a = 0; a < 3; ++a)
            {
                const autotune::AxisResult& x = p.axis[a];
                char why[640];
                snprintf(why, sizeof(why),
                         "axis %d: model pm %.1f gm %.1f ms %.1f, ANGLE pm %.1f gm %.1f ms %.1f, noise %.4f of %.4f; step %.0f ms %.0f%% err %.1f > %.0f ms %.0f%% err %.1f | %s",
                         a, x.margins.phaseMarginDeg, x.margins.gainMarginDb, x.margins.sensitivityDb, x.angleMargins.phaseMarginDeg,
                         x.angleMargins.gainMarginDb, x.angleMargins.sensitivityDb, x.noise, x.noiseLimit, r.before[a].t90 * 1e3,
                         r.before[a].overshoot * 100, r.before[a].error * 1e3, r.after[a].t90 * 1e3, r.after[a].overshoot * 100,
                         r.after[a].error * 1e3, line);
                // запасы — по измеренной модели
                TEST_ASSERT_TRUE_MESSAGE(x.margins.ok(lim.phaseMarginDeg, lim.gainMarginDb, lim.sensitivityDb + 0.05), why);
                TEST_ASSERT_TRUE_MESSAGE(x.angleMargins.ok(lim.phaseMarginDeg - 1.0, lim.gainMarginDb - 0.2, lim.sensitivityDb + 0.2), why);
                TEST_ASSERT_TRUE_MESSAGE(x.noise <= x.noiseLimit * 1.001, why);
                // переходная на модели коптера: не хуже, чем была
                TEST_ASSERT_TRUE_MESSAGE(r.before[a].flying && r.after[a].flying, why);
                TEST_ASSERT_TRUE_MESSAGE(r.after[a].error <= r.before[a].error * 1.05, why);
                if (a < 2)
                {
                    // время до 90 %: на модели коптера — не больше чем на четверть
                    // дольше (гексы с тяжёлыми винтами: +5–10 мс при меньшем
                    // перелёте). Не спрашивается там, где прежняя настройка шумит
                    // на моторах сильнее потолка (6 % хода): новая нарочно тише
                    if (p.noiseBefore[a] <= lim.noiseCeiling)
                        TEST_ASSERT_TRUE_MESSAGE(r.after[a].t90 > 0 && r.after[a].t90 <= r.before[a].t90 * 1.25 + 0.003, why);
                    TEST_ASSERT_TRUE_MESSAGE(r.after[a].overshoot <= std::fmax(r.before[a].overshoot, 0.08) + 0.03, why);
                }
                else
                {
                    TEST_ASSERT_TRUE_MESSAGE(r.after[a].overshoot <= std::fmax(r.before[a].overshoot, 0.15), why);
                }
                if (recheck)
                {
                    char flown[760];
                    snprintf(flown, sizeof(flown), "as flown: crossover %.1f Hz, pm %.1f, gm %.1f, ms %.1f | %s", r.flown[a].crossoverHz,
                             r.flown[a].phaseMarginDeg, r.flown[a].gainMarginDb, r.flown[a].sensitivityDb, why);
                    TEST_ASSERT_TRUE_MESSAGE(r.reflown, flown);
                    // запасы — как летает: повторный свип, без модели
                    TEST_ASSERT_TRUE_MESSAGE(r.flown[a].phaseMarginDeg >= 40.0, flown);
                    TEST_ASSERT_TRUE_MESSAGE(r.flown[a].gainMarginDb >= 5.0, flown);
                    TEST_ASSERT_TRUE_MESSAGE(r.flown[a].sensitivityDb <= 7.5, flown);
                }
            }
        }
    }
}
