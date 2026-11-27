// ============================================================
// Замкнутые полёты мультикоптера: настоящий CopterController и модель
// коптера с 6 степенями свободы (helpers/QuadSim.h, QuadHarness.h).
//
// Каждая сборка из каталога (вуп 65 мм … гекса с винтами 15") летает
// на настройке, выведенной из её описания, — и на модели, которая
// нарочно не совпадает с описанием:
//   • «вялая»: винт тянет на 20 % меньше, мотор и рама на 30 % инерционнее,
//     коптер на 20 % тяжелее, центр тяжести сдвинут вперёд;
//   • «резвая»: тяга +20 %, инерция −30 %, масса −15 %, центр тяжести
//     сдвинут вбок — усиление контура почти вдвое выше расчётного.
// Моторы при этом разбросаны между собой на ±5 %, батарея проседает,
// гироскоп шумит и видит вибрацию моторов.
//
// Проверяется не «какую команду выдал», а что сделал коптер: взлетел,
// висит на высоте ровно, наклоняется и выравнивается по стику, держит
// угловую скорость в акро, сел и выключил моторы при потере связи.
//
// Запуск: pio test -e native -f native/test_copter_sim
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "copter/Params.h"
#include "helpers/QuadHarness.h"
#include "helpers/QuadVariants.h"

using namespace copter;

void setUp() {}
void tearDown() {}

// test_attitude.cpp: фильтр ориентации (MEKF против Махони), посадка на склон
void test_attitude_ekf_learns_gyro_bias();
void test_attitude_ekf_against_mahony_in_manoeuvres();
void test_attitude_ekf_learns_rotor_drag_in_flight();
void test_attitude_ekf_consistency_nees_nis();
void test_attitude_ekf_compass_yaw_bias_and_interference_in_flight();
void test_failsafe_landing_on_a_slope_disarms();
// test_ident.cpp: chirp, идентификация и автонастройка (168, 169)
void test_chirp_runs_by_switch_in_flight_only();
void test_chirp_identifies_the_plant_within_10_percent();
void test_autotune_by_chirp_on_every_build();

namespace
{
    const char* label(const BuildSpec& b, const char* variant)
    {
        static char text[64];
        snprintf(text, sizeof(text), "%s / %s", b.id, variant);
        return text;
    }

    // Взлёт на 5 м в ALT_HOLD, 4 с висения: высота, горизонт, «горячие» моторы.
    void checkHover(const BuildSpec& b, const QuadPerturb& p, const char* variant, uint32_t seed)
    {
        QuadHarness h(b, p, seed);
        TEST_ASSERT_TRUE_MESSAGE(h.takeoffTo(5.0), label(b, variant));
        h.stats.reset();
        double thrustSum = 0;
        for (int i = 0; i < 4000; ++i)
        {
            h.step();
            thrustSum += h.ctl.getStatus().throttle;
        }
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.7, 5.0, h.stats.minHeight, label(b, variant));
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.7, 5.0, h.stats.maxHeight, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.stats.maxTilt < 3.0, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.stats.rateRms() * 57.3 < 3.0, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.stats.motorNoiseRms() < 0.15, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.stats.saturated < h.stats.samples / 100, label(b, variant));
        // газ висения выучен (постоянная времени 3 с): тот, на котором коптер
        // на самом деле висит, даже если модель ошиблась вдвое
        h.run(6.0);
        const double actual = thrustSum / 4000.0;
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15 * actual, actual, h.ctl.getStatus().hoverLearned, label(b, variant));
    }

    // Потеря связи на 10 м: выравнивание, снижение не быстрее ~1 м/с, касание — DISARM.
    void checkFailsafe(const BuildSpec& b, const QuadPerturb& p, const char* variant, uint32_t seed)
    {
        QuadHarness h(b, p, seed);
        TEST_ASSERT_TRUE_MESSAGE(h.takeoffTo(10.0), label(b, variant));
        // обычный взлёт выключил проверку runaway takeoff (и не сработал на ней)
        TEST_ASSERT_FALSE_MESSAGE(h.ctl.getStatus().runawayCheck, label(b, variant));
        h.rc.linkOk = false;
        const double t0 = h.sim.now();
        double maxDescent = 0;
        while (h.ctl.isArmed() && h.sim.now() - t0 < 45.0)
        {
            h.step();
            maxDescent = std::fmax(maxDescent, -h.sim.climbRate());
        }
        // моторы стоят уже в цикле, где сработал DISARM
        TEST_ASSERT_FALSE_MESSAGE(h.ctl.isArmed(), label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.sim.height() < 0.05, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(maxDescent < 1.6, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.stats.maxTilt < 15.0, label(b, variant));
        TEST_ASSERT_FALSE_MESSAGE(h.out.spinning, label(b, variant));
    }

    // ANGLE: полный стик по крену 0.7 с — наклон до предела, отпустил — ровно.
    void checkAngle(const BuildSpec& b, const QuadPerturb& p, const char* variant, uint32_t seed)
    {
        QuadHarness h(b, p, seed);
        TEST_ASSERT_TRUE_MESSAGE(h.takeoffTo(5.0), label(b, variant));
        const double maxAngle = h.tune.maxAngleDeg;
        h.rc.roll = 1.0f;
        const double t0 = h.sim.now();
        double reach = -1, peak = 0;
        for (int i = 0; i < 700; ++i)
        {
            h.step();
            peak = std::fmax(peak, h.sim.rollDeg());
            if (reach < 0 && h.sim.rollDeg() > 0.9 * maxAngle) reach = h.sim.now() - t0;
        }
        h.rc.roll = 0.0f;
        const double t1 = h.sim.now();
        double back = -1;
        for (int i = 0; i < 1500 && back < 0; ++i)
        {
            h.step();
            if (std::fabs(h.sim.rollDeg()) < 3.0) back = h.sim.now() - t1;
        }
        // грузовые наклоняются нарочно мягко
        const double reachLimit = b.style == FlyStyle::HEAVY_LIFT ? 0.45 : 0.35;
        TEST_ASSERT_TRUE_MESSAGE(reach > 0 && reach < reachLimit, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(peak < maxAngle * 1.15, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(back > 0 && back < 1.5, label(b, variant));
    }

    // ACRO: полный стик по крену — угловая скорость выходит на расход и держится.
    void checkAcro(const BuildSpec& b, const QuadPerturb& p, const char* variant, uint32_t seed)
    {
        QuadHarness h(b, p, seed);
        TEST_ASSERT_TRUE_MESSAGE(h.takeoffTo(20.0), label(b, variant));
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(0.3);
        h.rc.roll = 1.0f;
        const double target = h.tune.rates[0].apply(1.0f);
        double maxRate = 0, t90 = -1, sumErr = 0;
        int n = 0;
        const double t0 = h.sim.now();
        for (int i = 0; i < 400; ++i)
        {
            h.step();
            const double w = h.sim.rates().x * 57.29578;
            maxRate = std::fmax(maxRate, w);
            if (t90 < 0 && w > 0.9 * target) t90 = h.sim.now() - t0;
            if (i > 200)
            {
                sumErr += std::fabs(w - target);
                ++n;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(t90 > 0 && t90 < 0.2, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(maxRate < target * 1.15, label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(sumErr / n < 0.08 * target, label(b, variant));
        // стик отпустил, ANGLE — встаёт ровно, моторы работают
        h.rc.roll = 0.0f;
        h.rc.mode = CopterMode::ANGLE;
        h.run(1.5);
        TEST_ASSERT_TRUE_MESSAGE(h.ctl.isArmed(), label(b, variant));
        TEST_ASSERT_TRUE_MESSAGE(h.sim.tiltDeg() < 8.0, label(b, variant));
    }

    template <typename F>
    void forEachBuild(F check)
    {
        uint8_t n = 0;
        const BuildSpec* cat = buildCatalog(n);
        for (uint8_t i = 0; i < n; ++i) check(cat[i]);
    }
}

// ---------------- весь каталог ----------------

void test_every_build_hovers_on_its_derived_tune()
{
    forEachBuild([](const BuildSpec& b) { checkHover(b, QuadPerturb(), "как в описании", 3); });
}

void test_every_build_hovers_when_physics_differs_from_the_description()
{
    forEachBuild([](const BuildSpec& b) {
        checkHover(b, sluggish(), "вялая", 4);
        checkHover(b, twitchy(), "резвая", 5);
    });
}

void test_every_build_lands_and_disarms_on_link_loss()
{
    forEachBuild([](const BuildSpec& b) {
        checkFailsafe(b, QuadPerturb(), "как в описании", 6);
        checkFailsafe(b, sluggish(), "вялая", 7);
        checkFailsafe(b, twitchy(), "резвая", 8);
    });
}

void test_every_build_follows_angle_stick()
{
    forEachBuild([](const BuildSpec& b) {
        checkAngle(b, QuadPerturb(), "как в описании", 9);
        checkAngle(b, sluggish(), "вялая", 10);
        checkAngle(b, twitchy(), "резвая", 11);
    });
}

void test_every_build_tracks_acro_rate()
{
    forEachBuild([](const BuildSpec& b) {
        checkAcro(b, QuadPerturb(), "как в описании", 12);
        checkAcro(b, sluggish(), "вялая", 13);
        checkAcro(b, twitchy(), "резвая", 14);
    });
}

// ---------------- отдельные режимы и защиты (5") ----------------

static const BuildSpec& five() { return *findBuild("freestyle5"); }

// Airmode: газ в ноль на спуске — коптер всё равно крутится по стику.
void test_airmode_keeps_control_at_zero_throttle()
{
    QuadHarness h(five(), QuadPerturb(), 21);
    TEST_ASSERT_TRUE(h.takeoffTo(30.0));
    h.rc.mode = CopterMode::ACRO;
    h.rc.throttle = 0.0f;
    h.run(0.3);
    h.rc.roll = 0.5f;
    const double target = h.tune.rates[0].apply(0.5f);
    double maxRate = 0;
    for (int i = 0; i < 300; ++i)
    {
        h.step();
        maxRate = std::fmax(maxRate, h.sim.rates().x * 57.29578);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1 * target, target, maxRate);
}

// 039: EZLANDING. Посадка в ACRO на сброшенном газе; удар о землю —
// гироскоп на 40 мс «видит» рывок по крену, ACRO-PID разводит моторы, и
// airmode поднимает газ. С EZLANDING моторы расходятся меньше и коптер не
// задирает одну сторону, как с LEGACY (там ~19°).
static const double KICK_DPS = 300.0;
static void ezLandingRun(copter::MixerType type, double& bounce, double& tilt, float& spread)
{
    copter::CopterSettings cs;
    cs.mixerType = type;
    QuadHarness h(five(), QuadPerturb(), 23, copter::EscProtocol::DSHOT600, 1000.0f, cs);
    TEST_ASSERT_TRUE(h.takeoffTo(1.5));
    h.rc.mode = CopterMode::ACRO;
    h.rc.throttle = 0.1f;
    for (int i = 0; i < 10000 && !h.sim.grounded(); ++i) h.step();
    TEST_ASSERT_TRUE(h.sim.grounded());
    h.rc.throttle = 0.0f;
    bounce = 0.0;
    tilt = 0.0;
    spread = 0.0f;
    for (int i = 0; i < 1500; ++i)
    {
        h.gyroBiasDps[0] = i < 40 ? KICK_DPS : 0.0;
        h.step();
        bounce = std::fmax(bounce, h.sim.height());
        tilt = std::fmax(tilt, h.sim.tiltDeg());
        float lo = 1e9f, hi = -1e9f;
        for (uint8_t m = 0; m < h.out.count; ++m)
        {
            lo = std::fmin(lo, h.out.motor[m]);
            hi = std::fmax(hi, h.out.motor[m]);
        }
        if (h.out.count) spread = std::fmax(spread, hi - lo);
    }
}

void test_ez_landing_touchdown_without_bounce()
{
    double bounceLegacy = 0, bounceEz = 0, tiltLegacy = 0, tiltEz = 0;
    float spreadLegacy = 0, spreadEz = 0;
    ezLandingRun(copter::MixerType::LEGACY, bounceLegacy, tiltLegacy, spreadLegacy);
    ezLandingRun(copter::MixerType::EZLANDING, bounceEz, tiltEz, spreadEz);
    char msg[160];
    snprintf(msg, sizeof(msg), "EZ landing: bounce LEGACY %.3f m EZ %.3f m, tilt %.1f / %.1f deg, motor spread %.3f / %.3f",
             bounceLegacy, bounceEz, tiltLegacy, tiltEz, spreadLegacy, spreadEz);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(bounceEz <= bounceLegacy + 0.005, msg);
    TEST_ASSERT_TRUE_MESSAGE(tiltEz < 3.0, msg);                 // рама осталась на земле ровно
    TEST_ASSERT_TRUE_MESSAGE(tiltLegacy > tiltEz + 5.0, msg);    // а без EZ одна сторона приподнялась
    TEST_ASSERT_TRUE_MESSAGE(spreadEz < 0.6f * spreadLegacy, msg);
}

// 037: launch control. ARM в ACRO с режимом LAUNCH CONTROL, стик тангажа
// вперёд — рама встаёт на передние ноги до launch_angle_limit и стоит так и
// с отпущенным стиком (держит I); рывок газом — старт уже с наклоном: коптер
// сразу разгоняется вперёд и не кувыркается. Без режима рама на земле ровная.
struct LaunchRun
{
    double groundPitch = 0, heldPitch = 0, maxTilt = 0, height = 0, forward = 0, estPitch = 0;
    double groundLift = 0;   // выше всего до рывка, м
    LaunchState before = LaunchState::OFF, after = LaunchState::OFF;
};
static LaunchRun launchRun(bool mode)
{
    QuadHarness h(five(), QuadPerturb(), 29);
    h.sim.legPivot = true;
    CopterTune t = h.tune;
    t.launch.angleLimitDeg = 30.0f;
    h.useTune(t);
    h.rc.launchControl = mode;
    LaunchRun r;
    if (!h.armOnGround(CopterMode::ACRO)) return r;
    r.before = h.ctl.getStatus().launch;
    h.rc.pitch = -0.5f;   // стик от себя
    for (int i = 0; i < 3000; ++i)
    {
        h.step();
        r.groundLift = std::fmax(r.groundLift, h.sim.height());
    }
    r.groundPitch = h.sim.pitchDeg();
    r.estPitch = h.ctl.getStatus().pitch;
    h.rc.pitch = 0.0f;
    for (int i = 0; i < 1000; ++i)
    {
        h.step();
        r.groundLift = std::fmax(r.groundLift, h.sim.height());
    }
    r.heldPitch = h.sim.pitchDeg();
    h.rc.throttle = 0.8f;
    for (int i = 0; i < 600; ++i)
    {
        h.step();
        r.maxTilt = std::fmax(r.maxTilt, h.sim.tiltDeg());
    }
    r.after = h.ctl.getStatus().launch;
    r.height = h.sim.height();
    const auto p = h.sim.position();
    r.forward = std::sqrt(p.x * p.x + p.y * p.y);
    return r;
}

void test_launch_control_holds_tilt_then_launches()
{
    const LaunchRun lc = launchRun(true);
    const LaunchRun plain = launchRun(false);
    char msg[220];
    snprintf(msg, sizeof(msg),
             "launch: ground pitch %.1f (est %.1f) held %.1f, max tilt %.1f, lift %.3f, h %.2f m, forward %.2f m; without: pitch %.1f, forward %.2f m",
             lc.groundPitch, lc.estPitch, lc.heldPitch, lc.maxTilt, lc.groundLift, lc.height, lc.forward, plain.groundPitch, plain.forward);
    TEST_MESSAGE(msg);
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(LaunchState::ACTIVE), static_cast<int>(lc.before), msg);
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(LaunchState::TRIGGERED), static_cast<int>(lc.after), msg);
    TEST_ASSERT_TRUE_MESSAGE(lc.groundLift < 0.02, msg);                       // до рывка не взлетел
    TEST_ASSERT_TRUE_MESSAGE(lc.groundPitch < -24.0 && lc.groundPitch > -33.0, msg);   // предел 30°
    TEST_ASSERT_TRUE_MESSAGE(lc.heldPitch < -20.0, msg);                       // стик отпущен — стоит
    TEST_ASSERT_TRUE_MESSAGE(lc.maxTilt < 60.0, msg);                          // без кувырка
    TEST_ASSERT_TRUE_MESSAGE(lc.height > 0.3, msg);
    TEST_ASSERT_TRUE_MESSAGE(lc.forward > plain.forward + 0.3, msg);           // разгон сразу
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(plain.groundPitch) < 3.0, msg);
}

// 098: airmode_start_throttle_percent — на земле после ARM газ внизу, стик
// крена в сторону: без airmode моторы на холостых одинаково (коптер не
// прыгает); газ хоть раз выше 25 % — airmode работает и на нуле газа.
void test_airmode_starts_after_throttle_threshold()
{
    QuadHarness h(five(), QuadPerturb(), 22);
    TEST_ASSERT_TRUE(h.armOnGround(CopterMode::ACRO));
    h.rc.roll = 0.5f;
    h.run(0.1);
    auto spread = [&h]() {
        float lo = 1e9f, hi = -1e9f;
        for (uint8_t i = 0; i < h.out.count; ++i)
        {
            lo = std::fmin(lo, h.out.motor[i]);
            hi = std::fmax(hi, h.out.motor[i]);
        }
        return hi - lo;
    };
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, spread());
    h.rc.throttle = 0.3f;
    h.step();
    h.rc.throttle = 0.0f;
    h.run(0.02);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(spread() > 0.01f);
}

void test_yaw_rate_follows_stick_and_stays_level()
{
    QuadHarness h(five(), QuadPerturb(), 22);
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.rc.yaw = 0.5f;
    const double target = h.tune.rates[2].apply(0.5f);
    double sum = 0;
    int n = 0;
    for (int i = 0; i < 1500; ++i)
    {
        h.step();
        if (i > 500)
        {
            sum += h.sim.rates().z * 57.29578;
            ++n;
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(0.08 * target, target, sum / n);
    TEST_ASSERT_TRUE(h.sim.tiltDeg() < 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 5.0, h.sim.height());
}

// Ветер 4.5 м/с с порывами: висит, наклоняясь против ветра.
void test_hover_in_gusty_wind()
{
    QuadHarness h(five(), QuadPerturb(), 23);
    h.sim.setWind(4.0, 2.0, 3.0);
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.stats.reset();
    h.run(10.0);
    TEST_ASSERT_TRUE(h.stats.maxTilt < 12.0);
    // без GPS горизонт в порывах уходит на несколько градусов (P32), а с ним
    // и оценка высоты — до полуметра
    TEST_ASSERT_TRUE(h.stats.minHeight > 4.2 && h.stats.maxHeight < 5.8);
}

// Без барометра снижение при потере связи идёт по акселерометру.
void test_failsafe_without_barometer()
{
    QuadHarness h(five(), QuadPerturb(), 24);
    h.baroOk = false;
    TEST_ASSERT_TRUE(h.armOnGround(CopterMode::ALT_HOLD));
    h.rc.throttle = 0.9f;
    while (h.sim.height() < 10.0) h.step();
    h.rc.throttle = 0.5f;
    h.run(2.0);
    h.rc.linkOk = false;
    const double t0 = h.sim.now();
    double maxDescent = 0;
    while (h.ctl.isArmed() && h.sim.now() - t0 < 80.0)
    {
        h.step();
        maxDescent = std::fmax(maxDescent, -h.sim.climbRate());
    }
    TEST_ASSERT_FALSE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(h.sim.height() < 0.05);
    TEST_ASSERT_TRUE(maxDescent < 1.5);
}

// Обычный PWM-регулятор (самолётный ESC): контур уже, но летает.
void test_pwm_esc_flies()
{
    QuadHarness h(five(), QuadPerturb(), 25, EscProtocol::PWM);
    TEST_ASSERT_TRUE(h.tune.crossoverHzRP < deriveTune(five(), 1000.0f, EscProtocol::DSHOT600).crossoverHzRP);
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.stats.reset();
    h.run(3.0);
    TEST_ASSERT_TRUE(h.stats.maxTilt < 2.0);
    TEST_ASSERT_TRUE(h.stats.motorNoiseRms() < 0.05);
}

// Вуп на цикле 4 кГц: вибрация не заворачивается, моторы спокойнее, чем на 1 кГц.
void test_whoop_on_fast_loop()
{
    const BuildSpec& w = *findBuild("whoop65");
    QuadHarness slow(w, QuadPerturb(), 26);
    QuadHarness fast(w, QuadPerturb(), 26, EscProtocol::DSHOT600, 4000.0f);
    TEST_ASSERT_FALSE(fast.tune.warnings & WARN_VIBRATION_ALIAS);
    TEST_ASSERT_TRUE(slow.takeoffTo(3.0));
    TEST_ASSERT_TRUE(fast.takeoffTo(3.0));
    slow.stats.reset();
    fast.stats.reset();
    slow.run(2.0);
    fast.run(2.0);
    TEST_ASSERT_TRUE(fast.stats.motorNoiseRms() < slow.stats.motorNoiseRms());
    TEST_ASSERT_TRUE(fast.stats.maxTilt < 2.0);
}

// Провал связи на 0.2 с — ничего не происходит; на 2 с — failsafe;
// связь вернулась — управление снова у пилота.
void test_link_glitch_and_recovery_in_flight()
{
    QuadHarness h(five(), QuadPerturb(), 27);
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    h.rc.linkOk = false;
    h.run(0.2);
    h.rc.linkOk = true;
    h.run(0.5);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
    h.rc.linkOk = false;
    h.run(2.0);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    TEST_ASSERT_TRUE(h.sim.height() < 10.0);
    h.rc.linkOk = true;
    h.run(0.3);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    h.run(0.3);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
    // и тумблер снова работает как аварийный
    h.rc.armSwitch = false;
    h.step();
    TEST_ASSERT_FALSE(h.ctl.isArmed());
}

// Флип в землю: лёг на спину — моторы стоп, ARM — только после выключения тумблера.
void test_upside_down_crash_disarms()
{
    QuadHarness h(five(), QuadPerturb(), 28);
    TEST_ASSERT_TRUE(h.takeoffTo(3.0));
    h.rc.mode = CopterMode::ACRO;
    h.rc.throttle = 0.0f;
    h.rc.roll = 1.0f;
    h.run(0.25);
    h.rc.roll = 0.0f;
    const double t0 = h.sim.now();
    while (h.ctl.isArmed() && h.sim.now() - t0 < 5.0) h.step();
    TEST_ASSERT_FALSE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(h.sim.now() - t0 < 2.5);
    TEST_ASSERT_TRUE(h.ctl.getStatus().crashed);
    h.step();
    TEST_ASSERT_TRUE(h.ctl.getStatus().armBlockers & ARM_BLOCK_CRASH);
    TEST_ASSERT_FALSE(h.out.spinning);
    h.rc.armSwitch = false;
    h.step();
    TEST_ASSERT_FALSE(h.ctl.getStatus().crashed);
}

// 086: лежит на спине — turtle mode переворачивает обратно: крен (правые
// моторы, рама встаёт через левое ребро), тангаж от себя (передние), крен
// влево. Без стика моторы стоят и коптер лежит; после — обычный ARM.
void test_turtle_mode_flips_back_over()
{
    CopterSettings set;
    set.turtleAvailable = true;
    const float sticks[3][2] = { { 1.0f, 0.0f }, { 0.0f, -1.0f }, { -1.0f, 0.0f } };
    for (int k = 0; k < 3; ++k)
    {
        QuadHarness h(five(), QuadPerturb(), 40 + k, EscProtocol::DSHOT600, 1000.0f, set);
        h.sim.setAttitude(180, 0, 30);
        h.rc.mode = CopterMode::ANGLE;
        h.rc.throttle = 0.0f;
        h.run(2.5);
        TEST_ASSERT_TRUE(h.sim.tiltDeg() > 170.0);
        h.rc.armSwitch = true;
        h.run(0.05);
        TEST_ASSERT_FALSE(h.ctl.isArmed());   // на спине без режима — нет
        h.rc.armSwitch = false;
        h.rc.flipOverAfterCrash = true;
        h.run(0.05);
        h.rc.armSwitch = true;
        h.run(0.05);
        TEST_ASSERT_TRUE(h.ctl.getStatus().turtle);
        h.run(1.5);   // стик в центре — лежит, моторы стоят
        TEST_ASSERT_TRUE(h.sim.tiltDeg() > 170.0);
        TEST_ASSERT_TRUE(h.ctl.isArmed());
        h.rc.roll = sticks[k][0];
        h.rc.pitch = sticks[k][1];
        const double t0 = h.sim.now();
        while (h.sim.tiltDeg() > 60.0 && h.sim.now() - t0 < 3.0) h.step();
        char msg[64];
        snprintf(msg, sizeof(msg), "стик %d: переворот за %.2f с", k, h.sim.now() - t0);
        TEST_ASSERT_TRUE_MESSAGE(h.sim.tiltDeg() < 10.0, msg);
        TEST_ASSERT_TRUE_MESSAGE(h.sim.now() - t0 < 1.5, msg);
        h.rc.roll = h.rc.pitch = 0.0f;
        h.rc.armSwitch = false;
        h.rc.flipOverAfterCrash = false;
        h.run(3.0);
        h.rc.armSwitch = true;
        h.run(0.05);
        TEST_ASSERT_TRUE_MESSAGE(h.ctl.isArmed(), msg);
        TEST_ASSERT_FALSE(h.ctl.getStatus().turtle);
        TEST_ASSERT_FALSE(h.out.reversed);
    }
}

// Отказал мотор: коптер не держит горизонт (крутится и валится набок) —
// через 2 с моторы стоп, а не молотят винтами по земле после падения.
void test_motor_failure_in_angle_mode_disarms()
{
    QuadHarness h(five(), QuadPerturb(), 29);
    TEST_ASSERT_TRUE(h.takeoffTo(20.0));
    h.sim.failMotor(1);
    const double t0 = h.sim.now();
    while (h.ctl.isArmed() && h.sim.now() - t0 < 5.0) h.step();
    TEST_ASSERT_FALSE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(h.sim.now() - t0 < 3.0);
    TEST_ASSERT_TRUE(h.ctl.getStatus().crashed);
}

// Проверка моторов: крутится только выбранный и сам останавливается.
void test_motor_test_spins_one_motor()
{
    QuadHarness h(five(), QuadPerturb(), 30);
    h.run(2.5);
    TEST_ASSERT_TRUE(h.ctl.startMotorTest(2, 0.1f, 1.0f));
    h.run(0.5);
    TEST_ASSERT_TRUE(h.sim.rotorHz(2) > 50.0);
    TEST_ASSERT_TRUE(h.sim.rotorHz(0) < 1.0);
    TEST_ASSERT_TRUE(h.sim.grounded());
    h.run(1.5);
    TEST_ASSERT_FALSE(h.ctl.motorTestActive());
    TEST_ASSERT_TRUE(h.sim.rotorHz(2) < 20.0);
}

// Напряжение батареи видно прошивке (просадка под газом).
void test_battery_sag_is_seen()
{
    QuadHarness h(five(), QuadPerturb(), 31);
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    const float hoverVolts = h.ctl.getStatus().volts;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, static_cast<float>(h.sim.batteryVolts()), hoverVolts);
    h.rc.mode = CopterMode::ANGLE;
    h.rc.throttle = 1.0f;
    h.run(1.0);
    TEST_ASSERT_TRUE(h.ctl.getStatus().volts < hoverVolts - 1.0f);
}

// ---------------- тонкости PID (PID-профиль, пункты 011–015) ----------------

namespace
{
    struct Flight
    {
        bool ok = false;
        double noise = 0, rateRmsDps = 0;          // висение
        double t90 = -1, overshoot = 0, err = 0;   // полный стик по крену в ACRO
    };

    // Висение 4 с на 5 м с настройкой t (nullptr — выведенная из описания).
    Flight hoverWith(const BuildSpec& b, const QuadPerturb& p, const CopterTune* t, uint32_t seed, float loopHz = 1000.0f)
    {
        Flight r;
        QuadHarness h(b, p, seed, EscProtocol::DSHOT600, loopHz);
        if (t) h.useTune(*t);
        if (!h.takeoffTo(5.0)) return r;
        h.stats.reset();
        h.run(4.0);
        r.noise = h.stats.motorNoiseRms();
        r.rateRmsDps = h.stats.rateRms() * 57.29578;
        r.ok = h.ctl.isArmed() && h.stats.maxTilt < 3.0 && r.rateRmsDps < 3.0;
        return r;
    }

    // Полный стик по крену в ACRO на 20 м: время до 90 %, перелёт, ошибка.
    Flight acroWith(const BuildSpec& b, const QuadPerturb& p, const CopterTune* t, uint32_t seed, float loopHz = 1000.0f)
    {
        Flight r;
        QuadHarness h(b, p, seed, EscProtocol::DSHOT600, loopHz);
        if (t) h.useTune(*t);
        if (!h.takeoffTo(20.0)) return r;
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(0.3);
        h.rc.roll = 1.0f;
        const double target = h.tune.rates[0].apply(1.0f);
        double maxRate = 0, sumErr = 0;
        int n = 0;
        const double t0 = h.sim.now();
        while (h.sim.now() - t0 < 0.4)
        {
            h.step();
            const double w = h.sim.rates().x * 57.29578;
            maxRate = std::fmax(maxRate, w);
            if (r.t90 < 0 && w > 0.9 * target) r.t90 = h.sim.now() - t0;
            if (h.sim.now() - t0 > 0.2)
            {
                sumErr += std::fabs(w - target);
                ++n;
            }
        }
        r.overshoot = maxRate / target - 1.0;
        r.err = sumErr / n / target;
        r.ok = h.ctl.isArmed() && r.t90 > 0 && r.t90 < 0.2 && r.err < 0.08;
        return r;
    }
}

namespace
{
    // D в покое на 30 % меньше выведенного, на резком движении — прежний.
    CopterTune withDMax(CopterTune t)
    {
        AxisGains* g[3] = { &t.roll, &t.pitch, &t.yaw };
        for (uint8_t a = 0; a < 3; ++a)
        {
            t.pid.dMax[a] = g[a]->d;
            g[a]->d *= 0.7f;
        }
        return t;
    }
}

// 011: D на резком движении (d_max) — в покое D на 30 % меньше, до прежнего
// на манёвре: на висении моторы шумят не больше, чем с постоянным D, стик
// по-прежнему держится.
void test_d_max_keeps_hover_quiet()
{
    static int failed;
    failed = 0;
    forEachBuild([](const BuildSpec& b) {
        // вуп, у которого моторы крутятся быстрее половины частоты цикла, на
        // 1 кГц видит вибрацию в полосе контура (WARN_VIBRATION_ALIAS) — D рос
        // бы от шума; ему и так нужен цикл быстрее — на нём и проверяем
        // (акро — на 1 кГц у всех: манёвр, а не шум)
        const bool alias = deriveTune(b, 1000.0f, EscProtocol::DSHOT600).warnings & WARN_VIBRATION_ALIAS;
        const float loopHz = alias ? 4000.0f : 1000.0f;
        const CopterTune dyn = withDMax(deriveTune(b, loopHz, EscProtocol::DSHOT600));
        const CopterTune dyn1k = withDMax(deriveTune(b, 1000.0f, EscProtocol::DSHOT600));
        const Flight flat = hoverWith(b, QuadPerturb(), nullptr, 21, loopHz);
        const Flight boosted = hoverWith(b, QuadPerturb(), &dyn, 21, loopHz);
        const Flight acro = acroWith(b, QuadPerturb(), &dyn1k, 22);
        char line[200];
        snprintf(line, sizeof(line), "d_max %-12s %4.0f Hz motor noise %.4f -> %.4f, rate rms %.2f -> %.2f dps | acro t90 %.3f s, overshoot %.0f%%",
                 b.id, loopHz, flat.noise, boosted.noise, flat.rateRmsDps, boosted.rateRmsDps, acro.t90, acro.overshoot * 100.0);
        const bool good = boosted.ok && boosted.noise <= flat.noise * 1.02 + 1e-4 && acro.ok && acro.overshoot < 0.15;
        TEST_MESSAGE(good ? line : (std::string("BAD ") + line).c_str());
        if (!good) ++failed;
    });
    TEST_ASSERT_EQUAL_INT(0, failed);
}

// 012: I-relax как в BF (RP, срез 15 Гц, SETPOINT и GYRO) на «вялой» гексе —
// модель ошиблась, интеграл работает больше: перелёт по крену не больше 12 %.
void test_bf_iterm_relax_on_sluggish_hexa()
{
    const char* ids[] = { "hex13", "hex15" };
    const ItermRelaxType types[] = { ItermRelaxType::SETPOINT, ItermRelaxType::GYRO };
    for (const char* id : ids)
    {
        const BuildSpec& b = *findBuild(id);
        for (ItermRelaxType type : types)
        {
            CopterTune t = deriveTune(b, 1000.0f, EscProtocol::DSHOT600);
            t.pid.relax = ItermRelax::RP;
            t.pid.relaxType = type;
            t.pid.relaxCutoffHz = 15.0f;
            const Flight r = acroWith(b, sluggish(), &t, 13);
            char line[160];
            snprintf(line, sizeof(line), "relax RP %s %-6s t90 %.3f s, overshoot %.1f%%, err %.1f%%",
                     type == ItermRelaxType::GYRO ? "GYRO" : "SETPOINT", id, r.t90, r.overshoot * 100.0, r.err * 100.0);
            TEST_MESSAGE(line);
            TEST_ASSERT_TRUE_MESSAGE(r.ok, line);
            TEST_ASSERT_TRUE_MESSAGE(r.overshoot <= 0.12, line);
        }
    }
}

// ---------------- газ и TPA (газ и TPA, пункты 016–020) ----------------

namespace
{
    CopterTune fiveTune() { return deriveTune(five(), 1000.0f, EscProtocol::DSHOT600); }

    // 5" взлетел на height и висит в ACRO на выученном газе висения.
    void hoverInAcro(QuadHarness& h, double height)
    {
        TEST_ASSERT_TRUE(h.takeoffTo(height));
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(1.0);
    }

    // Рывок газом с висения до полного на 0.4 с: на сколько градусов ушёл
    // наклон (тангаж или крен — куда сдвинут центр тяжести).
    double punchPitchDip(const QuadPerturb& p, const CopterTune& t, uint32_t seed)
    {
        QuadHarness h(five(), p, seed);
        h.useTune(t);
        hoverInAcro(h, 5.0);
        const double pitch0 = h.sim.pitchDeg(), roll0 = h.sim.rollDeg();
        h.rc.throttle = 1.0f;
        double dip = 0;
        for (int i = 0; i < 400; ++i)
        {
            h.step();
            dip = std::fmax(dip, std::fmax(std::fabs(h.sim.pitchDeg() - pitch0), std::fabs(h.sim.rollDeg() - roll0)));
        }
        return dip;
    }

    // Газ с висения +0.3: прирост вертикальной скорости через after секунд.
    double climbAfterStep(const CopterTune& t, double after, double* tilt = nullptr)
    {
        QuadHarness h(five(), QuadPerturb(), 32);
        h.useTune(t);
        hoverInAcro(h, 5.0);
        const double v0 = h.sim.climbRate();
        h.rc.throttle = h.ctl.getStatus().hoverLearned + 0.3f;
        h.stats.reset();
        h.run(after);
        if (tilt) *tilt = h.stats.maxTilt;
        return h.sim.climbRate() - v0;
    }
}

// 016–017: anti-gravity — рывок газом с висения до полного в ACRO: перекос
// тяги (центр тяжести не в центре, моторы разные) растёт с газом, I должен
// его догнать. Умолчание модели (80, 5 Гц) и BF (то же с P +50 % и
// пределами суммы PID 500/400) против выключенной и прежней (I до ×4 по
// газу через 15 Гц): на 5" как в описании нос клюёт меньше 1°, у «вялого»
// и «резвого» — меньше 4° и втрое меньше, чем без anti-gravity.
void test_anti_gravity_holds_the_nose_on_punch()
{
    CopterTune off = fiveTune();
    off.pid.antiGravityI = 0.0f;
    CopterTune old = fiveTune();
    old.pid.antiGravityI = 3.0f;
    old.pid.antiGravityHz = 15.0f;
    CopterTune bf = fiveTune();
    bf.pid.antiGravityP = 1.0f;
    bf.pid.pidSumLimit = 0.5f;
    bf.pid.pidSumLimitYaw = 0.4f;
    const QuadPerturb phys[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        const double dOff = punchPitchDip(phys[k], off, 31);
        const double dOld = punchPitchDip(phys[k], old, 31);
        const double dModel = punchPitchDip(phys[k], fiveTune(), 31);
        const double dBf = punchPitchDip(phys[k], bf, 31);
        char line[160];
        snprintf(line, sizeof(line), "anti-gravity punch 5\" %-8s: tilt off %.2f, old %.2f, model %.2f, BF %.2f deg", names[k], dOff,
                 dOld, dModel, dBf);
        TEST_MESSAGE(line);
        const double limit = k == 0 ? 1.0 : 4.0;
        TEST_ASSERT_TRUE_MESSAGE(dModel < limit && dBf < limit, line);
        if (k > 0) TEST_ASSERT_TRUE_MESSAGE(dModel < dOff / 3.0 && dModel < dOld, line);
    }
}

// 018: throttle_boost — газ с висения +0.3: через 50 мс коптер набирает
// высоту быстрее, наклон тот же.
void test_throttle_boost_speeds_up_climb()
{
    CopterTune bf = fiveTune();
    bf.pid.throttleBoost = 0.5f;   // BF: 5
    CopterTune strong = fiveTune();
    strong.pid.throttleBoost = 1.0f;
    double tiltOff = 0, tiltBf = 0, tiltStrong = 0;
    const double vOff = climbAfterStep(fiveTune(), 0.05, &tiltOff);
    const double vBf = climbAfterStep(bf, 0.05, &tiltBf);
    const double vStrong = climbAfterStep(strong, 0.05, &tiltStrong);
    char line[200];
    snprintf(line, sizeof(line), "throttle_boost: climb rate after 50 ms off %.3f, 5 %.3f, 10 %.3f m/s; tilt %.2f / %.2f / %.2f deg",
             vOff, vBf, vStrong, tiltOff, tiltBf, tiltStrong);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(vBf > vOff * 1.05 && vStrong > vBf, line);
    TEST_ASSERT_TRUE_MESSAGE(tiltBf < 3.0 && tiltStrong < 3.0, line);
}

// 019: полный газ 1 с в ACRO — без раскачки во всех режимах TPA (PD и D —
// кривая BF 65 % от 1350, MODEL — из модели) на PID из модели.
void test_full_throttle_without_oscillation_in_every_tpa_mode()
{
    const TpaMode modes[] = { TpaMode::MODEL, TpaMode::PD, TpaMode::D };
    const char* names[] = { "MODEL", "PD", "D" };
    for (uint8_t m = 0; m < 3; ++m)
    {
        CopterTune t = fiveTune();
        t.pid.tpaMode = modes[m];
        QuadHarness h(five(), QuadPerturb(), 33);
        h.useTune(t);
        hoverInAcro(h, 5.0);
        h.rc.throttle = 1.0f;
        h.run(0.3);
        h.stats.reset();
        h.run(0.7);
        const double rms = h.stats.rateRms() * 57.29578;
        char line[160];
        snprintf(line, sizeof(line), "TPA %-5s full throttle: rate rms %.2f dps, motor noise %.4f, tilt %.1f deg", names[m], rms,
                 h.stats.motorNoiseRms(), h.stats.maxTilt);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(h.ctl.isArmed() && rms < 3.0 && h.stats.motorNoiseRms() < 0.15, line);
    }
}

// 020: tpa_low_always — на нулевом газе (airmode, спуск) PD слабее на 20 %
// (как в BF) или сильнее на 50 %: коптер крутится по стику и, отпустив
// стик, не раскачивается.
void test_tpa_low_on_zero_throttle_airmode()
{
    const float lowRates[] = { 0.0f, 0.2f, -0.5f };
    for (float low : lowRates)
    {
        CopterTune t = fiveTune();
        t.pid.tpaLowRate = low;
        t.pid.tpaLowAlways = true;
        QuadHarness h(five(), QuadPerturb(), 21);
        h.useTune(t);
        TEST_ASSERT_TRUE(h.takeoffTo(30.0));
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = 0.0f;
        h.run(0.3);
        h.rc.roll = 0.5f;
        const double target = h.tune.rates[0].apply(0.5f);
        double maxRate = 0;
        for (int i = 0; i < 300; ++i)
        {
            h.step();
            maxRate = std::fmax(maxRate, h.sim.rates().x * 57.29578);
        }
        h.rc.roll = 0.0f;
        h.run(0.1);
        h.stats.reset();
        h.run(0.3);
        const double rms = h.stats.rateRms() * 57.29578;
        char line[160];
        snprintf(line, sizeof(line), "tpa_low_rate %+3.0f%% always, zero throttle: max rate %.0f of %.0f dps, then rate rms %.2f dps",
                 low * 100.0f, maxRate, target, rms);
        TEST_MESSAGE(line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1 * target, target, maxRate, line);
        TEST_ASSERT_TRUE_MESSAGE(rms < 3.0, line);
    }
}

// ---------------- дефолты Betaflight 4.5 (tune_source = BETAFLIGHT) ----------------

namespace
{
    struct BfFlight
    {
        bool tookOff = false, hovers = false, tracksRate = false;
        double rateRmsDps = 0, motorNoise = 0, maxTilt = 0;
        double t90 = -1, overshoot = 0, err = 0;
    };

    CopterTune betaflightTune(const BuildSpec& b, EscProtocol esc)
    {
        CopterParams p;
        p.begin(b, 1000.0f, esc);
        char error[96];
        p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", error, sizeof(error));
        return p.tune();
    }

    // Наш цикл 1 кГц и OneShot125, номинальная физика: висение 4 с в ALT_HOLD и
    // полный стик в ACRO — те же мерки, что у настройки из модели.
    BfFlight flyBetaflightDefaults(const BuildSpec& b)
    {
        BfFlight r;
        const CopterTune bf = betaflightTune(b, EscProtocol::ONESHOT125);
        {
            QuadHarness h(b, QuadPerturb(), 41, EscProtocol::ONESHOT125);
            h.useTune(bf);
            r.tookOff = h.takeoffTo(5.0);
            if (r.tookOff)
            {
                h.stats.reset();
                h.run(4.0);
                r.rateRmsDps = h.stats.rateRms() * 57.29578;
                r.motorNoise = h.stats.motorNoiseRms();
                r.maxTilt = h.stats.maxTilt;
                r.hovers = h.ctl.isArmed() && std::fabs(h.stats.minHeight - 5.0) < 0.7 &&
                           std::fabs(h.stats.maxHeight - 5.0) < 0.7 && r.maxTilt < 3.0 && r.rateRmsDps < 3.0 &&
                           r.motorNoise < 0.15;
            }
        }
        {
            QuadHarness h(b, QuadPerturb(), 42, EscProtocol::ONESHOT125);
            h.useTune(bf);
            if (!h.takeoffTo(20.0)) return r;
            h.rc.mode = CopterMode::ACRO;
            h.rc.throttle = h.ctl.getStatus().hoverLearned;
            h.run(0.3);
            h.rc.roll = 1.0f;
            const double target = h.tune.rates[0].apply(1.0f);
            double maxRate = 0, sumErr = 0;
            int n = 0;
            const double t0 = h.sim.now();
            for (int i = 0; i < 400; ++i)
            {
                h.step();
                const double w = h.sim.rates().x * 57.29578;
                maxRate = std::fmax(maxRate, w);
                if (r.t90 < 0 && w > 0.9 * target) r.t90 = h.sim.now() - t0;
                if (i > 200)
                {
                    sumErr += std::fabs(w - target);
                    ++n;
                }
            }
            r.overshoot = maxRate / target - 1.0;
            r.err = sumErr / n / target;
            r.tracksRate = h.ctl.isArmed() && r.t90 > 0 && r.t90 < 0.2 && r.overshoot < 0.15 && r.err < 0.08;
        }
        return r;
    }
}

// 004: дефолты BF на нашем цикле 1 кГц и OneShot125. 5" обязан летать; по
// остальным сборкам — отчёт (он же — таблица в docs/COPTER.md): дефолты BF
// подобраны под 5", на вупах и гексах они не обязаны работать.
void test_betaflight_defaults_on_our_loop()
{
    uint8_t count = 0;
    const BuildSpec* cat = buildCatalog(count);
    for (uint8_t i = 0; i < count; ++i)
    {
        const BfFlight r = flyBetaflightDefaults(cat[i]);
        char line[200];
        snprintf(line, sizeof(line), "BF45 %-12s hover %s (rate rms %.1f dps, motor noise %.3f, tilt %.1f) | acro %s (t90 %.3f s, overshoot %.0f%%, err %.1f%%)",
                 cat[i].id, r.hovers ? "OK" : (r.tookOff ? "BAD" : "NO TAKEOFF"), r.rateRmsDps, r.motorNoise, r.maxTilt,
                 r.tracksRate ? "OK" : "BAD", r.t90, r.overshoot * 100.0, r.err * 100.0);
        TEST_MESSAGE(line);
        if (strcmp(cat[i].id, "freestyle5") == 0)
        {
            TEST_ASSERT_TRUE_MESSAGE(r.hovers, line);
            TEST_ASSERT_TRUE_MESSAGE(r.tracksRate, line);
        }
    }
}

namespace
{
    struct Flick
    {
        double t90 = -1, overshoot = 0;
    };

    // ACRO, стик по крену 0 -> target за rampS, кадры приёмника frameHz (стик
    // меняется только с кадром, rc.frame считает кадры, как в прошивке).
    Flick flickOnFrames(const CopterTune& t, const QuadPerturb& p, double rampS, double frameHz, float target)
    {
        QuadHarness h(five(), p, 12);
        h.useTune(t);
        Flick r;
        if (!h.takeoffTo(20.0)) return r;
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(0.3);
        const double goal = h.tune.rates[0].apply(target);
        const double t0 = h.sim.now();
        double next = t0, maxW = 0;
        for (int i = 0; i < 500; ++i)
        {
            const double now = h.sim.now();
            if (now >= next)
            {
                next += 1.0 / frameHz;
                ++h.rc.frame;
                h.rc.roll = static_cast<float>(std::fmin(1.0, (now - t0) / rampS) * target);
            }
            h.step();
            const double w = h.sim.rates().x * 57.29578;
            maxW = std::fmax(maxW, w);
            if (r.t90 < 0 && w > 0.9 * goal) r.t90 = h.sim.now() - t0;
        }
        r.overshoot = maxW / goal - 1.0;
        return r;
    }
}

// 021–025: дефолты BF (smooth_factor 25, jitter_factor 7, boost 15) на
// флике полстика за 20 мс с кадрами 250 Гц — перелёт не больше, чем со
// старым FF без этих настроек, выход на расход не позже.
void test_feedforward_settings_on_a_real_flick()
{
    const CopterTune bf = betaflightTune(five(), EscProtocol::DSHOT600);
    CopterTune plain = bf;
    plain.pid.ffSmoothFactor = 0.0f;
    plain.pid.ffJitterFactor = 0.0f;
    plain.pid.ffBoost = 0.0f;
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        const Flick on = flickOnFrames(bf, ps[k], 0.02, 250.0, 0.5f);
        const Flick off = flickOnFrames(plain, ps[k], 0.02, 250.0, 0.5f);
        char line[200];
        snprintf(line, sizeof(line), "feedforward BF flick %-8s: t90 %.0f ms / %.0f ms, overshoot %.1f%% / %.1f%% (with / without ff settings)",
                 names[k], on.t90 * 1000.0, off.t90 * 1000.0, on.overshoot * 100.0, off.overshoot * 100.0);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(on.t90 > 0 && off.t90 > 0, line);
        TEST_ASSERT_TRUE_MESSAGE(on.t90 < off.t90 + 0.003, line);
        TEST_ASSERT_TRUE_MESSAGE(on.overshoot < off.overshoot + 0.01, line);
    }
}

// 026: feedforward_max_rate_limit — флип полным стиком за 20 мс (кадры
// 250 Гц) на дефолтах BF: перелёт расхода меньше 5 %; у «резвого» 5" без
// предела FF разгоняет раму мимо расхода. У «вялого» перелёт от I, а не от
// FF — предел его не ухудшает.
void test_feedforward_max_rate_limit_on_a_full_stick_flip()
{
    const CopterTune bf = betaflightTune(five(), EscProtocol::DSHOT600);
    CopterTune off = bf;
    off.pid.ffMaxRateLimit = 0.0f;
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        const Flick on = flickOnFrames(bf, ps[k], 0.02, 250.0, 1.0f);
        const Flick no = flickOnFrames(off, ps[k], 0.02, 250.0, 1.0f);
        char line[200];
        snprintf(line, sizeof(line), "ff max rate limit 90%% full flick %-8s: overshoot %.1f%% / %.1f%%, t90 %.0f / %.0f ms (with / without)",
                 names[k], on.overshoot * 100.0, no.overshoot * 100.0, on.t90 * 1000.0, no.t90 * 1000.0);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(on.t90 > 0 && on.t90 < 0.1, line);
        TEST_ASSERT_TRUE_MESSAGE(on.overshoot < no.overshoot + 0.005, line);
        if (k != 1) TEST_ASSERT_TRUE_MESSAGE(on.overshoot < 0.05, line);
        if (k == 2) TEST_ASSERT_TRUE_MESSAGE(no.overshoot > 2.0 * on.overshoot, line);
    }
}

namespace
{
    struct YawHold
    {
        bool ok = false;
        double target = 0, mean = 0, rmsErr = 0, overshoot = 0;
    };

    // ANGLE на 5 м, стик рысканья stick 1.5 с: средний расход после 0.5 с,
    // СКЗ ошибки, перелёт.
    YawHold holdYaw(const BuildSpec& b, const CopterTune& t, const QuadPerturb& p, float stick)
    {
        YawHold r;
        QuadHarness h(b, p, 22);
        h.useTune(t);
        if (!h.takeoffTo(5.0)) return r;
        h.rc.yaw = stick;
        r.target = h.tune.rates[2].apply(stick);
        double sum = 0, sq = 0, maxW = 0;
        int n = 0;
        for (int i = 0; i < 1500; ++i)
        {
            h.step();
            const double w = h.sim.rates().z * 57.29578;
            maxW = std::fmax(maxW, w);
            if (i > 500)
            {
                sum += w;
                sq += (w - r.target) * (w - r.target);
                ++n;
            }
        }
        r.mean = sum / n;
        r.rmsErr = std::sqrt(sq / n);
        r.overshoot = maxW / r.target - 1.0;
        r.ok = h.ctl.isArmed() && h.sim.tiltDeg() < 10.0;
        return r;
    }
}

// 028: integrated yaw на гексе 15" — у больших винтов рысканье даёт разгон
// винта (в модели QuadSim он есть). С числами BF рысканье держит расход
// ровнее и с меньшим перелётом, чем без интеграла; с PID из модели
// (пересчитанными) — держит расход на всех трёх физиках.
void test_integrated_yaw_on_big_hexa()
{
    const BuildSpec& hex = *findBuild("hex15");
    CopterParams bfp;
    bfp.begin(hex, 1000.0f, EscProtocol::DSHOT600);
    char err[96];
    bfp.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err));
    const CopterTune bfOff = bfp.tune();
    bfp.parse(PARAM_USE_INTEGRATED_YAW, 0, "ON", err, sizeof(err));
    const CopterTune bfOn = bfp.tune();
    const YawHold off = holdYaw(hex, bfOff, QuadPerturb(), 0.5f);
    const YawHold on = holdYaw(hex, bfOn, QuadPerturb(), 0.5f);
    char line[200];
    snprintf(line, sizeof(line), "integrated yaw hex15 BF: mean %.1f / %.1f of %.0f dps, err rms %.1f / %.1f, overshoot %.1f%% / %.1f%% (on / off)",
             on.mean, off.mean, on.target, on.rmsErr, off.rmsErr, on.overshoot * 100.0, off.overshoot * 100.0);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(on.ok && off.ok, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.03 * on.target, on.target, on.mean, line);
    TEST_ASSERT_TRUE_MESSAGE(on.rmsErr < 3.0 && on.rmsErr < 0.5 * off.rmsErr, line);
    TEST_ASSERT_TRUE_MESSAGE(on.overshoot < 0.08 && on.overshoot < off.overshoot, line);

    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        CopterParams p;
        p.begin(hex, 1000.0f, EscProtocol::DSHOT600);
        p.parse(PARAM_USE_INTEGRATED_YAW, 0, "ON", err, sizeof(err));
        const YawHold m = holdYaw(hex, p.tune(), ps[k], 0.5f);
        snprintf(line, sizeof(line), "integrated yaw hex15 MODEL %-8s: mean %.1f of %.0f dps, err rms %.1f, overshoot %.1f%%", names[k],
                 m.mean, m.target, m.rmsErr, m.overshoot * 100.0);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(m.ok, line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.04 * m.target, m.target, m.mean, line);
        TEST_ASSERT_TRUE_MESSAGE(m.rmsErr < 2.0, line);
    }
}

// 029: absolute control — ACRO, стики в центре, на раму 2 с давит момент
// (5 % управляемости по крену и рысканью: порыв на несимметричную раму,
// перекос тяги). Без absolute control угол и курс уходят, пока I догоняет,
// и не возвращаются; с abs_control_gain 10 — остаются на месте.
void test_absolute_control_holds_attitude_against_torque()
{
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        double roll[2], yaw[2], yawPeak[2];
        for (uint8_t on = 0; on < 2; ++on)
        {
            CopterTune t = fiveTune();
            t.pid.absGain = on ? 10.0f : 0.0f;
            QuadHarness h(five(), ps[k], 31);
            h.useTune(t);
            hoverInAcro(h, 10.0);
            const double r0 = h.sim.rollDeg(), y0 = h.sim.yawDeg();
            h.sim.setDisturbance(0.05 * t.kRoll, 0.0, 0.05 * t.kYaw);
            yawPeak[on] = 0;
            for (int i = 0; i < 2000; ++i)
            {
                h.step();
                yawPeak[on] = std::fmax(yawPeak[on], std::fabs(h.sim.yawDeg() - y0));
            }
            roll[on] = std::fabs(h.sim.rollDeg() - r0);
            yaw[on] = std::fabs(h.sim.yawDeg() - y0);
            TEST_ASSERT_TRUE(h.ctl.isArmed());
        }
        char line[200];
        snprintf(line, sizeof(line), "abs control 5\" %-8s: roll off by %.2f / %.2f deg, heading %.2f / %.2f deg (peak %.1f / %.1f) (gain 10 / off)",
                 names[k], roll[1], roll[0], yaw[1], yaw[0], yawPeak[1], yawPeak[0]);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(roll[1] < 0.5 && yaw[1] < 1.0, line);
        TEST_ASSERT_TRUE_MESSAGE(roll[0] > 1.0 && yaw[0] > 10.0, line);
        TEST_ASSERT_TRUE_MESSAGE(yawPeak[1] < yawPeak[0] / 3.0, line);
    }
}

// 038: crash recovery — ACRO, висит; 10 мс на раму давит момент вдвое сильнее
// всей управляемости (задели ветку, столкнулись в воздухе): кувырок до 90–175°. Без восстановления
// ACRO держит тот угол, на котором удар кончился; с crash_recovery = ON
// коптер сам выравнивается и отдаёт управление. Полный флип стиком — не удар.
void test_crash_recovery_levels_after_a_hit()
{
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        double tilt[2] = {}, peak[2] = {}, levelAt = -1;
        bool recovered[2] = {};
        for (uint8_t on = 0; on < 2; ++on)
        {
            CopterTune t = fiveTune();
            t.pid.crashRecovery = on ? CrashRecovery::ON : CrashRecovery::OFF;
            QuadHarness h(five(), ps[k], 38);
            h.useTune(t);
            hoverInAcro(h, 20.0);
            h.sim.setDisturbance(2.0 * t.kRoll, 1.0 * t.kPitch, 0.0);
            for (int i = 0; i < 10; ++i) h.step();
            h.sim.setDisturbance(0.0, 0.0, 0.0);
            for (int i = 0; i < 1500; ++i)
            {
                h.step();
                peak[on] = std::fmax(peak[on], h.sim.tiltDeg());
                recovered[on] = recovered[on] || h.ctl.getStatus().crashRecovery;
                if (on && levelAt < 0 && recovered[on] && !h.ctl.getStatus().crashRecovery) levelAt = (i + 1) * 0.001;
            }
            tilt[on] = h.sim.tiltDeg();
            TEST_ASSERT_TRUE(h.ctl.isArmed());
        }
        char line[220];
        snprintf(line, sizeof(line), "crash recovery 5\" %-8s: tilt after 1.5 s %.1f deg (off %.1f), peak %.0f / %.0f deg, back to pilot at %.2f s",
                 names[k], tilt[1], tilt[0], peak[1], peak[0], levelAt);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(recovered[1] && !recovered[0], line);
        TEST_ASSERT_TRUE_MESSAGE(tilt[1] < 10.0 && tilt[0] > 30.0, line);
        TEST_ASSERT_TRUE_MESSAGE(levelAt > 0.0 && levelAt < 0.6, line);
    }

    // флип полным стиком с crash_recovery = ON: восстановление не включается
    CopterTune t = fiveTune();
    t.pid.crashRecovery = CrashRecovery::ON;
    QuadHarness h(five(), QuadPerturb(), 38);
    h.useTune(t);
    hoverInAcro(h, 20.0);
    h.rc.roll = 1.0f;
    bool falseHit = false;
    for (int i = 0; i < 400; ++i)
    {
        h.step();
        falseHit = falseHit || h.ctl.getStatus().crashRecovery;
    }
    h.rc.roll = 0.0f;
    for (int i = 0; i < 300; ++i)
    {
        h.step();
        falseHit = falseHit || h.ctl.getStatus().crashRecovery;
    }
    TEST_ASSERT_FALSE_MESSAGE(falseHit, "flip by stick taken for a crash");

    // crash_recovery = DISARM — моторы стоп по удару
    CopterTune td = fiveTune();
    td.pid.crashRecovery = CrashRecovery::DISARM;
    QuadHarness hd(five(), QuadPerturb(), 38);
    hd.useTune(td);
    hoverInAcro(hd, 20.0);
    hd.sim.setDisturbance(3.0 * td.kRoll, 0.0, 0.0);
    for (int i = 0; i < 12; ++i) hd.step();
    hd.sim.setDisturbance(0.0, 0.0, 0.0);
    TEST_ASSERT_FALSE(hd.ctl.isArmed());
    TEST_ASSERT_TRUE(hd.ctl.getStatus().crashed);
}

// P54: тот же удар, но гироскоп с пределом чипа ±2000 °/с — в кувырке он в
// упоре и видит меньше настоящего: горизонт уходит на 11–34°. Раньше фильтр
// этого не знал и ворота 25° держали ошибку навсегда (к 1.5 с: 9–28° у MEKF,
// с GPS — 7–32°). Теперь MEKF растит неуверенность, пока гироскоп в упоре,
// и после находит горизонт: с GPS и компасом — за доли секунды, без GPS —
// по сопротивлению винтов (медленнее, 1/k ≈ 3 с). Махони горизонта в полёте
// не видит вовсе (акселерометр там меряет тягу) — только печать.
namespace
{
    // угол между настоящим и оценённым «вниз», °
    double horizonErrDeg(const QuadHarness& h)
    {
        const double d2r = M_PI / 180.0;
        const double r1 = h.sim.rollDeg() * d2r, p1 = h.sim.pitchDeg() * d2r;
        const double r2 = h.ctl.getStatus().roll * d2r, p2 = h.ctl.getStatus().pitch * d2r;
        const double a[3] = { -std::sin(p1), std::sin(r1) * std::cos(p1), std::cos(r1) * std::cos(p1) };
        const double b[3] = { -std::sin(p2), std::sin(r2) * std::cos(p2), std::cos(r2) * std::cos(p2) };
        const double c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        return std::acos(std::fmax(-1.0, std::fmin(1.0, c))) / d2r;
    }
}

void test_crash_recovery_with_a_saturated_gyro()
{
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t v = 0; v < 3; ++v)
    {
        for (uint8_t k = 0; k < 3; ++k)
        {
            CopterSettings s;
            s.attitudeEkf = v != 0;
            CopterTune t = fiveTune();
            t.pid.crashRecovery = CrashRecovery::ON;
            QuadHarness h(five(), ps[k], 38);
            h.useTune(t, s);
            h.sim.setGyroRange(2000.0);
            h.gpsOn = h.compass = v == 2;   // GPS + компас: ускорение и курс видны
            hoverInAcro(h, 20.0);
            h.sim.setDisturbance(2.0 * t.kRoll, 1.0 * t.kPitch, 0.0);
            for (int i = 0; i < 10; ++i) h.step();
            h.sim.setDisturbance(0.0, 0.0, 0.0);
            double errPeak = 0, err1 = 0;
            bool recovered = false;
            for (int i = 0; i < 1500; ++i)
            {
                h.step();
                errPeak = std::fmax(errPeak, horizonErrDeg(h));
                recovered = recovered || h.ctl.getStatus().crashRecovery;
                if (i == 999) err1 = horizonErrDeg(h);
            }
            const double err15 = horizonErrDeg(h), tilt = h.sim.tiltDeg();
            char line[220];
            snprintf(line, sizeof(line), "saturated hit %s %-8s: horizon error peak %.1f, at 1 s %.1f, at 1.5 s %.1f deg; true tilt %.1f deg",
                     v == 0 ? "Mahony    " : v == 1 ? "MEKF      " : "MEKF + GPS", names[k], errPeak, err1, err15, tilt);
            TEST_MESSAGE(line);
            TEST_ASSERT_TRUE(h.ctl.isArmed());
            TEST_ASSERT_TRUE_MESSAGE(recovered, line);
            TEST_ASSERT_TRUE_MESSAGE(errPeak > 10.0, line);   // упор правда сбил горизонт
            if (v == 0) continue;
            TEST_ASSERT_TRUE_MESSAGE(h.ctl.getStatus().gyroSaturated, line);
            TEST_ASSERT_TRUE_MESSAGE(err15 < 0.75 * errPeak, line);
            if (v == 1) TEST_ASSERT_TRUE_MESSAGE(err15 < 12.0, line);
            // sluggish: после восстановления наклонён на 36° и разгоняется —
            // ускорению GPS вера малая (опоздание фикса), ошибка уходит медленно
            if (v == 2) TEST_ASSERT_TRUE_MESSAGE(err15 < (k == 1 ? 15.0 : 3.0), line);
        }
    }
}

// Yaw spin recovery (069): рама раскручена по рысканью до 2500 °/с —
// гироскоп в упоре (2000), видит меньше настоящего. С защитой (AUTO, порог
// от рейтов) вращение гасится одним P рысканья, крен и тангаж без PID,
// после — обычное управление; коптер не разоружается и не валится.
void test_yaw_spin_recovery_stops_a_saturated_spin()
{
    double stopAt[2] = { -1, -1 }, tilt[2] = {};
    bool seen[2] = {};
    for (uint8_t on = 0; on < 2; ++on)
    {
        CopterSettings s;
        s.yawSpin = on ? YawSpinMode::AUTO : YawSpinMode::OFF;
        CopterTune t = fiveTune();
        QuadHarness h(five(), QuadPerturb(), 41);
        h.useTune(t, s);
        h.sim.setGyroRange(2000.0);   // чип в упоре
        hoverInAcro(h, 30.0);
        h.sim.setRates({ 0.0, 0.0, 2500.0 / 57.29578 });
        for (int i = 0; i < 2000; ++i)
        {
            h.step();
            seen[on] = seen[on] || h.ctl.getStatus().yawSpin;
            if (stopAt[on] < 0 && std::fabs(h.sim.rates().z) * 57.29578 < 200.0) stopAt[on] = (i + 1) * 0.001;
        }
        tilt[on] = h.sim.tiltDeg();
        TEST_ASSERT_TRUE(h.ctl.isArmed());
        TEST_ASSERT_FALSE(h.ctl.getStatus().yawSpin);
    }
    char line[200];
    snprintf(line, sizeof(line), "yaw spin 2500 deg/s: below 200 deg/s at %.3f s (off %.3f s), tilt after 2 s %.1f deg (off %.1f)",
             stopAt[1], stopAt[0], tilt[1], tilt[0]);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(seen[1] && !seen[0], line);
    TEST_ASSERT_TRUE_MESSAGE(stopAt[1] > 0.0 && stopAt[1] < 0.5, line);
    TEST_ASSERT_TRUE_MESSAGE(tilt[1] < 15.0, line);

    // флип и пируэт полным стиком рысканья на своих рейтах — защита молчит
    CopterSettings s;
    QuadHarness h(five(), QuadPerturb(), 41);
    h.useTune(fiveTune(), s);
    hoverInAcro(h, 30.0);
    h.rc.yaw = 1.0f;
    bool falseHit = false;
    for (int i = 0; i < 1500; ++i)
    {
        h.step();
        falseHit = falseHit || h.ctl.getStatus().yawSpin;
    }
    TEST_ASSERT_FALSE_MESSAGE(falseHit, "full-stick yaw pirouette taken for a spin");
}

// P56: тот же раскрут, но чип в упоре ведёт себя как настоящий — в крен и
// тангаж течёт мусор (перекрёстная утечка и наложение, доля превышения
// предела). Без защиты PID крена и тангажа гоняются за мусором: на 3500 и
// 5000 °/с раму валит на 11–18°, горизонт уезжает на 7–10°, остановка
// вдвое дольше. С защитой (крен и тангаж без PID) — наклон 0, горизонт ≤ 3°.
void test_yaw_spin_recovery_ignores_saturation_junk()
{
    const double spins[] = { 2500.0, 3500.0, 5000.0 };
    for (double spin : spins)
    {
        double stopAt[2] = { -1, -1 }, tiltPeak[2] = {}, tilt[2] = {}, drop[2] = {}, horizon[2] = {};
        bool armed[2] = {};
        for (uint8_t on = 0; on < 2; ++on)
        {
            CopterSettings s;
            s.yawSpin = on ? YawSpinMode::AUTO : YawSpinMode::OFF;
            QuadHarness h(five(), QuadPerturb(), 41);
            h.useTune(fiveTune(), s);
            h.sim.setGyroRange(2000.0);
            h.sim.setGyroJunk(0.3);
            hoverInAcro(h, 30.0);
            const double h0 = h.sim.height();
            h.sim.setRates({ 0.0, 0.0, spin / 57.29578 });
            for (int i = 0; i < 2000; ++i)
            {
                h.step();
                tiltPeak[on] = std::fmax(tiltPeak[on], h.sim.tiltDeg());
                drop[on] = std::fmax(drop[on], h0 - h.sim.height());
                if (stopAt[on] < 0 && std::fabs(h.sim.rates().z) * 57.29578 < 200.0) stopAt[on] = (i + 1) * 0.001;
            }
            tilt[on] = h.sim.tiltDeg();
            horizon[on] = horizonErrDeg(h);
            armed[on] = h.ctl.isArmed();
        }
        char line[300];
        snprintf(line, sizeof(line),
                 "yaw spin %.0f + junk: below 200 deg/s at %.3f s (off %.3f), tilt peak %.0f (off %.0f), after 2 s %.1f (off %.1f), "
                 "height lost %.1f m (off %.1f), horizon error %.1f (off %.1f) deg, armed %d (off %d)",
                 spin, stopAt[1], stopAt[0], tiltPeak[1], tiltPeak[0], tilt[1], tilt[0], drop[1], drop[0], horizon[1], horizon[0],
                 armed[1], armed[0]);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(armed[0] && armed[1], line);
        TEST_ASSERT_TRUE_MESSAGE(stopAt[1] > 0.0 && stopAt[1] < 0.6, line);
        TEST_ASSERT_TRUE_MESSAGE(tiltPeak[1] < 3.0, line);   // крен и тангаж мусор не видят
        if (spin < 3000.0) continue;
        // чем дальше за пределом, тем больше мусора: без защиты раму валит,
        // горизонт уезжает, а остановка вдвое дольше (рули тратятся на мусор)
        TEST_ASSERT_TRUE_MESSAGE(tiltPeak[0] > 8.0, line);
        TEST_ASSERT_TRUE_MESSAGE(stopAt[1] < 0.75 * stopAt[0], line);
        TEST_ASSERT_TRUE_MESSAGE(horizon[1] < 0.5 * horizon[0], line);
    }
}

// Runaway takeoff: два мотора на чужих выходах (провода перепутаны) —
// на газе коптер кувыркается, DISARM меньше чем за 1 с, ARM — после
// выключения тумблера. Без защиты runaway её DISARM не срабатывает.
void test_runaway_takeoff_disarms_on_wrong_motor_wiring()
{
    for (int prevent = 1; prevent >= 0; --prevent)
    {
        CopterSettings s;
        s.runawayPrevention = prevent != 0;
        QuadHarness h(five(), QuadPerturb(), 40, EscProtocol::DSHOT600, 1000.0f, s);
        h.wiring[0] = 1;
        h.wiring[1] = 0;
        TEST_ASSERT_TRUE(h.armOnGround(CopterMode::ANGLE));
        TEST_ASSERT_EQUAL(prevent != 0, h.ctl.getStatus().runawayCheck);
        h.rc.throttle = 0.6f;
        const double t0 = h.sim.now();
        while (h.ctl.isArmed() && h.sim.now() - t0 < 1.0) h.step();
        char line[160];
        snprintf(line, sizeof(line), "runaway %s: %s after %.3f s, max tilt %.0f deg", prevent ? "ON" : "OFF",
                 h.ctl.isArmed() ? "still armed" : "DISARM", h.sim.now() - t0, h.stats.maxTilt);
        TEST_MESSAGE(line);
        if (prevent)
        {
            TEST_ASSERT_FALSE_MESSAGE(h.ctl.isArmed(), line);
            TEST_ASSERT_TRUE(h.ctl.getStatus().runaway);
            TEST_ASSERT_FALSE(h.out.spinning);
            h.step();
            const uint32_t b = h.ctl.getStatus().armBlockers;
            TEST_ASSERT_TRUE(b & ARM_BLOCK_RUNAWAY);
            TEST_ASSERT_TRUE(b & ARM_BLOCK_NOT_DISARMED);
            h.rc.throttle = 0.0f;
            h.rc.armSwitch = false;
            h.step();
            TEST_ASSERT_FALSE(h.ctl.getStatus().runaway);
        }
        else
        {
            TEST_ASSERT_FALSE(h.ctl.getStatus().runaway);
        }
    }
}

// Этапы failsafe: этап 1 (до failsafe_delay) — ровно и держит высоту,
// этап 2 — снижение; связь вернулась на этапе 1 — управление сразу, на
// этапе 2 — через failsafe_recovery_delay.
void test_failsafe_stage1_holds_altitude_then_stage2_descends()
{
    QuadHarness h(five(), QuadPerturb(), 41);
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    const double h0 = h.sim.height();
    h.rc.linkOk = false;
    h.run(1.0);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    TEST_ASSERT_EQUAL_UINT8(1, h.ctl.getStatus().failsafeStage);
    TEST_ASSERT_FLOAT_WITHIN(0.5, h0, h.sim.height());
    h.rc.linkOk = true;
    h.step();
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
    h.run(1.0);
    h.rc.linkOk = false;
    h.run(1.6);
    TEST_ASSERT_EQUAL_UINT8(2, h.ctl.getStatus().failsafeStage);
    h.run(2.0);
    TEST_ASSERT_TRUE(h.sim.height() < h0 - 0.7);
    h.rc.linkOk = true;
    h.run(0.4);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    h.run(0.2);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
    TEST_ASSERT_EQUAL_UINT8(0, h.ctl.getStatus().failsafeStage);
}

// 104: rxfail стиков HOLD — этап 1 летит на последних газе и крене (как в
// BF), этап 2 — снова автомат: ровно и снижение.
void test_failsafe_stage1_rxfail_hold()
{
    QuadHarness h(five(), QuadPerturb(), 43);
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    const double h0 = h.sim.height();
    h.rc.stickHold = 0x01 | 0x08;   // rxfail 0 h, rxfail 3 h
    h.rc.throttle = 0.8f;
    h.rc.roll = 0.3f;
    h.step();
    h.rc.linkOk = false;
    h.run(1.0);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    TEST_ASSERT_EQUAL_UINT8(1, h.ctl.getStatus().failsafeStage);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(h.ctl.getStatus().mode));
    TEST_ASSERT_TRUE_MESSAGE(h.sim.height() > h0 + 1.5, "газ HOLD: этап 1 — на последнем газе, а не на высоте");
    TEST_ASSERT_TRUE_MESSAGE(h.ctl.getStatus().roll > 5.0f, "крен HOLD: последний наклон");
    h.run(1.5);
    TEST_ASSERT_EQUAL_UINT8(2, h.ctl.getStatus().failsafeStage);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ALT_HOLD), static_cast<int>(h.ctl.getStatus().mode));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, h.ctl.getStatus().roll);
}

// DROP: этап 2 — моторы стоп. ARM снова — только когда связь продержалась
// failsafe_recovery_delay и тумблер выключили.
void test_failsafe_drop_procedure()
{
    CopterSettings s;
    s.failsafeProcedure = FailsafeProcedure::DROP;
    QuadHarness h(five(), QuadPerturb(), 42, EscProtocol::DSHOT600, 1000.0f, s);
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.rc.linkOk = false;
    h.run(1.4);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
    h.run(0.2);
    TEST_ASSERT_FALSE(h.ctl.isArmed());
    TEST_ASSERT_FALSE(h.out.spinning);
    while (h.sim.height() > 0.01) h.step();
    h.rc.linkOk = true;
    h.rc.throttle = 0.0f;
    h.rc.armSwitch = false;
    h.run(0.2);
    TEST_ASSERT_TRUE(h.ctl.getStatus().armBlockers & ARM_BLOCK_FAILSAFE);
    h.run(0.4);
    TEST_ASSERT_FALSE(h.ctl.getStatus().armBlockers & ARM_BLOCK_FAILSAFE);
}

// Режим FAILSAFE с тумблера: STAGE2 — сразу снижение, тумблер назад —
// управление сразу; KILL — моторы стоп сразу, ARM не раньше чем через 1 с.
void test_failsafe_switch_modes()
{
    {
        CopterSettings s;
        s.failsafeSwitchMode = FailsafeSwitchMode::STAGE2;
        QuadHarness h(five(), QuadPerturb(), 43, EscProtocol::DSHOT600, 1000.0f, s);
        TEST_ASSERT_TRUE(h.takeoffTo(5.0));
        h.rc.failsafeSwitch = true;
        h.step();
        TEST_ASSERT_EQUAL_UINT8(2, h.ctl.getStatus().failsafeStage);
        h.run(1.0);
        h.rc.failsafeSwitch = false;
        h.step();
        TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
    }
    {
        CopterSettings s;
        s.failsafeSwitchMode = FailsafeSwitchMode::KILL;
        QuadHarness h(five(), QuadPerturb(), 44, EscProtocol::DSHOT600, 1000.0f, s);
        TEST_ASSERT_TRUE(h.takeoffTo(3.0));
        h.rc.failsafeSwitch = true;
        h.step();
        TEST_ASSERT_FALSE(h.ctl.isArmed());
        TEST_ASSERT_FALSE(h.out.spinning);
        while (h.sim.height() > 0.01) h.step();
        h.rc.failsafeSwitch = false;
        h.rc.throttle = 0.0f;
        h.rc.armSwitch = false;
        h.run(0.8);
        TEST_ASSERT_TRUE(h.ctl.getStatus().armBlockers & ARM_BLOCK_FAILSAFE);
        h.run(0.3);
        TEST_ASSERT_FALSE(h.ctl.getStatus().armBlockers & ARM_BLOCK_FAILSAFE);
    }
}

// GPS-RESCUE (пока без GPS — снижение): связь вернулась — управление
// пилоту, только когда он отклонит стик дальше failsafe_stick_threshold.
// 034: angle_feedforward — наклон стиком без запаздывания: до 90 % угла
// быстрее, без перелёта больше 10 % (5" на трёх физиках, в ALT_HOLD на 10 м).
namespace
{
    struct TiltStep
    {
        double t90 = -1;
        double peak = 0;
    };

    TiltStep tiltStep(const CopterTune& t, const QuadPerturb& p)
    {
        QuadHarness h(five(), p, 3);
        h.useTune(t);
        TiltStep r;
        if (!h.takeoffTo(10.0)) return r;
        const double target = 0.5 * t.maxAngleDeg;
        h.rc.roll = 0.5f;
        const double t0 = h.sim.now();
        while (h.sim.now() - t0 < 1.5)
        {
            h.step();
            const double roll = h.sim.rollDeg();
            r.peak = std::fmax(r.peak, roll);
            if (r.t90 < 0 && roll >= 0.9 * target) r.t90 = h.sim.now() - t0;
        }
        r.peak /= target;
        return r;
    }
}

void test_angle_feedforward_tilts_faster()
{
    const CopterTune base = deriveTune(five(), 1000.0f, EscProtocol::DSHOT600);
    CopterTune ff = base;
    ff.pid.angleFf = 0.5f;
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        const TiltStep off = tiltStep(base, ps[k]);
        const TiltStep on = tiltStep(ff, ps[k]);
        char line[200];
        snprintf(line, sizeof(line), "angle_feedforward 50 %-8s: t90 %.0f ms / %.0f ms, peak %.0f%% / %.0f%% (with / without)",
                 names[k], on.t90 * 1000.0, off.t90 * 1000.0, on.peak * 100.0, off.peak * 100.0);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(on.t90 > 0 && off.t90 > 0, line);
        TEST_ASSERT_TRUE_MESSAGE(on.t90 < off.t90, line);
        // перелёт с упреждением — до 13 % (с прежним фильтром ориентации он
        // прятался: горизонт уплывал, и «без упреждения» выходило 105 %)
        TEST_ASSERT_TRUE_MESSAGE(on.peak < 1.15, line);
        TEST_ASSERT_TRUE_MESSAGE(off.peak < 1.04, line);
    }
}

// 035: HORIZON как в BF 4.5 — полный стик крутит флип (выравнивание гаснет
// к horizon_limit_sticks), стик в центре — ровно.
void test_horizon_flips_and_levels()
{
    QuadHarness h(five(), QuadPerturb(), 5);
    TEST_ASSERT_TRUE(h.takeoffTo(15.0));
    const float hover = h.ctl.getStatus().hoverLearned;
    h.rc.mode = CopterMode::HORIZON;
    h.rc.throttle = hover;
    h.run(1.0);
    TEST_ASSERT_TRUE(h.sim.tiltDeg() < 5.0);
    // полстика — наклон держится, как в мягком ANGLE (не крутится дальше)
    h.rc.roll = 0.3f;
    h.run(1.5);
    const double held = h.sim.tiltDeg();
    char line[160];
    snprintf(line, sizeof(line), "HORIZON: 30%% stick holds %.0f deg", held);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(held > 5.0 && held < 80.0, line);
    // полный стик — флип: коптер побывал вверх ногами; стик отпущен, когда
    // оборот закончен (вверх ногами за horizon_limit_degrees выравнивания нет — как в BF)
    h.rc.roll = 1.0f;
    double maxTilt = 0;
    for (int i = 0; i < 3000; ++i)
    {
        h.step();
        maxTilt = std::fmax(maxTilt, h.sim.tiltDeg());
        if (maxTilt > 150.0 && h.sim.tiltDeg() < 60.0) break;
    }
    h.rc.roll = 0.0f;
    snprintf(line, sizeof(line), "HORIZON: full stick reaches %.0f deg tilt", maxTilt);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(maxTilt > 150.0, line);
    // центр — выравнивается (после horizon_delay_ms)
    h.run(2.5);
    snprintf(line, sizeof(line), "HORIZON: %.1f deg 2.5 s after the flip", h.sim.tiltDeg());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(h.sim.tiltDeg() < 8.0, line);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
}

// 036: ACRO TRAINER — полный стик в ACRO, наклон не больше предела (+ запас на
// перелёт); без режима тот же стик крутит коптер дальше.
void test_acro_trainer_limits_tilt()
{
    for (int trainer = 1; trainer >= 0; --trainer)
    {
        QuadHarness h(five(), QuadPerturb(), 7);
        TEST_ASSERT_TRUE(h.takeoffTo(15.0));
        const float hover = h.ctl.getStatus().hoverLearned;
        h.rc.mode = CopterMode::ACRO;
        h.rc.acroTrainer = trainer != 0;
        h.rc.throttle = hover;
        h.run(0.5);
        double maxRoll = 0, maxPitch = 0;
        h.rc.roll = 1.0f;
        for (int i = 0; i < 1500; ++i)
        {
            h.step();
            maxRoll = std::fmax(maxRoll, std::fabs(h.sim.rollDeg()));
            if (!trainer && maxRoll > 60.0) break;
        }
        if (trainer)
        {
            h.rc.roll = 0.0f;
            h.rc.pitch = -1.0f;
            for (int i = 0; i < 1500; ++i)
            {
                h.step();
                maxPitch = std::fmax(maxPitch, h.sim.tiltDeg());
            }
        }
        char line[160];
        snprintf(line, sizeof(line), "ACRO %s: full roll stick -> %.0f deg, full pitch -> tilt %.0f deg",
                 trainer ? "TRAINER" : "plain", maxRoll, maxPitch);
        TEST_MESSAGE(line);
        if (trainer)
        {
            TEST_ASSERT_TRUE_MESSAGE(maxRoll < 30.0, line);   // предел 20° + перелёт
            TEST_ASSERT_TRUE_MESSAGE(maxPitch < 30.0, line);
            TEST_ASSERT_TRUE(h.ctl.isArmed());
        }
        else
        {
            TEST_ASSERT_TRUE_MESSAGE(maxRoll > 60.0, line);
        }
    }
}

// 129–136: GPS Rescue в симуляторе — 5" с GPS (10 Гц, шум 0.4 м) улетел
// за 150 м, связь пропала: набор высоты, разворот, домой при боковом ветре,
// снижение и посадка у дома с DISARM.
namespace
{
    struct RescueRun
    {
        bool disarmed = false;
        double homeDist = 1e9;    // где сел, м от дома
        double seconds = 0;       // от потери связи
        double maxTilt = 0;
        double maxHeight = 0;
        RescuePhase lastPhase = RescuePhase::IDLE;
    };

    double homeDistance(QuadHarness& h)
    {
        const double n = h.sim.position().x - h.homeN, e = h.sim.position().y - h.homeE;
        return std::sqrt(n * n + e * e);
    }

    // Улететь на dist м носом на курс yawDeg (ALT_HOLD), потом — что скажет setup.
    bool flyAway(QuadHarness& h, double dist, float yawStick, double yawSeconds)
    {
        if (!h.takeoffTo(10.0)) return false;
        h.rc.yaw = yawStick;
        h.run(yawSeconds);
        h.rc.yaw = 0.0f;
        h.rc.pitch = -0.6f;
        for (int i = 0; i < 120000 && homeDistance(h) < dist; ++i) h.step();
        h.rc.pitch = 0.0f;
        h.run(2.0);
        return homeDistance(h) >= dist;
    }

    RescueRun linkLostRescue(QuadHarness& h, double limitS)
    {
        RescueRun r;
        h.rc.linkOk = false;
        const double t0 = h.sim.now();
        while (h.ctl.isArmed() && h.sim.now() - t0 < limitS)
        {
            h.step();
            r.maxTilt = std::fmax(r.maxTilt, h.sim.tiltDeg());
            r.maxHeight = std::fmax(r.maxHeight, h.sim.height());
            if (h.ctl.getStatus().rescuePhase != RescuePhase::IDLE) r.lastPhase = h.ctl.getStatus().rescuePhase;
        }
        r.disarmed = !h.ctl.isArmed();
        r.homeDist = homeDistance(h);
        r.seconds = h.sim.now() - t0;
        return r;
    }

    CopterSettings rescueSettings()
    {
        CopterSettings s;
        s.failsafeProcedure = FailsafeProcedure::GPS_RESCUE;
        return s;
    }
}

void test_gps_rescue_returns_home_in_crosswind()
{
    const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
    const char* names[] = { "nominal", "sluggish", "twitchy" };
    for (uint8_t k = 0; k < 3; ++k)
    {
        QuadHarness h(five(), ps[k], 61 + k, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
        h.gpsOn = true;
        TEST_ASSERT_TRUE(flyAway(h, 150.0, 0.5f, 1.0));
        h.sim.setWind(0.0, 4.0, 0.5);   // боковой ветер 4 м/с
        const RescueRun r = linkLostRescue(h, 150.0);
        char line[260];
        snprintf(line, sizeof(line), "GPS rescue %-8s from 150 m, crosswind 4 m/s: landed %.1f m from home in %.0f s, max tilt %.0f, max height %.0f m (%s %s, spoof %s)",
                 names[k], r.homeDist, r.seconds, r.maxTilt, r.maxHeight, rescuePhaseName(r.lastPhase),
                 rescueFailName(h.ctl.getStatus().rescueFail), gpsSpoofName(h.ctl.getStatus().gpsSpoof));
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(r.disarmed, line);
        TEST_ASSERT_TRUE_MESSAGE(r.homeDist < 6.0, line);
        TEST_ASSERT_TRUE_MESSAGE(r.maxTilt < 50.0, line);
        TEST_ASSERT_TRUE_MESSAGE(r.maxHeight > 18.0, line);   // MAX_ALT: 10 м полёта + 10 м initial_climb
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(RescuePhase::LAND), static_cast<int>(r.lastPhase), line);
    }
}

// Без компаса: гироскоп считает курс от включения, а коптер стоял носом на
// 70° — возврат выравнивает курс по GPS в пути и всё равно садится дома.
void test_gps_rescue_without_compass_aligns_heading()
{
    QuadHarness h(five(), QuadPerturb(), 62, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
    h.gpsOn = true;
    h.compass = false;
    h.sim.setAttitude(0.0, 0.0, 70.0);
    TEST_ASSERT_TRUE(flyAway(h, 120.0, -0.4f, 1.5));
    const RescueRun r = linkLostRescue(h, 120.0);
    char line[200];
    snprintf(line, sizeof(line), "GPS rescue without compass (70 deg off): heading by %s, landed %.1f m from home in %.0f s",
             headingSourceName(h.ctl.getStatus().headingSource), r.homeDist, r.seconds);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(r.disarmed, line);
    TEST_ASSERT_TRUE_MESSAGE(r.homeDist < 8.0, line);
}

// P39: возврат летит по общей оценке курса. Компас врёт на 100° (не
// откалиброван, силовой провод рядом), боковой ветер: оценка по первым
// манёврам отбрасывает компас и учит курс по полёту — возврат идёт по нему,
// а не по компасу: 48 с, 1.4 м от дома. До P39 (по компасу) — дуга 102 с,
// наклон до 50°, ложная «подмена GPS» и посадка в 13.7 м от дома.
void test_gps_rescue_with_lying_compass()
{
    QuadHarness h(five(), QuadPerturb(), 63, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
    h.gpsOn = true;
    h.compassErrorDeg = 100.0;
    TEST_ASSERT_TRUE(flyAway(h, 120.0, 0.5f, 1.0));
    h.sim.setWind(0.0, 4.0, 0.5);
    const bool rejected = h.ctl.getStatus().magDisagrees;
    const RescueRun r = linkLostRescue(h, 150.0);
    const CopterStatus& st = h.ctl.getStatus();
    char line[260];
    snprintf(line, sizeof(line), "GPS rescue, compass 100 deg off, crosswind 4 m/s: compass rejected %s, landed %.1f m from home in %.0f s, max tilt %.0f (%s %s)",
             rejected ? "yes" : "no", r.homeDist, r.seconds, r.maxTilt, rescuePhaseName(r.lastPhase), rescueFailName(st.rescueFail));
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(rejected, line);
    TEST_ASSERT_TRUE_MESSAGE(r.disarmed, line);
    TEST_ASSERT_TRUE_MESSAGE(r.homeDist < 8.0, line);
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(RescuePhase::LAND), static_cast<int>(r.lastPhase), line);
}

// GPS пропал на пути домой, полёт вслепую выключен (deadreck_max_s = 0) —
// возврат прерван, посадка на месте (у BF — DISARM).
void test_gps_rescue_gps_loss_lands_in_place()
{
    CopterSettings cs = rescueSettings();
    cs.deadReck.maxS = 0.0f;
    QuadHarness h(five(), QuadPerturb(), 63, EscProtocol::DSHOT600, 1000.0f, cs);
    h.gpsOn = true;
    TEST_ASSERT_TRUE(flyAway(h, 150.0, 0.0f, 0.0));
    h.rc.linkOk = false;
    while (h.ctl.getStatus().rescuePhase != RescuePhase::FLY_HOME && h.sim.now() < 300.0) h.step();
    h.run(3.0);
    const double before = homeDistance(h);
    h.gpsLost = true;
    double tilt = 0;
    const double t0 = h.sim.now();
    while (h.ctl.isArmed() && h.sim.now() - t0 < 120.0)
    {
        h.step();
        if (h.sim.now() - t0 > 4.0) tilt = std::fmax(tilt, h.sim.tiltDeg());
    }
    char line[200];
    snprintf(line, sizeof(line), "GPS lost at %.0f m: %s, landed %.0f m from home, tilt after 4 s %.0f",
             before, rescueFailName(h.ctl.getStatus().rescueFail), homeDistance(h), tilt);
    TEST_MESSAGE(line);
    TEST_ASSERT_FALSE_MESSAGE(h.ctl.isArmed(), line);
    TEST_ASSERT_EQUAL(static_cast<int>(RescueFail::GPS_LOST), static_cast<int>(h.ctl.getStatus().rescueFail));
    TEST_ASSERT_TRUE_MESSAGE(homeDistance(h) > 60.0, line);   // не долетел — сел, где был
    TEST_ASSERT_TRUE_MESSAGE(tilt < 15.0, line);
}

// Домой вслепую (copter/DeadReckonHome.h, P43 в): 7" улетел на 250 м,
// ветер 3.6 м/с, связь пропала, GPS Rescue повёл домой — и тут (1) GPS
// заглушили: точка — последний фикс, дальше путь по модели «наклон →
// скорость» и ветру из фильтра; (2) GPS подменили — точка прыгнула на 1 км
// (JUMP, сразу): точка — последний честный фикс. Коптер долетает и садится
// у дома с ошибкой в несколько процентов пути, а не на месте.
void test_dead_reckon_home_when_gps_jammed_or_spoofed()
{
    for (int spoof = 0; spoof <= 1; ++spoof)
    {
        QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 71 + spoof, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
        h.gpsOn = true;
        h.gpsDelayS = 0.12;
        h.sim.setWind(3.0, 2.0, 0.5);
        TEST_ASSERT_TRUE(flyAway(h, 250.0, 0.6f, 1.0));
        h.run(20.0);   // сопротивление и ветер выучены в полёте
        h.rc.linkOk = false;
        while (h.ctl.getStatus().rescuePhase != RescuePhase::FLY_HOME && h.sim.now() < 600.0) h.step();
        h.run(4.0);
        const double from = homeDistance(h);
        if (!spoof) h.gpsLost = true;
        double maxTilt = 0.0, worstErr = 0.0, blindAt = -1.0, spoofAt = -1.0, ledAway = 0.0;
        DeadReckonPhase lastPhase = DeadReckonPhase::IDLE;
        const double t0 = h.sim.now();
        int i = 0;
        while (h.ctl.isArmed() && h.sim.now() - t0 < 200.0)
        {
            if (spoof) h.spoofE = 1000.0;   // ложная точка — в километре
            ++i;
            h.step();
            const CopterStatus& st = h.ctl.getStatus();
            maxTilt = std::fmax(maxTilt, h.sim.tiltDeg());
            if (spoofAt < 0.0 && st.gpsSpoof != GpsSpoof::NONE)
            {
                spoofAt = h.sim.now() - t0;
                ledAway = homeDistance(h) - from;
            }
            if (st.deadReckPhase != DeadReckonPhase::IDLE)
            {
                if (blindAt < 0.0) blindAt = h.sim.now() - t0;
                lastPhase = st.deadReckPhase;
                const double en = st.deadReckN - (h.sim.position().x - h.homeN);
                const double ee = st.deadReckE - (h.sim.position().y - h.homeE);
                worstErr = std::fmax(worstErr, std::sqrt(en * en + ee * ee));
            }
        }
        const CopterStatus& st = h.ctl.getStatus();
        char line[320];
        snprintf(line, sizeof(line), "blind RTH (%s at %.0f m, wind 3.6 m/s): spoof %s seen after %.1f s (led %+.0f m), blind after %.1f s, landed %.0f m from home in %.0f s, est. error worst %.0f m (reported %.0f), max tilt %.0f, %s/%s",
                 spoof ? "GPS spoofed" : "GPS jammed", from, gpsSpoofName(st.gpsSpoof), spoofAt, ledAway, blindAt, homeDistance(h), h.sim.now() - t0, worstErr,
                 static_cast<double>(st.deadReckErrM), maxTilt, deadReckonPhaseName(lastPhase), rescueFailName(st.rescueFail));
        TEST_MESSAGE(line);
        TEST_ASSERT_FALSE_MESSAGE(h.ctl.isArmed(), line);
        TEST_ASSERT_TRUE_MESSAGE(blindAt >= 0.0, line);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(DeadReckonPhase::LAND), static_cast<int>(lastPhase), line);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(spoof ? RescueFail::GPS_SPOOF : RescueFail::GPS_LOST),
                                  static_cast<int>(st.rescueFail), line);
        TEST_ASSERT_TRUE_MESSAGE(homeDistance(h) < 0.1 * from, line);
        TEST_ASSERT_TRUE_MESSAGE(maxTilt < 50.0, line);
    }
}

// Глушили и отпустили: GPS годен 3 с подряд — снова GPS Rescue, садится у дома точно.
void test_dead_reckon_hands_back_to_gps()
{
    QuadHarness h(five(), QuadPerturb(), 73, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
    h.gpsOn = true;
    TEST_ASSERT_TRUE(flyAway(h, 150.0, 0.0f, 0.0));
    h.run(10.0);
    h.rc.linkOk = false;
    while (h.ctl.getStatus().rescuePhase != RescuePhase::FLY_HOME && h.sim.now() < 300.0) h.step();
    h.gpsLost = true;
    h.run(8.0);
    const bool blind = h.ctl.getStatus().deadReckPhase != DeadReckonPhase::IDLE;
    h.gpsLost = false;
    h.run(5.0);
    const bool back = h.ctl.getStatus().deadReckPhase == DeadReckonPhase::IDLE &&
                      h.ctl.getStatus().rescuePhase != RescuePhase::ABORT;
    const RescueRun r = linkLostRescue(h, 120.0);
    char line[200];
    snprintf(line, sizeof(line), "GPS jammed 8 s on the way home: blind %s, GPS back -> rescue %s, landed %.1f m from home",
             blind ? "yes" : "NO", back ? "yes" : "NO", r.homeDist);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(blind, line);
    TEST_ASSERT_TRUE_MESSAGE(back, line);
    TEST_ASSERT_TRUE_MESSAGE(r.disarmed, line);
    TEST_ASSERT_TRUE_MESSAGE(r.homeDist < 6.0, line);
}

// Режим GPS RESCUE с тумблера: связь есть, коптер летит домой; тумблер назад —
// управление пилоту сразу. Без GPS и с GPS-RESCUE — ARM запрещён (флаг GPS).
void test_gps_rescue_switch_and_arming_check()
{
    {
        QuadHarness h(five(), QuadPerturb(), 64, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
        TEST_ASSERT_FALSE(h.armOnGround());
        TEST_ASSERT_TRUE(h.ctl.getStatus().armBlockers & ARM_BLOCK_GPS);
    }
    QuadHarness h(five(), QuadPerturb(), 65, EscProtocol::DSHOT600, 1000.0f, rescueSettings());
    h.gpsOn = true;
    TEST_ASSERT_TRUE(flyAway(h, 80.0, 0.0f, 0.0));
    const double start = homeDistance(h);
    h.rc.gpsRescue = true;
    h.run(15.0);
    const double mid = homeDistance(h);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
    TEST_ASSERT_TRUE(h.ctl.getStatus().rescuePhase == RescuePhase::FLY_HOME || h.ctl.getStatus().rescuePhase == RescuePhase::DESCEND);
    char line[120];
    snprintf(line, sizeof(line), "GPS RESCUE switch: %.0f m -> %.0f m in 15 s", start, mid);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(mid < start - 30.0, line);
    h.rc.gpsRescue = false;
    h.step();
    TEST_ASSERT_EQUAL(static_cast<int>(RescuePhase::IDLE), static_cast<int>(h.ctl.getStatus().rescuePhase));
    h.run(2.0);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(h.sim.tiltDeg() < 10.0);   // ALT_HOLD пилота, стики в центре — ровно
}

void test_failsafe_rescue_returns_control_by_stick()
{
    CopterSettings s;
    s.failsafeProcedure = FailsafeProcedure::GPS_RESCUE;
    QuadHarness h(five(), QuadPerturb(), 45, EscProtocol::DSHOT600, 1000.0f, s);
    h.gpsOn = true;   // GPS-RESCUE без GPS не армится
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    h.rc.linkOk = false;
    h.run(2.0);
    TEST_ASSERT_EQUAL_UINT8(2, h.ctl.getStatus().failsafeStage);
    h.rc.linkOk = true;
    h.rc.roll = 0.2f;
    h.run(1.0);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    h.rc.roll = 0.4f;
    h.step();
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::ARMED), static_cast<int>(h.ctl.getStatus().state));
}

// 105: acc_trim_roll / acc_trim_pitch — ANGLE со стиками в центре держит
// наклон на величину подстройки (+ — вправо и нос вниз, как в BF).
void test_level_trim_shifts_angle_hover()
{
    CopterSettings s;
    s.levelTrimRollDeg = 5.0f;
    s.levelTrimPitchDeg = 3.0f;
    QuadHarness h(five(), QuadPerturb(), 31, EscProtocol::DSHOT600, 1000.0f, s);
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.run(2.0);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5.0f, h.ctl.getStatus().roll);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -3.0f, h.ctl.getStatus().pitch);
}

// P09: учебные ограничения в симуляторе.
namespace
{
    CopterSettings trainingSettings()
    {
        CopterSettings s;
        s.training.enabled = true;
        s.training.maxAltM = 20.0f;
        s.training.maxDistM = 60.0f;
        s.training.maxAngleDeg = 30.0f;
        s.training.maxSpeedMs = 0.0f;
        return s;
    }
}

// Потолок: новичок в ANGLE держит газ в полный — коптер упирается в
// training_max_alt и висит под ним; вниз — по стику, как обычно.
void test_training_ceiling_holds_full_throttle()
{
    QuadHarness h(five(), QuadPerturb(), 71, EscProtocol::DSHOT600, 1000.0f, trainingSettings());
    TEST_ASSERT_TRUE(h.armOnGround(CopterMode::ANGLE));
    h.rc.throttle = 1.0f;
    double top = 0;
    bool flagged = false;
    for (int i = 0; i < 20000; ++i)   // 20 с
    {
        h.step();
        top = std::fmax(top, h.sim.height());
        flagged = flagged || (h.ctl.getStatus().trainingFlags & TRAINING_ALT);
    }
    const double held = h.sim.height();
    h.rc.throttle = 0.1f;
    h.run(4.0);
    char line[160];
    snprintf(line, sizeof(line), "training ceiling 20 m, full throttle in ANGLE: top %.1f m, after 20 s %.1f m, throttle down -> %.1f m",
             top, held, h.sim.height());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE(flagged);
    TEST_ASSERT_TRUE_MESSAGE(top < 23.0, line);
    TEST_ASSERT_TRUE_MESSAGE(held > 15.0, line);   // не падает: висит у потолка
    TEST_ASSERT_TRUE_MESSAGE(h.sim.height() < held - 3.0, line);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
}

// Круг: стик вперёд до упора и держать — за краем коптер сам летит назад,
// внутри 80 % радиуса управление снова у пилота; дальше края не уходит.
void test_training_fence_turns_back()
{
    QuadHarness h(five(), QuadPerturb(), 72, EscProtocol::DSHOT600, 1000.0f, trainingSettings());
    h.gpsOn = true;
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    h.rc.pitch = -1.0f;
    double far = 0, high = 0;
    bool returned = false;
    for (int i = 0; i < 60000; ++i)   // минута стика вперёд
    {
        h.step();
        far = std::fmax(far, homeDistance(h));
        high = std::fmax(high, h.sim.height());
        returned = returned || (h.ctl.getStatus().trainingFlags & TRAINING_RETURN);
    }
    h.rc.pitch = 0.0f;
    h.run(25.0);
    char line[180];
    snprintf(line, sizeof(line), "training fence 60 m, stick forward for 60 s: farthest %.0f m, highest %.0f m, after release %.0f m, phase %s",
             far, high, homeDistance(h), rescuePhaseName(h.ctl.getStatus().rescuePhase));
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE(returned);
    TEST_ASSERT_TRUE_MESSAGE(far < 75.0, line);
    TEST_ASSERT_TRUE_MESSAGE(homeDistance(h) < 60.0, line);
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(RescuePhase::IDLE), static_cast<int>(h.ctl.getStatus().rescuePhase), line);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
    TEST_ASSERT_TRUE(h.sim.height() < 23.0);   // и возврат — не выше потолка
}

// P44: геозабор в симуляторе.
namespace
{
    CopterSettings fenceSettings(float radius, float ceiling)
    {
        CopterSettings s;
        s.geofence.enabled = true;
        s.geofence.radiusM = radius;
        s.geofence.ceilingM = ceiling;
        return s;
    }

    // Куда наклоняется коптер от стиков (в ANGLE): pitchSign — знак тангажа
    // при стике вперёд, rollSign — крена при стике вправо.
    struct StickSigns
    {
        double pitch = 1.0, roll = 1.0;
        float hover = 0.4f;
    };
    StickSigns learnStickSigns(QuadHarness& h)
    {
        StickSigns g;
        h.rc.pitch = -1.0f;
        h.run(0.6);
        g.pitch = h.sim.pitchDeg() > 0 ? 1.0 : -1.0;
        h.rc.pitch = 0.0f;
        h.run(3.0);
        h.rc.roll = 1.0f;
        h.run(0.6);
        g.roll = h.sim.rollDeg() > 0 ? 1.0 : -1.0;
        h.rc.roll = 0.0f;
        h.run(3.0);
        g.hover = h.ctl.getStatus().throttle;
        return g;
    }

    // «Пилот» в ACRO: держит наклон носом вниз targetDeg (стик — скорость
    // вращения), крен ноль и высоту газом.
    void acroPilot(QuadHarness& h, double targetDeg, const StickSigns& g, double holdAlt)
    {
        const double pitch = h.sim.pitchDeg() * g.pitch;   // > 0 — туда, куда стик вперёд
        h.rc.pitch = static_cast<float>(-std::fmax(-1.0, std::fmin(1.0, 0.03 * (targetDeg - pitch))));
        h.rc.roll = static_cast<float>(-std::fmax(-1.0, std::fmin(1.0, 0.03 * h.sim.rollDeg() * g.roll)));
        const double tilt = std::fmin(70.0, h.sim.tiltDeg()) * 0.017453293;
        // скорость по z — вниз
        const double thr = g.hover / std::cos(tilt) + 0.08 * (holdAlt - h.sim.height()) + 0.05 * h.sim.velocity().z;
        h.rc.throttle = static_cast<float>(std::fmax(0.05, std::fmin(1.0, thr)));
    }
}

// Пилот в ACRO рвётся наружу на 50° (25–30 м/с) — минуту, после каждого
// возврата снова: коптер ни разу не уходит за край дальше чем на несколько
// метров, стики во время возврата не слушаются, потом управление снова у пилота.
void test_geofence_acro_pilot_cannot_leave()
{
    QuadHarness h(five(), QuadPerturb(), 81, EscProtocol::DSHOT600, 1000.0f, fenceSettings(100.0f, 0.0f));
    h.gpsOn = true;
    TEST_ASSERT_TRUE(h.takeoffTo(15.0));
    const StickSigns g = learnStickSigns(h);
    h.rc.mode = CopterMode::ACRO;
    double far = 0, fast = 0;
    int returns = 0;
    bool was = false, pilotAgain = false;
    for (int i = 0; i < 60000; ++i)
    {
        acroPilot(h, 50.0, g, 15.0);
        h.step();
        far = std::fmax(far, homeDistance(h));
        fast = std::fmax(fast, std::hypot(h.sim.velocity().x, h.sim.velocity().y));
        const bool ret = h.ctl.getStatus().fenceFlags & FENCE_RETURN;
        if (ret && !was) ++returns;
        if (!ret && was) pilotAgain = true;
        was = ret;
        if (h.sim.height() < 1.0) break;
    }
    char line[200];
    snprintf(line, sizeof(line), "geofence 100 m, ACRO pilot at 50 deg for 60 s: farthest %.1f m, top speed %.1f m/s, %d returns, height %.1f m",
             far, fast, returns, h.sim.height());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(fast > 15.0, line);   // пилот правда разгонялся
    TEST_ASSERT_TRUE_MESSAGE(far < 105.0, line);
    TEST_ASSERT_TRUE_MESSAGE(returns >= 1, line);
    TEST_ASSERT_TRUE_MESSAGE(pilotAgain, line);
    TEST_ASSERT_TRUE_MESSAGE(h.sim.height() > 3.0, line);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
}

// Потолок забора и в ACRO: газ в полный — коптер висит под потолком.
void test_geofence_ceiling_in_acro()
{
    QuadHarness h(five(), QuadPerturb(), 82, EscProtocol::DSHOT600, 1000.0f, fenceSettings(0.0f, 25.0f));
    h.gpsOn = true;
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    const StickSigns g = learnStickSigns(h);
    h.rc.mode = CopterMode::ACRO;
    double top = 0;
    for (int i = 0; i < 15000; ++i)
    {
        acroPilot(h, 0.0, g, 100.0);
        h.rc.throttle = 1.0f;
        h.step();
        top = std::fmax(top, h.sim.height());
    }
    char line[160];
    snprintf(line, sizeof(line), "geofence ceiling 25 m, full throttle in ACRO: top %.1f m, after 15 s %.1f m", top, h.sim.height());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(top < 28.0, line);
    TEST_ASSERT_TRUE_MESSAGE(h.sim.height() > 20.0, line);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
}

// GPS пропал внутри забора, пилот тянет наружу: через 3 с стики больше не
// слушаются — домой вслепую или посадка; дальше нескольких метров не уходит.
void test_geofence_gps_lost_takes_control()
{
    QuadHarness h(five(), QuadPerturb(), 83, EscProtocol::DSHOT600, 1000.0f, fenceSettings(100.0f, 0.0f));
    h.gpsOn = true;
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    h.gpsLost = true;
    h.rc.pitch = -1.0f;
    double lostAt = homeDistance(h), far = 0;
    bool took = false;
    for (int i = 0; i < 30000; ++i)
    {
        h.step();
        far = std::fmax(far, homeDistance(h));
        took = took || (h.ctl.getStatus().fenceFlags & (FENCE_RETURN | FENCE_LAND));
        if (!h.ctl.isArmed()) break;
    }
    char line[200];
    snprintf(line, sizeof(line), "geofence, GPS lost at %.0f m, stick forward: took control %d, farthest %.0f m, end height %.1f m, %.0f m from home",
             lostAt, took ? 1 : 0, far, h.sim.height(), homeDistance(h));
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(took, line);
    TEST_ASSERT_TRUE_MESSAGE(far < 60.0, line);         // 3 с стика вперёд — и всё
    TEST_ASSERT_TRUE_MESSAGE(h.sim.height() < 1.0, line);   // сел (вслепую домой или на месте)
}

// С забором без GPS не армится; с GPS — армится.
void test_geofence_blocks_arming_without_gps()
{
    QuadHarness h(five(), QuadPerturb(), 84, EscProtocol::DSHOT600, 1000.0f, fenceSettings(100.0f, 0.0f));
    TEST_ASSERT_FALSE(h.armOnGround());
    TEST_ASSERT_TRUE(h.ctl.getStatus().armBlockers & ARM_BLOCK_GPS);
    h.rc.armSwitch = false;
    h.gpsOn = true;
    TEST_ASSERT_TRUE(h.armOnGround());
}

// P45 а: залетел под кроны и потерял связь. 5" на 15 м отлетел на 50 м,
// спустился до 3 м и ушёл вбок на 25 м «под деревья». Возврат идёт назад
// по следу на 3 м, поднимается там, где спускался, и только потом — домой.
// С rth_breadcrumbs OFF (как BF) — набор высоты прямо под кронами.
namespace
{
    struct CanopyRun
    {
        bool disarmed = false, trail = false;
        double homeDist = 0, forestTop = 0, seconds = 0;
    };

    CanopyRun canopyRescue(bool breadcrumbs)
    {
        CopterSettings cs = rescueSettings();
        cs.rescue.breadcrumbs = breadcrumbs;
        QuadHarness h(five(), QuadPerturb(), 91, EscProtocol::DSHOT600, 1000.0f, cs);
        h.gpsOn = true;
        CanopyRun r;
        if (!h.takeoffTo(15.0)) return r;
        h.rc.pitch = -0.5f;
        for (int i = 0; i < 60000 && homeDistance(h) < 50.0; ++i) h.step();
        h.rc.pitch = 0.0f;
        h.run(4.0);
        // вниз до 3 м
        h.rc.throttle = 0.15f;
        for (int i = 0; i < 60000 && h.sim.height() > 3.5; ++i) h.step();
        h.rc.throttle = 0.5f;
        h.run(3.0);
        const double en = h.sim.position().x - h.homeN, ee = h.sim.position().y - h.homeE;
        const double el = std::hypot(en, ee);
        // вбок 25 м
        h.rc.roll = 0.4f;
        for (int i = 0; i < 60000; ++i)
        {
            h.step();
            const double n = h.sim.position().x - h.homeN, e = h.sim.position().y - h.homeE;
            if (std::hypot(n - en, e - ee) > 25.0) break;
        }
        h.rc.roll = 0.0f;
        h.run(3.0);
        h.rc.linkOk = false;
        const double t0 = h.sim.now();
        while (h.ctl.isArmed() && h.sim.now() - t0 < 200.0)
        {
            h.step();
            const double n = h.sim.position().x - h.homeN, e = h.sim.position().y - h.homeE;
            // «лес» — сбоку от линии дом → место спуска дальше 6 м
            const double lateral = std::fabs(en / el * e - ee / el * n);
            if (lateral > 6.0) r.forestTop = std::fmax(r.forestTop, h.sim.height());
            if (h.ctl.getStatus().rescuePhase == RescuePhase::TRAIL) r.trail = true;
        }
        r.disarmed = !h.ctl.isArmed();
        r.homeDist = homeDistance(h);
        r.seconds = h.sim.now() - t0;
        return r;
    }
}

void test_gps_rescue_breadcrumbs_out_of_canopy()
{
    const CanopyRun on = canopyRescue(true), off = canopyRescue(false);
    char line[260];
    snprintf(line, sizeof(line), "under canopy (3 m, 25 m aside): breadcrumbs ON - max height under trees %.1f m, landed %.1f m from home in %.0f s; OFF - %.1f m",
             on.forestTop, on.homeDist, on.seconds, off.forestTop);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(on.trail, line);
    TEST_ASSERT_TRUE_MESSAGE(on.disarmed, line);
    TEST_ASSERT_TRUE_MESSAGE(on.homeDist < 6.0, line);
    TEST_ASSERT_TRUE_MESSAGE(on.forestTop < 7.0, line);    // под кронами не поднимался
    TEST_ASSERT_TRUE_MESSAGE(off.forestTop > 12.0, line);  // без следа — полез вверх в ветки
}

// P44 б2: запретная зона между коптером и домом. Коптер облетел её
// (60 м на север, 60 на восток, 60 на юг), связь пропала: прямо домой —
// через зону, поэтому назад по следу, пока прямая не станет свободной.
void test_gps_rescue_goes_around_nofly_zone()
{
    static GeoZoneTable zones;
    zones.reset();
    char err[64];
    auto pt = [&](int i, double n, double e) {
        char s[80];
        snprintf(s, sizeof(s), "0 %d %.7f %.7f", i, 55.75 + n / 111194.93,
                 37.62 + e / (111194.93 * std::cos(55.75 * 0.017453293)));
        TEST_ASSERT_TRUE(zones.parse(s, err, sizeof(err)));
    };
    TEST_ASSERT_TRUE(zones.parse("0 NOFLY", err, sizeof(err)));
    pt(0, -20, 15);
    pt(1, 20, 15);
    pt(2, 20, 45);
    pt(3, -20, 45);
    CopterSettings cs = rescueSettings();
    cs.geofence.enabled = true;
    cs.geofence.radiusM = 0.0f;
    cs.geofence.ceilingM = 0.0f;
    cs.geofence.zones = &zones;
    QuadHarness h(five(), QuadPerturb(), 92, EscProtocol::DSHOT600, 1000.0f, cs);
    h.gpsOn = true;
    TEST_ASSERT_TRUE(h.takeoffTo(12.0));
    auto pos = [&](double& n, double& e) {
        n = h.sim.position().x - h.homeN;
        e = h.sim.position().y - h.homeE;
    };
    double n = 0, e = 0;
    // ALT_HOLD без GPS-удержания: отпущенный стик — коптер ещё катится, поэтому
    // стик убирается заранее (по скорости)
    auto leg = [&](float CopterRc::*axis, float stick, int axisN, double until) {
        h.rc.*axis = stick;
        for (int i = 0; i < 60000; ++i)
        {
            pos(n, e);
            const double p = axisN ? n : e;
            const double v = axisN ? h.sim.velocity().x : h.sim.velocity().y;
            if ((until - p) * (stick > 0 ? 1 : -1) * (axisN ? -1 : 1) <= std::fabs(v) * 1.5) break;
            h.step();
        }
        h.rc.*axis = 0.0f;
        h.run(5.0);
    };
    leg(&CopterRc::pitch, -0.25f, 1, 60.0);
    leg(&CopterRc::roll, 0.25f, 0, 60.0);
    leg(&CopterRc::pitch, 0.25f, 1, 0.0);
    pos(n, e);
    const double startN = n, startE = e;
    h.rc.linkOk = false;
    bool trail = false, entered = false;
    double nearest = 1e9;
    const double t0 = h.sim.now();
    while (h.ctl.isArmed() && h.sim.now() - t0 < 200.0)
    {
        h.step();
        pos(n, e);
        if (n > -20 && n < 20 && e > 15 && e < 45) entered = true;
        const double dn = n < -20 ? -20 - n : n > 20 ? n - 20 : 0, de = e < 15 ? 15 - e : e > 45 ? e - 45 : 0;
        nearest = std::fmin(nearest, std::hypot(dn, de));
        if (h.ctl.getStatus().rescuePhase == RescuePhase::TRAIL) trail = true;
    }
    char line[260];
    snprintf(line, sizeof(line), "NOFLY between: start %.0f/%.0f m, trail %d, entered zone %d, closest %.1f m, landed %.1f m from home in %.0f s",
             startN, startE, trail, entered, nearest, homeDistance(h), h.sim.now() - t0);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(startN) < 12.0 && startE > 50.0, line);   // правда за зоной
    TEST_ASSERT_TRUE_MESSAGE(nearest > 2.0, line);
    TEST_ASSERT_TRUE_MESSAGE(trail, line);
    TEST_ASSERT_FALSE_MESSAGE(entered, line);
    TEST_ASSERT_FALSE_MESSAGE(h.ctl.isArmed(), line);
    TEST_ASSERT_TRUE_MESSAGE(homeDistance(h) < 6.0, line);
}

// P32: долгий разгон в ALT_HOLD с удержанием высоты — модуль ускорения
// около 1 g, а «верх» акселерометра наклонён меньше настоящего. Горизонт
// не должен уходить (раньше — на 12°: лимит 30°, реально 42°), а высота —
// врать на метры. С GPS поправка гаснет по ускорению, без GPS — по наклону.
void test_horizon_and_altitude_hold_in_long_acceleration()
{
    for (int withGps = 1; withGps >= 0; --withGps)
    {
        QuadHarness h(five(), QuadPerturb(), 73, EscProtocol::DSHOT600, 1000.0f, trainingSettings());
        h.gpsOn = withGps != 0;
        TEST_ASSERT_TRUE(h.takeoffTo(10.0));
        double horizon = 0, alt = 0;
        auto watch = [&]() {
            const CopterStatus& st = h.ctl.getStatus();
            horizon = std::fmax(horizon, std::fabs(st.pitch - h.sim.pitchDeg()));
            alt = std::fmax(alt, std::fabs(st.altitude - h.sim.height()));
        };
        h.rc.pitch = -1.0f;   // 6 с стика вперёд — разгон до 15 м/с с лимитом 30°
        for (int i = 0; i < 6000; ++i, watch()) h.step();
        h.rc.pitch = 1.0f;    // торможение
        for (int i = 0; i < 3000; ++i, watch()) h.step();
        h.rc.pitch = 0.0f;
        for (int i = 0; i < 5000; ++i, watch()) h.step();
        char line[160];
        snprintf(line, sizeof(line), "long acceleration in ALT_HOLD %s GPS: horizon error up to %.1f deg, altitude error up to %.2f m",
                 withGps ? "with" : "without", horizon, alt);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(horizon < (withGps ? 6.0 : 8.0), line);
        TEST_ASSERT_TRUE_MESSAGE(alt < 1.0, line);
    }
}

// Скорость: в ANGLE полный стик вперёд — с пределом 20 км/ч наклон
// уменьшается, коптер разгоняется вдвое меньше, чем без него.
void test_training_speed_limit()
{
    double speeds[2];
    for (int k = 0; k < 2; ++k)
    {
        CopterSettings s = trainingSettings();
        s.training.maxAltM = 0.0f;
        s.training.maxDistM = 0.0f;
        s.training.maxSpeedMs = k ? 20.0f / 3.6f : 0.0f;
        QuadHarness h(five(), QuadPerturb(), 73, EscProtocol::DSHOT600, 1000.0f, s);
        h.gpsOn = true;
        TEST_ASSERT_TRUE(h.takeoffTo(10.0));
        h.rc.pitch = -1.0f;
        h.run(15.0);
        speeds[k] = std::hypot(h.sim.velocity().x, h.sim.velocity().y);
    }
    char line[140];
    snprintf(line, sizeof(line), "ANGLE full stick 15 s, tilt 30 deg: %.1f km/h free, %.1f km/h with 20 km/h limit",
             speeds[0] * 3.6, speeds[1] * 3.6);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(speeds[1] * 3.6 < 28.0, line);
    TEST_ASSERT_TRUE_MESSAGE(speeds[1] < 0.7 * speeds[0], line);
}
// 061: вибрация не на тех оборотах, что думает модель (±30 %), — режекторы по
// модели мимо; динамический режектор находит её по спектру — моторы тише.
void test_dyn_notch_catches_vibration_the_rpm_model_misses()
{
    const double scales[] = { 0.7, 1.3 };
    for (double scale : scales)
    {
        QuadPerturb p;
        p.vibFrequency = scale;
        p.vibration = 2.0;
        double noise[2] = {};
        for (int k = 0; k < 2; ++k)
        {
            QuadHarness h(five(), p, 31);
            CopterTune t = h.tune;
            t.filters.dynNotch.count = k ? 1 : 0;
            t.filters.dynNotch.q = 5.0f;
            t.filters.dynNotch.maxHz = 450.0f;
            h.useTune(t);
            TEST_ASSERT_TRUE(h.takeoffTo(5.0));
            h.stats.reset();
            h.run(3.0);
            noise[k] = h.stats.motorNoiseRms();
            TEST_ASSERT_TRUE(h.stats.maxTilt < 3.0);
        }
        char line[160];
        snprintf(line, sizeof(line), "dyn notch, vibration at %.0f%% of model rpm: motor noise %.4f -> %.4f", scale * 100.0, noise[0],
                 noise[1]);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(noise[1] < 0.8 * noise[0], line);
    }
}

// 041: dynamic idle. По оборотам DShot самый медленный мотор на газу в ноль
// (спуск в ACRO) не проваливается ниже dyn_idle_min_rpm; на земле до airmode
// подъём не больше dyn_idle_start_increase и коптер не подпрыгивает; без
// телеметрии добавки нет.
static double slowestRotorHz(QuadHarness& h)
{
    double lo = 1e9;
    for (uint8_t m = 0; m < h.tune.frame.count; ++m) lo = std::fmin(lo, h.sim.rotorHz(m));
    return lo;
}

static void dynIdleRun(float minRpm, bool telemetry, double& slowest, float& increase, float& groundIncrease,
                       double& groundHeight)
{
    QuadHarness h(five(), QuadPerturb(), 41);
    copter::CopterTune t = h.tune;
    t.dynIdle.minRpm = minRpm;
    h.useTune(t);
    h.rpmTelemetry = telemetry;
    TEST_ASSERT_TRUE(h.armOnGround(CopterMode::ACRO));
    groundIncrease = 0.0f;
    groundHeight = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        h.step();
        groundIncrease = std::fmax(groundIncrease, h.ctl.getStatus().dynIdleIncrease);
        groundHeight = std::fmax(groundHeight, h.sim.height());
    }
    h.rc.armSwitch = false;
    h.run(0.5);
    TEST_ASSERT_TRUE(h.takeoffTo(6.0));
    h.rc.mode = CopterMode::ACRO;
    h.rc.throttle = 0.0f;
    slowest = 1e9;
    increase = 0.0f;
    for (int i = 0; i < 600; ++i)
    {
        h.step();
        if (i < 250) continue;
        slowest = std::fmin(slowest, slowestRotorHz(h));
        increase = std::fmax(increase, h.ctl.getStatus().dynIdleIncrease);
    }
    TEST_ASSERT_TRUE(h.ctl.isArmed());
}

void test_dynamic_idle_holds_min_rpm_on_throttle_cut()
{
    // холостые сборки без dynamic idle — от них и берём минимум (в 1,6 раза выше)
    double idleHz = 0;
    {
        QuadHarness h(five(), QuadPerturb(), 41);
        TEST_ASSERT_TRUE(h.armOnGround(CopterMode::ACRO));
        h.run(1.0);
        idleHz = slowestRotorHz(h);
    }
    const double targetHz = idleHz * 1.6;
    const float minRpm = static_cast<float>(targetHz * 60.0);
    double slowOff = 0, slowOn = 0, slowNoTel = 0, hOn = 0, hOff = 0, hNoTel = 0;
    float incOff = 0, incOn = 0, incNoTel = 0, gOff = 0, gOn = 0, gNoTel = 0;
    dynIdleRun(0.0f, true, slowOff, incOff, gOff, hOff);
    dynIdleRun(minRpm, true, slowOn, incOn, gOn, hOn);
    dynIdleRun(minRpm, false, slowNoTel, incNoTel, gNoTel, hNoTel);
    const copter::DynIdleSettings def;
    char msg[220];
    snprintf(msg, sizeof(msg),
             "dynamic idle: idle %.0f Hz, min %.0f Hz; throttle cut slowest motor off %.0f / on %.0f / no DShot %.0f Hz, "
             "increase %.3f (ground %.3f, max %.3f/%.3f)",
             idleHz, targetHz, slowOff, slowOn, slowNoTel, incOn, gOn, def.startIncrease, def.maxIncrease);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(slowOn >= 0.85 * targetHz, msg);     // минимум держится
    TEST_ASSERT_TRUE_MESSAGE(slowOn > slowOff * 1.2, msg);        // а без dynamic idle обороты ниже
    TEST_ASSERT_TRUE_MESSAGE(incOn > 0.0f && incOn <= def.maxIncrease + 1e-6f, msg);
    TEST_ASSERT_TRUE_MESSAGE(gOn <= def.startIncrease + 1e-6f, msg);   // на земле до airmode — малый предел
    TEST_ASSERT_TRUE_MESSAGE(hOn < 0.01, msg);                         // и коптер стоит
    TEST_ASSERT_EQUAL_FLOAT(0.0f, incOff);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, incNoTel);                           // без телеметрии — выключен
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gNoTel);
}

// 063: обороты по DShot — режекторы на каждом моторе там, где он крутится;
// модель — на средних и по команде. В манёвре на «вялой» физике моторы тише.
void test_rpm_filter_per_motor_from_dshot_in_manoeuvre()
{
    QuadPerturb p = sluggish();
    p.motorSpread = 0.1;
    p.vibration = 2.0;
    double noise[2] = {}, residual[2] = {};
    for (int k = 0; k < 2; ++k)
    {
        QuadHarness h(five(), p, 32);
        h.rpmTelemetry = k == 1;
        TEST_ASSERT_TRUE(h.takeoffTo(5.0));
        h.rc.mode = CopterMode::ANGLE;
        h.stats.reset();
        // остаток вибрации в гироскопе после фильтров: отфильтрованное минус
        // настоящее вращение, выше ~50 Гц (задержка фильтров на манёвре — ниже)
        double lag[2] = {}, sum = 0;
        long n = 0;
        h.afterStep = [&]() {
            const auto w = h.sim.rates();
            const double truth[2] = { w.x * 57.29578, w.y * 57.29578 };
            for (int a = 0; a < 2; ++a)
            {
                const double d = h.ctl.getStatus().rates[a] - truth[a];
                lag[a] += 0.24 * (d - lag[a]);
                sum += (d - lag[a]) * (d - lag[a]);
            }
            ++n;
        };
        for (int i = 0; i < 12; ++i)
        {
            h.rc.roll = i % 2 ? -0.6f : 0.6f;
            h.run(0.25);
        }
        h.rc.roll = 0.0f;
        h.afterStep = nullptr;
        noise[k] = h.stats.motorNoiseRms();
        residual[k] = std::sqrt(sum / n);
        TEST_ASSERT_TRUE(h.ctl.isArmed());
        TEST_ASSERT_EQUAL(k == 1, h.ctl.getStatus().rpmTelemetry);
        TEST_ASSERT_EQUAL_UINT8(k == 1 ? 4 : 1, h.ctl.getGyroFilter().motorCount());
    }
    char line[160];
    snprintf(line, sizeof(line), "rpm filter in manoeuvre: motor noise %.4f -> %.4f, gyro vibration left %.2f -> %.2f dps (model -> DShot per motor)",
             noise[0], noise[1], residual[0], residual[1]);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(noise[1] < noise[0], line);
    TEST_ASSERT_TRUE_MESSAGE(residual[1] < residual[0], line);
}


// 090: модель оборотов учится у DShot. Винт тяжелее, чем думает модель
// (kQ ×1.4 — обороты ниже): на висении поправка уходит ниже 1, а когда
// телеметрия пропала — режекторы едут по поправленной модели, ближе к правде.
void test_rpm_model_learns_from_dshot()
{
    QuadPerturb p;
    p.torque = 1.4;
    QuadHarness h(five(), p, 7);
    h.rpmTelemetry = true;
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.run(6.0);
    const float scale = h.ctl.getStatus().rpmModelScale;
    TEST_ASSERT_TRUE(h.ctl.getStatus().rpmTelemetry);
    h.rpmTelemetry = false;
    double errScaled = 0, errRaw = 0;
    long n = 0;
    h.afterStep = [&]() {
        double truth = 0;
        for (int i = 0; i < 4; ++i) truth += h.sim.rotorHz(i) / 4;
        const double est = h.ctl.getStatus().rotationHz;
        errScaled += std::fabs(est - truth);
        errRaw += std::fabs(est / scale - truth);
        ++n;
    };
    h.run(2.0);
    h.afterStep = nullptr;
    char line[160];
    snprintf(line, sizeof(line), "rpm model scale %.3f, model error without DShot %.1f -> %.1f Hz", scale, errRaw / n, errScaled / n);
    TEST_MESSAGE(line);
    TEST_ASSERT_FALSE(h.ctl.getStatus().rpmTelemetry);
    TEST_ASSERT_TRUE_MESSAGE(scale < 0.97f && scale > 0.6f, line);
    TEST_ASSERT_TRUE_MESSAGE(errScaled < 0.5 * errRaw, line);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, scale, h.ctl.getStatus().rpmModelScale);   // без DShot не учится
}


// 128, 130–132: оценка положения (GPS + акселерометр), удержание позиции
// (POS HOLD) и высоты как в BF 2025 — в симуляторе. GPS: 10 Гц, шум, фикс
// опаздывает на 0.12 с (если не сказано иное).
namespace
{
    struct HoldRun
    {
        double maxDrift = 0;     // наибольший уход от точки, м
        double lateDrift = 0;    // то же за последнюю треть
        double maxTilt = 0;
        double lateTilt = 0;
        double minH = 1e9, maxH = -1e9;
    };

    // Висеть seconds секунд, мерить уход от точки (n0, e0).
    HoldRun holdFor(QuadHarness& h, double n0, double e0, double seconds)
    {
        HoldRun r;
        const long n = std::lround(seconds * 1000.0);
        for (long i = 0; i < n; ++i)
        {
            h.step();
            const double dn = h.sim.position().x - n0, de = h.sim.position().y - e0;
            const double d = std::sqrt(dn * dn + de * de);
            r.maxDrift = std::fmax(r.maxDrift, d);
            r.maxTilt = std::fmax(r.maxTilt, h.sim.tiltDeg());
            r.minH = std::fmin(r.minH, h.sim.height());
            r.maxH = std::fmax(r.maxH, h.sim.height());
            if (i > 2 * n / 3)
            {
                r.lateDrift = std::fmax(r.lateDrift, d);
                r.lateTilt = std::fmax(r.lateTilt, h.sim.tiltDeg());
            }
        }
        return r;
    }

    // Взлететь на 10 м и включить POS HOLD; true — режим взялся.
    bool engagePosHold(QuadHarness& h)
    {
        h.gpsOn = true;
        if (!h.takeoffTo(10.0)) return false;
        h.rc.mode = CopterMode::POS_HOLD;
        h.run(2.0);
        return h.ctl.getStatus().mode == CopterMode::POS_HOLD && h.ctl.getStatus().posHoldPhase == PosHoldPhase::HOLD;
    }

    int asInt(CopterMode m) { return static_cast<int>(m); }
    int asInt(PosHoldFail f) { return static_cast<int>(f); }
    int asInt(GpsSpoof s) { return static_cast<int>(s); }
}

// Ветер 5 м/с включается разом (хуже, чем в жизни) и дует с порывами:
// коптер не уходит от точки дальше 2 м, а когда интеграл выучил ветер, —
// стоит в пределах метра. Дальний 7" и 5" в трёх вариантах физики.
void test_pos_hold_keeps_place_in_wind()
{
    struct Case
    {
        const char* build;
        QuadPerturb perturb;
        const char* name;
    };
    const Case cases[] = { { "longrange7", QuadPerturb(), "7in nominal" },
                           { "longrange7", sluggish(), "7in sluggish" },
                           { "freestyle5", QuadPerturb(), "5in nominal" },
                           { "freestyle5", twitchy(), "5in twitchy" },
                           { "hex15", QuadPerturb(), "hex15 nominal" } };
    uint32_t seed = 81;
    for (const Case& c : cases)
    {
        QuadHarness h(*findBuild(c.build), c.perturb, seed++);
        h.gpsDelayS = 0.12;
        TEST_ASSERT_TRUE_MESSAGE(engagePosHold(h), c.name);
        const double n0 = h.sim.position().x, e0 = h.sim.position().y;
        h.sim.setWind(3.0, 4.0, 1.0);   // 5 м/с, порывы
        const HoldRun r = holdFor(h, n0, e0, 30.0);
        char line[200];
        snprintf(line, sizeof(line), "POS HOLD %-13s wind 5 m/s step + gusts: drift max %.2f m, last 10 s %.2f m, tilt max %.0f deg, height %.1f..%.1f m",
                 c.name, r.maxDrift, r.lateDrift, r.maxTilt, r.minH, r.maxH);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(r.maxDrift < 2.0, line);
        TEST_ASSERT_TRUE_MESSAGE(r.lateDrift < 1.0, line);
        TEST_ASSERT_TRUE_MESSAGE(r.maxTilt < 30.0, line);
        TEST_ASSERT_TRUE_MESSAGE(r.minH > 9.0 && r.maxH < 11.0, line);
        TEST_ASSERT_EQUAL_MESSAGE(asInt(CopterMode::POS_HOLD), asInt(h.ctl.getStatus().mode), line);
    }
}

// Плохой GPS: шум положения ±1.5 м, скорости ±0.3 м/с, фикс опаздывает на
// 0.25 с. Коптер не гоняется за шумом (наклон в штиль — единицы градусов) и
// не уходит дальше 1.5 м: шум ±1.5 м усредняется акселерометром (постоянная
// времени оценки ~1 с, 10 фиксов), остаётся ~0.5 м. На скорости 5 м/с оценка
// не отстаёт от коптера, хотя сам фикс отстаёт на 1.25 м.
void test_pos_hold_with_noisy_delayed_gps()
{
    QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 86);
    h.gpsDelayS = 0.25;
    h.gpsNoisePosM = 1.5;
    h.gpsNoiseVelMs = 0.3;
    {
        // оценка знает задержку приёмника (у u-blox M10 на 10 Гц — около 0.1 с; здесь — нарочно больше)
        CopterSettings s;
        s.gpsDelayS = 0.25f;
        h.ctl.configure(h.tune, s);
    }
    TEST_ASSERT_TRUE(engagePosHold(h));
    const double n0 = h.sim.position().x, e0 = h.sim.position().y;
    const HoldRun calm = holdFor(h, n0, e0, 30.0);
    // полёт по стику: ошибка оценки положения на скорости
    h.rc.pitch = -1.0f;
    h.run(4.0);
    double estErr = 0, lag = 0;
    for (int i = 0; i < 3000; ++i)
    {
        h.step();
        const CopterStatus& st = h.ctl.getStatus();
        const double dn = st.posN - (h.sim.position().x - h.homeN), de = st.posE - (h.sim.position().y - h.homeE);
        estErr = std::fmax(estErr, std::sqrt(dn * dn + de * de));
        lag = std::fmax(lag, 0.25 * std::sqrt(h.sim.velocity().x * h.sim.velocity().x + h.sim.velocity().y * h.sim.velocity().y));
    }
    char line[220];
    snprintf(line, sizeof(line), "POS HOLD noisy GPS (pos +-1.5 m, vel +-0.3 m/s, delay 0.25 s): calm drift %.2f m, tilt max %.1f deg; at speed estimate error %.2f m (raw fix lags %.2f m + noise)",
             calm.maxDrift, calm.maxTilt, estErr, lag);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(calm.maxDrift < 1.5, line);
    TEST_ASSERT_TRUE_MESSAGE(calm.maxTilt < 8.0, line);
    TEST_ASSERT_TRUE_MESSAGE(estErr < 1.2, line);
}

// Стики: полный тангаж вперёд — летит со скоростью pos_hold_adjust_rate в
// сторону носа; отпустил — тормозит не резче половины autopilot_max_angle и
// встаёт; точка — где остановился, назад не возвращается.
void test_pos_hold_sticks_fly_and_release_stops()
{
    QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 87);
    h.gpsDelayS = 0.12;
    TEST_ASSERT_TRUE(engagePosHold(h));
    h.rc.pitch = -1.0f;
    h.run(6.0);
    const double speed = std::sqrt(h.sim.velocity().x * h.sim.velocity().x + h.sim.velocity().y * h.sim.velocity().y);
    const double side = std::fabs(h.sim.position().y);   // нос на север — летит вдоль севера
    TEST_ASSERT_EQUAL(static_cast<int>(PosHoldPhase::MOVE), static_cast<int>(h.ctl.getStatus().posHoldPhase));
    h.rc.pitch = 0.0f;
    const double nRelease = h.sim.position().x;
    double tilt = 0, far = 0;
    for (int i = 0; i < 6000; ++i)
    {
        h.step();
        tilt = std::fmax(tilt, h.sim.tiltDeg());
        far = std::fmax(far, h.sim.position().x - nRelease);
    }
    const double stop = h.sim.position().x - nRelease;
    const double rest = std::sqrt(h.sim.velocity().x * h.sim.velocity().x + h.sim.velocity().y * h.sim.velocity().y);
    char line[200];
    snprintf(line, sizeof(line), "POS HOLD full stick forward: %.1f m/s, sideways %.1f m; release -> stops in %.1f m (overshoot %.1f m), brake tilt %.0f deg, rest speed %.2f m/s",
             speed, side, stop, far - stop, tilt, rest);
    TEST_MESSAGE(line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.6, 5.0, speed, line);
    TEST_ASSERT_TRUE_MESSAGE(side < 1.5, line);
    TEST_ASSERT_TRUE_MESSAGE(stop > 1.0 && stop < 8.0, line);
    TEST_ASSERT_TRUE_MESSAGE(far - stop < 0.7, line);
    TEST_ASSERT_TRUE_MESSAGE(tilt < 35.0, line);
    TEST_ASSERT_TRUE_MESSAGE(rest < 0.3, line);
    TEST_ASSERT_EQUAL(static_cast<int>(PosHoldPhase::HOLD), static_cast<int>(h.ctl.getStatus().posHoldPhase));
}

// GPS пропал посреди удержания (ветер 4 м/с): полторы секунды коптер ещё
// держится по акселерометру, потом — ALT_HOLD (ровно, высота та же, стики —
// наклон) и причина NO_GPS для очков; моторы работают. GPS вернулся — снова
// держит место, уже новое.
void test_pos_hold_gps_loss_falls_back_to_alt_hold()
{
    QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 88);
    h.gpsDelayS = 0.12;
    TEST_ASSERT_TRUE(engagePosHold(h));
    h.sim.setWind(4.0, 0.0, 0.5);
    h.run(10.0);
    const double holdTilt = h.sim.tiltDeg();   // стоит, наклонившись на ветер
    h.gpsLost = true;
    h.run(1.0);
    TEST_ASSERT_EQUAL(asInt(CopterMode::POS_HOLD), asInt(h.ctl.getStatus().mode));   // секунду — по акселерометру
    h.run(1.5);
    TEST_ASSERT_EQUAL(asInt(CopterMode::ALT_HOLD), asInt(h.ctl.getStatus().mode));
    TEST_ASSERT_EQUAL(asInt(PosHoldFail::NO_GPS), asInt(h.ctl.getStatus().posHoldFail));
    TEST_ASSERT_EQUAL_STRING("POSHOLD NO GPS", posHoldFailText(h.ctl.getStatus().posHoldFail));
    h.stats.reset();
    h.run(6.0);
    const double levelTilt = h.sim.tiltDeg();
    const double drifted = std::sqrt(h.sim.position().x * h.sim.position().x + h.sim.position().y * h.sim.position().y);
    TEST_ASSERT_TRUE(h.ctl.isArmed());
    // пилот рулит как в ANGLE
    h.rc.roll = 0.5f;
    h.run(1.0);
    const double stickTilt = h.sim.rollDeg();
    h.rc.roll = 0.0f;
    // GPS вернулся
    h.gpsLost = false;
    h.run(4.0);
    const bool back = h.ctl.getStatus().mode == CopterMode::POS_HOLD;
    h.run(6.0);
    const double n1 = h.sim.position().x, e1 = h.sim.position().y;
    const HoldRun r = holdFor(h, n1, e1, 10.0);
    char line[240];
    snprintf(line, sizeof(line), "POS HOLD, GPS lost in 4 m/s wind: hold tilt %.1f deg -> ALT_HOLD tilt %.1f deg, height %.1f..%.1f m, drifted to %.0f m, roll by stick %.0f deg; GPS back -> POS_HOLD %s, drift %.2f m",
             holdTilt, levelTilt, h.stats.minHeight, h.stats.maxHeight, drifted, stickTilt, back ? "yes" : "NO", r.maxDrift);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(holdTilt > 2.0, line);
    TEST_ASSERT_TRUE_MESSAGE(levelTilt < 5.0, line);   // ровно с точностью до ошибки горизонта в порывах
    TEST_ASSERT_TRUE_MESSAGE(h.stats.minHeight > 9.0 && h.stats.maxHeight < 11.0, line);
    TEST_ASSERT_TRUE_MESSAGE(stickTilt > 10.0, line);
    TEST_ASSERT_TRUE_MESSAGE(back, line);
    TEST_ASSERT_EQUAL(asInt(PosHoldFail::NONE), asInt(h.ctl.getStatus().posHoldFail));
    TEST_ASSERT_TRUE_MESSAGE(r.maxDrift < 1.5, line);
}

// Подмена GPS в POS HOLD (copter/GpsIntegrity.h, P43 а): (1) увод — ложная
// точка уезжает на восток с разгоном 2 м/с² до 4 м/с, удержание тянет
// коптер за ней на запад; борт видит, что акселерометр и GPS разошлись, —
// GPS выключен, ALT_HOLD, в воздухе GPS не возвращается; (2) скачок на 500 м — сразу.
void test_pos_hold_gps_spoof_detected()
{
    for (int jump = 0; jump <= 1; ++jump)
    {
        QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 91);
        h.gpsDelayS = 0.12;
        TEST_ASSERT_TRUE(engagePosHold(h));
        h.sim.setWind(4.0, 0.0, 0.5);
        h.run(10.0);
        TEST_ASSERT_EQUAL(asInt(GpsSpoof::NONE), asInt(h.ctl.getStatus().gpsSpoof));
        const double n0 = h.sim.position().x, e0 = h.sim.position().y;
        double detectedAt = -1.0, led = -1.0;
        for (int i = 0; i < 100; ++i)
        {
            if (jump) h.spoofN = 500.0;
            else
            {
                h.spoofVe = std::fmin(4.0, 0.2 * i);
                h.spoofE += h.spoofVe * 0.1;
            }
            h.run(0.1);
            if (detectedAt < 0.0 && h.ctl.getStatus().gpsSpoof != GpsSpoof::NONE)
            {
                detectedAt = 0.1 * (i + 1);
                led = std::sqrt((h.sim.position().x - n0) * (h.sim.position().x - n0) +
                                (h.sim.position().y - e0) * (h.sim.position().y - e0));
            }
        }
        const CopterStatus& st = h.ctl.getStatus();
        char line[200];
        snprintf(line, sizeof(line), "GPS spoof (%s) in POS HOLD, 4 m/s wind: detected %s after %.1f s, led away %.1f m by then, trust %u, mode %s",
                 jump ? "jump 500 m" : "drift 2 m/s^2 to 4 m/s", gpsSpoofName(st.gpsSpoof), detectedAt, led,
                 static_cast<unsigned>(st.gpsTrust), copterModeName(st.mode));
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(detectedAt > 0.0, line);
        TEST_ASSERT_EQUAL_MESSAGE(asInt(jump ? GpsSpoof::JUMP : GpsSpoof::VELOCITY), asInt(st.gpsSpoof), line);
        TEST_ASSERT_EQUAL_MESSAGE(asInt(CopterMode::ALT_HOLD), asInt(st.mode), line);
        TEST_ASSERT_EQUAL(asInt(PosHoldFail::NO_GPS), asInt(st.posHoldFail));
        TEST_ASSERT_TRUE_MESSAGE(led < 10.0, line);
        TEST_ASSERT_TRUE(h.ctl.isArmed());
        // подменщик отпустил — в воздухе GPS всё равно выключен
        h.spoofN = h.spoofE = h.spoofVe = 0.0;
        h.run(5.0);
        TEST_ASSERT_TRUE(h.ctl.getStatus().gpsSpoof != GpsSpoof::NONE);
        TEST_ASSERT_EQUAL(asInt(CopterMode::ALT_HOLD), asInt(h.ctl.getStatus().mode));
    }
}

// Без годного фикса в POS HOLD не войти: GPS нет — ALT_HOLD и NO_GPS.
// Компаса нет: с pos_hold_without_mag = OFF — NO_HEADING; с ON — тоже, пока
// курс не выучен по полёту; после разгона и торможения (коптер стоял носом
// на 70°, гироскоп об этом не знает) режим берётся и держит место.
void test_pos_hold_needs_fix_and_heading()
{
    {
        QuadHarness h(five(), QuadPerturb(), 89);
        TEST_ASSERT_TRUE(h.takeoffTo(10.0));
        h.rc.mode = CopterMode::POS_HOLD;
        h.run(3.0);
        TEST_ASSERT_EQUAL(asInt(CopterMode::ALT_HOLD), asInt(h.ctl.getStatus().mode));
        TEST_ASSERT_EQUAL(asInt(PosHoldFail::NO_GPS), asInt(h.ctl.getStatus().posHoldFail));
        TEST_ASSERT_TRUE(h.ctl.isArmed());
        TEST_ASSERT_FLOAT_WITHIN(1.0, 10.0, h.sim.height());
    }
    for (int withoutMag = 0; withoutMag < 2; ++withoutMag)
    {
        CopterSettings s;
        s.posHold.withoutMag = withoutMag != 0;
        QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 90, EscProtocol::DSHOT600, 1000.0f, s);
        h.gpsOn = true;
        h.gpsDelayS = 0.12;
        h.compass = false;
        h.sim.setAttitude(0.0, 0.0, 70.0);
        TEST_ASSERT_TRUE(h.takeoffTo(10.0));
        h.rc.mode = CopterMode::POS_HOLD;
        h.run(3.0);
        TEST_ASSERT_EQUAL(asInt(CopterMode::ALT_HOLD), asInt(h.ctl.getStatus().mode));
        TEST_ASSERT_EQUAL(asInt(PosHoldFail::NO_HEADING), asInt(h.ctl.getStatus().posHoldFail));
        // пилот (режим — ALT_HOLD, стики — наклон) разгоняется и тормозит
        h.rc.pitch = -0.5f;
        h.run(1.5);
        h.rc.pitch = 0.4f;
        h.run(1.0);
        h.rc.pitch = 0.0f;
        h.run(4.0);
        const CopterStatus& st = h.ctl.getStatus();
        char line[200];
        snprintf(line, sizeof(line), "POS HOLD without compass (pos_hold_without_mag %s): heading %s %.0f deg (true %.0f), mode %s",
                 withoutMag ? "ON" : "OFF", headingSourceName(st.headingSource), st.heading, h.sim.yawDeg(), copterModeName(st.mode));
        TEST_MESSAGE(line);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(HeadingSource::GPS), static_cast<int>(st.headingSource), line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(15.0, h.sim.yawDeg(), st.heading, line);
        if (!withoutMag)
        {
            TEST_ASSERT_EQUAL_MESSAGE(asInt(CopterMode::ALT_HOLD), asInt(st.mode), line);
            TEST_ASSERT_EQUAL(asInt(PosHoldFail::NO_HEADING), asInt(st.posHoldFail));
            continue;
        }
        TEST_ASSERT_EQUAL_MESSAGE(asInt(CopterMode::POS_HOLD), asInt(st.mode), line);
        h.run(3.0);
        const double n0 = h.sim.position().x, e0 = h.sim.position().y;
        h.sim.setWind(0.0, 5.0, 0.5);
        const HoldRun r = holdFor(h, n0, e0, 20.0);
        snprintf(line, sizeof(line), "POS HOLD on GPS-learned heading, wind 5 m/s: drift max %.2f m, last third %.2f m", r.maxDrift, r.lateDrift);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(r.maxDrift < 2.5, line);
        TEST_ASSERT_TRUE_MESSAGE(r.lateDrift < 1.2, line);
    }
}

// Компас врёт на 100° (не откалиброван, рядом силовой провод): удержание
// раскачивается, но по первым же манёврам курс по полёту расходится с
// компасом — компасу больше не верим. pos_hold_without_mag = OFF — режим
// отдаёт управление (ALT_HOLD, NO_HEADING или FAIL), коптер не улетает;
// ON — держит место по курсу GPS.
void test_pos_hold_lying_compass()
{
    for (int withoutMag = 0; withoutMag < 2; ++withoutMag)
    {
        CopterSettings s;
        s.posHold.withoutMag = withoutMag != 0;
        QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 91, EscProtocol::DSHOT600, 1000.0f, s);
        h.gpsOn = true;
        h.gpsDelayS = 0.12;
        h.compassErrorDeg = 100.0;
        TEST_ASSERT_TRUE(h.takeoffTo(10.0));
        h.rc.mode = CopterMode::POS_HOLD;
        h.run(1.5);
        const double n0 = h.sim.position().x, e0 = h.sim.position().y;
        h.sim.setWind(4.0, 0.0, 1.0);
        const HoldRun r = holdFor(h, n0, e0, 30.0);
        const CopterStatus& st = h.ctl.getStatus();
        char line[220];
        snprintf(line, sizeof(line), "POS HOLD, compass 100 deg off, pos_hold_without_mag %s: mode %s, fail %s, heading by %s, compass rejected %s, went %.1f m, tilt max %.0f",
                 withoutMag ? "ON" : "OFF", copterModeName(st.mode), posHoldFailName(st.posHoldFail),
                 headingSourceName(st.headingSource), st.magDisagrees ? "yes" : "no", r.maxDrift, r.maxTilt);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(h.ctl.isArmed(), line);
        TEST_ASSERT_TRUE_MESSAGE(st.magDisagrees, line);
        TEST_ASSERT_TRUE_MESSAGE(r.maxTilt < 55.0, line);
        TEST_ASSERT_TRUE_MESSAGE(r.minH > 8.5 && r.maxH < 11.5, line);
        if (withoutMag)
        {
            TEST_ASSERT_EQUAL_MESSAGE(asInt(CopterMode::POS_HOLD), asInt(st.mode), line);
            TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(HeadingSource::GPS), static_cast<int>(st.headingSource), line);
            // после того как компас отброшен — стоит
            const double n1 = h.sim.position().x, e1 = h.sim.position().y;
            const HoldRun after = holdFor(h, n1, e1, 10.0);
            TEST_ASSERT_TRUE_MESSAGE(after.maxDrift < 1.5, line);
        }
        else
        {
            TEST_ASSERT_EQUAL_MESSAGE(asInt(CopterMode::ALT_HOLD), asInt(st.mode), line);
            TEST_ASSERT_TRUE_MESSAGE(st.posHoldFail != PosHoldFail::NONE, line);
        }
    }
}

// Потеря связи в ветер 5 м/с: с GPS коптер садится там, где пропала связь
// (без GPS его за время посадки уносит на десятки метров).
void test_failsafe_landing_holds_place_with_gps()
{
    double landed[2];
    for (int withGps = 0; withGps < 2; ++withGps)
    {
        QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 92);
        h.gpsOn = withGps != 0;
        h.gpsDelayS = 0.12;
        TEST_ASSERT_TRUE(h.takeoffTo(10.0));
        if (withGps)
        {
            h.rc.mode = CopterMode::POS_HOLD;
            h.run(2.0);
        }
        h.sim.setWind(0.0, 5.0, 0.5);
        h.run(8.0);
        const double n0 = h.sim.position().x, e0 = h.sim.position().y;
        h.rc.linkOk = false;
        const double t0 = h.sim.now();
        while (h.ctl.isArmed() && h.sim.now() - t0 < 60.0) h.step();
        TEST_ASSERT_FALSE(h.ctl.isArmed());
        TEST_ASSERT_TRUE(h.sim.height() < 0.1);
        landed[withGps] = std::sqrt((h.sim.position().x - n0) * (h.sim.position().x - n0) + (h.sim.position().y - e0) * (h.sim.position().y - e0));
    }
    char line[160];
    snprintf(line, sizeof(line), "link lost in 5 m/s wind, AUTO-LAND from 10 m: landed %.1f m away with GPS, %.0f m without", landed[1], landed[0]);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(landed[1] < 3.0, line);
    TEST_ASSERT_TRUE_MESSAGE(landed[0] > 15.0, line);
}

// 128: удержание высоты с шумным барометром. СКО шума 0.4 м — втрое хуже
// обычного (поток винтов, порывы на корпусе). Оценка высоты усредняет
// барометр акселерометром с постоянной ~2.5 с: из 0.4 м шума на 50 Гц
// остаётся около 0.4/sqrt(2·2.5·50/2) ≈ 0.04 м плюс медленные уходы — порог
// ±0.5 м за 20 с с запасом на порывы; тяга при этом не дёргается за шумом.
// Стик газа: вверх до упора — alt_hold_adjust_rate (5 м/с), отпустил —
// тормозит и держит, где встал (без возврата и провала).
void test_alt_hold_with_noisy_barometer()
{
    QuadHarness h(*findBuild("longrange7"), QuadPerturb(), 93);
    h.sim.setBaroNoise(0.4);
    h.sim.setWind(2.0, 1.0, 1.0);
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    const double start = h.sim.height();
    h.stats.reset();
    h.run(20.0);
    const double lo = h.stats.minHeight - start, hi = h.stats.maxHeight - start;
    const double motorNoise = h.stats.motorNoiseRms();
    // стик вверх до упора 3 с, отпустить
    h.rc.throttle = 1.0f;
    h.run(3.0);
    const double climb = h.sim.climbRate();
    h.rc.throttle = 0.5f;
    const double released = h.sim.height();
    double top = released, bottom = 1e9;
    for (int i = 0; i < 8000; ++i)
    {
        h.step();
        top = std::fmax(top, h.sim.height());
        if (i > 3000) bottom = std::fmin(bottom, h.sim.height());
    }
    char line[220];
    snprintf(line, sizeof(line), "ALT HOLD, baro noise 0.4 m rms: height %+.2f..%+.2f m in 20 s, motor noise %.3f; full throttle -> %.1f m/s, release -> stops %.1f m higher, then sinks back %.2f m",
             lo, hi, motorNoise, climb, top - released, top - bottom);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(lo > -0.5 && hi < 0.5, line);
    TEST_ASSERT_TRUE_MESSAGE(motorNoise < 0.05, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.7, 5.0, climb, line);
    TEST_ASSERT_TRUE_MESSAGE(top - released < 3.5, line);
    TEST_ASSERT_TRUE_MESSAGE(top - bottom < 0.6, line);
}

// ---------------- 042: что дали функции BF регулятора (весь каталог × 3 физики) ----------------

namespace
{
    // Регулятор без функций BF 011–026: D постоянный, без I-relax,
    // anti-gravity, throttle boost и тонкостей feedforward.
    CopterTune bfFeaturesOff(CopterTune t)
    {
        for (uint8_t a = 0; a < 3; ++a) t.pid.dMax[a] = 0.0f;
        t.pid.relax = ItermRelax::OFF;
        t.pid.antiGravityI = 0.0f;
        t.pid.antiGravityP = 0.0f;
        t.pid.throttleBoost = 0.0f;
        t.pid.ffSmoothFactor = 0.0f;
        t.pid.ffJitterFactor = 0.0f;
        t.pid.ffBoost = 0.0f;
        t.pid.ffMaxRateLimit = 0.0f;
        return t;
    }

    // Те же PID из модели, функции — с умолчаниями BF 4.5: d_max (D в покое
    // 70 %), iterm_relax RP SETPOINT 15 Гц, anti_gravity_gain 80 + p_gain 100,
    // throttle_boost 5, feedforward smooth 25 / jitter 7 / boost 15 / max_rate_limit 90.
    CopterTune bfFeaturesOn(CopterTune t)
    {
        t = withDMax(t);
        t.pid.relax = ItermRelax::RP;
        t.pid.relaxType = ItermRelaxType::SETPOINT;
        t.pid.relaxCutoffHz = 15.0f;
        t.pid.antiGravityI = 8.0f;
        t.pid.antiGravityP = 1.0f;
        t.pid.throttleBoost = 0.5f;
        t.pid.ffSmoothFactor = 0.25f;
        t.pid.ffJitterFactor = 7.0f;
        t.pid.ffBoost = 15.0f;
        t.pid.ffMaxRateLimit = 0.9f;
        return t;
    }

    struct FeatureFlight
    {
        bool ok = false;
        double noise = 0, rateRmsDps = 0;   // висение 3 с
        double punchDeg = 0;                // рывок газом до полного 0.3 с: уход наклона
        double t90 = -1, overshoot = 0;     // флик полстика за 20 мс, кадры 250 Гц
    };

    FeatureFlight flyFeatures(const BuildSpec& b, const QuadPerturb& p, const CopterTune& t, uint32_t seed)
    {
        FeatureFlight r;
        bool hoverOk = false;
        {
            QuadHarness h(b, p, seed);
            h.useTune(t);
            if (!h.takeoffTo(5.0)) return r;
            h.stats.reset();
            h.run(3.0);
            r.noise = h.stats.motorNoiseRms();
            r.rateRmsDps = h.stats.rateRms() * 57.29578;
            hoverOk = h.ctl.isArmed() && h.stats.maxTilt < 3.0 && r.rateRmsDps < 3.0;
            h.rc.mode = CopterMode::ACRO;
            h.rc.throttle = h.ctl.getStatus().hoverLearned;
            h.run(1.0);
            const double pitch0 = h.sim.pitchDeg(), roll0 = h.sim.rollDeg();
            h.rc.throttle = 1.0f;
            for (int i = 0; i < 300; ++i)
            {
                h.step();
                r.punchDeg = std::fmax(r.punchDeg, std::fmax(std::fabs(h.sim.pitchDeg() - pitch0), std::fabs(h.sim.rollDeg() - roll0)));
            }
        }
        QuadHarness h(b, p, seed + 1);
        h.useTune(t);
        if (!h.takeoffTo(20.0)) return r;
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(0.3);
        const double goal = h.tune.rates[0].apply(0.5f);
        const double t0 = h.sim.now();
        double next = t0, maxW = 0;
        while (h.sim.now() - t0 < 0.5)
        {
            const double now = h.sim.now();
            if (now >= next)
            {
                next += 1.0 / 250.0;
                ++h.rc.frame;
                h.rc.roll = static_cast<float>(std::fmin(1.0, (now - t0) / 0.02) * 0.5);
            }
            h.step();
            const double w = h.sim.rates().x * 57.29578;
            maxW = std::fmax(maxW, w);
            if (r.t90 < 0 && w > 0.9 * goal) r.t90 = h.sim.now() - t0;
        }
        r.overshoot = maxW / goal - 1.0;
        r.ok = hoverOk && h.ctl.isArmed() && r.t90 > 0 && r.t90 < 0.2;
        return r;
    }
}

// 042: каждая сборка каталога × 3 физики (как в описании, вялая, резвая),
// PID из модели — с функциями BF и без. Строки BF042 — таблица «Что дали
// функции BF» в docs/COPTER.md. С функциями летает всё; шум моторов на
// висении не растёт (d_max снижает D в покое), рывок газом не валит нос
// сильнее, флик не медленнее.
void test_bf_regulator_features_on_every_build()
{
    static int failed;
    failed = 0;
    static const char* names[] = { "nominal", "sluggish", "twitchy" };
    forEachBuild([](const BuildSpec& b) {
        const QuadPerturb ps[] = { QuadPerturb(), sluggish(), twitchy() };
        const CopterTune base = deriveTune(b, 1000.0f, EscProtocol::DSHOT600);
        const CopterTune off = bfFeaturesOff(base), on = bfFeaturesOn(base);
        for (uint8_t k = 0; k < 3; ++k)
        {
            const FeatureFlight a = flyFeatures(b, ps[k], off, 61 + k);
            const FeatureFlight c = flyFeatures(b, ps[k], on, 61 + k);
            char line[260];
            snprintf(line, sizeof(line),
                     "BF042 %-12s %-8s | noise %.4f -> %.4f | rate rms %.2f -> %.2f dps | punch %.2f -> %.2f deg | flick t90 %.0f -> %.0f ms, overshoot %.1f -> %.1f %%",
                     b.id, names[k], a.noise, c.noise, a.rateRmsDps, c.rateRmsDps, a.punchDeg, c.punchDeg, a.t90 * 1000.0,
                     c.t90 * 1000.0, a.overshoot * 100.0, c.overshoot * 100.0);
            // вуп с моторами быстрее половины частоты цикла (WARN_VIBRATION_ALIAS)
            // на 1 кГц видит вибрацию в полосе контура — D от d_max растёт от шума
            const bool alias = base.warnings & WARN_VIBRATION_ALIAS;
            const bool good = c.ok && (alias || c.noise <= a.noise * 1.02 + 1e-4) &&
                              (k == 0 ? c.punchDeg < 1.0 : c.punchDeg < a.punchDeg) && c.t90 < a.t90 + 0.003 &&
                              c.overshoot < a.overshoot + 0.05;
            TEST_MESSAGE(good ? line : (std::string("BAD ") + line).c_str());
            if (!good) ++failed;
        }
    });
    TEST_ASSERT_EQUAL_INT(0, failed);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_every_build_hovers_on_its_derived_tune);
    RUN_TEST(test_every_build_hovers_when_physics_differs_from_the_description);
    RUN_TEST(test_every_build_lands_and_disarms_on_link_loss);
    RUN_TEST(test_every_build_follows_angle_stick);
    RUN_TEST(test_every_build_tracks_acro_rate);
    RUN_TEST(test_airmode_keeps_control_at_zero_throttle);
    RUN_TEST(test_airmode_starts_after_throttle_threshold);
    RUN_TEST(test_ez_landing_touchdown_without_bounce);
    RUN_TEST(test_launch_control_holds_tilt_then_launches);
    RUN_TEST(test_yaw_rate_follows_stick_and_stays_level);
    RUN_TEST(test_hover_in_gusty_wind);
    RUN_TEST(test_failsafe_without_barometer);
    RUN_TEST(test_pwm_esc_flies);
    RUN_TEST(test_whoop_on_fast_loop);
    RUN_TEST(test_dyn_notch_catches_vibration_the_rpm_model_misses);
    RUN_TEST(test_rpm_filter_per_motor_from_dshot_in_manoeuvre);
    RUN_TEST(test_rpm_model_learns_from_dshot);
    RUN_TEST(test_dynamic_idle_holds_min_rpm_on_throttle_cut);
    RUN_TEST(test_link_glitch_and_recovery_in_flight);
    RUN_TEST(test_upside_down_crash_disarms);
    RUN_TEST(test_turtle_mode_flips_back_over);
    RUN_TEST(test_motor_failure_in_angle_mode_disarms);
    RUN_TEST(test_motor_test_spins_one_motor);
    RUN_TEST(test_battery_sag_is_seen);
    RUN_TEST(test_d_max_keeps_hover_quiet);
    RUN_TEST(test_bf_iterm_relax_on_sluggish_hexa);
    RUN_TEST(test_anti_gravity_holds_the_nose_on_punch);
    RUN_TEST(test_throttle_boost_speeds_up_climb);
    RUN_TEST(test_full_throttle_without_oscillation_in_every_tpa_mode);
    RUN_TEST(test_tpa_low_on_zero_throttle_airmode);
    RUN_TEST(test_betaflight_defaults_on_our_loop);
    RUN_TEST(test_feedforward_settings_on_a_real_flick);
    RUN_TEST(test_feedforward_max_rate_limit_on_a_full_stick_flip);
    RUN_TEST(test_bf_regulator_features_on_every_build);
    RUN_TEST(test_integrated_yaw_on_big_hexa);
    RUN_TEST(test_absolute_control_holds_attitude_against_torque);
    RUN_TEST(test_crash_recovery_levels_after_a_hit);
    RUN_TEST(test_crash_recovery_with_a_saturated_gyro);
    RUN_TEST(test_yaw_spin_recovery_stops_a_saturated_spin);
    RUN_TEST(test_yaw_spin_recovery_ignores_saturation_junk);
    RUN_TEST(test_runaway_takeoff_disarms_on_wrong_motor_wiring);
    RUN_TEST(test_failsafe_stage1_holds_altitude_then_stage2_descends);
    RUN_TEST(test_failsafe_stage1_rxfail_hold);
    RUN_TEST(test_failsafe_drop_procedure);
    RUN_TEST(test_failsafe_switch_modes);
    RUN_TEST(test_failsafe_rescue_returns_control_by_stick);
    RUN_TEST(test_angle_feedforward_tilts_faster);
    RUN_TEST(test_horizon_flips_and_levels);
    RUN_TEST(test_acro_trainer_limits_tilt);
    RUN_TEST(test_gps_rescue_returns_home_in_crosswind);
    RUN_TEST(test_gps_rescue_without_compass_aligns_heading);
    RUN_TEST(test_gps_rescue_with_lying_compass);
    RUN_TEST(test_gps_rescue_gps_loss_lands_in_place);
    RUN_TEST(test_dead_reckon_home_when_gps_jammed_or_spoofed);
    RUN_TEST(test_dead_reckon_hands_back_to_gps);
    RUN_TEST(test_gps_rescue_switch_and_arming_check);
    RUN_TEST(test_level_trim_shifts_angle_hover);
    RUN_TEST(test_training_ceiling_holds_full_throttle);
    RUN_TEST(test_training_fence_turns_back);
    RUN_TEST(test_geofence_acro_pilot_cannot_leave);
    RUN_TEST(test_geofence_ceiling_in_acro);
    RUN_TEST(test_geofence_gps_lost_takes_control);
    RUN_TEST(test_geofence_blocks_arming_without_gps);
    RUN_TEST(test_gps_rescue_breadcrumbs_out_of_canopy);
    RUN_TEST(test_gps_rescue_goes_around_nofly_zone);
    RUN_TEST(test_horizon_and_altitude_hold_in_long_acceleration);
    RUN_TEST(test_training_speed_limit);
    RUN_TEST(test_pos_hold_keeps_place_in_wind);
    RUN_TEST(test_pos_hold_with_noisy_delayed_gps);
    RUN_TEST(test_pos_hold_sticks_fly_and_release_stops);
    RUN_TEST(test_pos_hold_gps_loss_falls_back_to_alt_hold);
    RUN_TEST(test_pos_hold_gps_spoof_detected);
    RUN_TEST(test_pos_hold_needs_fix_and_heading);
    RUN_TEST(test_pos_hold_lying_compass);
    RUN_TEST(test_failsafe_landing_holds_place_with_gps);
    RUN_TEST(test_alt_hold_with_noisy_barometer);
    RUN_TEST(test_attitude_ekf_learns_gyro_bias);
    RUN_TEST(test_attitude_ekf_against_mahony_in_manoeuvres);
    RUN_TEST(test_attitude_ekf_learns_rotor_drag_in_flight);
    RUN_TEST(test_attitude_ekf_consistency_nees_nis);
    RUN_TEST(test_attitude_ekf_compass_yaw_bias_and_interference_in_flight);
    RUN_TEST(test_failsafe_landing_on_a_slope_disarms);
    RUN_TEST(test_chirp_runs_by_switch_in_flight_only);
    RUN_TEST(test_chirp_identifies_the_plant_within_10_percent);
    RUN_TEST(test_autotune_by_chirp_on_every_build);
    return UNITY_END();
}
