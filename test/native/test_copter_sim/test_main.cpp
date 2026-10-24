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

using namespace copter;

void setUp() {}
void tearDown() {}

namespace
{
    QuadPerturb sluggish()
    {
        QuadPerturb p;
        p.thrust = 0.8;
        p.resistance = 1.3;
        p.rotorInertia = 1.3;
        p.bodyInertia = 1.3;
        p.mass = 1.2;
        p.motorSpread = 0.05;
        p.cgX = 0.08;
        return p;
    }

    QuadPerturb twitchy()
    {
        QuadPerturb p;
        p.thrust = 1.2;
        p.torque = 1.2;
        p.resistance = 0.75;
        p.rotorInertia = 0.7;
        p.bodyInertia = 0.7;
        p.mass = 0.85;
        p.motorSpread = 0.05;
        p.cgY = 0.05;
        return p;
    }

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
    TEST_ASSERT_TRUE(h.stats.minHeight > 4.3 && h.stats.maxHeight < 5.7);
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
    h.run(0.5);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterState::FAILSAFE), static_cast<int>(h.ctl.getStatus().state));
    h.run(1.0);
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

// ---------------- тонкости PID (PLAN_COPTER 011–015) ----------------

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

// ---------------- газ и TPA (PLAN_COPTER 016–020) ----------------

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

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_every_build_hovers_on_its_derived_tune);
    RUN_TEST(test_every_build_hovers_when_physics_differs_from_the_description);
    RUN_TEST(test_every_build_lands_and_disarms_on_link_loss);
    RUN_TEST(test_every_build_follows_angle_stick);
    RUN_TEST(test_every_build_tracks_acro_rate);
    RUN_TEST(test_airmode_keeps_control_at_zero_throttle);
    RUN_TEST(test_yaw_rate_follows_stick_and_stays_level);
    RUN_TEST(test_hover_in_gusty_wind);
    RUN_TEST(test_failsafe_without_barometer);
    RUN_TEST(test_pwm_esc_flies);
    RUN_TEST(test_whoop_on_fast_loop);
    RUN_TEST(test_link_glitch_and_recovery_in_flight);
    RUN_TEST(test_upside_down_crash_disarms);
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
    return UNITY_END();
}
