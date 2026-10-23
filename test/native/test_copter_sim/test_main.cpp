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
    return UNITY_END();
}
