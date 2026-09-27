// ============================================================
// Замкнутые полёты: прошивка управления целиком + модель самолёта
// (helpers/PlaneSim.h, helpers/SimHarness.h). Каждый режим и
// функция проверяются в полёте: не "какую команду выдал", а "что
// в итоге сделал самолёт" — выровнялся, вернулся домой, сел, набрал
// высоту в термике.
//
// Траектории для графиков README:
//   OPENPLANE_SIM_DIR=docs/images/sim pio test -e native -f native/test_sim
//   python3 tools/plot_sim.py
//
// Запуск: pio test -e native -f native/test_sim
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <cmath>

#include "helpers/SimHarness.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    // Тумблеры стенда: SwC — MANUAL / STABILIZE / CRUISE, SwD — RTH,
    // SwB — геозабор, крутилки — сила стабилизации и скорость круиза.
    // Остальные режимы — как с дашборда (autopilot.setMode()).
    constexpr Binding TABLE[] = {
        Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
        Bind::mode   (Channels::SWD, MODE_RTH),
        Bind::feature(Channels::SWB, Feature::GEOFENCE),
        Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
        Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
    };

    constexpr Binding TRIM_TABLE[] = {
        Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
        Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
    };

    using Sim = SimHarness<5>;

    constexpr uint16_t SWITCH_UP = 1000, SWITCH_MID = 1500, SWITCH_DOWN = 2000;

    // Взлёт "в воздухе": ровный полёт на высоте, режим MANUAL, заармлен.
    void startCruising(Sim& sim, double height, double speed, double headingDeg, uint16_t throttle = 1400)
    {
        sim.plane.setCruise(height, speed, headingDeg / 57.29578);
        sim.rc.set(Channels::SWC, SWITCH_UP);
        sim.arm();
        sim.rc.set(Channels::THROTTLE, throttle);
    }
}

void test_sim_stabilize_recovers_from_upset()
{
    Sim sim(TABLE, false, false, "stabilize");
    startCruising(sim, 60, 15, 0);
    sim.plane.s.roll = 50 / 57.29578;    // порыв кладёт на крыло
    sim.plane.s.pitch = -20 / 57.29578;
    sim.rc.set(Channels::SWC, SWITCH_MID);
    sim.run(3.0);
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, fabs(sim.rollDeg()));
    sim.run(3.0);
    TEST_ASSERT_LESS_THAN_FLOAT(2.0f, fabs(sim.rollDeg()));
    TEST_ASSERT_LESS_THAN_FLOAT(10.0f, fabs(sim.pitchDeg()));
    TEST_ASSERT_GREATER_THAN_FLOAT(40.0f, sim.plane.s.height);
}

void test_sim_cruise_holds_course_and_altitude_in_crosswind()
{
    Sim sim(TABLE, false, false, "cruise");
    startCruising(sim, 50, 15, 90);
    sim.plane.windNorth = 4;   // боковой ветер 4 м/с
    sim.rc.set(Channels::SWC, SWITCH_DOWN);
    double worstCourse = 0, worstAlt = 0;
    for (int i = 0; i < 30; ++i)
    {
        sim.run(1.0);
        if (i >= 10)
        {
            worstCourse = std::max(worstCourse, static_cast<double>(fabs(Geo::wrap180(static_cast<float>(sim.plane.groundCourse() * 57.29578 - 90)))));
            worstAlt = std::max(worstAlt, fabs(sim.plane.s.height - 50));
        }
    }
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, worstCourse);
    TEST_ASSERT_LESS_THAN_FLOAT(3.0f, worstAlt);
}

void test_sim_loiter_holds_circle()
{
    Sim sim(TABLE, false, false, "loiter");
    startCruising(sim, 50, 15, 0);
    sim.run(1.0);
    const double cn = sim.plane.s.north, ce = sim.plane.s.east;
    sim.autopilot.setMode(MODE_LOITER);
    sim.run(20);
    double minD = 1e9, maxD = 0, worstAlt = 0;
    for (int i = 0; i < 400; ++i)
    {
        sim.run(0.1);
        const double d = sim.distanceFrom(cn, ce);
        minD = std::min(minD, d);
        maxD = std::max(maxD, d);
        worstAlt = std::max(worstAlt, fabs(sim.plane.s.height - 50));
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(40.0f, minD);   // круг 50 м
    TEST_ASSERT_LESS_THAN_FLOAT(60.0f, maxD);
    TEST_ASSERT_LESS_THAN_FLOAT(3.0f, worstAlt);
}

void test_sim_rth_returns_and_circles_home()
{
    Sim sim(TABLE, false, false, "rth");
    startCruising(sim, 20, 15, 45);
    sim.run(40);   // улетели
    TEST_ASSERT_GREATER_THAN_FLOAT(500.0f, sim.distanceHome());
    sim.rc.set(Channels::SWD, SWITCH_DOWN);
    TEST_ASSERT_TRUE(sim.runUntil([&] { return sim.distanceHome() < 80; }, 120));
    double farthest = 0, lowest = 1e9;
    for (int i = 0; i < 400; ++i)   // 40 с над домом
    {
        sim.run(0.1);
        farthest = std::max(farthest, sim.distanceHome());
        lowest = std::min(lowest, sim.plane.s.height);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(110.0f, farthest);
    TEST_ASSERT_GREATER_THAN_FLOAT(Config::RTH_ALTITUDE_M - 5, lowest);
    TEST_ASSERT_EQUAL(MODE_RTH, sim.autopilot.getMode());
}


void test_sim_failsafe_returns_home_or_glides()
{
    Sim sim(TABLE, false, false, "failsafe_rth");
    startCruising(sim, 30, 15, 0);
    sim.run(30);
    const double away = sim.distanceHome();
    sim.linkUp = false;   // пульт выключили
    TEST_ASSERT_GREATER_THAN_FLOAT(300.0f, away);
    TEST_ASSERT_TRUE(sim.runUntil([&] { return sim.distanceHome() < 80; }, 120));
    TEST_ASSERT_EQUAL_STRING("FAILSAFE_RTH", sim.autopilot.getModeName());
    TEST_ASSERT_GREATER_THAN_FLOAT(0.2f, sim.lastControl().throttle);   // мотор работает
    sim.linkUp = true;                                                   // связь вернулась — снова режим пилота
    sim.run(0.5);
    TEST_ASSERT_FALSE(sim.autopilot.isFailsafeActive());

    Sim glide(TABLE, false, false, "failsafe_glide");
    startCruising(glide, 80, 15, 0);
    glide.gpsWorking = false;
    glide.run(1);
    glide.linkUp = false;
    double worstRoll = 0;
    for (int i = 0; i < 20; ++i)
    {
        glide.run(1);
        if (i > 2) worstRoll = std::max(worstRoll, fabs(glide.rollDeg()));
    }
    TEST_ASSERT_EQUAL_STRING("FAILSAFE_GLIDE", glide.autopilot.getModeName());
    TEST_ASSERT_LESS_THAN_FLOAT(3.0f, worstRoll);                       // крылья ровно
    TEST_ASSERT_EQUAL_FLOAT(0.0f, glide.lastControl().throttle);         // мотор выключен
    TEST_ASSERT_LESS_THAN_FLOAT(75.0f, glide.plane.s.height);            // снижается
    TEST_ASSERT_GREATER_THAN_FLOAT(9.0f, glide.plane.s.speed);           // не свалился
}

void test_sim_geofence_brings_plane_back()
{
    Sim sim(TABLE, false, false, "geofence");
    startCruising(sim, 40, 15, 90);
    sim.rc.set(Channels::SWB, SWITCH_DOWN);    // геозабор включён
    sim.rc.set(Channels::SWC, SWITCH_DOWN);    // CRUISE на восток — прочь от дома
    double farthest = 0;
    for (int i = 0; i < 1200; ++i)
    {
        sim.run(0.1);
        farthest = std::max(farthest, sim.distanceHome());
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(Config::FENCE_RADIUS_M, farthest);          // вылетел за забор...
    TEST_ASSERT_LESS_THAN_FLOAT(Config::FENCE_RADIUS_M + 100, farthest);       // ...но недалеко
    TEST_ASSERT_LESS_THAN_FLOAT(150.0f, sim.distanceHome());                   // и вернулся
    TEST_ASSERT_EQUAL(MODE_RTH, sim.autopilot.getMode());
}

void test_sim_auto_takeoff_from_runway()
{
    Sim sim(TABLE, false, false, "takeoff");
    sim.rc.set(Channels::SWC, SWITCH_UP);
    sim.arm();
    sim.autopilot.setMode(MODE_AUTO_TAKEOFF);
    sim.rc.set(Channels::THROTTLE, 1600);   // газ вверх — старт программы
    double worstRoll = 0;
    for (int i = 0; i < 150; ++i)
    {
        sim.run(0.1);
        worstRoll = std::max(worstRoll, fabs(sim.rollDeg()));
    }
    TEST_ASSERT_FALSE(sim.plane.s.onGround);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, sim.plane.s.height);
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, worstRoll);
    TEST_ASSERT_EQUAL(0, sim.plane.touchdowns);
}

void test_sim_hand_launch()
{
    Sim sim(TABLE, false, false, "launch");
    sim.frozen = true;
    sim.plane.s.onGround = false;
    sim.plane.s.height = 1.8;
    sim.rc.set(Channels::SWC, SWITCH_UP);
    sim.arm();
    sim.autopilot.setMode(MODE_LAUNCH);
    sim.rc.set(Channels::THROTTLE, 1700);   // взвести
    sim.run(1.0);
    TEST_ASSERT_EQUAL(LaunchController::State::READY, sim.autopilot.getLaunchState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.lastControl().throttle);   // в руке мотор стоит
    // Бросок: 0.15 с толчка 3g, самолёт уходит из руки на 9 м/с.
    sim.throw_(3.0, 0.15);
    sim.run(0.15);
    sim.frozen = false;
    sim.plane.s.speed = 9;
    sim.plane.s.pitch = 10 / 57.29578;
    sim.plane.s.gamma = 5 / 57.29578;
    double minHeight = 99, minSpeed = 99;
    for (int i = 0; i < 150; ++i)
    {
        sim.run(0.1);
        minHeight = std::min(minHeight, sim.plane.s.height);
        minSpeed = std::min(minSpeed, sim.plane.s.speed);
    }
    TEST_ASSERT_EQUAL(LaunchController::State::DONE, sim.autopilot.getLaunchState());
    TEST_ASSERT_EQUAL(0, sim.plane.touchdowns);                 // не коснулся земли
    TEST_ASSERT_GREATER_THAN_FLOAT(1.0f, minHeight);
    TEST_ASSERT_GREATER_THAN_FLOAT(PlaneSim::V_REF * 0.5, minSpeed);
    TEST_ASSERT_GREATER_THAN_FLOAT(15.0f, sim.plane.s.height);
}

void test_sim_auto_land()
{
    Sim sim(TABLE, false, false, "land");
    startCruising(sim, 40, 15, 0);
    sim.autopilot.setMode(MODE_AUTO_LAND);
    TEST_ASSERT_TRUE(sim.runUntil([&] { return sim.plane.touchdowns > 0; }, 120));
    TEST_ASSERT_GREATER_THAN_FLOAT(-1.5f, sim.plane.touchdownVerticalSpeed);   // мягко
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, fabs(sim.plane.touchdownRoll * 57.3));
    TEST_ASSERT_GREATER_THAN_FLOAT(-5.0f, sim.plane.touchdownPitch * 57.3);    // не носом в землю
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.lastControl().throttle);
}

void test_sim_soaring_climbs_in_thermal()
{
    Sim sim(TABLE, false, false, "soaring");
    // Термик в 250 м к северу: ядро 3 м/с, радиус ~70 м.
    sim.plane.updraft = [](double n, double e, double) {
        const double d2 = (n - 250) * (n - 250) + e * e;
        return 3.0 * exp(-d2 / (70.0 * 70.0));
    };
    startCruising(sim, 80, 14, 0);
    sim.autopilot.setMode(MODE_SOARING);
    double maxH = 0;
    bool thermal = false;
    for (int i = 0; i < 1800; ++i)
    {
        sim.run(0.1);
        maxH = std::max(maxH, sim.plane.s.height);
        thermal = thermal || sim.autopilot.getSoaringState() == SoaringController::State::THERMAL;
    }
    TEST_ASSERT_TRUE(thermal);
    TEST_ASSERT_GREATER_THAN_FLOAT(130.0f, maxH);   // +50 м без мотора
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.lastControl().throttle);
}

void test_sim_rescue_from_spiral_dive()
{
    Sim sim(TABLE, false, false, "rescue");
    startCruising(sim, 150, 25, 0);
    sim.plane.s.roll = 70 / 57.29578;
    sim.plane.s.pitch = -40 / 57.29578;
    sim.plane.s.gamma = -35 / 57.29578;
    sim.autopilot.setMode(MODE_RESCUE);
    sim.run(4);
    TEST_ASSERT_LESS_THAN_FLOAT(10.0f, fabs(sim.rollDeg()));
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, sim.plane.s.climbRate);
    TEST_ASSERT_GREATER_THAN_FLOAT(100.0f, sim.plane.s.height);
}

void test_sim_airspeed_hold_and_stall_protection()
{
    Sim sim(TABLE, true, false, "stall");
    startCruising(sim, 60, 9, 0);   // медленно — у сваливания
    sim.rc.set(Channels::SWC, SWITCH_DOWN);
    double minSpeed = 99;
    for (int i = 0; i < 300; ++i)
    {
        sim.run(0.1);
        minSpeed = std::min(minSpeed, sim.plane.s.speed);
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(Config::STALL_SPEED_MS, minSpeed);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, Config::CRUISE_AIRSPEED_MS, sim.plane.s.speed);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 60.0f, sim.plane.s.height);
}

void test_sim_real_pitot_in_the_loop()
{
    Sim sim(TABLE, true, true, "pitot");
    sim.run(1.5);   // на земле, трубка набирает ноль (как при включении на поле)
    TEST_ASSERT_FALSE(sim.pitot.isZeroing());
    startCruising(sim, 50, 15, 0);
    sim.rc.set(Channels::SWC, SWITCH_DOWN);
    double worst = 0, sum = 0;
    int n = 0;
    for (int i = 0; i < 600; ++i)
    {
        sim.run(0.1);
        if (i > 200)
        {
            const double err = fabs(sim.plane.s.speed - Config::CRUISE_AIRSPEED_MS);
            worst = std::max(worst, err);
            sum += err;
            n++;
        }
    }
    const AirspeedData& a = sim.autopilot.getAirspeedSensor()->getAirspeedData();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, sim.plane.s.speed, a.trueMs);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, worst);
    TEST_ASSERT_LESS_THAN_FLOAT(0.2f, sum / n);
}

void test_sim_auto_trim_learns_crooked_plane()
{
    SimHarness<2> sim(TRIM_TABLE, false, false, "autotrim");
    sim.plane.setCruise(60, 15, 0);
    sim.plane.aileronBias = 0.06;   // самолёт тянет вправо
    sim.rc.set(Channels::SWC, SWITCH_UP);
    sim.arm();
    sim.rc.set(Channels::THROTTLE, 1350);
    sim.rc.set(Channels::SWB, SWITCH_DOWN);   // автотриммер
    // Пилот держит крылья ровно — P-регулятор вместо человека.
    for (int i = 0; i < 10000; ++i)
    {
        const int roll = static_cast<int>(-8.0 * sim.rollDeg());
        sim.sticks(std::clamp(roll, -300, 300), 0);
        sim.tick();
    }
    sim.rc.set(Channels::SWB, SWITCH_UP);
    sim.sticks(0, 0);   // руки с ручки
    double worst = 0;
    for (int i = 0; i < 2500; ++i)
    {
        sim.tick();
        worst = std::max(worst, fabs(sim.rollDeg()));
    }
    TEST_ASSERT_LESS_THAN_FLOAT(-20.0f, sim.autopilot.getAutoTrim().getRoll());   // триммер против перекоса
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, worst);                                      // летит ровно сам
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_sim_stabilize_recovers_from_upset);
    RUN_TEST(test_sim_cruise_holds_course_and_altitude_in_crosswind);
    RUN_TEST(test_sim_loiter_holds_circle);
    RUN_TEST(test_sim_rth_returns_and_circles_home);
    RUN_TEST(test_sim_failsafe_returns_home_or_glides);
    RUN_TEST(test_sim_geofence_brings_plane_back);
    RUN_TEST(test_sim_auto_takeoff_from_runway);
    RUN_TEST(test_sim_hand_launch);
    RUN_TEST(test_sim_auto_land);
    RUN_TEST(test_sim_soaring_climbs_in_thermal);
    RUN_TEST(test_sim_rescue_from_spiral_dive);
    RUN_TEST(test_sim_airspeed_hold_and_stall_protection);
    RUN_TEST(test_sim_real_pitot_in_the_loop);
    RUN_TEST(test_sim_auto_trim_learns_crooked_plane);
    return UNITY_END();
}
