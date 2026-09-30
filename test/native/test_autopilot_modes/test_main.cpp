// ============================================================
// Режимы и функции автопилота по отдельности (без модели самолёта):
//   • привязки тумблеров и крутилок (PilotSwitches, проверки таблицы);
//   • навигационная математика и наведение по окружности;
//   • высота/скорость, автотриммер, запуск с руки, парение, пищалка;
//   • каждый режим Autopilot с фейковыми датчиками;
//   • функции уровня FlightController: мотор-килл, груз, камера,
//     чувствительность стиков, закрылки/тормоз, пищалка, failsafe RTH.
// Замкнутые полёты с моделью самолёта — test_sim.
//
// Запуск: pio test -e native -f native/test_autopilot_modes
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <cmath>

#include "autopilot/AltitudeSpeedController.h"
#include "autopilot/AutoTrim.h"
#include "autopilot/Autopilot.h"
#include "autopilot/ControlBinding.h"
#include "autopilot/LaunchController.h"
#include "autopilot/Navigation.h"
#include "autopilot/PilotSwitches.h"
#include "autopilot/SoaringController.h"
#include "control/ArmingManager.h"
#include "control/Beeper.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    const GeoPoint HOME = [] {
        GeoPoint p;
        p.lat = 55.75;
        p.lon = 37.61;
        return p;
    }();

    RcChannelState rcWith(uint8_t channel, uint16_t us)
    {
        RcChannelState rc;
        rc.set(channel, us);
        return rc;
    }

    PilotInputs with(Feature f)
    {
        PilotInputs in;
        in.features[static_cast<uint8_t>(f)] = true;
        return in;
    }

    PilotInputs withKnob(Knob k, float value)
    {
        PilotInputs in;
        in.knobs[static_cast<uint8_t>(k)] = value;
        in.knobBound[static_cast<uint8_t>(k)] = true;
        return in;
    }

    struct Rig
    {
        FakeImu imu;
        FakeBaro baro;
        FakeMag mag;
        FakeGps gps;
        FakeAirspeed airspeed;
        Autopilot autopilot{ &imu, &baro, &mag, &gps, &airspeed };

        Rig()
        {
            airspeed.available = false;
            mag.available = false;
            gps.available = false;
            baro.available = true;
        }

        void goodGpsAt(const GeoPoint& p, float courseDeg, float speedMs = 15.0f)
        {
            gps.available = true;
            gps.data.fixType = 3;
            gps.data.numSatellites = 10;
            gps.data.horizontalAccuracy = 1.0f;
            gps.data.latitude = p.lat;
            gps.data.longitude = p.lon;
            gps.data.heading = courseDeg;
            gps.data.groundSpeed = speedMs;
        }

        void step(bool armed = true, bool linkLost = false, uint16_t throttle = 1000,
                  const ControlCommand& sticks = ControlCommand())
        {
            fake::advanceMs(2);
            autopilot.update(armed, linkLost, throttle, sticks);
        }

        // Заармить с хорошим GPS в точке HOME — дом записывается.
        void armAtHome(float courseDeg = 0)
        {
            goodGpsAt(HOME, courseDeg);
            step(false);
            step(true);
        }
    };
}

// ------------------------------------------------------------
// Привязки тумблеров и крутилок
// ------------------------------------------------------------

void test_switches_modes_overrides_features_and_knobs()
{
    static constexpr Binding TABLE[] = {
        Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
        Bind::mode   (Channels::SWD, MODE_RTH),
        Bind::mode   (Channels::SWB, MODE_RESCUE),
        Bind::feature(Channels::VRA, Feature::BEEPER),
        Bind::knob   (Channels::VRB, Knob::MAX_BANK),
    };
    Autopilot autopilot;
    PilotSwitches switches(&autopilot, TABLE);
    TEST_ASSERT_EQUAL(5u, switches.size());

    RcChannelState rc;
    rc.set(Channels::SWC, 1500);
    switches.update(rc);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());

    rc.set(Channels::SWD, 2000);   // оба "поверх": выше в таблице — RTH
    rc.set(Channels::SWB, 2000);
    switches.update(rc);
    TEST_ASSERT_EQUAL(MODE_RTH, autopilot.getMode());

    rc.set(Channels::SWD, 1000);
    switches.update(rc);
    TEST_ASSERT_EQUAL(MODE_RESCUE, autopilot.getMode());

    rc.set(Channels::SWB, 1000);
    switches.update(rc);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());   // снова с тумблера режимов

    rc.set(Channels::SWC, 2000);
    switches.update(rc);
    TEST_ASSERT_EQUAL(MODE_CRUISE, autopilot.getMode());

    TEST_ASSERT_FALSE(switches.getInputs().has(Feature::BEEPER));
    rc.set(Channels::VRA, 1800);
    rc.set(Channels::VRB, 2000);
    switches.update(rc);
    TEST_ASSERT_TRUE(switches.getInputs().has(Feature::BEEPER));
    TEST_ASSERT_TRUE(autopilot.getInputs().has(Feature::BEEPER));   // передано автопилоту
    TEST_ASSERT_TRUE(switches.getInputs().isBound(Knob::MAX_BANK));
    TEST_ASSERT_FALSE(switches.getInputs().isBound(Knob::STAB_GAIN));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, switches.getInputs().knob(Knob::MAX_BANK));

    rc.set(Channels::VRB, 1000);
    switches.update(rc);
    const PilotInputs& in = switches.getInputs();
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, in.knob(Knob::MAX_BANK));
    TEST_ASSERT_EQUAL_FLOAT(15.0f, in.knobValue(Knob::MAX_BANK, 15, 45, 60));
    rc.set(Channels::VRB, 1750);
    switches.update(rc);
    TEST_ASSERT_EQUAL_FLOAT(52.5f, switches.getInputs().knobValue(Knob::MAX_BANK, 15, 45, 60));
    rc.set(Channels::VRB, 2600);   // вне диапазона — ограничено
    switches.update(rc);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, switches.getInputs().knob(Knob::MAX_BANK));
}

void test_two_position_mode_switch_and_zones()
{
    static constexpr Binding TABLE[] = {
        Bind::modes(Channels::SWD, MODE_MANUAL, MODE_LOITER),
    };
    Autopilot autopilot;
    PilotSwitches switches(&autopilot, TABLE);
    switches.update(rcWith(Channels::SWD, 1499));
    TEST_ASSERT_EQUAL(MODE_MANUAL, autopilot.getMode());
    switches.update(rcWith(Channels::SWD, 1500));
    TEST_ASSERT_EQUAL(MODE_LOITER, autopilot.getMode());

    TEST_ASSERT_EQUAL(0, PilotSwitches::zoneOf(1249, 3));
    TEST_ASSERT_EQUAL(1, PilotSwitches::zoneOf(1250, 3));
    TEST_ASSERT_EQUAL(2, PilotSwitches::zoneOf(1750, 3));
}

void test_switches_without_mode_bindings_leave_mode_alone()
{
    static constexpr Binding TABLE[] = {
        Bind::feature(Channels::SWB, Feature::FLAPS),
    };
    Autopilot autopilot;
    autopilot.setMode(MODE_ALT_HOLD);
    PilotSwitches switches(&autopilot, TABLE);
    switches.update(rcWith(Channels::SWB, 2000));
    TEST_ASSERT_EQUAL(MODE_ALT_HOLD, autopilot.getMode());
    TEST_ASSERT_TRUE(switches.getInputs().has(Feature::FLAPS));
}

void test_binding_table_checks_catch_mistakes()
{
    static constexpr Binding STICK[] = { Bind::feature(Channels::THROTTLE, Feature::FLAPS) };
    static constexpr Binding ARM[] = { Bind::feature(Channels::ARM, Feature::FLAPS) };
    static constexpr Binding OUT_OF_RANGE[] = { Bind::knob(12, Knob::RATES) };
    static constexpr Binding DUPLICATE[] = {
        Bind::feature(Channels::SWB, Feature::FLAPS),
        Bind::knob(Channels::SWB, Knob::RATES),
    };
    static constexpr Binding TWO_MODE_SWITCHES[] = {
        Bind::modes(Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
        Bind::modes(Channels::SWD, MODE_MANUAL, MODE_RTH),
    };
    TEST_ASSERT_FALSE(BindingCheck::channelsFree(STICK));
    TEST_ASSERT_FALSE(BindingCheck::channelsFree(ARM));
    TEST_ASSERT_FALSE(BindingCheck::channelsFree(OUT_OF_RANGE));
    TEST_ASSERT_FALSE(BindingCheck::channelsUnique(DUPLICATE));
    TEST_ASSERT_FALSE(BindingCheck::atMostOneModeSwitch(TWO_MODE_SWITCHES));

    TEST_ASSERT_TRUE(BindingCheck::channelsFree(Controls::BINDINGS));
    TEST_ASSERT_TRUE(BindingCheck::channelsUnique(Controls::BINDINGS));
    TEST_ASSERT_TRUE(BindingCheck::atMostOneModeSwitch(Controls::BINDINGS));
}

void test_default_bindings_are_printed_at_start()
{
    PilotSwitches switches;
    switches.printBindings();
    const std::string text = takeSerial();
    TEST_ASSERT_TRUE(contains(text, "SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)"));
    TEST_ASSERT_TRUE(contains(text, "SwB (CH6): FLAPS, пока включён"));
    TEST_ASSERT_TRUE(contains(text, "SwD (CH8): RTH, пока включён"));
    TEST_ASSERT_TRUE(contains(text, "VrA (CH9): крутилка STAB_GAIN"));
    TEST_ASSERT_TRUE(contains(text, "VrB (CH10): крутилка CRUISE_SPEED"));

    static constexpr Binding TWO_POS[] = { Bind::modes(Channels::VRB, MODE_MANUAL, MODE_ACRO) };
    PilotSwitches other(nullptr, TWO_POS);
    other.printBindings();
    TEST_ASSERT_TRUE(contains(takeSerial(), "VrB (CH10): MANUAL / ACRO (вверх / вниз)"));
    TEST_ASSERT_EQUAL_STRING("CH?", PilotSwitches::channelName(Channels::AILERON));
}

void test_names_of_everything()
{
    for (uint8_t m = 0; m < MODE_COUNT; ++m)
    {
        TEST_ASSERT_TRUE(strcmp(AutopilotNames::mode(static_cast<AutopilotMode>(m)), "UNKNOWN") != 0);
        TEST_ASSERT_TRUE(strlen(AutopilotNames::modeShort(static_cast<AutopilotMode>(m))) <= 5);
    }
    for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
    {
        TEST_ASSERT_TRUE(strcmp(AutopilotNames::feature(static_cast<Feature>(f)), "?") != 0);
    }
    for (uint8_t k = 0; k < static_cast<uint8_t>(Knob::COUNT); ++k)
    {
        TEST_ASSERT_TRUE(strcmp(AutopilotNames::knob(static_cast<Knob>(k)), "?") != 0);
    }
    TEST_ASSERT_EQUAL_STRING("?", AutopilotNames::feature(Feature::COUNT));
    TEST_ASSERT_EQUAL_STRING("?", AutopilotNames::knob(Knob::COUNT));
    TEST_ASSERT_EQUAL_STRING("?", LaunchController::stateName(static_cast<LaunchController::State>(9)));
    TEST_ASSERT_EQUAL_STRING("?", SoaringController::stateName(static_cast<SoaringController::State>(9)));
}

// ------------------------------------------------------------
// Навигация
// ------------------------------------------------------------

void test_geo_distance_bearing_and_moved()
{
    const GeoPoint north = Geo::moved(HOME, 100, 0);
    const GeoPoint east = Geo::moved(HOME, 0, 100);
    const GeoPoint southWest = Geo::moved(HOME, -300, -400);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, Geo::distance(HOME, north));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, Geo::distance(HOME, east));
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 500.0f, Geo::distance(HOME, southWest));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, Geo::bearing(HOME, north));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 90.0f, Geo::bearing(HOME, east));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 180.0f + 53.13f, Geo::bearing(HOME, southWest));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 53.13f, Geo::bearing(southWest, HOME));

    float n, e;
    Geo::offsetNE(HOME, southWest, n, e);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -300.0f, n);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -400.0f, e);

    TEST_ASSERT_EQUAL_FLOAT(-170.0f, Geo::wrap180(190.0f));
    TEST_ASSERT_EQUAL_FLOAT(170.0f, Geo::wrap180(-190.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, Geo::wrap180(180.0f));
    TEST_ASSERT_EQUAL_FLOAT(350.0f, Geo::wrap360(-10.0f));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, Geo::wrap360(370.0f));

    GpsData gps = {};
    gps.latitude = 55.123456789;
    gps.longitude = 37.987654321;
    const GeoPoint p = Geo::fromGps(gps);
    TEST_ASSERT_TRUE(p.lat == gps.latitude && p.lon == gps.longitude);
}

void test_guidance_course_and_orbit()
{
    TEST_ASSERT_EQUAL_FLOAT(20.0f, Guidance::rollForCourse(90, 70, 35));
    TEST_ASSERT_EQUAL_FLOAT(-35.0f, Guidance::rollForCourse(270, 0, 35));   // короче — налево
    TEST_ASSERT_EQUAL_FLOAT(35.0f, Guidance::rollForCourse(10, 300, 35));   // через север — направо

    // На окружности — касательная по часовой (с севера — на восток).
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, Guidance::orbitCourse(0, 50, 50, true));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 270.0f, Guidance::orbitCourse(0, 50, 50, false));
    // Далеко — почти прямо на центр; внутри — наружу по касательной.
    TEST_ASSERT_FLOAT_WITHIN(3.5f, 180.0f, Guidance::orbitCourse(0, 5000, 50, true));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f - atanf(2.0f) * 57.29578f, Guidance::orbitCourse(0, 0, 50, true));

    TEST_ASSERT_FLOAT_WITHIN(0.05f, 24.65f, Guidance::orbitBankDeg(15, 50));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 87.50f, Guidance::orbitBankDeg(15, 0));   // радиус 0 -> 1 м
}

// ------------------------------------------------------------
// Высота/скорость, автотриммер, запуск, парение, пищалка
// ------------------------------------------------------------

void test_altitude_speed_controller()
{
    AltitudeSpeedController c;
    // Ниже цели на 5 м: хотим +2 м/с, упреждение asin(2/15) + Kp·2.
    const float pitch = c.pitchFor(50, 45, 0, 15, 0.02f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, asinf(2.0f / 15.0f) * 57.29578f + 6.0f, pitch);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, c.getWantedClimb());
    TEST_ASSERT_EQUAL_FLOAT(Config::NAV_MAX_CLIMB_PITCH_DEG, c.pitchFor(200, 0, 0, 15, 0.02f, false));
    TEST_ASSERT_EQUAL_FLOAT(Config::NAV_MAX_DIVE_PITCH_DEG, c.pitchFor(0, 200, 0, 15, 0.02f, false));

    // Интегратор — только когда разрешено.
    c.reset();
    const float a = c.pitchFor(50, 50, -1.0f, 15, 1.0f, false);
    const float b = c.pitchFor(50, 50, -1.0f, 15, 1.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, Config::NAV_CLIMB_KI_DEG, b - a);

    // Газ без трубки: газ круиза + на набор.
    c.reset();
    c.pitchFor(50, 45, 0, 15, 0.02f, false);   // хотим +2 м/с
    TEST_ASSERT_EQUAL_FLOAT(55.0f + 16.0f, c.throttleFor(false, 0, 0, 55, 0.02f, false));
    // С трубкой: ПИ по воздушной скорости.
    c.reset();
    c.pitchFor(50, 50, 0, 15, 0.02f, false);
    TEST_ASSERT_EQUAL_FLOAT(Config::CRUISE_THROTTLE_PCT + 12.0f, c.throttleFor(true, 12, 14, 0, 0.02f, false));
    TEST_ASSERT_EQUAL_FLOAT(Config::AUTO_THROTTLE_MIN_PCT, c.throttleFor(true, 30, 14, 0, 0.02f, false));
    TEST_ASSERT_EQUAL_FLOAT(Config::AUTO_THROTTLE_MAX_PCT, c.throttleFor(true, 2, 14, 0, 0.02f, false));
}

void test_auto_trim_learns_clamps_and_persists()
{
    AutoTrim trim;
    trim.load();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, trim.getRoll());

    trim.update(false, true, 100, -50, 1.0f);   // функция выключена
    trim.update(true, false, 100, -50, 1.0f);   // не летит ровно
    TEST_ASSERT_EQUAL_FLOAT(0.0f, trim.getRoll());
    TEST_ASSERT_FALSE(trim.saveIfChanged());

    trim.update(true, true, 100, -50, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100 * Config::AUTOTRIM_RATE, trim.getRoll());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -50 * Config::AUTOTRIM_RATE, trim.getPitch());
    for (int i = 0; i < 100; ++i) trim.update(true, true, 500, -500, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(Config::AUTOTRIM_MAX_US, trim.getRoll());
    TEST_ASSERT_EQUAL_FLOAT(-Config::AUTOTRIM_MAX_US, trim.getPitch());

    TEST_ASSERT_TRUE(trim.saveIfChanged());
    TEST_ASSERT_TRUE(contains(takeSerial(), "AutoTrim: сохранён"));
    TEST_ASSERT_FALSE(trim.saveIfChanged());   // без изменений — не пишет

    AutoTrim restored;
    restored.load();
    TEST_ASSERT_EQUAL_FLOAT(Config::AUTOTRIM_MAX_US, restored.getRoll());
    restored.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, restored.getRoll());

    fake::nvs().failBegin = true;
    AutoTrim broken;
    broken.load();
    broken.update(true, true, 100, 0, 1.0f);
    TEST_ASSERT_FALSE(broken.saveIfChanged());
}

void test_launch_controller_sequence()
{
    LaunchController launch;
    launch.update(false, true, 3.0f, false, 0, 0);
    TEST_ASSERT_EQUAL(LaunchController::State::IDLE, launch.getState());

    launch.update(true, true, 1.0f, false, 0, 10);
    TEST_ASSERT_EQUAL(LaunchController::State::READY, launch.getState());
    TEST_ASSERT_TRUE(contains(takeSerial(), "взведён"));
    launch.update(true, false, 1.0f, false, 0, 20);   // газ убрали — отбой
    TEST_ASSERT_EQUAL(LaunchController::State::IDLE, launch.getState());

    launch.update(true, true, 1.0f, false, 0, 30);
    launch.update(true, true, 3.0f, false, 0, 40);    // толчок 20 мс — не бросок
    launch.update(true, true, 3.0f, false, 0, 60);
    launch.update(true, true, 1.0f, false, 0, 70);
    TEST_ASSERT_EQUAL(LaunchController::State::READY, launch.getState());

    launch.update(true, true, 3.0f, false, 0, 100);
    launch.update(true, true, 3.0f, false, 0, 140);   // 40 мс — бросок
    TEST_ASSERT_EQUAL(LaunchController::State::THROWN, launch.getState());
    TEST_ASSERT_FALSE(launch.motorOn());
    TEST_ASSERT_EQUAL_FLOAT(Config::LAUNCH_CLIMB_PITCH_DEG, launch.pitchTargetDeg());

    launch.update(true, true, 0.0f, false, 0, 140 + Config::LAUNCH_MOTOR_DELAY_MS);
    TEST_ASSERT_EQUAL(LaunchController::State::CLIMB, launch.getState());
    TEST_ASSERT_TRUE(launch.motorOn());
    launch.update(true, true, 0.0f, false, Config::LAUNCH_ALTITUDE_M, 1000);
    TEST_ASSERT_EQUAL(LaunchController::State::DONE, launch.getState());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, launch.pitchTargetDeg());
    TEST_ASSERT_EQUAL_STRING("DONE", LaunchController::stateName(launch.getState()));

    // Пилот взял стики в наборе — запуск закончен; по времени — тоже.
    LaunchController a;
    a.update(true, true, 0, false, 0, 0);
    a.update(true, true, 3, false, 0, 1);
    a.update(true, true, 3, false, 0, 50);
    a.update(true, true, 0, false, 0, 400);
    TEST_ASSERT_EQUAL(LaunchController::State::CLIMB, a.getState());
    a.update(true, true, 0, true, 5, 410);
    TEST_ASSERT_EQUAL(LaunchController::State::DONE, a.getState());

    LaunchController b;
    b.update(true, true, 0, false, 0, 0);
    b.update(true, true, 3, false, 0, 1);
    b.update(true, true, 3, false, 0, 50);
    b.update(true, true, 0, false, 0, 400);
    b.update(true, true, 0, false, 5, 400 + Config::LAUNCH_CLIMB_MS);
    TEST_ASSERT_EQUAL(LaunchController::State::DONE, b.getState());
    b.update(false, true, 0, false, 5, 10000);   // DISARM — сначала
    TEST_ASSERT_EQUAL(LaunchController::State::IDLE, b.getState());
}

void test_soaring_controller_states()
{
    SoaringController s;
    s.reset(0);
    uint32_t t = 0;
    auto run = [&](float climb, float alt, float dist, uint32_t ms) {
        for (uint32_t i = 0; i < ms; i += 20)
        {
            t += 20;
            s.update(climb, alt, dist, 0.02f, t);
        }
    };

    run(1.0f, 60, 100, 1000);   // подъём, но меньше 1.5 с
    TEST_ASSERT_EQUAL(SoaringController::State::GLIDE, s.getState());
    run(-0.5f, 60, 100, 100);   // оборвался — счёт заново
    run(1.0f, 60, 100, 1000);
    TEST_ASSERT_EQUAL(SoaringController::State::GLIDE, s.getState());
    run(1.0f, 60, 100, 600);
    TEST_ASSERT_EQUAL(SoaringController::State::THERMAL, s.getState());
    TEST_ASSERT_TRUE(contains(takeSerial(), "термик"));

    run(1.0f, 70, 100, 9000);   // держит — в круге
    TEST_ASSERT_EQUAL(SoaringController::State::THERMAL, s.getState());
    TEST_ASSERT_GREATER_THAN_FLOAT(0.5f, s.getAverageClimb());
    run(-1.0f, 70, 100, 9000);  // кончился
    TEST_ASSERT_EQUAL(SoaringController::State::GLIDE, s.getState());

    run(-1.0f, 25, 100, 20);    // низко — мотор
    TEST_ASSERT_EQUAL(SoaringController::State::MOTOR_CLIMB, s.getState());
    TEST_ASSERT_TRUE(s.motorOn());
    run(2.0f, Config::SOAR_MAX_ALTITUDE_M, 100, 20);
    TEST_ASSERT_EQUAL(SoaringController::State::GLIDE, s.getState());

    run(-1.0f, 90, 500, 20);    // далеко от дома
    TEST_ASSERT_EQUAL(SoaringController::State::RETURN, s.getState());
    run(-1.0f, 90, 350, 20);
    TEST_ASSERT_EQUAL(SoaringController::State::RETURN, s.getState());
    run(-1.0f, 90, 200, 20);
    TEST_ASSERT_EQUAL(SoaringController::State::GLIDE, s.getState());
    TEST_ASSERT_EQUAL_STRING("GLIDE", SoaringController::stateName(s.getState()));
    TEST_ASSERT_EQUAL_STRING("THERMAL", SoaringController::stateName(SoaringController::State::THERMAL));
    TEST_ASSERT_EQUAL_STRING("MOTOR_CLIMB", SoaringController::stateName(SoaringController::State::MOTOR_CLIMB));
    TEST_ASSERT_EQUAL_STRING("RETURN", SoaringController::stateName(SoaringController::State::RETURN));
}

void test_beeper_patterns()
{
    Beeper beeper;
    TEST_ASSERT_FALSE(beeper.update(false, false, false, 0));
    TEST_ASSERT_TRUE(beeper.update(true, true, false, 0));        // по тумблеру — в любой момент
    TEST_ASSERT_FALSE(beeper.update(true, true, false, 250));     // 2 Гц
    TEST_ASSERT_TRUE(beeper.update(true, true, false, 500));

    // Связи нет на земле: пищит только через LOST_MODEL_BEEP_DELAY_MS.
    TEST_ASSERT_FALSE(beeper.update(false, false, true, 1000));
    TEST_ASSERT_FALSE(beeper.update(false, false, true, 1000 + Config::LOST_MODEL_BEEP_DELAY_MS - 1));
    TEST_ASSERT_TRUE(beeper.update(false, false, true, 1000 + Config::LOST_MODEL_BEEP_DELAY_MS + 250 * 4));
    TEST_ASSERT_TRUE(beeper.isLostModel());
    // В воздухе по потере связи не пищит.
    TEST_ASSERT_FALSE(beeper.update(false, true, true, 60000));
    TEST_ASSERT_FALSE(beeper.isLostModel());
}

// ------------------------------------------------------------
// Режимы Autopilot
// ------------------------------------------------------------

void test_home_is_set_on_arm_and_by_switch()
{
    Rig rig;
    rig.goodGpsAt(HOME, 0);
    rig.gps.data.numSatellites = 4;   // мало спутников — дома нет
    rig.step(false);
    rig.step(true);
    TEST_ASSERT_FALSE(rig.autopilot.getNavStatus().homeValid);
    TEST_ASSERT_FALSE(rig.autopilot.getNavStatus().gpsGood);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, rig.autopilot.getNavStatus().distanceHomeM);

    rig.gps.data.numSatellites = 9;   // фикс стал хорошим в воздухе — дом записан
    rig.step(true);
    TEST_ASSERT_TRUE(rig.autopilot.getNavStatus().homeValid);
    TEST_ASSERT_TRUE(contains(takeSerial(), "дом записан (ARM)"));

    const GeoPoint there = Geo::moved(HOME, 200, 0);
    rig.goodGpsAt(there, 0);
    rig.step(true);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 200.0f, rig.autopilot.getNavStatus().distanceHomeM);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 180.0f, rig.autopilot.getNavStatus().bearingHomeDeg);

    rig.autopilot.setInputs(with(Feature::HOME_RESET));   // тумблер — дом здесь
    rig.step(true);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, rig.autopilot.getNavStatus().distanceHomeM);
    TEST_ASSERT_TRUE(contains(takeSerial(), "HOME_RESET"));
    rig.step(true);   // тумблер держат — второй раз не срабатывает
    TEST_ASSERT_EQUAL(0u, takeSerial().size());

    rig.autopilot.setInputs(PilotInputs());
    rig.step(true);
    rig.gps.data.fixType = 2;
    rig.autopilot.setInputs(with(Feature::HOME_RESET));
    rig.step(true);
    TEST_ASSERT_TRUE(contains(takeSerial(), "нет хорошего GPS"));
}

void test_course_source_gps_compass_gyro()
{
    Rig rig;
    rig.imu.data.yaw = -30;
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::GYRO, (int)rig.autopilot.getCourseSource());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 330.0f, rig.autopilot.getNavStatus().courseDeg);

    rig.mag.available = true;
    rig.mag.data.headingDegrees = 45;
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::COMPASS, (int)rig.autopilot.getCourseSource());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, rig.autopilot.getNavStatus().targetCourseDeg);   // перехвачена

    rig.goodGpsAt(HOME, 120, 2.0f);   // медленно — курсу GPS не верим
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::COMPASS, (int)rig.autopilot.getCourseSource());
    rig.gps.data.groundSpeed = 12;
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::GPS, (int)rig.autopilot.getCourseSource());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, rig.autopilot.getNavStatus().courseDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, rig.autopilot.getNavStatus().speedMs);   // скорость — GPS

    // Гистерезис: чуть ниже порога курс остаётся по GPS, на 1 м/с ниже — компас.
    rig.gps.data.groundSpeed = Config::NAV_GPS_COURSE_MIN_SPEED_MS - 0.5f;
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::GPS, (int)rig.autopilot.getCourseSource());
    rig.gps.data.groundSpeed = Config::NAV_GPS_COURSE_MIN_SPEED_MS - 1.5f;
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::COMPASS, (int)rig.autopilot.getCourseSource());
    rig.gps.data.groundSpeed = Config::NAV_GPS_COURSE_MIN_SPEED_MS - 0.5f;
    rig.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::COMPASS, (int)rig.autopilot.getCourseSource());
    rig.gps.data.groundSpeed = 12;
    rig.step();

    rig.airspeed.available = true;
    rig.airspeed.setSpeed(17);
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.0f, rig.autopilot.getNavStatus().speedMs);   // трубка важнее

    Rig blind;
    blind.imu.available = false;
    blind.step();
    TEST_ASSERT_EQUAL((int)Autopilot::CourseSource::NONE, (int)blind.autopilot.getCourseSource());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, Config::NAV_ASSUMED_SPEED_MS, blind.autopilot.getNavStatus().speedMs);
}

void test_cruise_holds_course_and_altitude_and_lets_pilot_steer()
{
    Rig rig;
    rig.goodGpsAt(HOME, 90);
    rig.baro.data.altitude = 50;
    rig.step();
    rig.autopilot.setMode(MODE_CRUISE);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, rig.autopilot.getNavStatus().targetCourseDeg);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, rig.autopilot.getTargetAltitude());

    rig.gps.data.heading = 70;   // снесло влево — довернуть вправо
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(20.0f, rig.autopilot.getDesiredRoll());
    TEST_ASSERT_TRUE(rig.autopilot.isAutoThrottle());
    TEST_ASSERT_EQUAL_UINT16(1000 + 550, rig.autopilot.applyThrottle(1000));   // газ круиза, стик не важен

    ControlCommand sticks;
    sticks.roll = 250;           // пилот поворачивает сам
    rig.step(true, false, 1000, sticks);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 22.5f, rig.autopilot.getDesiredRoll());
    rig.step();                  // отпустил — держит курс, на котором отпустил
    TEST_ASSERT_EQUAL_FLOAT(70.0f, rig.autopilot.getNavStatus().targetCourseDeg);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredRoll());

    rig.baro.data.altitude = 45;   // ниже — нос вверх, газа больше
    rig.step();
    TEST_ASSERT_GREATER_THAN_FLOAT(10.0f, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_UINT16(1000 + 710, rig.autopilot.applyThrottle(1000));

    // Крутилка скорости круиза: газ 85 % на краю.
    rig.autopilot.setInputs(withKnob(Knob::CRUISE_SPEED, 1.0f));
    rig.baro.data.altitude = 50;
    rig.step();
    TEST_ASSERT_UINT16_WITHIN(20, 1000 + 850, rig.autopilot.applyThrottle(1000));
}

void test_loiter_circles_with_or_without_gps()
{
    Rig blind;
    blind.step();
    blind.autopilot.setMode(MODE_LOITER);
    blind.step();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 24.65f, blind.autopilot.getDesiredRoll());   // круг на месте

    Rig rig;
    rig.goodGpsAt(HOME, 0);
    rig.step();
    rig.autopilot.setMode(MODE_LOITER);   // центр — здесь
    rig.goodGpsAt(Geo::moved(HOME, 500, 0), 0);   // улетели на север, летим на север
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 176.8f, rig.autopilot.getNavStatus().targetCourseDeg);   // назад к центру
    TEST_ASSERT_EQUAL_FLOAT(Config::NAV_BANK_LIMIT_DEG, rig.autopilot.getDesiredRoll());

    // На окружности, по касательной — только упреждающий крен круга.
    rig.goodGpsAt(Geo::moved(HOME, 50, 0), 90);
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 24.65f, rig.autopilot.getDesiredRoll());

    // Крутилка радиуса: 150 м — крен меньше.
    rig.autopilot.setInputs(withKnob(Knob::LOITER_RADIUS, 1.0f));
    rig.goodGpsAt(Geo::moved(HOME, 150, 0), 90);
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, Guidance::orbitBankDeg(15, 150), rig.autopilot.getDesiredRoll());
}

void test_rth_flies_home_climbs_and_circles()
{
    Rig rig;
    rig.armAtHome();
    rig.baro.data.altitude = 10;
    rig.goodGpsAt(Geo::moved(HOME, 0, 800), 0);   // в 800 м к востоку, нос на север
    rig.step();
    rig.autopilot.setMode(MODE_RTH);
    TEST_ASSERT_EQUAL_FLOAT(Config::RTH_ALTITUDE_M, rig.autopilot.getTargetAltitude());
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 270.0f, rig.autopilot.getNavStatus().targetCourseDeg);
    TEST_ASSERT_EQUAL_FLOAT(-Config::NAV_BANK_LIMIT_DEG, rig.autopilot.getDesiredRoll());   // влево, к дому
    TEST_ASSERT_GREATER_THAN_FLOAT(5.0f, rig.autopilot.getDesiredPitch());   // набор до 40 м
    TEST_ASSERT_TRUE(rig.autopilot.isAutoThrottle());
    TEST_ASSERT_GREATER_THAN_UINT16(1550, rig.autopilot.applyThrottle(1000));

    rig.goodGpsAt(Geo::moved(HOME, 0, 60), 270);   // пришли
    rig.step();
    TEST_ASSERT_TRUE(contains(takeSerial(), "над домом"));

    // Выше высоты RTH — остаётся на своей.
    Rig high;
    high.armAtHome();
    high.baro.data.altitude = 90;
    high.step();
    high.autopilot.setMode(MODE_RTH);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, high.autopilot.getTargetAltitude());

    // Без GPS — круг на месте, без барометра — горизонт.
    Rig blind;
    blind.baro.available = false;
    blind.step();
    blind.autopilot.setMode(MODE_RTH);
    blind.step();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 24.65f, blind.autopilot.getDesiredRoll());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, blind.autopilot.getDesiredPitch());
}

void test_link_loss_returns_home_with_gps_and_glides_without()
{
    Rig rig;
    rig.armAtHome();
    rig.goodGpsAt(Geo::moved(HOME, 300, 0), 0);
    rig.step();
    takeSerial();
    rig.step(true, true);
    TEST_ASSERT_TRUE(rig.autopilot.isFailsafeActive());
    TEST_ASSERT_TRUE(rig.autopilot.isFailsafeReturning());
    TEST_ASSERT_FALSE(rig.autopilot.isFailsafeGliding());
    TEST_ASSERT_EQUAL_STRING("FAILSAFE_RTH", rig.autopilot.getModeName());
    TEST_ASSERT_TRUE(contains(takeSerial(), "возврат домой"));
    TEST_ASSERT_GREATER_THAN_UINT16(1000, rig.autopilot.applyThrottle(1000));   // мотор работает
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 180.0f, rig.autopilot.getNavStatus().targetCourseDeg);

    rig.gps.available = false;   // GPS пропал в пути — всё равно домой (круг на месте)
    rig.step(true, true);
    TEST_ASSERT_TRUE(rig.autopilot.isFailsafeReturning());

    rig.gps.available = true;
    rig.step(true, false);       // связь вернулась
    TEST_ASSERT_FALSE(rig.autopilot.isFailsafeActive());
    TEST_ASSERT_TRUE(contains(takeSerial(), "связь восстановлена"));

    rig.gps.available = false;   // без GPS — планирование
    rig.step(true, true);
    TEST_ASSERT_TRUE(rig.autopilot.isFailsafeGliding());
    TEST_ASSERT_EQUAL_UINT16(1000, rig.autopilot.applyThrottle(1800));
}

void test_geofence_sends_home_once_per_breach()
{
    Rig rig;
    rig.armAtHome();
    rig.autopilot.setInputs(with(Feature::GEOFENCE));
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.goodGpsAt(Geo::moved(HOME, 600, 0), 0);
    rig.step();
    TEST_ASSERT_EQUAL(MODE_RTH, rig.autopilot.getMode());
    TEST_ASSERT_TRUE(rig.autopilot.getNavStatus().fenceBreached);
    TEST_ASSERT_TRUE(contains(takeSerial(), "геозабор"));

    rig.autopilot.setMode(MODE_STABILIZE);   // пилот забрал управление
    rig.step();
    TEST_ASSERT_EQUAL(MODE_STABILIZE, rig.autopilot.getMode());

    rig.goodGpsAt(Geo::moved(HOME, 400, 0), 0);   // вернулся внутрь с запасом
    rig.step();
    TEST_ASSERT_FALSE(rig.autopilot.getNavStatus().fenceBreached);
    rig.baro.data.altitude = Config::FENCE_ALTITUDE_M + 5;   // теперь выше потолка
    rig.step();
    TEST_ASSERT_EQUAL(MODE_RTH, rig.autopilot.getMode());

    Rig off;   // без функции забор молчит
    off.armAtHome();
    off.autopilot.setMode(MODE_STABILIZE);
    off.goodGpsAt(Geo::moved(HOME, 900, 0), 0);
    off.step();
    TEST_ASSERT_EQUAL(MODE_STABILIZE, off.autopilot.getMode());
}

void test_rescue_land_and_acro()
{
    Rig rig;
    rig.autopilot.setMode(MODE_RESCUE);
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredRoll());
    TEST_ASSERT_EQUAL_FLOAT(Config::RESCUE_PITCH_DEG, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_UINT16(1700, rig.autopilot.applyThrottle(1000));

    rig.autopilot.setMode(MODE_AUTO_LAND);
    rig.baro.data.altitude = 20;
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(Config::LAND_GLIDE_PITCH_DEG, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_UINT16(1000, rig.autopilot.applyThrottle(1800));   // мотор выключен
    rig.baro.data.altitude = 2;
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(Config::LAND_FLARE_PITCH_DEG, rig.autopilot.getDesiredPitch());

    rig.autopilot.setMode(MODE_ACRO);
    rig.imu.data.gyroX = 20;     // порыв крутит вправо
    rig.imu.data.gyroY = -10;
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(-30.0f, rig.autopilot.getRollCorrection());
    TEST_ASSERT_EQUAL_FLOAT(15.0f, rig.autopilot.getPitchCorrection());
    rig.imu.available = false;   // без IMU — рули по стикам
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getRollCorrection());
}

void test_stall_protection_with_airspeed()
{
    Rig rig;
    rig.airspeed.available = true;
    rig.airspeed.setSpeed(7);
    rig.baro.data.altitude = 30;
    rig.step();
    rig.autopilot.setMode(MODE_CRUISE);
    rig.baro.data.altitude = 20;    // ниже цели — хотел бы нос вверх
    rig.step();
    TEST_ASSERT_TRUE(rig.autopilot.getNavStatus().stallWarning);
    TEST_ASSERT_TRUE(rig.autopilot.getDesiredPitch() <= 0.0f);
    TEST_ASSERT_EQUAL_UINT16(2000, rig.autopilot.applyThrottle(1000));

    rig.airspeed.setSpeed(14);
    rig.step();
    TEST_ASSERT_FALSE(rig.autopilot.getNavStatus().stallWarning);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, rig.autopilot.getDesiredPitch());

    rig.airspeed.setSpeed(5);        // на земле (не заармлен) — не защищает
    rig.step(false);
    TEST_ASSERT_FALSE(rig.autopilot.getNavStatus().stallWarning);
}

void test_turn_coordination_and_auto_trim()
{
    Rig rig;
    rig.autopilot.setInputs(with(Feature::TURN_COORDINATION));
    rig.imu.data.roll = 45;
    ControlCommand sticks;
    sticks.roll = 200;
    rig.step(true, false, 1000, sticks);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, rig.autopilot.getYawCorrection());        // руль направления в разворот
    TEST_ASSERT_EQUAL_FLOAT(62.0f, rig.autopilot.getPitchCorrection());      // руль высоты в крене

    rig.imu.data.roll = 0;
    rig.autopilot.setInputs(with(Feature::AUTO_TRIM));
    sticks.roll = 100;               // пилот держит ручку — самолёт "тянет" влево
    for (int i = 0; i < 1000; ++i) rig.step(true, false, 1000, sticks);   // 2 с
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 40.0f, rig.autopilot.getRollCorrection());
    rig.step(false, false, 1000, sticks);   // DISARM — сохраняет
    TEST_ASSERT_TRUE(contains(takeSerial(), "AutoTrim: сохранён"));

    Rig next;                        // после перезагрузки — триммер на месте
    next.autopilot.begin();
    next.step();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 40.0f, next.autopilot.getRollCorrection());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 40.0f, next.autopilot.getAutoTrim().getRoll());
}

void test_auto_trim_is_saved_only_after_landing()
{
    Rig rig;
    rig.autopilot.setInputs(with(Feature::AUTO_TRIM));
    ControlCommand sticks;
    sticks.roll = 100;
    rig.baro.data.altitude = 60;     // летим
    for (int i = 0; i < 500; ++i) rig.step(true, false, 1000, sticks);
    takeSerial();
    const uint32_t commitsBefore = fake::nvs().commits;

    // DISARM в воздухе (мотор выключен тумблером): флеш не трогаем —
    // запись NVS на ESP32 заморозила бы рули на ~0.4 с.
    for (int i = 0; i < 100; ++i) rig.step(false, false, 1000, sticks);
    TEST_ASSERT_EQUAL(commitsBefore, fake::nvs().commits);

    // Снижается у земли, но ещё едет по трубке — ждём.
    rig.baro.data.altitude = 0.5f;
    rig.airspeed.available = true;
    rig.airspeed.setSpeed(8);
    rig.step(false);
    TEST_ASSERT_EQUAL(commitsBefore, fake::nvs().commits);

    // GPS говорит "едет" — тоже ждём.
    rig.airspeed.setSpeed(0);
    rig.goodGpsAt(HOME, 0, 5.0f);
    rig.step(false);
    rig.step(false);
    TEST_ASSERT_EQUAL(commitsBefore, fake::nvs().commits);

    // Встал — записано ровно один раз.
    rig.gps.data.groundSpeed = 0;
    rig.step(false);
    rig.step(false);
    TEST_ASSERT_EQUAL(commitsBefore + 1, fake::nvs().commits);
    TEST_ASSERT_TRUE(contains(takeSerial(), "AutoTrim: сохранён"));
    for (int i = 0; i < 10; ++i) rig.step(false);
    TEST_ASSERT_EQUAL(commitsBefore + 1, fake::nvs().commits);
}

void test_knobs_scale_stabilization_and_bank_limit()
{
    Rig rig;
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.imu.data.roll = 10;
    rig.step(false);
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, rig.autopilot.getRollCorrection());

    rig.autopilot.setInputs(withKnob(Knob::STAB_GAIN, 1.0f));   // ×2
    rig.step(false);
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, rig.autopilot.getRollCorrection());
    rig.autopilot.setInputs(withKnob(Knob::STAB_GAIN, -1.0f));  // ×0.25
    rig.step(false);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -12.5f, rig.autopilot.getRollCorrection());

    rig.autopilot.setInputs(withKnob(Knob::MAX_BANK, -1.0f));
    ControlCommand sticks;
    sticks.roll = 500;
    rig.step(false, false, 1000, sticks);
    TEST_ASSERT_EQUAL_FLOAT(Config::MAX_BANK_MIN_DEG, rig.autopilot.getDesiredRoll());
}

void test_launch_through_autopilot()
{
    Rig rig;
    rig.autopilot.setMode(MODE_LAUNCH);
    rig.step(true, false, 1600);
    TEST_ASSERT_EQUAL(LaunchController::State::READY, rig.autopilot.getLaunchState());
    TEST_ASSERT_EQUAL_UINT16(1000, rig.autopilot.applyThrottle(1600));   // мотор стоит

    rig.imu.data.accelX = 3.0f;
    for (int i = 0; i < 30; ++i) rig.step(true, false, 1600);
    TEST_ASSERT_EQUAL(LaunchController::State::THROWN, rig.autopilot.getLaunchState());
    TEST_ASSERT_EQUAL_UINT16(1000, rig.autopilot.applyThrottle(1600));
    rig.imu.data.accelX = 0.0f;
    for (int i = 0; i < 160; ++i) rig.step(true, false, 1600);
    TEST_ASSERT_EQUAL(LaunchController::State::CLIMB, rig.autopilot.getLaunchState());
    TEST_ASSERT_EQUAL_UINT16(2000, rig.autopilot.applyThrottle(1600));
    TEST_ASSERT_EQUAL_FLOAT(Config::LAUNCH_CLIMB_PITCH_DEG, rig.autopilot.getDesiredPitch());

    rig.baro.data.altitude = Config::LAUNCH_ALTITUDE_M;
    rig.step(true, false, 1600);
    rig.step(true, false, 1600);
    TEST_ASSERT_EQUAL(LaunchController::State::DONE, rig.autopilot.getLaunchState());
    TEST_ASSERT_EQUAL_FLOAT(Config::LAUNCH_ALTITUDE_M, rig.autopilot.getTargetAltitude());   // держит высоту
    TEST_ASSERT_UINT16_WITHIN(5, 1550, rig.autopilot.applyThrottle(1600));                    // газ круиза
}

void test_soaring_through_autopilot()
{
    Rig rig;
    rig.baro.data.altitude = 60;
    rig.step();
    rig.autopilot.setMode(MODE_SOARING);
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(Config::SOAR_GLIDE_PITCH_DEG, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_UINT16(1000, rig.autopilot.applyThrottle(1800));

    rig.baro.data.verticalSpeed = 1.0f;   // термик
    for (int i = 0; i < 800; ++i) rig.step();
    TEST_ASSERT_EQUAL(SoaringController::State::THERMAL, rig.autopilot.getSoaringState());
    TEST_ASSERT_EQUAL_FLOAT(Config::SOAR_CIRCLE_BANK_DEG, rig.autopilot.getDesiredRoll());

    rig.baro.data.altitude = 20;          // низко — мотор
    rig.baro.data.verticalSpeed = -1.0f;
    rig.step();
    TEST_ASSERT_EQUAL(SoaringController::State::MOTOR_CLIMB, rig.autopilot.getSoaringState());
    TEST_ASSERT_GREATER_THAN_UINT16(1400, rig.autopilot.applyThrottle(1000));

    // Далеко от дома в наборе — к дому.
    Rig far;
    far.armAtHome();
    far.baro.data.altitude = 10;
    far.goodGpsAt(Geo::moved(HOME, 0, 600), 0);
    far.step();
    far.autopilot.setMode(MODE_SOARING);
    far.step();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 270.0f, far.autopilot.getNavStatus().targetCourseDeg);
}

// ------------------------------------------------------------
// Функции уровня FlightController
// ------------------------------------------------------------

namespace
{
    template <size_t N>
    struct Plane
    {
        FakeBoard board;
        FakeImu imu;
        FakeBaro baro;
        FakeGps gps;
        IBusReceiver receiver{ board.rcUart() };
        ControlMixer mixer;
        ThrottleManager throttle;
        FlightOutputs outputs{ board };
        Autopilot autopilot{ &imu, &baro, nullptr, &gps };
        PilotSwitches switches;
        ArmingManager arming{ &autopilot };
        FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &switches };

        explicit Plane(const Binding (&table)[N]) : switches(&autopilot, table)
        {
            gps.available = false;
            outputs.begin();
            controller.begin();
        }

        void tick(const RcChannels& rc)
        {
            board.rc.push(ibusFrame(rc));
            fake::advanceMs(2);
            controller.update();
        }

        void tickWithoutFrame()
        {
            fake::advanceMs(2);
            controller.update();
        }

        void arm(RcChannels rc)
        {
            rc.set(Channels::ARM, 1000).set(Channels::THROTTLE, 1000);
            tick(rc);
            rc.set(Channels::ARM, 2000);
            tick(rc);
        }

        uint16_t pwm(uint8_t channel) const { return board.servos[channel].lastUs; }
    };
}

void test_motor_kill_payload_camera_and_rates()
{
    static constexpr Binding TABLE[] = {
        Bind::feature(Channels::SWB, Feature::MOTOR_KILL),
        Bind::feature(Channels::SWD, Feature::PAYLOAD_DROP),
        Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
        Bind::knob   (Channels::VRB, Knob::RATES),
    };
    Plane<4> plane(TABLE);
    RcChannels rc;
    plane.arm(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());

    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1600);
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(1300, plane.pwm(ServoChannel::ESC));   // ограничен THROTTLE_LIMIT_PCT
    rc.set(Channels::SWB, 2000);   // мотор-килл
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(1000, plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_TRUE(plane.controller.isArmed());

    TEST_ASSERT_EQUAL_UINT16(Config::PAYLOAD_CLOSED_US, plane.pwm(ServoChannel::AUX1));
    rc.set(Channels::SWD, 2000);   // сброс груза
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(Config::PAYLOAD_OPEN_US, plane.pwm(ServoChannel::AUX1));

    TEST_ASSERT_EQUAL_UINT16(1500, plane.pwm(ServoChannel::AUX2));   // камера по центру
    rc.set(Channels::VRA, 2000);
    plane.tick(rc);
    TEST_ASSERT_UINT16_WITHIN(1, 1500 + 30 * 500 / 90, plane.pwm(ServoChannel::AUX2));
    rc.set(Channels::VRA, 1000);
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(1000, plane.pwm(ServoChannel::AUX2));

    rc.set(Channels::VRB, 1000).set(Channels::AILERON, 2000);   // чувствительность 30 %
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(Config::AILERON_LEFT_REVERSED ? 1350 : 1650, plane.pwm(ServoChannel::AILERON_LEFT));
}

void test_airbrake_flap_knob_camera_stab_and_beeper()
{
    static constexpr Binding TABLE[] = {
        Bind::feature(Channels::SWB, Feature::AIRBRAKE),
        Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
        Bind::feature(Channels::VRA, Feature::BEEPER),
        Bind::knob   (Channels::VRB, Knob::FLAPS),
    };
    Plane<4> plane(TABLE);
    RcChannels rc;
    rc.set(Channels::VRB, 1000);   // закрылки убраны
    plane.tick(rc);

    rc.set(Channels::VRB, 2000);   // крутилка до упора — закрылки полностью, плавно
    for (int i = 0; i < 600; ++i) plane.tick(rc);
    TEST_ASSERT_EQUAL_INT16(Config::FLAPS_DEPLOYED_US, plane.controller.getFlapsUs());
    rc.set(Channels::SWB, 2000);   // тормоз важнее закрылков: 470 мкс хода — ~2.1 с
    for (int i = 0; i < 1200; ++i) plane.tick(rc);
    TEST_ASSERT_EQUAL_INT16(-Config::AIRBRAKE_US, plane.controller.getFlapsUs());

    plane.imu.data.pitch = 10;
    rc.set(Channels::SWD, 2000);   // камера держит горизонт
    plane.tick(rc);
    TEST_ASSERT_UINT16_WITHIN(1, 1500 - 10 * 500 / 90, plane.pwm(ServoChannel::AUX2));

    rc.set(Channels::VRA, 2000);   // пищалка
    unsigned changes = plane.board.buzzerChanges;
    for (int i = 0; i < 500; ++i) plane.tick(rc);
    TEST_ASSERT_GREATER_THAN(changes + 2, plane.board.buzzerChanges);
    TEST_ASSERT_TRUE(plane.outputs.isBuzzerOn() == plane.board.buzzer);
}

void test_lost_model_beeps_after_link_loss_on_ground()
{
    static constexpr Binding TABLE[] = { Bind::feature(Channels::SWB, Feature::FLAPS) };
    Plane<1> plane(TABLE);
    RcChannels rc;
    plane.tick(rc);
    for (int i = 0; i < 5000; ++i) plane.tickWithoutFrame();   // 10 с без связи
    TEST_ASSERT_FALSE(plane.controller.isLostModelBeeping());
    for (int i = 0; i < 300; ++i) plane.tickWithoutFrame();
    TEST_ASSERT_TRUE(plane.controller.isLostModelBeeping());
}

void test_failsafe_rth_keeps_motor_and_payload_closed()
{
    static constexpr Binding TABLE[] = { Bind::feature(Channels::SWD, Feature::PAYLOAD_DROP) };
    Plane<1> plane(TABLE);
    plane.gps.available = true;
    plane.gps.data.fixType = 3;
    plane.gps.data.numSatellites = 10;
    plane.gps.data.horizontalAccuracy = 1;
    plane.gps.data.latitude = HOME.lat;
    plane.gps.data.longitude = HOME.lon;
    plane.gps.data.groundSpeed = 15;

    RcChannels rc;
    plane.arm(rc);
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);   // автопилот видит ARM со следующего такта — тогда и пишет дом
    TEST_ASSERT_TRUE(plane.autopilot.getNavStatus().homeValid);

    const GeoPoint away = Geo::moved(HOME, 400, 0);
    plane.gps.data.latitude = away.lat;
    plane.gps.data.longitude = away.lon;
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1500);
    plane.tick(rc);

    for (int i = 0; i < 300; ++i) plane.tickWithoutFrame();   // 600 мс без кадров
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    TEST_ASSERT_TRUE(plane.autopilot.isFailsafeReturning());
    TEST_ASSERT_GREATER_THAN_UINT16(1000, plane.pwm(ServoChannel::ESC));   // мотор работает — домой
    TEST_ASSERT_EQUAL_UINT16(Config::PAYLOAD_CLOSED_US, plane.pwm(ServoChannel::AUX1));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_switches_modes_overrides_features_and_knobs);
    RUN_TEST(test_two_position_mode_switch_and_zones);
    RUN_TEST(test_switches_without_mode_bindings_leave_mode_alone);
    RUN_TEST(test_binding_table_checks_catch_mistakes);
    RUN_TEST(test_default_bindings_are_printed_at_start);
    RUN_TEST(test_names_of_everything);
    RUN_TEST(test_geo_distance_bearing_and_moved);
    RUN_TEST(test_guidance_course_and_orbit);
    RUN_TEST(test_altitude_speed_controller);
    RUN_TEST(test_auto_trim_learns_clamps_and_persists);
    RUN_TEST(test_launch_controller_sequence);
    RUN_TEST(test_soaring_controller_states);
    RUN_TEST(test_beeper_patterns);
    RUN_TEST(test_home_is_set_on_arm_and_by_switch);
    RUN_TEST(test_course_source_gps_compass_gyro);
    RUN_TEST(test_cruise_holds_course_and_altitude_and_lets_pilot_steer);
    RUN_TEST(test_loiter_circles_with_or_without_gps);
    RUN_TEST(test_rth_flies_home_climbs_and_circles);
    RUN_TEST(test_link_loss_returns_home_with_gps_and_glides_without);
    RUN_TEST(test_geofence_sends_home_once_per_breach);
    RUN_TEST(test_rescue_land_and_acro);
    RUN_TEST(test_stall_protection_with_airspeed);
    RUN_TEST(test_turn_coordination_and_auto_trim);
    RUN_TEST(test_auto_trim_is_saved_only_after_landing);
    RUN_TEST(test_knobs_scale_stabilization_and_bank_limit);
    RUN_TEST(test_launch_through_autopilot);
    RUN_TEST(test_soaring_through_autopilot);
    RUN_TEST(test_motor_kill_payload_camera_and_rates);
    RUN_TEST(test_airbrake_flap_knob_camera_stab_and_beeper);
    RUN_TEST(test_lost_model_beeps_after_link_loss_on_ground);
    RUN_TEST(test_failsafe_rth_keeps_motor_and_payload_closed);
    return UNITY_END();
}
