// ============================================================
// Прошивка целиком (src/main.cpp) на ESP32-S3 с лётным набором
// датчиков SENSOR_KIT_LSM6DSV_PITOT:
//   LSM6DSV (I2C 0x6A) + QMC6309 (0x7C) — модуль IMU с компасом,
//   SPL06-001 (0x76) — статическое давление в фюзеляже,
//   BMP581 (0x47) — полное давление в самодельной трубке Пито,
//   u-blox M10 — UART2, OLED — вторая шина I2C.
// Чипы — регистровые эмуляторы (helpers/ChipEmulators.h), данные
// берутся из "мира": углы, высота, скорость, координаты. Проверяется
// путь от регистров чипов до ШИМ, дашборда и консоли. Тесты идут по
// порядку, как день на поле (мир из теста в тест не сбрасывается).
//
// Запуск: pio test -e native -f native/test_app_lsm6dsv_pitot
// ============================================================

#define SENSOR_KIT SENSOR_KIT_LSM6DSV_PITOT
#include "../../../src/main.cpp"

#include <unity.h>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    chips::Lsm6dsv imuChip;
    chips::Qmc6309 magChip;
    chips::Spl06 staticChip;
    chips::Bmp581 tubeChip;
    fake::RegisterMapDevice screenChip;
    uint32_t lastGpsMs = 0;

    void wireUpKit()
    {
        imuChip.attach(Wire, 0x6A);
        imuChip.install();
        magChip.attach(Wire, 0x7C);
        magChip.install();
        staticChip.attach(Wire, 0x76);
        staticChip.install();
        tubeChip.attach(Wire, 0x47);
        tubeChip.install();
        Wire1.attach(0x3C, &screenChip);
    }

    void syncChips()
    {
        imuChip.update(world);
        magChip.update(world);
        staticChip.update(world);
        tubeChip.setSample(world.tubePa(), world.temperatureC);
    }

    void tick(const RcChannels& rc)
    {
        syncChips();
        fake::uart(1)->pushRx(ibusFrame(rc));
        if (millis() - lastGpsMs >= 100)
        {
            lastGpsMs = millis();
            const std::vector<uint8_t> frame = chips::navPvt(world);
            fake::uart(Config::UART_NUM_GPS)->pushRx(std::string(frame.begin(), frame.end()));
        }
        loop();
    }

    void fly(const RcChannels& rc, double seconds)
    {
        const int ticks = static_cast<int>(seconds * 1000 / Config::LOOP_PERIOD_MS);
        for (int i = 0; i < ticks; ++i) tick(rc);
    }

    uint32_t pulseUs(uint8_t channel)
    {
        const fake::LedcChannel& ch = fake::ledc().channel[channel];
        return static_cast<uint32_t>(ch.duty * 20000.0 / 16384.0 + 0.5);
    }

    RcChannels manual()
    {
        RcChannels rc;
        rc.set(Channels::SWC, 1000);
        return rc;
    }
}

void test_boot_finds_every_chip_of_the_kit()
{
    syncChips();
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "LSM6DSV: подключён (LSM6DSV/16X)"));
    TEST_ASSERT_TRUE(contains(log, "LSM6DSV: предполётная проверка пройдена"));
    TEST_ASSERT_TRUE(contains(log, "QMC6309: подключён"));
    TEST_ASSERT_TRUE(contains(log, "SPL06: подключён"));
    TEST_ASSERT_TRUE(contains(log, "PITOT-BMP581: подключён"));
    TEST_ASSERT_TRUE(contains(log, "OLED: подключён (SSD1306)"));
    TEST_ASSERT_TRUE(contains(log, "SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF"));
    TEST_ASSERT_TRUE(imuSensor.isAvailable());
    TEST_ASSERT_TRUE(magSensor.isAvailable());
    TEST_ASSERT_TRUE(baroSensor.isAvailable());
    TEST_ASSERT_TRUE(pitotBaro.isAvailable());
}

void test_pitot_zeroes_on_ground_then_measures_airspeed()
{
    // Первая секунда: самолёт стоит, трубка "врёт" на 150 Па — это ноль.
    fly(manual(), 1.5);
    TEST_ASSERT_FALSE(pitotSensor.isZeroing());
    TEST_ASSERT_TRUE(pitotSensor.isAvailable());
    TEST_ASSERT_FLOAT_WITHIN(0.8f, 0.0f, pitotSensor.getAirspeedData().indicatedMs);

    world.airspeedMs = 15;
    fly(manual(), 1.0);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 15.0f, pitotSensor.getAirspeedData().indicatedMs);
    world.airspeedMs = 0;
    fly(manual(), 1.0);
    takeSerial();
}

void test_static_baro_tracks_altitude()
{
    world.altitudeM = 30;
    fly(manual(), 3.0);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 30.0f, baroSensor.getBarometerData().altitude);
    world.altitudeM = 0;
    fly(manual(), 3.0);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 0.0f, baroSensor.getBarometerData().altitude);
}

void test_arm_sets_home_from_gps()
{
    RcChannels rc = manual();
    fly(rc, 0.2);
    TEST_ASSERT_TRUE(gpsSensor.isAvailable());
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.3);
    TEST_ASSERT_TRUE(flightController.isArmed());
    TEST_ASSERT_TRUE(autopilot.getNavStatus().homeValid);
    TEST_ASSERT_TRUE(contains(takeSerial(), "ArmingManager: ARM"));
}

void test_stabilize_levels_a_tilted_plane()
{
    RcChannels rc = manual();
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 1500);
    fly(rc, 0.1);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());

    world.rollDeg = 20;   // правое крыло вниз
    fly(rc, 1.0);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 20.0f, imuSensor.getImuData().roll);
    TEST_ASSERT_LESS_THAN_FLOAT(-20.0f, autopilot.getRollCorrection());   // крен влево

    world.rollDeg = 0;
    world.pitchDeg = -15;   // нос вниз
    fly(rc, 1.0);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, -15.0f, imuSensor.getImuData().pitch);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, autopilot.getPitchCorrection());  // нос вверх
    world.pitchDeg = 0;
    fly(rc, 1.0);
    takeSerial();
}

void test_rth_switch_turns_toward_home()
{
    // Улетели на 300 м к северу, летим на север.
    world.lat += 300.0 / 111195.0;
    world.courseDeg = 0;
    world.groundSpeedMs = 15;
    world.airspeedMs = 15;
    world.altitudeM = 50;
    RcChannels rc = manual();
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 1500).set(Channels::THROTTLE, 1400);
    fly(rc, 2.0);

    rc.set(Channels::SWD, 2000);
    fly(rc, 0.5);
    TEST_ASSERT_EQUAL(MODE_RTH, autopilot.getMode());
    const NavStatus& nav = autopilot.getNavStatus();
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 300.0f, nav.distanceHomeM);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 180.0f, nav.bearingHomeDeg);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, fabsf(autopilot.getDesiredRoll()));   // разворот
    TEST_ASSERT_TRUE(autopilot.isAutoThrottle());

    rc.set(Channels::SWD, 1000).set(Channels::SWC, 1000).set(Channels::ARM, 1000);
    fly(rc, 0.2);
    TEST_ASSERT_FALSE(flightController.isArmed());
    takeSerial();
}

void test_console_bus_scan_names_the_chips()
{
    Serial.pushRx("b");
    loop();
    const std::string out = takeSerial();
    TEST_ASSERT_TRUE(contains(out, "0x47  BMP581"));
    TEST_ASSERT_TRUE(contains(out, "0x6A  LSM6DSV"));
    TEST_ASSERT_TRUE(contains(out, "0x76  BME280/BMP388/SPL06"));
    TEST_ASSERT_TRUE(contains(out, "0x7C  QMC6309"));
    TEST_ASSERT_TRUE(contains(out, "I2C экрана"));
    TEST_ASSERT_TRUE(contains(out, "0x3C  OLED"));
}

void test_dashboard_shows_airspeed_gps_and_modes()
{
    world.airspeedMs = 12;
    fly(manual(), 1.0);
    WebServer* http = fake::webServers().back();
    http->request(HTTP_GET, "/api/status");
    const std::string json = http->lastResponse().body;
    TEST_ASSERT_TRUE(contains(json, "\"airspeed\":{\"attached\":true,\"available\":true"));
    TEST_ASSERT_TRUE(contains(json, "\"gps\":{\"attached\":true,\"available\":true"));
    TEST_ASSERT_TRUE(contains(json, "\"mag\":{\"attached\":true,\"available\":true"));
    TEST_ASSERT_TRUE(contains(json, "\"ias\":12"));
    takeSerial();
}

void test_outputs_reach_the_s3_pins()
{
    RcChannels rc = manual();
    rc.set(Channels::AILERON, 2000);
    fly(rc, 0.1);
    TEST_ASSERT_EQUAL(Config::PIN_AILERON_LEFT, fake::ledc().channel[ServoChannel::AILERON_LEFT].pin);
    TEST_ASSERT_EQUAL(Config::PIN_ESC, fake::ledc().channel[ServoChannel::ESC].pin);
    TEST_ASSERT_UINT32_WITHIN(3, 2000, pulseUs(ServoChannel::AILERON_LEFT));
    TEST_ASSERT_UINT32_WITHIN(3, 1000, pulseUs(ServoChannel::ESC));
    takeSerial();
}

int main()
{
    wireUpKit();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_every_chip_of_the_kit);
    RUN_TEST(test_pitot_zeroes_on_ground_then_measures_airspeed);
    RUN_TEST(test_static_baro_tracks_altitude);
    RUN_TEST(test_arm_sets_home_from_gps);
    RUN_TEST(test_stabilize_levels_a_tilted_plane);
    RUN_TEST(test_rth_switch_turns_toward_home);
    RUN_TEST(test_console_bus_scan_names_the_chips);
    RUN_TEST(test_dashboard_shows_airspeed_gps_and_modes);
    RUN_TEST(test_outputs_reach_the_s3_pins);
    return UNITY_END();
}
