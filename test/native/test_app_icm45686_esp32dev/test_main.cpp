// ============================================================
// Прошивка целиком (src/main.cpp) на обычной ESP32 38-pin
// (BOARD_ESP32_CLASSIC, env esp32-dev) с набором
// SENSOR_KIT_ICM45686_PITOT:
//   ICM-45686 (I2C 0x68) + QMC6309 (0x7C),
//   SPL06-001 (0x76) — статика, BMP581 (0x47) — трубка Пито,
//   u-blox M10 — UART2. Второй шины I2C нет — экрана нет.
// Ветка feature/split-headers: пины компилируются в src/core/*.cpp,
// поэтому плата задаётся средой сборки (-D BOARD_ESP32_CLASSIC),
// а не #define в тесте, как в header-only ветке.
//
// Запуск: pio test -e native-esp32dev
// ============================================================

#if !defined(BOARD_ESP32_CLASSIC) || defined(BOARD_ESP32_S3)
#error "Этот тест собирается в среде native-esp32dev"
#endif
#define SENSOR_KIT SENSOR_KIT_ICM45686_PITOT
#include "../../../src/main.cpp"

#include <unity.h>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    chips::Icm45686 imuChip;
    chips::Qmc6309 magChip;
    chips::Spl06 staticChip;
    chips::Bmp581 tubeChip;
    uint32_t lastGpsMs = 0;

    void wireUpKit()
    {
        imuChip.attach(Wire, 0x68);
        imuChip.install();
        magChip.attach(Wire, 0x7C);
        magChip.install();
        staticChip.attach(Wire, 0x76);
        staticChip.install();
        tubeChip.attach(Wire, 0x47);
        tubeChip.install();
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
}

void test_boot_on_38pin_board_with_icm45686_kit()
{
    syncChips();
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "ICM45686: подключён"));
    TEST_ASSERT_TRUE(contains(log, "ICM45686: предполётная проверка пройдена"));
    TEST_ASSERT_TRUE(contains(log, "QMC6309: подключён"));
    TEST_ASSERT_TRUE(contains(log, "SPL06: подключён"));
    TEST_ASSERT_TRUE(contains(log, "PITOT-BMP581: подключён"));
    TEST_ASSERT_FALSE(contains(log, "OLED: подключён"));   // второй шины на этой плате нет

    // Распиновка — блок BOARD_ESP32_CLASSIC.
    TEST_ASSERT_EQUAL(13, Config::PIN_AILERON_LEFT);
    TEST_ASSERT_EQUAL(Config::PIN_AILERON_LEFT, fake::ledc().channel[ServoChannel::AILERON_LEFT].pin);
    TEST_ASSERT_EQUAL(Config::PIN_ESC, fake::ledc().channel[ServoChannel::ESC].pin);
    TEST_ASSERT_EQUAL(Config::PIN_I2C_SDA, Wire.sdaPin());
    TEST_ASSERT_EQUAL(Config::PIN_I2C_SCL, Wire.sclPin());
    TEST_ASSERT_TRUE(imuSensor.isAvailable() && magSensor.isAvailable() && baroSensor.isAvailable());
}

void test_icm45686_filters_were_programmed()
{
    // Косвенные регистры фильтров: LPF ODR/32 у гироскопа и акселерометра.
    TEST_ASSERT_EQUAL_HEX8(0x04, imuChip.indirect[0xA4AC] & 0x07);
    TEST_ASSERT_EQUAL_HEX8(0x04, imuChip.indirect[0xA583] & 0x07);
}

void test_hand_launch_mode_arms_and_waits_for_throw()
{
    RcChannels rc;
    rc.set(Channels::SWC, 1000);
    fly(rc, 1.5);   // трубка обнуляется
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.2);
    TEST_ASSERT_TRUE(flightController.isArmed());

    autopilot.setMode(MODE_LAUNCH);
    rc.set(Channels::THROTTLE, 1700);   // взвести
    fly(rc, 0.5);
    TEST_ASSERT_EQUAL(LaunchController::State::READY, autopilot.getLaunchState());
    TEST_ASSERT_UINT32_WITHIN(3, 1000, pulseUs(ServoChannel::ESC));   // в руке мотор стоит
    takeSerial();
}

void test_stabilize_with_icm45686_and_airspeed()
{
    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 1500).set(Channels::THROTTLE, 1300);
    world.airspeedMs = 14;
    world.rollDeg = -25;   // левое крыло вниз
    fly(rc, 1.0);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, -25.0f, imuSensor.getImuData().roll);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, autopilot.getRollCorrection());   // крен вправо
    TEST_ASSERT_FLOAT_WITHIN(0.7f, 14.0f, pitotSensor.getAirspeedData().indicatedMs);

    world.rollDeg = 0;
    rc.set(Channels::ARM, 1000);
    fly(rc, 0.2);
    TEST_ASSERT_FALSE(flightController.isArmed());
    takeSerial();
}

void test_console_scan_on_single_bus_board()
{
    Serial.pushRx("b");
    loop();
    const std::string out = takeSerial();
    TEST_ASSERT_TRUE(contains(out, "0x68  MPU6050/6500, ICM-42688/45686"));
    TEST_ASSERT_TRUE(contains(out, "0x7C  QMC6309"));
    TEST_ASSERT_FALSE(contains(out, "I2C экрана"));
}

int main()
{
    wireUpKit();
    UNITY_BEGIN();
    RUN_TEST(test_boot_on_38pin_board_with_icm45686_kit);
    RUN_TEST(test_icm45686_filters_were_programmed);
    RUN_TEST(test_hand_launch_mode_arms_and_waits_for_throw);
    RUN_TEST(test_stabilize_with_icm45686_and_airspeed);
    RUN_TEST(test_console_scan_on_single_bus_board);
    return UNITY_END();
}
