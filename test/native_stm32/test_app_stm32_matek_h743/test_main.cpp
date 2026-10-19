// ============================================================
// Прошивка целиком на Matek H743-WING V3 (src/stm32/main.cpp, блок пинов
// BOARD_MATEK_H743): датчики на плате — ICM-42688-P (SPI1, CS PC15) и
// DPS310 (I2C2, 0x76); компас QMC6309 — на внешнем разъёме I2C1
// (PB7/PB6), отдельной шине; приёмник на USART6 (PC7), GPS — USART2
// (PD6), MAVLink — UART7 (PE7/PE8), мотор — S1 (PB0).
//
// Плату поставили в самолёт стрелкой к хвосту. Пока установка IMU не
// откалибрована, ARM со стабилизацией запрещён (в MANUAL — можно); три
// позы — и углы верные, ARM разрешён.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_matek_h743
// ============================================================

#define BOARD_MATEK_H743
#define SENSOR_KIT SENSOR_KIT_MATEK_H743
#define SENSOR_MAG SENSOR_MAG_QMC6309
#include "../../../src/stm32/main.cpp"

#include <unity.h>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;    // DPS310 — те же регистры
    chips::Qmc6309 magChip;
    uint32_t lastGpsMs = 0;

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

    void wireUpBoard()
    {
        imuChip.rotationCwDeg = 180;   // плата стрелкой к хвосту
        imuChip.attach(SPI, static_cast<uint8_t>(PC15));
        imuChip.install();
        baroChip.attach(Wire, 0x76);
        baroChip.install();
        TwoWire* external = fake::wireWithSda(PB7);
        TEST_ASSERT_NOT_NULL(external);
        magChip.attach(*external, 0x7C);
        magChip.install();
    }

    void syncChips()
    {
        imuChip.update(world);
        magChip.update(world);
        baroChip.update(world);
    }

    void tick(const RcChannels& rc)
    {
        syncChips();
        port(PC7).pushRx(ibusFrame(rc));
        if (millis() - lastGpsMs >= 100)
        {
            lastGpsMs = millis();
            const std::vector<uint8_t> frame = chips::navPvt(world);
            port(PD6).pushRx(std::string(frame.begin(), frame.end()));
        }
        fake::runTask(*fake::findTask("flight"), 1);
    }

    void fly(const RcChannels& rc, double seconds)
    {
        const int ticks = static_cast<int>(seconds * 1000 / Config::LOOP_PERIOD_MS);
        for (int i = 0; i < ticks; ++i) tick(rc);
    }

    RcChannels stabilized()
    {
        RcChannels rc;
        rc.set(Channels::SWC, 1500);
        return rc;
    }
}

void test_boot_finds_sensors_on_board_and_compass_on_external_bus()
{
    syncChips();
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "ICM42688: подключён"));
    TEST_ASSERT_TRUE(contains(log, "DPS310: подключён"));
    TEST_ASSERT_TRUE(contains(log, "QMC6309: подключён"));
    TEST_ASSERT_TRUE(imuSensor.isAvailable() && baroSensor.isAvailable() && magSensor.isAvailable());

    // SPI1 и обе шины I2C — на пинах Matek; шины экрана нет.
    TEST_ASSERT_EQUAL(PA5, SPI.stm32Pins[0]);
    TEST_ASSERT_EQUAL(PA6, SPI.stm32Pins[1]);
    TEST_ASSERT_EQUAL(PD7, SPI.stm32Pins[2]);
    TEST_ASSERT_EQUAL(PB11, Wire.sdaPin());
    TEST_ASSERT_TRUE(fake::wireWithSda(PB7)->isStarted());
    TEST_ASSERT_NULL(board.displayI2c());

    // Плата на 8 МГц кварца — такты в src/stm32/clock_matek.cpp, здесь
    // их нет; а вот проверка установки IMU — есть.
    TEST_ASSERT_TRUE(contains(imuSensor.getPreflightProblem(), "установка не откалибрована"));
}

void test_app_scan_shows_the_external_bus_separately()
{
    Serial.pushRx("\x02" "scan\n");
    tick(RcChannels());
    const std::string scan = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(scan, "@scan {\"sensor\":[{\"a\":118,"), scan.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(scan, "\"ext\":[{\"a\":124,"), scan.c_str());
    TEST_ASSERT_TRUE(contains(scan, "\"screen\":null}"));

    // Паспорт для приложения — плата matek-h743, а не общий stm32h743.
    Serial.pushRx("\x02" "id\n");
    tick(RcChannels());
    TEST_ASSERT_TRUE(contains(takeSerial(), "\"board\":\"matek-h743\""));
}

void test_stabilized_arm_waits_for_mount_calibration_manual_does_not()
{
    RcChannels rc = stabilized();
    fly(rc, 0.5);
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.3);
    TEST_ASSERT_FALSE(flightController.isArmed());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ARM отклонён — IMU на плате полётника: установка не откалибрована"));

    // В MANUAL IMU не нужен.
    rc.set(Channels::ARM, 1000).set(Channels::SWC, 1000);
    fly(rc, 0.3);
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.3);
    TEST_ASSERT_TRUE(flightController.isArmed());
    rc.set(Channels::THROTTLE, 1600);
    fly(rc, 0.3);
    TEST_ASSERT_GREATER_THAN(escUs(1100), fake::timerPulseUs(PB0));   // мотор — S1
    rc.set(Channels::ARM, 1000).set(Channels::THROTTLE, 1000);
    fly(rc, 0.3);
    TEST_ASSERT_FALSE(flightController.isArmed());
    takeSerial();
}

void test_three_poses_fix_the_axes_and_allow_arm()
{
    // Пилот ставит самолёт: ровно, нос вверх, правое крыло вниз — по
    // расписанию времени, чип читает мир при каждом чтении данных.
    const uint64_t startUs = fake::nowUs();
    imuChip.beforeRead([startUs](uint8_t reg) {
        if (reg != 0x1D) return;
        const float t = (fake::nowUs() - startUs) / 1e6f;
        World pose = world;
        pose.pitchDeg = (t >= 3.5f && t < 5.5f) ? 40.0f : 0.0f;
        pose.rollDeg = t >= 5.5f ? 35.0f : 0.0f;
        imuChip.update(pose);
    });
    imuSensor.calibrateOrientation();
    imuChip.beforeRead([](uint8_t) {});
    TEST_ASSERT_TRUE(contains(takeSerial(), "установка сохранена: нос = -X чипа"));
    TEST_ASSERT_NULL(imuSensor.getPreflightProblem());

    // Нос вверх на 15° — прошивка видит тангаж, а не крен.
    world.pitchDeg = 15;
    fly(RcChannels(), 3.0);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 15.0f, imuSensor.getImuData().pitch);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 0.0f, imuSensor.getImuData().roll);
    world.pitchDeg = 0;
    fly(RcChannels(), 3.0);

    RcChannels rc = stabilized();
    fly(rc, 0.3);
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.3);
    TEST_ASSERT_TRUE(flightController.isArmed());
    TEST_ASSERT_NOT_EQUAL(MODE_MANUAL, autopilot.getMode());
    rc.set(Channels::ARM, 1000);
    fly(rc, 0.2);
    takeSerial();
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_sensors_on_board_and_compass_on_external_bus);
    RUN_TEST(test_app_scan_shows_the_external_bus_separately);
    RUN_TEST(test_stabilized_arm_waits_for_mount_calibration_manual_does_not);
    RUN_TEST(test_three_poses_fix_the_axes_and_allow_arm);
    return UNITY_END();
}
