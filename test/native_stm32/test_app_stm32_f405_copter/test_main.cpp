// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на полётнике
// STM32F405 под Betaflight — распиновка из заголовка платы, как его пишет
// tools/bf_board.py. Эталон — AocodaRC F405 V3
// (include/config/boards/AOCODARCF405V3.h): ICM-42688-P на SPI1 (CS PA4),
// SPL06 на I2C1, приёмник на UART2 (RX PA3), моторы M1–M4 — PC6–PC9,
// батарея — PC2 через делитель 1:11 (vbat_scale 110).
//
// Проверяется, что заголовок платы доходит до железа: датчики находятся на
// своих шинах, импульсы «стоп» — на выходах M1–M4 и только на них, пульт —
// с UART2, ARM — после калибровки установки, холостые после ARM.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_f405_copter
// ============================================================

#define BOARD_STM32F405
#define OP_BOARD_FILE "config/boards/AOCODARCF405V3.h"
#define SENSOR_KIT SENSOR_KIT_BF_BOARD
#define VEHICLE_MULTIROTOR
#include "../../../src/stm32/main_copter.cpp"

#include <unity.h>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;

    const int16_t MOTORS[] = { PC6, PC7, PC8, PC9 };

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

    void wireUpBoard()
    {
        imuChip.attach(SPI, static_cast<uint8_t>(PA4));
        imuChip.install();
        baroChip.attach(Wire, 0x76);
        baroChip.install();
        fake::gpio().analogMv[PC2] = 25200 / 11;   // 6S, 25.2 В через делитель 1:11
    }

    void tick(const RcChannels* rc)
    {
        imuChip.update(world);
        baroChip.update(world);
        if (rc && millis() % 7 == 0) port(PA3).pushRx(ibusFrame(*rc));
        fake::runTask(*fake::findTask("flight"), 1);
    }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(&rc);
    }

    long pulse(int16_t pin) { return fake::timerPulseUs(static_cast<uint32_t>(pin)); }
}

void test_board_header_reaches_the_hardware()
{
    imuChip.update(world);
    baroChip.update(world);
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Коптер: freestyle5"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "ICM42688: подключён"));
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "SPL06: подключён"), log.c_str());
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "ОШИБКА"), log.c_str());
    TEST_ASSERT_EQUAL_UINT8(4, motors.attached());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));   // OneShot125: «стоп»
    TEST_ASSERT_EQUAL(-1, pulse(PA15));   // M5 квадро не нужен
    TEST_ASSERT_EQUAL_STRING("AOCODARCF405V3", OPB_BOARD_NAME);
    TEST_ASSERT_EQUAL(PA3, Config::PIN_RC_RX);   // приёмник — UART2, как у Betaflight
    TEST_ASSERT_EQUAL(PA10, Config::PIN_GPS_RX);   // GPS — UART1
    TEST_ASSERT_EQUAL(PB7, Config::PIN_I2C_SDA);   // барометр — I2C1
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 11.0f, Config::BLACKBOX_VBAT_DIVIDER);
}

void test_receiver_on_uart2_and_battery()
{
    RcChannels rc;
    run(rc, 2.5);
    TEST_ASSERT_FALSE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_NO_LINK);
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_NO_ACC_CAL);
    Serial.pushRx("s");
    run(rc, 0.01);
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(status, "Батарея 25."), status.c_str());
    TEST_ASSERT_TRUE(contains(status, "связь есть (iBUS)"));
}

void test_mount_calibration_then_arm_spins_m1_to_m4()
{
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
    TEST_ASSERT_TRUE(contains(takeSerial(), "установка сохранена"));

    RcChannels rc;
    run(rc, 3.0);
    TEST_ASSERT_EQUAL_HEX32(0, copterController.getStatus().armBlockers);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    for (int16_t pin : MOTORS)
    {
        TEST_ASSERT_GREATER_THAN(125, pulse(pin));   // холостые
        TEST_ASSERT_LESS_THAN(140, pulse(pin));
    }
    TEST_ASSERT_EQUAL(-1, pulse(PA15));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.05);
    TEST_ASSERT_FALSE(copterController.isArmed());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_board_header_reaches_the_hardware);
    RUN_TEST(test_receiver_on_uart2_and_battery);
    RUN_TEST(test_mount_calibration_then_arm_spins_m1_to_m4);
    return UNITY_END();
}
