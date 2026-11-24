// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на HAKRC F405
// (P57b): MPU6000 на SPI1 (CS PA4) — на плате бывает и ICM-20689, драйвер
// один; флеш W25Q128 на SPI2 (CS PB12), барометра нет, приёмник — UART1
// (RX PB7, на нём инвертор SBUS). Загрузка находит гироскоп (I2C у него
// выключен) и флеш, барометра нет — не ошибка; ARM, газ — моторы M1–M4
// крутятся; полёт пишется во флеш.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_hakrc_f405
// ============================================================

#define BOARD_STM32F405
#define OP_BOARD_FILE "config/boards/HAKRCF405.h"
#define SENSOR_KIT SENSOR_KIT_BF_BOARD
#define VEHICLE_MULTIROTOR
#include "../../../src/stm32/main_copter.cpp"

#include <unity.h>

#include <string>
#include <vector>

#include "helpers/BlackBoxParse.h"
#include "helpers/ChipEmulators.h"
#include "helpers/SpiNorFlashChip.h"
#include "helpers/TestSupport.h"

using namespace bbparse;

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    // M1–M4: TIM3 (PB0 — вторая периферия, ALT1), TIM8, TIM1, TIM1 — по OPB_MOTOR_AF
    const int16_t MOTORS[] = { PB0 | ALT1, PC6, PA10, PA8 };
    chips::Mpu6000 imuChip;
    chips::SpiNorFlashChip flashChip;   // W25Q128, 16 МБ

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
        flashChip.attach(flashSpi, static_cast<uint8_t>(PB12));
        fake::gpio().analogMv[PC1] = 25200 / 11;   // 6S, делитель 1:11
    }

    void tick(const RcChannels& rc)
    {
        imuChip.update(world);
        if (millis() % 7 == 0) port(PB7).pushRx(ibusFrame(rc));
        fake::runTask(*fake::findTask("flight"), 1);
        fake::runTask(*fake::findTask("bbox"), 2);
    }

    long pulse(int16_t pin) { return fake::timerPulseUs(static_cast<uint32_t>(pin)); }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(rc);
    }
}

void test_boot_finds_mpu6000_and_flash()
{
    imuChip.update(world);
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "WHO_AM_I=0x68 -> MPU6000"), log.c_str());
    TEST_ASSERT_TRUE(imuChip.i2cDisabled());
    TEST_ASSERT_TRUE(imuSensor.isAvailable());
    TEST_ASSERT_EQUAL_STRING("MPU6000", imuSensor.getSensorType());
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "SPI-флеш ящика: Winbond, JEDEC 0xEF4018, 16 МБ"), log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BlackBox: ждёт ARM"), log.c_str());
    TEST_ASSERT_FALSE(baroSensor.isAvailable());   // на плате барометра нет
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "ОШИБКА"), log.c_str());
}

void test_arm_spins_motors_and_flight_goes_to_flash()
{
    const uint64_t startUs = fake::nowUs();
    imuChip.beforeRead([startUs](uint8_t reg) {
        if (reg != 0x3B) return;
        const float t = (fake::nowUs() - startUs) / 1e6f;
        World pose = world;
        pose.pitchDeg = (t >= 3.5f && t < 5.5f) ? 40.0f : 0.0f;
        pose.rollDeg = t >= 5.5f ? 35.0f : 0.0f;
        imuChip.update(pose);
    });
    imuSensor.calibrateOrientation();
    imuChip.beforeRead([](uint8_t) {});
    takeSerial();

    RcChannels rc;
    run(rc, 3.0);
    long stopPulse[4];
    for (int m = 0; m < 4; ++m) stopPulse[m] = pulse(MOTORS[m]);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE_MESSAGE(copterController.isArmed(), takeSerial().c_str());
    rc.set(Channels::THROTTLE, 1300);
    run(rc, 2.0);
    for (int m = 0; m < 4; ++m)
    {
        char why[64];
        snprintf(why, sizeof(why), "M%d: %ld при стопе %ld", m + 1, pulse(MOTORS[m]), stopPulse[m]);
        TEST_ASSERT_TRUE_MESSAGE(stopPulse[m] > 0 && pulse(MOTORS[m]) > stopPulse[m], why);
    }
    rc.set(Channels::THROTTLE, 1000).set(Channels::ARM, 1000);
    run(rc, CopterBlackBox::POSTROLL_MS / 1000.0 + 0.5);
    for (int i = 0; i < 5000 && copterBlackBox.getState() == BlackBoxCore::State::Stopping; ++i) tick(rc);
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    TEST_ASSERT_TRUE_MESSAGE(contains(takeSerial(), "BlackBox: полёт #1 записан"), "полёт не закрылся");

    SpiNorFlash reread(flashOwnBus, PB12);
    TEST_ASSERT_TRUE(reread.begin());
    BlackBoxStorage storage(reread);
    TEST_ASSERT_TRUE(storage.begin());
    TEST_ASSERT_EQUAL_UINT32(1, storage.flightCount());
    const std::vector<Rec> recs = readFlight(storage, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "vehicle=copter"));
    TEST_ASSERT_GREATER_THAN_UINT32(1000, countType(recs, REC_QGYRO));
    TEST_ASSERT_EQUAL_UINT32(0, flashChip.rejected);
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_mpu6000_and_flash);
    RUN_TEST(test_arm_spins_motors_and_flight_goes_to_flash);
    return UNITY_END();
}
