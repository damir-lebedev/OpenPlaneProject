// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на Matek
// H743-WING: ICM-42688-P (SPI1) и DPS310 (I2C2) на плате, приёмник iBUS
// на USART6 (PC7), моторы — площадки S1–S4 (PB0, PB1, PA0, PA1),
// регуляторы OneShot125, батарея 6S на делителе PC0.
//
// Проверяется склейка с железом: что вывелось из описания коптера,
// импульсы «стоп» до ARM, ARM только после калибровки установки IMU,
// холостые после ARM, режимы с тумблера, потеря связи на земле, консоль
// и проверка моторов с подтверждением.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_copter
// ============================================================

#define BOARD_MATEK_H743
#define SENSOR_KIT SENSOR_KIT_MATEK_H743
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
    chips::Spl06 baroChip;   // DPS310 — те же регистры

    const int16_t MOTORS[] = { PB0, PB1, PA0, PA1 };

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

    void wireUpBoard()
    {
        imuChip.rotationCwDeg = 90;   // плата повёрнута стрелкой вправо
        imuChip.attach(SPI, static_cast<uint8_t>(PC15));
        imuChip.install();
        baroChip.attach(Wire, 0x76);
        baroChip.install();
        fake::gpio().analogMv[PC0] = 25200 / 11;   // 6S, 25.2 В через делитель 1:11
    }

    void tick(const RcChannels* rc)
    {
        imuChip.update(world);
        baroChip.update(world);
        if (rc && millis() % 7 == 0) port(PC7).pushRx(ibusFrame(*rc));   // кадр iBUS ~каждые 7 мс
        fake::runTask(*fake::findTask("flight"), 1);
    }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(&rc);
    }

    void runWithoutLink(double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(nullptr);
    }

    long pulse(int16_t pin) { return fake::timerPulseUs(static_cast<uint32_t>(pin)); }
}

void test_boot_derives_tune_and_holds_escs_at_stop()
{
    imuChip.update(world);
    baroChip.update(world);
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Коптер: freestyle5"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "регуляторы OneShot125, цикл 1000 Гц"));
    TEST_ASSERT_TRUE(contains(log, "Тяговооружённость"));
    TEST_ASSERT_TRUE(contains(log, "ICM42688: подключён"));
    TEST_ASSERT_TRUE(contains(log, "DPS310: подключён"));
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "ОШИБКА"), log.c_str());
    TEST_ASSERT_EQUAL_UINT8(4, motors.attached());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));   // OneShot125: «стоп»
    TEST_ASSERT_EQUAL(-1, pulse(PA2));   // пятый выход квадро не нужен
    TEST_ASSERT_EQUAL_UINT32(2000, Stm32MotorOutput::outputRateHz(copter::EscProtocol::ONESHOT125));
    TEST_ASSERT_FALSE(copterController.isArmed());
}

void test_arm_waits_for_mount_calibration()
{
    RcChannels rc;
    run(rc, 2.5);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.2);
    TEST_ASSERT_FALSE(copterController.isArmed());
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_EXTERNAL);
    Serial.pushRx("s");
    run(rc, 0.01);
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(status, "нужна калибровка или настройка"), status.c_str());
    TEST_ASSERT_TRUE(contains(status, "Батарея 25."));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.2);
}

void test_three_poses_then_arm_spins_motors_at_idle()
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
    TEST_ASSERT_EQUAL_HEX16(0, copterController.getStatus().armBlockers);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    for (int16_t pin : MOTORS)
    {
        TEST_ASSERT_GREATER_THAN(125, pulse(pin));   // холостые
        TEST_ASSERT_LESS_THAN(140, pulse(pin));
    }
    // тумблер режима: SwC вниз — ACRO
    rc.set(Channels::SWC, 2000);
    run(rc, 0.05);
    TEST_ASSERT_EQUAL(static_cast<int>(copter::CopterMode::ACRO), static_cast<int>(copterController.getStatus().mode));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.05);
    TEST_ASSERT_FALSE(copterController.isArmed());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));
    takeSerial();
}

void test_link_loss_on_ground_disarms()
{
    RcChannels rc;
    run(rc, 0.3);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    runWithoutLink(1.5);   // таймаут приёмника 0.5 с + 0.3 с ожидания
    TEST_ASSERT_FALSE(copterController.isArmed());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));
    // связь вернулась, тумблер так и стоит — сам не армится
    run(rc, 0.5);
    TEST_ASSERT_FALSE(copterController.isArmed());
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);
    takeSerial();
}

void test_console_tune_report_and_motor_test()
{
    RcChannels rc;
    Serial.pushRx("t");
    run(rc, 0.01);
    const std::string tune = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(tune, "Сборка: freestyle5"), tune.c_str());
    TEST_ASSERT_TRUE(contains(tune, "Контур: срез"));
    TEST_ASSERT_TRUE(contains(tune, "Предупреждений нет."));

    // проверка моторов — только после подтверждения
    Serial.pushRx("m");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "СНИМИТЕ ПРОПЕЛЛЕРЫ"));
    Serial.pushRx("n");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "отменена"));
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));

    Serial.pushRx("m");
    run(rc, 0.01);
    Serial.pushRx("y");
    run(rc, 0.5);
    const std::string first = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(first, "Мотор 1: задний правый, по часовой"), first.c_str());
    TEST_ASSERT_GREATER_THAN(125, pulse(PB0));
    TEST_ASSERT_EQUAL(125, pulse(PB1));
    run(rc, 2.0);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Мотор 2: передний правый, против часовой"));
    TEST_ASSERT_EQUAL(125, pulse(PB0));
    TEST_ASSERT_GREATER_THAN(125, pulse(PB1));
    // любая клавиша — стоп
    Serial.pushRx("x");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Проверка моторов окончена"));
    TEST_ASSERT_EQUAL(125, pulse(PB1));
    TEST_ASSERT_FALSE(copterConsole.motorTestRunning());

    // заармленному — не крутит
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    Serial.pushRx("m");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Только без ARM."));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);

    Serial.pushRx("hlr?");
    run(rc, 0.3);
    const std::string misc = takeSerial();
    TEST_ASSERT_TRUE(contains(misc, "m — проверка моторов"));
    TEST_ASSERT_TRUE(contains(misc, "Живая строка: вкл"));
    TEST_ASSERT_TRUE(contains(misc, "DISARMED ANGLE"));
    TEST_ASSERT_TRUE(contains(misc, "Каналы: "));
    Serial.pushRx("lq");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Неизвестная команда"));
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_derives_tune_and_holds_escs_at_stop);
    RUN_TEST(test_arm_waits_for_mount_calibration);
    RUN_TEST(test_three_poses_then_arm_spins_motors_at_idle);
    RUN_TEST(test_link_loss_on_ground_disarms);
    RUN_TEST(test_console_tune_report_and_motor_test);
    return UNITY_END();
}
