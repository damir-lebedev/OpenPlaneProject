// ============================================================
// Прошивка целиком (src/main.cpp) на виртуальном стенде: на шинах —
// симулированные MPU6500, BMP388, QMC5883P и OLED, на UART1 —
// приёмник iBUS, выходы — LEDC. Тесты идут по порядку, как день на
// поле: включение, первый кадр пульта, ARM, газ, режимы, потеря
// связи, консоль, дашборд и экран. Состояние прошивки (глобальные
// объекты main.cpp) переходит из теста в тест, поэтому setUp()
// здесь мир НЕ сбрасывает.
//
// Запуск: pio test -e native -f native/test_app
// ============================================================

#include "../../../src/main.cpp"

#include <unity.h>

#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    fake::RegisterMapDevice imuChip;
    fake::RegisterMapDevice baroChip;
    fake::RegisterMapDevice magChip;
    fake::RegisterMapDevice screenChip;

    void wireUpBench()
    {
        imuChip.regs[0x75] = 0x70;            // MPU6500
        imuChip.setBigEndian16(0x3F, 2048);   // лежит ровно, 1g по Z
        Wire.attach(0x68, &imuChip);

        baroChip.regs[0x00] = 0x50;           // BMP388
        const uint8_t nvm[21] = { 0x15, 0x6A, 0x60, 0x49, 0xF9, 0x3A, 0x0A, 0x21, 0x04, 0x07, 0xF1,
                                  0x28, 0x4A, 0xF8, 0x5A, 0x03, 0xF9, 0xF2, 0x0F, 0x06, 0xF1 };
        memcpy(&baroChip.regs[0x31], nvm, sizeof(nvm));
        baroChip.regs[0x03] = 0x20;
        const uint8_t data[6] = { 0x40, 0x2F, 0x63, 0x40, 0x1E, 0x7D };
        memcpy(&baroChip.regs[0x04], data, sizeof(data));
        Wire.attach(0x76, &baroChip);

        magChip.regs[0x00] = 0x80;            // QMC5883P, поле на север
        magChip.setLittleEndian16(0x01, 750);
        Wire.attach(0x2C, &magChip);

        Wire1.attach(0x3C, &screenChip);      // SSD1306 на второй шине
    }

    // Импульс выхода ServoChannel ch (канал LEDC = индекс), мкс.
    uint32_t pulseUs(uint8_t channel)
    {
        const fake::LedcChannel& ch = fake::ledc().channel[channel];
        return static_cast<uint32_t>(ch.duty * 20000.0 / 16384.0 + 0.5);
    }

    void sendFrame(const RcChannels& rc)
    {
        fake::uart(1)->pushRx(ibusFrame(rc));
    }

    // n тактов полётного цикла с кадром пульта перед каждым.
    void fly(const RcChannels& rc, int ticks)
    {
        for (int i = 0; i < ticks; ++i)
        {
            sendFrame(rc);
            loop();
        }
    }

    RcChannels armedSticks()
    {
        RcChannels rc;
        rc.set(Channels::ARM, 2000);
        return rc;
    }
}

void test_setup_brings_up_the_whole_bench()
{
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_EQUAL(4096u, Serial.txBuffer());   // буфер задан до begin()
    TEST_ASSERT_EQUAL_UINT32(115200, Serial.baud());
    TEST_ASSERT_TRUE(contains(log, "AEROS-001 FLIGHT CONTROLLER"));
    TEST_ASSERT_TRUE(contains(log, "Outputs: aileronLeft(GPIO4)=OK"));
    TEST_ASSERT_TRUE(contains(log, "WHO_AM_I=0x70 -> MPU6500"));
    TEST_ASSERT_TRUE(contains(log, "MPU6500: предполётная проверка пройдена"));
    TEST_ASSERT_TRUE(contains(log, "BMP388: подключён"));
    TEST_ASSERT_TRUE(contains(log, "BMP388: калибровка завершена"));
    TEST_ASSERT_TRUE(contains(log, "QMC5883P: подключён"));
    TEST_ASSERT_TRUE(contains(log, "Autopilot: инициализирован"));
    TEST_ASSERT_TRUE(contains(log, "OLED: подключён (SSD1306)"));
    TEST_ASSERT_TRUE(contains(log, "WebDebugServer: запуск точки доступа... OK"));
    TEST_ASSERT_TRUE(contains(log, "Готово. iBUS 115200 бод"));

    // Выходы — в безопасном положении до всякого пульта.
    TEST_ASSERT_UINT32_WITHIN(2, 1000, pulseUs(ServoChannel::ESC));
    TEST_ASSERT_UINT32_WITHIN(2, 1500, pulseUs(ServoChannel::AILERON_LEFT));
    TEST_ASSERT_EQUAL_UINT32(Config::IBUS_BAUDRATE, fake::uart(1)->baud());

    // Курс IMU при старте взят с компаса (поле вдоль +X — 0°).
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, imuSensor.getImuData().yaw);
    TEST_ASSERT_NOT_NULL(fake::findTask("web"));
    TEST_ASSERT_NOT_NULL(fake::findTask("oled"));
}

void test_loop_keeps_fixed_period_and_does_not_catch_up()
{
    loop();
    const uint32_t start = millis();
    for (int i = 0; i < 10; ++i) loop();
    TEST_ASSERT_EQUAL_UINT32(start + 10 * Config::LOOP_PERIOD_MS, millis());

    // Долгая блокировка (калибровка из консоли) — отсчёт с нуля, без
    // пачки "догоняющих" тактов.
    fake::advanceMs(500);
    loop();
    const uint32_t after = millis();
    loop();
    TEST_ASSERT_EQUAL_UINT32(after + Config::LOOP_PERIOD_MS, millis());
    takeSerial();
}

void test_sticks_move_surfaces_but_motor_needs_arm()
{
    RcChannels rc;
    rc.set(Channels::AILERON, 2000).set(Channels::THROTTLE, 1600);
    fly(rc, 5);
    TEST_ASSERT_UINT32_WITHIN(2, Config::AILERON_LEFT_REVERSED ? 1000 : 2000, pulseUs(ServoChannel::AILERON_LEFT));
    TEST_ASSERT_UINT32_WITHIN(2, 1000, pulseUs(ServoChannel::ESC));
    TEST_ASSERT_FALSE(flightController.isArmed());
}

void test_arm_then_throttle_reaches_esc()
{
    fly(RcChannels(), 2);           // тумблер OFF, газ внизу
    fly(armedSticks(), 2);          // OFF -> ON
    TEST_ASSERT_TRUE(flightController.isArmed());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ArmingManager: ARM"));

    RcChannels rc = armedSticks();
    rc.set(Channels::THROTTLE, 1500);
    fly(rc, 3);
    TEST_ASSERT_UINT32_WITHIN(2, 1500, pulseUs(ServoChannel::ESC));
}

void test_mode_switch_and_stabilisation_react_to_tilt()
{
    RcChannels rc = armedSticks();
    rc.set(Channels::AUX_2, 1500).set(Channels::THROTTLE, 1200);
    fly(rc, 3);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());

    // Правое крыло вниз на 20°: "верх" отклоняется влево, а ось X чипа
    // (IMU_ROTATION_CW_DEG = 90) смотрит вправо — на X чипа −sin 20°.
    // Крен вправо -> коррекция влево.
    imuChip.setBigEndian16(0x3B, static_cast<int16_t>(-2048 * 0.342f));
    imuChip.setBigEndian16(0x3F, static_cast<int16_t>(2048 * 0.94f));
    fly(rc, 50);
    TEST_ASSERT_GREATER_THAN_FLOAT(5.0f, imuSensor.getImuData().roll);
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, autopilot.getRollCorrection());

    imuChip.setBigEndian16(0x3B, 0);
    imuChip.setBigEndian16(0x3F, 2048);
    rc.set(Channels::AUX_2, 1000);
    fly(rc, 3);
    TEST_ASSERT_EQUAL(MODE_MANUAL, autopilot.getMode());
    takeSerial();
}

void test_link_loss_in_air_cuts_motor_and_glides()
{
    RcChannels rc = armedSticks();
    rc.set(Channels::THROTTLE, 1700);
    fly(rc, 3);
    TEST_ASSERT_UINT32_WITHIN(2, 1700, pulseUs(ServoChannel::ESC));

    for (int i = 0; i < 300; ++i) loop();   // 600 мс без кадров
    TEST_ASSERT_TRUE(flightController.isReceiverFailsafe());
    TEST_ASSERT_TRUE(autopilot.isFailsafeGliding());
    TEST_ASSERT_UINT32_WITHIN(2, 1000, pulseUs(ServoChannel::ESC));
    TEST_ASSERT_TRUE(contains(takeSerial(), "связь потеряна"));

    fly(rc, 3);   // связь вернулась
    TEST_ASSERT_FALSE(autopilot.isFailsafeGliding());
    TEST_ASSERT_UINT32_WITHIN(2, 1700, pulseUs(ServoChannel::ESC));

    RcChannels disarm;
    fly(disarm, 2);
    TEST_ASSERT_FALSE(flightController.isArmed());
    takeSerial();
}

void test_console_commands_over_serial()
{
    Serial.pushRx("s");
    loop();
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE(contains(status, "MPU6500: available=YES"));
    TEST_ASSERT_TRUE(contains(status, "BMP388: available=YES"));
    TEST_ASSERT_TRUE(contains(status, "QMC5883P: available=YES"));

    Serial.pushRx("p");
    loop();
    TEST_ASSERT_TRUE(contains(takeSerial(), "элерон L GPIO4:"));
}

void test_dashboard_status_and_mode_command()
{
    WebServer* http = fake::webServers().back();
    http->request(HTTP_GET, "/api/status");
    const std::string json = http->lastResponse().body;
    TEST_ASSERT_TRUE(contains(json, "\"armed\":false"));
    TEST_ASSERT_TRUE(contains(json, "\"imu\":{\"attached\":true,\"available\":true"));
    TEST_ASSERT_TRUE(contains(json, "\"gps\":{\"attached\":false"));   // GPS не выбран

    http->request(HTTP_POST, "/api/setmode", "{\"mode\": 3}");
    TEST_ASSERT_EQUAL(200, http->lastResponse().code);
    loop();   // команда применяется полётным циклом
    TEST_ASSERT_EQUAL(MODE_ALT_HOLD, autopilot.getMode());
    fake::runTask(*fake::findTask("web"), 2);
    takeSerial();
}

void test_oled_task_draws_live_state()
{
    fly(RcChannels(), 2);
    fake::runTask(*fake::findTask("oled"), 1);
    const U8G2* screen = fake::displays().back();
    TEST_ASSERT_FALSE(screen->lastFrame().empty());
    TEST_ASSERT_EQUAL_STRING("RX ok safe ALT", screen->lastFrame()[0].text.c_str());
    TEST_ASSERT_FALSE(screenChip.writes.empty());   // байты кадра ушли по Wire1
}

int main()
{
    wireUpBench();
    UNITY_BEGIN();
    RUN_TEST(test_setup_brings_up_the_whole_bench);
    RUN_TEST(test_loop_keeps_fixed_period_and_does_not_catch_up);
    RUN_TEST(test_sticks_move_surfaces_but_motor_needs_arm);
    RUN_TEST(test_arm_then_throttle_reaches_esc);
    RUN_TEST(test_mode_switch_and_stabilisation_react_to_tilt);
    RUN_TEST(test_link_loss_in_air_cuts_motor_and_glides);
    RUN_TEST(test_console_commands_over_serial);
    RUN_TEST(test_dashboard_status_and_mode_command);
    RUN_TEST(test_oled_task_draws_live_state);
    return UNITY_END();
}
