// ============================================================
// Прошивка STM32H743 целиком (src/stm32/main.cpp) на ПК, с лётным
// набором SENSOR_KIT_LSM6DSV_PITOT:
//   LSM6DSV (I2C2 0x6A) + QMC6309 (0x7C), SPL06-001 (0x76) — статика,
//   BMP581 (0x47) — трубка Пито, u-blox M10 — USART3 (PD9),
//   iBUS — UART7 (PE7), радиомодем MAVLink — UART4 (PD0/PD1),
//   OLED — I2C1 (PB9/PB8), сервовыходы — TIM2/TIM4.
// Слой STM32duino — test/native/support_stm32; <Preferences.h> —
// настоящий compat/Preferences.h (KeyValueStore во "флеше").
// Задачи FreeRTOS тест запускает сам: fake::runTask("flight", n).
//
// Запуск: pio test -e native-stm32
// ============================================================

#define SENSOR_KIT SENSOR_KIT_LSM6DSV_PITOT
#include "../../../src/stm32/main.cpp"

#include <unity.h>

#include <map>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"
#include "telemetry/MavlinkCodec.h"

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

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

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
        TwoWire* display = fake::wireWithSda(PB9);
        TEST_ASSERT_NOT_NULL(display);
        display->attach(0x3C, &screenChip);
    }

    void syncChips()
    {
        imuChip.update(world);
        magChip.update(world);
        staticChip.update(world);
        tubeChip.setSample(world.tubePa(), world.temperatureC);
    }

    // Один такт задачи flight с кадром пульта (и GPS раз в 100 мс).
    void tick(const RcChannels& rc)
    {
        syncChips();
        port(PE7).pushRx(ibusFrame(rc));
        if (millis() - lastGpsMs >= 100)
        {
            lastGpsMs = millis();
            const std::vector<uint8_t> frame = chips::navPvt(world);
            port(PD9).pushRx(std::string(frame.begin(), frame.end()));
        }
        fake::runTask(*fake::findTask("flight"), 1);
    }

    void fly(const RcChannels& rc, double seconds)
    {
        const int ticks = static_cast<int>(seconds * 1000 / Config::LOOP_PERIOD_MS);
        for (int i = 0; i < ticks; ++i) tick(rc);
    }

    RcChannels manual()
    {
        RcChannels rc;
        rc.set(Channels::SWC, 1000);
        return rc;
    }

    std::vector<Mavlink::Message> takeMavlink()
    {
        const std::string bytes = port(PD0).takeTx();
        Mavlink::Parser parser;
        std::vector<Mavlink::Message> out;
        for (char c : bytes)
        {
            if (parser.feed(static_cast<uint8_t>(c))) out.push_back(parser.message());
        }
        TEST_ASSERT_EQUAL_MESSAGE(0, parser.badCrcCount(), "кадр MAVLink с неверной CRC");
        return out;
    }

    const Mavlink::Message* last(const std::vector<Mavlink::Message>& messages, uint32_t id)
    {
        for (auto it = messages.rbegin(); it != messages.rend(); ++it)
        {
            if (it->msgid == id) return &*it;
        }
        return nullptr;
    }

    // Образ настроек прямо из "флеша" EEPROM-эмуляции.
    class FlashView : public IFlashStorage
    {
    public:
        size_t capacity() const override { return KeyValueStore::CAPACITY; }
        void read(uint8_t* destination, size_t size) override { memcpy(destination, fake::eeprom().flash, size); }
        bool write(const uint8_t*, size_t) override { return false; }
    };
}

void test_boot_brings_up_board_sensors_and_tasks()
{
    syncChips();
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "STM32H743 / FreeRTOS"));
    TEST_ASSERT_TRUE(contains(log, "Настройки во флеше: пусто"));   // чистый флеш — не ошибка
    TEST_ASSERT_TRUE(contains(log, "LSM6DSV: подключён (LSM6DSV/16X)"));
    TEST_ASSERT_TRUE(contains(log, "QMC6309: подключён"));
    TEST_ASSERT_TRUE(contains(log, "SPL06: подключён"));
    TEST_ASSERT_TRUE(contains(log, "PITOT-BMP581: подключён"));
    TEST_ASSERT_TRUE(contains(log, "OLED: подключён (SSD1306)"));
    TEST_ASSERT_TRUE(contains(log, "MAVLink 57600"));

    // Пины шин — из блока BOARD_STM32H743.
    TEST_ASSERT_EQUAL(PB11, Wire.sdaPin());
    TEST_ASSERT_EQUAL(PB10, Wire.sclPin());
    TEST_ASSERT_EQUAL_UINT32(Config::IBUS_BAUDRATE, port(PE7).baud());
    TEST_ASSERT_EQUAL_UINT32(Config::TELEM_BAUDRATE, port(PD0).baud());

    // Задачи: полёт — высший приоритет, экран и флеш — фоном.
    const fake::TaskRecord* flight = fake::findTask("flight");
    const fake::TaskRecord* oled = fake::findTask("oled");
    const fake::TaskRecord* storage = fake::findTask("storage");
    TEST_ASSERT_NOT_NULL(flight);
    TEST_ASSERT_NOT_NULL(oled);
    TEST_ASSERT_NOT_NULL(storage);
    TEST_ASSERT_EQUAL(Rtos::PRIORITY_FLIGHT, flight->priority);
    TEST_ASSERT_GREATER_THAN(oled->priority, flight->priority);
    TEST_ASSERT_GREATER_THAN(storage->priority, flight->priority);
    TEST_ASSERT_EQUAL_UINT32(16384, flight->stackDepth);
    TEST_ASSERT_TRUE(fake::schedulerStarted());

    // Выходы — таймеры, сразу в безопасном положении.
    TEST_ASSERT_EQUAL(1000, fake::timerPulseUs(Config::PIN_ESC));
    TEST_ASSERT_EQUAL(1500, fake::timerPulseUs(Config::PIN_AILERON_LEFT));
    TEST_ASSERT_EQUAL(1500, fake::timerPulseUs(Config::PIN_RUDDER));
}

void test_flight_task_keeps_2ms_period()
{
    const uint32_t start = millis();
    fly(manual(), 0.2);
    TEST_ASSERT_UINT32_WITHIN(2, start + 200, millis());
    takeSerial();
}

void test_pitot_zero_and_airspeed_on_stm32()
{
    fly(manual(), 1.5);
    TEST_ASSERT_FALSE(pitotSensor.isZeroing());
    world.airspeedMs = 18;
    fly(manual(), 1.0);
    TEST_ASSERT_FLOAT_WITHIN(0.6f, 18.0f, pitotSensor.getAirspeedData().indicatedMs);
    takeSerial();
}

void test_sticks_and_stabilize_drive_timer_outputs()
{
    RcChannels rc = manual();
    rc.set(Channels::AILERON, 2000);
    fly(rc, 0.1);
    TEST_ASSERT_EQUAL(Config::AILERON_LEFT_REVERSED ? 1000 : 2000, fake::timerPulseUs(Config::PIN_AILERON_LEFT));

    rc = manual();
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.2);
    TEST_ASSERT_TRUE(flightController.isArmed());
    TEST_ASSERT_TRUE(autopilot.getNavStatus().homeValid);   // GPS был — дом записан

    rc.set(Channels::SWC, 1500).set(Channels::THROTTLE, 1400);
    world.rollDeg = 20;
    fly(rc, 1.0);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());
    TEST_ASSERT_LESS_THAN_FLOAT(-20.0f, autopilot.getRollCorrection());
    TEST_ASSERT_EQUAL(cappedThrottleUs(1400), fake::timerPulseUs(Config::PIN_ESC));
    // Выход реально читается обратно (самопроверка 'p' на STM32 — pulseIn()).
    TEST_ASSERT_EQUAL(cappedThrottleUs(1400), pulseIn(static_cast<uint8_t>(Config::PIN_ESC), HIGH));
    world.rollDeg = 0;
    takeSerial();
}

void test_mavlink_shows_the_flight_to_ground_station()
{
    world.altitudeM = 25;
    world.groundSpeedMs = 16;
    RcChannels rc = manual();
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 1500).set(Channels::THROTTLE, 1400);
    port(PD0).takeTx();
    fly(rc, 5.5);   // HOME_POSITION — раз в 5 с
    const auto msgs = takeMavlink();

    std::map<uint32_t, int> counts;
    for (const auto& m : msgs) counts[m.msgid]++;
    TEST_ASSERT_INT_WITHIN(3, 55, counts[Mavlink::Msg::ATTITUDE]);
    TEST_ASSERT_INT_WITHIN(1, 5, counts[Mavlink::Msg::HEARTBEAT]);

    const Mavlink::Message* hb = last(msgs, Mavlink::Msg::HEARTBEAT);
    TEST_ASSERT_NOT_NULL(hb);
    TEST_ASSERT_EQUAL(MavlinkModes::PLANE_FBWA, hb->u32(0));    // STABILIZE -> FBWA
    TEST_ASSERT_EQUAL(128, hb->u8(6) & 128);                    // заармлен

    const Mavlink::Message* hud = last(msgs, Mavlink::Msg::VFR_HUD);
    TEST_ASSERT_NOT_NULL(hud);
    TEST_ASSERT_FLOAT_WITHIN(0.8f, 18.0f, hud->f32(0));   // скорость — по трубке
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 16.0f, hud->f32(4));   // путевая — GPS
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 25.0f, hud->f32(8));   // высота — барометр
    TEST_ASSERT_EQUAL((cappedThrottleUs(1400) - 1000) / 10, hud->u16(18));   // газ: стик 1400 с учётом лимита

    const Mavlink::Message* status = last(msgs, Mavlink::Msg::SYS_STATUS);
    TEST_ASSERT_NOT_NULL(status);
    TEST_ASSERT_EQUAL_HEX32(0x3F, status->u32(8) & 0x3F);   // все датчики исправны
    TEST_ASSERT_NOT_NULL(last(msgs, Mavlink::Msg::HOME_POSITION));
}

void test_mode_change_from_ground_station()
{
    // QGroundControl: SET_MODE LOITER (12).
    Mavlink::Encoder gcs(255, 190);
    Mavlink::Payload p;
    p.u32(MavlinkModes::PLANE_LOITER).u8(1).u8(1);
    uint8_t frame[Mavlink::MAX_FRAME];
    const size_t n = gcs.encode(frame, Mavlink::Msg::SET_MODE, p);
    port(PD0).pushRx(std::string(reinterpret_cast<const char*>(frame), n));

    RcChannels rc = manual();
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 1500).set(Channels::THROTTLE, 1400);
    fly(rc, 0.1);
    TEST_ASSERT_EQUAL(MODE_LOITER, autopilot.getMode());
    fly(rc, 1.2);
    const auto msgs = takeMavlink();
    TEST_ASSERT_EQUAL(MavlinkModes::PLANE_LOITER, last(msgs, Mavlink::Msg::HEARTBEAT)->u32(0));

    rc.set(Channels::SWC, 1000).set(Channels::ARM, 1000);
    fly(rc, 0.2);
    TEST_ASSERT_EQUAL(MODE_MANUAL, autopilot.getMode());
    takeSerial();
}

void test_settings_go_to_flash_in_background_task()
{
    const unsigned flushesBefore = fake::eeprom().flushes;

    // Меню лога: сменить канал, закрыть — настройки сохраняются (не заармлен).
    Serial.pushRx("l1l");
    fly(manual(), 0.1);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Настройки лога сохранены"));

    // Полётная задача флеш не трогает: запись ждёт фоновую задачу.
    TEST_ASSERT_EQUAL(flushesBefore, fake::eeprom().flushes);
    TEST_ASSERT_TRUE(Stm32FlashStorage::instance().hasPending());

    fake::runTask(*fake::findTask("storage"), 1);
    TEST_ASSERT_EQUAL(flushesBefore + 1, fake::eeprom().flushes);
    TEST_ASSERT_FALSE(Stm32FlashStorage::instance().hasPending());

    // Во флеше — образ с CRC, в нём настройки лога.
    FlashView view;
    KeyValueStore reread(view);
    reread.mount();
    TEST_ASSERT_FALSE(reread.wasCorrupt());
    TEST_ASSERT_TRUE(reread.bytesUsed() > KeyValueStore::HEADER_SIZE);
    TEST_ASSERT_EQUAL_MEMORY("OPKV", fake::eeprom().flash, 4);

    // Пустой проход фоновой задачи флеш не трогает.
    fake::runTask(*fake::findTask("storage"), 2);
    TEST_ASSERT_EQUAL(flushesBefore + 1, fake::eeprom().flushes);
}

void test_oled_task_draws_over_second_i2c()
{
    fake::runTask(*fake::findTask("oled"), 1);
    TEST_ASSERT_FALSE(fake::displays().back()->lastFrame().empty());
    TEST_ASSERT_FALSE(screenChip.writes.empty());   // кадр ушёл по I2C1, а не по шине датчиков
}

void test_console_on_stm32()
{
    Serial.pushRx("b");
    fly(manual(), 0.01);
    const std::string scan = takeSerial();
    TEST_ASSERT_TRUE(contains(scan, "0x6A  LSM6DSV"));
    TEST_ASSERT_TRUE(contains(scan, "0x3C  OLED"));

    Serial.pushRx("p");
    fly(manual(), 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Выходы"));
}

int main()
{
    // Шины создаются глобальными объектами main.cpp — до main().
    wireUpKit();
    UNITY_BEGIN();
    RUN_TEST(test_boot_brings_up_board_sensors_and_tasks);
    RUN_TEST(test_flight_task_keeps_2ms_period);
    RUN_TEST(test_pitot_zero_and_airspeed_on_stm32);
    RUN_TEST(test_sticks_and_stabilize_drive_timer_outputs);
    RUN_TEST(test_mavlink_shows_the_flight_to_ground_station);
    RUN_TEST(test_mode_change_from_ground_station);
    RUN_TEST(test_settings_go_to_flash_in_background_task);
    RUN_TEST(test_oled_task_draws_over_second_i2c);
    RUN_TEST(test_console_on_stm32);
    return UNITY_END();
}
