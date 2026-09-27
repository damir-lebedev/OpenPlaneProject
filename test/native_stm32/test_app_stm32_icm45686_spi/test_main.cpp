// ============================================================
// Прошивка STM32H743 целиком (src/stm32/main.cpp) с датчиками на SPI:
//   ICM-45686 (SPI2, CS PB12) + QMC6309 (I2C2 0x7C),
//   BMP581 (SPI2, CS PD10) — основной барометр, без трубки Пито,
//   u-blox M10 — USART3, MAVLink — UART4.
// Флеш при включении испорчен (питание пропало во время записи):
// прошивка должна стартовать со значениями по умолчанию.
//
// Запуск: pio test -e native-stm32
// ============================================================

#define SENSOR_KIT SENSOR_KIT_CUSTOM
#define SENSOR_IMU SENSOR_IMU_ICM45686_SPI
#define SENSOR_BARO SENSOR_BARO_BMP581_SPI
#define SENSOR_MAG SENSOR_MAG_QMC6309
#define SENSOR_AIRSPEED SENSOR_AIRSPEED_NONE
#define SENSOR_GPS SENSOR_GPS_UBLOX_M10
#include "../../../src/stm32/main.cpp"

#include <unity.h>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"
#include "telemetry/MavlinkCodec.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    chips::Icm45686 imuChip;
    chips::Qmc6309 magChip;
    chips::Bmp581 baroChip;
    uint32_t lastGpsMs = 0;
    bool linkUp = true;

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL(p);
        return *p;
    }

    void wireUpKit()
    {
        imuChip.attach(SPI, static_cast<uint8_t>(Config::PIN_SPI_CS_IMU));
        imuChip.install();
        baroChip.attach(SPI, static_cast<uint8_t>(Config::PIN_SPI_CS_BARO));
        baroChip.install();
        magChip.attach(Wire, 0x7C);
        magChip.install();

        // "Питание пропало во время записи": магия есть, CRC — нет.
        const uint8_t broken[16] = { 'O', 'P', 'K', 'V', 1, 0, 4, 0, 0xDE, 0xAD, 0xBE, 0xEF, 1, 1, 0, 0 };
        memcpy(fake::eeprom().flash, broken, sizeof(broken));
    }

    void syncChips()
    {
        imuChip.update(world);
        magChip.update(world);
        baroChip.setSample(world.staticPa(), world.temperatureC);
    }

    void tick(const RcChannels& rc)
    {
        syncChips();
        if (linkUp) port(PE7).pushRx(ibusFrame(rc));
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

    std::vector<Mavlink::Message> takeMavlink()
    {
        const std::string bytes = port(PD0).takeTx();
        Mavlink::Parser parser;
        std::vector<Mavlink::Message> out;
        for (char c : bytes)
        {
            if (parser.feed(static_cast<uint8_t>(c))) out.push_back(parser.message());
        }
        TEST_ASSERT_EQUAL(0, parser.badCrcCount());
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

    void sendFromGcs(uint32_t msgid, const Mavlink::Payload& p)
    {
        static Mavlink::Encoder gcs(255, 190);
        uint8_t frame[Mavlink::MAX_FRAME];
        const size_t n = gcs.encode(frame, msgid, p);
        port(PD0).pushRx(std::string(reinterpret_cast<const char*>(frame), n));
    }
}

void test_boot_with_spi_sensors_and_broken_flash()
{
    syncChips();
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "Настройки во флеше: образ повреждён"));
    TEST_ASSERT_TRUE(contains(log, "ICM45686: подключён"));
    TEST_ASSERT_TRUE(contains(log, "BMP581: подключён"));
    TEST_ASSERT_TRUE(contains(log, "QMC6309: подключён"));
    TEST_ASSERT_FALSE(contains(log, "PITOT"));
    TEST_ASSERT_FALSE(contains(log, "OLED: подключён"));   // экрана на шине нет

    // SPI2 на пинах из Config.h.
    TEST_ASSERT_EQUAL(PB13, SPI.stm32Pins[0]);
    TEST_ASSERT_EQUAL(PB14, SPI.stm32Pins[1]);
    TEST_ASSERT_EQUAL(PB15, SPI.stm32Pins[2]);
    TEST_ASSERT_NULL(autopilot.getAirspeedSensor());
    TEST_ASSERT_TRUE(imuSensor.isAvailable() && baroSensor.isAvailable() && magSensor.isAvailable());
}

void test_alt_hold_from_ground_station_holds_height()
{
    world.altitudeM = 40;
    RcChannels rc;
    rc.set(Channels::SWC, 1000);
    fly(rc, 1.0);
    rc.set(Channels::ARM, 2000);
    fly(rc, 0.3);
    TEST_ASSERT_TRUE(flightController.isArmed());

    // COMMAND_LONG DO_SET_MODE, custom = FBWB (ALT_HOLD).
    Mavlink::Payload p;
    p.f32(1).f32(static_cast<float>(MavlinkModes::PLANE_FBWB)).f32(0).f32(0).f32(0).f32(0).f32(0)
     .u16(176).u8(1).u8(1).u8(0);
    sendFromGcs(Mavlink::Msg::COMMAND_LONG, p);
    rc.set(Channels::THROTTLE, 1500);
    fly(rc, 0.5);
    TEST_ASSERT_EQUAL(MODE_ALT_HOLD, autopilot.getMode());
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 40.0f, autopilot.getTargetAltitude());

    // Просели на 6 м — руль высоты тянет вверх.
    world.altitudeM = 34;
    fly(rc, 2.0);
    TEST_ASSERT_GREATER_THAN_FLOAT(3.0f, autopilot.getDesiredPitch());
    const auto msgs = takeMavlink();
    const Mavlink::Message* ack = last(msgs, Mavlink::Msg::COMMAND_ACK);
    TEST_ASSERT_NOT_NULL(ack);
    TEST_ASSERT_EQUAL(0, ack->u8(2));
    world.altitudeM = 40;
    takeSerial();
}

void test_link_loss_returns_home_and_ground_station_sees_it()
{
    // Далеко от дома, связь пропала — FAILSAFE_RTH с мотором.
    world.lat += 400.0 / 111195.0;
    world.groundSpeedMs = 15;
    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 1500).set(Channels::THROTTLE, 1400);
    fly(rc, 1.0);
    port(PD0).takeTx();

    linkUp = false;
    fly(rc, 1.5);
    TEST_ASSERT_TRUE(flightController.isReceiverFailsafe());
    TEST_ASSERT_TRUE(autopilot.isFailsafeReturning());
    TEST_ASSERT_GREATER_THAN(1100, fake::timerPulseUs(Config::PIN_ESC));   // мотор работает

    const auto msgs = takeMavlink();
    const Mavlink::Message* hb = last(msgs, Mavlink::Msg::HEARTBEAT);
    TEST_ASSERT_NOT_NULL(hb);
    TEST_ASSERT_EQUAL(MavlinkModes::PLANE_RTL, hb->u32(0));
    TEST_ASSERT_EQUAL(5, hb->u8(7));   // CRITICAL
    bool failsafeText = false;
    for (const auto& m : msgs)
    {
        char text[51];
        m.chars(1, 50, text);
        if (m.msgid == Mavlink::Msg::STATUSTEXT && std::string(text) == "FAILSAFE: return home") failsafeText = true;
    }
    TEST_ASSERT_TRUE(failsafeText);

    linkUp = true;
    fly(rc, 0.3);
    TEST_ASSERT_FALSE(autopilot.isFailsafeActive());
    rc.set(Channels::ARM, 1000);
    fly(rc, 0.2);
    takeSerial();
}

void test_broken_flash_is_rewritten_by_first_save()
{
    Serial.pushRx("l1l");
    RcChannels rc;
    fly(rc, 0.1);
    fake::runTask(*fake::findTask("storage"), 1);

    class FlashView : public IFlashStorage
    {
    public:
        size_t capacity() const override { return KeyValueStore::CAPACITY; }
        void read(uint8_t* destination, size_t size) override { memcpy(destination, fake::eeprom().flash, size); }
        bool write(const uint8_t*, size_t) override { return false; }
    } view;
    KeyValueStore reread(view);
    reread.mount();
    TEST_ASSERT_FALSE(reread.wasCorrupt());
    TEST_ASSERT_TRUE(reread.bytesUsed() > KeyValueStore::HEADER_SIZE);
    takeSerial();
}

int main()
{
    wireUpKit();
    UNITY_BEGIN();
    RUN_TEST(test_boot_with_spi_sensors_and_broken_flash);
    RUN_TEST(test_alt_hold_from_ground_station_holds_height);
    RUN_TEST(test_link_loss_returns_home_and_ground_station_sees_it);
    RUN_TEST(test_broken_flash_is_rewritten_by_first_save);
    return UNITY_END();
}
