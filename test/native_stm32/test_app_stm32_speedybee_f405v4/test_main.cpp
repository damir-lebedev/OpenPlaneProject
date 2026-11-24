// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на SpeedyBee
// F405 V4 (P57): ICM-42688-P на SPI1, DPS310 на I2C1, приёмник iBUS на UART2,
// слот microSD на SPI3 (CS PC14) — эмулятор карты на фейковой шине. Загрузка
// находит карту и файл BLACKBOX.BIN, ARM включает запись, задача bbox пишет
// полёт на карту (блоками по SPI); после «перезагрузки» полёт читается с
// карты; blackbox_device = SDCARD, blackbox_sample_rate — 1/2.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_speedybee_f405v4
// ============================================================

#define BOARD_STM32F405
#define OP_BOARD_FILE "config/boards/SPEEDYBEEF405V4.h"
#define SENSOR_KIT SENSOR_KIT_BF_BOARD
#define VEHICLE_MULTIROTOR
#include "../../../src/stm32/main_copter.cpp"

#include <unity.h>

#include <string>
#include <vector>

#include "helpers/BlackBoxParse.h"
#include "helpers/ChipEmulators.h"
#include "helpers/FatImage.h"
#include "helpers/SpiSdCardChip.h"
#include "helpers/TestSupport.h"

using namespace bbparse;

void setUp() {}
void tearDown() {}

namespace
{
    constexpr uint32_t FILE_BYTES = 8u * 1024 * 1024;

    World world;
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;              // DPS310 — те же регистры
    chips::SpiSdCardChip cardChip(32768);   // 16 МБ SDHC
    fat_image::Result image;

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
        image = fat_image::format(cardChip, { { "BLACKBOX.BIN", FILE_BYTES } });   // tools/blackbox.py sd-prepare
        std::fill_n(cardChip.data.begin() + static_cast<size_t>(image.firstBlock.at("BLACKBOX.BIN")) * 512, FILE_BYTES, 0xFF);
        cardChip.attach(sdSpi, static_cast<uint8_t>(PC14));
        fake::gpio().analogMv[PC0] = 25200 / 11;
    }

    void tick(const RcChannels& rc)
    {
        imuChip.update(world);
        baroChip.update(world);
        if (millis() % 7 == 0) port(PA3).pushRx(ibusFrame(rc));
        fake::runTask(*fake::findTask("flight"), 1);
        fake::runTask(*fake::findTask("bbox"), 2);
    }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(rc);
    }

    void host(const char* command) { Serial.pushRx(std::string(1, '\x02') + command + "\n"); }
}

void test_boot_finds_the_card()
{
    imuChip.update(world);
    baroChip.update(world);
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "SD-карта: 16 МБ, файл BLACKBOX.BIN: ок"), log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BlackBox: ждёт ARM"), log.c_str());
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "SPI-флеш"), log.c_str());
    TEST_ASSERT_TRUE(sdCard.highCapacity());
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, copterParams.value(copter::PARAM_BLACKBOX_SAMPLE_RATE));   // 1/2
    TEST_ASSERT_EQUAL_FLOAT(copter::BLACKBOX_SDCARD, copterParams.value(copter::PARAM_BLACKBOX_DEVICE));
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "ОШИБКА"), log.c_str());
}

void test_flight_goes_to_the_card()
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
    takeSerial();

    RcChannels rc;
    run(rc, 3.0);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Recording, copterBlackBox.getState());
    rc.set(Channels::THROTTLE, 1300);
    run(rc, 3.0);
    rc.set(Channels::THROTTLE, 1000).set(Channels::ARM, 1000);
    run(rc, CopterBlackBox::POSTROLL_MS / 1000.0 + 0.5);
    for (int i = 0; i < 5000 && copterBlackBox.getState() == BlackBoxCore::State::Stopping; ++i) tick(rc);
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    TEST_ASSERT_TRUE_MESSAGE(contains(takeSerial(), "BlackBox: полёт #1 записан"), "полёт не закрылся");

    // карта глазами нового драйвера и хранилища — как после перезагрузки
    SpiSdCard reread(sdBus, PC14);
    TEST_ASSERT_TRUE(reread.begin());
    SdFileRegion file(reread, "BLACKBOX.BIN");
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(file.begin()));
    BlackBoxStorage storage(file);
    TEST_ASSERT_TRUE(storage.begin());
    TEST_ASSERT_EQUAL_UINT32(1, storage.flightCount());
    const std::vector<Rec> recs = readFlight(storage, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "vehicle=copter"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    // 1/2: каждый второй цикл 1 кГц — ~3.6 с ARM + 2 с после + предзапись очереди
    const size_t gyro = countType(recs, REC_QGYRO);
    char why[96];
    snprintf(why, sizeof(why), "QGYRO %u", static_cast<unsigned>(gyro));
    TEST_ASSERT_TRUE_MESSAGE(gyro > 2200 && gyro < 4200, why);
    TEST_ASSERT_GREATER_THAN_UINT32(100, cardChip.blockWrites);
    TEST_ASSERT_EQUAL_UINT32(0, sdCard.timeoutCount());
    TEST_ASSERT_FALSE(hasText(recs, REC_EVENT, "очередь переполнена"));   // предзапись длиннее очереди — не потеря
}

void test_bb_list_sees_the_flight()
{
    RcChannels rc;
    host("bb list");
    run(rc, 0.05);
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BB:FLIGHT n=1"), log.c_str());
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_the_card);
    RUN_TEST(test_flight_goes_to_the_card);
    RUN_TEST(test_bb_list_sees_the_flight);
    return UNITY_END();
}
