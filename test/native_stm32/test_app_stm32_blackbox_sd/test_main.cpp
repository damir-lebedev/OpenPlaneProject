// ============================================================
// Прошивка STM32H743 целиком (src/stm32/main.cpp) с SD-картой и
// чёрным ящиком: загрузка находит карту и файл BLACKBOX.BIN, задача
// bbox пишет полёт на карту, консоль выгружает его. Лётный набор —
// тот же, что в test_app_stm32_lsm6dsv_pitot.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_blackbox_sd
// ============================================================

#define SENSOR_KIT SENSOR_KIT_LSM6DSV_PITOT
#include "../../../src/stm32/main.cpp"

#include <unity.h>

#include "helpers/BlackBoxParse.h"
#include "helpers/ChipEmulators.h"
#include "helpers/FatImage.h"
#include "helpers/TestSupport.h"

using namespace bbparse;

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
    fat_image::Result image;

    constexpr uint32_t CARD_BLOCKS = 32768;
    constexpr uint32_t FILE_BYTES = 4u * 1024 * 1024;

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

    // Один такт: полётная задача, затем задача записи — как планировщик:
    // bbox ниже приоритетом и работает в паузе цикла.
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
        fake::runTask(*fake::findTask("bbox"), 2);   // ожидание уведомления — в начале цикла: первый проход пропускает, второй выполняет шаг
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

    // Весь файл BLACKBOX.BIN на карте глазами хранилища — как после перезагрузки.
    struct CardView
    {
        Stm32SdCard card;
        SdFileRegion file{ card, Config::BLACKBOX_SD_FILE };
        BlackBoxStorage storage{ file };

        CardView()
        {
            TEST_ASSERT_TRUE(card.begin());
            TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(file.begin()));
            TEST_ASSERT_TRUE(storage.begin());
        }
    };
}

void test_boot_finds_the_card_and_the_file_and_starts_the_writer_task()
{
    syncChips();
    setup();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "SD-карта: 16 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок"));
    TEST_ASSERT_TRUE(contains(log, "BlackBox: ждёт ARM и газ"));
    TEST_ASSERT_TRUE(contains(log, "BlackBox: готов за"));
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, blackBox.getState());
    TEST_ASSERT_EQUAL_UINT32(FILE_BYTES / BlackBoxFormat::SECTOR_SIZE - 1, blackBoxStorage.totalSectors());   // без служебного сектора

    // Задача записи: ниже полётной (вытесняется циклом), выше экрана и флеша настроек.
    const fake::TaskRecord* bbox = fake::findTask("bbox");
    const fake::TaskRecord* flight = fake::findTask("flight");
    const fake::TaskRecord* oled = fake::findTask("oled");
    TEST_ASSERT_NOT_NULL(bbox);
    TEST_ASSERT_EQUAL(Rtos::PRIORITY_TELEMETRY, bbox->priority);
    TEST_ASSERT_GREATER_THAN(bbox->priority, flight->priority);
    TEST_ASSERT_GREATER_THAN(oled->priority, bbox->priority);
    TEST_ASSERT_EQUAL_UINT32(8192, bbox->stackDepth);
}

void test_flight_goes_to_the_card_and_the_loop_keeps_its_period()
{
    fly(manual(), 3.0);   // земля: предзапись копится в очереди, карта простаивает
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, blackBox.getState());

    RcChannels rc = manual();
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1000);
    fly(rc, 0.5);
    rc.set(Channels::THROTTLE, 1500);
    const uint32_t start = millis();
    fly(rc, 12.0);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, blackBox.getState());
    TEST_ASSERT_UINT32_WITHIN(12, start + 12000, millis());   // запись на карту период цикла не растягивает

    rc.set(Channels::THROTTLE, 1000).set(Channels::ARM, 1000);
    fly(rc, Config::BLACKBOX_POSTROLL_MS / 1000.0 + 1.5);
    for (int i = 0; i < 5000 && blackBox.getState() == BlackBox::State::Stopping; ++i) tick(rc);   // дописать очередь
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, blackBox.getState());
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BlackBox: полёт #1 записан"), log.c_str());

    CardView card;
    TEST_ASSERT_EQUAL_MESSAGE(1u, card.storage.flightCount(), log.c_str());
    const std::vector<Rec> recs = readFlight(card.storage, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "board=STM32H743"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "sensor.imu=LSM6DSV"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "sensor.airspeed="));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "ARM"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "DISARM"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    TEST_ASSERT_TRUE(countType(recs, REC_IMU) > 7000);   // 3 с предзаписи + 12.5 с полёта + 10 с после, по 500 Гц
    TEST_ASSERT_TRUE(countType(recs, REC_CTRL) > 1400);
    TEST_ASSERT_TRUE(countType(recs, REC_BARO) > 100);
    TEST_ASSERT_TRUE(countType(recs, REC_GPS) > 100);
    TEST_ASSERT_TRUE(countType(recs, REC_AIR) > 100);
    TEST_ASSERT_TRUE(countType(recs, REC_SYS) >= 20);
    TEST_ASSERT_EQUAL_UINT32(0, sdCard.errors);
}

void test_console_lists_and_downloads_the_flight_over_serial()
{
    takeSerial();
    Serial.pushRx(std::string("\x02") + "bb list\n");
    fly(manual(), 0.01);
    const std::string list = takeSerial();
    TEST_ASSERT_TRUE(contains(list, "BB:STATE state=idle"));
    TEST_ASSERT_TRUE(contains(list, "BB:FLIGHT n=1 sectors="));
    TEST_ASSERT_TRUE(contains(list, "BB:END"));

    // Меню 'k' показывает карту так же, как раздел флеша.
    Serial.pushRx("k");
    fly(manual(), 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "#1 "));
}

int main()
{
    // Карта с FAT32 и файлом, залитым 0xFF, — до загрузки, как после sd-prepare.
    fake::sdCard().resize(CARD_BLOCKS);
    image = fat_image::format(fake::sdCard(), { { "BLACKBOX.BIN", FILE_BYTES } });
    std::fill_n(fake::sdCard().data.begin() + static_cast<size_t>(image.firstBlock.at("BLACKBOX.BIN")) * 512, FILE_BYTES, 0xFF);

    // Шины создаются глобальными объектами main.cpp — до main().
    wireUpKit();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_the_card_and_the_file_and_starts_the_writer_task);
    RUN_TEST(test_flight_goes_to_the_card_and_the_loop_keeps_its_period);
    RUN_TEST(test_console_lists_and_downloads_the_flight_over_serial);
    return UNITY_END();
}
