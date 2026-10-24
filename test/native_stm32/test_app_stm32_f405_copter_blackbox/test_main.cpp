// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на F405 с
// SPI-флешем (AocodaRC F405 V3: W25Q128 на SPI3, CS PC0) — эмулятор
// микросхемы на фейковой шине. Загрузка опознаёт флеш по JEDEC ID, ARM
// включает запись, задача bbox пишет полёт во флеш страницами; `bb list`,
// `flash_info` и `flash_erase` (как в BF); половина флеша держится
// стёртой; blackbox_sample_rate по умолчанию 1/2.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_f405_copter_blackbox
// ============================================================

#define BOARD_STM32F405
#define OP_BOARD_FILE "config/boards/AOCODARCF405V3.h"
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
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;
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
        baroChip.attach(Wire, 0x76);
        baroChip.install();
        flashChip.attach(flashSpi, static_cast<uint8_t>(PC0));
        fake::gpio().analogMv[PC2] = 25200 / 11;
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

void test_boot_finds_the_flash()
{
    imuChip.update(world);
    baroChip.update(world);
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "SPI-флеш ящика: Winbond, JEDEC 0xEF4018, 16 МБ"), log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BlackBox: ждёт ARM"), log.c_str());
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    TEST_ASSERT_EQUAL_UINT32(8u * 1024 * 1024, copterBlackBox.minFree());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, copterParams.value(copter::PARAM_BLACKBOX_SAMPLE_RATE));   // 1/2
    TEST_ASSERT_EQUAL_FLOAT(copter::BLACKBOX_SPIFLASH, copterParams.value(copter::PARAM_BLACKBOX_DEVICE));
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "ОШИБКА"), log.c_str());
}

void test_flight_goes_to_the_flash()
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

    // флеш глазами нового хранилища — как после перезагрузки
    SpiNorFlash reread(flashOwnBus, PC0);
    TEST_ASSERT_TRUE(reread.begin());
    BlackBoxStorage storage(reread);
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
    TEST_ASSERT_GREATER_THAN_UINT32(100, flashChip.programs);   // страницами
    TEST_ASSERT_EQUAL_UINT32(0, flashChip.rejected);            // WREN и ожидание «занята» — всегда
    TEST_ASSERT_EQUAL_UINT32(0, flashChip.readsWhileBusy);
}

void test_flash_info_bb_list_and_erase()
{
    RcChannels rc;
    Serial.pushRx("#flash_info\r\nexit\r\n");
    run(rc, 0.05);
    std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Flash sectors=4096, sectorSize=4096, pageSize=256, totalSize=16777216 JEDEC ID=0xef4018 (Winbond)"),
                             log.c_str());
    host("bb list");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BB:FLIGHT n=1"), log.c_str());

    Serial.pushRx("#flash_erase\r\nexit\r\n");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Erasing, please wait ...") && contains(log, "Done."), log.c_str());
    TEST_ASSERT_EQUAL_UINT32(1, flashChip.chipErases);   // одной командой, а не 4096 секторов
    host("bb list");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "BB:FLIGHT"), log.c_str());
    TEST_ASSERT_EQUAL_UINT32(0, flashChip.rejected);
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_the_flash);
    RUN_TEST(test_flight_goes_to_the_flash);
    RUN_TEST(test_flash_info_bb_list_and_erase);
    return UNITY_END();
}
