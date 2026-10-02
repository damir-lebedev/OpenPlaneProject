// ============================================================
// Чёрный ящик на SD-карте (STM32H743): весь путь от файла на FAT32 до
// записей полёта.
//
//   Fat32::locate        — MBR и без него, каталог на двух кластерах,
//                          шумовые записи, разбросанный/пустой/чужой том;
//   SdFileRegion         — неполные блоки, кэш, стирание 0xFF, границы,
//                          сбои карты;
//   Stm32SdCard          — настоящий драйвер поверх фейкового HAL_SD:
//                          4-битная шина, запасные скорости, повтор при
//                          сбое, занятая карта, невыровненные буферы;
//   BlackBoxStorage      — кольцо на карте: пережить перезагрузку и
//                          пропажу питания, стоимость сверки при включении;
//   BlackBox             — запись полёта на настоящих FlightController и
//                          Autopilot, сбойная перезагрузка по RCC->RSR, АЦП
//                          батареи, ошибки носителя, выгрузка по консоли.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_blackbox_sd
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <algorithm>
#include <string>
#include <vector>

#include "autopilot/Autopilot.h"
#include "autopilot/PilotSwitches.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "hal/SdFileRegion.h"
#include "hal/stm32/Stm32SdCard.h"
#include "helpers/BlackBoxParse.h"
#include "helpers/FatImage.h"
#include "helpers/TestSupport.h"
#include "rc/IBusReceiver.h"
#include "storage/Fat32File.h"
#include "telemetry/BlackBox.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/DebugConsole.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LoopStats.h"

using namespace BlackBoxFormat;
using namespace bbparse;

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    constexpr uint32_t BLOCK = IBlockDevice::BLOCK_SIZE;
    constexpr uint32_t SECTOR = BlackBoxFormat::SECTOR_SIZE;
    constexpr uint32_t CARD_BLOCKS = 32768;                 // карта 16 МБ
    constexpr uint32_t FILE_BYTES = 4u * 1024 * 1024;       // BLACKBOX.BIN: 1024 сектора, первый — служебный
    constexpr uint32_t RING_BYTES = FILE_BYTES - SECTOR;    // что видит SdFileRegion

    // Карта с FAT32 и файлом BLACKBOX.BIN, залитым 0xFF — как после sd-prepare.
    fat_image::Result prepareCard(uint32_t fileBytes = FILE_BYTES, bool fillFf = true,
                                  const fat_image::Options& options = fat_image::Options())
    {
        fake::SdCardModel& card = fake::sdCard();
        card.resize(CARD_BLOCKS);
        const fat_image::Result image = fat_image::format(card, { { "BLACKBOX.BIN", fileBytes } }, options);
        if (fillFf)
        {
            std::fill_n(card.data.begin() + static_cast<size_t>(image.firstBlock.at("BLACKBOX.BIN")) * BLOCK, fileBytes, 0xFF);
        }
        return image;
    }

    // Начало файла на карте: служебный сектор (метка "кольцо пусто").
    uint8_t* markBytes(const fat_image::Result& image)
    {
        return fake::sdCard().data.data() + static_cast<size_t>(image.firstBlock.at("BLACKBOX.BIN")) * BLOCK;
    }

    // Начало кольца: нулевое смещение SdFileRegion — сразу за служебным сектором.
    uint8_t* fileBytes(const fat_image::Result& image) { return markBytes(image) + SECTOR; }

    // --------------------------------------------------------
    // Самолёт целиком + чёрный ящик на карте
    // --------------------------------------------------------

    struct Rig
    {
        FakeBoard board;
        FakeImu imu;
        FakeBaro baro;
        FakeGps gps;
        IBusReceiver receiver{ board.rcUart() };
        ControlMixer mixer;
        ThrottleManager throttle;
        FlightOutputs outputs{ board };
        Autopilot autopilot{ &imu, &baro, nullptr, &gps };
        PilotSwitches switches{ &autopilot };
        ArmingManager arming{ &autopilot };
        FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &switches };
        LoopStats stats;
        Stm32SdCard card;
        SdFileRegion file{ card, Config::BLACKBOX_SD_FILE };
        BlackBoxStorage storage{ file };
        BlackBox box{ controller, autopilot, stats, storage, &switches };
        RcChannels rc;
        bool started = false;

        Rig()
        {
            imu.data.accelZ = 1.0f;
            outputs.begin();
            controller.begin();
            if (card.begin()) file.begin();
            started = box.begin(false);
            takeSerial();
        }

        void tick(bool frame = true)
        {
            if (frame) board.rc.push(ibusFrame(rc));
            fake::advanceMs(2);
            imu.data.timestamp = micros();
            controller.update();
            stats.record(300);
            box.update(300);
            box.writerStep();
        }

        void run(uint32_t ms)
        {
            for (uint32_t i = 0; i < ms / 2; ++i) tick();
        }

        void arm()
        {
            rc.set(Channels::ARM, 1000).set(Channels::THROTTLE, 1000);
            tick();
            rc.set(Channels::ARM, 2000);
            tick();
        }

        void drain()
        {
            for (int i = 0; i < 5000 && box.getState() == BlackBox::State::Stopping; ++i) tick();
        }

        void shake(uint32_t ms)
        {
            for (uint32_t i = 0; i < ms / 2; ++i)
            {
                imu.data.gyroX = (i % 2) ? 20.0f : -20.0f;
                imu.data.roll = static_cast<float>(i % 30);
                tick();
            }
            imu.data.gyroX = 0;
        }
    };

    // Тот же носитель "после перезагрузки": новые объекты поверх той же карты.
    struct Reopened
    {
        Stm32SdCard card;
        SdFileRegion file{ card, Config::BLACKBOX_SD_FILE };
        BlackBoxStorage storage{ file };

        // verify — как BlackBox::begin(): сразу сверить стёртое место впереди
        // (только чтение), иначе писать некуда.
        explicit Reopened(bool verify = true)
        {
            TEST_ASSERT_TRUE(card.begin());
            TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(file.begin()));
            TEST_ASSERT_TRUE(storage.begin());
            while (verify && storage.eraseStep(storage.totalSectors(), 0, false)) {}
        }
    };
}


// ============================================================
// FAT32
// ============================================================

void test_fat_finds_file_behind_mbr_with_noise_and_a_second_root_cluster()
{
    for (const uint32_t perCluster : { 1u, 2u, 8u })
    {
        fake::SdCardModel& card = fake::sdCard();
        card.resize(16384);
        fat_image::Options options;
        options.perCluster = perCluster;
        options.rootClusters = 2;
        const fat_image::Result image = fat_image::format(card, { { "BLACKBOX.BIN", 1024 * 1024 } }, options);

        Fat32::Extent extent;
        TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(Fat32::locate(card, "BLACKBOX.BIN", extent)));
        TEST_ASSERT_EQUAL_UINT32(image.firstBlock.at("BLACKBOX.BIN"), extent.firstBlock);
        TEST_ASSERT_EQUAL_UINT32(1024 * 1024, extent.bytes);
    }
}

void test_fat_without_partition_table_and_with_lowercase_name()
{
    fake::SdCardModel& card = fake::sdCard();
    card.resize(16384);
    fat_image::Options options;
    options.mbr = false;
    const fat_image::Result image = fat_image::format(card, { { "BLACKBOX.BIN", 65536 } }, options);

    Fat32::Extent extent;
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(Fat32::locate(card, "blackbox.bin", extent)));
    TEST_ASSERT_EQUAL_UINT32(image.firstBlock.at("BLACKBOX.BIN"), extent.firstBlock);
}

void test_fat_explains_why_a_card_cannot_be_used()
{
    fake::SdCardModel& card = fake::sdCard();
    Fat32::Extent extent;
    auto result = [&](const char* name = "BLACKBOX.BIN") { return static_cast<int>(Fat32::locate(card, name, extent)); };

    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NoCard), result());   // слот пуст

    card.resize(16384);   // нули: ни MBR, ни FAT
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFat32), result());

    fat_image::Options ntfs;
    ntfs.partitionType = 0x07;   // exFAT/NTFS
    fat_image::format(card, { { "BLACKBOX.BIN", 65536 } }, ntfs);
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFat32), result());

    fat_image::format(card, { { "OTHER.BIN", 65536 } });
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFound), result());

    fat_image::File deleted;
    deleted.name = "BLACKBOX.BIN";
    deleted.bytes = 65536;
    deleted.deleted = true;
    fat_image::format(card, { deleted });
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFound), result());   // удалённая запись не считается

    fat_image::File scattered;
    scattered.name = "BLACKBOX.BIN";
    scattered.bytes = 64 * BLOCK;
    scattered.fragmented = true;
    fat_image::format(card, { scattered });
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Fragmented), result());

    fat_image::format(card, { { "BLACKBOX.BIN", 0 } });
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Empty), result());

    fat_image::format(card, { { "BLACKBOX.BIN", 65536 } });
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), result());
    card.failReads = 1;
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::ReadError), result());

    // Файл по таблице уходит за край карты — таблице не верим.
    fat_image::format(card, { { "BLACKBOX.BIN", 64u * 1024 * 1024 } });
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFat32), result());

    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFound), result("TOOLONGNAME.BIN"));   // не 8.3
    TEST_ASSERT_TRUE(strlen(Fat32::describe(Fat32::Result::Fragmented)) > 10);
}

void test_short_names_follow_8_3_rules()
{
    uint8_t name[11];
    TEST_ASSERT_TRUE(Fat32::shortName("blackbox.bin", name));
    TEST_ASSERT_EQUAL_MEMORY("BLACKBOXBIN", name, 11);
    TEST_ASSERT_TRUE(Fat32::shortName("A.B", name));
    TEST_ASSERT_EQUAL_MEMORY("A       B  ", name, 11);
    TEST_ASSERT_TRUE(Fat32::shortName("LOG", name));
    TEST_ASSERT_EQUAL_MEMORY("LOG        ", name, 11);
    TEST_ASSERT_FALSE(Fat32::shortName("a.b.c", name));
    TEST_ASSERT_FALSE(Fat32::shortName("verylongname.bin", name));
    TEST_ASSERT_FALSE(Fat32::shortName("x.long", name));
    TEST_ASSERT_FALSE(Fat32::shortName(".bin", name));
    TEST_ASSERT_FALSE(Fat32::shortName("", name));
}


// ============================================================
// SdFileRegion
// ============================================================

void test_region_pages_of_256_bytes_share_a_block_without_rereading_the_card()
{
    const fat_image::Result image = prepareCard();
    Stm32SdCard card;
    TEST_ASSERT_TRUE(card.begin());
    SdFileRegion region(card, "BLACKBOX.BIN");
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(region.begin()));
    TEST_ASSERT_EQUAL_UINT32(RING_BYTES, region.size());

    fake::SdCardModel& model = fake::sdCard();
    std::vector<uint8_t> a(256, 0xA1), b(256, 0xB2);
    const uint32_t readsBefore = model.readCalls;
    TEST_ASSERT_TRUE(region.write(0, a.data(), a.size()));
    TEST_ASSERT_TRUE(region.write(256, b.data(), b.size()));
    TEST_ASSERT_EQUAL_UINT32(1, model.readCalls - readsBefore);   // старое читается один раз
    TEST_ASSERT_EQUAL_UINT32(2, model.writeCalls);                // а пишется сквозь: каждая страница сразу на карте

    const uint8_t* onCard = fileBytes(image);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a.data(), onCard, 256);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(b.data(), onCard + 256, 256);
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[512]);

    // Запись, пересекающая границу блоков, дополняет оба.
    std::vector<uint8_t> c(40, 0xC3);
    TEST_ASSERT_TRUE(region.write(1024 - 20, c.data(), c.size()));
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[1024 - 21]);
    TEST_ASSERT_EQUAL_HEX8(0xC3, onCard[1024 - 20]);
    TEST_ASSERT_EQUAL_HEX8(0xC3, onCard[1024 + 19]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[1024 + 20]);

    std::vector<uint8_t> back(1024);
    TEST_ASSERT_TRUE(region.read(0, back.data(), back.size()));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(onCard, back.data(), 1024);
}

void test_region_bulk_io_goes_by_whole_sectors_and_any_alignment()
{
    const fat_image::Result image = prepareCard();
    Stm32SdCard card;
    card.begin();
    SdFileRegion region(card, "BLACKBOX.BIN");
    region.begin();
    fake::SdCardModel& model = fake::sdCard();

    std::vector<uint8_t> data(SECTOR * 2);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 7 + 3);

    model.writeCalls = 0;
    TEST_ASSERT_TRUE(region.write(SECTOR * 4, data.data(), data.size()));
    TEST_ASSERT_EQUAL_UINT32(2, model.writeCalls);   // два сектора = два обращения по 8 блоков
    TEST_ASSERT_EQUAL_UINT32(8, model.maxBlocksPerCall);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data.data(), fileBytes(image) + SECTOR * 4, data.size());

    std::vector<uint8_t> span(6000);
    TEST_ASSERT_TRUE(region.read(SECTOR * 4 + 1000, span.data(), span.size()));   // не с границы блока
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data.data() + 1000, span.data(), span.size());

    // Невыровненный старт и хвост: середина — блоками, края — через кэш.
    std::vector<uint8_t> mixed(3000, 0x5A);
    TEST_ASSERT_TRUE(region.write(SECTOR * 10 + 100, mixed.data(), mixed.size()));
    const uint8_t* onCard = fileBytes(image) + SECTOR * 10;
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[99]);
    TEST_ASSERT_EQUAL_HEX8(0x5A, onCard[100]);
    TEST_ASSERT_EQUAL_HEX8(0x5A, onCard[3099]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[3100]);
}

void test_region_erase_writes_ff_and_guards_the_bounds()
{
    const fat_image::Result image = prepareCard();
    Stm32SdCard card;
    card.begin();
    SdFileRegion region(card, "BLACKBOX.BIN");
    region.begin();

    std::vector<uint8_t> junk(SECTOR * 3, 0x00);
    TEST_ASSERT_TRUE(region.write(SECTOR * 2, junk.data(), junk.size()));
    TEST_ASSERT_TRUE(region.erase(SECTOR * 3, SECTOR));
    const uint8_t* onCard = fileBytes(image);
    TEST_ASSERT_EQUAL_HEX8(0x00, onCard[SECTOR * 3 - 1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[SECTOR * 3]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, onCard[SECTOR * 4 - 1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, onCard[SECTOR * 4]);

    // Стёртый сектор не берётся из устаревшего кэша блока.
    uint8_t one = 0x11;
    TEST_ASSERT_TRUE(region.write(SECTOR * 4 + 1, &one, 1));   // кэш — блок внутри сектора 4
    TEST_ASSERT_TRUE(region.erase(SECTOR * 4, SECTOR));
    uint8_t read = 0;
    TEST_ASSERT_TRUE(region.read(SECTOR * 4 + 1, &read, 1));
    TEST_ASSERT_EQUAL_HEX8(0xFF, read);

    TEST_ASSERT_FALSE(region.erase(100, SECTOR));             // не по границе сектора
    TEST_ASSERT_FALSE(region.erase(0, 100));
    TEST_ASSERT_FALSE(region.erase(RING_BYTES, SECTOR));      // за концом
    TEST_ASSERT_FALSE(region.read(RING_BYTES - 10, &read, 11));
    TEST_ASSERT_FALSE(region.write(RING_BYTES, &one, 1));
    TEST_ASSERT_TRUE(region.read(RING_BYTES - 1, &read, 1));
    TEST_ASSERT_TRUE(region.read(RING_BYTES, &read, 0));
}

void test_region_follows_the_max_size_limit_and_is_empty_without_a_file()
{
    prepareCard();
    Stm32SdCard card;
    card.begin();

    SdFileRegion limited(card, "BLACKBOX.BIN", 1024 * 1024 + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(limited.begin()));
    TEST_ASSERT_EQUAL_UINT32(1024 * 1024 - SECTOR, limited.size());   // до сектора 4 КБ вниз, без служебного
    TEST_ASSERT_EQUAL_UINT32(FILE_BYTES, limited.fileSize());

    SdFileRegion tiny(card, "BLACKBOX.BIN", 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Empty), static_cast<int>(tiny.begin()));
    TEST_ASSERT_EQUAL_UINT32(0, tiny.size());

    SdFileRegion missing(card, "NOPE.BIN");
    uint8_t one = 0;
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::NotFound), static_cast<int>(missing.begin()));
    TEST_ASSERT_EQUAL_UINT32(0, missing.size());
    TEST_ASSERT_FALSE(missing.read(0, &one, 1));
    TEST_ASSERT_FALSE(missing.write(0, &one, 1));
}

void test_region_card_failures_are_reported_and_leave_no_stale_cache()
{
    const fat_image::Result image = prepareCard();
    Stm32SdCard card;
    card.begin();
    SdFileRegion region(card, "BLACKBOX.BIN");
    region.begin();
    fake::SdCardModel& model = fake::sdCard();

    uint8_t a = 0xA0;
    TEST_ASSERT_TRUE(region.write(0, &a, 1));

    model.failWrites = 2;   // драйвер повторяет обращение один раз: оба захода сорвались
    uint8_t b = 0xB0;
    TEST_ASSERT_FALSE(region.write(1, &b, 1));
    TEST_ASSERT_EQUAL_HEX8(0xFF, fileBytes(image)[1]);

    // Кэш сброшен: следующая запись заново читает блок и не размножает мусор.
    const uint32_t reads = model.readCalls;
    TEST_ASSERT_TRUE(region.write(2, &b, 1));
    TEST_ASSERT_EQUAL_UINT32(1, model.readCalls - reads);
    TEST_ASSERT_EQUAL_HEX8(0xA0, fileBytes(image)[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, fileBytes(image)[1]);
    TEST_ASSERT_EQUAL_HEX8(0xB0, fileBytes(image)[2]);

    model.failReads = 2;
    uint8_t sink;
    TEST_ASSERT_FALSE(region.read(SECTOR * 5, &sink, 1));
    model.failWrites = 2;
    TEST_ASSERT_FALSE(region.erase(0, SECTOR));
}


// ============================================================
// Stm32SdCard поверх фейкового HAL_SD
// ============================================================

void test_card_driver_brings_up_four_bit_bus_at_full_speed()
{
    prepareCard();
    Stm32SdCard card;
    TEST_ASSERT_TRUE(card.begin());
    TEST_ASSERT_EQUAL_UINT32(CARD_BLOCKS, card.blockCount());
    TEST_ASSERT_EQUAL_UINT32(1, card.clockDivider());                 // 48 МГц / 2 = 24 МГц
    TEST_ASSERT_EQUAL_UINT32(SDMMC_BUS_WIDE_4B, fake::sdCard().lastBusWide);
    TEST_ASSERT_EQUAL_UINT32(1, card.cardType());
}

void test_card_driver_drops_to_slower_clocks_when_the_bus_does_not_hold()
{
    prepareCard();
    fake::sdCard().minWorkingClockDiv = 4;   // быстрее 6 МГц карта не читает
    Stm32SdCard card;
    TEST_ASSERT_TRUE(card.begin());
    TEST_ASSERT_EQUAL_UINT32(4, card.clockDivider());

    fake::sdCard().minWorkingClockDiv = 9;   // совсем не читает
    Stm32SdCard hopeless;
    TEST_ASSERT_FALSE(hopeless.begin());
    TEST_ASSERT_EQUAL_UINT32(0, hopeless.blockCount());
}

void test_card_driver_survives_a_failed_init_and_reports_a_missing_card()
{
    prepareCard();
    fake::sdCard().failInit = 1;
    Stm32SdCard card;
    TEST_ASSERT_TRUE(card.begin());
    TEST_ASSERT_EQUAL_UINT32(2, card.clockDivider());   // первая попытка сорвалась, вторая — на 12 МГц

    fake::sdCard().failInit = 3;
    Stm32SdCard unlucky;
    TEST_ASSERT_FALSE(unlucky.begin());
    TEST_ASSERT_NOT_EQUAL(0u, unlucky.initError());

    fake::sdCard().present = false;
    Stm32SdCard empty;
    TEST_ASSERT_FALSE(empty.begin());
    TEST_ASSERT_EQUAL_UINT32(0, empty.blockCount());
    uint8_t buffer[BLOCK];
    TEST_ASSERT_FALSE(empty.read(0, buffer, 1));
    TEST_ASSERT_FALSE(empty.write(0, buffer, 1));
}

void test_card_driver_chunks_io_bounces_unaligned_buffers_and_checks_the_range()
{
    prepareCard();
    Stm32SdCard card;
    card.begin();
    fake::SdCardModel& model = fake::sdCard();

    std::vector<uint8_t> data(20 * BLOCK + 1);   // +1: ниже берём адрес со сдвигом
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i ^ 0x5C);
    model.unalignedCalls = 0;
    model.maxBlocksPerCall = 0;
    TEST_ASSERT_TRUE(card.write(1000, data.data() + 1, 20));   // адрес нечётный
    TEST_ASSERT_EQUAL_UINT32(0, model.unalignedCalls);          // в HAL уходит выровненная копия
    TEST_ASSERT_EQUAL_UINT32(8, model.maxBlocksPerCall);        // 20 блоков = 8 + 8 + 4

    std::vector<uint8_t> back(20 * BLOCK + 1);
    TEST_ASSERT_TRUE(card.read(1000, back.data() + 1, 20));
    TEST_ASSERT_EQUAL_UINT32(0, model.unalignedCalls);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data.data() + 1, back.data() + 1, 20 * BLOCK);

    TEST_ASSERT_FALSE(card.read(CARD_BLOCKS, back.data(), 1));
    TEST_ASSERT_FALSE(card.read(CARD_BLOCKS - 1, back.data(), 2));
    TEST_ASSERT_FALSE(card.write(0, back.data(), 0));
}

void test_card_driver_retries_once_and_gives_up_on_a_dead_card()
{
    prepareCard();
    Stm32SdCard card;
    card.begin();
    uint8_t buffer[BLOCK];

    fake::sdCard().failReads = 1;
    TEST_ASSERT_TRUE(card.read(0, buffer, 1));
    TEST_ASSERT_EQUAL_UINT32(1, card.retries);
    TEST_ASSERT_EQUAL_UINT32(1, card.errors);

    fake::sdCard().failWrites = 2;
    TEST_ASSERT_FALSE(card.write(0, buffer, 1));
    TEST_ASSERT_EQUAL_UINT32(3, card.errors);
}

void test_card_driver_waits_for_programming_and_times_out_on_a_stuck_card()
{
    prepareCard();
    Stm32SdCard card;
    card.begin();
    uint8_t buffer[BLOCK] = {};
    fake::sdCard().busyPollsAfterWrite = 5;

    TEST_ASSERT_TRUE(card.write(0, buffer, 1));
    const uint32_t before = millis();
    TEST_ASSERT_TRUE(card.read(0, buffer, 1));   // ждёт, пока карта допишет
    TEST_ASSERT_UINT32_WITHIN(1, 5, millis() - before);

    fake::sdCard().busyPollsAfterWrite = 1000000;   // не освобождается никогда
    TEST_ASSERT_TRUE(card.write(0, buffer, 1));
    TEST_ASSERT_FALSE(card.read(0, buffer, 1));
}


// ============================================================
// BlackBoxStorage на карте
// ============================================================

void test_storage_keeps_flights_across_reboot_and_scans_one_block_per_sector()
{
    prepareCard();
    uint16_t first;
    {
        Reopened card;
        const uint32_t sectors = card.storage.totalSectors();
        TEST_ASSERT_EQUAL_UINT32(RING_BYTES / SECTOR, sectors);

        first = card.storage.openFlight();
        for (size_t i = 0; i < 400; ++i)
        {
            const std::vector<uint8_t> r = record(REC_SYS, static_cast<uint32_t>(i), 58, static_cast<uint8_t>(i));
            TEST_ASSERT_TRUE(card.storage.append(r.data(), r.size(), static_cast<uint32_t>(i)) >= 0);
        }
        card.storage.closeFlight();
        TEST_ASSERT_EQUAL_UINT32(0, card.storage.writeErrors);
    }

    // После "перезагрузки" сверка при включении читает по одному блоку на сектор,
    // а не по 4 КБ: иначе на карте на сотни МБ она шла бы минутами.
    fake::sdCard().readCalls = 0;
    Reopened again(false);
    // Сверка при включении идёт двумя проходами по заголовкам (голова кольца, потом полёты): ~2 чтения на сектор.
    TEST_ASSERT_TRUE(fake::sdCard().readCalls <= 2 * again.storage.totalSectors() + 128);   // + чтение таблицы FAT при поиске файла
    TEST_ASSERT_EQUAL(1u, again.storage.flightCount());
    TEST_ASSERT_EQUAL_UINT16(first, again.storage.flight(0).number);
    const std::vector<Rec> recs = readFlight(again.storage, first);
    TEST_ASSERT_EQUAL_UINT32(400, recs.size());
    TEST_ASSERT_EQUAL_UINT32(399, recs.back().t());
}

void test_empty_ring_mark_makes_the_next_boot_instant_and_goes_before_the_first_data()
{
    const fat_image::Result image = prepareCard();   // файл залит 0xFF, метки нет
    {
        Reopened first(false);                       // нечего сверять, но убедиться можно только полным проходом
        TEST_ASSERT_FALSE(first.storage.scanWasSparse);
        TEST_ASSERT_TRUE(first.storage.scanReads > 1000);
        TEST_ASSERT_TRUE(first.file.ringMarkedEmpty());   // и теперь метка стоит
        TEST_ASSERT_EQUAL_MEMORY("OPEM", markBytes(image), 4);
        TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(~'O'), markBytes(image)[4]);
    }

    uint16_t flight;
    {
        Reopened second(false);                      // по метке: одно чтение
        TEST_ASSERT_EQUAL_UINT32(1, second.storage.scanReads);
        TEST_ASSERT_EQUAL_UINT32(0, second.storage.flightCount());
        while (second.storage.eraseStep(second.storage.totalSectors(), 0, false)) {}   // стёртое впереди сверяется отдельно
        TEST_ASSERT_TRUE(second.file.ringMarkedEmpty());

        flight = second.storage.openFlight();
        const std::vector<uint8_t> r = record(REC_SYS, 1, 58, 0x11);
        TEST_ASSERT_TRUE(second.storage.append(r.data(), r.size(), 5) >= 0);
        TEST_ASSERT_FALSE(second.file.ringMarkedEmpty());   // метка снята до записи данных
        second.storage.closeFlight();
    }

    Reopened third(false);                           // метки нет — настоящая сверка, полёт найден
    TEST_ASSERT_TRUE(third.storage.scanReads > 1000);
    TEST_ASSERT_EQUAL(1u, third.storage.flightCount());
    TEST_ASSERT_EQUAL_UINT16(flight, third.storage.flight(0).number);
    TEST_ASSERT_FALSE(third.file.ringMarkedEmpty());

    TEST_ASSERT_TRUE(third.storage.eraseAll());      // стёрли всё — метка снова на месте
    TEST_ASSERT_TRUE(third.file.ringMarkedEmpty());
    Reopened fourth(false);
    TEST_ASSERT_EQUAL_UINT32(1, fourth.storage.scanReads);
    TEST_ASSERT_EQUAL(0u, fourth.storage.flightCount());
}

void test_garbage_in_the_service_sector_is_not_an_empty_mark()
{
    const fat_image::Result image = prepareCard();
    Stm32SdCard card;
    card.begin();
    SdFileRegion region(card, "BLACKBOX.BIN");
    region.begin();
    TEST_ASSERT_FALSE(region.ringMarkedEmpty());     // свежий файл из 0xFF — метки нет
    region.markRingEmpty();
    TEST_ASSERT_TRUE(region.ringMarkedEmpty());
    markBytes(image)[5] ^= 0x01;                     // испорчена инвертированная половина (за спиной кэша области)
    region.begin();
    TEST_ASSERT_FALSE(region.ringMarkedEmpty());
    region.markRingEmpty();
    region.clearRingMark();
    TEST_ASSERT_FALSE(region.ringMarkedEmpty());
    TEST_ASSERT_EQUAL_HEX8(0xFF, markBytes(image)[0]);

    SdFileRegion missing(card, "NOPE.BIN");          // области нет — безопасные пустышки
    missing.begin();
    TEST_ASSERT_FALSE(missing.ringMarkedEmpty());
    missing.markRingEmpty();
    missing.clearRingMark();
}

void test_storage_write_cost_is_one_card_write_per_256_byte_page()
{
    prepareCard();
    Reopened card;
    card.storage.openFlight();
    fake::sdCard().writeCalls = 0;
    // Ровно один сектор записей: 4096 / 64 → 63 записи по 64 байта (с заголовком сектора и CRC).
    const std::vector<uint8_t> r = record(REC_SYS, 0, 58, 0xAB);
    int pages = 0;
    for (int i = 0; i < 60; ++i) pages += card.storage.append(r.data(), r.size(), 1);
    // 60 записей по 61 байту ≈ 3.6 КБ = 14 страниц по 256 байт.
    TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(pages), fake::sdCard().writeCalls);
    TEST_ASSERT_TRUE(pages >= 13 && pages <= 15);
}

void test_storage_erases_old_flights_on_the_card_to_make_room()
{
    prepareCard(64 * SECTOR);   // кольцо на 64 сектора
    Reopened card;
    uint16_t numbers[4];
    for (int f = 0; f < 4; ++f)
    {
        const BlackBoxStorage::Flight* newest = card.storage.newestFlight();
        while (card.storage.eraseStep(20, newest ? newest->number : 0)) {}
        numbers[f] = card.storage.openFlight();
        for (int i = 0; i < 1200; ++i)   // ~18 секторов на полёт: четыре не влезают в 64
        {
            const std::vector<uint8_t> r = record(REC_SYS, static_cast<uint32_t>(i), 58, static_cast<uint8_t>(f));
            card.storage.append(r.data(), r.size(), static_cast<uint32_t>(f * 1000 + i));
        }
        card.storage.closeFlight();
    }
    char why[96];
    snprintf(why, sizeof(why), "eraseOps=%lu free=%lu flights=%u", (unsigned long)card.storage.eraseOps,
             (unsigned long)card.storage.freeSectors(), (unsigned)card.storage.flightCount());
    TEST_ASSERT_TRUE_MESSAGE(card.storage.eraseOps > 0, why);
    TEST_ASSERT_EQUAL_UINT32(0, card.storage.eraseErrors);

    Reopened again;
    TEST_ASSERT_NOT_NULL(again.storage.findFlight(numbers[3]));   // последний цел
    TEST_ASSERT_EQUAL_UINT32(1200, readFlight(again.storage, numbers[3]).size());
    TEST_ASSERT_TRUE(again.storage.flightCount() >= 2);
}

void test_storage_power_loss_keeps_everything_written_before_it()
{
    const fat_image::Result image = prepareCard();
    Reopened card;
    const uint16_t flight = card.storage.openFlight();
    fake::sdCard().cutPowerAfterBlocks = 40;   // питание пропадает на 40-м блоке записи
    for (size_t i = 0; i < 300; ++i)
    {
        const std::vector<uint8_t> r = record(REC_SYS, static_cast<uint32_t>(i), 58, 0x77);
        card.storage.append(r.data(), r.size(), static_cast<uint32_t>(i));
    }
    card.storage.closeFlight();
    TEST_ASSERT_TRUE(fake::sdCard().lostBlocks > 0);

    fake::sdCard().cutPowerAfterBlocks = fake::SdCardModel::NEVER;
    Reopened after;
    TEST_ASSERT_EQUAL(1u, after.storage.flightCount());
    size_t torn = 0;
    const std::vector<Rec> recs = readFlight(after.storage, flight, &torn);
    TEST_ASSERT_TRUE(recs.size() > 100 && recs.size() < 300);
    for (size_t i = 0; i < recs.size(); ++i) TEST_ASSERT_EQUAL_UINT32(i, recs[i].t());   // ни дыр, ни мусора
    (void)image;
}


// ============================================================
// BlackBox на карте
// ============================================================

void test_flight_is_recorded_to_the_card_with_stm32_specifics()
{
    prepareCard();
    fake::gpio().analogMv[Config::PIN_VBAT_ADC] = 1818;      // 3S ~12.0 В за делителем 56k/10k
    fake::gpio().analogMv[Config::PIN_CURRENT_ADC] = 1500;
    Rig rig;
    TEST_ASSERT_TRUE(rig.started);

    rig.run(2000);
    rig.arm();
    rig.rc.set(Channels::THROTTLE, 1600);
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    rig.shake(10000);
    rig.rc.set(Channels::THROTTLE, 1000).set(Channels::ARM, 1000);
    rig.run(Config::BLACKBOX_POSTROLL_MS + 1200);
    rig.drain();
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BlackBox: полёт #1 записан"));

    Reopened card;
    TEST_ASSERT_EQUAL(1u, card.storage.flightCount());
    const std::vector<Rec> recs = readFlight(card.storage, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "board=STM32H743"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "reset_reason=POWERON"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "start=ARM и газ"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "pid.roll="));   // float в snprintf: прошивка собрана с -u_printf_float
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "power.vbat_divider=6.6000"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "запись: старт (ARM и газ)"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    TEST_ASSERT_TRUE(countType(recs, REC_IMU) > 5000);
    TEST_ASSERT_TRUE(countType(recs, REC_CTRL) > 1000);

    // Батарея и ток читаются с АЦП STM32 (12 бит, 3.3 В) и масштабируются делителями.
    size_t power = 0;
    for (const Rec& r : recs)
    {
        if (r.type != REC_POWER) continue;
        const PowerRecord p = r.as<PowerRecord>();
        TEST_ASSERT_UINT16_WITHIN(8, 11999, p.vbatMv);   // шаг АЦП 12 бит: ±5 мВ после делителя
        TEST_ASSERT_UINT16_WITHIN(3, 2500, p.currentMv);
        power++;
    }
    TEST_ASSERT_TRUE(power > 100);

    // Карта пишется страницами сквозь: ни одного сбоя, стирания в воздухе нет.
    TEST_ASSERT_EQUAL_UINT32(0, rig.card.errors);
    TEST_ASSERT_EQUAL_UINT32(0, rig.storage.writeErrors);
}

void test_crash_reset_by_watchdog_starts_recording_and_power_on_does_not()
{
    prepareCard();
    fake::rcc().RSR = RCC_RSR_PORRSTF | RCC_RSR_PINRSTF | RCC_RSR_BORRSTF;   // обычное включение: POR ставит и BOR, и PIN
    {
        Rig rig;
        rig.tick();
        TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    }

    resetWorld();
    prepareCard();
    fake::rcc().RSR = RCC_RSR_PINRSTF | RCC_RSR_IWDG1RSTF;   // сторожевой таймер: PIN выставлен, но причина — WDG
    Rig rig;
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    rig.run(Config::BLACKBOX_RESET_HOLD_MS + 2000);   // без ARM запись держится минуту
    rig.drain();

    Reopened card;
    const std::vector<Rec> recs = readFlight(card.storage, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "reset_reason=WDT"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "start=перезагрузка после сбоя"));

    resetWorld();
    prepareCard();
    fake::rcc().RSR = RCC_RSR_BORRSTF | RCC_RSR_PINRSTF;   // просадка питания без POR
    Rig brownout;
    brownout.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, brownout.box.getState());
}

void test_card_errors_in_flight_do_not_stop_the_box_and_are_logged()
{
    prepareCard();
    Rig rig;
    rig.box.requestManualStart();
    rig.run(1000);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    fake::sdCard().failWrites = 4;    // карта "моргнула": две страницы потеряны (каждое обращение — две попытки)
    rig.run(2000);
    TEST_ASSERT_TRUE(rig.storage.writeErrors > 0);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    rig.run(4000);                    // карта вернулась
    rig.box.requestManualStop();
    rig.tick();
    rig.drain();

    Reopened card;
    const std::vector<Rec> recs = readFlight(card.storage, 1);
    // Потерянная страница оставляет дыру 0xFF — разбор сектора на ней останавливается
    // (так же, как tools/blackbox.py), но остальные сектора целы, а событие записано.
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "носитель: ошибок записи"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    TEST_ASSERT_TRUE(countType(recs, REC_IMU) > 2500);
}

void test_slow_card_with_long_programming_still_keeps_up()
{
    prepareCard();
    fake::sdCard().busyPollsAfterWrite = 3;   // после каждой записи ~3 мс "программирования"
    Rig rig;
    rig.box.requestManualStart();
    rig.run(8000);
    rig.box.requestManualStop();
    rig.tick();
    rig.drain();

    Reopened card;
    const std::vector<Rec> recs = readFlight(card.storage, 1);
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    TEST_ASSERT_TRUE(countType(recs, REC_IMU) >= 3990);   // 8 с по 500 Гц: на медленной карте ничего не потеряно
    TEST_ASSERT_TRUE(contains(takeSerial(), "записан"));
}

void test_host_downloads_a_flight_from_the_card_over_the_console()
{
    prepareCard();
    Rig rig;
    rig.box.requestManualStart();
    rig.run(3000);
    rig.box.requestManualStop();
    rig.tick();
    rig.drain();
    takeSerial();

    rig.box.handleHostCommand("bb list");
    const std::string list = takeSerial();
    TEST_ASSERT_TRUE(contains(list, "BB:STATE state=idle"));
    TEST_ASSERT_TRUE(contains(list, "BB:FLIGHT n=1 sectors="));

    Serial.begin(115200);
    const unsigned beginsBefore = Serial.beginCount();
    Serial.pushRx("G");
    rig.box.handleHostCommand("bb get 1 2000000");
    const std::string out = takeSerial();
    TEST_ASSERT_TRUE(contains(out, "BB:SEND n=1 sectors="));
    TEST_ASSERT_TRUE(contains(out, "BB:DONE n=1 crc="));
    TEST_ASSERT_EQUAL_UINT(beginsBefore + 2, Serial.beginCount());   // 115200 -> 2000000 -> 115200 (Serial.end/begin)
    TEST_ASSERT_EQUAL_UINT32(115200, Serial.baud());

    const BlackBoxStorage::Flight& f = *rig.storage.findFlight(1);
    const size_t begin = out.find('\n') + 1;
    for (uint32_t k = 0; k < f.sectors; ++k)
    {
        const size_t at = begin + k * (4 + SECTOR + 4);
        TEST_ASSERT_EQUAL_HEX8(0xA5, static_cast<uint8_t>(out[at]));
        const uint8_t* data = reinterpret_cast<const uint8_t*>(out.data() + at + 4);
        uint32_t crc;
        memcpy(&crc, data + SECTOR, 4);
        TEST_ASSERT_EQUAL_HEX32(crc32(data, SECTOR), crc);
        std::vector<uint8_t> sector(SECTOR);
        rig.storage.readSector(rig.storage.sectorOf(f, k), sector.data());
        TEST_ASSERT_EQUAL_UINT8_ARRAY(sector.data(), data, SECTOR);
    }
}

void test_console_menu_works_on_the_card_and_erase_all_clears_the_file()
{
    const fat_image::Result image = prepareCard();
    Rig rig;
    DebugLogger logger(rig.controller, &rig.autopilot, &rig.stats);
    DebugConsole console(rig.controller, rig.outputs, rig.autopilot, logger, nullptr, &rig.box);

    Serial.pushRx("k");
    console.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "BlackBox: ждёт ARM и газ"));

    Serial.pushRx("r");
    console.update();
    rig.run(1000);
    Serial.pushRx("k");
    console.update();
    Serial.pushRx("r");
    console.update();
    rig.tick();
    rig.drain();
    TEST_ASSERT_EQUAL(1u, rig.storage.flightCount());
    TEST_ASSERT_NOT_EQUAL(0xFF, fileBytes(image)[0]);   // заголовок сектора на карте

    Serial.pushRx("k");
    console.update();
    Serial.pushRx("ey");
    console.update();
    const std::string erased = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(erased, "Стёрто."), erased.c_str());
    TEST_ASSERT_EQUAL(0u, rig.storage.flightCount());
    // Весь файл — снова 0xFF: следующая запись начнёт с чистого места.
    const uint8_t* bytes = fileBytes(image);
    TEST_ASSERT_TRUE(std::all_of(bytes, bytes + RING_BYTES, [](uint8_t b) { return b == 0xFF; }));
}

void test_box_stays_off_without_a_card_or_a_file_and_the_plane_still_flies()
{
    // Слот пуст.
    {
        Rig rig;
        TEST_ASSERT_FALSE(rig.started);
        const std::string log = takeSerial();
        (void)log;
        rig.box.requestManualStart();
        rig.arm();
        rig.rc.set(Channels::THROTTLE, 1500);
        rig.run(1000);
        TEST_ASSERT_EQUAL(BlackBox::State::Off, rig.box.getState());
        TEST_ASSERT_EQUAL_UINT16(cappedThrottleUs(1500), rig.board.servos[ServoChannel::ESC].lastUs);
    }

    // Карта есть, файла нет.
    resetWorld();
    fake::sdCard().resize(CARD_BLOCKS);
    fat_image::format(fake::sdCard(), { { "OTHER.BIN", 65536 } });
    Rig noFile;
    TEST_ASSERT_FALSE(noFile.started);
    TEST_ASSERT_EQUAL_UINT32(0, noFile.file.size());

    // Файл разбросан.
    resetWorld();
    fake::sdCard().resize(CARD_BLOCKS);
    fat_image::File scattered;
    scattered.name = "BLACKBOX.BIN";
    scattered.bytes = 128 * BLOCK;
    scattered.fragmented = true;
    fat_image::format(fake::sdCard(), { scattered });
    Rig fragmented;
    TEST_ASSERT_FALSE(fragmented.started);
}

namespace
{
    int bootloaderCalls = 0;
    void fakeBootloader() { bootloaderCalls++; }
}

void test_console_d_key_reboots_to_the_bootloader_only_when_disarmed()
{
    prepareCard();
    Rig rig;
    DebugLogger logger(rig.controller, &rig.autopilot, &rig.stats);
    DebugConsole console(rig.controller, rig.outputs, rig.autopilot, logger, nullptr, &rig.box);

    Serial.pushRx("D");   // хука нет — молчит
    console.update();
    TEST_ASSERT_EQUAL(0, bootloaderCalls);

    bootloaderCalls = 0;
    console.setBootloaderHook(fakeBootloader);
    Serial.pushRx("D");
    console.update();
    TEST_ASSERT_EQUAL(1, bootloaderCalls);
    TEST_ASSERT_TRUE(contains(takeSerial(), "загрузчик USB DFU"));

    Serial.pushRx("kD");   // и из меню тоже
    console.update();
    TEST_ASSERT_EQUAL(2, bootloaderCalls);
    takeSerial();

    rig.arm();
    Serial.pushRx("D");
    console.update();
    TEST_ASSERT_EQUAL(2, bootloaderCalls);   // заармлено — отказ
    TEST_ASSERT_TRUE(contains(takeSerial(), "загрузчик недоступен"));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_console_d_key_reboots_to_the_bootloader_only_when_disarmed);
    RUN_TEST(test_fat_finds_file_behind_mbr_with_noise_and_a_second_root_cluster);
    RUN_TEST(test_fat_without_partition_table_and_with_lowercase_name);
    RUN_TEST(test_fat_explains_why_a_card_cannot_be_used);
    RUN_TEST(test_short_names_follow_8_3_rules);
    RUN_TEST(test_region_pages_of_256_bytes_share_a_block_without_rereading_the_card);
    RUN_TEST(test_region_bulk_io_goes_by_whole_sectors_and_any_alignment);
    RUN_TEST(test_region_erase_writes_ff_and_guards_the_bounds);
    RUN_TEST(test_region_follows_the_max_size_limit_and_is_empty_without_a_file);
    RUN_TEST(test_region_card_failures_are_reported_and_leave_no_stale_cache);
    RUN_TEST(test_card_driver_brings_up_four_bit_bus_at_full_speed);
    RUN_TEST(test_card_driver_drops_to_slower_clocks_when_the_bus_does_not_hold);
    RUN_TEST(test_card_driver_survives_a_failed_init_and_reports_a_missing_card);
    RUN_TEST(test_card_driver_chunks_io_bounces_unaligned_buffers_and_checks_the_range);
    RUN_TEST(test_card_driver_retries_once_and_gives_up_on_a_dead_card);
    RUN_TEST(test_card_driver_waits_for_programming_and_times_out_on_a_stuck_card);
    RUN_TEST(test_storage_keeps_flights_across_reboot_and_scans_one_block_per_sector);
    RUN_TEST(test_empty_ring_mark_makes_the_next_boot_instant_and_goes_before_the_first_data);
    RUN_TEST(test_garbage_in_the_service_sector_is_not_an_empty_mark);
    RUN_TEST(test_storage_write_cost_is_one_card_write_per_256_byte_page);
    RUN_TEST(test_storage_erases_old_flights_on_the_card_to_make_room);
    RUN_TEST(test_storage_power_loss_keeps_everything_written_before_it);
    RUN_TEST(test_flight_is_recorded_to_the_card_with_stm32_specifics);
    RUN_TEST(test_crash_reset_by_watchdog_starts_recording_and_power_on_does_not);
    RUN_TEST(test_card_errors_in_flight_do_not_stop_the_box_and_are_logged);
    RUN_TEST(test_slow_card_with_long_programming_still_keeps_up);
    RUN_TEST(test_host_downloads_a_flight_from_the_card_over_the_console);
    RUN_TEST(test_console_menu_works_on_the_card_and_erase_all_clears_the_file);
    RUN_TEST(test_box_stays_off_without_a_card_or_a_file_and_the_plane_still_flies);
    return UNITY_END();
}
