// ============================================================
// Чёрный ящик на настоящей SD-карте — тест НА ПЛАТЕ STM32H743.
//
// Не нативный: драйвер SDMMC, карта и время — настоящие. Проверяет то,
// что на ПК не проверить: опознаётся ли карта, находится ли файл
// BLACKBOX.BIN, как быстро и с какими задержками она пишет страницами по
// 256 байт, держит ли запись 20 секунд потока в реальном времени
// (500 Гц IMU, как в полёте) и читается ли записанное после "перезагрузки".
// Цифры (скорость, худшая задержка записи, время сверки) печатаются
// в вывод теста.
//
// Тесты идут в задаче FreeRTOS, а рядом работает задача-имитатор полётного
// цикла с высшим приоритетом (период 2 мс): она вытесняет тесты посреди
// обращений к карте, как в прошивке. Без этого не поймать ошибки вроде
// переполнения FIFO SDMMC при вытеснении (HAL_SD_ERROR_RX_OVERRUN) — на
// "голом" цикле их нет. Худшее отклонение периода имитатора проверяется.
//
// Перед запуском файл должен быть на карте:
//   python tools/blackbox.py sd-prepare <диск карты>      (на ПК, карта в картридере)
// Файл будет ПЕРЕЗАПИСАН: тест стирает все полёты в нём.
//
//   pio test -e stm32h743-devebox -f test_blackbox_sd
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <vector>

#include "autopilot/Autopilot.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "hal/ResetCause.h"
#include "hal/Rtos.h"
#include "hal/SdFileRegion.h"
#include "hal/stm32/Stm32Board.h"
#include "hal/stm32/Stm32SdCard.h"
#include "rc/IBusReceiver.h"
#include "telemetry/BlackBox.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/LoopStats.h"

// Тесты платы собираются без src/: выводы SDMMC1 (HAL_SD_MspInit) и перезагрузка
// в DFU берутся из тех же файлов, что в прошивке.
#include "../../src/stm32/bootloader.cpp"
#include "../../src/stm32/sd_msp.cpp"

using namespace BlackBoxFormat;

namespace
{
    constexpr uint32_t SECTOR = BlackBoxFormat::SECTOR_SIZE;

    // Имитатор полётной задачи: период 2 мс, ~40 мкс работы, приоритет полётной.
    volatile uint32_t flightTicks = 0;
    volatile uint32_t flightWorstErrorUs = 0;

    void flightLikeTask(void*)
    {
        TickType_t last = xTaskGetTickCount();
        uint32_t previous = micros();
        for (;;)
        {
            const uint32_t t0 = micros();
            while (micros() - t0 < 40) {}
            vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
            const uint32_t now = micros();
            const uint32_t period = now - previous;
            previous = now;
            const uint32_t error = period > 2000 ? period - 2000 : 2000 - period;
            if (flightTicks > 10 && error > flightWorstErrorUs) flightWorstErrorUs = error;
            flightTicks++;
        }
    }

    Stm32SdCard sdCard;
    SdFileRegion file(sdCard, Config::BLACKBOX_SD_FILE, Config::BLACKBOX_SD_MAX_BYTES);
    ResetCause startCause = ResetCause::Unknown;

    void say(const char* format, ...)
    {
        char text[160];
        va_list args;
        va_start(args, format);
        vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        TEST_MESSAGE(text);
    }

    // Датчик-имитатор: гироскоп "дрожит", акселерометр — 1 g. Хватает, чтобы
    // чёрный ящик писал IMU 500 Гц и условие "стоит на земле" не срабатывало.
    class BenchImu : public ImuSensor
    {
    public:
        ImuData data = {};
        uint32_t phase = 0;

        bool begin() override { return true; }
        bool isAvailable() const override { return true; }
        void update() override
        {
            phase++;
            data.gyroX = (phase & 1) ? 20.0f : -20.0f;
            data.roll = static_cast<float>(phase % 30);
            data.accelZ = 1.0f;
            data.timestamp = micros();
        }
        const char* getSensorType() const override { return "BenchIMU"; }
        void printStatus() const override {}
        const ImuData& getImuData() const override { return data; }
        void calibrate() override {}
        void setYaw(float) override {}
    };

    void requireFile()
    {
        TEST_ASSERT_TRUE_MESSAGE(sdCard.blockCount() > 0, "карты нет");
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(Fat32::Result::Ok), static_cast<int>(file.begin()), "нет файла");
    }
}

void setUp() {}
void tearDown() {}

void test_the_flight_task_was_not_disturbed()
{
    say("задача-имитатор: тактов %lu, худшее отклонение периода %lu мкс", static_cast<unsigned long>(flightTicks),
        static_cast<unsigned long>(flightWorstErrorUs));
    TEST_ASSERT_TRUE(flightTicks > 1000);
    TEST_ASSERT_TRUE_MESSAGE(flightWorstErrorUs < 3000, "запись на карту сбила период полётной задачи");
}

void test_reset_cause_is_a_normal_one()
{
    // Прошивку только что залили кнопкой/DFU: сторожевой таймер и просадка питания — не норма.
    say("причина перезагрузки: %s", resetCauseName(startCause));
    TEST_ASSERT_FALSE(isCrashReset(startCause));
}

void test_card_is_detected_on_four_bit_bus()
{
    const uint32_t start = micros();
    const bool ok = sdCard.begin();
    const uint32_t spent = micros() - start;
    if (!ok) say("SD init: код ошибки HAL 0x%lx", static_cast<unsigned long>(sdCard.initError()));
    TEST_ASSERT_TRUE_MESSAGE(ok, "SD-карта не отвечает (не вставлена? слот на SDMMC1: PC8..PC12, PD2)");
    say("карта: %lu МБ, тип %lu, шина 4 бита, ClockDiv %lu (%lu МГц), опознание %lu мс",
        static_cast<unsigned long>(sdCard.blockCount() / 2048), static_cast<unsigned long>(sdCard.cardType()),
        static_cast<unsigned long>(sdCard.clockDivider()), static_cast<unsigned long>(48 / (2 * sdCard.clockDivider())),
        static_cast<unsigned long>(spent / 1000));
    TEST_ASSERT_TRUE(sdCard.blockCount() > 100000);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, sdCard.clockDivider(), "карта не держит 24 МГц, ящик работает на пониженной скорости");
}

void test_file_is_found_and_contiguous()
{
    TEST_ASSERT_TRUE(sdCard.blockCount() > 0);
    const Fat32::Result found = file.begin();
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(Fat32::Result::Ok), static_cast<int>(found), Fat32::describe(found));
    say("файл %s: %lu МБ, с блока %lu; область ящика %lu МБ", file.fileName(),
        static_cast<unsigned long>(file.fileSize() / (1024 * 1024)), static_cast<unsigned long>(file.firstCardBlock()),
        static_cast<unsigned long>(file.size() / (1024 * 1024)));
    TEST_ASSERT_TRUE(file.size() >= 1024 * 1024);
}

void test_multi_block_writes_work_at_every_length()
{
    requireFile();
    // Диагностика: запись 1, 2, 4, 8 блоков одним обращением (стирание и сектора идут по 8).
    std::vector<uint8_t> data(8 * 512), back(8 * 512);
    for (uint32_t n : { 1u, 2u, 4u, 8u })
    {
        for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 5 + n);
        const bool wrote = sdCard.write(file.firstCardBlock() + 100, data.data(), n);
        say("запись %lu блок(ов): %s, код 0x%lx", static_cast<unsigned long>(n), wrote ? "ок" : "СБОЙ",
            static_cast<unsigned long>(sdCard.lastErrorCode()));
        TEST_ASSERT_TRUE(wrote);
        TEST_ASSERT_TRUE(sdCard.read(file.firstCardBlock() + 100, back.data(), n));
        TEST_ASSERT_EQUAL_UINT8_ARRAY(data.data(), back.data(), n * 512);
    }
    // Восстановить 0xFF.
    std::vector<uint8_t> ff(8 * 512, 0xFF);
    TEST_ASSERT_TRUE(sdCard.write(file.firstCardBlock() + 100, ff.data(), 8));
}

void test_pages_write_with_bounded_latency_and_read_back_intact()
{
    requireFile();
    // Как BlackBox: страницы по 256 байт подряд, в 8 секторах (32 КБ).
    constexpr uint32_t BYTES = 256 * 1024;
    std::vector<uint8_t> pattern(256);
    uint32_t worst = 0, total = 0, writes = 0;
    const uint32_t startedAt = micros();
    for (uint32_t offset = 0; offset < BYTES; offset += 256)
    {
        for (size_t i = 0; i < pattern.size(); ++i) pattern[i] = static_cast<uint8_t>(offset / 256 + i * 3);
        const uint32_t t0 = micros();
        TEST_ASSERT_TRUE(file.write(offset, pattern.data(), pattern.size()));
        const uint32_t spent = micros() - t0;
        total += spent;
        worst = spent > worst ? spent : worst;
        writes++;
    }
    const uint32_t wall = micros() - startedAt;
    say("запись страниц 256 Б: %lu шт, средняя %lu мкс, худшая %lu мкс, %lu КБ/с", static_cast<unsigned long>(writes),
        static_cast<unsigned long>(total / writes), static_cast<unsigned long>(worst),
        static_cast<unsigned long>(BYTES * 1000000ULL / wall / 1024));
    TEST_ASSERT_TRUE_MESSAGE(worst < 250000, "запись одной страницы дольше 250 мс — карта слишком медленная для чёрного ящика");
    TEST_ASSERT_TRUE_MESSAGE(BYTES * 1000000ULL / wall > 40 * 1024, "карта пишет страницами медленнее 40 КБ/с (нужно ~20 КБ/с с запасом)");

    // Чтение секторами и проверка содержимого.
    std::vector<uint8_t> sector(SECTOR);
    const uint32_t r0 = micros();
    for (uint32_t offset = 0; offset < BYTES; offset += SECTOR)
    {
        TEST_ASSERT_TRUE(file.read(offset, sector.data(), SECTOR));
        for (uint32_t k = 0; k < SECTOR; k += 256)
        {
            for (size_t i = 0; i < 256; ++i)
            {
                if (sector[k + i] != static_cast<uint8_t>((offset + k) / 256 + i * 3)) TEST_FAIL_MESSAGE("прочитано не то, что записано");
            }
        }
    }
    const uint32_t rspent = micros() - r0;
    say("чтение секторов 4 КБ: %lu КБ/с", static_cast<unsigned long>(BYTES * 1000000ULL / rspent / 1024));

    // Стирание 64 КБ и проверка 0xFF.
    const uint32_t e0 = micros();
    const bool erased = file.erase(0, 64 * 1024);
    if (!erased) say("erase не прошёл: код HAL 0x%lx, ошибок %lu, повторов %lu, записей %lu", static_cast<unsigned long>(sdCard.lastErrorCode()),
                     static_cast<unsigned long>(sdCard.errors), static_cast<unsigned long>(sdCard.retries), static_cast<unsigned long>(sdCard.writeOps));
    TEST_ASSERT_TRUE(erased);
    const uint32_t espent = micros() - e0;
    say("стирание 64 КБ: %lu мс", static_cast<unsigned long>(espent / 1000));
    TEST_ASSERT_TRUE(file.read(0, sector.data(), SECTOR));
    for (uint32_t i = 0; i < SECTOR; ++i) TEST_ASSERT_EQUAL_HEX8(0xFF, sector[i]);
}

void test_header_scan_cost_on_the_whole_area()
{
    requireFile();
    // Сверка при включении читает заголовок каждого сектора (два прохода).
    const uint32_t sectors = file.size() / SECTOR;
    const uint32_t probe = sectors < 2000 ? sectors : 2000;
    uint8_t header[16];
    const uint32_t t0 = micros();
    for (uint32_t i = 0; i < probe; ++i) TEST_ASSERT_TRUE(file.read(i * SECTOR, header, sizeof(header)));
    const uint32_t perRead = (micros() - t0) / probe;
    say("чтение заголовка сектора: %lu мкс; сверка при включении области %lu МБ: ~%lu мс", static_cast<unsigned long>(perRead),
        static_cast<unsigned long>(file.size() / (1024 * 1024)), static_cast<unsigned long>(2ULL * sectors * perRead / 1000));
    TEST_ASSERT_TRUE_MESSAGE(perRead < 3000, "чтение блока дольше 3 мс");
}

void test_storage_erase_all_write_flights_and_find_them_after_reopen()
{
    requireFile();
    BlackBoxStorage storage(file);
    TEST_ASSERT_TRUE(storage.begin());

    const uint32_t e0 = millis();
    TEST_ASSERT_TRUE(storage.eraseAll());
    say("стирание всей области %lu МБ: %lu мс", static_cast<unsigned long>(file.size() / (1024 * 1024)),
        static_cast<unsigned long>(millis() - e0));

    // Два полёта по ~1 МБ записями SYS по 28 байт данных.
    uint16_t numbers[2] = {};
    uint32_t perFlight = 0;
    for (int f = 0; f < 2; ++f)
    {
        while (storage.eraseStep(storage.totalSectors(), 0, false)) {}   // сверка стёртого впереди
        numbers[f] = storage.openFlight();
        uint32_t records = 0;
        const uint32_t t0 = millis();
        for (; records < 20000; ++records)
        {
            uint8_t record[2 + 4 + 24];
            record[0] = REC_SYS;
            record[1] = 28;
            memcpy(record + 2, &records, 4);
            memset(record + 6, f + 1, 24);
            TEST_ASSERT_TRUE(storage.append(record, sizeof(record), millis()) >= 0);
        }
        storage.closeFlight();
        perFlight = records;
        say("полёт #%u: %lu записей за %lu мс, ошибок записи %lu", numbers[f], static_cast<unsigned long>(records),
            static_cast<unsigned long>(millis() - t0), static_cast<unsigned long>(storage.writeErrors));
        TEST_ASSERT_EQUAL_UINT32(0, storage.writeErrors);
    }

    // "Перезагрузка": новое хранилище поверх той же карты.
    BlackBoxStorage again(file);
    const uint32_t s0 = millis();
    TEST_ASSERT_TRUE(again.begin());
    const uint32_t scanMs = millis() - s0;
    say("сверка при включении (выборочная): %lu мс, прочитано заголовков %lu", static_cast<unsigned long>(scanMs),
        static_cast<unsigned long>(again.scanReads));
    TEST_ASSERT_TRUE_MESSAGE(again.scanWasSparse, "область большая — сверка должна быть выборочной");
    TEST_ASSERT_TRUE_MESSAGE(scanMs < 2000, "сверка при включении дольше 2 с");
    TEST_ASSERT_EQUAL(2u, again.flightCount());
    TEST_ASSERT_EQUAL_UINT16(numbers[0], again.flight(0).number);
    TEST_ASSERT_EQUAL_UINT16(numbers[1], again.flight(1).number);

    // Записи второго полёта читаются целиком, по порядку, с верным CRC-8.
    const BlackBoxStorage::Flight& flight = again.flight(1);
    std::vector<uint8_t> sector(SECTOR);
    uint32_t seen = 0;
    for (uint32_t k = 0; k < flight.sectors; ++k)
    {
        TEST_ASSERT_TRUE(again.readSector(again.sectorOf(flight, k), sector.data()));
        size_t pos = SECTOR_HEADER_SIZE;
        while (pos + RECORD_HEADER <= SECTOR && sector[pos] != REC_ERASED)
        {
            const size_t end = pos + RECORD_HEADER + sector[pos + 1];
            if (end + RECORD_CHECK > SECTOR) break;
            TEST_ASSERT_EQUAL_HEX8(crc8(&sector[pos], RECORD_HEADER + sector[pos + 1]), sector[end]);
            uint32_t index;
            memcpy(&index, &sector[pos + 2], 4);
            TEST_ASSERT_EQUAL_UINT32(seen, index);
            seen++;
            pos = end + RECORD_CHECK;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(perFlight, seen);
    TEST_ASSERT_TRUE(storage.eraseAll());

    // Пустое кольцо — по метке, без прохода по области.
    BlackBoxStorage empty(file);
    const uint32_t e1 = millis();
    TEST_ASSERT_TRUE(empty.begin());
    const uint32_t emptyMs = millis() - e1;
    say("включение с пустым кольцом (по метке): %lu мс", static_cast<unsigned long>(emptyMs));
    TEST_ASSERT_EQUAL_UINT32(1, empty.scanReads);
    TEST_ASSERT_TRUE(emptyMs < 100);
}

void test_blackbox_keeps_up_for_20_seconds_in_real_time()
{
    requireFile();
    {
        BlackBoxStorage clean(file);
        TEST_ASSERT_TRUE(clean.begin());
        TEST_ASSERT_TRUE(clean.eraseAll());
    }

    Stm32Board board;
    BenchImu imu;
    IBusReceiver receiver(board.rcUart());
    ControlMixer mixer;
    ThrottleManager throttle;
    FlightOutputs outputs(board);
    Autopilot autopilot(&imu);
    ArmingManager arming(&autopilot);
    FlightController controller(receiver, mixer, throttle, arming, outputs, &autopilot);
    LoopStats stats;
    BlackBoxStorage storage(file);
    BlackBox box(controller, autopilot, stats, storage);
    outputs.begin();
    outputs.setFailsafe();
    controller.begin();
    TEST_ASSERT_TRUE(box.begin(false));

    // Цикл 500 Гц, а запись — по шагу в каждом такте (в прошивке — задача bbox,
    // здесь шаг идёт в том же цикле, поэтому задержка карты видна в длине такта:
    // это худший случай).
    box.requestManualStart();
    uint32_t worstLoopUs = 0, slowTicks = 0, ticks = 0;
    uint32_t next = micros();
    const uint32_t start = millis();
    while (millis() - start < 20000)
    {
        const uint32_t t0 = micros();
        imu.update();
        controller.update();
        stats.record(300);
        box.update(300);
        box.writerStep();
        const uint32_t spent = micros() - t0;
        worstLoopUs = spent > worstLoopUs ? spent : worstLoopUs;
        slowTicks += spent > 5000;
        ticks++;
        next += 2000;
        while (static_cast<int32_t>(micros() - next) < 0) {}
        if (static_cast<int32_t>(micros() - next) > 20000) next = micros();   // догнать, а не мчаться пачкой
    }
    box.requestManualStop();
    for (int i = 0; i < 200000 && box.getState() != BlackBox::State::Idle; ++i)
    {
        box.update(300);
        box.writerStep();
    }
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, box.getState());
    say("такты: %lu, худший %lu мкс, дольше 5 мс: %lu", static_cast<unsigned long>(ticks),
        static_cast<unsigned long>(worstLoopUs), static_cast<unsigned long>(slowTicks));
    say("ошибок записи %lu, стирания %lu", static_cast<unsigned long>(storage.writeErrors),
        static_cast<unsigned long>(storage.eraseErrors));
    TEST_ASSERT_EQUAL_UINT32(0, storage.writeErrors);

    // После "перезагрузки": полёт один, IMU ~ 500 Гц × 20 с, ничего не потеряно очередью.
    BlackBoxStorage again(file);
    TEST_ASSERT_TRUE(again.begin());
    TEST_ASSERT_EQUAL(1u, again.flightCount());
    const BlackBoxStorage::Flight& flight = again.flight(0);
    std::vector<uint8_t> sector(SECTOR);
    uint32_t imuRecords = 0, ctrl = 0, end = 0;
    for (uint32_t k = 0; k < flight.sectors; ++k)
    {
        TEST_ASSERT_TRUE(again.readSector(again.sectorOf(flight, k), sector.data()));
        size_t pos = SECTOR_HEADER_SIZE;
        while (pos + RECORD_HEADER <= SECTOR && sector[pos] != REC_ERASED)
        {
            const size_t stop = pos + RECORD_HEADER + sector[pos + 1];
            if (stop + RECORD_CHECK > SECTOR || crc8(&sector[pos], RECORD_HEADER + sector[pos + 1]) != sector[stop]) break;
            imuRecords += sector[pos] == REC_IMU;
            ctrl += sector[pos] == REC_CTRL;
            end += sector[pos] == REC_END;
            pos = stop + RECORD_CHECK;
        }
    }
    say("в полёте #%u: %lu секторов, IMU %lu, CTRL %lu, END %lu, ящик: ошибок/потерь очереди — см. выше", flight.number,
        static_cast<unsigned long>(flight.sectors), static_cast<unsigned long>(imuRecords), static_cast<unsigned long>(ctrl),
        static_cast<unsigned long>(end));
    TEST_ASSERT_EQUAL_UINT32(1, end);
    TEST_ASSERT_UINT32_WITHIN(ticks / 100, ticks, imuRecords);   // по записи IMU на такт: потеряно не больше 1 %
    TEST_ASSERT_UINT32_WITHIN(ticks / 500, ticks / 5, ctrl);

    TEST_ASSERT_TRUE(again.eraseAll());
}

static void runAllTests(void*)
{
    UNITY_BEGIN();
    RUN_TEST(test_reset_cause_is_a_normal_one);
    RUN_TEST(test_card_is_detected_on_four_bit_bus);
    RUN_TEST(test_file_is_found_and_contiguous);
    RUN_TEST(test_multi_block_writes_work_at_every_length);
    RUN_TEST(test_pages_write_with_bounded_latency_and_read_back_intact);
    RUN_TEST(test_header_scan_cost_on_the_whole_area);
    RUN_TEST(test_storage_erase_all_write_flights_and_find_them_after_reopen);
    RUN_TEST(test_blackbox_keeps_up_for_20_seconds_in_real_time);
    RUN_TEST(test_the_flight_task_was_not_disturbed);
    UNITY_END();

    // После тестов плата ждёт: клавиша 'D' перезагружает её в загрузчик DFU, чтобы
    // следующую прошивку залить без проводка BT0 (src/stm32/bootloader.cpp).
    for (;;)
    {
        if (Serial.available() && Serial.read() == 'D') stm32RebootToBootloader();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup()
{
    // USB CDC после сброса переоткрывается: ждём, пока ПК откроет порт (до 60 с).
    const uint32_t waitStart = millis();
    while (!Serial && millis() - waitStart < 60000) {}
    delay(500);

    // Причина перезагрузки читается один раз (флаги сбрасываются): до BlackBox::begin().
    startCause = readResetCause();
    Rtos::startTask(flightLikeTask, "flight", 2048, nullptr, Rtos::PRIORITY_FLIGHT);
    Rtos::startTask(runAllTests, "tests", 32768, nullptr, Rtos::PRIORITY_TELEMETRY);
    vTaskStartScheduler();   // не возвращается
}

void loop() {}
