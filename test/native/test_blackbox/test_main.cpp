// ============================================================
// Чёрный ящик: формат, кольцо секторов во флеше (NOR-фейк раздела —
// стирание секторами, запись только опускает биты), запись полёта
// на настоящих FlightController/Autopilot/IBusReceiver, старт и
// остановка, события, переполнение флеша в воздухе, выгрузка по UART.
//
// Запуск: pio test -e native -f native/test_blackbox
// Образ полёта для проверки декодера:
//   OPENPLANE_BLACKBOX_DUMP=/tmp/flight.bbl pio test -e native -f native/test_blackbox
//   python tools/blackbox.py decode /tmp/flight.bbl
// ============================================================

#include <Arduino.h>
#include <esp_partition.h>
#include <unity.h>

#include <cstdio>
#include <string>
#include <vector>

#include "autopilot/Autopilot.h"
#include "autopilot/PilotSwitches.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "hal/esp32/Esp32FlashPartition.h"
#include "rc/IBusReceiver.h"
#include "telemetry/BlackBox.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/DebugConsole.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LoopStats.h"
#include "helpers/TestSupport.h"

using namespace BlackBoxFormat;

void setUp()
{
    resetWorld();
    fake::resetPartitions();
}
void tearDown() {}

namespace
{
    constexpr uint32_t SECTOR = BlackBoxFormat::SECTOR_SIZE;

    // --------------------------------------------------------
    // Разбор флеша (как tools/blackbox.py)
    // --------------------------------------------------------

    struct Rec
    {
        uint8_t type = 0;
        std::vector<uint8_t> data;

        uint32_t t() const
        {
            uint32_t v = 0;
            memcpy(&v, data.data(), 4);
            return v;
        }
        std::string text() const { return std::string(data.begin() + 4, data.end()); }
        template <typename T> T as() const
        {
            T v;
            memcpy(&v, data.data(), sizeof(T));
            return v;
        }
    };

    // Записи сектора до первой стёртой или битой (CRC-8 не сошёлся).
    std::vector<Rec> parseSector(const uint8_t* sector, size_t* torn = nullptr)
    {
        std::vector<Rec> out;
        size_t pos = SECTOR_HEADER_SIZE;
        while (pos + RECORD_HEADER <= SECTOR)
        {
            const uint8_t type = sector[pos];
            const uint8_t length = sector[pos + 1];
            const size_t end = pos + RECORD_HEADER + length;
            if (type == REC_ERASED || end + RECORD_CHECK > SECTOR) break;
            if (crc8(sector + pos, RECORD_HEADER + length) != sector[end])
            {
                if (torn) (*torn)++;
                break;
            }
            Rec r;
            r.type = type;
            r.data.assign(sector + pos + 2, sector + end);
            out.push_back(r);
            pos = end + RECORD_CHECK;
        }
        return out;
    }

    std::vector<Rec> readFlight(BlackBoxStorage& storage, uint16_t number)
    {
        std::vector<Rec> out;
        const BlackBoxStorage::Flight* f = storage.findFlight(number);
        if (!f) return out;
        std::vector<uint8_t> sector(SECTOR);
        for (uint32_t k = 0; k < f->sectors; ++k)
        {
            storage.readSector(storage.sectorOf(*f, k), sector.data());
            const std::vector<Rec> part = parseSector(sector.data());
            out.insert(out.end(), part.begin(), part.end());
        }
        return out;
    }

    size_t countType(const std::vector<Rec>& recs, uint8_t type)
    {
        size_t n = 0;
        for (const Rec& r : recs) n += r.type == type;
        return n;
    }

    bool hasText(const std::vector<Rec>& recs, uint8_t type, const char* fragment)
    {
        for (const Rec& r : recs)
        {
            if (r.type == type && contains(r.text(), fragment)) return true;
        }
        return false;
    }

    std::vector<uint8_t> record(uint8_t type, uint32_t tUs, size_t payload, uint8_t fill)
    {
        std::vector<uint8_t> r(RECORD_HEADER + payload, fill);
        r[0] = type;
        r[1] = static_cast<uint8_t>(payload);
        memcpy(r.data() + 2, &tUs, 4);
        return r;
    }

    fake::FlashPartition& partitionOrNew(uint32_t sectors, uint8_t fill)
    {
        fake::FlashPartition* p = fake::partition("blackbox");
        return p ? *p : fake::addPartition("blackbox", sectors * SECTOR, fill);
    }

    // Полёт прямо через хранилище: n записей по 60 байт. Как после
    // включения: последний полёт защищён, остальное — под запись.
    uint16_t writeFlightDirect(Esp32FlashPartition& flash, size_t records, uint32_t startMs = 0)
    {
        BlackBoxStorage storage(flash);
        storage.begin();
        const BlackBoxStorage::Flight* newest = storage.newestFlight();
        while (storage.eraseStep(records * 60 / SECTOR + 2, newest ? newest->number : 0)) {}
        const uint16_t number = storage.openFlight();
        for (size_t i = 0; i < records; ++i)
        {
            const std::vector<uint8_t> r = record(REC_SYS, static_cast<uint32_t>(i), 58, static_cast<uint8_t>(i));
            storage.append(r.data(), r.size(), startMs + static_cast<uint32_t>(i));
        }
        storage.closeFlight();
        return number;
    }

    // --------------------------------------------------------
    // Самолёт целиком + чёрный ящик
    // --------------------------------------------------------

    struct Rig
    {
        fake::FlashPartition& part;
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
        Esp32FlashPartition flash{ "blackbox" };
        BlackBoxStorage storage{ flash };
        BlackBox box{ controller, autopilot, stats, storage, &switches };
        RcChannels rc;
        uint32_t maxWritesPerStep = 0;

        explicit Rig(uint32_t sectors = 512, uint8_t fill = 0xFF)
            : part(partitionOrNew(sectors, fill))
        {
            imu.data.accelZ = 1.0f;
            outputs.begin();
            controller.begin();
            flash.begin();
            box.begin(false);
            takeSerial();
        }

        // Один такт цикла: кадр пульта, FlightController, снимок, шаг записи.
        void tick(bool frame = true)
        {
            if (frame) board.rc.push(ibusFrame(rc));
            fake::advanceMs(2);
            imu.data.timestamp = micros();
            controller.update();
            stats.record(300);
            box.update(300);
            const uint32_t before = part.writes;
            box.writerStep();
            maxWritesPerStep = std::max(maxWritesPerStep, part.writes - before);
        }

        void run(uint32_t ms, bool frames = true)
        {
            for (uint32_t i = 0; i < ms / 2; ++i) tick(frames);
        }

        void arm()
        {
            rc.set(Channels::ARM, 1000).set(Channels::THROTTLE, 1000);
            tick();
            rc.set(Channels::ARM, 2000);
            tick();
        }

        void disarm() { rc.set(Channels::ARM, 1000).set(Channels::THROTTLE, 1000); }

        // Пока задача записи не закроет полёт.
        void drain()
        {
            for (int i = 0; i < 5000 && box.getState() == BlackBox::State::Stopping; ++i) tick();
        }

        // Трясёт IMU, как в полёте: условие "стоит на земле" не выполняется.
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
}


// ============================================================
// Формат
// ============================================================

void test_format_header_check_and_crc()
{
    SectorHeader h = makeHeader(42, 7, 1234);
    TEST_ASSERT_TRUE(isValid(h));

    SectorHeader torn = h;
    torn.seq ^= 0x10;   // бит не дописался
    TEST_ASSERT_FALSE(isValid(torn));

    SectorHeader erased;
    memset(&erased, 0xFF, sizeof(erased));
    TEST_ASSERT_FALSE(isValid(erased));

    const uint8_t digits[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc32(digits, sizeof(digits)));   // эталон CRC-32/zlib
    TEST_ASSERT_EQUAL_HEX8(0xF4, crc8(digits, sizeof(digits)));            // эталон CRC-8/SMBUS
    TEST_ASSERT_EQUAL_HEX32(crc32(digits, 9), crc32(digits + 4, 5, crc32(digits, 4)));

    TEST_ASSERT_EQUAL(sizeof(CtrlRecord), payloadSize(REC_CTRL));
    TEST_ASSERT_EQUAL(0u, payloadSize(REC_EVENT));
    TEST_ASSERT_TRUE(isKnownType(REC_END));
    TEST_ASSERT_FALSE(isKnownType(0x7E));
}


// ============================================================
// Хранилище
// ============================================================

void test_storage_fresh_partition_needs_no_erase_and_keeps_a_flight_across_reboot()
{
    fake::FlashPartition& part = fake::addPartition("blackbox", 64 * SECTOR);
    Esp32FlashPartition flash("blackbox");
    TEST_ASSERT_TRUE(flash.begin());

    BlackBoxStorage storage(flash);
    TEST_ASSERT_TRUE(storage.begin());
    TEST_ASSERT_EQUAL(64u, storage.totalSectors());
    TEST_ASSERT_EQUAL(0u, storage.freeSectors());   // пока не сверено — не известно

    while (storage.eraseStep(1, 0)) {}
    TEST_ASSERT_EQUAL(64u, storage.freeSectors());  // мусора нет — сверено всё
    TEST_ASSERT_EQUAL(0u, part.erases);             // и ничего не стиралось

    const uint16_t number = storage.openFlight();
    TEST_ASSERT_EQUAL(1, number);
    std::vector<std::vector<uint8_t>> written;
    for (uint32_t i = 0; i < 400; ++i)
    {
        written.push_back(record(REC_IMU, i * 2000, sizeof(ImuRecord), static_cast<uint8_t>(i)));
        TEST_ASSERT_TRUE(storage.append(written.back().data(), written.back().size(), i) >= 0);
    }
    storage.closeFlight();
    TEST_ASSERT_EQUAL(0u, part.bitRaises);

    // "Перезагрузка": новый объект читает только флеш.
    BlackBoxStorage after(flash);
    after.begin();
    TEST_ASSERT_EQUAL(1u, after.flightCount());
    const BlackBoxStorage::Flight& f = after.flight(0);
    TEST_ASSERT_EQUAL(1, f.number);
    TEST_ASSERT_TRUE(f.hasStart);
    TEST_ASSERT_EQUAL(3u, f.sectors);   // 400 × 21 байт (с CRC) — три сектора
    TEST_ASSERT_EQUAL(3u, after.headSector());

    const std::vector<Rec> recs = readFlight(after, 1);
    TEST_ASSERT_EQUAL(written.size(), recs.size());
    for (size_t i = 0; i < recs.size(); ++i)
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY(written[i].data() + 2, recs[i].data.data(), recs[i].data.size());
    }
    TEST_ASSERT_EQUAL(2, after.openFlight());   // нумерация продолжается
}

void test_storage_erases_garbage_always_but_old_flights_only_for_space()
{
    fake::FlashPartition& part = fake::addPartition("blackbox", 64 * SECTOR, 0x00);   // старая прошивка
    Esp32FlashPartition flash("blackbox");
    flash.begin();

    BlackBoxStorage storage(flash);
    storage.begin();
    while (storage.eraseStep(0, 0)) {}   // цель 0 — а мусор всё равно стирается
    TEST_ASSERT_EQUAL(64u, storage.freeSectors());
    TEST_ASSERT_EQUAL(4u, part.erases);  // четыре блока по 64 КБ, не 64 сектора

    // Три полёта по ~10 секторов.
    const uint16_t a = writeFlightDirect(flash, 650);
    const uint16_t b = writeFlightDirect(flash, 650);
    const uint16_t c = writeFlightDirect(flash, 650);

    BlackBoxStorage st(flash);
    st.begin();
    TEST_ASSERT_EQUAL(3u, st.flightCount());
    const uint32_t used = st.flight(0).sectors + st.flight(1).sectors + st.flight(2).sectors;
    while (st.eraseStep(64 - used, c)) {}   // места хватает — полёты не трогаются
    TEST_ASSERT_EQUAL(3u, st.flightCount());
    TEST_ASSERT_EQUAL(64u - used, st.freeSectors());

    // Нужно на 2 сектора больше — самый старый полёт стирается ЦЕЛИКОМ.
    while (st.eraseStep(64 - used + 2, c)) {}
    TEST_ASSERT_EQUAL(2u, st.flightCount());
    TEST_ASSERT_NULL(st.findFlight(a));
    TEST_ASSERT_NOT_NULL(st.findFlight(b));

    // Нужно всё — последний (защищённый) полёт остаётся.
    while (st.eraseStep(1000, c)) {}
    TEST_ASSERT_EQUAL(1u, st.flightCount());
    TEST_ASSERT_EQUAL(c, st.flight(0).number);
    TEST_ASSERT_TRUE(st.flight(0).hasStart);
    TEST_ASSERT_EQUAL(0u, part.bitRaises);
}

void test_storage_wraps_around_the_ring_and_finds_the_head_after_reboot()
{
    fake::FlashPartition& part = fake::addPartition("blackbox", 20 * SECTOR);
    Esp32FlashPartition flash("blackbox");
    flash.begin();

    uint16_t last = 0;
    for (int flight = 0; flight < 7; ++flight)
    {
        // Каждый раз — как после включения: найти голову, освободить место.
        BlackBoxStorage st(flash);
        st.begin();
        const BlackBoxStorage::Flight* newest = st.newestFlight();
        while (st.eraseStep(8, newest ? newest->number : 0)) {}
        TEST_ASSERT_TRUE(st.freeSectors() >= 8);

        last = st.openFlight();
        for (uint32_t i = 0; i < 400; ++i)   // 6 секторов
        {
            const std::vector<uint8_t> r = record(REC_RC, i, 40, static_cast<uint8_t>(flight));
            TEST_ASSERT_TRUE(st.append(r.data(), r.size(), i) >= 0);
        }
        st.closeFlight();
    }
    TEST_ASSERT_EQUAL(7, last);

    BlackBoxStorage st(flash);
    st.begin();
    const BlackBoxStorage::Flight* newest = st.newestFlight();
    TEST_ASSERT_NOT_NULL(newest);
    TEST_ASSERT_EQUAL(7, newest->number);
    TEST_ASSERT_TRUE(newest->hasStart);
    TEST_ASSERT_EQUAL((newest->firstSector + newest->sectors) % 20, st.headSector());

    const std::vector<Rec> recs = readFlight(st, 7);
    TEST_ASSERT_EQUAL(400u, recs.size());
    TEST_ASSERT_EQUAL_UINT8(6, recs.back().data[10]);
    TEST_ASSERT_EQUAL(0u, part.bitRaises);
}

void test_storage_power_loss_keeps_everything_written_before_it()
{
    fake::FlashPartition& part = fake::addPartition("blackbox", 16 * SECTOR);
    Esp32FlashPartition flash("blackbox");
    flash.begin();

    BlackBoxStorage st(flash);
    st.begin();
    while (st.eraseStep(100, 0)) {}
    st.openFlight();
    uint32_t pagesBefore = 0;
    bool powerLost = false;
    part.beforeWrite = [&](uint32_t, size_t) { return !powerLost; };
    for (uint32_t i = 0; i < 300; ++i)
    {
        if (i == 250)
        {
            powerLost = true;   // дальше во флеш ничего не доходит
            pagesBefore = part.writes;
        }
        const std::vector<uint8_t> r = record(REC_IMU, i, sizeof(ImuRecord), 0x11);
        st.append(r.data(), r.size(), i);
    }
    TEST_ASSERT_EQUAL(pagesBefore, part.writes);

    BlackBoxStorage after(flash);
    after.begin();
    TEST_ASSERT_EQUAL(1u, after.flightCount());
    const std::vector<Rec> recs = readFlight(after, 1);
    // Всё, что успело уйти целыми страницами, на месте; хвост — нет.
    TEST_ASSERT_TRUE(recs.size() >= 240 && recs.size() <= 250);
    for (size_t i = 0; i < recs.size(); ++i) TEST_ASSERT_EQUAL_UINT32(i, recs[i].t());
}


void test_torn_tail_record_is_dropped_by_its_crc()
{
    fake::FlashPartition& part = fake::addPartition("blackbox", 8 * SECTOR);
    Esp32FlashPartition flash("blackbox");
    flash.begin();
    BlackBoxStorage st(flash);
    st.begin();
    while (st.eraseStep(100, 0)) {}
    st.openFlight();
    for (uint32_t i = 0; i < 100; ++i)
    {
        const std::vector<uint8_t> r = record(REC_IMU, i, sizeof(ImuRecord), 0x22);
        st.append(r.data(), r.size(), i);
    }
    st.closeFlight();

    // Питание пропало посреди записи: последние байты не запрограммированы.
    const size_t end = SECTOR_HEADER_SIZE + 100 * (RECORD_HEADER + sizeof(ImuRecord) + RECORD_CHECK);
    for (size_t i = end - 5; i < end; ++i) part.data[i] = 0xFF;

    size_t torn = 0;
    const std::vector<Rec> recs = parseSector(part.data.data(), &torn);
    TEST_ASSERT_EQUAL(99u, recs.size());
    TEST_ASSERT_EQUAL(1u, torn);
    TEST_ASSERT_EQUAL_UINT32(98, recs.back().t());
}


// ============================================================
// Запись полёта
// ============================================================

void test_arm_and_throttle_start_recording_with_preroll_and_disarm_stops_it()
{
    Rig rig;
    rig.run(10000);                         // на земле: предзапись
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());

    rig.arm();
    rig.run(4000);                          // заармлен, газ внизу — не пишет
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());

    const uint32_t throttleUpUs = micros();
    rig.rc.set(Channels::THROTTLE, 1600);
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BlackBox: запись полёта #1 — ARM и газ"));

    rig.shake(20000);
    rig.rc.set(Channels::THROTTLE, 1000);
    rig.disarm();
    rig.run(9000);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());   // ещё 10 с после DISARM
    rig.run(1200);
    rig.drain();
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BlackBox: полёт #1 записан"));

    BlackBoxStorage st(rig.flash);
    st.begin();
    TEST_ASSERT_EQUAL(1u, st.flightCount());
    const std::vector<Rec> recs = readFlight(st, 1);

    // Начало: схема и параметры полёта.
    TEST_ASSERT_EQUAL(REC_SCHEMA, recs.front().type);
    TEST_ASSERT_TRUE(hasText(recs, REC_SCHEMA, "16 IMU t_us:I gx:h/10"));
    TEST_ASSERT_TRUE(hasText(recs, REC_SCHEMA, "17 + "));   // CTRL длинная — продолжение
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "start=ARM и газ"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "reset_reason=POWERON"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "sensor.imu=FakeIMU"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "pid.roll="));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "cfg.RUDDER_MAX_US=300"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "bind=SwC (CH7): MANUAL / STABILIZE"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "ARM"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "запись: старт (ARM и газ)"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "состояние: ARM=YES MODE=MANUAL RX=OK"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "DISARM"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    TEST_ASSERT_EQUAL_STRING("DISARM", recs.back().text().c_str());

    // Предзапись: IMU за 10 с до газа (включая момент ARM), дальше — каждый такт.
    uint32_t firstImu = 0;
    size_t imu = 0;
    for (const Rec& r : recs)
    {
        if (r.type != REC_IMU) continue;
        if (!imu++) firstImu = r.t();
    }
    TEST_ASSERT_UINT32_WITHIN(20000, Config::BLACKBOX_PREROLL_MS * 1000, throttleUpUs - firstImu);
    // Предзапись 10 с + полёт 20 с + 10 с после DISARM.
    TEST_ASSERT_UINT32_WITHIN(5, (10000 + 20000 + 10000) / 2, imu);
    TEST_ASSERT_UINT32_WITHIN(5, (10000 + 20000 + 10000) / 10, countType(recs, REC_CTRL));
    TEST_ASSERT_TRUE(countType(recs, REC_RC) > 1900);
    TEST_ASSERT_TRUE(countType(recs, REC_SYS) >= 39);

    // В воздухе газ дошёл до ESC — видно в CTRL.
    bool sawThrottle = false;
    for (const Rec& r : recs)
    {
        if (r.type != REC_CTRL) continue;
        const CtrlRecord c = r.as<CtrlRecord>();
        if (c.out[4] == 1300 && (c.flags & Flag::ARMED)) sawThrottle = true;   // стик 1600, ограничен THROTTLE_LIMIT_PCT
    }
    TEST_ASSERT_TRUE(sawThrottle);

    // Запись флеша — не больше двух страниц за шаг, стирания в полёте нет.
    TEST_ASSERT_TRUE(rig.maxWritesPerStep <= 2);
    TEST_ASSERT_EQUAL(0u, rig.part.bitRaises);
}

void test_armed_with_throttle_down_does_not_record()
{
    Rig rig;
    rig.arm();
    rig.run(60000);
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    BlackBoxStorage st(rig.flash);
    st.begin();
    TEST_ASSERT_EQUAL(0u, st.flightCount());
}

void test_link_loss_and_motor_off_in_the_air_keep_recording()
{
    Rig rig;
    rig.arm();
    rig.rc.set(Channels::THROTTLE, 1700);
    rig.shake(3000);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    // Связь пропала на 3 с: автопилот планирует, мотор выключен.
    for (int i = 0; i < 1500; ++i)
    {
        rig.imu.data.gyroY = (i % 2) ? 8.0f : -8.0f;
        rig.tick(false);
    }
    TEST_ASSERT_TRUE(rig.controller.isReceiverFailsafe());
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    // Связь вернулась, мотор в ноль — планирование 40 с, самолёт "летит".
    rig.rc.set(Channels::THROTTLE, 1000);
    rig.shake(40000);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    rig.box.requestManualStop();
    rig.tick();
    rig.drain();
    BlackBoxStorage st(rig.flash);
    st.begin();
    const std::vector<Rec> recs = readFlight(st, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "связь: потеряна — нет кадров iBUS"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "режим FAILSAFE_GLIDE"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "связь: есть"));

    bool sawGlideFlag = false;
    for (const Rec& r : recs)
    {
        if (r.type == REC_CTRL && (r.as<CtrlRecord>().flags & Flag::FS_GLIDE)) sawGlideFlag = true;
    }
    TEST_ASSERT_TRUE(sawGlideFlag);
}

void test_armed_but_still_on_the_ground_stops_after_landed_timeout()
{
    Rig rig;
    rig.arm();
    rig.rc.set(Channels::THROTTLE, 1500);
    rig.shake(5000);
    rig.rc.set(Channels::THROTTLE, 1000);

    // Сел: мотор стоит, всё неподвижно; DISARM забыли.
    rig.run(Config::BLACKBOX_LANDED_STOP_MS - 1000);
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    rig.run(1100);
    rig.drain();
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    TEST_ASSERT_TRUE(rig.controller.isArmed());

    BlackBoxStorage st(rig.flash);
    st.begin();
    const std::vector<Rec> recs = readFlight(st, 1);
    TEST_ASSERT_EQUAL_STRING("стоит на земле, мотор выключен", recs.back().text().c_str());

    // Газ снова — новый полёт.
    rig.rc.set(Channels::THROTTLE, 1500);
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
}

void test_crash_reset_starts_recording_at_boot_and_holds_it()
{
    fake::chip().resetReason = ESP_RST_BROWNOUT;
    Rig rig;
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    rig.run(Config::BLACKBOX_RESET_HOLD_MS - 2000);   // без ARM — всё равно пишет
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    rig.run(3000);
    rig.drain();
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());

    BlackBoxStorage st(rig.flash);
    st.begin();
    const std::vector<Rec> recs = readFlight(st, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "reset_reason=BROWNOUT"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "start=перезагрузка после сбоя"));
}

void test_manual_recording_on_the_bench_ignores_disarm()
{
    Rig rig;
    rig.box.requestManualStart();
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    TEST_ASSERT_TRUE(rig.box.isManual());

    rig.run(30000);   // без ARM дольше, чем POSTROLL
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());

    rig.box.requestManualStop();
    rig.tick();
    rig.drain();
    BlackBoxStorage st(rig.flash);
    st.begin();
    const std::vector<Rec> recs = readFlight(st, 1);
    TEST_ASSERT_EQUAL_STRING("вручную", recs.back().text().c_str());
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "start=вручную"));
}

void test_events_mode_sensors_gps_and_features()
{
    Rig rig;
    rig.box.requestManualStart();
    rig.tick();

    rig.rc.set(Channels::SWC, 1500);        // STABILIZE
    rig.run(100);
    rig.imu.available = false;              // IMU отвалился
    rig.run(100);
    rig.imu.available = true;
    rig.gps.data.fixType = 3;
    rig.gps.data.numSatellites = 9;
    rig.gps.data.timestamp = 1;
    rig.run(100);
    rig.rc.set(Channels::SWB, 2000);        // FLAPS
    rig.run(100);

    rig.box.requestManualStop();
    rig.tick();
    rig.drain();
    BlackBoxStorage st(rig.flash);
    st.begin();
    const std::vector<Rec> recs = readFlight(st, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "режим STABILIZE"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "датчики: -IMU"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "датчики: +IMU"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "GPS: фикс 3, спутников 9"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "+FLAPS"));
    TEST_ASSERT_TRUE(countType(recs, REC_GPS) >= 1);
    TEST_ASSERT_TRUE(countType(recs, REC_NAV) >= 3);
}

void test_power_record_scales_battery_and_current_dividers()
{
    fake::gpio().analogMv[Config::PIN_VBAT_ADC] = 1818;      // 3S ~12.0 В за делителем 56k/10k
    fake::gpio().analogMv[Config::PIN_CURRENT_ADC] = 1500;   // 2.5 В с датчика за 10k/15k
    Rig rig;
    rig.box.requestManualStart();
    rig.run(1000);
    rig.box.requestManualStop();
    rig.tick();
    rig.drain();

    BlackBoxStorage st(rig.flash);
    st.begin();
    const std::vector<Rec> recs = readFlight(st, 1);
    TEST_ASSERT_UINT32_WITHIN(1, 10, countType(recs, REC_POWER));   // 10 Гц
    for (const Rec& r : recs)
    {
        if (r.type != REC_POWER) continue;
        const PowerRecord p = r.as<PowerRecord>();
        TEST_ASSERT_UINT16_WITHIN(2, 11999, p.vbatMv);
        TEST_ASSERT_UINT16_WITHIN(2, 2500, p.currentMv);
    }
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "power.vbat_divider=6.6000"));
}

void test_flash_full_in_the_air_keeps_data_in_ring_and_flushes_after_landing()
{
    // 120 секторов (~480 КБ); старый полёт занимает половину.
    fake::FlashPartition& part = fake::addPartition("blackbox", 120 * SECTOR);
    {
        Esp32FlashPartition flash("blackbox");
        flash.begin();
        writeFlightDirect(flash, 4000);   // ~60 секторов
    }
    Rig rig;
    rig.run(5000);   // на земле: старый полёт — последний, он защищён
    const uint32_t erasesOnGround = part.erases;

    rig.arm();
    rig.rc.set(Channels::THROTTLE, 1600);
    rig.shake(10000);   // ~15 с с предзаписью — больше, чем свободно
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    TEST_ASSERT_EQUAL(erasesOnGround, part.erases);   // в воздухе флеш не стирается

    rig.rc.set(Channels::THROTTLE, 1000);
    rig.disarm();
    rig.run(10500);
    rig.drain();
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    TEST_ASSERT_TRUE(part.erases > erasesOnGround);   // после посадки — стёрт старый

    BlackBoxStorage st(rig.flash);
    st.begin();
    TEST_ASSERT_NULL(st.findFlight(1));               // старый полёт ушёл целиком
    const std::vector<Rec> recs = readFlight(st, 2);
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "флеш: стёртое место кончилось"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);     // полёт целиком, до конца
    TEST_ASSERT_EQUAL_STRING("DISARM", recs.back().text().c_str());
    TEST_ASSERT_EQUAL(0u, part.bitRaises);
}

void test_flight_longer_than_the_partition_is_closed_without_its_tail()
{
    fake::FlashPartition& part = fake::addPartition("blackbox", 24 * SECTOR);
    Rig rig;
    rig.box.requestManualStart();
    rig.run(20000);   // ~360 КБ в разделе на ~96 КБ
    rig.box.requestManualStop();
    rig.tick();
    rig.drain();
    TEST_ASSERT_EQUAL(BlackBox::State::Idle, rig.box.getState());
    TEST_ASSERT_TRUE(contains(takeSerial(), "длиннее раздела — конец не поместился"));

    BlackBoxStorage st(rig.flash);
    st.begin();
    TEST_ASSERT_EQUAL(1u, st.flightCount());
    TEST_ASSERT_TRUE(st.flight(0).hasStart);          // начало со схемой на месте
    TEST_ASSERT_EQUAL(0u, part.bitRaises);

    // Следующая запись идёт как обычно.
    rig.box.requestManualStart();
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
}

void test_host_list_and_download_frames_with_crc()
{
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
    TEST_ASSERT_TRUE(contains(list, "BB:END"));

    Serial.begin(115200);
    Serial.pushRx("G");   // ПК подтвердил новую скорость
    rig.box.handleHostCommand("bb get 1 2000000");
    const std::string out = takeSerial();
    TEST_ASSERT_TRUE(contains(out, "BB:SEND n=1 sectors="));
    TEST_ASSERT_EQUAL(2u, Serial.baudChanges().size());
    TEST_ASSERT_EQUAL_UINT32(2000000, Serial.baudChanges()[0]);
    TEST_ASSERT_EQUAL_UINT32(115200, Serial.baudChanges()[1]);

    BlackBoxStorage st(rig.flash);
    st.begin();
    const BlackBoxStorage::Flight& f = *st.findFlight(1);
    const size_t begin = out.find('\n') + 1;
    std::vector<uint8_t> image;
    for (uint32_t k = 0; k < f.sectors; ++k)
    {
        const size_t at = begin + k * (4 + SECTOR + 4);
        TEST_ASSERT_EQUAL_HEX8(0xA5, static_cast<uint8_t>(out[at]));
        TEST_ASSERT_EQUAL_HEX8(0x5A, static_cast<uint8_t>(out[at + 1]));
        TEST_ASSERT_EQUAL(k, static_cast<uint8_t>(out[at + 2]) | static_cast<uint8_t>(out[at + 3]) << 8);
        const uint8_t* data = reinterpret_cast<const uint8_t*>(out.data() + at + 4);
        uint32_t crc;
        memcpy(&crc, data + SECTOR, 4);
        TEST_ASSERT_EQUAL_HEX32(crc32(data, SECTOR), crc);
        std::vector<uint8_t> sector(SECTOR);
        st.readSector(st.sectorOf(f, k), sector.data());
        TEST_ASSERT_EQUAL_UINT8_ARRAY(sector.data(), data, SECTOR);
        image.insert(image.end(), data, data + SECTOR);
    }
    TEST_ASSERT_TRUE(contains(out, "BB:DONE n=1 crc="));

    if (const char* path = getenv("OPENPLANE_BLACKBOX_DUMP"))
    {
        FILE* file = fopen(path, "wb");
        if (file)
        {
            fwrite(image.data(), 1, image.size(), file);
            fclose(file);
        }
    }

    // Нет ответа на новой скорости — скорость возвращается, ошибка.
    rig.box.handleHostCommand("bb get 1 921600");
    TEST_ASSERT_TRUE(contains(takeSerial(), "BB:ERR ПК не ответил"));
    TEST_ASSERT_EQUAL_UINT32(115200, Serial.baud());

    rig.box.handleHostCommand("bb get 9");
    TEST_ASSERT_TRUE(contains(takeSerial(), "BB:ERR нет полёта 9"));

    rig.arm();
    rig.box.handleHostCommand("bb get 1");
    TEST_ASSERT_TRUE(contains(takeSerial(), "BB:ERR занято"));
}

void test_console_menu_and_host_commands()
{
    Rig rig;
    DebugLogger logger(rig.controller, &rig.autopilot, &rig.stats);
    DebugConsole console(rig.controller, rig.outputs, rig.autopilot, logger, nullptr, &rig.box);

    Serial.pushRx("k");
    console.update();
    const std::string menu = takeSerial();
    TEST_ASSERT_TRUE(contains(menu, "Чёрный ящик"));
    TEST_ASSERT_TRUE(contains(menu, "BlackBox: ждёт ARM и газ"));
    TEST_ASSERT_TRUE(contains(menu, "полётов нет"));

    Serial.pushRx("r");   // запись вручную
    console.update();
    rig.tick();
    TEST_ASSERT_EQUAL(BlackBox::State::Recording, rig.box.getState());
    rig.run(1000);

    Serial.pushRx("k");
    console.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "r  остановить запись"));
    Serial.pushRx("r");
    console.update();
    rig.tick();
    rig.drain();

    // Команда ПК — байт STX и строка; лог и меню не мешают.
    Serial.pushRx(std::string("\x02") + "bb list\n");
    console.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "BB:FLIGHT n=1"));

    // Стереть всё: 'e', затем подтверждение.
    Serial.pushRx("k");
    console.update();
    Serial.pushRx("e");
    console.update();
    Serial.pushRx("n");
    console.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "Стирание отменено"));
    Serial.pushRx("ey");
    console.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "Стёрто."));
    BlackBoxStorage st(rig.flash);
    st.begin();
    TEST_ASSERT_EQUAL(0u, st.flightCount());
}

void test_no_partition_leaves_the_box_off()
{
    FakeImu imu;
    FakeBoard board;
    IBusReceiver receiver(board.rcUart());
    ControlMixer mixer;
    ThrottleManager throttle;
    FlightOutputs outputs(board);
    Autopilot autopilot(&imu);
    ArmingManager arming(&autopilot);
    FlightController controller(receiver, mixer, throttle, arming, outputs, &autopilot);
    LoopStats stats;
    Esp32FlashPartition flash("blackbox");
    TEST_ASSERT_FALSE(flash.begin());
    BlackBoxStorage storage(flash);
    BlackBox box(controller, autopilot, stats, storage);
    TEST_ASSERT_FALSE(box.begin(false));
    TEST_ASSERT_TRUE(contains(takeSerial(), "нет раздела blackbox"));

    box.update(100);
    box.writerStep();
    box.requestManualStart();
    box.update(100);
    TEST_ASSERT_EQUAL(BlackBox::State::Off, box.getState());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_format_header_check_and_crc);
    RUN_TEST(test_storage_fresh_partition_needs_no_erase_and_keeps_a_flight_across_reboot);
    RUN_TEST(test_storage_erases_garbage_always_but_old_flights_only_for_space);
    RUN_TEST(test_storage_wraps_around_the_ring_and_finds_the_head_after_reboot);
    RUN_TEST(test_storage_power_loss_keeps_everything_written_before_it);
    RUN_TEST(test_torn_tail_record_is_dropped_by_its_crc);
    RUN_TEST(test_arm_and_throttle_start_recording_with_preroll_and_disarm_stops_it);
    RUN_TEST(test_armed_with_throttle_down_does_not_record);
    RUN_TEST(test_link_loss_and_motor_off_in_the_air_keep_recording);
    RUN_TEST(test_armed_but_still_on_the_ground_stops_after_landed_timeout);
    RUN_TEST(test_crash_reset_starts_recording_at_boot_and_holds_it);
    RUN_TEST(test_manual_recording_on_the_bench_ignores_disarm);
    RUN_TEST(test_events_mode_sensors_gps_and_features);
    RUN_TEST(test_power_record_scales_battery_and_current_dividers);
    RUN_TEST(test_flash_full_in_the_air_keeps_data_in_ring_and_flushes_after_landing);
    RUN_TEST(test_flight_longer_than_the_partition_is_closed_without_its_tail);
    RUN_TEST(test_host_list_and_download_frames_with_crc);
    RUN_TEST(test_console_menu_and_host_commands);
    RUN_TEST(test_no_partition_leaves_the_box_off);
    return UNITY_END();
}
