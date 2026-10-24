// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на Matek
// H743 с SD-картой: загрузка находит карту и BLACKBOX.BIN, ARM включает
// запись (blackbox_mode = NORMAL), задача bbox пишет полёт на карту —
// гироскоп, PID и моторы каждый цикл, состояние 50 Гц, события, заголовок
// с настройкой; консоль показывает полёты ('k') и выгружает их на ПК
// ("bb list", "bb get N" — как у самолёта, tools/blackbox.py и Studio).
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_copter_blackbox
// ============================================================

#define BOARD_MATEK_H743
#define SENSOR_KIT SENSOR_KIT_MATEK_H743
#define VEHICLE_MULTIROTOR
#include "../../../src/stm32/main_copter.cpp"

#include <unity.h>

#include <string>
#include <vector>

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
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;   // DPS310 — те же регистры
    fat_image::Result image;

    constexpr uint32_t CARD_BLOCKS = 32768;
    constexpr uint32_t FILE_BYTES = 8u * 1024 * 1024;

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

    void wireUpBoard()
    {
        imuChip.rotationCwDeg = 90;
        imuChip.attach(SPI, static_cast<uint8_t>(PC15));
        imuChip.install();
        baroChip.attach(Wire, 0x76);
        baroChip.install();
        fake::gpio().analogMv[PC0] = 25200 / 11;   // 6S
    }

    // Такт: полётная задача, потом задача записи (ниже приоритетом — в паузе цикла).
    void tick(const RcChannels& rc)
    {
        imuChip.update(world);
        baroChip.update(world);
        if (millis() % 7 == 0) port(PC7).pushRx(ibusFrame(rc));
        fake::runTask(*fake::findTask("flight"), 1);
        fake::runTask(*fake::findTask("bbox"), 2);
    }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(rc);
    }

    // Весь BLACKBOX.BIN глазами хранилища — как после перезагрузки.
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

    const Rec* firstOf(const std::vector<Rec>& recs, uint8_t type)
    {
        for (const Rec& r : recs)
        {
            if (r.type == type) return &r;
        }
        return nullptr;
    }

    template <typename T>
    T payloadAs(const Rec& r)
    {
        T v;
        TEST_ASSERT_EQUAL(sizeof(T), r.data.size());
        memcpy(&v, r.data.data(), sizeof(T));
        return v;
    }
}

void test_boot_finds_the_card_and_waits_for_arm()
{
    imuChip.update(world);
    baroChip.update(world);
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "SD-карта: 16 МБ, файл BLACKBOX.BIN: ок"), log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BlackBox: ждёт ARM и газ"), log.c_str());
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    const fake::TaskRecord* bbox = fake::findTask("bbox");
    const fake::TaskRecord* flight = fake::findTask("flight");
    TEST_ASSERT_NOT_NULL(bbox);
    TEST_ASSERT_GREATER_THAN(bbox->priority, flight->priority);
}

void test_arm_records_the_flight_to_the_card()
{
    // установка IMU — три позы (без неё H743-WING не армится)
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

    // заданное в CLI попадает в заголовок полёта
    char err[96];
    TEST_ASSERT_TRUE(copterParams.parse(copter::PARAM_P_ROLL, copterParams.pidProfile(), "52", err, sizeof(err)));
    applySettings();

    RcChannels rc;
    run(rc, 3.0);
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());   // без ARM — только предзапись
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Recording, copterBlackBox.getState());
    rc.set(Channels::THROTTLE, 1300);
    const uint32_t start = millis();
    run(rc, 5.0);
    TEST_ASSERT_UINT32_WITHIN(10, start + 5000, millis());   // запись на карту цикл не растягивает
    rc.set(Channels::THROTTLE, 1000).set(Channels::ARM, 1000);
    run(rc, CopterBlackBox::POSTROLL_MS / 1000.0 + 0.5);
    for (int i = 0; i < 5000 && copterBlackBox.getState() == BlackBoxCore::State::Stopping; ++i) tick(rc);
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "BlackBox: полёт #1 записан"), log.c_str());

    CardView card;
    TEST_ASSERT_EQUAL_UINT32(1, card.storage.flightCount());
    const std::vector<Rec> recs = readFlight(card.storage, 1);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "vehicle=copter"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "build=freestyle5"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "motors=4"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "loop_hz=1000"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "set.p_roll=52"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "pid.roll="));   // с чем летит (без save — прежнее), set.* — что задано
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "bits.arm_blockers=RXLOSS THROTTLE"));
    TEST_ASSERT_TRUE(hasText(recs, REC_SCHEMA, "QGYRO"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "запись: старт (ARM)"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "ARM"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "DISARM"));
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);

    // 2 с предзаписи + 5.1 с ARM + 2 с после — каждый цикл 1 кГц
    const size_t gyro = countType(recs, REC_QGYRO);
    char why[120];
    snprintf(why, sizeof(why), "QGYRO %u QCTRL %u QSTATE %u", (unsigned)gyro, (unsigned)countType(recs, REC_QCTRL),
             (unsigned)countType(recs, REC_QSTATE));
    TEST_ASSERT_TRUE_MESSAGE(gyro > 8500 && gyro < 9800, why);
    TEST_ASSERT_EQUAL_MESSAGE(gyro, countType(recs, REC_QCTRL), why);
    TEST_ASSERT_EQUAL_MESSAGE(0, countType(recs, REC_QMOT8), why);   // квадро — моторы 1–4 в QCTRL
    TEST_ASSERT_TRUE_MESSAGE(countType(recs, REC_QSTATE) > 420 && countType(recs, REC_QSTATE) < 500, why);
    TEST_ASSERT_TRUE(countType(recs, REC_SYS) >= 8);
    TEST_ASSERT_EQUAL_UINT32(0, copterBlackBox.isRecording() ? 1 : 0);

    // в ARM: моторы крутятся, флаг ARM, газ стика 0.3
    bool sawArmed = false, sawSpin = false;
    for (const Rec& r : recs)
    {
        if (r.type == REC_QSTATE)
        {
            const auto s = payloadAs<BlackBoxFormat::CopterStateRecord>(r);
            if ((s.flags & BlackBoxFormat::CopterFlag::ARMED) && s.rcThrottle > 250 && s.rcThrottle < 350) sawArmed = true;
            TEST_ASSERT_TRUE(s.vbatMv > 24000 && s.vbatMv < 26000);
        }
        if (r.type == REC_QCTRL)
        {
            const auto c = payloadAs<BlackBoxFormat::CopterCtrlRecord>(r);
            if (c.motor[0] > 0 && c.motor[3] > 0) sawSpin = true;
        }
    }
    TEST_ASSERT_TRUE(sawArmed);
    TEST_ASSERT_TRUE(sawSpin);
    const Rec* g = firstOf(recs, REC_QGYRO);
    TEST_ASSERT_NOT_NULL(g);
    TEST_ASSERT_EQUAL(sizeof(BlackBoxFormat::CopterGyroRecord), g->data.size());
    TEST_ASSERT_EQUAL_UINT32(0, sdCard.errors);
}

void test_console_lists_and_downloads_the_flight()
{
    RcChannels rc;
    takeSerial();
    Serial.pushRx("k");
    run(rc, 0.01);
    const std::string menu = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(menu, "#1 "), menu.c_str());

    Serial.pushRx(std::string("\x02") + "bb list\n");
    run(rc, 0.01);
    const std::string list = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(list, "BB:STATE state=idle"), list.c_str());
    TEST_ASSERT_TRUE(contains(list, "BB:FLIGHT n=1 sectors="));
    TEST_ASSERT_TRUE(contains(list, "BB:END"));

    // выгрузка: BB:SEND, ПК отвечает 'G', кадры A5 5A + номер + сектор + CRC-32, BB:DONE
    Serial.pushRx(std::string("\x02") + "bb get 1\n");
    Serial.pushRx("G");
    run(rc, 0.01);
    const std::string out = takeSerial();
    const size_t at = out.find("BB:SEND n=1 sectors=");
    TEST_ASSERT_TRUE_MESSAGE(at != std::string::npos, out.substr(0, 200).c_str());
    const unsigned sectors = static_cast<unsigned>(atoi(out.c_str() + at + strlen("BB:SEND n=1 sectors=")));
    size_t pos = out.find('\n', at) + 1;
    std::vector<uint8_t> data;
    for (unsigned k = 0; k < sectors; ++k)
    {
        TEST_ASSERT_TRUE(pos + 4 + SECTOR_SIZE + 4 <= out.size());
        TEST_ASSERT_EQUAL_HEX8(0xA5, static_cast<uint8_t>(out[pos]));
        TEST_ASSERT_EQUAL_HEX8(0x5A, static_cast<uint8_t>(out[pos + 1]));
        const uint8_t* sector = reinterpret_cast<const uint8_t*>(out.data() + pos + 4);
        uint32_t crc;
        memcpy(&crc, out.data() + pos + 4 + SECTOR_SIZE, 4);
        TEST_ASSERT_EQUAL_HEX32(BlackBoxFormat::crc32(sector, SECTOR_SIZE), crc);
        data.insert(data.end(), sector, sector + SECTOR_SIZE);
        pos += 4 + SECTOR_SIZE + 4;
    }
    TEST_ASSERT_TRUE(contains(out.substr(pos), "BB:DONE n=1"));
    std::vector<Rec> recs;
    for (unsigned k = 0; k < sectors; ++k)
    {
        const std::vector<Rec> part = parseSector(data.data() + k * SECTOR_SIZE);
        recs.insert(recs.end(), part.begin(), part.end());
    }
    TEST_ASSERT_TRUE(countType(recs, REC_QGYRO) > 8500);
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "vehicle=copter"));
}

void test_blackbox_settings_from_cli()
{
    // NONE — ARM не пишет
    char err[96];
    TEST_ASSERT_TRUE(copterParams.parse(copter::PARAM_BLACKBOX_DEVICE, 0, "NONE", err, sizeof(err)));
    applySettings();
    RcChannels rc;
    run(rc, 0.5);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.3);
    TEST_ASSERT_TRUE(copterController.isArmed());
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, copterBlackBox.getState());
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);

    // SDCARD, 1/4 — гироскоп 250 Гц
    TEST_ASSERT_TRUE(copterParams.parse(copter::PARAM_BLACKBOX_DEVICE, 0, "SDCARD", err, sizeof(err)));
    TEST_ASSERT_TRUE(copterParams.parse(copter::PARAM_BLACKBOX_SAMPLE_RATE, 0, "1/4", err, sizeof(err)));
    applySettings();
    rc.set(Channels::ARM, 2000);
    run(rc, 2.0);
    rc.set(Channels::ARM, 1000);
    run(rc, CopterBlackBox::POSTROLL_MS / 1000.0 + 0.5);
    for (int i = 0; i < 5000 && copterBlackBox.getState() == BlackBoxCore::State::Stopping; ++i) tick(rc);
    takeSerial();

    CardView card;
    TEST_ASSERT_EQUAL_UINT32(2, card.storage.flightCount());
    const std::vector<Rec> recs = readFlight(card.storage, 2);
    const size_t gyro = countType(recs, REC_QGYRO);
    char why[64];
    snprintf(why, sizeof(why), "QGYRO %u", (unsigned)gyro);
    TEST_ASSERT_TRUE_MESSAGE(gyro > 1300 && gyro < 1700, why);   // ~6 с по 250 Гц
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "rate.gyro_hz=250"));
}

int main()
{
    fake::sdCard().resize(CARD_BLOCKS);
    image = fat_image::format(fake::sdCard(), { { "BLACKBOX.BIN", FILE_BYTES } });
    std::fill_n(fake::sdCard().data.begin() + static_cast<size_t>(image.firstBlock.at("BLACKBOX.BIN")) * 512, FILE_BYTES, 0xFF);
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_finds_the_card_and_waits_for_arm);
    RUN_TEST(test_arm_records_the_flight_to_the_card);
    RUN_TEST(test_console_lists_and_downloads_the_flight);
    RUN_TEST(test_blackbox_settings_from_cli);
    return UNITY_END();
}
