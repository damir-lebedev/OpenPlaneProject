// ============================================================
// FPV-обвес коптера на полётнике F405 (src/stm32/main_copter.cpp целиком):
// видеопередатчик по SmartAudio и PINIO с тумблеров. Плата — эталонная
// AocodaRC F405 V3, только провод VTX — на TX4 (PA0), как у GETFUN F405V3
// (площадка «TX4» к SmartAudio передатчика).
//
// Передатчик — эмулятор SmartAudio 2.1 на 4 Вт (test/native/helpers/
// VtxEmulator.h) на однопроводном UART: прошивка сама находит его, ставит
// полосу и канал из настроек, мощность — с тумблера (`vtx` в CLI), но не
// выше потолка vtx_max_power_mw. PINIO1 (PC5, режим USER1, инверсия) —
// переключатель камер у GETFUN. GPS (u-blox M10, UART1 этой платы) и
// телеметрия CRSF в пульт — в тот же UART2, откуда приходят каналы ELRS.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_f405_fpv
// ============================================================

#define BOARD_STM32F405
#define OP_BOARD_FILE "config/boards/AOCODARCF405V3.h"
#define SENSOR_KIT SENSOR_KIT_BF_BOARD
#define VEHICLE_MULTIROTOR
#include "config/boards/AOCODARCF405V3.h"
// как у GETFUN F405V3: SmartAudio — на TX4
#undef OPB_UART_VTX
#define OPB_UART_VTX 4
#include "../../../src/stm32/main_copter.cpp"

#include <unity.h>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"
#include "helpers/VtxEmulator.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;
    VtxEmulator vtxChip;
    bool crsfReceiver = false;
    uint32_t lastGpsMs = 0;

    HardwareSerial& port(uint32_t pin)
    {
        HardwareSerial* p = fake::uartByRx(pin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким пином не создан");
        return *p;
    }

    void tick(const RcChannels& rc)
    {
        imuChip.update(world);
        baroChip.update(world);
        if (millis() % 7 == 0) port(PA3).pushRx(crsfReceiver ? crsfChannels(rc) : ibusFrame(rc));
        if (millis() - lastGpsMs >= 100)
        {
            lastGpsMs = millis();
            const std::vector<uint8_t> frame = chips::navPvt(world);
            port(PA10).pushRx(std::string(frame.begin(), frame.end()));
        }
        fake::runTask(*fake::findTask("flight"), 1);
        // провод SmartAudio: что прошивка послала — передатчику, его ответ — обратно
        HardwareSerial& wire = port(PA0);
        const std::string sent = wire.takeTx();
        if (!sent.empty())
        {
            const std::vector<uint8_t> reply = vtxChip.receive(reinterpret_cast<const uint8_t*>(sent.data()), sent.size(),
                                                               static_cast<uint32_t>(wire.baud()), wire.config() == SERIAL_8N2);
            if (!reply.empty()) wire.pushRx(reply.data(), reply.size());
        }
    }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(rc);
    }
}

void test_vtx_found_on_tx4_and_tuned_from_settings()
{
    imuChip.attach(SPI, static_cast<uint8_t>(PA4));
    imuChip.install();
    baroChip.attach(Wire, 0x76);
    baroChip.install();
    fake::gpio().analogMv[PC2] = 25200 / 11;
    imuChip.update(world);
    baroChip.update(world);
    setup();
    takeSerial();
    TEST_ASSERT_TRUE(port(PA0).isHalfDuplex());

    RcChannels rc;
    run(rc, 1.5);
    TEST_ASSERT_TRUE(vtx.state().online);
    TEST_ASSERT_EQUAL_UINT8(21, vtx.state().version);
    TEST_ASSERT_EQUAL_UINT16(5658, vtxChip.freq);   // по умолчанию R1
    TEST_ASSERT_EQUAL_UINT8(14, vtxChip.dbm);       // уровень 1 — 25 мВт
    Serial.pushRx("s");
    run(rc, 0.01);
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(status, "VTX: SmartAudio 2.1, R1, 5658 МГц, 25 мВт (уровень 1 из 6, потолок 4000)"),
                             status.c_str());
}

void test_power_from_switch_capped_for_the_city()
{
    RcChannels rc;
    // ARM — CH5, камера (USER1) — CH8, мощность — трёхпозиционный CH7:
    // вверх 100 мВт, вниз — 4 Вт (но потолок 400)
    Serial.pushRx("#aux 0 0 0 1700 2100 0 0\r\naux 1 40 3 1700 2100 0 0\r\n"
                  "vtx 0 2 0 0 2 900 1300\r\nvtx 1 2 0 0 6 1700 2100\r\n"
                  "set vtx_max_power_mw = 400\r\nsave\r\n");
    run(rc, 0.1);
    std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Сохранено"), log.c_str());

    rc.set(Channels::SWC, 1000);
    run(rc, 2.0);
    TEST_ASSERT_EQUAL_UINT8(20, vtxChip.dbm);   // 100 мВт
    rc.set(Channels::SWC, 2000);
    run(rc, 2.0);
    TEST_ASSERT_EQUAL_UINT8(26, vtxChip.dbm);   // просили 4 Вт — потолок 400 мВт
    TEST_ASSERT_EQUAL_UINT16(400, vtx.state().powerMw);

    // кнопкой на передатчике подняли — прошивка опускает
    vtxChip.dbm = 36;
    run(rc, 2.5);
    TEST_ASSERT_EQUAL_UINT8(26, vtxChip.dbm);

    Serial.pushRx("#vtx_info\r\nexit\r\n");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Мощности: 1=25 мВт 2=100 мВт 3=400 мВт 4=1000 мВт (выше потолка)"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "6=4000 мВт (выше потолка)"));

    // за городом — потолок снят
    Serial.pushRx("#set vtx_max_power_mw = 0\r\nsave\r\n");
    run(rc, 2.5);
    TEST_ASSERT_EQUAL_UINT8(36, vtxChip.dbm);
    takeSerial();
}

void test_user1_switches_pinio_with_board_inversion()
{
    RcChannels rc;
    run(rc, 0.1);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(PC5));   // USER1 выключен, PINIO1 инвертирован — высокий
    rc.set(Channels::SWD, 2000);                 // CH8 — USER1
    run(rc, 0.1);
    TEST_ASSERT_TRUE(rxInput.isActive(copter::BOX_USER1));
    TEST_ASSERT_EQUAL(LOW, digitalRead(PC5));
    rc.set(Channels::SWD, 1000);
    run(rc, 0.1);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(PC5));
    TEST_ASSERT_EQUAL(HIGH, digitalRead(PA14));  // PINIO2 (USER2) — не включали
}

// CRSF-кадры телеметрии из того, что прошивка написала в UART приёмника.
namespace
{
    std::vector<std::vector<uint8_t>> crsfFrames(const std::string& tx)
    {
        std::vector<std::vector<uint8_t>> frames;
        size_t at = 0;
        while (at + 2 < tx.size())
        {
            const uint8_t len = static_cast<uint8_t>(tx[at + 1]);
            if (static_cast<uint8_t>(tx[at]) != 0xC8 || at + 2 + len > tx.size())
            {
                ++at;
                continue;
            }
            const std::vector<uint8_t> f(tx.begin() + static_cast<long>(at), tx.begin() + static_cast<long>(at + 2 + len));
            if (copter::crsfCrc(f.data() + 2, static_cast<uint8_t>(len - 1)) == f.back()) frames.push_back(f);
            at += 2 + len;
        }
        return frames;
    }
}

void test_gps_and_telemetry_to_the_radio()
{
    RcChannels rc;
    run(rc, 1.0);
    Serial.pushRx("s");
    run(rc, 0.01);
    std::string status = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(status, "GPS: 3D, 12 спутн."), status.c_str());
    TEST_ASSERT_TRUE(contains(status, "дом — при ARM"));
    TEST_ASSERT_TRUE(contains(status, "Компас: нет"));
    TEST_ASSERT_TRUE(port(PA3).takeTx().empty());   // iBUS — телеметрии CRSF нет

    // приёмник ELRS: CRSF на том же UART2
    Serial.pushRx("#set serialrx_provider = CRSF\r\nsave\r\n");
    run(rc, 0.05);
    crsfReceiver = true;
    run(rc, 1.0);
    TEST_ASSERT_FALSE(rcReceiver.isSignalLost());
    const auto frames = crsfFrames(port(PA3).takeTx());
    TEST_ASSERT_TRUE(frames.size() >= 15);   // 20 кадров в секунду
    bool batteryFrame = false, gpsFrame = false, mode = false, attitude = false;
    for (const auto& f : frames)
    {
        if (f[2] == copter::CRSF_FRAME_BATTERY && ((f[3] << 8) | f[4]) == 252) batteryFrame = true;   // 25.2 В
        if (f[2] == copter::CRSF_FRAME_GPS && f[17] == 12) gpsFrame = true;
        if (f[2] == copter::CRSF_FRAME_FLIGHT_MODE && std::string(reinterpret_cast<const char*>(&f[3])) == "ACRO*") mode = true;
        if (f[2] == copter::CRSF_FRAME_ATTITUDE) attitude = true;
    }
    TEST_ASSERT_TRUE(batteryFrame);
    TEST_ASSERT_TRUE(gpsFrame);
    TEST_ASSERT_TRUE(mode);
    TEST_ASSERT_TRUE(attitude);

    // выключить — молчит
    Serial.pushRx("#set crsf_telemetry = OFF\r\nsave\r\n");
    run(rc, 0.1);
    port(PA3).takeTx();
    run(rc, 0.5);
    TEST_ASSERT_TRUE(port(PA3).takeTx().empty());
    takeSerial();
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_vtx_found_on_tx4_and_tuned_from_settings);
    RUN_TEST(test_power_from_switch_capped_for_the_city);
    RUN_TEST(test_user1_switches_pinio_with_board_inversion);
    RUN_TEST(test_gps_and_telemetry_to_the_radio);
    return UNITY_END();
}
