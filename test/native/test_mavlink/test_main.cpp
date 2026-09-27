// ============================================================
// MAVLINK: кодек кадров (CRC X.25, v1/v2, подпись, обрезка нулей) и
// телеметрия борта (потоки, параметры ПИД, смена режима с земли,
// события). Эталонные кадры сгенерированы pymavlink 2.4 (common.xml):
// если байты совпали, QGroundControl/Mission Planner поймут борт.
//
// Запуск: pio test -e native -f native/test_mavlink
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>

#include "autopilot/Autopilot.h"
#include "autopilot/PilotSwitches.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"
#include "telemetry/LoopStats.h"
#include "telemetry/MavlinkCodec.h"
#include "telemetry/MavlinkTelemetry.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    using Bytes = std::vector<uint8_t>;

    // --- кадры от pymavlink (GCS: sysid 255, compid 190) ---
    const Bytes GCS_HEARTBEAT = { 0xFD, 0x09, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x06, 0x08, 0x00, 0x00, 0x03, 0x5C, 0x2B };
    const Bytes PARAM_REQUEST_LIST = { 0xFD, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x15, 0x00, 0x00, 0x01, 0x01, 0x88, 0xC0 };
    const Bytes PARAM_REQUEST_READ_PTCH_KD = { 0xFD, 0x0B, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x14, 0x00, 0x00, 0xFF, 0xFF,
                                               0x01, 0x01, 0x50, 0x54, 0x43, 0x48, 0x5F, 0x4B, 0x44, 0x62, 0x46 };
    const Bytes PARAM_SET_RLL_KP_2_5 = { 0xFD, 0x17, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x17, 0x00, 0x00, 0x00, 0x00,
                                         0x20, 0x40, 0x01, 0x01, 0x52, 0x4C, 0x4C, 0x5F, 0x4B, 0x50, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0xF0, 0x59 };
    const Bytes SET_MODE_LOITER = { 0xFD, 0x06, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x0B, 0x00, 0x00, 0x0C, 0x00, 0x00,
                                    0x00, 0x01, 0x01, 0x2A, 0x4B };
    const Bytes CMD_DO_SET_MODE_RTL = { 0xFD, 0x20, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80,
                                        0x3F, 0x00, 0x00, 0x30, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0,
                                        0x00, 0x01, 0x01, 0x89, 0x2D };
    const Bytes CMD_ARM = { 0xFD, 0x20, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x01, 0x01, 0x01, 0x9E, 0x4E };
    const Bytes MISSION_REQUEST_LIST_FENCE = { 0xFD, 0x03, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x2B, 0x00, 0x00, 0x01, 0x01,
                                               0x01, 0x27, 0x1C };
    const Bytes V1_PARAM_REQUEST_LIST = { 0xFE, 0x02, 0x00, 0xFF, 0xBE, 0x15, 0x01, 0x01, 0x79, 0x37 };
    const Bytes SIGNED_SET_MODE_CRUISE = { 0xFD, 0x06, 0x01, 0x00, 0x00, 0xFF, 0xBE, 0x0B, 0x00, 0x00, 0x07, 0x00,
                                           0x00, 0x00, 0x01, 0x01, 0x82, 0x75, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0xF7, 0xF6, 0xA6, 0x81, 0x9E, 0x4D };

    // --- кадры борта (sysid 1, compid 1, seq 0), тоже от pymavlink ---
    const Bytes OUT_HEARTBEAT = { 0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x01, 0x03, 0x41, 0x03, 0x03, 0x3E, 0x5D };
    const Bytes OUT_ATTITUDE = { 0xFD, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x1E, 0x00, 0x00, 0xD2, 0x04, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0xBE, 0x00, 0x00, 0x80, 0x3F, 0x90, 0x7B };

    Bytes encode(Mavlink::Encoder& encoder, uint32_t msgid, const Mavlink::Payload& p)
    {
        uint8_t out[Mavlink::MAX_FRAME];
        const size_t n = encoder.encode(out, msgid, p);
        return Bytes(out, out + n);
    }

    // Разобрать всё, что борт отправил, в список сообщений.
    std::vector<Mavlink::Message> decodeAll(const std::vector<uint8_t>& bytes)
    {
        Mavlink::Parser parser;
        std::vector<Mavlink::Message> out;
        for (uint8_t b : bytes)
        {
            if (parser.feed(b)) out.push_back(parser.message());
        }
        TEST_ASSERT_EQUAL_MESSAGE(0, parser.badCrcCount(), "борт отправил кадр с неверной CRC");
        return out;
    }

    std::map<uint32_t, int> countById(const std::vector<Mavlink::Message>& messages)
    {
        std::map<uint32_t, int> counts;
        for (const auto& m : messages) counts[m.msgid]++;
        return counts;
    }

    const Mavlink::Message* last(const std::vector<Mavlink::Message>& messages, uint32_t id)
    {
        for (auto it = messages.rbegin(); it != messages.rend(); ++it)
        {
            if (it->msgid == id) return &*it;
        }
        return nullptr;
    }

    std::string text(const Mavlink::Message& m)
    {
        char buffer[51];
        m.chars(1, 50, buffer);
        return buffer;
    }

    struct Plane
    {
        FakeBoard board;
        FakeImu imu;
        FakeBaro baro;
        FakeMag mag;
        FakeGps gps;
        FakeAirspeed airspeed;
        FakeUart radio;
        IBusReceiver receiver{ board.rcUart() };
        ControlMixer mixer;
        ThrottleManager throttle;
        FlightOutputs outputs{ board };
        Autopilot autopilot{ &imu, &baro, &mag, &gps, &airspeed };
        PilotSwitches switches{ &autopilot };
        ArmingManager arming{ &autopilot };
        FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &switches };
        LoopStats stats;
        MavlinkTelemetry telemetry{ radio, controller, &autopilot, &stats };
        RcChannels rc;

        Plane()
        {
            outputs.begin();
            controller.begin();
            autopilot.begin();
            telemetry.begin();
            gps.data.latitude = 55.75;
            gps.data.longitude = 37.61;
            gps.data.altitude = 150.0f;
            gps.data.fixType = 3;
            gps.data.numSatellites = 11;
            gps.data.groundSpeed = 14.0f;
            gps.data.heading = 90.0f;
            gps.data.horizontalAccuracy = 1.2f;
            gps.data.verticalAccuracy = 2.0f;
            takeSerial();
        }

        void tick()
        {
            board.rc.push(ibusFrame(rc));
            fake::advanceMs(2);
            controller.update();
            stats.record(400);
            telemetry.update();
        }

        void run(uint32_t ms)
        {
            for (uint32_t t = 0; t < ms; t += 2) tick();
        }

        void arm()
        {
            rc.set(Channels::ARM, 1000);
            run(20);
            rc.set(Channels::ARM, 2000);
            run(20);
        }

        std::vector<Mavlink::Message> take()
        {
            auto out = decodeAll(radio.tx);
            radio.tx.clear();
            return out;
        }
    };
}


// ---------------- кодек ----------------

void test_crc_matches_mavlink_reference()
{
    // crc_calculate("123456789") у MAVLink (CRC-16/MCRF4XX) = 0x6F91.
    const uint8_t data[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX16(0x6F91, Mavlink::crcCalculate(data, sizeof(data)));
}

void test_encoder_matches_pymavlink_heartbeat_and_attitude()
{
    Mavlink::Encoder encoder(1, 1);
    Mavlink::Payload hb;
    hb.u32(0).u8(1).u8(3).u8(65).u8(3).u8(3);
    const Bytes frame = encode(encoder, Mavlink::Msg::HEARTBEAT, hb);
    TEST_ASSERT_EQUAL(OUT_HEARTBEAT.size(), frame.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(OUT_HEARTBEAT.data(), frame.data(), frame.size());

    // Хвостовые нули (угловые скорости = 0) обрезаются, как у pymavlink.
    Mavlink::Encoder encoder2(1, 1);
    Mavlink::Payload att;
    att.u32(1234).f32(0.5f).f32(-0.25f).f32(1.0f).f32(0).f32(0).f32(0);
    const Bytes attitude = encode(encoder2, Mavlink::Msg::ATTITUDE, att);
    TEST_ASSERT_EQUAL(OUT_ATTITUDE.size(), attitude.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(OUT_ATTITUDE.data(), attitude.data(), attitude.size());
}

void test_encoder_sequence_and_unknown_message()
{
    Mavlink::Encoder encoder(7, 9);
    Mavlink::Payload p;
    p.u8(0);   // одни нули — остаётся 1 байт
    const Bytes a = encode(encoder, Mavlink::Msg::PARAM_REQUEST_LIST, p);
    const Bytes b = encode(encoder, Mavlink::Msg::PARAM_REQUEST_LIST, p);
    TEST_ASSERT_EQUAL(1, a[1]);
    TEST_ASSERT_EQUAL(0, a[4]);
    TEST_ASSERT_EQUAL(1, b[4]);
    TEST_ASSERT_EQUAL(7, a[5]);
    TEST_ASSERT_EQUAL(9, a[6]);

    uint8_t out[Mavlink::MAX_FRAME];
    TEST_ASSERT_EQUAL(0, encoder.encode(out, 999999, p));   // CRC_EXTRA неизвестен
    TEST_ASSERT_EQUAL(-1, Mavlink::crcExtraOf(999999));
}

void test_parser_reads_pymavlink_frames()
{
    Mavlink::Parser parser;
    const Bytes* frames[] = { &GCS_HEARTBEAT, &PARAM_SET_RLL_KP_2_5, &V1_PARAM_REQUEST_LIST, &SIGNED_SET_MODE_CRUISE };
    const uint32_t ids[] = { Mavlink::Msg::HEARTBEAT, Mavlink::Msg::PARAM_SET, Mavlink::Msg::PARAM_REQUEST_LIST,
                             Mavlink::Msg::SET_MODE };

    for (int i = 0; i < 4; ++i)
    {
        int completed = 0;
        for (uint8_t b : *frames[i])
        {
            if (parser.feed(b)) completed++;
        }
        TEST_ASSERT_EQUAL_MESSAGE(1, completed, "кадр не разобран");
        TEST_ASSERT_EQUAL(ids[i], parser.message().msgid);
        TEST_ASSERT_EQUAL(255, parser.message().sysid);
        TEST_ASSERT_EQUAL(190, parser.message().compid);
    }
    TEST_ASSERT_EQUAL(0, parser.badCrcCount());

    // Поля PARAM_SET: значение 2.5, имя RLL_KP.
    Mavlink::Parser p2;
    for (uint8_t b : PARAM_SET_RLL_KP_2_5) p2.feed(b);
    TEST_ASSERT_EQUAL_FLOAT(2.5f, p2.message().f32(0));
    char name[17];
    p2.message().chars(6, 16, name);
    TEST_ASSERT_EQUAL_STRING("RLL_KP", name);
}

void test_parser_rejects_corruption_and_resyncs()
{
    Mavlink::Parser parser;
    Bytes broken = SET_MODE_LOITER;
    broken[11] ^= 0x01;   // бит в payload
    int completed = 0;
    for (uint8_t b : broken) completed += parser.feed(b) ? 1 : 0;
    TEST_ASSERT_EQUAL(0, completed);
    TEST_ASSERT_EQUAL(1, parser.badCrcCount());

    // Мусор между кадрами и неизвестное сообщение не мешают.
    Bytes stream = { 0x00, 0x13, 0x37 };
    Mavlink::Encoder encoder(3, 3);
    Mavlink::Payload unknown;
    unknown.u8(1);
    uint8_t out[Mavlink::MAX_FRAME];
    const size_t n = encoder.encode(out, Mavlink::Msg::HEARTBEAT, unknown);
    out[7] = 0x99;   // подменили msgid на неизвестный — CRC не проверить
    stream.insert(stream.end(), out, out + n);
    stream.insert(stream.end(), SET_MODE_LOITER.begin(), SET_MODE_LOITER.end());

    completed = 0;
    for (uint8_t b : stream) completed += parser.feed(b) ? 1 : 0;
    TEST_ASSERT_EQUAL(1, completed);
    TEST_ASSERT_EQUAL(Mavlink::Msg::SET_MODE, parser.message().msgid);
    TEST_ASSERT_EQUAL(12, parser.message().u32(0));
}

void test_mode_mapping_roundtrip()
{
    for (uint8_t m = 0; m < MODE_COUNT; ++m)
    {
        const AutopilotMode mode = static_cast<AutopilotMode>(m);
        const uint32_t custom = MavlinkModes::toCustomMode(mode, false, false);
        AutopilotMode back;
        if (MavlinkModes::fromCustomMode(custom, back))
        {
            // Туда-обратно — тот же режим, кроме общих номеров (LAUNCH -> TAKEOFF).
            if (mode != MODE_LAUNCH) TEST_ASSERT_EQUAL(mode, back);
        }
        else
        {
            TEST_ASSERT_EQUAL(MODE_AUTO_LAND, mode);   // AUTO с земли не включается
        }
    }
    TEST_ASSERT_EQUAL(MavlinkModes::PLANE_RTL, MavlinkModes::toCustomMode(MODE_STABILIZE, true, false));
    TEST_ASSERT_EQUAL(MavlinkModes::PLANE_CIRCLE, MavlinkModes::toCustomMode(MODE_STABILIZE, false, true));
    AutopilotMode unused;
    TEST_ASSERT_FALSE(MavlinkModes::fromCustomMode(15, unused));   // GUIDED
}


// ---------------- телеметрия борта ----------------

void test_streams_have_expected_rates()
{
    Plane plane;
    plane.arm();
    plane.run(10000);

    // Поток целиком — для сверки внешним декодером:
    //   OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
    //   python3 tools/check_mavlink.py /tmp/tlm.bin
    if (const char* dump = getenv("OPENPLANE_MAVLINK_DUMP"))
    {
        if (FILE* f = fopen(dump, "wb"))
        {
            fwrite(plane.radio.tx.data(), 1, plane.radio.tx.size(), f);
            fclose(f);
        }
    }
    const auto counts = countById(plane.take());

    auto near = [&](uint32_t id, int expected) {
        const auto it = counts.find(id);
        const int got = it == counts.end() ? 0 : it->second;
        TEST_ASSERT_INT_WITHIN_MESSAGE(expected / 10 + 1, expected, got, "частота потока");
    };
    near(Mavlink::Msg::HEARTBEAT, 10);
    near(Mavlink::Msg::ATTITUDE, 100);
    near(Mavlink::Msg::GLOBAL_POSITION_INT, 50);
    near(Mavlink::Msg::VFR_HUD, 50);
    near(Mavlink::Msg::SYS_STATUS, 10);
    near(Mavlink::Msg::GPS_RAW_INT, 20);
    near(Mavlink::Msg::RC_CHANNELS, 20);
    near(Mavlink::Msg::SERVO_OUTPUT_RAW, 20);
    near(Mavlink::Msg::NAV_CONTROLLER_OUTPUT, 20);
    TEST_ASSERT_EQUAL(2, counts.at(Mavlink::Msg::STATUSTEXT));   // "OpenPlane online", "ARMED"
    near(Mavlink::Msg::HOME_POSITION, 2);

    // Байт в секунду — с запасом для SiK на 57600 (~5.7 КБ/с).
    TEST_ASSERT_TRUE(plane.telemetry.getSentFrames() > 300);
}

void test_heartbeat_reports_mode_and_arming()
{
    Plane plane;
    plane.run(1100);
    auto msgs = plane.take();
    const Mavlink::Message* hb = last(msgs, Mavlink::Msg::HEARTBEAT);
    TEST_ASSERT_NOT_NULL(hb);
    TEST_ASSERT_EQUAL(MavlinkModes::PLANE_MANUAL, hb->u32(0));
    TEST_ASSERT_EQUAL(1, hb->u8(4));                 // FIXED_WING
    TEST_ASSERT_EQUAL(3, hb->u8(5));                 // ARDUPILOTMEGA
    TEST_ASSERT_EQUAL(0, hb->u8(6) & 128);           // не заармлен
    TEST_ASSERT_EQUAL(3, hb->u8(7));                 // STANDBY

    plane.rc.set(Channels::SWC, 1500);               // STABILIZE по умолчанию таблицы
    plane.arm();
    plane.run(1100);
    msgs = plane.take();
    hb = last(msgs, Mavlink::Msg::HEARTBEAT);
    TEST_ASSERT_NOT_NULL(hb);
    TEST_ASSERT_EQUAL(128, hb->u8(6) & 128);
    TEST_ASSERT_EQUAL(4, hb->u8(7));                 // ACTIVE
    TEST_ASSERT_EQUAL(MavlinkModes::toCustomMode(plane.autopilot.getMode(), false, false), hb->u32(0));

    bool armedText = false;
    for (const auto& m : msgs)
    {
        if (m.msgid == Mavlink::Msg::STATUSTEXT && text(m) == "ARMED") armedText = true;
    }
    TEST_ASSERT_TRUE(armedText);
}

void test_attitude_position_and_hud_values()
{
    Plane plane;
    plane.imu.data.roll = 30.0f;
    plane.imu.data.pitch = -10.0f;
    plane.imu.data.yaw = 45.0f;
    plane.imu.data.gyroX = 90.0f;
    plane.baro.data.altitude = 42.0f;
    plane.baro.data.verticalSpeed = 1.5f;
    plane.mag.data.headingDegrees = 91.0f;
    plane.airspeed.setSpeed(16.0f);
    plane.run(1100);
    const auto msgs = plane.take();

    const Mavlink::Message* att = last(msgs, Mavlink::Msg::ATTITUDE);
    TEST_ASSERT_NOT_NULL(att);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5236f, att->f32(4));    // 30° в радианах
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.1745f, att->f32(8));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.5708f, att->f32(16));   // 90 °/с

    const Mavlink::Message* pos = last(msgs, Mavlink::Msg::GLOBAL_POSITION_INT);
    TEST_ASSERT_NOT_NULL(pos);
    TEST_ASSERT_EQUAL_INT32(557500000, static_cast<int32_t>(pos->u32(4)));
    TEST_ASSERT_EQUAL_INT32(376100000, static_cast<int32_t>(pos->u32(8)));
    TEST_ASSERT_EQUAL_INT32(150000, static_cast<int32_t>(pos->u32(12)));
    TEST_ASSERT_EQUAL_INT32(42000, static_cast<int32_t>(pos->u32(16)));
    TEST_ASSERT_INT_WITHIN(1, 0, pos->i16(20));       // на восток: vx ≈ 0
    TEST_ASSERT_EQUAL(1400, pos->i16(22));            // vy = 14 м/с
    TEST_ASSERT_EQUAL(-150, pos->i16(24));            // вверх = отрицательный vz
    TEST_ASSERT_EQUAL(9100, pos->u16(26));            // курс по компасу

    const Mavlink::Message* hud = last(msgs, Mavlink::Msg::VFR_HUD);
    TEST_ASSERT_NOT_NULL(hud);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 16.0f, hud->f32(0));   // по трубке Пито
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 14.0f, hud->f32(4));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.0f, hud->f32(8));
    TEST_ASSERT_EQUAL(91, hud->i16(16));

    const Mavlink::Message* raw = last(msgs, Mavlink::Msg::GPS_RAW_INT);
    TEST_ASSERT_NOT_NULL(raw);
    TEST_ASSERT_EQUAL(3, raw->u8(28));
    TEST_ASSERT_EQUAL(11, raw->u8(29));
    TEST_ASSERT_EQUAL(1200, raw->u32(34));            // h_acc, мм

    const Mavlink::Message* status = last(msgs, Mavlink::Msg::SYS_STATUS);
    TEST_ASSERT_NOT_NULL(status);
    const uint32_t present = status->u32(0), health = status->u32(8);
    TEST_ASSERT_EQUAL_HEX32(0x3F, present & 0x3F);    // все датчики в сборке
    TEST_ASSERT_EQUAL_HEX32(0x3F, health & 0x3F);
    TEST_ASSERT_EQUAL(200, status->u16(12));          // 400 мкс из 2 мс = 20%
}

void test_lost_link_and_sensor_faults_are_visible()
{
    Plane plane;
    plane.run(600);
    plane.take();

    plane.gps.available = false;
    plane.airspeed.available = false;
    // Пульт замолчал: failsafe через 500 мс.
    for (int i = 0; i < 1000; ++i)
    {
        fake::advanceMs(2);
        plane.controller.update();
        plane.telemetry.update();
    }
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    const auto msgs = plane.take();
    const Mavlink::Message* status = last(msgs, Mavlink::Msg::SYS_STATUS);
    TEST_ASSERT_NOT_NULL(status);
    TEST_ASSERT_EQUAL(0, status->u32(8) & 0x10000);    // RC_RECEIVER нездоров
    TEST_ASSERT_EQUAL(0, status->u32(8) & 0x20);       // GPS
    TEST_ASSERT_EQUAL(0, status->u32(8) & 0x10);       // трубка Пито

    const Mavlink::Message* hb = last(msgs, Mavlink::Msg::HEARTBEAT);
    TEST_ASSERT_EQUAL(5, hb->u8(7));                   // CRITICAL

    bool failsafeText = false;
    for (const auto& m : msgs)
    {
        if (m.msgid == Mavlink::Msg::STATUSTEXT && text(m).find("FAILSAFE") == 0) failsafeText = true;
    }
    TEST_ASSERT_TRUE(failsafeText);
    TEST_ASSERT_NULL(last(msgs, Mavlink::Msg::GLOBAL_POSITION_INT));   // без GPS не шлём
}

void test_parameters_list_read_and_set()
{
    Plane plane;
    plane.run(100);
    plane.take();

    plane.radio.push(PARAM_REQUEST_LIST);
    plane.run(100);
    auto msgs = plane.take();
    std::vector<std::string> names;
    for (const auto& m : msgs)
    {
        if (m.msgid != Mavlink::Msg::PARAM_VALUE) continue;
        char name[17];
        m.chars(8, 16, name);
        names.push_back(name);
        TEST_ASSERT_EQUAL(MavlinkTelemetry::PARAM_COUNT, m.u16(4));
        TEST_ASSERT_EQUAL(9, m.u8(24));   // REAL32
    }
    TEST_ASSERT_EQUAL(6, names.size());
    TEST_ASSERT_EQUAL_STRING("RLL_KP", names[0].c_str());
    TEST_ASSERT_EQUAL_STRING("PTCH_KD", names[5].c_str());

    // Чтение по имени.
    plane.radio.push(PARAM_REQUEST_READ_PTCH_KD);
    plane.run(20);
    msgs = plane.take();
    const Mavlink::Message* value = last(msgs, Mavlink::Msg::PARAM_VALUE);
    TEST_ASSERT_NOT_NULL(value);
    TEST_ASSERT_EQUAL(5, value->u16(6));
    TEST_ASSERT_EQUAL_FLOAT(plane.autopilot.getPitchPid().getKd(), value->f32(0));

    // Чтение по номеру; номер вне списка — без ответа.
    Mavlink::Encoder gcs(255, 190);
    for (int16_t index : { 2, 9 })
    {
        Mavlink::Payload p;
        p.i16(index).u8(1).u8(1).chars("", 16);
        uint8_t out[Mavlink::MAX_FRAME];
        const size_t n = gcs.encode(out, Mavlink::Msg::PARAM_REQUEST_READ, p);
        plane.radio.push(Bytes(out, out + n));
        plane.run(20);
        msgs = plane.take();
        value = last(msgs, Mavlink::Msg::PARAM_VALUE);
        if (index == 2)
        {
            TEST_ASSERT_NOT_NULL(value);
            TEST_ASSERT_EQUAL(2, value->u16(6));
            TEST_ASSERT_EQUAL_FLOAT(plane.autopilot.getRollPid().getKd(), value->f32(0));
        }
        else
        {
            TEST_ASSERT_NULL(value);
        }
    }

    // Запись: ПИД меняется сразу, ответ — новое значение.
    const float kiRollBefore = plane.autopilot.getRollPid().getKi();
    plane.radio.push(PARAM_SET_RLL_KP_2_5);
    plane.run(20);
    msgs = plane.take();
    TEST_ASSERT_EQUAL_FLOAT(2.5f, plane.autopilot.getRollPid().getKp());
    TEST_ASSERT_EQUAL_FLOAT(kiRollBefore, plane.autopilot.getRollPid().getKi());
    value = last(msgs, Mavlink::Msg::PARAM_VALUE);
    TEST_ASSERT_NOT_NULL(value);
    TEST_ASSERT_EQUAL_FLOAT(2.5f, value->f32(0));
}

void test_bad_parameter_values_are_rejected()
{
    Plane plane;
    const float before = plane.autopilot.getRollPid().getKp();

    Mavlink::Encoder gcs(255, 190);
    const float bad[] = { -1.0f, 1000.0f, NAN };
    for (float v : bad)
    {
        Mavlink::Payload p;
        p.f32(v).u8(1).u8(1).chars("RLL_KP", 16).u8(9);
        uint8_t out[Mavlink::MAX_FRAME];
        const size_t n = gcs.encode(out, Mavlink::Msg::PARAM_SET, p);
        plane.radio.push(Bytes(out, out + n));
        plane.run(20);
    }
    TEST_ASSERT_EQUAL_FLOAT(before, plane.autopilot.getRollPid().getKp());

    // Чужой борт (target_system 2) — не наш запрос.
    Mavlink::Payload p;
    p.f32(3.0f).u8(2).u8(1).chars("RLL_KP", 16).u8(9);
    uint8_t out[Mavlink::MAX_FRAME];
    const size_t n = gcs.encode(out, Mavlink::Msg::PARAM_SET, p);
    plane.radio.push(Bytes(out, out + n));
    plane.run(20);
    TEST_ASSERT_EQUAL_FLOAT(before, plane.autopilot.getRollPid().getKp());
}

void test_mode_change_from_gcs_until_switch_moves()
{
    Plane plane;
    plane.rc.set(Channels::SWC, 1000);
    plane.run(100);
    const AutopilotMode switchMode = plane.autopilot.getMode();

    plane.radio.push(SET_MODE_LOITER);
    plane.run(20);
    TEST_ASSERT_EQUAL(MODE_LOITER, plane.autopilot.getMode());

    plane.radio.push(CMD_DO_SET_MODE_RTL);
    plane.run(20);
    TEST_ASSERT_EQUAL(MODE_RTH, plane.autopilot.getMode());
    auto msgs = plane.take();
    const Mavlink::Message* ack = last(msgs, Mavlink::Msg::COMMAND_ACK);
    TEST_ASSERT_NOT_NULL(ack);
    TEST_ASSERT_EQUAL(176, ack->u16(0));
    TEST_ASSERT_EQUAL(0, ack->u8(2));             // ACCEPTED
    TEST_ASSERT_EQUAL(255, ack->u8(8));           // адресовано GCS

    // Пилот щёлкнул тумблером режима — пульт главнее.
    plane.rc.set(Channels::SWC, 2000);
    plane.run(100);
    TEST_ASSERT_NOT_EQUAL(MODE_RTH, plane.autopilot.getMode());
    (void)switchMode;
}

void test_arm_from_gcs_is_denied()
{
    Plane plane;
    plane.radio.push(CMD_ARM);
    plane.run(40);
    TEST_ASSERT_FALSE(plane.controller.isArmed());
    const auto msgs = plane.take();
    const Mavlink::Message* ack = last(msgs, Mavlink::Msg::COMMAND_ACK);
    TEST_ASSERT_NOT_NULL(ack);
    TEST_ASSERT_EQUAL(400, ack->u16(0));
    TEST_ASSERT_EQUAL(2, ack->u8(2));             // DENIED
}

void test_unknown_command_and_request_message()
{
    Plane plane;
    Mavlink::Encoder gcs(255, 190);
    auto command = [&](uint16_t id, float p1) {
        Mavlink::Payload p;
        p.f32(p1).f32(0).f32(0).f32(0).f32(0).f32(0).f32(0).u16(id).u8(1).u8(1).u8(0);
        uint8_t out[Mavlink::MAX_FRAME];
        const size_t n = gcs.encode(out, Mavlink::Msg::COMMAND_LONG, p);
        plane.radio.push(Bytes(out, out + n));
        plane.run(20);
        const auto msgs = plane.take();
        const Mavlink::Message* ack = last(msgs, Mavlink::Msg::COMMAND_ACK);
        TEST_ASSERT_NOT_NULL(ack);
        TEST_ASSERT_EQUAL(id, ack->u16(0));
        return ack->u8(2);
    };
    TEST_ASSERT_EQUAL(3, command(31000, 0));                                  // UNSUPPORTED
    TEST_ASSERT_EQUAL(0, command(512, static_cast<float>(Mavlink::Msg::HEARTBEAT)));
    TEST_ASSERT_EQUAL(3, command(512, 148.0f));                               // AUTOPILOT_VERSION — нет
}

void test_mission_requests_get_empty_answer()
{
    Plane plane;
    plane.radio.push(MISSION_REQUEST_LIST_FENCE);
    plane.run(20);
    const auto msgs = plane.take();
    const Mavlink::Message* count = last(msgs, Mavlink::Msg::MISSION_COUNT);
    TEST_ASSERT_NOT_NULL(count);
    TEST_ASSERT_EQUAL(0, count->u16(0));
    TEST_ASSERT_EQUAL(255, count->u8(2));
    TEST_ASSERT_EQUAL(1, count->u8(4));           // тот же mission_type (забор)
}

void test_gcs_heartbeat_marks_connection()
{
    Plane plane;
    TEST_ASSERT_FALSE(plane.telemetry.isGcsConnected());
    plane.radio.push(GCS_HEARTBEAT);
    plane.run(10);
    TEST_ASSERT_TRUE(plane.telemetry.isGcsConnected());
    plane.run(3200);
    TEST_ASSERT_FALSE(plane.telemetry.isGcsConnected());
}

void test_full_tx_buffer_defers_instead_of_blocking()
{
    Plane plane;
    plane.radio.room = 10;   // меньше любого кадра
    plane.run(1000);
    TEST_ASSERT_EQUAL(0, plane.radio.tx.size());
    TEST_ASSERT_TRUE(plane.telemetry.getDeferredFrames() > 0);

    plane.radio.room = 4096;
    plane.run(1000);
    const auto msgs = plane.take();
    TEST_ASSERT_TRUE(msgs.size() > 10);
    // Отложенное сообщение "online" не потерялось.
    TEST_ASSERT_NOT_NULL(last(msgs, Mavlink::Msg::STATUSTEXT));
}

void test_home_position_after_arming_with_gps()
{
    Plane plane;
    plane.arm();
    plane.run(6000);
    const auto msgs = plane.take();
    const Mavlink::Message* home = last(msgs, Mavlink::Msg::HOME_POSITION);
    TEST_ASSERT_NOT_NULL(home);
    TEST_ASSERT_EQUAL_INT32(557500000, static_cast<int32_t>(home->u32(0)));
    TEST_ASSERT_EQUAL_INT32(376100000, static_cast<int32_t>(home->u32(4)));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, home->f32(24));   // q[0]
}

void test_status_text_queue_limit()
{
    Plane plane;
    plane.radio.room = 0;
    for (int i = 0; i < 10; ++i) plane.telemetry.statusText(MavlinkTelemetry::SEVERITY_INFO, "0123456789012345678901234567890123456789012345678901234567890");
    plane.radio.room = -1;
    plane.run(200);
    const auto msgs = plane.take();
    int texts = 0;
    for (const auto& m : msgs)
    {
        if (m.msgid != Mavlink::Msg::STATUSTEXT) continue;
        texts++;
        if (texts > 1) TEST_ASSERT_EQUAL(50, text(m).size());   // обрезано до 50 символов
    }
    TEST_ASSERT_EQUAL(4, texts);   // "online" + 3 из 10 — очередь на 4
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_matches_mavlink_reference);
    RUN_TEST(test_encoder_matches_pymavlink_heartbeat_and_attitude);
    RUN_TEST(test_encoder_sequence_and_unknown_message);
    RUN_TEST(test_parser_reads_pymavlink_frames);
    RUN_TEST(test_parser_rejects_corruption_and_resyncs);
    RUN_TEST(test_mode_mapping_roundtrip);
    RUN_TEST(test_streams_have_expected_rates);
    RUN_TEST(test_heartbeat_reports_mode_and_arming);
    RUN_TEST(test_attitude_position_and_hud_values);
    RUN_TEST(test_lost_link_and_sensor_faults_are_visible);
    RUN_TEST(test_parameters_list_read_and_set);
    RUN_TEST(test_bad_parameter_values_are_rejected);
    RUN_TEST(test_mode_change_from_gcs_until_switch_moves);
    RUN_TEST(test_arm_from_gcs_is_denied);
    RUN_TEST(test_unknown_command_and_request_message);
    RUN_TEST(test_mission_requests_get_empty_answer);
    RUN_TEST(test_gcs_heartbeat_marks_connection);
    RUN_TEST(test_full_tx_buffer_defers_instead_of_blocking);
    RUN_TEST(test_home_position_after_arming_with_gps);
    RUN_TEST(test_status_text_queue_limit);
    return UNITY_END();
}
