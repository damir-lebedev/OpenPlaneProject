// Камера OV5640: настройка по SCCB (фейк регистров), кадр — строками
// "@cam" для приложения и записями REC_CAM для чёрного ящика.
// pio test -e native -f native/test_camera

#include <unity.h>

#include <map>
#include <string>
#include <vector>

#include "sensors/camera/Ov5640.h"
#include "telemetry/CameraStream.h"

void setUp() {}
void tearDown() {}

namespace
{
    // Камера на SCCB: регистры, ID, счёт записей; может молчать.
    struct FakeSccb : public ISccb
    {
        std::map<uint16_t, uint8_t> regs;
        std::vector<uint16_t> written;
        bool present = true;
        uint16_t failAt = 0;
        uint32_t slept = 0;

        FakeSccb()
        {
            regs[0x300A] = 0x56;
            regs[0x300B] = 0x40;
        }
        bool write(uint16_t reg, uint8_t value) override
        {
            if (!present || reg == failAt) return false;
            regs[reg] = value;
            written.push_back(reg);
            return true;
        }
        bool read(uint16_t reg, uint8_t& value) override
        {
            if (!present) return false;
            value = regs.count(reg) ? regs[reg] : 0;
            return true;
        }
        void delayMs(uint32_t ms) override { slept += ms; }
    };

    std::vector<uint8_t> unbase64(const std::string& s)
    {
        auto v = [](char c) -> int {
            if (c >= 'A' && c <= 'Z') return c - 'A';
            if (c >= 'a' && c <= 'z') return c - 'a' + 26;
            if (c >= '0' && c <= '9') return c - '0' + 52;
            return c == '+' ? 62 : c == '/' ? 63 : -1;
        };
        std::vector<uint8_t> out;
        uint32_t acc = 0;
        int bits = 0;
        for (char c : s)
        {
            const int x = v(c);
            if (x < 0) continue;
            acc = acc << 6 | static_cast<uint32_t>(x);
            bits += 6;
            if (bits >= 8)
            {
                bits -= 8;
                out.push_back(static_cast<uint8_t>(acc >> bits));
            }
        }
        return out;
    }

    std::string field(const std::string& line, const char* name)
    {
        const std::string key = std::string("\"") + name + "\":";
        const size_t at = line.find(key);
        if (at == std::string::npos) return "";
        size_t start = at + key.size();
        if (line[start] == '"')
        {
            ++start;
            return line.substr(start, line.find('"', start) - start);
        }
        return line.substr(start, line.find_first_of(",}", start) - start);
    }
}

void test_ov5640_probe_and_jpeg_table()
{
    FakeSccb bus;
    TEST_ASSERT_EQUAL_HEX16(0x5640, Ov5640::probe(bus));
    Ov5640 cam;
    CameraSettings s;
    s.width = 643;   // кратно 8 — вниз
    s.height = 480;
    s.quality = 1;   // меньше 2 не бывает
    TEST_ASSERT_TRUE_MESSAGE(cam.begin(bus, s), cam.error());
    TEST_ASSERT_NULL(cam.error());
    TEST_ASSERT_EQUAL_UINT16(640, cam.frameWidth());
    TEST_ASSERT_EQUAL_UINT16(480, cam.frameHeight());
    // сброс — первым, пробуждение — последним; JPEG включён
    TEST_ASSERT_EQUAL_HEX16(0x3103, bus.written.front());
    TEST_ASSERT_EQUAL_HEX8(0x02, bus.regs[0x3008]);
    TEST_ASSERT_EQUAL_HEX8(0x30, bus.regs[0x4300]);
    TEST_ASSERT_EQUAL_HEX8(0x03, bus.regs[0x4713]);
    TEST_ASSERT_TRUE(bus.regs[0x3821] & 0x20);
    TEST_ASSERT_EQUAL_HEX8(0x02, bus.regs[0x3808]);
    TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[0x3809]);
    TEST_ASSERT_EQUAL_HEX8(0x01, bus.regs[0x380A]);
    TEST_ASSERT_EQUAL_HEX8(0xE0, bus.regs[0x380B]);
    TEST_ASSERT_EQUAL_HEX8(2, bus.regs[0x4407]);
    TEST_ASSERT_TRUE(bus.slept >= 60);   // сброс и пробуждение — с паузами
    // переворот и качество — на ходу
    s.flip = true;
    s.quality = 30;
    TEST_ASSERT_TRUE(cam.setFrame(bus, s));
    TEST_ASSERT_EQUAL_HEX8(0x47, bus.regs[0x3820]);
    TEST_ASSERT_EQUAL_HEX8(30, bus.regs[0x4407]);
}

void test_ov5640_errors()
{
    FakeSccb silent;
    silent.present = false;
    Ov5640 cam;
    TEST_ASSERT_FALSE(cam.begin(silent, CameraSettings()));
    TEST_ASSERT_EQUAL_UINT16(0, cam.chipId());
    TEST_ASSERT_EQUAL_STRING("нет ответа по SCCB (0x3C)", cam.error());

    FakeSccb other;
    other.regs[0x300A] = 0x26;   // OV2640 и т. п.
    other.regs[0x300B] = 0x42;
    TEST_ASSERT_FALSE(cam.begin(other, CameraSettings()));
    TEST_ASSERT_EQUAL_STRING("это не OV5640", cam.error());

    FakeSccb broken;
    broken.failAt = 0x4300;
    TEST_ASSERT_FALSE(cam.begin(broken, CameraSettings()));
    TEST_ASSERT_EQUAL_HEX16(0x4300, cam.failedRegister());
}

// Кадр для приложения: строки "@cam" по CHUNK байт в base64, склеиваются обратно.
void test_camera_stream_lines_roundtrip()
{
    std::vector<uint8_t> jpeg(1000);
    for (size_t i = 0; i < jpeg.size(); ++i) jpeg[i] = static_cast<uint8_t>(i * 37 + 5);
    jpeg[0] = 0xFF;
    jpeg[1] = 0xD8;
    CameraStream cs;
    char line[CameraStream::LINE_CAPACITY];
    TEST_ASSERT_EQUAL(0, cs.nextLine(line, sizeof(line)));   // кадра нет — молчим
    cs.request();
    TEST_ASSERT_TRUE(cs.snapshotWanted());
    cs.frameReady(jpeg.data(), jpeg.size(), 7, 640, 480, true);
    TEST_ASSERT_FALSE(cs.snapshotWanted());
    TEST_ASSERT_TRUE(cs.busy());
    std::vector<uint8_t> got;
    int lines = 0;
    for (size_t n; (n = cs.nextLine(line, sizeof(line))) > 0; ++lines)
    {
        const std::string l(line, n);
        TEST_ASSERT_TRUE(l.rfind("@cam {", 0) == 0);
        TEST_ASSERT_EQUAL_CHAR('\n', l.back());
        TEST_ASSERT_TRUE(n < CameraStream::LINE_CAPACITY);
        TEST_ASSERT_EQUAL_STRING("7", field(l, "n").c_str());
        TEST_ASSERT_EQUAL_STRING("1000", field(l, "len").c_str());
        TEST_ASSERT_EQUAL(got.size(), std::stoul(field(l, "off")));
        const std::vector<uint8_t> part = unbase64(field(l, "d"));
        got.insert(got.end(), part.begin(), part.end());
        cs.lineSent();
    }
    TEST_ASSERT_EQUAL(3, lines);   // 384 + 384 + 232
    TEST_ASSERT_EQUAL(jpeg.size(), got.size());
    TEST_ASSERT_EQUAL_MEMORY(jpeg.data(), got.data(), jpeg.size());
    TEST_ASSERT_FALSE(cs.busy());

    // кадр для ящика — не отправляется; ошибка снимка — строкой
    cs.frameReady(jpeg.data(), jpeg.size(), 8, 640, 480, false);
    TEST_ASSERT_FALSE(cs.busy());
    TEST_ASSERT_EQUAL_UINT32(2, cs.frames);
    cs.request();
    cs.snapshotFailed("кадр не пришёл");
    const size_t n = cs.errorLine(line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING("@cam {\"error\":\"кадр не пришёл\"}\n", std::string(line, n).c_str());
    TEST_ASSERT_EQUAL(0, cs.errorLine(line, sizeof(line)));
    TEST_ASSERT_EQUAL_STRING("none", cs.stateName());
    cs.state = CameraStream::State::OK;
    TEST_ASSERT_EQUAL_STRING("ok", cs.stateName());
}

// Запись REC_CAM: заголовок, кусок кадра, длина не больше записи ящика.
void test_camera_blackbox_record()
{
    uint8_t data[400];
    for (size_t i = 0; i < sizeof(data); ++i) data[i] = static_cast<uint8_t>(i);
    uint8_t rec[BlackBoxFormat::MAX_RECORD];
    const size_t len = CameraFormat::record(rec, 123456, 9, 241, 5000, data, sizeof(data));
    TEST_ASSERT_EQUAL(BlackBoxFormat::MAX_RECORD, len);   // кусок обрезан до CAM_DATA
    TEST_ASSERT_EQUAL_HEX8(BlackBoxFormat::REC_CAM, rec[0]);
    TEST_ASSERT_EQUAL(BlackBoxFormat::MAX_PAYLOAD, rec[1]);
    uint32_t t, off, size;
    uint16_t frame;
    memcpy(&t, rec + 2, 4);
    memcpy(&frame, rec + 6, 2);
    memcpy(&off, rec + 8, 4);
    memcpy(&size, rec + 12, 4);
    TEST_ASSERT_EQUAL_UINT32(123456, t);
    TEST_ASSERT_EQUAL_UINT16(9, frame);
    TEST_ASSERT_EQUAL_UINT32(241, off);
    TEST_ASSERT_EQUAL_UINT32(5000, size);
    TEST_ASSERT_EQUAL_MEMORY(data, rec + 16, CameraFormat::CAM_DATA);
    // схема в журнале — с полем байтов
    const BlackBoxFormat::Schema* s = BlackBoxFormat::findSchema(BlackBoxFormat::REC_CAM);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_EQUAL_STRING("t_us:I frame:H off:I size:I data:y", s->fields);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_ov5640_probe_and_jpeg_table);
    RUN_TEST(test_ov5640_errors);
    RUN_TEST(test_camera_stream_lines_roundtrip);
    RUN_TEST(test_camera_blackbox_record);
    return UNITY_END();
}
