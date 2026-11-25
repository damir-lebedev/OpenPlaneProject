// ============================================================
// Чёрный ящик коптера на полёте в симуляторе: настоящий
// CopterController + модель 5" (helpers/QuadHarness.h) и настоящий
// CopterBlackBox (telemetry/CopterBlackBox.h) на кольце секторов в ОЗУ.
// Полёт: взлёт в ALT_HOLD, наклоны в ANGLE, флипы и пируэт в ACRO, рывок
// газом, посадка, DISARM. Проверяется, что в логе то, что было на самом
// деле: число записей по частотам, флип виден в гироскопе и уставке,
// высота, события, заголовок.
//
// Запуск: pio test -e native -f native/test_copter_blackbox
// Образ полёта (для OpenPlane Studio и tools/blackbox.py):
//   OPENPLANE_COPTER_BLACKBOX_DUMP=flight.bbl pio test -e native -f native/test_copter_blackbox
// Тот же полёт в формате Betaflight (Blackbox Explorer, PIDtoolbox):
//   OPENPLANE_COPTER_BFL_DUMP=flight.bfl pio test -e native -f native/test_copter_blackbox
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "copter/Params.h"
#include "hal/IFlashRegion.h"
#include "hal/SpiNorFlash.h"
#include "helpers/SpiNorFlashChip.h"
#include "hal/SdFileRegion.h"
#include "hal/SpiSdCard.h"
#include "helpers/FatImage.h"
#include "helpers/SpiSdCardChip.h"
#include "helpers/BflParse.h"
#include "helpers/BlackBoxParse.h"
#include "helpers/ChirpFlight.h"
#include "helpers/QuadHarness.h"
#include "helpers/TestSupport.h"
#include "rc/RcReceiver.h"
#include "telemetry/BflEncoder.h"
#include "telemetry/BlackBoxRing.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/CopterBlackBox.h"
#include "telemetry/LoopStats.h"

using namespace bbparse;
using namespace copter;

void setUp() {}
void tearDown() {}

namespace
{
    // Кольцо в ОЗУ: стирание — 0xFF, запись — как есть.
    class MemoryRegion : public IFlashRegion
    {
    public:
        explicit MemoryRegion(uint32_t bytes) : mem(bytes, 0xFF) {}
        uint32_t size() const override { return static_cast<uint32_t>(mem.size()); }
        bool read(uint32_t offset, void* data, size_t length) override
        {
            if (offset + length > mem.size()) return false;
            memcpy(data, mem.data() + offset, length);
            return true;
        }
        bool write(uint32_t offset, const void* data, size_t length) override
        {
            if (offset + length > mem.size()) return false;
            memcpy(mem.data() + offset, data, length);
            return true;
        }
        bool erase(uint32_t offset, uint32_t length) override
        {
            if (offset + length > mem.size()) return false;
            memset(mem.data() + offset, 0xFF, length);
            return true;
        }
        uint32_t writeUnit() const override { return 512; }   // как SD-карта

    private:
        std::vector<uint8_t> mem;
    };

    const BuildSpec& five() { return *findBuild("freestyle5"); }

    // Стенд: модель, контроллер, приёмник (iBUS из стиков стенда) и ящик.
    struct Flight
    {
        QuadHarness h{ five(), QuadPerturb(), 21 };
        CopterParams params;
        FakeUart uart;
        RcReceiver rx{ uart };
        LoopStats loop;
        MemoryRegion region{ 16u * 1024 * 1024 };
        BlackBoxStorage storage{ region };
        CopterBlackBox box{ h.ctl, params, storage, loop, rx };
        uint32_t lastFrameMs = 0;

        Flight()
        {
            params.begin(five(), 1000.0f, EscProtocol::DSHOT600);
            rx.begin();
            TEST_ASSERT_TRUE(box.begin(false));
            box.configure(true, CopterBlackBox::Mode::NORMAL, 1);
            h.afterStep = [this] { afterStep(); };
        }

        static uint16_t us(float stick) { return static_cast<uint16_t>(lroundf(1500.0f + 500.0f * stick)); }

        void afterStep()
        {
            fake::advanceUs(1000);
            if (millis() - lastFrameMs >= 7)
            {
                lastFrameMs = millis();
                RcChannels c;
                c.value[Channels::AILERON] = us(h.rc.roll);
                c.value[Channels::ELEVATOR] = us(h.rc.pitch);
                c.value[Channels::THROTTLE] = static_cast<uint16_t>(lroundf(1000.0f + 1000.0f * h.rc.throttle));
                c.value[Channels::RUDDER] = us(h.rc.yaw);
                c.value[Channels::ARM] = h.rc.armSwitch ? 2000 : 1000;
                const std::string frame = ibusFrame(c);
                for (char b : frame) uart.rx.push_back(static_cast<uint8_t>(b));
            }
            rx.update();
            loop.record(250);
            box.update(h.lastSens, h.rc, h.out, 250 + (h.steps % 7) * 10);
            box.writerStep();
        }

        // Плавно довести стики до значений за seconds (пилот не дёргает стик за кадр).
        void ease(float roll, float pitch, float yaw, double seconds)
        {
            const float r0 = h.rc.roll, p0 = h.rc.pitch, y0 = h.rc.yaw;
            const int n = static_cast<int>(seconds * 1000);
            for (int i = 1; i <= n; ++i)
            {
                const float k = static_cast<float>(i) / n;
                h.rc.roll = r0 + (roll - r0) * k;
                h.rc.pitch = p0 + (pitch - p0) * k;
                h.rc.yaw = y0 + (yaw - y0) * k;
                h.step();
            }
        }

        // Флип, как у пилота: ACRO, стик в упор, пока коптер не повернётся на
        // ~330°, отпустить; дальше выравнивает ANGLE.
        void flip(int axis)
        {
            h.rc.mode = CopterMode::ACRO;
            h.rc.throttle = h.ctl.getStatus().hoverLearned * 1.5f;
            h.run(0.15);
            double turned = 0.0;
            float& stick = axis == 0 ? h.rc.roll : h.rc.pitch;
            for (int i = 0; i < 2000 && turned < 330.0; ++i)
            {
                stick = std::fmin(1.0f, stick + 0.025f);   // 40 мс до упора
                h.step();
                const auto w = h.sim.rates();
                turned += std::fabs(axis == 0 ? w.x : w.y) * 57.29578 / 1000.0;
            }
            ease(0.0f, 0.0f, 0.0f, 0.04);
            h.rc.mode = CopterMode::ANGLE;
            h.rc.throttle = h.ctl.getStatus().hoverLearned;
            h.rc.acroTrainer = true;   // в ANGLE ничего не меняет — только событие в журнале
            h.run(0.8);
            h.rc.acroTrainer = false;
        }

        void fly()
        {
            TEST_ASSERT_TRUE(h.takeoffTo(6.0));                   // ALT_HOLD: на 6 м и 3 с висения
            h.rc.mode = CopterMode::ANGLE;
            h.rc.throttle = h.ctl.getStatus().hoverLearned;
            ease(0.6f, 0.0f, 0.0f, 0.3);                          // наклон вправо
            h.run(0.8);
            ease(-0.6f, 0.0f, 0.0f, 0.5);                         // влево
            h.run(0.8);
            ease(0.0f, -0.5f, 0.0f, 0.4);                         // вперёд
            h.run(1.0);
            ease(0.0f, 0.0f, 0.0f, 0.4);
            h.rc.mode = CopterMode::ALT_HOLD;
            h.rc.throttle = 0.5f;
            h.run(1.5);
            h.rc.throttle = 0.9f;                                 // набор высоты перед флипами
            h.run(4.0);
            h.rc.throttle = 0.5f;
            h.run(1.0);
            flip(0);                                              // флип по крену
            h.rc.mode = CopterMode::ALT_HOLD;
            h.rc.throttle = 0.9f;
            h.run(2.0);
            h.rc.throttle = 0.5f;
            h.run(0.5);
            flip(1);                                              // флип по тангажу
            h.rc.mode = CopterMode::ANGLE;
            h.rc.throttle = h.ctl.getStatus().hoverLearned;
            h.run(1.2);
            h.rc.chirp = true;                                    // chirp по крену: полторы секунды и выключить
            h.run(1.5);
            h.rc.chirp = false;
            h.run(0.3);
            ease(0.0f, 0.0f, 1.0f, 0.1);                          // пируэт
            h.run(1.0);
            ease(0.0f, 0.0f, 0.0f, 0.1);
            h.run(0.8);
            h.rc.throttle = 0.35f;                                // рывок газом (висение ~0.15)
            h.run(0.25);
            h.rc.throttle = h.ctl.getStatus().hoverLearned;
            h.run(1.5);
            h.rc.mode = CopterMode::ALT_HOLD;                     // посадка
            h.rc.throttle = 0.1f;
            for (int i = 0; i < 30000 && h.sim.height() > 0.05; ++i) h.step();
            h.run(1.5);
            h.rc.throttle = 0.0f;
            h.rc.armSwitch = false;
            h.run(0.2);
            TEST_ASSERT_FALSE(h.ctl.isArmed());
            h.run(CopterBlackBox::POSTROLL_MS / 1000.0 + 0.5);
            for (int i = 0; i < 20000 && box.getState() != BlackBoxCore::State::Idle; ++i) h.step();
            TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, box.getState());
        }

        std::vector<uint8_t> image()
        {
            const BlackBoxStorage::Flight* f = storage.findFlight(1);
            TEST_ASSERT_NOT_NULL(f);
            std::vector<uint8_t> out(f->sectors * SECTOR_SIZE);
            for (uint32_t k = 0; k < f->sectors; ++k) storage.readSector(storage.sectorOf(*f, k), out.data() + k * SECTOR_SIZE);
            return out;
        }
    };

    double fieldMax(const std::vector<Rec>& recs, uint8_t type, int16_t (*get)(const Rec&))
    {
        double m = 0;
        for (const Rec& r : recs)
        {
            if (r.type == type) m = std::fmax(m, std::fabs(get(r)));
        }
        return m;
    }
}

// 139: тот же полёт -> .BFL. Разбор по заголовку самого журнала (как
// Blackbox Explorer): поля BF, кадр на каждый записанный цикл, I каждые 32,
// время и номер цикла идут подряд, гироскоп и моторы — те же, что в нашем
// журнале, режимы — кадрами S, в конце «End of log».
namespace
{
    struct VectorSink : bfl::Sink
    {
        std::vector<uint8_t> bytes;
        void put(const uint8_t* d, size_t n) override { bytes.insert(bytes.end(), d, d + n); }
    };

    void checkBfl(const std::vector<uint8_t>& image, const std::vector<Rec>& recs)
    {
        VectorSink sink;
        bfl::Writer writer(sink);
        bfl::CopterBflConverter conv(writer);
        for (size_t at = 0; at + SECTOR_SIZE <= image.size(); at += SECTOR_SIZE) conv.sector(image.data() + at);
        conv.finish();
        if (const char* path = getenv("OPENPLANE_COPTER_BFL_DUMP"))
        {
            if (FILE* file = fopen(path, "wb"))
            {
                fwrite(sink.bytes.data(), 1, sink.bytes.size(), file);
                fclose(file);
                TEST_MESSAGE("bfl written");
            }
        }

        const bflparse::Log log = bflparse::parse(sink.bytes);
        TEST_ASSERT_TRUE_MESSAGE(log.error.empty(), log.error.c_str());
        TEST_ASSERT_TRUE(log.ended);
        TEST_ASSERT_EQUAL_STRING("Product:Blackbox flight data recorder by Nicholas Sherlock", log.firstHeader.c_str());
        TEST_ASSERT_EQUAL_STRING("2", log.headers.at("Data version").c_str());
        TEST_ASSERT_EQUAL_STRING("1000", log.headers.at("looptime").c_str());
        TEST_ASSERT_EQUAL_STRING("48,2047", log.headers.at("motorOutput").c_str());
        TEST_ASSERT_EQUAL(0, log.headers.at("Firmware revision").find("Betaflight 4.5.0 (OpenPlane "));
        TEST_ASSERT_EQUAL_STRING("freestyle5", log.headers.at("Craft name").c_str());
        TEST_ASSERT_TRUE(log.headers.count("rollPID"));
        const int loopF = log.field("loopIteration"), timeF = log.field("time"), gyroF = log.field("gyroADC[0]"),
                  motorF = log.field("motor[0]"), motor3 = log.field("motor[3]"), spF = log.field("setpoint[0]"),
                  rcT = log.field("rcCommand[3]"), vbat = log.field("vbatLatest");
        TEST_ASSERT_EQUAL(0, loopF);
        TEST_ASSERT_EQUAL(1, timeF);
        TEST_ASSERT_TRUE(gyroF > 0 && motorF > 0 && motor3 > 0 && spF > 0 && rcT > 0 && vbat > 0);
        TEST_ASSERT_EQUAL(-1, log.field("motor[4]"));   // квадро

        // кадр на каждый QGYRO; I — каждый 32-й
        const size_t gyroRecs = countType(recs, REC_QGYRO);
        TEST_ASSERT_UINT32_WITHIN(1, gyroRecs, log.frames.size());
        for (size_t k = 0; k < log.frames.size(); ++k)
        {
            TEST_ASSERT_EQUAL(k % 32 == 0 ? 'I' : 'P', log.frameTypes[k]);
            TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(k), log.frames[k][loopF]);
            if (k) TEST_ASSERT_TRUE(log.frames[k][timeF] > log.frames[k - 1][timeF]);
            const int32_t m = log.frames[k][motorF];
            TEST_ASSERT_TRUE(m >= 48 && m <= 2047);
        }
        // гироскоп кадра — тот же, что в QGYRO (°/с с округлением)
        size_t g = 0;
        int32_t maxGyro = 0, maxSp = 0;
        for (const Rec& r : recs)
        {
            if (r.type != REC_QGYRO) continue;
            BlackBoxFormat::CopterGyroRecord q;
            memcpy(&q, r.data.data(), sizeof(q));
            if (g < log.frames.size())
            {
                TEST_ASSERT_INT32_WITHIN(1, q.gyroF[0] / 10, log.frames[g][gyroF]);
                maxGyro = std::max(maxGyro, std::abs(log.frames[g][gyroF]));
                maxSp = std::max(maxSp, std::abs(log.frames[g][spF]));
            }
            ++g;
        }
        char why[96];
        snprintf(why, sizeof(why), "bfl: %u frames, %u KB, gyro max %d, setpoint max %d", static_cast<unsigned>(log.frames.size()),
                 static_cast<unsigned>(sink.bytes.size() / 1024), static_cast<int>(maxGyro), static_cast<int>(maxSp));
        TEST_MESSAGE(why);
        TEST_ASSERT_TRUE_MESSAGE(maxGyro > 500 && maxSp > 500, why);   // флипы видны
        // газ при посадке/взлёте и режимы — кадрами S (ANGLE и ALT_HOLD были)
        TEST_ASSERT_TRUE(log.slow.size() >= 2);
        bool sawAngle = false, sawAltHold = false;
        for (const auto& sframe : log.slow)
        {
            sawAngle = sawAngle || (sframe[0] & 1u);
            sawAltHold = sawAltHold || (sframe[0] & 8u);
        }
        TEST_ASSERT_TRUE(sawAngle && sawAltHold);
        // P-кадры сжимают: в среднем меньше 2 байт на поле
        TEST_ASSERT_TRUE(sink.bytes.size() < log.frames.size() * log.fields.size() * 2);
    }
}

// Кодировщик сам по себе: zigzag, переменная длина, I/P, S, конец.
void test_bfl_writer_encoding()
{
    TEST_ASSERT_EQUAL_UINT32(0, bfl::zigzag(0));
    TEST_ASSERT_EQUAL_UINT32(1, bfl::zigzag(-1));
    TEST_ASSERT_EQUAL_UINT32(2, bfl::zigzag(1));
    TEST_ASSERT_EQUAL_UINT32(4294967295u, bfl::zigzag(INT32_MIN));
    VectorSink sink;
    bfl::Writer w(sink);
    bfl::Header h;
    h.motors = 6;
    snprintf(h.craft, sizeof(h.craft), "hexa");
    w.begin(h);
    TEST_ASSERT_EQUAL(bfl::F_MOTOR + 6, w.fieldCount());
    std::vector<int32_t> v(w.fieldCount(), 0);
    for (int k = 0; k < 40; ++k)
    {
        v[bfl::F_LOOP] = k;
        v[bfl::F_TIME] = k * 500;
        v[bfl::F_GYRO] = (k % 2 ? -1 : 1) * 70000;   // большие и со знаком
        v[bfl::F_MOTOR + 5] = 2047;
        w.mainFrame(v.data());
        if (k == 3) w.slowFrame(8, 1, 0, true, true);
    }
    w.end();
    const bflparse::Log log = bflparse::parse(sink.bytes);
    TEST_ASSERT_TRUE_MESSAGE(log.error.empty(), log.error.c_str());
    TEST_ASSERT_EQUAL(40, log.frames.size());
    TEST_ASSERT_EQUAL('I', log.frameTypes[32]);
    TEST_ASSERT_EQUAL_INT32(-70000, log.frames[39][log.field("gyroADC[0]")]);
    TEST_ASSERT_EQUAL_INT32(19500, log.frames[39][log.field("time")]);
    TEST_ASSERT_EQUAL_INT32(2047, log.frames[39][log.field("motor[5]")]);
    TEST_ASSERT_EQUAL(1, log.slow.size());
    TEST_ASSERT_EQUAL_UINT32(8, log.slow[0][0]);
    TEST_ASSERT_EQUAL_STRING("hexa", log.headers.at("Craft name").c_str());
    TEST_ASSERT_TRUE(log.ended);
}

void test_simulated_flight_is_in_the_log()
{
    Flight f;
    f.fly();
    TEST_ASSERT_EQUAL_UINT32(1, f.storage.flightCount());
    const std::vector<Rec> recs = readFlight(f.storage, 1);
    TEST_ASSERT_EQUAL(REC_END, recs.back().type);
    if (const char* path = getenv("OPENPLANE_COPTER_BLACKBOX_DUMP"))
    {
        const std::vector<uint8_t> img = f.image();
        if (FILE* file = fopen(path, "wb"))
        {
            fwrite(img.data(), 1, img.size(), file);
            fclose(file);
            TEST_MESSAGE("dump written");
        }
    }
    checkBfl(f.image(), recs);

    const BlackBoxStorage::Flight& fl = *f.storage.findFlight(1);
    // по меткам самих записей цикла: заголовок пишется при открытии полёта — позже предзаписи
    uint32_t t0 = 0, t1 = 0;
    for (const Rec& r : recs)
    {
        if (r.type != REC_QGYRO) continue;
        if (!t0) t0 = r.t();
        t1 = r.t();
    }
    const double seconds = (t1 - t0) / 1e6;
    char line[240];
    snprintf(line, sizeof(line), "copter log: %.1f s, %lu KB, %u QGYRO, %u QSTATE, %u events, recording %d", seconds,
             (unsigned long)(fl.sectors * SECTOR_SIZE / 1024), (unsigned)countType(recs, REC_QGYRO),
             (unsigned)countType(recs, REC_QSTATE), (unsigned)countType(recs, REC_EVENT), f.box.isRecording() ? 1 : 0);
    TEST_MESSAGE(line);

    // гироскоп/PID/моторы — каждый цикл, состояние — 50 Гц
    const double gyroHz = countType(recs, REC_QGYRO) / seconds;
    const double stateHz = countType(recs, REC_QSTATE) / seconds;
    TEST_ASSERT_TRUE_MESSAGE(gyroHz > 980 && gyroHz < 1010, line);
    TEST_ASSERT_TRUE_MESSAGE(stateHz > 48 && stateHz < 51, line);
    TEST_ASSERT_EQUAL(countType(recs, REC_QGYRO), countType(recs, REC_QCTRL));

    // флип: гироскоп после фильтров и уставка — за 500 °/с
    const double flipGyro = fieldMax(recs, REC_QGYRO, [](const Rec& r) { return r.as<BlackBoxFormat::CopterGyroRecord>().gyroF[0]; }) / 10.0;
    const double flipSp = fieldMax(recs, REC_QGYRO, [](const Rec& r) { return r.as<BlackBoxFormat::CopterGyroRecord>().setpoint[0]; }) / 10.0;
    const double maxAlt = fieldMax(recs, REC_QSTATE, [](const Rec& r) { return r.as<BlackBoxFormat::CopterStateRecord>().altDm; }) / 10.0;
    snprintf(line, sizeof(line), "flip gyro %.0f dps, setpoint %.0f dps, max altitude %.1f m", flipGyro, flipSp, maxAlt);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(flipGyro > 500 && flipSp > 500, line);
    TEST_ASSERT_TRUE_MESSAGE(maxAlt > 8.0 && maxAlt < 30.0, line);

    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "запись: старт (ARM)"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "режим ACRO"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "режим ANGLE"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "ACRO TRAINER: вкл"));

    // chirp (168): запись QCHIRP каждый цикл, пока идёт свип, и события
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "chirp: roll, свип 1"));
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "chirp: конец — SWITCH"));
    TEST_ASSERT_INT_WITHIN(5, 1500, static_cast<int>(countType(recs, REC_QCHIRP)));
    {
        int excite = 0, sum = 0, n = 0;
        uint16_t firstHz = 0, lastHz = 0;
        for (const Rec& r : recs)
        {
            if (r.type != REC_QCHIRP) continue;
            const BlackBoxFormat::CopterChirpRecord q = r.as<BlackBoxFormat::CopterChirpRecord>();
            TEST_ASSERT_EQUAL(0, q.axis);
            TEST_ASSERT_EQUAL(1, q.run);
            excite = std::max(excite, std::abs(static_cast<int>(q.excite)));
            sum = std::max(sum, std::abs(q.command - q.excite));
            if (!firstHz) firstHz = q.hz;
            lastHz = q.hz;
            ++n;
        }
        snprintf(line, sizeof(line), "chirp in the log: %d records, excitation up to %.3f, PID answers up to %.3f, %.1f -> %.1f Hz", n, excite / 1e4,
                 sum / 1e4, firstHz / 10.0, lastHz / 10.0);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(excite > 400 && excite < 700, line);   // 6 % хода
        TEST_ASSERT_TRUE_MESSAGE(sum > 50, line);                       // PID отвечает на раскачку
        TEST_ASSERT_TRUE_MESSAGE(firstHz >= 10 && lastHz > firstHz, line);
    }
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "ACRO TRAINER: выкл"));
    TEST_ASSERT_FALSE(hasText(recs, REC_EVENT, "GPS Rescue"));   // без GPS возврата не было
    TEST_ASSERT_TRUE(hasText(recs, REC_EVENT, "DISARM"));
    TEST_ASSERT_FALSE(hasText(recs, REC_EVENT, "очередь переполнена"));
    TEST_ASSERT_FALSE(hasText(recs, REC_EVENT, "авария"));
    TEST_ASSERT_FALSE(hasText(recs, REC_EVENT, "ARM отклонён"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "vehicle=copter"));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "frame=QUAD_X") || hasText(recs, REC_INFO, "frame="));

}

// 177: драйвер SPI-NOR (hal/SpiNorFlash.h) на эмуляторе микросхемы: JEDEC,
// запись через границу страницы — двумя командами, сектор и блок, вся
// микросхема — одной командой, ожидание «занята», чужой и пустой ответ.
namespace
{
    class TestSpiBus : public ISpiBus
    {
    public:
        explicit TestSpiBus(SPIClass& b) : bus(b) {}
        void begin() override { bus.begin(); }
        void beginTransaction(uint32_t hz, uint8_t) override { bus.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0)); }
        uint8_t transfer(uint8_t d) override { return bus.transfer(d); }
        void endTransaction() override { bus.endTransaction(); }

    private:
        SPIClass& bus;
    };
}

// 168: три полных свипа в журнале — ни одной записи QCHIRP не потеряно, и по
// самому журналу (целые числа: команда 1e-4 хода, гироскоп 0.1 °/с) коптер
// измеряется так же, как по точным данным: K, τ и задержка — в пределах 10 %.
// OPENPLANE_COPTER_CHIRP_DUMP=<файл> — сохранить журнал (сквозная проверка
// расчёта в OpenPlane Studio: src/core/chirplog.test.ts).
void test_chirp_sweeps_in_the_log_identify_the_copter()
{
    Flight f;
    TEST_ASSERT_TRUE(f.h.takeoffTo(12.0));
    f.h.rc.mode = CopterMode::ANGLE;
    f.h.rc.throttle = f.h.ctl.getStatus().hoverLearned;
    f.h.run(1.0);
    TruePlant truth[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        truth[axis] = truePlant(f.h, axis);
        const ChirpLog log = flyChirp(f.h);
        TEST_ASSERT_EQUAL(axis, log.axis);
        TEST_ASSERT_EQUAL(static_cast<int>(ChirpEnd::DONE), static_cast<int>(log.end));
    }
    f.h.rc.mode = CopterMode::ALT_HOLD;
    f.h.rc.throttle = 0.1f;
    for (int i = 0; i < 40000 && f.h.sim.height() > 0.05; ++i) f.h.step();
    f.h.run(1.5);
    f.h.rc.throttle = 0.0f;
    f.h.rc.armSwitch = false;
    f.h.run(CopterBlackBox::POSTROLL_MS / 1000.0 + 1.0);
    for (int i = 0; i < 20000 && f.box.getState() != BlackBoxCore::State::Idle; ++i) f.h.step();
    TEST_ASSERT_EQUAL(BlackBoxCore::State::Idle, f.box.getState());

    const std::vector<Rec> recs = readFlight(f.storage, 1);
    if (const char* path = getenv("OPENPLANE_COPTER_CHIRP_DUMP"))
    {
        const std::vector<uint8_t> img = f.image();
        if (FILE* file = fopen(path, "wb"))
        {
            fwrite(img.data(), 1, img.size(), file);
            fclose(file);
            TEST_MESSAGE("chirp log written");
        }
    }
    TEST_ASSERT_FALSE(hasText(recs, REC_EVENT, "очередь переполнена"));
    std::vector<double> r[3], u[3], y[3], yf[3];
    for (const Rec& rec : recs)
    {
        if (rec.type != REC_QCHIRP) continue;
        const BlackBoxFormat::CopterChirpRecord q = rec.as<BlackBoxFormat::CopterChirpRecord>();
        TEST_ASSERT_TRUE(q.axis < 3);
        TEST_ASSERT_EQUAL(q.axis + 1, q.run);
        r[q.axis].push_back(q.excite / 1e4);
        u[q.axis].push_back(q.command / 1e4);
        y[q.axis].push_back(q.gyro / 10.0);
        yf[q.axis].push_back(q.gyroF / 10.0);
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        const sysid::Response resp = sysid::estimate(r[axis], u[axis], y[axis], yf[axis], 1000.0);
        const sysid::Plant p = sysid::fit(resp, axis == 2);
        char line[220];
        snprintf(line, sizeof(line), "from the log, axis %d: %d records, K %.0f (true %.0f), tau %.1f ms (%.1f), delay %.2f ms (%.2f), fit rms %.3f on %d bands", axis,
                 static_cast<int>(r[axis].size()), p.gain, truth[axis].gain, p.tau * 1e3, truth[axis].tau * 1e3, p.delay * 1e3,
                 truth[axis].delay * 1e3, p.error, p.points);
        TEST_MESSAGE(line);
        TEST_ASSERT_INT_WITHIN_MESSAGE(3, 20000, static_cast<int>(r[axis].size()), line);
        TEST_ASSERT_TRUE_MESSAGE(p.valid() && p.points >= 40, line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10 * truth[axis].delay, truth[axis].delay, p.delay, line);
        if (axis < 2)
        {
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10 * truth[axis].gain, truth[axis].gain, p.gain, line);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10 * truth[axis].tau, truth[axis].tau, p.tau, line);
        }
    }
    // заголовок: то, по чему Studio восстанавливает регулятор
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "pid.roll="));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "filter.dterm="));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "tune.level_gain="));
    TEST_ASSERT_TRUE(hasText(recs, REC_INFO, "rates.yaw="));
}

void test_spi_nor_flash_driver()
{
    SPIClass spi;
    TestSpiBus bus(spi);
    chips::SpiNorFlashChip chip(0x202015, 2u * 1024 * 1024);   // M25P16, 2 МБ
    chip.attach(spi, 5);
    SpiNorFlash flash(bus, 5);
    TEST_ASSERT_TRUE(flash.begin());
    TEST_ASSERT_EQUAL_UINT32(2u * 1024 * 1024, flash.size());
    TEST_ASSERT_EQUAL_STRING("Micron/ST", flash.vendor());
    TEST_ASSERT_EQUAL_UINT32(256, flash.writeUnit());

    // 300 байт с адреса 200: страница 0 (56 байт) и страница 1 (244)
    std::vector<uint8_t> data(300);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 7 + 1);
    TEST_ASSERT_TRUE(flash.write(200, data.data(), data.size()));
    TEST_ASSERT_EQUAL_UINT32(2, chip.programs);
    std::vector<uint8_t> back(300);
    TEST_ASSERT_TRUE(flash.read(200, back.data(), back.size()));
    TEST_ASSERT_TRUE(data == back);
    TEST_ASSERT_EQUAL_UINT32(0, chip.rejected);
    TEST_ASSERT_EQUAL_UINT32(0, chip.readsWhileBusy);

    // сектор 4 КБ; 64 КБ с выровненного адреса — блоком; вся — одной командой
    TEST_ASSERT_TRUE(flash.erase(0, 4096));
    TEST_ASSERT_EQUAL_UINT8(0xFF, chip.mem[250]);
    TEST_ASSERT_EQUAL_UINT32(1, chip.sectorErases);
    TEST_ASSERT_TRUE(flash.erase(65536, 65536 + 8192));
    TEST_ASSERT_EQUAL_UINT32(1, chip.blockErases);
    TEST_ASSERT_EQUAL_UINT32(3, chip.sectorErases);
    TEST_ASSERT_FALSE(flash.erase(100, 4096));                 // не по границе сектора
    TEST_ASSERT_FALSE(flash.write(2u * 1024 * 1024 - 10, data.data(), 20));   // за концом
    TEST_ASSERT_TRUE(flash.erase(0, flash.size()));
    TEST_ASSERT_EQUAL_UINT32(1, chip.chipErases);

    // хранилище ящика поверх него — кольцо секторов находится
    BlackBoxStorage storage(flash);
    TEST_ASSERT_TRUE(storage.begin());
    TEST_ASSERT_EQUAL_UINT32(512, storage.totalSectors());

    // нет микросхемы (0xFF) и нет пина — begin() = false, размер 0
    SPIClass empty;
    TestSpiBus emptyBus(empty);
    SpiNorFlash none(emptyBus, 6);
    TEST_ASSERT_FALSE(none.begin());
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFF, none.jedecId());
    TEST_ASSERT_EQUAL_UINT32(0, none.size());
    SpiNorFlash noPin(bus, -1);
    TEST_ASSERT_FALSE(noPin.begin());
    uint8_t b = 0;
    TEST_ASSERT_FALSE(noPin.read(0, &b, 1));

    // W25Q256 (32 МБ) — берётся как 16 МБ: трёхбайтовые адреса
    SPIClass big;
    TestSpiBus bigBus(big);
    chips::SpiNorFlashChip w256(0xEF4019, 1024);
    w256.attach(big, 7);
    SpiNorFlash flash256(bigBus, 7);
    TEST_ASSERT_TRUE(flash256.begin());
    TEST_ASSERT_EQUAL_UINT32(16u * 1024 * 1024, flash256.size());
}

// P57: SD-карта по SPI (hal/SpiSdCard.h) — слот microSD у SpeedyBee F405 V3/V4.
// SDHC: запуск, блоки по одному и подряд (CMD18/CMD25), файл BLACKBOX.BIN на
// FAT32 и кольцо ящика поверх; SDSC v1 — адреса байтами; нет карты, нет пина,
// за концом, карта отвергла запись.
void test_spi_sd_card_driver()
{
    SPIClass spi;
    TestSpiBus bus(spi);
    chips::SpiSdCardChip chip(32768);   // 16 МБ SDHC
    chip.attach(spi, 9);
    SpiSdCard card(bus, 9);
    TEST_ASSERT_TRUE(card.begin());
    TEST_ASSERT_EQUAL_UINT8(SpiSdCard::INIT_OK, card.initError());
    TEST_ASSERT_TRUE(card.highCapacity());
    TEST_ASSERT_EQUAL_UINT32(32768, card.blockCount());

    std::vector<uint8_t> one(512), back(512);
    for (size_t i = 0; i < one.size(); ++i) one[i] = static_cast<uint8_t>(i * 3 + 1);
    TEST_ASSERT_TRUE(card.write(100, one.data(), 1));
    TEST_ASSERT_EQUAL_UINT8(one[7], chip.data[100 * 512 + 7]);
    TEST_ASSERT_TRUE(card.read(100, back.data(), 1));
    TEST_ASSERT_TRUE(one == back);

    std::vector<uint8_t> many(512 * 5), manyBack(512 * 5);
    for (size_t i = 0; i < many.size(); ++i) many[i] = static_cast<uint8_t>(i * 13 + 5);
    TEST_ASSERT_TRUE(card.write(2000, many.data(), 5));
    TEST_ASSERT_EQUAL_UINT32(1, chip.multiWrites);
    TEST_ASSERT_EQUAL_UINT32(2004, chip.lastWriteBlock);
    TEST_ASSERT_TRUE(card.read(2000, manyBack.data(), 5));
    TEST_ASSERT_EQUAL_UINT32(1, chip.multiReads);
    TEST_ASSERT_TRUE(chip.sawCmd12);
    TEST_ASSERT_TRUE(many == manyBack);
    // после чтения подряд карта снова слушает команды
    TEST_ASSERT_TRUE(card.read(100, back.data(), 1));
    TEST_ASSERT_TRUE(one == back);

    TEST_ASSERT_FALSE(card.read(32768, back.data(), 1));        // за концом
    TEST_ASSERT_FALSE(card.write(32766, many.data(), 5));
    TEST_ASSERT_FALSE(card.read(0, back.data(), 0));
    TEST_ASSERT_EQUAL_UINT32(0, card.timeoutCount());

    // файл ящика на FAT32 и кольцо секторов поверх — как у H743
    const fat_image::Result img = fat_image::format(chip, { { "BLACKBOX.BIN", 4u * 1024 * 1024 } });
    std::fill_n(chip.data.begin() + static_cast<size_t>(img.firstBlock.at("BLACKBOX.BIN")) * 512, 4u * 1024 * 1024, 0xFF);
    SdFileRegion file(card, "BLACKBOX.BIN");
    TEST_ASSERT_EQUAL(static_cast<int>(Fat32::Result::Ok), static_cast<int>(file.begin()));
    BlackBoxStorage storage(file);
    TEST_ASSERT_TRUE(storage.begin());
    TEST_ASSERT_TRUE(storage.totalSectors() > 0);

    // SDSC (v1, без CMD8): адреса — байтами, блок 512 — CMD16
    SPIClass spi2;
    TestSpiBus bus2(spi2);
    chips::SpiSdCardChip old(4096, false);
    old.v2 = false;
    old.attach(spi2, 10);
    SpiSdCard sdsc(bus2, 10);
    TEST_ASSERT_TRUE(sdsc.begin());
    TEST_ASSERT_FALSE(sdsc.highCapacity());
    TEST_ASSERT_EQUAL_UINT32(4096, sdsc.blockCount());
    TEST_ASSERT_TRUE(sdsc.write(5, one.data(), 1));
    TEST_ASSERT_EQUAL_UINT8(one[0], old.data[5 * 512]);
    TEST_ASSERT_EQUAL_UINT8(one[511], old.data[5 * 512 + 511]);
    TEST_ASSERT_TRUE(sdsc.read(5, back.data(), 1));
    TEST_ASSERT_TRUE(one == back);

    // карта отвергла запись — false, не зависает
    old.rejectWrites = true;
    TEST_ASSERT_FALSE(sdsc.write(6, one.data(), 1));
    TEST_ASSERT_FALSE(sdsc.write(6, many.data(), 3));

    // нет карты, нет пина
    SPIClass empty;
    TestSpiBus emptyBus(empty);
    SpiSdCard none(emptyBus, 11);
    TEST_ASSERT_FALSE(none.begin());
    TEST_ASSERT_EQUAL_UINT8(SpiSdCard::INIT_NO_IDLE, none.initError());
    TEST_ASSERT_EQUAL_UINT32(0, none.blockCount());
    TEST_ASSERT_FALSE(none.read(0, back.data(), 1));
    SpiSdCard noPin(bus, -1);
    TEST_ASSERT_FALSE(noPin.begin());
    TEST_ASSERT_EQUAL_UINT8(SpiSdCard::INIT_NO_PIN, noPin.initError());

    // карта, что не выходит из ожидания, — отказ за секунду, не вечный цикл
    SPIClass spi3;
    TestSpiBus bus3(spi3);
    chips::SpiSdCardChip stuck(4096);
    stuck.readyPolls = 1000000;
    stuck.attach(spi3, 12);
    SpiSdCard slow(bus3, 12);
    TEST_ASSERT_FALSE(slow.begin());
    TEST_ASSERT_EQUAL_UINT8(SpiSdCard::INIT_NOT_READY, slow.initError());
}

// SpeedyBee F405 V3 (P57c): SD и OSD на одной SPI2. Карта долго занята после
// блока — на время сна шина отпущена: CS карты поднят, транзакция закрыта,
// другой на этой SPI (OSD) делает свои обмены; запись при этом цела.
void test_spi_sd_card_releases_shared_bus_while_busy()
{
    class SharedBus : public ISpiBus
    {
    public:
        SharedBus(SPIClass& b, uint8_t sdCs, uint8_t otherCs) : bus(b), sd(sdCs), other(otherCs) {}
        void begin() override { bus.begin(); }
        void beginTransaction(uint32_t hz, uint8_t) override { bus.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0)); }
        uint8_t transfer(uint8_t d) override { return bus.transfer(d); }
        void endTransaction() override
        {
            bus.endTransaction();
            if (digitalRead(sd) == LOW)
            {
                ++heldByCard;   // карта осталась выбранной — чужой обмен её бы испортил
                return;
            }
            // пауза на шине — обмен OSD: запись регистра
            bus.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
            digitalWrite(other, LOW);
            bus.transfer(0x01);
            bus.transfer(static_cast<uint8_t>(++foreign));
            digitalWrite(other, HIGH);
            bus.endTransaction();
        }
        uint32_t foreign = 0, heldByCard = 0;

    private:
        SPIClass& bus;
        uint8_t sd, other;
    };

    SPIClass spi;
    SharedBus bus(spi, 9, 13);
    chips::SpiSdCardChip chip(32768);
    chip.attach(spi, 9);
    fake::SpiRegisterMapDevice osd;
    spi.attach(13, &osd);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    SpiSdCard card(bus, 9);
    TEST_ASSERT_TRUE(card.begin());

    std::vector<uint8_t> blocks(512 * 3);
    for (size_t i = 0; i < blocks.size(); ++i) blocks[i] = static_cast<uint8_t>(i * 7 + 3);
    chip.busyBytes = 512 + 100;   // ~100 мс занятости после каждого блока
    const uint32_t before = bus.foreign;
    TEST_ASSERT_TRUE(card.write(300, blocks.data(), 3));
    TEST_ASSERT_TRUE(card.write(400, blocks.data(), 1));
    TEST_ASSERT_EQUAL_UINT32(0, card.timeoutCount());
    TEST_ASSERT_EQUAL_UINT32(0, bus.heldByCard);
    char why[64];
    snprintf(why, sizeof(why), "чужих обменов %u", static_cast<unsigned>(bus.foreign - before));
    TEST_ASSERT_TRUE_MESSAGE(bus.foreign - before > 300, why);   // 4 блока × ~100 пауз
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(bus.foreign), osd.regs[0x01]);
    std::vector<uint8_t> back(512 * 3);
    chip.busyBytes = 4;
    TEST_ASSERT_TRUE(card.read(300, back.data(), 3));
    TEST_ASSERT_TRUE(blocks == back);
    TEST_ASSERT_TRUE(card.read(400, back.data(), 1));
    TEST_ASSERT_TRUE(std::equal(back.begin(), back.begin() + 512, blocks.begin()));
}

// Предзапись длиннее очереди (F405: 40 КБ на ~1.2 с): в покое старые записи
// уходят молча, место под такт остаётся — «потеряно записей» не растёт.
void test_ring_preroll_trim_keeps_headroom_without_drops()
{
    uint8_t memory[200];
    BlackBoxRing ring;
    ring.begin(memory, sizeof(memory));
    uint8_t rec[20] = { BlackBoxFormat::REC_EVENT, 18 };
    for (uint32_t t = 0; t < 100; ++t)
    {
        memcpy(rec + 2, &t, 4);
        ring.trimOlderThan(t, 1000000, 2 * sizeof(rec));
        TEST_ASSERT_TRUE(ring.push(rec, sizeof(rec)));
    }
    TEST_ASSERT_EQUAL_UINT32(0, ring.dropped());
    TEST_ASSERT_TRUE(ring.freeBytes() >= sizeof(rec));
    TEST_ASSERT_EQUAL_UINT32(9 * sizeof(rec), ring.used());   // до push свободно ≥ 40, после — 20

    // по возрасту — как раньше; без запаса push выталкивает и считает потерю
    ring.trimOlderThan(150, 55);
    TEST_ASSERT_EQUAL_UINT32(5 * sizeof(rec), ring.used());   // остались t = 95..99
    for (uint32_t t = 100; t < 110; ++t) ring.push(rec, sizeof(rec));
    TEST_ASSERT_EQUAL_UINT32(5, ring.dropped());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_ring_preroll_trim_keeps_headroom_without_drops);
    RUN_TEST(test_bfl_writer_encoding);
    RUN_TEST(test_simulated_flight_is_in_the_log);
    RUN_TEST(test_chirp_sweeps_in_the_log_identify_the_copter);
    RUN_TEST(test_spi_nor_flash_driver);
    RUN_TEST(test_spi_sd_card_driver);
    RUN_TEST(test_spi_sd_card_releases_shared_bus_while_busy);
    return UNITY_END();
}
