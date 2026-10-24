// ============================================================
// Прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на Matek
// H743-WING: ICM-42688-P (SPI1) и DPS310 (I2C2) на плате, приёмник iBUS
// на USART6 (PC7), моторы — площадки S1–S4 (PB0, PB1, PA0, PA1),
// регуляторы OneShot125, батарея 6S на делителе PC0.
//
// Проверяется склейка с железом: что вывелось из описания коптера,
// импульсы «стоп» до ARM, ARM только после калибровки установки IMU,
// холостые после ARM, режимы с тумблера, потеря связи на земле, консоль,
// проверка моторов с подтверждением и CLI как в Betaflight (флеш): aux,
// map, протокол приёмника, status/tasks, смена сборки, протокол
// настроек для приложения (@params/@set/@save).
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_copter
// ============================================================

#define BOARD_MATEK_H743
#define SENSOR_KIT SENSOR_KIT_MATEK_H743
#define VEHICLE_MULTIROTOR
#include "../../../src/stm32/main_copter.cpp"

#include <unity.h>

#include <cctype>
#include <string>
#include <vector>

#include "helpers/ChipEmulators.h"
#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    World world;
    std::string bootLog;
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;   // DPS310 — те же регистры

    const int16_t MOTORS[] = { PB0, PB1, PA0, PA1 };

    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

    void wireUpBoard()
    {
        imuChip.rotationCwDeg = 90;   // плата повёрнута стрелкой вправо
        imuChip.attach(SPI, static_cast<uint8_t>(PC15));
        imuChip.install();
        baroChip.attach(Wire, 0x76);
        baroChip.install();
        fake::gpio().analogMv[PC0] = 25200 / 11;   // 6S, 25.2 В через делитель 1:11
    }

    void tick(const RcChannels* rc)
    {
        imuChip.update(world);
        baroChip.update(world);
        if (rc && millis() % 7 == 0) port(PC7).pushRx(ibusFrame(*rc));   // кадр iBUS ~каждые 7 мс
        fake::runTask(*fake::findTask("flight"), 1);
    }

    void run(const RcChannels& rc, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(&rc);
    }

    void runWithoutLink(double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000); ++i) tick(nullptr);
    }

    long pulse(int16_t pin) { return fake::timerPulseUs(static_cast<uint32_t>(pin)); }
}

void test_boot_derives_tune_and_holds_escs_at_stop()
{
    imuChip.update(world);
    baroChip.update(world);
    setup();
    const std::string log = takeSerial();
    bootLog = log;
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Коптер: freestyle5"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "регуляторы OneShot125, цикл 1000 Гц"));
    TEST_ASSERT_TRUE(contains(log, "Тяговооружённость"));
    TEST_ASSERT_TRUE(contains(log, "ICM42688: подключён"));
    TEST_ASSERT_TRUE(contains(log, "DPS310: подключён"));
    TEST_ASSERT_FALSE_MESSAGE(contains(log, "ОШИБКА"), log.c_str());
    TEST_ASSERT_EQUAL_UINT8(4, motors.attached());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));   // OneShot125: «стоп»
    TEST_ASSERT_EQUAL(-1, pulse(PA2));   // пятый выход квадро не нужен
    TEST_ASSERT_EQUAL_UINT32(2000, Stm32MotorOutput::outputRateHz(copter::EscProtocol::ONESHOT125));
    TEST_ASSERT_FALSE(copterController.isArmed());
}

void test_arm_waits_for_mount_calibration()
{
    RcChannels rc;
    run(rc, 2.5);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.2);
    TEST_ASSERT_FALSE(copterController.isArmed());
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_NO_ACC_CAL);
    Serial.pushRx("s");
    run(rc, 0.01);
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(status, "  - NO_ACC_CAL: не откалибрована установка платы"), status.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(status, "Arming disable flags: NO_ACC_CAL ARMSWITCH\r\n"), status.c_str());
    TEST_ASSERT_TRUE(contains(status, "Батарея 25."));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.2);
}

void test_three_poses_then_arm_spins_motors_at_idle()
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
    TEST_ASSERT_TRUE(contains(takeSerial(), "установка сохранена"));

    RcChannels rc;
    run(rc, 3.0);
    TEST_ASSERT_EQUAL_HEX32(0, copterController.getStatus().armBlockers);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    for (int16_t pin : MOTORS)
    {
        TEST_ASSERT_GREATER_THAN(125, pulse(pin));   // холостые
        TEST_ASSERT_LESS_THAN(140, pulse(pin));
    }
    // тумблер режима: SwC вниз — ACRO
    rc.set(Channels::SWC, 2000);
    run(rc, 0.05);
    TEST_ASSERT_EQUAL(static_cast<int>(copter::CopterMode::ACRO), static_cast<int>(copterController.getStatus().mode));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.05);
    TEST_ASSERT_FALSE(copterController.isArmed());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));
    takeSerial();
}

void test_link_loss_on_ground_disarms()
{
    RcChannels rc;
    run(rc, 0.3);
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    runWithoutLink(1.5);   // таймаут приёмника 0.5 с + 0.3 с ожидания
    TEST_ASSERT_FALSE(copterController.isArmed());
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));
    // связь вернулась, тумблер так и стоит — сам не армится
    run(rc, 0.5);
    TEST_ASSERT_FALSE(copterController.isArmed());
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);
    takeSerial();
}

void test_console_tune_report_and_motor_test()
{
    RcChannels rc;
    Serial.pushRx("t");
    run(rc, 0.01);
    const std::string tune = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(tune, "Сборка: freestyle5"), tune.c_str());
    TEST_ASSERT_TRUE(contains(tune, "Контур: срез"));
    TEST_ASSERT_TRUE(contains(tune, "Предупреждений нет."));

    // проверка моторов — только после подтверждения
    Serial.pushRx("m");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "СНИМИТЕ ПРОПЕЛЛЕРЫ"));
    Serial.pushRx("n");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "отменена"));
    for (int16_t pin : MOTORS) TEST_ASSERT_EQUAL(125, pulse(pin));

    Serial.pushRx("m");
    run(rc, 0.01);
    Serial.pushRx("y");
    run(rc, 0.5);
    const std::string first = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(first, "Мотор 1: задний правый, по часовой"), first.c_str());
    TEST_ASSERT_GREATER_THAN(125, pulse(PB0));
    TEST_ASSERT_EQUAL(125, pulse(PB1));
    run(rc, 2.0);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Мотор 2: передний правый, против часовой"));
    TEST_ASSERT_EQUAL(125, pulse(PB0));
    TEST_ASSERT_GREATER_THAN(125, pulse(PB1));
    // любая клавиша — стоп
    Serial.pushRx("x");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Проверка моторов окончена"));
    TEST_ASSERT_EQUAL(125, pulse(PB1));
    TEST_ASSERT_FALSE(copterConsole.motorTestRunning());

    // заармленному — не крутит
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(copterController.isArmed());
    Serial.pushRx("m");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Только без ARM."));
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);

    Serial.pushRx("hlr?");
    run(rc, 0.3);
    const std::string misc = takeSerial();
    TEST_ASSERT_TRUE(contains(misc, "m — проверка моторов"));
    TEST_ASSERT_TRUE(contains(misc, "Живая строка: вкл"));
    TEST_ASSERT_TRUE(contains(misc, "DISARMED ANGLE"));
    TEST_ASSERT_TRUE(contains(misc, "Каналы: "));
    Serial.pushRx("lq");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Неизвестная команда"));
}

// CLI как в Betaflight: ARM запрещён, пока CLI открыт; set → save — во флеш
// и в контроллер; после «перезагрузки» (настройки заново из флеша) — то же.
void test_cli_set_save_and_reboot()
{
    RcChannels rc;
    Serial.pushRx("#");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(copterConsole.cliActive());
    TEST_ASSERT_TRUE(contains(takeSerial(), "CLI коптера"));
    rc.set(Channels::ARM, 2000);
    run(rc, 0.1);
    TEST_ASSERT_FALSE(copterController.isArmed());
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_CLI);
    rc.set(Channels::ARM, 1000);
    run(rc, 0.05);

    // строки пачкой — как вставка diff
    Serial.pushRx("get p_roll\r\nset p_roll = 50\r\nprofile 1\r\nset profile_name = acro\r\nprofile 0\r\nstatus\r\nsave\r\n");
    run(rc, 0.05);
    std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "(из модели коптера)"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "p_roll = 50"));
    TEST_ASSERT_TRUE(contains(log, "Состояние: DISARMED"));
    TEST_ASSERT_TRUE(contains(log, "Сохранено, настройка пересчитана."));
    TEST_ASSERT_FALSE(copterConsole.cliActive());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, copter::bfToGain(copter::PidTerm::P, 0, 50.0f), copterController.getTune().roll.p);

    // запись во флеш — фоновой задачей; «перезагрузка» — настройки заново из флеша
    fake::runTask(*fake::findTask("storage"), 1);
    class FlashView : public IFlashStorage
    {
    public:
        size_t capacity() const override { return KeyValueStore::CAPACITY; }
        void read(uint8_t* destination, size_t size) override { memcpy(destination, fake::eeprom().flash, size); }
        bool write(const uint8_t*, size_t) override { return false; }
    } view;
    KeyValueStore reread(view);
    copterParams.resetAll();
    TEST_ASSERT_TRUE(copterParams.load(reread));
    TEST_ASSERT_TRUE(reread.contains(copter::CopterParams::STORE_SPACE, copter::CopterParams::STORE_KEY));
    Serial.pushRx("#get p_roll\r\nprofile 1\r\nexit\r\nt");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "p_roll = 50"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "profile 1 # acro"));
    TEST_ASSERT_TRUE(contains(log, "Несохранённые изменения отменены."));
    TEST_ASSERT_TRUE(contains(log, "PID: из модели, профиль 0; расходы — профиль 0"));
    TEST_ASSERT_TRUE(contains(log, "| BF: P 50 I "));
    TEST_ASSERT_TRUE(contains(log, "запас не проверен"));
}

namespace
{
    // Команда приложения: STX, строка, '\n'.
    void host(const char* command)
    {
        Serial.pushRx(std::string(1, '\x02') + command + "\n");
    }

    // Строгий разбор JSON: объект, массив, строка, число, true/false/null.
    struct Json
    {
        const char* p;
        bool ws()
        {
            while (*p == ' ' || *p == '\t') ++p;
            return true;
        }
        bool value()
        {
            ws();
            if (*p == '{') return object();
            if (*p == '[') return array();
            if (*p == '"') return string();
            if (!strncmp(p, "true", 4)) return p += 4, true;
            if (!strncmp(p, "false", 5)) return p += 5, true;
            if (!strncmp(p, "null", 4)) return p += 4, true;
            return number();
        }
        bool object()
        {
            ++p;
            ws();
            if (*p == '}') return ++p, true;
            for (;;)
            {
                ws();
                if (!string()) return false;
                ws();
                if (*p++ != ':') return false;
                if (!value()) return false;
                ws();
                if (*p == '}') return ++p, true;
                if (*p++ != ',') return false;
            }
        }
        bool array()
        {
            ++p;
            ws();
            if (*p == ']') return ++p, true;
            for (;;)
            {
                if (!value()) return false;
                ws();
                if (*p == ']') return ++p, true;
                if (*p++ != ',') return false;
            }
        }
        bool string()
        {
            if (*p++ != '"') return false;
            while (*p && *p != '"')
            {
                if (static_cast<unsigned char>(*p) < 0x20) return false;
                if (*p == '\\') ++p;
                ++p;
            }
            return *p++ == '"';
        }
        bool number()
        {
            const char* start = p;
            if (*p == '-') ++p;
            while (isdigit(static_cast<unsigned char>(*p))) ++p;
            if (*p == '.')
            {
                ++p;
                while (isdigit(static_cast<unsigned char>(*p))) ++p;
            }
            return p > start && isdigit(static_cast<unsigned char>(p[-1]));
        }
    };

    bool isJson(const std::string& text)
    {
        Json j{ text.c_str() };
        return j.value() && j.ws() && *j.p == '\0';
    }

    // Строки лога, начинающиеся с prefix (без него и без '\n').
    std::vector<std::string> linesWith(const std::string& log, const char* prefix)
    {
        std::vector<std::string> out;
        size_t at = 0;
        while (at < log.size())
        {
            size_t end = log.find('\n', at);
            if (end == std::string::npos) end = log.size();
            const std::string line = log.substr(at, end - at);
            if (line.compare(0, strlen(prefix), prefix) == 0) out.push_back(line.substr(strlen(prefix)));
            at = end + 1;
        }
        return out;
    }

    void armWith(RcChannels& rc, uint8_t channel)
    {
        rc.set(channel, 2000);
        run(rc, 0.1);
    }
}

// 006: режимы на тумблерах — таблица aux из CLI заменяет SwA/SwC прошивки.
void test_aux_moves_arm_and_modes()
{
    RcChannels rc;
    run(rc, 0.1);
    TEST_ASSERT_TRUE(contains(bootLog, "Режимы на тумблерах (умолчание прошивки): ARM — CH5 1700..2100; ANGLE — CH7 900..1300;"));
    Serial.pushRx("#aux 0 0 1 1700 2100 0 0\r\naux 1 1 2 1300 1700 0 0\r\nsave\r\n");
    run(rc, 2.5);   // после save — заново ориентация
    std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Сохранено"), log.c_str());

    armWith(rc, Channels::ARM);   // SwA больше не ARM
    TEST_ASSERT_FALSE(copterController.isArmed());
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);
    rc.set(Channels::SWC, 1500);   // CH7 посередине — ANGLE
    armWith(rc, Channels::SWB);    // ARM — CH6
    TEST_ASSERT_TRUE_MESSAGE(copterController.isArmed(), std::to_string(copterController.getStatus().armBlockers).c_str());
    TEST_ASSERT_EQUAL(static_cast<int>(copter::CopterMode::ANGLE), static_cast<int>(copterController.getStatus().mode));
    rc.set(Channels::SWC, 2000);   // остальное — ACRO
    run(rc, 0.05);
    TEST_ASSERT_EQUAL(static_cast<int>(copter::CopterMode::ACRO), static_cast<int>(copterController.getStatus().mode));
    Serial.pushRx("r");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Режимы: ARM\r\n"));
    rc.set(Channels::SWB, 1000);
    run(rc, 0.1);
    TEST_ASSERT_FALSE(copterController.isArmed());

    // обратно: пустая таблица — снова SwA и SwC
    Serial.pushRx("#aux 0 0 0 900 900 0 0\r\naux 1 0 0 900 900 0 0\r\nsave\r\n");
    run(rc, 2.5);
    rc.set(Channels::SWC, 1000);
    armWith(rc, Channels::ARM);
    TEST_ASSERT_TRUE(copterController.isArmed());
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);
    takeSerial();
}

// 007: map и протокол приёмника — без пересборки.
void test_map_and_receiver_protocol_from_cli()
{
    RcChannels rc;
    Serial.pushRx("#map TAER1234\r\nsave\r\n");
    run(rc, 2.5);
    takeSerial();
    // газ теперь CH1: он в центре — газ не внизу; CH3 (тангаж) газу не мешает
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_THROTTLE);
    rc.set(0, 1000).set(Channels::THROTTLE, 2000);
    run(rc, 0.1);
    TEST_ASSERT_EQUAL_HEX32(0, copterController.getStatus().armBlockers);
    // failsafe по газу — тоже по CH1
    rc.set(0, 900);
    run(rc, 0.1);
    TEST_ASSERT_TRUE(rcReceiver.isFailsafeReported());
    rc.set(0, 1000).set(Channels::THROTTLE, 1000);
    run(rc, 0.1);
    TEST_ASSERT_FALSE(rcReceiver.isSignalLost());

    // другой протокол: iBUS-кадры больше не разбираются
    Serial.pushRx("#set serialrx_provider = SBUS\r\nsave\r\n");
    run(rc, 1.0);
    TEST_ASSERT_EQUAL_STRING("SBUS", rcReceiver.getProtocolName());
    TEST_ASSERT_TRUE(rcReceiver.isSignalLost());
    Serial.pushRx("s");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "связь НЕТ (SBUS)"));

    Serial.pushRx("#set serialrx_provider = IBUS\r\nmap AETR1234\r\nsave\r\n");
    run(rc, 0.3);
    TEST_ASSERT_EQUAL_STRING("iBUS", rcReceiver.getProtocolName());
    TEST_ASSERT_FALSE(rcReceiver.isSignalLost());
    rc.set(0, 1500);
    run(rc, 2.5);
    TEST_ASSERT_EQUAL_HEX32(0, copterController.getStatus().armBlockers);
    takeSerial();
}

// 008: status и tasks как в Betaflight.
void test_status_and_tasks_like_betaflight()
{
    RcChannels rc;
    run(rc, 1.1);
    Serial.pushRx("#status\r\ntasks\r\nexit\r\n");
    run(rc, 0.05);
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "CPU:"), log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "GYRO rate: 1000"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "Voltage: 25"));
    TEST_ASSERT_TRUE(contains(log, "(6S battery - OK)"));
    TEST_ASSERT_TRUE(contains(log, "Arming disable flags: CLI\r\n"));
    TEST_ASSERT_TRUE(contains(log, "  - CLI: открыт CLI\n"));
    TEST_ASSERT_TRUE(contains(log, "Task list"));
    TEST_ASSERT_TRUE(contains(log, "(        GYRO)    1000"));
    TEST_ASSERT_TRUE(contains(log, "(   PID/MIXER)"));
    TEST_ASSERT_TRUE(contains(log, "01 - (        BARO)"));   // частота — фейк планировщика заходит в задачу заново
    TEST_ASSERT_TRUE(contains(log, "Цикл целиком: 1000 Гц"));

    // без связи — RXLOSS
    runWithoutLink(0.7);
    Serial.pushRx("s");
    runWithoutLink(0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "Arming disable flags: RXLOSS"));
    run(rc, 0.3);
    takeSerial();
}

// 009: описание коптера — настройки: сменить сборку без пересборки.
void test_copter_build_from_cli()
{
    RcChannels rc;
    Serial.pushRx("#set copter_build = race5\r\nset copter_mass_g = 600\r\nsave\r\nt");
    run(rc, 0.05);
    std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "Сборка: race5 — "), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "масса 600 г"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.6f, copterController.getTune().massKg);

    // шесть моторов на четырёх выходах — только после перезагрузки
    Serial.pushRx("#set copter_build = hex13\r\nsave\r\n");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "У рамы теперь 6 моторов (было 4) — перезагрузите плату"), log.c_str());
    run(rc, 0.05);
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_REBOOT);
    Serial.pushRx("s");
    run(rc, 0.01);
    TEST_ASSERT_TRUE(contains(takeSerial(), "REBOOT_REQD"));

    Serial.pushRx("#set copter_build = freestyle5\r\nsave\r\n");
    run(rc, 2.5);
    TEST_ASSERT_FALSE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_REBOOT);
    TEST_ASSERT_EQUAL_STRING("freestyle5", copterParams.buildSpec().id);
    takeSerial();
}

// 010: машинный протокол настроек для приложения.
void test_app_protocol_params_set_save()
{
    RcChannels rc;
    host("params");
    run(rc, 0.3);
    std::string log = takeSerial();
    const auto header = linesWith(log, "@params ");
    TEST_ASSERT_EQUAL_MESSAGE(1, static_cast<int>(header.size()), log.c_str());
    TEST_ASSERT_TRUE_MESSAGE(isJson(header[0]), header[0].c_str());
    TEST_ASSERT_TRUE(contains(header[0], "\"build\":\"freestyle5\""));
    const auto lines = linesWith(log, "@param ");
    TEST_ASSERT_EQUAL(copter::PARAM_COUNT, static_cast<int>(lines.size()));
    for (const std::string& l : lines) TEST_ASSERT_TRUE_MESSAGE(isJson(l), l.c_str());
    TEST_ASSERT_TRUE_MESSAGE(contains(lines[copter::PARAM_P_ROLL], "{\"name\":\"p_roll\",\"type\":\"int\",\"group\":\"pid\","
                                                                   "\"scope\":\"pid\",\"min\":0,\"max\":250,\"default\":"),
                             lines[copter::PARAM_P_ROLL].c_str());
    TEST_ASSERT_TRUE(contains(lines[copter::PARAM_P_ROLL], "\"value\":50,\"set\":true,\"profiles\":[50,"));
    TEST_ASSERT_TRUE(contains(lines[copter::PARAM_TUNE_SOURCE], "\"options\":[\"MODEL\",\"BETAFLIGHT\"]"));
    TEST_ASSERT_TRUE(contains(lines[copter::PARAM_COPTER_BUILD], "\"from\":\"table\""));
    TEST_ASSERT_TRUE(contains(lines[copter::PARAM_COPTER_MASS_G], "\"from\":\"build\""));
    const auto rcLine = linesWith(log, "@rc ");
    TEST_ASSERT_EQUAL(1, static_cast<int>(rcLine.size()));
    TEST_ASSERT_TRUE_MESSAGE(isJson(rcLine[0]), rcLine[0].c_str());
    TEST_ASSERT_TRUE(contains(rcLine[0], "\"map\":\"AETR1234\""));
    TEST_ASSERT_TRUE(contains(rcLine[0], "\"fallback\":[[0,0,0,1700,2100,0,0],[1,1,2,900,1300,0,0]"));
    const auto modes = linesWith(log, "@modes ");
    TEST_ASSERT_EQUAL(1, static_cast<int>(modes.size()));
    TEST_ASSERT_TRUE(isJson(modes[0]));
    TEST_ASSERT_TRUE(contains(modes[0], "[40,\"USER1\",true]"));
    // каждая строка влезает в очередь USB CDC целиком (у H743 — 16 × 64 байта),
    // иначе она не уйдёт никогда
    for (const std::string& l : lines) TEST_ASSERT_TRUE_MESSAGE(l.size() < 1000, l.c_str());
    TEST_ASSERT_TRUE(rcLine[0].size() < 1000);
    TEST_ASSERT_TRUE(modes[0].size() < 1000);

    // set: в ОЗУ, ARM запрещён (MSP), пока не save
    host("set p_pitch 47");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "@set {\"name\":\"p_pitch\",\"ok\":true,\"value\":47}\n"), log.c_str());
    TEST_ASSERT_TRUE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_MSP);
    host("set p_pitch = 300");
    host("set nonsense 1");
    run(rc, 0.1);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "@set {\"name\":\"p_pitch\",\"ok\":false,\"error\":\"допустимо 0 - 250\",\"value\":47}"),
                             log.c_str());
    TEST_ASSERT_TRUE(contains(log, "@set {\"name\":\"nonsense\",\"ok\":false,\"error\":\"нет такой настройки\"}"));
    host("aux 0 0 0 1700 2100 0 0");
    host("map TAER");
    host("vtx 0 2 0 0 1 900 1300");   // мощность — тумблером AUX3
    run(rc, 0.1);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "@rc {\"map\":\"TAER1234\""), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "\"aux\":[[0,0,0,1700,2100,0,0]]"));
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "\"vtx\":[[0,2,0,0,1,900,1300]]"), log.c_str());
    host("revert");
    run(rc, 0.05);
    TEST_ASSERT_TRUE(contains(takeSerial(), "@revert {\"ok\":true}"));
    TEST_ASSERT_FALSE(copterConsole.appUnsaved());
    TEST_ASSERT_TRUE(copterParams.rc().mapIsDefault());

    host("set p_pitch 47");
    host("save");
    run(rc, 0.1);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "@save {\"ok\":true,\"stored\":true}"), log.c_str());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, copter::bfToGain(copter::PidTerm::P, 1, 47.0f), copterController.getTune().pitch.p);
    TEST_ASSERT_FALSE(copterController.getStatus().armBlockers & copter::ARM_BLOCK_MSP);

    host("id");
    host("live");
    run(rc, 0.2);
    log = takeSerial();
    const auto id = linesWith(log, "@id ");
    TEST_ASSERT_EQUAL_MESSAGE(1, static_cast<int>(id.size()), log.c_str());
    TEST_ASSERT_TRUE(isJson(id[0]));
    TEST_ASSERT_TRUE(contains(id[0], "\"vehicle\":\"copter\""));
    TEST_ASSERT_TRUE(contains(id[0], "\"config\":\"\",\"plane\":\"\""));   // без Project.h — пусто
    const auto live = linesWith(log, "@live ");
    TEST_ASSERT_TRUE(live.size() >= 2);
    TEST_ASSERT_TRUE_MESSAGE(isJson(live[0]), live[0].c_str());
    TEST_ASSERT_TRUE(contains(live[0], "\"state\":\"DISARMED\",\"mode\":\"ANGLE\""));
    TEST_ASSERT_TRUE(contains(live[0], "\"motors\":[0.000,0.000,0.000,0.000]"));
    TEST_ASSERT_TRUE(contains(live[0], "\"modes\":[1]"));

    // заармленному менять нельзя
    run(rc, 2.5);
    host("live");
    armWith(rc, Channels::ARM);
    TEST_ASSERT_TRUE(copterController.isArmed());
    host("set p_pitch 40");
    run(rc, 0.05);
    log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "@error {\"cmd\":\"set\",\"error\":\"только без ARM\"}"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "\"arm\":true"));
    host("live 0");
    rc.set(Channels::ARM, 1000);
    run(rc, 0.1);
    takeSerial();
}

int main()
{
    wireUpBoard();
    UNITY_BEGIN();
    RUN_TEST(test_boot_derives_tune_and_holds_escs_at_stop);
    RUN_TEST(test_arm_waits_for_mount_calibration);
    RUN_TEST(test_three_poses_then_arm_spins_motors_at_idle);
    RUN_TEST(test_link_loss_on_ground_disarms);
    RUN_TEST(test_console_tune_report_and_motor_test);
    RUN_TEST(test_cli_set_save_and_reboot);
    RUN_TEST(test_aux_moves_arm_and_modes);
    RUN_TEST(test_map_and_receiver_protocol_from_cli);
    RUN_TEST(test_status_and_tasks_like_betaflight);
    RUN_TEST(test_copter_build_from_cli);
    RUN_TEST(test_app_protocol_params_set_save);
    return UNITY_END();
}
