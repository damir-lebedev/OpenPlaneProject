// ============================================================
// TELEMETRY: статистика цикла, настройки и каналы лога, текстовое
// меню консоли, веб-дашборд (маршруты, JSON /api/status, команды
// через почтовый ящик) и OLED (U8g2 поверх II2CBus, задача ядра 0).
//
// Запуск: pio test -e native -f native/test_telemetry
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotModeSelector.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"
#include "telemetry/DebugConsole.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LogSettings.h"
#include "telemetry/LoopStats.h"
#include "telemetry/OledDisplay.h"
#include "telemetry/WebDashboardPage.h"
#include "telemetry/WebDebugServer.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    // Борт целиком, но с фейковыми датчиками и FakeBoard.
    struct Plane
    {
        FakeBoard board;
        FakeImu imu;
        FakeBaro baro;
        FakeMag mag;
        FakeGps gps;
        IBusReceiver receiver{ board.rcUart() };
        ControlMixer mixer;
        ThrottleManager throttle;
        FlightOutputs outputs{ board };
        Autopilot autopilot{ &imu, &baro, &mag, &gps };
        AutopilotModeSelector selector{ &autopilot };
        ArmingManager arming{ &autopilot };
        FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &selector };
        LoopStats stats;

        Plane()
        {
            outputs.begin();
            controller.begin();
            takeSerial();
        }

        void tick(const RcChannels& rc)
        {
            board.rc.push(ibusFrame(rc));
            fake::advanceMs(2);
            controller.update();
        }

        void arm()
        {
            RcChannels rc;
            tick(rc);
            rc.set(Channels::ARM, 2000);
            tick(rc);
        }
    };

    // Строки лога с данным префиксом из вывода Serial.
    std::vector<std::string> linesWith(const std::string& log, const char* prefix)
    {
        std::vector<std::string> out;
        size_t start = 0;
        while (start < log.size())
        {
            size_t end = log.find("\r\n", start);
            if (end == std::string::npos) end = log.size();
            const std::string line = log.substr(start, end - start);
            if (line.compare(0, strlen(prefix), prefix) == 0) out.push_back(line);
            start = end + 2;
        }
        return out;
    }

    // Минимальная проверка корректности JSON: скобки сбалансированы вне
    // строк, нет висящих запятых.
    bool looksLikeJson(const std::string& s)
    {
        int depth = 0;
        bool inString = false;
        char previous = 0;
        for (char c : s)
        {
            if (c == '"' && previous != '\\') inString = !inString;
            if (!inString)
            {
                if (c == '{' || c == '[') depth++;
                if (c == '}' || c == ']')
                {
                    if (previous == ',') return false;
                    depth--;
                }
                if (depth < 0) return false;
            }
            if (!inString && c != ' ') previous = c;
        }
        return depth == 0 && !inString && !s.empty() && s.front() == '{';
    }
}

// ------------------------------------------------------------
// LoopStats
// ------------------------------------------------------------

void test_loop_stats_publish_once_per_second()
{
    LoopStats stats;
    for (int i = 0; i < 500; ++i)            // такты в 0, 2, ... 998 мс
    {
        stats.record(i == 100 ? 1500 : 700);
        fake::advanceMs(2);
    }
    TEST_ASSERT_EQUAL_UINT32(0, stats.hz);   // окно ещё не закрыто

    stats.record(700);                       // 1000 мс — окно закрывается этим тактом
    TEST_ASSERT_EQUAL_UINT32(501, stats.hz);
    TEST_ASSERT_UINT32_WITHIN(2, 701, stats.avgUs);
    TEST_ASSERT_EQUAL_UINT32(1500, stats.maxUs);

    for (int i = 0; i < 500; ++i)            // следующее окно — счёт с нуля
    {
        fake::advanceMs(2);
        stats.record(900);
    }
    TEST_ASSERT_EQUAL_UINT32(500, stats.hz);
    TEST_ASSERT_EQUAL_UINT32(900, stats.avgUs);
    TEST_ASSERT_EQUAL_UINT32(900, stats.maxUs);

    TEST_ASSERT_EQUAL_UINT32(1500, stats.takePeakUs());
    TEST_ASSERT_EQUAL_UINT32(0, stats.takePeakUs());   // худшее — с прошлого чтения
}

// ------------------------------------------------------------
// LogSettings
// ------------------------------------------------------------

void test_log_settings_defaults_and_cycling()
{
    LogSettings s;
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::OnChange), static_cast<int>(s.mode(LogChannel::Status)));
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off), static_cast<int>(s.mode(LogChannel::Rc)));
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Periodic), static_cast<int>(s.mode(LogChannel::System)));
    TEST_ASSERT_EQUAL_UINT16(1000, s.periodMs());

    const uint8_t rc = static_cast<uint8_t>(LogChannel::Rc);
    s.cycleMode(rc);
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::OnChange), static_cast<int>(s.mode(rc)));
    s.cycleMode(rc);
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Periodic), static_cast<int>(s.mode(rc)));
    s.cycleMode(rc);
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off), static_cast<int>(s.mode(rc)));

    // SYS — только выкл/вкл.
    const uint8_t sys = static_cast<uint8_t>(LogChannel::System);
    s.cycleMode(sys);
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off), static_cast<int>(s.mode(sys)));
    s.cycleMode(sys);
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Periodic), static_cast<int>(s.mode(sys)));

    s.setAll(LogMode::OnChange);   // SYS не трогается
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::OnChange), static_cast<int>(s.mode(LogChannel::Gps)));
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Periodic), static_cast<int>(s.mode(sys)));
    s.setAll(LogMode::Off);
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off), static_cast<int>(s.mode(sys)));

    const uint16_t periods[] = { 2000, 200, 500, 1000 };
    for (uint16_t p : periods)
    {
        s.cyclePeriod();
        TEST_ASSERT_EQUAL_UINT16(p, s.periodMs());
    }

    TEST_ASSERT_EQUAL_STRING("выкл", LogSettings::modeName(LogMode::Off, false));
    TEST_ASSERT_EQUAL_STRING("при изменении", LogSettings::modeName(LogMode::OnChange, false));
    TEST_ASSERT_EQUAL_STRING("постоянно", LogSettings::modeName(LogMode::Periodic, false));
    TEST_ASSERT_EQUAL_STRING("вкл", LogSettings::modeName(LogMode::Periodic, true));
    TEST_ASSERT_EQUAL_STRING("?", LogSettings::modeName(static_cast<LogMode>(9), false));
}

void test_log_settings_persist_and_reject_foreign_data()
{
    LogSettings saved;
    saved.setAll(LogMode::Periodic);
    saved.cyclePeriod();
    saved.save();

    LogSettings loaded;
    loaded.load();
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Periodic), static_cast<int>(loaded.mode(LogChannel::Imu)));
    TEST_ASSERT_EQUAL_UINT16(2000, loaded.periodMs());

    // Неизвестный код режима в NVS -> режим по умолчанию для канала.
    {
        Preferences p;
        p.begin("debuglog", false);
        uint8_t modes[LogSettings::COUNT] = {};
        modes[static_cast<uint8_t>(LogChannel::Rc)] = 7;
        p.putBytes("modes", modes, sizeof(modes));
        p.end();
    }
    LogSettings sanitised;
    sanitised.load();
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off), static_cast<int>(sanitised.mode(LogChannel::Rc)));

    // Другая версия формата — настройки по умолчанию.
    {
        Preferences p;
        p.begin("debuglog", false);
        p.putUChar("v", 99);
        p.end();
    }
    LogSettings fresh;
    fresh.load();
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off), static_cast<int>(fresh.mode(LogChannel::Imu)));

    fake::nvs().failBegin = true;
    fresh.save();
    fresh.load();
    fake::nvs().failBegin = false;
}

// ------------------------------------------------------------
// DebugLogger
// ------------------------------------------------------------

void test_logger_prints_enabled_channels_then_only_changes()
{
    Plane plane;
    DebugLogger logger(plane.controller, &plane.autopilot, &plane.stats);
    logger.begin();

    fake::advanceMs(100);
    logger.update();
    std::string log = takeSerial();
    TEST_ASSERT_EQUAL(1u, linesWith(log, "STAT ").size());
    TEST_ASSERT_TRUE(contains(log, "STAT RX=LOST(нет кадров) ARM=NO MODE=MANUAL FLAPS=UP IMU=OK BARO=OK"));
    TEST_ASSERT_TRUE(contains(log, "SYS  loop: статистика ещё набирается"));
    TEST_ASSERT_EQUAL(0u, linesWith(log, "RC ").size());   // выкл по умолчанию

    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_EQUAL(0u, takeSerial().size());   // ничего не изменилось

    logger.update();   // раньше DEBUG_INTERVAL_MS — не проверяет
    plane.tick(RcChannels());
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "STAT RX=OK ARM=NO"));
}

void test_logger_channel_contents_with_all_sensors()
{
    Plane plane;
    plane.imu.data.roll = 1.25f;
    plane.imu.data.pitch = -0.4f;
    plane.imu.data.yaw = 123.0f;
    plane.imu.data.gyroX = 1.0f;
    plane.imu.data.accelZ = 1.0f;
    plane.baro.data.altitude = 12.3f;
    plane.baro.data.verticalSpeed = 0.5f;
    plane.mag.data.headingDegrees = 271.0f;
    plane.gps.data.fixType = 3;
    plane.gps.data.numSatellites = 9;
    plane.gps.data.latitude = 55.7558123;
    plane.gps.data.longitude = 37.6173456;
    plane.gps.data.groundSpeed = 14.2f;
    plane.gps.data.horizontalAccuracy = 1.5f;
    plane.stats.hz = 500;
    plane.stats.avgUs = 700;

    RcChannels rc;
    rc.set(Channels::AILERON, 1750).set(Channels::AUX_2, 1500);
    plane.tick(rc);

    DebugLogger logger(plane.controller, &plane.autopilot, &plane.stats);
    logger.getSettings().setAll(LogMode::Periodic);
    fake::advanceMs(100);
    logger.update();
    const std::string log = takeSerial();

    TEST_ASSERT_TRUE(contains(log, "STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK"));
    TEST_ASSERT_TRUE(contains(log, "RC   1:1750 2:1500 3:1000"));
    TEST_ASSERT_TRUE(contains(log, "OUT  AIL-L "));
    TEST_ASSERT_TRUE(contains(log, "ATT  R +1.2 P -0.4 Y 123.0"));
    TEST_ASSERT_TRUE(contains(log, "AP   STABILIZE want R +0.0 P +0.0 corr R -7 P +2"));
    TEST_ASSERT_TRUE(contains(log, "ALT  12.3 м  Vz +0.50 м/с"));
    TEST_ASSERT_TRUE(contains(log, "MAG  курс 271°"));
    // Координаты не теряют 6-й знак (раньше хранились во float).
    TEST_ASSERT_TRUE(contains(log, "GPS  fix=3 sats=9 lat 55.755812 lon 37.617346 v 14.2 м/с hacc 1.5 м"));
    TEST_ASSERT_TRUE(contains(log, "IMU  gyro +1.0 +0.0 +0.0 °/с  acc +0.00 +0.00 +1.00 g"));
    TEST_ASSERT_TRUE(contains(log, "SYS  loop 500 Hz, avg 700 us"));
    TEST_ASSERT_TRUE(contains(log, "heap 200 KB"));
}

void test_logger_messages_for_missing_and_silent_sensors()
{
    Plane plane;
    plane.imu.available = false;
    plane.baro.available = false;
    plane.mag.available = false;
    plane.gps.available = false;
    DebugLogger logger(plane.controller, &plane.autopilot);
    logger.getSettings().setAll(LogMode::Periodic);
    fake::advanceMs(100);
    logger.update();
    std::string log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "IMU=NO_RESPONSE BARO=NO_RESPONSE"));
    TEST_ASSERT_TRUE(contains(log, "ATT  IMU не отвечает"));
    TEST_ASSERT_TRUE(contains(log, "ALT  барометр не отвечает"));
    TEST_ASSERT_TRUE(contains(log, "MAG  компас не отвечает"));
    TEST_ASSERT_TRUE(contains(log, "GPS  GPS не отвечает"));
    TEST_ASSERT_TRUE(contains(log, "SYS  loop n/a"));

    plane.imu.available = true;
    plane.imu.preflightProblem = "IMU: самолёт двигали";
    logger.refresh();
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "IMU=CHECK_FAILED"));

    // Датчиков нет в сборке, автопилота нет.
    Autopilot bare;
    FlightController& fc = plane.controller;
    DebugLogger withoutSensors(fc, &bare);
    withoutSensors.getSettings().setAll(LogMode::Periodic);
    fake::advanceMs(100);
    withoutSensors.update();
    log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "IMU=NONE BARO=NONE"));
    TEST_ASSERT_TRUE(contains(log, "ATT  IMU нет в схеме"));
    TEST_ASSERT_TRUE(contains(log, "ALT  барометра нет в схеме"));
    TEST_ASSERT_TRUE(contains(log, "MAG  компаса нет в схеме"));
    TEST_ASSERT_TRUE(contains(log, "GPS  GPS нет в схеме"));

    DebugLogger noAutopilot(fc);
    noAutopilot.getSettings().setAll(LogMode::Periodic);
    fake::advanceMs(100);
    noAutopilot.update();
    log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "MODE=-"));
    TEST_ASSERT_TRUE(contains(log, "AP   нет"));
}

void test_logger_status_reports_failsafe_arm_and_flaps()
{
    Plane plane;
    plane.arm();
    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::FLAPS, 2000);
    for (int i = 0; i < 50; ++i) plane.tick(rc);   // закрылки в пути

    DebugLogger logger(plane.controller, &plane.autopilot);
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "ARM=YES MODE=MANUAL FLAPS=MOVING"));

    for (int i = 0; i < 600; ++i) plane.tick(rc);
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "FLAPS=DOWN"));

    rc.set(Channels::THROTTLE, 900);   // failsafe пульта
    plane.tick(rc);
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "RX=LOST(failsafe пульта) ARM=YES MODE=FAILSAFE_GLIDE"));
}

void test_logger_on_change_uses_deadband()
{
    Plane plane;
    DebugLogger logger(plane.controller, &plane.autopilot);
    logger.getSettings().setMode(static_cast<uint8_t>(LogChannel::Attitude), LogMode::OnChange);
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_EQUAL(1u, linesWith(takeSerial(), "ATT").size());

    plane.imu.data.roll = 0.3f;   // шум — меньше допуска 0.5°
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_EQUAL(0u, linesWith(takeSerial(), "ATT").size());

    plane.imu.data.roll = 0.8f;
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_TRUE(contains(takeSerial(), "ATT  R +0.8"));
}

void test_logger_periodic_period_and_system_line_every_10s()
{
    Plane plane;
    DebugLogger logger(plane.controller, &plane.autopilot, &plane.stats);
    logger.getSettings().setMode(static_cast<uint8_t>(LogChannel::Rc), LogMode::Periodic);
    fake::advanceMs(100);
    logger.update();
    takeSerial();

    int rcLines = 0, sysLines = 0;
    for (int i = 0; i < 100; ++i)   // 10 с
    {
        fake::advanceMs(100);
        logger.update();
        const std::string log = takeSerial();
        rcLines += static_cast<int>(linesWith(log, "RC ").size());
        sysLines += static_cast<int>(linesWith(log, "SYS").size());
    }
    TEST_ASSERT_EQUAL(10, rcLines);   // период 1 с
    TEST_ASSERT_EQUAL(1, sysLines);   // SYS — раз в 10 с
}

void test_logger_pause_and_suspend_silence_output_then_refresh()
{
    Plane plane;
    DebugLogger logger(plane.controller, &plane.autopilot);
    fake::advanceMs(100);
    logger.update();
    takeSerial();

    logger.setPaused(true);
    TEST_ASSERT_TRUE(logger.isPaused());
    plane.tick(RcChannels());
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_EQUAL(0u, takeSerial().size());

    logger.setPaused(false);
    logger.suspend(true);
    fake::advanceMs(100);
    logger.update();
    TEST_ASSERT_EQUAL(0u, takeSerial().size());

    logger.suspend(false);   // после меню — все включённые каналы заново
    fake::advanceMs(100);
    logger.update();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "STAT"));
    TEST_ASSERT_TRUE(contains(log, "SYS"));
}

// ------------------------------------------------------------
// DebugConsole
// ------------------------------------------------------------

namespace
{
    struct ConsoleRig
    {
        Plane plane;
        DebugLogger logger{ plane.controller, &plane.autopilot };
        DebugConsole console{ plane.controller, plane.outputs, plane.autopilot, logger };

        std::string type(const char* keys)
        {
            Serial.pushRx(std::string(keys));
            console.update();
            return takeSerial();
        }
    };
}

void test_console_hotkeys_run_actions()
{
    ConsoleRig c;
    c.console.printHint();
    TEST_ASSERT_TRUE(contains(takeSerial(), "Консоль: h — меню"));

    TEST_ASSERT_TRUE(contains(c.type("z"), "Консоль: h — меню"));   // неизвестная — подсказка
    TEST_ASSERT_EQUAL(0u, c.type("\r\n").size());

    TEST_ASSERT_TRUE(contains(c.type("s"), "FakeIMU status"));
    c.type("i");
    TEST_ASSERT_EQUAL(1u, c.plane.imu.calibrations);
    c.type("o");
    TEST_ASSERT_EQUAL(1u, c.plane.imu.orientationCalibrations);
    c.type("m");
    TEST_ASSERT_EQUAL(1u, c.plane.mag.calibrations);
    TEST_ASSERT_TRUE(contains(c.type("p"), "Выходы: GPIO -> измерено / ожидается"));

    TEST_ASSERT_TRUE(contains(c.type(" "), "Лог: пауза"));
    TEST_ASSERT_TRUE(c.logger.isPaused());
    TEST_ASSERT_TRUE(contains(c.type(" "), "Лог: продолжен"));
}

void test_console_blocks_calibrations_while_armed()
{
    ConsoleRig c;
    c.plane.arm();
    takeSerial();
    for (const char* key : { "i", "o", "m", "p" })
    {
        TEST_ASSERT_TRUE(contains(c.type(key), "недоступна, пока заармлено"));
    }
    TEST_ASSERT_EQUAL(0u, c.plane.imu.calibrations);
    TEST_ASSERT_TRUE(contains(c.type("s"), "FakeIMU status"));   // статус — можно
}

void test_console_main_menu_navigation()
{
    ConsoleRig c;
    std::string out = c.type("h");
    TEST_ASSERT_TRUE(contains(out, "OpenPlane · консоль"));
    TEST_ASSERT_TRUE(contains(out, "Калибровка установки IMU"));

    TEST_ASSERT_TRUE(contains(c.type("?"), "OpenPlane · консоль"));   // неизвестная — перерисовка
    TEST_ASSERT_TRUE(contains(c.type(" "), "лог на паузе"));
    TEST_ASSERT_TRUE(contains(c.type(" "), "пробел — пауза лога"));
    TEST_ASSERT_TRUE(contains(c.type("0"), "Меню закрыто"));

    const char* actions[] = { "2", "3", "4", "5", "6" };
    for (const char* key : actions)
    {
        c.type("h");
        TEST_ASSERT_TRUE(contains(c.type(key), "Меню закрыто"));
    }
    TEST_ASSERT_EQUAL(1u, c.plane.imu.calibrations);
    TEST_ASSERT_EQUAL(1u, c.plane.imu.orientationCalibrations);
    TEST_ASSERT_EQUAL(1u, c.plane.mag.calibrations);

    c.type("h");
    TEST_ASSERT_TRUE(contains(c.type("q"), "Меню закрыто"));
    c.type("h");
    TEST_ASSERT_TRUE(contains(c.type("1"), "Лог: что выводить"));
    TEST_ASSERT_TRUE(contains(c.type("0"), "OpenPlane · консоль"));   // назад в главное
    TEST_ASSERT_TRUE(contains(c.type("h"), "Меню закрыто"));
}

void test_console_log_menu_edits_and_saves_when_disarmed()
{
    ConsoleRig c;
    std::string out = c.type("l");
    TEST_ASSERT_TRUE(contains(out, "1  STAT"));
    TEST_ASSERT_TRUE(contains(out, "s  SYS"));
    TEST_ASSERT_TRUE(contains(out, "сохранится при выходе из меню"));

    c.type("2");   // RC: выкл -> при изменении
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::OnChange),
                      static_cast<int>(c.logger.getSettings().mode(LogChannel::Rc)));
    c.type("s");   // SYS: вкл -> выкл
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off),
                      static_cast<int>(c.logger.getSettings().mode(LogChannel::System)));
    TEST_ASSERT_TRUE(contains(c.type("p"), "период для \"постоянно\": 2.0 с"));
    c.type("a");
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::OnChange),
                      static_cast<int>(c.logger.getSettings().mode(LogChannel::Gps)));
    c.type("x");
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::Off),
                      static_cast<int>(c.logger.getSettings().mode(LogChannel::Status)));
    c.type("d");
    TEST_ASSERT_EQUAL(static_cast<int>(LogMode::OnChange),
                      static_cast<int>(c.logger.getSettings().mode(LogChannel::Status)));
    TEST_ASSERT_TRUE(contains(c.type("?"), "Лог: что выводить"));   // неизвестная — перерисовка
    TEST_ASSERT_EQUAL(0u, fake::nvs().spaces.count("debuglog"));    // пока меню открыто — не пишем

    out = c.type("l");   // закрыть меню
    TEST_ASSERT_TRUE(contains(out, "Меню закрыто"));
    TEST_ASSERT_TRUE(contains(out, "Настройки лога сохранены."));
    TEST_ASSERT_EQUAL(1u, fake::nvs().spaces.count("debuglog"));
}

void test_console_defers_saving_until_disarm()
{
    ConsoleRig c;
    c.plane.arm();
    std::string out = c.type("l");
    TEST_ASSERT_TRUE(contains(out, "сохранится после DISARM"));
    c.type("3");
    TEST_ASSERT_FALSE(contains(c.type("h"), "сохранены"));   // заармлено — флеш не трогаем
    TEST_ASSERT_EQUAL(0u, fake::nvs().spaces.count("debuglog"));

    RcChannels rc;   // DISARM
    c.plane.tick(rc);
    TEST_ASSERT_TRUE(contains(c.type(""), "Настройки лога сохранены."));
}

void test_console_reports_sensors_missing_from_build()
{
    Plane plane;
    Autopilot bare;
    DebugLogger logger(plane.controller, &bare);
    DebugConsole console(plane.controller, plane.outputs, bare, logger);
    Serial.pushRx("iom");
    console.update();
    const std::string out = takeSerial();
    TEST_ASSERT_TRUE(contains(out, "Консоль: IMU не выбран в SensorSelection.h"));
    TEST_ASSERT_TRUE(contains(out, "Консоль: компас не выбран в SensorSelection.h"));
    Serial.pushRx("s");
    console.update();
    TEST_ASSERT_EQUAL(0u, takeSerial().size());   // статуса нет — датчиков нет
}

// ------------------------------------------------------------
// WebDebugServer
// ------------------------------------------------------------

void test_web_server_starts_access_point_and_task()
{
    Plane plane;
    WebDebugServer server(plane.controller, &plane.autopilot);
    TEST_ASSERT_TRUE(server.begin());
    TEST_ASSERT_FALSE(fake::wifi().persistent);   // без записи во флеш
    TEST_ASSERT_EQUAL(WIFI_AP, fake::wifi().mode);
    TEST_ASSERT_EQUAL_STRING(Config::WIFI_AP_SSID, fake::wifi().ssid.c_str());
    TEST_ASSERT_EQUAL_STRING(Config::WIFI_AP_PASSWORD, fake::wifi().password.c_str());
    TEST_ASSERT_TRUE(contains(takeSerial(), "http://192.168.4.1"));

    const fake::TaskRecord* task = fake::findTask("web");
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_EQUAL(0, task->core);
    TEST_ASSERT_EQUAL_UINT32(8192, task->stackDepth);
    fake::runTask(*task, 3);   // три прохода бесконечного цикла задачи
}

void test_web_server_reports_failed_access_point()
{
    Plane plane;
    fake::wifi().softApResult = false;
    WebDebugServer server(plane.controller, &plane.autopilot);
    TEST_ASSERT_FALSE(server.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "FAILED"));
    TEST_ASSERT_NULL(fake::findTask("web"));
}

namespace
{
    struct WebRig
    {
        Plane plane;
        WebDebugServer server;
        WebServer* http = nullptr;

        explicit WebRig(bool withAutopilot = true)
            : server(plane.controller, withAutopilot ? &plane.autopilot : nullptr)
        {
            http = fake::webServers().back();   // создан внутри WebDebugServer
            server.begin();
            takeSerial();
        }

        const WebServer::Response& get(const char* uri)
        {
            http->request(HTTP_GET, uri);
            return http->lastResponse();
        }

        const WebServer::Response& post(const char* uri, const char* body)
        {
            http->request(HTTP_POST, uri, body);
            return http->lastResponse();
        }
    };
}

void test_web_root_serves_dashboard_page()
{
    WebRig web;
    const WebServer::Response& r = web.get("/");
    TEST_ASSERT_EQUAL(200, r.code);
    TEST_ASSERT_EQUAL_STRING("text/html", r.contentType.c_str());
    TEST_ASSERT_EQUAL_STRING(WebDashboardPage::HTML, r.body.c_str());
    TEST_ASSERT_TRUE(contains(r.body, "fetch('/api/status')"));

    TEST_ASSERT_EQUAL(404, web.get("/nope").code);
}

void test_web_status_json_with_all_sensors()
{
    WebRig web;
    web.plane.imu.data.roll = 1.5f;
    web.plane.baro.data.altitude = 3.25f;
    web.plane.mag.data.headingDegrees = 90.0f;
    web.plane.gps.data.latitude = 55.123456;
    web.plane.gps.data.fixType = 3;
    RcChannels rc;
    rc.set(Channels::AILERON, 1600);
    web.plane.tick(rc);

    const WebServer::Response& r = web.get("/api/status");
    TEST_ASSERT_EQUAL(200, r.code);
    TEST_ASSERT_EQUAL_STRING("application/json", r.contentType.c_str());
    const std::string& json = r.body;
    TEST_ASSERT_TRUE(looksLikeJson(json));
    TEST_ASSERT_TRUE(contains(json, "\"rc\":[1600,1500,1000,1500,1000,1000,1000,1500,1500,1500]"));
    TEST_ASSERT_TRUE(contains(json, "\"armed\":false,\"failsafe\":false,\"flapsUs\":0"));
    TEST_ASSERT_TRUE(contains(json, "\"esc\":{\"us\":1000,\"attached\":true}"));
    TEST_ASSERT_TRUE(contains(json, "\"imu\":{\"attached\":true,\"available\":true,\"roll\":1.50"));
    TEST_ASSERT_TRUE(contains(json, "\"baro\":{\"attached\":true,\"available\":true,\"altitude\":3.25"));
    TEST_ASSERT_TRUE(contains(json, "\"mag\":{\"attached\":true,\"available\":true,\"heading\":90.0}"));
    TEST_ASSERT_TRUE(contains(json, "\"gps\":{\"attached\":true,\"available\":true,\"fix\":3,\"numSV\":0,\"lat\":55.123456"));
    TEST_ASSERT_TRUE(contains(json, "\"autopilot\":{\"attached\":true,\"mode\":0,\"modeName\":\"MANUAL\""));
    TEST_ASSERT_TRUE(contains(json, "\"kpRoll\":5.000"));
}

void test_web_status_json_without_sensors_or_autopilot()
{
    WebRig web(false);
    const std::string json = web.get("/api/status").body;
    TEST_ASSERT_TRUE(looksLikeJson(json));
    TEST_ASSERT_TRUE(contains(json, "\"imu\":{\"attached\":false,\"available\":false}"));
    TEST_ASSERT_TRUE(contains(json, "\"gps\":{\"attached\":false,\"available\":false}"));
    TEST_ASSERT_TRUE(contains(json, "\"autopilot\":{\"attached\":false}"));

    WebRig silent;
    silent.plane.imu.available = false;
    const std::string json2 = silent.get("/api/status").body;
    TEST_ASSERT_TRUE(contains(json2, "\"imu\":{\"attached\":true,\"available\":false}"));
}

void test_web_set_mode_goes_through_mailbox()
{
    WebRig web;
    TEST_ASSERT_EQUAL(400, web.post("/api/setmode", nullptr).code);
    TEST_ASSERT_TRUE(contains(web.http->lastResponse().body, "no data"));
    TEST_ASSERT_EQUAL(400, web.post("/api/setmode", "{\"mode\":9}").code);
    TEST_ASSERT_EQUAL(400, web.post("/api/setmode", "{\"speed\":1}").code);

    TEST_ASSERT_EQUAL(200, web.post("/api/setmode", "{\"mode\":3}").code);
    TEST_ASSERT_EQUAL(MODE_MANUAL, web.plane.autopilot.getMode());   // ещё не применено
    web.server.applyPendingCommands();
    TEST_ASSERT_EQUAL(MODE_ALT_HOLD, web.plane.autopilot.getMode());
    TEST_ASSERT_EQUAL(0, fake::tasks().criticalDepth);                // спинлок отпущен

    web.server.applyPendingCommands();   // почтовый ящик пуст — ничего
    TEST_ASSERT_EQUAL(MODE_ALT_HOLD, web.plane.autopilot.getMode());
}

// JSON с пробелами — как в примерах документации и у curl — раньше
// отклонялся (setmode) или молча игнорировался (setpid).
void test_web_accepts_whitespace_and_exponent_numbers()
{
    WebRig web;
    TEST_ASSERT_EQUAL(200, web.post("/api/setmode", "{ \"mode\" : 1 }").code);
    web.server.applyPendingCommands();
    TEST_ASSERT_EQUAL(MODE_STABILIZE, web.plane.autopilot.getMode());

    TEST_ASSERT_EQUAL(200, web.post("/api/setpid", "{\"kpRoll\": 2.5,\n \"kiRoll\":1e-7, \"kdPitch\":-0.25}").code);
    web.server.applyPendingCommands();
    TEST_ASSERT_TRUE(contains(takeSerial(), "PID обновлены"));
    TEST_ASSERT_EQUAL_FLOAT(2.5f, web.plane.autopilot.getRollPid().getKp());
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 1e-7f, web.plane.autopilot.getRollPid().getKi());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, web.plane.autopilot.getRollPid().getKd());    // не указан — прежний
    TEST_ASSERT_EQUAL_FLOAT(5.0f, web.plane.autopilot.getPitchPid().getKp());
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, web.plane.autopilot.getPitchPid().getKd());

    // Ключ без двоеточия или без числа — значение не меняется.
    web.post("/api/setpid", "{\"kpPitch\" 7, \"kiPitch\":\"x\"}");
    web.server.applyPendingCommands();
    TEST_ASSERT_EQUAL_FLOAT(5.0f, web.plane.autopilot.getPitchPid().getKp());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, web.plane.autopilot.getPitchPid().getKi());
    web.post("/api/setpid", "{\"kdRoll\":");
    web.server.applyPendingCommands();
    TEST_ASSERT_EQUAL_FLOAT(0.5f, web.plane.autopilot.getRollPid().getKd());
}

void test_web_commands_need_autopilot()
{
    WebRig web(false);
    TEST_ASSERT_EQUAL(503, web.post("/api/setmode", "{\"mode\":1}").code);
    TEST_ASSERT_EQUAL(503, web.post("/api/setpid", "{\"kpRoll\":1}").code);
    web.server.applyPendingCommands();   // без автопилота — ничего
    TEST_PASS();
}

// ------------------------------------------------------------
// OledDisplay
// ------------------------------------------------------------

namespace
{
    struct OledRig
    {
        Plane plane;
        TwoWire wire{ 8 };
        Esp32I2CBus bus{ wire, 1, 2 };
        fake::RegisterMapDevice screen;
        OledDisplay oled{ plane.controller, &plane.autopilot, plane.stats };
        U8G2* display = fake::displays().back();   // экран внутри OledDisplay

        OledRig() { bus.begin(); }
    };

    // Один проход задачи OLED; строки отправленного кадра.
    std::vector<std::string> drawFrame(const U8G2& display)
    {
        const fake::TaskRecord* task = nullptr;
        for (const fake::TaskRecord& t : fake::tasks().created)
        {
            if (t.name == "oled") task = &t;   // последняя созданная задача экрана
        }
        TEST_ASSERT_NOT_NULL(task);
        fake::runTask(*task, 1);
        std::vector<std::string> lines;
        for (const U8G2::DrawnText& t : display.lastFrame()) lines.push_back(t.text);
        return lines;
    }
}

void test_oled_without_bus_or_screen_is_disabled()
{
    Plane plane;
    OledDisplay oled(plane.controller, &plane.autopilot, plane.stats);
    TEST_ASSERT_FALSE(oled.begin(nullptr));

    TwoWire wire(8);
    Esp32I2CBus bus(wire, 1, 2);
    bus.begin();
    TEST_ASSERT_FALSE(oled.begin(&bus));
    TEST_ASSERT_TRUE(contains(takeSerial(), "OLED: не отвечает"));
    TEST_ASSERT_NULL(fake::findTask("oled"));
}

void test_oled_sends_bytes_over_bus_and_draws_status()
{
    OledRig rig;
    rig.wire.attach(0x3C, &rig.screen);
    TEST_ASSERT_TRUE(rig.oled.begin(&rig.bus));
    TEST_ASSERT_TRUE(contains(takeSerial(), "OLED: подключён (SSD1306)"));
    TEST_ASSERT_FALSE(rig.screen.writes.empty());   // инициализация ушла по I2C на 0x3C
    TEST_ASSERT_EQUAL(0, rig.display->lastUnknownMessageResult());

    const fake::TaskRecord* task = fake::findTask("oled");
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_EQUAL(0, task->core);

    // Связи нет, датчики живые.
    rig.plane.imu.data.roll = 1.2f;
    rig.plane.imu.data.pitch = -0.4f;
    rig.plane.baro.data.altitude = 0.3f;
    rig.plane.baro.data.verticalSpeed = 0.1f;
    rig.plane.mag.data.headingDegrees = 123.0f;
    rig.plane.stats.hz = 500;
    rig.plane.stats.maxUs = 1100;
    std::vector<std::string> lines = drawFrame(*rig.display);
    TEST_ASSERT_EQUAL(6u, lines.size());
    TEST_ASSERT_EQUAL_STRING("RX LOST safe MAN", lines[0].c_str());
    TEST_ASSERT_EQUAL(1u, rig.display->lastBoxes().size());   // инверсия строки
    TEST_ASSERT_EQUAL_STRING("R  +1.2 P  -0.4", lines[1].c_str());
    TEST_ASSERT_EQUAL_STRING("Alt  +0.3 Vz +0.1", lines[2].c_str());
    TEST_ASSERT_EQUAL_STRING("H123 T1000 Y1500", lines[3].c_str());
    TEST_ASSERT_EQUAL_STRING("L1500 R1500 E1500", lines[4].c_str());
    TEST_ASSERT_EQUAL_STRING("Loop 500Hz max1100us", lines[5].c_str());

    // Связь есть, ARM, STABILIZE, закрылки.
    rig.plane.arm();
    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::AUX_2, 1500).set(Channels::FLAPS, 2000);
    for (int i = 0; i < 20; ++i) rig.plane.tick(rc);
    lines = drawFrame(*rig.display);
    TEST_ASSERT_EQUAL_STRING("RX ok ARM STAB FL", lines[0].c_str());
    TEST_ASSERT_EQUAL(0u, rig.display->lastBoxes().size());
}

void test_oled_mode_names_glide_and_missing_sensors()
{
    OledRig rig;
    rig.wire.attach(0x3C, &rig.screen);
    rig.oled.begin(&rig.bus);

    RcChannels rc;
    rig.plane.tick(rc);
    rig.plane.autopilot.setMode(MODE_AUTO_TAKEOFF);
    TEST_ASSERT_TRUE(contains(drawFrame(*rig.display)[0], "TKOFF"));
    rig.plane.autopilot.setMode(MODE_ALT_HOLD);
    TEST_ASSERT_TRUE(contains(drawFrame(*rig.display)[0], "ALT"));
    rig.plane.autopilot.setMode(static_cast<AutopilotMode>(7));
    TEST_ASSERT_TRUE(contains(drawFrame(*rig.display)[0], "?"));

    rig.plane.autopilot.setMode(MODE_MANUAL);
    rig.plane.arm();
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 900);   // потеря связи в воздухе
    rig.plane.tick(rc);
    TEST_ASSERT_EQUAL_STRING("RX LOST ARM GLIDE", drawFrame(*rig.display)[0].c_str());

    rig.plane.imu.available = false;
    rig.plane.baro.available = false;
    rig.plane.mag.available = false;
    const std::vector<std::string> lines = drawFrame(*rig.display);
    TEST_ASSERT_EQUAL_STRING("IMU --", lines[1].c_str());
    TEST_ASSERT_EQUAL_STRING("BARO --", lines[2].c_str());
    TEST_ASSERT_TRUE(contains(lines[3], "H---"));

    // Без автопилота — MAN, датчиков нет.
    OledDisplay bare(rig.plane.controller, nullptr, rig.plane.stats);
    const U8G2& bareDisplay = *fake::displays().back();
    bare.begin(&rig.bus);
    const std::vector<std::string> bareLines = drawFrame(bareDisplay);
    TEST_ASSERT_TRUE(contains(bareLines[0], "MAN"));
    TEST_ASSERT_EQUAL_STRING("IMU --", bareLines[1].c_str());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_loop_stats_publish_once_per_second);
    RUN_TEST(test_log_settings_defaults_and_cycling);
    RUN_TEST(test_log_settings_persist_and_reject_foreign_data);
    RUN_TEST(test_logger_prints_enabled_channels_then_only_changes);
    RUN_TEST(test_logger_channel_contents_with_all_sensors);
    RUN_TEST(test_logger_messages_for_missing_and_silent_sensors);
    RUN_TEST(test_logger_status_reports_failsafe_arm_and_flaps);
    RUN_TEST(test_logger_on_change_uses_deadband);
    RUN_TEST(test_logger_periodic_period_and_system_line_every_10s);
    RUN_TEST(test_logger_pause_and_suspend_silence_output_then_refresh);
    RUN_TEST(test_console_hotkeys_run_actions);
    RUN_TEST(test_console_blocks_calibrations_while_armed);
    RUN_TEST(test_console_main_menu_navigation);
    RUN_TEST(test_console_log_menu_edits_and_saves_when_disarmed);
    RUN_TEST(test_console_defers_saving_until_disarm);
    RUN_TEST(test_console_reports_sensors_missing_from_build);
    RUN_TEST(test_web_server_starts_access_point_and_task);
    RUN_TEST(test_web_server_reports_failed_access_point);
    RUN_TEST(test_web_root_serves_dashboard_page);
    RUN_TEST(test_web_status_json_with_all_sensors);
    RUN_TEST(test_web_status_json_without_sensors_or_autopilot);
    RUN_TEST(test_web_set_mode_goes_through_mailbox);
    RUN_TEST(test_web_accepts_whitespace_and_exponent_numbers);
    RUN_TEST(test_web_commands_need_autopilot);
    RUN_TEST(test_oled_without_bus_or_screen_is_disabled);
    RUN_TEST(test_oled_sends_bytes_over_bus_and_draws_status);
    RUN_TEST(test_oled_mode_names_glide_and_missing_sensors);
    return UNITY_END();
}
