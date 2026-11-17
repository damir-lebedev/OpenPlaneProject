// ============================================================
// SITL: прошивка мультикоптера целиком (src/stm32/main_copter.cpp) на ПК.
//
// Та же склейка, что летает на Matek H743: драйверы, CLI, настройки,
// живые кадры для Studio — только железо заменено фейками нативных
// тестов (test/native/support*), а датчики — эмуляторами чипов
// (ICM-42688 на SPI1, DPS310 на I2C2, делитель батареи), которые
// показывают модель полёта test/native/helpers/QuadSim.h. Моторы —
// команды CopterOutputs прямо в модель (sitlOutputs()).
//
// Наружу — TCP 127.0.0.1:5761 (--port): поток USB-консоли платы как
// есть (Studio подключается как к COM-порту) плюс строки симулятора:
//   ← "\x02sim rc 1500,1500,1000,1500,1000,...\n" — каналы пульта, мкс
//     (≥ 4; нет 0.3 с — пульт пропал, как в жизни);
//   ← "\x02sim reset\n" — коптер на землю в точку старта;
//   ← "\x02sim wind <север> <восток> <порывы>\n" — ветер, м/с;
//   → "@sim {...}\n" ~30 раз в секунду: положение (NED, м), кватернион,
//     скорость, команды и обороты моторов, ARM, батарея, ветер.
// Время — как в жизни: 1 мс модели = 1 мс часов ПК.
//
// Настройки — во «флеше» файла --eeprom (по умолчанию sitl-eeprom.bin
// рядом с exe): переживают перезапуск. reboot/save в CLI — перезапуск
// процесса (родитель запускает себя заново с --child).
//
// Сборка: pio run -e sitl; запуск: .pio/build/sitl/program[.exe].
// ============================================================

#define BOARD_MATEK_H743
#define SENSOR_KIT SENSOR_KIT_MATEK_H743
#define VEHICLE_MULTIROTOR
#include "../stm32/main_copter.cpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "helpers/ChipEmulators.h"
#include "helpers/QuadSim.h"
#include "helpers/TestSupport.h"
#include "SitlHost.h"

namespace
{
    constexpr int EXIT_REBOOT = 3;
    constexpr uint32_t STEP_US = 1000;   // шаг модели — цикл прошивки 1 кГц

    World world;
    chips::Icm42688 imuChip;
    chips::Spl06 baroChip;   // DPS310 — те же регистры

    QuadSim* sim = nullptr;
    copter::CopterTune simTune;
    copter::CopterOutputs lastOut;

    uint64_t simUs = 0;            // время модели (часы фейка), мкс
    uint64_t wallStartUs = 0, fakeStartUs = 0;
    uint64_t lastRcFrameUs = 0, lastRcInputUs = 0, lastSimFrameUs = 0, lastStorageUs = 0, lastModelCheckUs = 0;
    bool haveRcInput = false;
    RcChannels rc;
    std::string inbox;             // строка "\x02..." от клиента, пока не пришёл '\n'
    bool inCommand = false;
    std::string outbox;            // вывод консоли до конца строки (чтобы @sim не врезался в середину)
    std::string bootLog;
    bool rebootRequested = false;
    std::string eepromPath = "sitl-eeprom.bin";
    unsigned savedFlushes = 0;

    // ---------- флеш в файл ----------

    void loadEeprom()
    {
        FILE* f = fopen(eepromPath.c_str(), "rb");
        if (!f) return;
        const size_t n = fread(fake::eeprom().flash, 1, sizeof(fake::eeprom().flash), f);
        fclose(f);
        std::printf("SITL: настройки из %s (%u байт)\n", eepromPath.c_str(), static_cast<unsigned>(n));
    }

    void saveEeprom()
    {
        if (fake::eeprom().flushes == savedFlushes) return;
        savedFlushes = fake::eeprom().flushes;
        FILE* f = fopen(eepromPath.c_str(), "wb");
        if (!f) return;
        fwrite(fake::eeprom().flash, 1, sizeof(fake::eeprom().flash), f);
        fclose(f);
    }

    // ---------- модель ----------

    void buildModel()
    {
        simTune = copterParams.modelTune();
        QuadPerturb perturb;
        perturb.motorSpread = 0.03;   // моторы чуть разные, как в жизни
        perturb.vibration = 0.6;
        delete sim;
        sim = new QuadSim(quadParamsFrom(simTune, copterParams.buildSpec().battery, perturb, 7), 1e6 / STEP_US);
    }

    // Описание коптера поменяли (Studio/CLI) — модель по новому, пока стоит на земле.
    void checkModel()
    {
        const copter::CopterTune& t = copterParams.modelTune();
        if (copterController.isArmed() || !sim->grounded()) return;
        if (t.frame.count == simTune.frame.count && t.massKg == simTune.massKg && t.prop.kT == simTune.prop.kT &&
            t.armM == simTune.armM && t.cells == simTune.cells)
            return;
        buildModel();
        std::printf("SITL: модель пересобрана под новое описание коптера\n");
    }

    void stepModel()
    {
        float motor[copter::MAX_MOTORS] = {};
        for (uint8_t i = 0; i < lastOut.count && i < copter::MAX_MOTORS; ++i) motor[i] = lastOut.motor[i];
        sim->setReversed(lastOut.reversed);
        sim->step(motor, lastOut.spinning);
    }

    void writeSensors()
    {
        float gyro[3], accel[3];
        sim->readImu(gyro, accel);
        // наружу модель отдаёт гироскоп в авиационных знаках — чипу нужен FLU
        const float gyroFlu[3] = { gyro[0], -gyro[1], -gyro[2] };
        imuChip.writeMotion(accel, gyroFlu, world.temperatureC);
        world.altitudeM = sim->readBaro();
        baroChip.update(world);
        fake::gpio().analogMv[PC0] = static_cast<uint32_t>(sim->readVolts() * 1000.0f / 11.0f);   // делитель 1:11
    }

    // ---------- связь ----------

    void sendText(const std::string& s)
    {
        if (sitlhost::connected()) sitlhost::send(reinterpret_cast<const uint8_t*>(s.data()), s.size());
    }

    void simCommand(const std::string& line)
    {
        if (line.rfind("rc ", 0) == 0)
        {
            const char* p = line.c_str() + 3;
            for (uint8_t ch = 0; ch < Config::RC_CHANNELS && *p; ++ch)
            {
                char* end = nullptr;
                const long v = std::strtol(p, &end, 10);
                if (end == p) break;
                rc.value[ch] = static_cast<uint16_t>(v < 800 ? 800 : (v > 2200 ? 2200 : v));
                p = (*end == ',') ? end + 1 : end;
            }
            haveRcInput = true;
            lastRcInputUs = fake::nowUs();
        }
        else if (line == "reset")
        {
            sim->reset();
        }
        else if (line.rfind("wind ", 0) == 0)
        {
            double n = 0, e = 0, g = 0;
            if (std::sscanf(line.c_str() + 5, "%lf %lf %lf", &n, &e, &g) >= 2) sim->setWind(n, e, g);
        }
    }

    // Байты клиента: "\x02sim ...\n" — симулятору, остальное — в USB-консоль прошивки.
    void receive()
    {
        if (sitlhost::acceptPending())
        {
            inbox.clear();
            inCommand = false;
            sendText(bootLog);
        }
        uint8_t buf[1024];
        for (int round = 0; round < 8; ++round)
        {
            const int n = sitlhost::receive(buf, sizeof(buf));
            if (n <= 0) break;
            std::string toFirmware;
            for (int i = 0; i < n; ++i)
            {
                const char c = static_cast<char>(buf[i]);
                if (!inCommand)
                {
                    if (c == '\x02')
                    {
                        inCommand = true;
                        inbox.clear();
                    }
                    else toFirmware.push_back(c);
                    continue;
                }
                if (c != '\n')
                {
                    if (inbox.size() < 512) inbox.push_back(c);
                    continue;
                }
                inCommand = false;
                if (inbox.rfind("sim ", 0) == 0) simCommand(inbox.substr(4));
                else toFirmware += "\x02" + inbox + "\n";
            }
            if (!toFirmware.empty()) Serial.pushRx(toFirmware);
        }
    }

    void transmit()
    {
        outbox += Serial.takeTx();
        if (!sitlhost::connected())
        {
            outbox.clear();
            return;
        }
        const size_t end = outbox.rfind('\n');
        if (end != std::string::npos)
        {
            sendText(outbox.substr(0, end + 1));
            outbox.erase(0, end + 1);
        }
        else if (outbox.size() > 4096)
        {
            sendText(outbox);
            outbox.clear();
        }
    }

    void sendSimFrame()
    {
        if (!sitlhost::connected()) return;
        double qw, qx, qy, qz;
        sim->attitude(qw, qx, qy, qz);
        const QuadSim::Vec p = sim->position(), v = sim->velocity(), w = sim->wind();
        char buf[768];
        int len = std::snprintf(buf, sizeof(buf),
                                "@sim {\"t\":%.3f,\"pos\":[%.3f,%.3f,%.3f],\"q\":[%.5f,%.5f,%.5f,%.5f],\"vel\":[%.2f,%.2f,%.2f],"
                                "\"arm\":%s,\"gnd\":%s,\"volts\":%.2f,\"amps\":%.1f,\"wind\":[%.1f,%.1f],\"rc\":%s,\"m\":[",
                                sim->now(), p.x, p.y, p.z, qw, qx, qy, qz, v.x, v.y, v.z, copterController.isArmed() ? "true" : "false",
                                sim->grounded() ? "true" : "false", sim->batteryVolts(), sim->totalCurrent(), w.x, w.y,
                                haveRcInput ? "true" : "false");
        const uint8_t n = simTune.frame.count;
        for (uint8_t i = 0; i < n && len < 700; ++i)
            len += std::snprintf(buf + len, sizeof(buf) - len, "%s%.3f", i ? "," : "", lastOut.spinning ? lastOut.motor[i] : 0.0f);
        len += std::snprintf(buf + len, sizeof(buf) - len, "],\"hz\":[");
        for (uint8_t i = 0; i < n && len < 740; ++i) len += std::snprintf(buf + len, sizeof(buf) - len, "%s%.0f", i ? "," : "", sim->rotorHz(i));
        std::snprintf(buf + len, sizeof(buf) - len, "]}\n");
        sendText(buf);
    }

    // ---------- такт: каждый цикл полётной задачи (чтение гироскопа) ----------

    void onImuRead()
    {
        const uint64_t now = fake::nowUs();
        while (simUs + STEP_US <= now)
        {
            stepModel();
            simUs += STEP_US;
        }
        writeSensors();

        // пульт: кадр iBUS ~каждые 7 мс, пока клиент шлёт каналы
        if (haveRcInput && now - lastRcInputUs > 300000) haveRcInput = false;
        if (haveRcInput && now - lastRcFrameUs >= 7000)
        {
            lastRcFrameUs = now;
            if (HardwareSerial* rx = fake::uartByRx(PC7)) rx->pushRx(ibusFrame(rc));
        }

        receive();
        transmit();
        if (now - lastSimFrameUs >= 33000)
        {
            lastSimFrameUs = now;
            sendSimFrame();
        }
        if (now - lastStorageUs >= 100000)
        {
            lastStorageUs = now;
            Stm32FlashStorage::instance().service();
            saveEeprom();
            if (rebootRequested)
            {
                std::printf("SITL: перезагрузка\n");
                sitlhost::shutdown();
                std::exit(EXIT_REBOOT);
            }
        }
        if (now - lastModelCheckUs >= 1000000)
        {
            lastModelCheckUs = now;
            checkModel();
        }

        // темп как в жизни: модель не обгоняет часы ПК
        for (;;)
        {
            const int64_t ahead = static_cast<int64_t>(now - fakeStartUs) - static_cast<int64_t>(sitlhost::monotonicUs() - wallStartUs);
            if (ahead < 1000) break;
            sitlhost::sleepUs(ahead > 3000 ? 2000 : 500);
        }
        // отстали больше чем на полсекунды (ПК был занят) — не догоняем рывком
        const int64_t behind = static_cast<int64_t>(sitlhost::monotonicUs() - wallStartUs) - static_cast<int64_t>(now - fakeStartUs);
        if (behind > 500000) wallStartUs += static_cast<uint64_t>(behind);
    }

    // Калибровка установки IMU (Matek требует её до ARM): три позы, как
    // на столе. Один раз — дальше она во «флеше» файла.
    void calibrateMount()
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
        imuChip.update(world);
        Stm32FlashStorage::instance().service();
    }

    std::string argValue(int argc, char** argv, const char* name, const char* fallback)
    {
        for (int i = 1; i + 1 < argc; ++i)
            if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
        return fallback;
    }

    bool hasArg(int argc, char** argv, const char* name)
    {
        for (int i = 1; i < argc; ++i)
            if (std::strcmp(argv[i], name) == 0) return true;
        return false;
    }
}

void sitlOutputs(const copter::CopterOutputs& out) { lastOut = out; }

int main(int argc, char** argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (!hasArg(argc, argv, "--child"))
    {
        // родитель: перезапускает прошивку после reboot, как питание платы
        for (;;)
        {
            const int code = sitlhost::runChild(argc, argv);
            if (code != EXIT_REBOOT) return code;
        }
    }

    const int port = std::atoi(argValue(argc, argv, "--port", "5761").c_str());
    eepromPath = argValue(argc, argv, "--eeprom", "sitl-eeprom.bin");
    if (!sitlhost::listen(static_cast<uint16_t>(port)))
    {
        std::printf("SITL: порт %d занят\n", port);
        return 2;
    }
    loadEeprom();

    imuChip.attach(SPI, static_cast<uint8_t>(PC15));
    imuChip.install();
    baroChip.attach(Wire, 0x76);
    baroChip.install();
    fake::gpio().analogMv[PC0] = 16800 / 11;
    imuChip.update(world);
    baroChip.update(world);

    setup();
    copterConsole.setRebootHook([]() { rebootRequested = true; });
    copterConsole.setBootloaderHook([]() { Serial.println("SITL: загрузчика (DFU) нет — это симулятор"); });
    bootLog = Serial.takeTx();
    const size_t junk = bootLog.find("FreeRTOS не запустился");
    if (junk != std::string::npos) bootLog.erase(junk);
    if (Config::IMU_MOUNT_CALIBRATION_REQUIRED && !imuSensor.isMountCalibrated())
    {
        calibrateMount();
        // первый запуск: «проверка не пройдена» из setup() уже не правда
        const size_t bad = bootLog.find("ICM42688: ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА");
        if (bad != std::string::npos) bootLog.erase(bad, bootLog.find('\n', bad) - bad + 1);
        bootLog += "SITL: установка IMU откалибрована тремя позами модели\r\n";
        Serial.takeTx();
    }
    savedFlushes = ~0u;
    saveEeprom();
    bootLog = "SITL: прошивка OpenPlane на ПК, модель полёта QuadSim\r\n" + bootLog;

    buildModel();
    imuChip.beforeRead([](uint8_t reg) {
        if (reg == 0x1D) onImuRead();
    });
    simUs = fakeStartUs = fake::nowUs();
    wallStartUs = sitlhost::monotonicUs();
    std::printf("SITL: слушаю 127.0.0.1:%d\n", port);

    const fake::TaskRecord* flight = fake::findTask("flight");
    if (!flight)
    {
        std::printf("SITL: задача flight не создана\n");
        return 1;
    }
    fake::runTask(*flight, 0);   // без ограничения: не возвращается
    return 0;
}
