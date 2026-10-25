#pragma once
#include <Arduino.h>

#include "copter/CopterCli.h"
#include "copter/Battery.h"
#include "hal/SpiNorFlash.h"
#include "copter/CopterController.h"
#include "copter/GpsHome.h"
#include "copter/Params.h"
#include "copter/RcSetup.h"
#include "copter/Vtx.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "telemetry/CopterLink.h"
#include "telemetry/CopterMsp.h"
#include "telemetry/FirmwareIdentity.h"
#include "telemetry/IFlightRecorder.h"
#include "telemetry/TaskStats.h"

// ============================================================
// Консоль мультикоптера (USB / UART, 115200): клавиша — команда.
//
//   h, ?  — подсказка
//   s     — состояние, как `status` в Betaflight: загрузка цикла,
//           частоты, батарея, флаги запрета ARM кодами BF и словами,
//           ARM, режим, углы, высота, связь
//   t     — настройка, выведенная из описания коптера: тяга, висение,
//           обороты, фильтры, полоса контура, PID, расходы и
//           предупреждения (мотор не тянет, винты задевают…)
//   r     — каналы пульта
//   l     — живая строка каждые 0.2 с (вкл/выкл)
//   m     — проверка моторов: по очереди, на малом газе, с описанием,
//           где мотор и куда должен крутиться. ПРОПЕЛЛЕРЫ СНЯТЬ — консоль
//           просит подтвердить клавишей 'y'
//   i     — калибровка гироскопа (коптер неподвижен ~2 с)
//   o     — калибровка установки платы (три позы)
//   c     — калибровка компаса (15 с вращать коптер по всем осям)
//   D     — перезагрузка в загрузчик USB DFU
//   k     — чёрный ящик: состояние и полёты на карте; ПК выгружает их
//           командами "bb list", "bb get N" (STX), как у самолёта
//   #     — CLI как в Betaflight: set/get/diff/dump/aux/map/profile/
//           status/tasks/save (copter/CopterCli.h); пока он открыт, ARM
//           запрещён
// Команды приложения — STX (0x02), строка, '\n' (telemetry/CopterLink.h).
// Всё, что крутит моторы или останавливает цикл, — только без ARM.
// ============================================================

class CopterConsole : public copter::CliPort
{
public:
    static constexpr char STX = 0x02;

    CopterConsole(Stream& stream, copter::CopterController& controller, copter::CopterParams& paramsRef,
                  ImuSensor& imuSensor, RcReceiver& receiver)
        : io(stream), ctl(controller), params(paramsRef), imu(imuSensor), rc(receiver), cli(paramsRef, controller, *this),
          link(stream, controller, paramsRef, cli, receiver), msp(controller, paramsRef, cli)
    {
        cli.setVersion("OpenPlane copter / " OP_BOARD_ENV " / " OP_SOURCE_COMMIT);
    }

    // Куда CLI сохраняет настройки (save); без него — только до перезагрузки.
    void setStore(KeyValueStore* store) { cli.setStore(store); }
    bool cliActive() const { return cli.active(); }
    // Приложение поменяло настройки и не сохранило — ARM нельзя.
    bool appUnsaved() const { return link.hasUnsaved() || msp.hasUnsaved(); }
    // Конфигуратор BF запретил ARM (MSP_SET_ARMING_DISABLED) и ещё на связи — флаг MSP.
    bool mspBlocksArming(uint32_t nowMs) const { return msp.blocksArming(nowMs); }
    // MSP конфигуратора Betaflight (telemetry/CopterMsp.h): плата, датчики — main_copter.cpp.
    copter::CopterMsp& mspHandler() { return msp; }
    // MSP_REBOOT 0 — перезагрузка платы.
    void setRebootHook(void (*hook)()) { rebootHook = hook; }

    // CliPort: вывод CLI — в этот же порт.
    void write(const char* text) override { io.print(text); }

    void setBootloaderHook(void (*hook)()) { bootloaderHook = hook; }
    // Вызывается после save и отмены в CLI и в приложении: приёмник,
    // выходы — по новым настройкам (main_copter.cpp).
    void setApplyHook(void (*hook)()) { applyHook = hook; }
    void setTaskStats(const TaskStats* stats) { tasks = stats; }
    void setMotorTelemetry(const copter::IMotorTelemetry* t) { motorTelemetry = t; }
    // SPI-флеш ящика (F405) — для flash_info.
    void setFlashChip(const SpiNorFlash* f) { flashChip = f; }
    // Чёрный ящик: 'k' и команды ПК "bb …" (telemetry/CopterBlackBox.h).
    void setRecorder(IFlightRecorder* r) { recorder = r; }
    void setRxInput(const copter::RxInput* input)
    {
        rxInput = input;
        link.setRxInput(input);
        msp.setRx(input, rc.getState().data(), Config::RC_CHANNELS);
    }
    void setOutputs(const copter::CopterOutputs& out)
    {
        link.setOutputs(out);
        msp.setOutputs(out);
    }
    void setBattery(const copter::BatteryMonitor* monitor)
    {
        battery = monitor;
        link.setBattery(monitor);
        msp.setBattery(monitor);
    }
    void setVtx(const copter::VtxController* controller)
    {
        vtx = controller;
        link.setVtx(controller);
    }
    void setGps(const GpsSensor* sensor, const copter::GpsHome* home)
    {
        gps = sensor;
        gpsHome = home;
        link.setGps(sensor, home);
    }
    void setCompass(MagnetometerSensor* sensor) { compass = sensor; }
    // Курс носа, 0 — север (компас, GPS или гироскоп — main_copter.cpp).
    void setHeading(float deg)
    {
        headingDeg = deg;
        link.setHeading(deg);
    }
    void setLoopLoad(uint32_t avgUs, uint32_t maxUs, uint32_t hz)
    {
        loopAvgUs = avgUs;
        loopMaxUs = maxUs;
        loopHz = hz;
        link.setLoop(avgUs, hz);
    }

    // Загрузка цикла, %: среднее время к периоду.
    uint32_t cpuPercent() const
    {
        const float periodUs = 1000000.0f / ctl.getTune().loopHz;
        return static_cast<uint32_t>(loopAvgUs * 100.0f / periodUs);
    }

    void printHint()
    {
        io.println("Консоль коптера: h — команды, s — состояние, t — настройка, m — проверка моторов, # — CLI.");
    }

    // Каждый цикл: одна клавиша за раз, проверка моторов идёт по шагам.
    void update(uint32_t nowMs)
    {
        countRxFrames(nowMs);
        if (cli.active())
        {
            // строками, и вставка diff целиком: всё, что пришло, за один цикл
            for (int n = 0; n < 512 && io.available() && cli.active(); ++n) cli.feed(static_cast<char>(io.read()));
            // вышли из CLI посреди вставки (save в diff BF): хвост вставки — не клавиши
            if (!cli.active() && cli.takePastedTail()) drainUntilMs = nowMs + DRAIN_AFTER_CLI_MS;
            return;
        }
        if (static_cast<int32_t>(drainUntilMs - nowMs) > 0)
        {
            while (io.available()) io.read();
            return;
        }
        link.update(nowMs);
        stepMotorTest(nowMs);
        if (live && nowMs - lastLiveMs >= 200)
        {
            lastLiveMs = nowMs;
            printLive();
        }
        // строка команды приложения — сразу вся, что пришла
        for (int n = 0; n < 160 && hostLine && io.available(); ++n) collectHostCommand(static_cast<char>(io.read()), nowMs);
        // кадр MSP (конфигуратор Betaflight): начинается с '$' — весь, что пришёл
        for (int n = 0; n < 600 && !hostLine && io.available() && (mspParser.busy() || io.peek() == '$'); ++n)
        {
            if (mspParser.feed(static_cast<uint8_t>(io.read()))) answerMsp(nowMs);
        }
        if (hostLine || !io.available()) return;
        const int key = io.read();
        if (key == STX)
        {
            collectHostCommand(STX, nowMs);
            return;
        }
        if (awaitingConfirm)
        {
            awaitingConfirm = false;
            if (key == 'y' && !ctl.isArmed())
            {
                testIndex = 0;
                testRunning = true;
                nextTestMs = nowMs;
                io.println("Проверка моторов: по 2 с каждый. Любая клавиша — стоп.");
            }
            else
            {
                io.println("Проверка моторов отменена.");
            }
            return;
        }
        if (testRunning)
        {
            stopMotorTest();
            return;
        }
        handle(static_cast<char>(key));
    }

    bool motorTestRunning() const { return testRunning; }

    // CliPort: status — как в Betaflight и словами.
    void printStatus() override
    {
        const copter::CopterStatus& s = ctl.getStatus();
        const float periodUs = 1000000.0f / ctl.getTune().loopHz;
        io.printf("MCU %s, System Uptime: %lu seconds\n", OP_BOARD_ENV, static_cast<unsigned long>(millis() / 1000));
        io.printf("CPU:%lu%%, cycle time: %.0f, GYRO rate: %lu, RX rate: %lu, System rate: %lu\n",
                  static_cast<unsigned long>(cpuPercent()), periodUs, static_cast<unsigned long>(loopHz),
                  static_cast<unsigned long>(rxRateHz), static_cast<unsigned long>(loopHz));
        if (battery)
        {
            io.printf("Voltage: %ld * 0.01V (%uS battery - %s)\n", lroundf(battery->volts() * 100.0f),
                      battery->config().cells, copter::batteryStateName(battery->state()));
            if (battery->hasCurrent())
            {
                io.printf("Amperage: %.1f A, %.0f mAh drawn (%.0f %% left)\n", battery->amps(), battery->mahDrawn(),
                          battery->percentLeft());
            }
        }
        io.print("Arming disable flags:");
        for (uint32_t bit = 1; bit <= copter::ARM_BLOCK_LAST; bit <<= 1)
        {
            if (s.armBlockers & bit) io.printf(" %s", copter::armBlockCode(bit));
        }
        io.println();

        io.printf("Состояние: %s, режим %s%s\n", stateName(s.state), copter::copterModeName(s.mode),
                  s.airborne ? ", в воздухе" : "");
        if (!ctl.isArmed())
        {
            if (s.armBlockers == 0)
            {
                io.println("ARM: можно");
            }
            else
            {
                io.println("ARM нельзя:");
                for (uint32_t bit = 1; bit <= copter::ARM_BLOCK_LAST; bit <<= 1)
                {
                    if (s.armBlockers & bit) io.printf("  - %s: %s\n", copter::armBlockCode(bit), copter::armBlockText(bit));
                }
            }
        }
        io.printf("Крен %.1f°, тангаж %.1f°, курс %.0f°; высота %.2f м, вертикальная %.2f м/с\n", s.roll, s.pitch, s.yaw,
                  s.altitude, s.climbRate);
        io.printf("Батарея %.2f В; связь %s (%s); цикл %lu мкс (пик %lu) из %.0f\n", s.volts,
                  rc.isSignalLost() ? "НЕТ" : "есть", rc.getProtocolName(), static_cast<unsigned long>(loopAvgUs),
                  static_cast<unsigned long>(loopMaxUs), periodUs);
        io.printf("Газ висения %.3f (расчёт %.3f), обороты %.0f Гц\n", s.hoverLearned, ctl.getTune().hoverThrust, s.rotationHz);
        printVtxLine();
        printGpsLines();
    }

    // CliPort: vtx_info — что ответил видеопередатчик и его мощности.
    void printVtxInfo() override
    {
        printVtxLine();
        if (!vtx || !vtx->state().online) return;
        io.print("Мощности:");
        for (uint8_t l = 1; l <= vtx->levelCount(); ++l)
        {
            io.printf(" %u=%u мВт%s", l, vtx->levelMw(l), l > vtx->topAllowedLevel() ? " (выше потолка)" : "");
        }
        io.println();
    }

    // CliPort: dshot_telemetry_info — как в BF: eRPM, обороты, Гц и ошибки по моторам.
    void printDshotTelemetry() override
    {
        if (!motorTelemetry || !motorTelemetry->telemetryEnabled())
        {
            io.println("Dshot telemetry not enabled (dshot_bidir = OFF или регуляторы не DShot)");
            return;
        }
        const uint8_t poles = static_cast<uint8_t>(params.value(copter::PARAM_MOTOR_POLES));
        io.println("Motor    eRPM    RPM     Hz Invalid");
        io.println("=====   =====  =====  ===== =======");
        for (uint8_t i = 0; i < motorTelemetry->telemetryMotors(); ++i)
        {
            const uint32_t e = motorTelemetry->erpm(i);
            const float rpm = copter::dshot::erpmToRpm(e, poles);
            io.printf("%5u %7lu %6.0f %6.0f %6.2f%%%s\n", i, static_cast<unsigned long>(e), rpm, rpm / 60.0f,
                      motorTelemetry->telemetryErrorPercent(i), motorTelemetry->hasTelemetry(i) ? "" : "  (ответа не было)");
        }
    }

    // CliPort: flash_info — как в BF: микросхема и объём, затем состояние ящика и полёты.
    void printFlashInfo() override
    {
        if (flashChip)
        {
            if (flashChip->present())
            {
                io.printf("Flash sectors=%lu, sectorSize=%lu, pageSize=%lu, totalSize=%lu JEDEC ID=0x%06lx (%s)\n",
                          static_cast<unsigned long>(flashChip->size() / SpiNorFlash::SECTOR),
                          static_cast<unsigned long>(SpiNorFlash::SECTOR), static_cast<unsigned long>(SpiNorFlash::PAGE),
                          static_cast<unsigned long>(flashChip->size()), static_cast<unsigned long>(flashChip->jedecId()),
                          flashChip->vendor());
            }
            else
            {
                io.printf("Flash not present (JEDEC ID=0x%06lx)\n", static_cast<unsigned long>(flashChip->jedecId()));
            }
        }
        if (recorder)
        {
            recorder->printStatus(io);
            recorder->printFlights(io);
        }
        else if (!flashChip)
        {
            io.println("Flash not present");
        }
    }

    // CliPort: flash_erase — стереть все полёты (ждёт, как в BF: W25Q128 — до минуты).
    void flashErase() override
    {
        if (!recorder)
        {
            io.println("###ОШИБКА: чёрного ящика нет###");
            return;
        }
        io.println("Erasing, please wait ... ");
        io.println(recorder->eraseAll() ? "Done." : "###ОШИБКА: идёт запись или заармлен — не стёрто###");
    }

    // CliPort: tasks — части полётного цикла, как в Betaflight.
    void printTasks() override
    {
        io.println("Task list          rate/hz  max/us  avg/us maxload avgload");
        if (tasks)
        {
            for (uint8_t i = 0; i < tasks->size(); ++i)
            {
                const TaskStats::Task& t = tasks->task(i);
                io.printf("%02u - (%12s) %7lu %7lu %7lu %6.1f%% %6.1f%%\n", i, t.name, static_cast<unsigned long>(t.rateHz),
                          static_cast<unsigned long>(t.maxUs), static_cast<unsigned long>(t.avgUs), TaskStats::maxLoad(t),
                          TaskStats::avgLoad(t));
            }
        }
        io.printf("Цикл целиком: %lu Гц, %lu мкс в среднем (пик %lu) — загрузка %lu %%\n", static_cast<unsigned long>(loopHz),
                  static_cast<unsigned long>(loopAvgUs), static_cast<unsigned long>(loopMaxUs),
                  static_cast<unsigned long>(cpuPercent()));
    }

    // CliPort: настройки сохранены или отменены.
    void settingsApplied() override
    {
        link.settingsApplied();
        msp.settingsApplied();
        if (applyHook) applyHook();
    }

private:
    void answerMsp(uint32_t nowMs)
    {
        const ImuData& d = imu.getImuData();
        msp.setImu(d.gyroX, d.gyroY, d.gyroZ, d.accelX, d.accelY, d.accelZ);
        msp.setLoad(static_cast<uint16_t>(cpuPercent()));
        msp.setLinkQuality(rc.getLinkQuality());
        static uint8_t out[msp::MAX_PAYLOAD + 16];
        const size_t n = msp.handle(mspParser, out, sizeof(out), nowMs);
        if (n) io.write(out, n);
        uint8_t mode = 0;
        if (msp.takeReboot(mode))
        {
            io.flush();
            delay(50);
            if (mode == 1 && bootloaderHook) bootloaderHook();
            else if (mode == 0 && rebootHook) rebootHook();
        }
    }

    void handle(char key)
    {
        switch (key)
        {
            case 'h':
            case '?': printHelp(); break;
            case 's': printStatus(); break;
            case 't': printTune(); break;
            case 'r': printChannels(); break;
            case 'l':
                live = !live;
                io.println(live ? "Живая строка: вкл" : "Живая строка: выкл");
                break;
            case 'm':
                if (refuseArmed()) return;
                io.println("ПРОВЕРКА МОТОРОВ. СНИМИТЕ ПРОПЕЛЛЕРЫ! Батарея подключена, ARM выключен.");
                io.println("Продолжить — 'y', отмена — любая другая клавиша.");
                awaitingConfirm = true;
                break;
            case 'i':
                if (refuseArmed()) return;
                imu.calibrate();
                break;
            case 'c':
                if (refuseArmed()) return;
                if (!compass)
                {
                    io.println("Компаса нет (QMC5883L/P на I2C модуля GPS не ответил).");
                    return;
                }
                compass->calibrate();
                break;
            case 'o':
                if (refuseArmed()) return;
                io.println("Калибровка установки: «нос вверх» — передние моторы вверх, «правое крыло вниз» — правые моторы вниз.");
                imu.calibrateOrientation();
                break;
            case '#':
                if (refuseArmed()) return;
                cli.enter();
                break;
            case 'k':
                if (!recorder)
                {
                    io.println("Чёрного ящика на этой плате нет.");
                    return;
                }
                recorder->printStatus(io);
                recorder->printFlights(io);
                return;
            case 'D':
                if (refuseArmed() || !bootloaderHook) return;
                io.println("Перезагрузка в загрузчик USB DFU...");
                io.flush();
                bootloaderHook();
                break;
            case '\r':
            case '\n':
            case ' ': break;
            default: io.println("Неизвестная команда. h — список."); break;
        }
    }

    void printGpsLines()
    {
        if (gps && gpsHome)
        {
            const copter::GpsFix& f = gpsHome->fix();
            if (!gps->isAvailable())
            {
                io.println("GPS: не отвечает");
            }
            else if (!f.valid)
            {
                io.printf("GPS: нет 3D-фикса, %u спутн.\n", f.satellites);
            }
            else
            {
                io.printf("GPS: 3D, %u спутн., ±%.1f м, %.1f м/с; ", f.satellites, f.accuracyM, f.speedMs);
                if (gpsHome->hasHome())
                {
                    io.printf("дом: %.0f м, на %.0f°%s\n", gpsHome->distanceM(), gpsHome->bearingHomeDeg(),
                              gpsHome->homeSetInFlight() ? " (поставлен уже в полёте)" : "");
                }
                else
                {
                    io.println("дом — при ARM");
                }
            }
        }
        if (!compass)
        {
            io.println("Компас: нет");
            return;
        }
        io.printf("Компас: %s, курс %.0f°%s\n", compass->isAvailable() ? "есть" : "не отвечает", headingDeg,
                  compass->isCalibrated() ? "" : " — не откалиброван ('c')");
    }

    void printVtxLine()
    {
        if (!vtx) return;
        const copter::VtxState& v = vtx->state();
        if (!v.online)
        {
            io.println("VTX: не отвечает (провод управления, vtx_protocol)");
            return;
        }
        uint8_t band = 0, channel = 0;
        char where[24] = "";
        if (copter::vtxBandChannel(v.frequencyMhz, band, channel))
        {
            snprintf(where, sizeof(where), "%c%u, ", copter::vtxBandLetters()[band - 1], channel);
        }
        io.printf("VTX: %s%s, %s%u МГц, %u мВт (уровень %u из %u, потолок %u)%s\n", copter::vtxProtocolName(v.protocol),
                  v.protocol == copter::VtxProtocol::SMARTAUDIO ? (v.version == 21 ? " 2.1" : v.version == 2 ? " 2.0" : " 1.0") : "",
                  where, v.frequencyMhz, v.powerMw, vtx->effectiveLevel(), vtx->levelCount(),
                  vtx->levelMw(vtx->topAllowedLevel()), v.pit ? ", PIT" : "");
    }

    void collectHostCommand(char key, uint32_t nowMs)
    {
        if (key == STX)
        {
            hostLine = true;
            hostLength = 0;
            return;
        }
        if (key == '\r') return;
        if (key != '\n')
        {
            if (hostLength + 1 < sizeof(hostBuffer)) hostBuffer[hostLength++] = key;
            return;
        }
        hostLine = false;
        hostBuffer[hostLength] = '\0';
        if (strncmp(hostBuffer, "bb ", 3) == 0)
        {
            if (recorder) recorder->handleHostCommand(hostBuffer);
            else io.println("BB:ERR чёрного ящика на этой плате нет");
            return;
        }
        if (!link.handle(hostBuffer, nowMs)) io.printf("@error {\"cmd\":\"?\",\"error\":\"нет команды\"}\n");
    }

    // Кадров приёмника в секунду — для status.
    void countRxFrames(uint32_t nowMs)
    {
        if (nowMs - rxWindowMs < 1000) return;
        const uint32_t frames = rc.getGoodFrameCount();
        rxRateHz = (frames - rxFramesAtWindow) * 1000u / (nowMs - rxWindowMs);
        rxFramesAtWindow = frames;
        rxWindowMs = nowMs;
    }

    bool refuseArmed()
    {
        if (!ctl.isArmed()) return false;
        io.println("Только без ARM.");
        return true;
    }

    void printHelp()
    {
        io.println("s — состояние, t — настройка, r — каналы пульта, l — живая строка,");
        io.println("m — проверка моторов (без пропеллеров!), i — калибровка гироскопа,");
        io.println("o — калибровка установки платы, c — калибровка компаса, D — загрузчик DFU,");
        io.println("# — CLI как в Betaflight (set, get, diff, aux, map, profile, save; help — внутри),");
        io.println("k — чёрный ящик (состояние и полёты на карте).");
    }

    void printTune()
    {
        const copter::CopterTune& t = ctl.getTune();
        const copter::BuildSpec build = params.buildSpec();
        io.printf("Сборка: %s — %s, %s, стиль «%s»\n", build.id, build.title, copter::frameName(t.frame.type),
                  copter::flyStyleName(t.style));
        io.printf("Винт %.1fx%.1f, %u лоп.; мотор %.0f%02.0f %.0fKV; %uS %u мА·ч %s; масса %.0f г; рама %.0f мм%s\n",
                  build.prop.diameterIn, build.prop.pitchIn, build.prop.blades, build.motor.statorWidthMm,
                  build.motor.statorHeightMm, build.motor.kv, build.battery.cells, build.battery.capacityMah,
                  copter::chemistryName(build.battery.chemistry), t.massKg * 1000.0f, build.wheelbaseMm,
                  build.propsOut ? ", пропеллеры наружу" : "");
        io.printf("Тяговооружённость %.1f, висение на %.0f %% тяги, %.1f А на висении\n", t.thrustToWeight,
                  t.hoverThrust * 100.0f, t.hoverAmpsTotal);
        io.printf("Обороты: холостые %.0f Гц, висение %.0f, полный газ %.0f; мотор на полном — %.0f А, %.0f Вт\n",
                  t.rotHzIdle, t.rotHzHover, t.rotHzFull, t.fullAmpsPerMotor, t.fullWattsPerMotor);
        io.printf("Отклик мотора τ = %.0f мс; регулятор %.2f мс; цикл %.0f Гц\n", t.motorTauS * 1000.0f,
                  t.escDelayS * 1000.0f, t.loopHz);
        io.printf("Фильтры: гироскоп %.0f Гц, D %.0f Гц, рысканье %.0f Гц, режекторы от %.0f Гц (Q %.1f)\n", t.gyroLpfHz,
                  t.dtermLpfHz, t.yawLpfHz, t.minNotchHz, t.notchQ);
        io.printf("Контур: срез %.1f Гц (рысканье %.1f), запас по фазе %.0f°, по усилению %.1f дБ%s\n", t.crossoverHzRP,
                  t.crossoverHzYaw, t.phaseMarginDeg, t.gainMarginDb,
                  t.gainsFromModel ? "" : " — для PID из модели, а работают другие (запас не проверен)");
        const char* pidName = params.text(copter::PARAM_PROFILE_NAME, params.pidProfile());
        const char* rateName = params.text(copter::PARAM_RATEPROFILE_NAME, params.rateProfile());
        io.printf("PID: %s, профиль %u%s%s; расходы — профиль %u%s%s\n",
                  params.source() == copter::TuneSource::BETAFLIGHT ? "дефолты Betaflight 4.5" : "из модели",
                  params.pidProfile(), *pidName ? " " : "", pidName, params.rateProfile(), *rateName ? " " : "", rateName);
        printGains("Крен   ", t.roll, 0);
        printGains("Тангаж ", t.pitch, 1);
        printGains("Рыск.  ", t.yaw, 2);
        io.printf("Расходы: крен/тангаж %.0f..%.0f °/с, рысканье %.0f..%.0f °/с; наклон в ANGLE до %.0f°\n",
                  t.rates[0].centerDps, t.rates[0].maxDps, t.rates[2].centerDps, t.rates[2].maxDps, t.maxAngleDeg);
        io.printf("Холостые %.1f %%, линеаризация тяги %.2f, предел моторов %.0f %%\n", t.idle * 100.0f, t.thrustLinear,
                  t.motorOutputLimit * 100.0f);
        printWarnings(t.warnings);
    }

    void printGains(const char* axis, const copter::AxisGains& g, uint8_t a)
    {
        using copter::PidTerm;
        io.printf("  %s P %.5f  I %.4f  D %.6f  FF %.6f  | BF: P %.0f I %.0f D %.0f F %.0f\n", axis, g.p, g.i, g.d, g.ff,
                  copter::gainToBf(PidTerm::P, a, g.p), copter::gainToBf(PidTerm::I, a, g.i),
                  copter::gainToBf(PidTerm::D, a, g.d), copter::gainToBf(PidTerm::F, a, g.ff));
    }

    void printWarnings(uint16_t w)
    {
        if (w == 0)
        {
            io.println("Предупреждений нет.");
            return;
        }
        io.println("ПРЕДУПРЕЖДЕНИЯ:");
        if (w & copter::WARN_NO_LIFT_MARGIN) io.println("  - тяги почти нет (меньше 1.2 веса) — может не взлететь");
        if (w & copter::WARN_LOW_THRUST) io.println("  - тяги мало (меньше 1.8 веса) — вяло, ветер не перетянет");
        if (w & copter::WARN_MOTOR_OVERLOAD) io.println("  - мотор перегружен этим винтом на полном газе — перегрев");
        if (w & copter::WARN_PROP_OVERLAP) io.println("  - соседние винты задевают друг друга — рама мала для винтов");
        if (w & copter::WARN_HEAVY_HOVER) io.println("  - висит выше 60 % газа — запаса на манёвр мало");
        if (w & copter::WARN_VIBRATION_ALIAS) io.println("  - моторы крутятся быстрее половины частоты цикла — нужен цикл быстрее");
    }

    void printChannels()
    {
        const RcChannelState& st = rc.getState();
        io.print("Каналы:");
        for (uint8_t i = 0; i < Config::RC_CHANNELS; ++i) io.printf(" %u", st.get(i));
        io.println(rc.isSignalLost() ? " (связи нет)" : "");
        if (!rxInput) return;
        io.print("Режимы:");
        bool any = false;
        for (uint8_t b = 0; b < copter::BOX_COUNT; ++b)
        {
            if (!rxInput->isActive(b)) continue;
            const copter::BoxInfo* box = copter::findBox(b);
            if (box) io.printf(" %s", box->name);
            else io.printf(" %u", b);
            any = true;
        }
        io.println(any ? "" : " ни одного (ACRO)");
    }

    void printLive()
    {
        const copter::CopterStatus& s = ctl.getStatus();
        io.printf("%s %s R%.1f P%.1f Y%.0f alt %.2f vz %.2f thr %.3f %.2fV%s\n", stateName(s.state),
                  copter::copterModeName(s.mode), s.roll, s.pitch, s.yaw, s.altitude, s.climbRate, s.throttle, s.volts,
                  s.saturated ? " SAT" : "");
    }

    static const char* stateName(copter::CopterState st)
    {
        switch (st)
        {
            case copter::CopterState::ARMED: return "ARMED";
            case copter::CopterState::FAILSAFE: return "FAILSAFE";
            default: return "DISARMED";
        }
    }

    void stepMotorTest(uint32_t nowMs)
    {
        if (!testRunning || static_cast<int32_t>(nowMs - nextTestMs) < 0) return;
        if (ctl.isArmed() || testIndex >= ctl.getTune().frame.count)
        {
            stopMotorTest();
            return;
        }
        char where[96];
        copter::describeMotor(ctl.getTune().frame, testIndex, where, sizeof(where));
        io.printf("Мотор %u: %s — так и должен крутиться\n", testIndex + 1, where);
        ctl.startMotorTest(testIndex, 0.06f, 1.5f);
        ++testIndex;
        nextTestMs = nowMs + 2000;
    }

    void stopMotorTest()
    {
        ctl.stopMotorTest();
        testRunning = false;
        io.println("Проверка моторов окончена. Крутится не тот мотор — поменяйте порядок в CopterConfig::MOTOR_PINS;");
        io.println("не в ту сторону — направление в конфигураторе регулятора (провода не перепаивать).");
    }

    Stream& io;
    copter::CopterController& ctl;
    copter::CopterParams& params;
    ImuSensor& imu;
    RcReceiver& rc;
    copter::CopterCli cli;
    CopterLink link;
    const copter::RxInput* rxInput = nullptr;
    const copter::BatteryMonitor* battery = nullptr;
    const copter::IMotorTelemetry* motorTelemetry = nullptr;
    const SpiNorFlash* flashChip = nullptr;
    copter::CopterMsp msp;
    msp::Parser mspParser;
    void (*rebootHook)() = nullptr;
    static constexpr uint32_t DRAIN_AFTER_CLI_MS = 500;
    uint32_t drainUntilMs = 0;
    const copter::VtxController* vtx = nullptr;
    const GpsSensor* gps = nullptr;
    const copter::GpsHome* gpsHome = nullptr;
    MagnetometerSensor* compass = nullptr;
    float headingDeg = 0.0f;
    const TaskStats* tasks = nullptr;
    IFlightRecorder* recorder = nullptr;
    void (*bootloaderHook)() = nullptr;
    void (*applyHook)() = nullptr;
    uint32_t loopAvgUs = 0, loopMaxUs = 0, loopHz = 0;
    uint32_t rxRateHz = 0, rxFramesAtWindow = 0, rxWindowMs = 0;
    bool hostLine = false;
    char hostBuffer[128] = {};
    size_t hostLength = 0;
    bool live = false;
    uint32_t lastLiveMs = 0;
    bool awaitingConfirm = false;
    bool testRunning = false;
    uint8_t testIndex = 0;
    uint32_t nextTestMs = 0;
};
