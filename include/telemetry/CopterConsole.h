#pragma once
#include <Arduino.h>

#include "copter/CopterCli.h"
#include "copter/CopterController.h"
#include "copter/Params.h"
#include "copter/RcSetup.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "telemetry/CopterLink.h"
#include "telemetry/FirmwareIdentity.h"
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
//   D     — перезагрузка в загрузчик USB DFU
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
          link(stream, controller, paramsRef, cli, receiver)
    {
        cli.setVersion("OpenPlane copter / " OP_BOARD_ENV " / " OP_SOURCE_COMMIT);
    }

    // Куда CLI сохраняет настройки (save); без него — только до перезагрузки.
    void setStore(KeyValueStore* store) { cli.setStore(store); }
    bool cliActive() const { return cli.active(); }
    // Приложение поменяло настройки и не сохранило — ARM нельзя.
    bool appUnsaved() const { return link.hasUnsaved(); }

    // CliPort: вывод CLI — в этот же порт.
    void write(const char* text) override { io.print(text); }

    void setBootloaderHook(void (*hook)()) { bootloaderHook = hook; }
    // Вызывается после save и отмены в CLI и в приложении: приёмник,
    // выходы — по новым настройкам (main_copter.cpp).
    void setApplyHook(void (*hook)()) { applyHook = hook; }
    void setTaskStats(const TaskStats* stats) { tasks = stats; }
    void setRxInput(const copter::RxInput* input)
    {
        rxInput = input;
        link.setRxInput(input);
    }
    void setOutputs(const copter::CopterOutputs& out) { link.setOutputs(out); }
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
        const float cells = roundf(ctl.getTune().volts / 3.75f);
        const char* battery = s.volts < 1.0f                                                     ? "NOT PRESENT"
                              : cells >= 1.0f && s.volts / cells < ctl.getSettings().minCellVolts ? "LOW"
                                                                                                  : "OK";
        io.printf("Voltage: %ld * 0.01V (%.0fS battery - %s)\n", lroundf(s.volts * 100.0f), cells, battery);
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
        if (applyHook) applyHook();
    }

private:
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
            case 'o':
                if (refuseArmed()) return;
                io.println("Калибровка установки: «нос вверх» — передние моторы вверх, «правое крыло вниз» — правые моторы вниз.");
                imu.calibrateOrientation();
                break;
            case '#':
                if (refuseArmed()) return;
                cli.enter();
                break;
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
        io.println("o — калибровка установки платы, D — загрузчик DFU,");
        io.println("# — CLI как в Betaflight (set, get, diff, aux, map, profile, save; help — внутри).");
    }

    void printTune()
    {
        const copter::CopterTune& t = ctl.getTune();
        const copter::BuildSpec build = params.buildSpec();
        io.printf("Сборка: %s — %s, %s, стиль «%s»\n", build.id, build.title, copter::frameName(t.frame.type),
                  copter::flyStyleName(t.style));
        io.printf("Винт %.1fx%.1f, %u лоп.; мотор %.0f%02.0f %.0fKV; %uS %u мА·ч; масса %.0f г; рама %.0f мм%s\n",
                  build.prop.diameterIn, build.prop.pitchIn, build.prop.blades, build.motor.statorWidthMm,
                  build.motor.statorHeightMm, build.motor.kv, build.battery.cells, build.battery.capacityMah,
                  t.massKg * 1000.0f, build.wheelbaseMm, build.propsOut ? ", пропеллеры наружу" : "");
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
        io.printf("Холостые %.1f %%, линеаризация тяги %.2f\n", t.idle * 100.0f, t.thrustLinear);
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
    const TaskStats* tasks = nullptr;
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
