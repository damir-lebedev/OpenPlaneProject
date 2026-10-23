#pragma once
#include <Arduino.h>

#include "copter/CopterController.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"

// ============================================================
// Консоль мультикоптера (USB / UART, 115200): клавиша — команда.
//
//   h, ?  — подсказка
//   s     — состояние: ARM, режим, почему не армится, углы, высота,
//           батарея, связь, загрузка цикла
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
// Всё, что крутит моторы или останавливает цикл, — только без ARM.
// ============================================================

class CopterConsole
{
public:
    CopterConsole(Stream& stream, copter::CopterController& controller, const copter::BuildSpec& spec, ImuSensor& imuSensor,
                  RcReceiver& receiver)
        : io(stream), ctl(controller), build(spec), imu(imuSensor), rc(receiver)
    {
    }

    void setBootloaderHook(void (*hook)()) { bootloaderHook = hook; }
    void setLoopLoad(uint32_t avgUs, uint32_t maxUs)
    {
        loopAvgUs = avgUs;
        loopMaxUs = maxUs;
    }

    void printHint() { io.println("Консоль коптера: h — команды, s — состояние, t — настройка, m — проверка моторов."); }

    // Каждый цикл: одна клавиша за раз, проверка моторов идёт по шагам.
    void update(uint32_t nowMs)
    {
        stepMotorTest(nowMs);
        if (live && nowMs - lastLiveMs >= 200)
        {
            lastLiveMs = nowMs;
            printLive();
        }
        if (!io.available()) return;
        const int key = io.read();
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
        io.println("o — калибровка установки платы, D — загрузчик DFU.");
    }

    void printStatus()
    {
        const copter::CopterStatus& s = ctl.getStatus();
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
                for (uint16_t bit = 1; bit <= copter::ARM_BLOCK_EXTERNAL; bit <<= 1)
                {
                    if (s.armBlockers & bit) io.printf("  - %s\n", copter::armBlockText(bit));
                }
            }
        }
        io.printf("Крен %.1f°, тангаж %.1f°, курс %.0f°; высота %.2f м, вертикальная %.2f м/с\n", s.roll, s.pitch, s.yaw,
                  s.altitude, s.climbRate);
        io.printf("Батарея %.2f В; связь %s (%s); цикл %lu мкс (пик %lu) из %lu\n", s.volts,
                  rc.isSignalLost() ? "НЕТ" : "есть", rc.getProtocolName(), static_cast<unsigned long>(loopAvgUs),
                  static_cast<unsigned long>(loopMaxUs), static_cast<unsigned long>(1000000.0f / ctl.getTune().loopHz));
        io.printf("Газ висения %.3f (расчёт %.3f), обороты %.0f Гц\n", s.hoverLearned, ctl.getTune().hoverThrust, s.rotationHz);
    }

    void printTune()
    {
        const copter::CopterTune& t = ctl.getTune();
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
        io.printf("Контур: срез %.1f Гц (рысканье %.1f), запас по фазе %.0f°, по усилению %.1f дБ\n", t.crossoverHzRP,
                  t.crossoverHzYaw, t.phaseMarginDeg, t.gainMarginDb);
        printGains("Крен   ", t.roll);
        printGains("Тангаж ", t.pitch);
        printGains("Рыск.  ", t.yaw);
        io.printf("Расходы: крен/тангаж %.0f..%.0f °/с, рысканье %.0f..%.0f °/с; наклон в ANGLE до %.0f°\n",
                  t.rates[0].centerDps, t.rates[0].maxDps, t.rates[2].centerDps, t.rates[2].maxDps, t.maxAngleDeg);
        io.printf("Холостые %.1f %%, линеаризация тяги %.2f\n", t.idle * 100.0f, t.thrustLinear);
        printWarnings(t.warnings);
    }

    void printGains(const char* axis, const copter::AxisGains& g)
    {
        io.printf("  %s P %.5f  I %.4f  D %.6f  FF %.6f\n", axis, g.p, g.i, g.d, g.ff);
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
    const copter::BuildSpec& build;
    ImuSensor& imu;
    RcReceiver& rc;
    void (*bootloaderHook)() = nullptr;
    uint32_t loopAvgUs = 0, loopMaxUs = 0;
    bool live = false;
    uint32_t lastLiveMs = 0;
    bool awaitingConfirm = false;
    bool testRunning = false;
    uint8_t testIndex = 0;
    uint32_t nextTestMs = 0;
};
