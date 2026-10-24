#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "copter/Battery.h"
#include "copter/CopterController.h"
#include "copter/GpsHome.h"
#include "copter/Params.h"
#include "hal/ResetCause.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "telemetry/BlackBoxCore.h"
#include "telemetry/FirmwareIdentity.h"
#include "telemetry/LoopStats.h"

// ============================================================
// ЧЁРНЫЙ ЯЩИК МУЛЬТИКОПТЕРА — на SD-карту (STM32H743), тем же форматом и
// той же выгрузкой, что у самолёта (BlackBoxCore.h, BlackBoxFormat.h):
// tools/blackbox.py и вкладка «Ящик» в OpenPlane Studio читают оба.
//
// Что пишется:
//   QGYRO  каждый цикл (blackbox_sample_rate): гироскоп до и после
//          фильтров, уставки, время работы цикла — для настройки PID и
//          спектра шума;
//   QCTRL  тогда же: P, I, D, F по осям, команды моторам 1–4, газ;
//   QMOT8  тогда же, если моторов больше четырёх: моторы 5–8;
//   QSTATE 50 Гц: углы, высота, стики, батарея, обороты, связь, режим,
//          флаги, запреты ARM;
//   GPS    каждое новое решение; SYS — 1 Гц: цикл, приёмник, сам ящик;
//   EVENT  ARM/DISARM, режим, failsafe, авария, связь, батарея, профили,
//          запрет ARM, GPS и дом, проверка моторов;
//   в начале полёта — схема, сборка, PID и фильтры, всё, что задано в CLI.
//
// Когда пишет (blackbox_mode, как в Betaflight): NORMAL — пока ARM,
// MOTOR_TEST — пока крутятся моторы (и проверка моторов на столе), ALWAYS —
// с включения. С предзаписью PREROLL_MS и дописью POSTROLL_MS. После
// перезагрузки из-за сбоя — сразу. blackbox_device = NONE — не пишет.
// ============================================================

class CopterBlackBox : public BlackBoxCore
{
public:
    enum class Mode : uint8_t { NORMAL, MOTOR_TEST, ALWAYS };

    static constexpr uint32_t PREROLL_MS = 2000;
    static constexpr uint32_t POSTROLL_MS = 2000;
    static constexpr uint32_t STATE_PERIOD_MS = 20;
    static constexpr uint32_t SYS_PERIOD_MS = 1000;

    CopterBlackBox(copter::CopterController& controller, copter::CopterParams& paramsRef, BlackBoxStorage& flightStorage,
                   LoopStats& stats, RcReceiver& receiver)
        : BlackBoxCore(flightStorage), ctl(controller), params(paramsRef), loopStats(stats), rx(receiver)
    {
    }

    void setBattery(const copter::BatteryMonitor* b) { battery = b; }
    void setGps(const GpsSensor* g, const copter::GpsHome* h)
    {
        gps = g;
        home = h;
    }
    void setSensors(const ImuSensor* i, const BarometerSensor* b)
    {
        imu = i;
        baro = b;
    }

    // Настройки blackbox_* (при включении и после save).
    void configure(bool on, Mode m, uint8_t sampleDivider)
    {
        enabled = on;
        mode = m;
        divider = sampleDivider ? sampleDivider : 1;
    }

    // Носитель, очередь, задача записи. false — места нет (карты или файла
    // BLACKBOX.BIN нет): ящик выключен, полёту это не мешает.
    bool begin(bool startTask = true)
    {
        resetReason = readResetCause();
        resetStartPending = isCrashReset(resetReason);
        const uint32_t size = Config::BLACKBOX_RING_STM32_BYTES;
        if (!beginCore(size, static_cast<uint8_t*>(malloc(size)))) return false;
        printStatus(Serial);
        if (resetStartPending)
        {
            Serial.printf("BlackBox: перезагрузка из-за сбоя (%s) — запись включается сразу\n", resetCauseName(resetReason));
        }
        if (startTask) startWriterTask();
        return true;
    }

    // Полётный цикл: после каждого такта. sens — датчики как пришли, rc —
    // пульт, out — что ушло на моторы, workUs — сколько длилась работа такта.
    void update(const copter::CopterSensors& sens, const copter::CopterRc& rc, const copter::CopterOutputs& out,
                uint32_t workUs)
    {
        if (state == State::Off) return;

        nowUs = micros();
        nowMs = millis();
        const bool armed = ctl.isArmed();
        armedFlag = armed;

        if (state != State::Stopping) detectEvents(rc);
        decideStartStop(armed, out.spinning);

        if (state == State::Idle) ring.trimOlderThan(nowUs, PREROLL_MS * 1000UL);
        if (state != State::Stopping && enabled) sample(sens, rc, out, workUs);
        cycle++;
        wakeWriter();
    }

    Mode getMode() const { return mode; }
    bool isEnabled() const { return enabled; }

private:
    copter::CopterController& ctl;
    copter::CopterParams& params;
    LoopStats& loopStats;
    RcReceiver& rx;
    const copter::BatteryMonitor* battery = nullptr;
    const GpsSensor* gps = nullptr;
    const copter::GpsHome* home = nullptr;
    const ImuSensor* imu = nullptr;
    const BarometerSensor* baro = nullptr;

    bool enabled = true;
    Mode mode = Mode::NORMAL;
    uint8_t divider = 1;

    ResetCause resetReason = ResetCause::Unknown;
    bool resetStartPending = false;
    uint32_t lastStateMs = 0, lastSysMs = 0;
    uint32_t lastGpsStamp = 0;
    uint32_t armRequestMs = 0;
    static constexpr uint32_t ARM_REFUSE_MS = 100;

    struct Seen
    {
        bool valid = false;
        bool armed = false;
        uint8_t mode = 0xFF;
        uint8_t state = 0xFF;
        bool crashed = false;
        bool linkLost = false;
        uint8_t battery = 0xFF;
        uint8_t pidProfile = 0xFF, rateProfile = 0xFF;
        bool armSwitch = false;
        uint8_t gpsFix = 0xFF;
        bool home = false;
        bool motorTest = false;
    } seen;

    bool vehicleArmed() const override { return ctl.isArmed(); }

    void onRecordingStart() override
    {
        const copter::CopterStatus& s = ctl.getStatus();
        event("состояние: ARM=%s MODE=%s RX=%s БАТАРЕЯ=%.2f В", ctl.isArmed() ? "YES" : "NO", copter::copterModeName(s.mode),
              rx.isSignalLost() ? "LOST" : "OK", static_cast<double>(battery ? battery->volts() : s.volts));
    }

    // ---------------- старт и остановка ----------------

    void decideStartStop(bool armed, bool spinning)
    {
        if (!enabled)
        {
            if (state == State::Recording) stop("blackbox_device = NONE");
            return;
        }
        if (state == State::Idle)
        {
            if (manualStartRequested) start("вручную", true, false);
            else if (resetStartPending) start("перезагрузка после сбоя", false, true);
            else if (mode == Mode::ALWAYS) start("blackbox_mode = ALWAYS", false, false);
            else if (mode == Mode::NORMAL && armed) start("ARM", false, false);
            else if (mode == Mode::MOTOR_TEST && spinning) start("моторы крутятся", false, false);
            manualStartRequested = false;
            manualStopRequested = false;
            resetStartPending = false;
            return;
        }
        if (state != State::Recording) return;

        if (manualStopRequested)
        {
            manualStopRequested = false;
            stop("вручную");
            return;
        }
        if (armed)
        {
            manualRecording = false;
            resetRecording = false;
        }
        if (manualRecording || mode == Mode::ALWAYS) return;

        const bool active = mode == Mode::MOTOR_TEST ? spinning : armed;
        disarmedSinceMs = active ? 0 : (disarmedSinceMs ? disarmedSinceMs : (nowMs ? nowMs : 1));
        const bool holdAfterReset = resetRecording && nowMs - recordingStartMs < Config::BLACKBOX_RESET_HOLD_MS;
        if (disarmedSinceMs && nowMs - disarmedSinceMs >= POSTROLL_MS && !holdAfterReset)
        {
            stop(mode == Mode::MOTOR_TEST ? "моторы остановились" : "DISARM");
        }
    }

    // ---------------- события ----------------

    static void blockerCodes(uint32_t mask, char* out, size_t size)
    {
        out[0] = '\0';
        for (uint32_t bit = 1; bit && bit <= copter::ARM_BLOCK_LAST; bit <<= 1)
        {
            if (!(mask & bit)) continue;
            if (out[0]) strncat(out, " ", size - strlen(out) - 1);
            strncat(out, copter::armBlockCode(bit), size - strlen(out) - 1);
        }
    }

    void detectEvents(const copter::CopterRc& rc)
    {
        const copter::CopterStatus& s = ctl.getStatus();
        const bool first = !seen.valid;
        seen.valid = true;
        const bool armed = ctl.isArmed();

        if (first || armed != seen.armed)
        {
            if (!first || armed) event(armed ? "ARM" : "DISARM");
            seen.armed = armed;
        }
        // пилот включил ARM, а коптер за ARM_REFUSE_MS не заармился — почему (один раз)
        if (rc.armSwitch && !seen.armSwitch && !armed) armRequestMs = nowMs ? nowMs : 1;
        if (armed || !rc.armSwitch) armRequestMs = 0;
        if (armRequestMs && nowMs - armRequestMs >= ARM_REFUSE_MS)
        {
            char codes[160];
            blockerCodes(s.armBlockers, codes, sizeof(codes));
            event("ARM отклонён: %s", codes[0] ? codes : "?");
            armRequestMs = 0;
        }
        seen.armSwitch = rc.armSwitch;
        if (static_cast<uint8_t>(s.mode) != seen.mode)
        {
            event("режим %s", copter::copterModeName(s.mode));
            seen.mode = static_cast<uint8_t>(s.mode);
        }
        if (static_cast<uint8_t>(s.state) != seen.state)
        {
            if (s.state == copter::CopterState::FAILSAFE) event("FAILSAFE: нет связи — выравнивание и снижение");
            else if (seen.state == static_cast<uint8_t>(copter::CopterState::FAILSAFE)) event("FAILSAFE: кончился");
            seen.state = static_cast<uint8_t>(s.state);
        }
        if (s.crashed != seen.crashed)
        {
            if (s.crashed) event("авария: переворот или потеря управления — моторы выключены");
            seen.crashed = s.crashed;
        }
        const bool lost = rx.isSignalLost();
        if (first || lost != seen.linkLost)
        {
            if (!first || lost) event(lost ? "связь: потеряна" : "связь: есть");
            seen.linkLost = lost;
        }
        if (battery && static_cast<uint8_t>(battery->state()) != seen.battery)
        {
            if (!first) event("батарея: %s, %.2f В", copter::batteryStateName(battery->state()), static_cast<double>(battery->volts()));
            seen.battery = static_cast<uint8_t>(battery->state());
        }
        if (params.pidProfile() != seen.pidProfile || params.rateProfile() != seen.rateProfile)
        {
            if (!first) event("профиль PID %u, профиль расходов %u", params.pidProfile() + 1u, params.rateProfile() + 1u);
            seen.pidProfile = params.pidProfile();
            seen.rateProfile = params.rateProfile();
        }
        if (gps && gps->isAvailable() && gps->getGpsData().fixType != seen.gpsFix)
        {
            seen.gpsFix = gps->getGpsData().fixType;
            event("GPS: фикс %u, спутников %u", seen.gpsFix, gps->getGpsData().numSatellites);
        }
        if (home && home->hasHome() != seen.home)
        {
            event(home->hasHome() ? "дом: поставлен" : "дом: сброшен");
            seen.home = home->hasHome();
        }
        if (ctl.motorTestActive() != seen.motorTest)
        {
            event(ctl.motorTestActive() ? "проверка моторов: старт" : "проверка моторов: конец");
            seen.motorTest = ctl.motorTestActive();
        }
        if (ring.dropped() != reportedDropped && nowMs - dropReportMs >= 1000)
        {
            event("очередь переполнена: потеряно записей %lu (blackbox_sample_rate = 1/2 — вдвое меньше)",
                  static_cast<unsigned long>(ring.dropped() - reportedDropped));
            reportedDropped = ring.dropped();
            dropReportMs = nowMs;
        }
    }

    // ---------------- снимки ----------------

    static int16_t i16(float value)
    {
        if (!(value == value)) return 0;   // NaN
        return static_cast<int16_t>(constrain(lroundf(value), -32768L, 32767L));
    }

    static uint16_t u16(float value)
    {
        if (!(value == value)) return 0;
        return static_cast<uint16_t>(constrain(lroundf(value), 0L, 65535L));
    }

    uint16_t flags(const copter::CopterSensors& sens, const copter::CopterRc& rc) const
    {
        namespace F = BlackBoxFormat::CopterFlag;
        const copter::CopterStatus& s = ctl.getStatus();
        uint16_t f = 0;
        if (ctl.isArmed()) f |= F::ARMED;
        if (s.airborne) f |= F::AIRBORNE;
        if (s.saturated) f |= F::SATURATED;
        if (s.crashed) f |= F::CRASHED;
        if (s.state == copter::CopterState::FAILSAFE) f |= F::FAILSAFE;
        if (rx.isSignalLost()) f |= F::RX_LOST;
        if (ctl.getSettings().airmode || rc.airmode) f |= F::AIRMODE;
        if (sens.imuOk) f |= F::IMU_OK;
        if (sens.baroOk) f |= F::BARO_OK;
        if (gps && gps->isAvailable() && gps->getGpsData().fixType >= 3) f |= F::GPS_FIX;
        if (home && home->hasHome()) f |= F::HOME;
        if (battery && battery->state() == copter::BatteryState::WARNING) f |= F::BAT_WARN;
        if (battery && battery->state() == copter::BatteryState::CRITICAL) f |= F::BAT_CRIT;
        if (ctl.motorTestActive()) f |= F::MOTOR_TEST;
        if (manualRecording) f |= F::MANUAL;
        return f;
    }

    void sample(const copter::CopterSensors& sens, const copter::CopterRc& rc, const copter::CopterOutputs& out, uint32_t workUs)
    {
        countRate();
        const copter::CopterStatus& s = ctl.getStatus();
        if (cycle % divider == 0)
        {
            BlackBoxFormat::CopterGyroRecord g;
            g.tUs = nowUs;
            for (uint8_t a = 0; a < 3; ++a)
            {
                g.gyro[a] = i16(sens.gyro[a] * 10.0f);
                g.gyroF[a] = i16(s.rates[a] * 10.0f);
                g.setpoint[a] = i16(s.setpoint[a] * 10.0f);
            }
            g.workUs = static_cast<uint16_t>(workUs > 65535 ? 65535 : workUs);
            pushRecord(BlackBoxFormat::REC_QGYRO, g);

            BlackBoxFormat::CopterCtrlRecord c;
            c.tUs = nowUs;
            for (uint8_t a = 0; a < 3; ++a)
            {
                const copter::AxisTerms& t = ctl.axisTerms(a);
                c.pid[a][0] = i16(t.p * 10000.0f);
                c.pid[a][1] = i16(t.i * 10000.0f);
                c.pid[a][2] = i16(t.d * 10000.0f);
                c.pid[a][3] = i16(t.ff * 10000.0f);
            }
            for (uint8_t m = 0; m < 4; ++m) c.motor[m] = u16(out.motor[m] * 1000.0f);
            c.thrust = u16(s.throttle * 1000.0f);
            pushRecord(BlackBoxFormat::REC_QCTRL, c);

            if (out.count > 4)
            {
                BlackBoxFormat::CopterMotor8Record m8;
                m8.tUs = nowUs;
                for (uint8_t m = 0; m < 4; ++m) m8.motor[m] = u16(out.motor[4 + m] * 1000.0f);
                pushRecord(BlackBoxFormat::REC_QMOT8, m8);
            }
        }

        if (nowMs - lastStateMs >= STATE_PERIOD_MS)
        {
            lastStateMs = nowMs;
            BlackBoxFormat::CopterStateRecord r;
            r.tUs = nowUs;
            r.roll = i16(s.roll * 100.0f);
            r.pitch = i16(s.pitch * 100.0f);
            float yaw = fmodf(s.yaw, 360.0f);
            if (yaw < 0.0f) yaw += 360.0f;
            r.yaw = u16(yaw * 100.0f);
            r.altDm = i16(s.altitude * 10.0f);
            r.climbCms = i16(s.climbRate * 100.0f);
            r.altTargetDm = i16(s.altTarget * 10.0f);
            r.rc[0] = i16(rc.roll * 1000.0f);
            r.rc[1] = i16(rc.pitch * 1000.0f);
            r.rc[2] = i16(rc.yaw * 1000.0f);
            r.rcThrottle = u16(rc.throttle * 1000.0f);
            r.hover = u16(s.hoverLearned * 1000.0f);
            r.vbatMv = battery ? u16(battery->volts() * 1000.0f) : u16(s.volts * 1000.0f);
            r.ampsCa = battery ? u16(battery->amps() * 100.0f) : 0;
            r.mah = battery ? u16(battery->mahDrawn()) : 0;
            r.rotHz = u16(s.rotationHz);
            r.linkQuality = rx.getLinkQuality();
            r.mode = static_cast<uint8_t>(s.mode);
            r.state = static_cast<uint8_t>(s.state);
            r.profiles = static_cast<uint8_t>((params.pidProfile() << 4) | (params.rateProfile() & 0x0F));
            r.flags = flags(sens, rc);
            r.armBlockers = s.armBlockers;
            pushRecord(BlackBoxFormat::REC_QSTATE, r);
        }

        if (gps && gps->isAvailable() && gps->getGpsData().timestamp != lastGpsStamp)
        {
            const GpsData& d = gps->getGpsData();
            lastGpsStamp = d.timestamp;
            BlackBoxFormat::GpsRecord r;
            r.tUs = nowUs;
            r.lat = static_cast<int32_t>(llround(d.latitude * 1e7));
            r.lon = static_cast<int32_t>(llround(d.longitude * 1e7));
            r.altMm = static_cast<int32_t>(lroundf(d.altitude * 1000.0f));
            r.speedCms = i16(d.groundSpeed * 100.0f);
            r.course = u16(d.heading * 100.0f);
            r.sats = d.numSatellites;
            r.fix = d.fixType;
            r.haccCm = u16(d.horizontalAccuracy * 100.0f);
            r.vaccCm = u16(d.verticalAccuracy * 100.0f);
            pushRecord(BlackBoxFormat::REC_GPS, r);
        }

        if (nowMs - lastSysMs >= SYS_PERIOD_MS)
        {
            lastSysMs = nowMs;
            BlackBoxFormat::SysRecord r;
            r.tUs = nowUs;
            r.loopHz = static_cast<uint16_t>(loopStats.hz > 65535 ? 65535 : loopStats.hz);
            r.loopAvgUs = static_cast<uint16_t>(loopStats.avgUs > 65535 ? 65535 : loopStats.avgUs);
            r.loopMaxUs = static_cast<uint16_t>(loopStats.maxUs > 65535 ? 65535 : loopStats.maxUs);
            r.heapFree = Rtos::freeHeapBytes();
            r.ibusGood = rx.getGoodFrameCount();
            r.ibusBad = rx.getBadFrameCount();
            r.imuTempC = imu ? i16(imu->getImuData().temperature * 100.0f) : 0;
            r.ringUsed = ring.used();
            r.dropped = ring.dropped();
            r.flashMaxUs = flashMaxUs;
            flashMaxUs = 0;
            const uint32_t freeKb = storage.freeBytes() / 1024;
            r.freeKb = static_cast<uint16_t>(freeKb > 65535 ? 65535 : freeKb);
            pushRecord(BlackBoxFormat::REC_SYS, r);
        }
    }

    // ---------------- заголовок полёта ----------------

    static const char* sensorName(const Sensor* sensor) { return sensor ? sensor->getSensorType() : "нет"; }

    void headerInfo(uint16_t flight, uint32_t ms) override
    {
        (void)flight;
        const copter::CopterTune& t = ctl.getTune();
        const copter::BuildSpec b = params.buildSpec();
        info("vehicle=copter");
        info("firmware=%s", OP_SOURCE_COMMIT);
        info("board=%s", OP_BOARD_ENV);
        info("build=%s", b.id ? b.id : "?");
        info("frame=%s", copter::frameName(t.frame.type));
        info("motors=%u", t.frame.count);
        info("start=%s", startReason);
        info("reset_reason=%s", resetCauseName(resetReason));
        info("uptime_ms=%lu", static_cast<unsigned long>(ms));
        info("loop_hz=%.0f", static_cast<double>(t.loopHz));
        info("rate.gyro_hz=%.0f", static_cast<double>(t.loopHz / divider));
        info("rate.state_hz=%lu", static_cast<unsigned long>(1000 / STATE_PERIOD_MS));
        static const char* const MODES[] = { "NORMAL", "MOTOR_TEST", "ALWAYS" };
        info("blackbox_mode=%s", MODES[static_cast<uint8_t>(mode) % 3]);
        info("bits.flags=%s", BlackBoxFormat::CopterFlag::NAMES);
        info("names.mode=ACRO HORIZON ANGLE ALT_HOLD");
        info("names.state=DISARMED ARMED FAILSAFE");
        char codes[220];
        blockerCodes(0xFFFFFFFFu, codes, sizeof(codes));
        info("bits.arm_blockers=%s", codes);
        info("sensor.imu=%s", sensorName(imu));
        info("sensor.baro=%s", sensorName(baro));
        info("sensor.gps=%s", sensorName(gps));

        // настройка, по которой летит (в единицах BF), и откуда она
        info("tune.source=%s", params.value(copter::PARAM_TUNE_SOURCE) > 0.5f ? "BETAFLIGHT" : "MODEL");
        const copter::AxisGains* g[3] = { &t.roll, &t.pitch, &t.yaw };
        static const char* const AXES[] = { "roll", "pitch", "yaw" };
        for (uint8_t a = 0; a < 3; ++a)
        {
            info("pid.%s=%.0f %.0f %.0f %.0f", AXES[a],
                 static_cast<double>(copter::gainToBf(copter::PidTerm::P, a, g[a]->p)),
                 static_cast<double>(copter::gainToBf(copter::PidTerm::I, a, g[a]->i)),
                 static_cast<double>(copter::gainToBf(copter::PidTerm::D, a, g[a]->d)),
                 static_cast<double>(copter::gainToBf(copter::PidTerm::F, a, g[a]->ff)));
            info("rates.%s=%.0f %.0f %.2f", AXES[a], static_cast<double>(t.rates[a].centerDps),
                 static_cast<double>(t.rates[a].maxDps), static_cast<double>(t.rates[a].expo));
        }
        info("filter.gyro_lpf_hz=%.0f", static_cast<double>(t.gyroLpfHz));
        info("filter.dterm_lpf_hz=%.0f", static_cast<double>(t.dtermLpfHz));
        info("filter.yaw_lpf_hz=%.0f", static_cast<double>(t.yawLpfHz));
        info("filter.notch_q=%.1f", static_cast<double>(t.notchQ));
        info("model.crossover_hz=%.1f", static_cast<double>(t.crossoverHzRP));
        info("model.hover=%.3f", static_cast<double>(t.hoverThrust));
        info("model.rot_hz_hover=%.0f", static_cast<double>(t.rotHzHover));
        info("profile=%u", params.pidProfile() + 1u);
        info("rateprofile=%u", params.rateProfile() + 1u);

        // всё, что задано руками (как diff в CLI): «set.<имя>=<значение>»
        char value[48];
        for (uint8_t i = 0; i < copter::PARAM_COUNT; ++i)
        {
            const copter::ParamId id = static_cast<copter::ParamId>(i);
            const copter::ParamInfo& p = copter::paramInfo(id);
            const uint8_t slot = p.scope == copter::ParamScope::PID_PROFILE    ? params.pidProfile()
                                 : p.scope == copter::ParamScope::RATE_PROFILE ? params.rateProfile()
                                                                              : 0;
            if (!params.isSet(id, slot)) continue;
            params.format(id, slot, value, sizeof(value));
            info("set.%s=%s", p.name, value);
            if (headerLength + 300 > sizeof(header)) break;
        }
    }
};
