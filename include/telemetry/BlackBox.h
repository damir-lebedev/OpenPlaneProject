#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/PilotSwitches.h"
#include "autopilot/TakeoffController.h"
#include "config/Channels.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "control/GroundCalibration.h"
#include "hal/ResetCause.h"
#include "hal/Rtos.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/BlackBoxCore.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/LoopStats.h"

// ============================================================
// ЧЁРНЫЙ ЯЩИК — запись полёта на встроенный флеш (ESP32-S3) или на
// SD-карту (STM32H743)
//
// Что пишется (формат — BlackBoxFormat.h, частоты при цикле 500 Гц):
//   IMU   каждый такт — гироскоп, акселерометр, время работы такта;
//   CTRL  100 Гц — углы и цели, стики, команды автопилота, все
//         выходы, составляющие ПИД, режим, флаги, функции;
//   RC    50 Гц — все каналы пульта, счётчики кадров iBUS;
//   BARO, MAG, GPS, AIR — каждый новый отсчёт датчика;
//   NAV   10 Гц; POWER 10 Гц — батарея и датчик тока (S3);
//   SYS   1 Гц — цикл, память, приёмник, сам ящик;
//   EVENT — ARM/DISARM, режимы, связь, отказы датчиков, GPS, функции;
//   в начале полёта — схема записей и параметры (ПИД, триммеры,
//   Config, датчики, привязки тумблеров, причина перезагрузки).
//
// Как устроено (общее с коптером — telemetry/BlackBoxCore.h):
//   • полётный цикл каждый такт вызывает update(): снимок кладётся в
//     очередь (BlackBoxRing: PSRAM у ESP32, ОЗУ у STM32) — микросекунды,
//     носитель не трогается;
//   • задача "bbox" после каждого такта пишет одну страницу (256 байт).
//     ESP32: запись флеша останавливает оба ядра на ~0.5 мс — поэтому
//     она идёт сразу после такта, в паузе цикла (ядро 0). STM32: задача
//     с приоритетом ниже полётной вытесняется циклом сама; карта пишет
//     блок за единицы мс, иногда за десятки — очередь это переживает;
//   • стирание — только в покое на земле: не записывается и не
//     заармлено (BlackBoxStorage::eraseStep). В воздухе носитель не
//     стирается никогда.
//
// Когда пишет: ARM + газ (стик или ESC выше THROTTLE_LOW_US), с
// предзаписью BLACKBOX_PREROLL_MS; после перезагрузки из-за сбоя —
// сразу; вручную — из консоли (стенд). Когда перестаёт — см. Config
// (BLACKBOX_*) и decideStop().
//
// Выгрузка: tools/blackbox.py (list / download / decode), команды
// "bb ..." по UART — handleHostCommand(). Консоль — меню 'k'.
// ============================================================

namespace BlackBoxRate
{
    // Каждый какой такт цикла (Config::LOOP_PERIOD_MS) — период ms.
    constexpr uint32_t every(uint32_t ms)
    {
        return ms / Config::LOOP_PERIOD_MS > 0 ? ms / Config::LOOP_PERIOD_MS : 1;
    }
}

class BlackBox : public BlackBoxCore
{
public:

    BlackBox(FlightController& flightController, Autopilot& ap, LoopStats& stats, BlackBoxStorage& flightStorage,
             const PilotSwitches* pilotSwitches = nullptr)
        : BlackBoxCore(flightStorage),
          controller(flightController),
          autopilot(ap),
          loopStats(stats),
          switches(pilotSwitches)
    {
    }

    // Прочитать носитель, выделить очередь, запустить задачу записи.
    // Места под запись нет (раздела флеша нет, файла на карте нет) —
    // ящик выключен, полёту это не мешает.
    bool begin(bool startTask = true)
    {
        resetReason = readResetCause();
        resetStartPending = isCrashReset(resetReason);

#if defined(BOARD_STM32H743)
        uint32_t size = Config::BLACKBOX_RING_STM32_BYTES;
        uint8_t* memory = static_cast<uint8_t*>(malloc(size));
#else
        uint32_t size = Config::BLACKBOX_RING_BYTES;
        uint8_t* memory = psramFound() ? static_cast<uint8_t*>(ps_malloc(size)) : nullptr;
        if (!memory)
        {
            size = Config::BLACKBOX_RING_NO_PSRAM_BYTES;
            memory = static_cast<uint8_t*>(malloc(size));
        }
#endif
        if (!beginCore(size, memory)) return false;
#if defined(BOARD_ESP32_S3)
        analogReadMilliVolts(Config::PIN_VBAT_ADC);   // первое чтение — калибровка АЦП, мс
#elif defined(BOARD_STM32H743)
        analogReadResolution(12);
        analogRead(Config::PIN_VBAT_ADC);
#endif

        printStatus(Serial);
        if (resetStartPending)
        {
            Serial.print("BlackBox: перезагрузка из-за сбоя (");
            Serial.print(resetCauseName(resetReason));
            Serial.println(") — запись включается сразу");
        }

        if (startTask) startWriterTask();
        return true;
    }

    // --------------------------------------------------------
    // Полётный цикл (ядро 1): после каждого такта. workUs — сколько
    // длилась работа такта (без ожидания).
    // --------------------------------------------------------

    void update(uint32_t workUs)
    {
        if (state == State::Off) return;

        nowUs = micros();
        nowMs = millis();
        const bool armed = controller.isArmed();
        armedFlag = armed;

        if (state != State::Stopping) detectEvents(armed);
        decideStartStop(armed);

        if (state == State::Idle) ring.trimOlderThan(nowUs, Config::BLACKBOX_PREROLL_MS * 1000UL);
        if (state != State::Stopping) sample(workUs);
        cycle++;

        wakeWriter();
    }

private:

    bool vehicleArmed() const override { return controller.isArmed(); }
    void onRecordingStart() override { snapshotEvent(); }

    static constexpr uint32_t CTRL_EVERY = BlackBoxRate::every(10);
    static constexpr uint32_t RC_EVERY = BlackBoxRate::every(20);
    static constexpr uint32_t NAV_EVERY = BlackBoxRate::every(100);
    static constexpr uint32_t SYS_EVERY = BlackBoxRate::every(1000);
    static constexpr uint32_t MAG_MIN_US = 20000;
    static constexpr uint32_t AIR_MIN_US = 20000;


    FlightController& controller;
    Autopilot& autopilot;
    LoopStats& loopStats;
    const PilotSwitches* switches;


    // --- сторона полётного цикла ---
    ResetCause resetReason = ResetCause::Unknown;
    bool resetStartPending = false;
    uint32_t lastBaroTs = 0, lastMagTs = 0, lastGpsTs = 0, lastAirTs = 0;
    uint32_t lastMagUs = 0, lastAirUs = 0;

    struct Seen
    {
        bool valid = false;
        bool armed = false;
        const char* mode = nullptr;
        const char* refusal = nullptr;
        uint8_t link = 0;
        uint8_t sensors = 0;   // биты: IMU BARO MAG GPS AIR
        uint8_t gpsFix = 0;
        bool home = false;
        bool fence = false;
        bool stall = false;
        uint16_t features = 0;
        uint8_t launch = 0;
        uint8_t soaring = 0;
        uint8_t takeoff = 0;
        uint8_t calibration = 0;
    } seen;


    // ========================================================
    // Старт и остановка
    // ========================================================

    bool motorOn() const
    {
        return controller.getOutputState().throttle > Config::THROTTLE_LOW_US ||
               controller.getRcState().get(Channels::THROTTLE) > Config::THROTTLE_LOW_US;
    }

    void decideStartStop(bool armed)
    {
        if (state == State::Idle)
        {
            if (manualStartRequested) start("вручную", true, false);
            else if (resetStartPending) start("перезагрузка после сбоя", false, true);
            else if (armed && motorOn()) start("ARM и газ", false, false);
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
        if (manualRecording) return;

        disarmedSinceMs = armed ? 0 : (disarmedSinceMs ? disarmedSinceMs : nowMs);
        const bool holdAfterReset = resetRecording && nowMs - recordingStartMs < Config::BLACKBOX_RESET_HOLD_MS;
        if (disarmedSinceMs && nowMs - disarmedSinceMs >= Config::BLACKBOX_POSTROLL_MS && !holdAfterReset)
        {
            stop("DISARM");
            return;
        }

        stillSinceMs = (armed && stillOnGround()) ? (stillSinceMs ? stillSinceMs : nowMs) : 0;
        if (stillSinceMs && nowMs - stillSinceMs >= Config::BLACKBOX_LANDED_STOP_MS)
        {
            stop("стоит на земле, мотор выключен");
        }
    }

    // Мотор стоит и самолёт неподвижен (см. Config::BLACKBOX_LANDED_*).
    bool stillOnGround() const
    {
        if (controller.getOutputState().throttle > Config::THROTTLE_LOW_US) return false;

        const ImuSensor* imu = autopilot.getImuSensor();
        if (!imu || !imu->isAvailable()) return false;
        const ImuData& d = imu->getImuData();
        const float gyroLimit = Config::BLACKBOX_LANDED_GYRO_DPS;
        if (fabsf(d.gyroX) > gyroLimit || fabsf(d.gyroY) > gyroLimit || fabsf(d.gyroZ) > gyroLimit) return false;
        const float g = sqrtf(d.accelX * d.accelX + d.accelY * d.accelY + d.accelZ * d.accelZ);
        if (fabsf(g - 1.0f) > Config::BLACKBOX_LANDED_ACCEL_G) return false;

        const BarometerSensor* baro = autopilot.getBarometerSensor();
        if (baro && baro->isAvailable() && fabsf(baro->getBarometerData().verticalSpeed) > Config::BLACKBOX_LANDED_CLIMB_MS)
        {
            return false;
        }
        const GpsSensor* gps = autopilot.getGpsSensor();
        if (gps && gps->isAvailable() && gps->hasFix() && gps->getGpsData().groundSpeed > Config::BLACKBOX_LANDED_SPEED_MS)
        {
            return false;
        }
        const AirspeedSensor* air = autopilot.getAirspeedSensor();
        if (air && air->isAvailable() && air->getAirspeedData().indicatedMs > Config::BLACKBOX_LANDED_SPEED_MS * 2)
        {
            return false;
        }
        return true;
    }


    // ========================================================
    // События
    // ========================================================

    uint8_t sensorBits() const
    {
        uint8_t bits = 0;
        if (autopilot.getImuSensor() && autopilot.getImuSensor()->isAvailable()) bits |= 1;
        if (autopilot.getBarometerSensor() && autopilot.getBarometerSensor()->isAvailable()) bits |= 2;
        if (autopilot.getMagnetometerSensor() && autopilot.getMagnetometerSensor()->isAvailable()) bits |= 4;
        if (autopilot.getGpsSensor() && autopilot.getGpsSensor()->isAvailable()) bits |= 8;
        if (autopilot.getAirspeedSensor() && autopilot.getAirspeedSensor()->isAvailable()) bits |= 16;
        return bits;
    }

    static uint16_t featureBits(const PilotInputs& in)
    {
        uint16_t bits = 0;
        for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
        {
            if (in.features[f]) bits |= static_cast<uint16_t>(1u << f);
        }
        return bits;
    }

    uint8_t linkState() const
    {
        const RcReceiver& rx = controller.getReceiver();
        return rx.isFrameTimeout() ? 1 : (rx.isFailsafeReported() ? 2 : 0);
    }

    void detectEvents(bool armed)
    {
        const bool first = !seen.valid;
        seen.valid = true;

        if (first || armed != seen.armed)
        {
            if (!first || armed) event(armed ? "ARM" : "DISARM");
            seen.armed = armed;
        }

        const char* refusal = controller.getArming().getLastRefusalReason();
        if (refusal != seen.refusal)
        {
            if (refusal) event("ARM отклонён: %s", refusal);
            seen.refusal = refusal;
        }

        const char* mode = autopilot.getModeName();
        if (!seen.mode || strcmp(mode, seen.mode) != 0)
        {
            event("режим %s", mode);
            seen.mode = mode;
        }

        const uint8_t link = linkState();
        if (first || link != seen.link)
        {
            static const char* const LINK[] = { "связь: есть", "связь: потеряна — нет кадров iBUS",
                                                 "связь: потеряна — failsafe пульта" };
            event("%s", LINK[link]);
            seen.link = link;
        }

        const uint8_t sensors = sensorBits();
        if (first || sensors != seen.sensors)
        {
            static const char* const NAMES[] = { "IMU", "BARO", "MAG", "GPS", "AIR" };
            char text[80] = "датчики:";
            for (uint8_t i = 0; i < 5; ++i)
            {
                const bool now = sensors & (1u << i);
                if (!first && now == static_cast<bool>(seen.sensors & (1u << i))) continue;
                strncat(text, now ? " +" : " -", sizeof(text) - strlen(text) - 1);
                strncat(text, NAMES[i], sizeof(text) - strlen(text) - 1);
            }
            event("%s", text);
            seen.sensors = sensors;
        }

        const GpsSensor* gps = autopilot.getGpsSensor();
        if (gps && gps->isAvailable() && gps->getGpsData().fixType != seen.gpsFix)
        {
            seen.gpsFix = gps->getGpsData().fixType;
            event("GPS: фикс %u, спутников %u", seen.gpsFix, gps->getGpsData().numSatellites);
        }

        const NavStatus& nav = autopilot.getNavStatus();
        if (nav.homeValid != seen.home)
        {
            if (nav.homeValid) event("дом: %.7f %.7f", nav.home.lat, nav.home.lon);
            else event("дом: сброшен");
            seen.home = nav.homeValid;
        }
        if (nav.fenceBreached != seen.fence)
        {
            event(nav.fenceBreached ? "геозабор: нарушен" : "геозабор: в пределах");
            seen.fence = nav.fenceBreached;
        }
        if (nav.stallWarning != seen.stall)
        {
            if (nav.stallWarning) event("малая скорость: защита от сваливания");
            seen.stall = nav.stallWarning;
        }

        const uint16_t features = featureBits(autopilot.getInputs());
        if (features != seen.features)
        {
            for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
            {
                const uint16_t bit = static_cast<uint16_t>(1u << f);
                if ((features ^ seen.features) & bit)
                {
                    event("%c%s", (features & bit) ? '+' : '-', AutopilotNames::feature(static_cast<Feature>(f)));
                }
            }
            seen.features = features;
        }

        const uint8_t launch = static_cast<uint8_t>(autopilot.getLaunchState());
        if (launch != seen.launch)
        {
            event("запуск с руки: стадия %u", launch);
            seen.launch = launch;
        }
        const uint8_t soaring = static_cast<uint8_t>(autopilot.getSoaringState());
        if (soaring != seen.soaring)
        {
            event("парение: стадия %u", soaring);
            seen.soaring = soaring;
        }
        const TakeoffController::Stage takeoff = autopilot.getTakeoffStage();
        if (static_cast<uint8_t>(takeoff) != seen.takeoff)
        {
            if (takeoff == TakeoffController::Stage::ABORTED && autopilot.getTakeoffAbortReason())
            {
                event("автовзлёт: ABORTED — %s", autopilot.getTakeoffAbortReason());
            }
            else
            {
                event("автовзлёт: %s", TakeoffController::stageName(takeoff));
            }
            seen.takeoff = static_cast<uint8_t>(takeoff);
        }
        const GroundCalibration& calibration = controller.getCalibration();
        if (static_cast<uint8_t>(calibration.getStage()) != seen.calibration)
        {
            switch (calibration.getStage())
            {
                case GroundCalibration::Stage::LEVEL:  event("калибровка самолёта: шаг 1 — горизонт"); break;
                case GroundCalibration::Stage::PARK:   event("калибровка самолёта: шаг 2 — стоянка"); break;
                case GroundCalibration::Stage::DONE:
                    event("калибровка самолёта: сохранена, стоянка тангаж %.1f крен %.1f",
                          static_cast<double>(calibration.getParkPitch()), static_cast<double>(calibration.getParkRoll()));
                    break;
                case GroundCalibration::Stage::FAILED:
                    event("калибровка самолёта: ошибка — %s", calibration.getFailure() ? calibration.getFailure() : "?");
                    break;
                case GroundCalibration::Stage::IDLE:   event("калибровка самолёта: отменена"); break;
            }
            seen.calibration = static_cast<uint8_t>(calibration.getStage());
        }

        if (ring.dropped() != reportedDropped && nowMs - dropReportMs >= 1000)
        {
            event("очередь переполнена: потеряно записей %lu", (unsigned long)(ring.dropped() - reportedDropped));
            reportedDropped = ring.dropped();
            dropReportMs = nowMs;
        }
    }

    void snapshotEvent()
    {
        const uint8_t s = sensorBits();
        event("состояние: ARM=%s MODE=%s RX=%s IMU=%s BARO=%s MAG=%s GPS=%s AIR=%s",
              controller.isArmed() ? "YES" : "NO", autopilot.getModeName(), linkState() ? "LOST" : "OK",
              (s & 1) ? "OK" : "-", (s & 2) ? "OK" : "-", (s & 4) ? "OK" : "-", (s & 8) ? "OK" : "-",
              (s & 16) ? "OK" : "-");
    }


    void sample(uint32_t workUs)
    {
        countRate();

        // Делитель — настройка Config, сейчас 1.
        // cppcheck-suppress knownConditionTrueFalse
        // cppcheck-suppress moduloofone
        if (cycle % Config::BLACKBOX_IMU_DIVIDER == 0) sampleImu(workUs);
        if (cycle % CTRL_EVERY == 0) sampleCtrl();
        if (cycle % RC_EVERY == 0) sampleRc();
        sampleBaro();
        sampleMag();
        sampleGps();
        sampleAir();
        if (cycle % NAV_EVERY == 0) sampleNav();
        if (cycle % NAV_EVERY == 1) samplePower();
        if (cycle % SYS_EVERY == 0) sampleSys();
    }

    void sampleImu(uint32_t workUs)
    {
        const ImuSensor* imu = autopilot.getImuSensor();
        if (!imu) return;
        const ImuData& d = imu->getImuData();
        BlackBoxFormat::ImuRecord r;
        r.tUs = nowUs;
        r.gyro[0] = i16(d.gyroX * 10.0f);
        r.gyro[1] = i16(d.gyroY * 10.0f);
        r.gyro[2] = i16(d.gyroZ * 10.0f);
        r.accel[0] = i16(d.accelX * 1000.0f);
        r.accel[1] = i16(d.accelY * 1000.0f);
        r.accel[2] = i16(d.accelZ * 1000.0f);
        r.workUs = static_cast<uint16_t>(min(workUs, static_cast<uint32_t>(65535)));
        pushRecord(BlackBoxFormat::REC_IMU, r);
    }

    uint16_t flags(bool armed) const
    {
        namespace F = BlackBoxFormat::Flag;
        const RcReceiver& rx = controller.getReceiver();
        const NavStatus& nav = autopilot.getNavStatus();
        const uint8_t sensors = sensorBits();
        const GpsSensor* gps = autopilot.getGpsSensor();

        uint16_t f = 0;
        if (armed) f |= F::ARMED;
        if (rx.isSignalLost()) f |= F::RX_LOST;
        if (rx.isFrameTimeout()) f |= F::RX_TIMEOUT;
        if (rx.isFailsafeReported()) f |= F::RX_FAILSAFE;
        if (autopilot.isFailsafeActive()) f |= F::FS_ACTIVE;
        if (autopilot.isFailsafeGliding()) f |= F::FS_GLIDE;
        if (autopilot.isFailsafeReturning()) f |= F::FS_RTH;
        if (autopilot.isAutoThrottle()) f |= F::AUTO_THR;
        if (nav.stallWarning) f |= F::STALL;
        if (nav.fenceBreached) f |= F::FENCE;
        if (sensors & 1) f |= F::IMU_OK;
        if (sensors & 2) f |= F::BARO_OK;
        if (sensors & 4) f |= F::MAG_OK;
        if (sensors & 8) f |= F::GPS_OK;
        if (gps && gps->isAvailable() && gps->hasFix()) f |= F::GPS_FIX;
        if (sensors & 16) f |= F::AIR_OK;
        return f;
    }

    void sampleCtrl()
    {
        BlackBoxFormat::CtrlRecord r;
        r.tUs = nowUs;

        const ImuSensor* imu = autopilot.getImuSensor();
        const ImuData* d = imu ? &imu->getImuData() : nullptr;
        r.roll = d ? i16(d->roll * 100.0f) : 0;
        r.pitch = d ? i16(d->pitch * 100.0f) : 0;
        r.yaw = d ? i16(d->yaw * 100.0f) : 0;
        r.wantRoll = i16(autopilot.getDesiredRoll() * 100.0f);
        r.wantPitch = i16(autopilot.getDesiredPitch() * 100.0f);

        const ControlCommand& pilot = autopilot.getPilotCommand();
        const ControlCommand command = autopilot.getCommand();
        r.stickRoll = pilot.roll;
        r.stickPitch = pilot.pitch;
        r.stickYaw = pilot.yaw;
        r.cmdRoll = command.roll;
        r.cmdPitch = command.pitch;
        r.cmdYaw = command.yaw;
        r.flaps = controller.getFlapsUs();
        r.thrPilot = controller.getRcState().get(Channels::THROTTLE);
        r.thrAuto = i16(autopilot.getThrottleCorrection() * 10.0f);

        const FlightOutputState& out = controller.getOutputState();
        r.out[0] = out.aileronLeft;
        r.out[1] = out.aileronRight;
        r.out[2] = out.elevator;
        r.out[3] = out.rudder;
        r.out[4] = out.throttle;
        r.out[5] = out.aux1;
        r.out[6] = out.aux2;

        const PidController& pr = autopilot.getRollPid();
        const PidController& pp = autopilot.getPitchPid();
        r.pidRoll[0] = i16(pr.getLastP() * 10.0f);
        r.pidRoll[1] = i16(pr.getLastI() * 10.0f);
        r.pidRoll[2] = i16(pr.getLastD() * 10.0f);
        r.pidPitch[0] = i16(pp.getLastP() * 10.0f);
        r.pidPitch[1] = i16(pp.getLastI() * 10.0f);
        r.pidPitch[2] = i16(pp.getLastD() * 10.0f);

        r.mode = static_cast<uint8_t>(autopilot.getMode());
        r.flags = flags(armedFlag);
        r.features = featureBits(autopilot.getInputs());
        pushRecord(BlackBoxFormat::REC_CTRL, r);
    }

    void sampleRc()
    {
        const RcChannelState& rc = controller.getRcState();
        const RcReceiver& rx = controller.getReceiver();
        BlackBoxFormat::RcRecord r = {};
        static_assert(Config::RC_CHANNELS <= sizeof(r.ch) / sizeof(r.ch[0]), "RC: каналов больше, чем в записи");
        r.tUs = nowUs;
        for (uint8_t i = 0; i < Config::RC_CHANNELS; ++i) r.ch[i] = rc.get(i);
        r.status = static_cast<uint8_t>((rx.isFrameTimeout() ? 1 : 0) | (rx.isFailsafeReported() ? 2 : 0));
        r.good = static_cast<uint16_t>(rx.getGoodFrameCount());
        r.bad = static_cast<uint16_t>(rx.getBadFrameCount());
        pushRecord(BlackBoxFormat::REC_RC, r);
    }

    void sampleBaro()
    {
        const BarometerSensor* baro = autopilot.getBarometerSensor();
        if (!baro || !baro->isAvailable()) return;
        const BarometerData& d = baro->getBarometerData();
        if (d.timestamp == lastBaroTs) return;
        lastBaroTs = d.timestamp;

        BlackBoxFormat::BaroRecord r;
        r.tUs = nowUs;
        r.pressurePa = d.pressure;
        r.tempC = i16(d.temperature * 100.0f);
        r.altM = d.altitude;
        r.vzCms = i16(d.verticalSpeed * 100.0f);
        r.targetDm = i16(autopilot.getTargetAltitude() * 10.0f);
        pushRecord(BlackBoxFormat::REC_BARO, r);
    }

    void sampleMag()
    {
        const MagnetometerSensor* mag = autopilot.getMagnetometerSensor();
        if (!mag || !mag->isAvailable()) return;
        const MagData& d = mag->getMagData();
        if (d.timestamp == lastMagTs || nowUs - lastMagUs < MAG_MIN_US) return;
        lastMagTs = d.timestamp;
        lastMagUs = nowUs;

        BlackBoxFormat::MagRecord r;
        r.tUs = nowUs;
        r.mag[0] = i16(d.magX * 10.0f);
        r.mag[1] = i16(d.magY * 10.0f);
        r.mag[2] = i16(d.magZ * 10.0f);
        r.heading = angle100(d.headingDegrees);
        pushRecord(BlackBoxFormat::REC_MAG, r);
    }

    void sampleGps()
    {
        const GpsSensor* gps = autopilot.getGpsSensor();
        if (!gps || !gps->isAvailable()) return;
        const GpsData& d = gps->getGpsData();
        if (d.timestamp == lastGpsTs) return;
        lastGpsTs = d.timestamp;

        BlackBoxFormat::GpsRecord r;
        r.tUs = nowUs;
        r.lat = static_cast<int32_t>(llround(d.latitude * 1e7));
        r.lon = static_cast<int32_t>(llround(d.longitude * 1e7));
        r.altMm = static_cast<int32_t>(lroundf(d.altitude * 1000.0f));
        r.speedCms = i16(d.groundSpeed * 100.0f);
        r.course = angle100(d.heading);
        r.sats = d.numSatellites;
        r.fix = d.fixType;
        r.haccCm = u16(d.horizontalAccuracy * 100.0f);
        r.vaccCm = u16(d.verticalAccuracy * 100.0f);
        pushRecord(BlackBoxFormat::REC_GPS, r);
    }

    void sampleAir()
    {
        const AirspeedSensor* air = autopilot.getAirspeedSensor();
        if (!air || !air->isAvailable()) return;
        const AirspeedData& d = air->getAirspeedData();
        if (d.timestamp == lastAirTs || nowUs - lastAirUs < AIR_MIN_US) return;
        lastAirTs = d.timestamp;
        lastAirUs = nowUs;

        BlackBoxFormat::AirRecord r;
        r.tUs = nowUs;
        r.dpPa = d.differentialPressurePa;
        r.iasCms = i16(d.indicatedMs * 100.0f);
        r.tasCms = i16(d.trueMs * 100.0f);
        r.rho = u16(d.airDensity * 10000.0f);
        pushRecord(BlackBoxFormat::REC_AIR, r);
    }

    void sampleNav()
    {
        const NavStatus& nav = autopilot.getNavStatus();
        BlackBoxFormat::NavRecord r;
        r.tUs = nowUs;
        r.flags = static_cast<uint8_t>((nav.gpsGood ? 1 : 0) | (nav.homeValid ? 2 : 0) | (nav.fenceBreached ? 4 : 0) |
                                       (nav.stallWarning ? 8 : 0));
        r.homeDistM = nav.distanceHomeM;
        r.homeBearing = angle100(nav.bearingHomeDeg);
        r.course = angle100(nav.courseDeg);
        r.targetCourse = angle100(nav.targetCourseDeg);
        r.speedCms = i16(nav.speedMs * 100.0f);
        r.courseSource = static_cast<uint8_t>(autopilot.getCourseSource());
        r.launchState = static_cast<uint8_t>(autopilot.getLaunchState());
        r.soaringState = static_cast<uint8_t>(autopilot.getSoaringState());
        r.takeoffStage = static_cast<uint8_t>(autopilot.getTakeoffStage());
        r.trimRoll = i16(autopilot.getAutoTrim().getRoll() * 10.0f);
        r.trimPitch = i16(autopilot.getAutoTrim().getPitch() * 10.0f);
        pushRecord(BlackBoxFormat::REC_NAV, r);
    }

#if defined(BOARD_ESP32_S3) || defined(BOARD_STM32H743)
    // Напряжение на пине АЦП, мВ.
    static uint32_t pinMillivolts(int16_t pin)
    {
#if defined(BOARD_ESP32_S3)
        return analogReadMilliVolts(pin);
#else
        return static_cast<uint32_t>(analogRead(pin)) * 3300u / 4095u;   // STM32: 12 бит, опорное 3.3 В
#endif
    }
#endif

    void samplePower()
    {
#if defined(BOARD_ESP32_S3) || defined(BOARD_STM32H743)
        BlackBoxFormat::PowerRecord r;
        r.tUs = nowUs;
        r.vbatMv = u16(pinMillivolts(Config::PIN_VBAT_ADC) * Config::BLACKBOX_VBAT_DIVIDER);
        r.currentMv = u16(pinMillivolts(Config::PIN_CURRENT_ADC) * Config::BLACKBOX_CURRENT_DIVIDER);
        pushRecord(BlackBoxFormat::REC_POWER, r);
#endif
    }

    void sampleSys()
    {
        const RcReceiver& rx = controller.getReceiver();
        const ImuSensor* imu = autopilot.getImuSensor();
        BlackBoxFormat::SysRecord r;
        r.tUs = nowUs;
        const uint32_t hz = loopStats.hz;
        const uint32_t avgUs = loopStats.avgUs;
        const uint32_t maxUs = loopStats.maxUs;
        r.loopHz = static_cast<uint16_t>(min(hz, static_cast<uint32_t>(65535)));
        r.loopAvgUs = static_cast<uint16_t>(min(avgUs, static_cast<uint32_t>(65535)));
        r.loopMaxUs = static_cast<uint16_t>(min(maxUs, static_cast<uint32_t>(65535)));
        r.heapFree = Rtos::freeHeapBytes();
        r.ibusGood = rx.getGoodFrameCount();
        r.ibusBad = rx.getBadFrameCount();
        r.imuTempC = imu ? i16(imu->getImuData().temperature * 100.0f) : 0;
        r.ringUsed = ring.used();
        r.dropped = ring.dropped();
        r.flashMaxUs = flashMaxUs;
        flashMaxUs = 0;
        r.freeKb = static_cast<uint16_t>(min(storage.freeBytes() / 1024, static_cast<uint32_t>(65535)));
        pushRecord(BlackBoxFormat::REC_SYS, r);
    }


    // --- заголовок полёта: схема и параметры ---


    static const char* sensorName(const Sensor* sensor) { return sensor ? sensor->getSensorType() : "нет"; }

    void headerInfo(uint16_t flight, uint32_t ms) override
    {
        (void)flight;
        info("firmware=%s %s", __DATE__, __TIME__);
        info("board=%s", boardName());
        info("start=%s", startReason);
        info("reset_reason=%s", resetCauseName(resetReason));
        info("uptime_ms=%lu", (unsigned long)ms);
        info("loop_period_ms=%lu", (unsigned long)Config::LOOP_PERIOD_MS);
        info("rate.imu_hz=%lu", (unsigned long)(1000 / Config::LOOP_PERIOD_MS / Config::BLACKBOX_IMU_DIVIDER));
        info("rate.ctrl_hz=%lu", (unsigned long)(1000 / (Config::LOOP_PERIOD_MS * CTRL_EVERY)));
        info("bits.flags=%s", BlackBoxFormat::Flag::NAMES);

        char names[200] = "";
        for (uint8_t m = 0; m < MODE_COUNT; ++m)
        {
            if (m) strncat(names, " ", sizeof(names) - strlen(names) - 1);
            strncat(names, AutopilotNames::mode(static_cast<AutopilotMode>(m)), sizeof(names) - strlen(names) - 1);
        }
        info("names.mode=%s", names);
        names[0] = '\0';
        for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
        {
            if (f) strncat(names, " ", sizeof(names) - strlen(names) - 1);
            strncat(names, AutopilotNames::feature(static_cast<Feature>(f)), sizeof(names) - strlen(names) - 1);
        }
        info("bits.features=%s", names);
        info("names.course_source=NONE GYRO COMPASS GPS");

        const ImuSensor* imu = autopilot.getImuSensor();
        info("sensor.imu=%s", sensorName(imu));
        info("sensor.baro=%s", sensorName(autopilot.getBarometerSensor()));
        info("sensor.mag=%s", sensorName(autopilot.getMagnetometerSensor()));
        info("sensor.gps=%s", sensorName(autopilot.getGpsSensor()));
        info("sensor.airspeed=%s", sensorName(autopilot.getAirspeedSensor()));
        if (imu) info("imu.preflight=%s", imu->getPreflightProblem() ? imu->getPreflightProblem() : "OK");

        const PidController& pr = autopilot.getRollPid();
        const PidController& pp = autopilot.getPitchPid();
        info("pid.roll=%.4f %.4f %.4f", pr.getKp(), pr.getKi(), pr.getKd());
        info("pid.pitch=%.4f %.4f %.4f", pp.getKp(), pp.getKi(), pp.getKd());
        info("trim.roll_us=%.1f", autopilot.getAutoTrim().getRoll());
        info("trim.pitch_us=%.1f", autopilot.getAutoTrim().getPitch());

        infoConfig();
#if defined(BOARD_ESP32_S3) || defined(BOARD_STM32H743)
        info("power.vbat_divider=%.4f", static_cast<double>(Config::BLACKBOX_VBAT_DIVIDER));
        info("power.current_divider=%.4f", static_cast<double>(Config::BLACKBOX_CURRENT_DIVIDER));
#endif

        if (switches)
        {
            InfoLines lines(*this, "bind");
            switches->printBindings(lines);
        }
    }

    // Номера реверсированных каналов пульта ("-" — ни одного).
    static String reversedChannels()
    {
        String out;
        for (uint8_t ch = 0; ch < Config::RC_CHANNELS; ++ch)
        {
            if (!Config::CHANNEL_REVERSED[ch]) continue;
            if (out.length()) out += ',';
            out += "CH";
            out += static_cast<int>(ch + 1);
        }
        return out.length() ? out : String("-");
    }

    static const char* boardName()
    {
#if defined(BOARD_ESP32_S3)
        return "ESP32-S3";
#elif defined(BOARD_ESP32_C3)
        return "ESP32-C3";
#elif defined(BOARD_ESP32_CLASSIC)
        return "ESP32";
#elif defined(BOARD_STM32H743)
        return "STM32H743";
#else
        return "?";
#endif
    }

    // Настройки, от которых зависит поведение в полёте.
    void infoConfig()
    {
#define BB_CFG_INT(name) info("cfg." #name "=%ld", static_cast<long>(Config::name))
#define BB_CFG_FLT(name) info("cfg." #name "=%.3f", static_cast<double>(Config::name))
        BB_CFG_INT(AILERON_MAX_US);
        BB_CFG_INT(ELEVATOR_MAX_US);
        BB_CFG_INT(RUDDER_MAX_US);
        BB_CFG_INT(AILERON_LEFT_REVERSED);
        BB_CFG_INT(AILERON_RIGHT_REVERSED);
        BB_CFG_INT(ELEVATOR_REVERSED);
        BB_CFG_INT(RUDDER_REVERSED);
        BB_CFG_INT(AUX1_REVERSED);
        BB_CFG_INT(AUX2_REVERSED);
        BB_CFG_INT(MOTOR_REVERSED);
        BB_CFG_INT(FLAPS_REVERSED);
        info("cfg.CHANNEL_REVERSED=%s", reversedChannels().c_str());
        BB_CFG_INT(ESC_BIDIRECTIONAL);
        BB_CFG_INT(REVERSE_THROTTLE_LIMIT_PCT);
        BB_CFG_INT(FLAPS_DEPLOYED_US);
        BB_CFG_INT(AIRBRAKE_US);
        BB_CFG_INT(IMU_ROTATION_CW_DEG);
        BB_CFG_INT(MAG_ROTATION_CW_DEG);
        BB_CFG_INT(THROTTLE_LOW_US);
        BB_CFG_INT(RX_FAILSAFE_THROTTLE_US);
        BB_CFG_FLT(MAX_BANK_DEG);
        BB_CFG_FLT(STAB_MAX_PITCH_DEG);
        BB_CFG_FLT(STAB_INTEGRATOR_ZONE_DEG);
        BB_CFG_FLT(ACRO_MAX_RATE_DPS);
        BB_CFG_FLT(ACRO_RATE_GAIN_US_PER_DPS);
        BB_CFG_FLT(FAILSAFE_GLIDE_ROLL_DEG);
        BB_CFG_FLT(FAILSAFE_GLIDE_PITCH_DEG);
        BB_CFG_INT(FAILSAFE_RTH);
        BB_CFG_FLT(RTH_ALTITUDE_M);
        BB_CFG_FLT(CRUISE_THROTTLE_PCT);
        BB_CFG_FLT(CRUISE_AIRSPEED_MS);
        BB_CFG_FLT(STALL_SPEED_MS);
        BB_CFG_FLT(LOITER_RADIUS_M);
        BB_CFG_FLT(NAV_ALT_GAIN);
        BB_CFG_FLT(NAV_CLIMB_KP_DEG);
        BB_CFG_FLT(NAV_CLIMB_KI_DEG);
        BB_CFG_FLT(TURN_COORD_RUDDER_MIX);
        BB_CFG_FLT(TURN_COORD_PITCH_US);
        BB_CFG_INT(GEOFENCE_ALWAYS_ON);
        BB_CFG_FLT(FENCE_RADIUS_M);
        BB_CFG_FLT(FENCE_ALTITUDE_M);
        BB_CFG_INT(LANDING_GEAR);
        BB_CFG_INT(TD_TAIL_UP_ELEVATOR_US);
        BB_CFG_FLT(TD_ROLL_PITCH_DEG);
        BB_CFG_FLT(TD_TAIL_CLEARANCE_DEG);
        BB_CFG_FLT(TAKEOFF_CLIMB_PITCH_DEG);
        BB_CFG_FLT(TAKEOFF_SAFE_ALTITUDE_M);
        BB_CFG_FLT(TAKEOFF_HEADING_KP);
        BB_CFG_FLT(LAUNCH_ACCEL_G);
        BB_CFG_FLT(LAUNCH_CLIMB_PITCH_DEG);
        BB_CFG_FLT(LAND_GLIDE_PITCH_DEG);
        BB_CFG_FLT(AUTOTRIM_RATE);
        BB_CFG_FLT(PITOT_SCALE);
#undef BB_CFG_INT
#undef BB_CFG_FLT
    }
};
