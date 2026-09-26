#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "control/FlightOutputState.h"
#include "rc/IBusReceiver.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"
#include "telemetry/LogSettings.h"
#include "telemetry/LoopStats.h"

// ============================================================
// DEBUG LOGGER — вывод состояния в монитор порта по каналам
//
// Каждый канал (LogSettings.h) — своя строка со своим префиксом:
//   STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
//   ATT  R +1.2 P -0.4 Y 123.0
// и свой режим: выкл / при изменении / постоянно. "При изменении" —
// строка печатается, только когда значения ушли дальше допуска
// (дребезг стиков в пару мкс и шум датчиков в десятые градуса не в
// счёт), поэтому лежащий на столе самолёт лог не засыпает, а
// меняющийся канал не тащит за собой в лог все остальные.
//
// Что выводить — меню консоли ('l'), настройки в NVS. Пока открыто
// меню, лог молчит (suspend), чтобы не затирать экран; пробел —
// пауза. После меню/паузы все включённые каналы печатаются заново —
// видно текущее состояние.
//
// Полностью отдельно от полётной логики: только читает геттеры.
// ============================================================

class DebugLogger
{
public:

    explicit DebugLogger(
        FlightController& flightController,
        Autopilot* ap = nullptr,
        LoopStats* stats = nullptr
    )
        : controller(flightController),
          autopilot(ap),
          loopStats(stats)
    {
        refresh();
    }

    // Загрузить настройки каналов из NVS.
    void begin()
    {
        settings.load();
    }

    void update()
    {
        if (paused || suspended) return;

        const uint32_t now = millis();
        if (now - lastTickMs < Config::DEBUG_INTERVAL_MS) return;
        lastTickMs = now;

        for (uint8_t channel = 0; channel < LogSettings::COUNT; ++channel)
        {
            updateChannel(channel, now);
        }
    }

    LogSettings& getSettings() { return settings; }
    void saveSettings() const { settings.save(); }

    // Меню открыто — лог молчит. После — всё включённое заново.
    void suspend(bool isSuspended)
    {
        suspended = isSuspended;
        if (!suspended) refresh();
    }

    void setPaused(bool isPaused)
    {
        paused = isPaused;
        if (!paused) refresh();
    }

    bool isPaused() const { return paused; }

    // Следующий такт напечатает все включённые каналы, даже без изменений.
    void refresh()
    {
        for (uint8_t i = 0; i < LogSettings::COUNT; ++i) refreshPending[i] = true;
    }


private:

    static constexpr size_t LINE_SIZE = 200;
    static constexpr uint32_t SYSTEM_INTERVAL_MS = 10000;

    // Print в буфер строки — чтобы сравнить её с прошлой перед печатью.
    class LineBuffer : public Print
    {
    public:
        size_t write(uint8_t c) override
        {
            if (length + 1 < sizeof(buffer))
            {
                buffer[length++] = static_cast<char>(c);
                buffer[length] = '\0';
            }
            return 1;
        }

        void reset()
        {
            length = 0;
            buffer[0] = '\0';
        }

        const char* c_str() const { return buffer; }

    private:
        char buffer[LINE_SIZE] = {};
        size_t length = 0;
    };

    // Значение "для показа": держит старое, пока новое не уйдёт дальше
    // допуска. Допуск 0 (режим "постоянно") — всегда свежее значение.
    // double — чтобы не терять точность координат GPS (float хранит
    // широту с шагом ~0.4 м).
    struct Shown
    {
        double value = 0;
        bool valid = false;

        double update(double raw, double band)
        {
            if (!valid || fabs(raw - value) > band)
            {
                value = raw;
                valid = true;
            }
            return value;
        }
    };

    FlightController& controller;
    Autopilot* autopilot;
    LoopStats* loopStats;

    LogSettings settings;
    bool paused = false;
    bool suspended = false;
    uint32_t lastTickMs = 0;

    LineBuffer line;
    char previous[LogSettings::COUNT][LINE_SIZE] = {};
    uint32_t lastPrintMs[LogSettings::COUNT] = {};
    bool refreshPending[LogSettings::COUNT] = {};

    Shown rc[Config::IBUS_CHANNELS];
    Shown outputs[5];
    Shown roll, pitch, yaw;
    Shown wantRoll, wantPitch, corrRoll, corrPitch, corrThrottle;
    Shown altitude, climb;
    Shown heading;
    Shown gpsLat, gpsLon, gpsSpeed;
    Shown gyro[3], accel[3];

    void updateChannel(uint8_t channel, uint32_t now)
    {
        const LogMode mode = settings.mode(channel);
        if (mode == LogMode::Off)
        {
            refreshPending[channel] = false;
            return;
        }

        const bool periodic = mode == LogMode::Periodic;
        const uint32_t period = static_cast<LogChannel>(channel) == LogChannel::System
                                    ? SYSTEM_INTERVAL_MS : settings.periodMs();
        if (periodic && !refreshPending[channel] && now - lastPrintMs[channel] < period) return;

        line.reset();
        const LogChannelInfo& info = LogSettings::info(channel);
        line.print(info.tag);
        for (size_t i = strlen(info.tag); i < 5; ++i) line.print(' ');
        format(static_cast<LogChannel>(channel), periodic);

        if (!periodic && !refreshPending[channel] && strcmp(line.c_str(), previous[channel]) == 0) return;

        Serial.println(line.c_str());
        snprintf(previous[channel], LINE_SIZE, "%s", line.c_str());
        lastPrintMs[channel] = now;
        refreshPending[channel] = false;
    }

    void format(LogChannel channel, bool periodic)
    {
        // В режиме "постоянно" допуски не нужны — показываем как есть.
        const float k = periodic ? 0.0f : 1.0f;

        switch (channel)
        {
            case LogChannel::Status:    formatStatus(); break;
            case LogChannel::Rc:        formatRc(k); break;
            case LogChannel::Outputs:   formatOutputs(k); break;
            case LogChannel::Attitude:  formatAttitude(k); break;
            case LogChannel::Autopilot: formatAutopilot(k); break;
            case LogChannel::Altitude:  formatAltitude(k); break;
            case LogChannel::Heading:   formatHeading(k); break;
            case LogChannel::Gps:       formatGps(k); break;
            case LogChannel::Imu:       formatImu(k); break;
            case LogChannel::System:    formatSystem(); break;
            case LogChannel::Count:     break;
        }
    }

    // --- каналы ---

    void formatStatus()
    {
        const IBusReceiver& receiver = controller.getReceiver();
        line.print("RX=");
        if (receiver.isFrameTimeout()) line.print("LOST(нет кадров)");
        else if (receiver.isFailsafeReported()) line.print("LOST(failsafe пульта)");
        else line.print("OK");

        line.print(" ARM=");
        line.print(controller.isArmed() ? "YES" : "NO");

        line.print(" MODE=");
        line.print(autopilot ? autopilot->getModeName() : "-");

        const int16_t flaps = controller.getFlapsUs();
        line.print(" FLAPS=");
        line.print(flaps <= 0 ? "UP" : (flaps >= Config::FLAPS_DEPLOYED_US ? "DOWN" : "MOVING"));

        if (autopilot)
        {
            const ImuSensor* imu = autopilot->getImuSensor();
            line.print(" IMU=");
            if (!imu) line.print("NONE");
            else if (!imu->isAvailable()) line.print("NO_RESPONSE");
            else if (imu->getPreflightProblem()) line.print("CHECK_FAILED");
            else line.print("OK");

            const BarometerSensor* baro = autopilot->getBarometerSensor();
            line.print(" BARO=");
            line.print(!baro ? "NONE" : (baro->isAvailable() ? "OK" : "NO_RESPONSE"));
        }
    }

    void formatRc(float k)
    {
        const RcChannelState& state = controller.getRcState();
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            line.printf("%u:%.0f ", i + 1, rc[i].update(state.get(i), k * Config::DEBUG_CHANGE_DEADBAND_US));
        }
    }

    void formatOutputs(float k)
    {
        const FlightOutputState& out = controller.getOutputState();
        const float band = k * Config::DEBUG_CHANGE_DEADBAND_US;
        line.printf("AIL-L %.0f AIL-R %.0f ELE %.0f RUD %.0f ESC %.0f",
                    outputs[0].update(out.aileronLeft, band), outputs[1].update(out.aileronRight, band),
                    outputs[2].update(out.elevator, band), outputs[3].update(out.rudder, band),
                    outputs[4].update(out.throttle, band));
    }

    bool imuReady(const char*& problem) const
    {
        const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        if (!imu) { problem = "IMU нет в схеме"; return false; }
        if (!imu->isAvailable()) { problem = "IMU не отвечает"; return false; }
        return true;
    }

    void formatAttitude(float k)
    {
        const char* problem;
        if (!imuReady(problem)) { line.print(problem); return; }

        const ImuData& d = autopilot->getImuSensor()->getImuData();
        line.printf("R %+.1f P %+.1f Y %.1f",
                    roll.update(d.roll, k * 0.5f), pitch.update(d.pitch, k * 0.5f),
                    yaw.update(d.yaw, k * 1.0f));
    }

    void formatAutopilot(float k)
    {
        if (!autopilot) { line.print("нет"); return; }

        line.printf("%s want R %+.1f P %+.1f corr R %+.0f P %+.0f THR %+.0f",
                    autopilot->getModeName(),
                    wantRoll.update(autopilot->getDesiredRoll(), k * 0.5f),
                    wantPitch.update(autopilot->getDesiredPitch(), k * 0.5f),
                    corrRoll.update(autopilot->getRollCorrection(), k * 2.0f),
                    corrPitch.update(autopilot->getPitchCorrection(), k * 2.0f),
                    corrThrottle.update(autopilot->getThrottleCorrection(), k * 2.0f));
    }

    void formatAltitude(float k)
    {
        const BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
        if (!baro) { line.print("барометра нет в схеме"); return; }
        if (!baro->isAvailable()) { line.print("барометр не отвечает"); return; }

        const BarometerData& d = baro->getBarometerData();
        line.printf("%.1f м  Vz %+.2f м/с  цель %.1f м",
                    altitude.update(d.altitude, k * 0.3f), climb.update(d.verticalSpeed, k * 0.3f),
                    autopilot->getTargetAltitude());
    }

    void formatHeading(float k)
    {
        const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
        if (!mag) { line.print("компаса нет в схеме"); return; }
        if (!mag->isAvailable()) { line.print("компас не отвечает"); return; }

        line.printf("курс %.0f°", heading.update(mag->getMagData().headingDegrees, k * 1.0f));
    }

    void formatGps(float k)
    {
        const GpsSensor* gps = autopilot ? autopilot->getGpsSensor() : nullptr;
        if (!gps) { line.print("GPS нет в схеме"); return; }
        if (!gps->isAvailable()) { line.print("GPS не отвечает"); return; }

        const GpsData& d = gps->getGpsData();
        // ~1 м по координатам, 0.3 м/с по скорости.
        line.printf("fix=%u sats=%u lat %.6f lon %.6f v %.1f м/с hacc %.1f м",
                    d.fixType, d.numSatellites,
                    gpsLat.update(d.latitude, k * 0.00001f), gpsLon.update(d.longitude, k * 0.00001f),
                    gpsSpeed.update(d.groundSpeed, k * 0.3f), d.horizontalAccuracy);
    }

    void formatImu(float k)
    {
        const char* problem;
        if (!imuReady(problem)) { line.print(problem); return; }

        const ImuData& d = autopilot->getImuSensor()->getImuData();
        line.printf("gyro %+.1f %+.1f %+.1f °/с  acc %+.2f %+.2f %+.2f g",
                    gyro[0].update(d.gyroX, k * 1.0f), gyro[1].update(d.gyroY, k * 1.0f),
                    gyro[2].update(d.gyroZ, k * 1.0f),
                    accel[0].update(d.accelX, k * 0.03f), accel[1].update(d.accelY, k * 0.03f),
                    accel[2].update(d.accelZ, k * 0.03f));
    }

    void formatSystem()
    {
        const IBusReceiver& receiver = controller.getReceiver();
        if (!loopStats)
        {
            line.print("loop n/a");
        }
        else if (loopStats->hz == 0)
        {
            line.print("loop: статистика ещё набирается");   // первые секунды после старта
        }
        else
        {
            line.printf("loop %u Hz, avg %u us, max %u us (худший за 10 с)",
                        (unsigned)loopStats->hz, (unsigned)loopStats->avgUs,
                        (unsigned)loopStats->takePeakUs());
        }
        line.printf(" | iBUS ok=%u crc_err=%u | heap %u KB | uptime %u s",
                    (unsigned)receiver.getGoodFrameCount(), (unsigned)receiver.getBadFrameCount(),
                    (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(millis() / 1000));
    }
};
