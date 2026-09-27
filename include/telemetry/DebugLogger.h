#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "control/FlightOutputState.h"
#include "hal/Rtos.h"
#include "rc/IBusReceiver.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
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
    );

    // Загрузить настройки каналов из NVS.
    void begin();

    void update();

    LogSettings& getSettings() { return settings; }
    void saveSettings() const { settings.save(); }

    // Меню открыто — лог молчит. После — всё включённое заново.
    void suspend(bool isSuspended);

    void setPaused(bool isPaused);

    bool isPaused() const { return paused; }

    // Следующий такт напечатает все включённые каналы, даже без изменений.
    void refresh();


private:

    static constexpr size_t LINE_SIZE = 200;
    static constexpr uint32_t SYSTEM_INTERVAL_MS = 10000;

    // Print в буфер строки — чтобы сравнить её с прошлой перед печатью.
    class LineBuffer : public Print
    {
    public:
        size_t write(uint8_t c) override;

        void reset();

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

        double update(double raw, double band);
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
    Shown homeDistance, navSpeed, airspeedShown;
    Shown heading;
    Shown gpsLat, gpsLon, gpsSpeed;
    Shown gyro[3], accel[3];

    void updateChannel(uint8_t channel, uint32_t now);

    void format(LogChannel channel, bool periodic);

    // --- каналы ---

    void formatStatus();

    void formatRc(float k);

    void formatOutputs(float k);

    bool imuReady(const char*& problem) const;

    void formatAttitude(float k);

    void formatAutopilot(float k);

    // NAV: дом, курс -> цель, скорость (и чем меряна), воздушная
    // скорость трубки, включённые функции тумблеров.
    void formatNav(float k);

    void formatAltitude(float k);

    void formatHeading(float k);

    void formatGps(float k);

    void formatImu(float k);

    void formatSystem();
};
