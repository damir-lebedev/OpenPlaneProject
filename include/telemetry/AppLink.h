#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "control/FlightOutputState.h"
#include "control/GroundCalibration.h"
#include "rc/IBusReceiver.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/LoopStats.h"

// ============================================================
// ЖИВАЯ СВЯЗЬ С ПРИЛОЖЕНИЕМ OpenPlane Studio
//
// Команды ПК — как "id" и "bb": STX (0x02), строка, '\n' (DebugConsole):
//   live    — присылать состояние строками "@live {...}" ещё LIVE_HOLD_MS.
//             Приложение повторяет команду раз в секунду, пока слушает:
//             закрыли приложение или выдернули кабель — плата замолкает
//             сама и не тратит на это полётный цикл;
//   live 0  — замолчать сразу.
//
// "@live {...}" — одна строка JSON: ARM, режим, связь с пультом, каналы,
// выходы, углы, датчики, причина запрета ARM (FlightController::
// armBlockNow()), шаг наземной калибровки, включённые функции, частота
// цикла. Печатается, только если в буфере порта есть место под строку
// целиком: полётный цикл никогда не ждёт ПК — места нет, кадр пропущен.
// ============================================================

class AppLink
{
public:

    // USB CDC у STM32 — 20 кадров в секунду; UART 115200 у ESP32 — 10.
#if defined(BOARD_STM32H743)
    static constexpr uint32_t FRAME_PERIOD_MS = 50;
#else
    static constexpr uint32_t FRAME_PERIOD_MS = 100;
#endif
    static constexpr uint32_t LIVE_HOLD_MS = 3000;
    static constexpr size_t LIVE_LINE_MAX = 768;   // не LINE_MAX: это макрос <limits.h>

    explicit AppLink(FlightController& flightController, Autopilot* ap = nullptr, LoopStats* stats = nullptr)
        : controller(flightController),
          autopilot(ap),
          loopStats(stats)
    {
    }

    // Команда ПК (без STX и '\n'). true — команда наша.
    bool handleHostCommand(const char* command, uint32_t nowMs)
    {
        if (strcmp(command, "live") == 0 || strcmp(command, "live 1") == 0)
        {
            live = true;
            liveUntilMs = nowMs + LIVE_HOLD_MS;
            return true;
        }
        if (strcmp(command, "live 0") == 0)
        {
            live = false;
            return true;
        }
        return false;
    }

    // Вызывать каждый такт.
    void update(uint32_t nowMs)
    {
        if (!live) return;
        if (static_cast<int32_t>(nowMs - liveUntilMs) >= 0)
        {
            live = false;
            return;
        }
        if (framesSent + framesSkipped > 0 && nowMs - lastFrameMs < FRAME_PERIOD_MS) return;
        lastFrameMs = nowMs;

        char line[LIVE_LINE_MAX];
        const size_t length = formatLive(line, sizeof(line), nowMs);
        if (length == 0 || Serial.availableForWrite() < static_cast<int>(length))
        {
            framesSkipped++;
            return;
        }
        Serial.write(reinterpret_cast<const uint8_t*>(line), length);
        framesSent++;
    }

    bool isLive() const { return live; }
    uint32_t getFramesSent() const { return framesSent; }
    uint32_t getFramesSkipped() const { return framesSkipped; }

    // Строка "@live {...}\n" в buffer; 0 — не поместилась.
    size_t formatLive(char* buffer, size_t capacity, uint32_t nowMs) const
    {
        Writer w(buffer, capacity);
        const IBusReceiver& receiver = controller.getReceiver();
        const RcChannelState& rc = controller.getRcState();
        const FlightOutputState& out = controller.getOutputState();

        w.add("@live {\"t\":%lu,\"arm\":%s,\"mode\":", static_cast<unsigned long>(nowMs),
              controller.isArmed() ? "true" : "false");
        w.str(autopilot ? autopilot->getModeName() : "MANUAL");
        w.add(",\"rx\":\"%s\"", receiver.isFrameTimeout() ? "lost" : receiver.isFailsafeReported() ? "fs" : "ok");

        w.add(",\"ch\":[");
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            w.add(i ? ",%u" : "%u", static_cast<unsigned>(rc.get(i)));
        }
        w.add("],\"out\":[%u,%u,%u,%u,%u,%u,%u],\"rev\":%s", out.aileronLeft, out.aileronRight, out.elevator,
              out.rudder, out.throttle, out.aux1, out.aux2, out.motorReverse ? "true" : "false");

        const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        w.add(",\"imu\":\"%s\"", !imu                          ? "none"
                                 : !imu->isAvailable()         ? "noresp"
                                 : imu->getPreflightProblem()  ? "check"
                                                               : "ok");
        if (imu && imu->isAvailable())
        {
            const ImuData& d = imu->getImuData();
            w.add(",\"att\":[%.1f,%.1f,%.1f]", d.roll, d.pitch, d.yaw);
        }

        const BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
        w.add(",\"baro\":\"%s\"", sensorState(baro));
        if (baro && baro->isAvailable())
        {
            const BarometerData& d = baro->getBarometerData();
            w.add(",\"alt\":%.2f,\"vz\":%.2f", d.altitude, d.verticalSpeed);
        }

        const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
        w.add(",\"mag\":\"%s\"", sensorState(mag));
        if (mag && mag->isAvailable()) w.add(",\"hdg\":%.0f", mag->getMagData().headingDegrees);

        const GpsSensor* gps = autopilot ? autopilot->getGpsSensor() : nullptr;
        w.add(",\"gps\":\"%s\"", sensorState(gps));
        if (gps && gps->isAvailable())
        {
            const GpsData& d = gps->getGpsData();
            w.add(",\"fix\":%u,\"sats\":%u", static_cast<unsigned>(d.fixType), static_cast<unsigned>(d.numSatellites));
        }

        const AirspeedSensor* pitot = autopilot ? autopilot->getAirspeedSensor() : nullptr;
        w.add(",\"pitot\":\"%s\"", sensorState(pitot));
        if (pitot && pitot->isAvailable()) w.add(",\"ias\":%.1f", pitot->getAirspeedData().indicatedMs);

        w.add(",\"block\":");
        const char* block = controller.armBlockNow();
        if (block) w.str(block);
        else w.add("null");

        const GroundCalibration::Stage stage = controller.getCalibration().getStage();
        w.add(",\"cal\":%s", stage == GroundCalibration::Stage::LEVEL ? "\"LEVEL\""
                             : stage == GroundCalibration::Stage::PARK ? "\"PARK\""
                                                                      : "null");

        w.add(",\"feat\":[");
        if (autopilot)
        {
            const PilotInputs& in = autopilot->getInputs();
            bool first = true;
            for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
            {
                if (!in.features[f]) continue;
                if (!first) w.add(",");
                first = false;
                w.str(AutopilotNames::feature(static_cast<Feature>(f)));
            }
        }
        w.add("],\"hz\":%u}\n", loopStats ? static_cast<unsigned>(loopStats->hz) : 0u);
        return w.overflow ? 0 : w.length;
    }

private:

    // Дописывает в буфер с проверкой границ; не влезло — overflow.
    struct Writer
    {
        Writer(char* out, size_t size) : buffer(out), capacity(size) {}

        char* buffer;
        size_t capacity;
        size_t length = 0;
        bool overflow = false;

        void add(const char* format, ...) __attribute__((format(printf, 2, 3)))
        {
            if (overflow) return;
            va_list args;
            va_start(args, format);
            const int n = vsnprintf(buffer + length, capacity - length, format, args);
            va_end(args);
            if (n < 0 || static_cast<size_t>(n) >= capacity - length)
            {
                overflow = true;
                return;
            }
            length += static_cast<size_t>(n);
        }

        // Строка JSON: " и \ экранируются, управляющие символы — пропуск,
        // UTF-8 (причины ARM по-русски) — как есть.
        void str(const char* text)
        {
            add("\"");
            for (const char* p = text; *p && !overflow; ++p)
            {
                const char c = *p;
                if (c == '"' || c == '\\') add("\\%c", c);
                else if (static_cast<unsigned char>(c) >= 0x20) add("%c", c);
            }
            add("\"");
        }
    };

    template <typename SensorType>
    static const char* sensorState(const SensorType* sensor)
    {
        return !sensor ? "none" : sensor->isAvailable() ? "ok" : "noresp";
    }

    FlightController& controller;
    Autopilot* autopilot;
    LoopStats* loopStats;

    bool live = false;
    uint32_t liveUntilMs = 0;
    uint32_t lastFrameMs = 0;
    uint32_t framesSent = 0;
    uint32_t framesSkipped = 0;
};
