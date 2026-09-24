#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "autopilot/Autopilot.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "telemetry/WebDashboardPage.h"

// ============================================================
// WEB DEBUG SERVER
//
// HTTP-дашборд по Wi-Fi (точка доступа Config::WIFI_AP_SSID):
//   GET  /             — страница (WebDashboardPage.h)
//   GET  /api/status   — всё состояние борта одним JSON
//   POST /api/setmode  — {mode: 0..3}
//   POST /api/setpid   — {kpRoll, kiRoll, ... kdPitch}, любые поля
//
// Сервер крутится в отдельной FreeRTOS-задаче на ядре 0 (там же
// Wi-Fi), а не в полётном цикле: WebServer синхронный, и медленный
// клиент или сборка ответа задерживали бы FlightController::update().
// Чтение состояния для /api/status из другой задачи безопасно
// (отдельные 16/32-битные поля, в худшем случае — значения из
// соседних циклов). Команды setmode/setpid из веб-задачи напрямую не
// применяются: они кладутся в "почтовый ящик" под спинлоком, и
// полётный цикл забирает их сам (applyPendingCommands() в loop()).
//
// Поля attached/available в JSON есть всегда — дашборд честно
// различает "нет в сборке" и "есть, но не отвечает".
// ============================================================

class WebDebugServer
{
public:

    WebDebugServer(FlightController& controller, Autopilot* autopilot = nullptr)
        : controller(controller),
          autopilot(autopilot),
          webServer(Config::WEB_SERVER_PORT)
    {
    }

    bool begin()
    {
        Serial.print("WebDebugServer: запуск точки доступа... ");

        // Не сохранять настройки Wi-Fi во флеш: они и так в Config, а
        // запись во флеш останавливает оба ядра — полётный цикл вставал
        // на ~0.3 с в первые секунды после загрузки.
        WiFi.persistent(false);
        WiFi.mode(WIFI_AP);

        if (!WiFi.softAP(Config::WIFI_AP_SSID, Config::WIFI_AP_PASSWORD))
        {
            Serial.println("FAILED");
            return false;
        }

        Serial.println("OK");
        Serial.print("  SSID: "); Serial.println(Config::WIFI_AP_SSID);
        Serial.print("  http://"); Serial.println(WiFi.softAPIP());

        webServer.on("/", [this]() { webServer.send_P(200, "text/html", WebDashboardPage::HTML); });
        webServer.on("/api/status", [this]() { webServer.send(200, "application/json", buildStatusJson()); });
        webServer.on("/api/setmode", HTTP_POST, [this]() { handleSetMode(); });
        webServer.on("/api/setpid", HTTP_POST, [this]() { handleSetPid(); });
        webServer.onNotFound([this]() { webServer.send(404, "text/plain", "404 - Not Found"); });
        webServer.begin();

        // Ядро 0, приоритет 1 — рядом с Wi-Fi, подальше от полётного
        // цикла (loop() крутится на ядре 1).
        xTaskCreatePinnedToCore(serverTask, "web", 8192, this, 1, nullptr, 0);
        return true;
    }

    // Вызывать из полётного цикла: применяет команды с дашборда в
    // контексте задачи, которая владеет автопилотом.
    void applyPendingCommands()
    {
        if (!autopilot || (!pending.hasMode && !pending.hasPid)) return;

        PendingCommands commands;
        portENTER_CRITICAL(&pendingLock);
        commands = pending;
        pending.hasMode = false;
        pending.hasPid = false;
        portEXIT_CRITICAL(&pendingLock);

        if (commands.hasMode)
        {
            autopilot->setMode(commands.mode);
        }

        if (commands.hasPid)
        {
            autopilot->setPIDGains(commands.pid[0], commands.pid[1], commands.pid[2],
                                   commands.pid[3], commands.pid[4], commands.pid[5]);
            Serial.println("WebDebugServer: PID обновлены с дашборда");
        }
    }


private:

    FlightController& controller;
    Autopilot* autopilot;
    WebServer webServer;

    struct PendingCommands
    {
        bool hasMode = false;
        AutopilotMode mode = MODE_MANUAL;
        bool hasPid = false;
        float pid[6] = {};  // kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch
    };

    PendingCommands pending;
    portMUX_TYPE pendingLock = portMUX_INITIALIZER_UNLOCKED;

    static void serverTask(void* arg)
    {
        WebDebugServer* self = static_cast<WebDebugServer*>(arg);

        for (;;)
        {
            self->webServer.handleClient();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }


    // ---------------- GET /api/status ----------------

    String buildStatusJson() const
    {
        String json;
        json.reserve(1024);

        json += "{\"rc\":[";
        const RcChannelState& rc = controller.getRcState();
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            if (i) json += ',';
            json += rc.get(i);
        }
        json += ']';

        json += ",\"armed\":";
        json += controller.isArmed() ? "true" : "false";
        json += ",\"failsafe\":";
        json += controller.isReceiverFailsafe() ? "true" : "false";
        json += ",\"flapsUs\":";
        json += controller.getFlapsUs();

        json += ",\"outputs\":{";
        const FlightOutputs& outputs = controller.getOutputs();
        for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
        {
            if (ch) json += ',';
            json += '"'; json += FlightOutputs::outputInfo(ch).key; json += "\":{\"us\":";
            json += FlightOutputs::valueOf(outputs.getLastState(), ch);
            json += ",\"attached\":";
            json += outputs.isAttached(ch) ? "true" : "false";
            json += '}';
        }
        json += '}';

        appendImu(json);
        appendBaro(json);
        appendMag(json);
        appendGps(json);
        appendAutopilot(json);

        json += '}';
        return json;
    }

    // "name":{"attached":..,"available":.. — без закрывающей скобки.
    static void openSensor(String& json, const char* name, const Sensor* sensor)
    {
        json += ",\""; json += name; json += "\":{\"attached\":";
        json += sensor ? "true" : "false";
        json += ",\"available\":";
        json += (sensor && sensor->isAvailable()) ? "true" : "false";
    }

    static void field(String& json, const char* name, double value, unsigned decimals)
    {
        json += ",\""; json += name; json += "\":";
        json += String(value, decimals);
    }

    void appendImu(String& json) const
    {
        const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        openSensor(json, "imu", imu);
        if (imu && imu->isAvailable())
        {
            const ImuData& d = imu->getImuData();
            field(json, "roll", d.roll, 2);
            field(json, "pitch", d.pitch, 2);
            field(json, "yaw", d.yaw, 2);
        }
        json += '}';
    }

    void appendBaro(String& json) const
    {
        const BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
        openSensor(json, "baro", baro);
        if (baro && baro->isAvailable())
        {
            const BarometerData& d = baro->getBarometerData();
            field(json, "altitude", d.altitude, 2);
            field(json, "climb", d.verticalSpeed, 2);
        }
        json += '}';
    }

    void appendMag(String& json) const
    {
        const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
        openSensor(json, "mag", mag);
        if (mag && mag->isAvailable())
        {
            field(json, "heading", mag->getMagData().headingDegrees, 1);
        }
        json += '}';
    }

    void appendGps(String& json) const
    {
        const GpsSensor* gps = autopilot ? autopilot->getGpsSensor() : nullptr;
        openSensor(json, "gps", gps);
        if (gps && gps->isAvailable())
        {
            const GpsData& d = gps->getGpsData();
            field(json, "fix", d.fixType, 0);
            field(json, "numSV", d.numSatellites, 0);
            field(json, "lat", d.latitude, 6);
            field(json, "lon", d.longitude, 6);
            field(json, "alt", d.altitude, 1);
        }
        json += '}';
    }

    void appendAutopilot(String& json) const
    {
        json += ",\"autopilot\":{\"attached\":";
        json += autopilot ? "true" : "false";
        if (autopilot)
        {
            json += ",\"mode\":"; json += (int)autopilot->getMode();
            json += ",\"modeName\":\""; json += autopilot->getModeName(); json += '"';
            field(json, "desiredRoll", autopilot->getDesiredRoll(), 1);
            field(json, "desiredPitch", autopilot->getDesiredPitch(), 1);
            field(json, "targetAlt", autopilot->getTargetAltitude(), 1);
            field(json, "rollCorr", autopilot->getRollCorrection(), 1);
            field(json, "pitchCorr", autopilot->getPitchCorrection(), 1);
            field(json, "throttleCorr", autopilot->getThrottleCorrection(), 1);
            field(json, "kpRoll", autopilot->getRollPid().getKp(), 3);
            field(json, "kiRoll", autopilot->getRollPid().getKi(), 3);
            field(json, "kdRoll", autopilot->getRollPid().getKd(), 3);
            field(json, "kpPitch", autopilot->getPitchPid().getKp(), 3);
            field(json, "kiPitch", autopilot->getPitchPid().getKi(), 3);
            field(json, "kdPitch", autopilot->getPitchPid().getKd(), 3);
        }
        json += '}';
    }


    // ---------------- POST-команды ----------------

    bool requireBodyAndAutopilot()
    {
        if (!webServer.hasArg("plain"))
        {
            webServer.send(400, "application/json", "{\"error\":\"no data\"}");
            return false;
        }
        if (!autopilot)
        {
            webServer.send(503, "application/json", "{\"error\":\"autopilot not attached\"}");
            return false;
        }
        return true;
    }

    void handleSetMode()
    {
        if (!requireBodyAndAutopilot()) return;

        const int mode = (int)extractJsonNumber(webServer.arg("plain"), "mode", -1);
        if (mode < MODE_MANUAL || mode > MODE_ALT_HOLD)
        {
            webServer.send(400, "application/json", "{\"error\":\"invalid mode\"}");
            return;
        }

        portENTER_CRITICAL(&pendingLock);
        pending.mode = (AutopilotMode)mode;
        pending.hasMode = true;
        portEXIT_CRITICAL(&pendingLock);

        webServer.send(200, "application/json", "{\"status\":\"ok\"}");
    }

    // Не указанные поля остаются текущими.
    void handleSetPid()
    {
        if (!requireBodyAndAutopilot()) return;

        const String body = webServer.arg("plain");
        const PidController& roll = autopilot->getRollPid();
        const PidController& pitch = autopilot->getPitchPid();

        const float values[6] = {
            extractJsonNumber(body, "kpRoll", roll.getKp()),
            extractJsonNumber(body, "kiRoll", roll.getKi()),
            extractJsonNumber(body, "kdRoll", roll.getKd()),
            extractJsonNumber(body, "kpPitch", pitch.getKp()),
            extractJsonNumber(body, "kiPitch", pitch.getKi()),
            extractJsonNumber(body, "kdPitch", pitch.getKd()),
        };

        portENTER_CRITICAL(&pendingLock);
        memcpy(pending.pid, values, sizeof(values));
        pending.hasPid = true;
        portEXIT_CRITICAL(&pendingLock);

        webServer.send(200, "application/json", "{\"status\":\"ok\"}");
    }

    // Минимальный разбор плоского JSON вида {"key":123.45} — проект
    // намеренно не тащит ArduinoJson ради двух POST-запросов.
    static float extractJsonNumber(const String& body, const char* key, float fallback)
    {
        const String needle = String("\"") + key + "\":";
        const int start = body.indexOf(needle);
        if (start < 0) return fallback;

        const int from = start + needle.length();
        int to = from;
        while (to < (int)body.length() && (isDigit(body[to]) || body[to] == '-' || body[to] == '.'))
        {
            to++;
        }

        return to == from ? fallback : body.substring(from, to).toFloat();
    }
};
