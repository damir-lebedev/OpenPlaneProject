#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "hal/IBoard.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
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

    explicit WebDebugServer(FlightController& flightController, Autopilot* ap = nullptr);

    bool begin();

    // Вызывать из полётного цикла: применяет команды с дашборда в
    // контексте задачи, которая владеет автопилотом.
    void applyPendingCommands();


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

    static void serverTask(void* arg);


    // ---------------- GET /api/status ----------------

    String buildStatusJson() const;

    // "name":{"attached":..,"available":.. — без закрывающей скобки.
    static void openSensor(String& json, const char* name, const Sensor* sensor);

    static void field(String& json, const char* name, double value, unsigned decimals);

    void appendImu(String& json) const;

    void appendBaro(String& json) const;

    void appendMag(String& json) const;

    void appendGps(String& json) const;

    void appendAirspeed(String& json) const;

    void appendAutopilot(String& json) const;


    // ---------------- POST-команды ----------------

    bool requireBodyAndAutopilot();

    void handleSetMode();

    // Не указанные поля остаются текущими.
    void handleSetPid();

    // Минимальный разбор плоского JSON вида {"key": 123.45} — проект
    // намеренно не тащит ArduinoJson ради двух POST-запросов. Пробелы
    // вокруг двоеточия допустимы; число — в любой записи JSON, включая
    // экспоненту (JSON.stringify(0.0000001) даёт "1e-7").
    static float extractJsonNumber(const String& body, const char* key, float fallback);

    static bool isJsonSpace(char c);

    static bool isJsonNumberChar(char c);
};
