#pragma once

// ============================================================
// 🌐 WEB DEBUG SERVER
//
// Простой HTTP сервер для отладки и туёвши через Wi-Fi
// 
// Режимы работы:
//   1. AP Mode: ESP32 создаёт своё сеть (OpenPlane-Debug)
//   2. STA Mode: Подключается к существующему Wi-Fi
//
// Функциональность:
//   • JSON API для получения данных датчиков в реальном времени
//   • HTML дашборд с живыми графиками
//   • Настройка PID коэффициентов
//   • Переключение режимов автопилота
//   • WebSocket для потока данных (опционально)
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include "FlightController.h"
#include "Autopilot.h"
#include "FeatureManager.h"

// ============================================================
// КОНФИГУРАЦИЯ СЕТИ
// ============================================================

#define WIFI_SSID_AP "OpenPlane-Debug"
#define WIFI_PASSWORD_AP "12345678"
#define WIFI_PORT 80

// ============================================================
// WEB SERVER
// ============================================================

class WebDebugServer
{
public:

    // ========================================================
    // КОНСТРУКТОР
    // ========================================================

    WebDebugServer(FlightController* fc = nullptr,
                   Autopilot* ap = nullptr,
                   FeatureManager* fm = nullptr)
        : flightController(fc),
          autopilot(ap),
          featureManager(fm),
          webServer(WIFI_PORT),
          isRunning(false)
    {
    }

    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ
    // ========================================================
    // mode: 0 = AP (точка доступа), 1 = STA (клиент существующей сети)

    bool begin(int mode = 0)
    {
        if (mode == 0)
        {
            // Режим точки доступа (AP)
            Serial.print("📡 WebServer: Starting AP mode... ");
            WiFi.mode(WIFI_AP);

            if (!WiFi.softAP(WIFI_SSID_AP, WIFI_PASSWORD_AP))
            {
                Serial.println("FAILED");
                return false;
            }

            Serial.println("OK");
            Serial.print("   SSID: ");
            Serial.println(WIFI_SSID_AP);
            Serial.print("   Password: ");
            Serial.println(WIFI_PASSWORD_AP);
            Serial.print("   IP: ");
            Serial.println(WiFi.softAPIP());
        }
        else
        {
            // Режим станции (STA) - подключение к существующей сети
            // (Не реализовано в этой заглушке, нужно добавить конфигурацию)
            Serial.println("❌ WebServer: STA mode not implemented yet");
            return false;
        }

        // Настраиваем маршруты
        setupRoutes();

        // Запускаем сервер
        webServer.begin();
        isRunning = true;

        Serial.println("✅ WebServer: Started successfully");
        Serial.println("   Open browser: http://192.168.4.1");

        return true;
    }

    // ========================================================
    // ОБНОВЛЕНИЕ СЕРВЕРА
    // ========================================================
    // Должна вызваться регулярно из loop()

    void update()
    {
        if (isRunning)
        {
            webServer.handleClient();
        }
    }

    // ========================================================
    // ДИАГНОСТИКА
    // ========================================================

    void printStatus() const
    {
        Serial.println("\n🌐 WebServer Status:");
        Serial.print("  Running: ");
        Serial.println(isRunning ? "YES" : "NO");

        if (isRunning)
        {
            Serial.print("  Connected clients: ");
            Serial.println(WiFi.softAPgetStationNum());
            Serial.print("  IP address: ");
            Serial.println(WiFi.softAPIP());
        }
    }

private:

    // ========================================================
    // ПРИВАТНЫЕ ПЕРЕМЕННЫЕ
    // ========================================================

    FlightController* flightController;
    Autopilot* autopilot;
    FeatureManager* featureManager;

    WebServer webServer;
    bool isRunning;


    // ========================================================
    // НАСТРОЙКА МАРШРУТОВ
    // ========================================================

    void setupRoutes()
    {
        // Главная страница (HTML)
        webServer.on("/", [this]() { handleRoot(); });

        // JSON API для данных датчиков
        webServer.on("/api/sensors", [this]() { handleSensorsAPI(); });

        // JSON API для состояния автопилота
        webServer.on("/api/autopilot", [this]() { handleAutopilotAPI(); });

        // JSON API для конфигурации функций
        webServer.on("/api/features", [this]() { handleFeaturesAPI(); });

        // POST: Установить режим автопилота
        webServer.on("/api/setmode", HTTP_POST, [this]() { handleSetMode(); });

        // POST: Установить PID коэффициенты
        webServer.on("/api/setpid", HTTP_POST, [this]() { handleSetPID(); });

        // 404 - файл не найден
        webServer.onNotFound([this]() { handleNotFound(); });
    }


    // ========================================================
    // ГЛАВНАЯ СТРАНИЦА (HTML + CSS + JS)
    // ========================================================

    void handleRoot()
    {
        // Простой HTML для дашборда (без специальных символов)
        String html = 
            "<!DOCTYPE html>"
            "<html><head><title>OpenPlane Debug</title>"
            "<meta charset='UTF-8'><meta name='viewport' content='width=device-width'>"
            "<style>"
            "body{font-family:Arial;background:#667eea;margin:0;padding:20px}"
            ".container{max-width:1200px;margin:0 auto}"
            "h1{color:white;text-align:center;margin:20px 0}"
            ".card{background:white;border-radius:10px;padding:20px;margin:10px 0;box-shadow:0 4px 8px rgba(0,0,0,0.2)}"
            ".card h2{color:#667eea;border-bottom:2px solid #667eea;padding-bottom:10px}"
            ".row{display:flex;justify-content:space-between;padding:5px 0}"
            ".label{font-weight:bold;color:#333}"
            ".value{color:#667eea;font-weight:bold}"
            ".button{background:#667eea;color:white;border:none;padding:10px 20px;margin:5px;border-radius:5px;cursor:pointer}"
            ".button:hover{background:#764ba2}"
            ".button.danger{background:#f44336}"
            ".button.success{background:#4CAF50}"
            "</style></head><body>"
            "<div class='container'>"
            "<h1>OpenPlane Debug Dashboard</h1>"
            
            "<div class='card'>"
            "<h2>Sensors</h2>"
            "<div class='row'><span class='label'>Roll:</span><span class='value' id='roll'>--</span></div>"
            "<div class='row'><span class='label'>Pitch:</span><span class='value' id='pitch'>--</span></div>"
            "<div class='row'><span class='label'>Yaw:</span><span class='value' id='yaw'>--</span></div>"
            "<div class='row'><span class='label'>Altitude:</span><span class='value' id='altitude'>--</span></div>"
            "<div class='row'><span class='label'>Climb Rate:</span><span class='value' id='climb'>--</span></div>"
            "</div>"
            
            "<div class='card'>"
            "<h2>Autopilot</h2>"
            "<div class='row'><span class='label'>Mode:</span><span class='value' id='mode'>MANUAL</span></div>"
            "<div class='row'><span class='label'>Desired Roll:</span><span class='value' id='desired-roll'>0</span></div>"
            "<div class='row'><span class='label'>Desired Pitch:</span><span class='value' id='desired-pitch'>0</span></div>"
            "<div class='row'><span class='label'>Target Alt:</span><span class='value' id='target-alt'>0</span></div>"
            "</div>"
            
            "<div class='card'>"
            "<h2>Controls</h2>"
            "<button class='button success' onclick='setMode(2)'>Takeoff</button>"
            "<button class='button success' onclick='setMode(3)'>Alt Hold</button>"
            "<button class='button success' onclick='setMode(1)'>Stabilize</button>"
            "<button class='button danger' onclick='setMode(0)'>Manual</button>"
            "</div>"
            
            "</div><script>"
            "setInterval(updateDashboard,200);"
            "async function updateDashboard(){"
            "try{"
            "const s=await fetch('/api/sensors');const sensors=await s.json();"
            "document.getElementById('roll').textContent=sensors.roll.toFixed(2);"
            "document.getElementById('pitch').textContent=sensors.pitch.toFixed(2);"
            "document.getElementById('yaw').textContent=sensors.yaw.toFixed(2);"
            "document.getElementById('altitude').textContent=sensors.altitude.toFixed(2);"
            "document.getElementById('climb').textContent=sensors.climb.toFixed(2);"
            "const a=await fetch('/api/autopilot');const ap=await a.json();"
            "const modes=['MANUAL','STABILIZE','AUTO_TAKEOFF','ALT_HOLD'];"
            "document.getElementById('mode').textContent=modes[ap.mode]||'UNKNOWN';"
            "document.getElementById('desired-roll').textContent=ap.desired_roll.toFixed(1);"
            "document.getElementById('desired-pitch').textContent=ap.desired_pitch.toFixed(1);"
            "document.getElementById('target-alt').textContent=ap.target_alt.toFixed(1);"
            "}catch(e){console.error(e);}}"
            "async function setMode(m){"
            "try{await fetch('/api/setmode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})});"
            "}catch(e){console.error(e);}}"
            "updateDashboard();"
            "</script></body></html>";

        webServer.send(200, "text/html", html);
    }


    // ========================================================
    // API: GET /api/sensors
    // ========================================================

    void handleSensorsAPI()
{
    String json = "{";

    if (autopilot)
    {
        ImuSensor* imuSensor = autopilot->getImuSensor();
        if (imuSensor)
        {
            const ImuData& imu = imuSensor->getImuData();
            json += "\"roll\":" + String(imu.roll, 2) + ",";
            json += "\"pitch\":" + String(imu.pitch, 2) + ",";
            json += "\"yaw\":" + String(imu.yaw, 2) + ",";
        }

        BarometerSensor* baroSensor = autopilot->getBarometerSensor();
        if (baroSensor)
        {
            const BarometerData& baro = baroSensor->getBarometerData();
            json += "\"altitude\":" + String(baro.altitude, 2) + ",";
            json += "\"climb\":" + String(baro.verticalSpeed, 2);
        }
    }

    json += "}";

    webServer.send(200, "application/json", json);
}


    // ========================================================
    // API: GET /api/autopilot
    // ========================================================

    void handleAutopilotAPI()
    {
        String json = "{";

        if (autopilot)
        {
            json += "\"mode\":" + String((int)autopilot->getMode()) + ",";
            json += "\"desired_roll\":" + String(autopilot->getDesiredRoll(), 1) + ",";
            json += "\"desired_pitch\":" + String(autopilot->getDesiredPitch(), 1) + ",";
            json += "\"target_alt\":" + String(autopilot->getTargetAltitude(), 1);
        }

        json += "}";

        webServer.send(200, "application/json", json);
    }


    // ========================================================
    // API: GET /api/features
    // ========================================================

    void handleFeaturesAPI()
    {
        String json = "{";

        if (featureManager)
        {
            const char* featureNames[] = {"DISABLED", "AUTO_TAKEOFF", "ALT_HOLD", "STABILIZE", "MANUAL"};

            for (int i = 0; i < 4; i++)
            {
                int ch = 7 + i;
                int feat = (int)featureManager->getFeature(ch);
                bool active = featureManager->isFeatureActive(ch);

                json += "\"ch" + String(ch) + "\":\"" + featureNames[feat] + "\",";
                json += "\"ch" + String(ch) + "_active\":" + String(active ? "true" : "false");

                if (i < 3) json += ",";
            }
        }

        json += "}";

        webServer.send(200, "application/json", json);
    }


    // ========================================================
    // API: POST /api/setmode
    // ========================================================

    void handleSetMode()
    {
        if (webServer.hasArg("plain"))
        {
            String body = webServer.arg("plain");
            // Простой парсер (в реальности нужен JSON парсер)
            int mode = 0;
            if (body.indexOf("\"mode\":1") > -1) mode = 1;
            else if (body.indexOf("\"mode\":2") > -1) mode = 2;
            else if (body.indexOf("\"mode\":3") > -1) mode = 3;

            if (autopilot)
            {
                autopilot->setMode((AutopilotMode)mode);
            }

            webServer.send(200, "application/json", "{\"status\":\"ok\"}");
        }
        else
        {
            webServer.send(400, "application/json", "{\"error\":\"No data\"}");
        }
    }


    // ========================================================
    // API: POST /api/setpid
    // ========================================================

    void handleSetPID()
    {
        webServer.send(501, "application/json", "{\"error\":\"Not implemented\"}");
    }


    // ========================================================
    // 404 - Not Found
    // ========================================================

    void handleNotFound()
    {
        webServer.send(404, "text/plain", "404 - Not Found");
    }
};
