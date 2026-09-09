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

#pragma once
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
        String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>OpenPlane Debug Dashboard</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 8px 16px rgba(0,0,0,0.2);
        }
        .card h2 {
            color: #667eea;
            margin-bottom: 15px;
            font-size: 1.5em;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .data-row {
            display: flex;
            justify-content: space-between;
            margin: 10px 0;
            font-size: 1.1em;
        }
        .label {
            font-weight: 600;
            color: #333;
        }
        .value {
            color: #667eea;
            font-weight: bold;
        }
        .status {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .status.armed {
            background: #4CAF50;
        }
        .status.disarmed {
            background: #f44336;
        }
        .status.stabilize {
            background: #2196F3;
        }
        .button {
            background: #667eea;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 1em;
            margin: 5px;
            transition: background 0.3s;
        }
        .button:hover {
            background: #764ba2;
        }
        .button.danger {
            background: #f44336;
        }
        .button.danger:hover {
            background: #da190b;
        }
        .button.success {
            background: #4CAF50;
        }
        .button.success:hover {
            background: #45a049;
        }
        .controls {
            text-align: center;
            margin-top: 20px;
        }
        .gauge {
            width: 100%;
            height: 200px;
            border: 2px solid #667eea;
            border-radius: 10px;
            margin: 10px 0;
            background: #f5f5f5;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 2em;
            color: #667eea;
        }
        .slider {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #d3d3d3;
            outline: none;
            -webkit-appearance: none;
            margin: 10px 0;
        }
        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }
        .slider::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>✈️ OpenPlane Debug Dashboard</h1>

        <div class="grid">
            <!-- Датчики -->
            <div class="card">
                <h2>📊 Датчики</h2>
                <div class="data-row">
                    <span class="label">Roll:</span>
                    <span class="value" id="roll">--.--°</span>
                </div>
                <div class="data-row">
                    <span class="label">Pitch:</span>
                    <span class="value" id="pitch">--.--°</span>
                </div>
                <div class="data-row">
                    <span class="label">Yaw:</span>
                    <span class="value" id="yaw">--.--°</span>
                </div>
                <div class="data-row">
                    <span class="label">Altitude:</span>
                    <span class="value" id="altitude">--.-- m</span>
                </div>
                <div class="data-row">
                    <span class="label">Climb Rate:</span>
                    <span class="value" id="climb">--.-- m/s</span>
                </div>
            </div>

            <!-- Состояние Автопилота -->
            <div class="card">
                <h2>🚀 Autopilot</h2>
                <div class="data-row">
                    <span class="label">Mode:</span>
                    <span class="value" id="mode">MANUAL</span>
                </div>
                <div class="data-row">
                    <span class="label">Desired Roll:</span>
                    <span class="value" id="desired-roll">0.0°</span>
                </div>
                <div class="data-row">
                    <span class="label">Desired Pitch:</span>
                    <span class="value" id="desired-pitch">0.0°</span>
                </div>
                <div class="data-row">
                    <span class="label">Target Alt:</span>
                    <span class="value" id="target-alt">0.0 m</span>
                </div>
            </div>

            <!-- Функции (Features) -->
            <div class="card">
                <h2>🎛️ Features</h2>
                <div id="features-list"></div>
            </div>
        </div>

        <!-- Контролы -->
        <div class="card">
            <h2>⚙️ Controls</h2>
            <div class="controls">
                <button class="button success" onclick="setMode(1)">🚀 Takeoff</button>
                <button class="button success" onclick="setMode(3)">📈 Alt Hold</button>
                <button class="button success" onclick="setMode(2)">🛫 Stabilize</button>
                <button class="button danger" onclick="setMode(0)">🛑 Manual</button>
            </div>
        </div>

        <!-- PID Tune (опциональный раздел) -->
        <div class="card">
            <h2>⚙️ PID Configuration</h2>
            <p style="color: #666; font-size: 0.9em;">Экспериментальная функция - используйте с осторожностью!</p>
            <div style="margin-top: 10px; color: #999; font-size: 0.9em;">
                Текущие коэффициенты отображаются в консоли.
            </div>
        </div>
    </div>

    <script>
        // Обновляем данные каждые 200ms
        setInterval(updateDashboard, 200);

        async function updateDashboard() {
            try {
                // Получаем данные датчиков
                const sensorsResp = await fetch('/api/sensors');
                const sensors = await sensorsResp.json();

                document.getElementById('roll').textContent = sensors.roll.toFixed(2) + '°';
                document.getElementById('pitch').textContent = sensors.pitch.toFixed(2) + '°';
                document.getElementById('yaw').textContent = sensors.yaw.toFixed(2) + '°';
                document.getElementById('altitude').textContent = sensors.altitude.toFixed(2) + ' m';
                document.getElementById('climb').textContent = sensors.climb.toFixed(2) + ' m/s';

                // Получаем состояние автопилота
                const apResp = await fetch('/api/autopilot');
                const ap = await apResp.json();

                const modes = ['MANUAL', 'STABILIZE', 'AUTO_TAKEOFF', 'ALT_HOLD'];
                document.getElementById('mode').textContent = modes[ap.mode] || 'UNKNOWN';
                document.getElementById('desired-roll').textContent = ap.desired_roll.toFixed(1) + '°';
                document.getElementById('desired-pitch').textContent = ap.desired_pitch.toFixed(1) + '°';
                document.getElementById('target-alt').textContent = ap.target_alt.toFixed(1) + ' m';

                // Получаем конфигурацию функций
                const featuresResp = await fetch('/api/features');
                const features = await featuresResp.json();

                let featuresList = document.getElementById('features-list');
                featuresList.innerHTML = '';
                for (let i = 0; i < 4; i++) {
                    let ch = 7 + i;
                    let active = features['ch' + ch + '_active'] ? '✓' : '✗';
                    let status = features['ch' + ch + '_active'] ? 'active' : 'inactive';
                    featuresList.innerHTML += `
                        <div class="data-row">
                            <span class="label">CH${ch}:</span>
                            <span class="value" style="color: ${features['ch' + ch + '_active'] ? '#4CAF50' : '#999'}">${active} ${features['ch' + ch]}</span>
                        </div>
                    `;
                }
            } catch (error) {
                console.error('Error updating dashboard:', error);
            }
        }

        async function setMode(mode) {
            try {
                const response = await fetch('/api/setmode', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ mode: mode })
                });
                const result = await response.json();
                console.log('Mode set:', result);
            } catch (error) {
                console.error('Error setting mode:', error);
            }
        }

        // Первое обновление
        updateDashboard();
    </script>
</body>
</html>
        )";

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
            const ImuData& imu = autopilot->imuSensor->getImuData();
            json += "\"roll\":" + String(imu.roll, 2) + ",";
            json += "\"pitch\":" + String(imu.pitch, 2) + ",";
            json += "\"yaw\":" + String(imu.yaw, 2) + ",";
        }

        if (autopilot && autopilot->baroSensor)
        {
            const BarometerData& baro = autopilot->baroSensor->getBarometerData();
            json += "\"altitude\":" + String(baro.altitude, 2) + ",";
            json += "\"climb\":" + String(baro.verticalSpeed, 2);
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
