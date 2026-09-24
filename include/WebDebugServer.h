#pragma once

// ============================================================
// WEB DEBUG SERVER
//
// HTTP-дашборд для отладки по Wi-Fi (AP: OpenPlane-Debug).
// Отдаёт один агрегированный /api/status (RC-каналы, ARM/failsafe,
// выходы с флагом attached, датчики с флагом available,
// автопилот) и принимает setmode/setpid. Режим автопилота с RC
// сейчас выбирается только через CH7 (см. AutopilotModeSelector.h) —
// /api/setmode остаётся отдельным способом сменить режим с
// дашборда, независимо от положения CH7. Явно показывает
// отсутствие датчика/серво, а не молчит про них.
//
// Сервер крутится в отдельной FreeRTOS-задаче на ядре 0 (там же
// Wi-Fi), а не в полётном цикле: WebServer синхронный, и медленный
// клиент или сборка страницы (~8 КБ String) раньше задерживали
// FlightController::update() на миллисекунды-секунды. Чтение
// состояния для /api/status из другой задачи безопасно (отдельные
// 16/32-битные поля, в худшем случае — значения из соседних
// циклов), а команды setmode/setpid не применяются из веб-задачи
// напрямую: они кладутся в "почтовый ящик" под спинлоком, и полётный
// цикл забирает их сам (applyPendingCommands() в loop()).
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include "FlightController.h"
#include "Autopilot.h"

#define WIFI_SSID_AP "OpenPlane-Debug"
#define WIFI_PASSWORD_AP "12345678"
#define WIFI_PORT 80

class WebDebugServer
{
public:

    WebDebugServer(FlightController* fc = nullptr,
                   Autopilot* ap = nullptr)
        : flightController(fc),
          autopilot(ap),
          webServer(WIFI_PORT),
          isRunning(false)
    {
    }

    // mode: 0 = точка доступа (AP), 1 = клиент существующей сети (пока не реализовано)
    bool begin(int mode = 0)
    {
        if (mode != 0)
        {
            Serial.println("WebDebugServer: STA режим ещё не реализован");
            return false;
        }

        Serial.print("WebDebugServer: запуск AP... ");
        WiFi.mode(WIFI_AP);

        if (!WiFi.softAP(WIFI_SSID_AP, WIFI_PASSWORD_AP))
        {
            Serial.println("FAILED");
            return false;
        }

        Serial.println("OK");
        Serial.print("  SSID: "); Serial.println(WIFI_SSID_AP);
        Serial.print("  IP: "); Serial.println(WiFi.softAPIP());

        setupRoutes();
        webServer.begin();
        isRunning = true;

        // Ядро 0, приоритет 1 — рядом с Wi-Fi, подальше от полётного
        // цикла (loop() крутится на ядре 1).
        xTaskCreatePinnedToCore(serverTask, "web", 8192, this, 1, nullptr, 0);

        Serial.println("WebDebugServer: открой http://192.168.4.1 в браузере");

        return true;
    }

    // Вызывать из полётного цикла: применяет команды, пришедшие с
    // дашборда, в контексте той задачи, которая владеет автопилотом.
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

    void printStatus() const
    {
        Serial.print("WebDebugServer: running=");
        Serial.print(isRunning ? "YES" : "NO");

        if (isRunning)
        {
            Serial.print(" clients=");
            Serial.print(WiFi.softAPgetStationNum());
            Serial.print(" ip=");
            Serial.println(WiFi.softAPIP());
        }
        else
        {
            Serial.println();
        }
    }


private:

    FlightController* flightController;
    Autopilot* autopilot;

    WebServer webServer;
    bool isRunning;

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


    void setupRoutes()
    {
        webServer.on("/", [this]() { handleRoot(); });
        webServer.on("/api/status", [this]() { handleStatusAPI(); });
        webServer.on("/api/setmode", HTTP_POST, [this]() { handleSetMode(); });
        webServer.on("/api/setpid", HTTP_POST, [this]() { handleSetPID(); });
        webServer.onNotFound([this]() { handleNotFound(); });
    }


    // ========================================================
    // GET /api/status — всё состояние системы в одном JSON.
    // Поля attached/available всегда присутствуют, поэтому
    // фронтенд может честно показать "нет датчика"/"нет серво"
    // вместо того, чтобы упасть на undefined или молчать.
    // ========================================================

    String buildStatusJson()
    {
        String json = "{";

        json += "\"rc\":[";
        if (flightController)
        {
            const RcChannelState& rc = flightController->getRcState();

            for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
            {
                json += String(rc.get(i));
                if (i + 1 < Config::IBUS_CHANNELS) json += ",";
            }
        }
        json += "],";

        json += "\"armed\":";
        json += (flightController && flightController->isArmed()) ? "true" : "false";
        json += ",\"failsafe\":";
        json += (flightController && flightController->isReceiverFailsafe()) ? "true" : "false";
        json += ",\"flapsUs\":";
        json += String(flightController ? flightController->getFlapsUs() : 0);

        json += ",\"outputs\":{";
        if (flightController)
        {
            const FlightOutputs& outputs = flightController->getOutputs();
            const FlightOutputState& state = outputs.getLastState();

            json += outputJson("aileronLeft", state.aileronLeft, outputs.isAileronLeftAttached()) + ",";
            json += outputJson("aileronRight", state.aileronRight, outputs.isAileronRightAttached()) + ",";
            json += outputJson("elevator", state.elevator, outputs.isElevatorAttached()) + ",";
            json += outputJson("rudder", state.rudder, outputs.isRudderAttached()) + ",";
            json += outputJson("esc", state.throttle, outputs.isEscAttached());
        }
        json += "}";

        ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        json += ",\"imu\":{\"attached\":";
        json += imu ? "true" : "false";
        json += ",\"available\":";
        json += (imu && imu->isAvailable()) ? "true" : "false";
        if (imu && imu->isAvailable())
        {
            const ImuData& d = imu->getImuData();
            json += ",\"roll\":" + String(d.roll, 2);
            json += ",\"pitch\":" + String(d.pitch, 2);
            json += ",\"yaw\":" + String(d.yaw, 2);
        }
        json += "}";

        BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
        json += ",\"baro\":{\"attached\":";
        json += baro ? "true" : "false";
        json += ",\"available\":";
        json += (baro && baro->isAvailable()) ? "true" : "false";
        if (baro && baro->isAvailable())
        {
            const BarometerData& d = baro->getBarometerData();
            json += ",\"altitude\":" + String(d.altitude, 2);
            json += ",\"climb\":" + String(d.verticalSpeed, 2);
        }
        json += "}";

        MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
        json += ",\"mag\":{\"attached\":";
        json += mag ? "true" : "false";
        json += ",\"available\":";
        json += (mag && mag->isAvailable()) ? "true" : "false";
        if (mag && mag->isAvailable())
        {
            const MagData& d = mag->getMagData();
            json += ",\"heading\":" + String(d.headingDegrees, 1);
        }
        json += "}";

        GpsSensor* gps = autopilot ? autopilot->getGpsSensor() : nullptr;
        json += ",\"gps\":{\"attached\":";
        json += gps ? "true" : "false";
        json += ",\"available\":";
        json += (gps && gps->isAvailable()) ? "true" : "false";
        if (gps && gps->isAvailable())
        {
            const GpsData& d = gps->getGpsData();
            json += ",\"fix\":" + String((int)d.fixType);
            json += ",\"numSV\":" + String((int)d.numSatellites);
            json += ",\"lat\":" + String(d.latitude, 6);
            json += ",\"lon\":" + String(d.longitude, 6);
            json += ",\"alt\":" + String(d.altitude, 1);
        }
        json += "}";

        json += ",\"autopilot\":{\"attached\":";
        json += autopilot ? "true" : "false";
        if (autopilot)
        {
            json += ",\"mode\":" + String((int)autopilot->getMode());
            json += ",\"modeName\":\"" + String(autopilot->getModeName()) + "\"";
            json += ",\"desiredRoll\":" + String(autopilot->getDesiredRoll(), 1);
            json += ",\"desiredPitch\":" + String(autopilot->getDesiredPitch(), 1);
            json += ",\"targetAlt\":" + String(autopilot->getTargetAltitude(), 1);
            json += ",\"rollCorr\":" + String(autopilot->getRollCorrection(), 1);
            json += ",\"pitchCorr\":" + String(autopilot->getPitchCorrection(), 1);
            json += ",\"throttleCorr\":" + String(autopilot->getThrottleCorrection(), 1);
            json += ",\"kpRoll\":" + String(autopilot->getRollPid().getKp(), 3);
            json += ",\"kiRoll\":" + String(autopilot->getRollPid().getKi(), 3);
            json += ",\"kdRoll\":" + String(autopilot->getRollPid().getKd(), 3);
            json += ",\"kpPitch\":" + String(autopilot->getPitchPid().getKp(), 3);
            json += ",\"kiPitch\":" + String(autopilot->getPitchPid().getKi(), 3);
            json += ",\"kdPitch\":" + String(autopilot->getPitchPid().getKd(), 3);
        }
        json += "}";

        json += "}";

        return json;
    }

    String outputJson(const char* name, uint16_t us, bool attached) const
    {
        return "\"" + String(name) + "\":{\"us\":" + String(us) +
               ",\"attached\":" + (attached ? "true" : "false") + "}";
    }

    void handleStatusAPI()
    {
        webServer.send(200, "application/json", buildStatusJson());
    }


    // ========================================================
    // POST /api/setmode  {mode: 0..3}
    // ========================================================

    void handleSetMode()
    {
        if (!webServer.hasArg("plain"))
        {
            webServer.send(400, "application/json", "{\"error\":\"no data\"}");
            return;
        }

        if (!autopilot)
        {
            webServer.send(503, "application/json", "{\"error\":\"autopilot not attached\"}");
            return;
        }

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


    // ========================================================
    // POST /api/setpid  {kpRoll,kiRoll,kdRoll,kpPitch,kiPitch,kdPitch}
    // Поля можно не указывать — берётся текущее значение.
    // ========================================================

    void handleSetPID()
    {
        if (!webServer.hasArg("plain"))
        {
            webServer.send(400, "application/json", "{\"error\":\"no data\"}");
            return;
        }

        if (!autopilot)
        {
            webServer.send(503, "application/json", "{\"error\":\"autopilot not attached\"}");
            return;
        }

        const String body = webServer.arg("plain");

        const float kpRoll = extractJsonNumber(body, "kpRoll", autopilot->getRollPid().getKp());
        const float kiRoll = extractJsonNumber(body, "kiRoll", autopilot->getRollPid().getKi());
        const float kdRoll = extractJsonNumber(body, "kdRoll", autopilot->getRollPid().getKd());
        const float kpPitch = extractJsonNumber(body, "kpPitch", autopilot->getPitchPid().getKp());
        const float kiPitch = extractJsonNumber(body, "kiPitch", autopilot->getPitchPid().getKi());
        const float kdPitch = extractJsonNumber(body, "kdPitch", autopilot->getPitchPid().getKd());

        portENTER_CRITICAL(&pendingLock);
        pending.pid[0] = kpRoll;  pending.pid[1] = kiRoll;  pending.pid[2] = kdRoll;
        pending.pid[3] = kpPitch; pending.pid[4] = kiPitch; pending.pid[5] = kdPitch;
        pending.hasPid = true;
        portEXIT_CRITICAL(&pendingLock);

        webServer.send(200, "application/json", "{\"status\":\"ok\"}");
    }


    void handleNotFound()
    {
        webServer.send(404, "text/plain", "404 - Not Found");
    }


    // Простой парсер плоского JSON вида {"key":123.45} — без
    // вложенности и без сторонней библиотеки (проект намеренно не
    // тащит ArduinoJson ради двух POST-запросов).
    static float extractJsonNumber(const String& body, const char* key, float fallback)
    {
        const String needle = String("\"") + key + "\":";
        const int start = body.indexOf(needle);

        if (start < 0) return fallback;

        int pos = start + needle.length();
        const int end = pos;
        int len = body.length();

        while (pos < len && (isDigit(body[pos]) || body[pos] == '-' || body[pos] == '.'))
        {
            pos++;
        }

        if (pos == end) return fallback;

        return body.substring(end, pos).toFloat();
    }


    // ========================================================
    // GET / — HTML-дашборд. Опрашивает /api/status раз в 200мс.
    // ========================================================

    void handleRoot()
    {
        String html =
            "<!DOCTYPE html>"
            "<html><head><title>OpenPlane Debug</title>"
            "<meta charset='UTF-8'><meta name='viewport' content='width=device-width'>"
            "<style>"
            "body{font-family:Arial;background:#667eea;margin:0;padding:20px}"
            ".container{max-width:900px;margin:0 auto}"
            "h1{color:white;text-align:center;margin:20px 0}"
            ".card{background:white;border-radius:10px;padding:20px;margin:10px 0;box-shadow:0 4px 8px rgba(0,0,0,0.2)}"
            ".card h2{color:#667eea;border-bottom:2px solid #667eea;padding-bottom:10px;font-size:18px}"
            ".row{display:flex;justify-content:flex-start;align-items:center;gap:10px;padding:6px 0;flex-wrap:wrap}"
            ".label{font-weight:bold;color:#333;min-width:140px}"
            ".value{color:#667eea;font-weight:bold}"
            ".badge{padding:3px 10px;border-radius:12px;font-size:12px;font-weight:bold;color:white;background:#999}"
            ".badge.ok{background:#4CAF50}"
            ".badge.bad{background:#f44336}"
            ".bartrack{flex:1;min-width:80px;height:10px;background:#eee;border-radius:5px;overflow:hidden}"
            ".barfill{height:100%;background:#667eea;width:50%}"
            ".button{background:#667eea;color:white;border:none;padding:8px 16px;margin:4px;border-radius:5px;cursor:pointer}"
            ".button:hover{background:#764ba2}"
            ".button.danger{background:#f44336}"
            ".button.success{background:#4CAF50}"
            "input[type=number]{width:60px}"
            "</style></head><body>"
            "<div class='container'>"
            "<h1>OpenPlane Debug Dashboard</h1>"

            "<div class='card'>"
            "<h2>Состояние</h2>"
            "<div class='row'>"
            "<span class='label'>Приёмник</span><span class='badge' id='rx'>--</span>"
            "<span class='label'>ARM</span><span class='badge' id='arm'>--</span>"
            "</div></div>"

            "<div class='card'><h2>RC каналы</h2>";

        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            html += "<div class='row'><span class='label'>CH" + String(i + 1) + "</span>"
                    "<div class='bartrack'><div class='barfill' id='ch" + String(i) + "'></div></div>"
                    "<span class='value' id='chv" + String(i) + "'>--</span></div>";
        }

        html +=
            "</div>"

            "<div class='card'><h2>Выходы (Servo/ESC)</h2>"
            "<div class='row'><span class='label'>Left aileron</span><span class='value' id='ail-l-val'>--</span><span class='badge' id='ail-l-badge'>--</span></div>"
            "<div class='row'><span class='label'>Right aileron</span><span class='value' id='ail-r-val'>--</span><span class='badge' id='ail-r-badge'>--</span></div>"
            "<div class='row'><span class='label'>Elevator</span><span class='value' id='elevator-val'>--</span><span class='badge' id='elevator-badge'>--</span></div>"
            "<div class='row'><span class='label'>Rudder</span><span class='value' id='rudder-val'>--</span><span class='badge' id='rudder-badge'>--</span></div>"
            "<div class='row'><span class='label'>Закрылки</span><span class='value' id='flaps-val'>--</span></div>"
            "<div class='row'><span class='label'>ESC (мотор)</span><span class='value' id='esc-val'>--</span><span class='badge' id='esc-badge'>--</span></div>"
            "</div>"

            "<div class='card'><h2>Датчики</h2>"
            "<div class='row'><span class='label'>IMU</span><span class='value' id='imu-val'>--</span><span class='badge' id='imu-badge'>--</span></div>"
            "<div class='row'><span class='label'>Барометр</span><span class='value' id='baro-val'>--</span><span class='badge' id='baro-badge'>--</span></div>"
            "<div class='row'><span class='label'>Магнитометр</span><span class='value' id='mag-val'>--</span><span class='badge' id='mag-badge'>--</span></div>"
            "<div class='row'><span class='label'>GPS</span><span class='value' id='gps-val'>--</span><span class='badge' id='gps-badge'>--</span></div>"
            "</div>"

            "<div class='card'><h2>Автопилот</h2>"
            "<div class='row'><span class='label'>Режим</span><span class='value' id='mode'>--</span></div>"
            "<div class='row'><span class='label'>Desired roll/pitch</span><span class='value' id='desired-roll'>--</span><span class='value' id='desired-pitch'>--</span></div>"
            "<div class='row'><span class='label'>Target alt</span><span class='value' id='target-alt'>--</span></div>"
            "<div class='row'>"
            "<button class='button danger' onclick='setMode(0)'>Manual</button>"
            "<button class='button success' onclick='setMode(1)'>Stabilize</button>"
            "<button class='button success' onclick='setMode(2)'>Takeoff</button>"
            "<button class='button success' onclick='setMode(3)'>Alt Hold</button>"
            "</div></div>"

            "<div class='card'><h2>PID (roll / pitch)</h2>"
            "<div class='row'>Kp<input type='number' step='0.01' id='kpRoll'> Ki<input type='number' step='0.01' id='kiRoll'> Kd<input type='number' step='0.01' id='kdRoll'> (roll)</div>"
            "<div class='row'>Kp<input type='number' step='0.01' id='kpPitch'> Ki<input type='number' step='0.01' id='kiPitch'> Kd<input type='number' step='0.01' id='kdPitch'> (pitch)</div>"
            "<div class='row'><button class='button' onclick='applyPID()'>Применить</button></div>"
            "</div>"

            "</div><script>"
            "function setBadge(id,ok,onText,offText){var el=document.getElementById(id);if(!el)return;el.textContent=ok?onText:offText;el.className='badge '+(ok?'ok':'bad');}"
            "function setOutputRow(id,out){var v=document.getElementById(id+'-val');var b=document.getElementById(id+'-badge');if(v)v.textContent=out.us+' us';if(b){b.textContent=out.attached?'OK':'НЕ ПОДКЛЮЧЕН';b.className='badge '+(out.attached?'ok':'bad');}}"
            "function setSensorRow(id,sensor,fields){var b=document.getElementById(id+'-badge');var v=document.getElementById(id+'-val');if(!sensor.attached){if(b){b.textContent='НЕТ В СХЕМЕ';b.className='badge bad';}if(v)v.textContent='--';return;}if(!sensor.available){if(b){b.textContent='НЕ ОТВЕЧАЕТ';b.className='badge bad';}if(v)v.textContent='--';return;}if(b){b.textContent='OK';b.className='badge ok';}if(v){var parts=[];for(var i=0;i<fields.length;i++){parts.push(fields[i]+'='+sensor[fields[i]].toFixed(2));}v.textContent=parts.join(' ');}}"
            "async function updateDashboard(){"
            "try{"
            "const r=await fetch('/api/status');const s=await r.json();"
            "for(var i=0;i<s.rc.length;i++){var val=s.rc[i];var pct=Math.max(0,Math.min(100,(val-1000)/10));var bar=document.getElementById('ch'+i);if(bar)bar.style.width=pct+'%';var lab=document.getElementById('chv'+i);if(lab)lab.textContent=val;}"
            "setBadge('rx',!s.failsafe,'OK','LOST');"
            "setBadge('arm',s.armed,'ARMED','DISARMED');"
            "setOutputRow('ail-l',s.outputs.aileronLeft);"
            "setOutputRow('ail-r',s.outputs.aileronRight);"
            "setOutputRow('elevator',s.outputs.elevator);"
            "setOutputRow('rudder',s.outputs.rudder);"
            "document.getElementById('flaps-val').textContent=s.flapsUs+' us';"
            "setOutputRow('esc',s.outputs.esc);"
            "setSensorRow('imu',s.imu,['roll','pitch','yaw']);"
            "setSensorRow('baro',s.baro,['altitude','climb']);"
            "setSensorRow('mag',s.mag,['heading']);"
            "setSensorRow('gps',s.gps,['fix','numSV','lat','lon']);"
            "document.getElementById('mode').textContent=s.autopilot.attached?s.autopilot.modeName:'НЕТ АВТОПИЛОТА';"
            "document.getElementById('desired-roll').textContent=s.autopilot.attached?('roll='+s.autopilot.desiredRoll.toFixed(1)):'--';"
            "document.getElementById('desired-pitch').textContent=s.autopilot.attached?('pitch='+s.autopilot.desiredPitch.toFixed(1)):'--';"
            "document.getElementById('target-alt').textContent=s.autopilot.attached?s.autopilot.targetAlt.toFixed(1):'--';"
            "if(s.autopilot.attached){var pidIds=['kpRoll','kiRoll','kdRoll','kpPitch','kiPitch','kdPitch'];for(var p=0;p<pidIds.length;p++){var inp=document.getElementById(pidIds[p]);if(inp&&!inp.dataset.touched)inp.value=s.autopilot[pidIds[p]];}}"
            "}catch(e){console.error(e);}"
            "}"
            "async function setMode(m){try{await fetch('/api/setmode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})});}catch(e){console.error(e);}}"
            "async function applyPID(){function g(id){document.getElementById(id).dataset.touched='1';return parseFloat(document.getElementById(id).value)||0;}try{await fetch('/api/setpid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kpRoll:g('kpRoll'),kiRoll:g('kiRoll'),kdRoll:g('kdRoll'),kpPitch:g('kpPitch'),kiPitch:g('kiPitch'),kdPitch:g('kdPitch')})});}catch(e){console.error(e);}}"
            "setInterval(updateDashboard,200);"
            "updateDashboard();"
            "</script></body></html>";

        webServer.send(200, "text/html", html);
    }
};
