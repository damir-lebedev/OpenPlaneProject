// Реализация telemetry/WebDebugServer.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "telemetry/WebDebugServer.h"


WebDebugServer::WebDebugServer(FlightController& flightController, Autopilot* ap)
: controller(flightController),
      autopilot(ap),
      webServer(Config::WEB_SERVER_PORT)
{
}

auto WebDebugServer::begin() -> bool
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

auto WebDebugServer::applyPendingCommands() -> void
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

auto WebDebugServer::serverTask(void* arg) -> void
{
    WebDebugServer* self = static_cast<WebDebugServer*>(arg);

    for (;;)
    {
        self->webServer.handleClient();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

auto WebDebugServer::buildStatusJson() const -> String
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
    appendAirspeed(json);
    appendAutopilot(json);

    json += '}';
    return json;
}

auto WebDebugServer::openSensor(String& json, const char* name, const Sensor* sensor) -> void
{
    json += ",\""; json += name; json += "\":{\"attached\":";
    json += sensor ? "true" : "false";
    json += ",\"available\":";
    json += (sensor && sensor->isAvailable()) ? "true" : "false";
}

auto WebDebugServer::field(String& json, const char* name, double value, unsigned decimals) -> void
{
    json += ",\""; json += name; json += "\":";
    json += String(value, decimals);
}

auto WebDebugServer::appendImu(String& json) const -> void
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

auto WebDebugServer::appendBaro(String& json) const -> void
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

auto WebDebugServer::appendMag(String& json) const -> void
{
    const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
    openSensor(json, "mag", mag);
    if (mag && mag->isAvailable())
    {
        field(json, "heading", mag->getMagData().headingDegrees, 1);
    }
    json += '}';
}

auto WebDebugServer::appendGps(String& json) const -> void
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

auto WebDebugServer::appendAirspeed(String& json) const -> void
{
    const AirspeedSensor* airspeed = autopilot ? autopilot->getAirspeedSensor() : nullptr;
    openSensor(json, "airspeed", airspeed);
    if (airspeed && airspeed->isAvailable())
    {
        const AirspeedData& d = airspeed->getAirspeedData();
        field(json, "ias", d.indicatedMs, 1);
        field(json, "tas", d.trueMs, 1);
        field(json, "dp", d.differentialPressurePa, 1);
    }
    json += '}';
}

auto WebDebugServer::appendAutopilot(String& json) const -> void
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

        const NavStatus& nav = autopilot->getNavStatus();
        json += ",\"nav\":{\"gps\":"; json += nav.gpsGood ? "true" : "false";
        json += ",\"home\":"; json += nav.homeValid ? "true" : "false";
        field(json, "homeDist", nav.distanceHomeM, 0);
        field(json, "homeBearing", nav.bearingHomeDeg, 0);
        field(json, "course", nav.courseDeg, 0);
        field(json, "targetCourse", nav.targetCourseDeg, 0);
        field(json, "speed", nav.speedMs, 1);
        json += ",\"fence\":"; json += nav.fenceBreached ? "true" : "false";
        json += ",\"stall\":"; json += nav.stallWarning ? "true" : "false";
        json += '}';

        // Включённые функции тумблеров — именами.
        const PilotInputs& in = autopilot->getInputs();
        json += ",\"features\":[";
        bool first = true;
        for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
        {
            if (!in.features[f]) continue;
            if (!first) json += ',';
            first = false;
            json += '"'; json += AutopilotNames::feature(static_cast<Feature>(f)); json += '"';
        }
        json += ']';
    }
    json += '}';
}

auto WebDebugServer::requireBodyAndAutopilot() -> bool
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

auto WebDebugServer::handleSetMode() -> void
{
    if (!requireBodyAndAutopilot()) return;

    const int mode = (int)extractJsonNumber(webServer.arg("plain"), "mode", -1);
    if (mode < MODE_MANUAL || mode >= MODE_COUNT)
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

auto WebDebugServer::handleSetPid() -> void
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

auto WebDebugServer::extractJsonNumber(const String& body, const char* key, float fallback) -> float
{
    const String needle = String("\"") + key + "\"";
    const int start = body.indexOf(needle);
    if (start < 0) return fallback;

    const int length = static_cast<int>(body.length());
    int from = start + static_cast<int>(needle.length());
    while (from < length && isJsonSpace(body[from])) from++;
    if (from >= length || body[from] != ':') return fallback;
    from++;
    while (from < length && isJsonSpace(body[from])) from++;

    int to = from;
    while (to < length && isJsonNumberChar(body[to]))
    {
        to++;
    }

    return to == from ? fallback : body.substring(from, to).toFloat();
}

auto WebDebugServer::isJsonSpace(char c) -> bool
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

auto WebDebugServer::isJsonNumberChar(char c) -> bool
{
    return isDigit(c) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
}
