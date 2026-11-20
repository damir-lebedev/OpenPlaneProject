#pragma once
#include <Arduino.h>
#include <Update.h>
#include <WebServer.h>

#include "telemetry/FirmwareIdentity.h"
#include "telemetry/OtaGate.h"

// ============================================================
// ПРОШИВКА ПО WIFI (ESP32)
//
// Маршруты на сервере дашборда (WebDebugServer, та же точка доступа
// Config::WIFI_AP_SSID, http://192.168.4.1):
//   GET  /ota     — паспорт и окно: {"board","commit","config","open",
//                   "left_ms","receiving","armed"}
//   POST /update  — образ прошивки (firmware.bin сборки PlatformIO)
//                   формой multipart/form-data, поле "firmware".
//                   Ответ — текст: "OK" (через секунду перезагрузка) или
//                   код отказа: DISABLED / CLOSED / ARMED / THROTTLE /
//                   BUSY (OtaGate) или IMAGE <ошибка Update>.
// Можно ли принимать, решает OtaGate (окно, ARM, газ). Образ пишется во
// вторую область прошивки (Update.h сам берёт неактивную), проверяется
// (заголовок, чип, SHA-256 образа) и только потом становится загрузочным;
// оборванный приём старую прошивку не трогает.
// Обработчики идут в веб-задаче (ядро 0); перезагрузку делает
// полётный цикл (update()), когда ответ уже ушёл.
// Отправить с ПК: curl -F firmware=@.pio/build/esp32-s3/firmware.bin http://192.168.4.1/update
// ============================================================

class WifiOta
{
public:

    explicit WifiOta(OtaGate& otaGate) : gate(otaGate) {}

    // До WebDebugServer::begin(): маршруты на его сервер.
    void attach(WebServer& webServer)
    {
        server = &webServer;
        webServer.on("/ota", HTTP_GET, [this]() { sendStatus(); });
        webServer.on("/update", HTTP_POST, [this]() { finish(); }, [this]() { receive(); });
    }

    // Полётный цикл: перезагрузка после удачной прошивки (не в воздухе).
    void update(bool armed, uint32_t nowMs)
    {
        if (rebootAtMs != 0 && !armed && static_cast<int32_t>(nowMs - rebootAtMs) >= 0)
        {
            Serial.println("WiFi-прошивка: перезагрузка в новую прошивку");
            Serial.flush();
            ESP.restart();
        }
    }


private:

    void sendStatus()
    {
        const uint32_t nowMs = millis();
        String json = "{\"board\":\"" OP_BOARD_ENV "\",\"commit\":\"" OP_SOURCE_COMMIT "\",\"config\":\"" OP_CONFIG_HASH "\"";
        json += ",\"open\":";
        json += gate.isOpen() ? "true" : "false";
        json += ",\"left_ms\":";
        json += String(gate.remainingMs(nowMs));
        json += ",\"receiving\":";
        json += gate.isReceiving() ? "true" : "false";
        json += ",\"armed\":";
        json += gate.isArmed() ? "true" : "false";
        json += "}";
        server->send(200, "application/json", json);
    }

    void receive()
    {
        HTTPUpload& up = server->upload();
        switch (up.status)
        {
            case UPLOAD_FILE_START:
            {
                result = gate.beginUpload();
                imageError = "";
                started = result == OtaGate::Result::Ok;
                if (!started)
                {
                    Serial.printf("WiFi-прошивка: отказ — %s\n", OtaGate::text(result));
                    return;
                }
                Serial.printf("WiFi-прошивка: приём %s\n", up.filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) fail();
                return;
            }
            case UPLOAD_FILE_WRITE:
                if (!started || imageError.length()) return;
                if (gate.shouldAbort())
                {
                    imageError = "ARMED";
                    Update.abort();
                    return;
                }
                if (Update.write(up.buf, up.currentSize) != up.currentSize) fail();
                return;
            case UPLOAD_FILE_END:
                if (!started || imageError.length()) return;
                if (!Update.end(true)) fail();
                else Serial.printf("WiFi-прошивка: принято %u байт, образ проверен\n", (unsigned)up.totalSize);
                return;
            case UPLOAD_FILE_ABORTED:
                // Клиент пропал: обработчик маршрута (finish) уже не позовут —
                // отпустить приём здесь, иначе окно навсегда «занято» (BUSY).
                if (!started) return;
                if (!imageError.length()) Update.abort();
                Serial.println("WiFi-прошивка: связь оборвалась, старая прошивка цела");
                started = false;
                gate.finishUpload(false);
                result = OtaGate::Result::Closed;
                return;
            default:
                return;
        }
    }

    void fail()
    {
        imageError = String("IMAGE ") + Update.errorString();
        Serial.printf("WiFi-прошивка: %s\n", imageError.c_str());
        Update.abort();
    }

    void finish()
    {
        if (!started)
        {
            // Поля "firmware" не было — приём даже не начинался.
            const OtaGate::Result r = result == OtaGate::Result::Ok ? OtaGate::Result::Closed : result;
            server->send(r == OtaGate::Result::Busy ? 409 : 403, "text/plain", OtaGate::code(r));
            result = OtaGate::Result::Closed;
            return;
        }
        started = false;
        const bool ok = imageError.length() == 0 && Update.isFinished();
        gate.finishUpload(ok);
        result = OtaGate::Result::Closed;
        if (!ok)
        {
            server->send(imageError == "ARMED" ? 403 : 400, "text/plain", imageError.length() ? imageError : String("IMAGE"));
            return;
        }
        server->send(200, "text/plain", "OK");
        rebootAtMs = millis() + 1000;
        if (rebootAtMs == 0) rebootAtMs = 1;
    }

    OtaGate& gate;
    WebServer* server = nullptr;
    OtaGate::Result result = OtaGate::Result::Closed;
    bool started = false;
    String imageError;
    volatile uint32_t rebootAtMs = 0;
};
