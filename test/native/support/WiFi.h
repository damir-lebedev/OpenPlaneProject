#pragma once

// ============================================================
// Нативная замена WiFi (точка доступа) из Arduino core ESP32 2.0.x —
// только то, что использует прошивка: persistent(), mode(), softAP(),
// softAPIP(). Результат softAP() задаёт тест (fake::wifi()).
// ============================================================

#include <cstdint>
#include <string>

#include "Print.h"

class IPAddress : public Printable
{
public:
    IPAddress(uint8_t a = 0, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) : octets{ a, b, c, d } {}

    size_t printTo(Print& p) const override
    {
        size_t n = 0;
        for (uint8_t i = 0; i < 4; ++i)
        {
            if (i) n += p.print('.');
            n += p.print(octets[i], DEC);
        }
        return n;
    }

    uint8_t operator[](int index) const { return octets[index]; }

private:
    uint8_t octets[4];
};

typedef enum
{
    WIFI_OFF = 0,
    WIFI_STA = 1,
    WIFI_AP = 2,
    WIFI_AP_STA = 3
} wifi_mode_t;

namespace fake
{
    struct WifiState
    {
        bool softApResult = true;
        bool persistent = true;
        wifi_mode_t mode = WIFI_OFF;
        std::string ssid;
        std::string password;
        bool apStarted = false;
    };

    inline WifiState& wifi()
    {
        static WifiState state;
        return state;
    }
}

class WiFiClass
{
public:
    void persistent(bool enabled) { fake::wifi().persistent = enabled; }

    bool mode(wifi_mode_t mode)
    {
        fake::wifi().mode = mode;
        return true;
    }

    bool softAP(const char* ssid, const char* password = nullptr, int channel = 1, int hidden = 0,
                int maxConnections = 4, bool ftm = false)
    {
        (void)channel;
        (void)hidden;
        (void)maxConnections;
        (void)ftm;
        fake::WifiState& w = fake::wifi();
        w.ssid = ssid ? ssid : "";
        w.password = password ? password : "";
        w.apStarted = w.softApResult;
        return w.softApResult;
    }

    IPAddress softAPIP() const { return IPAddress(192, 168, 4, 1); }
};

inline WiFiClass WiFi;
