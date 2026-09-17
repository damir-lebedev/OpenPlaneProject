#pragma once
#include <Wire.h>
#include "../II2CBus.h"

// ============================================================
// Реализация II2CBus для ESP32 (Arduino core) — тонкая обёртка
// над глобальным Wire. Пины передаются один раз в конструкторе
// (обычно Config::PIN_I2C_SDA/SCL) и используются только в
// begin() — единственном месте, где Wire.begin() вызывается
// (раньше это делал каждый датчик сам, что при двух активных
// I2C-датчиках приводило к двойному Wire.begin()).
// ============================================================

class Esp32I2CBus : public II2CBus
{
public:
    Esp32I2CBus(uint8_t sdaPin, uint8_t sclPin)
        : sda(sdaPin), scl(sclPin)
    {
    }

    void begin() override
    {
        Wire.begin(sda, scl);
        Wire.setClock(400000);
    }

    void setClock(uint32_t hz) override { Wire.setClock(hz); }

    void beginTransmission(uint8_t address) override { Wire.beginTransmission(address); }
    size_t write(uint8_t data) override { return Wire.write(data); }
    uint8_t endTransmission(bool sendStop = true) override { return Wire.endTransmission(sendStop); }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) override
    {
        return Wire.requestFrom(address, quantity);
    }

    int available() override { return Wire.available(); }
    int read() override { return Wire.read(); }


private:
    uint8_t sda;
    uint8_t scl;
};
