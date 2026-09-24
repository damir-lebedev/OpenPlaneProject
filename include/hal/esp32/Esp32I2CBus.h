#pragma once
#include <Wire.h>

#include "hal/II2CBus.h"

// ============================================================
// Реализация II2CBus для ESP32 (Arduino core) — тонкая обёртка
// над аппаратным контроллером I2C (TwoWire: Wire или Wire1). Пины
// и частота передаются один раз в конструкторе и применяются только
// в begin() — единственном месте, где вызывается wire.begin(), даже
// если шину используют несколько устройств.
//
// Таймаут транзакции — TIMEOUT_MS вместо штатных 50 мс: чтение
// 14 байт IMU на 400 кГц занимает ~0.4 мс, а зависшая транзакция
// (помеха на проводах) иначе останавливала бы полётный цикл на 50 мс.
// ============================================================

class Esp32I2CBus : public II2CBus
{
public:

    Esp32I2CBus(TwoWire& wire, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)
        : wire(wire),
          sda(sdaPin),
          scl(sclPin),
          frequency(frequencyHz)
    {
    }

    void begin() override
    {
        wire.begin(sda, scl, frequency);
        wire.setTimeOut(TIMEOUT_MS);
    }

    void setClock(uint32_t hz) override { wire.setClock(hz); }

    void beginTransmission(uint8_t address) override { wire.beginTransmission(address); }
    size_t write(uint8_t data) override { return wire.write(data); }
    size_t write(const uint8_t* data, size_t length) override { return wire.write(data, length); }
    uint8_t endTransmission(bool sendStop = true) override { return wire.endTransmission(sendStop); }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) override
    {
        return wire.requestFrom(address, quantity);
    }

    int available() override { return wire.available(); }
    int read() override { return wire.read(); }


private:

    static constexpr uint16_t TIMEOUT_MS = 5;

    TwoWire& wire;
    int8_t sda;
    int8_t scl;
    uint32_t frequency;
};
