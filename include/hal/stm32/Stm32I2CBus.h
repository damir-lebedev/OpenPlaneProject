#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "hal/II2CBus.h"

// ============================================================
// Реализация II2CBus для STM32 (STM32duino 3.x) — обёртка над
// TwoWire. ЗАГОТОВКА: на железе не проверялась.
//
// Как и у Esp32I2CBus, пины и частота задаются один раз в
// конструкторе и применяются только в begin(). Контроллер (I2C1,
// I2C2...) ядро выбирает само по паре пинов SDA/SCL.
//
// Таймаут транзакции у STM32duino — макрос I2C_TIMEOUT_TICK (мс,
// по умолчанию 100), а не метод объекта: задаётся флагом сборки в
// platformio.ini (env stm32h743), по той же причине, что
// TIMEOUT_MS у Esp32I2CBus, — зависшая транзакция не должна
// останавливать полётный цикл.
// ============================================================

class Stm32I2CBus : public II2CBus
{
public:

    Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)
        : wire(bus),
          sda(sdaPin),
          scl(sclPin),
          frequency(frequencyHz)
    {
    }

    void begin() override
    {
        // setSDA()/setSCL() действуют только до begin().
        wire.setSDA(sda);
        wire.setSCL(scl);
        wire.begin();
        wire.setClock(frequency);
    }

    void setClock(uint32_t hz) override { wire.setClock(hz); }

    void beginTransmission(uint8_t address) override { wire.beginTransmission(address); }
    size_t write(uint8_t data) override { return wire.write(data); }
    size_t write(const uint8_t* data, size_t length) override { return wire.write(data, length); }
    uint8_t endTransmission(bool sendStop = true) override { return wire.endTransmission(sendStop); }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) override
    {
        return static_cast<uint8_t>(wire.requestFrom(address, static_cast<size_t>(quantity)));
    }

    int available() override { return wire.available(); }
    int read() override { return wire.read(); }


private:

    TwoWire& wire;
    pin_size_t sda;
    pin_size_t scl;
    uint32_t frequency;
};
