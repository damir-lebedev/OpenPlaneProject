#pragma once
#include <Arduino.h>

#include "hal/IUartPort.h"

// ============================================================
// Реализация IUartPort для ESP32 (Arduino core) — обёртка над
// HardwareSerial. Пины фиксируются в конструкторе; вызывающая
// сторона (RcReceiver, GPS-драйвер) задаёт только скорость и формат
// кадра. Инверсию для SBUS делает сам UART ESP32.
//
// txPin = -1 для приёмников на чистый приём.
// ============================================================

class Esp32UartPort : public IUartPort
{
public:
    Esp32UartPort(HardwareSerial& port, int8_t rxPin, int8_t txPin = -1)
        : serial(port), rx(rxPin), tx(txPin)
    {
    }

    void begin(uint32_t baud) override
    {
        serial.begin(baud, SERIAL_8N1, rx, tx);
    }

    bool begin(uint32_t baud, UartFormat format) override
    {
        if (format == UartFormat::SBUS_8E2_INVERTED) serial.begin(baud, SERIAL_8E2, rx, tx, true);
        else serial.begin(baud, SERIAL_8N1, rx, tx);
        return true;
    }

    int available() override { return serial.available(); }
    int read() override { return serial.read(); }
    size_t write(uint8_t value) override { return serial.write(value); }
    size_t write(const uint8_t* buffer, size_t size) override { return serial.write(buffer, size); }
    int availableForWrite() override { return serial.availableForWrite(); }


private:
    HardwareSerial& serial;
    int8_t rx;
    int8_t tx;
};
