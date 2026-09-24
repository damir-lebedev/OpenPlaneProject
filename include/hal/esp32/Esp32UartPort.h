#pragma once
#include <Arduino.h>
#include "hal/IUartPort.h"

// ============================================================
// Реализация IUartPort для ESP32 (Arduino core) — обёртка над
// HardwareSerial. Пины и формат кадра (8N1) фиксируются в
// конструкторе; begin(baud) вызывающая сторона (IBusReceiver,
// GPS-драйвер) видит только скорость.
//
// txPin = -1 для приёмников на чистый приём (как раньше у iBUS).
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

    int available() override { return serial.available(); }
    int read() override { return serial.read(); }
    size_t write(uint8_t byte) override { return serial.write(byte); }
    size_t write(const uint8_t* buffer, size_t size) override { return serial.write(buffer, size); }


private:
    HardwareSerial& serial;
    int8_t rx;
    int8_t tx;
};
