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
    Esp32UartPort(HardwareSerial& port, int8_t rxPin, int8_t txPin = -1);

    void begin(uint32_t baud) override;

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
