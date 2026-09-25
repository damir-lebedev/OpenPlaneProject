#pragma once
#include <Arduino.h>

// ============================================================
// 🔌 АБСТРАКЦИЯ UART-ПОРТА
//
// Мирроит форму HardwareSerial (available/read/write), но
// begin() берёт только скорость — пины и формат кадра (8N1 и
// т.п.) фиксируются один раз при создании конкретной реализации
// (см. Esp32UartPort), а не передаются сюда. Так IBusReceiver и
// GPS-драйвер остаются полностью платформо-независимыми: они
// вызывают port.begin(baud) и ничего не знают про конкретные
// GPIO или про то, что на ESP32 это HardwareSerial.
// ============================================================

class IUartPort
{
public:
    virtual ~IUartPort() = default;

    virtual void begin(uint32_t baud) = 0;

    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t value) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) = 0;
};
