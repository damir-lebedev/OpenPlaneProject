#pragma once
#include <Arduino.h>

// ============================================================
// 🔌 АБСТРАКЦИЯ UART-ПОРТА
//
// Мирроит форму HardwareSerial (available/read/write), но
// begin() берёт только скорость и формат кадра — пины фиксируются
// один раз при создании конкретной реализации (см. Esp32UartPort),
// а не передаются сюда. Так RcReceiver и GPS-драйвер остаются
// полностью платформо-независимыми: они вызывают port.begin(baud)
// и ничего не знают про конкретные GPIO или про то, что на ESP32
// это HardwareSerial.
// ============================================================

// Формат кадра. Почти всё — 8N1; SBUS — 8E2 и инвертированный сигнал
// (инверсию делает сам UART — у ESP32 и STM32H7 она встроенная).
enum class UartFormat : uint8_t
{
    STANDARD_8N1,
    SBUS_8E2_INVERTED,
};

class IUartPort
{
public:
    virtual ~IUartPort() = default;

    virtual void begin(uint32_t baud) = 0;   // 8N1

    // Другой формат кадра; false — этот порт так не умеет (и не открыт).
    virtual bool begin(uint32_t baud, UartFormat format)
    {
        if (format != UartFormat::STANDARD_8N1) return false;
        begin(baud);
        return true;
    }

    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t value) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) = 0;

    // Свободно в буфере передачи, байт; −1 — неизвестно (запись
    // может ждать). Телеметрия по нему решает, не отложить ли кадр.
    virtual int availableForWrite() { return -1; }
};
