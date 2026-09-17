#pragma once
#include <Arduino.h>

// ============================================================
// 🔌 АБСТРАКЦИЯ ОДНОГО PWM/SERVO-ВЫХОДА
//
// Заменяет прямое использование Servo/ESP32PWM в FlightOutputs.
// Пин фиксируется при создании конкретной реализации (см.
// Esp32ServoOutput) — так же, как у II2CBus/IUartPort.
//
// attach() отражает только то, что MCU смог выделить таймер/канал
// и настроить пин на выдачу PWM — не то, что физический серво
// подключён (обратной связи по факту нет ни в старой реализации
// на Servo, ни здесь).
// ============================================================

class IServoOutput
{
public:
    virtual ~IServoOutput() = default;

    virtual bool attach(uint16_t minUs, uint16_t maxUs) = 0;
    virtual void writeMicroseconds(uint16_t us) = 0;
    virtual bool isAttached() const = 0;
};
