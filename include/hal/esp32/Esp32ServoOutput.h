#pragma once
#include <ESP32Servo.h>
#include "../IServoOutput.h"

// ============================================================
// Реализация IServoOutput для ESP32 (ESP32Servo) — обёртка над
// Servo. Пин фиксируется в конструкторе; ESP32PWM::allocateTimer()
// вызывается один раз в Esp32Board::begin(), не здесь.
// ============================================================

class Esp32ServoOutput : public IServoOutput
{
public:
    explicit Esp32ServoOutput(uint8_t servoPin)
        : pin(servoPin)
    {
    }

    bool attach(uint16_t minUs, uint16_t maxUs) override
    {
        servo.setPeriodHertz(50);
        attached = servo.attach(pin, minUs, maxUs);
        return attached;
    }

    void writeMicroseconds(uint16_t us) override { servo.writeMicroseconds(us); }
    bool isAttached() const override { return attached; }


private:
    Servo servo;
    uint8_t pin;
    bool attached = false;
};
