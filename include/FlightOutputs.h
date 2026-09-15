#pragma once

#include <ESP32Servo.h>          // для Servo и ESP32PWM
#include "Config.h"              // для Config::PIN_..., PWM_MIN и т.д.
#include "FlightOutputState.h"   // для FlightOutputState

// ============================================================
// FLIGHT OUTPUTS
//
// Единственный класс, который знает о Servo/PWM-железе. Если
// позже появится другой драйвер (PCA9685, другой MCU, симулятор)
// — меняется только этот файл.
//
// Важная оговорка: attach() сообщает только то, что ESP32Servo
// смог выделить таймер/канал и настроить пин. Он НЕ проверяет,
// подключён ли физический сервопривод — обратной связи по току
// или положению у нас нет. Поэтому "attached=false" означает
// реальную аппаратную проблему (занятый пин/таймер), а
// "attached=true" означает лишь "ESP32 готов слать PWM сюда".
// ============================================================

class FlightOutputs
{
public:

    bool begin()
    {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);

        // Все поверхности и ESC работают на 50 Hz.
        aileronLeft.setPeriodHertz(50);
        aileronRight.setPeriodHertz(50);
        elevator.setPeriodHertz(50);
        esc.setPeriodHertz(50);

        aileronLeftAttached = aileronLeft.attach(
            Config::PIN_AILERON_LEFT, Config::PWM_MIN, Config::PWM_MAX);

        aileronRightAttached = aileronRight.attach(
            Config::PIN_AILERON_RIGHT, Config::PWM_MIN, Config::PWM_MAX);

        elevatorAttached = elevator.attach(
            Config::PIN_ELEVATOR, Config::PWM_MIN, Config::PWM_MAX);

        escAttached = esc.attach(
            Config::PIN_ESC, Config::PWM_MIN, Config::PWM_MAX);

        printStatus();

        return
            aileronLeftAttached &&
            aileronRightAttached &&
            elevatorAttached &&
            escAttached;
    }

    bool isAileronLeftAttached() const  { return aileronLeftAttached; }
    bool isAileronRightAttached() const { return aileronRightAttached; }
    bool isElevatorAttached() const     { return elevatorAttached; }
    bool isEscAttached() const          { return escAttached; }

    void printStatus() const
    {
        Serial.print("Outputs: aileronL(GPIO");
        Serial.print(Config::PIN_AILERON_LEFT);
        Serial.print(")="); Serial.print(aileronLeftAttached ? "OK" : "FAIL");

        Serial.print(" aileronR(GPIO");
        Serial.print(Config::PIN_AILERON_RIGHT);
        Serial.print(")="); Serial.print(aileronRightAttached ? "OK" : "FAIL");

        Serial.print(" elevator(GPIO");
        Serial.print(Config::PIN_ELEVATOR);
        Serial.print(")="); Serial.print(elevatorAttached ? "OK" : "FAIL");

        Serial.print(" esc(GPIO");
        Serial.print(Config::PIN_ESC);
        Serial.print(")="); Serial.println(escAttached ? "OK" : "FAIL");
    }

    // --------------------------------------------------------
    // Применить рассчитанное состояние к физическим Servo.
    // --------------------------------------------------------

    void write(const FlightOutputState& state)
    {
        aileronLeft.writeMicroseconds(state.aileronLeft);
        aileronRight.writeMicroseconds(state.aileronRight);
        elevator.writeMicroseconds(state.elevator);
        esc.writeMicroseconds(state.throttle);

        lastState = state;
    }

    // Немедленно выставить безопасные значения (нейтраль/газ выключен).
    void setFailsafe()
    {
        FlightOutputState safe;

        safe.aileronLeft = Config::FAILSAFE_AILERON;
        safe.aileronRight = Config::FAILSAFE_AILERON;
        safe.elevator = Config::FAILSAFE_ELEVATOR;
        safe.throttle = Config::FAILSAFE_THROTTLE;

        write(safe);
    }

    const FlightOutputState& getLastState() const
    {
        return lastState;
    }


private:

    Servo aileronLeft;
    Servo aileronRight;
    Servo elevator;
    Servo esc;

    bool aileronLeftAttached = false;
    bool aileronRightAttached = false;
    bool elevatorAttached = false;
    bool escAttached = false;

    FlightOutputState lastState;
};