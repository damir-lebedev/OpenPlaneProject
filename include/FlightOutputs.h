#pragma once

#include "hal/IBoard.h"
#include "FlightOutputState.h"   // для FlightOutputState

// ============================================================
// FLIGHT OUTPUTS
//
// Единственный класс, который знает про порядок серво-каналов
// (aileronLeft/Right, elevator, esc). Само железо (Servo/PWM или
// что угодно другое) спрятано за IBoard/IServoOutput — этот файл
// его не видит, поэтому смена MCU/драйвера сюда не проникает.
//
// Важная оговорка: attach() сообщает только то, что плата смогла
// выделить таймер/канал и настроить пин. Он НЕ проверяет,
// подключён ли физический сервопривод — обратной связи по току
// или положению у нас нет. Поэтому "attached=false" означает
// реальную аппаратную проблему (занятый пин/таймер), а
// "attached=true" означает лишь "MCU готов слать PWM сюда".
// ============================================================

class FlightOutputs
{
public:

    explicit FlightOutputs(IBoard& board)
        : aileronLeft(board.servo(ServoChannel::AILERON_LEFT)),
          aileronRight(board.servo(ServoChannel::AILERON_RIGHT)),
          elevator(board.servo(ServoChannel::ELEVATOR)),
          esc(board.servo(ServoChannel::ESC))
    {
    }

    bool begin()
    {
        aileronLeftAttached = aileronLeft.attach(Config::PWM_MIN, Config::PWM_MAX);
        aileronRightAttached = aileronRight.attach(Config::PWM_MIN, Config::PWM_MAX);
        elevatorAttached = elevator.attach(Config::PWM_MIN, Config::PWM_MAX);
        escAttached = esc.attach(Config::PWM_MIN, Config::PWM_MAX);

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
    // Применить рассчитанное состояние к физическим выходам.
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

    IServoOutput& aileronLeft;
    IServoOutput& aileronRight;
    IServoOutput& elevator;
    IServoOutput& esc;

    bool aileronLeftAttached = false;
    bool aileronRightAttached = false;
    bool elevatorAttached = false;
    bool escAttached = false;

    FlightOutputState lastState;
};
