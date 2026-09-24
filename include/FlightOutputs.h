#pragma once

#include "hal/IBoard.h"
#include "FlightOutputState.h"   // для FlightOutputState

// ============================================================
// FLIGHT OUTPUTS
//
// Единственный класс, который знает про порядок серво-каналов
// (aileronLeft/Right, elevator, rudder, esc). Само железо (LEDC/PWM или
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
          rudder(board.servo(ServoChannel::RUDDER)),
          esc(board.servo(ServoChannel::ESC))
    {
    }

    bool begin()
    {
        aileronLeftAttached = aileronLeft.attach(Config::PWM_MIN, Config::PWM_MAX);
        aileronRightAttached = aileronRight.attach(Config::PWM_MIN, Config::PWM_MAX);
        elevatorAttached = elevator.attach(Config::PWM_MIN, Config::PWM_MAX);
        rudderAttached = rudder.attach(Config::PWM_MIN, Config::PWM_MAX);
        escAttached = esc.attach(Config::PWM_MIN, Config::PWM_MAX);

        printStatus();

        // Руль направления необязателен (на ESP32-C3 под него нет пина).
        return
            aileronLeftAttached &&
            aileronRightAttached &&
            elevatorAttached &&
            escAttached;
    }

    bool isAileronLeftAttached() const  { return aileronLeftAttached; }
    bool isAileronRightAttached() const { return aileronRightAttached; }
    bool isElevatorAttached() const     { return elevatorAttached; }
    bool isRudderAttached() const       { return rudderAttached; }
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

        Serial.print(" rudder(");
        if (Config::PIN_RUDDER >= 0)
        {
            Serial.print("GPIO"); Serial.print(Config::PIN_RUDDER);
            Serial.print(")="); Serial.print(rudderAttached ? "OK" : "FAIL");
        }
        else
        {
            Serial.print("нет пина)");
        }

        Serial.print(" esc(GPIO");
        Serial.print(Config::PIN_ESC);
        Serial.print(")="); Serial.println(escAttached ? "OK" : "FAIL");
    }

    // Самопроверка выходов: на каждом GPIO измеряется реальный
    // импульс и сравнивается с тем, что туда пишет прошивка. Если
    // совпадает, а серво/ESC реагирует "не на тот" стик — значит,
    // провод воткнут не в тот пин.
    void printPulseSelfTest()
    {
        Serial.println("Выходы: GPIO -> измерено / ожидается (мкс)");
        printPulseLine("элерон L", Config::PIN_AILERON_LEFT, aileronLeft, lastState.aileronLeft);
        printPulseLine("элерон R", Config::PIN_AILERON_RIGHT, aileronRight, lastState.aileronRight);
        printPulseLine("руль выс", Config::PIN_ELEVATOR, elevator, lastState.elevator);
        if (Config::PIN_RUDDER >= 0)
        {
            printPulseLine("руль нап", Config::PIN_RUDDER, rudder, lastState.rudder);
        }
        printPulseLine("ESC     ", Config::PIN_ESC, esc, lastState.throttle);
    }

    // --------------------------------------------------------
    // Применить рассчитанное состояние к физическим выходам.
    // --------------------------------------------------------

    void write(const FlightOutputState& state)
    {
        aileronLeft.writeMicroseconds(state.aileronLeft);
        aileronRight.writeMicroseconds(state.aileronRight);
        elevator.writeMicroseconds(state.elevator);
        rudder.writeMicroseconds(state.rudder);
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
        safe.rudder = Config::FAILSAFE_RUDDER;
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
    IServoOutput& rudder;
    IServoOutput& esc;

    bool aileronLeftAttached = false;
    bool aileronRightAttached = false;
    bool elevatorAttached = false;
    bool rudderAttached = false;
    bool escAttached = false;

    FlightOutputState lastState;

    static void printPulseLine(const char* name, uint8_t pin, IServoOutput& output, uint16_t expected)
    {
        const int32_t measured = output.measurePulseUs();

        Serial.print("  "); Serial.print(name);
        Serial.print(" GPIO"); Serial.print(pin);
        Serial.print(": ");
        if (measured < 0) Serial.print("нет импульса"); else Serial.print(measured);
        Serial.print(" / "); Serial.print(expected);

        const bool ok = measured >= 0 && abs(measured - static_cast<int32_t>(expected)) <= 15;
        Serial.println(ok ? "  OK" : "  НЕ СОВПАДАЕТ");
    }
};
