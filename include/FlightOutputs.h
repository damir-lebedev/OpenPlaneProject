#pragma once
// ============================================================
// 10. FLIGHT OUTPUTS
//
// Единственный класс, который знает о Servo.
//
// Это очень важная граница.
//
// Если позже вместо ESP32Servo появится:
// - другой PWM driver;
// - PCA9685;
// - другой MCU;
// - simulator;
//
// остальные классы менять не придётся.
// ============================================================

class FlightOutputs
{
public:

    // --------------------------------------------------------
    // Инициализация PWM.
    // --------------------------------------------------------

    bool begin()
    {
        // ----------------------------------------------------
        // Выделяем все четыре hardware timers ESP32Servo.
        // ----------------------------------------------------

        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);


        // ----------------------------------------------------
        // Все поверхности и ESC работают на 50 Hz.
        // ----------------------------------------------------

        aileronLeft.setPeriodHertz(50);
        aileronRight.setPeriodHertz(50);
        elevator.setPeriodHertz(50);
        esc.setPeriodHertz(50);


        // ----------------------------------------------------
        // Подключаем левый элерон.
        // ----------------------------------------------------

        const bool leftOK =
            aileronLeft.attach(
                Config::PIN_AILERON_LEFT,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Подключаем правый элерон.
        // ----------------------------------------------------

        const bool rightOK =
            aileronRight.attach(
                Config::PIN_AILERON_RIGHT,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Подключаем elevator.
        // ----------------------------------------------------

        const bool elevatorOK =
            elevator.attach(
                Config::PIN_ELEVATOR,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Подключаем ESC.
        // ----------------------------------------------------

        const bool escOK =
            esc.attach(
                Config::PIN_ESC,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        return
            leftOK &&
            rightOK &&
            elevatorOK &&
            escOK;
    }


    // --------------------------------------------------------
    // Применить рассчитанное состояние к физическим Servo.
    // --------------------------------------------------------

    void write(const FlightOutputState& state)
    {
        aileronLeft.writeMicroseconds(
            state.aileronLeft
        );

        aileronRight.writeMicroseconds(
            state.aileronRight
        );

        elevator.writeMicroseconds(
            state.elevator
        );

        esc.writeMicroseconds(
            state.throttle
        );


        // ----------------------------------------------------
        // Сохраняем последнее состояние для debug/telemetry.
        // ----------------------------------------------------

        lastState = state;
    }


    // --------------------------------------------------------
    // Немедленно выставить безопасные выходы.
    // --------------------------------------------------------

    void setFailsafe()
    {
        FlightOutputState safe;

        safe.aileronLeft =
            Config::FAILSAFE_AILERON;

        safe.aileronRight =
            Config::FAILSAFE_AILERON;

        safe.elevator =
            Config::FAILSAFE_ELEVATOR;

        safe.throttle =
            Config::FAILSAFE_THROTTLE;


        write(safe);
    }


    // --------------------------------------------------------
    // Получить последние записанные значения.
    // --------------------------------------------------------

    const FlightOutputState& getLastState() const
    {
        return lastState;
    }


private:

    Servo aileronLeft;
    Servo aileronRight;
    Servo elevator;
    Servo esc;

    FlightOutputState lastState;
};