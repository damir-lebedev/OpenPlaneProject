#include "../include/include.h"

// ============================================================
// AEROS-001 FLIGHT CONTROLLER
// ESP32-C3 SuperMini
//
// Текущая архитектура:
//
//   Receiver
//       ↓
//   FlightController
//       ↓
//   ControlMixer
//       ↓
//   ThrottleManager
//       ↓
//   FlightOutputs
//
// В будущем сюда можно независимо добавить:
//
//   IMU
//   Gyroscope
//   Accelerometer
//   GPS
//   Autopilot
//   WaypointNavigator
//   Telemetry
//   GUI
//   FlightModes
//
// Весь код пока находится в одном файле намеренно.
// Классы уже изолированы так, чтобы позже их можно было
// безболезненно разнести по отдельным .h/.cpp.
// ============================================================




// ============================================================
// 11. FLIGHT CONTROLLER
//
// Это главный координатор.
//
// Очень важно:
//
// FlightController НЕ содержит реализацию iBUS.
// FlightController НЕ управляет Servo напрямую.
// FlightController НЕ считает mixer вручную.
// FlightController НЕ занимается debug.
//
// Он только координирует подсистемы:
//
// Receiver
//   ↓
// Failsafe
//   ↓
// Arming
//   ↓
// Mixer
//   ↓
// Throttle
//   ↓
// Outputs
//
// Именно этот класс в будущем станет точкой объединения
// ручного управления и автопилота.
// ============================================================

class FlightController
{
public:

    FlightController(
        IBusReceiver& receiver,
        ControlMixer& mixer,
        ThrottleManager& throttle,
        ArmingManager& arming,
        FlightOutputs& outputs
    )
        : receiver(receiver),
          mixer(mixer),
          throttle(throttle),
          arming(arming),
          outputs(outputs)
    {
    }


    // --------------------------------------------------------
    // Основная инициализация.
    // --------------------------------------------------------

    void begin()
    {
        outputs.setFailsafe();

        receiver.begin();
    }


    // --------------------------------------------------------
    // Главный цикл flight controller.
    // --------------------------------------------------------

    void update()
    {
        // ----------------------------------------------------
        // 1. Получаем новые iBUS кадры.
        // ----------------------------------------------------

        receiver.update();


        // ----------------------------------------------------
        // 2. Проверяем состояние радиоканала.
        // ----------------------------------------------------

        const bool receiverFailsafe =
            receiver.isSignalLost();


        // ----------------------------------------------------
        // 3. Получаем текущее состояние RC.
        // ----------------------------------------------------

        const RcChannelState& rc =
            receiver.getState();


        // ----------------------------------------------------
        // 4. Failsafe имеет абсолютный приоритет.
        // ----------------------------------------------------

        if (receiverFailsafe)
        {
            arming.update(
                Config::PWM_MIN,
                true
            );

            throttle.update(
                rc,
                true
            );

            outputs.setFailsafe();

            return;
        }


        // ----------------------------------------------------
        // 5. Обновляем ARM state.
        // ----------------------------------------------------

        arming.update(
            rc.get(Channels::THROTTLE),
            false
        );


        // ----------------------------------------------------
        // 6. Рассчитываем поверхности управления.
        // ----------------------------------------------------

        FlightOutputState output =
            mixer.calculate(rc);


        // ----------------------------------------------------
        // 7. Рассчитываем throttle.
        // ----------------------------------------------------

        output.throttle =
            throttle.update(
                rc,
                false
            );


        // ----------------------------------------------------
        // 8. Отправляем весь рассчитанный state
        //    физическим выходам.
        // ----------------------------------------------------

        outputs.write(output);
    }


    // --------------------------------------------------------
    // Состояние приёмника.
    // --------------------------------------------------------

    bool isReceiverFailsafe() const
    {
        return receiver.isSignalLost();
    }


    // --------------------------------------------------------
    // Состояние ARM.
    // --------------------------------------------------------

    bool isArmed() const
    {
        return arming.isArmed();
    }


    // --------------------------------------------------------
    // Состояние boost.
    // --------------------------------------------------------

    bool isBoostActive() const
    {
        return throttle.isBoostActive();
    }


    // --------------------------------------------------------
    // Последние PWM outputs.
    // --------------------------------------------------------

    const FlightOutputState& getOutputState() const
    {
        return outputs.getLastState();
    }


    // --------------------------------------------------------
    // Текущее RC состояние.
    // --------------------------------------------------------

    const RcChannelState& getRcState() const
    {
        return receiver.getState();
    }


private:

    IBusReceiver& receiver;

    ControlMixer& mixer;

    ThrottleManager& throttle;

    ArmingManager& arming;

    FlightOutputs& outputs;
};


// ============================================================
// 12. DEBUG LOGGER
//
// Debug полностью отделён от flight logic.
//
// В будущем этот класс можно заменить на:
//
// SerialLogger
// TelemetryLogger
// WiFiLogger
// WebSocketLogger
// SDLogger
//
// При этом FlightController менять не потребуется.
// ============================================================

class DebugLogger
{
public:

    explicit DebugLogger(
        FlightController& controller
    )
        : controller(controller)
    {
    }


    // --------------------------------------------------------
    // Периодический вывод состояния.
    // --------------------------------------------------------

    void update()
    {
        const uint32_t now = millis();

        if (
            now - lastDebugTime <
            Config::DEBUG_INTERVAL_MS
        )
        {
            return;
        }


        lastDebugTime = now;

        printState();
    }


private:

    FlightController& controller;

    uint32_t lastDebugTime = 0;


    // --------------------------------------------------------
    // Вывод полного текущего состояния.
    // --------------------------------------------------------

    void printState()
    {
        const RcChannelState& rc =
            controller.getRcState();

        const FlightOutputState& output =
            controller.getOutputState();


        // ----------------------------------------------------
        // RC channels.
        // ----------------------------------------------------

        Serial.print("IBUS: ");

        for (
            uint8_t i = 0;
            i < Config::IBUS_CHANNELS;
            ++i
        )
        {
            Serial.print("CH");
            Serial.print(i + 1);
            Serial.print("=");

            Serial.print(rc.get(i));

            Serial.print(" ");
        }


        // ----------------------------------------------------
        // Receiver status.
        // ----------------------------------------------------

        Serial.print("| RX=");

        Serial.print(
            controller.isReceiverFailsafe()
                ? "LOST"
                : "OK"
        );


        // ----------------------------------------------------
        // ARM status.
        // ----------------------------------------------------

        Serial.print(" | ARM=");

        Serial.print(
            controller.isArmed()
                ? "YES"
                : "NO"
        );


        // ----------------------------------------------------
        // Boost status.
        // ----------------------------------------------------

        Serial.print(" | BOOST=");

        Serial.print(
            controller.isBoostActive()
                ? "ON"
                : "OFF"
        );


        // ----------------------------------------------------
        // Calculated outputs.
        // ----------------------------------------------------

        Serial.print(" | OUT LAIL=");
        Serial.print(output.aileronLeft);

        Serial.print(" RAIL=");
        Serial.print(output.aileronRight);

        Serial.print(" ELE=");
        Serial.print(output.elevator);

        Serial.print(" ESC=");
        Serial.println(output.throttle);
    }
};


// ============================================================
// 13. SYSTEM OBJECTS
//
// Здесь создаётся конкретная конфигурация системы.
//
// В будущем именно этот участок будет похож на composition
// root приложения:
//   sensors
//   controllers
//   navigation
//   telemetry
//   GUI
//   etc.
// ============================================================

HardwareSerial IBusSerial(1);

IBusReceiver ibusReceiver(IBusSerial);

ControlMixer controlMixer;

ThrottleManager throttleManager;

ArmingManager armingManager;

FlightOutputs flightOutputs;

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs
);

DebugLogger debugLogger(
    flightController
);


// ============================================================
// 14. SETUP
//
// setup() только запускает систему.
//
// Здесь не должно быть flight logic.
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial debug.
    // --------------------------------------------------------

    Serial.begin(115200);

    delay(1000);


    // --------------------------------------------------------
    // Startup message.
    // --------------------------------------------------------

    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" ESP32-C3");
    Serial.println(" OOP ARCHITECTURE");
    Serial.println("=================================");
    Serial.println();


    // --------------------------------------------------------
    // Инициализация физических PWM outputs.
    // --------------------------------------------------------

    const bool outputsOK =
        flightOutputs.begin();


    Serial.print("Flight outputs: ");

    Serial.println(
        outputsOK
            ? "OK"
            : "FAILED"
    );


    // --------------------------------------------------------
    // Сразу после старта выставляем безопасные значения.
    // --------------------------------------------------------

    flightOutputs.setFailsafe();


    // --------------------------------------------------------
    // Инициализация flight controller.
    // --------------------------------------------------------

    flightController.begin();


    // --------------------------------------------------------
    // Информационный вывод.
    // --------------------------------------------------------

    Serial.println();
    Serial.println("iBUS input initialized.");
    Serial.println("115200 baud.");
    Serial.println("10 channels.");
    Serial.println("Throttle must be LOW.");
    Serial.println("Motor is DISARMED.");
    Serial.println();
}


// ============================================================
// 15. MAIN LOOP
//
// loop() намеренно максимально маленький.
//
// Это одна из главных целей новой архитектуры.
//
// В будущем сюда можно будет добавить:
//
//   sensors.update();
//   autopilot.update();
//   navigation.update();
//   telemetry.update();
//   gui.update();
//
// При этом отдельные системы останутся независимыми.
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Основной flight controller.
    // --------------------------------------------------------

    flightController.update();


    // --------------------------------------------------------
    // Отдельная debug-подсистема.
    // --------------------------------------------------------

    debugLogger.update();


    // --------------------------------------------------------
    // Небольшая пауза.
    // --------------------------------------------------------

    delay(2);
}