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