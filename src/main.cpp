// ============================================================
// AEROS-001 FLIGHT CONTROLLER — ESP32-S3
//
// Composition root: создаёт все объекты, связывает их и крутит
// полётный цикл. Логики полёта здесь нет — только сборка.
//
// RC-пульт -> приёмник -> iBUS (UART) -> IBusReceiver
//   -> FlightController: ArmingManager / ControlMixer (+ закрылки)
//      / Autopilot (коррекции и газ режима) / ThrottleManager
//   -> FlightOutputs -> сервы и ESC
//
// Потеря связи (нет кадров >500 мс или failsafe-значение газа от
// приёмника) — абсолютный приоритет: мотор в ноль, рули в нейтраль.
// ARM — тумблер SwA при газе внизу. Режим автопилота — SwC (CH7).
//
// Задачи FreeRTOS:
//   ядро 1 — loop(): полётный цикл с фиксированным периодом
//            Config::LOOP_PERIOD_MS (vTaskDelayUntil);
//   ядро 0 — Wi-Fi, веб-дашборд (WebDebugServer), OLED (OledDisplay).
// ============================================================

#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotModeSelector.h"
#include "config/Config.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "hal/esp32/Esp32Board.h"
#include "rc/IBusReceiver.h"
#include "sensors/SensorSelection.h"
#include "telemetry/DebugConsole.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LoopStats.h"
#include "telemetry/OledDisplay.h"
#include "telemetry/WebDebugServer.h"


// ------------------------------------------------------------
// Железо. Плата — единственная точка входа в MCU (hal/IBoard.h).
// ------------------------------------------------------------

Esp32Board board;


// ------------------------------------------------------------
// Датчики. Какой чип и на какой шине — sensors/SensorSelection.h.
// Не ответил — isAvailable() == false, автопилот не даёт коррекций.
// ------------------------------------------------------------

auto imuDevice = SELECTED_IMU_DEVICE(board);
SelectedImu imuSensor(imuDevice);

auto baroDevice = SELECTED_BARO_DEVICE(board);
SelectedBaro baroSensor(baroDevice);

#if SENSOR_MAG != SENSOR_MAG_NONE
auto magDevice = SELECTED_MAG_DEVICE(board);
SelectedMag magSensor(magDevice);
MagnetometerSensor* const magnetometer = &magSensor;
#else
MagnetometerSensor* const magnetometer = nullptr;
#endif

#if SENSOR_GPS != SENSOR_GPS_NONE
SelectedGps gpsSensor(board.gpsUart());
GpsSensor* const gps = &gpsSensor;
#else
GpsSensor* const gps = nullptr;
#endif


// ------------------------------------------------------------
// Управление полётом.
// ------------------------------------------------------------

IBusReceiver ibusReceiver(board.rcUart());
ControlMixer controlMixer;
ThrottleManager throttleManager;
FlightOutputs flightOutputs(board);

Autopilot autopilot(&imuSensor, &baroSensor, magnetometer, gps);
AutopilotModeSelector modeSelector(&autopilot);
ArmingManager armingManager(&autopilot);

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs,
    &autopilot,
    &modeSelector
);


// ------------------------------------------------------------
// Отладка и телеметрия.
// ------------------------------------------------------------

LoopStats loopStats;
DebugLogger debugLogger(flightController, &autopilot, &loopStats);
DebugConsole debugConsole(flightController, flightOutputs, autopilot, debugLogger);
WebDebugServer webDebugServer(flightController, &autopilot);
OledDisplay oledDisplay(flightController, &autopilot, loopStats);


static void printBanner()
{
    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" ESP32-S3 / OOP ARCHITECTURE");
    Serial.println("=================================");
    Serial.println();
}

// Датчики: опознать, откалибровать те, что ответили. Калибровка
// гироскопа держит самолёт неподвижным ~2 с и заодно проверяет
// установку IMU (предполётная проверка) — включать неподвижно; ровно —
// только пока установка не откалибрована командой 'o'.
static void setupSensors()
{
    if (imuSensor.begin())
    {
        imuSensor.calibrate();
    }

    if (baroSensor.begin())
    {
        baroSensor.calibrateAltitude();
    }

#if SENSOR_MAG != SENSOR_MAG_NONE
    // Калибровка компаса хранится в NVS (пересчитать — 'm' в консоли).
    // Первый отсчёт задаёт начальный курс вместо произвольного 0 —
    // дальше рысканье ведёт гироскоп.
    if (magSensor.begin())
    {
        delay(25);
        magSensor.update();
        imuSensor.setYaw(magSensor.getMagData().headingDegrees);
    }
#endif

#if SENSOR_GPS != SENSOR_GPS_NONE
    gpsSensor.begin();
#endif

    autopilot.begin();
}


void setup()
{
    // Строки лога и меню консоли на 115200 уходят десятки мс. Без
    // буфера Serial.print() ждёт аппаратный FIFO (128 байт) и тормозит
    // полётный цикл на это время.
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    delay(200);

    printBanner();

    board.begin();               // шины I2C/SPI
    flightOutputs.begin();       // PWM-выходы, сразу в безопасное положение
    flightOutputs.setFailsafe();

    setupSensors();

    flightController.begin();    // приёмник iBUS

    oledDisplay.begin(board.displayI2c());
    webDebugServer.begin();

    Serial.println();
    Serial.println("Готово. iBUS 115200 бод, 10 каналов.");
    Serial.println("ARM: SwA вниз, к себе (CH5=2000) при газе внизу. DISARM: SwA вверх.");
    Serial.println("Режим (SwC, CH7): вверх MANUAL, середина STABILIZE, вниз AUTO_TAKEOFF.");
    Serial.println("Закрылки: SwB (CH6) вниз.");
    debugLogger.begin();         // что выводить в лог — из NVS
    debugConsole.printHint();
    Serial.println();
}


void loop()
{
    static TickType_t lastWake = xTaskGetTickCount();

    const uint32_t start = micros();

    webDebugServer.applyPendingCommands();
    flightController.update();
    debugLogger.update();
    debugConsole.update();

    loopStats.record(micros() - start);

    // После долгой блокировки (калибровка из консоли) не "догоняем"
    // пропущенные такты пачкой — просто начинаем отсчёт заново.
    if (xTaskGetTickCount() - lastWake > pdMS_TO_TICKS(100))
    {
        lastWake = xTaskGetTickCount();
    }

    // Фиксированный период цикла, независимо от длительности работы выше.
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(Config::LOOP_PERIOD_MS));
}
