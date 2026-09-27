// ============================================================
// AEROS-001 FLIGHT CONTROLLER — STM32H743VIT6
//
// Та же прошивка, что src/main.cpp для ESP32-S3: те же датчики,
// автопилот, тумблеры (config/Controls.h), консоль и экран. Отличия
// платформы спрятаны ниже уровня этого файла:
//   - плата — Stm32Board (hal/stm32/): шины, UART, таймеры PWM;
//   - калибровки и настройки — во флеше, не в NVS: заголовок
//     <Preferences.h> подменяет hal/stm32/compat/Preferences.h
//     (KeyValueStore в последнем секторе флеша);
//   - вместо Wi-Fi-дашборда — телеметрия MAVLink по радиомодему на
//     UART4 (QGroundControl / Mission Planner);
//   - задачи FreeRTOS (библиотека STM32duino FreeRTOS) на одном ядре
//     с вытеснением по приоритету (hal/Rtos.h):
//       flight  (приоритет 5) — полётный цикл каждые LOOP_PERIOD_MS:
//                FlightController, MAVLink, лог, консоль;
//       oled    (1) — экран раз в 200 мс (OledDisplay);
//       storage (1) — запись настроек во флеш: стирание сектора
//                длится секунды, полётная задача его вытесняет.
//
// На железе пока не проверялась (платы ещё нет). Первое включение:
// консоль 115200 (LPUART1, PA9/PA10), 'b' — опрос шин I2C, 's' —
// датчики, 'p' — импульсы на выходах (пропеллер снять!).
// ============================================================

#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "autopilot/PilotSwitches.h"
#include "config/Config.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "hal/Rtos.h"
#include "hal/stm32/Stm32Board.h"
#include "hal/stm32/Stm32FlashStorage.h"
#include "rc/IBusReceiver.h"
#include "sensors/SensorInterface.h"
#include "sensors/SensorSelection.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/DebugConsole.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LoopStats.h"
#include "telemetry/MavlinkTelemetry.h"
#include "telemetry/OledDisplay.h"


// ------------------------------------------------------------
// Железо.
// ------------------------------------------------------------

Stm32Board board;


// ------------------------------------------------------------
// Датчики — как на ESP32 (sensors/SensorSelection.h).
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
GpsSensor* const gpsReceiver = &gpsSensor;
#else
GpsSensor* const gpsReceiver = nullptr;
#endif

#if SENSOR_AIRSPEED != SENSOR_AIRSPEED_NONE
auto pitotDevice = SELECTED_PITOT_DEVICE(board);
SelectedPitotBaro pitotBaro(pitotDevice, "PITOT-BMP581");
PitotDualBaroAirspeed pitotSensor(pitotBaro, baroSensor);
AirspeedSensor* const airspeedSensor = &pitotSensor;
#else
AirspeedSensor* const airspeedSensor = nullptr;
#endif


// ------------------------------------------------------------
// Управление полётом.
// ------------------------------------------------------------

IBusReceiver ibusReceiver(board.rcUart());
ControlMixer controlMixer;
ThrottleManager throttleManager;
FlightOutputs flightOutputs(board);

Autopilot autopilot(&imuSensor, &baroSensor, magnetometer, gpsReceiver, airspeedSensor);
PilotSwitches pilotSwitches(&autopilot);
ArmingManager armingManager(&autopilot);

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs,
    &autopilot,
    &pilotSwitches
);


// ------------------------------------------------------------
// Отладка и телеметрия.
// ------------------------------------------------------------

LoopStats loopStats;
DebugLogger debugLogger(flightController, &autopilot, &loopStats);
DebugConsole debugConsole(flightController, flightOutputs, autopilot, debugLogger, &board);
MavlinkTelemetry mavlink(*board.telemetryUart(), flightController, &autopilot, &loopStats);
OledDisplay oledDisplay(flightController, &autopilot, loopStats);


static void printBanner()
{
    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" STM32H743 / FreeRTOS");
    Serial.println("=================================");
    Serial.println();
}

// Как на ESP32: опознать датчики и откалибровать ответившие (самолёт
// неподвижен ~2 с).
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

#if SENSOR_AIRSPEED != SENSOR_AIRSPEED_NONE
    pitotSensor.begin();
#endif

    autopilot.begin();
}


// ------------------------------------------------------------
// Задачи.
// ------------------------------------------------------------

static void flightTask(void*)
{
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        const uint32_t start = micros();

        flightController.update();
        mavlink.update();
        debugLogger.update();
        debugConsole.update();

        loopStats.record(micros() - start);

        // После долгой блокировки (калибровка из консоли) не догоняем
        // пропущенные такты пачкой.
        if (xTaskGetTickCount() - lastWake > pdMS_TO_TICKS(100))
        {
            lastWake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(Config::LOOP_PERIOD_MS));
    }
}

static void storageTask(void*)
{
    for (;;)
    {
        Stm32FlashStorage::instance().service();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// Вызывается ядром Arduino, а не из кода проекта.
// cppcheck-suppress unusedFunction
void setup()
{
    Serial.begin(115200);
    delay(200);

    printBanner();

    board.begin();               // шины I2C/SPI
    flightOutputs.begin();       // PWM-выходы, сразу в безопасное положение
    flightOutputs.setFailsafe();

    // Калибровки и настройки: образ из последнего сектора флеша.
    KeyValueStore& settings = Stm32FlashStorage::store();
    settings.mount();
    Serial.print("Настройки во флеше: ");
    if (settings.wasCorrupt())
    {
        Serial.println("образ повреждён — значения по умолчанию, калибровки повторить");
    }
    else if (settings.bytesUsed() <= KeyValueStore::HEADER_SIZE)
    {
        Serial.println("пусто — значения по умолчанию");
    }
    else
    {
        Serial.print(static_cast<unsigned>(settings.bytesUsed()));
        Serial.println(" байт");
    }

    setupSensors();

    flightController.begin();    // приёмник iBUS
    mavlink.begin();             // радиомодем
    oledDisplay.begin(board.displayI2c());

    Serial.println();
    Serial.println("Готово. iBUS 115200 бод, 10 каналов; MAVLink 57600 бод на UART4 (PD0/PD1).");
    Serial.println("ARM: SwA вниз, к себе (CH5=2000) при газе внизу. DISARM: SwA вверх.");
    pilotSwitches.printBindings();
    debugLogger.begin();
    debugConsole.printHint();
    Serial.println();

    // Стек в байтах; полётной задаче — с запасом под printf лога.
    Rtos::startTask(flightTask, "flight", 16384, nullptr, Rtos::PRIORITY_FLIGHT);
    Rtos::startTask(storageTask, "storage", 2048, nullptr, Rtos::PRIORITY_BACKGROUND);
    vTaskStartScheduler();       // не возвращается

    Serial.println("FreeRTOS не запустился (мало памяти?)");
}


// Не вызывается: после vTaskStartScheduler() работают только задачи.
// cppcheck-suppress unusedFunction
void loop()
{
}
