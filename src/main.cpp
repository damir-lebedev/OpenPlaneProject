#include "../include/include.h"
#include "../include/WebDebugServer.h"
#include "../include/FlightController.h"

// ============================================================
// AEROS-001 FLIGHT CONTROLLER — ESP32-C3
//
// RC TRANSMITTER -> RC RECEIVER -> UART (115200) -> IBusReceiver
//   -> FlightController -> ArmingManager / ControlMixer /
//      Autopilot (коррекции, если armed) / ThrottleManager
//   -> FlightOutputs -> Servo/ESC
//
// Failsafe (потеря сигнала >500мс) обрывает update() сразу после
// ARM/throttle в безопасное состояние — это абсолютный приоритет,
// выше автопилота и ручного управления. Подробнее про приоритеты
// см. FlightController.h.
//
// Автопилот (IMU + барометр) и переключение его режимов
// (FeatureManager, каналы Channels::FEATURE_SLOTS) — опциональная
// надстройка: при отсутствии датчиков или заданных каналов система
// продолжает работать как чистое ручное управление.
// ============================================================

// Плата — единственная точка входа в железо (I2C/SPI/UART/PWM).
// main.cpp и всё остальное дальше работают через IBoard, не зная,
// что за ним реально ESP32 (см. hal/IBoard.h).
Esp32Board board;

IBusReceiver ibusReceiver(board.rcUart());
ControlMixer controlMixer;
ThrottleManager throttleManager;
ArmingManager armingManager;
FlightOutputs flightOutputs(board);

// Датчики автопилота. Если физически не подключены/не отвечают —
// isAvailable() == false, а Autopilot просто не даёт коррекций
// (см. Autopilot.h). Реальный прототип по README пока без них.
// Какой конкретно чип скомпилирован — см. sensors/SensorSelection.h.
SelectedImu imuSensor(board.i2c(), 0x68);
SelectedBaro baroSensor(board.i2c(), 0x76);

#if SENSOR_MAG != SENSOR_MAG_NONE
SelectedMag magSensor(board.i2c());
#endif

#if SENSOR_GPS != SENSOR_GPS_NONE
SelectedGps gpsSensor(board.gpsUart());
#endif

Autopilot autopilot(
    &imuSensor, &baroSensor,
#if SENSOR_MAG != SENSOR_MAG_NONE
    &magSensor,
#else
    nullptr,
#endif
#if SENSOR_GPS != SENSOR_GPS_NONE
    &gpsSensor
#else
    nullptr
#endif
);
FeatureManager featureManager(&autopilot);

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs,
    &autopilot,
    &featureManager
);

WebDebugServer webDebugServer(
    &flightController,
    &autopilot,
    &featureManager
);

DebugLogger debugLogger(
    flightController,
    &autopilot,
    &featureManager
);


void setup()
{
    Serial.begin(115200);
    delay(1000);  // время на инициализацию USB CDC

    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" ESP32-C3 / OOP ARCHITECTURE");
    Serial.println("=================================");
    Serial.println();

    // Плата: PWM-таймеры + шины I2C/SPI. Один раз, до того как
    // их начнут использовать FlightOutputs и датчики.
    board.begin();

    // Servo/ESC. begin() сам печатает OK/FAIL по каждому каналу
    // (см. FlightOutputs::printStatus) — это всё, что можно
    // проверить программно без обратной связи от серво.
    flightOutputs.begin();
    flightOutputs.setFailsafe();

    // Датчики автопилота. begin() возвращает false и печатает
    // причину, если чип не отвечает на I2C — самолёт при этом
    // продолжает работать в ручном режиме.
    imuSensor.begin();
    baroSensor.begin();

    // Калибровка держит самолёт неподвижным ~1-3 сек, поэтому
    // выполняется только если датчик реально откликнулся.
    if (imuSensor.isAvailable())
    {
        imuSensor.calibrate();
    }

    if (baroSensor.isAvailable())
    {
        baroSensor.calibrateAltitude();
    }

#if SENSOR_MAG != SENSOR_MAG_NONE
    magSensor.begin();
    if (magSensor.isAvailable())
    {
        magSensor.calibrate();

        // Разовая установка начального курса по магнитометру вместо
        // произвольного 0 — дальше yaw ведёт только гироскоп (см.
        // оговорку про дрейф в ImuSensor::setYaw()).
        imuSensor.setYaw(magSensor.getMagData().headingDegrees);
    }
#endif

#if SENSOR_GPS != SENSOR_GPS_NONE
    gpsSensor.begin();
#endif

    autopilot.begin();
    featureManager.begin();

    flightController.begin();

    webDebugServer.begin(0);  // 0 = AP: подключиться к Wi-Fi "OpenPlane-Debug"

    Serial.println();
    Serial.println("Готово. iBUS 115200 baud, 10 каналов.");
    Serial.print("Каналы автопилота: ");
    for (uint8_t slot = 0; slot < FEATURE_SLOT_COUNT; ++slot)
    {
        Serial.print("CH");
        Serial.print(FeatureManager::slotChannelNumber(slot));
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("Для ARM: throttle (CH3) на минимуме 1.5 секунды.");
    Serial.println();
}


void loop()
{
    webDebugServer.update();

    flightController.update();

    debugLogger.update();

    delay(2);  // ~500Hz — стабильная частота цикла, приёмник копит кадры
}
