#include "../include/include.h"

// ============================================================
// AEROS-001 FLIGHT CONTROLLER — ESP32-S3
//
// RC TRANSMITTER -> RC RECEIVER -> UART (115200) -> IBusReceiver
//   -> FlightController -> ArmingManager / ControlMixer /
//      Autopilot (коррекции крена/тангажа, газ режима) / ThrottleManager
//   -> FlightOutputs -> Servo/ESC
//
// Failsafe (потеря связи: нет кадров >500мс или failsafe-значение
// газа от приёмника) обрывает update() сразу после чтения датчиков:
// мотор выключен, рули в нейтрали — это абсолютный приоритет, выше
// автопилота и ручного управления. Подробнее см. FlightController.h.
//
// ARM — тумблер SwA (CH5) при газе внизу, см. ArmingManager.h.
//
// Автопилот (IMU + барометр) и переключение его режимов (CH7, см.
// AutopilotModeSelector.h) — опциональная надстройка: при отсутствии
// датчиков система продолжает работать как чистое ручное управление.
//
// Задачи FreeRTOS:
//   ядро 1 — loop(): полётный цикл с фиксированным периодом
//            Config::LOOP_PERIOD_MS (vTaskDelayUntil);
//   ядро 0 — Wi-Fi, веб-дашборд (WebDebugServer), OLED (OledDisplay).
// ============================================================

// Плата — единственная точка входа в железо (I2C/SPI/UART/PWM).
// main.cpp и всё остальное дальше работают через IBoard, не зная,
// что за ним реально ESP32 (см. hal/IBoard.h).
Esp32Board board;

IBusReceiver ibusReceiver(board.rcUart());
ControlMixer controlMixer;
ThrottleManager throttleManager;
FlightOutputs flightOutputs(board);

// Датчики автопилота. Если физически не подключены/не отвечают —
// isAvailable() == false, а Autopilot просто не даёт коррекций
// (см. Autopilot.h). Какой конкретно чип и с какими аргументами
// скомпилирован — см. sensors/SensorSelection.h.
SelectedImu imuSensor(SELECTED_IMU_ARGS(board));
SelectedBaro baroSensor(SELECTED_BARO_ARGS(board));

#if SENSOR_MAG != SENSOR_MAG_NONE
SelectedMag magSensor(SELECTED_MAG_ARGS(board));
#endif

#if SENSOR_GPS != SENSOR_GPS_NONE
SelectedGps gpsSensor(SELECTED_GPS_ARGS(board));
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

// ArmingManager объявлен после Autopilot, чтобы получить на него
// указатель — нужен для предполётных проверок (см. ArmingManager.h).
ArmingManager armingManager(&autopilot);

AutopilotModeSelector modeSelector(&autopilot);

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs,
    &autopilot,
    &modeSelector
);

LoopStats loopStats;

WebDebugServer webDebugServer(
    &flightController,
    &autopilot
);

DebugLogger debugLogger(
    flightController,
    &autopilot,
    &loopStats
);

OledDisplay oledDisplay(
    flightController,
    &autopilot,
    loopStats
);


// ------------------------------------------------------------
// Консоль отладки по Serial: одна буква + Enter.
// Калибровки блокируют цикл на секунды, поэтому разрешены только
// когда мотор не заармлен.
// ------------------------------------------------------------

static void printConsoleHelp()
{
    Serial.println("Команды: h — помощь, s — статус датчиков, "
                   "i — калибровка IMU (2 с, не двигать), "
                   "m — калибровка компаса (15 с, вращать по всем осям), "
                   "p — проверка импульсов на выходах серво/ESC");
}

static void handleConsole()
{
    if (!Serial.available()) return;

    const char command = static_cast<char>(Serial.read());

    if (command == '\r' || command == '\n' || command == ' ') return;

    if ((command == 'i' || command == 'm' || command == 'p') && flightController.isArmed())
    {
        Serial.println("Консоль: калибровка недоступна, пока заармлено");
        return;
    }

    switch (command)
    {
        case 's':
            imuSensor.printStatus();
            baroSensor.printStatus();
#if SENSOR_MAG != SENSOR_MAG_NONE
            magSensor.printStatus();
#endif
#if SENSOR_GPS != SENSOR_GPS_NONE
            gpsSensor.printStatus();
#endif
            break;

        case 'p':
            flightOutputs.printPulseSelfTest();
            break;

        case 'i':
            imuSensor.calibrate();
            break;

        case 'm':
#if SENSOR_MAG != SENSOR_MAG_NONE
            magSensor.calibrate();
#else
            Serial.println("Консоль: магнитометр не выбран в SensorSelection.h");
#endif
            break;

        default:
            printConsoleHelp();
            break;
    }
}


void setup()
{
    // Отладочный кадр (~600 символов) при 115200 бод уходит ~50 мс.
    // Без буфера Serial.print() ждёт, пока всё уйдёт в аппаратный
    // FIFO (128 байт), и тормозит полётный цикл на это время.
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" ESP32-S3 / OOP ARCHITECTURE");
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
    // причину, если чип не отвечает — самолёт при этом
    // продолжает работать в ручном режиме.
    imuSensor.begin();
    baroSensor.begin();

    // Калибровка держит самолёт неподвижным ~2 сек, поэтому
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
    // Калибровка компаса (15 с вращения) больше не запускается при
    // каждом включении: смещения хранятся в NVS, пересчитать —
    // командой 'm' в консоли.
    magSensor.begin();
    if (magSensor.isAvailable())
    {
        // Первый отсчёт, затем разовая установка начального курса по
        // магнитометру вместо произвольного 0 — дальше yaw ведёт только
        // гироскоп (см. оговорку про дрейф в ImuSensor::setYaw()).
        delay(25);
        magSensor.update();
        imuSensor.setYaw(magSensor.getMagData().headingDegrees);
    }
#endif

#if SENSOR_GPS != SENSOR_GPS_NONE
    gpsSensor.begin();
#endif

    autopilot.begin();

    flightController.begin();

    oledDisplay.begin();

    webDebugServer.begin(0);  // 0 = AP: подключиться к Wi-Fi "OpenPlane-Debug"

    Serial.println();
    Serial.println("Готово. iBUS 115200 baud, 10 каналов.");
    Serial.println("ARM: SwA (CH5) вниз, к себе (CH5=2000) при газе внизу. DISARM: SwA вверх.");
    Serial.println("CH7 (SwC): <1250=MANUAL, 1250-1749=STABILIZE, >=1750=AUTO_TAKEOFF");
    printConsoleHelp();
    Serial.println();
}


void loop()
{
    static TickType_t lastWake = xTaskGetTickCount();

    const uint32_t start = micros();

    webDebugServer.applyPendingCommands();

    flightController.update();

    debugLogger.update();

    handleConsole();

    loopStats.record(micros() - start);

    // После долгой блокировки (калибровка из консоли) не "догоняем"
    // пропущенные такты пачкой — просто начинаем отсчёт заново.
    if (xTaskGetTickCount() - lastWake > pdMS_TO_TICKS(100))
    {
        lastWake = xTaskGetTickCount();
    }

    // Фиксированный период цикла, независимо от того, сколько заняла
    // работа выше (раньше было delay(2) после работы — период плавал).
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(Config::LOOP_PERIOD_MS));
}
