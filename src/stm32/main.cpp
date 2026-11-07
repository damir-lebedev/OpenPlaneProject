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
//                длится секунды, полётная задача его вытесняет;
//       bbox    (2) — чёрный ящик: снимки полётного цикла пишутся на
//                SD-карту (SDMMC1) блоками по 512 байт.
//
// На железе проверена голая плата DevEBox H743 (без датчиков и серв):
// загрузка, консоль по USB, SD-карта и чёрный ящик. Датчики, ШИМ на серво
// и iBUS на STM32 ещё ждут стенда. Первое включение с датчиками: консоль
// (env stm32h743-devebox — USB CDC; stm32h743 — LPUART1 115200, PA9/PA10),
// 'b' — опрос шин I2C, 's' — датчики, 'p' — импульсы на выходах (пропеллер
// снять!), 'D' — перезагрузка в загрузчик USB DFU.
//
// В сборке мультикоптера (VEHICLE_MULTIROTOR) этот файл выключен — точка
// входа там src/stm32/main_copter.cpp.
// ============================================================

#if !defined(VEHICLE_MULTIROTOR)

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
#include "hal/SdFileRegion.h"
#include "hal/stm32/Stm32Board.h"
#include "hal/stm32/Stm32FlashStorage.h"
#include "hal/stm32/Stm32SdCard.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "sensors/SensorSelection.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/BlackBox.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/AppLink.h"
#include "telemetry/DebugConsole.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LoopStats.h"
#include "telemetry/MavlinkTelemetry.h"
#include "telemetry/OledDisplay.h"
#include "telemetry/CameraStream.h"
#if SENSOR_CAMERA == SENSOR_CAMERA_OV5640
#include "hal/stm32/Stm32Camera.h"
#endif


// ------------------------------------------------------------
// Железо.
// ------------------------------------------------------------

Stm32Board board;

#if defined(ARDUINO_ARCH_STM32)
void stm32RebootToBootloader();   // src/stm32/bootloader.cpp
#endif


// ------------------------------------------------------------
// Датчики — как на ESP32 (sensors/SensorSelection.h).
// По типу может быть выбрано несколько чипов (основной и запасные):
// работает первый ответивший, искать его — на всех шинах I2C платы
// (sensors/SensorChain.h). Не ответил ни один — isAvailable() == false,
// автопилот не даёт коррекций.
// ------------------------------------------------------------

SelectedSensors<Stm32Board> sensors(board);
ImuChain& imuSensor = sensors.imu;
BaroChain& baroSensor = sensors.baro;
MagnetometerSensor* const magnetometer = sensors.magnetometer();

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

RcReceiver rcReceiver(board.rcUart());
ControlMixer controlMixer;
ThrottleManager throttleManager;
FlightOutputs flightOutputs(board);

Autopilot autopilot(&imuSensor, &baroSensor, magnetometer, gpsReceiver, airspeedSensor);
PilotSwitches pilotSwitches(&autopilot);
ArmingManager armingManager(&autopilot);

FlightController flightController(
    rcReceiver,
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

// Чёрный ящик: файл BLACKBOX.BIN в корне FAT32-карты, заранее созданный на
// ПК (tools/blackbox.py sd-prepare). Нет карты или файла — ящик выключен.
Stm32SdCard sdCard;
SdFileRegion blackBoxFile(sdCard, Config::BLACKBOX_SD_FILE, Config::BLACKBOX_SD_MAX_BYTES);
BlackBoxStorage blackBoxStorage(blackBoxFile);
BlackBox blackBox(flightController, autopilot, loopStats, blackBoxStorage, &pilotSwitches);

DebugLogger debugLogger(flightController, &autopilot, &loopStats);
DebugConsole debugConsole(flightController, flightOutputs, autopilot, debugLogger, &board, &blackBox);
AppLink appLink(flightController, &autopilot, &loopStats);   // живая связь с OpenPlane Studio
MavlinkTelemetry mavlink(*board.telemetryUart(), flightController, &autopilot, &loopStats);
OledDisplay oledDisplay(flightController, &autopilot, loopStats);

// ------------------------------------------------------------
// Камера OV5640 (SENSOR_CAMERA, разъём DCMI DevEBox): своя задача снимает
// кадр JPEG раз в CAMERA_BLACKBOX_PERIOD_MS, пока пишет чёрный ящик, и по
// просьбе приложения ("cam snap"); кадры — telemetry/CameraStream.h.
// Буфер кадра — в SRAM домена D2 (0x30000000, 288 КБ): AXI SRAM почти вся у
// очереди ящика, а DMA2 видит обе.
// ------------------------------------------------------------
CameraStream cameraStream;
#if SENSOR_CAMERA == SENSOR_CAMERA_OV5640
Stm32Camera cameraHw;
Ov5640 ov5640;
static uint8_t* const cameraBuffer = reinterpret_cast<uint8_t*>(0x30000000);
static_assert(Config::CAMERA_BUFFER_BYTES <= 288UL * 1024, "буфер кадра — в D2 SRAM, 288 КБ");

static void setupCamera()
{
    RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN | RCC_AHB2ENR_D2SRAM3EN;
    cameraHw.powerUp(Config::PIN_CAM_PWDN, Config::PIN_CAM_RESET);
    I2cSccb sccb(board.i2c());
    CameraSettings s;
    s.width = Config::CAMERA_WIDTH;
    s.height = Config::CAMERA_HEIGHT;
    s.quality = Config::CAMERA_QUALITY;
    Serial.print("Камера OV5640: ");
    if (!ov5640.begin(sccb, s))
    {
        cameraStream.state = ov5640.chipId() == 0 ? CameraStream::State::NORESP : CameraStream::State::ERROR;
        cameraStream.chip = ov5640.chipId();
        Serial.print(ov5640.error());
        if (ov5640.failedRegister())
        {
            Serial.print(" 0x");
            Serial.print(ov5640.failedRegister(), HEX);
        }
        Serial.println();
        return;
    }
    cameraStream.chip = ov5640.chipId();
    cameraStream.width = ov5640.frameWidth();
    cameraStream.height = ov5640.frameHeight();
    delay(300);   // автоэкспозиция успевает схватиться
    const uint32_t start = millis();
    const uint32_t n = cameraHw.snapshot(cameraBuffer, Config::CAMERA_BUFFER_BYTES, 1000);
    if (n == 0)
    {
        cameraStream.state = CameraStream::State::ERROR;
        Serial.print("SCCB ответил, но кадра нет (DCMI: принято ");
        Serial.print(static_cast<unsigned long>(cameraHw.lastRawBytes()));
        Serial.print(" байт, RIS 0x");
        Serial.print(cameraHw.lastRis(), HEX);
        Serial.println(")");
        return;
    }
    cameraStream.state = CameraStream::State::OK;
    cameraStream.lastBytes = n;
    Serial.print(cameraStream.width);
    Serial.print("x");
    Serial.print(cameraStream.height);
    Serial.print(" JPEG, кадр ");
    Serial.print(static_cast<unsigned long>(n));
    Serial.print(" байт за ");
    Serial.print(static_cast<unsigned long>(millis() - start));
    Serial.println(" мс");
}

// "cam diag": такты XCLK, PWDN/RESET, ответ SCCB при обеих полярностях PWDN.
static void cameraDiag()
{
    Serial.printf("HSI48 %s, MCO1SEL %lu MCO1PRE %lu, PA8 MODER %lu AF %lu\n", (RCC->CR & RCC_CR_HSI48RDY) ? "RDY" : "off",
                  static_cast<unsigned long>((RCC->CFGR & RCC_CFGR_MCO1) >> RCC_CFGR_MCO1_Pos),
                  static_cast<unsigned long>((RCC->CFGR & RCC_CFGR_MCO1PRE) >> RCC_CFGR_MCO1PRE_Pos),
                  static_cast<unsigned long>((GPIOA->MODER >> 16) & 3), static_cast<unsigned long>((GPIOA->AFR[1] >> 0) & 0xF));
    I2cSccb sccb(board.i2c());
    I2cSccb sccb2640(board.i2c(), 0x30);
    for (int pwdn = 0; pwdn < 2; ++pwdn)
    {
        digitalWrite(Config::PIN_CAM_PWDN, pwdn ? HIGH : LOW);
        digitalWrite(Config::PIN_CAM_RESET, LOW);
        delay(5);
        digitalWrite(Config::PIN_CAM_RESET, HIGH);
        delay(30);
        uint8_t a = 0, b = 0;
        const bool ok = sccb.read(0x300A, a) && sccb.read(0x300B, b);
        uint8_t c = 0;
        const bool ok2640 = sccb2640.write(0xFF, 0x01) && sccb2640.read(0x0A, c);
        Serial.printf("PWDN=%d: OV5640 0x3C %s %02X%02X; 0x30 %s %02X\n", pwdn, ok ? "ответ" : "молчит", a, b,
                      ok2640 ? "ответ" : "молчит", c);
    }
    digitalWrite(Config::PIN_CAM_PWDN, LOW);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr)
    {
        board.i2c().beginTransmission(addr);
        if (board.i2c().endTransmission(true) == 0)
        {
            Serial.printf("I2C датчиков: 0x%02X\n", addr);
            ++found;
        }
    }
    Serial.printf("на шине датчиков устройств: %d; камера: %s, кадров %lu, ошибок %lu, RIS 0x%lX, принято %lu\n", found,
                  cameraStream.stateName(), static_cast<unsigned long>(cameraStream.frames),
                  static_cast<unsigned long>(cameraStream.errors), static_cast<unsigned long>(cameraHw.lastRis()),
                  static_cast<unsigned long>(cameraHw.lastRawBytes()));
}

static void cameraTask(void*)
{
    uint16_t frame = 0;
    uint32_t lastRecordMs = 0;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (cameraStream.state != CameraStream::State::OK || cameraStream.busy()) continue;
        const bool forHost = cameraStream.snapshotWanted();
        const uint32_t now = millis();
        const bool forBox = Config::CAMERA_BLACKBOX_PERIOD_MS > 0 && blackBox.isRecording() &&
                            now - lastRecordMs >= Config::CAMERA_BLACKBOX_PERIOD_MS;
        if (!forHost && !forBox) continue;
        const uint32_t tUs = micros();
        const uint32_t n = cameraHw.snapshot(cameraBuffer, Config::CAMERA_BUFFER_BYTES, 500);
        if (n == 0)
        {
            if (forHost) cameraStream.snapshotFailed("кадр не пришёл");
            else cameraStream.errors++;
            continue;
        }
        ++frame;
        if (forBox)
        {
            lastRecordMs = now;
            if (blackBox.pushCameraFrame(cameraBuffer, n, frame, tUs, Config::CAMERA_BLACKBOX_RESERVE)) cameraStream.recordedFrames++;
        }
        cameraStream.frameReady(cameraBuffer, n, frame, cameraStream.width, cameraStream.height, forHost);
    }
}
#endif


static void printBanner()
{
    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" STM32H743 / FreeRTOS");
    Serial.println("=================================");
    Serial.println();
}

// SD-карта и файл чёрного ящика: что нашлось — в консоль, дальше BlackBox
// сам сверяет сектора и выключается, если места для записи нет.
static void setupBlackBox()
{
    Serial.print("SD-карта: ");
    if (!sdCard.begin())
    {
        Serial.print("нет ответа (код ");
        Serial.print(static_cast<unsigned long>(sdCard.initError()));
        Serial.println(") — карта не вставлена или не опознана");
    }
    else
    {
        Serial.print(static_cast<unsigned long>(sdCard.blockCount() / 2048));
        Serial.print(" МБ, SDMMC ");
        Serial.print(static_cast<unsigned long>(48 / (2 * sdCard.clockDivider())));   // ядро SDMMC — 48 МГц
        Serial.print(" МГц, 4 бита; файл ");
        Serial.print(Config::BLACKBOX_SD_FILE);
        Serial.print(": ");
        const Fat32::Result found = blackBoxFile.begin();
        Serial.println(Fat32::describe(found));
    }

    const uint32_t start = millis();
    blackBox.begin();   // сверка стёртого места, очередь, задача записи
    Serial.print("BlackBox: готов за ");
    Serial.print(static_cast<unsigned long>(millis() - start));
    Serial.println(" мс");
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

    if (magnetometer && magnetometer->begin())
    {
        delay(25);
        magnetometer->update();
        imuSensor.setYaw(magnetometer->getMagData().headingDegrees);
    }

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
        appLink.update(millis());

        const uint32_t workUs = micros() - start;
        loopStats.record(workUs);
        blackBox.update(workUs);   // снимок в очередь; на карту пишет задача bbox

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
#if SENSOR_CAMERA == SENSOR_CAMERA_OV5640
    setupCamera();   // до задач: SCCB — на шине датчиков, потом шина у полётной задачи
#endif

    flightController.begin();    // приёмник пульта
    mavlink.begin();             // радиомодем
    oledDisplay.begin(OledDisplay::findBus(board));
    setupBlackBox();             // последним: задача записи начинает готовить место

    Serial.println();
    Serial.print("Готово. Приёмник ");
    Serial.print(rcReceiver.getProtocolName());
    Serial.print(rcReceiver.isPortReady() ? ", " : " (UART не умеет формат этого протокола — связи не будет), ");
    Serial.print(Config::RC_CHANNELS);
    Serial.println(" каналов; MAVLink 57600 бод на UART4 (PD0/PD1).");
    Serial.println("ARM: SwA вниз, к себе (CH5=2000) при газе внизу. DISARM: SwA вверх.");
    pilotSwitches.printBindings();
    debugLogger.begin();
#if defined(ARDUINO_ARCH_STM32)
    debugConsole.setBootloaderHook(stm32RebootToBootloader);   // 'D' — перепрошивка без BOOT0
#endif
    debugConsole.setAppLink(&appLink);
    appLink.setCamera(&cameraStream);
#if SENSOR_CAMERA == SENSOR_CAMERA_OV5640
    appLink.setCameraDiag(cameraDiag);
#endif
    debugConsole.printHint();
    Serial.println();

    // Стек в байтах; полётной задаче — с запасом под printf лога.
    Rtos::startTask(flightTask, "flight", 16384, nullptr, Rtos::PRIORITY_FLIGHT);
    Rtos::startTask(storageTask, "storage", 2048, nullptr, Rtos::PRIORITY_BACKGROUND);
#if SENSOR_CAMERA == SENSOR_CAMERA_OV5640
    Rtos::startTask(cameraTask, "camera", 2048, nullptr, Rtos::PRIORITY_BACKGROUND);
#endif
    vTaskStartScheduler();       // не возвращается

    Serial.println("FreeRTOS не запустился (мало памяти?)");
}


// Не вызывается: после vTaskStartScheduler() работают только задачи.
// cppcheck-suppress unusedFunction
void loop()
{
}

#endif   // !VEHICLE_MULTIROTOR
