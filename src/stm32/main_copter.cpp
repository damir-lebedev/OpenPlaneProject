// ============================================================
// МУЛЬТИКОПТЕР — STM32H743 (сборка VEHICLE_MULTIROTOR:
// env matek-h743-quad, stm32h743-devebox-quad)
//
// Те же датчики, приёмник и плата, что у самолёта (src/stm32/main.cpp
// в такой сборке выключен), а управление — copter/CopterController.h:
// контур угловой скорости, режимы ACRO/HORIZON/ANGLE/ALT_HOLD,
// failsafe с посадкой, защита от переворота. Настройка (PID, фильтры,
// холостые, расходы) выводится из описания коптера — config/CopterConfig.h;
// поверх неё — настройки из CLI (copter/Params.h, во флеше): и само
// описание, и пульт — карта каналов, режимы на тумблерах (copter/RcSetup.h).
//
// Задачи FreeRTOS:
//   flight  (приоритет 5) — цикл стабилизации каждые 1 мс: IMU,
//           барометр (раз в 20 мс), батарея, пульт, контроллер, моторы,
//           консоль; время каждой части — `tasks` в CLI;
//   storage (1) — запись настроек (калибровка установки) во флеш.
//
// Проверено пока только в симуляторе (test/native/test_copter_sim) и
// сборкой; на железе — нет. Первый запуск: БЕЗ ПРОПЕЛЛЕРОВ, консоль 't'
// (что вывелось из описания), 'm' (порядок и направление моторов),
// 's' (почему не армится).
// ============================================================

#if defined(VEHICLE_MULTIROTOR)

#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "config/CopterConfig.h"
#include "copter/Battery.h"
#include "copter/CopterController.h"
#include "copter/Params.h"
#include "hal/Rtos.h"
#include "hal/stm32/Stm32Board.h"
#include "hal/stm32/Stm32FlashStorage.h"
#include "hal/stm32/Stm32MotorOutput.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "sensors/SensorSelection.h"
#include "telemetry/CopterConsole.h"
#include "telemetry/LoopStats.h"
#include "telemetry/TaskStats.h"

static_assert(static_cast<uint8_t>(Config::RxProtocol::IBUS) == 0 && static_cast<uint8_t>(Config::RxProtocol::SBUS) == 1 &&
                  static_cast<uint8_t>(Config::RxProtocol::CRSF) == 2,
              "serialrx_provider (copter/Params.h) — номер как в Config::RxProtocol");


Stm32Board board;

#if defined(ARDUINO_ARCH_STM32)
void stm32RebootToBootloader();   // src/stm32/bootloader.cpp
#endif

auto imuDevice = SELECTED_IMU_DEVICE(board);
SelectedImu imuSensor(imuDevice);

auto baroDevice = SELECTED_BARO_DEVICE(board);
SelectedBaro baroSensor(baroDevice);

RcReceiver rcReceiver(board.rcUart());
Stm32MotorOutput motors;
copter::CopterController copterController;
copter::BuildSpec copterBuild;
copter::CopterParams copterParams;
copter::RxInput rxInput;
LoopStats loopStats;
TaskStats taskStats;
CopterConsole copterConsole(Serial, copterController, copterParams, imuSensor, rcReceiver);

static bool configOk = false;
static bool rebootRequired = false;
static uint8_t outputsAtBoot = 0;   // выходов на моторы настроено при включении
copter::BatteryMonitor battery;

// Части цикла для `tasks` (TaskStats::add — по порядку).
enum : uint8_t
{
    TASK_GYRO,
    TASK_BARO,
    TASK_BATTERY,
    TASK_RX,
    TASK_PID,
    TASK_MOTORS,
    TASK_CONSOLE,
};


// Пульт -> ручки контроллера: карта каналов, rxrange, режимы (copter/RcSetup.h).
static copter::CopterRc readRc()
{
    return rxInput.read(rcReceiver.getState().data(), Config::RC_CHANNELS, !rcReceiver.isSignalLost());
}

// Настройки пульта и описание коптера — в работу (при включении, после
// save и отмены в CLI или приложении).
static void applySettings()
{
    copterParams.rc().setChannelCount(Config::RC_CHANNELS);
    rxInput.configure(copterParams.rc(), static_cast<uint16_t>(copterParams.value(copter::PARAM_RX_MIN_USEC)),
                      static_cast<uint16_t>(copterParams.value(copter::PARAM_RX_MAX_USEC)));
    rcReceiver.setThrottleChannel(rxInput.throttleChannel());
    rcReceiver.setProtocol(static_cast<Config::RxProtocol>(static_cast<uint8_t>(copterParams.value(copter::PARAM_SERIALRX_PROVIDER))));
    copterBuild = copterParams.buildSpec();
    battery.configure(copterParams.batteryConfig());
    // другая рама — выходы на моторы настраиваются только при включении
    const uint8_t motorCount = copterController.getTune().frame.count;
    const bool reboot = outputsAtBoot && motorCount != outputsAtBoot;
    if (reboot && !rebootRequired)
    {
        Serial.printf("У рамы теперь %u моторов (было %u) — перезагрузите плату, до того ARM запрещён.\n", motorCount,
                      outputsAtBoot);
    }
    rebootRequired = reboot;
}

static uint32_t lap(uint8_t task, uint32_t sinceUs)
{
    const uint32_t now = micros();
    taskStats.record(task, now - sinceUs);
    return now;
}

// Батарея и ток (раз в 10 мс): мВ на пинах АЦП -> copter/Battery.h (масштабы
// vbat_scale, ibata_scale, ibata_offset — настройки коптера).
static void readBattery()
{
    if (Config::PIN_VBAT_ADC < 0) return;
    auto pinMv = [](int16_t pin) {
        return pin < 0 ? 0.0f : static_cast<float>(analogRead(static_cast<uint32_t>(pin))) * 3300.0f / 4095.0f;
    };
    battery.update(pinMv(Config::PIN_VBAT_ADC), pinMv(Config::PIN_CURRENT_ADC), 0.01f);
}

static void flightTask(void*)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t tick = 0;

    for (;;)
    {
        const uint32_t start = micros();
        uint32_t t = start;

        imuSensor.update();
        t = lap(TASK_GYRO, t);
        if (tick % 20 == 0)
        {
            baroSensor.update();
            t = lap(TASK_BARO, t);
        }
        if (tick % 10 == 0)
        {
            readBattery();
            t = lap(TASK_BATTERY, t);
        }
        rcReceiver.update();
        const copter::CopterRc rc = readRc();
        t = lap(TASK_RX, t);

        copter::CopterSensors sens;
        const ImuData& imu = imuSensor.getImuData();
        sens.gyro[0] = imu.gyroX;
        sens.gyro[1] = imu.gyroY;
        sens.gyro[2] = imu.gyroZ;
        sens.accel[0] = imu.accelX;
        sens.accel[1] = imu.accelY;
        sens.accel[2] = imu.accelZ;
        sens.imuOk = imuSensor.isAvailable();
        sens.baroOk = baroSensor.isAvailable();
        sens.baroAltM = baroSensor.getBarometerData().altitude;
        sens.batteryVolts = battery.state() == copter::BatteryState::NOT_PRESENT ? 0.0f : battery.volts();

        // ARM запрещён, пока описание коптера не сошлось с платой,
        // установка IMU не откалибрована (если плата этого требует),
        // открыт CLI, в приложении несохранённое, нужна перезагрузка или
        // цикл не успевает
        uint32_t blockers = 0;
        if (!configOk) blockers |= copter::ARM_BLOCK_CONFIG;
        if (Config::IMU_MOUNT_CALIBRATION_REQUIRED && !imuSensor.isMountCalibrated()) blockers |= copter::ARM_BLOCK_NO_ACC_CAL;
        if (copterConsole.cliActive()) blockers |= copter::ARM_BLOCK_CLI;
        if (copterConsole.appUnsaved()) blockers |= copter::ARM_BLOCK_MSP;
        if (rebootRequired) blockers |= copter::ARM_BLOCK_REBOOT;
        if (copterConsole.cpuPercent() >= 100) blockers |= copter::ARM_BLOCK_LOAD;
        copterController.setExternalBlockers(blockers);

        const copter::CopterOutputs out = copterController.update(sens, rc);
        t = lap(TASK_PID, t);
        motors.write(out.motor, out.spinning);
        t = lap(TASK_MOTORS, t);

        copterConsole.setOutputs(out);
        copterConsole.update(millis());
        lap(TASK_CONSOLE, t);

        const uint32_t workUs = micros() - start;
        loopStats.record(workUs);
        taskStats.publish(millis());
        copterConsole.setLoopLoad(loopStats.avgUs, loopStats.maxUs, loopStats.hz);
        ++tick;

        // После долгой блокировки (калибровка из консоли) не догоняем такты пачкой.
        if (xTaskGetTickCount() - lastWake > pdMS_TO_TICKS(100)) lastWake = xTaskGetTickCount();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}

static void storageTask(void*)
{
    for (;;)
    {
#if defined(BOARD_STM32F405)
        // у F405 один банк флеша: пока стирается сектор (1–2 с), процессор
        // стоит — и полётный цикл тоже. Пишем только без ARM (настройки и
        // калибровки и так меняются только на земле).
        if (!copterController.isArmed())
#endif
        Stm32FlashStorage::instance().service();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Описание коптера -> настройка (+ заданное в CLI); что не так — в консоль и запрет ARM.
static void setupCopter(KeyValueStore& store)
{
    configOk = CopterConfig::build(copterBuild);
    if (!configOk)
    {
        Serial.printf("ОШИБКА: сборки \"%s\" нет в каталоге copter/Presets.h — ARM запрещён.\n", COPTER_BUILD);
        uint8_t n = 0;
        copterBuild = copter::buildCatalog(n)[0];   // чтобы контроллер был настроен хоть как-то
    }

    copterParams.setTableDefault(copter::PARAM_SERIALRX_PROVIDER, static_cast<uint8_t>(Config::RX_PROTOCOL));
    copterParams.setTableDefault(copter::PARAM_VBAT_SCALE, roundf(Config::BLACKBOX_VBAT_DIVIDER * 10.0f));
    copterParams.setTableDefault(copter::PARAM_CURRENT_METER, Config::PIN_CURRENT_ADC >= 0 ? 1.0f : 0.0f);
    copterParams.setTableDefault(copter::PARAM_IBATA_SCALE, Config::BOARD_IBATA_SCALE);
    copterParams.setTableDefault(copter::PARAM_IBATA_OFFSET, Config::BOARD_IBATA_OFFSET_MV);
    copter::AuxRange fallback[copter::RcConfig::AUX_SLOTS];
    copterParams.rc().setFallback(fallback, CopterConfig::defaultAux(fallback, copter::RcConfig::AUX_SLOTS));
    copterParams.begin(copterBuild, CopterConfig::LOOP_HZ, CopterConfig::ESC_PROTOCOL, CopterConfig::applyOverrides);
    copterParams.load(store);   // описание во флеше другое — модель заново
    copterConsole.setStore(&store);
    const copter::CopterTune tune = copterParams.tune();
    copter::CopterSettings settings = CopterConfig::settings();
    copterParams.applyTo(settings);
    copterController.configure(tune, settings);
    copterBuild = copterParams.buildSpec();

    if (tune.frame.count > CopterConfig::MOTOR_PIN_COUNT)
    {
        Serial.printf("ОШИБКА: у рамы %s %u моторов, а выходов на плате %u — ARM запрещён.\n",
                      copter::frameName(tune.frame.type), tune.frame.count, CopterConfig::MOTOR_PIN_COUNT);
        configOk = false;
    }
    if (!motors.begin(CopterConfig::MOTOR_PINS, tune.frame.count, CopterConfig::ESC_PROTOCOL))
    {
        Serial.printf("ОШИБКА: выходы на моторы не настроились (%s) — ARM запрещён.\n",
                      copter::escProtocolName(CopterConfig::ESC_PROTOCOL));
        configOk = false;
    }
    else if (motors.attached() < tune.frame.count)
    {
        Serial.printf("ОШИБКА: настроено %u выходов из %u — ARM запрещён.\n", motors.attached(), tune.frame.count);
        configOk = false;
    }
    outputsAtBoot = tune.frame.count;

    Serial.printf("Коптер: %s (%s), %s, регуляторы %s, цикл %.0f Гц.\n", copterBuild.id, copterBuild.title,
                  copter::frameName(tune.frame.type), copter::escProtocolName(CopterConfig::ESC_PROTOCOL), tune.loopHz);
    Serial.printf("Тяговооружённость %.1f, висение на %.0f %% тяги, полоса контура %.1f Гц ('t' — подробно).\n",
                  tune.thrustToWeight, tune.hoverThrust * 100.0f, tune.crossoverHzRP);
    Serial.printf("PID %s, профиль %u, расходы — профиль %u ('#' — CLI как в Betaflight).\n",
                  copterParams.source() == copter::TuneSource::BETAFLIGHT ? "— дефолты Betaflight 4.5" : "из модели",
                  copterParams.pidProfile(), copterParams.rateProfile());
    if (tune.warnings) Serial.println("Есть ПРЕДУПРЕЖДЕНИЯ по сборке — 't' в консоли.");
}


// Вызывается ядром Arduino, а не из кода проекта.
// cppcheck-suppress unusedFunction
void setup()
{
    // порядок — как номера TASK_*
    taskStats.add("GYRO");
    taskStats.add("BARO");
    taskStats.add("BATTERY");
    taskStats.add("RX");
    taskStats.add("PID/MIXER");
    taskStats.add("MOTORS");
    taskStats.add("CONSOLE");

    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" OPENPLANE — МУЛЬТИКОПТЕР");
    Serial.println(" STM32H743 / FreeRTOS");
    Serial.println("=================================");

    board.begin();
    analogReadResolution(12);

    KeyValueStore& settings = Stm32FlashStorage::store();
    settings.mount();
    if (settings.wasCorrupt()) Serial.println("Настройки во флеше повреждены — значения по умолчанию.");

    setupCopter(settings);   // выходы на моторы — сразу «стоп»

    if (imuSensor.begin()) imuSensor.calibrate();
    if (baroSensor.begin()) baroSensor.calibrateAltitude();

    rcReceiver.begin();
    applySettings();   // протокол из настроек, карта каналов, режимы
    char mapText[copter::RcConfig::MAPPABLE + 1];
    copterParams.rc().formatMap(mapText, sizeof(mapText));
    Serial.printf("Приёмник %s%s, каналы %s. Режимы на тумблерах (%s):", rcReceiver.getProtocolName(),
                  rcReceiver.isPortReady() ? "" : " (UART не умеет этот формат — связи не будет)", mapText,
                  copterParams.rc().anyAux() ? "aux из CLI" : "умолчание прошивки");
    const copter::AuxRange* aux = copterParams.rc().effectiveAux();
    for (uint8_t i = 0; i < copter::RcConfig::AUX_SLOTS; ++i)
    {
        if (!aux[i].used()) continue;
        const copter::BoxInfo* box = copter::findBox(aux[i].mode);
        Serial.printf(" %s — CH%u %u..%u;", box ? box->name : "?", copterParams.rc().auxChannel(aux[i].channel) + 1,
                      aux[i].startUs, aux[i].endUs);
    }
    Serial.println(" иначе ACRO.");
#if defined(ARDUINO_ARCH_STM32)
    copterConsole.setBootloaderHook(stm32RebootToBootloader);
#endif
    copterConsole.setApplyHook(applySettings);
    copterConsole.setRxInput(&rxInput);
    copterConsole.setBattery(&battery);
    copterConsole.setTaskStats(&taskStats);
    copterConsole.printHint();

    Rtos::startTask(flightTask, "flight", 16384, nullptr, Rtos::PRIORITY_FLIGHT);
    Rtos::startTask(storageTask, "storage", 2048, nullptr, Rtos::PRIORITY_BACKGROUND);
    vTaskStartScheduler();   // не возвращается

    Serial.println("FreeRTOS не запустился (мало памяти?)");
}

// Не вызывается: после vTaskStartScheduler() работают только задачи.
// cppcheck-suppress unusedFunction
void loop()
{
}

#endif   // VEHICLE_MULTIROTOR
