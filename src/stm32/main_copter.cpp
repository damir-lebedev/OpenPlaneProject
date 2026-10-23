// ============================================================
// МУЛЬТИКОПТЕР — STM32H743 (сборка VEHICLE_MULTIROTOR:
// env matek-h743-quad, stm32h743-devebox-quad)
//
// Те же датчики, приёмник и плата, что у самолёта (src/stm32/main.cpp
// в такой сборке выключен), а управление — copter/CopterController.h:
// контур угловой скорости, режимы ACRO/HORIZON/ANGLE/ALT_HOLD,
// failsafe с посадкой, защита от переворота. Настройка (PID, фильтры,
// холостые, расходы) выводится из описания коптера — config/CopterConfig.h;
// поверх неё — настройки из CLI (copter/Params.h, во флеше).
//
// Задачи FreeRTOS:
//   flight  (приоритет 5) — цикл стабилизации каждые 1 мс: IMU,
//           барометр (раз в 20 мс), батарея, пульт, контроллер, моторы,
//           консоль;
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
LoopStats loopStats;
CopterConsole copterConsole(Serial, copterController, copterParams, copterBuild, imuSensor, rcReceiver);

static bool configOk = false;
static float batteryVolts = 0.0f;


// Пульт -> ручки контроллера (каналы Mode 2, config/Channels.h).
static copter::CopterRc readRc()
{
    copter::CopterRc rc;
    const RcChannelState& st = rcReceiver.getState();
    auto stick = [](uint16_t us) { return constrain((static_cast<int32_t>(us) - 1500) / 500.0f, -1.0f, 1.0f); };
    rc.linkOk = !rcReceiver.isSignalLost();
    rc.roll = stick(st.get(Channels::AILERON));
    rc.pitch = -stick(st.get(Channels::ELEVATOR));   // от себя — нос вниз
    rc.yaw = stick(st.get(Channels::RUDDER));
    rc.throttle = constrain((static_cast<int32_t>(st.get(Channels::THROTTLE)) - 1000) / 1000.0f, 0.0f, 1.0f);
    rc.armSwitch = st.get(Channels::ARM) > 1700;
    rc.mode = CopterConfig::modeFromChannel(st.get(CopterConfig::MODE_CHANNEL));
    return rc;
}

// Батарея: делитель на плате, усреднение ~0.1 с (контроллер сглаживает дальше).
static void readBattery()
{
    if (Config::PIN_VBAT_ADC < 0) return;
    const float mv = static_cast<float>(analogRead(Config::PIN_VBAT_ADC)) * 3300.0f / 4095.0f;
    const float volts = mv * 0.001f * Config::BLACKBOX_VBAT_DIVIDER;
    batteryVolts = batteryVolts <= 0.0f ? volts : batteryVolts + 0.1f * (volts - batteryVolts);
}

static void flightTask(void*)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t tick = 0;

    for (;;)
    {
        const uint32_t start = micros();

        imuSensor.update();
        if (tick % 20 == 0) baroSensor.update();
        if (tick % 10 == 0) readBattery();
        rcReceiver.update();

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
        sens.batteryVolts = batteryVolts;

        // ARM запрещён, пока описание коптера не сошлось с платой,
        // установка IMU не откалибрована (если плата этого требует) или
        // открыт CLI
        const bool mountMissing = Config::IMU_MOUNT_CALIBRATION_REQUIRED && !imuSensor.isMountCalibrated();
        copterController.setExternalBlock(!configOk || mountMissing || copterConsole.cliActive());

        const copter::CopterOutputs out = copterController.update(sens, readRc());
        motors.write(out.motor, out.spinning);

        copterConsole.update(millis());

        const uint32_t workUs = micros() - start;
        loopStats.record(workUs);
        copterConsole.setLoopLoad(loopStats.avgUs, loopStats.maxUs);
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

    copterParams.begin(copterBuild, CopterConfig::LOOP_HZ, CopterConfig::ESC_PROTOCOL);
    copterParams.load(store);
    copterConsole.setStore(&store);
    const copter::CopterTune tune = copterParams.tune();
    copterController.configure(tune, CopterConfig::settings());

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
    Serial.printf("Приёмник %s%s. ARM — SwA (CH5) вниз при газе внизу; режим — SwC (CH7): %s / %s / %s.\n",
                  rcReceiver.getProtocolName(), rcReceiver.isPortReady() ? "" : " (UART не умеет этот формат — связи не будет)",
                  copter::copterModeName(CopterConfig::MODE_UP), copter::copterModeName(CopterConfig::MODE_MIDDLE),
                  copter::copterModeName(CopterConfig::MODE_DOWN));
#if defined(ARDUINO_ARCH_STM32)
    copterConsole.setBootloaderHook(stm32RebootToBootloader);
#endif
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
