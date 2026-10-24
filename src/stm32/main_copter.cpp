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
//   storage (1) — запись настроек (калибровка установки) во флеш;
//   bbox    (2) — чёрный ящик на SD-карту (только H743: telemetry/CopterBlackBox.h).
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
#include "copter/Beeper.h"
#include "copter/CopterController.h"
#include "copter/CrsfTelemetry.h"
#include "copter/GpsHome.h"
#include "copter/Osd.h"
#include "copter/Params.h"
#include "copter/Vtx.h"
#include "hal/Max7456.h"
#include "hal/Rtos.h"
#include "hal/stm32/Stm32Board.h"
#include "hal/stm32/Stm32FlashStorage.h"
#include "hal/stm32/Stm32MotorOutput.h"
#include "hal/stm32/Stm32SpiBus.h"
#include "hal/stm32/Stm32VtxPort.h"
#include "rc/RcReceiver.h"
#include "sensors/SensorInterface.h"
#include "sensors/SensorSelection.h"
#include "sensors/gps/UbloxM10_Gps.h"
#include "sensors/mag/QMC5883L_Sensor.h"
#include "sensors/mag/QMC5883P_Sensor.h"
#include "telemetry/CopterConsole.h"
#include "telemetry/LoopStats.h"
#include "telemetry/TaskStats.h"

// Чёрный ящик — на SD-карту по SDMMC: есть у H743 (Matek H743, DevEBox), у F405 — нет.
#if defined(BOARD_STM32H743)
#define COPTER_BLACKBOX 1
#include "hal/SdFileRegion.h"
#include "hal/stm32/Stm32SdCard.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/CopterBlackBox.h"
#endif

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

// GPS (u-blox M10) — на UART, где его держал Betaflight; компас на модуле
// GPS (HGLRC M100-5883 и т.п.) — QMC5883L или QMC5883P, какой ответит.
UbloxM10_Gps gps(board.gpsUart());
I2cRegisterDevice magLDevice(board.externalI2c(), QMC5883L_Sensor::DEFAULT_ADDRESS);
QMC5883L_Sensor magL(magLDevice);
I2cRegisterDevice magPDevice(board.externalI2c(), QMC5883P_Sensor::DEFAULT_ADDRESS);
QMC5883P_Sensor magP(magPDevice);
MagnetometerSensor* compass = nullptr;

RcReceiver rcReceiver(board.rcUart());
Stm32MotorOutput motors;
copter::CopterController copterController;
copter::BuildSpec copterBuild;
copter::CopterParams copterParams;
copter::RxInput rxInput;
LoopStats loopStats;
TaskStats taskStats;
CopterConsole copterConsole(Serial, copterController, copterParams, imuSensor, rcReceiver);
#if defined(COPTER_BLACKBOX)
Stm32SdCard sdCard;
SdFileRegion blackBoxFile(sdCard, Config::BLACKBOX_SD_FILE, Config::BLACKBOX_SD_MAX_BYTES);
BlackBoxStorage blackBoxStorage(blackBoxFile);
CopterBlackBox copterBlackBox(copterController, copterParams, blackBoxStorage, loopStats, rcReceiver);
#endif

static bool configOk = false;
static bool rebootRequired = false;
static uint8_t outputsAtBoot = 0;   // выходов на моторы настроено при включении
copter::BatteryMonitor battery;
Stm32VtxPort vtxPort(Config::BOARD_VTX_PIN);
copter::VtxController vtx(vtxPort);
static copter::VtxProtocol vtxProtocol = copter::VtxProtocol::NONE;
static bool everArmed = false;
static uint8_t pinioLevels = 0;
copter::GpsHome gpsHome;
static uint32_t lastGpsStamp = 0;
static uint32_t lastGpsMs = 0, lastTelemetryMs = 0;
copter::CrsfTelemetry crsfTelemetry;
static bool crsfTelemetryOn = false;
static copter::CopterMode switchMode = copter::CopterMode::ACRO;   // режим на тумблерах (без ARM — для пульта)

// OSD (AT7456E/MAX7456 на своей шине SPI) — рисуется фоновой задачей.
SPIClass osdSpi;
Stm32SpiBus osdBus(osdSpi, static_cast<pin_size_t>(Config::BOARD_OSD_SCK < 0 ? 0 : Config::BOARD_OSD_SCK),
                   static_cast<pin_size_t>(Config::BOARD_OSD_MISO < 0 ? 0 : Config::BOARD_OSD_MISO),
                   static_cast<pin_size_t>(Config::BOARD_OSD_MOSI < 0 ? 0 : Config::BOARD_OSD_MOSI));
Max7456 osdChip(osdBus, Config::BOARD_OSD_CS);
copter::OsdScreen osdScreen;
static uint32_t armedMs = 0;   // время в полёте (заармлен), для OSD

// Пищалка — copter/Beeper.h.
copter::Beeper beeper;
static bool wasArmedForBeeper = false;
static bool readyBeeped = false;
static uint32_t lastBeeperMs = 0;

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
    TASK_GPS,
    TASK_TELEMETRY,
};


// Пульт -> ручки контроллера: карта каналов, rxrange, режимы (copter/RcSetup.h).
static copter::CopterRc readRc()
{
    copter::CopterRc rc = rxInput.read(rcReceiver.getState().data(), Config::RC_CHANNELS, !rcReceiver.isSignalLost(), millis());
    // кадры приёмника — для упреждения (feedforward_*); 0 до первого кадра
    rc.frame = rcReceiver.getGoodFrameCount();
    return rc;
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
    const auto protocol = static_cast<copter::VtxProtocol>(static_cast<uint8_t>(copterParams.value(copter::PARAM_VTX_PROTOCOL)));
    if (protocol != vtxProtocol || !vtxPort.isConnected())
    {
        vtxProtocol = vtxPort.isConnected() ? protocol : copter::VtxProtocol::NONE;
        vtx.setProtocol(vtxProtocol);
    }
    vtx.setMaxPowerMw(static_cast<uint16_t>(copterParams.value(copter::PARAM_VTX_MAX_POWER_MW)));
    gpsHome.setMinimumSats(static_cast<uint8_t>(copterParams.value(copter::PARAM_GPS_MINIMUM_SATS)));
    crsfTelemetryOn = copterParams.value(copter::PARAM_CRSF_TELEMETRY) > 0.5f &&
                      rcReceiver.getProtocol() == Config::RxProtocol::CRSF;
    // другая рама — выходы на моторы настраиваются только при включении
    const uint8_t motorCount = copterController.getTune().frame.count;
    const bool reboot = outputsAtBoot && motorCount != outputsAtBoot;
    if (reboot && !rebootRequired)
    {
        Serial.printf("У рамы теперь %u моторов (было %u) — перезагрузите плату, до того ARM запрещён.\n", motorCount,
                      outputsAtBoot);
    }
    rebootRequired = reboot;
#if defined(COPTER_BLACKBOX)
    copterBlackBox.configure(copterParams.value(copter::PARAM_BLACKBOX_DEVICE) > 0.5f,
                             static_cast<CopterBlackBox::Mode>(static_cast<uint8_t>(copterParams.value(copter::PARAM_BLACKBOX_MODE))),
                             static_cast<uint8_t>(1u << static_cast<uint8_t>(copterParams.value(copter::PARAM_BLACKBOX_SAMPLE_RATE))));
#endif
}

// Видеопередатчику — что нужно сейчас: настройки vtx_*, строки `vtx` с
// тумблера, режимы VTX PIT MODE / VTX CONTROL DISABLE, без ARM — слабее
// (vtx_low_power_disarm). Потолок мощности держит сам VtxController.
static copter::VtxRequest vtxRequest()
{
    using namespace copter;
    VtxRequest r;
    const VtxSelection sel = rxInput.vtxSelection();
    r.band = sel.band ? sel.band : static_cast<uint8_t>(copterParams.value(PARAM_VTX_BAND));
    r.channel = sel.channel ? sel.channel : static_cast<uint8_t>(copterParams.value(PARAM_VTX_CHANNEL));
    r.frequencyMhz = r.band ? vtxFrequency(r.band, r.channel) : static_cast<uint16_t>(copterParams.value(PARAM_VTX_FREQ));
    r.powerLevel = sel.power ? sel.power : static_cast<uint8_t>(copterParams.value(PARAM_VTX_POWER));
    const uint8_t lowPower = static_cast<uint8_t>(copterParams.value(PARAM_VTX_LOW_POWER_DISARM));
    if (!copterController.isArmed() && (lowPower == 1 || (lowPower == 2 && !everArmed))) r.powerLevel = 1;
    r.pit = rxInput.isActive(BOX_VTX_PIT_MODE);
    r.control = !rxInput.isActive(BOX_VTX_CONTROL_DISABLE);
    return r;
}

// PINIO: ножка в «включено», пока включён её режим USER1…USER4 (с инверсией
// платы, как pinio_config в Betaflight).
static void updatePinio(bool force)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        const int16_t pin = Config::BOARD_PINIO_PINS[i];
        if (pin < 0) continue;
        const uint8_t box = Config::BOARD_PINIO_BOX[i];
        const bool on = box < copter::BOX_COUNT && rxInput.isActive(box);
        const bool level = on != Config::BOARD_PINIO_INVERTED[i];
        if (!force && level == static_cast<bool>((pinioLevels >> i) & 1u)) continue;
        if (force) pinMode(static_cast<uint32_t>(pin), OUTPUT);
        digitalWrite(static_cast<uint32_t>(pin), level ? HIGH : LOW);
        pinioLevels = static_cast<uint8_t>(level ? (pinioLevels | (1u << i)) : (pinioLevels & ~(1u << i)));
    }
}

// Новый фикс GPS -> дом, расстояние, направление (copter/GpsHome.h).
static void updateGps()
{
    const GpsData& d = gps.getGpsData();
    if (d.timestamp == lastGpsStamp) return;
    lastGpsStamp = d.timestamp;
    copter::GpsFix f;
    f.valid = gps.isAvailable() && d.fixType >= 3;
    f.satellites = d.numSatellites;
    f.latitude = d.latitude;
    f.longitude = d.longitude;
    f.altitudeM = d.altitude;
    f.speedMs = d.groundSpeed;
    f.courseDeg = d.heading;
    f.accuracyM = d.horizontalAccuracy;
    gpsHome.update(f, copterController.isArmed());
}

// Курс носа, 0 — север: по компасу (если откалиброван), на скорости — по
// GPS, иначе — то, что насчитал гироскоп (от точки включения).
static float headingDeg()
{
    if (compass && compass->isAvailable() && compass->isCalibrated()) return compass->getMagData().headingDegrees;
    const copter::GpsFix& f = gpsHome.fix();
    if (f.valid && f.speedMs > 3.0f) return f.courseDeg;
    float yaw = copterController.getStatus().yaw;
    if (yaw < 0.0f) yaw += 360.0f;
    return yaw;
}

// Режим для пульта, как пишет Betaflight (4 буквы, без ARM — «*»).
static const char* crsfModeName(copter::CopterMode m)
{
    switch (m)
    {
        case copter::CopterMode::ANGLE: return "ANGL";
        case copter::CopterMode::HORIZON: return "HOR";
        case copter::CopterMode::ALT_HOLD: return "ALTH";
        case copter::CopterMode::ACRO:
        default: return "ACRO";
    }
}

// Следующий кадр телеметрии CRSF — в UART приёмника (copter/CrsfTelemetry.h).
static void sendTelemetry()
{
    const copter::CopterStatus& s = copterController.getStatus();
    copter::CrsfTelemetryData d;
    d.volts = battery.volts();
    d.amps = battery.amps();
    d.mahDrawn = battery.mahDrawn();
    d.percentLeft = battery.percentLeft();
    d.rollDeg = s.roll;
    d.pitchDeg = s.pitch;
    d.yawDeg = headingDeg();
    d.climbMs = s.climbRate;
    const copter::GpsFix& f = gpsHome.fix();
    d.gps = f.valid;
    d.latitude = f.latitude;
    d.longitude = f.longitude;
    d.speedMs = f.speedMs;
    d.courseDeg = f.courseDeg;
    d.altitudeM = f.altitudeM;
    d.satellites = f.satellites;
    const copter::CopterMode mode = copterController.isArmed() ? s.mode : switchMode;
    copter::CrsfTelemetry::modeName(d.mode, sizeof(d.mode), crsfModeName(mode), copterController.isArmed(),
                                    s.state == copter::CopterState::FAILSAFE);
    uint8_t frame[copter::CrsfTelemetry::MAX_FRAME];
    const uint8_t n = crsfTelemetry.next(d, frame);
    if (n) board.rcUart().write(frame, n);
}

// Пищалка раз в 10 мс: ARM/DISARM/готов, батарея, failsafe, режим BEEPER.
static void updateBeeper()
{
    if (Config::PIN_BUZZER < 0) return;
    const bool armed = copterController.isArmed();
    if (armed != wasArmedForBeeper) beeper.event(armed ? copter::BeeperEvent::ARMED : copter::BeeperEvent::DISARMED);
    wasArmedForBeeper = armed;
    if (!readyBeeped && !armed && copterController.getStatus().armBlockers == 0)
    {
        readyBeeped = true;
        beeper.event(copter::BeeperEvent::READY);
    }
    copter::BeeperInputs in;
    in.batteryWarning = armed && battery.alertState() == copter::BatteryState::WARNING;     // use_vbat_alerts
    in.batteryCritical = armed && battery.alertState() == copter::BatteryState::CRITICAL;
    in.failsafe = copterController.getStatus().state == copter::CopterState::FAILSAFE;
    in.findMe = rxInput.isActive(copter::BOX_BEEPER);
    in.muted = rxInput.isActive(copter::BOX_BEEPER_MUTE);
    const bool on = beeper.tick(in);
    digitalWrite(static_cast<uint32_t>(Config::PIN_BUZZER), on == Config::BOARD_BEEPER_ACTIVE_HIGH ? HIGH : LOW);
}

// Первый код BF причины, почему не армится (для OSD), или nullptr.
static const char* firstArmBlock()
{
    const uint32_t b = copterController.getStatus().armBlockers;
    for (uint32_t bit = 1; bit <= copter::ARM_BLOCK_LAST; bit <<= 1)
    {
        if (b & bit) return copter::armBlockCode(bit);
    }
    return nullptr;
}

// Экран OSD из того, что знает прошивка (copter/Osd.h).
static void composeOsd()
{
    if (rxInput.isActive(copter::BOX_OSD))   // режим OSD DISABLE — пусто
    {
        osdScreen.clear();
        return;
    }
    const copter::CopterStatus& s = copterController.getStatus();
    copter::OsdData d;
    d.armed = copterController.isArmed();
    d.failsafe = s.state == copter::CopterState::FAILSAFE;
    d.mode = crsfModeName(d.armed ? s.mode : switchMode);
    d.linkQuality = rcReceiver.getLinkQuality();
    d.linkLost = rcReceiver.isSignalLost();
    d.volts = battery.volts();
    d.cellVolts = battery.cellVolts();
    d.hasCurrent = battery.hasCurrent();
    d.amps = battery.amps();
    d.mahDrawn = battery.mahDrawn();
    d.batteryState = static_cast<uint8_t>(battery.alertState());
    d.batteryNotFull = battery.state() != copter::BatteryState::NOT_PRESENT && !battery.wasFullWhenPluggedIn();
    d.flightSeconds = armedMs / 1000;
    d.altitudeM = s.altitude;
    d.gpsPresent = gps.isAvailable();
    d.satellites = gpsHome.fix().satellites;
    d.home = gpsHome.hasHome();
    d.homeDistanceM = gpsHome.distanceM();
    d.homeRelativeDeg = gpsHome.relativeHomeDeg(headingDeg());
    d.vtxMw = vtx.state().online ? vtx.state().powerMw : 0;
    d.vtxPit = vtx.state().pit;
    d.armBlock = d.armed ? nullptr : firstArmBlock();
    osdScreen.setRows(osdChip.rows());
    osdScreen.compose(d);
}

// Фоновая задача OSD: 20 раз в секунду экран — в чип (только изменения), раз
// в секунду — проверить, не сменилась ли камера (PAL/NTSC).
static void osdTask(void*)
{
    uint32_t lastCheckMs = 0;
    for (;;)
    {
        composeOsd();
        osdChip.flush(osdScreen, 60);
        if (millis() - lastCheckMs >= 1000)
        {
            lastCheckMs = millis();
            osdChip.check();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
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
        switchMode = rc.mode;
        t = lap(TASK_RX, t);
        if (millis() - lastGpsMs >= 5)
        {
            lastGpsMs = millis();
            gps.update();
            updateGps();
            if (compass) compass->update();   // сам — 50 Гц
            t = lap(TASK_GPS, t);
        }

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
        // компенсации просадки — быстрый фильтр (vbat_sag_lpf_period), а не показ
        sens.batteryVolts = battery.state() == copter::BatteryState::NOT_PRESENT ? 0.0f : battery.sagVolts();
        copterController.setThrottleLimit(battery.throttleLimit());   // vbat_cutoff_percent

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

        if (copterController.isArmed())
        {
            everArmed = true;
            ++armedMs;
        }
        if (millis() - lastBeeperMs >= 10)
        {
            lastBeeperMs = millis();
            updateBeeper();
        }
        updatePinio(false);
        vtx.request(vtxRequest());
        vtx.update(millis());
        copterConsole.setOutputs(out);
        copterConsole.setHeading(headingDeg());
        copterConsole.update(millis());
        t = lap(TASK_CONSOLE, t);
        if (crsfTelemetryOn && millis() - lastTelemetryMs >= 50)
        {
            lastTelemetryMs = millis();
            sendTelemetry();
            lap(TASK_TELEMETRY, t);
        }

        const uint32_t workUs = micros() - start;
        loopStats.record(workUs);
#if defined(COPTER_BLACKBOX)
        copterBlackBox.update(sens, rc, out, workUs);   // снимок в очередь; на карту пишет задача bbox
#endif
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
    copterParams.setTableDefault(copter::PARAM_BATTERY_METER, Config::PIN_VBAT_ADC >= 0 ? 1.0f : 0.0f);
#if defined(OPB_VBAT_DIVIDER)   // делитель из дампа BF (tools/bf_board.py)
    copterParams.setTableDefault(copter::PARAM_VBAT_SCALE, OPB_VBAT_SCALE);
    copterParams.setTableDefault(copter::PARAM_VBAT_DIVIDER, OPB_VBAT_DIVIDER);
    copterParams.setTableDefault(copter::PARAM_VBAT_MULTIPLIER, OPB_VBAT_MULTIPLIER);
#endif
    copterParams.setTableDefault(copter::PARAM_IBATA_SCALE, Config::BOARD_IBATA_SCALE);
    copterParams.setTableDefault(copter::PARAM_IBATA_OFFSET, Config::BOARD_IBATA_OFFSET_MV);
    copterParams.setTableDefault(copter::PARAM_VTX_PROTOCOL, static_cast<float>(Config::BOARD_VTX_PIN >= 0 ? copter::VtxProtocol::AUTO
                                                                                                        : copter::VtxProtocol::NONE));
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
    if (copter::isDshot(CopterConfig::ESC_PROTOCOL) && tune.loopHz > copter::dshot::maxLoopHz(CopterConfig::ESC_PROTOCOL))
    {
        Serial.printf("ОШИБКА: %s не успевает за циклом %.0f Гц (до %lu) — ARM запрещён.\n",
                      copter::escProtocolName(CopterConfig::ESC_PROTOCOL), tune.loopHz,
                      static_cast<unsigned long>(copter::dshot::maxLoopHz(CopterConfig::ESC_PROTOCOL)));
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


#if defined(COPTER_BLACKBOX)
// SD-карта и файл чёрного ящика (BLACKBOX.BIN, tools/blackbox.py sd-prepare):
// что нашлось — в консоль; места нет — ящик выключен, полёту это не мешает.
static void setupBlackBox()
{
    Serial.print("SD-карта: ");
    if (!sdCard.begin())
    {
        Serial.printf("нет ответа (код %lu) — карта не вставлена или не опознана\n",
                      static_cast<unsigned long>(sdCard.initError()));
    }
    else
    {
        Serial.printf("%lu МБ, файл %s: %s\n", static_cast<unsigned long>(sdCard.blockCount() / 2048), Config::BLACKBOX_SD_FILE,
                      Fat32::describe(blackBoxFile.begin()));
    }
    copterBlackBox.setBattery(&battery);
    copterBlackBox.setGps(&gps, &gpsHome);
    copterBlackBox.setSensors(&imuSensor, &baroSensor);
    copterBlackBox.begin();
    copterConsole.setRecorder(&copterBlackBox);   // и без карты: "bb list" ответит state=off
}
#endif

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
    taskStats.add("GPS");
    taskStats.add("TELEMETRY");

    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" OPENPLANE — МУЛЬТИКОПТЕР");
    Serial.println(" STM32H743 / FreeRTOS");
    Serial.println("=================================");

    board.begin();
    if (Config::PIN_BUZZER >= 0)   // молчать с первой миллисекунды (у платы может быть «пищит низким»)
    {
        digitalWrite(static_cast<uint32_t>(Config::PIN_BUZZER), Config::BOARD_BEEPER_ACTIVE_HIGH ? LOW : HIGH);
    }
    analogReadResolution(12);

    KeyValueStore& settings = Stm32FlashStorage::store();
    settings.mount();
    if (settings.wasCorrupt()) Serial.println("Настройки во флеше повреждены — значения по умолчанию.");

    setupCopter(settings);   // выходы на моторы — сразу «стоп»

    if (imuSensor.begin()) imuSensor.calibrate();
    if (baroSensor.begin()) baroSensor.calibrateAltitude();
    if (Config::PIN_GPS_RX >= 0) gps.begin();
    if (magL.begin()) compass = &magL;
    else if (magP.begin()) compass = &magP;

    rcReceiver.begin();
    applySettings();   // протокол из настроек, карта каналов, режимы
    updatePinio(true);
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
    copterConsole.setVtx(&vtx);
    copterConsole.setGps(&gps, &gpsHome);
    copterConsole.setCompass(compass);
    if (Config::BOARD_OSD_CS >= 0)
    {
        osdBus.begin();
        Serial.printf("OSD: %s\n", osdChip.begin() ? (osdChip.isPal() ? "AT7456E/MAX7456, PAL" : "AT7456E/MAX7456, NTSC")
                                                     : "чип не отвечает");
    }
    copterConsole.setTaskStats(&taskStats);
#if defined(COPTER_BLACKBOX)
    setupBlackBox();   // последним: задача записи начинает готовить место
#endif
    copterConsole.printHint();

    Rtos::startTask(flightTask, "flight", 16384, nullptr, Rtos::PRIORITY_FLIGHT);
    Rtos::startTask(storageTask, "storage", 2048, nullptr, Rtos::PRIORITY_BACKGROUND);
    if (osdChip.isPresent()) Rtos::startTask(osdTask, "osd", 3072, nullptr, Rtos::PRIORITY_BACKGROUND);
    vTaskStartScheduler();   // не возвращается

    Serial.println("FreeRTOS не запустился (мало памяти?)");
}

// Не вызывается: после vTaskStartScheduler() работают только задачи.
// cppcheck-suppress unusedFunction
void loop()
{
}

#endif   // VEHICLE_MULTIROTOR
