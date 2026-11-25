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
#include "copter/StickCommands.h"
#include "copter/OsdMenu.h"
#include "copter/CopterController.h"
#include "copter/CrsfMsp.h"
#include "copter/CrsfTelemetry.h"
#include "copter/DisplayPort.h"
#include "copter/Compass.h"
#include "copter/Declination.h"
#include "copter/GpsHome.h"
#include "copter/Osd.h"
#include "copter/Params.h"
#include "copter/Rssi.h"
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
#include "sensors/imu/ICM42688_Sensor.h"
#include "sensors/gps/UbloxM10_Gps.h"
#include "sensors/mag/QMC5883L_Sensor.h"
#include "sensors/mag/QMC5883P_Sensor.h"
#include "telemetry/CopterConsole.h"
#include "telemetry/LoopStats.h"
#include "telemetry/MspPort.h"
#include "telemetry/TaskStats.h"

// Чёрный ящик: H743 — на SD-карту по SDMMC (Matek H743, DevEBox); F405 — на
// SPI-флеш платы (W25Q128 и т. п., как blackbox_device = SPIFLASH у BF), а
// если у платы слот microSD на SPI (SpeedyBee F405 V3/V4) — на карту по SPI.
// (F405 — первым: тесты на ПК собираются с BOARD_STM32H743 и добавляют F405.)
#if defined(BOARD_STM32F405) && OPB_SDCARD_SPI == 0
#define COPTER_BLACKBOX 1
#define COPTER_BLACKBOX_FLASH 1
#include "hal/LockedSpiBus.h"
#include "hal/SpiNorFlash.h"
#elif defined(BOARD_STM32F405)
#define COPTER_BLACKBOX 1
#define COPTER_BLACKBOX_SD 1
#define COPTER_BLACKBOX_SD_SPI 1
#define SD_ERASE_CHUNK_BYTES 1024   // ОЗУ F405: кусок стирания 1 КБ вместо 16
#include "hal/LockedSpiBus.h"
#include "hal/SdFileRegion.h"
#include "hal/SpiSdCard.h"
#elif defined(BOARD_STM32H743)
#define COPTER_BLACKBOX 1
#define COPTER_BLACKBOX_SD 1
#include "hal/SdFileRegion.h"
#include "hal/stm32/Stm32SdCard.h"
#endif
#if defined(COPTER_BLACKBOX)
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
#if defined(OPENPLANE_SITL)
void sitlOutputs(const copter::CopterOutputs& out);   // src/sitl/main.cpp: моторы — в модель
#endif

auto imuDevice = SELECTED_IMU_DEVICE(board);
SelectedImu imuSensor(imuDevice);

// Второй гироскоп (Matek H743-SLIM V4: ICM-42688-P на SPI4) — запасной или
// в среднее, настройка gyro_to_use (copter/GyroSelect.h). Нет на плате — CS −1.
constexpr bool HAS_GYRO2 = Config::BOARD_GYRO2_CS >= 0;
SPIClass gyro2Spi;
Stm32SpiBus gyro2Bus(gyro2Spi, static_cast<pin_size_t>(HAS_GYRO2 ? Config::BOARD_GYRO2_SCK : 0),
                     static_cast<pin_size_t>(HAS_GYRO2 ? Config::BOARD_GYRO2_MISO : 0),
                     static_cast<pin_size_t>(HAS_GYRO2 ? Config::BOARD_GYRO2_MOSI : 0));
SpiRegisterDevice gyro2Device = ICM42688_Sensor::spiDevice(gyro2Bus, static_cast<uint8_t>(HAS_GYRO2 ? Config::BOARD_GYRO2_CS : 0));
ICM42688_Sensor gyro2Sensor(gyro2Device);
static copter::GyroToUse gyroToUse = copter::GyroToUse::FIRST;
static copter::GyroStuckDetector gyroStuck[2];
static copter::GyroAgreement gyroAgreement;
static copter::GyroPick gyroPick;
static uint8_t gyroAlive = 0;
static bool gyroFallbackSaid = false;
static bool gyro2Present = false;   // второй ответил при включении

static copter::GyroSample gyroSampleOf(const ImuData& d, bool ok)
{
    copter::GyroSample s;
    s.gyro[0] = d.gyroX;
    s.gyro[1] = d.gyroY;
    s.gyro[2] = d.gyroZ;
    s.accel[0] = d.accelX;
    s.accel[1] = d.accelY;
    s.accel[2] = d.accelZ;
    s.ok = ok;
    return s;
}

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
copter::AutoDeclination autoDeclination;   // склонение по координатам GPS (copter/Declination.h)
copter::Compass copterCompass;   // калибровка, установка, наклон, склонение (copter/Compass.h)
static uint32_t lastMagStamp = 0;
static bool gpsPortOn = Config::PIN_GPS_RX >= 0;   // GPS на каком-то UART (setupPorts)
static bool mspPortOn = Config::PIN_TELEM_TX >= 0;  // MSP (очки) на каком-то UART

RcReceiver rcReceiver(board.rcUart());
Stm32MotorOutput motors;
copter::CopterController copterController;
copter::BuildSpec copterBuild;
copter::CopterParams copterParams;
copter::RxInput rxInput;
LoopStats loopStats;
TaskStats taskStats;
CopterConsole copterConsole(Serial, copterController, copterParams, imuSensor, rcReceiver);
// OSD (AT7456E/MAX7456 на своей шине SPI) — рисуется фоновой задачей.
SPIClass osdSpi;
Stm32SpiBus osdBus(osdSpi, static_cast<pin_size_t>(Config::BOARD_OSD_SCK < 0 ? 0 : Config::BOARD_OSD_SCK),
                   static_cast<pin_size_t>(Config::BOARD_OSD_MISO < 0 ? 0 : Config::BOARD_OSD_MISO),
                   static_cast<pin_size_t>(Config::BOARD_OSD_MOSI < 0 ? 0 : Config::BOARD_OSD_MOSI));
#if defined(COPTER_BLACKBOX_FLASH) || defined(COPTER_BLACKBOX_SD_SPI)
// флеш или SD-карта ящика бывают на одной SPI с OSD (HAKRC F405, SpeedyBee F405 V3): тогда каждая транзакция — под мьютексом
LockedSpiBus osdLockedBus(osdBus);
ISpiBus& osdAccess = osdLockedBus;
#else
ISpiBus& osdAccess = osdBus;
#endif
Max7456 osdChip(osdAccess, Config::BOARD_OSD_CS);
#if defined(COPTER_BLACKBOX_SD_SPI)
// слот microSD: своя шина SPI или общая с OSD (SpeedyBee F405 V3 — SPI2 с MAX7456;
// пока карта пишет блок, шину она отпускает — SpiSdCard::waitIdle); на шине
// гироскопа — запрещено в Config.h.
SPIClass sdSpi;
Stm32SpiBus sdBus(sdSpi, static_cast<pin_size_t>(Config::BOARD_SDCARD_SCK < 0 ? 0 : Config::BOARD_SDCARD_SCK),
                  static_cast<pin_size_t>(Config::BOARD_SDCARD_MISO < 0 ? 0 : Config::BOARD_SDCARD_MISO),
                  static_cast<pin_size_t>(Config::BOARD_SDCARD_MOSI < 0 ? 0 : Config::BOARD_SDCARD_MOSI));
SpiSdCard sdCard(Config::BOARD_SDCARD_SHARES_OSD ? osdAccess : static_cast<ISpiBus&>(sdBus), Config::BOARD_SDCARD_CS);
// 64 МБ (~30 минут при 1/2): сверка секторов при включении берёт из кучи бит на
// сектор 4 КБ — на 256 МБ это 8 КБ, у F405 столько свободного ОЗУ нет.
static constexpr uint32_t SD_REGION_MAX_BYTES = 64UL * 1024 * 1024;
#elif defined(COPTER_BLACKBOX_SD)
Stm32SdCard sdCard;
static constexpr uint32_t SD_REGION_MAX_BYTES = Config::BLACKBOX_SD_MAX_BYTES;
#endif
#if defined(COPTER_BLACKBOX_SD)
SdFileRegion blackBoxFile(sdCard, Config::BLACKBOX_SD_FILE, SD_REGION_MAX_BYTES);
BlackBoxStorage blackBoxStorage(blackBoxFile);
CopterBlackBox copterBlackBox(copterController, copterParams, blackBoxStorage, loopStats, rcReceiver);
#endif

static bool configOk = false;
static bool rebootRequired = false;
static bool bidirAtBoot = false;   // dshot_bidir при включении (меняется после перезагрузки)
static uint8_t outputsAtBoot = 0;   // выходов на моторы настроено при включении
copter::BatteryMonitor battery;
// Перегрузка регулятора и батареи по току — copter/EscGuard.h.
copter::EscGuard escGuard;
// EDT (dshot_edt): команда регуляторам при старте и повтор, если молчат.
static copter::dshot::EdtEnabler edtEnabler;
copter::Rssi rssi;   // сила сигнала пульта: канал, битые кадры или dBm CRSF (113)
copter::VtxHeatWatch vtxHeat;
static bool wasArmedForGuard = false;
Stm32VtxPort vtxPort(Config::BOARD_VTX_PIN);
copter::VtxController vtx(vtxPort);
static copter::VtxProtocol vtxProtocol = copter::VtxProtocol::NONE;
static bool everArmed = false;
static bool armedLastLoop = false;   // ARM в прошлом цикле: новый ARM — очередь команд DShot прочь
static uint8_t pinioLevels = 0;
copter::GpsHome gpsHome;
static uint32_t lastGpsStamp = 0;
static uint32_t lastGpsMs = 0, lastTelemetryMs = 0;
copter::CrsfTelemetry crsfTelemetry;
copter::CrsfMsp crsfMsp;   // настройка с пульта: Lua-скрипты BF, VTX Admin ELRS
static bool crsfTelemetryOn = false;
static copter::CopterMode switchMode = copter::CopterMode::ACRO;   // режим на тумблерах (без ARM — для пульта)
static volatile bool sticksUp = false;   // газ или тангаж вверх — убрать статистику OSD


#if defined(COPTER_BLACKBOX_FLASH)
// SPI-флеш ящика: своя шина или общая с OSD; на шине гироскопа — не берём
// (страница флеша держала бы шину ~0.1 мс посреди чтения гироскопа).
SPIClass flashSpi;
Stm32SpiBus flashOwnBus(flashSpi, static_cast<pin_size_t>(Config::BOARD_FLASH_SCK < 0 ? 0 : Config::BOARD_FLASH_SCK),
                        static_cast<pin_size_t>(Config::BOARD_FLASH_MISO < 0 ? 0 : Config::BOARD_FLASH_MISO),
                        static_cast<pin_size_t>(Config::BOARD_FLASH_MOSI < 0 ? 0 : Config::BOARD_FLASH_MOSI));
SpiNorFlash blackBoxFlash(Config::BOARD_FLASH_SHARES_OSD ? osdAccess : static_cast<ISpiBus&>(flashOwnBus),
                          Config::BOARD_FLASH_SHARES_GYRO ? -1 : Config::BOARD_FLASH_CS);
BlackBoxStorage blackBoxStorage(blackBoxFlash);
CopterBlackBox copterBlackBox(copterController, copterParams, blackBoxStorage, loopStats, rcReceiver);
#endif
copter::OsdScreen osdScreen;
copter::OsdStats osdStats;
static uint32_t armedMs = 0;   // время в полёте (заармлен), для OSD

// Пищалка — copter/Beeper.h; маяк моторами и команды стиками — там же и в
// copter/StickCommands.h.
copter::Beeper beeper;
copter::DshotBeacon dshotBeacon;
copter::StickCommands stickCommands;
// Меню настроек в очках стиками (CMS, copter/OsdMenu.h).
copter::OsdMenu osdMenu;

// Цифровые очки и видеопередатчики (DJI, Walksnail, HDZero) — на UART «MSP»
// платы (PIN_TELEM_*): OSD по MSP DisplayPort и ответы на их опрос
// (copter/DisplayPort.h, telemetry/MspPort.h). Экран собирает задача OSD,
// в порт пишет полётный цикл.
MspPort devicePort(*board.telemetryUart());
copter::DisplayPort displayPort;
copter::MspVtxLink mspVtx;
copter::OsdOutputStatus osdOutput;
static copter::OsdRouteInput osdRouteIn;        // настройки osd_displayport_device, vcd_video_system, osd_canvas_*
static volatile bool osdToDevice = false;       // задача OSD решила: шлём в очки
static bool wasArmedForBeeper = false;
static bool readyBeeped = false;
static uint32_t lastBeeperMs = 0;
static bool buzzerActiveHigh = Config::BOARD_BEEPER_ACTIVE_HIGH;   // beeper_inversion
static bool buzzerOpenDrain = false;                               // beeper_od
static uint16_t buzzerHz = 0;                                      // beeper_frequency: 0 — активная
static bool buzzerOn = false;
static bool rcSeen = false;     // связь с пультом хоть раз была (RX_LOST — только после)
static bool rcLinkOk = false;
static uint32_t lastStickUs = 0;

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

// Вывод пищалки: beeper_inversion (пищит высоким), beeper_od (открытый
// сток), beeper_frequency (пассивной — меандр этой частоты).
static void writeBuzzer(bool on)
{
    if (Config::PIN_BUZZER < 0) return;
    const uint32_t pin = static_cast<uint32_t>(Config::PIN_BUZZER);
    if (buzzerHz > 0)
    {
        if (on != buzzerOn)
        {
            if (on) tone(static_cast<uint8_t>(pin), buzzerHz);
            else noTone(static_cast<uint8_t>(pin));
        }
        buzzerOn = on;
        return;
    }
    buzzerOn = on;
    digitalWrite(pin, on == buzzerActiveHigh ? HIGH : LOW);
}

static void configureBuzzer()
{
    if (Config::PIN_BUZZER < 0) return;
    const uint32_t pin = static_cast<uint32_t>(Config::PIN_BUZZER);
    const bool od = copterParams.value(copter::PARAM_BEEPER_OD) > 0.5f;
    const uint16_t hz = static_cast<uint16_t>(copterParams.value(copter::PARAM_BEEPER_FREQUENCY));
    if (buzzerHz > 0 && buzzerOn) noTone(static_cast<uint8_t>(pin));
    if (od != buzzerOpenDrain) pinMode(pin, od ? OUTPUT_OPEN_DRAIN : OUTPUT);
    buzzerOpenDrain = od;
    buzzerHz = hz;
    buzzerActiveHigh = copterParams.value(copter::PARAM_BEEPER_INVERSION) > 0.5f;
    buzzerOn = true;
    writeBuzzer(false);   // тихо
}

// Настройки пульта и описание коптера — в работу (при включении, после
// save и отмены в CLI или приложении).
// Поле геозабора для очков (P44 б): ближний край, масштаб радара и стороны
// рядом с коптером в осях носа (дальние отбрасываем — их радар не покажет).
static void fillFenceOsd(copter::OsdData& d, float headingDeg)
{
    const copter::Geofence& f = copterController.getGeofence();
    d.fencePolygon = f.hasPolygons() && f.positionKnown();
    d.fenceEdgeCount = 0;
    if (!d.fencePolygon) return;
    d.fenceMarginM = f.marginM();
    d.fenceEdgeRelDeg = f.polygonNearer() ? f.edgeBearingDeg() - headingDeg : d.homeRelativeDeg + 180.0f;
    d.fenceScaleM = d.fenceRadiusM > 0.0f ? fminf(d.fenceRadiusM, f.radarSpanM()) : f.radarSpanM();
    const float window = 6.0f * fmaxf(1.0f, d.fenceScaleM) / copter::OsdScreen::FENCE_MAP_SPAN;
    float pn, pe;
    f.positionM(pn, pe);
    const float h = headingDeg * 0.017453293f, ch = cosf(h), sh = sinf(h);
    for (uint8_t i = 0; i < f.polygonCount(); ++i)
    {
        const uint8_t n = f.polygonPoints(i);
        for (uint8_t k = 0; k < n && d.fenceEdgeCount < copter::OsdData::FENCE_EDGES_MAX; ++k)
        {
            float an, ae, bn, be;
            f.polygonPoint(i, k, an, ae);
            f.polygonPoint(i, static_cast<uint8_t>((k + 1) % n), bn, be);
            an -= pn; ae -= pe; bn -= pn; be -= pe;
            if (copter::segmentDistance(0.0f, 0.0f, an, ae, bn, be) > window) continue;
            float* e = d.fenceEdges[d.fenceEdgeCount++];
            e[0] = an * ch + ae * sh;    // вперёд
            e[1] = -an * sh + ae * ch;   // вправо
            e[2] = bn * ch + be * sh;
            e[3] = -bn * sh + be * ch;
        }
    }
}

static void configureGyroCalibration();

static void applySettings()
{
    copterParams.rc().setChannelCount(Config::RC_CHANNELS);
    rxInput.configure(copterParams.rc(), static_cast<uint16_t>(copterParams.value(copter::PARAM_RX_MIN_USEC)),
                      static_cast<uint16_t>(copterParams.value(copter::PARAM_RX_MAX_USEC)));
    rcReceiver.setThrottleChannel(rxInput.throttleChannel());
    rcReceiver.setProtocol(static_cast<Config::RxProtocol>(static_cast<uint8_t>(copterParams.value(copter::PARAM_SERIALRX_PROVIDER))));
    rssi.configure(copterParams.rssiConfig());
    copterBuild = copterParams.buildSpec();
    battery.configure(copterParams.batteryConfig());
    escGuard.configure(copterParams.escGuardConfig(copterController.getTune().frame.count));
    const auto protocol = static_cast<copter::VtxProtocol>(static_cast<uint8_t>(copterParams.value(copter::PARAM_VTX_PROTOCOL)));
    if (protocol != vtxProtocol || !vtxPort.isConnected())
    {
        vtxProtocol = vtxPort.isConnected() ? protocol : copter::VtxProtocol::NONE;
        vtx.setProtocol(vtxProtocol);
    }
    vtx.setMaxPowerMw(static_cast<uint16_t>(copterParams.value(copter::PARAM_VTX_MAX_POWER_MW)));
    gpsHome.setMinimumSats(static_cast<uint8_t>(copterParams.value(copter::PARAM_GPS_MINIMUM_SATS)));
    copterCompass.configure(copterParams.compassSettings());
    beeper.setOffFlags(static_cast<uint32_t>(copterParams.value(copter::PARAM_BEEPER_OFF_FLAGS)));
    dshotBeacon.configure(static_cast<uint32_t>(copterParams.value(copter::PARAM_DSHOT_BEACON_OFF_FLAGS)),
                          static_cast<uint8_t>(copterParams.value(copter::PARAM_BEEPER_DSHOT_BEACON_TONE)));
    gyroToUse = static_cast<copter::GyroToUse>(static_cast<uint8_t>(copterParams.value(copter::PARAM_GYRO_TO_USE)));
    configureGyroCalibration();
    stickCommands.configure(static_cast<uint16_t>(copterParams.value(copter::PARAM_MIN_CHECK)),
                            static_cast<uint16_t>(copterParams.value(copter::PARAM_MAX_CHECK)));
    osdMenu.configure(static_cast<uint16_t>(copterParams.value(copter::PARAM_MIN_CHECK)),
                      static_cast<uint16_t>(copterParams.value(copter::PARAM_MAX_CHECK)));
    // куда и каким размером рисовать OSD
    osdRouteIn.device = static_cast<copter::OsdDevice>(static_cast<uint8_t>(copterParams.value(copter::PARAM_OSD_DISPLAYPORT_DEVICE)));
    osdRouteIn.video = static_cast<copter::VideoSystem>(static_cast<uint8_t>(copterParams.value(copter::PARAM_VCD_VIDEO_SYSTEM)));
    osdRouteIn.canvasCols = static_cast<uint8_t>(copterParams.value(copter::PARAM_OSD_CANVAS_WIDTH));
    osdRouteIn.canvasRows = static_cast<uint8_t>(copterParams.value(copter::PARAM_OSD_CANVAS_HEIGHT));
    displayPort.configure(static_cast<int8_t>(copterParams.value(copter::PARAM_DISPLAYPORT_MSP_COL_ADJUST)),
                          static_cast<int8_t>(copterParams.value(copter::PARAM_DISPLAYPORT_MSP_ROW_ADJUST)),
                          static_cast<uint8_t>(copterParams.value(copter::PARAM_OSD_FRAMERATE_HZ)));
    osdChip.forceStandard(osdRouteIn.video == copter::VideoSystem::PAL ? 1 : osdRouteIn.video == copter::VideoSystem::NTSC ? 2 : 0);
    configureBuzzer();
    crsfTelemetryOn = copterParams.value(copter::PARAM_CRSF_TELEMETRY) > 0.5f &&
                      rcReceiver.getProtocol() == Config::RxProtocol::CRSF;
    // другая рама — выходы на моторы настраиваются только при включении
    const uint8_t motorCount = copterController.getTune().frame.count;
    const bool bidirChanged = outputsAtBoot && (copterParams.value(copter::PARAM_DSHOT_BIDIR) > 0.5f) != bidirAtBoot;
    if (bidirChanged && !rebootRequired) Serial.println("dshot_bidir изменён — перезагрузите плату, до того ARM запрещён.");
    const bool frameChanged = outputsAtBoot && motorCount != outputsAtBoot;
    const bool reboot = frameChanged || bidirChanged;
    if (frameChanged && !rebootRequired)
    {
        Serial.printf("У рамы теперь %u моторов (было %u) — перезагрузите плату, до того ARM запрещён.\n", motorCount,
                      outputsAtBoot);
    }
    rebootRequired = reboot;
#if defined(COPTER_BLACKBOX)
    // ящик пишет, если выбран носитель этой платы (diff из BF с SPIFLASH на H743 — выключен)
#if defined(COPTER_BLACKBOX_FLASH)
    const uint8_t boardDevice = copter::BLACKBOX_SPIFLASH;
#else
    const uint8_t boardDevice = copter::BLACKBOX_SDCARD;
#endif
    copterBlackBox.configure(static_cast<uint8_t>(copterParams.value(copter::PARAM_BLACKBOX_DEVICE)) == boardDevice,
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
static bool gpsFresh = false;   // новый фикс с прошлого цикла — возврату домой

static void updateGps()
{
    const GpsData& d = gps.getGpsData();
    if (d.timestamp == lastGpsStamp) return;
    lastGpsStamp = d.timestamp;
    gpsFresh = true;
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
    if (autoDeclination.update(f.valid, f.latitude, f.longitude)) copterCompass.setAutoDeclination(autoDeclination.value());
}

// Компас: новый отсчёт датчика -> курс с поправкой на наклон (copter/Compass.h).
static void updateCompass()
{
    if (!compass) return;
    compass->update();   // сам — 50 Гц
    if (!compass->isAvailable())
    {
        copterCompass.lost();
        return;
    }
    const MagData& m = compass->getMagData();
    if (m.timestamp == lastMagStamp) return;
    lastMagStamp = m.timestamp;
    const float raw[3] = { m.magX, m.magY, m.magZ };
    copterCompass.update(raw, copterController.getAttitude());
}

static bool compassTrusted() { return compass && compass->isAvailable() && copterCompass.trusted(); }

// Курс носа, 0 — север: оценка контроллера (компас или курс по полёту —
// copter/PositionEstimator.h), на скорости — курс GPS, иначе — то, что
// насчитал гироскоп (от точки включения).
static float headingDeg()
{
    const copter::CopterStatus& s = copterController.getStatus();
    if (s.headingSource != copter::HeadingSource::NONE) return s.heading;
    if (compassTrusted()) return copterCompass.headingDeg(s.yaw);
    const copter::GpsFix& f = gpsHome.fix();
    if (f.valid && f.speedMs > 3.0f) return f.courseDeg;
    float yaw = copterController.getStatus().yaw;
    if (yaw < 0.0f) yaw += 360.0f;
    return yaw;
}

// GPS для возврата домой (copter/GpsRescue.h): смещение от дома, скорость по
// земле (север, восток), курс носа — по компасу, если он откалиброван.
static void fillGps(copter::GpsInput& g)
{
    const copter::GpsFix& f = gpsHome.fix();
    g.fresh = gpsFresh;
    gpsFresh = false;
    g.fix = f.valid;
    g.sats = f.satellites;
    g.home = gpsHome.offsetFromHome(g.northM, g.eastM);
    const float course = f.courseDeg * 0.017453293f;
    g.velN = f.speedMs * cosf(course);
    g.velE = f.speedMs * sinf(course);
    g.headingTrusted = compassTrusted();
    g.headingDeg = g.headingTrusted ? copterCompass.headingDeg(copterController.getStatus().yaw) : headingDeg();
    // для проверки подмены (copter/GpsIntegrity.h)
    g.latitude = f.latitude;
    g.longitude = f.longitude;
    g.altitudeM = f.altitudeM;
    g.receiverSpoof = gps.isAvailable() ? gps.getGpsData().spoofState : 0;
}

// Режим для пульта, как пишет Betaflight (4 буквы, без ARM — «*»).
static const char* crsfModeName(copter::CopterMode m)
{
    switch (m)
    {
        case copter::CopterMode::ANGLE: return "ANGL";
        case copter::CopterMode::HORIZON: return "HOR";
        case copter::CopterMode::ALT_HOLD: return "ALTH";
        case copter::CopterMode::POS_HOLD: return "POSH";
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
    const copter::GpsFix f = gpsHome.located();   // фикс пропал — последняя точка (найти упавший)
    d.gps = f.valid || gpsHome.hasLastGood();
    d.latitude = f.latitude;
    d.longitude = f.longitude;
    d.speedMs = f.speedMs;
    d.courseDeg = f.courseDeg;
    d.altitudeM = f.altitudeM;
    d.satellites = f.satellites;
    const copter::CopterMode mode = copterController.isArmed() ? s.mode : switchMode;
    const bool rescue = (s.rescuePhase != copter::RescuePhase::IDLE && s.rescuePhase != copter::RescuePhase::ABORT) ||
                        s.deadReckPhase != copter::DeadReckonPhase::IDLE;
    const char* name = s.turtle ? "TURT" : rescue ? "RTH" : crsfModeName(mode);   // turtle mode — как OSD BF
    copter::CrsfTelemetry::modeName(d.mode, sizeof(d.mode), name, copterController.isArmed(),
                                    s.state == copter::CopterState::FAILSAFE && !rescue);
    uint8_t frame[copter::CrsfTelemetry::MAX_FRAME];
    const uint8_t n = crsfTelemetry.next(d, frame);
    if (n) board.rcUart().write(frame, n);
}

// MSP с пульта (CRSF 0x7A/0x7C): собрать запрос, ответить через CopterMsp.
// Ответ уходит кусками вместо кадров телеметрии (sendCrsfMsp).
static void pollCrsfMsp()
{
    uint8_t frame[copter::CrsfMsp::MAX_FRAME];
    uint8_t n;
    while ((n = rcReceiver.takeMsp(frame)) != 0)
    {
        if (!crsfMsp.feed(frame, n)) continue;
        msp::Writer w;
        const bool ok = copterConsole.mspHandler().process(crsfMsp.request(), w, millis());
        if (crsfMsp.wantsReply() && crsfTelemetryOn) crsfMsp.reply(ok, w.buf, w.length);
    }
}

static void sendCrsfMsp()
{
    uint8_t frame[copter::CrsfMsp::MAX_FRAME];
    const uint8_t n = crsfMsp.next(frame);
    if (n) board.rcUart().write(frame, n);
}

// Порт очков: ответить на их запросы (ARM, батарея, канал VTX, размер
// экрана), отдать готовые кадры OSD и новые настройки передатчику на MSP.
// Ничего не ждёт: что не влезло в UART — в следующий такт.
static void pollDevicePort(const copter::VtxRequest& vtxWanted)
{
    if (!devicePort.isOpen()) return;
    const uint32_t nowMs = millis();
    copterConsole.mspHandler().setMspVtx(mspVtx.online(), vtxWanted);
    devicePort.poll(nowMs, [nowMs, &vtxWanted](const msp::Parser& in, uint8_t* out, size_t capacity) {
        // передатчик на MSP узнаётся по тому, что спрашивает свои настройки
        if (in.cmd == copter::mspcmd::VTX_CONFIG || in.cmd == copter::mspcmd::SET_VTX_CONFIG)
        {
            mspVtx.polled();
            copterConsole.mspHandler().setMspVtx(true, vtxWanted);   // уже первый ответ — как передатчику
        }
        return copterConsole.answerDeviceMsp(in, out, capacity, nowMs);
    });
    if (mspVtx.changed(vtxWanted, static_cast<uint8_t>(copterParams.value(copter::PARAM_VTX_LOW_POWER_DISARM))))
    {
        uint8_t frame[48];
        const size_t n = copterConsole.mspHandler().vtxConfigFrame(frame, sizeof(frame));
        if (n) devicePort.queue(frame, n);
    }
    if (osdToDevice || displayPort.pending()) displayPort.drain(devicePort);
}

// ARM: с GPS — свой писк, есть ли фикс (ARMING_GPS_FIX / ARMING_GPS_NO_FIX), как в BF.
static copter::BeeperCondition armingBeep()
{
    if (!gps.isAvailable()) return copter::BeeperCondition::ARMING;
    return gpsHome.good(gpsHome.fix()) ? copter::BeeperCondition::ARMING_GPS_FIX : copter::BeeperCondition::ARMING_GPS_NO_FIX;
}

// Пищалка раз в 10 мс (copter/Beeper.h): ARM/DISARM/готов, батарея, связь,
// режимы BEEPER, BEEPER MUTE, GPS BEEP SATELLITE COUNT; что выключено — beeper_off_flags.
static void updateBeeper()
{
    if (Config::PIN_BUZZER < 0) return;
    const copter::CopterStatus& st = copterController.getStatus();
    const bool armed = copterController.isArmed();
    if (armed != wasArmedForBeeper) beeper.event(armed ? armingBeep() : copter::BeeperCondition::DISARMING);
    wasArmedForBeeper = armed;
    if (!readyBeeped && !armed && st.armBlockers == 0)
    {
        readyBeeped = true;
        beeper.event(copter::BeeperCondition::READY_BEEP);
    }
    copter::BeeperInputs in;
    // перегрузка по току (copter/EscGuard.h) пищит как LOW BATTERY
    in.batteryWarning = armed && (battery.alertState() == copter::BatteryState::WARNING || escGuard.state() != copter::Overload::NONE);
    in.batteryCritical = armed && battery.alertState() == copter::BatteryState::CRITICAL;
    in.rxLost = rcSeen && !rcLinkOk;
    in.rxLostLanding = st.failsafeStage == 2;
    in.crashRecovery = st.crashRecovery && copterController.getTune().pid.crashRecovery == copter::CrashRecovery::BEEP;
    in.crashFlip = st.turtle;
    in.findMe = rxInput.isActive(copter::BOX_BEEPER);
    in.muted = rxInput.isActive(copter::BOX_BEEPER_MUTE);
    in.gpsCount = rxInput.isActive(copter::BOX_GPS_BEEP_SATELLITE_COUNT);
    in.gpsFix = gpsHome.fix().valid;
    in.gpsSats = gpsHome.fix().satellites;
    in.disarmRepeat = stickCommands.disarmPosition();
    in.usbOnly = battery.state() == copter::BatteryState::NOT_PRESENT;
    writeBuzzer(beeper.tick(in));
}

// «Послушать» из приложения и события консоли (стёрт ящик).
static bool beepFromConsole(copter::BeeperCondition c, bool preview)
{
    if (Config::PIN_BUZZER < 0) return false;
    if (preview) return beeper.preview(c);
    beeper.event(c);
    return true;
}

// Нуль акселерометра (acc_calibration, copter/ImuAlign.h): стики, консоль 'a',
// MSP_ACC_CALIBRATION. Копится в задаче полёта (feedAccCalibration), итог — писком.
static copter::AccZeroCalibrator accZeroCal;
static copter::ImuAlign accCalAlign;   // поворот платы на время калибровки
static void startAccCalibration()
{
    if (copterController.isArmed()) return;
    copter::ImuAlignSettings a;
    a.alignRollDeg = static_cast<int16_t>(copterParams.value(copter::PARAM_ALIGN_BOARD_ROLL));
    a.alignPitchDeg = static_cast<int16_t>(copterParams.value(copter::PARAM_ALIGN_BOARD_PITCH));
    a.alignYawDeg = static_cast<int16_t>(copterParams.value(copter::PARAM_ALIGN_BOARD_YAW));
    accCalAlign.configure(a);
    accZeroCal.start();
}

static void feedAccCalibration(const copter::CopterSensors& sens, float dt)
{
    if (!accZeroCal.running()) return;
    using St = copter::AccZeroCalibrator::State;
    const St st = copterController.isArmed() ? St::FAILED : accZeroCal.feed(sens.accel, sens.gyro, dt, accCalAlign);
    if (st == St::RUNNING) return;
    if (st == St::DONE)
    {
        char text[24];
        copter::formatAccCalibration(accZeroCal.result(), true, text, sizeof(text));
        copterParams.setText(copter::PARAM_ACC_CALIBRATION, 0, text);
        applySettings();
        copterConsole.saveSettings();
        Serial.print("Нуль акселерометра: acc_calibration = ");
        Serial.println(text);
        beeper.event(copter::BeeperCondition::ACC_CALIBRATION);
    }
    else
    {
        Serial.println("Нуль акселерометра: не удалось — коптер двигали или стоит с наклоном больше 25°");
        beeper.event(copter::BeeperCondition::ACC_CALIBRATION_FAIL);
    }
    accZeroCal.clear();
}

// gyro_calib_duration / gyro_calib_noise_limit -> калибровка гироскопа (до 4 попыток,
// пока не перестанут трясти; порог — в отсчётах чипа на ±2000 °/с: 16.4 на °/с).
static void configureGyroCalibration()
{
    const int samples = static_cast<int>(copterParams.value(copter::PARAM_GYRO_CALIB_DURATION));
    const float noiseDps = copterParams.value(copter::PARAM_GYRO_CALIB_NOISE_LIMIT) / 16.4f;
    imuSensor.setGyroCalibration(samples, noiseDps, 4);
    if (HAS_GYRO2) gyro2Sensor.setGyroCalibration(samples, noiseDps, 4);
}

// Команда стиками (copter/StickCommands.h) — сделать; подтверждение — писком.
static void runStickCommand(const copter::StickCommand& c)
{
    using namespace copter;
    switch (c.action)
    {
        case StickAction::NONE: return;
        case StickAction::CALIBRATE_GYRO:
            imuSensor.calibrate();
            if (HAS_GYRO2 && gyro2Sensor.isAvailable()) gyro2Sensor.calibrate();
            copterController.gyroRecalibrated();
            if (baroSensor.isAvailable()) baroSensor.calibrateAltitude();
            beeper.event(BeeperCondition::GYRO_CALIBRATED);
            return;
        case StickAction::CALIBRATE_ACC:
            // нуль акселерометра (acc_calibration): рама стоит ровно ~0.5 с — как BF
            startAccCalibration();
            return;
        case StickAction::PID_PROFILE:
        case StickAction::RATE_PROFILE:
        {
            const bool rate = c.action == StickAction::RATE_PROFILE;
            if (!(rate ? copterParams.selectRateProfile(c.value) : copterParams.selectPidProfile(c.value))) return;
            copterConsole.saveSettings();
            beeper.event(BeeperCondition::MULTI_BEEPS, static_cast<uint8_t>(c.value + 1));
            return;
        }
        case StickAction::SAVE:
            copterConsole.saveSettings();
            beeper.event(BeeperCondition::MULTI_BEEPS, 1);
            return;
        case StickAction::TRIM:
        {
            const float roll = fminf(300.0f, fmaxf(-300.0f, copterParams.value(PARAM_ACC_TRIM_ROLL) + c.trimRoll));
            const float pitch = fminf(300.0f, fmaxf(-300.0f, copterParams.value(PARAM_ACC_TRIM_PITCH) + c.trimPitch));
            copterParams.setValue(PARAM_ACC_TRIM_ROLL, 0, roll);
            copterParams.setValue(PARAM_ACC_TRIM_PITCH, 0, pitch);
            copterConsole.saveSettings();
            beeper.event(BeeperCondition::MULTI_BEEPS, 1);
            return;
        }
    }
}

// Фильтр ориентации выучил сопротивление винтов (attitude_drag) — записать.
// Контроллер не пересчитывается: оценка в нём уже работает. Если у пилота
// есть несохранённые правки — в другой раз: чужое «save» за него не делаем.
static void saveLearnedDrag()
{
    const copter::AttitudeQuat& attitude = copterController.getAttitude();
    if (!attitude.dragLearned() || copterConsole.cliActive() || copterConsole.appUnsaved()) return;
    if (copterParams.learnDrag(attitude.dragCoef())) copterConsole.storeLearned();
}

// Стики до контроллера: ARM стиками (ARM нет в таблице), команды стиками,
// ARM после DShot-маяка — через 1.2 с (как в BF). Возвращает тумблер ARM
// пилота (до отсрочки) — маяк молчит, пока пилот армится.
static bool handleSticks(copter::CopterRc& rc)
{
    using namespace copter;
    const uint32_t now = micros();
    const float dt = lastStickUs ? fminf(0.1f, (now - lastStickUs) * 1e-6f) : 0.001f;
    lastStickUs = now;
    const bool armed = copterController.isArmed();
    // меню в очках: стики — его, команды стиками молчат
    switch (osdMenu.update(rc, armed, dt, copterParams))
    {
        case OsdMenu::Exit::SAVE:
            copterConsole.saveSettings();
            beeper.event(BeeperCondition::MULTI_BEEPS, 1);
            break;
        case OsdMenu::Exit::DISCARD: copterConsole.revertSettings(); break;
        case OsdMenu::Exit::NONE: break;
    }
    const bool disabled = rxInput.isActive(BOX_STICK_COMMANDS_DISABLE) || copterConsole.cliActive() || osdMenu.isOpen();
    const StickCommand cmd = stickCommands.update(rc, armed, copterController.getStatus().airborne,
                                                  rc.mode != CopterMode::ACRO, disabled, dt);
    const bool armSwitch = rc.armSwitch;
    if (!armed && dshotBeacon.holdArm(millis())) rc.armSwitch = false;
    runStickCommand(cmd);
    return armSwitch;
}

// Первый код BF причины, почему не армится (для OSD), или nullptr.
static const char* firstArmBlock()
{
    const uint32_t b = copterController.getStatus().armBlockers;
    for (uint32_t bit = 1; bit && bit <= copter::ARM_BLOCK_LAST; bit <<= 1)
    {
        if (b & bit) return copter::armBlockWord(bit);
    }
    return nullptr;
}

// Экран OSD из того, что знает прошивка (copter/Osd.h).
static void composeOsd()
{
    osdScreen.setCanvas(osdOutput.cols, osdOutput.rows);
    if (osdMenu.isOpen())
    {
        osdMenu.draw(osdScreen, copterParams);
        return;
    }
    if (rxInput.isActive(copter::BOX_OSD))   // режим OSD DISABLE — пусто
    {
        osdScreen.clear();
        return;
    }
    const copter::CopterStatus& s = copterController.getStatus();
    copter::OsdData d;
    d.armed = copterController.isArmed();
    d.failsafe = s.state == copter::CopterState::FAILSAFE;
    d.mode = s.turtle ? "TURT" : crsfModeName(d.armed ? s.mode : switchMode);
    d.rescueBlind = s.deadReckPhase != copter::DeadReckonPhase::IDLE;
    d.rescue = (s.rescuePhase != copter::RescuePhase::IDLE && s.rescuePhase != copter::RescuePhase::ABORT) || d.rescueBlind;
    d.rescueAbort = s.rescuePhase == copter::RescuePhase::ABORT && !d.rescueBlind;
    d.linkQuality = rcReceiver.getLinkQuality();
    d.rssi = rssi.percent();
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
    d.climbMs = s.climbRate;
    d.throttle = s.throttle;
    d.gpsPresent = gps.isAvailable();
    const copter::GpsFix& fix = gpsHome.fix();
    d.satellites = fix.satellites;
    d.gpsFix = fix.valid;
    d.speedKmh = fix.speedMs * 3.6f;
    const copter::GpsFix where = gpsHome.located();
    d.gpsLastKnown = !fix.valid && gpsHome.hasLastGood();
    d.latitude = where.latitude;
    d.longitude = where.longitude;
    d.home = gpsHome.hasHome();
    d.homeDistanceM = gpsHome.distanceM();
    d.homeRelativeDeg = gpsHome.relativeHomeDeg(headingDeg());
    d.vtxMw = vtx.state().online ? vtx.state().powerMw : 0;
    d.vtxPit = vtx.state().pit;
    d.armBlock = d.armed ? nullptr : firstArmBlock();
    d.craftName = copterParams.text(copter::PARAM_CRAFT_NAME, 0);
    d.blink = (millis() / 250) % 2 == 0;
    d.exitStats = sticksUp;
    d.overload = copter::overloadText(escGuard.state());
    d.posHold = copter::posHoldFailText(s.posHoldFail);
    d.gpsSpoof = s.gpsSpoof != copter::GpsSpoof::NONE;
    if (!d.overload && escGuard.limiting()) d.overload = "CURRENT LIMIT";
    d.training = copter::trainingText(s.trainingFlags);
    d.fence = copter::geofenceText(s.fenceFlags);
    d.fenceRadiusM = copterController.getGeofence().settings().circle() ? copterController.getGeofence().settings().radiusM : 0.0f;
    fillFenceOsd(d, headingDeg());
    vtxHeat.update(d.armed, vtx.state(), millis());
    d.vtxHot = vtxHeat.warn();
    d.escOverloads = escGuard.flightStats().escEvents;
    d.batteryOverloads = escGuard.flightStats().batteryEvents;
    d.rollDeg = s.roll;
    d.pitchDeg = s.pitch;
    d.onSeconds = millis() / 1000;
    {
        // обороты и температура регуляторов (dshot_bidir, dshot_edt)
        const copter::IMotorTelemetry& tel = motors.dshotOutput();
        const uint8_t poles = static_cast<uint8_t>(copterParams.value(copter::PARAM_MOTOR_POLES));
        const uint8_t n = tel.telemetryEnabled() ? tel.telemetryMotors() : 0;
        d.escRpmCount = n < copter::OsdData::ESC_MAX ? n : copter::OsdData::ESC_MAX;
        for (uint8_t i = 0; i < d.escRpmCount; ++i)
        {
            if (!tel.hasTelemetry(i))
            {
                d.escRpm[i] = copter::OsdData::NO_RPM;
                continue;
            }
            d.escRpm[i] = static_cast<uint32_t>(lroundf(copter::dshot::erpmToRpm(tel.erpm(i), poles)));
            const copter::EscExtTelemetry* x = tel.extended(i);
            if (!x || !x->has(copter::EscExtTelemetry::TEMPERATURE)) continue;
            if (!d.hasEscTemp || x->temperatureC > d.escTempC) d.escTempC = x->temperatureC;
            d.hasEscTemp = true;
        }
    }
    osdStats.update(d, millis());
    osdScreen.compose(d, copter::osdLayoutFrom(copterParams), &osdStats);
}

// Куда рисовать сейчас: настройки + чип на плате + что сказали очки
// (copter/DisplayPort.h, osdRoute). Итог — в osdOutput (экран, живой кадр).
static copter::OsdRoute routeOsd()
{
    copter::OsdRouteInput in = osdRouteIn;
    in.chipPresent = osdChip.isPresent();
    in.chipRows = osdChip.rows();
    in.portPresent = devicePort.isOpen();
    in.deviceSeen = devicePort.seen();
    in.deviceCols = copterConsole.mspHandler().deviceCanvasCols();
    in.deviceRows = copterConsole.mspHandler().deviceCanvasRows();
    const copter::OsdRoute route = copter::osdRoute(in);
    osdOutput.port = in.portPresent;
    osdOutput.chip = route.chip;
    osdOutput.msp = route.msp;
    osdOutput.cols = route.canvas.cols;
    osdOutput.rows = route.canvas.rows;
    osdOutput.hd = route.canvas.hd;
    osdOutput.deviceAlive = devicePort.alive(millis());
    osdOutput.deviceCanvas = route.msp && in.deviceCols && in.deviceRows;
    osdOutput.mspVtx = mspVtx.online();
    osdOutput.requests = devicePort.requests;
    copterConsole.mspHandler().setOsdCanvas(route.canvas.cols, route.canvas.rows);
    return route;
}

// Фоновая задача OSD: 20 раз в секунду экран — в чип (только изменения) и
// в очки по MSP (целиком, osd_framerate_hz раз в секунду); раз в секунду —
// проверить, не сменилась ли камера (PAL/NTSC).
static void osdTask(void*)
{
    uint32_t lastCheckMs = 0;
    for (;;)
    {
        const copter::OsdRoute route = routeOsd();
        composeOsd();
        if (route.chip) osdChip.flush(osdScreen, 60);
        else osdChip.blank();
        if (route.msp) displayPort.render(osdScreen, millis());
        else displayPort.release();   // экран отдан очкам (один раз, если до того рисовали)
        osdToDevice = route.msp;
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

// auto_profile_cell_count: подключили батарею — PID-профиль, у которого
// столько банок (как BF; только без ARM, в флеш не пишется).
static uint32_t batteryPlugsSeen = 0;
static void autoProfile()
{
    if (battery.plugEvents() == batteryPlugsSeen) return;
    batteryPlugsSeen = battery.plugEvents();
    if (copterController.isArmed()) return;
    const int p = copterParams.profileForCells(battery.detectedCells());
    if (p < 0 || p == copterParams.pidProfile()) return;
    if (copterParams.selectPidProfile(static_cast<uint8_t>(p))) copterController.applyProfile(copterParams.tune());
}

// Батарея и ток (раз в 10 мс): мВ на пинах АЦП -> copter/Battery.h (масштабы
// vbat_scale, ibata_scale, ibata_offset — настройки коптера).
// Ток регуляторов по EDT: наибольший на мотор и сумма — когда отвечают все
// моторы рамы. Заодно включает EDT у регуляторов (dshot_edt, моторы стоят).
struct EscCurrents
{
    bool all = false;
    float maxA = 0.0f;
    float sumA = 0.0f;
};

static EscCurrents escCurrents()
{
    EscCurrents c;
    const copter::IMotorTelemetry& tel = motors.dshotOutput();
    const uint8_t n = tel.telemetryMotors();
    bool replying = false, extended = true;
    uint8_t withCurrent = 0;
    for (uint8_t i = 0; tel.telemetryEnabled() && i < n; ++i)
    {
        if (!tel.hasTelemetry(i)) continue;
        replying = true;
        const copter::EscExtTelemetry* x = tel.extended(i);
        if (!x)
        {
            extended = false;
            continue;
        }
        if (!x->has(copter::EscExtTelemetry::CURRENT)) continue;
        ++withCurrent;
        c.sumA += x->amps;
        if (x->amps > c.maxA) c.maxA = x->amps;
    }
    c.all = n > 0 && withCurrent == n;
    const bool edt = bidirAtBoot && copterParams.value(copter::PARAM_DSHOT_EDT) > 0.5f && tel.telemetryEnabled();
    if (edtEnabler.update(millis(), edt, replying, replying && extended, !copterController.isArmed()))
    {
        motors.dshotOutput().queueCommand(copter::dshot::CommandQueue::ALL, copter::dshot::EXTENDED_TELEMETRY_ON);
    }
    return c;
}

static void readBattery()
{
    const EscCurrents esc = escCurrents();
    if (Config::PIN_VBAT_ADC < 0) return;
    auto pinMv = [](int16_t pin) {
        return pin < 0 ? 0.0f : static_cast<float>(analogRead(static_cast<uint32_t>(pin))) * 3300.0f / 4095.0f;
    };
    battery.update(pinMv(Config::PIN_VBAT_ADC), pinMv(Config::PIN_CURRENT_ADC), 0.01f);
    const bool armed = copterController.isArmed();
    if (armed && !wasArmedForGuard) escGuard.resetFlight();
    wasArmedForGuard = armed;
    // ток на мотор — по EDT, если отвечают все; нет датчика тока — сумма по EDT
    if (battery.hasCurrent())
        escGuard.update(armed, true, battery.amps(), 0.01f, esc.all ? esc.maxA : -1.0f);
    else
        escGuard.update(armed, esc.all, esc.sumA, 0.01f, esc.maxA);
    copterController.setOutputScale(escGuard.outputScale());
    autoProfile();
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
        if (HAS_GYRO2 && gyro2Sensor.isAvailable()) gyro2Sensor.update();
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
        pollCrsfMsp();
        copter::CopterRc rc = readRc();
        rssi.update(rcReceiver.getState().data(), Config::RC_CHANNELS, rcReceiver.getRssiDbm(), rcReceiver.getGoodFrameCount(),
                    rcReceiver.getBadFrameCount(), !rcReceiver.isSignalLost(), millis());
        rcLinkOk = rc.linkOk;
        rcSeen = rcSeen || rc.linkOk;
        const bool pilotArmSwitch = handleSticks(rc);
        switchMode = rc.mode;
        sticksUp = rc.linkOk && (rc.throttle > 0.9f || rc.pitch > 0.9f);
        t = lap(TASK_RX, t);
        if (millis() - lastGpsMs >= 5)
        {
            lastGpsMs = millis();
            gps.update();
            updateGps();
            updateCompass();
            t = lap(TASK_GPS, t);
        }

        copter::CopterSensors sens;
        {
            const copter::GyroSample g1 = gyroSampleOf(imuSensor.getImuData(), imuSensor.isAvailable());
            // «застывший» гироскоп ищем, только когда есть чем его заменить:
            // с одним чипом ложная тревога стоила бы больше, чем дала
            bool alive1 = g1.ok;
            copter::GyroSample g2;
            bool alive2 = false;
            if (gyro2Present)
            {
                g2 = gyroSampleOf(gyro2Sensor.getImuData(), gyro2Sensor.isAvailable());
                alive1 = gyroStuck[0].update(g1);
                alive2 = gyroStuck[1].update(g2);
            }
            gyroAlive = static_cast<uint8_t>((alive1 ? 1u : 0u) | (alive2 ? 2u : 0u));
            const copter::GyroAgree agree = alive1 && alive2 ? gyroAgreement.update(g1.gyro, g2.gyro) : gyroAgreement.state();
            gyroPick = copter::pickGyro(gyroToUse, g1, alive1, gyro2Present ? &g2 : nullptr, alive2, agree);
            if (gyroPick.fallback && !gyroFallbackSaid) Serial.println("Гироскоп: выбранный не отвечает — взят второй (gyro_to_use)");
            gyroFallbackSaid = gyroPick.fallback;
        }
        for (int i = 0; i < 3; ++i)
        {
            sens.gyro[i] = gyroPick.gyro[i];
            sens.accel[i] = gyroPick.accel[i];
        }
        sens.imuOk = gyroPick.ok;
        sens.baroOk = baroSensor.isAvailable();
        sens.baroAltM = baroSensor.getBarometerData().altitude;
        // компенсации просадки — быстрый фильтр (vbat_sag_lpf_period), а не показ
        sens.batteryVolts = battery.state() == copter::BatteryState::NOT_PRESENT ? 0.0f : battery.sagVolts();
        fillGps(sens.gps);
        // обороты по DShot (dshot_bidir) — RPM-фильтру; нет ответа хоть от одного мотора — по модели
        {
            const copter::IMotorTelemetry& tel = motors.dshotOutput();
            const uint8_t poles = static_cast<uint8_t>(copterParams.value(copter::PARAM_MOTOR_POLES));
            sens.motorHzOk = tel.telemetryEnabled() && tel.telemetryMotors() > 0;
            for (uint8_t i = 0; sens.motorHzOk && i < tel.telemetryMotors() && i < copter::MAX_MOTORS; ++i)
            {
                if (!tel.hasTelemetry(i)) sens.motorHzOk = false;
                else sens.motorHz[i] = copter::dshot::erpmToRpm(tel.erpm(i), poles) / 60.0f;
            }
        }
        copterController.setThrottleLimit(battery.throttleLimit());   // vbat_cutoff_percent

        // ARM запрещён, пока описание коптера не сошлось с платой,
        // установка IMU не откалибрована (если плата этого требует),
        // открыт CLI, в приложении несохранённое, нужна перезагрузка или
        // цикл не успевает
        uint32_t blockers = 0;
        if (!configOk) blockers |= copter::ARM_BLOCK_CONFIG;
        if (Config::IMU_MOUNT_CALIBRATION_REQUIRED && !imuSensor.isMountCalibrated()) blockers |= copter::ARM_BLOCK_NO_ACC_CAL;
        if (copterConsole.cliActive()) blockers |= copter::ARM_BLOCK_CLI;
        if (osdMenu.isOpen()) blockers |= copter::ARM_BLOCK_CMS;
        // приложение или конфигуратор BF (MSP) не сохранили настройки, конфигуратор запретил ARM
        if (copterConsole.appUnsaved() || copterConsole.mspBlocksArming(millis())) blockers |= copter::ARM_BLOCK_MSP;
        if (rebootRequired) blockers |= copter::ARM_BLOCK_REBOOT;
        if (copterConsole.cpuPercent() >= 100) blockers |= copter::ARM_BLOCK_LOAD;
        copterController.setExternalBlockers(blockers);

        if (copterConsole.mspHandler().takeAccCalibration()) startAccCalibration();
        feedAccCalibration(sens, 0.001f);   // задача — раз в 1 мс
        const copter::CopterOutputs out = copterController.update(sens, rc);
        t = lap(TASK_PID, t);
        // ARM — недосланные команды (пищалка, настройки) отменяются; turtle
        // mode — регуляторам «наоборот», и моторы стоят, пока команда не уйдёт
        if (copterController.isArmed() && !armedLastLoop) motors.dshotOutput().clearCommands();
        // DISARM после полёта: сопротивление винтов, которое фильтр ориентации
        // выучил по GPS, — в настройку (следующий полёт начнёт с него)
        if (!copterController.isArmed() && armedLastLoop) saveLearnedDrag();
        armedLastLoop = copterController.isArmed();
        motors.dshotOutput().setReversed(out.reversed);
        // DShot-маяк: нет связи / режим BEEPER — регуляторы пищат (без ARM)
        if (motors.dshotOutput().commandsAvailable())
        {
            const uint8_t beacon = dshotBeacon.update(millis(), copterController.isArmed(), pilotArmSwitch, rcSeen && !rc.linkOk,
                                                      rxInput.isActive(copter::BOX_BEEPER));
            if (beacon && !copterController.motorTestActive() && motors.dshotOutput().commandsFree() > 0)
                motors.dshotOutput().queueCommand(copter::dshot::CommandQueue::ALL, beacon);
        }
        motors.write(out.motor, out.spinning);
#if defined(OPENPLANE_SITL)
        sitlOutputs(out);
#endif
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
        const copter::VtxRequest vtxWanted = vtxRequest();
        vtx.request(vtxWanted);
        vtx.update(millis());
        pollDevicePort(vtxWanted);
        copterConsole.mspHandler().setVtx(&vtx.state(), vtx.levelCount());
        copterConsole.setOutputs(out);
        copterConsole.setHeading(headingDeg());
        copterConsole.update(millis());
        t = lap(TASK_CONSOLE, t);
        // ответ MSP пульту — вне очереди и чаще (20 мс), телеметрия — раз в 50 мс
        const bool mspReply = crsfTelemetryOn && crsfMsp.pending();
        if (crsfTelemetryOn && millis() - lastTelemetryMs >= (mspReply ? 20u : 50u))
        {
            lastTelemetryMs = millis();
            if (mspReply) sendCrsfMsp();
            else sendTelemetry();
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
// P63: роли UART из настроек serial_*_uart — до begin() приёмника, GPS и MSP.
// Карта не годится (UART нет на плате, две роли на одном) — как в прошивке платы.
static copter::BoardUarts boardUarts()
{
    copter::BoardUarts b;
    for (uint8_t n = 1; n <= copter::MAX_UARTS; ++n)
    {
        b.rx[n] = Config::PORT_UART_RX[n] >= 0;
        b.tx[n] = Config::PORT_UART_TX[n] >= 0;
    }
    return b;
}

static void setupPorts()
{
    const copter::BoardUarts uarts = boardUarts();
    copterConsole.setBoardUarts(uarts);
    if (!uarts.any()) return;
    copter::PortMap m = copter::portMap(copterParams);
    char error[96];
    if (!copter::checkPortMap(m, uarts, error, sizeof(error)))
    {
        Serial.printf("Порты: %s — беру как в прошивке платы.\n", error);
        m = copter::portMap(copterParams, true);
    }
    const auto pins = [&](copter::PortRole r, int16_t out[2]) {
        const uint8_t n = m[r];
        out[0] = n ? Config::PORT_UART_RX[n] : -1;
        out[1] = n ? Config::PORT_UART_TX[n] : -1;
    };
    int16_t rc[2], gpsPins[2], msp[2];
    pins(copter::PortRole::RX, rc);
    pins(copter::PortRole::GPS, gpsPins);
    pins(copter::PortRole::MSP, msp);
    board.rebindUarts(rc, gpsPins, msp);
    const uint8_t vtxUart = m[copter::PortRole::VTX];
    vtxPort.rebind(vtxUart ? Config::PORT_UART_TX[vtxUart] : -1);
    gpsPortOn = gpsPins[0] >= 0;
    mspPortOn = msp[1] >= 0;
    Serial.printf("Порты: приёмник UART%u, GPS UART%u, MSP UART%u, ESC UART%u, VTX UART%u (0 — нет; serial — как в BF)\n",
                  m[copter::PortRole::RX], m[copter::PortRole::GPS], m[copter::PortRole::MSP],
                  m[copter::PortRole::ESC_SENSOR], vtxUart);
}

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
    copterParams.setTableDefault(copter::PARAM_BEEPER_INVERSION, Config::BOARD_BEEPER_ACTIVE_HIGH ? 1.0f : 0.0f);
    copterParams.setTableDefault(copter::PARAM_VBAT_SCALE, roundf(Config::BLACKBOX_VBAT_DIVIDER * 10.0f));
    copterParams.setTableDefault(copter::PARAM_CURRENT_METER, Config::PIN_CURRENT_ADC >= 0 ? 1.0f : 0.0f);
    copterParams.setTableDefault(copter::PARAM_BATTERY_METER, Config::PIN_VBAT_ADC >= 0 ? 1.0f : 0.0f);
#if defined(OPB_VBAT_DIVIDER)   // делитель из дампа BF (tools/bf_board.py)
    copterParams.setTableDefault(copter::PARAM_VBAT_SCALE, OPB_VBAT_SCALE);
    copterParams.setTableDefault(copter::PARAM_VBAT_DIVIDER, OPB_VBAT_DIVIDER);
    copterParams.setTableDefault(copter::PARAM_VBAT_MULTIPLIER, OPB_VBAT_MULTIPLIER);
#endif
    copterParams.setTableDefault(copter::PARAM_IBATA_SCALE, Config::BOARD_IBATA_SCALE);
#if defined(COPTER_BLACKBOX_FLASH)
    copterParams.setTableDefault(copter::PARAM_BLACKBOX_DEVICE, copter::BLACKBOX_SPIFLASH);
    copterParams.setTableDefault(copter::PARAM_BLACKBOX_SAMPLE_RATE, 1);   // 1/2: 16 МБ флеша — ~8 минут
#else
    copterParams.setTableDefault(copter::PARAM_BLACKBOX_DEVICE, copter::BLACKBOX_SDCARD);
#if defined(COPTER_BLACKBOX_SD_SPI)
    copterParams.setTableDefault(copter::PARAM_BLACKBOX_SAMPLE_RATE, 1);   // 1/2: SPI у F405 медленнее SDMMC
#endif
#endif
    copterParams.setTableDefault(copter::PARAM_IBATA_OFFSET, Config::BOARD_IBATA_OFFSET_MV);
    // провод VTX теперь можно назначить на любой UART (P63): без него протокол и так NONE
    copterParams.setTableDefault(copter::PARAM_VTX_PROTOCOL, static_cast<float>(copter::VtxProtocol::AUTO));
    for (uint8_t i = 0; i < copter::PORT_ROLES; ++i)
        copterParams.setTableDefault(copter::portParam(static_cast<copter::PortRole>(i)), Config::PORT_DEFAULT[i]);
    copter::AuxRange fallback[copter::RcConfig::AUX_SLOTS];
    copterParams.rc().setFallback(fallback, CopterConfig::defaultAux(fallback, copter::RcConfig::AUX_SLOTS));
    copterParams.begin(copterBuild, CopterConfig::LOOP_HZ, CopterConfig::ESC_PROTOCOL, CopterConfig::applyOverrides);
    copterParams.load(store);   // описание во флеше другое — модель заново
    copterConsole.setStore(&store);
    const copter::CopterTune tune = copterParams.tune();
    copter::CopterSettings settings = CopterConfig::settings();
    copterParams.applyTo(settings);
    settings.turtleAvailable = copter::isDshot(CopterConfig::ESC_PROTOCOL);   // реверс — командой DShot
    copterController.configure(tune, settings);
    copterBuild = copterParams.buildSpec();

    if (tune.frame.count > CopterConfig::MOTOR_PIN_COUNT)
    {
        Serial.printf("ОШИБКА: у рамы %s %u моторов, а выходов на плате %u — ARM запрещён.\n",
                      copter::frameName(tune.frame.type), tune.frame.count, CopterConfig::MOTOR_PIN_COUNT);
        configOk = false;
    }
    const bool bidir = copterParams.value(copter::PARAM_DSHOT_BIDIR) > 0.5f;
    motors.dshotOutput().setBidirectional(bidir && copter::isDshot(CopterConfig::ESC_PROTOCOL));
    bidirAtBoot = bidir;
    if (!motors.begin(CopterConfig::MOTOR_PINS, tune.frame.count, CopterConfig::ESC_PROTOCOL, CopterConfig::MOTOR_AF,
                      CopterConfig::MOTOR_AF_COUNT))
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
    if (copter::isDshot(CopterConfig::ESC_PROTOCOL) && tune.loopHz > copter::dshot::maxLoopHz(CopterConfig::ESC_PROTOCOL, bidir))
    {
        Serial.printf("ОШИБКА: %s%s не успевает за циклом %.0f Гц (до %lu) — ARM запрещён.\n",
                      copter::escProtocolName(CopterConfig::ESC_PROTOCOL), bidir ? " с dshot_bidir" : "", tune.loopHz,
                      static_cast<unsigned long>(copter::dshot::maxLoopHz(CopterConfig::ESC_PROTOCOL, bidir)));
        configOk = false;
    }
    copterConsole.setMotorTelemetry(&motors.dshotOutput());
    if (copter::isDshot(CopterConfig::ESC_PROTOCOL)) copterConsole.setEscCommands(&motors.dshotOutput());
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


#if defined(COPTER_BLACKBOX_SD)
// SD-карта и файл чёрного ящика (BLACKBOX.BIN, tools/blackbox.py sd-prepare):
// что нашлось — в консоль; места нет — ящик выключен, полёту это не мешает.
static void setupBlackBox()
{
    Serial.print("SD-карта: ");
#if defined(COPTER_BLACKBOX_SD_SPI)
    if (Config::BOARD_SDCARD_SHARES_OSD) osdAccess.begin();
    else sdBus.begin();
#endif
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
    copterBlackBox.setGuard(&escGuard);
    copterBlackBox.setGps(&gps, &gpsHome);
    copterBlackBox.setSensors(&imuSensor, &baroSensor);
#if defined(COPTER_BLACKBOX_SD_SPI) && defined(ARDUINO_ARCH_STM32)
    // F405: очередь — в CCM, как у SPI-флеша (в основных 128 КБ места нет)
    copterBlackBox.begin(true, reinterpret_cast<uint8_t*>(Config::F405_CCM_BASE), Config::BLACKBOX_RING_F405_BYTES);
#elif defined(COPTER_BLACKBOX_SD_SPI)
    copterBlackBox.begin(true, nullptr, Config::BLACKBOX_RING_F405_BYTES);
#else
    copterBlackBox.begin();
#endif
    copterConsole.setRecorder(&copterBlackBox);   // и без карты: "bb list" ответит state=off
}
#endif

#if defined(COPTER_BLACKBOX_FLASH)
// SPI-флеш ящика (F405): опознать по JEDEC ID; нет — ящик выключен, полёту
// это не мешает. Очередь записей — в CCM RAM (64 КБ, ядро STM32duino её не
// берёт; DMA туда не ходит, а очереди и не нужно), запас стёртого — половина
// флеша (на 16 МБ это ~4 минуты при 1/1, ~8 при 1/2).
static void setupBlackBox()
{
    Serial.print("SPI-флеш ящика: ");
    if (Config::BOARD_FLASH_CS < 0)
    {
        Serial.println("нет на плате");
    }
    else if (Config::BOARD_FLASH_SHARES_GYRO)
    {
        Serial.println("на одной шине с гироскопом — ящик выключен");
    }
    else
    {
        if (Config::BOARD_FLASH_SHARES_OSD) osdAccess.begin();
        else flashOwnBus.begin();
        if (blackBoxFlash.begin())
        {
            Serial.printf("%s, JEDEC 0x%06lX, %lu МБ%s\n", blackBoxFlash.vendor(), static_cast<unsigned long>(blackBoxFlash.jedecId()),
                          static_cast<unsigned long>(blackBoxFlash.size() >> 20), Config::BOARD_FLASH_SHARES_OSD ? " (шина — с OSD)" : "");
        }
        else
        {
            Serial.printf("не отвечает (JEDEC 0x%06lX)\n", static_cast<unsigned long>(blackBoxFlash.jedecId()));
        }
    }
    copterBlackBox.setMinFreeBytes(blackBoxFlash.size() / 2);
    copterBlackBox.setBattery(&battery);
    copterBlackBox.setGuard(&escGuard);
    copterBlackBox.setGps(&gps, &gpsHome);
    copterBlackBox.setSensors(&imuSensor, &baroSensor);
#if defined(ARDUINO_ARCH_STM32)
    copterBlackBox.begin(true, reinterpret_cast<uint8_t*>(Config::F405_CCM_BASE), Config::BLACKBOX_RING_F405_BYTES);   // CCM
#else
    copterBlackBox.begin(true, nullptr, Config::BLACKBOX_RING_F405_BYTES);
#endif
    copterConsole.setRecorder(&copterBlackBox);
    copterConsole.setFlashChip(&blackBoxFlash);
}
#endif

// MSP для конфигуратора Betaflight (telemetry/CopterMsp.h): что за плата и датчики.
static void setupMsp()
{
    copter::MspBoard b;
    b.boardName = OP_BOARD_ENV;
#if defined(BOARD_STM32F405)
    b.target = "OPENPLANE_F405";
    b.mcuType = 1;
#else
    b.target = "OPENPLANE_H743";
    b.mcuType = 12;
#endif
    b.commit = OP_SOURCE_COMMIT;
#if defined(ARDUINO_ARCH_STM32)
    b.uid[0] = HAL_GetUIDw0();
    b.uid[1] = HAL_GetUIDw1();
    b.uid[2] = HAL_GetUIDw2();
#endif
    uint16_t sensors = 0;
    if (imuSensor.isAvailable()) sensors |= 1u | (1u << 5);   // ACC, GYRO
    if (baroSensor.isAvailable()) sensors |= 1u << 1;
    if (compass) sensors |= 1u << 2;
    if (gps.isAvailable()) sensors |= 1u << 3;
    b.sensors = sensors;
    b.gps = gps.isAvailable();
    b.osd = Config::BOARD_OSD_CS >= 0;
    b.rcChannels = Config::RC_CHANNELS;
    copterConsole.mspHandler().setBoard(b);
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
    setupPorts();

    configureGyroCalibration();
    if (imuSensor.begin()) imuSensor.calibrate();
    if (HAS_GYRO2)
    {
        gyro2Bus.begin();
        gyro2Present = gyro2Sensor.begin();
        if (gyro2Present)
        {
            gyro2Sensor.calibrate();
            copterConsole.setGyros(&gyroPick, &gyroAlive, &gyroAgreement, &gyro2Sensor);
        }
        else Serial.println("Второй гироскоп (SPI4) не ответил — у SLIM V3/WING его и нет");
    }
    if (baroSensor.begin()) baroSensor.calibrateAltitude();
    copterConsole.setBaro(&baroSensor);
    if (gpsPortOn) gps.begin();
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
    copterConsole.setRebootHook([]() { NVIC_SystemReset(); });
#endif
    copterConsole.setApplyHook(applySettings);
    copterConsole.setAccCalibrationHook(startAccCalibration);
    copterConsole.setBeepHook(beepFromConsole);
    copterConsole.setRxInput(&rxInput);
    copterConsole.setBattery(&battery);
    copterConsole.setGuard(&escGuard);
    copterConsole.setRssi(&rssi);
    copterConsole.setVtx(&vtx);
    copterConsole.setGps(&gps, &gpsHome);
    copterConsole.setCompass(compass, &copterCompass);
    setupMsp();
    // CS всех чипов на SPI OSD — вверх до первого обмена: карта или флеш с
    // неподнятым CS (вход после сброса) приняли бы байты OSD (как spiPreinit у BF).
#if defined(COPTER_BLACKBOX_SD_SPI)
    if (Config::BOARD_SDCARD_SHARES_OSD && Config::BOARD_SDCARD_CS >= 0)
    {
        pinMode(static_cast<uint32_t>(Config::BOARD_SDCARD_CS), OUTPUT);
        digitalWrite(static_cast<uint32_t>(Config::BOARD_SDCARD_CS), HIGH);
    }
#elif defined(COPTER_BLACKBOX_FLASH)
    if (Config::BOARD_FLASH_SHARES_OSD && Config::BOARD_FLASH_CS >= 0)
    {
        pinMode(static_cast<uint32_t>(Config::BOARD_FLASH_CS), OUTPUT);
        digitalWrite(static_cast<uint32_t>(Config::BOARD_FLASH_CS), HIGH);
    }
#endif
    if (Config::BOARD_OSD_CS >= 0)
    {
        osdAccess.begin();
        Serial.printf("OSD: %s\n", osdChip.begin() ? (osdChip.isPal() ? "AT7456E/MAX7456, PAL" : "AT7456E/MAX7456, NTSC")
                                                     : "чип не отвечает");
    }
    // порт очков: слушаем всегда (на опрос MSP отвечаем), OSD шлём по osd_displayport_device
    if (mspPortOn)
    {
        devicePort.begin();
        Serial.println("Порт MSP для очков и видеопередатчика: 115200");
    }
    copterConsole.setOsdOutput(&osdOutput);
    copterConsole.setTaskStats(&taskStats);
#if defined(COPTER_BLACKBOX)
    setupBlackBox();   // последним: задача записи начинает готовить место
#endif
    copterConsole.printHint();
    beeper.event(copter::BeeperCondition::GYRO_CALIBRATED);   // гироскоп откалиброван при включении
    beeper.event(copter::BeeperCondition::SYSTEM_INIT);

    // Стеки задач берутся из остатка ОЗУ: не хватило — сказать прямо, а не молчать
    // (на F405 его мало; без задачи OSD нет экрана, без flight не летает ничего).
#if defined(BOARD_STM32F405)
    // F405: после статики в основных 128 КБ остаётся ~19 КБ кучи — стекам
    // 16 + 2 + 3 КБ мало. Стеки — в CCM за очередью ящика (DMA их не касается).
    Rtos::StackArena ccm(Config::F405_CCM_STACKS_BASE, Config::F405_CCM_STACKS_BYTES);
    auto start = [&ccm](TaskFunction_t fn, const char* name, uint32_t bytes, UBaseType_t prio) {
        StackType_t* stack = ccm.take(bytes);
        return stack ? Rtos::startTaskStatic(fn, name, stack, bytes, nullptr, prio) : Rtos::startTask(fn, name, bytes, nullptr, prio);
    };
#else
    auto start = [](TaskFunction_t fn, const char* name, uint32_t bytes, UBaseType_t prio) {
        return Rtos::startTask(fn, name, bytes, nullptr, prio);
    };
#endif
    bool tasksOk = start(flightTask, "flight", 16384, Rtos::PRIORITY_FLIGHT);
    tasksOk = start(storageTask, "storage", 2048, Rtos::PRIORITY_BACKGROUND) && tasksOk;
    if (osdChip.isPresent() || devicePort.isOpen())
    {
        tasksOk = start(osdTask, "osd", 3072, Rtos::PRIORITY_BACKGROUND) && tasksOk;
    }
    if (!tasksOk) Serial.println("ОШИБКА: не хватило памяти на задачи (flight / storage / osd) — прошивка работает не полностью");
    vTaskStartScheduler();   // не возвращается

    Serial.println("FreeRTOS не запустился (мало памяти?)");
}

// Не вызывается: после vTaskStartScheduler() работают только задачи.
// cppcheck-suppress unusedFunction
void loop()
{
}

#endif   // VEHICLE_MULTIROTOR
