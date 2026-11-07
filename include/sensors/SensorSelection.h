#pragma once

// ============================================================
// 🔧 ВЫБОР ДАТЧИКОВ ПРОЕКТА
//
// Единственное место, которое нужно менять, чтобы сменить
// физический датчик. Проще всего — одной строкой выбрать готовый
// набор (SENSOR_KIT ниже); можно и каждый датчик по отдельности.
// main.cpp работает через SelectedImu/SelectedBaro/SelectedMag/
// SelectedGps/SelectedPitotBaro и не знает, какой класс за ними стоит.
//
// Несколько чипов одного типа: SENSOR_IMU — основной, SENSOR_IMU_2 и
// SENSOR_IMU_3 — запасные (так же у барометра и компаса). Прошивка
// опознаёт их по порядку и берёт первый ответивший
// (sensors/SensorChain.h) — одна прошивка летает с любым из выбранных.
//
// Шина: датчик на I2C ищется на ВСЕХ шинах платы (IBoard::i2cBuses():
// основная PIN_I2C_*, вторая PIN_I2C2_*, внешний разъём) — какой модуль
// на какую шину повесить, всё равно; пины шин — в Config.h. Для SPI
// задан CS-пин (Config::PIN_SPI_CS_*). Устройство создаёт
// <Тип>Chip<id>::device(board) (I2cRegisterDevice или SpiRegisterDevice,
// см. hal/RegisterDevice.h). Драйверы шины не различают — один и тот же
// BMP581_Sensor работает и по I2C, и по SPI.
//
// ПОДДЕРЖИВАЕМЫЕ ДАТЧИКИ:
//   IMU (гироскоп + акселерометр):
//     • MPU6050 / MPU6500 (GY-521) — I2C 0x68; чип по WHO_AM_I
//     • ICM42688 ("601N1")        — SPI, CS = PIN_SPI_CS_IMU
//     • LSM6DSV / 16X / 32X       — I2C 0x6A или 0x6B (сам находит), или SPI
//     • ICM-45686                 — I2C 0x68 или 0x69 (сам находит), или SPI
//   Барометр:
//     • BMP388                    — I2C 0x76 или SPI
//     • BME280 / BMP280           — I2C 0x76
//     • SPL06-001                 — I2C 0x76 или 0x77 (сам находит), или SPI
//     • BMP581                    — I2C 0x46 (0x47 — если нет трубки Пито), или SPI
//     • DPS310                    — I2C 0x76 или 0x77 (сам находит); на плате Matek H743 — 0x76
//   Компас и трубка Пито ищутся сначала на шине внешнего разъёма
//   (board.externalI2c(): у самодельных плат это шина датчиков, у
//   Matek H743 — I2C1), потом на остальных.
//   Магнитометр:
//     • QMC5883P (GY-273)         — I2C 0x2C
//     • QMC5883L                  — I2C 0x0D
//     • QMC6309                   — I2C 0x7C (модули с LSM6DSV / ICM-45686)
//     • нет
//   Воздушная скорость:
//     • самодельная трубка Пито: BMP581 в трубке (I2C 0x47, SDO = VDD)
//       + основной барометр в фюзеляже как статика
//       (sensors/airspeed/PitotDualBaroAirspeed.h)
//     • нет
//   GPS:
//     • u-blox M10 (UBX)           — UART
//     • нет
//   Камера (только STM32H743 DevEBox: разъём DCMI FPC-24):
//     • OV5640                     — DCMI, настройка по I2C датчиков 0x3C; кадры JPEG в чёрный ящик
//     • нет
// ============================================================

// Нужны тем, кто раскрывает SELECTED_*_DEVICE(board): макросы
// ссылаются на I2cRegisterDevice/SpiRegisterDevice и пины Config.
#include "config/Config.h"          // IWYU pragma: export
#include "hal/RegisterDevice.h"     // IWYU pragma: export

#define SENSOR_IMU_NONE          0   // только для запасных SENSOR_IMU_2/3
#define SENSOR_IMU_MPU6050       1
#define SENSOR_IMU_ICM42688      2   // SPI
#define SENSOR_IMU_LSM6DSV       3   // I2C
#define SENSOR_IMU_LSM6DSV_SPI   4
#define SENSOR_IMU_ICM45686      5   // I2C
#define SENSOR_IMU_ICM45686_SPI  6

#define SENSOR_BARO_NONE        0
#define SENSOR_BARO_BME280      1
#define SENSOR_BARO_BMP388      2   // SPI
#define SENSOR_BARO_BMP388_I2C  3
#define SENSOR_BARO_SPL06       4   // I2C
#define SENSOR_BARO_SPL06_SPI   5
#define SENSOR_BARO_BMP581      6   // I2C
#define SENSOR_BARO_BMP581_SPI  7
#define SENSOR_BARO_DPS310      8   // I2C

#define SENSOR_MAG_NONE      0
#define SENSOR_MAG_QMC5883P  1
#define SENSOR_MAG_QMC5883L  2
#define SENSOR_MAG_QMC6309   3

#define SENSOR_AIRSPEED_NONE          0
#define SENSOR_AIRSPEED_PITOT_BMP581  1

#define SENSOR_GPS_NONE      0
#define SENSOR_GPS_UBLOX_M10 1

#define SENSOR_CAMERA_NONE   0
#define SENSOR_CAMERA_OV5640 1   // DCMI

// Готовые наборы: SENSOR_KIT задаёт все датчики разом.
#define SENSOR_KIT_CUSTOM          0   // каждый датчик — своей строкой ниже
#define SENSOR_KIT_BENCH_GY521     1   // стенд: GY-521 (MPU6500), BMP581 I2C, GY-273 (QMC5883P)
#define SENSOR_KIT_LSM6DSV_PITOT   2   // LSM6DSV+QMC6309, SPL06 в фюзеляже, BMP581 в трубке Пито, GPS
#define SENSOR_KIT_ICM45686_PITOT  3   // ICM-45686+QMC6309, SPL06, BMP581 в трубке, GPS
#define SENSOR_KIT_MATEK_H743      4   // датчики на плате Matek H743: ICM-42688-P (SPI), DPS310, GPS
#define SENSOR_KIT_BF_BOARD        5   // полётник под Betaflight (config/boards/*.h): ICM-42688-P (SPI), барометр платы, GPS


// --------------------------------------------------------
// АКТИВНЫЙ ВЫБОР — одна строка SENSOR_KIT...
//
// Любую строку можно переопределить флагом сборки без правки файла,
// например build_flags = -D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT. Так
// сделано у полётников с датчиками на плате: env matek-h743 сам ставит
// SENSOR_KIT_MATEK_H743, строка ниже для него не действует.
// --------------------------------------------------------

#ifndef SENSOR_KIT
#define SENSOR_KIT SENSOR_KIT_BENCH_GY521
#endif

#if SENSOR_KIT == SENSOR_KIT_BENCH_GY521
    #define KIT_IMU      SENSOR_IMU_MPU6050
    #define KIT_BARO     SENSOR_BARO_BMP581
    #define KIT_MAG      SENSOR_MAG_QMC5883P
    #define KIT_AIRSPEED SENSOR_AIRSPEED_NONE
    #define KIT_GPS      SENSOR_GPS_NONE
#elif SENSOR_KIT == SENSOR_KIT_LSM6DSV_PITOT
    #define KIT_IMU      SENSOR_IMU_LSM6DSV
    #define KIT_BARO     SENSOR_BARO_SPL06
    #define KIT_MAG      SENSOR_MAG_QMC6309
    #define KIT_AIRSPEED SENSOR_AIRSPEED_PITOT_BMP581
    #define KIT_GPS      SENSOR_GPS_UBLOX_M10
#elif SENSOR_KIT == SENSOR_KIT_ICM45686_PITOT
    #define KIT_IMU      SENSOR_IMU_ICM45686
    #define KIT_BARO     SENSOR_BARO_SPL06
    #define KIT_MAG      SENSOR_MAG_QMC6309
    #define KIT_AIRSPEED SENSOR_AIRSPEED_PITOT_BMP581
    #define KIT_GPS      SENSOR_GPS_UBLOX_M10
#elif SENSOR_KIT == SENSOR_KIT_MATEK_H743
    #define KIT_IMU      SENSOR_IMU_ICM42688
    #define KIT_BARO     SENSOR_BARO_DPS310
    #define KIT_MAG      SENSOR_MAG_NONE
    #define KIT_AIRSPEED SENSOR_AIRSPEED_NONE
    #define KIT_GPS      SENSOR_GPS_UBLOX_M10
#elif SENSOR_KIT == SENSOR_KIT_BF_BOARD
    // Тип барометра — в заголовке платы (OPB_BARO_*, флаг tools/bf_board.py --baro).
    #define KIT_IMU      SENSOR_IMU_ICM42688
    #if defined(OPB_BARO_DPS310)
        #define KIT_BARO SENSOR_BARO_DPS310
    #else
        #define KIT_BARO SENSOR_BARO_SPL06
    #endif
    #define KIT_MAG      SENSOR_MAG_NONE
    #define KIT_AIRSPEED SENSOR_AIRSPEED_NONE
    #define KIT_GPS      SENSOR_GPS_UBLOX_M10
#elif SENSOR_KIT != SENSOR_KIT_CUSTOM
    #error "SensorSelection.h: неизвестный SENSOR_KIT"
#endif

// ...или каждый датчик отдельно (SENSOR_KIT_CUSTOM или поверх набора).

#ifndef SENSOR_IMU
#define SENSOR_IMU  KIT_IMU
#endif

#ifndef SENSOR_BARO
#define SENSOR_BARO KIT_BARO
#endif

#ifndef SENSOR_MAG
#define SENSOR_MAG  KIT_MAG
#endif

#ifndef SENSOR_AIRSPEED
#define SENSOR_AIRSPEED KIT_AIRSPEED
#endif

#ifndef SENSOR_GPS
#define SENSOR_GPS  KIT_GPS
#endif

// Запасные чипы (SensorChain): опрашиваются, если основной не ответил.
#ifndef SENSOR_IMU_2
#define SENSOR_IMU_2 SENSOR_IMU_NONE
#endif
#ifndef SENSOR_IMU_3
#define SENSOR_IMU_3 SENSOR_IMU_NONE
#endif
#ifndef SENSOR_BARO_2
#define SENSOR_BARO_2 SENSOR_BARO_NONE
#endif
#ifndef SENSOR_BARO_3
#define SENSOR_BARO_3 SENSOR_BARO_NONE
#endif
#ifndef SENSOR_MAG_2
#define SENSOR_MAG_2 SENSOR_MAG_NONE
#endif
#ifndef SENSOR_MAG_3
#define SENSOR_MAG_3 SENSOR_MAG_NONE
#endif

// Камеры в наборах нет: ставится этой строкой (или флагом сборки).
#ifndef SENSOR_CAMERA
#define SENSOR_CAMERA SENSOR_CAMERA_NONE
#endif
#if SENSOR_CAMERA != SENSOR_CAMERA_NONE && !defined(BOARD_STM32H743)
#error "SensorSelection.h: камера OV5640 — только на STM32H743 (разъём DCMI DevEBox)"
#endif


// ============================================================
// РАЗРЕШЕНИЕ В КОНКРЕТНЫЕ ТИПЫ (менять не нужно, кроме адресов/пинов)
//
// ImuChip<id>, BaroChip<id>, MagChip<id>: класс драйвера (Type) и где
// искать чип (device(board)). Неизвестный id — ошибка сборки.
// Подключены все драйверы: класс, который не выбран, в прошивку не
// попадает (всё в заголовках, код — только у используемых).
// ============================================================

#include <utility>

#include "sensors/SensorChain.h"
#include "sensors/baro/BME280_Sensor.h"
#include "sensors/baro/BMP388_Sensor.h"
#include "sensors/baro/BMP581_Sensor.h"
#include "sensors/baro/DPS310_Sensor.h"
#include "sensors/baro/SPL06_Sensor.h"
#include "sensors/imu/ICM42688_Sensor.h"
#include "sensors/imu/ICM45686_Sensor.h"
#include "sensors/imu/LSM6DSV_Sensor.h"
#include "sensors/imu/MPU6050_Sensor.h"
#include "sensors/mag/QMC5883L_Sensor.h"
#include "sensors/mag/QMC5883P_Sensor.h"
#include "sensors/mag/QMC6309_Sensor.h"

template <int Id> struct ImuChip;
template <int Id> struct BaroChip;
template <int Id> struct MagChip;

// IMU и барометр — сначала на шине датчиков, компас и трубка — сначала
// на шине внешнего разъёма; потом на всех остальных шинах платы.
#define OP_SENSOR_BUSES(board) (board).i2cBuses((board).i2c())
#define OP_EXTERNAL_BUSES(board) (board).i2cBuses((board).externalI2c())

template <> struct ImuChip<SENSOR_IMU_MPU6050>
{
    using Type = MPU6050_Sensor;
    template <class B> static I2cRegisterDevice device(B& b) { return I2cRegisterDevice(OP_SENSOR_BUSES(b), 0x68); }
};
template <> struct ImuChip<SENSOR_IMU_ICM42688>
{
    using Type = ICM42688_Sensor;
    template <class B> static SpiRegisterDevice device(B& b) { return ICM42688_Sensor::spiDevice(b.spi(), Config::PIN_SPI_CS_IMU); }
};
template <> struct ImuChip<SENSOR_IMU_LSM6DSV>
{
    using Type = LSM6DSV_Sensor;
    template <class B> static I2cRegisterDevice device(B& b)
    {
        return I2cRegisterDevice(OP_SENSOR_BUSES(b), LSM6DSV_Sensor::DEFAULT_ADDRESS, LSM6DSV_Sensor::ALTERNATE_ADDRESS);
    }
};
template <> struct ImuChip<SENSOR_IMU_LSM6DSV_SPI>
{
    using Type = LSM6DSV_Sensor;
    template <class B> static SpiRegisterDevice device(B& b) { return LSM6DSV_Sensor::spiDevice(b.spi(), Config::PIN_SPI_CS_IMU); }
};
template <> struct ImuChip<SENSOR_IMU_ICM45686>
{
    using Type = ICM45686_Sensor;
    template <class B> static I2cRegisterDevice device(B& b)
    {
        return I2cRegisterDevice(OP_SENSOR_BUSES(b), ICM45686_Sensor::DEFAULT_ADDRESS, ICM45686_Sensor::ALTERNATE_ADDRESS);
    }
};
template <> struct ImuChip<SENSOR_IMU_ICM45686_SPI>
{
    using Type = ICM45686_Sensor;
    template <class B> static SpiRegisterDevice device(B& b) { return ICM45686_Sensor::spiDevice(b.spi(), Config::PIN_SPI_CS_IMU); }
};

template <> struct BaroChip<SENSOR_BARO_BME280>
{
    using Type = BME280_Sensor;
    template <class B> static I2cRegisterDevice device(B& b) { return I2cRegisterDevice(OP_SENSOR_BUSES(b), 0x76); }
};
template <> struct BaroChip<SENSOR_BARO_BMP388>
{
    using Type = BMP388_Sensor;
    template <class B> static SpiRegisterDevice device(B& b) { return BMP388_Sensor::spiDevice(b.spi(), Config::PIN_SPI_CS_BARO); }
};
template <> struct BaroChip<SENSOR_BARO_BMP388_I2C>
{
    using Type = BMP388_Sensor;
    template <class B> static I2cRegisterDevice device(B& b) { return I2cRegisterDevice(OP_SENSOR_BUSES(b), 0x76); }
};
template <> struct BaroChip<SENSOR_BARO_SPL06>
{
    using Type = SPL06_Sensor;
    template <class B> static I2cRegisterDevice device(B& b)
    {
        return I2cRegisterDevice(OP_SENSOR_BUSES(b), SPL06_Sensor::DEFAULT_ADDRESS, SPL06_Sensor::ALTERNATE_ADDRESS);
    }
};
template <> struct BaroChip<SENSOR_BARO_SPL06_SPI>
{
    using Type = SPL06_Sensor;
    template <class B> static SpiRegisterDevice device(B& b) { return SPL06_Sensor::spiDevice(b.spi(), Config::PIN_SPI_CS_BARO); }
};
template <> struct BaroChip<SENSOR_BARO_BMP581>
{
    using Type = BMP581_Sensor;
    template <class B> static I2cRegisterDevice device(B& b)
    {
#if SENSOR_AIRSPEED == SENSOR_AIRSPEED_PITOT_BMP581
        // С трубкой Пито на BMP581 адрес 0x47 занят ею — основной только 0x46.
        return I2cRegisterDevice(OP_SENSOR_BUSES(b), BMP581_Sensor::DEFAULT_ADDRESS);
#else
        return I2cRegisterDevice(OP_SENSOR_BUSES(b), BMP581_Sensor::DEFAULT_ADDRESS, BMP581_Sensor::ALTERNATE_ADDRESS);
#endif
    }
};
template <> struct BaroChip<SENSOR_BARO_BMP581_SPI>
{
    using Type = BMP581_Sensor;
    template <class B> static SpiRegisterDevice device(B& b) { return BMP581_Sensor::spiDevice(b.spi(), Config::PIN_SPI_CS_BARO); }
};
template <> struct BaroChip<SENSOR_BARO_DPS310>
{
    using Type = DPS310_Sensor;
    template <class B> static I2cRegisterDevice device(B& b)
    {
        return I2cRegisterDevice(OP_SENSOR_BUSES(b), SPL06_Sensor::DEFAULT_ADDRESS, SPL06_Sensor::ALTERNATE_ADDRESS);
    }
};

template <> struct MagChip<SENSOR_MAG_QMC5883P>
{
    using Type = QMC5883P_Sensor;
    template <class B> static I2cRegisterDevice device(B& b) { return I2cRegisterDevice(OP_EXTERNAL_BUSES(b), QMC5883P_Sensor::DEFAULT_ADDRESS); }
};
template <> struct MagChip<SENSOR_MAG_QMC5883L>
{
    using Type = QMC5883L_Sensor;
    template <class B> static I2cRegisterDevice device(B& b) { return I2cRegisterDevice(OP_EXTERNAL_BUSES(b), QMC5883L_Sensor::DEFAULT_ADDRESS); }
};
template <> struct MagChip<SENSOR_MAG_QMC6309>
{
    using Type = QMC6309_Sensor;
    template <class B> static I2cRegisterDevice device(B& b) { return I2cRegisterDevice(OP_EXTERNAL_BUSES(b), QMC6309_Sensor::DEFAULT_ADDRESS); }
};

// Один выбранный чип: устройство на шине + драйвер. Id = 0 (NONE) —
// пустое место в цепочке.
template <template <int> class Chip, int Id, class Board>
struct SensorSlot
{
    decltype(Chip<Id>::device(std::declval<Board&>())) device;
    typename Chip<Id>::Type sensor;

    explicit SensorSlot(Board& board)
        : device(Chip<Id>::device(board)),
          sensor(device)
    {
    }

    SensorChainCore::Entry entry() { return SensorChainCore::Entry(&sensor, &device); }
    II2CBus* aux() { return sensor.auxI2c(); }
};

template <template <int> class Chip, class Board>
struct SensorSlot<Chip, 0, Board>
{
    explicit SensorSlot(Board&) {}
    SensorChainCore::Entry entry() { return SensorChainCore::Entry(); }
    II2CBus* aux() { return nullptr; }
};

// Все выбранные IMU, барометры и компасы платы — по цепочке на тип.
// main.cpp создаёт один объект и отдаёт автопилоту imu/baro/magnetometer().
template <class Board>
struct SelectedSensors
{
    SensorSlot<ImuChip, SENSOR_IMU, Board> imu1;
    SensorSlot<ImuChip, SENSOR_IMU_2, Board> imu2;
    SensorSlot<ImuChip, SENSOR_IMU_3, Board> imu3;
    // Шина за IMU (компас за LSM6DSV) — к шинам платы, до барометров и
    // компасов: их устройства берут список шин при создании.
    bool auxRegistered;
    SensorSlot<BaroChip, SENSOR_BARO, Board> baro1;
    SensorSlot<BaroChip, SENSOR_BARO_2, Board> baro2;
    SensorSlot<BaroChip, SENSOR_BARO_3, Board> baro3;
    SensorSlot<MagChip, SENSOR_MAG, Board> mag1;
    SensorSlot<MagChip, SENSOR_MAG_2, Board> mag2;
    SensorSlot<MagChip, SENSOR_MAG_3, Board> mag3;

    ImuChain imu;
    BaroChain baro;
    MagChain mag;

    explicit SelectedSensors(Board& board)
        : imu1(board), imu2(board), imu3(board),
          auxRegistered(registerAux(board)),
          baro1(board), baro2(board), baro3(board),
          mag1(board), mag2(board), mag3(board),
          imu(imu1.entry(), imu2.entry(), imu3.entry()),
          baro(baro1.entry(), baro2.entry(), baro3.entry()),
          mag(mag1.entry(), mag2.entry(), mag3.entry())
    {
    }

    bool registerAux(Board& board)
    {
        II2CBus* aux = imu1.aux() ? imu1.aux() : imu2.aux() ? imu2.aux() : imu3.aux();
        if (aux) board.setAuxI2c(aux);
        return aux != nullptr;
    }

    // Компаса не выбрано — nullptr (автопилот так и понимает «компаса нет»).
    MagnetometerSensor* magnetometer() { return mag.chain().configuredCount() ? &mag : nullptr; }
};

// Основной чип типа — для сборок с одним датчиком (коптер, тесты).
#if SENSOR_IMU != SENSOR_IMU_NONE
using SelectedImu = ImuChip<SENSOR_IMU>::Type;
#define SELECTED_IMU_DEVICE(board) ImuChip<SENSOR_IMU>::device(board)
#endif
#if SENSOR_BARO != SENSOR_BARO_NONE
using SelectedBaro = BaroChip<SENSOR_BARO>::Type;
#define SELECTED_BARO_DEVICE(board) BaroChip<SENSOR_BARO>::device(board)
#endif
#if SENSOR_MAG != SENSOR_MAG_NONE
using SelectedMag = MagChip<SENSOR_MAG>::Type;
#define SELECTED_MAG_DEVICE(board) MagChip<SENSOR_MAG>::device(board)
#endif

#if SENSOR_AIRSPEED == SENSOR_AIRSPEED_PITOT_BMP581
    #include "sensors/airspeed/PitotDualBaroAirspeed.h"
    using SelectedPitotBaro = BMP581_Sensor;
    // Трубка — всегда 0x47 (SDO на VDD), чтобы не спутать её с основным BMP581.
    #define SELECTED_PITOT_DEVICE(board) I2cRegisterDevice(OP_EXTERNAL_BUSES(board), BMP581_Sensor::ALTERNATE_ADDRESS)
#elif SENSOR_AIRSPEED == SENSOR_AIRSPEED_NONE
    // main.cpp оборачивает создание трубки в #if SENSOR_AIRSPEED != SENSOR_AIRSPEED_NONE.
#else
    #error "SensorSelection.h: не задан SENSOR_AIRSPEED"
#endif

#if SENSOR_GPS == SENSOR_GPS_UBLOX_M10
    #include "sensors/gps/UbloxM10_Gps.h"
    using SelectedGps = UbloxM10_Gps;
#elif SENSOR_GPS == SENSOR_GPS_NONE
    // main.cpp оборачивает создание GPS в #if SENSOR_GPS != SENSOR_GPS_NONE.
#else
    #error "SensorSelection.h: не задан SENSOR_GPS"
#endif
