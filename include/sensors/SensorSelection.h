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
// Для каждого датчика здесь же задано, на какой шине он висит:
// SELECTED_*_DEVICE(board) создаёт регистровое устройство
// (I2cRegisterDevice с адресом или SpiRegisterDevice с CS-пином,
// см. hal/RegisterDevice.h), которое main.cpp передаёт драйверу.
// Драйверы шины не различают — один и тот же BMP581_Sensor
// работает и по I2C, и по SPI.
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
// ============================================================

// Нужны тем, кто раскрывает SELECTED_*_DEVICE(board): макросы
// ссылаются на I2cRegisterDevice/SpiRegisterDevice и пины Config.
#include "config/Config.h"          // IWYU pragma: export
#include "hal/RegisterDevice.h"     // IWYU pragma: export

#define SENSOR_IMU_MPU6050       1
#define SENSOR_IMU_ICM42688      2   // SPI
#define SENSOR_IMU_LSM6DSV       3   // I2C
#define SENSOR_IMU_LSM6DSV_SPI   4
#define SENSOR_IMU_ICM45686      5   // I2C
#define SENSOR_IMU_ICM45686_SPI  6

#define SENSOR_BARO_BME280      1
#define SENSOR_BARO_BMP388      2   // SPI
#define SENSOR_BARO_BMP388_I2C  3
#define SENSOR_BARO_SPL06       4   // I2C
#define SENSOR_BARO_SPL06_SPI   5
#define SENSOR_BARO_BMP581      6   // I2C
#define SENSOR_BARO_BMP581_SPI  7

#define SENSOR_MAG_NONE      0
#define SENSOR_MAG_QMC5883P  1
#define SENSOR_MAG_QMC5883L  2
#define SENSOR_MAG_QMC6309   3

#define SENSOR_AIRSPEED_NONE          0
#define SENSOR_AIRSPEED_PITOT_BMP581  1

#define SENSOR_GPS_NONE      0
#define SENSOR_GPS_UBLOX_M10 1

// Готовые наборы: SENSOR_KIT задаёт все датчики разом.
#define SENSOR_KIT_CUSTOM          0   // каждый датчик — своей строкой ниже
#define SENSOR_KIT_BENCH_GY521     1   // стенд: GY-521 (MPU6500), BMP581 I2C, GY-273 (QMC5883P)
#define SENSOR_KIT_LSM6DSV_PITOT   2   // LSM6DSV+QMC6309, SPL06 в фюзеляже, BMP581 в трубке Пито, GPS
#define SENSOR_KIT_ICM45686_PITOT  3   // ICM-45686+QMC6309, SPL06, BMP581 в трубке, GPS


// --------------------------------------------------------
// АКТИВНЫЙ ВЫБОР — одна строка SENSOR_KIT...
//
// Любую строку можно переопределить флагом сборки без правки файла,
// например build_flags = -D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT.
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


// ============================================================
// РАЗРЕШЕНИЕ В КОНКРЕТНЫЕ ТИПЫ (менять не нужно, кроме адресов/пинов)
// ============================================================

#if SENSOR_IMU == SENSOR_IMU_MPU6050
    #include "sensors/imu/MPU6050_Sensor.h"
    using SelectedImu = MPU6050_Sensor;
    #define SELECTED_IMU_DEVICE(board) I2cRegisterDevice((board).i2c(), 0x68)
#elif SENSOR_IMU == SENSOR_IMU_ICM42688
    #include "sensors/imu/ICM42688_Sensor.h"
    using SelectedImu = ICM42688_Sensor;
    #define SELECTED_IMU_DEVICE(board) ICM42688_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_IMU)
#elif SENSOR_IMU == SENSOR_IMU_LSM6DSV
    #include "sensors/imu/LSM6DSV_Sensor.h"
    using SelectedImu = LSM6DSV_Sensor;
    #define SELECTED_IMU_DEVICE(board) I2cRegisterDevice((board).i2c(), LSM6DSV_Sensor::DEFAULT_ADDRESS, LSM6DSV_Sensor::ALTERNATE_ADDRESS)
#elif SENSOR_IMU == SENSOR_IMU_LSM6DSV_SPI
    #include "sensors/imu/LSM6DSV_Sensor.h"
    using SelectedImu = LSM6DSV_Sensor;
    #define SELECTED_IMU_DEVICE(board) LSM6DSV_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_IMU)
#elif SENSOR_IMU == SENSOR_IMU_ICM45686
    #include "sensors/imu/ICM45686_Sensor.h"
    using SelectedImu = ICM45686_Sensor;
    #define SELECTED_IMU_DEVICE(board) I2cRegisterDevice((board).i2c(), ICM45686_Sensor::DEFAULT_ADDRESS, ICM45686_Sensor::ALTERNATE_ADDRESS)
#elif SENSOR_IMU == SENSOR_IMU_ICM45686_SPI
    #include "sensors/imu/ICM45686_Sensor.h"
    using SelectedImu = ICM45686_Sensor;
    #define SELECTED_IMU_DEVICE(board) ICM45686_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_IMU)
#else
    #error "SensorSelection.h: не задан SENSOR_IMU"
#endif

#if SENSOR_BARO == SENSOR_BARO_BME280
    #include "sensors/baro/BME280_Sensor.h"
    using SelectedBaro = BME280_Sensor;
    #define SELECTED_BARO_DEVICE(board) I2cRegisterDevice((board).i2c(), 0x76)
#elif SENSOR_BARO == SENSOR_BARO_BMP388
    #include "sensors/baro/BMP388_Sensor.h"
    using SelectedBaro = BMP388_Sensor;
    #define SELECTED_BARO_DEVICE(board) BMP388_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_BARO)
#elif SENSOR_BARO == SENSOR_BARO_BMP388_I2C
    #include "sensors/baro/BMP388_Sensor.h"
    using SelectedBaro = BMP388_Sensor;
    #define SELECTED_BARO_DEVICE(board) I2cRegisterDevice((board).i2c(), 0x76)
#elif SENSOR_BARO == SENSOR_BARO_SPL06
    #include "sensors/baro/SPL06_Sensor.h"
    using SelectedBaro = SPL06_Sensor;
    #define SELECTED_BARO_DEVICE(board) I2cRegisterDevice((board).i2c(), SPL06_Sensor::DEFAULT_ADDRESS, SPL06_Sensor::ALTERNATE_ADDRESS)
#elif SENSOR_BARO == SENSOR_BARO_SPL06_SPI
    #include "sensors/baro/SPL06_Sensor.h"
    using SelectedBaro = SPL06_Sensor;
    #define SELECTED_BARO_DEVICE(board) SPL06_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_BARO)
#elif SENSOR_BARO == SENSOR_BARO_BMP581
    #include "sensors/baro/BMP581_Sensor.h"
    using SelectedBaro = BMP581_Sensor;
    // С трубкой Пито на BMP581 адрес 0x47 занят ею — основной только 0x46.
    #if SENSOR_AIRSPEED == SENSOR_AIRSPEED_PITOT_BMP581
        #define SELECTED_BARO_DEVICE(board) I2cRegisterDevice((board).i2c(), BMP581_Sensor::DEFAULT_ADDRESS)
    #else
        #define SELECTED_BARO_DEVICE(board) I2cRegisterDevice((board).i2c(), BMP581_Sensor::DEFAULT_ADDRESS, BMP581_Sensor::ALTERNATE_ADDRESS)
    #endif
#elif SENSOR_BARO == SENSOR_BARO_BMP581_SPI
    #include "sensors/baro/BMP581_Sensor.h"
    using SelectedBaro = BMP581_Sensor;
    #define SELECTED_BARO_DEVICE(board) BMP581_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_BARO)
#else
    #error "SensorSelection.h: не задан SENSOR_BARO"
#endif

#if SENSOR_MAG == SENSOR_MAG_QMC5883P
    #include "sensors/mag/QMC5883P_Sensor.h"
    using SelectedMag = QMC5883P_Sensor;
    #define SELECTED_MAG_DEVICE(board) I2cRegisterDevice((board).i2c(), QMC5883P_Sensor::DEFAULT_ADDRESS)
#elif SENSOR_MAG == SENSOR_MAG_QMC5883L
    #include "sensors/mag/QMC5883L_Sensor.h"
    using SelectedMag = QMC5883L_Sensor;
    #define SELECTED_MAG_DEVICE(board) I2cRegisterDevice((board).i2c(), QMC5883L_Sensor::DEFAULT_ADDRESS)
#elif SENSOR_MAG == SENSOR_MAG_QMC6309
    #include "sensors/mag/QMC6309_Sensor.h"
    using SelectedMag = QMC6309_Sensor;
    #define SELECTED_MAG_DEVICE(board) I2cRegisterDevice((board).i2c(), QMC6309_Sensor::DEFAULT_ADDRESS)
#elif SENSOR_MAG == SENSOR_MAG_NONE
    // main.cpp оборачивает создание компаса в #if SENSOR_MAG != SENSOR_MAG_NONE.
#else
    #error "SensorSelection.h: не задан SENSOR_MAG"
#endif

#if SENSOR_AIRSPEED == SENSOR_AIRSPEED_PITOT_BMP581
    #include "sensors/airspeed/PitotDualBaroAirspeed.h"
    #include "sensors/baro/BMP581_Sensor.h"
    using SelectedPitotBaro = BMP581_Sensor;
    // Трубка — всегда 0x47 (SDO на VDD), чтобы не спутать её с основным BMP581.
    #define SELECTED_PITOT_DEVICE(board) I2cRegisterDevice((board).i2c(), BMP581_Sensor::ALTERNATE_ADDRESS)
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
