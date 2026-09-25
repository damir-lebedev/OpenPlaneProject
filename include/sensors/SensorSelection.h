#pragma once

// ============================================================
// 🔧 ВЫБОР ДАТЧИКОВ ПРОЕКТА
//
// Единственное место, которое нужно менять, чтобы сменить
// физический датчик: поменяйте #define в блоке "АКТИВНЫЙ ВЫБОР".
// main.cpp работает через SelectedImu/SelectedBaro/SelectedMag/
// SelectedGps и не знает, какой класс за ними стоит.
//
// Для каждого датчика здесь же задано, на какой шине он висит:
// SELECTED_*_DEVICE(board) создаёт регистровое устройство
// (I2cRegisterDevice с адресом или SpiRegisterDevice с CS-пином,
// см. hal/RegisterDevice.h), которое main.cpp передаёт драйверу.
// Драйверы шины не различают — один и тот же BMP388_Sensor
// работает и по I2C, и по SPI.
//
// ПОДДЕРЖИВАЕМЫЕ ДАТЧИКИ:
//   IMU (гироскоп + акселерометр):
//     • MPU6050 / MPU6500 (GY-521) — I2C 0x68; чип по WHO_AM_I
//     • ICM42688 ("601N1")        — SPI, свой CS
//   Барометр:
//     • BMP388 по I2C              — 0x76
//     • BMP388 по SPI              — свой CS
//     • BME280 / BMP280            — I2C 0x76
//   Магнитометр (плата GY-273, чип бывает разный):
//     • QMC5883P                   — I2C 0x2C (на текущем стенде)
//     • QMC5883L                   — I2C 0x0D
//     • нет
//   GPS:
//     • u-blox M10 (UBX)           — UART
//     • нет
// ============================================================

// Нужны тем, кто раскрывает SELECTED_*_DEVICE(board): макросы
// ссылаются на I2cRegisterDevice/SpiRegisterDevice и пины Config.
#include "config/Config.h"          // IWYU pragma: export
#include "hal/RegisterDevice.h"     // IWYU pragma: export

#define SENSOR_IMU_MPU6050   1
#define SENSOR_IMU_ICM42688  2

#define SENSOR_BARO_BME280      1
#define SENSOR_BARO_BMP388      2   // SPI
#define SENSOR_BARO_BMP388_I2C  3

#define SENSOR_MAG_NONE      0
#define SENSOR_MAG_QMC5883P  1
#define SENSOR_MAG_QMC5883L  2

#define SENSOR_GPS_NONE      0
#define SENSOR_GPS_UBLOX_M10 1


// --------------------------------------------------------
// АКТИВНЫЙ ВЫБОР — редактируйте эти 4 строки
// (текущий стенд: GY-521 с MPU6500, BMP388 по I2C, GY-273 с QMC5883P)
//
// Любую можно переопределить флагом сборки без правки файла, например
// build_flags = -D SENSOR_IMU=SENSOR_IMU_ICM42688 в platformio.ini.
// --------------------------------------------------------

#ifndef SENSOR_IMU
#define SENSOR_IMU  SENSOR_IMU_MPU6050
#endif

#ifndef SENSOR_BARO
#define SENSOR_BARO SENSOR_BARO_BMP388_I2C
#endif

#ifndef SENSOR_MAG
#define SENSOR_MAG  SENSOR_MAG_QMC5883P
#endif

#ifndef SENSOR_GPS
#define SENSOR_GPS  SENSOR_GPS_NONE
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
    #define SELECTED_IMU_DEVICE(board) ICM42688_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_ICM42688)
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
    #define SELECTED_BARO_DEVICE(board) BMP388_Sensor::spiDevice((board).spi(), Config::PIN_SPI_CS_BMP388)
#elif SENSOR_BARO == SENSOR_BARO_BMP388_I2C
    #include "sensors/baro/BMP388_Sensor.h"
    using SelectedBaro = BMP388_Sensor;
    #define SELECTED_BARO_DEVICE(board) I2cRegisterDevice((board).i2c(), 0x76)
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
#elif SENSOR_MAG == SENSOR_MAG_NONE
    // main.cpp оборачивает создание компаса в #if SENSOR_MAG != SENSOR_MAG_NONE.
#else
    #error "SensorSelection.h: не задан SENSOR_MAG"
#endif

#if SENSOR_GPS == SENSOR_GPS_UBLOX_M10
    #include "sensors/gps/UbloxM10_Gps.h"
    using SelectedGps = UbloxM10_Gps;
#elif SENSOR_GPS == SENSOR_GPS_NONE
    // main.cpp оборачивает создание GPS в #if SENSOR_GPS != SENSOR_GPS_NONE.
#else
    #error "SensorSelection.h: не задан SENSOR_GPS"
#endif
