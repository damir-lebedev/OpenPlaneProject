#pragma once

// ============================================================
// 🔧 ВЫБОР ДАТЧИКОВ ПРОЕКТА
//
// Единственное место, которое нужно менять, чтобы сменить
// физический датчик — поменяйте #define ниже на один из
// поддерживаемых вариантов. main.cpp дальше работает через
// SelectedImu/SelectedBaro/SelectedMag/SelectedGps, не зная,
// какой конкретно класс за ними стоит (тот же приём, что выбор
// платы через BOARD_ESP32_* в Config.h).
//
// ПОДДЕРЖИВАЕМЫЕ ДАТЧИКИ:
//   IMU (гироскоп + акселерометр):
//     • MPU6050/MPU6500 (GY-521) — I2C, адрес 0x68/0x69; чип
//                                   определяется по WHO_AM_I
//     • ICM42688 ("601N1")    — SPI, свой CS-пин
//   Барометр:
//     • BME280                — I2C, адрес 0x76/0x77, приближённая формула
//     • BMP388                — SPI, свой CS-пин, точная компенсация по датащиту
//     • BMP388 (I2C)          — I2C, адрес 0x76/0x77, та же компенсация
//   Магнитометр:
//     • QMC5883L ("GY-273")   — I2C, адрес 0x0D
//     • QMC5883P ("GY-273")   — I2C, адрес 0x2C (на текущем стенде — он)
//     • нет — компас недоступен, Autopilot получает nullptr
//   GPS:
//     • u-blox M10 (UBX)      — UART, парсит UBX-NAV-PVT
//     • нет — GPS недоступен, Autopilot получает nullptr
//
// Аргументы конструктора каждого датчика (I2C-шина + адрес, SPI +
// CS-пин, UART) заданы здесь же, в SELECTED_*_ARGS — main.cpp
// менять при смене датчика не нужно.
// ============================================================

#define SENSOR_IMU_MPU6050   1
#define SENSOR_IMU_ICM42688  2

#define SENSOR_BARO_BME280      1
#define SENSOR_BARO_BMP388      2
#define SENSOR_BARO_BMP388_I2C  3

#define SENSOR_MAG_NONE      0
#define SENSOR_MAG_QMC5883P  1
#define SENSOR_MAG_QMC5883L  2

#define SENSOR_GPS_NONE      0
#define SENSOR_GPS_UBLOX_M10 1


// --------------------------------------------------------
// АКТИВНЫЙ ВЫБОР — редактируйте эти 4 строки
// (текущий стенд: GY-521 с MPU6500, BMP388 по I2C, GY-273 с QMC5883P)
// --------------------------------------------------------

#define SENSOR_IMU  SENSOR_IMU_MPU6050
#define SENSOR_BARO SENSOR_BARO_BMP388_I2C
#define SENSOR_MAG  SENSOR_MAG_QMC5883P
#define SENSOR_GPS  SENSOR_GPS_NONE


// ============================================================
// РАЗРЕШЕНИЕ В КОНКРЕТНЫЕ ТИПЫ (менять не нужно, кроме адресов)
// ============================================================

#if SENSOR_IMU == SENSOR_IMU_MPU6050
    #include "MPU6050_Sensor.h"
    using SelectedImu = MPU6050_Sensor;
    #define SELECTED_IMU_ARGS(board) (board).i2c(), 0x68
#elif SENSOR_IMU == SENSOR_IMU_ICM42688
    #include "ICM42688_Sensor.h"
    using SelectedImu = ICM42688_Sensor;
    #define SELECTED_IMU_ARGS(board) (board).spi(), Config::PIN_SPI_CS_ICM42688
#else
    #error "SensorSelection.h: не задан SENSOR_IMU"
#endif

#if SENSOR_BARO == SENSOR_BARO_BME280
    #include "BME280_Sensor.h"
    using SelectedBaro = BME280_Sensor;
    #define SELECTED_BARO_ARGS(board) (board).i2c(), 0x76
#elif SENSOR_BARO == SENSOR_BARO_BMP388
    #include "BMP388_Sensor.h"
    using SelectedBaro = BMP388_Sensor;
    #define SELECTED_BARO_ARGS(board) (board).spi(), Config::PIN_SPI_CS_BMP388
#elif SENSOR_BARO == SENSOR_BARO_BMP388_I2C
    #include "BMP388_I2C_Sensor.h"
    using SelectedBaro = BMP388_I2C_Sensor;
    #define SELECTED_BARO_ARGS(board) (board).i2c(), 0x76
#else
    #error "SensorSelection.h: не задан SENSOR_BARO"
#endif

#if SENSOR_MAG == SENSOR_MAG_QMC5883P
    #include "QMC5883P_Sensor.h"
    using SelectedMag = QMC5883P_Sensor;
    #define SELECTED_MAG_ARGS(board) (board).i2c()
#elif SENSOR_MAG == SENSOR_MAG_QMC5883L
    #include "QMC5883L_Sensor.h"
    using SelectedMag = QMC5883L_Sensor;
    #define SELECTED_MAG_ARGS(board) (board).i2c()
#elif SENSOR_MAG == SENSOR_MAG_NONE
    // main.cpp оборачивает создание объекта в #if SENSOR_MAG != SENSOR_MAG_NONE,
    // поэтому SelectedMag здесь не нужен.
#else
    #error "SensorSelection.h: не задан SENSOR_MAG"
#endif

#if SENSOR_GPS == SENSOR_GPS_UBLOX_M10
    #include "UbloxM10_Gps.h"
    using SelectedGps = UbloxM10_Gps;
    #define SELECTED_GPS_ARGS(board) (board).gpsUart()
#elif SENSOR_GPS == SENSOR_GPS_NONE
    // main.cpp оборачивает создание объекта в #if SENSOR_GPS != SENSOR_GPS_NONE,
    // поэтому SelectedGps здесь не нужен.
#else
    #error "SensorSelection.h: не задан SENSOR_GPS"
#endif
