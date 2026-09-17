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
//     • MPU6050 (GY-521)      — I2C, адрес 0x68/0x69
//     • ICM42688 ("601N1")    — SPI, свой CS-пин
//   Барометр:
//     • BME280                — I2C, адрес 0x76/0x77, приближённая формула
//     • BMP388                — SPI, свой CS-пин, точная компенсация по датащиту
//   Магнитометр:
//     • QMC5883L ("GY-273")   — I2C, самый распространённый чип на этих платах,
//                               регистры проверены по датащиту (QMC5883L_Sensor.h)
//     • QMC5883P ("GY-273")   — I2C, более редкий чип с другим адресом; регистры —
//                               заготовка, см. TODO в QMC5883P_Sensor.h
//     • нет — компас недоступен, Autopilot получает nullptr
//   GPS:
//     • u-blox M10 (UBX)      — UART, парсит UBX-NAV-PVT
//     • нет — GPS недоступен, Autopilot получает nullptr
//
// ВАЖНО: у каждого датчика свой конструктор (I2C-адрес vs SPI
// CS-пин vs ссылка на UART) — при смене #define сверьте вызов
// конструктора в main.cpp с комментарием у соответствующей ветки
// ниже.
// ============================================================

#define SENSOR_IMU_MPU6050   1
#define SENSOR_IMU_ICM42688  2

#define SENSOR_BARO_BME280   1
#define SENSOR_BARO_BMP388   2

#define SENSOR_MAG_NONE      0
#define SENSOR_MAG_QMC5883P  1
#define SENSOR_MAG_QMC5883L  2

#define SENSOR_GPS_NONE      0
#define SENSOR_GPS_UBLOX_M10 1


// --------------------------------------------------------
// АКТИВНЫЙ ВЫБОР — редактируйте эти 4 строки
// --------------------------------------------------------

#define SENSOR_IMU  SENSOR_IMU_MPU6050
#define SENSOR_BARO SENSOR_BARO_BME280
#define SENSOR_MAG  SENSOR_MAG_NONE
#define SENSOR_GPS  SENSOR_GPS_NONE


// ============================================================
// РАЗРЕШЕНИЕ В КОНКРЕТНЫЕ ТИПЫ (менять не нужно)
// ============================================================

#if SENSOR_IMU == SENSOR_IMU_MPU6050
    #include "MPU6050_Sensor.h"
    using SelectedImu = MPU6050_Sensor;
    // Конструктор: SelectedImu imuSensor(board.i2c(), /*address=*/0x68);
#elif SENSOR_IMU == SENSOR_IMU_ICM42688
    #include "ICM42688_Sensor.h"
    using SelectedImu = ICM42688_Sensor;
    // Конструктор: SelectedImu imuSensor(board.spi(), Config::PIN_SPI_CS_ICM42688);
#else
    #error "SensorSelection.h: не задан SENSOR_IMU"
#endif

#if SENSOR_BARO == SENSOR_BARO_BME280
    #include "BME280_Sensor.h"
    using SelectedBaro = BME280_Sensor;
    // Конструктор: SelectedBaro baroSensor(board.i2c(), /*address=*/0x76);
#elif SENSOR_BARO == SENSOR_BARO_BMP388
    #include "BMP388_Sensor.h"
    using SelectedBaro = BMP388_Sensor;
    // Конструктор: SelectedBaro baroSensor(board.spi(), Config::PIN_SPI_CS_BMP388);
#else
    #error "SensorSelection.h: не задан SENSOR_BARO"
#endif

#if SENSOR_MAG == SENSOR_MAG_QMC5883P
    #include "QMC5883P_Sensor.h"
    using SelectedMag = QMC5883P_Sensor;
    // Конструктор: SelectedMag magSensor(board.i2c());
#elif SENSOR_MAG == SENSOR_MAG_QMC5883L
    #include "QMC5883L_Sensor.h"
    using SelectedMag = QMC5883L_Sensor;
    // Конструктор: SelectedMag magSensor(board.i2c());
#elif SENSOR_MAG == SENSOR_MAG_NONE
    // main.cpp оборачивает создание объекта в #if SENSOR_MAG != SENSOR_MAG_NONE,
    // поэтому SelectedMag здесь не нужен.
#else
    #error "SensorSelection.h: не задан SENSOR_MAG"
#endif

#if SENSOR_GPS == SENSOR_GPS_UBLOX_M10
    #include "UbloxM10_Gps.h"
    using SelectedGps = UbloxM10_Gps;
    // Конструктор: SelectedGps gpsSensor(board.gpsUart());
#elif SENSOR_GPS == SENSOR_GPS_NONE
    // main.cpp оборачивает создание объекта в #if SENSOR_GPS != SENSOR_GPS_NONE,
    // поэтому SelectedGps здесь не нужен.
#else
    #error "SensorSelection.h: не задан SENSOR_GPS"
#endif
