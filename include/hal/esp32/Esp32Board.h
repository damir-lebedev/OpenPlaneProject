#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "soc/soc_caps.h"

#include "config/Config.h"
#include "hal/IBoard.h"
#include "hal/esp32/Esp32I2CBus.h"
#include "hal/esp32/Esp32ServoOutput.h"
#include "hal/esp32/Esp32SpiBus.h"
#include "hal/esp32/Esp32UartPort.h"

// ============================================================
// 🧠 РЕАЛИЗАЦИЯ "МОЗГА" ДЛЯ ESP32
//
// Единственное место, которое создаёт конкретные ESP32-шины
// (Wire/Wire1/SPI/HardwareSerial/LEDC) и знает пины из Config.h.
// Всё остальное (FlightOutputs, IBusReceiver, драйверы датчиков,
// экран, main.cpp) видит только интерфейс IBoard.
//
// Под другой MCU — своя плата с таким же публичным API (заготовка
// для STM32H743 — hal/stm32/Stm32Board.h); main.cpp меняет один тип
// объекта.
// ============================================================

class Esp32Board : public IBoard
{
public:

    Esp32Board()
        : i2cBus(Wire, Config::PIN_I2C_SDA, Config::PIN_I2C_SCL),
#if SOC_I2C_NUM > 1
          displayBus(Wire1, Config::PIN_I2C2_SDA, Config::PIN_I2C2_SCL),
#endif
          spiBus(Config::PIN_SENSOR_SPI_SCK, Config::PIN_SENSOR_SPI_MISO, Config::PIN_SENSOR_SPI_MOSI),
          rcSerial(1),
          gpsSerial(Config::UART_NUM_GPS),
          rcPort(rcSerial, Config::PIN_IBUS, -1),
          gpsPort(gpsSerial, Config::PIN_GPS_RX, Config::PIN_GPS_TX),
          servos{
              // Второй аргумент — канал LEDC, у каждого выхода свой.
              Esp32ServoOutput(Config::PIN_AILERON_LEFT, 0),
              Esp32ServoOutput(Config::PIN_AILERON_RIGHT, 1),
              Esp32ServoOutput(Config::PIN_ELEVATOR, 2),
              Esp32ServoOutput(Config::PIN_ESC, 3),
              Esp32ServoOutput(Config::PIN_RUDDER, 4)
          }
    {
    }

    void begin() override
    {
        i2cBus.begin();
        spiBus.begin();

#if SOC_I2C_NUM > 1
        if (hasDisplayBus())
        {
            displayBus.begin();
        }
#endif
    }

    II2CBus& i2c() override { return i2cBus; }
    ISpiBus& spi() override { return spiBus; }

    II2CBus* displayI2c() override
    {
#if SOC_I2C_NUM > 1
        return hasDisplayBus() ? &displayBus : nullptr;
#else
        return nullptr;
#endif
    }

    IUartPort& rcUart() override { return rcPort; }
    IUartPort& gpsUart() override { return gpsPort; }

    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }


private:

    Esp32I2CBus i2cBus;
#if SOC_I2C_NUM > 1
    Esp32I2CBus displayBus;
#endif
    Esp32SpiBus spiBus;

    HardwareSerial rcSerial;
    HardwareSerial gpsSerial;
    Esp32UartPort rcPort;
    Esp32UartPort gpsPort;

    Esp32ServoOutput servos[ServoChannel::COUNT];

#if SOC_I2C_NUM > 1
    // Вторая шина есть, только если у чипа два контроллера I2C
    // (у ESP32-C3 — один, там этой функции и поля displayBus нет) и
    // для неё заданы пины в Config.h.
    static constexpr bool hasDisplayBus()
    {
        return Config::PIN_I2C2_SDA >= 0 && Config::PIN_I2C2_SCL >= 0;
    }
#endif
};
