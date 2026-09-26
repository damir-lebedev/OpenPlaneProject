#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "config/Config.h"
#include "hal/IBoard.h"
#include "hal/stm32/Stm32I2CBus.h"
#include "hal/stm32/Stm32ServoOutput.h"
#include "hal/stm32/Stm32SpiBus.h"
#include "hal/stm32/Stm32UartPort.h"

// ============================================================
// 🧠 РЕАЛИЗАЦИЯ "МОЗГА" ДЛЯ STM32H743 (STM32duino 3.x)
//
// ЗАГОТОВКА: собирается (env stm32h743), на железе не проверялась.
// Основная плата по-прежнему ESP32-S3 (Esp32Board).
//
// Тот же публичный API, что у Esp32Board: единственное место, которое
// создаёт конкретные шины STM32 (TwoWire/SPIClass/Uart/HardwareTimer)
// и знает пины из Config.h (блок BOARD_STM32H743). Остальной код
// видит только IBoard.
//
// Отличия от ESP32, спрятанные здесь:
//   - периферию (I2C1/I2C2, SPI2, USART3, UART7, TIMx) ядро выбирает
//     само по номерам пинов — номеров контроллеров в Config.h нет;
//   - пины UART задаются при создании Uart, а не в begin();
//   - PWM — аппаратные таймеры, общие для выходов на одном TIMx
//     (см. Stm32ServoOutput.h).
// ============================================================

class Stm32Board : public IBoard
{
public:

    Stm32Board()
        : displayWire(pinOf(Config::PIN_I2C2_SDA), pinOf(Config::PIN_I2C2_SCL)),
          i2cBus(Wire, pinOf(Config::PIN_I2C_SDA), pinOf(Config::PIN_I2C_SCL)),
          displayBus(displayWire, pinOf(Config::PIN_I2C2_SDA), pinOf(Config::PIN_I2C2_SCL)),
          spiBus(SPI, pinOf(Config::PIN_SENSOR_SPI_SCK), pinOf(Config::PIN_SENSOR_SPI_MISO), pinOf(Config::PIN_SENSOR_SPI_MOSI)),
          rcSerial(pinOf(Config::PIN_IBUS), pinOf(Config::PIN_IBUS_TX)),
          gpsSerial(pinOf(Config::PIN_GPS_RX), pinOf(Config::PIN_GPS_TX)),
          rcPort(rcSerial),
          gpsPort(gpsSerial),
          servos{
              Stm32ServoOutput(Config::PIN_AILERON_LEFT),
              Stm32ServoOutput(Config::PIN_AILERON_RIGHT),
              Stm32ServoOutput(Config::PIN_ELEVATOR),
              Stm32ServoOutput(Config::PIN_ESC),
              Stm32ServoOutput(Config::PIN_RUDDER)
          }
    {
    }

    void begin() override
    {
        i2cBus.begin();
        spiBus.begin();
        displayBus.begin();
    }

    II2CBus& i2c() override { return i2cBus; }
    ISpiBus& spi() override { return spiBus; }
    II2CBus* displayI2c() override { return &displayBus; }

    IUartPort& rcUart() override { return rcPort; }
    IUartPort& gpsUart() override { return gpsPort; }

    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }


private:

    // Второй контроллер I2C — отдельный объект TwoWire (глобальный
    // Wire занят шиной датчиков). Объявлен раньше displayBus, который
    // хранит ссылку на него.
    TwoWire displayWire;

    Stm32I2CBus i2cBus;
    Stm32I2CBus displayBus;
    Stm32SpiBus spiBus;

    Uart rcSerial;
    Uart gpsSerial;
    Stm32UartPort rcPort;
    Stm32UartPort gpsPort;

    Stm32ServoOutput servos[ServoChannel::COUNT];

    // В Config.h пины — int16_t (−1 = не разведён); API ядра ждёт pin_size_t.
    static constexpr pin_size_t pinOf(int16_t pin)
    {
        return static_cast<pin_size_t>(pin);
    }
};
