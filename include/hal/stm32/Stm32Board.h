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
// Собирается полная прошивка (env stm32h743, src/stm32/main.cpp); на
// железе проверена голая плата DevEBox H743 (без датчиков и серв),
// шины, ШИМ и UART ещё ждут стенда — основная лётная плата ESP32-S3.
//
// Тот же публичный API, что у Esp32Board: единственное место, которое
// создаёт конкретные шины STM32 (TwoWire/SPIClass/Uart/HardwareTimer)
// и знает пины из Config.h (блок BOARD_STM32H743). Остальной код
// видит только IBoard.
//
// Отличия от ESP32, спрятанные здесь:
//   - периферию (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx) ядро выбирает
//     само по номерам пинов — номеров контроллеров в Config.h нет;
//   - пины UART задаются при создании Uart, а не в begin();
//   - PWM — аппаратные таймеры, общие для выходов на одном TIMx
//     (см. Stm32ServoOutput.h).
// ============================================================

class Stm32Board : public IBoard
{
public:

    Stm32Board()
        : secondWire(pinOf(Config::PIN_I2C2_SDA), pinOf(Config::PIN_I2C2_SCL)),
          externalWire(pinOf(Config::PIN_I2C_EXT_SDA), pinOf(Config::PIN_I2C_EXT_SCL)),
          i2cBus(Wire, pinOf(Config::PIN_I2C_SDA), pinOf(Config::PIN_I2C_SCL), Config::I2C_CLOCK_KHZ * 1000UL),
          secondBus(secondWire, pinOf(Config::PIN_I2C2_SDA), pinOf(Config::PIN_I2C2_SCL), Config::I2C_CLOCK_KHZ * 1000UL),
          externalBus(externalWire, pinOf(Config::PIN_I2C_EXT_SDA), pinOf(Config::PIN_I2C_EXT_SCL), Config::I2C_CLOCK_KHZ * 1000UL),
          spiBus(SPI, pinOf(Config::PIN_SENSOR_SPI_SCK), pinOf(Config::PIN_SENSOR_SPI_MISO), pinOf(Config::PIN_SENSOR_SPI_MOSI)),
          rcPort(Config::PIN_RC_RX, Config::PIN_RC_TX),
          gpsPort(Config::PIN_GPS_RX, Config::PIN_GPS_TX),
          telemetryPort(Config::PIN_TELEM_RX, Config::PIN_TELEM_TX),
          servos{
              Stm32ServoOutput(Config::PIN_AILERON_LEFT),
              Stm32ServoOutput(Config::PIN_AILERON_RIGHT),
              Stm32ServoOutput(Config::PIN_ELEVATOR),
              Stm32ServoOutput(Config::PIN_ESC),
              Stm32ServoOutput(Config::PIN_RUDDER),
              Stm32ServoOutput(Config::PIN_AUX1),
              Stm32ServoOutput(Config::PIN_AUX2)
          }
    {
    }

    void begin() override
    {
        i2cBus.begin();
        spiBus.begin();
        if (hasSecondBus()) secondBus.begin();
        if (hasExternalBus()) externalBus.begin();

        pinMode(pinOf(Config::PIN_BUZZER), OUTPUT);
        digitalWrite(pinOf(Config::PIN_BUZZER), LOW);
    }

    II2CBus& i2c() override { return i2cBus; }
    ISpiBus& spi() override { return spiBus; }
    II2CBus* secondI2c() override { return hasSecondBus() ? &secondBus : nullptr; }
    II2CBus& externalI2c() override { return hasExternalBus() ? static_cast<II2CBus&>(externalBus) : i2cBus; }

    IUartPort& rcUart() override { return rcPort; }
    IUartPort& gpsUart() override { return gpsPort; }
    IUartPort* telemetryUart() override { return &telemetryPort; }

    // P63 (коптер): приёмник, GPS и MSP — на UART из настроек serial_*_uart;
    // пара ножек {rx, tx}, −1 −1 — роли нет. До begin() портов.
    void rebindUarts(const int16_t rc[2], const int16_t gps[2], const int16_t msp[2])
    {
        rcPort.rebind(rc[0], rc[1]);
        gpsPort.rebind(gps[0], gps[1]);
        telemetryPort.rebind(msp[0], msp[1]);
    }

    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }

    void setBuzzer(bool on) override
    {
        digitalWrite(pinOf(Config::PIN_BUZZER), on ? HIGH : LOW);
    }


private:

    // Второй и третий контроллеры I2C — свои объекты TwoWire (глобальный
    // Wire занят шиной датчиков). Объявлены раньше шин, которые хранят
    // ссылки на них. Пин −1 — такой шины на плате нет: второй (Matek)
    // или отдельного внешнего разъёма (DevEBox).
    TwoWire secondWire;
    TwoWire externalWire;

    Stm32I2CBus i2cBus;
    Stm32I2CBus secondBus;
    Stm32I2CBus externalBus;
    Stm32SpiBus spiBus;

    Stm32UartPort rcPort;
    Stm32UartPort gpsPort;
    Stm32UartPort telemetryPort;   // самолёт: радиомодем MAVLink; коптер: MSP (очки)

    Stm32ServoOutput servos[ServoChannel::COUNT];

    static constexpr bool hasSecondBus() { return Config::PIN_I2C2_SDA >= 0 && Config::PIN_I2C2_SCL >= 0; }
    static constexpr bool hasExternalBus() { return Config::PIN_I2C_EXT_SDA >= 0 && Config::PIN_I2C_EXT_SCL >= 0; }

    // В Config.h пины — int16_t (−1 = не разведён); API ядра ждёт pin_size_t.
    static constexpr pin_size_t pinOf(int16_t pin)
    {
        return static_cast<pin_size_t>(pin);
    }
};
