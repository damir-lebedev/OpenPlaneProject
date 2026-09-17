#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

#include "../IBoard.h"
#include "Esp32I2CBus.h"
#include "Esp32SpiBus.h"
#include "Esp32UartPort.h"
#include "Esp32ServoOutput.h"
#include "../../Config.h"

// ============================================================
// 🧠 РЕАЛИЗАЦИЯ "МОЗГА" ДЛЯ ESP32
//
// Единственное место, которое инстанцирует конкретные ESP32-шины
// (Wire/SPI/HardwareSerial/ESP32Servo) и знает пины из Config.h.
// Всё остальное (FlightOutputs, IBusReceiver, драйверы датчиков,
// main.cpp) видит только интерфейс IBoard.
//
// Чтобы перейти на другой MCU — пишется hal/stm32/Stm32Board.h с
// таким же публичным API, main.cpp меняет один тип объекта, и всё.
// ============================================================

class Esp32Board : public IBoard
{
public:
    Esp32Board()
        : i2cBus(Config::PIN_I2C_SDA, Config::PIN_I2C_SCL),
          spiBus(Config::PIN_SPI_SCK, Config::PIN_SPI_MISO, Config::PIN_SPI_MOSI),
          rcSerial(1),
          gpsSerial(Config::UART_NUM_GPS),
          rcPort(rcSerial, Config::PIN_IBUS, -1),
          gpsPort(gpsSerial, Config::PIN_GPS_RX, Config::PIN_GPS_TX),
          servos{
              Esp32ServoOutput(Config::PIN_AILERON_LEFT),
              Esp32ServoOutput(Config::PIN_AILERON_RIGHT),
              Esp32ServoOutput(Config::PIN_ELEVATOR),
              Esp32ServoOutput(Config::PIN_ESC)
          }
    {
    }

    void begin() override
    {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);

        i2cBus.begin();
        spiBus.begin();

        // rcUart()/gpsUart() сознательно не инициализируются здесь —
        // begin(baud) вызывают их владельцы (IBusReceiver, GPS-драйвер),
        // как и раньше делал IBusReceiver сам для HardwareSerial.
    }

    II2CBus& i2c() override { return i2cBus; }
    ISpiBus& spi() override { return spiBus; }

    IUartPort& rcUart() override { return rcPort; }
    IUartPort& gpsUart() override { return gpsPort; }

    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }


private:
    Esp32I2CBus i2cBus;
    Esp32SpiBus spiBus;

    HardwareSerial rcSerial;
    HardwareSerial gpsSerial;
    Esp32UartPort rcPort;
    Esp32UartPort gpsPort;

    Esp32ServoOutput servos[ServoChannel::COUNT];
};
