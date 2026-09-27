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
// железе пока не проверялась — основная лётная плата ESP32-S3.
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

    Stm32Board();

    void begin() override;

    II2CBus& i2c() override { return i2cBus; }
    ISpiBus& spi() override { return spiBus; }
    II2CBus* displayI2c() override { return &displayBus; }

    IUartPort& rcUart() override { return rcPort; }
    IUartPort& gpsUart() override { return gpsPort; }
    IUartPort* telemetryUart() override { return &telemetryPort; }

    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }

    void setBuzzer(bool on) override;


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
    Uart telemetrySerial;   // UART4: радиомодем MAVLink
    Stm32UartPort rcPort;
    Stm32UartPort gpsPort;
    Stm32UartPort telemetryPort;

    Stm32ServoOutput servos[ServoChannel::COUNT];

    // В Config.h пины — int16_t (−1 = не разведён); API ядра ждёт pin_size_t.
    static constexpr pin_size_t pinOf(int16_t pin)
    {
        return static_cast<pin_size_t>(pin);
    }
};
