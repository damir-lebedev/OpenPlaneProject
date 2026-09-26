#pragma once
#include <Arduino.h>

#include "hal/IUartPort.h"

// ============================================================
// Реализация IUartPort для STM32 (STM32duino 3.x) — обёртка над
// HardwareSerial. ЗАГОТОВКА: на железе не проверялась.
//
// В отличие от ESP32, пины UART на STM32 выбираются не в begin(),
// а при создании объекта Uart(rx, tx) — по ним ядро само находит
// периферию (USART3, UART7...) в таблицах PeripheralPins варианта.
// Поэтому порт здесь получает уже созданный Uart (его держит
// Stm32Board) и в begin(baud) задаёт только скорость и формат 8N1.
//
// Размер приёмного буфера — SERIAL_RX_BUFFER_SIZE (задан в
// platformio.ini, env stm32h743): кадр iBUS — 32 байта, NAV-PVT —
// 100, а стандартных 64 байт мало, если цикл один раз задержится.
// ============================================================

class Stm32UartPort : public IUartPort
{
public:
    explicit Stm32UartPort(HardwareSerial& port)
        : serial(port)
    {
    }

    void begin(uint32_t baud) override
    {
        serial.begin(baud, SERIAL_8N1);
    }

    int available() override { return serial.available(); }
    int read() override { return serial.read(); }
    size_t write(uint8_t value) override { return serial.write(value); }
    size_t write(const uint8_t* buffer, size_t size) override { return serial.write(buffer, size); }


private:
    HardwareSerial& serial;
};
