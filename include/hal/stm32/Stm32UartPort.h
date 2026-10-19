#pragma once
#include <Arduino.h>

#include "hal/IUartPort.h"

// ============================================================
// Реализация IUartPort для STM32 (STM32duino 3.x) — обёртка над
// HardwareSerial. На железе (с датчиками) ещё не проверялась.
//
// В отличие от ESP32, пины UART на STM32 выбираются не в begin(),
// а при создании объекта Uart(rx, tx) — по ним ядро само находит
// периферию (USART3, UART7...) в таблицах PeripheralPins варианта.
// Поэтому порт здесь получает уже созданный Uart (его держит
// Stm32Board) и в begin() задаёт только скорость и формат кадра.
// SBUS: 8E2 (8 бит данных + чётность — ядро само берёт 9-битное
// слово) и инверсия RX средствами самого USART (RXINV) — внешний
// инвертор не нужен.
//
// Размер приёмного буфера — SERIAL_RX_BUFFER_SIZE (задан в
// platformio.ini, env stm32h743): кадр iBUS — 32 байта, NAV-PVT —
// 100, а стандартных 64 байт мало, если цикл один раз задержится.
// ============================================================

class Stm32UartPort : public IUartPort
{
public:
    explicit Stm32UartPort(Uart& port)
        : serial(port)
    {
    }

    void begin(uint32_t baud) override
    {
        serial.begin(baud, SERIAL_8N1);
    }

    bool begin(uint32_t baud, UartFormat format) override
    {
        if (format == UartFormat::SBUS_8E2_INVERTED)
        {
            serial.setRxInvert();   // только до begin()
            serial.begin(baud, SERIAL_8E2);
        }
        else
        {
            serial.begin(baud, SERIAL_8N1);
        }
        return true;
    }

    int available() override { return serial.available(); }
    int read() override { return serial.read(); }
    size_t write(uint8_t value) override { return serial.write(value); }
    size_t write(const uint8_t* buffer, size_t size) override { return serial.write(buffer, size); }
    int availableForWrite() override { return serial.availableForWrite(); }


private:
    Uart& serial;
};
