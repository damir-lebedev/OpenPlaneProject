#pragma once
#include <Arduino.h>

// ============================================================
// 🔌 АБСТРАКЦИЯ ШИНЫ SPI
//
// Интерфейс намеренно НЕ занимается CS-пинами: на одной шине
// сидит несколько устройств (ICM42688, BMP388), у каждого свой
// CS, и если увести CS сюда — интерфейс придётся параметризовать
// им на каждый вызов или превращать в мульти-девайсный менеджер.
// Вместо этого каждый датчик сам делает pinMode/digitalWrite на
// свой CS (digitalWrite — стандартная Arduino-функция, одинаково
// работает на ESP32 и STM32, отдельной абстракции не требует) и
// оборачивает обращение к регистру в
// beginTransaction()/transfer()/endTransaction().
// ============================================================

class ISpiBus
{
public:
    virtual ~ISpiBus() = default;

    // Настраивает SCK/MISO/MOSI. Пины фиксируются при создании
    // конкретной реализации (см. Esp32SpiBus), не здесь.
    virtual void begin() = 0;

    // spiMode: 0..3 (CPOL/CPHA), как в SPISettings.
    virtual void beginTransaction(uint32_t clockHz, uint8_t spiMode) = 0;
    virtual uint8_t transfer(uint8_t data) = 0;
    virtual void endTransaction() = 0;
};
