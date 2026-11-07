#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "hal/II2CBus.h"
#include "hal/Rtos.h"

// ============================================================
// Реализация II2CBus для STM32 (STM32duino 3.x) — обёртка над
// TwoWire. На железе (с датчиками) ещё не проверялась.
//
// Как и у Esp32I2CBus, пины и частота задаются один раз в
// конструкторе и применяются только в begin(). Контроллер (I2C1,
// I2C2...) ядро выбирает само по паре пинов SDA/SCL.
//
// Таймаут транзакции у STM32duino — макрос I2C_TIMEOUT_TICK (мс,
// по умолчанию 100), а не метод объекта: задаётся флагом сборки в
// platformio.ini (env stm32h743), по той же причине, что
// TIMEOUT_MS у Esp32I2CBus, — зависшая транзакция не должна
// останавливать полётный цикл.
//
// Зависшая шина: помеха на длинных проводах посреди чтения — и ведомый
// (датчик) держит SDA прижатой, ждёт тактов, которых контроллер уже не
// даст. Все транзакции на шине дальше кончаются ошибкой, пока не
// передёрнуть питание. Поэтому на ошибке шины (код 4: таймаут, занята,
// сбой) recover() прощёлкивает SCL вручную (до 16 тактов, пока ведомый
// не отпустит SDA), ставит STOP и заново поднимает контроллер. Датчик
// возвращается сам: первое удачное чтение снимает его счётчик ошибок.
// ============================================================

class Stm32I2CBus : public II2CBus
{
public:

    Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)
        : wire(bus),
          sda(sdaPin),
          scl(sclPin),
          frequency(frequencyHz)
    {
    }

    void begin() override
    {
        mutex.create();
        // Имя по пинам: "PB11/PB10" (SDA/SCL). PinName у STM32duino — порт << 4 | номер.
        const uint32_t s = static_cast<uint32_t>(digitalPinToPinName(sda));
        const uint32_t c = static_cast<uint32_t>(digitalPinToPinName(scl));
        snprintf(label, sizeof(label), "P%c%lu/P%c%lu", static_cast<char>('A' + ((s >> 4) & 0xF)),
                 static_cast<unsigned long>(s & 0xF), static_cast<char>('A' + ((c >> 4) & 0xF)),
                 static_cast<unsigned long>(c & 0xF));
        start();
    }

    void setClock(uint32_t hz) override { wire.setClock(hz); }

    void beginTransmission(uint8_t address) override { wire.beginTransmission(address); }
    size_t write(uint8_t data) override { return wire.write(data); }
    size_t write(const uint8_t* data, size_t length) override { return wire.write(data, length); }
    uint8_t endTransmission(bool sendStop = true) override
    {
        const uint8_t result = wire.endTransmission(sendStop);
        if (result == BUS_ERROR) recoverAfterError();
        return result;
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) override
    {
        const uint8_t n = static_cast<uint8_t>(wire.requestFrom(address, static_cast<size_t>(quantity)));
        if (n < quantity && digitalRead(sda) == LOW) recoverAfterError();
        return n;
    }

    // Освободить шину: ведомый держит SDA — такты на SCL, пока не отпустит,
    // затем STOP и контроллер заново. true — линии свободны.
    bool recover() override
    {
        wire.end();
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, OUTPUT_OPEN_DRAIN);
        digitalWrite(scl, HIGH);
        delayMicroseconds(HALF_CLOCK_US);
        for (uint8_t i = 0; i < 16 && digitalRead(sda) == LOW; ++i)
        {
            digitalWrite(scl, LOW);
            delayMicroseconds(HALF_CLOCK_US);
            digitalWrite(scl, HIGH);
            delayMicroseconds(HALF_CLOCK_US);
        }
        // STOP: SDA снизу вверх при высокой SCL.
        pinMode(sda, OUTPUT_OPEN_DRAIN);
        digitalWrite(sda, LOW);
        delayMicroseconds(HALF_CLOCK_US);
        digitalWrite(scl, HIGH);
        delayMicroseconds(HALF_CLOCK_US);
        digitalWrite(sda, HIGH);
        delayMicroseconds(HALF_CLOCK_US);
        pinMode(sda, INPUT_PULLUP);
        const bool free = digitalRead(sda) == HIGH && digitalRead(scl) == HIGH;
        start();
        recoveryCount++;
        return free;
    }

    uint32_t recoveries() const override { return recoveryCount; }
    bool linesHigh() const override { return digitalRead(sda) == HIGH && digitalRead(scl) == HIGH; }

    int available() override { return wire.available(); }
    int read() override { return wire.read(); }

    void lock() override { mutex.take(); }
    void unlock() override { mutex.give(); }
    const char* name() const override { return label; }


private:

    TwoWire& wire;
    pin_size_t sda;
    pin_size_t scl;
    uint32_t frequency;
    Rtos::RecursiveMutex mutex;
    char label[16] = "I2C";
    uint32_t recoveryCount = 0;
    uint32_t lastRecoveryMs = 0;

    static constexpr uint8_t BUS_ERROR = 4;          // TwoWire: таймаут, шина занята, сбой
    static constexpr uint32_t HALF_CLOCK_US = 5;     // ~100 кГц ручных тактов
    static constexpr uint32_t RECOVERY_PAUSE_MS = 20;  // не чаще: отсутствующий датчик не должен молотить шину

    void start()
    {
        // setSDA()/setSCL() действуют только до begin().
        wire.setSDA(sda);
        wire.setSCL(scl);
        wire.begin();
        wire.setClock(frequency);
    }

    void recoverAfterError()
    {
        const uint32_t now = millis();
        if (recoveryCount && now - lastRecoveryMs < RECOVERY_PAUSE_MS) return;
        lastRecoveryMs = now;
        recover();
    }
};
