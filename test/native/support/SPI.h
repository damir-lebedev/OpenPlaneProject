#pragma once

// ============================================================
// Нативная замена SPIClass из Arduino core ESP32 2.0.x.
//
// Симулированные устройства fake::SpiDevice подключаются к шине по
// номеру пина CS (attach()). Обмен идёт с тем устройством, чей CS
// сейчас в LOW (digitalWrite из esp32-hal-fake.h); начало и конец
// выборки устройство узнаёт через select()/deselect().
//
// fake::SpiRegisterMapDevice — датчик Bosch/InvenSense: первый байт
// — адрес регистра, бит 0x80 — чтение; перед данными чтения —
// dummyBytes "мусорных" байт (у BMP388 — 1).
// ============================================================

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "Arduino.h"

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
#define LSBFIRST 0
#define MSBFIRST 1

class SPISettings
{
public:
    SPISettings() : clock(1000000), bitOrder(MSBFIRST), dataMode(SPI_MODE0) {}
    SPISettings(uint32_t clockFreq, uint8_t order, uint8_t mode) : clock(clockFreq), bitOrder(order), dataMode(mode) {}

    uint32_t clock;
    uint8_t bitOrder;
    uint8_t dataMode;
};

namespace fake
{
    class SpiDevice
    {
    public:
        virtual ~SpiDevice() = default;
        virtual void select() {}
        virtual void deselect() {}
        virtual uint8_t transfer(uint8_t mosi) = 0;
    };

    class SpiRegisterMapDevice : public SpiDevice
    {
    public:
        uint8_t regs[256] = {};
        uint8_t dummyBytes = 0;
        std::vector<std::pair<uint8_t, uint8_t>> writes;

        void select() override
        {
            state = State::Address;
            dummyLeft = 0;
        }

        uint8_t transfer(uint8_t mosi) override
        {
            switch (state)
            {
                case State::Address:
                    address = mosi & 0x7F;
                    reading = (mosi & 0x80) != 0;
                    dummyLeft = reading ? dummyBytes : 0;
                    state = State::Data;
                    return 0xFF;

                case State::Data:
                    if (!reading)
                    {
                        regs[address] = mosi;
                        writes.emplace_back(address, mosi);
                        address++;
                        return 0xFF;
                    }
                    if (dummyLeft > 0)
                    {
                        dummyLeft--;
                        return 0xA5;   // "мусор" перед данными
                    }
                    return regs[address++];
            }
            return 0xFF;
        }

        int lastWrite(uint8_t reg) const
        {
            for (auto it = writes.rbegin(); it != writes.rend(); ++it)
            {
                if (it->first == reg) return it->second;
            }
            return -1;
        }

    private:
        enum class State : uint8_t { Address, Data };
        State state = State::Address;
        uint8_t address = 0;
        bool reading = false;
        uint8_t dummyLeft = 0;
    };
}

class SPIClass
{
public:
    explicit SPIClass(uint8_t spiBus = 0) : bus(spiBus) {}

    void begin(int8_t sckPin = -1, int8_t misoPin = -1, int8_t mosiPin = -1, int8_t ssPin = -1)
    {
        sck = sckPin;
        miso = misoPin;
        mosi = mosiPin;
        ss = ssPin;
        started = true;
    }

    void beginTransaction(SPISettings settings)
    {
        lastSettings = settings;
        inTransaction = true;
        transactions++;
    }

    void endTransaction()
    {
        inTransaction = false;
        for (auto& entry : devices)
        {
            if (entry.second.selected && digitalRead(entry.first) == HIGH)
            {
                entry.second.selected = false;
                entry.second.device->deselect();
            }
        }
    }

    uint8_t transfer(uint8_t data)
    {
        transfers++;
        for (auto& entry : devices)
        {
            if (digitalRead(entry.first) != LOW) continue;
            if (!entry.second.selected)
            {
                entry.second.selected = true;
                entry.second.device->select();
            }
            return entry.second.device->transfer(data);
        }
        return 0xFF;
    }

    // --- управление из тестов ---

    void attach(uint8_t csPin, fake::SpiDevice* device) { devices[csPin] = Slot{ device, false }; }

    int8_t sckPin() const { return sck; }
    int8_t misoPin() const { return miso; }
    int8_t mosiPin() const { return mosi; }
    int8_t ssPin() const { return ss; }
    bool isStarted() const { return started; }
    bool isInTransaction() const { return inTransaction; }
    const SPISettings& settings() const { return lastSettings; }
    uint32_t transactionCount() const { return transactions; }
    uint32_t transferCount() const { return transfers; }

    void resetFake()
    {
        devices.clear();
        sck = miso = mosi = ss = -1;
        started = false;
        inTransaction = false;
        lastSettings = SPISettings();
        transactions = 0;
        transfers = 0;
    }

private:
    struct Slot
    {
        fake::SpiDevice* device;
        bool selected;
    };

    uint8_t bus;
    std::map<uint8_t, Slot> devices;
    int8_t sck = -1;
    int8_t miso = -1;
    int8_t mosi = -1;
    int8_t ss = -1;
    bool started = false;
    bool inTransaction = false;
    SPISettings lastSettings;
    uint32_t transactions = 0;
    uint32_t transfers = 0;
};

inline SPIClass SPI;
