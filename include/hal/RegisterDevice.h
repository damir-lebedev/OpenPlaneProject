#pragma once
#include <Arduino.h>

#include "hal/II2CBus.h"  // IWYU pragma: export
#include "hal/ISpiBus.h"  // IWYU pragma: export

// ============================================================
// 🔌 РЕГИСТРОВОЕ УСТРОЙСТВО
//
// Почти все датчики проекта — это "набор 8-битных регистров":
// записать значение в регистр, прочитать N байт подряд начиная с
// регистра. Разница между I2C и SPI — только в том, как этот обмен
// упакован на шине. IRegisterDevice прячет эту разницу, и драйвер
// датчика (BMP388, BME280, ICM42688, ...) пишется один раз, а шина
// выбирается при создании объекта (см. SensorSelection.h).
//
// Реализации ниже платформонезависимы: они работают поверх
// II2CBus/ISpiBus (HAL) и стандартных Arduino pinMode/digitalWrite.
// ============================================================

class IRegisterDevice
{
public:
    virtual ~IRegisterDevice() = default;

    // Подготовка линий устройства (для SPI — пин CS). Сама шина к
    // этому моменту уже инициализирована платой (IBoard::begin()).
    virtual void begin() {}

    // true, если устройство отозвалось. I2C — ACK на адрес; у SPI
    // протокольного подтверждения нет, и проверять надо регистр
    // идентификации чипа.
    virtual bool probe() = 0;

    virtual bool writeRegister(uint8_t reg, uint8_t value) = 0;

    // Запись count байт подряд, начиная с reg, одной транзакцией. Нужна
    // чипам, у которых смысл имеет именно пакет (ICM-45686: адрес
    // косвенного регистра + данные в IREG_ADDR/IREG_DATA).
    virtual bool writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) = 0;

    // Читает count байт подряд, начиная с reg. false — устройство не
    // ответило; буфер в этом случае не трогается.
    virtual bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) = 0;

    // Значение регистра или -1, если устройство не ответило.
    int readRegister(uint8_t reg)
    {
        uint8_t value;
        return readRegisters(reg, &value, 1) ? value : -1;
    }

    // Следующее место, где может быть чип (другая шина или адрес, где
    // кто-то ответил): ответил чужой чип — драйвер его не опознал, и
    // SensorChain пробует дальше. false — мест больше нет.
    virtual bool nextPlacement() { return false; }

    // Шина I2C (а не SPI): драйверам, у которых настройки интерфейса разные.
    virtual bool isI2c() const { return false; }

    // Где устройство, для консоли: "I2C PB11/PB10 0x6A", "SPI CS 28".
    virtual void describe(char* out, size_t size) const = 0;
};


// Устройство на шине I2C по 7-битному адресу.
//
// Шин может быть несколько (I2cBusList, обычно — все шины платы,
// IBoard::i2cBuses()): модуль можно повесить на любую, begin() сам
// найдёт, где он ответил. alternateAddress — второй адрес того же чипа
// (выбирается ногой SDO/SA0/AD0, и у модулей разных производителей она
// подтянута по-разному); 0 — запасного нет.
//
// Места перебираются по порядку: шина 1 основной адрес, шина 1
// запасной, шина 2 основной... begin() встаёт на первое, где кто-то
// ответил (никто — на самое первое, драйвер просто не опознает чип).
// Ответил чужой чип с тем же адресом (MPU6050 и ICM-45686 — оба 0x68) —
// nextPlacement() переходит к следующему ответившему месту.
class I2cRegisterDevice : public IRegisterDevice
{
public:

    I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)
        : I2cRegisterDevice(I2cBusList(i2cBus), deviceAddress, alternateAddress)
    {
    }

    I2cRegisterDevice(const I2cBusList& busList, uint8_t deviceAddress, uint8_t alternateAddress = 0)
        : buses(busList),
          addresses{ deviceAddress, alternateAddress },
          bus(busList.count ? busList.bus[0] : nullptr),
          address(deviceAddress)
    {
    }

    // Драйвер зовёт begin() при каждом опознании чипа. Место ищется
    // заново, только если на текущем никто не отвечает: иначе повторный
    // begin() после nextPlacement() вернул бы поиск к первому месту.
    void begin() override
    {
        if (searched && bus && bus->probe(address)) return;
        searched = true;
        if (!select(0)) place(0);
    }

    bool nextPlacement() override
    {
        return select(static_cast<uint8_t>(current + 1));
    }

    bool probe() override
    {
        return bus && bus->probe(address);
    }

    bool writeRegister(uint8_t reg, uint8_t value) override
    {
        return bus && bus->writeRegister(address, reg, value);
    }

    bool writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) override
    {
        return bus && bus->writeRegisters(address, reg, data, count);
    }

    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) override
    {
        return bus && bus->readRegisters(address, reg, buffer, count);
    }

    void describe(char* out, size_t size) const override
    {
        snprintf(out, size, "I2C %s 0x%02X", bus ? bus->name() : "-", static_cast<unsigned>(address));
    }

    bool isI2c() const override { return true; }

    uint8_t getAddress() const { return address; }
    II2CBus* getBus() const { return bus; }


private:

    I2cBusList buses;
    uint8_t addresses[2];
    II2CBus* bus;
    uint8_t address;
    uint8_t current = 0;
    bool searched = false;

    uint8_t placementCount() const { return static_cast<uint8_t>(buses.count * 2); }

    void place(uint8_t index)
    {
        if (index >= placementCount()) return;
        current = index;
        bus = buses.bus[index / 2];
        address = addresses[index % 2];
    }

    // Первое место с номером >= from, где кто-то ответил.
    bool select(uint8_t from)
    {
        for (uint8_t i = from; i < placementCount(); ++i)
        {
            const uint8_t a = addresses[i % 2];
            if (a == 0) continue;
            if (buses.bus[i / 2]->probe(a))
            {
                place(i);
                return true;
            }
        }
        return false;
    }
};


// Устройство на шине SPI со своим пином CS. Чтение — адрес регистра
// с битом 0x80, запись — со сброшенным битом 7 (так у всех датчиков
// Bosch/InvenSense в проекте). dummyReadBytes — сколько байт после
// адреса чип отдаёт "мусором" перед данными: у BMP388 в режиме SPI
// это 1 байт (датащит BMP388, §5.3.2), у ICM42688 — 0.
class SpiRegisterDevice : public IRegisterDevice
{
public:

    SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin,
                      uint32_t clockFrequencyHz = 8000000, uint8_t dummyBytesBeforeData = 0,
                      uint8_t mode = 0)
        : bus(spiBus),
          csPin(chipSelectPin),
          clockHz(clockFrequencyHz),
          dummyReadBytes(dummyBytesBeforeData),
          spiMode(mode)
    {
    }

    void begin() override
    {
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
    }

    // По SPI нет ACK — существование чипа проверяет драйвер по
    // регистру идентификации.
    bool probe() override
    {
        return true;
    }

    void describe(char* out, size_t size) const override
    {
        snprintf(out, size, "SPI CS %u", static_cast<unsigned>(csPin));
    }

    bool writeRegister(uint8_t reg, uint8_t value) override
    {
        return writeRegisters(reg, &value, 1);
    }

    bool writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) override
    {
        select();
        bus.transfer(reg & 0x7F);
        for (uint8_t i = 0; i < count; ++i)
        {
            bus.transfer(data[i]);
        }
        deselect();
        return true;
    }

    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) override
    {
        select();
        bus.transfer(reg | 0x80);
        for (uint8_t i = 0; i < dummyReadBytes; ++i)
        {
            bus.transfer(0x00);
        }
        for (uint8_t i = 0; i < count; ++i)
        {
            buffer[i] = bus.transfer(0x00);
        }
        deselect();
        return true;
    }


private:

    ISpiBus& bus;
    uint8_t csPin;
    uint32_t clockHz;
    uint8_t dummyReadBytes;
    uint8_t spiMode;

    void select()
    {
        bus.beginTransaction(clockHz, spiMode);
        digitalWrite(csPin, LOW);
    }

    void deselect()
    {
        digitalWrite(csPin, HIGH);
        bus.endTransaction();
    }
};
