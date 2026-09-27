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
    int readRegister(uint8_t reg);
};


// Устройство на шине I2C по 7-битному адресу. alternateAddress — второй
// адрес того же чипа (выбирается ногой SDO/SA0/AD0, и у модулей разных
// производителей она подтянута по-разному): если по основному адресу
// никто не ответил, а по запасному ответили — begin() переключается на
// запасной. 0 — запасного нет.
class I2cRegisterDevice : public IRegisterDevice
{
public:

    I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0);

    void begin() override;

    bool probe() override;

    bool writeRegister(uint8_t reg, uint8_t value) override;

    bool writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) override;

    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) override;

    uint8_t getAddress() const { return address; }


private:

    II2CBus& bus;
    uint8_t address;
    uint8_t alternate;
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
                      uint8_t mode = 0);

    void begin() override;

    // По SPI нет ACK — существование чипа проверяет драйвер по
    // регистру идентификации.
    bool probe() override;

    bool writeRegister(uint8_t reg, uint8_t value) override;

    bool writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) override;

    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) override;


private:

    ISpiBus& bus;
    uint8_t csPin;
    uint32_t clockHz;
    uint8_t dummyReadBytes;
    uint8_t spiMode;

    void select();

    void deselect();
};
