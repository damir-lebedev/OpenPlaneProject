#pragma once
#include <Arduino.h>

#include "hal/II2CBus.h"
#include "hal/RegisterDevice.h"

// ============================================================
// ВСПОМОГАТЕЛЬНАЯ ШИНА LSM6DSV (sensor hub) КАК ОБЫЧНАЯ ШИНА I2C
//
// На модулях "LSM6DSV + QMC6309" компас распаян не на шине платы, а на
// вспомогательной шине LSM6DSV (SDx/SCx): хост его не видит, к нему
// ходит только встроенный мастер LSM6DSV. Проверено на стенде: через
// мастер QMC6309 отвечает на 0x7C (chip ID 0x90), напрямую — никто;
// pass-through у этого модуля шину не соединяет.
//
// Этот класс — II2CBus поверх мастера: плата добавляет его к своим
// шинам (IBoard::setAuxI2c()), и компас ищется на нём так же, как на
// остальных (I2cRegisterDevice) — драйвер QMC6309 не меняется.
//
//   • Запись регистра — один цикл мастера с SLV0 на запись (не WRITE_ONCE:
//     после него мастер переставал заканчивать чтения). Только при настройке.
//   • Чтение — разовое: SLV0 настраивается на адрес/регистр/длину,
//     мастер включается на один цикл, данные — в SENSOR_HUB_1..
//     Ждать цикл ~2 мс: годится для опознания и настройки.
//   • Повторяющееся чтение блока от 2 байт (данные компаса в полёте)
//     запоминается: мастер дальше читает его САМ каждый цикл (до 4 таких
//     блоков — SLV0..SLV3), а полётный цикл забирает готовые байты из
//     SENSOR_HUB одной транзакцией, без ожидания.
//
// Мастер запускается данными акселерометра — LSM6DSV должен быть уже
// настроен (IMU опознаётся раньше компаса, sensors/SensorSelection.h).
// Регистры — lsm6dsv16x_reg.h (банк sensor hub: FUNC_CFG_ACCESS.SHUB_REG_ACCESS).
// ============================================================

class Lsm6dsvAuxBus : public II2CBus
{
public:

    explicit Lsm6dsvAuxBus(IRegisterDevice& lsm6dsv)
        : imu(lsm6dsv)
    {
    }

    void begin() override {}
    void setClock(uint32_t) override {}

    void beginTransmission(uint8_t address) override
    {
        txAddress = address;
        txLength = 0;
    }

    size_t write(uint8_t data) override
    {
        if (txLength >= sizeof(tx)) return 0;
        tx[txLength++] = data;
        return 1;
    }

    size_t write(const uint8_t* data, size_t length) override
    {
        size_t n = 0;
        while (n < length && write(data[n])) ++n;
        return n;
    }

    // 0 — ок, 2 — нет ответа на адрес (как у TwoWire).
    uint8_t endTransmission(bool sendStop = true) override
    {
        if (txLength == 0)
        {
            uint8_t dummy;
            const bool ack = readOnce(txAddress, 0x00, &dummy, 1);   // проба: ответил ли кто-то
            resumeMirror();
            return ack ? 0 : 2;
        }
        pendingRegister = tx[0];
        if (txLength == 1 || !sendStop) return 0;   // указатель регистра, чтение — в requestFrom()
        for (uint8_t i = 1; i < txLength; ++i)
        {
            if (!writeOnce(txAddress, static_cast<uint8_t>(tx[0] + i - 1), tx[i])) return 3;
        }
        return 0;
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) override
    {
        rxLength = rxPosition = 0;
        if (quantity == 0 || quantity > sizeof(rx)) return 0;
        if (!readBlock(address, pendingRegister, rx, quantity)) return 0;
        rxLength = quantity;
        return quantity;
    }

    int available() override { return rxLength - rxPosition; }
    int read() override { return rxPosition < rxLength ? rx[rxPosition++] : -1; }

    const char* name() const override { return "за LSM6DSV"; }

    // Сколько блоков мастер читает сам (для тестов и консоли).
    uint8_t mirroredBlocks() const { return slotCount; }


private:

    // Основная страница
    static constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;
    static constexpr uint8_t SHUB_REG_ACCESS = 0x40;
    static constexpr uint8_t STATUS_MASTER_MAINPAGE = 0x48;   // те же флаги, что STATUS_MASTER
    // Банк sensor hub
    static constexpr uint8_t SENSOR_HUB_1 = 0x02;
    static constexpr uint8_t MASTER_CONFIG = 0x14;
    static constexpr uint8_t SLV0_ADD = 0x15;          // SLVn: 0x15 + 3n — адрес, регистр, CONFIG
    static constexpr uint8_t DATAWRITE_SLV0 = 0x21;
    static constexpr uint8_t STATUS_MASTER = 0x22;

    static constexpr uint8_t MASTER_ON = 0x04;
    static constexpr uint8_t SHUB_ODR_480HZ = 0xC0;    // SLV0_CONFIG[7:5] = 110
    static constexpr uint8_t STATUS_ENDOP = 0x01;
    static constexpr uint8_t STATUS_SLV0_NACK = 0x08;

    static constexpr uint8_t MAX_SLOTS = 4;
    static constexpr uint8_t MAX_BLOCK = 7;            // SLVn_CONFIG.numop — 3 бита
    static constexpr uint8_t WAIT_TRIES = 25;          // по 1 мс: цикл мастера ~2 мс

    struct Slot
    {
        uint8_t address;
        uint8_t reg;
        uint8_t length;
    };

    IRegisterDevice& imu;
    uint8_t txAddress = 0;
    uint8_t tx[8] = {};
    uint8_t txLength = 0;
    uint8_t pendingRegister = 0;
    uint8_t rx[16] = {};
    uint8_t rxLength = 0;
    uint8_t rxPosition = 0;
    Slot slots[MAX_SLOTS] = {};
    uint8_t slotCount = 0;
    uint8_t stalledCycles = 0;   // подряд циклов мастера без конца

    // Мастер раз за разом не заканчивает цикл (ведомый держит шину) —
    // каждая попытка стоила бы полётному циклу десятки мс. После
    // MAX_STALLS подряд шина считается мёртвой до перезагрузки.
    static constexpr uint8_t MAX_STALLS = 3;
    bool dead() const { return stalledCycles >= MAX_STALLS; }

    bool enterBank() { return imu.writeRegister(REG_FUNC_CFG_ACCESS, SHUB_REG_ACCESS); }
    bool leaveBank() { return imu.writeRegister(REG_FUNC_CFG_ACCESS, 0x00); }

    bool readBlock(uint8_t address, uint8_t reg, uint8_t* out, uint8_t length)
    {
        uint8_t offset = 0;
        for (uint8_t i = 0; i < slotCount; ++i)
        {
            const Slot& s = slots[i];
            if (s.address == address && s.reg == reg && s.length == length)
            {
                // Мастер читает этот блок сам — забрать готовое.
                const bool ok = enterBank() && imu.readRegisters(static_cast<uint8_t>(SENSOR_HUB_1 + offset), out, length);
                return leaveBank() && ok;
            }
            offset = static_cast<uint8_t>(offset + s.length);
        }
        if (!readOnce(address, reg, out, length)) return false;
        if (length >= 2 && length <= MAX_BLOCK && slotCount < MAX_SLOTS)
        {
            slots[slotCount++] = Slot{ address, reg, length };
        }
        resumeMirror();
        return true;
    }

    // Мастер выключен, ждём конца его цикла.
    bool stopMaster()
    {
        const bool ok = imu.writeRegister(MASTER_CONFIG, 0x00);
        delay(1);
        return ok;
    }

    // Один цикл мастера: включить, дождаться флага, выключить. Вызывается
    // в банке и возвращает в банк. Флаг ждём на основной странице
    // (STATUS_MASTER_MAINPAGE, как в примерах ST), не заходя в банк:
    // заходы в банк посреди цикла мешали мастеру — длинные чтения после
    // записи не заканчивались.
    int runCycle(uint8_t masterConfig, uint8_t doneFlag)
    {
        if (dead()) return -1;
        imu.readRegister(STATUS_MASTER);   // сбросить старые флаги
        if (!imu.writeRegister(MASTER_CONFIG, masterConfig)) return -1;
        leaveBank();
        int status = -1;
        for (uint8_t i = 0; i < WAIT_TRIES; ++i)
        {
            delay(1);
            status = imu.readRegister(STATUS_MASTER_MAINPAGE);
            if (status >= 0 && (status & doneFlag)) break;
        }
        enterBank();
        stopMaster();
        if (status >= 0 && (status & STATUS_ENDOP)) stalledCycles = 0;
        else if (stalledCycles < MAX_STALLS && ++stalledCycles == MAX_STALLS)
        {
            Serial.println("I2C за LSM6DSV: мастер не заканчивает обмен — шина занята ведомым, до перезагрузки не опрашивается");
        }
        return status;
    }

    bool readOnce(uint8_t address, uint8_t reg, uint8_t* out, uint8_t length)
    {
        if (length == 0 || length > MAX_BLOCK || dead()) return false;
        if (!enterBank()) return false;
        stopMaster();
        bool ok = imu.writeRegister(SLV0_ADD, static_cast<uint8_t>((address << 1) | 1)) &&
                  imu.writeRegister(SLV0_ADD + 1, reg) &&
                  imu.writeRegister(SLV0_ADD + 2, static_cast<uint8_t>(SHUB_ODR_480HZ | length));
        const int status = ok ? runCycle(MASTER_ON, STATUS_ENDOP) : -1;
        ok = status >= 0 && (status & STATUS_ENDOP) && !(status & STATUS_SLV0_NACK) &&
             imu.readRegisters(SENSOR_HUB_1, out, length);
        leaveBank();
        return ok;   // запомненные блоки снова запускает вызывающий (resumeMirror())
    }

    bool writeOnce(uint8_t address, uint8_t reg, uint8_t value)
    {
        if (dead() || !enterBank()) return false;
        stopMaster();
        bool ok = imu.writeRegister(SLV0_ADD, static_cast<uint8_t>(address << 1)) &&
                  imu.writeRegister(SLV0_ADD + 1, reg) &&
                  imu.writeRegister(SLV0_ADD + 2, SHUB_ODR_480HZ) &&
                  imu.writeRegister(DATAWRITE_SLV0, value);
        // Без WRITE_ONCE: после записи в этом режиме мастер больше не
        // заканчивал ни одного чтения (до снятия питания — SW_RESET LSM6DSV
        // банк sensor hub не сбрасывает). Обычный цикл пишет так же, а
        // повтор записи регистра настройки безвреден.
        const int status = ok ? runCycle(MASTER_ON, STATUS_ENDOP) : -1;
        ok = status >= 0 && (status & STATUS_ENDOP) && !(status & STATUS_SLV0_NACK);
        leaveBank();
        resumeMirror();
        return ok;
    }

    // Запомненные блоки — в SLV0..SLVn, мастер читает их сам каждый цикл.
    void resumeMirror()
    {
        if (slotCount == 0) return;
        if (!enterBank()) return;
        stopMaster();
        for (uint8_t i = 0; i < slotCount; ++i)
        {
            const uint8_t base = static_cast<uint8_t>(SLV0_ADD + 3 * i);
            imu.writeRegister(base, static_cast<uint8_t>((slots[i].address << 1) | 1));
            imu.writeRegister(static_cast<uint8_t>(base + 1), slots[i].reg);
            imu.writeRegister(static_cast<uint8_t>(base + 2),
                              static_cast<uint8_t>((i == 0 ? SHUB_ODR_480HZ : 0) | slots[i].length));
        }
        imu.writeRegister(MASTER_CONFIG, static_cast<uint8_t>(MASTER_ON | (slotCount - 1)));
        leaveBank();
    }
};
