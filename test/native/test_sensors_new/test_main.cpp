// ============================================================
// Новые датчики: LSM6DSV (16X/32X), ICM-45686, QMC6309, SPL06-001,
// BMP581 — и самодельная трубка Пито на двух барометрах.
//
// Каждый чип — регистровый фейк: драйвер видит те же регистры, что
// и на настоящем чипе (раскладка по датащитам/официальным драйверам,
// см. шапки драйверов). Проверяется: опознание (в том числе по
// запасному I2C-адресу), что записано в регистры настройки, разбор
// данных (порядок байт, знак, масштаб) и отказы.
//
// Трубка Пито дополнительно гоняется на "полёте": профиль скорости и
// высоты, два барометра с разной частотой, шумом и смещением.
//
// Запуск: pio test -e native -f native/test_sensors_new
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <cmath>
#include <map>
#include <random>

#include "sensors/SensorSelection.h"
#include "sensors/airspeed/PitotDualBaroAirspeed.h"
#include "sensors/baro/BMP581_Sensor.h"
#include "sensors/baro/SPL06_Sensor.h"
#include "sensors/imu/ICM45686_Sensor.h"
#include "sensors/imu/LSM6DSV_Sensor.h"
#include "sensors/mag/QMC6309_Sensor.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    // I2C-стенд, где чип отвечает только по запасному адресу.
    struct AltI2cRig
    {
        TwoWire wire{ 7 };
        Esp32I2CBus bus{ wire, 8, 9 };
        fake::RegisterMapDevice chip;
        I2cRegisterDevice device;

        AltI2cRig(uint8_t primary, uint8_t alternate, uint8_t actual)
            : device(bus, primary, alternate)
        {
            wire.attach(actual, &chip);
            bus.begin();
        }
    };

    void putLe16(uint8_t* regs, uint8_t reg, int16_t value)
    {
        regs[reg] = static_cast<uint8_t>(value & 0xFF);
        regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
    }
}

// ------------------------------------------------------------
// LSM6DSV
// ------------------------------------------------------------

namespace
{
    // SW_RESET сбрасывается чипом сам; CTRL8 после сброса — признак варианта.
    void emulateLsm6dsv(fake::RegisterMapDevice& chip, bool variant32x)
    {
        chip.regs[0x0F] = 0x70;
        chip.regs[0x17] = variant32x ? 0x04 : 0x00;
        chip.onRegisterWrite = [&chip](uint8_t reg, uint8_t value) {
            if (reg == 0x12 && (value & 0x01)) chip.regs[0x12] = value & ~0x01;
        };
    }

    // Отсчёт: gyro X (°/с), accel Z (g), температура (°C).
    void setLsm6dsvSample(uint8_t* regs, float gyroXDps, float accelZ, float temperatureC)
    {
        putLe16(regs, 0x20, static_cast<int16_t>((temperatureC - 25.0f) * 256.0f));
        putLe16(regs, 0x22, static_cast<int16_t>(gyroXDps * 1000.0f / 70.0f));
        putLe16(regs, 0x24, 0);
        putLe16(regs, 0x26, 0);
        putLe16(regs, 0x28, 0);
        putLe16(regs, 0x2A, 0);
        putLe16(regs, 0x2C, static_cast<int16_t>(accelZ * 1000.0f / 0.488f));
    }
}

void test_lsm6dsv_found_at_alternate_address_and_configured()
{
    AltI2cRig rig(LSM6DSV_Sensor::DEFAULT_ADDRESS, LSM6DSV_Sensor::ALTERNATE_ADDRESS, 0x6B);
    emulateLsm6dsv(rig.chip, false);
    setLsm6dsvSample(rig.chip.regs, 0, 1, 25);

    LSM6DSV_Sensor imu(rig.device);
    TEST_ASSERT_TRUE(imu.begin());
    TEST_ASSERT_EQUAL_HEX8(0x6B, rig.device.getAddress());
    TEST_ASSERT_FALSE(imu.isVariant32x());
    TEST_ASSERT_TRUE(contains(takeSerial(), "LSM6DSV: подключён (LSM6DSV/16X)"));
    TEST_ASSERT_EQUAL_STRING("LSM6DSV", imu.getSensorType());

    TEST_ASSERT_EQUAL_HEX8(0x44, rig.chip.lastWrite(0x12));   // BDU | IF_INC
    TEST_ASSERT_EQUAL_HEX8(0x44, rig.chip.lastWrite(0x15));   // ±2000 °/с, LPF1 "strong"
    TEST_ASSERT_EQUAL_HEX8(0x01, rig.chip.lastWrite(0x16));   // LPF1 включён
    TEST_ASSERT_EQUAL_HEX8(0x43, rig.chip.lastWrite(0x17));   // ±16g, LPF2 ODR/20
    TEST_ASSERT_EQUAL_HEX8(0x08, rig.chip.lastWrite(0x18));   // LPF2 включён
    TEST_ASSERT_EQUAL_HEX8(0x09, rig.chip.lastWrite(0x10));   // 960 Гц, high performance
    TEST_ASSERT_EQUAL_HEX8(0x09, rig.chip.lastWrite(0x11));

    imu.calibrate();
    setLsm6dsvSample(rig.chip.regs, 20, 1, 35);
    fake::advanceMs(2);
    imu.update();
    // Config::IMU_ROTATION_CW_DEG = 90: ось X чипа смотрит вправо — поворот вокруг
    // неё это тангаж самолёта.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, imu.getImuData().gyroY);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, imu.getImuData().accelZ);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 35.0f, imu.getImuData().temperature);
}

void test_lsm6dsv32x_gets_its_own_16g_code()
{
    I2cRig rig(LSM6DSV_Sensor::DEFAULT_ADDRESS);
    emulateLsm6dsv(rig.chip, true);
    LSM6DSV_Sensor imu(rig.device);
    TEST_ASSERT_TRUE(imu.begin());
    TEST_ASSERT_TRUE(imu.isVariant32x());
    // Код 10 + бит варианта: у 32X это ±16g (у 16X код 11 = ±16g).
    TEST_ASSERT_EQUAL_HEX8(0x46, rig.chip.lastWrite(0x17));
    TEST_ASSERT_TRUE(contains(takeSerial(), "LSM6DSV32X"));
}

void test_lsm6dsv_over_spi()
{
    SpiRig rig(Config::PIN_SPI_CS_IMU, 0);
    rig.chip.regs[0x0F] = 0x70;
    rig.chip.onRegisterWrite = [&rig](uint8_t reg, uint8_t value) {
        if (reg == 0x12 && (value & 0x01)) rig.chip.regs[0x12] = value & ~0x01;
    };
    SpiRegisterDevice device = LSM6DSV_Sensor::spiDevice(rig.bus, Config::PIN_SPI_CS_IMU);
    LSM6DSV_Sensor imu(device);
    TEST_ASSERT_TRUE(imu.begin());
    TEST_ASSERT_EQUAL_UINT32(8000000, SPI.settings().clock);
    TEST_ASSERT_EQUAL_HEX8(0x09, rig.chip.lastWrite(0x11));
}

void test_lsm6dsv_failure_paths()
{
    I2cRig wrong(0x6A);
    wrong.chip.regs[0x0F] = 0x6C;   // LSM6DSO и т.п.
    LSM6DSV_Sensor a(wrong.device);
    TEST_ASSERT_FALSE(a.begin());
    TEST_ASSERT_FALSE(a.isAvailable());
    TEST_ASSERT_TRUE(contains(takeSerial(), "неверный WHO_AM_I 0x6c"));

    I2cRig absent(0x6A);
    absent.chip.present = false;
    LSM6DSV_Sensor b(absent.device);
    TEST_ASSERT_FALSE(b.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "(нет ответа)"));

    I2cRig stuck(0x6A);
    stuck.chip.regs[0x0F] = 0x70;   // SW_RESET никогда не сбрасывается
    LSM6DSV_Sensor c(stuck.device);
    TEST_ASSERT_FALSE(c.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "не вышел из сброса"));

    I2cRig readOnly(0x6A);
    emulateLsm6dsv(readOnly.chip, false);
    readOnly.chip.failWrites = true;
    LSM6DSV_Sensor d(readOnly.device);
    TEST_ASSERT_FALSE(d.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "не вышел из сброса"));

    I2cRig noCtrl8(0x6A);
    emulateLsm6dsv(noCtrl8.chip, false);
    noCtrl8.chip.failReadIf = [](uint8_t reg) { return reg == 0x17; };
    LSM6DSV_Sensor e(noCtrl8.device);
    TEST_ASSERT_FALSE(e.begin());

    I2cRig failConfig(0x6A);
    emulateLsm6dsv(failConfig.chip, false);
    failConfig.chip.onRegisterWrite = [&failConfig](uint8_t reg, uint8_t value) {
        if (reg == 0x12 && (value & 0x01)) failConfig.chip.regs[0x12] = value & ~0x01;
        if (reg == 0x12 && !(value & 0x01)) failConfig.chip.failWrites = true;   // всё после сброса
    };
    LSM6DSV_Sensor f(failConfig.device);
    TEST_ASSERT_FALSE(f.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "LSM6DSV: ошибка записи регистров"));
}

// ------------------------------------------------------------
// ICM-45686
// ------------------------------------------------------------

namespace
{
    // Косвенные регистры IPREG: запись адреса в 0x7C/0x7D, данные в 0x7E
    // (адрес после каждого байта данных растёт).
    struct IcmIndirect
    {
        std::map<uint16_t, uint8_t> mem;
        uint16_t address = 0;
        bool addressLow = false;

        void onWrite(uint8_t reg, uint8_t value)
        {
            if (reg == 0x7C) address = static_cast<uint16_t>((value << 8) | (address & 0xFF));
            else if (reg == 0x7D) address = static_cast<uint16_t>((address & 0xFF00) | value);
            else if (reg == 0x7E) mem[address++] = value;
        }
    };

    void setIcm45686Sample(uint8_t* regs, float gyroXDps, float accelZ, float temperatureC)
    {
        putLe16(regs, 0x00, 0);
        putLe16(regs, 0x02, 0);
        putLe16(regs, 0x04, static_cast<int16_t>(accelZ * 2048.0f));
        putLe16(regs, 0x06, static_cast<int16_t>(gyroXDps * 16.4f));
        putLe16(regs, 0x08, 0);
        putLe16(regs, 0x0A, 0);
        putLe16(regs, 0x0C, static_cast<int16_t>((temperatureC - 25.0f) * 132.48f));
    }
}

void test_icm45686_configures_modes_and_indirect_filters()
{
    AltI2cRig rig(ICM45686_Sensor::DEFAULT_ADDRESS, ICM45686_Sensor::ALTERNATE_ADDRESS, 0x69);
    IcmIndirect ireg;
    ireg.mem[0xA4AC] = 0x80;   // соседний бит (OIS HPF) должен сохраниться
    ireg.mem[0xA583] = 0x00;
    rig.chip.regs[0x72] = 0xE9;
    rig.chip.onRegisterWrite = [&ireg](uint8_t reg, uint8_t value) { ireg.onWrite(reg, value); };
    rig.chip.beforeRead = [&](uint8_t reg, size_t) {
        if (reg == 0x7E) rig.chip.regs[0x7E] = ireg.mem[ireg.address];
    };
    setIcm45686Sample(rig.chip.regs, 0, 1, 25);

    ICM45686_Sensor imu(rig.device);
    TEST_ASSERT_TRUE(imu.begin());
    TEST_ASSERT_EQUAL_HEX8(0x69, rig.device.getAddress());
    TEST_ASSERT_EQUAL_STRING("ICM45686", imu.getSensorType());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ICM45686: подключён"));

    TEST_ASSERT_EQUAL_HEX8(0x02, rig.chip.writes.front().second);   // первым делом SOFT_RST
    TEST_ASSERT_EQUAL_HEX8(0x7F, rig.chip.writes.front().first);
    TEST_ASSERT_EQUAL_HEX8(0x15, rig.chip.lastWrite(0x1C));   // ±2000 °/с, 1.6 кГц
    TEST_ASSERT_EQUAL_HEX8(0x15, rig.chip.lastWrite(0x1B));   // ±16g, 1.6 кГц
    TEST_ASSERT_EQUAL_HEX8(0x0F, rig.chip.lastWrite(0x10));   // gyro + accel Low Noise
    TEST_ASSERT_EQUAL_HEX8(0x84, ireg.mem[0xA4AC]);           // ODR/32, бит 7 сохранён
    TEST_ASSERT_EQUAL_HEX8(0x04, ireg.mem[0xA583]);

    imu.calibrate();
    setIcm45686Sample(rig.chip.regs, -15, 1, 40);
    fake::advanceMs(2);
    imu.update();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -15.0f, imu.getImuData().gyroY);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, imu.getImuData().accelZ);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 40.0f, imu.getImuData().temperature);
}

void test_icm45686_over_spi_and_failures()
{
    SpiRig spiRig(Config::PIN_SPI_CS_IMU, 0);
    IcmIndirect ireg;
    spiRig.chip.regs[0x72] = 0xE9;
    spiRig.chip.onRegisterWrite = [&ireg](uint8_t reg, uint8_t value) { ireg.onWrite(reg, value); };
    spiRig.chip.beforeRead = [&](uint8_t reg) {
        if (reg == 0x7E) spiRig.chip.regs[0x7E] = ireg.mem[ireg.address];
    };
    SpiRegisterDevice device = ICM45686_Sensor::spiDevice(spiRig.bus, Config::PIN_SPI_CS_IMU);
    ICM45686_Sensor spiImu(device);
    TEST_ASSERT_TRUE(spiImu.begin());
    TEST_ASSERT_EQUAL_HEX8(0x04, ireg.mem[0xA583]);

    I2cRig wrong(0x68);
    wrong.chip.regs[0x72] = 0x47;   // ICM-42688 на месте ICM-45686
    ICM45686_Sensor a(wrong.device);
    TEST_ASSERT_FALSE(a.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ICM45686: неверный WHO_AM_I 0x47"));

    I2cRig absent(0x68);
    absent.chip.present = false;
    ICM45686_Sensor b(absent.device);
    TEST_ASSERT_FALSE(b.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "(нет ответа)"));

    I2cRig readOnly(0x68);
    readOnly.chip.regs[0x72] = 0xE9;
    readOnly.chip.onRegisterWrite = [&readOnly](uint8_t reg, uint8_t) {
        if (reg == 0x7F) readOnly.chip.failWrites = true;   // после сброса запись не проходит
    };
    ICM45686_Sensor c(readOnly.device);
    TEST_ASSERT_FALSE(c.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ICM45686: ошибка записи регистров"));

    I2cRig noIreg(0x68);
    noIreg.chip.regs[0x72] = 0xE9;
    noIreg.chip.failReadIf = [](uint8_t reg) { return reg == 0x7E; };
    ICM45686_Sensor d(noIreg.device);
    TEST_ASSERT_FALSE(d.begin());
}

// ------------------------------------------------------------
// QMC6309
// ------------------------------------------------------------

void test_qmc6309_waits_for_nvm_and_reads_field()
{
    I2cRig rig(QMC6309_Sensor::DEFAULT_ADDRESS);
    rig.chip.regs[0x00] = 0x90;
    rig.chip.regs[0x09] = 0x18;   // NVM_RDY | NVM_LOAD_DONE
    QMC6309_Sensor qmc(rig.device);
    TEST_ASSERT_TRUE(qmc.begin());
    TEST_ASSERT_EQUAL_STRING("QMC6309", qmc.getSensorType());

    // Сброс: 1, затем 0; потом CTRL2 (ODR 200 Гц, ±8 Гс), режим последним.
    std::vector<uint8_t> ctrl2;
    for (const auto& w : rig.chip.writes) if (w.first == 0x0B) ctrl2.push_back(w.second);
    TEST_ASSERT_EQUAL(3u, ctrl2.size());
    TEST_ASSERT_EQUAL_HEX8(0x80, ctrl2[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, ctrl2[1]);
    TEST_ASSERT_EQUAL_HEX8(0x48, ctrl2[2]);
    TEST_ASSERT_EQUAL_HEX8(0x81, rig.chip.lastWrite(0x0A));   // LPF 16, OSR ×8, normal
    TEST_ASSERT_EQUAL_HEX8(0x0B, rig.chip.writes[rig.chip.writes.size() - 2].first);

    putLe16(rig.chip.regs, 0x01, 2048);    // 0.5 Гс = 50 мкТл
    putLe16(rig.chip.regs, 0x03, 0);
    putLe16(rig.chip.regs, 0x05, -1024);
    fake::advanceMs(20);
    qmc.update();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, qmc.getMagData().magX);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, qmc.getMagData().magZ);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, qmc.getMagData().headingDegrees);
}

void test_qmc6309_failure_paths()
{
    I2cRig wrong(0x7C);
    wrong.chip.regs[0x00] = 0x80;   // QMC5883P
    QMC6309_Sensor a(wrong.device);
    TEST_ASSERT_FALSE(a.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "QMC6309: не отвечает (chip ID 0x80)"));

    I2cRig absent(0x7C);
    absent.chip.present = false;
    QMC6309_Sensor b(absent.device);
    TEST_ASSERT_FALSE(b.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "нет ответа"));

    I2cRig noNvm(0x7C);
    noNvm.chip.regs[0x00] = 0x90;   // NVM так и не готова
    QMC6309_Sensor c(noNvm.device);
    TEST_ASSERT_FALSE(c.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "NVM не загрузилась"));

    I2cRig readOnly(0x7C);
    readOnly.chip.regs[0x00] = 0x90;
    readOnly.chip.regs[0x09] = 0x18;
    readOnly.chip.failWrites = true;
    QMC6309_Sensor d(readOnly.device);
    TEST_ASSERT_FALSE(d.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "QMC6309: ошибка записи регистров"));
}

// ------------------------------------------------------------
// SPL06-001
// ------------------------------------------------------------

namespace
{
    struct Spl06Coef
    {
        int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;
    };

    // Типичные коэффициенты (есть отрицательные — проверка знаковых полей).
    constexpr Spl06Coef COEF = { 200, -260, 80000, -54000, -2900, 1400, -10500, 150, -1100 };

    void loadSpl06(uint8_t* regs, const Spl06Coef& c)
    {
        regs[0x0D] = 0x10;
        regs[0x08] = 0xC0;   // COEF_RDY | SENSOR_RDY
        regs[0x28] = 0x80;   // калиброван по внешнему (MEMS) датчику температуры
        auto u = [](int32_t v, int bits) { return static_cast<uint32_t>(v) & ((1u << bits) - 1); };
        const uint32_t c0 = u(c.c0, 12), c1 = u(c.c1, 12), c00 = u(c.c00, 20), c10 = u(c.c10, 20);
        uint8_t* k = regs + 0x10;
        k[0] = static_cast<uint8_t>(c0 >> 4);
        k[1] = static_cast<uint8_t>(((c0 & 0x0F) << 4) | (c1 >> 8));
        k[2] = static_cast<uint8_t>(c1 & 0xFF);
        k[3] = static_cast<uint8_t>(c00 >> 12);
        k[4] = static_cast<uint8_t>((c00 >> 4) & 0xFF);
        k[5] = static_cast<uint8_t>(((c00 & 0x0F) << 4) | (c10 >> 16));
        k[6] = static_cast<uint8_t>((c10 >> 8) & 0xFF);
        k[7] = static_cast<uint8_t>(c10 & 0xFF);
        const int32_t s16[] = { c.c01, c.c11, c.c20, c.c21, c.c30 };
        for (int i = 0; i < 5; ++i)
        {
            k[8 + 2 * i] = static_cast<uint8_t>(static_cast<uint16_t>(s16[i]) >> 8);
            k[9 + 2 * i] = static_cast<uint8_t>(s16[i] & 0xFF);
        }
    }

    void putInt24Be(uint8_t* regs, uint8_t reg, int32_t value)
    {
        const uint32_t v = static_cast<uint32_t>(value) & 0xFFFFFFu;
        regs[reg] = static_cast<uint8_t>(v >> 16);
        regs[reg + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        regs[reg + 2] = static_cast<uint8_t>(v & 0xFF);
    }

    // Датащит SPL06-001 §4.9, в double — эталон для float-драйвера.
    void spl06Reference(const Spl06Coef& c, int32_t praw, int32_t traw, double& pressure, double& temperature)
    {
        const double psc = praw / 253952.0, tsc = traw / 524288.0;
        temperature = c.c0 * 0.5 + c.c1 * tsc;
        pressure = c.c00 + psc * (c.c10 + psc * (c.c20 + psc * c.c30)) + tsc * c.c01 +
                   tsc * psc * (c.c11 + psc * c.c21);
    }
}

void test_spl06_configures_and_compensates_like_datasheet()
{
    AltI2cRig rig(SPL06_Sensor::DEFAULT_ADDRESS, SPL06_Sensor::ALTERNATE_ADDRESS, 0x77);
    loadSpl06(rig.chip.regs, COEF);
    const int32_t praw = -99041, traw = 151237;   // отрицательное int24 давления
    putInt24Be(rig.chip.regs, 0x00, praw);
    putInt24Be(rig.chip.regs, 0x03, traw);

    SPL06_Sensor spl(rig.device);
    TEST_ASSERT_TRUE(spl.begin());
    TEST_ASSERT_EQUAL_HEX8(0x77, rig.device.getAddress());
    TEST_ASSERT_TRUE(contains(takeSerial(), "SPL06: подключён"));
    TEST_ASSERT_EQUAL_HEX8(0x09, rig.chip.writes.front().second);   // мягкий сброс первым
    TEST_ASSERT_EQUAL_HEX8(0x54, rig.chip.lastWrite(0x06));   // 32 изм/с × 16
    TEST_ASSERT_EQUAL_HEX8(0xA0, rig.chip.lastWrite(0x07));   // внешний датчик T, 4 изм/с × 1
    TEST_ASSERT_EQUAL_HEX8(0x04, rig.chip.lastWrite(0x09));   // P_SHIFT
    TEST_ASSERT_EQUAL_HEX8(0x07, rig.chip.lastWrite(0x08));   // непрерывно P + T

    rig.chip.regs[0x08] = 0xD7;   // PRS_RDY
    fake::advanceMs(10);
    spl.update();
    double p, t;
    spl06Reference(COEF, praw, traw, p, t);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, static_cast<float>(t), spl.getBarometerData().temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, static_cast<float>(p), spl.getBarometerData().pressure);
    TEST_ASSERT_FLOAT_WITHIN(5000.0f, 100000.0f, spl.getBarometerData().pressure);   // правдоподобно
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 25.0f, spl.getBarometerData().temperature);

    // Флага нет — только MEAS_CFG, без чтения данных.
    rig.chip.regs[0x08] = 0xC7;
    const uint32_t before = rig.chip.readTransactions;
    fake::advanceMs(5);
    spl.update();
    TEST_ASSERT_EQUAL_UINT32(before + 1, rig.chip.readTransactions);
}

void test_spl06_uses_internal_temperature_sensor_when_calibrated_so()
{
    SpiRig rig(Config::PIN_SPI_CS_BARO, 0);
    loadSpl06(rig.chip.regs, COEF);
    rig.chip.regs[0x28] = 0x00;
    SpiRegisterDevice device = SPL06_Sensor::spiDevice(rig.bus, Config::PIN_SPI_CS_BARO);
    SPL06_Sensor spl(device, "STATIC");
    TEST_ASSERT_TRUE(spl.begin());
    TEST_ASSERT_EQUAL_HEX8(0x20, rig.chip.lastWrite(0x07));
    TEST_ASSERT_EQUAL_STRING("STATIC", spl.getSensorType());
}

void test_spl06_failure_paths()
{
    I2cRig spa(0x76);
    spa.chip.regs[0x0D] = 0x11;
    SPL06_Sensor a(spa.device);
    TEST_ASSERT_FALSE(a.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "это SPA06"));

    I2cRig wrong(0x76);
    wrong.chip.regs[0x0D] = 0x58;   // BMP280
    SPL06_Sensor b(wrong.device);
    TEST_ASSERT_FALSE(b.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "неверный ID 0x58"));

    I2cRig absent(0x76);
    absent.chip.present = false;
    SPL06_Sensor c(absent.device);
    TEST_ASSERT_FALSE(c.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "SPL06: не отвечает"));

    I2cRig notReady(0x76);
    loadSpl06(notReady.chip.regs, COEF);
    notReady.chip.regs[0x08] = 0x40;   // коэффициенты так и не готовы
    SPL06_Sensor d(notReady.device);
    TEST_ASSERT_FALSE(d.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "не удалось прочитать коэффициенты"));

    I2cRig readOnly(0x76);
    loadSpl06(readOnly.chip.regs, COEF);
    readOnly.chip.onRegisterWrite = [&readOnly](uint8_t reg, uint8_t) {
        if (reg == 0x0C) readOnly.chip.failWrites = true;
    };
    SPL06_Sensor e(readOnly.device);
    TEST_ASSERT_FALSE(e.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "SPL06: ошибка записи регистров"));
}

// ------------------------------------------------------------
// BMP581
// ------------------------------------------------------------

namespace
{
    void loadBmp581(uint8_t* regs)
    {
        regs[0x01] = 0x50;
        regs[0x27] = 0x10;   // сброс завершён
        regs[0x28] = 0x02;   // NVM готова, ошибок нет
        regs[0x30] = 0x03;   // DSP_CONFIG по умолчанию (биты теней IIR сброшены)
        regs[0x38] = 0x80;   // ODR выполним
    }

    void setBmp581Sample(uint8_t* regs, float pressurePa, float temperatureC)
    {
        const uint32_t t = static_cast<uint32_t>(static_cast<int32_t>(lroundf(temperatureC * 65536.0f))) & 0xFFFFFFu;
        const uint32_t p = static_cast<uint32_t>(lroundf(pressurePa * 64.0f));
        regs[0x1D] = static_cast<uint8_t>(t & 0xFF);
        regs[0x1E] = static_cast<uint8_t>((t >> 8) & 0xFF);
        regs[0x1F] = static_cast<uint8_t>(t >> 16);
        regs[0x20] = static_cast<uint8_t>(p & 0xFF);
        regs[0x21] = static_cast<uint8_t>((p >> 8) & 0xFF);
        regs[0x22] = static_cast<uint8_t>(p >> 16);
    }
}

void test_bmp581_configuration_and_scaling()
{
    I2cRig rig(BMP581_Sensor::DEFAULT_ADDRESS);
    loadBmp581(rig.chip.regs);
    BMP581_Sensor bmp(rig.device);
    TEST_ASSERT_TRUE(bmp.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BMP581: подключён"));
    TEST_ASSERT_EQUAL_HEX8(0xB6, rig.chip.lastWrite(0x7E));
    TEST_ASSERT_EQUAL_HEX8(0x61, rig.chip.lastWrite(0x36));   // press_en, ×16 / ×2
    TEST_ASSERT_EQUAL_HEX8(0x2B, rig.chip.lastWrite(0x30));   // + тени IIR для T и P
    TEST_ASSERT_EQUAL_HEX8(0x10, rig.chip.lastWrite(0x31));   // IIR давления = 3
    TEST_ASSERT_EQUAL_HEX8(0x01, rig.chip.lastWrite(0x15));   // источник drdy
    TEST_ASSERT_EQUAL_HEX8(0xBD, rig.chip.lastWrite(0x37));   // normal, 50 Гц, deep standby выкл
    // Сначала standby с настройками, потом normal.
    std::vector<uint8_t> odr;
    for (const auto& w : rig.chip.writes) if (w.first == 0x37) odr.push_back(w.second);
    TEST_ASSERT_EQUAL(2u, odr.size());
    TEST_ASSERT_EQUAL_HEX8(0xBC, odr[0]);

    setBmp581Sample(rig.chip.regs, 101325.0f, 25.5f);
    rig.chip.regs[0x27] = 0x01;   // drdy
    fake::advanceMs(10);
    bmp.update();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 101325.0f, bmp.getBarometerData().pressure);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.5f, bmp.getBarometerData().temperature);

    setBmp581Sample(rig.chip.regs, 95000.25f, -10.25f);   // отрицательная температура
    fake::advanceMs(10);
    bmp.update();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 95000.25f, bmp.getBarometerData().pressure);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.25f, bmp.getBarometerData().temperature);
}

void test_bmp581_reads_without_drdy_after_two_periods()
{
    I2cRig rig(0x46);
    loadBmp581(rig.chip.regs);
    BMP581_Sensor bmp(rig.device);
    TEST_ASSERT_TRUE(bmp.begin());
    rig.chip.regs[0x27] = 0x00;   // флаг готовности не приходит
    setBmp581Sample(rig.chip.regs, 100000.0f, 20.0f);

    fake::advanceMs(50);
    bmp.update();                 // прошло > 40 мс — читаем всё равно
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 100000.0f, bmp.getBarometerData().pressure);

    setBmp581Sample(rig.chip.regs, 100100.0f, 20.0f);
    fake::advanceMs(10);
    bmp.update();                 // 10 мс — рано
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 100000.0f, bmp.getBarometerData().pressure);
}

void test_bmp581_spi_and_failure_paths()
{
    SpiRig spiRig(Config::PIN_SPI_CS_BARO, 0);
    loadBmp581(spiRig.chip.regs);
    SpiRegisterDevice device = BMP581_Sensor::spiDevice(spiRig.bus, Config::PIN_SPI_CS_BARO);
    BMP581_Sensor spiBmp(device, "PITOT-BMP581");
    TEST_ASSERT_TRUE(spiBmp.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "PITOT-BMP581: подключён"));

    I2cRig bmp585(0x47);
    loadBmp581(bmp585.chip.regs);
    bmp585.chip.regs[0x01] = 0x51;
    bmp585.chip.regs[0x38] = 0x00;   // ODR невыполним — только предупреждение
    BMP581_Sensor a(bmp585.device);
    TEST_ASSERT_TRUE(a.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ODR не успевает"));

    I2cRig wrong(0x46);
    loadBmp581(wrong.chip.regs);
    wrong.chip.regs[0x01] = 0x60;
    BMP581_Sensor b(wrong.device);
    TEST_ASSERT_FALSE(b.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "неверный chip ID 0x60"));

    I2cRig absent(0x46);
    absent.chip.present = false;
    BMP581_Sensor c(absent.device);
    TEST_ASSERT_FALSE(c.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BMP581: не отвечает"));

    I2cRig nvmError(0x46);
    loadBmp581(nvmError.chip.regs);
    nvmError.chip.regs[0x28] = 0x06;   // NVM_RDY и NVM_ERR
    BMP581_Sensor d(nvmError.device);
    TEST_ASSERT_FALSE(d.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "не вышел из сброса"));

    I2cRig noDsp(0x46);
    loadBmp581(noDsp.chip.regs);
    noDsp.chip.failReadIf = [](uint8_t reg) { return reg == 0x30; };
    BMP581_Sensor e(noDsp.device);
    TEST_ASSERT_FALSE(e.begin());

    I2cRig readOnly(0x46);
    loadBmp581(readOnly.chip.regs);
    readOnly.chip.onRegisterWrite = [&readOnly](uint8_t reg, uint8_t) {
        if (reg == 0x7E) readOnly.chip.failWrites = true;
    };
    BMP581_Sensor f(readOnly.device);
    TEST_ASSERT_FALSE(f.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BMP581: ошибка записи регистров"));
}

// ------------------------------------------------------------
// Трубка Пито на двух барометрах
// ------------------------------------------------------------

namespace
{
    // Отсчёт барометра с явным временем — как новый отсчёт настоящего чипа.
    void sample(FakeBaro& baro, float pressurePa, float temperatureC)
    {
        baro.data.pressure = pressurePa;
        baro.data.temperature = temperatureC;
        baro.data.timestamp = micros();
    }

    // Обнуление: n отсчётов трубки с перепадом offset при нулевой скорости.
    void zeroPitot(PitotDualBaroAirspeed& pitot, FakeBaro& total, FakeBaro& stat, float offset, int n)
    {
        for (int i = 0; i < n; ++i)
        {
            fake::advanceMs(20);
            sample(stat, 101325.0f, 15.0f);
            sample(total, 101325.0f + offset, 15.0f);
            pitot.update();
        }
    }
}

void test_pitot_zeroes_offset_then_measures_speed()
{
    FakeBaro total, stat;
    PitotDualBaroAirspeed pitot(total, stat);
    TEST_ASSERT_TRUE(pitot.begin());
    TEST_ASSERT_TRUE(pitot.isZeroing());
    TEST_ASSERT_FALSE(pitot.isAvailable());
    TEST_ASSERT_EQUAL_STRING("PITOT", pitot.getSensorType());

    zeroPitot(pitot, total, stat, 37.0f, Config::PITOT_ZERO_SAMPLES);
    TEST_ASSERT_FALSE(pitot.isZeroing());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 37.0f, pitot.getZeroOffset());
    TEST_ASSERT_TRUE(contains(takeSerial(), "Pitot: ноль 37.0"));

    // 10 м/с на уровне моря при 15 °C: ½·1.225·100 = 61.25 Па.
    for (int i = 0; i < 100; ++i)
    {
        fake::advanceMs(20);
        sample(stat, 101325.0f, 15.0f);
        sample(total, 101325.0f + 37.0f + 61.25f, 15.0f);
        pitot.update();
    }
    TEST_ASSERT_TRUE(pitot.isAvailable());
    const AirspeedData& a = pitot.getAirspeedData();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 61.25f, a.differentialPressurePa);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.225f, a.airDensity);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 10.0f, a.indicatedMs);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 10.0f, a.trueMs);
}

void test_pitot_true_airspeed_grows_with_altitude_and_heat()
{
    FakeBaro total, stat;
    PitotDualBaroAirspeed pitot(total, stat);
    TEST_ASSERT_TRUE(pitot.begin());
    zeroPitot(pitot, total, stat, 0.0f, Config::PITOT_ZERO_SAMPLES);

    // Тот же перепад высоко и в жару: воздух реже — истинная скорость больше.
    for (int i = 0; i < 100; ++i)
    {
        fake::advanceMs(20);
        sample(stat, 90000.0f, 30.0f);
        sample(total, 90000.0f + 61.25f, 30.0f);
        pitot.update();
    }
    const AirspeedData& a = pitot.getAirspeedData();
    const float rho = 90000.0f / (287.05f * 303.15f);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, rho, a.airDensity);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 10.0f, a.indicatedMs);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, sqrtf(2.0f * 61.25f / rho), a.trueMs);
    TEST_ASSERT_GREATER_THAN_FLOAT(a.indicatedMs + 0.5f, a.trueMs);
}

void test_pitot_filter_smooths_steps()
{
    FakeBaro total, stat;
    PitotDualBaroAirspeed pitot(total, stat);
    TEST_ASSERT_TRUE(pitot.begin());
    zeroPitot(pitot, total, stat, 0.0f, Config::PITOT_ZERO_SAMPLES);

    fake::advanceMs(20);
    sample(stat, 101325.0f, 15.0f);
    sample(total, 101325.0f, 15.0f);
    pitot.update();   // первый отсчёт после обнуления — фильтр на нём
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pitot.getAirspeedData().differentialPressurePa);

    fake::advanceMs(20);
    sample(stat, 101325.0f, 15.0f);
    sample(total, 101325.0f + 100.0f, 15.0f);
    pitot.update();   // ступенька 100 Па: за 20 мс при τ = 0.1 с — 1/6 пути
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f * 0.02f / 0.12f, pitot.getAirspeedData().differentialPressurePa);

    // Старый отсчёт трубки — без изменений.
    stat.data.timestamp = micros() + 1;
    pitot.update();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 16.7f, pitot.getAirspeedData().differentialPressurePa);
}

void test_pitot_detects_reversed_hoses_and_stale_data()
{
    FakeBaro total, stat;
    PitotDualBaroAirspeed pitot(total, stat);
    TEST_ASSERT_TRUE(pitot.begin());
    zeroPitot(pitot, total, stat, 0.0f, Config::PITOT_ZERO_SAMPLES);
    takeSerial();

    for (int i = 0; i < 110; ++i)   // 2.2 с перепада −40 Па
    {
        fake::advanceMs(20);
        sample(stat, 101325.0f, 15.0f);
        sample(total, 101325.0f - 40.0f, 15.0f);
        pitot.update();
    }
    TEST_ASSERT_TRUE(pitot.hasFault());
    TEST_ASSERT_FALSE(pitot.isAvailable());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pitot.getAirspeedData().indicatedMs);   // отрицательный -> 0
    TEST_ASSERT_TRUE(contains(takeSerial(), "трубка неисправна"));
    pitot.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "шланги/вода"));

    // Короткий отрицательный выброс неисправностью не считается.
    pitot.calibrateZero();
    zeroPitot(pitot, total, stat, 0.0f, Config::PITOT_ZERO_SAMPLES);
    for (int i = 0; i < 20; ++i)
    {
        fake::advanceMs(20);
        sample(stat, 101325.0f, 15.0f);
        sample(total, 101325.0f + (i < 10 ? -40.0f : 20.0f), 15.0f);
        pitot.update();
    }
    TEST_ASSERT_FALSE(pitot.hasFault());
    TEST_ASSERT_TRUE(pitot.isAvailable());

    fake::advanceMs(300);   // трубка замолчала
    TEST_ASSERT_FALSE(pitot.isAvailable());

    stat.available = false;
    fake::advanceMs(20);
    sample(total, 101325.0f + 20.0f, 15.0f);
    pitot.update();
    TEST_ASSERT_FALSE(pitot.isAvailable());
    pitot.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "PITOT: available=NO"));
}

void test_pitot_reports_missing_tube_sensor()
{
    I2cRig rig(0x47);
    rig.chip.present = false;
    BMP581_Sensor tube(rig.device, "PITOT-BMP581");
    FakeBaro stat;
    PitotDualBaroAirspeed pitot(tube, stat);
    TEST_ASSERT_FALSE(pitot.begin());
    TEST_ASSERT_FALSE(pitot.isAvailable());
    TEST_ASSERT_TRUE(contains(takeSerial(), "барометр трубки не отвечает"));
    pitot.update();   // без begin ничего не делает
    TEST_ASSERT_EQUAL_UINT32(0, pitot.getAirspeedData().timestamp);
}

void test_pitot_speed_and_density_helpers()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, PitotDualBaroAirspeed::speedFrom(-5.0f, 1.2f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, PitotDualBaroAirspeed::speedFrom(50.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, PitotDualBaroAirspeed::speedFrom(245.0f, 1.225f));
    // Температура за пределами разумного ограничивается (сбой датчика не даёт ρ = ∞).
    TEST_ASSERT_FLOAT_WITHIN(0.01f, PitotDualBaroAirspeed::densityOf(101325.0f, 60.0f),
                             PitotDualBaroAirspeed::densityOf(101325.0f, 400.0f));
}

// "Полёт": скорость 0 -> 22 -> 8 м/с, набор 60 м; барометр трубки
// 50 Гц с шумом 0.3 Па, статики 32 Гц с шумом 1 Па и смещением между
// чипами 180 Па. Истинная скорость должна сходиться с оценкой.
void test_pitot_tracks_simulated_flight()
{
    FakeBaro total, stat;
    PitotDualBaroAirspeed pitot(total, stat);
    TEST_ASSERT_TRUE(pitot.begin());

    std::mt19937 rng(42);
    std::normal_distribution<float> noiseTube(0.0f, 0.3f), noiseStatic(0.0f, 1.0f);

    const float OFFSET = 180.0f;
    const float T = 15.0f;
    double sumSqError = 0;
    int samples = 0;
    float maxError = 0;

    uint32_t nextStaticMs = 0, nextTubeMs = 0;
    float staticP = 101325.0f;
    for (uint32_t ms = 0; ms < 60000; ms += 2)
    {
        fake::advanceMs(2);
        const float t = ms / 1000.0f;
        const float v = t < 2 ? 0.0f : t < 20 ? (t - 2) * 22.0f / 18.0f : t < 40 ? 22.0f : 8.0f + 14.0f * expf(-(t - 40));
        const float h = t < 10 ? 0.0f : std::min(60.0f, (t - 10) * 3.0f);
        const float ps = 101325.0f * powf(1.0f - h / 44330.0f, 5.255f);
        const float rho = ps / (287.05f * (T + 273.15f));

        if (ms >= nextStaticMs)
        {
            nextStaticMs += 31;
            staticP = ps + noiseStatic(rng);
            sample(stat, staticP, T);
        }
        if (ms >= nextTubeMs)
        {
            nextTubeMs += 20;
            sample(total, ps + 0.5f * rho * v * v + OFFSET + noiseTube(rng), T);
        }
        pitot.update();

        if (t > 8 && pitot.isAvailable() && v > 6.0f)
        {
            const float err = pitot.getAirspeedData().trueMs - v;
            sumSqError += err * err;
            samples++;
            maxError = std::max(maxError, std::fabs(err));
        }
    }

    TEST_ASSERT_GREATER_THAN(10000, samples);
    const float rms = static_cast<float>(sqrt(sumSqError / samples));
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, rms);        // среднеквадратичная ошибка < 0.5 м/с
    TEST_ASSERT_LESS_THAN_FLOAT(2.0f, maxError);   // на переходных — меньше 2 м/с
    TEST_ASSERT_FLOAT_WITHIN(2.0f, OFFSET, pitot.getZeroOffset());
}

// ------------------------------------------------------------
// Выбор датчиков: наборы и адреса
// ------------------------------------------------------------

void test_sensor_kit_constants_are_distinct()
{
    static_assert(SENSOR_KIT == SENSOR_KIT_BENCH_GY521, "по умолчанию — проверенный стенд");
    static_assert(SENSOR_KIT_LSM6DSV_PITOT != SENSOR_KIT_ICM45686_PITOT, "наборы различимы");
    static_assert(SENSOR_AIRSPEED == SENSOR_AIRSPEED_NONE, "на стенде трубки нет");
    TEST_ASSERT_EQUAL_HEX8(0x7C, QMC6309_Sensor::DEFAULT_ADDRESS);
    TEST_ASSERT_EQUAL_HEX8(0x47, BMP581_Sensor::ALTERNATE_ADDRESS);
}

void test_i2c_device_keeps_primary_when_both_or_none_answer()
{
    TwoWire wire{ 7 };
    Esp32I2CBus bus{ wire, 8, 9 };
    fake::RegisterMapDevice a, b;
    wire.attach(0x76, &a);
    wire.attach(0x77, &b);
    bus.begin();

    I2cRegisterDevice both(bus, 0x76, 0x77);
    both.begin();
    TEST_ASSERT_EQUAL_HEX8(0x76, both.getAddress());

    I2cRegisterDevice none(bus, 0x46, 0x47);
    none.begin();
    TEST_ASSERT_EQUAL_HEX8(0x46, none.getAddress());

    I2cRegisterDevice fixed(bus, 0x55);
    fixed.begin();
    TEST_ASSERT_EQUAL_HEX8(0x55, fixed.getAddress());

    // Пакетная запись по I2C — один проход с автоинкрементом.
    const uint8_t data[3] = { 1, 2, 3 };
    TEST_ASSERT_TRUE(both.writeRegisters(0x10, data, 3));
    TEST_ASSERT_EQUAL(3, a.regs[0x12]);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_lsm6dsv_found_at_alternate_address_and_configured);
    RUN_TEST(test_lsm6dsv32x_gets_its_own_16g_code);
    RUN_TEST(test_lsm6dsv_over_spi);
    RUN_TEST(test_lsm6dsv_failure_paths);
    RUN_TEST(test_icm45686_configures_modes_and_indirect_filters);
    RUN_TEST(test_icm45686_over_spi_and_failures);
    RUN_TEST(test_qmc6309_waits_for_nvm_and_reads_field);
    RUN_TEST(test_qmc6309_failure_paths);
    RUN_TEST(test_spl06_configures_and_compensates_like_datasheet);
    RUN_TEST(test_spl06_uses_internal_temperature_sensor_when_calibrated_so);
    RUN_TEST(test_spl06_failure_paths);
    RUN_TEST(test_bmp581_configuration_and_scaling);
    RUN_TEST(test_bmp581_reads_without_drdy_after_two_periods);
    RUN_TEST(test_bmp581_spi_and_failure_paths);
    RUN_TEST(test_pitot_zeroes_offset_then_measures_speed);
    RUN_TEST(test_pitot_true_airspeed_grows_with_altitude_and_heat);
    RUN_TEST(test_pitot_filter_smooths_steps);
    RUN_TEST(test_pitot_detects_reversed_hoses_and_stale_data);
    RUN_TEST(test_pitot_reports_missing_tube_sensor);
    RUN_TEST(test_pitot_speed_and_density_helpers);
    RUN_TEST(test_pitot_tracks_simulated_flight);
    RUN_TEST(test_sensor_kit_constants_are_distinct);
    RUN_TEST(test_i2c_device_keeps_primary_when_both_or_none_answer);
    return UNITY_END();
}
