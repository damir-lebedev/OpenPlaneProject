// ============================================================
// Барометры (BarometerBase, BMP388 по I2C и SPI, BME280/BMP280),
// компасы (MagnetometerBase, QMC5883P, QMC5883L), GPS u-blox M10
// (настройка CFG-VALSET, разбор UBX NAV-PVT), выбор датчиков.
//
// Компенсация BME280 проверяется на примере из даташита Bosch BMP280
// (§3.12): T = 25.08 °C, P = 100653.27 Па.
//
// Запуск: pio test -e native -f native/test_baro_mag_gps
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <type_traits>

#include "sensors/SensorMounting.h"
#include "sensors/SensorSelection.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "sensors/baro/BME280_Sensor.h"
#include "sensors/baro/BMP388_Sensor.h"
#include "sensors/baro/BarometerBase.h"
#include "sensors/gps/UbloxM10_Gps.h"
#include "sensors/mag/QMC5883L_Sensor.h"
#include "sensors/mag/QMC5883P_Sensor.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

// ------------------------------------------------------------
// BarometerBase (сценарный драйвер)
// ------------------------------------------------------------

namespace
{
    class ScriptedBaro : public BarometerBase
    {
    public:
        float pressure = 101325.0f;
        float temperature = 20.0f;
        bool ready = true;
        bool statusOk = true;
        bool readOk = true;
        unsigned reads = 0;

        ScriptedBaro() : BarometerBase("TESTBARO", 5000) {}

        bool begin() override
        {
            setAvailable(true);
            return true;
        }

    protected:
        bool isNewSampleReady(bool& isReady) override
        {
            if (!statusOk) return false;
            isReady = ready;
            return true;
        }

        bool readSample(float& pressurePa, float& temperatureC) override
        {
            if (!readOk) return false;
            reads++;
            pressurePa = pressure;
            temperatureC = temperature;
            return true;
        }
    };

    // Давление на высоте h над уровнем с давлением p0 (МСА).
    float pressureAt(float p0, float h)
    {
        return p0 * powf(1.0f - h / 44330.0f, 1.0f / 0.1903f);
    }
}

void test_barometer_does_nothing_before_begin()
{
    ScriptedBaro baro;
    TEST_ASSERT_FALSE(baro.isAvailable());
    fake::advanceMs(100);
    baro.update();
    baro.calibrateAltitude();
    TEST_ASSERT_EQUAL(0u, baro.reads);
    TEST_ASSERT_EQUAL_STRING("TESTBARO", baro.getSensorType());
}

void test_barometer_polls_at_its_period_and_only_new_samples()
{
    ScriptedBaro baro;
    baro.begin();
    fake::advanceMs(10);
    baro.update();
    TEST_ASSERT_EQUAL(1u, baro.reads);
    baro.update();                   // раньше периода опроса
    TEST_ASSERT_EQUAL(1u, baro.reads);

    fake::advanceMs(5);
    baro.ready = false;              // у чипа нет нового отсчёта
    baro.update();
    TEST_ASSERT_EQUAL(1u, baro.reads);

    fake::advanceMs(5);
    baro.ready = true;
    baro.update();
    TEST_ASSERT_EQUAL(2u, baro.reads);
    TEST_ASSERT_EQUAL_FLOAT(101325.0f, baro.getBarometerData().pressure);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, baro.getBarometerData().temperature);
    TEST_ASSERT_EQUAL_UINT32(micros(), baro.getBarometerData().timestamp);
}

void test_barometer_relative_altitude_and_filtered_climb()
{
    ScriptedBaro baro;
    baro.begin();
    baro.pressure = 100000.0f;
    baro.calibrateAltitude();
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка завершена"));
    TEST_ASSERT_EQUAL(20u, baro.reads);

    fake::advanceMs(20);
    baro.update();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, baro.getBarometerData().altitude);

    // Подъём 2 м/с в течение 3 с — высота ~6 м, вертикальная скорость
    // сходится к 2 м/с через ФНЧ (τ = 0.5 с).
    for (int i = 1; i <= 150; ++i)
    {
        fake::advanceMs(20);
        baro.pressure = pressureAt(100000.0f, 0.04f * i);
        baro.update();
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 6.0f, baro.getBarometerData().altitude);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.0f, baro.getBarometerData().verticalSpeed);
}

void test_barometer_errors_and_recovery()
{
    ScriptedBaro baro;
    baro.begin();
    baro.statusOk = false;
    for (int i = 0; i < 99; ++i)
    {
        fake::advanceMs(5);
        baro.update();
    }
    TEST_ASSERT_TRUE(baro.isAvailable());
    fake::advanceMs(5);
    baro.statusOk = true;
    baro.readOk = false;             // сотая ошибка — уже на чтении данных
    baro.update();
    TEST_ASSERT_FALSE(baro.isAvailable());

    baro.readOk = true;
    fake::advanceMs(5);
    baro.update();
    TEST_ASSERT_TRUE(baro.isAvailable());

    takeSerial();
    baro.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "TESTBARO: available=YES errors=100 pressure=1013.25hPa"));
}

void test_barometer_calibration_failure_and_sea_level_pressure()
{
    ScriptedBaro baro;
    baro.begin();
    baro.readOk = false;
    baro.calibrateAltitude();
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка не удалась"));

    baro.readOk = true;
    baro.setSeaLevelPressure(101325.0f);
    baro.pressure = 101325.0f;
    baro.calibrateAltitude();
    TEST_ASSERT_TRUE(contains(takeSerial(), "база=0.00 м"));
}

// ------------------------------------------------------------
// BMP388
// ------------------------------------------------------------

namespace
{
    // Коэффициенты NVM (21 байт с 0x31) и эталон, посчитанный по
    // формулам даташита BMP388 (§9.1 масштабы, §9.3 компенсация)
    // независимой реализацией.
    const uint8_t BMP388_NVM[21] = { 0x15, 0x6A, 0x60, 0x49, 0xF9, 0x3A, 0x0A, 0x21, 0x04, 0x07, 0xF1,
                                     0x28, 0x4A, 0xF8, 0x5A, 0x03, 0xF9, 0xF2, 0x0F, 0x06, 0xF1 };
    constexpr uint32_t BMP388_UT = 8200000;
    constexpr uint32_t BMP388_UP = 6500000;
    constexpr float BMP388_T = 21.790386f;
    constexpr float BMP388_P = 70945.307f;

    void loadBmp388(uint8_t* regs)
    {
        regs[0x00] = 0x50;
        memcpy(&regs[0x31], BMP388_NVM, sizeof(BMP388_NVM));
        regs[0x03] = 0x20;   // drdy_press
        const uint32_t p = BMP388_UP, t = BMP388_UT;
        const uint8_t data[6] = { static_cast<uint8_t>(p), static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p >> 16),
                                  static_cast<uint8_t>(t), static_cast<uint8_t>(t >> 8), static_cast<uint8_t>(t >> 16) };
        memcpy(&regs[0x04], data, sizeof(data));
    }
}

void test_bmp388_i2c_configuration_and_compensation()
{
    I2cRig rig(0x76);
    loadBmp388(rig.chip.regs);
    BMP388_Sensor bmp(rig.device);
    TEST_ASSERT_TRUE(bmp.begin());
    TEST_ASSERT_TRUE(bmp.isAvailable());
    TEST_ASSERT_EQUAL_STRING("BMP388", bmp.getSensorType());
    TEST_ASSERT_EQUAL(0xB6, rig.chip.lastWrite(0x7E));   // soft reset
    TEST_ASSERT_EQUAL(0x03, rig.chip.lastWrite(0x1C));   // OSR
    TEST_ASSERT_EQUAL(0x02, rig.chip.lastWrite(0x1D));   // 50 Гц
    TEST_ASSERT_EQUAL(0x04, rig.chip.lastWrite(0x1F));   // IIR
    TEST_ASSERT_EQUAL(0x33, rig.chip.lastWrite(0x1B));   // normal mode

    fake::advanceMs(10);
    bmp.update();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, BMP388_T, bmp.getBarometerData().temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, BMP388_P, bmp.getBarometerData().pressure);

    // Флаг готовности сброшен — нового отсчёта нет, данные не читаются.
    rig.chip.regs[0x03] = 0x00;
    const uint32_t before = rig.chip.readTransactions;
    fake::advanceMs(5);
    bmp.update();
    TEST_ASSERT_EQUAL_UINT32(before + 1, rig.chip.readTransactions);   // только STATUS
}

void test_bmp388_spi_skips_dummy_byte()
{
    SpiRig rig(Config::PIN_SPI_CS_BMP388, 1);   // чип отдаёт мусорный байт перед данными
    loadBmp388(rig.chip.regs);
    SpiRegisterDevice device = BMP388_Sensor::spiDevice(rig.bus, Config::PIN_SPI_CS_BMP388);
    BMP388_Sensor bmp(device);
    TEST_ASSERT_TRUE(bmp.begin());
    fake::advanceMs(10);
    bmp.update();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, BMP388_P, bmp.getBarometerData().pressure);
}

void test_bmp388_failure_paths()
{
    I2cRig absent(0x76);
    absent.chip.present = false;
    BMP388_Sensor none(absent.device);
    TEST_ASSERT_FALSE(none.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BMP388: не отвечает"));

    I2cRig wrong(0x76);
    wrong.chip.regs[0x00] = 0x60;
    BMP388_Sensor other(wrong.device);
    TEST_ASSERT_FALSE(other.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "неверный chip ID 0x60"));

    I2cRig noNvm(0x76);
    loadBmp388(noNvm.chip.regs);
    noNvm.chip.failReadIf = [](uint8_t reg) { return reg == 0x31; };
    BMP388_Sensor nvmFail(noNvm.device);
    TEST_ASSERT_FALSE(nvmFail.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "не удалось прочитать калибровку NVM"));

    I2cRig readOnly(0x76);
    loadBmp388(readOnly.chip.regs);
    readOnly.chip.failWrites = true;
    BMP388_Sensor writeFail(readOnly.device);
    TEST_ASSERT_FALSE(writeFail.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BMP388: ошибка записи регистров"));

    // Чип пропал после старта: чтение STATUS/данных не проходит.
    I2cRig flaky(0x76);
    loadBmp388(flaky.chip.regs);
    BMP388_Sensor bmp(flaky.device);
    TEST_ASSERT_TRUE(bmp.begin());
    flaky.chip.failReads = -1;
    for (int i = 0; i < 100; ++i)
    {
        fake::advanceMs(5);
        bmp.update();
    }
    TEST_ASSERT_FALSE(bmp.isAvailable());
}

// ------------------------------------------------------------
// BME280 / BMP280
// ------------------------------------------------------------

namespace
{
    void loadBmp280Example(uint8_t* regs, uint8_t chipId)
    {
        regs[0xD0] = chipId;
        const uint16_t t1 = 27504;
        const int16_t t2 = 26435, t3 = -1000;
        const uint16_t p1 = 36477;
        const int16_t p[8] = { -10685, 3024, 2855, 140, -7, 15500, -14600, 6000 };
        auto put = [&](uint8_t reg, uint16_t v) {
            regs[reg] = static_cast<uint8_t>(v & 0xFF);
            regs[reg + 1] = static_cast<uint8_t>(v >> 8);
        };
        put(0x88, t1);
        put(0x8A, static_cast<uint16_t>(t2));
        put(0x8C, static_cast<uint16_t>(t3));
        put(0x8E, p1);
        for (uint8_t i = 0; i < 8; ++i) put(static_cast<uint8_t>(0x90 + 2 * i), static_cast<uint16_t>(p[i]));
        // adc_P = 415148 (0x655AC), adc_T = 519888 (0x7EED0), 20 бит.
        const uint8_t data[6] = { 0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00 };
        memcpy(&regs[0xF7], data, sizeof(data));
    }
}

void test_bme280_compensation_matches_datasheet_example()
{
    I2cRig rig(0x76);
    loadBmp280Example(rig.chip.regs, 0x60);
    BME280_Sensor bme(rig.device);
    TEST_ASSERT_TRUE(bme.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BME280: подключён"));
    TEST_ASSERT_EQUAL(0xB6, rig.chip.lastWrite(0xE0));   // reset
    TEST_ASSERT_EQUAL(0x00, rig.chip.lastWrite(0xF2));   // влажность выкл
    TEST_ASSERT_EQUAL(0x08, rig.chip.lastWrite(0xF5));
    TEST_ASSERT_EQUAL(0x53, rig.chip.lastWrite(0xF4));

    // CTRL_HUM применяется только после записи CTRL_MEAS — пишется раньше.
    size_t humIndex = 0, measIndex = 0;
    for (size_t i = 0; i < rig.chip.writes.size(); ++i)
    {
        if (rig.chip.writes[i].first == 0xF2) humIndex = i;
        if (rig.chip.writes[i].first == 0xF4) measIndex = i;
    }
    TEST_ASSERT_LESS_THAN(measIndex, humIndex);

    fake::advanceMs(30);
    bme.update();
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 25.08f, bme.getBarometerData().temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 100653.27f, bme.getBarometerData().pressure);
}

void test_bmp280_uses_same_driver_without_humidity()
{
    I2cRig rig(0x76);
    loadBmp280Example(rig.chip.regs, 0x58);
    BME280_Sensor bmp(rig.device);
    TEST_ASSERT_TRUE(bmp.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "(BMP280)"));
    TEST_ASSERT_EQUAL(-1, rig.chip.lastWrite(0xF2));
}

void test_bme280_failure_paths()
{
    I2cRig absent(0x76);
    absent.chip.present = false;
    BME280_Sensor none(absent.device);
    TEST_ASSERT_FALSE(none.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BME280: не отвечает"));

    I2cRig wrong(0x76);
    wrong.chip.regs[0xD0] = 0x50;
    BME280_Sensor other(wrong.device);
    TEST_ASSERT_FALSE(other.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "неверный chip ID 0x50"));

    I2cRig noCal(0x76);
    loadBmp280Example(noCal.chip.regs, 0x60);
    noCal.chip.failReadIf = [](uint8_t reg) { return reg == 0x88; };
    BME280_Sensor calFail(noCal.device);
    TEST_ASSERT_FALSE(calFail.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "не удалось прочитать калибровку"));

    I2cRig readOnly(0x76);
    loadBmp280Example(readOnly.chip.regs, 0x60);
    readOnly.chip.failWrites = true;
    BME280_Sensor writeFail(readOnly.device);
    TEST_ASSERT_FALSE(writeFail.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "BME280: ошибка записи регистров"));

    // Нулевой dig_P1 — деление на ноль в формуле Bosch, давление 0.
    I2cRig zero(0x76);
    loadBmp280Example(zero.chip.regs, 0x60);
    zero.chip.regs[0x8E] = 0;
    zero.chip.regs[0x8F] = 0;
    BME280_Sensor guarded(zero.device);
    TEST_ASSERT_TRUE(guarded.begin());
    fake::advanceMs(30);
    guarded.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, guarded.getBarometerData().pressure);
    // Ошибка чтения данных.
    zero.chip.failReads = -1;
    fake::advanceMs(30);
    guarded.update();
    guarded.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "errors=1"));
}

// ------------------------------------------------------------
// QMC5883P / QMC5883L и MagnetometerBase
// ------------------------------------------------------------

namespace
{
    void setQmcP(fake::RegisterMapDevice& chip, int16_t x, int16_t y, int16_t z)
    {
        chip.setLittleEndian16(0x01, x);
        chip.setLittleEndian16(0x03, y);
        chip.setLittleEndian16(0x05, z);
    }

    struct QmcP
    {
        I2cRig rig{ QMC5883P_Sensor::DEFAULT_ADDRESS };
        QMC5883P_Sensor sensor{ rig.device };
        QmcP() { rig.chip.regs[0x00] = 0x80; }
    };
}

void test_qmc5883p_configuration_and_heading()
{
    QmcP qmc;
    TEST_ASSERT_TRUE(qmc.sensor.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка НЕ выполнена"));
    TEST_ASSERT_EQUAL(0x06, qmc.rig.chip.lastWrite(0x29));
    TEST_ASSERT_EQUAL(0x08, qmc.rig.chip.lastWrite(0x0B));
    TEST_ASSERT_EQUAL(0xCD, qmc.rig.chip.lastWrite(0x0A));
    TEST_ASSERT_EQUAL_STRING("QMC5883P", qmc.sensor.getSensorType());

    struct Case { int16_t x, y; float heading; };
    const Case cases[] = { { 375, 0, 0.0f }, { 0, 375, 90.0f }, { -375, 0, 180.0f }, { 0, -375, 270.0f },
                           { 375, 375, 45.0f } };
    for (const Case& c : cases)
    {
        setQmcP(qmc.rig.chip, c.x, c.y, 750);
        fake::advanceMs(20);
        qmc.sensor.update();
        TEST_ASSERT_FLOAT_WITHIN(0.01f, c.heading, qmc.sensor.getMagData().headingDegrees);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, qmc.sensor.getMagData().magZ);   // 37.5 LSB/мкТл
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, qmc.sensor.getMagData().magX);
}

void test_magnetometer_polls_at_50hz_and_counts_errors()
{
    QmcP qmc;
    qmc.sensor.begin();
    fake::advanceMs(20);
    qmc.sensor.update();
    const uint32_t reads = qmc.rig.chip.readTransactions;
    fake::advanceMs(10);
    qmc.sensor.update();
    TEST_ASSERT_EQUAL_UINT32(reads, qmc.rig.chip.readTransactions);

    qmc.rig.chip.failReads = -1;
    for (int i = 0; i < 24; ++i)
    {
        fake::advanceMs(20);
        qmc.sensor.update();
    }
    TEST_ASSERT_TRUE(qmc.sensor.isAvailable());
    fake::advanceMs(20);
    qmc.sensor.update();
    TEST_ASSERT_FALSE(qmc.sensor.isAvailable());
    qmc.rig.chip.failReads = 0;
    fake::advanceMs(20);
    qmc.sensor.update();
    TEST_ASSERT_TRUE(qmc.sensor.isAvailable());
}

void test_magnetometer_hard_iron_calibration_persists_in_nvs()
{
    QmcP qmc;
    qmc.sensor.begin();
    const uint64_t start = fake::nowUs();
    qmc.rig.chip.beforeRead = [&](uint8_t, size_t) {
        // Вращение: X от −275 до 475, Y от −125 до 625, Z от 0 до 200.
        const float t = (fake::nowUs() - start) / 15e6f;
        const float a = 2.0f * static_cast<float>(PI) * 3.0f * t;
        setQmcP(qmc.rig.chip, static_cast<int16_t>(100 + 375 * cosf(a)), static_cast<int16_t>(250 + 375 * sinf(a)),
                static_cast<int16_t>(100 + 100 * sinf(a)));
    };
    qmc.sensor.calibrate();
    qmc.rig.chip.beforeRead = nullptr;
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка сохранена"));

    Preferences prefs;
    prefs.begin("qmc5883p", true);
    TEST_ASSERT_TRUE(prefs.getBool("ok"));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, prefs.getFloat("x"));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 250.0f, prefs.getFloat("y"));
    prefs.end();

    // Смещение вычитается: поле (100 + 375, 250) — ровно на север.
    setQmcP(qmc.rig.chip, 475, 250, 100);
    fake::advanceMs(20);
    qmc.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, qmc.sensor.getMagData().headingDegrees);

    // После перезагрузки калибровка берётся из NVS.
    QmcP rebooted;
    rebooted.sensor.begin();
    TEST_ASSERT_TRUE(contains(takeSerial(), "загружена из NVS"));
    rebooted.sensor.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "QMC5883P: available=YES calibrated=YES"));
}

// Раньше калибровка без единого удачного чтения сохраняла в NVS
// смещения из INT16_MAX/INT16_MIN как "калибровку".
void test_magnetometer_calibration_without_samples_keeps_old_one()
{
    {
        Preferences prefs;
        prefs.begin("qmc5883p", false);
        prefs.putFloat("x", 11.0f);
        prefs.putFloat("y", 22.0f);
        prefs.putFloat("z", 33.0f);
        prefs.putBool("ok", true);
        prefs.end();
    }
    QmcP qmc;
    qmc.sensor.begin();
    qmc.rig.chip.failReads = -1;
    qmc.sensor.calibrate();
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка не удалась"));

    Preferences prefs;
    prefs.begin("qmc5883p", true);
    TEST_ASSERT_EQUAL_FLOAT(11.0f, prefs.getFloat("x"));
    TEST_ASSERT_EQUAL_FLOAT(22.0f, prefs.getFloat("y"));
    prefs.end();

    // И недоступный компас не калибруется вовсе.
    QmcP absent;
    absent.rig.chip.present = false;
    TEST_ASSERT_FALSE(absent.sensor.begin());
    absent.sensor.calibrate();
    TEST_ASSERT_FALSE(contains(takeSerial(), "вращайте"));
}

void test_qmc5883p_failure_paths()
{
    QmcP wrong;
    wrong.rig.chip.regs[0x00] = 0x12;
    TEST_ASSERT_FALSE(wrong.sensor.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "chip ID 0x12"));

    QmcP absent;
    absent.rig.chip.present = false;
    TEST_ASSERT_FALSE(absent.sensor.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "нет ответа"));

    QmcP readOnly;
    readOnly.rig.chip.failWrites = true;
    TEST_ASSERT_FALSE(readOnly.sensor.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "QMC5883P: ошибка записи регистров"));

    fake::nvs().failBegin = true;   // NVS недоступно — работаем без калибровки
    QmcP noNvs;
    TEST_ASSERT_TRUE(noNvs.sensor.begin());
    fake::nvs().failBegin = false;
}

void test_qmc5883l_register_layout()
{
    I2cRig rig(QMC5883L_Sensor::DEFAULT_ADDRESS);
    QMC5883L_Sensor qmc(rig.device);
    TEST_ASSERT_TRUE(qmc.begin());
    TEST_ASSERT_EQUAL(0x01, rig.chip.lastWrite(0x0B));
    TEST_ASSERT_EQUAL(0x1D, rig.chip.lastWrite(0x09));
    TEST_ASSERT_EQUAL_STRING("QMC5883L", qmc.getSensorType());

    rig.chip.setLittleEndian16(0x00, 0);
    rig.chip.setLittleEndian16(0x02, -300);   // 30 LSB/мкТл: −10 мкТл по Y
    rig.chip.setLittleEndian16(0x04, 60);
    fake::advanceMs(20);
    qmc.update();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, qmc.getMagData().magY);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, qmc.getMagData().magZ);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 270.0f, qmc.getMagData().headingDegrees);

    I2cRig absent(QMC5883L_Sensor::DEFAULT_ADDRESS);
    absent.chip.present = false;
    QMC5883L_Sensor none(absent.device);
    TEST_ASSERT_FALSE(none.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "QMC5883L: не отвечает"));

    I2cRig readOnly(QMC5883L_Sensor::DEFAULT_ADDRESS);
    readOnly.chip.failWrites = true;
    QMC5883L_Sensor failing(readOnly.device);
    TEST_ASSERT_FALSE(failing.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "QMC5883L: ошибка записи регистров"));
}

void test_sensor_mounting_rotations()
{
    float x, y;
    SensorMounting::rotateToBody(0, 1, 2, x, y);
    TEST_ASSERT_EQUAL_FLOAT(1, x); TEST_ASSERT_EQUAL_FLOAT(2, y);
    SensorMounting::rotateToBody(90, 1, 2, x, y);
    TEST_ASSERT_EQUAL_FLOAT(2, x); TEST_ASSERT_EQUAL_FLOAT(-1, y);
    SensorMounting::rotateToBody(180, 1, 2, x, y);
    TEST_ASSERT_EQUAL_FLOAT(-1, x); TEST_ASSERT_EQUAL_FLOAT(-2, y);
    SensorMounting::rotateToBody(270, 1, 2, x, y);
    TEST_ASSERT_EQUAL_FLOAT(-2, x); TEST_ASSERT_EQUAL_FLOAT(1, y);
    SensorMounting::rotateToBody(45, 1, 2, x, y);   // неподдержанный угол — как 0
    TEST_ASSERT_EQUAL_FLOAT(1, x); TEST_ASSERT_EQUAL_FLOAT(2, y);
}

// ------------------------------------------------------------
// u-blox M10
// ------------------------------------------------------------

namespace
{
    std::vector<uint8_t> ubx(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload, bool corrupt = false)
    {
        std::vector<uint8_t> frame = { 0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size() & 0xFF),
                                       static_cast<uint8_t>(payload.size() >> 8) };
        frame.insert(frame.end(), payload.begin(), payload.end());
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < frame.size(); ++i)
        {
            a = static_cast<uint8_t>(a + frame[i]);
            b = static_cast<uint8_t>(b + a);
        }
        frame.push_back(static_cast<uint8_t>(corrupt ? a + 1 : a));
        frame.push_back(b);
        return frame;
    }

    void put32(std::vector<uint8_t>& p, size_t offset, int32_t v)
    {
        for (int i = 0; i < 4; ++i) p[offset + i] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> (8 * i));
    }

    std::vector<uint8_t> navPvt(uint8_t fixType)
    {
        std::vector<uint8_t> p(92, 0);
        p[20] = fixType;
        p[23] = 12;
        put32(p, 24, 376173000);      // lon 37.6173°
        put32(p, 28, 557558000);      // lat 55.7558°
        put32(p, 36, 150250);         // hMSL, мм
        put32(p, 40, 2500);           // hAcc, мм
        put32(p, 44, 4000);           // vAcc, мм
        put32(p, 60, 12345);          // gSpeed, мм/с
        put32(p, 64, -9000000);       // headMot −90° -> 270°
        return p;
    }

    void feed(FakeUart& uart, const std::vector<uint8_t>& bytes)
    {
        for (uint8_t b : bytes) uart.rx.push_back(b);
    }
}

void test_gps_begin_switches_baud_and_configures_nav_pvt()
{
    FakeUart uart;
    UbloxM10_Gps gps(uart);
    TEST_ASSERT_TRUE(gps.begin());
    TEST_ASSERT_EQUAL(2u, uart.beginCalls);
    TEST_ASSERT_EQUAL_UINT32(115200, uart.baud);
    TEST_ASSERT_EQUAL_STRING("u-blox M10 (UBX-M10050-KB)", gps.getSensorType());

    // Первое сообщение — CFG-VALSET (RAM) UART1_BAUDRATE = 115200.
    const std::vector<uint8_t> baud = ubx(0x06, 0x8A, { 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40,
                                                         0x00, 0xC2, 0x01, 0x00 });
    TEST_ASSERT_TRUE(uart.tx.size() > baud.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(baud.data(), uart.tx.data(), baud.size());

    // Второе — 10 Гц, NAV-PVT на UART1, UBX вкл, NMEA выкл.
    const std::vector<uint8_t> config = ubx(0x06, 0x8A, {
        0x00, 0x01, 0x00, 0x00,
        0x01, 0x00, 0x21, 0x30, 0x64, 0x00,     // RATE_MEAS = 100 мс
        0x02, 0x00, 0x21, 0x30, 0x01, 0x00,     // RATE_NAV = 1
        0x07, 0x00, 0x91, 0x20, 0x01,           // MSGOUT NAV-PVT UART1 = 1
        0x01, 0x00, 0x74, 0x10, 0x01,           // UART1OUTPROT UBX = 1
        0x02, 0x00, 0x74, 0x10, 0x00 });        // UART1OUTPROT NMEA = 0
    TEST_ASSERT_EQUAL(baud.size() + config.size(), uart.tx.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(config.data(), uart.tx.data() + baud.size(), config.size());
}

void test_gps_parses_nav_pvt_and_times_out()
{
    FakeUart uart;
    UbloxM10_Gps gps(uart);
    TEST_ASSERT_FALSE(gps.isAvailable());

    const std::vector<uint8_t> frame = ubx(0x01, 0x07, navPvt(3));
    feed(uart, std::vector<uint8_t>(frame.begin(), frame.begin() + 40));
    gps.update();
    TEST_ASSERT_FALSE(gps.isAvailable());   // кадр ещё не весь
    feed(uart, std::vector<uint8_t>(frame.begin() + 40, frame.end()));
    gps.update();

    TEST_ASSERT_TRUE(gps.isAvailable());
    TEST_ASSERT_TRUE(gps.hasFix());
    const GpsData& d = gps.getGpsData();
    TEST_ASSERT_EQUAL(3, d.fixType);
    TEST_ASSERT_EQUAL(12, d.numSatellites);
    TEST_ASSERT_TRUE(fabs(d.longitude - 37.6173) < 1e-7);
    TEST_ASSERT_TRUE(fabs(d.latitude - 55.7558) < 1e-7);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 150.25f, d.altitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, d.horizontalAccuracy);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.0f, d.verticalAccuracy);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 12.345f, d.groundSpeed);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 270.0f, d.heading);

    gps.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "GPS: available=YES fix=3 numSV=12 lat=55.755800"));

    // Модуль замолчал: через GPS_TIMEOUT_US — недоступен (и статус не врёт).
    fake::advanceUs(Config::GPS_TIMEOUT_US + 1);
    TEST_ASSERT_FALSE(gps.isAvailable());
    TEST_ASSERT_FALSE(gps.hasFix());
    gps.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "GPS: available=NO"));
}

void test_gps_ignores_corrupt_foreign_and_malformed_frames()
{
    FakeUart uart;
    UbloxM10_Gps gps(uart);

    feed(uart, ubx(0x01, 0x07, navPvt(3), true));             // CRC
    feed(uart, ubx(0x01, 0x07, std::vector<uint8_t>(90, 0))); // не та длина
    feed(uart, ubx(0x05, 0x01, { 0x06, 0x8A }));              // ACK — не наш
    feed(uart, ubx(0x0A, 0x04, {}));                          // пустой payload
    feed(uart, { 0xB5, 0x00 });                               // сбой синхронизации
    feed(uart, { 0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF });       // длина > 512 — мусор
    gps.update();
    TEST_ASSERT_FALSE(gps.isAvailable());

    feed(uart, ubx(0x01, 0x07, navPvt(2)));
    gps.update();
    TEST_ASSERT_TRUE(gps.isAvailable());
    TEST_ASSERT_TRUE(gps.hasFix());   // 2D — тоже фикс

    feed(uart, ubx(0x01, 0x07, navPvt(0)));
    gps.update();
    TEST_ASSERT_FALSE(gps.hasFix());
}

// ------------------------------------------------------------
// Выбор датчиков и интерфейс воздушной скорости
// ------------------------------------------------------------

void test_sensor_selection_defaults()
{
    static_assert(std::is_same<SelectedImu, MPU6050_Sensor>::value, "IMU по умолчанию — MPU6050/6500");
    static_assert(std::is_same<SelectedBaro, BMP388_Sensor>::value, "барометр по умолчанию — BMP388");
    static_assert(std::is_same<SelectedMag, QMC5883P_Sensor>::value, "компас по умолчанию — QMC5883P");
    static_assert(SENSOR_GPS == SENSOR_GPS_NONE, "GPS по умолчанию не подключён");

    FakeBoard board;
    I2cRegisterDevice imuDevice = SELECTED_IMU_DEVICE(board);
    I2cRegisterDevice baroDevice = SELECTED_BARO_DEVICE(board);
    I2cRegisterDevice magDevice = SELECTED_MAG_DEVICE(board);
    TEST_ASSERT_EQUAL_HEX8(0x68, imuDevice.getAddress());
    TEST_ASSERT_EQUAL_HEX8(0x76, baroDevice.getAddress());
    TEST_ASSERT_EQUAL_HEX8(0x2C, magDevice.getAddress());
}

void test_airspeed_interface_can_be_implemented()
{
    struct PitotStub : AirspeedSensor
    {
        AirspeedData data = { 30.0f, 7.0f, 0 };
        bool begin() override { return true; }
        bool isAvailable() const override { return true; }
        void update() override {}
        const char* getSensorType() const override { return "stub"; }
        void printStatus() const override {}
        const AirspeedData& getAirspeedData() const override { return data; }
        void calibrateZero() override { data.differentialPressurePa = 0; }
    };
    PitotStub pitot;
    AirspeedSensor& sensor = pitot;
    sensor.calibrateZero();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sensor.getAirspeedData().differentialPressurePa);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, sensor.getAirspeedData().indicatedMs);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_barometer_does_nothing_before_begin);
    RUN_TEST(test_barometer_polls_at_its_period_and_only_new_samples);
    RUN_TEST(test_barometer_relative_altitude_and_filtered_climb);
    RUN_TEST(test_barometer_errors_and_recovery);
    RUN_TEST(test_barometer_calibration_failure_and_sea_level_pressure);
    RUN_TEST(test_bmp388_i2c_configuration_and_compensation);
    RUN_TEST(test_bmp388_spi_skips_dummy_byte);
    RUN_TEST(test_bmp388_failure_paths);
    RUN_TEST(test_bme280_compensation_matches_datasheet_example);
    RUN_TEST(test_bmp280_uses_same_driver_without_humidity);
    RUN_TEST(test_bme280_failure_paths);
    RUN_TEST(test_qmc5883p_configuration_and_heading);
    RUN_TEST(test_magnetometer_polls_at_50hz_and_counts_errors);
    RUN_TEST(test_magnetometer_hard_iron_calibration_persists_in_nvs);
    RUN_TEST(test_magnetometer_calibration_without_samples_keeps_old_one);
    RUN_TEST(test_qmc5883p_failure_paths);
    RUN_TEST(test_qmc5883l_register_layout);
    RUN_TEST(test_sensor_mounting_rotations);
    RUN_TEST(test_gps_begin_switches_baud_and_configures_nav_pvt);
    RUN_TEST(test_gps_parses_nav_pvt_and_times_out);
    RUN_TEST(test_gps_ignores_corrupt_foreign_and_malformed_frames);
    RUN_TEST(test_sensor_selection_defaults);
    RUN_TEST(test_airspeed_interface_can_be_implemented);
    return UNITY_END();
}
