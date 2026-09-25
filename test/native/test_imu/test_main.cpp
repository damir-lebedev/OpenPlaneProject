// ============================================================
// IMU: драйверы MPU6050/MPU6500 (I2C) и ICM-42688 (SPI/I2C) поверх
// симулированных чипов, общая часть ImuSensorBase (масштаб, поворот
// осей, авиационные знаки, калибровки, предполётная проверка,
// ошибки шины), ImuOrientation в NVS и AttitudeEstimator.
//
// Запуск: pio test -e native -f native/test_imu
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "sensors/imu/AttitudeEstimator.h"
#include "sensors/imu/ICM42688_Sensor.h"
#include "sensors/imu/ImuOrientation.h"
#include "sensors/imu/MPU6050_Sensor.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    constexpr float ACCEL_LSB = 2048.0f;   // ±16g
    constexpr float GYRO_LSB = 16.4f;      // ±2000°/с

    // "Верх" (показание акселерометра в покое, g) и угловые скорости
    // (°/с) в осях чипа — в регистры MPU (0x3B.., big-endian).
    void setMpu(fake::RegisterMapDevice& chip, float ax, float ay, float az,
                float gx = 0, float gy = 0, float gz = 0, int16_t temp = 0)
    {
        chip.setBigEndian16(0x3B, static_cast<int16_t>(ax * ACCEL_LSB));
        chip.setBigEndian16(0x3D, static_cast<int16_t>(ay * ACCEL_LSB));
        chip.setBigEndian16(0x3F, static_cast<int16_t>(az * ACCEL_LSB));
        chip.setBigEndian16(0x41, temp);
        chip.setBigEndian16(0x43, static_cast<int16_t>(gx * GYRO_LSB));
        chip.setBigEndian16(0x45, static_cast<int16_t>(gy * GYRO_LSB));
        chip.setBigEndian16(0x47, static_cast<int16_t>(gz * GYRO_LSB));
    }

    struct Mpu
    {
        I2cRig rig{ 0x68 };
        MPU6050_Sensor sensor{ rig.device };

        explicit Mpu(int whoAmI = 0x70)
        {
            if (whoAmI >= 0) rig.chip.regs[0x75] = static_cast<uint8_t>(whoAmI);
            else rig.chip.present = false;
            setMpu(rig.chip, 0, 0, 1);   // лежит ровно, чипом вверх
        }
    };

    float rad(float degrees) { return degrees * static_cast<float>(DEG_TO_RAD); }
}

// ------------------------------------------------------------
// MPU6050 / MPU6500: опознание и настройка
// ------------------------------------------------------------

void test_mpu_not_responding_is_unavailable()
{
    Mpu mpu(-1);
    TEST_ASSERT_FALSE(mpu.sensor.begin());
    TEST_ASSERT_FALSE(mpu.sensor.isAvailable());
    TEST_ASSERT_TRUE(contains(takeSerial(), "MPU6050: датчик не отвечает"));

    // Недоступный датчик ничего не читает и не калибруется.
    mpu.sensor.update();
    mpu.sensor.calibrate();
    TEST_ASSERT_EQUAL_UINT32(0, mpu.rig.chip.readTransactions);
    mpu.sensor.calibrateOrientation();
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка установки невозможна"));
}

void test_mpu6500_is_detected_and_configured()
{
    Mpu mpu(0x70);
    TEST_ASSERT_TRUE(mpu.sensor.begin());
    TEST_ASSERT_TRUE(mpu.sensor.isAvailable());
    TEST_ASSERT_EQUAL_STRING("MPU6500", mpu.sensor.getSensorType());

    fake::RegisterMapDevice& chip = mpu.rig.chip;
    TEST_ASSERT_EQUAL(0x01, chip.lastWrite(0x6B));   // из sleep, PLL
    TEST_ASSERT_EQUAL(0x18, chip.lastWrite(0x1B));   // ±2000°/с
    TEST_ASSERT_EQUAL(0x18, chip.lastWrite(0x1C));   // ±16g
    TEST_ASSERT_EQUAL(0x03, chip.lastWrite(0x1A));   // DLPF ~41 Гц
    TEST_ASSERT_EQUAL(0x00, chip.lastWrite(0x19));   // 1 кГц
    TEST_ASSERT_EQUAL(0x03, chip.lastWrite(0x1D));   // отдельный ФНЧ акселерометра 6500
    TEST_ASSERT_TRUE(contains(takeSerial(), "WHO_AM_I=0x70 -> MPU6500"));
}

void test_mpu_chip_names_and_temperature_formulas()
{
    struct Case { int id; const char* name; };
    const Case cases[] = { { 0x68, "MPU6050" }, { 0x71, "MPU9250" }, { 0x73, "MPU9255" },
                           { 0x12, "MPU6500-совместимый клон" } };
    for (const Case& c : cases)
    {
        resetWorld();
        Mpu mpu(c.id);
        TEST_ASSERT_TRUE(mpu.sensor.begin());
        TEST_ASSERT_EQUAL_STRING(c.name, mpu.sensor.getSensorType());
    }

    // MPU6050: без ACCEL_CONFIG2, температура raw/340 + 36.53.
    resetWorld();
    Mpu mpu6050(0x68);
    mpu6050.sensor.begin();
    TEST_ASSERT_EQUAL(-1, mpu6050.rig.chip.lastWrite(0x1D));
    setMpu(mpu6050.rig.chip, 0, 0, 1, 0, 0, 0, 340);
    mpu6050.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 37.53f, mpu6050.sensor.getImuData().temperature);

    resetWorld();
    Mpu mpu6500(0x70);
    mpu6500.sensor.begin();
    setMpu(mpu6500.rig.chip, 0, 0, 1, 0, 0, 0, 0);
    mpu6500.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.0f, mpu6500.sensor.getImuData().temperature);
}

void test_mpu_register_write_failure_makes_it_unavailable()
{
    Mpu mpu;
    mpu.rig.chip.failWrites = true;
    TEST_ASSERT_FALSE(mpu.sensor.begin());
    TEST_ASSERT_FALSE(mpu.sensor.isAvailable());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ошибка записи регистров"));
}

// ------------------------------------------------------------
// ImuSensorBase: данные, оси, знаки
// ------------------------------------------------------------

// Без калибровки установки — поворот из Config (плата чипом вверх).
// При IMU_ROTATION_CW_DEG = 90 ось X чипа смотрит вправо.
void test_config_mounting_maps_chip_axes_to_aviation_signs()
{
    TEST_ASSERT_EQUAL(90, Config::IMU_ROTATION_CW_DEG);
    Mpu mpu;
    TEST_ASSERT_TRUE(mpu.sensor.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "установка из Config"));
    mpu.sensor.calibrate();
    TEST_ASSERT_NULL(mpu.sensor.getPreflightProblem());

    // Нос вверх на 30°: "верх" в осях самолёта (sin30, 0, cos30); ось X
    // самолёта = ось Y чипа, ось Y самолёта (влево) = −X чипа.
    setMpu(mpu.rig.chip, 0, sinf(rad(30)), cosf(rad(30)));
    mpu.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, mpu.sensor.getImuData().pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, mpu.sensor.getImuData().roll);

    // Вращение вокруг оси X чипа (правое крыло) — тангаж, нос вверх.
    resetWorld();
    Mpu turning;
    turning.sensor.begin();
    turning.sensor.calibrate();
    setMpu(turning.rig.chip, 0, 0, 1, 10, 0, 0);
    turning.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, turning.sensor.getImuData().gyroY);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, turning.sensor.getImuData().gyroX);

    // Вокруг Z чипа (вверх) против часовой — нос влево: рысканье < 0.
    setMpu(turning.rig.chip, 0, 0, 1, 0, 0, 20);
    turning.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -20.0f, turning.sensor.getImuData().gyroZ);
    TEST_ASSERT_UINT32_WITHIN(1, micros(), turning.sensor.getImuData().timestamp);
}

void test_gyro_calibration_removes_offset()
{
    Mpu mpu;
    mpu.sensor.begin();
    setMpu(mpu.rig.chip, 0, 0, 1, 1.5f, -2.0f, 0.5f);   // смещение нуля
    mpu.sensor.calibrate();
    mpu.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, mpu.sensor.getImuData().gyroX);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, mpu.sensor.getImuData().gyroY);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, mpu.sensor.getImuData().gyroZ);
    TEST_ASSERT_TRUE(contains(takeSerial(), "предполётная проверка пройдена"));
}

void test_set_yaw_seeds_heading()
{
    Mpu mpu;
    mpu.sensor.begin();
    mpu.sensor.setYaw(200.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -160.0f, mpu.sensor.getImuData().yaw);
}

void test_bus_errors_keep_last_data_and_flip_availability()
{
    Mpu mpu;
    mpu.sensor.begin();
    mpu.sensor.calibrate();
    setMpu(mpu.rig.chip, 0, sinf(rad(20)), cosf(rad(20)));
    mpu.sensor.update();
    const float pitch = mpu.sensor.getImuData().pitch;

    mpu.rig.chip.failReads = -1;
    for (int i = 0; i < 49; ++i) mpu.sensor.update();
    TEST_ASSERT_TRUE(mpu.sensor.isAvailable());          // ещё терпим
    mpu.sensor.update();
    TEST_ASSERT_FALSE(mpu.sensor.isAvailable());         // ~0.1 с без ответа
    TEST_ASSERT_EQUAL_FLOAT(pitch, mpu.sensor.getImuData().pitch);   // не мусор

    mpu.rig.chip.failReads = 0;
    mpu.sensor.update();
    TEST_ASSERT_TRUE(mpu.sensor.isAvailable());

    takeSerial();
    mpu.sensor.printStatus();
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE(contains(status, "MPU6500: available=YES calibrated=YES mounting=CONFIG errors=50"));
    TEST_ASSERT_TRUE(contains(status, "preflight=OK"));
}

// ------------------------------------------------------------
// Предполётная проверка
// ------------------------------------------------------------

void test_preflight_detects_motion_during_calibration()
{
    Mpu mpu;
    mpu.sensor.begin();
    bool flip = false;
    mpu.rig.chip.beforeRead = [&](uint8_t reg, size_t) {
        if (reg != 0x3B) return;
        flip = !flip;
        setMpu(mpu.rig.chip, 0, 0, 1, flip ? 3.0f : -3.0f, 0, 0);   // самолёт качают
    };
    mpu.sensor.calibrate();
    TEST_ASSERT_TRUE(contains(mpu.sensor.getPreflightProblem(), "двигали"));
    TEST_ASSERT_TRUE(contains(takeSerial(), "ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА"));
}

void test_preflight_detects_wrong_gravity_and_upside_down_board()
{
    Mpu heavy;
    heavy.sensor.begin();
    setMpu(heavy.rig.chip, 0, 0, 1.5f);
    heavy.sensor.calibrate();
    TEST_ASSERT_TRUE(contains(heavy.sensor.getPreflightProblem(), "не 1g"));

    resetWorld();
    Mpu flipped;
    flipped.sensor.begin();
    setMpu(flipped.rig.chip, 0, 0, -1);
    flipped.sensor.calibrate();
    TEST_ASSERT_TRUE(contains(flipped.sensor.getPreflightProblem(), "не лежит чипом вверх"));
}

void test_preflight_reports_not_responding_calibration()
{
    Mpu mpu;
    mpu.sensor.begin();
    mpu.rig.chip.failReads = -1;
    mpu.sensor.calibrate();
    TEST_ASSERT_TRUE(contains(mpu.sensor.getPreflightProblem(), "калибровка не удалась"));
    TEST_ASSERT_TRUE(contains(takeSerial(), "датчик не отвечает"));
}

// ------------------------------------------------------------
// Калибровка установки по трём позам
// ------------------------------------------------------------

namespace
{
    // Пилот ставит самолёт: ровно, нос вверх, правое крыло вниз — по
    // расписанию симулированного времени. Плата стоит как у самолёта
    // (ось X чипа — к носу), в отличие от Config (90°).
    struct PosePilot
    {
        uint64_t startUs;
        float noseDeg;
        float wingDeg;
        bool holdLevelForever = false;

        void apply(fake::RegisterMapDevice& chip) const
        {
            const float t = (fake::nowUs() - startUs) / 1e6f;
            if (holdLevelForever || t < 3.5f) setMpu(chip, 0, 0, 1);
            else if (t < 5.5f) setMpu(chip, sinf(rad(noseDeg)), 0, cosf(rad(noseDeg)));
            else setMpu(chip, 0, sinf(rad(wingDeg)), cosf(rad(wingDeg)));
        }
    };
}

void test_orientation_calibration_learns_and_persists_mounting()
{
    Mpu mpu;
    mpu.sensor.begin();
    PosePilot pilot{ fake::nowUs(), 40, 35 };
    mpu.rig.chip.beforeRead = [&](uint8_t reg, size_t) { if (reg == 0x3B) pilot.apply(mpu.rig.chip); };
    mpu.sensor.calibrateOrientation();
    mpu.rig.chip.beforeRead = nullptr;

    const std::string log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "установка сохранена: нос = +X чипа, верх = +Z чипа"));
    TEST_ASSERT_TRUE(fake::nvs().spaces.count("imu_mpu6050") == 1);

    // Теперь "нос вверх" читается по новой установке, а не по Config.
    setMpu(mpu.rig.chip, sinf(rad(25)), 0, cosf(rad(25)));
    mpu.sensor.update();
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 25.0f, mpu.sensor.getImuData().pitch);

    // После перезагрузки установка берётся из NVS.
    Mpu rebooted;
    rebooted.sensor.begin();
    TEST_ASSERT_TRUE(contains(takeSerial(), "установка из калибровки"));
    rebooted.sensor.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "mounting=CALIBRATED"));
}

void test_orientation_calibration_rejects_wrong_wing()
{
    Mpu mpu;
    mpu.sensor.begin();
    PosePilot pilot{ fake::nowUs(), 40, -35 };   // опустили левое крыло
    mpu.rig.chip.beforeRead = [&](uint8_t reg, size_t) { if (reg == 0x3B) pilot.apply(mpu.rig.chip); };
    mpu.sensor.calibrateOrientation();
    TEST_ASSERT_TRUE(contains(takeSerial(), "калибровка установки отклонена"));
    TEST_ASSERT_TRUE(fake::nvs().spaces["imu_mpu6050"].empty());
}

void test_orientation_calibration_times_out_without_poses()
{
    Mpu mpu;
    mpu.sensor.begin();
    PosePilot pilot{ fake::nowUs(), 40, 35, true };   // так и не наклонили
    mpu.rig.chip.beforeRead = [&](uint8_t reg, size_t) { if (reg == 0x3B) pilot.apply(mpu.rig.chip); };
    mpu.sensor.calibrateOrientation();
    TEST_ASSERT_TRUE(contains(takeSerial(), "поза не дождалась за 30 с"));
}

// Установка откалибрована, а при включении "верх" оказался в другом
// месте — плату переставили. Новая калибровка установки снимает проблему.
void test_mounting_mismatch_after_board_moved()
{
    Mpu mpu;
    mpu.sensor.begin();
    PosePilot pilot{ fake::nowUs(), 40, 35 };
    mpu.rig.chip.beforeRead = [&](uint8_t reg, size_t) { if (reg == 0x3B) pilot.apply(mpu.rig.chip); };
    mpu.sensor.calibrateOrientation();
    mpu.rig.chip.beforeRead = nullptr;

    setMpu(mpu.rig.chip, 1, 0, 0);   // плата стоит на ребре
    mpu.sensor.calibrate();
    TEST_ASSERT_TRUE(contains(mpu.sensor.getPreflightProblem(), "плату переставили"));

    pilot.startUs = fake::nowUs();
    mpu.rig.chip.beforeRead = [&](uint8_t reg, size_t) { if (reg == 0x3B) pilot.apply(mpu.rig.chip); };
    mpu.sensor.calibrateOrientation();
    TEST_ASSERT_NULL(mpu.sensor.getPreflightProblem());
}

// ------------------------------------------------------------
// ICM-42688
// ------------------------------------------------------------

namespace
{
    void setIcm(uint8_t* regs, float ax, float ay, float az, float gx, int16_t temp)
    {
        auto put = [&](uint8_t reg, int16_t v) {
            regs[reg] = static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8);
            regs[reg + 1] = static_cast<uint8_t>(v & 0xFF);
        };
        put(0x1D, temp);
        put(0x1F, static_cast<int16_t>(ax * ACCEL_LSB));
        put(0x21, static_cast<int16_t>(ay * ACCEL_LSB));
        put(0x23, static_cast<int16_t>(az * ACCEL_LSB));
        put(0x25, static_cast<int16_t>(gx * GYRO_LSB));
        put(0x27, 0);
        put(0x29, 0);
    }
}

void test_icm42688_over_spi_is_configured_and_read()
{
    SpiRig rig(Config::PIN_SPI_CS_ICM42688, 0);
    rig.chip.regs[0x75] = 0x47;
    setIcm(rig.chip.regs, 0, 0, 1, 0, 0);

    SpiRegisterDevice device = ICM42688_Sensor::spiDevice(rig.bus, Config::PIN_SPI_CS_ICM42688);
    ICM42688_Sensor icm(device);
    TEST_ASSERT_TRUE(icm.begin());
    TEST_ASSERT_EQUAL_STRING("ICM42688", icm.getSensorType());
    TEST_ASSERT_EQUAL_UINT32(8000000, SPI.settings().clock);
    TEST_ASSERT_EQUAL(0x01, rig.chip.lastWrite(0x11));   // SOFT_RESET
    TEST_ASSERT_EQUAL(0x0F, rig.chip.lastWrite(0x4E));   // Low Noise
    TEST_ASSERT_EQUAL(0x06, rig.chip.lastWrite(0x4F));
    TEST_ASSERT_EQUAL(0x06, rig.chip.lastWrite(0x50));
    TEST_ASSERT_EQUAL(0x66, rig.chip.lastWrite(0x52));
    TEST_ASSERT_EQUAL(0x00, rig.chip.lastWrite(0x76));   // банк 0

    icm.calibrate();
    setIcm(rig.chip.regs, 0, 0, 1, 10, 1325);   // 1325/132.48 + 25 ≈ 35 °C
    icm.update();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, icm.getImuData().gyroY);   // тот же поворот 90°
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 35.0f, icm.getImuData().temperature);
}

void test_icm42688_rejects_wrong_or_missing_chip()
{
    SpiRig rig(Config::PIN_SPI_CS_ICM42688, 0);
    rig.chip.regs[0x75] = 0x12;
    SpiRegisterDevice device = ICM42688_Sensor::spiDevice(rig.bus, Config::PIN_SPI_CS_ICM42688);
    ICM42688_Sensor wrong(device);
    TEST_ASSERT_FALSE(wrong.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "неверный WHO_AM_I 0x12"));

    I2cRig absent(0x69);
    absent.chip.present = false;
    ICM42688_Sensor missing(absent.device);
    TEST_ASSERT_FALSE(missing.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "(нет ответа)"));

    I2cRig readOnly(0x68);
    readOnly.chip.regs[0x75] = 0x47;
    readOnly.chip.failWrites = true;
    ICM42688_Sensor failing(readOnly.device);
    TEST_ASSERT_FALSE(failing.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ICM42688: ошибка записи регистров"));
}

// ------------------------------------------------------------
// ImuOrientation и NVS
// ------------------------------------------------------------

void test_orientation_nvs_round_trip_and_validation()
{
    const float level[3] = { 0, 0, 1 };
    const float nose[3] = { sinf(rad(40)), 0, cosf(rad(40)) };
    const float wing[3] = { 0, sinf(rad(35)), cosf(rad(35)) };
    ImuOrientation calibrated;
    TEST_ASSERT_NULL(ImuOrientation::fromPoses(level, nose, wing, calibrated));
    calibrated.save("imu_test");

    ImuOrientation loaded = ImuOrientation::fromYawSteps(180);
    TEST_ASSERT_TRUE(loaded.load("imu_test"));
    const float chip[3] = { 0.1f, 0.2f, 0.97f };
    float a[3], b[3];
    calibrated.apply(chip, a);
    loaded.apply(chip, b);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, a[0], b[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, a[2], b[2]);

    // Нет пространства, битый флаг, неверный размер, не поворот — отказ.
    ImuOrientation other;
    TEST_ASSERT_FALSE(other.load("imu_missing"));
    {
        Preferences p;
        p.begin("imu_bad", false);
        const float garbage[9] = { 2, 0, 0, 0, 2, 0, 0, 0, 2 };
        p.putBytes("r", garbage, sizeof(garbage));
        p.putBool("ok", true);
        p.end();
    }
    TEST_ASSERT_FALSE(other.load("imu_bad"));
    {
        Preferences p;
        p.begin("imu_short", false);
        p.putBytes("r", level, sizeof(level));
        p.putBool("ok", true);
        p.end();
    }
    TEST_ASSERT_FALSE(other.load("imu_short"));

    fake::nvs().failBegin = true;
    calibrated.save("imu_other");   // NVS недоступно — просто не сохраняется
    TEST_ASSERT_FALSE(other.load("imu_test"));
    fake::nvs().failBegin = false;
    TEST_ASSERT_EQUAL(0u, fake::nvs().spaces.count("imu_other"));
}

void test_orientation_describe_and_degenerate_inputs()
{
    const ImuOrientation rotated = ImuOrientation::fromYawSteps(270);
    rotated.describe(Serial);
    TEST_ASSERT_EQUAL_STRING("нос = -Y чипа, верх = +Z чипа", takeSerial().c_str());

    // Ось, не совпадающая с осью чипа, описывается с углом.
    const float level[3] = { 0, 0, 1 };
    const float nose[3] = { sinf(rad(40)) * cosf(rad(30)), sinf(rad(40)) * sinf(rad(30)), cosf(rad(40)) };
    const float wing[3] = { -sinf(rad(35)) * sinf(rad(30)), sinf(rad(35)) * cosf(rad(30)), cosf(rad(35)) };
    ImuOrientation skewed;
    TEST_ASSERT_NULL(ImuOrientation::fromPoses(level, nose, wing, skewed));
    skewed.describe(Serial);
    TEST_ASSERT_TRUE(contains(takeSerial(), "(под углом 30°)"));

    const float zero[3] = { 0, 0, 0 };
    TEST_ASSERT_EQUAL_FLOAT(180.0f, rotated.tiltFromLevelDeg(zero));
    ImuOrientation out;
    TEST_ASSERT_EQUAL_STRING("нет показаний акселерометра", ImuOrientation::fromPoses(zero, nose, wing, out));
    TEST_ASSERT_EQUAL_STRING("в шаге 3 нужен наклон 30-60°", ImuOrientation::fromPoses(level, nose, level, out));
}

// ------------------------------------------------------------
// AttitudeEstimator
// ------------------------------------------------------------

void test_attitude_starts_from_accelerometer_and_fuses_gyro()
{
    AttitudeEstimator est;
    // Нос вверх 20°, крен 0: сразу угол по акселерометру.
    est.update(sinf(rad(20)), 0, cosf(rad(20)), 0, 0, 0, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, est.getPitch());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, est.getRoll());

    // Крен вправо по акселерометру: "верх" уходит влево (+Y).
    AttitudeEstimator banked;
    banked.update(0, sinf(rad(15)), cosf(rad(15)), 0, 0, 0, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, banked.getRoll());

    // Гироскоп: 100 °/с крена 0.1 с при неподвижном акселерометре —
    // угол растёт, но фильтр тянет к акселерометру.
    AttitudeEstimator fused;
    uint32_t t = 1000;
    fused.update(0, 0, 1, 0, 0, 0, t);
    for (int i = 0; i < 50; ++i)
    {
        t += 2000;
        fused.update(0, 0, 1, 100, 0, 30, t);
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(2.0f, fused.getRoll());
    TEST_ASSERT_LESS_THAN_FLOAT(10.0f, fused.getRoll());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, fused.getYaw());   // чистый интеграл: 30 °/с · 0.1 с
}

void test_attitude_skips_bad_dt_and_wraps_yaw()
{
    AttitudeEstimator est;
    est.update(0, 0, 1, 0, 0, 0, 1000);
    est.update(0, 0, 1, 1000, 0, 0, 1000);     // dt = 0
    est.update(0, 0, 1, 1000, 0, 0, 500000);   // dt ~0.5 с (пауза)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, est.getRoll());

    est.setYaw(179.0f);
    est.update(0, 0, 1, 0, 0, 100, 520000);    // +2°
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -179.0f, est.getYaw());
    est.setYaw(-190.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 170.0f, est.getYaw());

    est.reset();
    est.update(sinf(rad(-10)), 0, cosf(rad(-10)), 0, 0, 0, 530000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, est.getPitch());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_mpu_not_responding_is_unavailable);
    RUN_TEST(test_mpu6500_is_detected_and_configured);
    RUN_TEST(test_mpu_chip_names_and_temperature_formulas);
    RUN_TEST(test_mpu_register_write_failure_makes_it_unavailable);
    RUN_TEST(test_config_mounting_maps_chip_axes_to_aviation_signs);
    RUN_TEST(test_gyro_calibration_removes_offset);
    RUN_TEST(test_set_yaw_seeds_heading);
    RUN_TEST(test_bus_errors_keep_last_data_and_flip_availability);
    RUN_TEST(test_preflight_detects_motion_during_calibration);
    RUN_TEST(test_preflight_detects_wrong_gravity_and_upside_down_board);
    RUN_TEST(test_preflight_reports_not_responding_calibration);
    RUN_TEST(test_orientation_calibration_learns_and_persists_mounting);
    RUN_TEST(test_orientation_calibration_rejects_wrong_wing);
    RUN_TEST(test_orientation_calibration_times_out_without_poses);
    RUN_TEST(test_mounting_mismatch_after_board_moved);
    RUN_TEST(test_icm42688_over_spi_is_configured_and_read);
    RUN_TEST(test_icm42688_rejects_wrong_or_missing_chip);
    RUN_TEST(test_orientation_nvs_round_trip_and_validation);
    RUN_TEST(test_orientation_describe_and_degenerate_inputs);
    RUN_TEST(test_attitude_starts_from_accelerometer_and_fuses_gyro);
    RUN_TEST(test_attitude_skips_bad_dt_and_wraps_yaw);
    return UNITY_END();
}
