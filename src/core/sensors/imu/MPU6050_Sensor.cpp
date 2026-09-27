// Реализация sensors/imu/MPU6050_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/imu/MPU6050_Sensor.h"


MPU6050_Sensor::MPU6050_Sensor(IRegisterDevice& registerDevice)
: ImuSensorBase("MPU6050", "imu_mpu6050"),
      device(registerDevice)
{
}

auto MPU6050_Sensor::begin() -> bool
{
    device.begin();

    const int whoAmI = device.readRegister(REG_WHO_AM_I);
    if (whoAmI < 0)
    {
        Serial.println("MPU6050: датчик не отвечает, IMU недоступен");
        setAvailable(false);
        return false;
    }

    isMpu6500Family = (whoAmI != WHO_AM_I_MPU6050);
    setName(chipName(whoAmI));

    Serial.print("MPU6050: WHO_AM_I=0x");
    Serial.print(whoAmI, HEX);
    Serial.print(" -> ");
    Serial.println(chipName(whoAmI));

    if (!configure())
    {
        Serial.println("MPU6050: ошибка записи регистров, IMU недоступен");
        setAvailable(false);
        return false;
    }

    setAvailable(true);
    return true;
}

auto MPU6050_Sensor::readSample(RawImuSample& s) -> bool
{
    uint8_t b[14];  // accel XYZ, temp, gyro XYZ — big-endian
    if (!device.readRegisters(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;

    s.accelX      = be16(b + 0);
    s.accelY      = be16(b + 2);
    s.accelZ      = be16(b + 4);
    s.temperature = be16(b + 6);
    s.gyroX       = be16(b + 8);
    s.gyroY       = be16(b + 10);
    s.gyroZ       = be16(b + 12);
    return true;
}

auto MPU6050_Sensor::temperatureC(int16_t raw) const -> float
{
    return isMpu6500Family
        ? raw / 333.87f + 21.0f     // датащит MPU6500
        : raw / 340.0f + 36.53f;    // датащит MPU6050
}

auto MPU6050_Sensor::configure() -> bool
{
    device.writeRegister(REG_PWR_MGMT_1, 0x80);  // DEVICE_RESET
    delay(100);

    bool ok = true;
    ok &= device.writeRegister(REG_PWR_MGMT_1, 0x01);    // выход из sleep, такт от PLL гироскопа
    delay(10);

    // Широкие диапазоны: в резком манёвре и на вибрации узкие
    // (±250°/с, ±2g) насыщаются и дают мусор в углы.
    ok &= device.writeRegister(REG_GYRO_CONFIG, 0x18);   // ±2000°/с (FS_SEL=3)
    ok &= device.writeRegister(REG_ACCEL_CONFIG, 0x18);  // ±16g (AFS_SEL=3)

    // Цифровой ФНЧ гироскопа DLPF_CFG=3: ~42 Гц (MPU6050) / 41 Гц
    // (MPU6500), задержка ~5 мс. Срезает вибрацию мотора и винта
    // (10x5 на ~8000 об/мин — ~130 Гц), не добавляя заметного
    // запаздывания для стабилизации самолёта.
    ok &= device.writeRegister(REG_CONFIG, 0x03);
    ok &= device.writeRegister(REG_SMPLRT_DIV, 0x00);    // 1 кГц — быстрее цикла

    // У MPU6500 фильтр акселерометра отдельный и по умолчанию
    // почти выключен (218 Гц) — ставим те же ~41 Гц.
    if (isMpu6500Family)
    {
        ok &= device.writeRegister(REG_ACCEL_CONFIG2, 0x03);
    }

    return ok;
}

auto MPU6050_Sensor::chipName(int whoAmI) -> const char*
{
    switch (whoAmI)
    {
        case 0x68: return "MPU6050";
        case 0x70: return "MPU6500";
        case 0x71: return "MPU9250";
        case 0x73: return "MPU9255";
        default:   return "MPU6500-совместимый клон";
    }
}

auto MPU6050_Sensor::be16(const uint8_t* p) -> int16_t
{
    return static_cast<int16_t>((p[0] << 8) | p[1]);
}
