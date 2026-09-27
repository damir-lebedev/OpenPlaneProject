// Реализация sensors/imu/LSM6DSV_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/imu/LSM6DSV_Sensor.h"


auto LSM6DSV_Sensor::spiDevice(ISpiBus& bus, uint8_t chipSelectPin) -> SpiRegisterDevice
{
    return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
}

LSM6DSV_Sensor::LSM6DSV_Sensor(IRegisterDevice& registerDevice)
: ImuSensorBase("LSM6DSV", "imu_lsm6dsv"),
      device(registerDevice)
{
}

auto LSM6DSV_Sensor::begin() -> bool
{
    device.begin();
    delay(BOOT_TIME_MS);

    const int whoAmI = device.readRegister(REG_WHO_AM_I);
    if (whoAmI != WHO_AM_I_VALUE)
    {
        Serial.print("LSM6DSV: неверный WHO_AM_I ");
        Serial.println(whoAmI < 0 ? String("(нет ответа)") : String("0x") + String(whoAmI, HEX));
        return fail();
    }

    if (!softReset())
    {
        Serial.println("LSM6DSV: чип не вышел из сброса");
        return fail();
    }

    const int ctrl8 = device.readRegister(REG_CTRL8);
    if (ctrl8 < 0) return fail();
    is32x = (ctrl8 & CTRL8_VARIANT_32X) != 0;

    const uint8_t fsXl = is32x ? FS_XL_16G_ON_32X : FS_XL_16G_ON_16X;

    const bool ok =
        device.writeRegister(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC) &&
        device.writeRegister(REG_CTRL6, FS_G_2000DPS | (GYRO_LPF1_BW << 4)) &&
        device.writeRegister(REG_CTRL7, CTRL7_LPF1_G_EN) &&
        device.writeRegister(REG_CTRL8, fsXl | (ACCEL_LPF2_BW << 5)) &&
        device.writeRegister(REG_CTRL9, CTRL9_LPF2_XL_EN) &&
        device.writeRegister(REG_CTRL1, ODR_960HZ) &&   // OP_MODE = 0: high performance
        device.writeRegister(REG_CTRL2, ODR_960HZ);

    if (!ok)
    {
        Serial.println("LSM6DSV: ошибка записи регистров");
        return fail();
    }

    Serial.print("LSM6DSV: подключён (");
    Serial.print(is32x ? "LSM6DSV32X" : "LSM6DSV/16X");
    Serial.println(")");
    setAvailable(true);
    return true;
}

auto LSM6DSV_Sensor::readSample(RawImuSample& s) -> bool
{
    uint8_t b[14];  // температура, gyro XYZ, accel XYZ — little-endian
    if (!device.readRegisters(REG_OUT_TEMP_L, b, sizeof(b))) return false;

    s.temperature = le16(b + 0);
    s.gyroX       = le16(b + 2);
    s.gyroY       = le16(b + 4);
    s.gyroZ       = le16(b + 6);
    s.accelX      = le16(b + 8);
    s.accelY      = le16(b + 10);
    s.accelZ      = le16(b + 12);
    return true;
}

auto LSM6DSV_Sensor::temperatureC(int16_t raw) const -> float
{
    return raw / 256.0f + 25.0f;
}

auto LSM6DSV_Sensor::fail() -> bool
{
    setAvailable(false);
    return false;
}

auto LSM6DSV_Sensor::softReset() -> bool
{
    if (!device.writeRegister(REG_CTRL3, CTRL3_SW_RESET)) return false;

    for (uint8_t i = 0; i < RESET_POLL_TRIES; ++i)
    {
        delay(1);
        const int ctrl3 = device.readRegister(REG_CTRL3);
        if (ctrl3 >= 0 && (ctrl3 & CTRL3_SW_RESET) == 0) return true;
    }
    return false;
}

auto LSM6DSV_Sensor::le16(const uint8_t* p) -> int16_t
{
    return static_cast<int16_t>((p[1] << 8) | p[0]);
}
