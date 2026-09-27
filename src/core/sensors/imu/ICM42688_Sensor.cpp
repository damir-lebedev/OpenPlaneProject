// Реализация sensors/imu/ICM42688_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/imu/ICM42688_Sensor.h"


auto ICM42688_Sensor::spiDevice(ISpiBus& bus, uint8_t chipSelectPin) -> SpiRegisterDevice
{
    return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
}

ICM42688_Sensor::ICM42688_Sensor(IRegisterDevice& registerDevice)
: ImuSensorBase("ICM42688", "imu_icm42688"),
      device(registerDevice)
{
}

auto ICM42688_Sensor::begin() -> bool
{
    device.begin();
    delay(10);

    device.writeRegister(REG_BANK_SEL, 0x00);        // банк 0 — чип может включиться в другом
    device.writeRegister(REG_DEVICE_CONFIG, 0x01);   // SOFT_RESET
    delay(2);                                        // датащит: ≥1 мс после сброса
    device.writeRegister(REG_BANK_SEL, 0x00);

    const int whoAmI = device.readRegister(REG_WHO_AM_I);
    if (whoAmI != WHO_AM_I_VALUE)
    {
        Serial.print("ICM42688: неверный WHO_AM_I ");
        Serial.println(whoAmI < 0 ? String("(нет ответа)") : String("0x") + String(whoAmI, HEX));
        setAvailable(false);
        return false;
    }

    bool ok = true;

    // GYRO_MODE=11 и ACCEL_MODE=11 (Low Noise). Датащит требует
    // ≥200 мкс после включения режимов до других записей.
    ok &= device.writeRegister(REG_PWR_MGMT0, 0x0F);
    delay(1);

    // FS_SEL=000 -> ±2000°/с / ±16g; ODR=0110 -> 1 кГц.
    ok &= device.writeRegister(REG_GYRO_CONFIG0, 0x06);
    ok &= device.writeRegister(REG_ACCEL_CONFIG0, 0x06);

    // Полоса UI-фильтров: 6 = ODR/20 = 50 Гц для accel и gyro —
    // по смыслу как DLPF ~41 Гц у MPU6050.
    ok &= device.writeRegister(REG_GYRO_ACCEL_CONFIG0, 0x66);

    if (!ok)
    {
        Serial.println("ICM42688: ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    Serial.println("ICM42688: подключён");
    setAvailable(true);
    return true;
}

auto ICM42688_Sensor::readSample(RawImuSample& s) -> bool
{
    uint8_t b[14];  // temp, accel XYZ, gyro XYZ — big-endian
    if (!device.readRegisters(REG_TEMP_DATA1, b, sizeof(b))) return false;

    s.temperature = be16(b + 0);
    s.accelX      = be16(b + 2);
    s.accelY      = be16(b + 4);
    s.accelZ      = be16(b + 6);
    s.gyroX       = be16(b + 8);
    s.gyroY       = be16(b + 10);
    s.gyroZ       = be16(b + 12);
    return true;
}

auto ICM42688_Sensor::temperatureC(int16_t raw) const -> float
{
    return raw / 132.48f + 25.0f;  // датащит ICM-42688-P
}

auto ICM42688_Sensor::be16(const uint8_t* p) -> int16_t
{
    return static_cast<int16_t>((p[0] << 8) | p[1]);
}
