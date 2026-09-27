// Реализация sensors/imu/ICM45686_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/imu/ICM45686_Sensor.h"


auto ICM45686_Sensor::spiDevice(ISpiBus& bus, uint8_t chipSelectPin) -> SpiRegisterDevice
{
    return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
}

ICM45686_Sensor::ICM45686_Sensor(IRegisterDevice& registerDevice)
: ImuSensorBase("ICM45686", "imu_icm45686"),
      device(registerDevice)
{
}

auto ICM45686_Sensor::begin() -> bool
{
    device.begin();
    delay(3);

    device.writeRegister(REG_MISC2, MISC2_SOFT_RST);
    delay(2);  // TDK: сброс действует через 1 мс

    const int whoAmI = device.readRegister(REG_WHO_AM_I);
    if (whoAmI != WHO_AM_I_VALUE)
    {
        Serial.print("ICM45686: неверный WHO_AM_I ");
        Serial.println(whoAmI < 0 ? String("(нет ответа)") : String("0x") + String(whoAmI, HEX));
        return fail();
    }

    bool ok =
        device.writeRegister(REG_GYRO_CONFIG0, (FS_2000DPS << 4) | ODR_1600HZ) &&
        device.writeRegister(REG_ACCEL_CONFIG0, (FS_16G << 4) | ODR_1600HZ) &&
        device.writeRegister(REG_PWR_MGMT0, PWR_GYRO_LN | PWR_ACCEL_LN);
    delay(1);

    ok = ok &&
         setIndirectBits(IPREG_GYRO_LPF, LPF_MASK, LPF_ODR_DIV_32) &&
         setIndirectBits(IPREG_ACCEL_LPF, LPF_MASK, LPF_ODR_DIV_32);

    if (!ok)
    {
        Serial.println("ICM45686: ошибка записи регистров");
        return fail();
    }

    delay(GYRO_STARTUP_MS);  // гироскоп выдаёт данные через ~45 мс после включения

    Serial.println("ICM45686: подключён");
    setAvailable(true);
    return true;
}

auto ICM45686_Sensor::readSample(RawImuSample& s) -> bool
{
    uint8_t b[14];  // accel XYZ, gyro XYZ, температура — little-endian
    if (!device.readRegisters(REG_ACCEL_DATA_X1_UI, b, sizeof(b))) return false;

    s.accelX      = le16(b + 0);
    s.accelY      = le16(b + 2);
    s.accelZ      = le16(b + 4);
    s.gyroX       = le16(b + 6);
    s.gyroY       = le16(b + 8);
    s.gyroZ       = le16(b + 10);
    s.temperature = le16(b + 12);
    return true;
}

auto ICM45686_Sensor::temperatureC(int16_t raw) const -> float
{
    return raw / 132.48f + 25.0f;  // TDK/Zephyr: 132.48 LSB/°C, 0 = 25 °C
}

auto ICM45686_Sensor::fail() -> bool
{
    setAvailable(false);
    return false;
}

auto ICM45686_Sensor::setIndirectBits(uint16_t address, uint8_t mask, uint8_t value) -> bool
{
    const uint8_t addr[2] = { static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address & 0xFF) };
    if (!device.writeRegisters(REG_IREG_ADDR_15_8, addr, sizeof(addr))) return false;
    delayMicroseconds(4);

    const int current = device.readRegister(REG_IREG_DATA);
    if (current < 0) return false;

    const uint8_t packet[3] = {
        addr[0], addr[1],
        static_cast<uint8_t>((current & ~mask) | (value & mask))
    };
    delayMicroseconds(4);
    const bool ok = device.writeRegisters(REG_IREG_ADDR_15_8, packet, sizeof(packet));
    delay(1);  // запись во внутренний регистр вступает в силу не сразу
    return ok;
}

auto ICM45686_Sensor::le16(const uint8_t* p) -> int16_t
{
    return static_cast<int16_t>((p[1] << 8) | p[0]);
}
