// ============================================================
// HAL: помощники II2CBus, регистровые устройства I2C/SPI и
// реализация под ESP32 (Wire/SPI/HardwareSerial/LEDC — фейки из
// test/native/support).
//
// Запуск: pio test -e native -f native/test_hal
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "hal/IServoOutput.h"
#include "hal/RegisterDevice.h"
#include "hal/esp32/Esp32Board.h"
#include "hal/esp32/Esp32I2CBus.h"
#include "hal/esp32/Esp32ServoOutput.h"
#include "hal/esp32/Esp32SpiBus.h"
#include "hal/esp32/Esp32UartPort.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

// ------------------------------------------------------------
// II2CBus и Esp32I2CBus
// ------------------------------------------------------------

void test_i2c_bus_begin_applies_pins_frequency_and_short_timeout()
{
    TwoWire wire(3);
    Esp32I2CBus bus(wire, 21, 22, 100000);
    bus.begin();

    TEST_ASSERT_TRUE(wire.isStarted());
    TEST_ASSERT_EQUAL(21, wire.sdaPin());
    TEST_ASSERT_EQUAL(22, wire.sclPin());
    TEST_ASSERT_EQUAL_UINT32(100000, wire.clockHz());
    TEST_ASSERT_EQUAL_UINT16(5, wire.getTimeOut());   // не штатные 50 мс

    bus.setClock(400000);
    TEST_ASSERT_EQUAL_UINT32(400000, wire.clockHz());
}

void test_i2c_register_helpers_read_write_and_probe()
{
    I2cRig rig(0x40);
    rig.chip.regs[0x10] = 0xAB;
    rig.chip.regs[0x11] = 0xCD;

    TEST_ASSERT_TRUE(rig.bus.probe(0x40));
    TEST_ASSERT_FALSE(rig.bus.probe(0x41));

    TEST_ASSERT_TRUE(rig.bus.writeRegister(0x40, 0x20, 0x5A));
    TEST_ASSERT_EQUAL_HEX8(0x5A, rig.chip.regs[0x20]);
    TEST_ASSERT_FALSE(rig.bus.writeRegister(0x41, 0x20, 0x00));   // нет устройства

    uint8_t buffer[2] = {};
    TEST_ASSERT_TRUE(rig.bus.readRegisters(0x40, 0x10, buffer, 2));
    TEST_ASSERT_EQUAL_HEX8(0xAB, buffer[0]);
    TEST_ASSERT_EQUAL_HEX8(0xCD, buffer[1]);

    TEST_ASSERT_EQUAL(0xAB, rig.bus.readRegister(0x40, 0x10));
    TEST_ASSERT_EQUAL(-1, rig.bus.readRegister(0x41, 0x10));
}

void test_i2c_read_failure_leaves_buffer_untouched()
{
    I2cRig rig(0x40);
    uint8_t buffer[4] = { 1, 2, 3, 4 };

    // NACK на адрес регистра.
    TEST_ASSERT_FALSE(rig.bus.readRegisters(0x41, 0x00, buffer, 4));

    // Устройство отдало меньше байт, чем просили.
    rig.chip.shortRead = 2;
    TEST_ASSERT_FALSE(rig.bus.readRegisters(0x40, 0x00, buffer, 4));

    // NACK на чтение.
    rig.chip.shortRead = 0;
    rig.chip.failReads = 1;
    TEST_ASSERT_FALSE(rig.bus.readRegisters(0x40, 0x00, buffer, 4));

    const uint8_t expected[4] = { 1, 2, 3, 4 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buffer, 4);
}

void test_i2c_raw_primitives_delegate_to_wire()
{
    I2cRig rig(0x50);
    rig.chip.regs[0] = 0x11;

    rig.bus.beginTransmission(0x50);
    const uint8_t payload[] = { 0x05, 0x77, 0x88 };
    TEST_ASSERT_EQUAL(3, rig.bus.write(payload, sizeof(payload)));
    TEST_ASSERT_EQUAL(0, rig.bus.endTransmission());
    TEST_ASSERT_EQUAL_HEX8(0x77, rig.chip.regs[5]);
    TEST_ASSERT_EQUAL_HEX8(0x88, rig.chip.regs[6]);

    rig.bus.beginTransmission(0x50);
    rig.bus.write(static_cast<uint8_t>(0x00));
    TEST_ASSERT_EQUAL(0, rig.bus.endTransmission(false));
    TEST_ASSERT_EQUAL(1, rig.bus.requestFrom(0x50, 1));
    TEST_ASSERT_EQUAL(1, rig.bus.available());
    TEST_ASSERT_EQUAL(0x11, rig.bus.read());
    TEST_ASSERT_EQUAL(0, rig.bus.available());
}

void test_i2c_register_device_delegates_with_its_address()
{
    I2cRig rig(0x76);
    rig.chip.regs[0xD0] = 0x60;

    rig.device.begin();   // по умолчанию ничего не делает
    TEST_ASSERT_EQUAL_HEX8(0x76, rig.device.getAddress());
    TEST_ASSERT_TRUE(rig.device.probe());
    TEST_ASSERT_EQUAL(0x60, rig.device.readRegister(0xD0));
    TEST_ASSERT_TRUE(rig.device.writeRegister(0xF4, 0x53));
    TEST_ASSERT_EQUAL(0x53, rig.chip.lastWrite(0xF4));

    rig.chip.present = false;
    TEST_ASSERT_FALSE(rig.device.probe());
    TEST_ASSERT_EQUAL(-1, rig.device.readRegister(0xD0));
}

// ------------------------------------------------------------
// SPI
// ------------------------------------------------------------

void test_spi_bus_begin_and_transaction_settings()
{
    Esp32SpiBus bus(12, 13, 11);
    bus.begin();
    TEST_ASSERT_TRUE(SPI.isStarted());
    TEST_ASSERT_EQUAL(12, SPI.sckPin());
    TEST_ASSERT_EQUAL(13, SPI.misoPin());
    TEST_ASSERT_EQUAL(11, SPI.mosiPin());
    TEST_ASSERT_EQUAL(-1, SPI.ssPin());   // CS держат сами устройства

    const uint8_t expectedModes[] = { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3, SPI_MODE0 };
    for (uint8_t mode = 0; mode < 5; ++mode)
    {
        bus.beginTransaction(1000000u * (mode + 1), mode);   // 4 — неизвестный -> MODE0
        TEST_ASSERT_TRUE(SPI.isInTransaction());
        TEST_ASSERT_EQUAL_UINT32(1000000u * (mode + 1), SPI.settings().clock);
        TEST_ASSERT_EQUAL(MSBFIRST, SPI.settings().bitOrder);
        TEST_ASSERT_EQUAL(expectedModes[mode], SPI.settings().dataMode);
        bus.endTransaction();
        TEST_ASSERT_FALSE(SPI.isInTransaction());
    }
    TEST_ASSERT_EQUAL_HEX8(0xFF, bus.transfer(0x00));   // никто не выбран
}

void test_spi_register_device_protocol_with_dummy_byte()
{
    SpiRig rig(21, 1);
    rig.chip.regs[0x00] = 0x50;
    rig.chip.regs[0x04] = 0x11;
    rig.chip.regs[0x05] = 0x22;

    SpiRegisterDevice device(rig.bus, 21, 8000000, 1, 3);
    device.begin();
    TEST_ASSERT_EQUAL(OUTPUT, fake::gpio().mode[21]);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(21));
    TEST_ASSERT_TRUE(device.probe());   // у SPI нет ACK

    TEST_ASSERT_EQUAL(0x50, device.readRegister(0x00));
    uint8_t data[2] = {};
    TEST_ASSERT_TRUE(device.readRegisters(0x04, data, 2));
    TEST_ASSERT_EQUAL_HEX8(0x11, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x22, data[1]);
    TEST_ASSERT_EQUAL_UINT32(8000000, SPI.settings().clock);
    TEST_ASSERT_EQUAL(SPI_MODE3, SPI.settings().dataMode);

    // Запись — адрес со сброшенным битом 7, даже если его передали.
    TEST_ASSERT_TRUE(device.writeRegister(0x9B, 0x33));
    TEST_ASSERT_EQUAL(0x33, rig.chip.lastWrite(0x1B));
    TEST_ASSERT_EQUAL(HIGH, digitalRead(21));   // CS отпущен после обмена
}

void test_spi_without_dummy_byte_reads_shifted_data_from_dummy_chip()
{
    // Драйвер, не знающий про фиктивный байт BMP388, прочитал бы всё
    // со сдвигом — ровно та ошибка, которую исправляет dummyReadBytes.
    SpiRig rig(14, 1);
    rig.chip.regs[0x00] = 0x50;
    SpiRegisterDevice naive(rig.bus, 14);
    naive.begin();
    TEST_ASSERT_NOT_EQUAL(0x50, naive.readRegister(0x00));
}

// ------------------------------------------------------------
// UART
// ------------------------------------------------------------

void test_uart_port_opens_with_fixed_pins_and_8n1()
{
    HardwareSerial serial(9);   // номер вне реестра: не мешает глобальным портам
    Esp32UartPort port(serial, 17, -1);
    port.begin(115200);

    TEST_ASSERT_TRUE(serial.isStarted());
    TEST_ASSERT_EQUAL_UINT32(115200, serial.baud());
    TEST_ASSERT_EQUAL_UINT32(SERIAL_8N1, serial.config());
    TEST_ASSERT_EQUAL(17, serial.rxPin());
    TEST_ASSERT_EQUAL(-1, serial.txPin());

    serial.pushRx(std::string("\x01\x02", 2));
    TEST_ASSERT_EQUAL(2, port.available());
    TEST_ASSERT_EQUAL(1, port.read());
    TEST_ASSERT_EQUAL(2, port.read());
    TEST_ASSERT_EQUAL(-1, port.read());

    TEST_ASSERT_EQUAL(1, port.write(static_cast<uint8_t>(0xB5)));
    const uint8_t more[] = { 0x62, 0x06 };
    TEST_ASSERT_EQUAL(2, port.write(more, 2));
    TEST_ASSERT_EQUAL_STRING_LEN("\xB5\x62\x06", serial.txBytes().data(), 3);
}

// ------------------------------------------------------------
// PWM-выходы (LEDC)
// ------------------------------------------------------------

void test_servo_output_without_pin_is_disabled()
{
    Esp32ServoOutput servo(-1, 4);
    TEST_ASSERT_FALSE(servo.attach(1000, 2000));
    TEST_ASSERT_FALSE(servo.isAttached());
    servo.writeMicroseconds(1500);
    TEST_ASSERT_EQUAL_UINT32(0, fake::ledc().channel[4].writeCount);
    TEST_ASSERT_EQUAL(-1, servo.measurePulseUs());
}

void test_servo_output_configures_50hz_14bit_and_clamps_pulse()
{
    Esp32ServoOutput servo(6, 2);
    servo.writeMicroseconds(1500);                       // до attach — игнор
    TEST_ASSERT_EQUAL_UINT32(0, fake::ledc().channel[2].writeCount);

    TEST_ASSERT_TRUE(servo.attach(1000, 2000));
    TEST_ASSERT_TRUE(servo.isAttached());
    const fake::LedcChannel& ch = fake::ledc().channel[2];
    TEST_ASSERT_EQUAL_FLOAT(50.0f, static_cast<float>(ch.frequency));
    TEST_ASSERT_EQUAL(14, ch.resolutionBits);
    TEST_ASSERT_EQUAL(6, ch.pin);

    servo.writeMicroseconds(1500);
    TEST_ASSERT_EQUAL_UINT32(1500u * 16384u / 20000u, ch.duty);
    servo.writeMicroseconds(2500);                       // выше диапазона
    TEST_ASSERT_EQUAL_UINT32(2000u * 16384u / 20000u, ch.duty);
    servo.writeMicroseconds(500);                        // ниже диапазона
    TEST_ASSERT_EQUAL_UINT32(1000u * 16384u / 20000u, ch.duty);
}

void test_servo_output_attach_fails_when_ledc_refuses()
{
    fake::ledc().failSetup[3] = true;
    Esp32ServoOutput servo(7, 3);
    TEST_ASSERT_FALSE(servo.attach(1000, 2000));
    TEST_ASSERT_EQUAL(-1, fake::ledc().channel[3].pin);
}

void test_servo_measures_real_pulse_through_input_buffer()
{
    Esp32ServoOutput servo(5, 1);
    TEST_ASSERT_TRUE(servo.attach(1000, 2000));

    // Выход ещё ничего не выдаёт — импульса нет.
    TEST_ASSERT_EQUAL(-1, servo.measurePulseUs());
    TEST_ASSERT_TRUE(fake::gpio().inputEnabled[5]);   // вход включён, выход не тронут

    servo.writeMicroseconds(1234);
    const int32_t measured = servo.measurePulseUs();
    TEST_ASSERT_INT32_WITHIN(2, 1234, measured);   // шаг LEDC ~1.2 мкс
}

void test_servo_interface_default_measurement_is_unsupported()
{
    struct MinimalServo : IServoOutput
    {
        bool attach(uint16_t, uint16_t) override { return true; }
        void writeMicroseconds(uint16_t) override {}
        bool isAttached() const override { return true; }
    } servo;
    TEST_ASSERT_EQUAL(-1, servo.measurePulseUs());
}

// ------------------------------------------------------------
// Esp32Board
// ------------------------------------------------------------

void test_board_begin_brings_up_buses_on_configured_pins()
{
    Esp32Board board;
    board.begin();

    TEST_ASSERT_TRUE(Wire.isStarted());
    TEST_ASSERT_EQUAL(Config::PIN_I2C_SDA, Wire.sdaPin());
    TEST_ASSERT_EQUAL(Config::PIN_I2C_SCL, Wire.sclPin());
    TEST_ASSERT_EQUAL_UINT32(400000, Wire.clockHz());

    TEST_ASSERT_TRUE(Wire1.isStarted());   // у S3 вторая шина — под OLED
    TEST_ASSERT_EQUAL(Config::PIN_I2C2_SDA, Wire1.sdaPin());
    TEST_ASSERT_EQUAL(Config::PIN_I2C2_SCL, Wire1.sclPin());
    TEST_ASSERT_NOT_NULL(board.displayI2c());

    TEST_ASSERT_TRUE(SPI.isStarted());
    TEST_ASSERT_EQUAL(Config::PIN_SENSOR_SPI_SCK, SPI.sckPin());

    TEST_ASSERT_TRUE(&board.i2c() != board.displayI2c());
    board.spi().beginTransaction(1000000, 0);
    TEST_ASSERT_TRUE(SPI.isInTransaction());
    board.spi().endTransaction();
}

void test_board_uarts_map_to_hardware_serial_ports()
{
    Esp32Board board;
    board.rcUart().begin(Config::IBUS_BAUDRATE);
    board.gpsUart().begin(9600);

    HardwareSerial* rc = fake::uart(1);
    HardwareSerial* gps = fake::uart(Config::UART_NUM_GPS);
    TEST_ASSERT_NOT_NULL(rc);
    TEST_ASSERT_NOT_NULL(gps);
    TEST_ASSERT_EQUAL_UINT32(Config::IBUS_BAUDRATE, rc->baud());
    TEST_ASSERT_EQUAL(Config::PIN_IBUS, rc->rxPin());
    TEST_ASSERT_EQUAL(-1, rc->txPin());   // iBUS — только приём
    TEST_ASSERT_EQUAL_UINT32(9600, gps->baud());
    TEST_ASSERT_EQUAL(Config::PIN_GPS_RX, gps->rxPin());
    TEST_ASSERT_EQUAL(Config::PIN_GPS_TX, gps->txPin());
}

void test_board_servo_channels_follow_servo_channel_order()
{
    Esp32Board board;
    const int8_t pins[ServoChannel::COUNT] = {
        Config::PIN_AILERON_LEFT, Config::PIN_AILERON_RIGHT, Config::PIN_ELEVATOR,
        Config::PIN_ESC, Config::PIN_RUDDER,
    };
    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        TEST_ASSERT_TRUE(board.servo(ch).attach(1000, 2000));
        TEST_ASSERT_EQUAL(pins[ch], fake::ledc().channel[ch].pin);   // канал LEDC = индекс
    }
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_i2c_bus_begin_applies_pins_frequency_and_short_timeout);
    RUN_TEST(test_i2c_register_helpers_read_write_and_probe);
    RUN_TEST(test_i2c_read_failure_leaves_buffer_untouched);
    RUN_TEST(test_i2c_raw_primitives_delegate_to_wire);
    RUN_TEST(test_i2c_register_device_delegates_with_its_address);
    RUN_TEST(test_spi_bus_begin_and_transaction_settings);
    RUN_TEST(test_spi_register_device_protocol_with_dummy_byte);
    RUN_TEST(test_spi_without_dummy_byte_reads_shifted_data_from_dummy_chip);
    RUN_TEST(test_uart_port_opens_with_fixed_pins_and_8n1);
    RUN_TEST(test_servo_output_without_pin_is_disabled);
    RUN_TEST(test_servo_output_configures_50hz_14bit_and_clamps_pulse);
    RUN_TEST(test_servo_output_attach_fails_when_ledc_refuses);
    RUN_TEST(test_servo_measures_real_pulse_through_input_buffer);
    RUN_TEST(test_servo_interface_default_measurement_is_unsupported);
    RUN_TEST(test_board_begin_brings_up_buses_on_configured_pins);
    RUN_TEST(test_board_uarts_map_to_hardware_serial_ports);
    RUN_TEST(test_board_servo_channels_follow_servo_channel_order);
    return UNITY_END();
}
