// Гибкие шины и несколько датчиков одного типа:
//   • ThrottleRamp — плавный спад газа;
//   • I2cRegisterDevice — поиск чипа на всех шинах и перебор мест;
//   • SensorChain — первый ответивший из выбранных;
//   • IBoard::i2cBuses() — порядок шин и шина за IMU.

#include <Arduino.h>
#include <unity.h>

#include "control/ControlMixer.h"
#include "control/GyroStickCalibration.h"
#include "control/ThrottleRamp.h"
#include "hal/RegisterDevice.h"
#include "helpers/TestSupport.h"
#include "sensors/SensorChain.h"

void setUp() { resetWorld(); }
void tearDown() {}

// ------------------------------------------------------------
// ThrottleRamp
// ------------------------------------------------------------

void test_ramp_adds_throttle_at_once_and_cuts_it_slowly()
{
    ThrottleRamp ramp;
    TEST_ASSERT_EQUAL_UINT16(1800, ramp.apply(1800, 0, 1000));   // прибавка — сразу
    TEST_ASSERT_EQUAL_UINT16(1700, ramp.apply(1000, 100, 1000));  // 100 мс = 10 % хода
    TEST_ASSERT_EQUAL_UINT16(1200, ramp.apply(1000, 600, 1000));
    TEST_ASSERT_EQUAL_UINT16(1000, ramp.apply(1000, 900, 1000));  // не ниже цели
    TEST_ASSERT_EQUAL_UINT16(1500, ramp.apply(1500, 910, 1000));
}

void test_ramp_zero_means_no_ramp_and_reset_stops_at_once()
{
    ThrottleRamp ramp;
    ramp.apply(2000, 0, 0);
    TEST_ASSERT_EQUAL_UINT16(1000, ramp.apply(1000, 2, 0));
    ramp.apply(2000, 4, 1000);
    ramp.reset();   // DISARM / MOTOR_KILL
    TEST_ASSERT_EQUAL_UINT16(1000, ramp.currentUs());
    TEST_ASSERT_EQUAL_UINT16(1000, ramp.apply(1000, 6, 1000));
}

// ------------------------------------------------------------
// I2cRegisterDevice на нескольких шинах
// ------------------------------------------------------------

void test_device_finds_its_chip_on_any_bus_and_alternate_address()
{
    FakeI2cBus a;
    FakeI2cBus b;
    a.devices = { 0x10 };
    b.devices = { 0x6B };
    I2cBusList list;
    list.add(&a);
    list.add(&b);
    list.add(&a);   // повтор не попадает
    TEST_ASSERT_EQUAL(2, list.count);

    I2cRegisterDevice dev(list, 0x6A, 0x6B);
    dev.begin();
    TEST_ASSERT_EQUAL_PTR(&b, dev.getBus());
    TEST_ASSERT_EQUAL_HEX8(0x6B, dev.getAddress());
    char where[32];
    dev.describe(where, sizeof(where));
    TEST_ASSERT_EQUAL_STRING("I2C I2C 0x6B", where);
    TEST_ASSERT_FALSE(dev.nextPlacement());
}

void test_device_steps_to_the_next_bus_with_the_same_address()
{
    FakeI2cBus a;
    FakeI2cBus b;
    a.devices = { 0x68 };   // чужой чип с тем же адресом
    b.devices = { 0x68 };
    I2cBusList list;
    list.add(&a);
    list.add(&b);
    I2cRegisterDevice dev(list, 0x68);
    dev.begin();
    TEST_ASSERT_EQUAL_PTR(&a, dev.getBus());
    TEST_ASSERT_TRUE(dev.nextPlacement());
    TEST_ASSERT_EQUAL_PTR(&b, dev.getBus());
    dev.begin();   // драйвер зовёт begin() снова — место не сбрасывается
    TEST_ASSERT_EQUAL_PTR(&b, dev.getBus());
    TEST_ASSERT_FALSE(dev.nextPlacement());
}

void test_device_with_nobody_answering_stays_on_the_first_place()
{
    FakeI2cBus a;
    FakeI2cBus b;
    a.devices = { 0x01 };
    b.devices = { 0x02 };
    I2cBusList list;
    list.add(&a);
    list.add(&b);
    I2cRegisterDevice dev(list, 0x76, 0x77);
    dev.begin();
    TEST_ASSERT_EQUAL_PTR(&a, dev.getBus());
    TEST_ASSERT_EQUAL_HEX8(0x76, dev.getAddress());
    TEST_ASSERT_FALSE(dev.probe());
    b.devices = { 0x77 };   // модуль подключили — следующий begin() найдёт
    dev.begin();
    TEST_ASSERT_EQUAL_PTR(&b, dev.getBus());
    TEST_ASSERT_EQUAL_HEX8(0x77, dev.getAddress());
}

// ------------------------------------------------------------
// Цепочка датчиков
// ------------------------------------------------------------

namespace
{
    // IMU, который опознаётся, только если устройство стоит на нужной шине.
    class PickyImu : public ImuSensor
    {
    public:
        PickyImu(const char* chip, I2cRegisterDevice& registers, II2CBus* realBus)
            : name(chip), dev(registers), bus(realBus)
        {
        }

        bool begin() override
        {
            ++begins;
            dev.begin();
            ok = dev.getBus() == bus && dev.probe();
            return ok;
        }
        bool isAvailable() const override { return ok; }
        void update() override { ++updates; }
        const char* getSensorType() const override { return name; }
        void printStatus() const override {}
        const ImuData& getImuData() const override { return data; }
        void calibrate() override { ++calibrations; }
        void setYaw(float) override {}

        const char* name;
        I2cRegisterDevice& dev;
        II2CBus* bus;
        bool ok = false;
        int begins = 0;
        int updates = 0;
        int calibrations = 0;
        ImuData data{};
    };
}

void test_chain_takes_the_first_chip_that_answers_in_order()
{
    FakeI2cBus a;
    FakeI2cBus b;
    a.devices = { 0x68 };   // на шине A — чужой 0x68
    b.devices = { 0x68, 0x6A };
    I2cBusList list;
    list.add(&a);
    list.add(&b);
    I2cRegisterDevice mainDev(list, 0x6A, 0x6B);   // основной: LSM6DSV — нигде нет 0x6B, есть 0x6A на B
    I2cRegisterDevice spareDev(list, 0x68);
    PickyImu mainChip("LSM6DSV", mainDev, &a);     // опознаётся только на A — не выйдет
    PickyImu spare("MPU6050", spareDev, &b);       // на A ответит чужой, на B — свой

    ImuChain chain(SensorChainCore::Entry(&mainChip, &mainDev), SensorChainCore::Entry(&spare, &spareDev));
    TEST_ASSERT_TRUE(chain.begin());
    TEST_ASSERT_EQUAL_INT8(1, chain.chain().activeIndex());
    TEST_ASSERT_EQUAL_STRING("MPU6050", chain.getSensorType());
    TEST_ASSERT_EQUAL(2, spare.begins);   // A (чужой) — не опознан, B — опознан
    TEST_ASSERT_TRUE(contains(takeSerial(), "IMU: MPU6050 — I2C I2C 0x68 (запасной: основной не ответил)"));

    chain.update();
    chain.calibrate();
    TEST_ASSERT_EQUAL(1, spare.updates);
    TEST_ASSERT_EQUAL(1, spare.calibrations);
    TEST_ASSERT_EQUAL(0, mainChip.updates);
}

void test_chain_with_nothing_found_or_nothing_chosen()
{
    FakeI2cBus a;
    a.devices = { 0x01 };
    I2cRegisterDevice dev(a, 0x7C);
    PickyImu chip("QMC", dev, &a);
    ImuChain chain(SensorChainCore::Entry(&chip, &dev));
    TEST_ASSERT_FALSE(chain.begin());
    TEST_ASSERT_FALSE(chain.isAvailable());
    TEST_ASSERT_EQUAL_STRING("QMC", chain.getSensorType());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, chain.getImuData().roll);
    TEST_ASSERT_TRUE(contains(takeSerial(), "не найден ни один из выбранных (QMC)"));

    MagChain none{ SensorChainCore::Entry() };
    TEST_ASSERT_EQUAL(0, none.chain().configuredCount());
    TEST_ASSERT_FALSE(none.begin());
    TEST_ASSERT_EQUAL_STRING("нет", none.getSensorType());
}

// ------------------------------------------------------------
// Шины платы
// ------------------------------------------------------------

void test_board_lists_all_buses_preferred_first_with_the_one_behind_the_imu()
{
    FakeBoard board;
    FakeI2cBus behindImu;
    TEST_ASSERT_NULL(board.i2cBus(3));
    board.setAuxI2c(&behindImu);
    TEST_ASSERT_EQUAL_PTR(&behindImu, board.i2cBus(3));
    TEST_ASSERT_NULL(board.i2cBus(2));   // внешний разъём — та же шина датчиков

    const I2cBusList list = board.i2cBuses(board.screenBus);
    TEST_ASSERT_EQUAL(3, list.count);
    TEST_ASSERT_EQUAL_PTR(&board.screenBus, list.bus[0]);
    TEST_ASSERT_EQUAL_PTR(&board.sensorBus, list.bus[1]);
    TEST_ASSERT_EQUAL_PTR(&behindImu, list.bus[2]);
}

// ------------------------------------------------------------
// Калибровка гироскопа стиками
// ------------------------------------------------------------

void test_gyro_stick_command_fires_once_after_the_hold()
{
    ControlMixer mixer;
    GyroStickCalibration cmd;
    RcChannelState rc;
    rc.set(Channels::THROTTLE, 1000);
    rc.set(Channels::AILERON, 1500);
    // рысканье влево и тангаж «на себя» — в знаках команды: нос влево, нос вверх
    ControlCommand sticks;
    sticks.yaw = static_cast<int16_t>(-Config::RUDDER_MAX_US);
    sticks.pitch = static_cast<int16_t>(Config::ELEVATOR_MAX_US);
    TEST_ASSERT_FALSE(cmd.update(rc, sticks, 0));
    TEST_ASSERT_FALSE(cmd.update(rc, sticks, Config::STICK_GYRO_CAL_HOLD_MS - 1));
    TEST_ASSERT_TRUE(cmd.update(rc, sticks, Config::STICK_GYRO_CAL_HOLD_MS));
    TEST_ASSERT_FALSE(cmd.update(rc, sticks, Config::STICK_GYRO_CAL_HOLD_MS + 5000));   // держат дальше — повтора нет

    ControlCommand released;
    TEST_ASSERT_FALSE(cmd.update(rc, released, 7000));
    TEST_ASSERT_FALSE(cmd.update(rc, sticks, 8000));
    TEST_ASSERT_TRUE(cmd.update(rc, sticks, 8000 + Config::STICK_GYRO_CAL_HOLD_MS));

    // газ не внизу или крен не в центре — не считается
    GyroStickCalibration other;
    rc.set(Channels::THROTTLE, 1500);
    other.update(rc, sticks, 0);
    TEST_ASSERT_FALSE(other.update(rc, sticks, 10000));
    rc.set(Channels::THROTTLE, 1000);
    ControlCommand rolled = sticks;
    rolled.roll = static_cast<int16_t>(Config::AILERON_MAX_US);
    other.update(rc, rolled, 20000);
    TEST_ASSERT_FALSE(other.update(rc, rolled, 30000));
    (void)mixer;
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_ramp_adds_throttle_at_once_and_cuts_it_slowly);
    RUN_TEST(test_ramp_zero_means_no_ramp_and_reset_stops_at_once);
    RUN_TEST(test_device_finds_its_chip_on_any_bus_and_alternate_address);
    RUN_TEST(test_device_steps_to_the_next_bus_with_the_same_address);
    RUN_TEST(test_device_with_nobody_answering_stays_on_the_first_place);
    RUN_TEST(test_chain_takes_the_first_chip_that_answers_in_order);
    RUN_TEST(test_chain_with_nothing_found_or_nothing_chosen);
    RUN_TEST(test_board_lists_all_buses_preferred_first_with_the_one_behind_the_imu);
    RUN_TEST(test_gyro_stick_command_fires_once_after_the_hold);
    return UNITY_END();
}
