// ============================================================
// AEROS-001 — ТОЧКА ВХОДА ДЛЯ STM32H743 (ЗАГОТОВКА, bring-up)
//
// Собирается env stm32h743; на железе не проверялась. Основная
// прошивка — src/main.cpp под ESP32-S3.
//
// Полная прошивка на STM32 пока не собирается, и дело не в HAL
// (hal/stm32/ реализует весь IBoard), а в трёх ESP32-зависимостях
// уровнем выше:
//   - калибровки датчиков и настройки лога хранятся в NVS
//     (Preferences) — на STM32 нужна замена поверх EEPROM-эмуляции
//     во флеше;
//   - дашборд — Wi-Fi-точка доступа (WebDebugServer) — на STM32
//     Wi-Fi нет, телеметрия пойдёт по UART (радиомодем);
//   - веб и OLED крутятся задачами FreeRTOS на втором ядре.
//
// Поэтому здесь собрано то, что от MCU уже не зависит: ручной полёт
// (iBUS -> FlightController: ARM, закрылки, газ, failsafe ->
// 5 PWM-выходов) без автопилота, плюс проверка разводки шин при
// первом включении платы.
//
// Консоль (Serial — LPUART1, PA9/PA10, 115200):
//   s — состояние (приёмник, ARM, выходы)
//   p — самопроверка выходов: импульс на каждом пине
//   b — опрос шин: I2C-датчики, I2C экрана, SPI (ICM42688, BMP388)
// ============================================================

#include <Arduino.h>

#include "config/Config.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "hal/RegisterDevice.h"
#include "hal/stm32/Stm32Board.h"
#include "rc/IBusReceiver.h"


Stm32Board board;

IBusReceiver ibusReceiver(board.rcUart());
ControlMixer controlMixer;
ThrottleManager throttleManager;
FlightOutputs flightOutputs(board);
ArmingManager armingManager;

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs
);


// Устройства на SPI — те же параметры, что у драйверов
// (ICM42688_Sensor::spiDevice, BMP388_Sensor::spiDevice). Сами
// драйверы сюда не подключены: они хранят калибровку в Preferences.
SpiRegisterDevice imuSpi(board.spi(), static_cast<uint8_t>(Config::PIN_SPI_CS_ICM42688), 8000000, 0);
SpiRegisterDevice baroSpi(board.spi(), static_cast<uint8_t>(Config::PIN_SPI_CS_BMP388), 8000000, 1);


static void printBusAddresses(const char* name, II2CBus& bus)
{
    Serial.print(name);
    Serial.print(':');

    uint8_t found = 0;
    for (uint8_t address = 0x08; address < 0x78; ++address)
    {
        if (bus.probe(address))
        {
            Serial.print(" 0x");
            Serial.print(address, HEX);
            ++found;
        }
    }

    Serial.println(found ? "" : " никого");
}

static void printSpiId(const char* name, IRegisterDevice& device, uint8_t reg, int expected)
{
    const int id = device.readRegister(reg);

    Serial.print(name);
    if (id < 0)
    {
        Serial.println(": нет ответа");
        return;
    }
    // 0x00/0xFF — обычно чипа нет вовсе (MISO висит в воздухе).
    Serial.print(": id=0x");
    Serial.print(id, HEX);
    Serial.println(id == expected ? " OK" : " — не тот чип или нет ответа");
}

static void printBuses()
{
    printBusAddresses("I2C датчиков", board.i2c());
    printBusAddresses("I2C экрана", *board.displayI2c());
    printSpiId("SPI ICM42688 (WHO_AM_I, ждём 0x47)", imuSpi, 0x75, 0x47);
    printSpiId("SPI BMP388 (CHIP_ID, ждём 0x50)", baroSpi, 0x00, 0x50);
}

static void printStatus()
{
    const IBusReceiver& rx = flightController.getReceiver();

    Serial.print("RX: ");
    Serial.print(flightController.isReceiverFailsafe() ? "НЕТ СВЯЗИ" : "ok");
    Serial.print(", кадров ");
    Serial.print(rx.getGoodFrameCount());
    Serial.print(" / битых ");
    Serial.print(rx.getBadFrameCount());
    Serial.print(". ");
    Serial.println(flightController.isArmed() ? "ARMED" : "disarmed");

    const FlightOutputState& out = flightController.getOutputState();
    Serial.print("Выходы, мкс: элероны ");
    Serial.print(out.aileronLeft);
    Serial.print('/');
    Serial.print(out.aileronRight);
    Serial.print(", руль выс ");
    Serial.print(out.elevator);
    Serial.print(", газ ");
    Serial.print(out.throttle);
    Serial.print(", руль нап ");
    Serial.println(out.rudder);
}

static void handleConsole()
{
    while (Serial.available() > 0)
    {
        switch (Serial.read())
        {
            case 's': printStatus(); break;
            case 'p': flightOutputs.printPulseSelfTest(); break;
            case 'b': printBuses(); break;
            default: break;
        }
    }
}


// Вызываются ядром Arduino, а не из кода проекта.
// cppcheck-suppress unusedFunction
void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" STM32H743 / BRING-UP (заготовка)");
    Serial.println("=================================");

    board.begin();               // шины I2C/SPI
    flightOutputs.begin();       // PWM-выходы, сразу в безопасное положение
    flightOutputs.setFailsafe();

    imuSpi.begin();              // CS в неактивное состояние
    baroSpi.begin();
    printBuses();

    flightController.begin();    // приёмник iBUS

    Serial.println("Ручной полёт без автопилота. Консоль: s — состояние, p — выходы, b — шины.");
}


// cppcheck-suppress unusedFunction
void loop()
{
    static uint32_t lastTickMs = millis();

    flightController.update();
    handleConsole();

    // Фиксированный период цикла без FreeRTOS: ждём до следующего
    // такта. После долгой блокировки (вывод в консоль) не догоняем
    // пропущенные такты пачкой, а начинаем отсчёт заново.
    const uint32_t now = millis();
    if (now - lastTickMs > 100)
    {
        lastTickMs = now;
    }
    while (millis() - lastTickMs < Config::LOOP_PERIOD_MS)
    {
    }
    lastTickMs += Config::LOOP_PERIOD_MS;
}
