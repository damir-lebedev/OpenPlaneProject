// Реализация hal/esp32/Esp32Board.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/esp32/Esp32Board.h"


Esp32Board::Esp32Board()
: i2cBus(Wire, Config::PIN_I2C_SDA, Config::PIN_I2C_SCL),
#if SOC_I2C_NUM > 1
      displayBus(Wire1, Config::PIN_I2C2_SDA, Config::PIN_I2C2_SCL),
#endif
      spiBus(Config::PIN_SENSOR_SPI_SCK, Config::PIN_SENSOR_SPI_MISO, Config::PIN_SENSOR_SPI_MOSI),
      rcSerial(1),
      gpsSerial(Config::UART_NUM_GPS),
      rcPort(rcSerial, Config::PIN_IBUS, -1),
      gpsPort(gpsSerial, Config::PIN_GPS_RX, Config::PIN_GPS_TX),
      servos{
          // Второй аргумент — канал LEDC, у каждого выхода свой.
          Esp32ServoOutput(Config::PIN_AILERON_LEFT, 0),
          Esp32ServoOutput(Config::PIN_AILERON_RIGHT, 1),
          Esp32ServoOutput(Config::PIN_ELEVATOR, 2),
          Esp32ServoOutput(Config::PIN_ESC, 3),
          Esp32ServoOutput(Config::PIN_RUDDER, 4),
          Esp32ServoOutput(Config::PIN_AUX1, 5),
          Esp32ServoOutput(Config::PIN_AUX2, 6)
      }
{
}

auto Esp32Board::begin() -> void
{
    i2cBus.begin();
    spiBus.begin();

    if (Config::PIN_BUZZER >= 0)
    {
        pinMode(Config::PIN_BUZZER, OUTPUT);
        digitalWrite(Config::PIN_BUZZER, LOW);
    }

#if SOC_I2C_NUM > 1
    if (hasDisplayBus())
    {
        displayBus.begin();
    }
#endif
}

auto Esp32Board::displayI2c() -> II2CBus*
{
#if SOC_I2C_NUM > 1
    return hasDisplayBus() ? &displayBus : nullptr;
#else
    return nullptr;
#endif
}

auto Esp32Board::setBuzzer(bool on) -> void
{
    if (Config::PIN_BUZZER >= 0) digitalWrite(Config::PIN_BUZZER, on ? HIGH : LOW);
}
