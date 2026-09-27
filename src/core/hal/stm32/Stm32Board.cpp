// Реализация hal/stm32/Stm32Board.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/Stm32Board.h"


Stm32Board::Stm32Board()
: displayWire(pinOf(Config::PIN_I2C2_SDA), pinOf(Config::PIN_I2C2_SCL)),
      i2cBus(Wire, pinOf(Config::PIN_I2C_SDA), pinOf(Config::PIN_I2C_SCL)),
      displayBus(displayWire, pinOf(Config::PIN_I2C2_SDA), pinOf(Config::PIN_I2C2_SCL)),
      spiBus(SPI, pinOf(Config::PIN_SENSOR_SPI_SCK), pinOf(Config::PIN_SENSOR_SPI_MISO), pinOf(Config::PIN_SENSOR_SPI_MOSI)),
      rcSerial(pinOf(Config::PIN_IBUS), pinOf(Config::PIN_IBUS_TX)),
      gpsSerial(pinOf(Config::PIN_GPS_RX), pinOf(Config::PIN_GPS_TX)),
      telemetrySerial(pinOf(Config::PIN_TELEM_RX), pinOf(Config::PIN_TELEM_TX)),
      rcPort(rcSerial),
      gpsPort(gpsSerial),
      telemetryPort(telemetrySerial),
      servos{
          Stm32ServoOutput(Config::PIN_AILERON_LEFT),
          Stm32ServoOutput(Config::PIN_AILERON_RIGHT),
          Stm32ServoOutput(Config::PIN_ELEVATOR),
          Stm32ServoOutput(Config::PIN_ESC),
          Stm32ServoOutput(Config::PIN_RUDDER),
          Stm32ServoOutput(Config::PIN_AUX1),
          Stm32ServoOutput(Config::PIN_AUX2)
      }
{
}

auto Stm32Board::begin() -> void
{
    i2cBus.begin();
    spiBus.begin();
    displayBus.begin();

    pinMode(pinOf(Config::PIN_BUZZER), OUTPUT);
    digitalWrite(pinOf(Config::PIN_BUZZER), LOW);
}

auto Stm32Board::setBuzzer(bool on) -> void
{
    digitalWrite(pinOf(Config::PIN_BUZZER), on ? HIGH : LOW);
}
