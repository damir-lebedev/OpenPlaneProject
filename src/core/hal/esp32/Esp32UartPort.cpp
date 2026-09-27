// Реализация hal/esp32/Esp32UartPort.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/esp32/Esp32UartPort.h"


Esp32UartPort::Esp32UartPort(HardwareSerial& port, int8_t rxPin, int8_t txPin)
: serial(port), rx(rxPin), tx(txPin)
{
}

auto Esp32UartPort::begin(uint32_t baud) -> void
{
    serial.begin(baud, SERIAL_8N1, rx, tx);
}
