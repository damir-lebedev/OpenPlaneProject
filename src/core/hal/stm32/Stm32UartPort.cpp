// Реализация hal/stm32/Stm32UartPort.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/Stm32UartPort.h"


Stm32UartPort::Stm32UartPort(HardwareSerial& port)
: serial(port)
{
}

auto Stm32UartPort::begin(uint32_t baud) -> void
{
    serial.begin(baud, SERIAL_8N1);
}
