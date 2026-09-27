// Реализация hal/esp32/Esp32ServoOutput.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/esp32/Esp32ServoOutput.h"


Esp32ServoOutput::Esp32ServoOutput(int8_t servoPin, uint8_t ledcChannel)
: pin(servoPin),
      channel(ledcChannel)
{
}

auto Esp32ServoOutput::attach(uint16_t minUs, uint16_t maxUs) -> bool
{
    rangeMinUs = minUs;
    rangeMaxUs = maxUs;

    if (pin < 0)
    {
        attached = false;
        return false;
    }

    // ledcSetup() возвращает реально выставленную частоту, 0 — ошибка.
    attached = ledcSetup(channel, FREQUENCY_HZ, RESOLUTION_BITS) != 0;
    if (attached)
    {
        ledcAttachPin(pin, channel);
    }
    return attached;
}

auto Esp32ServoOutput::writeMicroseconds(uint16_t us) -> void
{
    if (!attached) return;

    const uint32_t clamped = constrain(us, rangeMinUs, rangeMaxUs);
    ledcWrite(channel, clamped * MAX_DUTY / PERIOD_US);
}

auto Esp32ServoOutput::measurePulseUs() -> int32_t
{
    if (!attached) return -1;

    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[pin]);
    const unsigned long width = pulseIn(pin, HIGH, 30000);
    return width ? static_cast<int32_t>(width) : -1;
}
