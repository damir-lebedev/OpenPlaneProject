// Реализация hal/stm32/Stm32ServoOutput.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/Stm32ServoOutput.h"


Stm32ServoOutput::Stm32ServoOutput(int16_t servoPin)
: pin(servoPin)
{
}

auto Stm32ServoOutput::attach(uint16_t minUs, uint16_t maxUs) -> bool
{
    rangeMinUs = minUs;
    rangeMaxUs = maxUs;
    attached = false;

    if (pin < 0) return false;

    const PinName name = digitalPinToPinName(static_cast<pin_size_t>(pin));
    auto* instance = static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM));
    if (instance == nullptr) return false;   // на этом пине нет канала таймера

    timer = acquireTimer(instance);
    if (timer == nullptr) return false;      // пул таймеров исчерпан

    channel = STM_PIN_CHANNEL(pinmap_function(name, PinMap_TIM));

    // Сравнение = 0: до первой записи импульса на пине нет вовсе
    // (серво держит положение, ESC не видит сигнала), а не
    // случайная ширина. FlightOutputs сразу после attach() пишет
    // безопасные значения.
    timer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, name);
    timer->setCaptureCompare(channel, 0, MICROSEC_COMPARE_FORMAT);
    timer->resume();

    attached = true;
    return true;
}

auto Stm32ServoOutput::writeMicroseconds(uint16_t us) -> void
{
    if (!attached) return;

    const uint16_t clamped = constrain(us, rangeMinUs, rangeMaxUs);
    timer->setCaptureCompare(channel, clamped, MICROSEC_COMPARE_FORMAT);
}

auto Stm32ServoOutput::measurePulseUs() -> int32_t
{
    if (!attached) return -1;

    const unsigned long width = pulseIn(static_cast<pin_size_t>(pin), HIGH, 30000);
    return width ? static_cast<int32_t>(width) : -1;
}

auto Stm32ServoOutput::acquireTimer(TIM_TypeDef* instance) -> HardwareTimer*
{
    struct Slot
    {
        TIM_TypeDef* instance = nullptr;
        HardwareTimer timer;   // без аргументов: setup() — позже, не при статической инициализации
    };
    static Slot slots[MAX_TIMERS];

    // Слоты занимаются по порядку, так что первый свободный значит,
    // что дальше тоже пусто и этот TIMx ещё не встречался.
    for (Slot& slot : slots)
    {
        if (slot.instance == nullptr)
        {
            slot.instance = instance;
            slot.timer.setup(instance);
            slot.timer.setOverflow(PERIOD_US, MICROSEC_FORMAT);
            return &slot.timer;
        }
        if (slot.instance == instance) return &slot.timer;
    }

    return nullptr;
}
