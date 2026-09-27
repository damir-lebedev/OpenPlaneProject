// Реализация rc/RcInput.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "rc/RcInput.h"


auto RcInput::clamp(uint16_t value) -> uint16_t
{
    return constrain(
        value,
        Config::PWM_MIN,
        Config::PWM_MAX
    );
}

auto RcInput::centered(
        uint16_t input,
        int16_t maximumDeflection,
        bool reverse
    ) -> int16_t
{
    input = clamp(input);

    int32_t output = static_cast<int32_t>(map(
        input,
        Config::PWM_MIN,
        Config::PWM_MAX,
        -maximumDeflection,
        maximumDeflection
    ));

    if (reverse)
    {
        output = -output;
    }

    return static_cast<int16_t>(
        constrain(
            output,
            -maximumDeflection,
            maximumDeflection
        )
    );
}
