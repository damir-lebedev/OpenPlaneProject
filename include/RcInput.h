// ============================================================
// 5. RC INPUT UTILITIES
//
// Маленький независимый класс для стандартной обработки
// значений RC.
//
// Здесь нет знания о самолёте.
// ============================================================

class RcInput
{
public:

    // --------------------------------------------------------
    // Ограничение RC значения стандартным диапазоном.
    // --------------------------------------------------------

    static uint16_t clamp(uint16_t value)
    {
        return constrain(
            value,
            Config::PWM_MIN,
            Config::PWM_MAX
        );
    }


    // --------------------------------------------------------
    // Преобразование:
//
// 1000 → -maximumDeflection
// 1500 → 0
// 2000 → +maximumDeflection
//
// reverse позволяет инвертировать канал.
// --------------------------------------------------------

    static int16_t centered(
        uint16_t input,
        int16_t maximumDeflection,
        bool reverse = false
    )
    {
        input = clamp(input);

        int32_t output = map(
            input,
            Config::PWM_MIN,
            Config::PWM_MAX,
            -maximumDeflection,
            maximumDeflection
        );


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
};