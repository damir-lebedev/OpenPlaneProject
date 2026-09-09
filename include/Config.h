// ============================================================
// 1. CONFIGURATION
// Все постоянные параметры проекта находятся здесь.
// Логика классов ниже не должна содержать "магических" пинов,
// таймаутов и прочих настроек.
// ============================================================

namespace Config
{
    // --------------------------------------------------------
    // Hardware pinout
    // --------------------------------------------------------

    constexpr uint8_t PIN_AILERON_LEFT  = 5;
    constexpr uint8_t PIN_AILERON_RIGHT = 4;
    constexpr uint8_t PIN_ELEVATOR      = 6;
    constexpr uint8_t PIN_ESC            = 7;
    constexpr uint8_t PIN_IBUS           = 8;


    // --------------------------------------------------------
    // iBUS configuration
    // --------------------------------------------------------

    constexpr uint8_t IBUS_CHANNELS = 10;
    constexpr uint8_t IBUS_FRAME_LENGTH = 32;

    constexpr uint8_t IBUS_HEADER_0 = 0x20;
    constexpr uint8_t IBUS_HEADER_1 = 0x40;

    constexpr uint32_t IBUS_BAUDRATE = 115200;

    // При отсутствии корректного iBUS кадра дольше этого
    // времени приёмник считается потерянным.
    constexpr uint32_t RX_TIMEOUT_US = 100000;


    // --------------------------------------------------------
    // Standard RC pulse range
    // --------------------------------------------------------

    constexpr uint16_t PWM_MIN    = 1000;
    constexpr uint16_t PWM_CENTER = 1500;
    constexpr uint16_t PWM_MAX    = 2000;


    // --------------------------------------------------------
    // Flight-control limits
    // --------------------------------------------------------

    // Максимальное отклонение элеронов относительно центра.
    constexpr int16_t AILERON_MAX_US = 1500;

    // Максимальное отклонение руля высоты.
    constexpr int16_t ELEVATOR_MAX_US = 1000;


    // --------------------------------------------------------
    // Throttle safety
    // --------------------------------------------------------

    // Ниже этого значения газ считается LOW.
    constexpr uint16_t THROTTLE_LOW_US = 1050;

    // Минимальное время нахождения газа в LOW.
    constexpr uint32_t ARM_LOW_TIME_MS = 1500;


    // --------------------------------------------------------
    // Failsafe outputs
    // --------------------------------------------------------

    constexpr uint16_t FAILSAFE_AILERON  = 1500;
    constexpr uint16_t FAILSAFE_ELEVATOR = 1500;
    constexpr uint16_t FAILSAFE_THROTTLE = 1000;


    // --------------------------------------------------------
    // Throttle boost
    // --------------------------------------------------------

    // В обычном режиме максимальный газ = 40%.
    constexpr uint16_t THROTTLE_LIMIT_PERCENT = 40;

    // Полный газ разрешается на 5 секунд.
    constexpr uint32_t THROTTLE_BOOST_TIME_MS = 5000;


    // --------------------------------------------------------
    // Debug
    // --------------------------------------------------------

    constexpr uint32_t DEBUG_INTERVAL_MS = 100;
}