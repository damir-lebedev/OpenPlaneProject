#pragma once

// ============================================================
// 1. CONFIGURATION
// Все постоянные параметры проекта находятся здесь.
// Логика классов ниже не должна содержать "магических" пинов,
// таймаутов и прочих настроек.
// ============================================================

namespace Config
{
    // --------------------------------------------------------
    // Hardware pinout — выбирается платой сборки (platformio.ini
    // задаёт ровно один из макросов ниже через build_flags, env
    // esp32-c3 / esp32-s3 / esp32-dev). Чтобы добавить новую плату:
    // скопируйте блок, поменяйте номера пинов, добавьте #elif
    // и одноимённый [env:...] в platformio.ini.
    //
    // Ни один пин здесь не проверен вживую, кроме ESP32-C3
    // SuperMini (текущий прототип) — остальные распиновки
    // подобраны по документации чипа (не заняты флешем/USB,
    // не input-only) и требуют проверки под конкретную плату.
    // --------------------------------------------------------

#if defined(BOARD_ESP32_S3)
    // ESP32-S3-DevKitC-1 — планируемый основной лётный контроллер.
    // GPIO19/20 (USB D-/D+) и GPIO26-32 (SPI flash) намеренно не используются.
    constexpr uint8_t PIN_AILERON_LEFT  = 4;
    constexpr uint8_t PIN_AILERON_RIGHT = 5;
    constexpr uint8_t PIN_ELEVATOR      = 6;
    constexpr uint8_t PIN_ESC           = 7;
    constexpr uint8_t PIN_IBUS          = 17;
    constexpr uint8_t PIN_I2C_SDA       = 8;   // = дефолт Wire для esp32s3
    constexpr uint8_t PIN_I2C_SCL       = 9;   // = дефолт Wire для esp32s3

#elif defined(BOARD_ESP32_CLASSIC)
    // Обычная ESP32 38-pin (esp32dev/DOIT/NodeMCU-32S).
    // GPIO0/2/5/12/15 (strapping) и GPIO6-11 (SPI flash) не используются;
    // 34-39 пропущены — они input-only и не годятся для Servo/ESC.
    constexpr uint8_t PIN_AILERON_LEFT  = 13;
    constexpr uint8_t PIN_AILERON_RIGHT = 14;
    constexpr uint8_t PIN_ELEVATOR      = 27;
    constexpr uint8_t PIN_ESC           = 26;
    constexpr uint8_t PIN_IBUS          = 16;
    constexpr uint8_t PIN_I2C_SDA       = 21;  // = дефолт Wire для esp32 classic
    constexpr uint8_t PIN_I2C_SCL       = 22;  // = дефолт Wire для esp32 classic

#elif defined(BOARD_ESP32_C3)
    // ESP32-C3 SuperMini — текущий прототип, единственная плата,
    // на которой это реально прошито и проверено.
    constexpr uint8_t PIN_AILERON_LEFT  = 5;
    constexpr uint8_t PIN_AILERON_RIGHT = 4;
    constexpr uint8_t PIN_ELEVATOR      = 6;
    constexpr uint8_t PIN_ESC           = 7;
    constexpr uint8_t PIN_IBUS          = 8;
    // ВАЖНО: дефолт Wire для esp32c3 — это как раз SDA=8/SCL=9, то
    // есть SDA совпал бы с PIN_IBUS. Поэтому I2C явно уведён на
    // GPIO1/3 — обычные GPIO, не участвующие во flash/USB.
    constexpr uint8_t PIN_I2C_SDA       = 1;
    constexpr uint8_t PIN_I2C_SCL       = 3;

#else
    #error "Не задана плата: используйте env esp32-c3/esp32-s3/esp32-dev из platformio.ini (или определите свой -D BOARD_... и добавьте ветку в Config.h)"
#endif


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
    constexpr uint32_t RX_TIMEOUT_US = 500000;


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