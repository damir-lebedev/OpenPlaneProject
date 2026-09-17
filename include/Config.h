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

    // SPI (ICM42688 + BMP388, общая шина, разные CS) — GPIO10-13
    // это дефолтная распиновка FSPI на большинстве S3-DevKitC плат.
    constexpr uint8_t PIN_SPI_SCK          = 12;
    constexpr uint8_t PIN_SPI_MISO         = 13;
    constexpr uint8_t PIN_SPI_MOSI         = 11;
    constexpr uint8_t PIN_SPI_CS_ICM42688  = 10;
    constexpr uint8_t PIN_SPI_CS_BMP388    = 21;

    // Второй UART — GPS (отдельно от iBUS, который на UART1/GPIO17).
    // У S3 всего 3 аппаратных UART (0/1/2); 0 не занят — Serial здесь
    // идёт через USB-CDC (см. ARDUINO_USB_CDC_ON_BOOT в platformio.ini),
    // но GPS всё равно на отдельном UART2, чтобы не зависеть от этого.
    constexpr int8_t PIN_GPS_RX = 15;
    constexpr int8_t PIN_GPS_TX = 16;
    constexpr uint8_t UART_NUM_GPS = 2;

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

    // SPI (ICM42688 + BMP388, общая шина, разные CS) — GPIO18/19/23
    // это аппаратный дефолт VSPI на классической ESP32.
    constexpr uint8_t PIN_SPI_SCK          = 18;
    constexpr uint8_t PIN_SPI_MISO         = 19;
    constexpr uint8_t PIN_SPI_MOSI         = 23;
    constexpr uint8_t PIN_SPI_CS_ICM42688  = 32;
    constexpr uint8_t PIN_SPI_CS_BMP388    = 33;

    // Второй UART — GPS (UART2, отдельно от iBUS на UART1/GPIO16).
    // У классической ESP32 3 аппаратных UART, Serial висит на UART0
    // (нативного USB тут нет) — UART2 свободен под GPS.
    constexpr int8_t PIN_GPS_RX = 4;
    constexpr int8_t PIN_GPS_TX = 17;
    constexpr uint8_t UART_NUM_GPS = 2;

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

    // --------------------------------------------------------
    // ВНИМАНИЕ: бюджет пинов на SuperMini (обычно доступны только
    // GPIO0-10 + GPIO20/21 на разъёмах = 13 пинов; GPIO18/19 заняты
    // нативным USB — ARDUINO_USB_CDC_ON_BOOT=1) — на пределе. После
    // ailerons/elevator/esc/ibus/i2c (7 пинов) свободно ровно 6:
    // 0, 2, 9, 10, 20, 21 — а нужно 5 под SPI + 2 под GPS UART = 7.
    // Поэтому здесь: GPIO2 и GPIO9 — strapping-пины (та же категория
    // риска, что уже принята для PIN_IBUS=8 выше: их состояние важно
    // только в момент включения/сброса, после старта прошивки это
    // обычные GPIO); GPIO9 особенно чувствителен — низкий уровень на
    // нём при сбросе переводит чип в режим прошивки, так что модуль
    // GPS должен либо молчать на TX до полной загрузки, либо стоит
    // выбрать другой пин под GPS_RX по месту.
    // Пина под GPS_TX физически не остаётся: GPS работает только на
    // приём (без отправки UBX-CFG для смены частоты/включения
    // NAV-PVT — см. PIN_GPS_TX = -1 и комментарий в UbloxM10_Gps.h).
    // Если нужен GPS с полной настройкой на C3 — освободите пин,
    // выбрав в SensorSelection.h I2C-барометр (BME280) вместо SPI
    // BMP388 (тогда PIN_SPI_CS_BMP388 не нужен), либо используйте
    // плату esp32-s3, где эта проблема не стоит.
    // --------------------------------------------------------
    constexpr uint8_t PIN_SPI_SCK          = 0;
    constexpr uint8_t PIN_SPI_MISO         = 10;
    constexpr uint8_t PIN_SPI_MOSI         = 20;
    constexpr uint8_t PIN_SPI_CS_ICM42688  = 21;
    constexpr uint8_t PIN_SPI_CS_BMP388    = 2;   // strapping, см. примечание выше

    constexpr int8_t PIN_GPS_RX = 9;   // strapping, см. примечание выше
    constexpr int8_t PIN_GPS_TX = -1;  // не хватило пина — приём без отправки конфигурации

    // У ESP32-C3 всего 2 аппаратных UART (0 и 1) — UART2, который тут
    // хардкодился раньше, физически не существует и ловил краш при
    // gpsUart().begin(). UART1 уже занят iBUS (rcSerial выше), так что
    // GPS — единственный оставшийся вариант, UART0. Serial при этом не
    // страдает: на C3 он идёт через USB-CDC (ARDUINO_USB_CDC_ON_BOOT в
    // platformio.ini), а не через физический UART0.
    constexpr uint8_t UART_NUM_GPS = 0;

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
    // GPS
    // --------------------------------------------------------

    // Если корректный кадр NAV-PVT не приходил дольше этого времени,
    // GPS считается отключённым/недоступным (см. UbloxM10_Gps::isAvailable()) —
    // тот же принцип, что RX_TIMEOUT_US выше для iBUS. Модуль обычно
    // шлёт решения не реже 1 раза в секунду, 2с — запас на джиттер.
    constexpr uint32_t GPS_TIMEOUT_US = 2000000;


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

    // Не печатать новый кадр отладки, если он не отличается от предыдущего
    // (с учётом допуска ниже) — иначе Serial Monitor заваливает одинаковыми
    // строками каждые DEBUG_INTERVAL_MS, даже когда самолёт просто лежит
    // на столе. Как только что-то реально меняется — печать возобновляется.
    constexpr bool DEBUG_ONLY_ON_CHANGE = true;

    // Допуск на дребезг RC-каналов и PWM-выходов (мкс): разница меньше
    // этого значения не считается изменением.
    constexpr uint16_t DEBUG_CHANGE_DEADBAND_US = 3;
}