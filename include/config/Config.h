#pragma once
#include <stdint.h>

#if defined(BOARD_STM32H743)
#include <Arduino.h>   // PA0...PE15 — номера пинов варианта STM32duino
#endif

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
    // esp32-c3 / esp32-s3 / esp32-dev / stm32h743). Чтобы добавить новую плату:
    // скопируйте блок, поменяйте номера пинов, добавьте #elif
    // и одноимённый [env:...] в platformio.ini.
    //
    // Вживую проверены ESP32-S3 N16R8 (основная плата) и ESP32-C3
    // SuperMini (старый прототип). Распиновка esp32-dev подобрана
    // по документации чипа (не заняты флешем/USB, не input-only) и
    // требует проверки под конкретную плату.
    // --------------------------------------------------------

#if defined(BOARD_ESP32_S3)
    // ESP32-S3 N16R8 (DevKitC-1-клон с двумя USB-C "USB"/"COM") —
    // основной лётный контроллер. Проверено вживую: сервы/ESC, iBUS,
    // обе I2C-шины (шина датчиков — ещё на GPIO8/9, см. ниже).
    // Не используются: GPIO0/45/46 (strapping), 19/20 (USB D-/D+),
    // 26-32 (SPI flash), 33-37 (octal PSRAM у N16R8), 43/44 (UART0 ->
    // разъём "COM", на нём Serial), 48 (RGB-светодиод на плате).
    // GPIO3 — тоже strapping (источник JTAG), но влияет, только если
    // это задано в eFuse (по умолчанию нет), — поэтому под вход АЦП
    // датчика тока он годится.
    //
    // Раскладка подогнана под плату полётника (docs/FC_BOARD.md): пины
    // каждого блока разъёмов идут подряд по гребёнке DevKit, чтобы
    // плата разводилась в один слой без пересечений.
    constexpr uint8_t PIN_AILERON_LEFT  = 4;
    constexpr uint8_t PIN_AILERON_RIGHT = 5;
    constexpr uint8_t PIN_ELEVATOR      = 6;
    constexpr uint8_t PIN_ESC           = 7;
    constexpr int8_t  PIN_RUDDER        = 18;  // руль направления + колесо
    constexpr uint8_t PIN_IBUS          = 17;
    // Шина датчиков — правый ряд DevKit, рядом с OLED: там же 3V3-зона
    // платы. На стенде проверена на GPIO8/9 — переезд на 41/42 это
    // только другие пины той же шины Wire.
    constexpr uint8_t PIN_I2C_SDA       = 41;
    constexpr uint8_t PIN_I2C_SCL       = 42;

    // Вторая I2C-шина (Wire1) — только OLED-дисплей. Отдельно от
    // датчиков, чтобы отрисовка кадра (~25 мс на 400 кГц) не
    // задерживала опрос IMU.
    constexpr int8_t PIN_I2C2_SDA       = 1;
    constexpr int8_t PIN_I2C2_SCL       = 2;

    // SPI (ICM42688 + BMP388, общая шина, разные CS) — GPIO11-13
    // это дефолтная распиновка FSPI на большинстве S3-DevKitC плат.
    // Только для стенда: на плате полётника SPI не разведён (датчики
    // там на I2C), GPIO11-14 остаются свободными. GPIO21 на плате —
    // выход LIGHT (ниже); CS барометра трогается, только если в
    // SensorSelection.h выбран барометр по SPI.
    constexpr uint8_t PIN_SENSOR_SPI_SCK  = 12;
    constexpr uint8_t PIN_SENSOR_SPI_MISO = 13;
    constexpr uint8_t PIN_SENSOR_SPI_MOSI = 11;
    constexpr uint8_t PIN_SPI_CS_IMU      = 14;
    constexpr uint8_t PIN_SPI_CS_BARO     = 21;

    // Второй UART — GPS (отдельно от iBUS, который на UART1/GPIO17).
    // У S3 всего 3 аппаратных UART (0/1/2); UART0 занят Serial
    // (GPIO43/44 -> разъём "COM", см. platformio.ini) — GPS на UART2.
    // Правый ряд DevKit, сразу под шиной датчиков: разъём GPS и
    // I2C-разъём его компаса стоят рядом.
    constexpr int8_t PIN_GPS_RX = 39;
    constexpr int8_t PIN_GPS_TX = 40;
    constexpr uint8_t UART_NUM_GPS = 2;

    // РЕЗЕРВ под плату полётника (docs/FC_BOARD.md) — разъёмы
    // разводятся сразу, прошивкой пока не используются.
    constexpr int8_t PIN_AUX1        = 15;  // серво-выход: сброс груза
    constexpr int8_t PIN_AUX2        = 16;  // серво-выход: закрылки (через Y-кабель)
    constexpr int8_t PIN_AUX3        = 47;  // любой цифровой, питание 5V логики (не сервы)
    constexpr int8_t PIN_BUZZER      = 38;  // пищалка через транзистор
    constexpr int8_t PIN_LIGHT       = 21;  // ключ Si2302: огни / фара / электромагнит
    constexpr int8_t PIN_VBAT_ADC    = 8;   // АЦП1: батарея через делитель 56k/10k
    constexpr int8_t PIN_CURRENT_ADC = 3;   // АЦП1: датчик тока через делитель 10k/15k
    constexpr int8_t PIN_TELEM_TX    = 9;   // радиомодем / iBUS-SENS
    constexpr int8_t PIN_TELEM_RX    = 10;

#elif defined(BOARD_ESP32_CLASSIC)
    // Обычная ESP32 38-pin (esp32dev/DOIT/NodeMCU-32S).
    // GPIO0/2/5/12/15 (strapping) и GPIO6-11 (SPI flash) не используются;
    // 34-39 пропущены — они input-only и не годятся для Servo/ESC.
    constexpr uint8_t PIN_AILERON_LEFT  = 13;
    constexpr uint8_t PIN_AILERON_RIGHT = 14;
    constexpr uint8_t PIN_ELEVATOR      = 27;
    constexpr uint8_t PIN_ESC           = 26;
    constexpr int8_t  PIN_RUDDER        = 25;
    constexpr uint8_t PIN_IBUS          = 16;
    constexpr uint8_t PIN_I2C_SDA       = 21;  // = дефолт Wire для esp32 classic
    constexpr uint8_t PIN_I2C_SCL       = 22;  // = дефолт Wire для esp32 classic

    // SPI (ICM42688 + BMP388, общая шина, разные CS) — GPIO18/19/23
    // это аппаратный дефолт VSPI на классической ESP32.
    constexpr uint8_t PIN_SENSOR_SPI_SCK  = 18;
    constexpr uint8_t PIN_SENSOR_SPI_MISO = 19;
    constexpr uint8_t PIN_SENSOR_SPI_MOSI = 23;
    constexpr uint8_t PIN_SPI_CS_IMU      = 32;
    // BMP388: CSB подключён на GPIO5 (физическая распайка модуля на
    // этой плате) — совпадает с CSB/CS-пином SPI-режима чипа.
    constexpr uint8_t PIN_SPI_CS_BARO = 5;

    // Второй UART — GPS (UART2, отдельно от iBUS на UART1/GPIO16).
    // У классической ESP32 3 аппаратных UART, Serial висит на UART0
    // (нативного USB тут нет) — UART2 свободен под GPS.
    constexpr int8_t PIN_GPS_RX = 4;
    constexpr int8_t PIN_GPS_TX = 17;
    constexpr uint8_t UART_NUM_GPS = 2;

    // OLED на этой плате не разведён (-1 = второй I2C-шины нет).
    constexpr int8_t PIN_I2C2_SDA = -1;
    constexpr int8_t PIN_I2C2_SCL = -1;

    // Доп. выходы автопилота (сброс груза, камера) и пищалка.
    // GPIO15 и GPIO2 — strapping-пины: при загрузке на них короткий
    // импульс/подтяжка, для серво и пищалки это безвредно.
    constexpr int8_t PIN_AUX1   = 33;
    constexpr int8_t PIN_AUX2   = 15;
    constexpr int8_t PIN_BUZZER = 2;

#elif defined(BOARD_ESP32_C3)
    // ESP32-C3 SuperMini — старый прототип, на нём борт уже летал
    // (на ручном управлении, без датчиков).
    constexpr uint8_t PIN_AILERON_LEFT  = 5;
    constexpr uint8_t PIN_AILERON_RIGHT = 4;
    constexpr uint8_t PIN_ELEVATOR      = 6;
    constexpr uint8_t PIN_ESC           = 7;
    constexpr int8_t  PIN_RUDDER        = -1;  // свободных пинов нет (см. бюджет ниже) — выход отключён
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
    // BMP388 (тогда PIN_SPI_CS_BARO не нужен), либо используйте
    // плату esp32-s3, где эта проблема не стоит.
    // --------------------------------------------------------
    constexpr uint8_t PIN_SENSOR_SPI_SCK  = 0;
    constexpr uint8_t PIN_SENSOR_SPI_MISO = 10;
    constexpr uint8_t PIN_SENSOR_SPI_MOSI = 20;
    constexpr uint8_t PIN_SPI_CS_IMU      = 21;
    constexpr uint8_t PIN_SPI_CS_BARO     = 2;   // strapping, см. примечание выше

    constexpr int8_t PIN_GPS_RX = 9;   // strapping, см. примечание выше
    constexpr int8_t PIN_GPS_TX = -1;  // не хватило пина — приём без отправки конфигурации

    // У ESP32-C3 всего 2 аппаратных UART (0 и 1) — UART2, который тут
    // хардкодился раньше, физически не существует и ловил краш при
    // gpsUart().begin(). UART1 уже занят iBUS (rcSerial выше), так что
    // GPS — единственный оставшийся вариант, UART0. Serial при этом не
    // страдает: на C3 он идёт через USB-CDC (ARDUINO_USB_CDC_ON_BOOT в
    // platformio.ini), а не через физический UART0.
    constexpr uint8_t UART_NUM_GPS = 0;

    // OLED на этой плате не разведён (-1 = второй I2C-шины нет).
    constexpr int8_t PIN_I2C2_SDA = -1;
    constexpr int8_t PIN_I2C2_SCL = -1;

    // Свободных пинов нет: доп. выходов и пищалки на C3 не будет.
    constexpr int8_t PIN_AUX1   = -1;
    constexpr int8_t PIN_AUX2   = -1;
    constexpr int8_t PIN_BUZZER = -1;

#elif defined(BOARD_STM32H743)
    // STM32H743VIT6 (Cortex-M7 480 МГц, 2 МБ флеша, 1 МБ ОЗУ) —
    // следующая платформа. ЗАГОТОВКА: собирается (env stm32h743), на
    // железе не проверялась — платы пока нет.
    //
    // Сборка идёт под WeAct MiniSTM32H743VITx (тот же чип), и пины
    // выбраны из свободных на ней и проверены по таблицам
    // PeripheralPins варианта STM32duino (у какого пина какой
    // таймер/UART/I2C). Заняты на WeAct: QSPI-флеш (PB2, PB6, PD11-13,
    // PE2), SPI-флеш (PB3, PB4, PD7), µSD (PC8-12, PD2), камера DVP,
    // TFT-LCD (PE10-14), USB (PA11/12), SWD (PA13/14), светодиод PE3,
    // кнопка PC13. Для своей платы — сверить со схемой.
    //
    // Номера — "Arduino-пины" варианта STM32duino (макросы PA0, PD14...
    // из <Arduino.h> выше), а не номера GPIO: у аналоговых пинов это
    // 0xC0+N, в int8_t не помещается — поэтому int16_t (−1 = не разведён).
    //
    // Serial (консоль) — LPUART1: TX PA9, RX PA10 (дефолт варианта).
    // Контроллеры (TIMx, USARTx, I2Cx, SPIx) ядро выбирает по пинам само.

    // Сервовыходы: четыре канала одного таймера TIM2 + TIM4 (50 Гц,
    // см. hal/stm32/Stm32ServoOutput.h).
    constexpr int16_t PIN_AILERON_LEFT  = PA0;   // TIM2_CH1
    constexpr int16_t PIN_AILERON_RIGHT = PA1;   // TIM2_CH2
    constexpr int16_t PIN_ELEVATOR      = PA2;   // TIM2_CH3
    constexpr int16_t PIN_ESC           = PA3;   // TIM2_CH4
    constexpr int16_t PIN_RUDDER        = PD14;  // TIM4_CH3

    // iBUS — UART7. TX на приём не нужен, но UART STM32duino создаётся
    // парой пинов; PE8 оставлен под iBUS-SENS (телеметрия в пульт).
    constexpr int16_t PIN_IBUS    = PE7;   // UART7_RX
    constexpr int16_t PIN_IBUS_TX = PE8;   // UART7_TX

    // I2C2 — датчики (= дефолт Wire варианта).
    constexpr int16_t PIN_I2C_SDA = PB11;
    constexpr int16_t PIN_I2C_SCL = PB10;

    // I2C1 — только экран, как вторая шина на S3. На WeAct это I2C
    // разъёма камеры: вместе с камерой не использовать.
    constexpr int16_t PIN_I2C2_SDA = PB9;
    constexpr int16_t PIN_I2C2_SCL = PB8;

    // SPI2 (= дефолт SPI варианта). Имена у всех плат —
    // PIN_SENSOR_SPI_*, а не PIN_SPI_*: PIN_SPI_SCK/MISO/MOSI — макросы
    // варианта STM32duino, они подменили бы константы Config.
    constexpr int16_t PIN_SENSOR_SPI_SCK  = PB13;
    constexpr int16_t PIN_SENSOR_SPI_MISO = PB14;
    constexpr int16_t PIN_SENSOR_SPI_MOSI = PB15;
    constexpr int16_t PIN_SPI_CS_IMU      = PB12;
    constexpr int16_t PIN_SPI_CS_BARO     = PD10;

    // GPS — USART3.
    constexpr int16_t PIN_GPS_RX = PD9;    // USART3_RX
    constexpr int16_t PIN_GPS_TX = PD8;    // USART3_TX

    // Резерв на будущее (в коде пока не используется) — как на S3.
    constexpr int16_t PIN_AUX1        = PD15;  // TIM4_CH4 — серво-выход
    constexpr int16_t PIN_AUX2        = PE9;   // TIM1_CH1 — серво-выход
    constexpr int16_t PIN_BUZZER      = PE15;
    constexpr int16_t PIN_VBAT_ADC    = PC0;   // ADC1_INP10: батарея через делитель
    constexpr int16_t PIN_CURRENT_ADC = PC1;   // ADC1_INP11: датчик тока
    constexpr int16_t PIN_TELEM_RX    = PD0;   // UART4 (эти же пины — FDCAN1)
    constexpr int16_t PIN_TELEM_TX    = PD1;

#else
    #error "Не задана плата: используйте env esp32-c3/esp32-s3/esp32-dev/stm32h743 из platformio.ini (или определите свой -D BOARD_... и добавьте ветку в Config.h)"
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

    // FS-iA6B при потере связи с пультом НЕ перестаёт слать iBUS —
    // он продолжает выдавать последние значения (проверено на стенде:
    // пульт выключен, кадры идут 130/с). Поэтому таймаут выше ловит
    // только обрыв провода/смерть приёмника, а потерю связи приёмник
    // сообщает значением failsafe, настроенным в пульте: газ (CH3)
    // выставляется ниже нормального диапазона (~900 мкс, см.
    // docs/PILOT_GUIDE.md "Настройка failsafe в пульте"). Газ ниже
    // этого порога = потеря связи. В обычной работе газ не бывает
    // ниже 1000 (конечные точки пульта 100%).
    constexpr uint16_t RX_FAILSAFE_THROTTLE_US = 950;


    // --------------------------------------------------------
    // GPS
    // --------------------------------------------------------

    // Если корректный кадр NAV-PVT не приходил дольше этого времени,
    // GPS считается отключённым/недоступным (см. UbloxM10_Gps::isAvailable()) —
    // тот же принцип, что RX_TIMEOUT_US выше для iBUS. Модуль обычно
    // шлёт решения не реже 1 раза в секунду, 2с — запас на джиттер.
    constexpr uint32_t GPS_TIMEOUT_US = 2000000;


    // --------------------------------------------------------
    // Трубка Пито на двух барометрах (sensors/airspeed/PitotDualBaroAirspeed.h)
    // --------------------------------------------------------

    // Сколько отсчётов (BMP581 — 50 в секунду) усреднять для нуля:
    // разность двух барометров при нулевой скорости.
    constexpr uint16_t PITOT_ZERO_SAMPLES = 50;

    // ФНЧ перепада, с. Барометры меряют не одновременно; 0.1 с убирает
    // этот шум и почти не запаздывает для автопилота.
    constexpr float PITOT_FILTER_TAU_S = 0.1f;

    // Поправочный множитель скорости (1.0 — без поправки). Давление в
    // фюзеляже не строго статическое, трубка может стоять под углом —
    // подбирается полётом по GPS в безветрие туда-обратно.
    constexpr float PITOT_SCALE = 1.0f;

    // Перепад ниже −PITOT_NEGATIVE_FAULT_PA дольше этого времени —
    // трубка неисправна (шланги перепутаны/пережаты, вода).
    constexpr float PITOT_NEGATIVE_FAULT_PA = 30.0f;
    constexpr uint32_t PITOT_NEGATIVE_FAULT_MS = 2000;

    // Отсчёт трубки старше этого — скорость не выдаётся.
    constexpr uint32_t PITOT_STALE_US = 200000;


    // --------------------------------------------------------
    // Standard RC pulse range
    // --------------------------------------------------------

    constexpr uint16_t PWM_MIN    = 1000;
    constexpr uint16_t PWM_CENTER = 1500;
    constexpr uint16_t PWM_MAX    = 2000;


    // --------------------------------------------------------
    // Flight-control limits
    // --------------------------------------------------------

    // Максимальное отклонение элеронов/руля высоты относительно центра
    // при полном ходе стика. Больше 500 не имеет смысла: выход всё
    // равно обрезается до PWM_MIN..PWM_MAX (1000..2000), и лишнее
    // превращается в "мёртвую" зону на краях стика (было 1500/1000 —
    // серво упиралась в край уже на 1/3 и 1/2 хода стика). Уменьшить
    // ход рулей — уменьшите эти числа (или расходы на пульте).
    constexpr int16_t AILERON_MAX_US = 500;
    constexpr int16_t ELEVATOR_MAX_US = 500;

    // Руль направления (CH4) — полный ход стика. На той же серве —
    // рулевое колесо шасси, поэтому ход общий.
    constexpr int16_t RUDDER_MAX_US = 300;


    // --------------------------------------------------------
    // Тумблеры пульта (привязки — config/Controls.h)
    // --------------------------------------------------------

    // Тумблер считается включённым выше этого значения (на FS-i6
    // вниз, к себе = 2000). Не 1500: до первого кадра iBUS все каналы
    // по умолчанию стоят в 1500, и функции (закрылки, сброс груза, RTH)
    // не должны включаться при включении платы.
    constexpr uint16_t SWITCH_ON_US = 1750;


    // --------------------------------------------------------
    // Закрылки (флапероны): Feature::FLAPS / Knob::FLAPS
    //
    // Оба элерона опускаются на FLAPS_DEPLOYED_US — это новая
    // "нейтраль", крен от стика и автопилота работает поверх неё в
    // разные стороны, как обычно. Опускающийся элерон при полном
    // крене упирается в край хода раньше поднимающегося — это
    // работает как дифференциал элеронов (меньше обратного рыскания).
    // --------------------------------------------------------

    // Отклонение каждого элерона вниз при выпущенных закрылках, мкс
    // хода серво. 220 мкс ≈ 20° поворота качалки MG90S; угол самой
    // поверхности зависит от плеч качалок — подбирается здесь.
    constexpr int16_t FLAPS_DEPLOYED_US = 220;

    // Время полного выпуска/уборки. Плавно, а не рывком: резкий
    // выпуск закрылков даёт клевок по тангажу.
    constexpr uint32_t FLAPS_TRANSITION_MS = 1000;


    // --------------------------------------------------------
    // Направление серво (реверс)
    //
    // ControlMixer считает отклонения в физических знаках (см.
    // ControlMixer.h): элерон "+" = задняя кромка вниз, руль высоты
    // "+" = задняя кромка вверх (нос вверх), руль направления "+" =
    // задняя кромка вправо (нос вправо). Как это ложится в PWM,
    // зависит только от того, как серво стоит в самолёте, — это и
    // задают флаги ниже (аналог реверса каналов на пульте, но на
    // стороне платы, чтобы автопилот и стики крутили рули одинаково).
    //
    // Значения по умолчанию повторяют прежнее поведение прошивки для
    // стиков. Проверка на собранном самолёте: правый стик вправо —
    // правый элерон ВВЕРХ, левый ВНИЗ; стик на себя — руль высоты
    // ВВЕРХ; левый стик вправо — руль направления и колесо ВПРАВО;
    // закрылки (SwB) — оба элерона ВНИЗ. Не так — поменять
    // соответствующий флаг.
    //
    // Элероны: серво в крыле стоят зеркально, поэтому реверсирован
    // ровно один. Признак, что оба флага одинаковые: крен двигает
    // элероны в одну сторону, а закрылки — в разные. Если закрылки
    // идут ВВЕРХ (и крен наоборот) — реверс не у того: поменять оба
    // флага местами. Микшер не трогать — крен и закрылки там в одних
    // знаках, и автопилот на них рассчитывает.
    // --------------------------------------------------------

    constexpr bool AILERON_LEFT_REVERSED  = false;
    constexpr bool AILERON_RIGHT_REVERSED = true;
    constexpr bool ELEVATOR_REVERSED      = true;
    constexpr bool RUDDER_REVERSED        = false;


    // --------------------------------------------------------
    // Установка IMU
    //
    // Проще всего — не думать об этом: команда 'o' в консоли
    // (калибровка установки, 3 позы) определяет, как плата стоит в
    // самолёте — под любым углом, хоть вверх ногами, — и хранит это в
    // NVS. Значение ниже используется, только пока такой калибровки
    // нет.
    //
    // Поворот осей ЧИПА IMU относительно самолёта вокруг вертикали,
    // по часовой стрелке, если смотреть сверху (0/90/180/270): куда
    // смотрит ось X самого чипа, если нос самолёта — "12 часов".
    //   0   — ось X чипа к носу
    //   90  — ось X чипа вправо
    //   180 — ось X чипа к хвосту
    //   270 — ось X чипа влево
    // Плата должна лежать чипом вверх.
    //
    // 90 — для текущего GY-521 (клон с MPU6500), уложенного стрелкой X
    // (на шелкографии) к носу: чип на нём запаян повёрнутым, и
    // нарисованные стрелки не совпадают с осями чипа (проверено на
    // стенде: стрелка X вверх -> ось Y чипа вверх, стрелка Y вверх ->
    // ось -X чипа вверх). Проверка после любой перестановки: нос
    // вверх -> тангаж (P на OLED) растёт в плюс, правое крыло вниз ->
    // крен (R) в плюс.
    // --------------------------------------------------------

    constexpr uint16_t IMU_ROTATION_CW_DEG = 90;

    // Установка компаса — то же правило, что для IMU: куда смотрит ось
    // X чипа магнитометра относительно носа. Проверка на собранном
    // самолёте: поворот носа по часовой (сверху) -> курс растёт, нос на
    // север -> ~0°.
    constexpr uint16_t MAG_ROTATION_CW_DEG = 0;


    // --------------------------------------------------------
    // ARM (SwA, CH5 — см. Channels::ARM и ArmingManager.h)
    // --------------------------------------------------------

    // Тумблер ARM считается включённым выше этого значения.
    constexpr uint16_t ARM_SWITCH_ON_US = 1750;

    // Ниже этого значения газ считается LOW — армиться можно только
    // с газом внизу.
    constexpr uint16_t THROTTLE_LOW_US = 1050;


    // --------------------------------------------------------
    // Failsafe outputs
    // --------------------------------------------------------

    constexpr uint16_t FAILSAFE_AILERON  = 1500;
    constexpr uint16_t FAILSAFE_ELEVATOR = 1500;
    constexpr uint16_t FAILSAFE_RUDDER   = 1500;
    constexpr uint16_t FAILSAFE_THROTTLE = 1000;

    // Планирование при потере связи в воздухе (борт заармлен): мотор
    // выключен, автопилот держит эти углы (градусы, авиационные знаки).
    // Тангаж чуть ниже горизонта — без мотора самолёт держит скорость
    // и не сваливается. Крен 0 — прямо; 10-20 — пологий круг, самолёт
    // остаётся рядом с пилотом, а не улетает по прямой. Если борт не
    // заармлен (на земле) или IMU не отвечает — рули просто в нейтраль.
    constexpr float FAILSAFE_GLIDE_ROLL_DEG  = 0.0f;
    constexpr float FAILSAFE_GLIDE_PITCH_DEG = -3.0f;

    // Потеря связи в воздухе при живом GPS и известном доме — не
    // планирование, а возврат домой с мотором (как у ArduPilot/INAV):
    // над домом самолёт кружит на высоте RTH, пока связь не вернётся.
    // Без GPS/дома — всегда планирование выше. false — только планирование.
    constexpr bool FAILSAFE_RTH = true;


    // --------------------------------------------------------
    // АВТОПИЛОТ: РЕЖИМЫ И ФУНКЦИИ
    //
    // Что включается каким тумблером — config/Controls.h; что делает
    // каждый режим — docs/AUTOPILOT_GUIDE.md. Здесь — числа.
    // Углы в градусах (авиационные знаки), скорости в м/с, высоты —
    // относительно точки включения (барометр), газ в % от хода.
    // --------------------------------------------------------

    // Стабилизация (STABILIZE, ALT_HOLD, навигация). Полный стик —
    // MAX_BANK (крутилка Knob::MAX_BANK: 15 ... 45 ... 60) по крену и
    // STAB_MAX_PITCH по тангажу.
    constexpr float MAX_BANK_DEG = 45.0f;
    constexpr float MAX_BANK_MIN_DEG = 15.0f;
    constexpr float MAX_BANK_MAX_DEG = 60.0f;
    constexpr float STAB_MAX_PITCH_DEG = 25.0f;

    // Интегратор ПИД крена/тангажа копится, только пока ошибка угла
    // меньше этого: I-составляющая — для постоянной поправки (перекос,
    // центровка), а не для выхода из большого крена.
    constexpr float STAB_INTEGRATOR_ZONE_DEG = 10.0f;

    // Крутилка Knob::STAB_GAIN — множитель выхода ПИД крена/тангажа.
    constexpr float STAB_GAIN_MIN = 0.25f;
    constexpr float STAB_GAIN_MAX = 2.0f;

    // ACRO: полный стик — скорость вращения; гироскоп гасит то, чего
    // пилот не просил (порыв ветра), с коэффициентом мкс на °/с.
    constexpr float ACRO_MAX_RATE_DPS = 180.0f;
    constexpr float ACRO_RATE_GAIN_US_PER_DPS = 1.5f;

    // Навигация: крен на ошибку курса (° крена на ° ошибки), не больше
    // NAV_BANK_LIMIT_DEG (и не больше MAX_BANK); курс по GPS — когда
    // путевая скорость выше NAV_GPS_COURSE_MIN_SPEED_MS, иначе по
    // компасу/гироскопу.
    constexpr float NAV_COURSE_GAIN = 1.0f;
    constexpr float NAV_BANK_LIMIT_DEG = 40.0f;
    constexpr float NAV_GPS_COURSE_MIN_SPEED_MS = 3.0f;
    // Скорость, если её не меряет ни трубка Пито, ни GPS.
    constexpr float NAV_ASSUMED_SPEED_MS = 15.0f;

    // Высота рулём высоты: желаемая вертикальная скорость =
    // NAV_ALT_GAIN · ошибка высоты (в пределах NAV_MAX_CLIMB/SINK),
    // тангаж — ПИ по ошибке вертикальной скорости (°/(м/с)).
    constexpr float NAV_ALT_GAIN = 0.4f;
    constexpr float NAV_MAX_CLIMB_MS = 3.0f;
    constexpr float NAV_MAX_SINK_MS = 3.0f;
    constexpr float NAV_CLIMB_KP_DEG = 3.0f;
    constexpr float NAV_CLIMB_KI_DEG = 1.0f;
    constexpr float NAV_MAX_CLIMB_PITCH_DEG = 15.0f;
    constexpr float NAV_MAX_DIVE_PITCH_DEG = -12.0f;

    // Газ в автоматических режимах (CRUISE, LOITER, RTH): крутилка
    // Knob::CRUISE_SPEED задаёт газ 30 ... 55 ... 85 %, а с трубкой
    // Пито — воздушную скорость 10 ... 14 ... 22 м/с (газ её держит).
    // На каждый 1 м/с требуемого набора — ещё THROTTLE_PER_CLIMB % газа.
    constexpr float CRUISE_THROTTLE_PCT = 55.0f;
    constexpr float CRUISE_THROTTLE_MIN_PCT = 30.0f;
    constexpr float CRUISE_THROTTLE_MAX_PCT = 85.0f;
    constexpr float CRUISE_AIRSPEED_MS = 14.0f;
    constexpr float CRUISE_AIRSPEED_MIN_MS = 10.0f;
    constexpr float CRUISE_AIRSPEED_MAX_MS = 22.0f;
    constexpr float AIRSPEED_THROTTLE_KP = 6.0f;   // % на 1 м/с
    constexpr float AIRSPEED_THROTTLE_KI = 2.0f;   // %/с на 1 м/с
    constexpr float THROTTLE_PER_CLIMB_PCT = 8.0f;
    constexpr float AUTO_THROTTLE_MIN_PCT = 15.0f;
    constexpr float AUTO_THROTTLE_MAX_PCT = 100.0f;

    // Защита от сваливания (с трубкой Пито): приборная скорость ниже
    // STALL_SPEED_MS + STALL_MARGIN_MS — нос не выше горизонта, крен не
    // больше STALL_BANK_LIMIT_DEG, газ на максимум.
    constexpr float STALL_SPEED_MS = 8.0f;
    constexpr float STALL_MARGIN_MS = 2.0f;
    constexpr float STALL_BANK_LIMIT_DEG = 20.0f;

    // Круги (LOITER, над домом в RTH): радиус 25 ... 50 ... 150 м
    // (Knob::LOITER_RADIUS), по часовой стрелке. LOITER_CONVERGENCE —
    // насколько круто самолёт заходит на окружность издалека.
    constexpr float LOITER_RADIUS_M = 50.0f;
    constexpr float LOITER_RADIUS_MIN_M = 25.0f;
    constexpr float LOITER_RADIUS_MAX_M = 150.0f;
    constexpr float LOITER_CONVERGENCE = 2.0f;

    // RTH: высота возврата (ниже — набирает по пути, выше — остаётся).
    constexpr float RTH_ALTITUDE_M = 40.0f;

    // Дом записывается при ARM, если GPS хороший: 3D-фикс, не меньше
    // HOME_MIN_SATELLITES спутников, точность не хуже HOME_MAX_HACC_M.
    constexpr uint8_t HOME_MIN_SATELLITES = 6;
    constexpr float HOME_MAX_HACC_M = 5.0f;

    // Геозабор (Feature::GEOFENCE или GEOFENCE_ALWAYS_ON): дальше
    // радиуса или выше потолка — RTH.
    constexpr bool GEOFENCE_ALWAYS_ON = false;
    constexpr float FENCE_RADIUS_M = 500.0f;
    constexpr float FENCE_ALTITUDE_M = 120.0f;

    // Запуск с руки (LAUNCH): пилот поднял газ — запуск взведён, мотор
    // стоит; бросок — ускорение вперёд больше LAUNCH_ACCEL_G дольше
    // LAUNCH_ACCEL_TIME_MS; через LAUNCH_MOTOR_DELAY_MS (рука ушла от
    // винта) — газ LAUNCH_THROTTLE_PCT и набор с тангажом
    // LAUNCH_CLIMB_PITCH_DEG, пока не пройдёт LAUNCH_CLIMB_MS или не
    // наберётся LAUNCH_ALTITUDE_M. Дальше — как CRUISE.
    constexpr float LAUNCH_ACCEL_G = 1.5f;
    constexpr uint32_t LAUNCH_ACCEL_TIME_MS = 40;
    constexpr uint32_t LAUNCH_MOTOR_DELAY_MS = 300;
    constexpr float LAUNCH_THROTTLE_PCT = 100.0f;
    constexpr float LAUNCH_CLIMB_PITCH_DEG = 15.0f;
    constexpr uint32_t LAUNCH_CLIMB_MS = 6000;
    constexpr float LAUNCH_ALTITUDE_M = 30.0f;

    // Посадка (AUTO_LAND): мотор выключен, планирование с тангажом
    // LAND_GLIDE_PITCH_DEG по курсу, ниже LAND_FLARE_ALTITUDE_M —
    // выравнивание LAND_FLARE_PITCH_DEG.
    constexpr float LAND_GLIDE_PITCH_DEG = -4.0f;
    constexpr float LAND_FLARE_ALTITUDE_M = 3.0f;
    constexpr float LAND_FLARE_PITCH_DEG = 4.0f;

    // Парение (SOARING): мотор выключен, планирование с тангажом
    // SOAR_GLIDE_PITCH_DEG. Вариометр (с поправкой на полную энергию,
    // если есть трубка Пито) выше SOAR_THERMAL_CLIMB_MS дольше
    // SOAR_THERMAL_CONFIRM_MS — термик, круги с креном SOAR_CIRCLE_BANK_DEG.
    // Среднее ниже SOAR_EXIT_CLIMB_MS за SOAR_EXIT_WINDOW_MS — выход.
    // Ниже SOAR_MIN_ALTITUDE_M — набор с мотором до SOAR_MAX_ALTITUDE_M;
    // дальше SOAR_MAX_DISTANCE_M от дома — планирование к дому.
    constexpr float SOAR_GLIDE_PITCH_DEG = -3.0f;
    constexpr float SOAR_THERMAL_CLIMB_MS = 0.5f;
    constexpr uint32_t SOAR_THERMAL_CONFIRM_MS = 1500;
    constexpr float SOAR_CIRCLE_BANK_DEG = 25.0f;
    constexpr float SOAR_CIRCLE_PITCH_DEG = -1.0f;
    constexpr float SOAR_EXIT_CLIMB_MS = -0.2f;
    constexpr uint32_t SOAR_EXIT_WINDOW_MS = 8000;
    constexpr float SOAR_MIN_ALTITUDE_M = 30.0f;
    constexpr float SOAR_MAX_ALTITUDE_M = 100.0f;
    constexpr float SOAR_MAX_DISTANCE_M = 400.0f;

    // RESCUE ("спасите"): крылья ровно, нос RESCUE_PITCH_DEG, газ RESCUE_THROTTLE_PCT.
    constexpr float RESCUE_PITCH_DEG = 8.0f;
    constexpr float RESCUE_THROTTLE_PCT = 70.0f;

    // Автотриммер (Feature::AUTO_TRIM): пока самолёт летит ровно
    // (крен < AUTOTRIM_MAX_ROLL_DEG, вращение < AUTOTRIM_MAX_RATE_DPS),
    // постоянная команда рулей "перетекает" в триммер со скоростью
    // AUTOTRIM_RATE (доля в секунду); триммер не больше AUTOTRIM_MAX_US.
    // Сохраняется в NVS (STM32 — во флеш) после DISARM на земле.
    constexpr float AUTOTRIM_RATE = 0.2f;
    constexpr float AUTOTRIM_MAX_US = 120.0f;
    constexpr float AUTOTRIM_MAX_ROLL_DEG = 15.0f;
    constexpr float AUTOTRIM_MAX_RATE_DPS = 30.0f;

    // Триммер записывается во флеш после DISARM, но только когда
    // самолёт стоит: на ESP32 запись NVS останавливает оба ядра на
    // ~0.4 с, и DISARM в воздухе (пилот выключил мотор тумблером) не
    // должен замораживать рули. "Стоит" — у земли по барометру, почти
    // без вертикальной скорости, и медленнее порога по трубке/GPS
    // (датчика нет — его условие не проверяется).
    constexpr float AUTOTRIM_SAVE_MAX_ALT_M = 3.0f;
    constexpr float AUTOTRIM_SAVE_MAX_CLIMB_MS = 0.5f;
    constexpr float AUTOTRIM_SAVE_MAX_SPEED_MS = 3.0f;

    // Координация разворота (Feature::TURN_COORDINATION; в
    // навигационных режимах включена всегда): руль направления =
    // TURN_COORD_RUDDER_MIX · команда элеронов (гасит обратное
    // рыскание), руль высоты += TURN_COORD_PITCH_US · (1/cos крена − 1)
    // (в крене подъёмная сила уходит вбок — без этого нос опускается).
    constexpr float TURN_COORD_RUDDER_MIX = 0.3f;
    constexpr float TURN_COORD_PITCH_US = 150.0f;
    constexpr bool TURN_COORD_IN_NAV_MODES = true;

    // Воздушный тормоз (Feature::AIRBRAKE): оба элерона вверх на столько мкс.
    constexpr int16_t AIRBRAKE_US = 250;

    // Сброс груза (Feature::PAYLOAD_DROP) — серво AUX1.
    constexpr uint16_t PAYLOAD_CLOSED_US = 1000;
    constexpr uint16_t PAYLOAD_OPEN_US = 2000;

    // Камера (серво AUX2): угол Knob::CAMERA_TILT −90 ... 0 ... +30°,
    // Feature::CAMERA_STAB — вычитается тангаж самолёта. Ход серво —
    // CAMERA_US_PER_DEG мкс на градус от 1500.
    constexpr float CAMERA_TILT_MIN_DEG = -90.0f;
    constexpr float CAMERA_TILT_MAX_DEG = 30.0f;
    constexpr float CAMERA_US_PER_DEG = 500.0f / 90.0f;

    // Чувствительность стиков (Knob::RATES): 30 ... 65 ... 100 %.
    constexpr float RATES_MIN_PCT = 30.0f;
    constexpr float RATES_MAX_PCT = 100.0f;

    // Пищалка: связь потеряна на земле (DISARM) дольше этого — пищит
    // "где я" (самолёт упал в траву). Feature::BEEPER — пищит сразу.
    constexpr uint32_t LOST_MODEL_BEEP_DELAY_MS = 10000;


    // --------------------------------------------------------
    // Цикл управления
    // --------------------------------------------------------

    // Период основного цикла (FlightController::update()). 2 мс = 500 Гц:
    // IMU обновляется с частотой 1 кГц, чтение по I2C занимает ~0.45 мс,
    // так что запас по времени большой. Цикл держится по
    // vTaskDelayUntil (см. main.cpp), а не "delay(2) после работы".
    constexpr uint32_t LOOP_PERIOD_MS = 2;


    // --------------------------------------------------------
    // Веб-дашборд (точка доступа Wi-Fi, см. WebDebugServer.h)
    //
    // Пароль слабый — дашборд для стенда и поля, не для полёта над
    // людьми: кто подключился к точке, может менять режим и ПИД.
    // --------------------------------------------------------

    constexpr const char* WIFI_AP_SSID = "OpenPlane-Debug";
    constexpr const char* WIFI_AP_PASSWORD = "12345678";
    constexpr uint16_t WEB_SERVER_PORT = 80;


    // --------------------------------------------------------
    // Телеметрия MAVLink по радиомодему (telemetry/MavlinkTelemetry.h)
    //
    // UART PIN_TELEM_RX/TX: SiK 433/868/915 МГц, ELRS-приёмник в режиме
    // MAVLink, ESP-01 как Wi-Fi-мост. Наземная станция —
    // QGroundControl или Mission Planner: борт представляется
    // самолётом ArduPilot (режимы видны именами ArduPlane).
    // На ESP32-S3 свободного UART нет — там дашборд по Wi-Fi.
    // --------------------------------------------------------

    constexpr uint32_t TELEM_BAUDRATE = 57600;   // дефолт радиомодемов SiK
    constexpr uint8_t MAVLINK_SYSTEM_ID = 1;
    constexpr uint8_t MAVLINK_COMPONENT_ID = 1;  // MAV_COMP_ID_AUTOPILOT1


    // --------------------------------------------------------
    // Чёрный ящик (telemetry/BlackBox.h, docs/BLACKBOX.md)
    //
    // Пишет полёт во встроенный флеш: раздел "blackbox" ~13.9 МБ
    // (partitions_blackbox.csv), около 20 КБ/с со стендовым набором
    // датчиков — ~11 минут. Только ESP32-S3 N16R8: на других платах
    // раздела нет, и ящик выключен.
    //
    // Запись начинается, когда заармлено и газ поднят (стик или ESC
    // выше THROTTLE_LOW_US), и захватывает BLACKBOX_PREROLL_MS до
    // этого. Кончается: через BLACKBOX_POSTROLL_MS после DISARM; или
    // заармлено, но мотор стоит и самолёт неподвижен
    // BLACKBOX_LANDED_STOP_MS (сел или упал, а DISARM забыли). Потеря
    // связи, мотор в ноль в воздухе и планирование запись не
    // останавливают.
    // --------------------------------------------------------

    // Очередь записей в PSRAM (8 МБ на N16R8): предзапись и запас на
    // случай, когда флеш не успевает или стёртое место кончилось в
    // воздухе. Без PSRAM — маленькая очередь во внутренней памяти.
    constexpr uint32_t BLACKBOX_RING_BYTES = 4UL * 1024 * 1024;
    constexpr uint32_t BLACKBOX_RING_NO_PSRAM_BYTES = 32UL * 1024;

    constexpr uint32_t BLACKBOX_PREROLL_MS = 10000;
    constexpr uint32_t BLACKBOX_POSTROLL_MS = 10000;

    // "Стоит на земле": мотор стоит, вращение по всем осям меньше
    // LANDED_GYRO, ускорение — 1g ± LANDED_ACCEL, по барометру почти
    // нет вертикальной скорости, по GPS/трубке медленнее LANDED_SPEED
    // (датчика нет — его условие не проверяется). Всё это без перерыва
    // BLACKBOX_LANDED_STOP_MS: в воздухе так ровно не бывает.
    constexpr uint32_t BLACKBOX_LANDED_STOP_MS = 30000;
    constexpr float BLACKBOX_LANDED_GYRO_DPS = 5.0f;
    constexpr float BLACKBOX_LANDED_ACCEL_G = 0.1f;
    constexpr float BLACKBOX_LANDED_CLIMB_MS = 0.5f;
    constexpr float BLACKBOX_LANDED_SPEED_MS = 2.0f;

    // Перезагрузка из-за сбоя (паника, сторожевой таймер, просадка
    // питания) — запись включается сразу после старта: если это
    // случилось в воздухе, видно, что было дальше. Такая запись идёт
    // не меньше BLACKBOX_RESET_HOLD_MS, даже без ARM.
    constexpr uint32_t BLACKBOX_RESET_HOLD_MS = 60000;

    // Сколько стёртого места держать наготове к следующему полёту.
    // Стирается только на земле без ARM (стирание останавливает оба
    // ядра на ~0.15 с), старые полёты — целиком, от самого старого;
    // последний записанный полёт не стирается никогда. Не хватило в
    // воздухе — новое копится в очереди и дописывается после посадки.
    constexpr uint32_t BLACKBOX_MIN_FREE_BYTES = 10UL * 1024 * 1024;

    // Пауза между стираниями, мс: полётный цикл на земле живёт между ними.
    constexpr uint32_t BLACKBOX_ERASE_PAUSE_MS = 100;

    // IMU пишется каждый BLACKBOX_IMU_DIVIDER-й такт (1 — каждый, 500 Гц).
    constexpr uint8_t BLACKBOX_IMU_DIVIDER = 1;

    // Батарея и датчик тока (только S3, пины PIN_VBAT_ADC/PIN_CURRENT_ADC,
    // делители на плате полётника — docs/FC_BOARD.md, блок B): во сколько
    // раз напряжение на входе больше, чем на пине АЦП. Делитель не
    // распаян — в логе будет шум, полёту это не мешает.
    constexpr float BLACKBOX_VBAT_DIVIDER = (56.0f + 10.0f) / 10.0f;
    constexpr float BLACKBOX_CURRENT_DIVIDER = (10.0f + 15.0f) / 15.0f;


    // --------------------------------------------------------
    // Debug
    // --------------------------------------------------------

    // Как часто DebugLogger проверяет каналы лога. Что именно выводить
    // и в каком режиме (выкл / при изменении / постоянно) — меню консоли
    // ('l' в мониторе порта), хранится в NVS (telemetry/LogSettings.h).
    constexpr uint32_t DEBUG_INTERVAL_MS = 100;

    // Допуск на дребезг RC-каналов и PWM-выходов (мкс) в режиме "при
    // изменении": разница меньше этого значения не считается изменением.
    constexpr uint16_t DEBUG_CHANGE_DEADBAND_US = 3;
}
