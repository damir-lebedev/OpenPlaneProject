#pragma once

// ============================================================
// Плата JXFPVF405V3 — из Betaflight (CLI), tools/bf_board.py.
// Источник: дамп Studio 2026-11-16 с родного BF 4.5.1 (JXFPVF405V3)
// Только факты о плате: ножки, номера шин, масштабы датчиков. Заново —
//   python tools/bf_board.py header <дамп или config.h> --out <этот файл>
// Номера пинов — макросы STM32duino (PA0…PE15), −1 — нет.
// Внимание: OSD на SPI2 — выведено (NSS шины PB12), в дампе нет dump hardware.
// Внимание: гироскоп на SPI1 — выведено (NSS шины PA4), в дампе нет dump hardware.
// Внимание: флеш на SPI3 — выведено (свободная шина PC13), в дампе нет dump hardware.
//
// GETFUN F405V3 (стек «F405V3 + 100A ESC», продавец WXL) внутри — таргет
// Betaflight JXFPVF405V3 (manufacturer JFPV), его тоже нет в публичных
// репозиториях. Проверено на столе 2026-11-16 пробной прошивкой:
// ICM42688P на SPI1 (CS PA4, WHO_AM_I 0x47), барометр DPS310 на I2C1 0x76
// (ID 0x10), батарея на PC0 со шкалой 110 (22.4 В на 6S). Кварца, видимо,
// нет: родной BF работал от HSI (status: PLLP-HSI) — clock_f405.cpp сам
// падает на HSI. Родной прошивки в запасе нет. Руками после генератора:
// PINIO1/2 — USER1/USER2 (переключатель камер и 12 В, по описанию платы).
// ============================================================

#define OPB_BOARD_NAME "JXFPVF405V3"
#define OPB_HSE_MHZ 8

// Выходы M1…M8 — номера Betaflight (у квадро 1 — задний правый).
#define OPB_MOTOR_COUNT 8
#define OPB_MOTOR_PINS PA15, PA10, PA9, PA8, PC9, PC8, PB11, PB10
// AF таймера каждого выхода, как у BF (−1 — неизвестно: первый таймер пина).
#define OPB_MOTOR_AF -1, -1, -1, -1, -1, -1, -1, -1

// UART1…UART6: TX, RX и инвертор (SBUS у F4 — через внешний инвертор).
#define OPB_UART1_TX PB6
#define OPB_UART1_RX PB7
#define OPB_UART1_INVERTER -1
#define OPB_UART2_TX PA2
#define OPB_UART2_RX PA3
#define OPB_UART2_INVERTER -1
#define OPB_UART3_TX PC10
#define OPB_UART3_RX PC11
#define OPB_UART3_INVERTER -1
#define OPB_UART4_TX PA0
#define OPB_UART4_RX PA1
#define OPB_UART4_INVERTER -1
#define OPB_UART5_TX PC12
#define OPB_UART5_RX PD2
#define OPB_UART5_INVERTER -1
#define OPB_UART6_TX PC6
#define OPB_UART6_RX PC7
#define OPB_UART6_INVERTER -1

// Что на каком UART (номер, 0 — нет): по `serial` в дампе / *_UART в config.h.
#define OPB_UART_RX 2
#define OPB_UART_GPS 6   // разъём GPS у пятаков RX6/TX6 (подпись на плате); в дампе BF порт не назначен
#define OPB_UART_MSP 0
#define OPB_UART_ESC_SENSOR 0
#define OPB_UART_VTX 0
#define OPB_VTX_PROTOCOL_NONE 1

// SPI1…SPI3: SCK, MISO (SDI), MOSI (SDO).
#define OPB_SPI1_SCK PA5
#define OPB_SPI1_MISO PA6
#define OPB_SPI1_MOSI PA7
#define OPB_SPI2_SCK PB13
#define OPB_SPI2_MISO PB14
#define OPB_SPI2_MOSI PB15
#define OPB_SPI3_SCK PB3
#define OPB_SPI3_MISO PB4
#define OPB_SPI3_MOSI PB5
#define OPB_GYRO_SPI 1
#define OPB_GYRO_CS PA4
#define OPB_GYRO_EXTI -1
#define OPB_GYRO_ALIGN "CW0_DEG"
#define OPB_OSD_SPI 2
#define OPB_OSD_CS PB12
#define OPB_FLASH_SPI 3
#define OPB_FLASH_CS PC13

// I2C1…I2C3: SCL, SDA; барометр и компас — на шине OPB_I2C_SENSORS.
#define OPB_I2C1_SCL PB8
#define OPB_I2C1_SDA PB9
#define OPB_I2C2_SCL -1
#define OPB_I2C2_SDA -1
#define OPB_I2C3_SCL -1
#define OPB_I2C3_SDA -1
#define OPB_I2C_SENSORS 1
#define OPB_BARO_DPS310 1   // тип — как его нашёл Betaflight (status)

// Батарея и ток: масштабы — как vbat_scale, vbat_divider, vbat_multiplier,
// ibata_scale, ibata_offset в BF.
#define OPB_ADC_VBAT PC0
#define OPB_ADC_CURR PC1
#define OPB_ADC_RSSI PC2
#define OPB_VBAT_SCALE 110
#define OPB_VBAT_DIVIDER 10
#define OPB_VBAT_MULTIPLIER 1
#define OPB_IBATA_SCALE 400
#define OPB_IBATA_OFFSET 0

// Пищалка, светодиоды, USB.
#define OPB_BEEPER PB0
#define OPB_BEEPER_INVERTED 0
#define OPB_LED PC15
#define OPB_LED_STRIP PB1
#define OPB_USB_DETECT PC3

// PINIO 1…4: ножка, режим BF, который её включает (40 — USER1…43 — USER4;
// 255 — никакой: ножка всегда в покое), инверсия (бит 0x80 pinio_config).
#define OPB_PINIO_PINS PC4, PC5, -1, -1
#define OPB_PINIO_BOX 40, 41, 255, 255
#define OPB_PINIO_INVERTED 0, 0, 0, 0
