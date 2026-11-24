#pragma once

// ============================================================
// Плата SPEEDYBEEF405V4 — из Betaflight (config.h), tools/bf_board.py.
// Источник: github.com/betaflight/config, configs/SPBE/SPEEDYBEEF405V4/config.h
// Только факты о плате: ножки, номера шин, масштабы датчиков. Заново —
//   python tools/bf_board.py header <дамп или config.h> --out <этот файл>
// Номера пинов — макросы STM32duino (PA0…PE15), −1 — нет.
// ============================================================

#define OPB_BOARD_NAME "SPEEDYBEEF405V4"
#define OPB_HSE_MHZ 8

// Выходы M1…M8 — номера Betaflight (у квадро 1 — задний правый).
#define OPB_MOTOR_COUNT 8
#define OPB_MOTOR_PINS PB6, PB7, PB0, PB1, PC8, PC9, PB10, PA15
// AF таймера каждого выхода, как у BF (−1 — неизвестно: первый таймер пина).
#define OPB_MOTOR_AF 2, 2, 2, 2, 3, 3, 1, 1

// UART1…UART6: TX, RX и инвертор (SBUS у F4 — через внешний инвертор).
#define OPB_UART1_TX PA9
#define OPB_UART1_RX PA10
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
#define OPB_UART5_TX -1
#define OPB_UART5_RX PD2
#define OPB_UART5_INVERTER -1
#define OPB_UART6_TX PC6
#define OPB_UART6_RX PC7
#define OPB_UART6_INVERTER -1

// Что на каком UART (номер, 0 — нет): по `serial` в дампе / *_UART в config.h.
#define OPB_UART_RX 2
#define OPB_UART_GPS 0
#define OPB_UART_MSP 4
#define OPB_UART_ESC_SENSOR 5
#define OPB_UART_VTX 0
#define OPB_VTX_PROTOCOL_NONE 1

// SPI1…SPI3: SCK, MISO (SDI), MOSI (SDO).
#define OPB_SPI1_SCK PA5
#define OPB_SPI1_MISO PA6
#define OPB_SPI1_MOSI PA7
#define OPB_SPI2_SCK PB13
#define OPB_SPI2_MISO PC2
#define OPB_SPI2_MOSI PC3
#define OPB_SPI3_SCK PB3
#define OPB_SPI3_MISO PB4
#define OPB_SPI3_MOSI PB5
#define OPB_GYRO_SPI 1
#define OPB_GYRO_CS PA4
#define OPB_GYRO_EXTI PC4
#define OPB_GYRO_ALIGN "CW90_DEG"
#define OPB_OSD_SPI 2
#define OPB_OSD_CS PB12
#define OPB_FLASH_SPI 0
#define OPB_FLASH_CS -1
// Слот microSD на SPI (sdcard_mode = SPI): чёрный ящик — файл BLACKBOX.BIN на карте.
#define OPB_SDCARD_SPI 3
#define OPB_SDCARD_CS PC14
// Гироскоп по BF (какой из них стоит — опрос при старте); драйвер есть для: ICM42688P.
#define OPB_GYRO_TYPES "ICM42688P"
#define OPB_GYRO_ICM42688P 1

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
#define OPB_ADC_RSSI PC5
#define OPB_VBAT_SCALE 110
#define OPB_VBAT_DIVIDER 10
#define OPB_VBAT_MULTIPLIER 1
#define OPB_IBATA_SCALE 400
#define OPB_IBATA_OFFSET 0

// Пищалка, светодиоды, USB.
#define OPB_BEEPER PC15
#define OPB_BEEPER_INVERTED 1
#define OPB_LED PC13
#define OPB_LED_STRIP PA8
#define OPB_USB_DETECT -1

// PINIO 1…4: ножка, режим BF, который её включает (40 — USER1…43 — USER4;
// 255 — никакой: ножка всегда в покое), инверсия (бит 0x80 pinio_config).
#define OPB_PINIO_PINS PB11, -1, -1, -1
#define OPB_PINIO_BOX 0, 255, 255, 255
#define OPB_PINIO_INVERTED 1, 0, 0, 0
