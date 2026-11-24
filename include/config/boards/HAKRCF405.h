#pragma once

// ============================================================
// Плата HAKRCF405 — из Betaflight (config.h), tools/bf_board.py.
// Источник: github.com/betaflight/config, configs/HARC/HAKRCF405/config.h
// Только факты о плате: ножки, номера шин, масштабы датчиков. Заново —
//   python tools/bf_board.py header <дамп или config.h> --out <этот файл>
// Номера пинов — макросы STM32duino (PA0…PE15), −1 — нет.
// ============================================================

#define OPB_BOARD_NAME "HAKRCF405"
#define OPB_HSE_MHZ 8

// Выходы M1…M6 — номера Betaflight (у квадро 1 — задний правый).
#define OPB_MOTOR_COUNT 6
#define OPB_MOTOR_PINS PB0, PC6, PA10, PA8, PC7, PC9
// AF таймера каждого выхода, как у BF (−1 — неизвестно: первый таймер пина).
#define OPB_MOTOR_AF 2, 3, 1, 1, 2, 3

// UART1…UART6: TX, RX и инвертор (SBUS у F4 — через внешний инвертор).
#define OPB_UART1_TX PA9
#define OPB_UART1_RX PB7
#define OPB_UART1_INVERTER PB4
#define OPB_UART2_TX PA2
#define OPB_UART2_RX PA3
#define OPB_UART2_INVERTER -1
#define OPB_UART3_TX PB10
#define OPB_UART3_RX PB11
#define OPB_UART3_INVERTER -1
#define OPB_UART4_TX PA0
#define OPB_UART4_RX PA1
#define OPB_UART4_INVERTER -1
#define OPB_UART5_TX PC12
#define OPB_UART5_RX PD2
#define OPB_UART5_INVERTER -1
#define OPB_UART6_TX -1
#define OPB_UART6_RX -1
#define OPB_UART6_INVERTER -1

// Что на каком UART (номер, 0 — нет): по `serial` в дампе / *_UART в config.h.
#define OPB_UART_RX 1
#define OPB_UART_GPS 0
#define OPB_UART_MSP 0
#define OPB_UART_ESC_SENSOR 0
#define OPB_UART_VTX 0
#define OPB_VTX_PROTOCOL_NONE 1

// SPI1…SPI3: SCK, MISO (SDI), MOSI (SDO).
#define OPB_SPI1_SCK PB3
#define OPB_SPI1_MISO PA6
#define OPB_SPI1_MOSI PA7
#define OPB_SPI2_SCK PB13
#define OPB_SPI2_MISO PB14
#define OPB_SPI2_MOSI PC3
#define OPB_SPI3_SCK PC10
#define OPB_SPI3_MISO PC11
#define OPB_SPI3_MOSI PB5
#define OPB_GYRO_SPI 1
#define OPB_GYRO_CS PA4
#define OPB_GYRO_EXTI PC4
#define OPB_GYRO_ALIGN "CW270_DEG"
#define OPB_OSD_SPI 3
#define OPB_OSD_CS PA15
#define OPB_FLASH_SPI 2
#define OPB_FLASH_CS PB12
// Гироскоп по BF (какой из них стоит — опрос при старте); драйвер есть для: ICM42688P, MPU6000, ICM20689, MPU6500.
#define OPB_GYRO_TYPES "MPU6000/ICM20689"
#define OPB_GYRO_MPU6000 1
#define OPB_GYRO_ICM20689 1

// I2C1…I2C3: SCL, SDA; барометр и компас — на шине OPB_I2C_SENSORS.
#define OPB_I2C1_SCL PB8
#define OPB_I2C1_SDA PB9
#define OPB_I2C2_SCL -1
#define OPB_I2C2_SDA -1
#define OPB_I2C3_SCL -1
#define OPB_I2C3_SDA -1
#define OPB_I2C_SENSORS 1
#define OPB_BARO_SPL06 1   // тип — на плате барометра нет (config.h BF): ищется внешний на площадках I2C

// Батарея и ток: масштабы — как vbat_scale, vbat_divider, vbat_multiplier,
// ibata_scale, ibata_offset в BF.
#define OPB_ADC_VBAT PC1
#define OPB_ADC_CURR PC2
#define OPB_ADC_RSSI PC0
#define OPB_VBAT_SCALE 110
#define OPB_VBAT_DIVIDER 10
#define OPB_VBAT_MULTIPLIER 1
#define OPB_IBATA_SCALE 400
#define OPB_IBATA_OFFSET 0

// Пищалка, светодиоды, USB.
#define OPB_BEEPER PC14
#define OPB_BEEPER_INVERTED 1
#define OPB_LED PC13
#define OPB_LED_STRIP PB6
#define OPB_USB_DETECT -1

// PINIO 1…4: ножка, режим BF, который её включает (40 — USER1…43 — USER4;
// 255 — никакой: ножка всегда в покое), инверсия (бит 0x80 pinio_config).
#define OPB_PINIO_PINS -1, -1, -1, -1
#define OPB_PINIO_BOX 255, 255, 255, 255
#define OPB_PINIO_INVERTED 0, 0, 0, 0
