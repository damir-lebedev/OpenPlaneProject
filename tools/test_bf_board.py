#!/usr/bin/env python3
"""
Тесты tools/bf_board.py — плата из Betaflight (дамп CLI или config.h).

    python tools/test_bf_board.py

Дампы ниже — в формате CLI Betaflight 4.4–4.5 (resource, resource show all,
set, serial), пины — вымышленной платы.
"""

import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bf_board  # noqa: E402

DUMP = """
# version
# Betaflight / STM32F405 (S405) 4.5.1 Nov 18 2024 / 09:06:46 (77d01ba3b) MSP API: 1.46
# config: manufacturer_id: GETF, board_name: GETFUNF405V3, version: 1a2b3c4d, date: 2024-05-01T00:00:00Z

# start the command batch
batch start

board_name GETFUNF405V3
manufacturer_id GETF

# resource
resource BEEPER 1 C13
resource MOTOR 1 B00
resource MOTOR 2 B01
resource MOTOR 3 A03
resource MOTOR 4 A02
resource MOTOR 5 C08
resource MOTOR 6 C09
resource SERIAL_TX 1 A09
resource SERIAL_TX 2 A02
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 2 A03
resource SERIAL_RX 3 B11
resource SERIAL_RX 6 C07
resource INVERTER 3 C00
resource I2C_SCL 1 B08
resource I2C_SDA 1 B09
resource LED 1 C14
resource SPI_SCK 1 A05
resource SPI_SCK 2 B13
resource SPI_SDI 1 A06
resource SPI_SDI 2 B14
resource SPI_SDO 1 A07
resource SPI_SDO 2 B15
resource ADC_BATT 1 C02
resource ADC_CURR 1 C01
resource PINIO 1 C05
resource PINIO 2 B05
resource FLASH_CS 1 A15
resource OSD_CS 1 B12
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 A04

# serial
serial 1 64 115200 57600 0 115200
serial 3 2048 115200 57600 0 115200
serial 5 2 115200 57600 0 115200

# master
set gyro_1_spibus = 1
set gyro_1_sensor_align = CW270
set max7456_spi_bus = 2
set flash_spi_bus = 2
set baro_i2c_device = 1
set mag_i2c_device = 1
set vbat_scale = 110
set ibata_scale = 100
set beeper_inversion = ON
set pinio_box = 40,41,255,255
set pinio_config = 1,129,1,1
set system_hse_mhz = 8
"""

SHOW_ALL = """
# resource show all
Currently active IO resource assignments:
(reboot required to change)
--------------------
A00: SERIAL_TX 4
A01: FREE
A04: GYRO_CS
A05: SPI_SCK 1
A06: SPI_SDI 1
A07: SPI_SDO 1
B00: MOTOR 1
B01: MOTOR 2
B04: MOTOR 3
B05: MOTOR 4
B06: I2C_SCL 1
B07: I2C_SDA 1
C02: ADC_BATT
"""

CONFIG_H = """
#pragma once
#define FC_TARGET_MCU     STM32F405
#define BOARD_NAME        TESTF405
#define MANUFACTURER_ID   TEST
#define MOTOR1_PIN PC6
#define MOTOR2_PIN PC7
#define MOTOR3_PIN PC8
#define MOTOR4_PIN PC9
#define UART2_TX_PIN PA2
#define UART2_RX_PIN PA3
#define SPI1_SCK_PIN PA5
#define SPI1_SDI_PIN PA6
#define SPI1_SDO_PIN PA7
#define GYRO_1_CS_PIN PA4
#define GYRO_1_SPI_INSTANCE SPI1
#define BARO_I2C_INSTANCE I2CDEV_2
#define I2C2_SCL_PIN PB10
#define I2C2_SDA_PIN PB11
#define SERIALRX_UART SERIAL_PORT_USART2
#define DEFAULT_CURRENT_METER_SCALE 179
#define BEEPER_INVERTED
#define PINIO1_PIN PC5
#define PINIO1_BOX 40
#define PINIO1_CONFIG 129
"""


def parsed(text):
    board = bf_board.Board()
    if "#define" in text:
        bf_board.parse_config_h(text, board)
    else:
        bf_board.parse_dump(text, board)
    return board


class BfBoardTest(unittest.TestCase):
    def test_pin_names(self):
        self.assertEqual("PB6", bf_board.pin_name("B06"))
        self.assertEqual("PC13", bf_board.pin_name("PC13"))
        self.assertEqual("PA0", bf_board.pin_name("a00"))
        self.assertIsNone(bf_board.pin_name("NONE"))
        with self.assertRaises(ValueError):
            bf_board.pin_name("X99")

    def test_cli_dump(self):
        b = parsed(DUMP)
        self.assertEqual("GETFUNF405V3", b.name)
        self.assertEqual("STM32F405", b.mcu)
        self.assertEqual(["PB0", "PB1", "PA3", "PA2", "PC8", "PC9"], [b.motors[i] for i in range(1, 7)])
        self.assertEqual("PC6", b.uart_tx[6])
        self.assertEqual("PB11", b.uart_rx[3])
        self.assertEqual("PC0", b.inverter[3])
        self.assertEqual({"RX": 2, "VTX_SMARTAUDIO": 4, "GPS": 6}, b.roles)
        self.assertEqual((1, "PA4", "CW270"), (b.gyro_spi, b.gyro_cs, b.gyro_align))
        self.assertEqual((2, "PB12", 2, "PA15"), (b.osd_spi, b.osd_cs, b.flash_spi, b.flash_cs))
        self.assertEqual((110, 100, True), (b.vbat_scale, b.ibata_scale, b.beeper_inverted))
        self.assertEqual({1: 40, 2: 41, 3: 255, 4: 255}, b.pinio_box)
        self.assertEqual("PB5", b.pinio[2])
        self.assertEqual([], bf_board.check(b))

    def test_resource_show_all_table(self):
        b = parsed(SHOW_ALL)
        self.assertEqual(["PB0", "PB1", "PB4", "PB5"], [b.motors[i] for i in range(1, 5)])
        self.assertEqual("PA0", b.uart_tx[4])
        self.assertEqual("PA4", b.gyro_cs)
        self.assertEqual("PB6", b.i2c[(1, "SCL")])
        self.assertNotIn(1, b.uart_rx)   # FREE — не ресурс

    def test_config_h(self):
        b = parsed(CONFIG_H)
        self.assertEqual("TESTF405", b.name)
        self.assertEqual(["PC6", "PC7", "PC8", "PC9"], [b.motors[i] for i in range(1, 5)])
        self.assertEqual({"RX": 2}, b.roles)
        self.assertEqual((2, 179), (b.baro_i2c, b.ibata_scale))
        self.assertTrue(b.beeper_inverted)
        self.assertEqual({1: 129}, b.pinio_config)

    def test_header(self):
        text = bf_board.generate(parsed(DUMP), "SPL06")
        self.assertIn('#define OPB_BOARD_NAME "GETFUNF405V3"', text)
        self.assertIn("#define OPB_MOTOR_COUNT 6", text)
        self.assertIn("#define OPB_MOTOR_PINS PB0, PB1, PA3, PA2, PC8, PC9", text)
        self.assertIn("#define OPB_UART3_TX -1", text)
        self.assertIn("#define OPB_UART3_INVERTER PC0", text)
        self.assertIn("#define OPB_UART_RX 2", text)
        self.assertIn("#define OPB_UART_GPS 6", text)
        self.assertIn("#define OPB_UART_VTX 4", text)
        self.assertIn("#define OPB_VTX_PROTOCOL_SMARTAUDIO 1", text)
        self.assertIn("#define OPB_SPI3_SCK -1", text)
        self.assertIn("#define OPB_GYRO_CS PA4", text)
        self.assertIn("#define OPB_I2C_SENSORS 1", text)
        self.assertIn("#define OPB_BARO_SPL06 1", text)
        self.assertIn("#define OPB_IBATA_SCALE 100", text)
        self.assertIn("#define OPB_BEEPER_INVERTED 1", text)
        self.assertIn("#define OPB_PINIO_PINS PC5, PB5, -1, -1", text)
        self.assertIn("#define OPB_PINIO_BOX 40, 41, 255, 255", text)
        self.assertIn("#define OPB_PINIO_INVERTED 0, 1, 0, 0", text)

    def test_missing_parts_are_reported(self):
        b = parsed("resource MOTOR 1 B00\n")
        problems = " ".join(bf_board.check(b))
        self.assertIn("моторов меньше четырёх", problems)
        self.assertIn("нет гироскопа", problems)
        self.assertIn("нет имени платы", problems)

    def test_cli_writes_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "dump.txt")
            out = os.path.join(tmp, "board.h")
            with open(src, "w", encoding="utf-8") as f:
                f.write(DUMP)
            self.assertEqual(0, bf_board.main(["header", src, "--baro", "DPS310", "--out", out]))
            with open(out, encoding="utf-8") as f:
                self.assertIn("#define OPB_BARO_DPS310 1", f.read())


if __name__ == "__main__":
    unittest.main()
