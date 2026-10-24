#!/usr/bin/env python3
"""
Плата из Betaflight — для полётников, у которых распиновки нет нигде, кроме
самой платы (GETFUN F405V3 и другие таргеты, которых нет в репозиториях BF).

    python tools/bf_board.py read --port COM5 --out stack/GETFUNF405V3_bf.txt
        с платы, на которой ещё стоит Betaflight: CLI — version, status,
        resource, dump hardware, diff all — в текстовый файл (pyserial: Python
        из PlatformIO, ~/.platformio/penv/Scripts/python.exe)
    python tools/bf_board.py backup --out stack/GETFUNF405V3_original.bin
        родная прошивка целиком (плата в DFU: зажать BOOT, вставить USB) —
        чтобы вернуть Betaflight, если что: dfu-util -D, адрес 0x08000000
    python tools/bf_board.py header DUMP_OR_CONFIG_H [--name ИМЯ] [--baro SPL06]
        [--out include/config/boards/ИМЯ.h] [--note "откуда дамп"]
        заголовок платы для прошивки: что на каких ножках

Источник — дамп CLI (строки resource / set / serial) или config.h из
github.com/betaflight/config (#define MOTOR1_PIN PC6 …). Из них берутся
только факты о плате: ножки, номера шин, масштабы датчиков; код Betaflight
не копируется.

Чего Betaflight не говорит: тип барометра (у него нет драйвера SPL06 —
такие платы собирают с DPS310) — флаг --baro по описанию платы.
"""

import argparse
import os
import re
import subprocess
import sys
import time

# Номера функций порта в `serial <id> <функции> …` (битовая маска BF).
SERIAL_FUNCTIONS = {
    "MSP": 1,
    "GPS": 2,
    "RX": 64,
    "ESC_SENSOR": 1024,
    "VTX_SMARTAUDIO": 2048,
    "VTX_TRAMP": 8192,
    "VTX_MSP": 131072,
}

BAROS = ("SPL06", "DPS310", "BMP280", "NONE")
UARTS = 6
SPIS = 3
I2CS = 3


class Board:
    """Что известно о плате (ключи — как в заголовке, без префикса OPB_)."""

    def __init__(self):
        self.name = None
        self.mcu = None
        self.hse_mhz = 8
        self.motors = {}          # номер BF -> пин
        self.uart_tx = {}
        self.uart_rx = {}
        self.inverter = {}        # UART -> пин инвертора
        self.spi = {}             # (шина, "SCK"|"SDI"|"SDO") -> пин
        self.i2c = {}             # (шина, "SCL"|"SDA") -> пин
        self.gyro_spi = None
        self.gyro_cs = None
        self.gyro_exti = None
        self.gyro_align = None
        self.osd_spi = None
        self.osd_cs = None
        self.flash_spi = None
        self.flash_cs = None
        self.baro_i2c = None
        self.mag_i2c = None
        self.adc_vbat = None
        self.adc_curr = None
        self.adc_rssi = None
        self.vbat_scale = 110       # BF: делитель ×10
        self.vbat_divider = 10      # BF: vbat_divider
        self.vbat_multiplier = 1    # BF: vbat_multiplier
        self.ibata_scale = 400      # BF: 0.1 мВ на ампер
        self.ibata_offset = 0       # BF: мВ
        self.beeper = None
        self.beeper_inverted = False
        self.led = None
        self.led_strip = None
        self.usb_detect = None
        self.pinio = {}             # номер -> пин
        self.pinio_box = {}         # номер -> режим BF (40 = USER1…)
        self.pinio_config = {}      # номер -> config BF (0x80 — инверсия)
        self.roles = {}             # "RX"/"GPS"/… -> номер UART (1…)
        self.sources = []


# ------------------------------------------------------------
# Пины
# ------------------------------------------------------------

def pin_name(text):
    """B06 / PB6 / PB06 -> 'PB6'; NONE -> None."""
    t = text.strip().upper()
    if t in ("NONE", "NONE,", ""):
        return None
    m = re.fullmatch(r"P?([A-I])0*(\d{1,2})", t)
    if not m:
        raise ValueError("не пин: " + text)
    return "P{}{}".format(m.group(1), int(m.group(2)))


def number_from(text, prefix):
    """SPI2 -> 2, I2CDEV_1 -> 1, SERIAL_PORT_USART2 / UART5 -> 2 / 5."""
    m = re.search(prefix + r"(\d+)", text)
    return int(m.group(1)) if m else None


def uart_number(text):
    m = re.search(r"U(?:S)?ART(\d)", text)
    return int(m.group(1)) if m else None


# ------------------------------------------------------------
# config.h из github.com/betaflight/config
# ------------------------------------------------------------

def parse_config_h(text, board):
    defines = {}
    for line in text.splitlines():
        m = re.match(r"\s*#define\s+(\w+)\s*(.*?)\s*(?://.*)?$", line)
        if m:
            defines[m.group(1)] = m.group(2).strip()
    board.sources.append("config.h")
    board.name = defines.get("BOARD_NAME", board.name)
    board.mcu = defines.get("FC_TARGET_MCU", board.mcu)
    if "SYSTEM_HSE_MHZ" in defines:
        board.hse_mhz = int(defines["SYSTEM_HSE_MHZ"])
    for key, value in defines.items():
        m = re.fullmatch(r"MOTOR(\d+)_PIN", key)
        if m:
            board.motors[int(m.group(1))] = pin_name(value)
        m = re.fullmatch(r"UART(\d)_(TX|RX)_PIN", key)
        if m:
            (board.uart_tx if m.group(2) == "TX" else board.uart_rx)[int(m.group(1))] = pin_name(value)
        m = re.fullmatch(r"INVERTER_PIN_UART(\d)", key)
        if m:
            board.inverter[int(m.group(1))] = pin_name(value)
        m = re.fullmatch(r"SPI(\d)_(SCK|SDI|SDO|MISO|MOSI)_PIN", key)
        if m:
            role = {"MISO": "SDI", "MOSI": "SDO"}.get(m.group(2), m.group(2))
            board.spi[(int(m.group(1)), role)] = pin_name(value)
        m = re.fullmatch(r"I2C(\d)_(SCL|SDA)_PIN", key)
        if m:
            board.i2c[(int(m.group(1)), m.group(2))] = pin_name(value)
        m = re.fullmatch(r"PINIO(\d)_(PIN|BOX|CONFIG)", key)
        if m:
            n = int(m.group(1))
            if m.group(2) == "PIN":
                board.pinio[n] = pin_name(value)
            elif m.group(2) == "BOX":
                board.pinio_box[n] = int(value, 0)
            else:
                board.pinio_config[n] = int(value, 0)
    simple = {
        "GYRO_1_CS_PIN": "gyro_cs", "GYRO_1_EXTI_PIN": "gyro_exti", "MAX7456_SPI_CS_PIN": "osd_cs",
        "FLASH_CS_PIN": "flash_cs", "ADC_VBAT_PIN": "adc_vbat", "ADC_CURR_PIN": "adc_curr",
        "ADC_RSSI_PIN": "adc_rssi", "BEEPER_PIN": "beeper", "LED0_PIN": "led", "LED_STRIP_PIN": "led_strip",
        "USB_DETECT_PIN": "usb_detect",
    }
    for key, attr in simple.items():
        if key in defines:
            setattr(board, attr, pin_name(defines[key]))
    if "GYRO_1_SPI_INSTANCE" in defines:
        board.gyro_spi = number_from(defines["GYRO_1_SPI_INSTANCE"], "SPI")
    if "GYRO_1_ALIGN" in defines:
        board.gyro_align = defines["GYRO_1_ALIGN"]
    if "MAX7456_SPI_INSTANCE" in defines:
        board.osd_spi = number_from(defines["MAX7456_SPI_INSTANCE"], "SPI")
    if "FLASH_SPI_INSTANCE" in defines:
        board.flash_spi = number_from(defines["FLASH_SPI_INSTANCE"], "SPI")
    if "BARO_I2C_INSTANCE" in defines:
        board.baro_i2c = number_from(defines["BARO_I2C_INSTANCE"], "I2CDEV_")
    if "MAG_I2C_INSTANCE" in defines:
        board.mag_i2c = number_from(defines["MAG_I2C_INSTANCE"], "I2CDEV_")
    if "DEFAULT_VOLTAGE_METER_SCALE" in defines:
        board.vbat_scale = int(defines["DEFAULT_VOLTAGE_METER_SCALE"])
    if "DEFAULT_CURRENT_METER_SCALE" in defines:
        board.ibata_scale = int(defines["DEFAULT_CURRENT_METER_SCALE"])
    if "DEFAULT_CURRENT_METER_OFFSET" in defines:
        board.ibata_offset = int(defines["DEFAULT_CURRENT_METER_OFFSET"])
    if "BEEPER_INVERTED" in defines:
        board.beeper_inverted = True
    for role, key in (("RX", "SERIALRX_UART"), ("GPS", "GPS_UART"), ("MSP", "MSP_UART"),
                      ("ESC_SENSOR", "ESC_SENSOR_UART"), ("VTX_SMARTAUDIO", "VTX_SMARTAUDIO_UART"),
                      ("VTX_TRAMP", "VTX_TRAMP_UART")):
        if key in defines and uart_number(defines[key]):
            board.roles[role] = uart_number(defines[key])


# ------------------------------------------------------------
# Дамп CLI (resource, dump hardware, diff all, resource show all)
# ------------------------------------------------------------

RESOURCE_KEYS = {
    "GYRO_CS": "gyro_cs", "GYRO_EXTI": "gyro_exti", "OSD_CS": "osd_cs", "FLASH_CS": "flash_cs",
    "ADC_BATT": "adc_vbat", "ADC_CURR": "adc_curr", "ADC_RSSI": "adc_rssi", "BEEPER": "beeper",
    "LED_STRIP": "led_strip", "USB_DETECT": "usb_detect",
}


def apply_resource(board, name, index, pin):
    name = name.upper()
    if name == "MOTOR":
        board.motors[index] = pin
    elif name == "SERIAL_TX":
        board.uart_tx[index] = pin
    elif name == "SERIAL_RX":
        board.uart_rx[index] = pin
    elif name == "INVERTER":
        board.inverter[index] = pin
    elif name in ("SPI_SCK", "SPI_SDI", "SPI_SDO", "SPI_MISO", "SPI_MOSI"):
        role = {"SPI_MISO": "SDI", "SPI_MOSI": "SDO"}.get(name, name[4:])
        board.spi[(index, role)] = pin
    elif name in ("I2C_SCL", "I2C_SDA"):
        board.i2c[(index, name[4:])] = pin
    elif name == "PINIO":
        board.pinio[index] = pin
    elif name == "LED":
        if index == 1:
            board.led = pin
    elif name in RESOURCE_KEYS and index == 1:
        setattr(board, RESOURCE_KEYS[name], pin)


def list_values(text):
    return [int(v) for v in re.findall(r"-?\d+", text)]


def parse_dump(text, board):
    board.sources.append("CLI")
    for raw in text.splitlines():
        line = raw.strip()
        m = re.match(r"#\s*Betaflight\s*/\s*(\S+)\s*\((\w+)\)", line)
        if m:
            board.mcu = board.mcu or m.group(1)
        m = re.match(r"board_name\s+(\S+)", line)
        if m:
            board.name = m.group(1)
        # resource MOTOR 1 B06
        m = re.match(r"resource\s+(\w+)\s+(\d+)\s+(\S+)", line, re.I)
        if m and m.group(1).upper() != "SHOW":
            apply_resource(board, m.group(1), int(m.group(2)), pin_name(m.group(3)))
            continue
        # resource show all: "B06: MOTOR 1" (или "B06: FREE")
        m = re.match(r"([A-I])(\d{2}):\s*([A-Z][A-Z0-9_]*)(?:\s+(\d+))?", line)
        if m and m.group(3) != "FREE":
            apply_resource(board, m.group(3), int(m.group(4) or 1), pin_name(m.group(1) + m.group(2)))
            continue
        # serial 1 64 115200 57600 0 115200 — порт 1 = UART2 (0 — UART1, 20 — USB)
        m = re.match(r"serial\s+(\d+)\s+(\d+)\s", line)
        if m:
            port, functions = int(m.group(1)), int(m.group(2))
            if 0 <= port < UARTS:
                for role, bit in SERIAL_FUNCTIONS.items():
                    if functions & bit:
                        board.roles[role] = port + 1
            continue
        m = re.match(r"set\s+(\w+)\s*=\s*(.*)$", line)
        if not m:
            continue
        key, value = m.group(1), m.group(2).strip()
        if key == "gyro_1_spibus":
            board.gyro_spi = int(value)
        elif key == "gyro_1_sensor_align":
            board.gyro_align = value
        elif key == "max7456_spi_bus":
            board.osd_spi = int(value)
        elif key == "flash_spi_bus":
            board.flash_spi = int(value)
        elif key == "baro_i2c_device":
            board.baro_i2c = int(value)
        elif key == "mag_i2c_device":
            board.mag_i2c = int(value)
        elif key == "vbat_scale":
            board.vbat_scale = int(value)
        elif key == "vbat_divider":
            board.vbat_divider = int(value)
        elif key == "vbat_multiplier":
            board.vbat_multiplier = int(value)
        elif key == "ibata_scale":
            board.ibata_scale = int(value)
        elif key == "ibata_offset":
            board.ibata_offset = int(value)
        elif key == "beeper_inversion":
            board.beeper_inverted = value.upper() == "ON"
        elif key == "system_hse_mhz":
            board.hse_mhz = int(value)
        elif key == "pinio_box":
            for i, v in enumerate(list_values(value), start=1):
                board.pinio_box[i] = v
        elif key == "pinio_config":
            for i, v in enumerate(list_values(value), start=1):
                board.pinio_config[i] = v


def load(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    board = Board()
    if re.search(r"^\s*#define\s+\w+_PIN\b", text, re.M):
        parse_config_h(text, board)
    else:
        parse_dump(text, board)
    return board


# ------------------------------------------------------------
# Заголовок
# ------------------------------------------------------------

def check(board):
    """Чего не хватает, чтобы на плате летать (список строк по-русски)."""
    problems = []
    if not board.name:
        problems.append("нет имени платы (board_name / BOARD_NAME) — задайте --name")
    if board.mcu and "F405" not in board.mcu.upper():
        problems.append("процессор {} — прошивка пока умеет F405 и H743".format(board.mcu))
    if len([m for m in board.motors.values() if m]) < 4:
        problems.append("моторов меньше четырёх")
    if not board.gyro_cs or not board.gyro_spi:
        problems.append("нет гироскопа на SPI (GYRO_CS и шина)")
    elif not all(board.spi.get((board.gyro_spi, r)) for r in ("SCK", "SDI", "SDO")):
        problems.append("нет ножек SPI{} гироскопа".format(board.gyro_spi))
    if board.roles.get("RX") is None:
        problems.append("не задан UART приёмника (serial … 64 …) — по умолчанию UART2")
    return problems


def c_pin(pin):
    return pin if pin else "-1"


def generate(board, baro="SPL06", source_note=""):
    name = board.name or "BOARD"
    motors = [board.motors.get(i) for i in range(1, 9)]
    while motors and not motors[-1]:
        motors.pop()
    roles = dict(board.roles)
    roles.setdefault("RX", 2)
    vtx = roles.get("VTX_SMARTAUDIO") or roles.get("VTX_TRAMP") or 0
    vtx_kind = "SMARTAUDIO" if roles.get("VTX_SMARTAUDIO") else "TRAMP" if roles.get("VTX_TRAMP") else "NONE"
    i2c_bus = board.baro_i2c or board.mag_i2c or next(iter(sorted({b for b, _ in board.i2c})), 0)

    out = []
    w = out.append
    w("#pragma once")
    w("")
    w("// ============================================================")
    w("// Плата {} — из Betaflight ({}), tools/bf_board.py.".format(name, ", ".join(board.sources) or "?"))
    if source_note:
        w("// {}".format(source_note))
    w("// Только факты о плате: ножки, номера шин, масштабы датчиков. Заново —")
    w("//   python tools/bf_board.py header <дамп или config.h> --out <этот файл>")
    w("// Номера пинов — макросы STM32duino (PA0…PE15), −1 — нет.")
    w("// ============================================================")
    w("")
    w('#define OPB_BOARD_NAME "{}"'.format(name))
    w("#define OPB_HSE_MHZ {}".format(board.hse_mhz))
    w("")
    w("// Выходы M1…M{} — номера Betaflight (у квадро 1 — задний правый).".format(len(motors)))
    w("#define OPB_MOTOR_COUNT {}".format(len(motors)))
    w("#define OPB_MOTOR_PINS {}".format(", ".join(c_pin(p) for p in motors) if motors else "-1"))
    w("")
    w("// UART1…UART6: TX, RX и инвертор (SBUS у F4 — через внешний инвертор).")
    for n in range(1, UARTS + 1):
        w("#define OPB_UART{0}_TX {1}".format(n, c_pin(board.uart_tx.get(n))))
        w("#define OPB_UART{0}_RX {1}".format(n, c_pin(board.uart_rx.get(n))))
        w("#define OPB_UART{0}_INVERTER {1}".format(n, c_pin(board.inverter.get(n))))
    w("")
    w("// Что на каком UART (номер, 0 — нет): по `serial` в дампе / *_UART в config.h.")
    w("#define OPB_UART_RX {}".format(roles.get("RX", 0)))
    w("#define OPB_UART_GPS {}".format(roles.get("GPS", 0)))
    w("#define OPB_UART_MSP {}".format(roles.get("MSP", 0)))
    w("#define OPB_UART_ESC_SENSOR {}".format(roles.get("ESC_SENSOR", 0)))
    w("#define OPB_UART_VTX {}".format(vtx))
    w("#define OPB_VTX_PROTOCOL_{} 1".format(vtx_kind))
    w("")
    w("// SPI1…SPI3: SCK, MISO (SDI), MOSI (SDO).")
    for n in range(1, SPIS + 1):
        w("#define OPB_SPI{0}_SCK {1}".format(n, c_pin(board.spi.get((n, "SCK")))))
        w("#define OPB_SPI{0}_MISO {1}".format(n, c_pin(board.spi.get((n, "SDI")))))
        w("#define OPB_SPI{0}_MOSI {1}".format(n, c_pin(board.spi.get((n, "SDO")))))
    w("#define OPB_GYRO_SPI {}".format(board.gyro_spi or 0))
    w("#define OPB_GYRO_CS {}".format(c_pin(board.gyro_cs)))
    w("#define OPB_GYRO_EXTI {}".format(c_pin(board.gyro_exti)))
    w('#define OPB_GYRO_ALIGN "{}"'.format(board.gyro_align or "CW0_DEG"))
    w("#define OPB_OSD_SPI {}".format(board.osd_spi or 0))
    w("#define OPB_OSD_CS {}".format(c_pin(board.osd_cs)))
    w("#define OPB_FLASH_SPI {}".format(board.flash_spi or 0))
    w("#define OPB_FLASH_CS {}".format(c_pin(board.flash_cs)))
    w("")
    w("// I2C1…I2C3: SCL, SDA; барометр и компас — на шине OPB_I2C_SENSORS.")
    for n in range(1, I2CS + 1):
        w("#define OPB_I2C{0}_SCL {1}".format(n, c_pin(board.i2c.get((n, "SCL")))))
        w("#define OPB_I2C{0}_SDA {1}".format(n, c_pin(board.i2c.get((n, "SDA")))))
    w("#define OPB_I2C_SENSORS {}".format(i2c_bus))
    w("#define OPB_BARO_{} 1   // тип — по описанию платы (у BF драйвера SPL06 нет)".format(baro))
    w("")
    w("// Батарея и ток: масштабы — как vbat_scale, vbat_divider, vbat_multiplier,")
    w("// ibata_scale, ibata_offset в BF.")
    w("#define OPB_ADC_VBAT {}".format(c_pin(board.adc_vbat)))
    w("#define OPB_ADC_CURR {}".format(c_pin(board.adc_curr)))
    w("#define OPB_ADC_RSSI {}".format(c_pin(board.adc_rssi)))
    w("#define OPB_VBAT_SCALE {}".format(board.vbat_scale))
    w("#define OPB_VBAT_DIVIDER {}".format(board.vbat_divider))
    w("#define OPB_VBAT_MULTIPLIER {}".format(board.vbat_multiplier))
    w("#define OPB_IBATA_SCALE {}".format(board.ibata_scale))
    w("#define OPB_IBATA_OFFSET {}".format(board.ibata_offset))
    w("")
    w("// Пищалка, светодиоды, USB.")
    w("#define OPB_BEEPER {}".format(c_pin(board.beeper)))
    w("#define OPB_BEEPER_INVERTED {}".format(1 if board.beeper_inverted else 0))
    w("#define OPB_LED {}".format(c_pin(board.led)))
    w("#define OPB_LED_STRIP {}".format(c_pin(board.led_strip)))
    w("#define OPB_USB_DETECT {}".format(c_pin(board.usb_detect)))
    w("")
    w("// PINIO 1…4: ножка, режим BF, который её включает (40 — USER1…43 — USER4;")
    w("// 255 — никакой: ножка всегда в покое), инверсия (бит 0x80 pinio_config).")
    pins = [board.pinio.get(i) for i in range(1, 5)]
    boxes = [board.pinio_box.get(i, 255) for i in range(1, 5)]
    inverted = [1 if board.pinio_config.get(i, 1) & 0x80 else 0 for i in range(1, 5)]
    w("#define OPB_PINIO_PINS {}".format(", ".join(c_pin(p) for p in pins)))
    w("#define OPB_PINIO_BOX {}".format(", ".join(str(b) for b in boxes)))
    w("#define OPB_PINIO_INVERTED {}".format(", ".join(str(v) for v in inverted)))
    w("")
    return "\n".join(out)


# ------------------------------------------------------------
# С платы: дамп CLI и родная прошивка
# ------------------------------------------------------------

CLI_COMMANDS = ("version", "status", "resource", "resource show all", "dump hardware", "diff all")


def read_cli(port, out_path, timeout=8.0):
    try:
        import serial  # pyserial — есть в Python PlatformIO
    except ImportError:
        sys.exit("нужен pyserial: запустите Python из PlatformIO (~/.platformio/penv/Scripts/python.exe)")
    with serial.Serial(port, 115200, timeout=0.2) as s:
        s.reset_input_buffer()
        s.write(b"#")
        time.sleep(0.5)
        text = s.read(4096).decode("utf-8", "replace")
        if "CLI" not in text and "#" not in text:
            sys.exit("плата не отвечает как Betaflight CLI (тот ли порт? Betaflight Configurator закрыт?)")
        dump = []
        for cmd in CLI_COMMANDS:
            s.write((cmd + "\r\n").encode())
            deadline = time.time() + timeout
            chunk = ""
            while time.time() < deadline:
                part = s.read(4096).decode("utf-8", "replace")
                if part:
                    chunk += part
                    deadline = time.time() + 1.0   # пока идут данные — ждём
                elif chunk.rstrip().endswith("#"):
                    break
            dump.append("### " + cmd + "\n" + chunk)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(dump))
    return out_path


def dfu_util():
    base = os.path.join(os.path.expanduser("~"), ".platformio", "packages", "tool-dfuutil", "bin")
    for name in ("dfu-util.exe", "dfu-util"):
        path = os.path.join(base, name)
        if os.path.exists(path):
            return path
    return "dfu-util"


def backup(out_path, size=1024 * 1024):
    if os.path.exists(out_path):
        sys.exit("{} уже есть — не перезаписываю родную прошивку".format(out_path))
    cmd = [dfu_util(), "-a", "0", "-s", "0x08000000:{}".format(size), "-U", out_path]
    print(" ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0 or not os.path.exists(out_path):
        sys.exit("не вышло: плата в DFU? (зажать BOOT и вставить USB; в Windows — драйвер WinUSB)")
    print("Сохранено {} ({} байт). Вернуть Betaflight: {} -a 0 -s 0x08000000:leave -D {}".format(
        out_path, os.path.getsize(out_path), cmd[0], out_path))


def main(argv=None):
    # консоль Windows (cp1252) русских букв не печатает
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description="Плата из Betaflight для прошивки OpenPlane")
    sub = parser.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("read", help="дамп CLI Betaflight с платы в файл")
    r.add_argument("--port", required=True)
    r.add_argument("--out", required=True)
    b = sub.add_parser("backup", help="родная прошивка целиком по DFU")
    b.add_argument("--out", required=True)
    h = sub.add_parser("header", help="заголовок платы из дампа или config.h")
    h.add_argument("source")
    h.add_argument("--name")
    h.add_argument("--baro", choices=BAROS, default="SPL06")
    h.add_argument("--out")
    h.add_argument("--note", help="откуда дамп (строка в заголовке)")
    args = parser.parse_args(argv)

    if args.cmd == "read":
        print("Сохранено:", read_cli(args.port, args.out))
        return 0
    if args.cmd == "backup":
        backup(args.out)
        return 0
    board = load(args.source)
    if args.name:
        board.name = args.name
    for problem in check(board):
        print("ВНИМАНИЕ:", problem, file=sys.stderr)
    text = generate(board, args.baro, "Источник: " + (args.note or os.path.basename(args.source)))
    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("Записано", args.out)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
