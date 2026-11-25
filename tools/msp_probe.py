#!/usr/bin/env python3
"""Проверка MSP платы так, как её спрашивает конфигуратор Betaflight.

Посылает те же запросы, что BF App 2025.12 / Configurator 10.10 при
подключении (serial_backend.js), раз в 250 мс (строка состояния) и при
открытии основных вкладок, — и печатает таблицу: ответ есть / «!» (команды
нет) / молчит, сколько байт и сколько читает конфигуратор. Только чтение:
ни одной записи, ARM и настройки не трогаются.

Когда пригодится: «конфигуратор BF открывает, но ничего не читает». Запуск
на нашей прошивке и на BF той же платы — разница видна построчно.

    python tools/msp_probe.py COM5
    python tools/msp_probe.py COM5 --log msp.txt   # и сырые кадры в файл

Нужен pyserial: pip install pyserial (или Python из PlatformIO:
~/.platformio/penv/Scripts/python.exe). Studio и конфигуратор порт держат —
закройте их.
"""

import argparse
import sys
import time

# (код, имя, сколько байт читает конфигуратор при API 1.46, нужен для подключения, данные запроса)
CONNECT = [
    (1, "API_VERSION", 3, True, b""),
    (2, "FC_VARIANT", 4, True, b""),
    (3, "FC_VERSION", 3, True, b""),
    (5, "BUILD_INFO", 26, True, b""),
    (4, "BOARD_INFO", 0, True, b""),
    (101, "STATUS", 11, True, b""),
    (160, "UID", 12, True, b""),
    (0x3006, "GET_TEXT build key", 2, False, b"\x05"),
    (0x3006, "GET_TEXT craft name", 2, False, b"\x02"),
    (0x3006, "GET_TEXT pilot name", 2, False, b"\x01"),
]
AFTER = [
    (36, "FEATURE_CONFIG", 4, False, b""),
    (32, "BATTERY_CONFIG", 13, False, b""),
    (80, "BLACKBOX_CONFIG", 11, False, b""),
    (70, "DATAFLASH_SUMMARY", 0, False, b""),
    (79, "SDCARD_SUMMARY", 11, False, b""),
    (110, "ANALOG", 9, False, b""),
    (130, "BATTERY_STATE", 11, False, b""),
    (116, "BOXNAMES", 1, False, b""),
    (150, "STATUS_EX", 24, False, b""),
]
TABS = [
    (240, "ACC_TRIM", 4, False, b""),
    (0x300C, "MCU_INFO", 0, False, b""),
    (42, "MIXER_CONFIG", 2, False, b""),
    (126, "SENSOR_ALIGNMENT", 7, False, b""),
    (108, "ATTITUDE", 6, False, b""),
    (102, "RAW_IMU", 18, False, b""),
    (38, "BOARD_ALIGNMENT_CONFIG", 6, False, b""),
    (112, "PID", 30, False, b""),
    (111, "RC_TUNING", 10, False, b""),
    (94, "PID_ADVANCED", 0, False, b""),
    (92, "FILTER_CONFIG", 0, False, b""),
    (90, "ADVANCED_CONFIG", 0, False, b""),
    (131, "MOTOR_CONFIG", 0, False, b""),
    (124, "MOTOR_3D_CONFIG", 0, False, b""),
    (139, "MOTOR_TELEMETRY", 0, False, b""),
    (104, "MOTOR", 16, False, b""),
    (44, "RX_CONFIG", 0, False, b""),
    (64, "RX_MAP", 0, False, b""),
    (105, "RC", 0, False, b""),
    (125, "RC_DEADBAND", 0, False, b""),
    (50, "RSSI_CONFIG", 0, False, b""),
    (34, "MODE_RANGES", 0, False, b""),
    (75, "FAILSAFE_CONFIG", 0, False, b""),
    (77, "RXFAIL_CONFIG", 0, False, b""),
    (61, "ARMING_CONFIG", 0, False, b""),
    (184, "BEEPER_CONFIG", 0, False, b""),
    (88, "VTX_CONFIG", 0, False, b""),
    (84, "OSD_CONFIG", 0, False, b""),
    (0x1009, "COMMON_SERIAL_CONFIG", 0, False, b""),
    (54, "CF_SERIAL_CONFIG", 0, False, b""),
    (56, "VOLTAGE_METER_CONFIG", 0, False, b""),
    (40, "CURRENT_METER_CONFIG", 0, False, b""),
    (96, "SENSOR_CONFIG", 0, False, b""),
    (106, "RAW_GPS", 0, False, b""),
    (132, "GPS_CONFIG", 0, False, b""),
    (135, "GPS_RESCUE", 0, False, b""),
    (109, "ALTITUDE", 0, False, b""),
]


def crc8_dvb_s2(crc, b):
    crc ^= b
    for _ in range(8):
        crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(cmd, data=b""):
    """Запрос: v1 для кодов до 254, иначе v2 — как у конфигуратора."""
    if cmd <= 254 and len(data) < 255:
        x = len(data) ^ cmd
        for b in data:
            x ^= b
        return bytes([ord("$"), ord("M"), ord("<"), len(data), cmd]) + data + bytes([x])
    body = bytes([0, cmd & 0xFF, cmd >> 8, len(data) & 0xFF, len(data) >> 8]) + data
    crc = 0
    for b in body:
        crc = crc8_dvb_s2(crc, b)
    return b"$X<" + body + bytes([crc])


class FrameReader:
    """Собирает ответы '$M>' '$M!' '$X>' '$X!' из потока; остальное (текст консоли) — в noise."""

    def __init__(self):
        self.buf = bytearray()
        self.noise = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"$")
            if i < 0:
                self.noise += len(self.buf)
                self.buf.clear()
                return out
            if i:
                self.noise += i
                del self.buf[:i]
            if len(self.buf) < 3:
                return out
            kind, direction = self.buf[1:2], self.buf[2:3]
            if kind not in (b"M", b"X") or direction not in (b">", b"!", b"<"):
                self.noise += 1
                del self.buf[:1]
                continue
            frame = self._v1() if kind == b"M" else self._v2()
            if frame is None:
                return out
            if frame is False:
                self.noise += 1
                del self.buf[:1]
                continue
            out.append(frame)

    def _v1(self):
        b = self.buf
        if len(b) < 6:
            return None
        size, cmd, at = b[3], b[4], 5
        if size == 255:   # jumbo (BF): настоящий размер — u16 следом
            if len(b) < 8:
                return None
            size, at = b[5] | b[6] << 8, 7
        if len(b) < at + size + 1:
            return None
        x = b[3] ^ b[4]
        if at == 7:
            x ^= b[5] ^ b[6]
        for v in b[at:at + size]:
            x ^= v
        if x != b[at + size]:
            return False
        f = (cmd, chr(b[2]), bytes(b[at:at + size]))
        del b[:at + size + 1]
        return f

    def _v2(self):
        b = self.buf
        if len(b) < 9:
            return None
        cmd, size = b[4] | b[5] << 8, b[6] | b[7] << 8
        if len(b) < 8 + size + 1:
            return None
        crc = 0
        for v in b[3:8 + size]:
            crc = crc8_dvb_s2(crc, v)
        if crc != b[8 + size]:
            return False
        f = (cmd, chr(b[2]), bytes(b[8:8 + size]))
        del b[:8 + size + 1]
        return f


def text(p, at):
    """Строка с длиной впереди (getText конфигуратора)."""
    n = p[at] if at < len(p) else 0
    return p[at + 1:at + 1 + n].decode("latin-1", "replace"), at + 1 + n


def board_info_need(p):
    """Сколько байт BOARD_INFO читает конфигуратор (с учётом трёх строк)."""
    at = 8
    for _ in range(3):
        if at >= len(p):
            return at + 1
        _, at = text(p, at)
    return at + 32 + 1 + 1 + 2 + 4


def ask(port, reader, cmd, data, timeout, log):
    req = encode(cmd, data)
    port.write(req)
    if log:
        log.write(f"> {req.hex(' ')}\n")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        chunk = port.read(port.in_waiting or 1)
        if not chunk:
            continue
        if log:
            log.write(f"< {chunk.hex(' ')}\n")
        for f in reader.feed(chunk):
            if f[0] == cmd:
                return f[1], f[2], (time.monotonic() - t0) * 1000
    return None, b"", timeout * 1000


def describe(cmd, p):
    try:
        if cmd == 1:
            return f"API {p[1]}.{p[2]}"
        if cmd == 2:
            return p[:4].decode("latin-1")
        if cmd == 3:
            return f"{p[0]}.{p[1]}.{p[2]}"
        if cmd == 5:
            return p[:26].decode("latin-1")
        if cmd == 4:
            target, at = text(p, 8)
            name, _ = text(p, at)
            return f"{p[:4].decode('latin-1')} / {target} / {name}"
        if cmd == 0x3006:
            return repr(text(p, 1)[0])
        if cmd == 108:
            r, pi, y = (int.from_bytes(p[i:i + 2], "little", signed=True) for i in (0, 2, 4))
            return f"крен {r / 10:.1f}°, тангаж {pi / 10:.1f}°, курс {y}°"
        if cmd in (101, 150):
            return f"датчики 0x{p[4] | p[5] << 8:04x}"
    except (IndexError, ValueError):
        return "не разобрать"
    return ""


def run(port, timeout=0.5, log=None, out=sys.stdout):
    reader = FrameReader()
    critical_ok = True
    rows = 0
    for title, group in (("Подключение", CONNECT), ("После подключения и строка состояния", AFTER),
                         ("Вкладки", TABS)):
        print(f"\n== {title}", file=out)
        for cmd, name, need, critical, data in group:
            direction, p, ms = ask(port, reader, cmd, data, timeout, log)
            rows += 1
            if cmd == 4 and direction == ">":
                need = board_info_need(p)
            if direction is None:
                state = "МОЛЧИТ"
            elif direction == "!":
                state = "нет команды (!)"
            elif len(p) < need:
                state = f"КОРОТКО {len(p)} < {need}"
            else:
                state = f"ok {len(p)} Б"
            bad = direction is None or (critical and (direction != ">" or len(p) < need))
            if bad and critical:
                critical_ok = False
            note = describe(cmd, p) if direction == ">" else ""
            mark = "!!" if bad else "  "
            print(f"{mark} {cmd:>5} {name:<24} {state:<18} {ms:6.0f} мс  {note}", file=out)
    print(f"\nПосторонних байт (текст консоли между кадрами): {reader.noise}", file=out)
    print("Конфигуратор BF подключится: " + ("да" if critical_ok else "НЕТ — смотрите строки с !!"), file=out)
    return critical_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("port", help="COM-порт платы, например COM5 или /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.5, help="ждать ответа, с (у конфигуратора — 1 с)")
    ap.add_argument("--log", help="сырые кадры в файл")
    a = ap.parse_args()
    try:
        import serial
    except ImportError:
        sys.exit("нужен pyserial: pip install pyserial")
    with serial.Serial(a.port, a.baud, timeout=0.05) as port:
        port.dtr = True
        time.sleep(0.3)
        port.reset_input_buffer()
        log = open(a.log, "w", encoding="utf-8") if a.log else None
        try:
            ok = run(port, a.timeout, log)
        finally:
            if log:
                log.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
