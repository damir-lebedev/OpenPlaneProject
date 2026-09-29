#!/usr/bin/env python3
"""Чёрный ящик OpenPlane: список полётов, выгрузка по USB, разбор в CSV.

    python tools/blackbox.py list                     # полёты на плате
    python tools/blackbox.py download                 # последний полёт -> blackbox/
    python tools/blackbox.py download --all           # все полёты
    python tools/blackbox.py download --flight 3
    python tools/blackbox.py decode blackbox/flight_003_....bbl

Порт находится сам (мост CH343/CH340/CP210x), иначе --port COM10.
Монитор порта (pio device monitor) перед выгрузкой нужно закрыть.

Выгрузка идёт на 2 Мбод (--baud), плата переключается сама и
возвращает 115200 после. Каждый сектор — с CRC-32; битый — ошибка.
После выгрузки файл сразу разбирается (--no-decode — не надо).

Разбор: CSV на каждый тип записи (IMU, CTRL, RC, BARO, MAG, GPS, AIR,
NAV, SYS) — время в секундах от старта записи (предзапись — в
минусе), значения уже в единицах (градусы, g, м, м/с, мкс);
events.txt — параметры полёта и все события; сводка — в консоль.
Схема полей читается из самого лога (формат — BlackBoxFormat.h).

Нужен pyserial (для list/download): pip install pyserial — или Python
из PlatformIO (~/.platformio/penv/Scripts/python), там он уже есть.
Для decode — только стандартная библиотека.
"""
import argparse
import csv
import datetime
import math
import os
import struct
import sys
import time
import zlib

SECTOR = 4096
HEADER = struct.Struct("<IIIHBB")   # magic, seq, start_ms, flight, version, check
MAGIC = 0x4242504F
FORMAT_VERSION = 2          # BlackBoxFormat::VERSION
FRAME = 4 + SECTOR + 4
STX = b"\x02"
TEXT_TYPES = {0x01: "SCHEMA", 0x02: "INFO", 0x03: "EVENT", 0x04: "END"}


# ----------------------------------------------------------------
# Разбор
# ----------------------------------------------------------------

def header_ok(raw):
    s = 0
    for b in raw[:15]:
        s = (s + b) & 0xFF
    return (~s & 0xFF) == raw[15]


def _crc8_byte(crc):
    for _ in range(8):
        crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


CRC8_TABLE = [_crc8_byte(i) for i in range(256)]


def crc8(data):
    """CRC-8/SMBUS (полином 0x07) — как BlackBoxFormat::crc8()."""
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


def parse_sectors(data):
    """Записи по порядку: (тип, байты данных). Сектор без заголовка —
    пропуск; запись с несошедшимся CRC-8 (питание пропало, пока она
    писалась) — конец сектора."""
    records = []
    bad = torn = 0
    for at in range(0, len(data) - SECTOR + 1, SECTOR):
        sector = data[at:at + SECTOR]
        magic, _seq, _ms, _flight, version, _check = HEADER.unpack_from(sector)
        if magic != MAGIC or not header_ok(sector[:16]) or version != FORMAT_VERSION:
            bad += 1
            continue
        pos = 16
        while pos + 2 <= SECTOR:
            kind, length = sector[pos], sector[pos + 1]
            end = pos + 2 + length
            if kind == 0xFF or end + 1 > SECTOR:
                break
            if crc8(sector[pos:end]) != sector[end]:
                torn += 1
                break
            records.append((kind, bytes(sector[pos + 2:end])))
            pos = end + 1
    return records, bad, torn


class Schema:
    def __init__(self, name):
        self.name = name
        self.fields = []   # (имя, символ struct, делитель)

    def add(self, tokens):
        for token in tokens:
            name, _, spec = token.partition(":")
            fmt, _, div = spec.partition("/")
            self.fields.append((name, fmt, float(div) if div else 1.0))

    def compile(self):
        fixed = [f for f in self.fields if f[1] != "s"]
        self.struct = struct.Struct("<" + "".join(f[1] for f in fixed))
        self.text = any(f[1] == "s" for f in self.fields)

    def decode(self, payload):
        values = list(self.struct.unpack_from(payload))
        if self.text:
            values.append(payload[self.struct.size:].decode("utf-8", "replace"))
        return values


def decode_records(records):
    """Схема из SCHEMA, затем все записи -> {имя: [(t_us, [значения])]}, info, события."""
    schemas = {}
    info = {}
    events = []
    rows = {}
    unknown = 0

    for kind, payload in records:
        if kind == 0x01:
            # "<id> <имя> <поля...>" или продолжение "<id> + <поля...>"
            text = payload[4:].decode("utf-8", "replace").split()
            if len(text) < 2 or not text[0].isdigit():
                continue
            sid = int(text[0])
            if text[1] == "+" and sid in schemas:
                schemas[sid].add(text[2:])
            else:
                schemas[sid] = Schema(text[1])
                schemas[sid].add(text[2:])
            schemas[sid].compile()
        elif kind in (0x02, 0x03, 0x04):
            t_us = struct.unpack_from("<I", payload)[0]
            text = payload[4:].decode("utf-8", "replace")
            if kind == 0x02:
                key, _, value = text.partition("=")
                if key == "bind":
                    info.setdefault("bind", []).append(value)
                else:
                    info[key] = value
            else:
                events.append((t_us, TEXT_TYPES[kind], text))
        elif kind in schemas:
            schema = schemas[kind]
            if len(payload) < schema.struct.size:
                unknown += 1
                continue
            rows.setdefault(schema.name, (schema, []))[1].append(schema.decode(payload))
        else:
            unknown += 1
    return schemas, info, events, rows, unknown


class Unwrap:
    """micros() — u32, переполняется раз в 71.6 мин."""

    def __init__(self):
        self.last = None
        self.offset = 0

    def __call__(self, t):
        if self.last is not None and t + self.offset < self.last - (1 << 31):
            self.offset += 1 << 32
        self.last = t + self.offset
        return self.last


def fmt_value(v):
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return ""
        return f"{v:.6g}"
    return str(v)


def decode_file(path, out_dir=None, quiet=False):
    with open(path, "rb") as f:
        data = f.read()
    records, bad, torn = parse_sectors(data)
    schemas, info, events, rows, unknown = decode_records(records)

    # Ноль времени — момент старта записи (событие "запись: старт").
    unwrap_events = Unwrap()
    events = [(unwrap_events(t), kind, text) for t, kind, text in events]
    zero = next((t for t, _, text in events if text.startswith("запись: старт")), None)
    if zero is None:
        first = [r[1][0][0] for r in rows.values() if r[1]]
        zero = min(first) if first else 0

    if out_dir is None:
        out_dir = os.path.splitext(path)[0]
    os.makedirs(out_dir, exist_ok=True)

    flag_names = info.get("bits.flags", "").split()
    feature_names = info.get("bits.features", "").split()
    mode_names = info.get("names.mode", "").split()

    stats = {}
    for name, (schema, items) in rows.items():
        unwrap = Unwrap()
        columns = [f[0] for f in schema.fields]
        scales = [f[2] for f in schema.fields]
        extra = []
        if name == "CTRL":
            extra = ["mode_name", "features_on"] + flag_names
        with open(os.path.join(out_dir, name + ".csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s"] + columns[1:] + extra)
            for values in items:
                t = unwrap(values[0])
                out = [f"{(t - zero) / 1e6:.6f}"]
                for v, scale in zip(values[1:], scales[1:]):
                    out.append(fmt_value(v / scale if scale != 1.0 and not isinstance(v, str) else v))
                if name == "CTRL":
                    rec = dict(zip(columns, values))
                    mode = rec.get("mode", 0)
                    out.append(mode_names[mode] if mode < len(mode_names) else str(mode))
                    feats = rec.get("features", 0)
                    out.append(" ".join(n for i, n in enumerate(feature_names) if feats >> i & 1))
                    flags = rec.get("flags", 0)
                    out.extend(str(flags >> i & 1) for i in range(len(flag_names)))
                w.writerow(out)
        stats[name] = (schema, items)

    with open(os.path.join(out_dir, "events.txt"), "w", encoding="utf-8") as f:
        f.write(f"Файл: {os.path.basename(path)}\n\nПараметры полёта:\n")
        for key, value in info.items():
            if key == "bind":
                for line in value:
                    f.write(f"  bind: {line}\n")
            else:
                f.write(f"  {key} = {value}\n")
        f.write("\nСобытия (время от старта записи, с):\n")
        for t, kind, text in events:
            f.write(f"  {(t - zero) / 1e6:9.3f}  {'КОНЕЦ: ' if kind == 'END' else ''}{text}\n")

    summary = summarize(info, events, stats, zero, bad, unknown, torn)
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(summary)
    if not quiet:
        print(summary)
        print(f"CSV и события: {out_dir}")
    return out_dir


def column(stats, name, field):
    if name not in stats:
        return []
    schema, items = stats[name]
    names = [f[0] for f in schema.fields]
    if field not in names:
        return []
    i = names.index(field)
    scale = schema.fields[i][2]
    return [row[i] / scale for row in items]


def summarize(info, events, stats, zero, bad, unknown, torn):
    lines = []
    add = lines.append
    add(f"Полёт #{info.get('flight', '?')}  |  прошивка {info.get('firmware', '?')}  |  {info.get('board', '?')}")
    add(f"Старт записи: {info.get('start', '?')}  |  перезагрузка: {info.get('reset_reason', '?')}")
    end = next((text for _, kind, text in events if kind == "END"), None)
    add(f"Конец записи: {end or 'нет (питание пропало или запись шла, когда выгружали)'}")

    t_all = []
    for name, (schema, items) in stats.items():
        if items:
            t_all += [items[0][0], items[-1][0]]
    if "IMU" in stats and stats["IMU"][1]:
        items = stats["IMU"][1]
        unwrap = Unwrap()
        ts = [unwrap(r[0]) for r in items]
        span = (ts[-1] - ts[0]) / 1e6
        add(f"Длительность: {span:.1f} с (предзапись {max(0.0, (zero - ts[0]) / 1e6):.1f} с), "
            f"IMU {len(items) / span:.0f} Гц" if span > 0 else "Длительность: —")
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        if gaps:
            add(f"Такт IMU: средний {sum(gaps) / len(gaps) / 1000:.2f} мс, самый долгий {max(gaps) / 1000:.1f} мс")
        work = column(stats, "IMU", "work_us")
        if work:
            add(f"Работа цикла: средняя {sum(work) / len(work):.0f} мкс, максимум {max(work):.0f} мкс")

    counts = ", ".join(f"{n} {len(stats[n][1])}" for n in sorted(stats))
    add(f"Записей: {counts}")
    if bad or unknown:
        add(f"ВНИМАНИЕ: секторов без заголовка {bad}, непонятных записей {unknown}")
    if torn:
        add(f"Недописанных записей (CRC): {torn} — обычно последние перед пропажей питания")

    def rng(name, field, unit, fmt="{:.1f}"):
        v = column(stats, name, field)
        if v:
            add(f"  {field}: {fmt.format(min(v))} … {fmt.format(max(v))} {unit}")

    add("Диапазоны:")
    rng("CTRL", "roll", "°")
    rng("CTRL", "pitch", "°")
    rng("IMU", "gx", "°/с")
    rng("IMU", "gy", "°/с")
    rng("IMU", "gz", "°/с")
    rng("IMU", "az", "g", "{:.2f}")
    rng("BARO", "alt_m", "м")
    rng("BARO", "vz_ms", "м/с")
    rng("GPS", "speed_ms", "м/с")
    rng("AIR", "ias_ms", "м/с")
    rng("CTRL", "esc", "мкс", "{:.0f}")
    rng("POWER", "vbat_v", "В", "{:.2f}")
    rng("POWER", "current_sensor_v", "В (выход датчика тока)", "{:.2f}")
    sys_max = column(stats, "SYS", "loop_max_us")
    if sys_max:
        add(f"Цикл: худший такт {max(sys_max):.0f} мкс; "
            f"запись флеша до {max(column(stats, 'SYS', 'flash_max_us')):.0f} мкс; "
            f"потеряно записей {max(column(stats, 'SYS', 'dropped')):.0f}")
    bad_frames = column(stats, "SYS", "ibus_bad")
    if bad_frames:
        add(f"iBUS: битых кадров за запись {bad_frames[-1] - bad_frames[0]:.0f}")

    add("События:")
    for t, kind, text in events[:60]:
        add(f"  {(t - zero) / 1e6:9.3f} с  {'КОНЕЦ: ' if kind == 'END' else ''}{text}")
    if len(events) > 60:
        add(f"  ... ещё {len(events) - 60} (events.txt)")
    return "\n".join(lines) + "\n"


# ----------------------------------------------------------------
# Связь с платой
# ----------------------------------------------------------------

def open_port(port):
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        sys.exit("Нужен pyserial: pip install pyserial (или Python из PlatformIO)")

    if not port:
        bridges = [p for p in serial.tools.list_ports.comports()
                   if p.vid in (0x1A86, 0x10C4, 0x0403, 0x303A)]
        if len(bridges) != 1:
            names = ", ".join(p.device for p in serial.tools.list_ports.comports()) or "нет"
            sys.exit(f"Укажите порт: --port (найдено: {names})")
        port = bridges[0].device

    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.2
    s.dtr = False   # без сброса платы (как monitor_dtr/rts = 0)
    s.rts = False
    try:
        s.open()
    except serial.SerialException as e:
        sys.exit(f"{port}: {e}\nЗакройте монитор порта (pio device monitor) и попробуйте снова.")
    time.sleep(0.1)
    s.reset_input_buffer()
    return s


def command(s, line):
    s.write(STX + line.encode() + b"\n")


def read_reply(s, prefixes, timeout=3.0):
    """Строки BB:..., пока не встретится одна из prefixes. Лог платы пропускается."""
    end = time.time() + timeout
    buf = b""
    got = []
    while time.time() < end:
        buf += s.read(4096)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            text = raw.decode("utf-8", "replace")
            at = text.find("BB:")   # после смены скорости в начале строки бывает мусор
            if at < 0:
                continue
            text = text[at:].strip()
            got.append(text)
            if text.startswith("BB:ERR"):
                raise RuntimeError(text[7:])
            if any(text.startswith(p) for p in prefixes):
                return got
    raise RuntimeError("плата не ответила (прошивка без чёрного ящика? монитор порта открыт?)")


def fields(line):
    out = {}
    for token in line.split()[1:]:
        key, _, value = token.partition("=")
        out[key] = value
    return out


def list_flights(s):
    command(s, "bb list")
    lines = read_reply(s, ["BB:END"])
    state = next((fields(l) for l in lines if l.startswith("BB:STATE")), {})
    flights = [fields(l) for l in lines if l.startswith("BB:FLIGHT")]
    return state, flights


def download(s, number, baud, out_dir):
    command(s, f"bb get {number} {baud}")
    reply = read_reply(s, ["BB:SEND"])[-1]
    sectors = int(fields(reply)["sectors"])

    s.baudrate = baud
    time.sleep(0.05)
    s.reset_input_buffer()
    s.write(b"G")

    image = bytearray()
    buf = bytearray()
    started = time.time()
    last_data = time.time()
    k = 0
    while k < sectors:
        # Ровно сколько осталось: иначе последний read ждёт таймаут и
        # глотает BB:DONE, который плата шлёт уже на 115200.
        left = (sectors - k) * FRAME - len(buf)
        chunk = s.read(max(1, min(65536, left)))
        if chunk:
            buf += chunk
            last_data = time.time()
        elif time.time() - last_data > 3:
            raise RuntimeError(f"поток оборвался на секторе {k} из {sectors}")
        while len(buf) >= FRAME and k < sectors:
            at = buf.find(b"\xA5\x5A")
            if at < 0:
                del buf[:-1]
                break
            if at:
                del buf[:at]
            if len(buf) < FRAME:
                break
            index = buf[2] | buf[3] << 8
            data = bytes(buf[4:4 + SECTOR])
            crc = struct.unpack_from("<I", buf, 4 + SECTOR)[0]
            if index != k or zlib.crc32(data) != crc:
                raise RuntimeError(f"сектор {k}: битые данные (скорость {baud} не держится? попробуйте --baud 921600)")
            image += data
            del buf[:FRAME]
            k += 1
        rate = len(image) / max(time.time() - started, 1e-3) / 1024
        print(f"\r  полёт #{number}: {k}/{sectors} секторов, {rate:.0f} КБ/с", end="", flush=True)
    print()

    s.baudrate = 115200
    try:
        done = read_reply(s, ["BB:DONE"], timeout=2.0)[-1]
        crc = int(fields(done)["crc"], 16)
        if crc != zlib.crc32(bytes(image)):
            raise RuntimeError("общая CRC не совпала")
    except RuntimeError as e:
        if "CRC" in str(e):
            raise
        print("  (подтверждение BB:DONE не пришло — сектора проверены по CRC)")

    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(out_dir, f"flight_{number:03d}_{stamp}.bbl")
    with open(path, "wb") as f:
        f.write(image)
    return path


def main():
    # Кириллица в выводе: при перенаправлении в файл Windows иначе берёт cp1252.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list", help="полёты на плате")
    p.add_argument("--port")

    p = sub.add_parser("download", help="выгрузить полёты по USB")
    p.add_argument("--port")
    p.add_argument("--flight", type=int, help="номер полёта (по умолчанию — последний)")
    p.add_argument("--all", action="store_true", help="все полёты")
    p.add_argument("--baud", type=int, default=2000000)
    p.add_argument("-o", "--out", default="blackbox", help="папка (по умолчанию ./blackbox)")
    p.add_argument("--no-decode", action="store_true")

    p = sub.add_parser("decode", help="разобрать .bbl в CSV")
    p.add_argument("file")
    p.add_argument("-o", "--out", help="папка (по умолчанию — рядом с файлом)")

    args = ap.parse_args()

    if args.cmd == "decode":
        decode_file(args.file, args.out)
        return

    s = open_port(args.port)
    try:
        state, flights = list_flights(s)
        if args.cmd == "list":
            free = int(state.get("free_kb", 0))
            rate = int(state.get("rate_bps", 0))
            minutes = f" (≈{free * 1024 // rate // 60} мин)" if rate else ""
            print(f"Состояние: {state.get('state')}, стёрто впереди {free / 1024:.1f} МБ{minutes} "
                  f"из {int(state.get('total_kb', 0)) / 1024:.1f} МБ")
            if not flights:
                print("Полётов нет.")
            for f in flights:
                sec = int(f["seconds"])
                note = "" if f.get("start") == "1" else "  (начало стёрто)"
                print(f"  #{int(f['n']):<4} {int(f['kb']):>6} КБ  {sec // 60:2d}:{sec % 60:02d}{note}")
            return

        if not flights:
            sys.exit("На плате нет полётов.")
        numbers = [int(f["n"]) for f in flights]
        if args.all:
            wanted = numbers
        elif args.flight is not None:
            if args.flight not in numbers:
                sys.exit(f"Полёта #{args.flight} нет (есть: {', '.join(map(str, numbers))})")
            wanted = [args.flight]
        else:
            wanted = [numbers[-1]]

        for n in wanted:
            path = download(s, n, args.baud, args.out)
            print(f"  -> {path}")
            if not args.no_decode:
                decode_file(path)
    except RuntimeError as e:
        sys.exit(f"Ошибка: {e}")
    finally:
        s.close()


if __name__ == "__main__":
    main()
