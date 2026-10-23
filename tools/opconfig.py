#!/usr/bin/env python3
"""
OpenPlane: описание и настройка прошивки для приложения OpenPlane Studio.

    python tools/opconfig.py describe [--root DIR] [--out FILE]
        Всё, что приложение знает о прошивке, одним JSON: платы
        (platformio.ini + блоки пинов в Config.h), параметры Config.h с
        описаниями из комментариев, каналы пульта, режимы, функции и
        крутилки — и что каждому нужно (метки @needs/@better в
        AutopilotTypes.h), датчики и готовые наборы, привязки тумблеров.

    python tools/opconfig.py apply --config FILE [--root DIR]
        Записать конфигурацию самолёта (JSON от приложения) в исходники DIR:
        Config.h, Channels.h, Controls.h, SensorSelection.h и config/Project.h
        (паспорт прошивки). Приложение делает это в своей копии исходников,
        не в репозитории. Значение, которое не изменилось, не трогается —
        текст строки остаётся как был.

    python tools/opconfig.py check [--root DIR]
        Метки на месте (@needs у каждого режима, функции и крутилки, @board у
        каждой платы) и apply с текущими значениями не меняет ни байта.
        Код выхода 1 — есть ошибки.

Метки в комментариях исходников:
    @needs(imu, baro)   без этого не работает      @better(gps)  с этим лучше
    @fixed              протокол, не настраивается  @channel      значение — канал пульта
    @unit(м)            единицы                     @range(10, 500) разумные пределы
    @board(blackbox=sd, telemetry=mavlink)          что умеет плата (блок пинов)

Только стандартная библиотека Python 3.8+.
"""

import argparse
import ast
import hashlib
import json
import math
import operator
import os
import re
import sys

SCHEMA_VERSION = 1

FILES = {
    "config": "include/config/Config.h",
    "channels": "include/config/Channels.h",
    "controls": "include/config/Controls.h",
    "sensors": "include/sensors/SensorSelection.h",
    "autopilot": "include/autopilot/AutopilotTypes.h",
    "platformio": "platformio.ini",
}
PROJECT_HEADER = "include/config/Project.h"

# Что может стоять в @needs/@better и как это назвать человеку.
NEED_TOKENS = {
    "imu": "IMU",
    "baro": "барометр",
    "mag": "компас",
    "gps": "GPS",
    "pitot": "трубка Пито",
    "wheels": "шасси с колёсами",
    "rudder": "руль направления",
    "flaperons": "элероны на двух сервах",
    "aux1": "серво на AUX1",
    "aux2": "серво на AUX2",
    "buzzer": "пищалка",
    "esc3d": "двунаправленный ESC",
}
SENSOR_CATEGORIES = ("imu", "baro", "mag", "airspeed", "gps")
BOARD_TAG_KEYS = {"blackbox": ("flash", "sd", "none"), "telemetry": ("wifi", "mavlink", "none")}

INT_LIMITS = {
    "uint8_t": (0, 0xFF),
    "uint16_t": (0, 0xFFFF),
    "uint32_t": (0, 0xFFFFFFFF),
    "int8_t": (-0x80, 0x7F),
    "int16_t": (-0x8000, 0x7FFF),
    "int32_t": (-0x80000000, 0x7FFFFFFF),
    "int": (-0x80000000, 0x7FFFFFFF),
}
STM32_PIN = re.compile(r"P[A-K](?:[0-9]|1[0-5])")


class ConfigError(Exception):
    """Ошибка конфигурации: что не так — по-русски, для приложения."""


# ------------------------------------------------------------
# Файлы: читаем и пишем с теми же переводами строк, что были
# ------------------------------------------------------------

class TextFile:
    def __init__(self, root, rel):
        self.rel = rel
        self.path = os.path.join(root, rel)
        with open(self.path, encoding="utf-8", newline="") as f:
            raw = f.read()
        self.eol = "\r\n" if "\r\n" in raw else "\n"
        self.original = raw.replace("\r\n", "\n")
        self.lines = self.original.split("\n")

    def text(self):
        return "\n".join(self.lines)

    def changed(self):
        return self.text() != self.original

    def save(self):
        with open(self.path, "w", encoding="utf-8", newline="") as f:
            f.write(self.text().replace("\n", self.eol))


def comment_text(line):
    """'    // текст' -> 'текст'; не комментарий -> None."""
    m = re.match(r"\s*//\s?(.*)$", line)
    return m.group(1).rstrip() if m else None


ANNOTATION = re.compile(r"@(\w+)(?:\(([^)]*)\))?")


def split_annotations(text):
    """'текст  @needs(imu, baro) @fixed' -> ('текст', {'needs': ['imu', 'baro'], 'fixed': True})."""
    tags = {}
    for m in ANNOTATION.finditer(text):
        key, args = m.group(1), m.group(2)
        tags[key] = True if args is None else [a.strip() for a in args.split(",") if a.strip()]
    clean = ANNOTATION.sub("", text)
    clean = re.sub(r"[ \t]{2,}", " ", clean).strip()
    if tags:    # «текст — @fixed»: без метки тире повисло; тире переноса строки в описании — часть текста
        clean = re.sub(r"\s+—\s*$", "", clean)
    return clean, tags


def tag_dict(values):
    """['blackbox=sd', 'telemetry=wifi'] -> {'blackbox': 'sd', 'telemetry': 'wifi'}."""
    out = {}
    for item in values or []:
        if "=" in item:
            k, v = item.split("=", 1)
            out[k.strip()] = v.strip()
    return out


# ------------------------------------------------------------
# Значения C++: литерал -> Python и обратно
# ------------------------------------------------------------

_BINOPS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul, ast.Div: None}


def _eval_node(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)) and not isinstance(node.value, bool):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        v = _eval_node(node.operand)
        return -v if isinstance(node.op, ast.USub) else v
    if isinstance(node, ast.BinOp) and type(node.op) in _BINOPS:
        a, b = _eval_node(node.left), _eval_node(node.right)
        if isinstance(node.op, ast.Div):
            if isinstance(a, int) and isinstance(b, int):
                return int(a / b)   # деление целых в C
            return a / b
        return _BINOPS[type(node.op)](a, b)
    raise ValueError("не арифметика")


def evaluate(expr):
    """Текст инициализатора -> (значение, вид): bool / number / expr / string / ident / raw."""
    e = expr.strip()
    if e in ("true", "false"):
        return e == "true", "bool"
    if len(e) >= 2 and e[0] == '"' and e[-1] == '"':
        try:
            return ast.literal_eval(e), "string"
        except (ValueError, SyntaxError):
            return e[1:-1], "string"
    if re.fullmatch(r"-?0[xX][0-9a-fA-F]+[uUlL]*", e):
        return int(e.rstrip("uUlL"), 16), "number"
    if re.fullmatch(r"[A-Za-z_]\w*(?:::\w+)?", e):
        return e, "ident"
    py = re.sub(r"(?<=[0-9.])(?:[fF]|[uU][lL]{0,2}|[lL]{1,2}[uU]?)\b", "", e)
    try:
        value = _eval_node(ast.parse(py, mode="eval").body)
    except (ValueError, SyntaxError, ZeroDivisionError, TypeError):
        return e, "raw"
    literal = re.fullmatch(r"-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?", py)
    return value, ("number" if literal else "expr")


def c_string(text):
    if any(ord(ch) < 0x20 for ch in text):
        raise ConfigError("в строке есть управляющие символы")
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def same_value(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same_value(x, y) for x, y in zip(a, b))
    return a == b


# ------------------------------------------------------------
# platformio.ini: окружения прошивки
# ------------------------------------------------------------

def parse_ini(text):
    sections, current, key = {}, None, None
    for raw in text.split("\n"):
        line = raw.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith((";", "#")):
            continue
        m = re.match(r"\[(.+)\]$", stripped)
        if m:
            current, key = m.group(1), None
            sections[current] = {}
            continue
        if current is None:
            continue
        if raw[:1] in (" ", "\t") and key:
            sections[current][key] += "\n" + stripped
            continue
        m = re.match(r"([\w.\-]+)\s*=\s*(.*)$", stripped)
        if m:
            key = m.group(1)
            sections[current][key] = m.group(2)
    return sections


def parse_platformio(text):
    sections = parse_ini(text)

    def resolve(name, seen=()):
        sec = dict(sections.get(name, {}))
        parent = sec.pop("extends", None)
        if parent and parent not in seen:
            base = resolve(parent.strip(), seen + (name,))
            base.update(sec)
            sec = base
        return sec

    titles = {}
    for m in re.finditer(r"^;\s+pio run -e (\S+)\s+\((.+?)\)\s*$", text, re.M):
        titles[m.group(1)] = m.group(2)
    default_envs = [e.strip() for e in sections.get("platformio", {}).get("default_envs", "").split(",") if e.strip()]

    envs = []
    for name in sections:
        if not name.startswith("env:"):
            continue
        env = name[4:]
        sec = resolve(name)
        if sec.get("platform", "").strip() == "native":
            continue
        flags = sec.get("build_flags", "")
        # ${env:stm32h743.build_flags} — флаги родителя; у родителя могут
        # быть свои ссылки на деда — раскрываются, пока есть новые
        expanded = set()
        while True:
            refs = [r for r in re.findall(r"\$\{([^}]+)\.build_flags\}", flags) if r not in expanded]
            if not refs:
                break
            for ref in refs:
                expanded.add(ref)
                flags += "\n" + resolve(ref if ref.startswith("env:") else ref).get("build_flags", "")
        # Мультикоптер (docs/COPTER.md) настраивается в CopterConfig.h — приложению
        # его сборки пока не показываются.
        if re.search(r"-D\s*VEHICLE_MULTIROTOR\b", flags):
            continue
        macro = next(iter(re.findall(r"-D\s*(BOARD_[A-Z0-9_]+)", flags)), None)
        # Полётник с датчиками на плате задаёт их набор сам (-D SENSOR_KIT=…).
        kit = next(iter(re.findall(r"-D\s*SENSOR_KIT=(SENSOR_KIT_\w+)", flags)), None)
        envs.append({
            "env": env,
            "title": titles.get(env, env),
            "macro": macro,
            "kit": kit,
            "board": sec.get("board", "").strip() or None,
            "platform": sec.get("platform", "").strip() or None,
            "uploadProtocol": sec.get("upload_protocol", "").strip() or None,
            "default": env in default_envs,
        })
    return envs


# ------------------------------------------------------------
# Объявления constexpr (Config.h, Channels.h)
# ------------------------------------------------------------

DECL = re.compile(
    r"^(?P<indent>\s*)constexpr\s+(?P<type>(?:const\s+)?[\w:]+(?:\s*\*)?)\s*"
    r"(?P<name>[A-Z][A-Z0-9_]*)\s*=\s*(?P<expr>[^;]*?)\s*;(?P<trail>.*)$")
ARRAY_DECL = re.compile(
    r"^\s*constexpr\s+(?P<type>[\w:]+)\s+(?P<name>[A-Z][A-Z0-9_]*)\s*\[(?P<size>[^\]]*)\]\s*=\s*\{\s*(?://.*)?$")
ARRAY_ITEM = re.compile(r"^(?P<lead>\s*)(?P<value>true|false|-?[\d.]+[fFuUlL]*)\s*,?(?P<trail>.*)$")
ENUM_CLASS = re.compile(r"enum\s+class\s+(\w+)\s*:\s*\w+\s*\{([^}]*)\}")
DASHES = re.compile(r"^\s*//\s*-{20,}\s*$")


def section_title(line):
    title = re.sub(r"\([^()]*\)", "", line)
    title = title.split(" — ")[0]
    if ": " in title and "::" in title:
        title = title.split(": ")[0]
    title = re.sub(r"\s+:", ":", title)
    return re.sub(r"\s{2,}", " ", title).strip(" :")


def parse_declarations(tf):
    """Все constexpr-параметры файла: имя, тип, значение, где стоит текст значения, описание."""
    params, enums, board_tags = [], {}, {}
    board_stack = []          # [(macro или None)] для #if defined(BOARD_X) ... #endif
    group, group_doc = None, []
    header_state = None       # None / "title" / "doc"
    paragraph_doc = []        # комментарий в начале текущего абзаца
    paragraph_has_decl = False
    i = 0
    lines = tf.lines
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        m = re.match(r"#if\s+defined\((BOARD_\w+)\)", stripped)
        if m:
            board_stack.append(m.group(1))
        elif stripped.startswith("#if"):
            board_stack.append(None)
        elif re.match(r"#elif\s+defined\((BOARD_\w+)\)", stripped):
            board_stack[-1] = re.match(r"#elif\s+defined\((BOARD_\w+)\)", stripped).group(1)
        elif stripped.startswith("#el"):
            board_stack[-1] = None
        elif stripped.startswith("#endif") and board_stack:
            board_stack.pop()
        board = next((b for b in reversed(board_stack) if b), None)

        if DASHES.match(line):
            if board is None:   # внутри блока пинов рамка — просто примечание, не раздел
                header_state = "title" if header_state is None else None
            paragraph_doc, paragraph_has_decl = [], False
            i += 1
            continue
        if header_state is not None:
            text = comment_text(line)
            if text is not None:
                if header_state == "title":
                    group, group_doc, header_state = section_title(text), [], "doc"
                elif text or group_doc:
                    group_doc.append(text)
            i += 1
            continue

        if not stripped:
            paragraph_doc, paragraph_has_decl = [], False
            i += 1
            continue

        text = comment_text(line)
        if text is not None:
            if paragraph_has_decl:              # комментарий после объявлений — новый абзац
                paragraph_doc, paragraph_has_decl = [], False
            clean, tags = split_annotations(text)
            if "board" in tags and board:
                board_tags[board] = tag_dict(tags["board"])
            elif clean or paragraph_doc:
                paragraph_doc.append(clean)
            i += 1
            continue

        m = ENUM_CLASS.search(line)
        if m:
            enums[m.group(1)] = [v.strip() for v in m.group(2).split(",") if v.strip()]
            i += 1
            continue

        m = ARRAY_DECL.match(line)
        if m:
            items, j = [], i + 1
            while j < len(lines) and not lines[j].strip().startswith("};"):
                im = ARRAY_ITEM.match(lines[j])
                if im:
                    value, _ = evaluate(im.group("value"))
                    start = len(im.group("lead"))
                    items.append({"line": j, "start": start, "end": start + len(im.group("value")),
                                  "value": value, "note": split_annotations(comment_text(im.group("trail")) or "")[0]})
                j += 1
            params.append({
                "name": m.group("name"), "type": m.group("type"), "array": True, "size": m.group("size"),
                "items": items, "value": [it["value"] for it in items], "expr": None, "valueKind": "array",
                "board": board, "group": group, "groupDoc": "\n".join(group_doc).strip(),
                "doc": "\n".join(paragraph_doc).strip(), "note": "", "tags": {}, "line": i,
            })
            paragraph_has_decl = True
            i = j + 1
            continue

        m = DECL.match(line)
        if m:
            note, tags = split_annotations(comment_text(m.group("trail")) or "")
            value, kind = evaluate(m.group("expr"))
            params.append({
                "name": m.group("name"), "type": re.sub(r"\s+", " ", m.group("type")).replace(" *", "*"),
                "array": False, "expr": m.group("expr"), "value": value, "valueKind": kind,
                "board": board, "group": group, "groupDoc": "\n".join(group_doc).strip(),
                "doc": "\n".join(paragraph_doc).strip(), "note": note, "tags": tags,
                "line": i, "start": m.start("expr"), "end": m.end("expr"),
            })
            paragraph_has_decl = True
            # Примечание перенесено на следующие строки — «//» под тем же столбцом: продолжение.
            col = line.find("//", m.start("trail"))
            while col >= 0 and i + 1 < len(lines) and lines[i + 1].find("//") == col and not lines[i + 1][:col].strip():
                more, more_tags = split_annotations(comment_text(lines[i + 1]))
                params[-1]["note"] = (params[-1]["note"] + " " + more).strip()
                params[-1]["tags"].update(more_tags)
                i += 1
        i += 1
    return params, enums, board_tags


def infer_unit(name, ctype):
    rules = [("_DEG", "°"), ("_DPS", "°/с"), ("_US", "мкс"), ("_PCT", "%"), ("_PA", "Па"), ("_HZ", "Гц"),
             ("_BYTES", "байт"), ("_G", "g"), ("_M", "м"), ("_S", "с")]
    special = {"_US_PER_DEG": "мкс/°", "_US_PER_DPS": "мкс/(°/с)"}
    for suffix, unit in special.items():
        if name.endswith(suffix):
            return unit
    if "_PER_" in name:
        return None
    if name.endswith("_MS"):
        return "м/с" if ctype == "float" else "мс"
    for suffix, unit in rules:
        if name.endswith(suffix):
            return unit
    return None


def classify(param, enums):
    t, tags = param["type"], param["tags"]
    if param["array"]:
        return "array"
    if tags.get("channel"):
        return "channel"
    if t == "bool":
        return "bool"
    if t == "float":
        return "float"
    if t in enums:
        return "enum"
    if t == "const char*":
        return "string"
    if t in INT_LIMITS:
        return "pin" if param["name"].startswith("PIN_") else "int"
    return "other"


# ------------------------------------------------------------
# AutopilotTypes.h: режимы, функции, крутилки
# ------------------------------------------------------------

def parse_enum_entries(lines, header):
    start = next(i for i, l in enumerate(lines) if re.match(header, l.strip()))
    entries = []
    for line in lines[start + 1:]:
        if line.strip().startswith("};"):
            break
        m = re.match(r"\s*(?P<id>[A-Z][A-Z0-9_]*)\s*(?:=\s*(?P<value>\d+))?\s*,?\s*(?://\s?(?P<doc>.*))?$", line)
        if not m or m.group("id") in ("COUNT", "MODE_COUNT"):
            continue
        doc, tags = split_annotations(m.group("doc") or "")
        entries.append({"id": m.group("id"), "value": int(m.group("value")) if m.group("value") else len(entries),
                        "doc": doc, "tags": tags})
    return entries


def parse_name_switch(text, function):
    m = re.search(r"inline const char\* " + function + r"\(.*?\{(.*?)\n    \}", text, re.S)
    return dict(re.findall(r"case\s+(?:\w+::)?(\w+):\s*return\s+\"([^\"]*)\"", m.group(1))) if m else {}


def parse_autopilot(tf):
    text = tf.text()
    out = {}
    for key, header in (("modes", r"enum AutopilotMode\b"), ("features", r"enum class Feature\b"),
                        ("knobs", r"enum class Knob\b")):
        out[key] = parse_enum_entries(tf.lines, header)
    names = {"modes": parse_name_switch(text, "mode"), "features": parse_name_switch(text, "feature"),
             "knobs": parse_name_switch(text, "knob")}
    short = parse_name_switch(text, "modeShort")
    for key, entries in out.items():
        for e in entries:
            e["name"] = names[key].get(e["id"], e["id"])
            if key == "modes":
                e["short"] = short.get(e["id"], e["name"])
            e["needs"] = e["tags"].get("needs") if isinstance(e["tags"].get("needs"), list) else (
                [] if e["tags"].get("needs") else None)
            e["better"] = e["tags"].get("better") if isinstance(e["tags"].get("better"), list) else []
    return out


# ------------------------------------------------------------
# SensorSelection.h
# ------------------------------------------------------------

KIT_GUARD = re.compile(r"^#ifndef SENSOR_KIT\s*$")
KIT_DEFINE = re.compile(r"^#define SENSOR_KIT\s+(SENSOR_KIT_\w+)")
STUDIO_BEGIN = "// --- OpenPlane Studio: датчики этого самолёта ---"
STUDIO_END = "// --- конец ---"


def norm(text):
    return re.sub(r"[^0-9a-zа-я]", "", text.lower())


def parse_sensors(tf):
    lines = tf.lines
    header = []
    for line in lines:
        if line.startswith(("#define", "#include", "#if")):
            break
        text = comment_text(line)
        if text and text.strip().startswith("•"):
            header.append(re.sub(r"[ 	]{2,}", " ", text.strip()[1:].strip()))

    options = {c: [] for c in SENSOR_CATEGORIES}
    kits, kit_map, current_kit, explicit = [], {}, None, {}
    kit_block = None
    for i, raw in enumerate(lines):
        line = raw.strip()
        m = re.match(r"#define (SENSOR_(IMU|BARO|MAG|AIRSPEED|GPS)_\w+)\s+(\d+)\s*(?://\s*(.*))?$", line)
        if m:
            ident, cat, value, note = m.group(1), m.group(2).lower(), int(m.group(3)), (m.group(4) or "").strip()
            chip = ident.split("_", 2)[2]
            if chip == "NONE":
                bus, title, doc = "none", "нет", ""
            else:
                base = chip[:-4] if chip.endswith("_SPI") else chip
                bus = "spi" if ("SPI" in note or chip.endswith("_SPI")) else ("uart" if cat == "gps" else "i2c")
                key = norm(base.replace("PITOT_", ""))
                pool = [h for h in header if "в трубке" in h.lower()] if cat == "airspeed" else header
                doc = next((h for h in pool if key and key in norm(h)), "")
                title = base.replace("PITOT_", "Пито ").replace("UBLOX_", "u-blox ").replace("_", " ")
            options[cat].append({"id": ident, "value": value, "chip": chip, "title": title, "bus": bus,
                                 "doc": doc, "note": note})
            continue
        m = re.match(r"#define (SENSOR_KIT_\w+)\s+(\d+)\s*(?://\s*(.*))?$", line)
        if m:
            kits.append({"id": m.group(1), "value": int(m.group(2)), "doc": (m.group(3) or "").strip()})
            continue
        m = re.match(r"#(?:el)?if SENSOR_KIT == (SENSOR_KIT_\w+)", line)
        if m:
            kit_block = m.group(1)
            kit_map[kit_block] = {}
            continue
        if line.startswith("#elif") or line.startswith("#endif"):
            kit_block = None
        m = re.match(r"#define KIT_(IMU|BARO|MAG|AIRSPEED|GPS)\s+(SENSOR_\w+)", line)
        if m and kit_block:
            kit_map[kit_block][m.group(1).lower()] = m.group(2)
            continue
        m = KIT_DEFINE.match(line)
        if m and i > 0 and KIT_GUARD.match(lines[i - 1].strip()):
            current_kit = m.group(1)
            continue
        m = re.match(r"#define SENSOR_(IMU|BARO|MAG|AIRSPEED|GPS)\s+(SENSOR_(?:IMU|BARO|MAG|AIRSPEED|GPS)_\w+)", line)
        if m:
            explicit[m.group(1).lower()] = m.group(2)
    for kit in kits:
        kit["sensors"] = kit_map.get(kit["id"], {})
    current = {c: explicit.get(c, kit_map.get(current_kit, {}).get(c)) for c in SENSOR_CATEGORIES}
    return {"options": options, "kits": kits, "currentKit": current_kit, "current": current, "explicit": explicit}


def board_sensors(info, board_kit):
    """Датчики, с которыми соберётся плата: строки SENSOR_* файла поверх набора —
    своего у платы (-D SENSOR_KIT=… в её env) или из строки SENSOR_KIT."""
    kits = {k["id"]: k["sensors"] for k in info["kits"]}
    base = kits.get(board_kit or info["currentKit"], {})
    return {c: info["explicit"].get(c, base.get(c)) for c in SENSOR_CATEGORIES}


# ------------------------------------------------------------
# Controls.h: привязки тумблеров
# ------------------------------------------------------------

BINDING = re.compile(r"^(?P<indent>\s*)Bind::(?P<kind>modes|mode|feature|knob)\s*\((?P<args>[^)]*)\)\s*,?\s*(?P<trail>//.*)?$")
SUGGESTION = re.compile(r"^\s*//\s*Bind::(?P<kind>modes|mode|feature|knob)\s*\((?P<args>[^)]*)\)\s*,?\s*(?://\s*(?P<doc>.*))?$")


def binding_from_args(kind, args):
    parts = [a.strip() for a in args.split(",")]
    channel = parts[0].split("::")[-1]
    if kind == "modes":
        return {"kind": "modes", "channel": channel, "modes": parts[1:]}
    if kind == "mode":
        return {"kind": "mode", "channel": channel, "mode": parts[1]}
    if kind == "feature":
        return {"kind": "feature", "channel": channel, "feature": parts[1].split("::")[-1]}
    return {"kind": "knob", "channel": channel, "knob": parts[1].split("::")[-1]}


def binding_key(b):
    return json.dumps(b, sort_keys=True, ensure_ascii=False)


def binding_line(b, indent):
    if b["kind"] == "modes":
        body = "Bind::modes  (Channels::{}, {})".format(b["channel"], ", ".join(b["modes"]))
    elif b["kind"] == "mode":
        body = "Bind::mode   (Channels::{}, {})".format(b["channel"], b["mode"])
    elif b["kind"] == "feature":
        body = "Bind::feature(Channels::{}, Feature::{})".format(b["channel"], b["feature"])
    else:
        body = "Bind::knob   (Channels::{}, Knob::{})".format(b["channel"], b["knob"])
    return indent + body + ","


def find_bindings_block(lines):
    start = next(i for i, l in enumerate(lines) if "constexpr Binding BINDINGS[]" in l)
    end = next(i for i in range(start + 1, len(lines)) if lines[i].strip().startswith("};"))
    return start, end


def parse_controls(tf):
    start, end = find_bindings_block(tf.lines)
    bindings, suggestions = [], []
    for line in tf.lines[start + 1:end]:
        m = BINDING.match(line)
        if m:
            bindings.append(binding_from_args(m.group("kind"), m.group("args")))
            continue
        m = SUGGESTION.match(line)
        if m:
            s = binding_from_args(m.group("kind"), m.group("args"))
            s["doc"] = (m.group("doc") or "").strip()
            suggestions.append(s)
    return {"bindings": bindings, "suggestions": suggestions}


# ------------------------------------------------------------
# Сборка описания (manifest)
# ------------------------------------------------------------

class Source:
    def __init__(self, root):
        self.root = os.path.abspath(root)
        self.files = {key: TextFile(self.root, rel) for key, rel in FILES.items()}

    def source_hash(self):
        h = hashlib.sha256()
        for key in sorted(self.files):
            h.update(self.files[key].text().encode("utf-8"))
        return h.hexdigest()


def public_param(p, enums):
    kind = classify(p, enums)
    tags = p["tags"]
    out = {
        "name": p["name"], "type": p["type"], "kind": kind, "value": p["value"],
        "expr": p["expr"], "valueKind": p["valueKind"], "board": p["board"],
        "group": p["group"], "doc": p["doc"] or p["groupDoc"], "note": p["note"],
        "fixed": bool(tags.get("fixed")), "line": p["line"] + 1,
        "unit": (tags["unit"][0] if isinstance(tags.get("unit"), list) and tags["unit"] else infer_unit(p["name"], p["type"])),
        "needs": tags.get("needs") if isinstance(tags.get("needs"), list) else [],
    }
    if isinstance(tags.get("range"), list) and len(tags["range"]) == 2:
        out["range"] = [float(tags["range"][0]), float(tags["range"][1])]
    if kind in ("int", "pin") and p["type"] in INT_LIMITS:
        out["min"], out["max"] = INT_LIMITS[p["type"]]
    if kind == "enum":
        out["options"] = enums[p["type"]]
    if kind == "array":
        out["size"] = p["size"]
        out["itemNotes"] = [it["note"] for it in p["items"]]
    return out


def describe(src):
    cfg_params, enums, board_tags = parse_declarations(src.files["config"])
    ch_params, _, _ = parse_declarations(src.files["channels"])
    autopilot = parse_autopilot(src.files["autopilot"])
    sensors = parse_sensors(src.files["sensors"])
    controls = parse_controls(src.files["controls"])
    envs = parse_platformio(src.files["platformio"].text())

    fixed_channels = {"AILERON", "ELEVATOR", "THROTTLE", "RUDDER", "ARM"}
    fixed_indices = {p["value"] for p in ch_params if p["name"] in fixed_channels}
    channels = []
    for p in ch_params:
        if p["name"] == "COUNT":
            continue
        positions = None
        m = re.search(r"(\d)\s+положени", p["note"])
        if m:
            positions = int(m.group(1))
        elif "крутилк" in p["note"]:
            positions = "knob"
        channels.append({"name": p["name"], "index": p["value"], "doc": p["note"], "positions": positions,
                         "stick": p["name"] in fixed_channels - {"ARM"},
                         "bindable": p["name"] not in fixed_channels and p["value"] not in fixed_indices})
    count = next((p["value"] for p in ch_params if p["name"] == "COUNT"), len(channels))

    params = [public_param(p, enums) for p in cfg_params]
    boards = []
    for env in envs:
        pins = {p["name"]: p["value"] for p in cfg_params if p["board"] and p["board"] == env["macro"]}

        def present(name):
            return name in pins and pins[name] != -1

        tags = board_tags.get(env["macro"], {})
        boards.append(dict(env, **{
            "blackbox": tags.get("blackbox", "none"),
            "telemetry": tags.get("telemetry", "none"),
            "sensors": board_sensors(sensors, env["kit"]),
            "outputs": {"rudder": present("PIN_RUDDER"), "aux1": present("PIN_AUX1"), "aux2": present("PIN_AUX2"),
                        "buzzer": present("PIN_BUZZER"), "screen": present("PIN_I2C2_SDA"),
                        "battery": present("PIN_VBAT_ADC"), "gpsTx": present("PIN_GPS_TX")},
        }))

    def strip_tags(entries):
        return [{k: v for k, v in e.items() if k != "tags"} for e in entries]

    return {
        "schema": SCHEMA_VERSION,
        "generator": "tools/opconfig.py",
        "sourceHash": src.source_hash(),
        "needTokens": NEED_TOKENS,
        "boards": boards,
        "params": params,
        "enums": enums,
        "channels": {"count": count, "list": channels},
        "modes": strip_tags(autopilot["modes"]),
        "features": strip_tags(autopilot["features"]),
        "knobs": strip_tags(autopilot["knobs"]),
        "sensors": {"options": sensors["options"], "kits": sensors["kits"]},
        "current": {"sensorKit": sensors["currentKit"], "sensors": sensors["current"],
                    "bindings": controls["bindings"]},
        "suggestions": controls["suggestions"],
    }


def current_config(manifest, board_env=None):
    """Конфигурация, которая описывает исходники как они есть (импорт в приложение, check)."""
    envs = manifest["boards"]
    env = next((b for b in envs if b["env"] == board_env), None) or next((b for b in envs if b["default"]), envs[0])
    params = {}
    for p in manifest["params"]:
        if p["fixed"] or (p["board"] and p["board"] != env["macro"]):
            continue
        params[p["name"]] = p["value"]
    return {
        "schema": SCHEMA_VERSION,
        "board": env["env"],
        "params": params,
        "channels": {c["name"]: c["index"] for c in manifest["channels"]["list"]},
        "sensors": dict(env["sensors"]),
        "bindings": list(manifest["current"]["bindings"]),
    }


# ------------------------------------------------------------
# Запись конфигурации
# ------------------------------------------------------------

def format_value(p, value, enums, channel_names):
    kind = classify(p, enums)
    name = p["name"]
    if kind == "bool":
        if not isinstance(value, bool):
            raise ConfigError("{}: нужно true или false".format(name))
        return "true" if value else "false"
    if kind == "float":
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ConfigError("{}: нужно число".format(name))
        text = repr(float(value))
        if "e" not in text and "." not in text:
            text += ".0"
        return text + "f"
    if kind in ("int", "pin"):
        if isinstance(value, str) and kind == "pin" and STM32_PIN.fullmatch(value):
            return value
        if isinstance(value, bool) or not isinstance(value, int):
            if isinstance(value, float) and value.is_integer():
                value = int(value)
            else:
                raise ConfigError("{}: нужно целое число{}".format(name, " или пин вида PA3" if kind == "pin" else ""))
        lo, hi = INT_LIMITS[p["type"]]
        if not lo <= value <= hi:
            raise ConfigError("{}: {} вне пределов {}…{}".format(name, value, lo, hi))
        return str(value)
    if kind == "string":
        if not isinstance(value, str):
            raise ConfigError("{}: нужна строка".format(name))
        return c_string(value)
    if kind == "enum":
        short = str(value).split("::")[-1]
        if short not in enums[p["type"]]:
            raise ConfigError("{}: нет варианта {} (есть: {})".format(name, short, ", ".join(enums[p["type"]])))
        return "{}::{}".format(p["type"], short)
    if kind == "channel":
        short = str(value).split("::")[-1]
        if short not in channel_names:
            raise ConfigError("{}: нет канала {}".format(name, short))
        return "Channels::{}".format(short)
    raise ConfigError("{}: этот параметр приложение менять не умеет".format(name))


def patch_line(tf, line, start, end, text):
    old = tf.lines[line]
    tf.lines[line] = old[:start] + text + old[end:]


def apply_params(tf, requested, board_macro, enums, channel_names, errors):
    params, _, _ = parse_declarations(tf)
    by_name = {}
    for p in params:
        if p["board"] and p["board"] != board_macro:
            continue
        by_name.setdefault(p["name"], p)
    # Правки с конца строки к началу: позиции в строке не сдвигаются.
    edits = []
    for name, value in requested.items():
        p = by_name.get(name)
        if p is None:
            errors.append("{}: такого параметра нет{}".format(
                name, " для этой платы" if name.startswith("PIN_") else ""))
            continue
        if same_value(p["value"], value) or (p["valueKind"] == "ident" and str(value).split("::")[-1] == str(p["value"]).split("::")[-1]):
            continue
        if p["tags"].get("fixed"):
            errors.append("{}: параметр протокола, менять нельзя".format(name))
            continue
        try:
            if p["array"]:
                if not isinstance(value, list) or len(value) != len(p["items"]):
                    raise ConfigError("{}: нужен список из {} значений".format(name, len(p["items"])))
                element = dict(p, array=False)
                for item, v in zip(p["items"], value):
                    if not same_value(item["value"], v):
                        edits.append((item["line"], item["start"], item["end"], format_value(element, v, enums, channel_names)))
            else:
                edits.append((p["line"], p["start"], p["end"], format_value(p, value, enums, channel_names)))
        except ConfigError as e:
            errors.append(str(e))
    for line, start, end, text in sorted(edits, key=lambda e: (e[0], -e[1])):
        patch_line(tf, line, start, end, text)


def apply_channels(tf, requested, errors):
    params, _, _ = parse_declarations(tf)
    by_name = {p["name"]: p for p in params}
    count = by_name.get("COUNT", {}).get("value", 10)
    for name, value in requested.items():
        p = by_name.get(name)
        if p is None or name == "COUNT":
            errors.append("Channels::{}: такого канала нет".format(name))
            continue
        if p["value"] == value:
            continue
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < count:
            errors.append("Channels::{}: номер канала 0…{}".format(name, count - 1))
            continue
        patch_line(tf, p["line"], p["start"], p["end"], str(value))


def check_bindings(bindings, manifest, errors):
    channels = {c["name"]: c for c in manifest["channels"]["list"]}
    modes = {m["id"] for m in manifest["modes"]}
    features = {f["id"] for f in manifest["features"]}
    knobs = {k["id"] for k in manifest["knobs"]}
    if not bindings:
        errors.append("Тумблеры: нужна хотя бы одна привязка")
    used, mode_switches = {}, 0
    for b in bindings:
        ch = channels.get(b.get("channel"))
        if ch is None:
            errors.append("Тумблеры: нет канала {}".format(b.get("channel")))
            continue
        if not ch["bindable"]:
            errors.append("Тумблеры: {} занимать нельзя — стики и ARM закреплены".format(b["channel"]))
        if ch["index"] in used:
            errors.append("Тумблеры: на {} уже есть привязка ({})".format(b["channel"], used[ch["index"]]))
        used[ch["index"]] = b["channel"]
        kind = b.get("kind")
        if kind == "modes":
            mode_switches += 1
            if len(b.get("modes", [])) not in (2, 3):
                errors.append("Тумблеры: у тумблера режимов {} — 2 или 3 положения".format(b["channel"]))
            bad = [m for m in b.get("modes", []) if m not in modes]
        elif kind == "mode":
            bad = [b.get("mode")] if b.get("mode") not in modes else []
        elif kind == "feature":
            bad = [b.get("feature")] if b.get("feature") not in features else []
        elif kind == "knob":
            bad = [b.get("knob")] if b.get("knob") not in knobs else []
        else:
            bad = ["вид привязки " + str(kind)]
        for item in bad:
            errors.append("Тумблеры: неизвестно {} на {}".format(item, b["channel"]))
    if mode_switches > 1:
        errors.append("Тумблеры: тумблер выбора режимов может быть только один")


def apply_bindings(tf, bindings, errors):
    start, end = find_bindings_block(tf.lines)
    body = tf.lines[start + 1:end]
    original = {}
    indent = "        "
    for line in body:
        m = BINDING.match(line)
        if m:
            indent = m.group("indent")
            original.setdefault(binding_key(binding_from_args(m.group("kind"), m.group("args"))), line)
    new_active = []
    for b in bindings:
        clean = {k: v for k, v in b.items() if k in ("kind", "channel", "modes", "mode", "feature", "knob")}
        new_active.append(original.get(binding_key(clean)) or binding_line(clean, indent))
    rest = [l for l in body if not BINDING.match(l)]
    while rest and not rest[0].strip():
        rest.pop(0)
    new_body = new_active + ([""] + rest if rest else [])
    if new_body != body:
        tf.lines[start + 1:end] = new_body


def apply_sensors(tf, requested, manifest, errors, board_kit=None):
    info = parse_sensors(tf)
    current = board_sensors(info, board_kit)
    wanted = {}
    for cat in SENSOR_CATEGORIES:
        value = requested.get(cat, current.get(cat))
        valid = {o["id"] for o in info["options"][cat]}
        if value not in valid:
            errors.append("Датчики: нет варианта {} для {}".format(value, cat))
            return
        wanted[cat] = value
    if wanted == current:
        return
    lines = tf.lines
    # убрать прежний блок приложения
    if STUDIO_BEGIN in lines:
        b = lines.index(STUDIO_BEGIN)
        e = lines.index(STUDIO_END, b)
        del lines[b:e + 1]
    guard = next(i for i in range(len(lines) - 1) if KIT_GUARD.match(lines[i]) and KIT_DEFINE.match(lines[i + 1]))
    if board_kit:
        # Набор задан флагом окружения, строка SENSOR_KIT для этой платы не
        # действует: свои датчики — только строками SENSOR_*.
        if wanted == board_sensors(dict(info, explicit={}), board_kit):
            return
    else:
        kit = next((k["id"] for k in info["kits"] if k["sensors"] and k["sensors"] == wanted), None)
        if kit:
            lines[guard + 1] = "#define SENSOR_KIT " + kit
            return
        lines[guard + 1] = "#define SENSOR_KIT SENSOR_KIT_CUSTOM"
    block = [STUDIO_BEGIN]
    for cat in SENSOR_CATEGORIES:
        macro = "SENSOR_" + cat.upper()
        block += ["#ifndef " + macro, "#define {} {}".format(macro, wanted[cat]), "#endif"]
    block.append(STUDIO_END)
    endif = guard + 2
    lines[endif + 1:endif + 1] = block


def project_header(project):
    out = [
        "#pragma once",
        "// Записано приложением OpenPlane Studio при сборке — не править руками:",
        "// при следующей прошивке файл перезапишется. Что это — telemetry/FirmwareIdentity.h.",
    ]
    for macro, key in (("OP_BOARD_ENV", "boardEnv"), ("OP_SOURCE_COMMIT", "commit"),
                       ("OP_CONFIG_HASH", "configHash"), ("OP_PLANE_NAME", "plane")):
        value = project.get(key)
        if value is not None:
            out.append("#define {} {}".format(macro, c_string(str(value))))
    return "\n".join(out) + "\n"


def apply(src, config, write=True):
    """Записать конфигурацию. Ошибки — ConfigError со списком; ничего не пишется."""
    manifest = describe(src)
    errors = []
    env = next((b for b in manifest["boards"] if b["env"] == config.get("board")), None)
    if env is None:
        raise ConfigError("Плата: нет окружения {} в platformio.ini".format(config.get("board")))
    channel_names = {c["name"] for c in manifest["channels"]["list"]}
    _, enums, _ = parse_declarations(src.files["config"])

    apply_params(src.files["config"], config.get("params", {}), env["macro"], enums, channel_names, errors)
    apply_channels(src.files["channels"], config.get("channels", {}), errors)
    if "bindings" in config:
        check_bindings(config["bindings"], manifest, errors)
        if not errors:
            apply_bindings(src.files["controls"], config["bindings"], errors)
    if "sensors" in config:
        apply_sensors(src.files["sensors"], config["sensors"], manifest, errors, env["kit"])
    header = None
    if "project" in config:
        try:
            header = project_header(dict(config["project"], boardEnv=config["project"].get("boardEnv", env["env"])))
        except ConfigError as e:
            errors.append("Проект: " + str(e))
    if errors:
        raise ConfigError("\n".join(errors))

    changed = [tf.rel for tf in src.files.values() if tf.changed()]
    if write:
        for tf in src.files.values():
            if tf.changed():
                tf.save()
        if header is not None:
            path = os.path.join(src.root, PROJECT_HEADER)
            old = None
            if os.path.exists(path):
                with open(path, encoding="utf-8") as f:
                    old = f.read()
            if old != header:
                with open(path, "w", encoding="utf-8", newline="\n") as f:
                    f.write(header)
                changed.append(PROJECT_HEADER)
    return {"board": env["env"], "changed": changed}


# ------------------------------------------------------------
# Проверка меток
# ------------------------------------------------------------

def check(root):
    errors = []
    src = Source(root)
    manifest = describe(src)
    for key, title in (("modes", "режим"), ("features", "функция"), ("knobs", "крутилка")):
        for e in manifest[key]:
            if e["needs"] is None:
                errors.append("AutopilotTypes.h: {} {} — нет метки @needs(...)".format(title, e["id"]))
            for token in (e["needs"] or []) + e["better"]:
                if token not in NEED_TOKENS:
                    errors.append("AutopilotTypes.h: {} — неизвестное «{}» (есть: {})".format(
                        e["id"], token, ", ".join(NEED_TOKENS)))
    for p in manifest["params"]:
        for token in p["needs"]:
            if token not in NEED_TOKENS:
                errors.append("Config.h: {} — неизвестное «{}»".format(p["name"], token))
        if p["kind"] == "channel" and not str(p["value"]).startswith("Channels::"):
            errors.append("Config.h: {} помечен @channel, но значение не Channels::...".format(p["name"]))
        if p["valueKind"] == "raw":
            errors.append("Config.h: {} — не разобрать значение «{}»".format(p["name"], p["expr"]))
        if "range" in p:
            lo, hi = p["range"]
            if lo > hi:
                errors.append("Config.h: {} — @range({}, {}): нижний предел больше верхнего".format(p["name"], lo, hi))
            elif isinstance(p["value"], (int, float)) and not isinstance(p["value"], bool) and not lo <= p["value"] <= hi:
                errors.append("Config.h: {} = {} — за пределами своей метки @range({}, {})".format(p["name"], p["value"], lo, hi))
    macros = {p["board"] for p in manifest["params"] if p["board"]}
    for b in manifest["boards"]:
        if b["macro"] not in macros:
            errors.append("Config.h: нет блока пинов для {} (env {})".format(b["macro"], b["env"]))
    _, _, board_tags = parse_declarations(src.files["config"])
    for macro in sorted(macros):
        tags = board_tags.get(macro)
        if not tags:
            errors.append("Config.h: в блоке {} нет метки @board(...)".format(macro))
            continue
        for key, allowed in BOARD_TAG_KEYS.items():
            if tags.get(key) not in allowed:
                errors.append("Config.h: @board в {}: {}= одно из {}".format(macro, key, ", ".join(allowed)))
        for key in tags:
            if key not in BOARD_TAG_KEYS:
                errors.append("Config.h: @board в {}: неизвестная метка {}".format(macro, key))
    kits = {k["id"] for k in manifest["sensors"]["kits"] if k["sensors"]}
    for b in manifest["boards"]:
        if b["kit"] and b["kit"] not in kits:
            errors.append("platformio.ini: env {}: SENSOR_KIT={} — нет такого набора в SensorSelection.h".format(
                b["env"], b["kit"]))
    for b in manifest["boards"]:
        try:
            result = apply(Source(root), current_config(manifest, b["env"]), write=False)
            if result["changed"]:
                errors.append("apply с текущими значениями ({}) меняет: {}".format(b["env"], ", ".join(result["changed"])))
        except ConfigError as e:
            errors.append("apply с текущими значениями ({}): {}".format(b["env"], e))
    return errors, manifest


# ------------------------------------------------------------

def default_root():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="Описание и настройка прошивки OpenPlane для OpenPlane Studio")
    sub = parser.add_subparsers(dest="command", required=True)
    p_describe = sub.add_parser("describe", help="описание прошивки (JSON)")
    p_describe.add_argument("--root", default=default_root())
    p_describe.add_argument("--out", help="файл вместо stdout")
    p_apply = sub.add_parser("apply", help="записать конфигурацию в исходники")
    p_apply.add_argument("--root", default=default_root())
    p_apply.add_argument("--config", required=True, help="JSON конфигурации ('-' — stdin)")
    p_apply.add_argument("--dry-run", action="store_true", help="только проверить")
    p_check = sub.add_parser("check", help="проверить метки и apply")
    p_check.add_argument("--root", default=default_root())
    p_current = sub.add_parser("current", help="конфигурация исходников как они есть (JSON)")
    p_current.add_argument("--root", default=default_root())
    p_current.add_argument("--board")
    args = parser.parse_args(argv)

    if args.command == "describe":
        text = json.dumps(describe(Source(args.root)), ensure_ascii=False, indent=1)
        if args.out:
            with open(args.out, "w", encoding="utf-8", newline="\n") as f:
                f.write(text + "\n")
        else:
            print(text)
        return 0
    if args.command == "current":
        print(json.dumps(current_config(describe(Source(args.root)), args.board), ensure_ascii=False, indent=1))
        return 0
    if args.command == "apply":
        if args.config == "-":
            raw = sys.stdin.read()
        else:
            with open(args.config, encoding="utf-8") as f:
                raw = f.read()
        try:
            result = apply(Source(args.root), json.loads(raw), write=not args.dry_run)
        except ConfigError as e:
            print(json.dumps({"ok": False, "errors": str(e).split("\n")}, ensure_ascii=False))
            return 2
        print(json.dumps(dict(result, ok=True), ensure_ascii=False))
        return 0
    errors, manifest = check(args.root)
    if errors:
        for e in errors:
            print("ОШИБКА: " + e)
        return 1
    print("opconfig check: OK — плат {}, параметров {}, режимов {}, функций {}, крутилок {}".format(
        len(manifest["boards"]), len(manifest["params"]), len(manifest["modes"]),
        len(manifest["features"]), len(manifest["knobs"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
