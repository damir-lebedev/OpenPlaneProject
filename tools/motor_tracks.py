"""Встроенные треки музыки моторами (176c) и формат трека.

Треки — только общественное достояние (народные и классика: авторы умерли
больше 70 лет назад). Свой трек (любой MIDI) — `motor_music.py трек.mid
--upload --port COM13`: он ляжет в слот custom во флеше платы.

    python tools/motor_tracks.py            # перегенерировать include/copter/MotorTracks.h
    python tools/motor_tracks.py --check    # заголовок совпадает с этим файлом
    python tools/motor_tracks.py --wav DIR  # послушать треки

Формат трека (прошивка: copter/MotorTracks.h):
    'T', голосов (1..4), единица длительности, мс (uint16 LE),
    по голосу: число событий (uint16 LE), события (нота MIDI 0..127, 0 — пауза;
    длительность в единицах 1..255).
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HEADER = os.path.join(HERE, "..", "include", "copter", "MotorTracks.h")

NAMES = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}


def note_number(tok):
    """'C#5', 'Eb4', 'R' -> MIDI (0 — пауза)."""
    if tok.upper() == "R":
        return 0
    m = re.fullmatch(r"([A-Ga-g])([#b]?)(-?\d)", tok)
    if not m:
        raise ValueError("нота? %r" % tok)
    n = NAMES[m.group(1).upper()] + {"#": 1, "b": -1, "": 0}[m.group(2)]
    return 12 * (int(m.group(3)) + 1) + n


def encode(voices, unit_ms):
    """[[(нота, единиц)], …] -> bytes. Длинные ноты режутся на куски по 255."""
    if not 1 <= len(voices) <= 4:
        raise ValueError("голосов 1..4")
    if not 1 <= unit_ms <= 60000:
        raise ValueError("единица 1..60000 мс")
    b = bytearray(b"T") + bytes([len(voices)]) + int(unit_ms).to_bytes(2, "little")
    for v in voices:
        ev = []
        for note, dur in v:
            if not 0 <= note <= 127 or dur < 1:
                raise ValueError("нота %r длительность %r" % (note, dur))
            while dur > 0:
                ev.append((note, min(255, dur)))
                dur -= 255
        b += len(ev).to_bytes(2, "little")
        for note, dur in ev:
            b += bytes([note, dur])
    return bytes(b)


def decode(data):
    """bytes -> (unit_ms, [[(нота, единиц)], …]); ValueError — битый."""
    if len(data) < 4 or data[0:1] != b"T" or not 1 <= data[1] <= 4:
        raise ValueError("не трек")
    unit = int.from_bytes(data[2:4], "little")
    i, voices = 4, []
    for _ in range(data[1]):
        if i + 2 > len(data):
            raise ValueError("обрезан")
        n = int.from_bytes(data[i:i + 2], "little")
        i += 2
        if i + 2 * n > len(data):
            raise ValueError("обрезан")
        voices.append([(data[i + 2 * k], data[i + 2 * k + 1]) for k in range(n)])
        i += 2 * n
    if i != len(data):
        raise ValueError("лишние байты")
    return unit, voices


# ---------------- треки ----------------
# Длительности — в единицах трека (у всех ниже единица — шестнадцатая).

KOROBEINIKI_MEL = """
E5:4 B4:2 C5:2 D5:4 C5:2 B4:2 | A4:4 A4:2 C5:2 E5:4 D5:2 C5:2 | B4:6 C5:2 D5:4 E5:4 | C5:4 A4:4 A4:8 |
R:2 D5:4 F5:2 A5:4 G5:2 F5:2 | E5:6 C5:2 E5:4 D5:2 C5:2 | B4:4 B4:2 C5:2 D5:4 E5:4 | C5:4 A4:4 A4:4 R:4
"""


def bounce(pairs):
    """Бас «восьмые через октаву»: [(низ, верх, восьмых)] -> текст."""
    return " ".join(("%s:2 %s:2 " % (lo, hi)) * (n // 2) for lo, hi, n in pairs)


KOROBEINIKI_BASS = bounce([("E3", "E4", 8), ("A3", "A4", 8), ("G#3", "G#4", 8), ("A3", "A4", 8),
                           ("D4", "D5", 8), ("C4", "C5", 8), ("B3", "B4", 4), ("E3", "E4", 4), ("A3", "A4", 8)])

ODE_MEL = """
E5:4 E5:4 F5:4 G5:4 | G5:4 F5:4 E5:4 D5:4 | C5:4 C5:4 D5:4 E5:4 | E5:6 D5:2 D5:8 |
E5:4 E5:4 F5:4 G5:4 | G5:4 F5:4 E5:4 D5:4 | C5:4 C5:4 D5:4 E5:4 | D5:6 C5:2 C5:8
"""
ODE_BASS = "C4:16 B3:16 C4:16 B3:16 C4:16 B3:16 C4:16 B3:8 C4:8"

GRIEG = """
B3:2 C#4:2 D4:2 E4:2 F#4:2 D4:2 F#4:4 | F4:2 C#4:2 F4:4 E4:2 C4:2 E4:4 |
B3:2 C#4:2 D4:2 E4:2 F#4:2 D4:2 F#4:2 B4:2 | A4:2 F#4:2 D4:2 F#4:2 A4:8
"""

ELISE = """
E5 D#5 | E5 D#5 E5 B4 D5 C5 | A4:2 R C4 E4 A4 | B4:2 R E4 G#4 B4 | C5:2 R E4 E5 D#5 |
E5 D#5 E5 B4 D5 C5 | A4:2 R C4 E4 A4 | B4:2 R E4 C5 B4 | A4:6
"""

SCALE = " ".join("%s:2" % n for n in
                 "C4 D4 E4 F4 G4 A4 B4 C5 D5 E5 F5 G5 A5 B5 C6 R B5 A5 G5 F5 E5 D5 C5 B4 A4 G4 F4 E4 D4 C4".split())


def octave(text, shift):
    """Голос октавой ниже/выше (для Грига — та же тема октавой ниже)."""
    out = []
    for tok in text.split():
        if tok == "|":
            continue
        name, sep, dur = tok.partition(":")
        n = note_number(name)
        out.append(("R" if n == 0 else "N%d" % (n + shift)) + (sep + dur if sep else ""))
    return " ".join(out)


def parse_voice_n(text):
    """'E5:4 B4:2 N47:2 R:2 …' (длительность — в единицах трека, N — номер MIDI) -> [(нота, единиц)]."""
    out = []
    for tok in text.split():
        if tok == "|":
            continue
        name, _, dur = tok.partition(":")
        n = int(name[1:]) if name.startswith("N") else note_number(name)
        out.append((n, int(dur or 1)))
    return out


# имя -> (единица, мс; голоса; что это)
TRACKS = [
    ("korobeiniki", 105, [KOROBEINIKI_MEL, KOROBEINIKI_BASS], "«Коробейники» — народная, мелодия и бас"),
    ("ode", 125, [ODE_MEL, ODE_BASS], "Бетховен, «Ода к радости» — мелодия и бас"),
    ("grieg", 110, [GRIEG + GRIEG, octave(GRIEG + GRIEG, -12)], "Григ, «В пещере горного короля» — дважды, в две октавы"),
    ("elise", 115, [ELISE], "Бетховен, «К Элизе» — тема"),
    ("scale", 250, [SCALE], "гамма C4…C6 и вниз — проверка высоты на стенде"),
]


def built():
    out = []
    for name, unit, voices, about in TRACKS:
        vs = [parse_voice_n(v) for v in voices]
        out.append((name, about, encode(vs, unit)))
    return out


def header_text():
    lines = [
        "#pragma once",
        "// Встроенные треки музыки моторами (176c). СГЕНЕРИРОВАНО tools/motor_tracks.py —",
        "// не править руками. Формат и проигрыватель — copter/MotorTrack.h.",
        "",
        "#include <stdint.h>",
        "",
        "namespace copter",
        "{",
    ]
    tracks = built()
    for name, about, data in tracks:
        lines.append("    // %s: %s, %d байт" % (name, about, len(data)))
        lines.append("    static const uint8_t MOTOR_TRACK_%s[] = {" % name.upper())
        for k in range(0, len(data), 16):
            lines.append("        " + ", ".join("%d" % x for x in data[k:k + 16]) + ",")
        lines.append("    };")
    lines += [
        "",
        "    struct MotorTrackInfo",
        "    {",
        "        const char* name;",
        "        const uint8_t* data;",
        "        uint16_t size;",
        "    };",
        "    static const MotorTrackInfo MOTOR_TRACKS[] = {",
    ]
    for name, _, data in tracks:
        lines.append('        { "%s", MOTOR_TRACK_%s, %d },' % (name, name.upper(), len(data)))
    lines += [
        "    };",
        "    constexpr uint8_t MOTOR_TRACK_COUNT = %d;" % len(tracks),
        "    // beeper_melody_motors: OFF, ON (тоны DShot), встроенные, свой (custom) — трек N = значение − 1",
        "    static const char* const MOTOR_TRACK_PARAM_NAMES[] = { \"OFF\", \"ON\", %s, \"custom\" };"
        % ", ".join('"%s"' % n for n, _, _ in tracks),
        "}",
        "",
    ]
    return "\n".join(lines)


def preview(directory):
    """WAV каждого трека — как его сыграют моторы (без переноса октав)."""
    import motor_music as mm
    os.makedirs(directory, exist_ok=True)
    for name, _, data in built():
        unit, voices = decode(data)
        lanes = []
        for v in voices:
            t, lane = 0.0, []
            for note, dur in v:
                if note:
                    lane.append((t, t + dur * unit / 1000.0, mm.note_hz(note)))
                t += dur * unit / 1000.0
            lanes.append(lane)
        path = os.path.join(directory, name + ".wav")
        mm.render_wav(path, lanes)
        print(path)


def main(argv):
    if "--wav" in argv:
        preview(argv[argv.index("--wav") + 1])
        return 0
    text = header_text()
    if "--check" in argv:
        with open(HEADER, encoding="utf-8") as f:
            same = f.read() == text
        print("совпадает" if same else "РАЗОШЁЛСЯ — запусти без --check")
        return 0 if same else 1
    with open(HEADER, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    for name, _, data in built():
        print("%-12s %4d байт" % (name, len(data)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
