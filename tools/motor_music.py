"""Музыка моторами по оборотам (176c): MIDI -> голоса по моторам -> плата.

Каждый мотор (без винта!) держит обороты, на которых его электрическая
частота равна ноте (прошивка: copter/MotorMusic.h, команды `music on [k]`,
`note N Гц …`). Скрипт раскладывает MIDI на голоса — верхний голос на мотор 0,
дальше вниз, — переносит октавами в диапазон, который мотор берёт, и шлёт
ноты по порту в такт.

    python tools/motor_music.py track.mid --wav preview.wav        # послушать раскладку
    python tools/motor_music.py track.mid --port COM13 --live      # сыграть на плате
    python tools/motor_music.py track.mid --list                   # дорожки и каналы
    python tools/motor_music.py track.mp3 --to-mid track.mid       # звук -> ноты (numpy, ffmpeg)

Винты снять. ARM, любая клавиша приложения или Ctrl+C — стоп. pyserial —
в ~/.platformio/penv/Scripts/python.exe.
"""

import argparse
import json
import math
import struct
import sys
import threading
import time
import wave

A4 = 440.0


def note_hz(n):
    return A4 * 2.0 ** ((n - 69) / 12.0)


def cents(hz, ref):
    return 1200.0 * math.log2(hz / ref) if hz > 0 and ref > 0 else 0.0


# ---------------- MIDI ----------------

def _varlen(d, i):
    v = 0
    while True:
        b = d[i]
        i += 1
        v = (v << 7) | (b & 0x7F)
        if not b & 0x80:
            return v, i


def parse_midi(data):
    """-> (notes, tempo_changes). notes: [(start_s, end_s, note, track, channel)]."""
    if data[:4] != b"MThd":
        raise ValueError("не MIDI (нет MThd)")
    hlen, fmt, ntracks, div = struct.unpack(">IHHH", data[4:14])
    if div & 0x8000:
        raise ValueError("SMPTE-время не поддерживается")
    i = 8 + hlen
    events = []   # (tick, order, kind, track, channel, note/tempo)
    order = 0
    for t in range(ntracks):
        while data[i:i + 4] != b"MTrk":
            if i + 8 > len(data):
                raise ValueError("мало дорожек")
            i += 8 + struct.unpack(">I", data[i + 4:i + 8])[0]
        tlen = struct.unpack(">I", data[i + 4:i + 8])[0]
        j, end = i + 8, i + 8 + tlen
        tick, status = 0, 0
        while j < end:
            dt, j = _varlen(data, j)
            tick += dt
            b = data[j]
            if b == 0xFF:
                mtype = data[j + 1]
                ln, k = _varlen(data, j + 2)
                if mtype == 0x51 and ln == 3:
                    events.append((tick, order, "tempo", t, 0, int.from_bytes(data[k:k + 3], "big")))
                j = k + ln
                if mtype == 0x2F:
                    break
                continue
            if b in (0xF0, 0xF7):
                ln, k = _varlen(data, j + 1)
                j = k + ln
                continue
            if b & 0x80:
                status = b
                j += 1
            hi, ch = status & 0xF0, status & 0x0F
            if hi in (0xC0, 0xD0):
                j += 1
                continue
            a, v = data[j], data[j + 1]
            j += 2
            if hi == 0x90 and v > 0:
                events.append((tick, order, "on", t, ch, a))
            elif hi == 0x80 or (hi == 0x90 and v == 0):
                events.append((tick, order, "off", t, ch, a))
            order += 1
        i = end
    events.sort(key=lambda e: (e[0], e[2] != "tempo", e[2] != "off", e[1]))
    # тики -> секунды по карте темпа
    tempo, last_tick, last_s = 500000, 0, 0.0
    open_notes, notes, tempos = {}, [], []
    for tick, _, kind, t, ch, val in events:
        s = last_s + (tick - last_tick) * tempo / 1e6 / div
        last_tick, last_s = tick, s
        if kind == "tempo":
            tempo = val
            tempos.append((s, 60e6 / val))
        elif kind == "on":
            open_notes.setdefault((t, ch, val), []).append(s)
        else:
            starts = open_notes.get((t, ch, val))
            if starts:
                st = starts.pop(0)
                if s > st:
                    notes.append((st, s, val, t, ch))
    notes.sort()
    return notes, tempos


def write_midi(path, notes, bpm=120.0, div=480):
    """notes [(start_s, end_s, note, …)] -> MIDI формата 0, канал 0."""
    sec_per_tick = 60.0 / bpm / div
    ev = []
    for n in notes:
        ev.append((int(round(n[0] / sec_per_tick)), 1, 0x90, n[2], 100))
        ev.append((int(round(n[1] / sec_per_tick)), 0, 0x80, n[2], 0))
    ev.sort()
    body, last = bytearray(), 0
    tempo = int(round(60e6 / bpm))

    def vl(v):
        out = [v & 0x7F]
        v >>= 7
        while v:
            out.insert(0, 0x80 | (v & 0x7F))
            v >>= 7
        return bytes(out)

    body += vl(0) + b"\xff\x51\x03" + tempo.to_bytes(3, "big")
    for tick, _, st, n, v in ev:
        body += vl(tick - last) + bytes([st, n, v])
        last = tick
    body += vl(0) + b"\xff\x2f\x00"
    with open(path, "wb") as f:
        f.write(b"MThd" + struct.pack(">IHHH", 6, 0, 1, div))
        f.write(b"MTrk" + struct.pack(">I", len(body)) + bytes(body))


# ---------------- звук -> ноты ----------------

def decode_audio(path, rate=16000):
    """mp3/wav/… -> numpy float32 моно (через ffmpeg)."""
    import subprocess

    import numpy as np
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-ac", "1", "-ar", str(rate), "-f", "f32le", "-"],
                         check=True, capture_output=True).stdout
    return np.frombuffer(raw, dtype=np.float32)


def transcribe(x, rate=16000, voices=4, lo_note=40, hi_note=96, hop=512, win=4096, rel=0.3, min_frames=3):
    """Грубая расшифровка звука в ноты: в каждом окне — до voices самых сильных
    тонов по сумме гармоник (найденный тон гасит свои гармоники), соседние
    окна склеиваются в ноты. Чистая мелодия и бас выходят хорошо, полный
    микс с барабанами и голосом — приблизительно. -> [(start, end, note, 0, 0)]."""
    import numpy as np
    w = np.hanning(win).astype(np.float32)
    nfft = win * 2
    freqs = np.fft.rfftfreq(nfft, 1.0 / rate)
    cand = np.arange(lo_note, hi_note + 1)
    f0 = A4 * 2.0 ** ((cand - 69) / 12.0)
    frames = max(0, (len(x) - win) // hop + 1)
    peak_all = 1e-9
    per = []
    for k in range(frames):
        mag = np.abs(np.fft.rfft(x[k * hop:k * hop + win] * w, nfft))
        per.append(mag)
        peak_all = max(peak_all, float(mag.max()))
    active = []
    for mag in per:
        mag = mag.copy()
        found = []
        first = None
        for _ in range(voices):
            sal = np.zeros(len(cand))
            for h in range(1, 6):
                idx = np.searchsorted(freqs, f0 * h)
                idx = np.clip(idx, 1, len(freqs) - 2)
                sal += 0.8 ** (h - 1) * np.maximum(np.maximum(mag[idx - 1], mag[idx]), mag[idx + 1])
            j = int(np.argmax(sal))
            if first is None:
                first = sal[j]
            if sal[j] < rel * first or mag.max() < 0.02 * peak_all:
                break
            found.append(int(cand[j]))
            for h in range(1, 9):
                c = np.searchsorted(freqs, f0[j] * h)
                mag[max(0, c - 3):c + 4] = 0.0
        active.append(set(found))
    notes, open_at = [], {}
    for k, s in enumerate(active + [set()]):
        nxt = active[k + 1] if k + 1 < len(active) else set()
        for n in list(open_at):
            if n not in s and n not in nxt:   # пропуск в одно окно ноту не рвёт
                st = open_at.pop(n)
                if k - st >= min_frames:
                    notes.append(((st * hop + win / 2) / rate, (k * hop + win / 2) / rate, n, 0, 0))   # по середине окна
        for n in s:
            open_at.setdefault(n, k)
    notes.sort()
    return notes


# ---------------- раскладка ----------------

def pick(notes, tracks=None, channels=None, drums=False):
    out = []
    for n in notes:
        if not drums and n[4] == 9:
            continue
        if tracks is not None and n[3] not in tracks:
            continue
        if channels is not None and n[4] not in channels:
            continue
        out.append(n)
    return out


def allocate(notes, voices):
    """Ноты -> голоса (моторы): в каждый момент самые высокие ноты, голос 0 — верх.

    -> [[(start, end, note)], …] по голосу; лишние нижние ноты отбрасываются,
    у одного голоса ноты не перекрываются (новая обрезает старую)."""
    times = sorted({n[0] for n in notes} | {n[1] for n in notes})
    lanes = [[] for _ in range(voices)]
    if not notes:
        return lanes
    active = []   # открытые ноты
    k = 0
    cur = [None] * voices   # (start, note) звучащая сейчас
    for idx, t in enumerate(times):
        active = [n for n in active if n[1] > t]
        while k < len(notes) and notes[k][0] <= t:
            if notes[k][1] > t:
                active.append(notes[k])
            k += 1
        want = sorted({n[2] for n in active}, reverse=True)[:voices]
        # голоса держат свою ноту, пока она в want; новые — по порядку высоты
        new = [None] * voices
        free = list(want)
        for v in range(voices):
            if cur[v] is not None and cur[v][1] in free:
                new[v] = cur[v]
                free.remove(cur[v][1])
        # оставшиеся ноты — в свободные голоса сверху вниз
        for v in range(voices):
            if new[v] is None and free:
                new[v] = (t, free.pop(0))
        # закончившиеся и сменившиеся ноты — в дорожку
        for v in range(voices):
            if cur[v] is not None and (new[v] is None or new[v][0] != cur[v][0]):
                if t > cur[v][0]:
                    lanes[v].append((cur[v][0], t, cur[v][1]))
        cur = new
    for v in range(voices):
        if cur[v] is not None and times[-1] > cur[v][0]:
            lanes[v].append((cur[v][0], times[-1], cur[v][1]))
    return lanes


def fit_range(lanes, lo_hz, hi_hz):
    """Перенос октавами: общий сдвиг так, чтобы больше нот влезло в [lo, hi];
    что не влезло — своей октавой внутрь. -> (lanes в Гц, сдвиг полутонов)."""
    flat = [n for lane in lanes for (_, _, n) in lane]
    if not flat:
        return [[] for _ in lanes], 0
    best, best_shift = -1, 0
    for octs in range(-6, 7):
        sh = 12 * octs
        fit = sum(1 for n in flat if lo_hz <= note_hz(n + sh) <= hi_hz)
        if fit > best or (fit == best and abs(sh) < abs(best_shift)):
            best, best_shift = fit, sh
    out = []
    for lane in lanes:
        o = []
        for s, e, n in lane:
            hz = note_hz(n + best_shift)
            while hz > hi_hz and hz / 2 >= lo_hz * 0.999:
                hz /= 2
            while hz < lo_hz and hz * 2 <= hi_hz * 1.001:
                hz *= 2
            o.append((s, e, hz))
        out.append(o)
    return out, best_shift


def events(lanes, gap_s=0.03):
    """-> [(t, motor, hz)]: начало ноты — её Гц, конец — 0 (холостые). Паузу
    короче gap_s не ставим: мотор не успеет и только «квакнет»."""
    ev = []
    for m, lane in enumerate(lanes):
        for i, (s, e, hz) in enumerate(lane):
            ev.append((s, m, hz))
            nxt = lane[i + 1][0] if i + 1 < len(lane) else None
            if nxt is None or nxt - e > gap_s:
                ev.append((e, m, 0.0))
    ev.sort(key=lambda x: (x[0], x[1], x[2] != 0.0))
    return ev


# ---------------- предпрослушка ----------------

def render_wav(path, lanes, rate=22050, harmonics=(1.0, 0.35, 0.2)):
    """Голоса — как пищали бы моторы (основной тон + пара гармоник)."""
    end = max((e for lane in lanes for (_, e, _) in lane), default=0.0) + 0.3
    n = int(end * rate)
    buf = [0.0] * n
    for lane in lanes:
        for s, e, hz in lane:
            a, b = int(s * rate), min(n, int(e * rate))
            for i in range(a, b):
                t = (i - a) / rate
                env = min(1.0, t / 0.01, (b - i) / rate / 0.01)
                v = 0.0
                for h, amp in enumerate(harmonics, start=1):
                    v += amp * math.sin(2 * math.pi * hz * h * t)
                buf[i] += env * v
    peak = max(1e-9, max(abs(x) for x in buf))
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(b"".join(struct.pack("<h", int(32000 * x / peak)) for x in buf))


# ---------------- плата ----------------

class Board:
    def __init__(self, port, baud=115200):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.s.dtr = True
        self.lines = []
        self.music = []   # (t, [[target, measured, cmd], …])
        self.errors = []
        self.reply = None
        self.stop = False
        self.th = threading.Thread(target=self._read, daemon=True)
        self.th.start()

    def send(self, line):
        self.s.write(b"\x02" + line.encode() + b"\n")

    def _read(self):
        acc = b""
        while not self.stop:
            acc += self.s.read(512)
            while b"\n" in acc:
                raw, acc = acc.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line.startswith("@music"):
                    self.reply = json.loads(line[6:])
                elif line.startswith("@error"):
                    self.errors.append(line)
                elif line.startswith("@live"):
                    try:
                        m = json.loads(line[5:]).get("music")
                    except ValueError:
                        m = None
                    if m:
                        self.music.append((time.monotonic(), m["v"]))

    def close(self):
        self.stop = True
        self.th.join(0.5)
        self.s.close()


def accuracy(frames, voices):
    """Отчёт: по мотору — медиана ошибки, центов, и доля кадров в пределах 50 центов."""
    rows = []
    for m in range(voices):
        errs = [abs(cents(f[m][1], f[m][0])) for _, f in frames
                if m < len(f) and f[m][0] > 0 and f[m][1] > 0]
        if errs:
            errs.sort()
            rows.append((m, len(errs), errs[len(errs) // 2], sum(1 for e in errs if e <= 50) / len(errs)))
        else:
            rows.append((m, 0, None, None))
    return rows


def play(a, lanes):
    b = Board(a.port)
    try:
        b.send("music off")
        time.sleep(0.2)
        b.send("music on" + (" %g" % a.k if a.k else ""))
        t0 = time.monotonic()
        while b.reply is None and not b.errors and time.monotonic() - t0 < 2.0:
            time.sleep(0.02)
        if b.errors or not b.reply or not b.reply.get("on"):
            print("плата не включила музыку:", b.errors or b.reply)
            return 1
        print("музыка: k=%s, мотор берёт до ~%s Гц (по KV и напряжению)" % (b.reply.get("k"), b.reply.get("max")))
        if a.live:
            b.send("live")
        ev = events(lanes, a.gap)
        cur = [None] * len(lanes)
        # разгон с места: все голоса на холостые до первой ноты
        b.send("note " + " ".join("%d 0" % m for m in range(len(lanes)) if lanes[m]))
        time.sleep(a.lead)
        start = time.monotonic()
        last_keep = start
        i = 0
        while i < len(ev):
            now = time.monotonic() - start
            batch = {}
            while i < len(ev) and ev[i][0] <= now:
                _, m, hz = ev[i]
                cur[m] = batch[m] = hz
                i += 1
            if batch:
                b.send("note " + " ".join("%d %.1f" % (m, h) for m, h in sorted(batch.items())))
                last_keep = time.monotonic()
            elif time.monotonic() - last_keep > 0.3:
                b.send("note " + " ".join("%d %.1f" % (m, h) for m, h in enumerate(cur) if h is not None))
                last_keep = time.monotonic()
            if b.errors:
                print("ошибка платы:", b.errors[-1])
                return 1
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("стоп")
    finally:
        try:
            b.send("note all -1")
            b.send("music off")
            if a.live:
                b.send("live 0")
            time.sleep(0.2)
        finally:
            b.close()
    if a.live:
        print("точность по оборотам (медиана ошибки, центов; доля кадров в пределах 50 центов):")
        for m, n, med, share in accuracy(b.music, len(lanes)):
            if n:
                print("  мотор %d: %d кадров, %.0f ц, %.0f %%" % (m, n, med, 100 * share))
            else:
                print("  мотор %d: нет оборотов (dshot_bidir выключен?)" % m)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("midi")
    ap.add_argument("--voices", type=int, default=4, help="голосов = моторов (4)")
    ap.add_argument("--tracks", help="дорожки через запятую (по умолчанию все)")
    ap.add_argument("--channels", help="каналы 0–15 через запятую")
    ap.add_argument("--drums", action="store_true", help="не выкидывать канал ударных (9)")
    ap.add_argument("--lo", type=float, default=250.0, help="нижняя нота, Гц (холостые мотора)")
    ap.add_argument("--hi", type=float, default=1100.0, help="верхняя нота, Гц (MAX_COMMAND)")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0, help="с секунды")
    ap.add_argument("--len", type=float, default=30.0, help="сколько секунд")
    ap.add_argument("--gap", type=float, default=0.03, help="паузы короче — легато, с")
    ap.add_argument("--k", type=float, default=0.0, help="звук / обороты (0 — motor_poles/2)")
    ap.add_argument("--lead", type=float, default=1.0, help="разгон на холостых до первой ноты, с")
    ap.add_argument("--list", action="store_true", help="показать дорожки и каналы")
    ap.add_argument("--wav", help="записать предпрослушку")
    ap.add_argument("--to-mid", help="записать ноты в MIDI (из mp3/wav — потом играть им же из penv)")
    ap.add_argument("--port", help="порт платы — сыграть")
    ap.add_argument("--live", action="store_true", help="снять точность по оборотам")
    a = ap.parse_args(argv)

    with open(a.midi, "rb") as f:
        data = f.read()
    if data[:4] == b"MThd":
        notes, tempos = parse_midi(data)
    else:
        # звук: расшифровка (нужны numpy и ffmpeg — системный python)
        x = decode_audio(a.midi)
        if a.t_from or a.len:
            x = x[int(a.t_from * 16000):int((a.t_from + a.len) * 16000)]
            a.t_from = 0.0
        notes, tempos = transcribe(x, voices=a.voices), []
        print("расшифровано нот: %d" % len(notes))
    if a.to_mid:
        write_midi(a.to_mid, notes)
        print("MIDI:", a.to_mid)
    if a.list:
        seen = {}
        for s, e, n, t, ch in notes:
            seen.setdefault((t, ch), []).append(n)
        for (t, ch), ns in sorted(seen.items()):
            print("дорожка %d канал %d: %d нот, %s…%s" % (t, ch, len(ns), min(ns), max(ns)))
        if tempos:
            print("темп: %.0f bpm" % tempos[0][1])
        return 0
    tracks = {int(x) for x in a.tracks.split(",")} if a.tracks else None
    channels = {int(x) for x in a.channels.split(",")} if a.channels else None
    sel = pick(notes, tracks, channels, a.drums)
    sel = [(s - a.t_from, min(e, a.t_from + a.len) - a.t_from, n, t, ch)
           for s, e, n, t, ch in sel if a.t_from <= s < a.t_from + a.len]
    lanes, shift = fit_range(allocate(sel, a.voices), a.lo, a.hi)
    print("нот: %d, по голосам: %s, перенос %+d полутонов" % (len(sel), [len(x) for x in lanes], shift))
    if a.wav:
        render_wav(a.wav, lanes)
        print("предпрослушка:", a.wav)
    if a.port:
        return play(a, lanes)
    return 0


if __name__ == "__main__":
    sys.exit(main())
