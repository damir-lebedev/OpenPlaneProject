"""Тесты tools/motor_music.py: разбор MIDI (темп, running status, note-on 0),
раскладка на голоса, перенос октавами, события и предпрослушка.

    python -m unittest tools.test_motor_music
"""

import os
import struct
import sys
import tempfile
import unittest
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import motor_music as mm  # noqa: E402


def varlen(v):
    out = [v & 0x7F]
    v >>= 7
    while v:
        out.insert(0, 0x80 | (v & 0x7F))
        v >>= 7
    return bytes(out)


def track(events):
    """events: [(delta, bytes)] -> MTrk."""
    body = b"".join(varlen(d) + e for d, e in events) + b"\x00\xff\x2f\x00"
    return b"MTrk" + struct.pack(">I", len(body)) + body


def midi(tracks, div=480, fmt=1):
    return b"MThd" + struct.pack(">IHHH", 6, fmt, len(tracks), div) + b"".join(tracks)


class MidiTest(unittest.TestCase):
    def test_tempo_and_running_status(self):
        # темп 120 bpm (500000 мкс), потом 60 bpm; четверть = 480 тиков
        t0 = track([(0, b"\xff\x51\x03" + (500000).to_bytes(3, "big")),
                    (480, b"\xff\x51\x03" + (1000000).to_bytes(3, "big"))])
        t1 = track([(0, b"\x90\x45\x64"),      # A4 on
                    (480, b"\x45\x00"),        # running status, velocity 0 = off (0.5 с)
                    (0, b"\x48\x64"),          # C5 on
                    (480, b"\x80\x48\x40"),    # off через 1 с (темп 60)
                    (0, b"\x99\x24\x64"),      # ударные
                    (240, b"\x89\x24\x40")])
        notes, tempos = mm.parse_midi(midi([t0, t1]))
        self.assertEqual(len(tempos), 2)
        n = [x for x in notes if x[4] != 9]
        self.assertEqual(len(n), 2)
        self.assertAlmostEqual(n[0][0], 0.0)
        self.assertAlmostEqual(n[0][1], 0.5)
        self.assertEqual(n[0][2], 69)
        self.assertAlmostEqual(n[1][0], 0.5)
        self.assertAlmostEqual(n[1][1], 1.5)
        self.assertEqual(len(mm.pick(notes)), 2)              # ударные выкинуты
        self.assertEqual(len(mm.pick(notes, drums=True)), 3)

    def test_not_midi(self):
        with self.assertRaises(ValueError):
            mm.parse_midi(b"RIFF....")


class ArrangeTest(unittest.TestCase):
    def test_chord_splits_top_down_and_drops_extra(self):
        # аккорд из 5 нот на 2 с, сверху мелодия меняется на 1 с
        chord = [(0.0, 2.0, n, 0, 0) for n in (48, 52, 55, 60)]
        melody = [(0.0, 1.0, 72, 0, 0), (1.0, 2.0, 74, 0, 0)]
        lanes = mm.allocate(sorted(chord + melody), 4)
        top = [n for lane in lanes for (_, _, n) in lane]
        self.assertNotIn(48, top)                  # пятая (нижняя) нота не влезла
        mel = [lane for lane in lanes if any(n in (72, 74) for (_, _, n) in lane)]
        self.assertEqual(len(mel), 1)              # мелодия — в одном голосе
        self.assertEqual([n for (_, _, n) in mel[0]], [72, 74])
        for lane in lanes:                         # ноты голоса не перекрываются
            for a, b in zip(lane, lane[1:]):
                self.assertLessEqual(a[1], b[0] + 1e-9)

    def test_fit_range_shifts_by_octaves(self):
        lanes = [[(0.0, 1.0, 45), (1.0, 2.0, 57)]]   # A2, A3 — ниже холостых
        out, shift = mm.fit_range(lanes, 250.0, 1100.0)
        self.assertEqual(shift % 12, 0)
        for _, _, hz in out[0]:
            self.assertTrue(250.0 <= hz <= 1100.0, hz)
        # интервал октавы между нотами сохранён
        self.assertAlmostEqual(out[0][1][2] / out[0][0][2], 2.0, places=3)

    def test_events_legato_and_rests(self):
        lanes = [[(0.0, 0.49, 440.0), (0.5, 1.0, 494.0), (1.5, 2.0, 523.0)]]
        ev = mm.events(lanes, gap_s=0.03)
        self.assertEqual([e[2] for e in ev], [440.0, 494.0, 0.0, 523.0, 0.0])   # 10 мс — легато

    def test_accuracy(self):
        frames = [(0, [[440, 440, 0.1], [0, 0, 0]]), (1, [[440, 466, 0.1], [0, 0, 0]])]
        rows = mm.accuracy(frames, 2)
        self.assertEqual(rows[0][1], 2)
        self.assertEqual(rows[0][3], 0.5)          # 466 Гц — почти полутон мимо
        self.assertEqual(rows[1][1], 0)

    def test_live_voices(self):
        lv = mm.LiveVoices(2, 250.0, 1100.0)
        self.assertEqual(lv.on(69), [(0, 440.0)])
        v, hz = lv.on(45)[0]                        # A2 — октавами внутрь диапазона
        self.assertEqual(v, 1)
        self.assertAlmostEqual(hz, 440.0)
        self.assertEqual(lv.on(40), [])             # ниже всех занятых — не вытесняет
        self.assertEqual(lv.on(76)[0][0], 1)        # выше — вытесняет самую низкую (45)
        self.assertEqual(lv.off(45), [])            # её уже нет
        self.assertEqual(lv.off(69), [(0, 0.0)])
        self.assertEqual([v for v, _ in lv.current()], [1])

    @unittest.skipUnless(sys.platform == "win32", "только Windows")
    def test_midi_inputs_lists(self):
        self.assertIsInstance(mm.midi_inputs(), list)

    def test_lanes_to_track_roundtrip_and_shortening(self):
        import motor_tracks as mt
        lanes = [[(0.0, 0.5, 69), (0.6, 1.0, 72)], [(0.2, 1.0, 57)]]
        data, secs = mm.lanes_to_track(lanes, unit_ms=20)
        unit, voices = mt.decode(data)
        self.assertEqual(unit, 20)
        self.assertEqual(voices[0], [(69, 25), (0, 5), (72, 20)])
        self.assertEqual(voices[1], [(0, 10), (57, 40)])
        self.assertAlmostEqual(secs, 1.0)
        # длинный трек не влезает в 1024 байта — укорачивается
        many = [[(k * 0.05, k * 0.05 + 0.04, 60 + k % 12) for k in range(2000)]]
        data, secs = mm.lanes_to_track(many)
        self.assertLessEqual(len(data), mm.TRACK_MAX)
        self.assertLess(secs, 100.0)
        mt.decode(data)

    def test_builtin_tracks_and_header(self):
        import motor_tracks as mt
        for name, _, data in mt.built():
            unit, voices = mt.decode(data)
            self.assertTrue(1 <= len(voices) <= 4, name)
            lens = {sum(d for _, d in v) for v in voices}
            self.assertEqual(len(lens), 1, "%s: голоса разной длины %s" % (name, lens))
        with open(mt.HEADER, encoding="utf-8") as f:
            self.assertEqual(f.read(), mt.header_text(), "MotorTracks.h разошёлся — python tools/motor_tracks.py")

    def test_write_midi_roundtrip(self):
        notes = [(0.0, 0.5, 69, 0, 0), (0.25, 1.0, 76, 0, 0)]
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.mid")
            mm.write_midi(p, notes)
            with open(p, "rb") as f:
                back, _ = mm.parse_midi(f.read())
        self.assertEqual([n[2] for n in back], [69, 76])
        self.assertAlmostEqual(back[1][1], 1.0, places=2)

    @unittest.skipUnless(__import__("importlib").util.find_spec("numpy"), "нет numpy")
    def test_transcribe_two_tones(self):
        import numpy as np
        rate = 16000
        t = np.arange(int(1.0 * rate)) / rate
        tone = lambda hz: sum(0.6 ** h * np.sin(2 * np.pi * hz * (h + 1) * t) for h in range(3))   # noqa: E731
        x = np.concatenate([tone(440.0) + 0.7 * tone(mm.note_hz(60)), tone(mm.note_hz(72))]).astype(np.float32)
        notes = mm.transcribe(x, rate=rate, voices=2)
        first = {n[2] for n in notes if n[0] < 0.5}
        self.assertEqual(first, {69, 60})
        late = {n[2] for n in notes if n[0] > 0.8}
        self.assertIn(72, late)

    def test_wav_preview(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.wav")
            mm.render_wav(p, [[(0.0, 0.2, 440.0)], [(0.1, 0.3, 660.0)]], rate=8000)
            with wave.open(p) as w:
                self.assertEqual(w.getframerate(), 8000)
                self.assertAlmostEqual(w.getnframes() / 8000, 0.6, places=2)


if __name__ == "__main__":
    unittest.main()
