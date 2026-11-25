"""Тесты tools/msp_probe.py: кадры v1/v2 туда и обратно, текст консоли между
кадрами, итог «подключится / нет» на поддельной плате.

    python -m unittest tools.test_msp_probe
"""

import io
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import msp_probe  # noqa: E402


def reply(cmd, payload, error=False):
    """Ответ платы — тем же кодированием, что запрос, с '>' или '!' (направление в сумму не входит)."""
    frame = bytearray(msp_probe.encode(cmd, payload))
    frame[2] = ord("!" if error else ">")
    return bytes(frame)


def board_info():
    p = b"OPLN" + b"\x00\x00" + b"\x00" + b"\x09"
    for s in (b"OPENPLANE_F405", b"speedybee-f405v4-quad", b"OPNP"):
        p += bytes([len(s)]) + s
    return p + bytes(32) + b"\x01\x02" + b"\xe8\x03" + bytes(4) + b"\x00\x00"


class FakePort:
    """Плата: отвечает по таблице, неизвестное — '!', silent — молчит; между кадрами — текст."""

    def __init__(self, answers, silent=()):
        self.answers = answers
        self.silent = set(silent)
        self.pending = bytearray()
        self.reader = msp_probe.FrameReader()

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, data):
        # запрос разбираем как кадр: направление '<' читатель тоже принимает
        for cmd, _, _ in self.reader.feed(data):
            if cmd in self.silent:
                continue
            self.pending += b"Log: hello $\r\n"   # консоль что-то печатает
            if cmd in self.answers:
                self.pending += reply(cmd, self.answers[cmd])
            else:
                self.pending += reply(cmd, b"", error=True)

    def read(self, n):
        chunk = bytes(self.pending[:n])
        del self.pending[:n]
        return chunk


GOOD = {
    1: b"\x00\x01\x2e", 2: b"BTFL", 3: b"\x04\x05\x00", 5: b"Nov 25 202612:00:00abcdef1",
    4: board_info(), 101: bytes(11), 160: bytes(12), 0x3006: b"\x02\x05SEVEN",
}


class EncodeDecode(unittest.TestCase):
    def test_v1_and_v2_roundtrip_through_noise(self):
        r = msp_probe.FrameReader()
        stream = b"text$ $Q" + reply(108, b"\x01\x00\x02\x00\x03\x00") + b"\r\nmore" + reply(0x3006, b"\x02\x01A")
        frames = []
        for i in range(0, len(stream), 3):   # кусками, как из порта
            frames += r.feed(stream[i:i + 3])
        self.assertEqual([(108, ">", b"\x01\x00\x02\x00\x03\x00"), (0x3006, ">", b"\x02\x01A")], frames)
        self.assertGreater(r.noise, 0)

    def test_jumbo_v1(self):
        payload = bytes(range(256)) * 2
        frame = bytearray(b"$M>\xff\x74") + bytes([len(payload) & 0xFF, len(payload) >> 8]) + payload
        x = 0xFF ^ 0x74 ^ (len(payload) & 0xFF) ^ (len(payload) >> 8)
        for b in payload:
            x ^= b
        frame.append(x)
        self.assertEqual([(0x74, ">", payload)], msp_probe.FrameReader().feed(bytes(frame)))

    def test_bad_checksum_is_skipped(self):
        f = bytearray(reply(1, b"\x00\x01\x2e"))
        f[-1] ^= 0xFF
        self.assertEqual([], msp_probe.FrameReader().feed(bytes(f)))


class Probe(unittest.TestCase):
    def test_connects_when_identity_answers(self):
        out = io.StringIO()
        self.assertTrue(msp_probe.run(FakePort(GOOD), timeout=0.05, out=out))
        text = out.getvalue()
        self.assertIn("подключится: да", text)
        self.assertIn("speedybee-f405v4-quad", text)
        self.assertIn("нет команды (!)", text)   # вкладки, которых нет, — не мешают

    def test_silence_or_short_reply_breaks_connection(self):
        out = io.StringIO()
        self.assertFalse(msp_probe.run(FakePort(GOOD, silent={160}), timeout=0.05, out=out))
        self.assertIn("МОЛЧИТ", out.getvalue())
        short = dict(GOOD)
        short[4] = board_info()[:40]
        out = io.StringIO()
        self.assertFalse(msp_probe.run(FakePort(short), timeout=0.05, out=out))
        self.assertIn("КОРОТКО", out.getvalue())


if __name__ == "__main__":
    unittest.main()
