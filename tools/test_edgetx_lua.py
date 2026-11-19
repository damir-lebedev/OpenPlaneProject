#!/usr/bin/env python3
"""
Тесты Lua-скриптов пульта (tools/edgetx/SCRIPTS) на ПК — Lua 5.2, как в EdgeTX,
через пакет lupa (pip install lupa; нет пакета — тесты пропускаются).

    python tools/test_edgetx_lua.py

API EdgeTX (getTime, getValue, getTxGPS, io.*, lcd.*, playTone...) — заглушки
ниже: время двигает тест, телеметрию задаёт тест, SD — таблица в памяти.
Проверяется логика (поиск упавшего: запись точки, фикс пропал, стрелка, писк)
и что каждая страница рисуется на экранах 128×64 и 480×272 без ошибок.
MSP через CRSF здесь не проверяется — только на пульте.
"""

import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "edgetx", "SCRIPTS")

try:
    from lupa import lua52
except ImportError:  # pragma: no cover
    lua52 = None

STUBS = r"""
T = 0
TEL = { }            -- getValue(name)
RSSI = 0
TXGPS = nil
SD = { }             -- путь -> текст
TONES = { }
TEXTS = { }
LCD_W, LCD_H = 128, 64
SMLSIZE, INVERS, BLINK, RIGHT, SOLID, PLAY_NOW = 1, 2, 4, 8, 0, 16
EVT_VIRTUAL_ENTER, EVT_VIRTUAL_EXIT = 101, 102
EVT_VIRTUAL_NEXT_PAGE, EVT_VIRTUAL_PREV_PAGE = 103, 104
EVT_VIRTUAL_NEXT, EVT_VIRTUAL_PREV = 105, 106

function getTime() return T end
function getValue(n) local v = TEL[n]; if v == nil then return 0 end; return v end
function getRSSI() return RSSI, 30, 25 end
function getTxGPS() return TXGPS end
function getDateTime() return { year = 2026, mon = 11, day = 19, hour = 12, min = 30, sec = math.floor(T / 100) % 60 } end
function playTone(f, d, p, fl) TONES[#TONES + 1] = f end
function crossfireTelemetryPush() return true end
function crossfireTelemetryPop() return nil end
model = { getInfo = function() return { name = "Quad7" } end }

local function check(x, y)
  assert(type(x) == "number" and type(y) == "number", "координаты не числа")
  assert(x == x and y == y, "NaN на экране")
end
lcd = {
  clear = function() TEXTS = { } end,
  drawText = function(x, y, s, f) check(x, y); assert(type(s) == "string", "drawText: не строка"); TEXTS[#TEXTS + 1] = s end,
  drawLine = function(a, b, c, d) check(a, b); check(c, d) end,
  drawCircle = function(x, y, r) check(x, y) end,
  drawFilledRectangle = function(x, y, w, h) check(x, y) end,
}

io = {
  open = function(path, mode)
    if mode == "r" then
      if SD[path] == nil then return nil end
      return { path = path, pos = 1 }
    end
    SD[path] = ""
    return { path = path }
  end,
  read = function(f, n) local s = string.sub(SD[f.path], f.pos, f.pos + n - 1); f.pos = f.pos + #s; return s end,
  write = function(f, ...) for _, s in ipairs({ ... }) do SD[f.path] = SD[f.path] .. s end end,
  close = function(f) end,
}

function loadScript(path)
  local src = SRC[path]
  assert(src, "нет скрипта " .. path)
  return assert(load(src, "@" .. path))
end

function screen()
  return table.concat(TEXTS, "|")
end
"""


@unittest.skipIf(lua52 is None, "нет пакета lupa (pip install lupa)")
class EdgeTxLua(unittest.TestCase):
    def setUp(self):
        self.lua = lua52.LuaRuntime(unpack_returned_tuples=True)
        self.lua.execute(STUBS)
        src = self.lua.eval("{}")
        for sub in ("OPLANE", "TOOLS", "TELEMETRY"):
            for name in os.listdir(os.path.join(SCRIPTS, sub)):
                with open(os.path.join(SCRIPTS, sub, name), encoding="utf-8") as f:
                    src["/SCRIPTS/%s/%s" % (sub, name)] = f.read()
        self.lua.globals().SRC = src
        self.run_lua("find = loadScript('/SCRIPTS/OPLANE/find.lua')()")

    def run_lua(self, code):
        return self.lua.execute(code)

    def fly(self, lat, lon, sats, steps=1, dt=100):
        g = self.lua.globals()
        g.RSSI = 80
        self.run_lua("TEL.GPS = { lat = %r, lon = %r }; TEL.Sats = %d; TEL.Alt = 12" % (lat, lon, sats))
        for _ in range(steps):
            g.T = g.T + dt
            self.run_lua("find.update()")

    def sd(self):
        return self.lua.eval("SD['/SCRIPTS/OPLANE/lastgps.txt']")

    def test_distance_bearing_and_compass(self):
        m, deg = self.lua.eval("find.distanceBearing({lat=55, lon=37}, {lat=55.001, lon=37})")
        self.assertAlmostEqual(m, 111.2, delta=0.5)
        self.assertAlmostEqual(deg, 0, delta=0.01)
        m, deg = self.lua.eval("find.distanceBearing({lat=55, lon=37}, {lat=55, lon=36.998})")
        self.assertAlmostEqual(m, 127.6, delta=1)
        self.assertAlmostEqual(deg, 270, delta=0.1)
        self.assertEqual(self.lua.eval("find.distanceBearing({lat=1, lon=1}, {lat=1, lon=1})"), (0, 0))
        words = [self.lua.eval("find.compass(%d)" % d) for d in (0, 44, 46, 90, 180, 225, 359, -90, 400)]
        self.assertEqual(words, ["N", "NE", "NE", "E", "S", "SW", "N", "W", "NE"])

    def test_atan2_fallback_matches(self):
        # EdgeTX может собрать math без atan2 — запасной путь даёт то же
        self.run_lua("math.atan2 = nil; find2 = loadScript('/SCRIPTS/OPLANE/find.lua')()")
        for b in ((55.001, 37), (55, 37.001), (54.999, 37), (55, 36.999), (55.001, 37.001), (54.999, 36.999)):
            _, d1 = self.lua.eval("find.distanceBearing({lat=55, lon=37}, {lat=%r, lon=%r})" % b)
            _, d2 = self.lua.eval("find2.distanceBearing({lat=55, lon=37}, {lat=%r, lon=%r})" % b)
            self.assertAlmostEqual(d1, d2, places=6)

    def test_encode_decode_roundtrip(self):
        text = self.lua.eval("find.encode({lat=55.1234567, lon=-37.5, sats=9, alt=12.4, time='2026-11-19 12:30:00', fixLost=true, model='Quad7'}, {lat=55.1, lon=-37.4})")
        self.assertIn("fixlost=1\n", text)
        self.assertIn("lat=55.1234567\n", text)
        self.lua.globals().TXT = text
        self.run_lua("P, S = find.decode(TXT)")
        p = self.lua.globals().P
        self.assertEqual((p.lat, p.lon, p.sats, p.alt, p.model, p.fixLost), (55.1234567, -37.5, 9, 12, "Quad7", True))
        self.assertEqual(self.lua.eval("S.lon"), -37.4)
        self.assertIsNone(self.lua.eval("find.decode('lat=0\\nlon=0\\n')"))
        self.assertIsNone(self.lua.eval("find.decode('garbage')"))
        self.assertIsNone(self.lua.eval("find.decode(nil)"))

    def test_no_link_no_point(self):
        self.run_lua("TEL.GPS = { lat = 55, lon = 37 }; TEL.Sats = 9; RSSI = 0; find.update()")
        self.assertIsNone(self.lua.eval("find.last()"))
        self.assertIsNone(self.sd())
        # GPS-датчика ещё нет — getValue даёт 0
        self.run_lua("TEL.GPS = nil; RSSI = 80; find.update()")
        self.assertIsNone(self.lua.eval("find.last()"))

    def test_flight_saves_point_and_fix_lost(self):
        self.fly(55.0, 37.0, 9)
        self.assertIn("lat=55.0000000", self.sd())
        self.assertIn("slat=55.0000000", self.sd())
        # дрожание на месте — файл не переписывается
        first = self.sd()
        self.fly(55.00001, 37.0, 9, steps=10)
        self.assertEqual(self.sd(), first)
        # улетел на 220 м к северу: пишется, но не чаще раза в 5 с
        self.fly(55.002, 37.0, 9, dt=100)
        self.assertIn("lat=55.0020000", self.sd())
        self.fly(55.003, 37.0, 9, dt=100)
        self.assertNotIn("lat=55.0030000", self.sd())
        self.fly(55.003, 37.0, 9, steps=5, dt=100)
        self.assertIn("lat=55.0030000", self.sd())
        # перевернулся: прошивка шлёт ту же точку, спутников мало
        self.fly(55.003, 37.0, 3, steps=6)
        self.assertIn("fixlost=1", self.sd())
        self.assertTrue(self.lua.eval("find.last().fixLost"))
        # старт остался первым фиксом
        self.assertIn("slat=55.0000000", self.sd())

    def test_point_survives_radio_restart(self):
        self.fly(55.003, 37.0, 3)
        self.fly(55.003, 37.0, 3, steps=6)
        saved = self.sd()
        self.run_lua("RSSI = 0; find = loadScript('/SCRIPTS/OPLANE/find.lua')(); find.update()")
        p = self.lua.eval("find.last()")
        self.assertEqual((p.lat, p.lon, p.fixLost), (55.003, 37.0, True))
        self.assertEqual(self.lua.eval("find.age()"), "11-19 12:30")
        self.assertEqual(self.sd(), saved)
        self.run_lua("find.draw(9, 8, SMLSIZE)")
        self.assertIn("Fix lost: it is HERE", self.lua.eval("screen()"))

    def test_pilot_gps_arrow_and_beep(self):
        self.fly(55.001, 37.0, 9)
        g = self.lua.globals()
        g.LCD_W, g.LCD_H = 128, 64
        self.run_lua("TXGPS = { fix = true, lat = 55.0, lon = 37.0, numsat = 8 }; find.update(); find.draw(9, 8, SMLSIZE)")
        self.assertIn("You: 111m N", self.lua.eval("screen()"))
        # пилот идёт на восток — коптер слева (стрелка ~270°): проверяем через beepPlan и текст
        self.run_lua("TXGPS = { fix = true, lat = 55.0, lon = 37.0001, numsat = 8 }; find.update()")
        self.run_lua("find.beep = true; T = T + 1000; find.draw(9, 8, SMLSIZE)")
        self.assertEqual(len(self.lua.eval("TONES")), 1)
        # писк чаще ближе
        far = self.lua.eval("find.beepPlan(100, nil)")
        near = self.lua.eval("find.beepPlan(5, nil)")
        self.assertGreater(far[0], near[0])
        self.assertGreater(near[1], far[1])
        # без GPS пульта — по силе сигнала
        weak = self.lua.eval("find.beepPlan(nil, 0.1)")
        strong = self.lua.eval("find.beepPlan(nil, 0.9)")
        self.assertGreater(weak[0], strong[0])
        self.assertGreater(strong[1], weak[1])
        self.assertIsNone(self.lua.eval("find.beepPlan(nil, nil)"))

    def test_signal_prefers_dbm(self):
        self.run_lua("TEL['1RSS'] = -75; RSSI = 50")
        s, dbm = self.lua.eval("find.signal()")
        self.assertAlmostEqual(s, 0.5, places=3)
        self.assertEqual(dbm, -75)
        self.run_lua("TEL['1RSS'] = nil")
        s, dbm = self.lua.eval("find.signal()")
        self.assertAlmostEqual(s, 0.5)
        self.assertIsNone(dbm)

    def test_tools_pages_render_and_find_toggles_beep(self):
        for w, h in ((128, 64), (480, 272)):
            g = self.lua.globals()
            g.LCD_W, g.LCD_H = w, h
            self.fly(55.001, 37.0, 9)
            self.run_lua("tool = loadScript('/SCRIPTS/TOOLS/OpenPlane.lua')(); tool.init()")
            for _ in range(5):
                self.assertEqual(self.lua.eval("tool.run(0)"), 0)
                self.run_lua("tool.run(EVT_VIRTUAL_NEXT_PAGE)")
            # пятая страница — Find (после 5 перелистываний вернулись на первую)
            for _ in range(4):
                self.run_lua("tool.run(EVT_VIRTUAL_NEXT_PAGE)")
            self.assertIn("ENT: beep on/off", self.lua.eval("screen()"))
            self.assertIn("Beep OFF", self.lua.eval("screen()"))
            self.run_lua("tool.run(EVT_VIRTUAL_ENTER)")
            self.assertIn("Beep ON", self.lua.eval("screen()"))
            self.assertEqual(self.lua.eval("tool.run(EVT_VIRTUAL_EXIT)"), 1)

    def test_telemetry_screen_shows_saved_point(self):
        self.run_lua("tel = loadScript('/SCRIPTS/TELEMETRY/oplan.lua')(); tel.init()")
        self.run_lua("tel.run(0)")
        self.assertNotIn("GPS saved", self.lua.eval("screen()"))
        self.run_lua("RSSI = 80; TEL.GPS = { lat = 55, lon = 37 }; TEL.Sats = 9; T = T + 100; tel.background(); tel.run(0)")
        self.assertIn("GPS saved 0s ago", self.lua.eval("screen()"))
        self.assertIn("lat=55.0000000", self.sd())


if __name__ == "__main__":
    unittest.main()
