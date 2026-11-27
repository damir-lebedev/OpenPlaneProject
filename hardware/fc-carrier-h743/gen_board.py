# Генератор односторонней платы-носителя под DevEBox H743 (формат KiCad 6).
# Детали THT — сверху (голый текстолит), медь и SMD — снизу (B.Cu).
# Вид — сверху, y вниз, мм. План — docs/FC_BOARD_H743.md.
import uuid, json, sys, os

OUT = sys.argv[1] if len(sys.argv) > 1 else "openplane_carrier"

def U():
    return str(uuid.uuid4())

# ---------------- геометрия DevEBox ----------------
P = 2.54
X2o, X2i, X3i, X3o = 24.0, 26.54, 59.56, 62.10
Y1 = 10.0
def ry(r): return round(Y1 + (r - 1) * P, 3)
def gp(r): return round(ry(r) + P / 2, 3)        # промежуток между рядами r и r+1
def j2(n): return (X2o if n % 2 else X2i, ry((n + 1) // 2))
def j3(n): return (X3i if n % 2 else X3o, ry((n + 1) // 2))

W, H = 90.0, 98.0

# ---------------- цепи ----------------
nets = {"": 0}
def N(name):
    if name not in nets:
        nets[name] = len(nets)
    return name

J2NET = {1: "+5VD", 2: "+5VD", 3: "+3V3B", 4: "+3V3B", 5: "GND", 6: "GND",
         7: "GPS_RX", 8: "GPS_TX", 13: "AUX1", 14: "RUD", 27: "TEL_RX", 30: "TEL_TX",
         33: "U2_RX", 34: "U2_TX", 41: "SDA_B", 42: "SCL_B"}
J3NET = {1: "+3V3", 2: "+3V3", 3: "GND", 4: "GND", 7: "MOSI", 8: "MISO", 9: "SCK", 10: "CS_IMU",
         11: "SDA_A", 12: "SCL_A", 13: "BUZ_IO", 15: "LIGHT_IO", 19: "AUX2", 20: "RC_TX", 21: "RC_RX",
         23: "PYRO_IO", 31: "ESC", 32: "ELE", 33: "AILR", 34: "AILL", 37: "CURR_S", 38: "VBAT_S"}
J2NAME = {1:"5V",2:"5V",3:"3V3",4:"3V3",5:"GND",6:"GND",7:"PD9",8:"PD8",9:"PD11",10:"PD10",11:"PD13",12:"PD12",
          13:"PD15",14:"PD14",15:"PC7",16:"PC6",17:"PC9",18:"PC8",19:"PA9",20:"PA8",21:"PA11",22:"PA10",
          23:"PA15",24:"PA12",25:"PC11",26:"PC10",27:"PD0",28:"PC12",29:"PD2",30:"PD1",31:"PD4",32:"PD3",
          33:"PD6",34:"PD5",35:"PB3",36:"PD7",37:"PB5",38:"PB4",39:"PB7",40:"PB6",41:"PB9",42:"PB8",43:"PE1",44:"PE0"}
J3NAME = {1:"3V3",2:"3V3",3:"GND",4:"GND",5:"VDDA",6:"VREF+",7:"PB15",8:"PB14",9:"PB13",10:"PB12",11:"PB11",
          12:"PB10",13:"PE15",14:"PE14",15:"PE13",16:"PE12",17:"PE11",18:"PE10",19:"PE9",20:"PE8",21:"PE7",
          22:"PB2",23:"PB1",24:"PB0",25:"PC5",26:"PC4",27:"PA7",28:"PA6",29:"PA5",30:"PA4",31:"PA3",32:"PA2",
          33:"PA1",34:"PA0",35:"PC3",36:"PC2",37:"PC1",38:"PC0",39:"PC13",40:"PE6",41:"PE5",42:"PE4",43:"PE3",44:"PE2"}

# ---------------- объекты ----------------
fps, segs, texts, lines = [], [], [], []

def pad_sx(num, x, y, kind, shape, sx, sy, net, drill=None, layer_side="B"):
    if net:
        N(net)
    s = f'(pad "{num}" {kind} {shape} (at {x:.4f} {y:.4f}) (size {sx} {sy})'
    if kind == "thru_hole":
        s += f' (drill {drill}) (layers "*.Cu" "*.Mask")'
    elif kind == "np_thru_hole":
        s += f' (drill {drill}) (layers "*.Cu" "*.Mask")'
    else:
        s += f' (layers "{layer_side}.Cu" "{layer_side}.Paste" "{layer_side}.Mask")'
    if net:
        s += f' (net {nets[net]} "{net}")'
    s += ")"
    return s

class FP:
    def __init__(self, ref, value, x, y, side="F", lib="OpenPlane"):
        self.ref, self.value, self.x, self.y, self.side = ref, value, x, y, side
        self.pads, self.gfx, self.lib = [], [], lib
    def pad(self, num, ax, ay, kind, shape, sx, sy, net="", drill=None):
        # ax, ay — абсолютные координаты платы
        self.pads.append(pad_sx(num, ax - self.x, ay - self.y, kind, shape, sx, sy, net, drill, self.side))
        return (ax, ay)
    def line(self, x1, y1, x2, y2, layer=None, w=0.12):
        layer = layer or f"{self.side}.SilkS"
        self.gfx.append(f'(fp_line (start {x1-self.x:.3f} {y1-self.y:.3f}) (end {x2-self.x:.3f} {y2-self.y:.3f}) (layer "{layer}") (width {w}))')
    def rect(self, x1, y1, x2, y2, layer=None):
        self.line(x1, y1, x2, y1, layer); self.line(x2, y1, x2, y2, layer)
        self.line(x2, y2, x1, y2, layer); self.line(x1, y2, x1, y1, layer)
    def circle(self, cx, cy, r, layer=None):
        layer = layer or f"{self.side}.SilkS"
        self.gfx.append(f'(fp_circle (center {cx-self.x:.3f} {cy-self.y:.3f}) (end {cx-self.x+r:.3f} {cy-self.y:.3f}) (layer "{layer}") (width 0.12))')
    def text(self, t, ax, ay, size=1.0, layer=None):
        layer = layer or f"{self.side}.SilkS"
        mir = " (justify mirror)" if layer.startswith("B.") else ""
        self.gfx.append(f'(fp_text user "{t}" (at {ax-self.x:.3f} {ay-self.y:.3f}) (layer "{layer}") (effects (font (size {size} {size}) (thickness {size*0.15:.3f})){mir}))')
    def sx(self):
        L = self.side
        mir = " (justify mirror)" if L == "B" else ""
        out = [f'(footprint "{self.lib}:{self.value}" (layer "{L}.Cu") (tedit 0) (tstamp {U()})',
               f'  (at {self.x:.4f} {self.y:.4f})',
               f'  (fp_text reference "{self.ref}" (at 0 0) (layer "{L}.Fab") {"hide" if L == "B" else ""} (effects (font (size 0.8 0.8) (thickness 0.12)){mir}))',
               f'  (fp_text value "{self.value}" (at 0 0) (layer "{L}.Fab") hide (effects (font (size 0.8 0.8) (thickness 0.12)){mir}))']
        if L == "B":   # номинал — на B.Fab, для сборочной картинки
            v = self.value.split("_")[-1]
            out.append(f'  (fp_text user "{self.ref} {v}" (at 0 0) (layer "B.Fab") (effects (font (size 0.5 0.5) (thickness 0.08)) (justify mirror)))')
        out += ["  " + g for g in self.gfx] + ["  " + p for p in self.pads] + [")"]
        return "\n".join(out)

def track(net, w, *pts):
    N(net)
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        if abs(x1 - x2) < 1e-6 and abs(y1 - y2) < 1e-6:
            continue
        segs.append((x1, y1, x2, y2, w, net))

def silk(t, x, y, size=1.0, layer="F.SilkS", angle=0):
    texts.append((t, x, y, size, layer, angle))

def sline(x1, y1, x2, y2, layer="F.SilkS", w=0.15):
    lines.append((x1, y1, x2, y2, layer, w))

# ---------------- библиотека футпринтов ----------------
def socket2x22(ref, col_out, col_in, odd_outer, netmap, namemap, label_side):
    fp = FP(ref, "Socket_2x22_DevEBox", (col_out + col_in) / 2, ry(11.5))
    for n in range(1, 45):
        r = (n + 1) // 2
        outer = (n % 2 == 1) == odd_outer
        x = col_out if outer else col_in
        shape = "rect" if n == 1 else "oval"
        fp.pad(str(n), x, ry(r), "thru_hole", shape, 2.0, 1.6, netmap.get(n, ""), 0.9)
    x1, x2 = min(col_out, col_in) - 1.3, max(col_out, col_in) + 1.3
    fp.rect(x1, ry(1) - 1.3, x2, ry(22) + 1.3)
    return fp

def xh(ref, pins, side_dir, nets_, label=None, body_dir=1):
    """pins — список абсолютных (x, y); body по y: -2.35/+3.4 (горизонтальный ряд) или по x."""
    fp = FP(ref, f"JST_XH_{len(pins)}", sum(p[0] for p in pins) / len(pins), sum(p[1] for p in pins) / len(pins))
    for i, ((x, y), n) in enumerate(zip(pins, nets_)):
        fp.pad(str(i + 1), x, y, "thru_hole", "rect" if i == 0 else "circle", 1.8, 1.8, n, 1.0)
    xs = [p[0] for p in pins]; ys = [p[1] for p in pins]
    if side_dir == "h":   # ряд вдоль x
        a, b = (-2.35, 3.4) if body_dir > 0 else (-3.4, 2.35)
        fp.rect(min(xs) - 2.45, ys[0] + a, max(xs) + 2.45, ys[0] + b)
    else:                 # ряд вдоль y
        a, b = (-2.35, 3.4) if body_dir > 0 else (-3.4, 2.35)
        fp.rect(xs[0] + a, min(ys) - 2.45, xs[0] + b, max(ys) + 2.45)
    fps.append(fp)
    return fp

def pinsocket(ref, pins, nets_):
    fp = FP(ref, f"PinSocket_1x{len(pins)}", sum(p[0] for p in pins) / len(pins), sum(p[1] for p in pins) / len(pins))
    for i, ((x, y), n) in enumerate(zip(pins, nets_)):
        fp.pad(str(i + 1), x, y, "thru_hole", "rect" if i == 0 else "circle", 1.7, 1.7, n, 0.9)
    xs = [p[0] for p in pins]; ys = [p[1] for p in pins]
    fp.rect(min(xs) - 1.27, min(ys) - 1.27, max(xs) + 1.27, max(ys) + 1.27)
    fps.append(fp)
    return fp

def testpad(ref, x, y, net, label):
    fp = FP(ref, "TestPad_1.7", x, y)
    fp.pad("1", x, y, "thru_hole", "circle", 1.6, 1.6, net, 0.9)
    fps.append(fp)
    return fp

def r0603(ref, value, x, y, n1, n2, vertical=False):
    fp = FP(ref, f"R_0603_{value}", x, y, side="B")
    if vertical:
        fp.pad("1", x, y - 0.8, "smd", "rect", 0.95, 0.9, n1); fp.pad("2", x, y + 0.8, "smd", "rect", 0.95, 0.9, n2)
    else:
        fp.pad("1", x - 0.8, y, "smd", "rect", 0.9, 0.95, n1); fp.pad("2", x + 0.8, y, "smd", "rect", 0.9, 0.95, n2)
    fps.append(fp)
    p1 = (x, y - 0.8) if vertical else (x - 0.8, y)
    p2 = (x, y + 0.8) if vertical else (x + 0.8, y)
    return p1, p2

def c0805(ref, value, x, y, n1, n2, vertical=False):
    fp = FP(ref, f"C_0805_{value}", x, y, side="B")
    if vertical:
        fp.pad("1", x, y - 0.95, "smd", "rect", 1.3, 1.0, n1); fp.pad("2", x, y + 0.95, "smd", "rect", 1.3, 1.0, n2)
    else:
        fp.pad("1", x - 0.95, y, "smd", "rect", 1.0, 1.3, n1); fp.pad("2", x + 0.95, y, "smd", "rect", 1.0, 1.3, n2)
    fps.append(fp)
    return ((x, y - 0.95), (x, y + 0.95)) if vertical else ((x - 0.95, y), (x + 0.95, y))

def sot23_dright(ref, x, y, g, s, d):
    """Si2302: G и S слева (сверху G), D справа."""
    fp = FP(ref, "SOT-23_Si2302", x, y, side="B")
    G = fp.pad("1", x - 1.1, y - 0.95, "smd", "rect", 1.0, 0.7, g)
    S = fp.pad("2", x - 1.1, y + 0.95, "smd", "rect", 1.0, 0.7, s)
    D = fp.pad("3", x + 1.1, y, "smd", "rect", 1.0, 0.7, d)
    fps.append(fp)
    return G, S, D

def sma(ref, x, y, na, nk):
    """SS16 в SMA: катод справа."""
    fp = FP(ref, "D_SMA_SS16", x, y, side="B")
    A = fp.pad("2", x - 2.1, y, "smd", "rect", 2.2, 1.6, na)
    K = fp.pad("1", x + 2.1, y, "smd", "rect", 2.2, 1.6, nk)
    fps.append(fp)
    return A, K

def ecap(ref, xp, yp, xm, ym, npos, nneg, d=10.0):
    fp = FP(ref, f"CP_Radial_D{d:g}", (xp + xm) / 2, (yp + ym) / 2)
    fp.pad("1", xp, yp, "thru_hole", "rect", 1.8, 1.8, npos, 0.9)
    fp.pad("2", xm, ym, "thru_hole", "circle", 1.8, 1.8, nneg, 0.9)
    fp.circle((xp + xm) / 2, (yp + ym) / 2, d / 2)
    fp.text("+", xp, yp - 1.8, 1.0)
    fps.append(fp)

def hole(ref, x, y):
    fp = FP(ref, "MountingHole_M3", x, y)
    fp.pad("", x, y, "np_thru_hole", "circle", 3.2, 3.2, "", 3.2)
    fp.circle(x, y, 3.0)
    fps.append(fp)

# =====================================================================
#                          РАЗМЕЩЕНИЕ И ДОРОЖКИ
# =====================================================================
WS, WG, W3, W5, WR = 0.4, 0.3, 0.8, 1.0, 2.0    # сигнал, в промежутке, 3V3, 5VD, рельс

# ----- гнёзда DevEBox -----
fps.append(socket2x22("J2", X2o, X2i, True, J2NET, J2NAME, "L"))
fps.append(socket2x22("J3", X3o, X3i, False, J3NET, J3NAME, "R"))
# перемычки в рядах питания
track("+5VD", W5, j2(1), j2(2)); track("+3V3B", W3, j2(3), j2(4)); track("GND", W5, j2(5), j2(6))
track("+3V3", W3, j3(1), j3(2)); track("GND", W5, j3(3), j3(4))

# ----- рельс 5V BEC -----
RAIL_L, RAIL_TOP, RAIL_RT, RAIL_R, JOG_Y = 5.5, 1.6, 80.5, 75.5, 21.0
track("+5VB", WR, (RAIL_L, 54.6), (RAIL_L, RAIL_TOP), (RAIL_RT, RAIL_TOP), (RAIL_RT, JOG_Y), (RAIL_R, JOG_Y), (RAIL_R, 87.0))

# ----- D1 SS16 + 5VD -----
A, K = sma("D1", 14.0, 5.2, "+5VB", "+5VD")
track("+5VB", W5, (A[0], RAIL_TOP), A)
track("+5VD", W5, K, (21.0, 5.2), (24.0, 8.2), j2(1))
ecap("CE3", 17.6, 9.4, 12.6, 9.4, "+5VD", "GND")
track("+5VD", W5, (17.6, 9.4), (17.6, 5.2))
p1, p2 = c0805("C1", "10u", 19.6, 6.9, "+5VD", "GND", vertical=True)
track("+5VD", 0.6, (19.6, 5.2), p1)
p1, p2 = c0805("C2", "100n", 21.6, 7.4, "+5VD", "GND", vertical=True)
track("+5VD", 0.6, (21.6, 5.8), p1)

# ----- правый верх: 3V3, GND, SPI, I2C-A (экран + внешний) -----
OLED_X, EXT_X = 70.0, 76.5
YI = [10.0, 12.5, 15.0, 17.5]       # 3V3 GND SCL SDA
track("+3V3", W3, j3(2), (EXT_X, YI[0]))
track("GND", 0.6, j3(4), (62.10 + 1.6, 12.54), (EXT_X, YI[1]))
xh("OLED", [(OLED_X, y) for y in YI], "v", ["+3V3", "GND", "SCL_A", "SDA_A"], body_dir=-1)
xh("EXT", [(EXT_X, y) for y in YI], "v", ["+3V3", "GND", "SCL_A", "SDA_A"], body_dir=1)
p1, p2 = c0805("C3", "10u", 65.0, 11.27, "+3V3", "GND", vertical=True)
p1, p2 = c0805("C4", "100n", 67.2, 11.27, "+3V3", "GND", vertical=True)
# SPI: площадки зигзагом
SP1, SP2 = 64.6, 66.6
for ref, x, y, n in [("TP_MOSI", SP1, gp(3), "MOSI"), ("TP_MISO", SP2, ry(4), "MISO"),
                     ("TP_SCK", SP1, gp(4), "SCK"), ("TP_CS", SP2, ry(5), "CS_IMU")]:
    testpad(ref, x, y, n, n)
track("MOSI", WG, j3(7), (X3i, gp(3)), (SP1, gp(3)))
track("MISO", WG, j3(8), (SP2, ry(4)))
track("SCK", WG, j3(9), (X3i, gp(4)), (SP1, gp(4)))
track("CS_IMU", WG, j3(10), (SP2, ry(5)))
# I2C-A: SCL выше SDA
track("SCL_A", WG, j3(12), (68.3, ry(6)), (68.3, YI[2]), (EXT_X, YI[2]))
track("SDA_A", WG, j3(11), (X3i, gp(6)), (69.6, gp(6)), (69.6, YI[3]), (EXT_X, YI[3]))
ecap("CE1", 84.0, 36.25, 87.5, 36.25, "+5VB", "GND", d=8.0)
track("+5VB", W5, (RAIL_R, 36.25), (84.0, 36.25))

# ----- правая колонка: BUZ, LIGHT, AUX2, RC, ESC, ELE, AIL-R, AIL-L, B1 -----
SX = 73.0
YC = {"BUZ": 26.5, "LIGHT": 33.0, "AUX2": 39.5, "RC": 46.0, "ESC": 52.5, "ELE": 59.0,
      "AILR": 65.5, "AILL": 72.0, "B1": 78.5}
def servo_r(ref, label, net_io, y):
    fp = xh(ref, [(SX, y), (SX + 2.5, y), (SX + 5.0, y)], "h", [net_io, "+5VB", "GND"])
    return fp
# BUZ и LIGHT: XH-2 (− ключ, + рельс)
xh("BUZ", [(SX, YC["BUZ"]), (SX + 2.5, YC["BUZ"])], "h", ["BUZ_D", "+5VB"])
xh("LIGHT", [(SX, YC["LIGHT"]), (SX + 2.5, YC["LIGHT"])], "h", ["LIGHT_D", "+5VB"])
servo_r("S1", "AUX2", "AUX2_O", YC["AUX2"])
xh("RC", [(SX - 2.5, YC["RC"]), (SX, YC["RC"]), (SX + 2.5, YC["RC"]), (SX + 5.0, YC["RC"])], "h",
   ["RC_RX", "RC_TX", "+5VB", "GND"])
servo_r("S3", "ESC", "ESC_O", YC["ESC"])
servo_r("S4", "ELE", "ELE_O", YC["ELE"])
servo_r("S5", "AILR", "AILR_O", YC["AILR"])
servo_r("S6", "AILL", "AILL_O", YC["AILL"])
xh("B1", [(SX - 5.0, YC["B1"]), (SX - 2.5, YC["B1"]), (SX, YC["B1"]), (SX + 2.5, YC["B1"]), (SX + 5.0, YC["B1"])],
   "h", ["VBAT_IN", "", "CURR_IN", "+5VB", "GND"])
ecap("CE2", RAIL_R, 87.0, RAIL_R + 5.0, 87.0, "+5VB", "GND")

# конденсаторы на рельсе (между + и наружной землёй), снизу каждого разъёма
for i, (k, y) in enumerate(YC.items()):
    if k in ("BUZ", "LIGHT"):
        continue
    c0805(f"CR{i}", "100n", RAIL_R + 1.0, y + 2.4, "+5VB", "GND")
c0805("CR_ESC10", "10u", RAIL_R + 1.0, YC["ESC"] - 2.6, "+5VB", "GND")

# --- ключи под DevEBox (Q1 пищалка, Q2 огни) ---
# Q1: затвор PE15 (J3-13, ряд 7), сток наружу через промежуток 7/8
G, S, D = sot23_dright("Q1", 51.9, 26.51 - 0.0, "BUZ_G", "GND", "BUZ_D")
track("BUZ_IO", WS, j3(13), (57.6, ry(7)))
r1, r2 = r0603("R1", "100", 56.4, ry(7), "BUZ_G", "BUZ_IO")
track("BUZ_G", WS, r1, (G[0], ry(7)), G)
track("BUZ_D", WG, D, (D[0], gp(7)), (SX, gp(7)), (SX, YC["BUZ"]))
r1, r2 = r0603("R2", "100k", 49.0, G[1], "GND", "BUZ_G")
track("BUZ_G", WS, r2, G)
# Q2: затвор PE13 (J3-15, ряд 8), сток через промежуток 8/9
G, S, D = sot23_dright("Q2", 55.4, gp(8) - 0.0, "LIGHT_G", "GND", "LIGHT_D")
track("LIGHT_IO", WS, j3(15), (58.6, ry(8)))
r1, r2 = r0603("R3", "100", 57.8, ry(8) - 0.0, "LIGHT_G", "LIGHT_IO")
track("LIGHT_G", WS, r1, (G[0], ry(8)), G)
track("LIGHT_D", WG, D, (D[0] + 0.0, gp(8)), (71.3, gp(8)), (71.3, YC["LIGHT"]), (SX, YC["LIGHT"]))
r1, r2 = r0603("R4", "100k", 52.6, G[1] + 0.0, "GND", "LIGHT_G")
track("LIGHT_G", WS, r2, G)

# --- сигналы колонки: выход из J3, лесенка ---
def servo_route(io_net, out_net, ref, start_pts, d, ytgt, rpos):
    """start_pts — путь от ножки J3 до точки выхода (x, y_exit); потом к d, вниз до ytgt, к ножке 1.
    rpos — 'exit' (резистор на выходной горизонтали) или 'final'."""
    xe, ye = start_pts[-1]
    if rpos == "exit":
        rx = 65.3
        p1, p2 = r0603(ref, "330", rx, ye, io_net, out_net)
        track(io_net, WG, *start_pts)
        track(io_net, WS, start_pts[-1], p1)
        track(out_net, WS, p2, (d, ye), (d, ytgt), (SX, ytgt))
    else:
        rx = (d + SX) / 2
        p1, p2 = r0603(ref, "330", rx, ytgt, io_net, out_net)
        track(io_net, WG, *start_pts)
        track(io_net, WS, start_pts[-1], (d, ye), (d, ytgt), p1)
        track(out_net, WS, p2, (SX, ytgt))

servo_route("AUX2", "AUX2_O", "R5", [j3(19), (X3i, gp(9)), (X3o + 1.6, gp(9))], 70.5, YC["AUX2"], "exit")
# RC: TX (PE8, J3-20) -> ножка 2, RX (PE7, J3-21 через 10/11) -> ножка 1
track("RC_TX", WS, j3(20), (69.7, ry(10)), (69.7, 43.6), (SX, 43.6), (SX, YC["RC"]))
track("RC_RX", WG, j3(21), (X3i, gp(10)), (X3o + 1.6, gp(10)))
track("RC_RX", WS, (X3o + 1.6, gp(10)), (68.9, gp(10)), (68.9, 44.5), (SX - 2.5, 44.5), (SX - 2.5, YC["RC"]))
servo_route("ESC", "ESC_O", "R6", [j3(31), (X3i, gp(15)), (X3o + 1.6, gp(15))], 68.1, YC["ESC"], "final")
servo_route("ELE", "ELE_O", "R7", [j3(32)], 67.3, YC["ELE"], "final")
servo_route("AILR", "AILR_O", "R8", [j3(33), (X3i, gp(16)), (X3o + 1.6, gp(16))], 66.5, YC["AILR"], "final")
servo_route("AILL", "AILL_O", "R9", [j3(34)], 65.7, YC["AILL"], "final")
r1, r2 = r0603("R10", "10k", 57.2, ry(16), "GND", "ESC")         # подтяжка ESC к GND
track("ESC", WS, r2, j3(31))

# --- батарея: делители под DevEBox ---
# CURR: S = PC1 (J3-37, ряд 19), IN через промежуток 18/19
track("CURR_IN", WG, (X3o + 1.6, gp(18)), (56.8, gp(18)))
track("CURR_IN", WS, (X3o + 1.6, gp(18)), (64.9, gp(18)), (64.9, 76.0), (SX, 76.0), (SX, YC["B1"]))
r1, r2 = r0603("R11", "10k", 56.0, gp(18), "CURR_S", "CURR_IN")          # верх CURR
track("CURR_S", WS, r1, (53.8, gp(18)), (53.8, gp(18) - 0.8))
r1b, r2b = (53.8, gp(18) - 2.4), (53.8, gp(18) - 0.8)
c1, c2 = c0805("C5", "100n", 51.8, gp(18) - 1.7, "GND", "CURR_S", vertical=True)
track("CURR_S", WS, (53.8, gp(18)), (51.8, gp(18)), c2)
r0603("R12", "15k", 53.8, gp(18) - 1.6, "GND", "CURR_S", vertical=True)   # низ CURR
track("CURR_S", WS, (55.2, gp(18)), (55.2, ry(19)), j3(37))
# VBAT: S = PC0 (J3-38, ряд 19) внутрь через 19/20, IN через 20/21
track("VBAT_S", WG, j3(38), (X3o, gp(19)), (55.2, gp(19)), (55.2, gp(20)))
track("VBAT_IN", WG, (X3o + 1.6, gp(20)), (56.8, gp(20)))
track("VBAT_IN", WS, (X3o + 1.6, gp(20)), (64.1, gp(20)), (64.1, 77.0), (SX - 5.0, 77.0), (SX - 5.0, YC["B1"]))
r1, r2 = r0603("R13", "56k", 56.0, gp(20), "VBAT_S", "VBAT_IN")
track("VBAT_S", WS, r1, (51.8, gp(20)))
track("VBAT_S", WS, (53.8, gp(20)), (53.8, gp(20) + 0.8))
track("VBAT_S", WS, (51.8, gp(20)), (51.8, gp(20) + 0.75))
r1b, r2b = r0603("R14", "10k", 53.8, gp(20) + 1.6, "VBAT_S", "GND", vertical=True)
c1, c2 = c0805("C6", "100n", 51.8, gp(20) + 1.7, "VBAT_S", "GND", vertical=True)

# --- PYRO: затвор PB1 (J3-23, ряд 12), ключ внизу ---
track("PYRO_IO", WS, j3(23), (46.0, ry(12)), (46.0, 70.0), (57.0, 70.0), (57.0, 81.4))
r1, r2 = r0603("R15", "100", 57.0, 82.2, "PYRO_IO", "PYRO_G", vertical=True)
G, S, D = sot23_dright("Q3", 60.0, 88.5, "PYRO_G", "GND", "PYRO_D")
track("PYRO_G", WS, r2, (57.0, G[1]), G)
r1, r2 = r0603("R16", "100k", 57.0, 90.4, "PYRO_G", "GND", vertical=True)
track("PYRO_G", WS, (57.0, G[1]), r1)
PY = 94.0
xh("PYRO", [(61.5, PY), (64.0, PY), (66.5, PY), (69.0, PY)], "h", ["PYRO_D", "PYRO_P", "VPYRO", "GND"])
track("PYRO_D", W5, D, (61.5, D[1]), (61.5, PY))
arm = FP("ARM", "Jumper_2.54", 65.27, 89.0)
arm.pad("1", 64.0, 89.0, "thru_hole", "rect", 1.7, 1.7, "PYRO_P", 0.9)
arm.pad("2", 66.54, 89.0, "thru_hole", "circle", 1.7, 1.7, "VPYRO", 0.9)
fps.append(arm)
track("PYRO_P", W5, (64.0, 89.0), (64.0, PY))
track("VPYRO", W5, (66.54, 89.0), (66.5, PY))

# ----- левая колонка: GPS, RUD, AUX1, TELEM, UART2 -----
LG, LP, L2, L1 = 3.0, 5.5, 8.0, 10.5     # GND, +, ножка 2, ножка 1
YL = {"GPS": 20.0, "S7": 26.5, "S8": 33.0, "TEL": 39.5, "U2": 53.0}
xh("GPS", [(L1, YL["GPS"]), (L2, YL["GPS"]), (LP, YL["GPS"]), (LG, YL["GPS"])], "h", ["GPS_RX", "GPS_TX", "+5VB", "GND"])
xh("S7", [(L2, YL["S7"]), (LP, YL["S7"]), (LG, YL["S7"])], "h", ["RUD_O", "+5VB", "GND"])
xh("S8", [(L2, YL["S8"]), (LP, YL["S8"]), (LG, YL["S8"])], "h", ["AUX1_O", "+5VB", "GND"])
xh("TEL", [(L1, YL["TEL"]), (L2, YL["TEL"]), (LP, YL["TEL"]), (LG, YL["TEL"])], "h", ["TEL_RX", "TEL_TX", "+5VB", "GND"], body_dir=-1)
xh("UART2", [(L1, YL["U2"]), (L2, YL["U2"]), (LP, YL["U2"]), (LG, YL["U2"])], "h", ["U2_RX", "U2_TX", "+5VB", "GND"])
for i, (k, y) in enumerate(YL.items()):
    dy = -2.6 if k == "TEL" else 2.4
    c0805(f"CL{i}", "100n", LP - 1.0, y + dy, "GND", "+5VB")
c0805("CL_GPS10", "10u", LP - 1.0, YL["GPS"] - 2.6, "GND", "+5VB")
# GPS: PD8 (TX, J2-8 внутр.) через 3/4 -> ножка 2; PD9 (RX) -> ножка 1
track("GPS_TX", WG, j2(8), (X2i, gp(3)), (X2o - 1.6, gp(3)))
track("GPS_TX", WS, (X2o - 1.6, gp(3)), (L2, gp(3)), (L2, YL["GPS"]))
track("GPS_RX", WS, j2(7), (L1, ry(4)), (L1, YL["GPS"]))
# RUD: PD14 (J2-14 внутр.) через 6/7 -> S7; AUX1: PD15 -> S8
track("RUD", WG, j2(14), (X2i, gp(6)), (X2o - 1.6, gp(6)))
r1, r2 = r0603("R17", "330", 17.0, gp(6), "RUD_O", "RUD")
track("RUD", WS, (X2o - 1.6, gp(6)), r2)
track("RUD_O", WS, r1, (L2, gp(6)), (L2, YL["S7"]))
r1, r2 = r0603("R18", "330", 18.0, ry(7), "AUX1_O", "AUX1")
track("AUX1", WS, j2(13), r2)
track("AUX1_O", WS, r1, (11.2, ry(7)), (11.2, YL["S8"]), (L2, YL["S8"]))
# TELEM (разъём выше дорожек): RX PD0 (J2-27) -> ножка 1, TX PD1 (J2-30 через 14/15) -> ножка 2
track("TEL_RX", WS, j2(27), (L1, ry(14)), (L1, YL["TEL"]))
track("TEL_TX", WG, j2(30), (X2i, gp(14)), (X2o - 1.6, gp(14)))
track("TEL_TX", WS, (X2o - 1.6, gp(14)), (L2, gp(14)), (L2, YL["TEL"]))
# UART2 (ниже): TX PD5 (J2-34 через 16/17) -> ножка 2, RX PD6 (J2-33) -> ножка 1
track("U2_TX", WG, j2(34), (X2i, gp(16)), (X2o - 1.6, gp(16)))
track("U2_TX", WS, (X2o - 1.6, gp(16)), (L2, gp(16)), (L2, YL["U2"]))
track("U2_RX", WS, j2(33), (L1, ry(17)), (L1, YL["U2"]))

# ----- низ: шина датчиков I2C-B (IMU, BARO), 3V3 из-под платы -----
YB = [76.0, 78.54, 81.08, 83.62]     # VCC GND SCL SDA
X3V, XGND, XSCL, XSDA = 32.5, 31.1, 29.7, 28.5
IMU_X, BARO_X = 36.0, 54.0
track("+3V3B", W3, j2(4), (X3V, ry(2)), (X3V, YB[0]), (BARO_X, YB[0]))
track("GND", 0.6, (XGND, 66.0), (XGND, YB[1]), (BARO_X + 2.0, YB[1]))
track("SCL_B", WS, j2(42), (XSCL, ry(21)), (XSCL, YB[2]), (BARO_X, YB[2]))
track("SDA_B", WG, j2(41), (X2o, gp(21)), (X2i + 1.0, gp(21)))
track("SDA_B", WS, (X2i + 1.0, gp(21)), (XSDA, gp(21)), (XSDA, YB[3]), (BARO_X, YB[3]))
pinsocket("IMU", [(IMU_X, YB[0] + i * P) for i in range(8)], ["+3V3B", "GND", "SCL_B", "SDA_B", "", "", "", ""])
pinsocket("BARO", [(BARO_X, y) for y in YB], ["+3V3B", "GND", "SCL_B", "SDA_B"])
c1, c2 = c0805("C7", "10u", 34.0, 77.27, "+3V3B", "GND", vertical=True)
c1, c2 = c0805("C8", "100n", 52.0, 77.27, "+3V3B", "GND", vertical=True)

# ----- контрольные площадки, отверстия -----
testpad("TP_5VB", RAIL_R + 4.0, 93.5, "+5VB", "5VB")
track("+5VB", W5, (RAIL_R, 87.0), (RAIL_R, 93.5), (RAIL_R + 4.0, 93.5))
testpad("TP_3V3", 38.5, 72.0, "+3V3B", "3V3")
track("+3V3B", W3, (X3V, 72.0), (38.5, 72.0))
testpad("TP_GND", 41.0, 72.0, "GND", "GND")
for ref, x, y in [("H1", 3.0, 64.0), ("H2", 3.0, 94.5), ("H3", 87.0, 45.5), ("H4", 86.5, 94.5)]:
    hole(ref, x, y)

# ----- надписи (F.SilkS — лазером на голом текстолите) -----
def lab(t, x, y, s=1.0):
    silk(t, x, y, s)
labels = [
    ("GPS 1RX 2TX + -", L2 - 1.0, YL["GPS"] - 3.0), ("RUD", 14.0, YL["S7"] + 0.2), ("AUX1", 14.0, YL["S8"]),
    ("TELEM 1RX 2TX + -", L2 - 1.0, YL["TEL"] + 3.4), ("UART2 1RX 2TX + -", L2 - 1.0, YL["U2"] + 3.6),
    ("BUZ - +", SX + 6.5, YC["BUZ"]), ("LIGHT - +", SX + 7.0, YC["LIGHT"]),
    ("AUX2", SX + 9.8, YC["AUX2"] - 2.2), ("RC 1RX 2TX", SX + 1.0, YC["RC"] + 4.0),
    ("ESC", SX + 9.8, YC["ESC"] - 2.2), ("ELE", SX + 9.8, YC["ELE"] - 2.2), ("AIL-R", SX + 9.8, YC["AILR"] - 2.2),
    ("AIL-L", SX + 9.8, YC["AILL"] - 2.2), ("BAT 1VBAT 3CURR 4+5V", SX, YC["B1"] + 4.2),
    ("PYRO 1- 2+ 3VBAT 4GND", 65.0, PY + 3.0), ("ARM", 65.3, 86.8),
    ("OLED", OLED_X, 7.0), ("EXT I2C", EXT_X + 1.0, 7.0), ("3V3 GND SCL SDA", 73.0, 20.0),
    ("SPI", 65.6, 14.6), ("IMU VCC GND SCL SDA", IMU_X + 8.5, 95.8), ("BARO", BARO_X + 4.0, 83.0),
    ("OpenPlane DevEBox H743 carrier", 43.0, 97.0), ("NOSE ^", 43.0, 90.0),
]
for t, x, y in labels:
    lab(t, x, y, 0.9)
silk("J2", X2o - 2.2, ry(1) - 2.5, 1.0); silk("J3", X3o + 2.2, ry(1) - 2.5, 1.0)
for n in range(1, 45):
    x, y = j2(n)
    if n % 2:
        silk(J2NAME[n], x - 2.6, y, 0.6)
for n in range(1, 45):
    x, y = j3(n)
    if n % 2 == 0:
        silk(J3NAME[n], x + 2.6, y, 0.6)
sline(X2o - 1.4, 3.0, X3o + 1.4, 3.0, "F.Fab"); sline(X2o - 1.4, 71.6, X3o + 1.4, 71.6, "F.Fab")
sline(X2o - 1.4, 3.0, X2o - 1.4, 71.6, "F.Fab"); sline(X3o + 1.4, 3.0, X3o + 1.4, 71.6, "F.Fab")
silk("DevEBox H743 (USB up)", 43.0, 6.0, 1.2)

# =====================================================================
#                             ВЫВОД
# =====================================================================
def board_sx():
    out = ['(kicad_pcb (version 20211014) (generator pcbnew)',
           '  (general (thickness 1.6))', '  (paper "A4")',
           '  (layers',
           '    (0 "F.Cu" signal) (31 "B.Cu" signal)',
           '    (32 "B.Adhes" user "B.Adhesive") (33 "F.Adhes" user "F.Adhesive")',
           '    (34 "B.Paste" user) (35 "F.Paste" user) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")',
           '    (38 "B.Mask" user) (39 "F.Mask" user) (40 "Dwgs.User" user "User.Drawings") (41 "Cmts.User" user "User.Comments")',
           '    (44 "Edge.Cuts" user) (45 "Margin" user) (46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard")',
           '    (48 "B.Fab" user) (49 "F.Fab" user)',
           '  )',
           '  (setup (pad_to_mask_clearance 0))']
    for name, i in nets.items():
        out.append(f'  (net {i} "{name}")')
    for f in fps:
        out.append(f.sx())
    for (x1, y1, x2, y2, w, n) in segs:
        out.append(f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width {w}) (layer "B.Cu") (net {nets[n]}) (tstamp {U()}))')
    for (t, x, y, s, layer, ang) in texts:
        mir = " (justify mirror)" if layer.startswith("B.") else ""
        out.append(f'  (gr_text "{t}" (at {x:.3f} {y:.3f} {ang}) (layer "{layer}") (tstamp {U()}) (effects (font (size {s} {s}) (thickness {s*0.15:.3f})){mir}))')
    for (x1, y1, x2, y2, layer, w) in lines:
        out.append(f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "{layer}") (width {w}) (tstamp {U()}))')
    out.append(f'  (gr_rect (start 0 0) (end {W} {H}) (layer "Edge.Cuts") (width 0.15) (fill none) (tstamp {U()}))')
    out.append(f'''  (zone (net {nets["GND"]}) (net_name "GND") (layer "B.Cu") (tstamp {U()}) (hatch edge 0.508)
    (connect_pads thru_hole_only (clearance 0.4))
    (min_thickness 0.3) (filled_areas_thickness no)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.6) (island_removal_mode 1))
    (polygon (pts (xy 0.3 0.3) (xy {W-0.3} 0.3) (xy {W-0.3} {H-0.3}) (xy 0.3 {H-0.3}))))''')
    out.append(")")
    return "\n".join(out)

with open(OUT + ".kicad_pcb", "w", encoding="utf-8") as f:
    f.write(board_sx())

pro = {
    "board": {"design_settings": {
        "defaults": {"board_outline_line_width": 0.15},
        "rules": {"min_clearance": 0.3, "min_track_width": 0.3, "min_through_hole_diameter": 0.8,
                  "min_hole_clearance": 0.25, "min_copper_edge_clearance": 0.4, "min_hole_to_hole": 0.25,
                  "min_via_diameter": 0.5, "min_via_annular_width": 0.1},
        "rule_severities": {"silk_overlap": "ignore", "silk_over_copper": "ignore", "silk_edge_clearance": "ignore",
                            "lib_footprint_issues": "ignore", "lib_footprint_mismatch": "ignore",
                            "text_height": "ignore", "text_thickness": "ignore", "courtyards_overlap": "ignore",
                            "missing_courtyard": "ignore", "isolated_copper": "ignore", "starved_thermal": "warning",
                            "footprint_type_mismatch": "ignore", "through_hole_pad_without_hole": "ignore"}}},
    "net_settings": {"classes": [{"name": "Default", "clearance": 0.3, "track_width": 0.4,
                                  "via_diameter": 0.8, "via_drill": 0.4, "wire_width": 6, "bus_width": 12,
                                  "line_style": 0, "schematic_color": "rgba(0, 0, 0, 0.000)", "pcb_color": "rgba(0, 0, 0, 0.000)",
                                  "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2,
                                  "microvia_diameter": 0.3, "microvia_drill": 0.1}], "meta": {"version": 3}},
    "meta": {"filename": os.path.basename(OUT) + ".kicad_pro", "version": 1},
}
with open(OUT + ".kicad_pro", "w", encoding="utf-8") as f:
    json.dump(pro, f, indent=2)
print("ok", len(fps), "footprints", len(segs), "segments", len(nets), "nets")
