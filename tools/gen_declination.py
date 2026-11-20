#!/usr/bin/env python3
"""Таблица магнитного склонения для прошивки: include/copter/DeclinationTable.h.

Коэффициенты — World Magnetic Model WMM-2025 (NOAA/NCEI и BGS, общественное
достояние), файл tools/WMM_2025.COF как его выдаёт NOAA. Расчёт — свой, по
техническому отчёту WMM: сферические гармоники до 12-й степени, полиномы
Лежандра с полунормировкой Шмидта, геодезические координаты WGS-84 ->
геоцентрические.

Прошивке весь расчёт не нужен (90 пар коэффициентов, тригонометрия на каждый
отсчёт GPS): она берёт склонение из сетки 10° x 10° (широта −80…80, долгота
−180…180) на середину срока модели и интерполирует между узлами. Ошибка сетки
— в пределах градуса почти везде, кроме окрестностей магнитных полюсов (там и
компасу верить нельзя: горизонтальное поле мало).

    python tools/gen_declination.py            # переписывает таблицу
    python tools/gen_declination.py --check    # таблица совпадает с моделью?

Новая модель (WMM-2030) — положить её .COF, поменять COF и EPOCH ниже,
запустить и закоммитить обе вещи.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
COF = os.path.join(HERE, "WMM_2025.COF")
OUT = os.path.join(HERE, "..", "include", "copter", "DeclinationTable.h")
EPOCH = 2027.5            # середина срока WMM-2025 (2025.0…2030.0)
LAT_MIN, LAT_MAX, LON_MIN, LON_MAX, STEP = -80, 80, -180, 180, 10
NMAX = 12


def load_cof(path):
    g, h, gd, hd = ({} for _ in range(4))
    with open(path, encoding="ascii") as f:
        epoch = float(f.readline().split()[0])
        for line in f:
            p = line.split()
            if len(p) < 6 or p[0].startswith("9999"):
                break
            n, m = int(p[0]), int(p[1])
            g[n, m], h[n, m], gd[n, m], hd[n, m] = (float(x) for x in p[2:6])
    return epoch, g, h, gd, hd


def field(model, lat_deg, lon_deg, alt_km, year):
    """Северная X, восточная Y, вниз Z (нТл) в точке."""
    epoch, g0, h0, gd, hd = model
    dt = year - epoch
    a_wgs, f_wgs = 6378.137, 1 / 298.257223563
    e2 = f_wgs * (2 - f_wgs)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    # геодезические -> геоцентрические
    rc = a_wgs / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    xp = (rc + alt_km) * math.cos(lat)
    zp = (rc * (1 - e2) + alt_km) * math.sin(lat)
    r = math.hypot(xp, zp)
    phi = math.asin(zp / r)          # геоцентрическая широта
    a = 6371.2                       # радиус модели
    # полиномы Лежандра P(n,m)(sin phi) с полунормировкой Шмидта и производные по phi
    s, c = math.sin(phi), math.cos(phi)
    P = [[0.0] * (NMAX + 2) for _ in range(NMAX + 2)]
    dP = [[0.0] * (NMAX + 2) for _ in range(NMAX + 2)]
    P[0][0] = 1.0
    for n in range(1, NMAX + 1):
        for m in range(0, n + 1):
            if n == m:
                k = 1.0 if n == 1 else math.sqrt((2 * n - 1) / (2 * n))
                P[n][n] = k * c * P[n - 1][n - 1]
                dP[n][n] = k * (c * dP[n - 1][n - 1] - s * P[n - 1][n - 1])
            else:
                k1 = (2 * n - 1) / math.sqrt(n * n - m * m)
                k2 = math.sqrt(((n - 1) ** 2 - m * m) / (n * n - m * m)) if n > 1 else 0.0
                p2 = P[n - 2][m] if n >= 2 else 0.0
                d2 = dP[n - 2][m] if n >= 2 else 0.0
                P[n][m] = k1 * s * P[n - 1][m] - k2 * p2
                dP[n][m] = k1 * (s * dP[n - 1][m] + c * P[n - 1][m]) - k2 * d2
    bx = by = bz = 0.0
    for n in range(1, NMAX + 1):
        ar = (a / r) ** (n + 2)
        for m in range(0, n + 1):
            g = g0[n, m] + dt * gd[n, m]
            h = h0[n, m] + dt * hd[n, m]
            cm, sm = math.cos(m * lon), math.sin(m * lon)
            t = g * cm + h * sm
            bx += -ar * t * dP[n][m]
            by += ar * m * (g * sm - h * cm) * P[n][m] / c if c > 1e-9 else 0.0
            bz += -(n + 1) * ar * t * P[n][m]
    # геоцентрические -> геодезические оси
    psi = phi - lat
    x = bx * math.cos(psi) - bz * math.sin(psi)
    z = bx * math.sin(psi) + bz * math.cos(psi)
    return x, by, z


def declination(model, lat, lon, alt_km=0.0, year=EPOCH):
    x, y, _ = field(model, lat, lon, alt_km, year)
    return math.degrees(math.atan2(y, x))


def table(model):
    rows = []
    for lat in range(LAT_MIN, LAT_MAX + 1, STEP):
        rows.append([round(declination(model, lat, lon) * 10) for lon in range(LON_MIN, LON_MAX + 1, STEP)])
    return rows


def render(rows):
    cols = len(rows[0])
    out = [
        "#pragma once",
        "#include <stdint.h>",
        "",
        "// Сгенерировано tools/gen_declination.py из WMM-2025 (NOAA/NCEI, BGS — общественное",
        f"// достояние) на {EPOCH}. Руками не править: поправить генератор и перезапустить.",
        f"// Склонение, 0.1°: + — магнитный север восточнее истинного. Строки — широта",
        f"// {LAT_MIN}…{LAT_MAX} через {STEP}°, столбцы — долгота {LON_MIN}…{LON_MAX} через {STEP}°.",
        "",
        "namespace copter",
        "{",
        "    namespace declination_table",
        "    {",
        f"        constexpr int LAT_MIN = {LAT_MIN}, LAT_MAX = {LAT_MAX}, LON_MIN = {LON_MIN}, LON_MAX = {LON_MAX}, STEP = {STEP};",
        f"        constexpr int ROWS = {len(rows)}, COLS = {cols};",
        f"        constexpr float EPOCH = {EPOCH}f;",
        "        static const int16_t DECI_DEG[ROWS][COLS] = {",
    ]
    for lat, row in zip(range(LAT_MIN, LAT_MAX + 1, STEP), rows):
        out.append("            { " + ", ".join(str(v) for v in row) + " },   // " + f"{lat:+d}")
    out += ["        };", "    }", "}", ""]
    return "\n".join(out)


def main():
    model = load_cof(COF)
    text = render(table(model))
    if "--check" in sys.argv:
        with open(OUT, encoding="utf-8") as f:
            same = f.read() == text
        print("OK" if same else "DIFFERS: run tools/gen_declination.py")
        return 0 if same else 1
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("written", os.path.relpath(OUT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
