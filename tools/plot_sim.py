#!/usr/bin/env python3
"""Графики замкнутых симуляций полёта для README (docs/images/sim).

Траектории пишет сам тест: вся прошивка управляет моделью самолёта
(test/native/test_sim), CSV — если задана OPENPLANE_SIM_DIR:

    OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
    python3 tools/plot_sim.py /tmp/sim docs/images/sim

Нужен matplotlib (pip install matplotlib).
"""
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

BG = "#0d1117"
PANEL = "#161b22"
GRID = "#30363d"
TEXT = "#e6edf3"
MUTED = "#8b949e"
CYAN = "#39d0ff"
GREEN = "#3fb950"
ORANGE = "#ff9f43"
PINK = "#ff5fa2"
YELLOW = "#f2cc60"
PURPLE = "#a371f7"


def load(directory, name):
    with open(os.path.join(directory, name + ".csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    out = {k: [] for k in rows[0]}
    for r in rows:
        for k, v in r.items():
            try:
                out[k].append(float(v))
            except ValueError:
                out[k].append(v)
    return out


def style(ax, title, xlabel, ylabel):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TEXT, fontsize=13, fontweight="bold", loc="left", pad=10)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=10)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=10)
    ax.tick_params(colors=MUTED, labelsize=9)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.grid(True, color=GRID, linewidth=0.6, alpha=0.8)


def figure(w=12, h=5.2, cols=1):
    fig, axes = plt.subplots(1, cols, figsize=(w, h), facecolor=BG)
    return fig, axes


def save(fig, out_dir, name, caption):
    fig.text(0.99, 0.015, caption, color=MUTED, fontsize=8.5, ha="right", va="bottom")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    path = os.path.join(out_dir, name + ".png")
    fig.savefig(path, dpi=110, facecolor=BG)
    plt.close(fig)
    print("  ", path, os.path.getsize(path) // 1024, "KB")


def path_by_height(ax, d, cmap="plasma", width=2.2):
    pts = list(zip(d["east"], d["north"]))
    segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
    lc = LineCollection(segs, cmap=cmap, linewidths=width)
    lc.set_array(d["height"][:-1])
    ax.add_collection(lc)
    return lc


def missions(src, out):
    """Карта: RTH, LOITER, геозабор, failsafe RTH — вид сверху."""
    fig, axes = figure(13, 7.2, 4)
    for ax, (name, title, note) in zip(axes, [
        ("rth", "RTH — домой", "с 600 м, круги над домом"),
        ("loiter", "LOITER — круги", "радиус 50 м по GPS"),
        ("geofence", "Геозабор", "вылет за 500 м → домой"),
        ("failsafe_rth", "Пульт выключен", "failsafe → домой с мотором"),
    ]):
        d = load(src, name)
        lc = path_by_height(ax, d)
        ax.plot(0, 0, marker="*", color=YELLOW, markersize=16, zorder=5)
        ax.plot(d["east"][0], d["north"][0], marker="o", color=CYAN, markersize=7, zorder=5)
        ax.autoscale()
        ax.set_aspect("equal", adjustable="datalim")
        style(ax, title, "восток, м", "север, м" if ax is axes[0] else "")
        ax.text(0.02, 0.02, note, transform=ax.transAxes, color=MUTED, fontsize=9)
        if name == "geofence":
            import math
            r = 500
            ax.plot([r * math.cos(a / 50 * math.pi) for a in range(101)],
                    [r * math.sin(a / 50 * math.pi) for a in range(101)],
                    color=PINK, linestyle="--", linewidth=1.2, alpha=0.8)
            ax.set_xlim(-650, 650)
            ax.set_ylim(-650, 650)
    cax = fig.add_axes((0.945, 0.2, 0.012, 0.6))
    cb = fig.colorbar(lc, cax=cax)
    cb.set_label("высота, м", color=MUTED)
    cb.ax.tick_params(colors=MUTED)
    fig.suptitle("Автопилот в замкнутом контуре: вся прошивка ведёт модель самолёта",
                 color=TEXT, fontsize=15, fontweight="bold", x=0.01, ha="left")
    fig.text(0.01, 0.905, "★ дом   ● старт   цвет — высота", color=MUTED, fontsize=9.5)
    fig.subplots_adjust(left=0.05, right=0.92, top=0.86, bottom=0.12, wspace=0.3)
    fig.text(0.99, 0.015, "test/native/test_sim · pio test -e native", color=MUTED, fontsize=8.5,
             ha="right", va="bottom")
    path = os.path.join(out, "missions.png")
    fig.savefig(path, dpi=110, facecolor=BG)
    plt.close(fig)
    print("  ", path, os.path.getsize(path) // 1024, "KB")


def soaring(src, out):
    d = load(src, "soaring")
    fig, ax = figure(12, 4.6)
    t, h, st = d["t"], d["height"], d["soaring"]
    gain = max(h) - min(h[: len(h) // 4] or h)
    style(ax, f"SOARING: термик найден сам — +{gain:.0f} м с выключенным мотором", "время, с", "высота, м")
    in_thermal = False
    start = 0
    for i, s in enumerate(st):
        if s == 1 and not in_thermal:
            in_thermal, start = True, t[i]
        if s != 1 and in_thermal:
            ax.axvspan(start, t[i], color=GREEN, alpha=0.13, linewidth=0)
            in_thermal = False
    if in_thermal:
        ax.axvspan(start, t[-1], color=GREEN, alpha=0.13, linewidth=0)
    ax.plot(t, h, color=CYAN, linewidth=2.2)
    ax2 = ax.twinx()
    ax2.plot(t, [x * 100 for x in d["throttle"]], color=ORANGE, linewidth=1.2, alpha=0.9)
    ax2.set_ylim(-5, 105)
    ax2.set_ylabel("газ, %", color=ORANGE)
    ax2.tick_params(colors=ORANGE, labelsize=9)
    ax.text(0.01, 0.92, "зелёное — круги в термике (вариометр полной энергии)", transform=ax.transAxes,
            color=GREEN, fontsize=9.5)
    save(fig, out, "soaring", "test_sim_soaring_climbs_in_thermal")


def launch_land(src, out):
    fig, axes = figure(12, 4.4, 2)
    d = load(src, "launch")
    ax = axes[0]
    style(ax, "LAUNCH: бросок с руки", "время, с", "высота, м / скорость, м/с")
    ax.plot(d["t"], d["height"], color=CYAN, linewidth=2.2, label="высота")
    ax.plot(d["t"], d["speed"], color=YELLOW, linewidth=1.4, label="скорость")
    ax.plot(d["t"], [x * 10 for x in d["throttle"]], color=ORANGE, linewidth=1.2, label="газ ×10")
    ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=TEXT, fontsize=9, loc="lower right")

    d = load(src, "land")
    ax = axes[1]
    style(ax, "AUTO_LAND: планирование и выравнивание", "время, с", "высота, м")
    ax.plot(d["t"], d["height"], color=CYAN, linewidth=2.2, label="высота")
    ax2 = ax.twinx()
    ax2.plot(d["t"], d["pitch"], color=PURPLE, linewidth=1.3)
    ax2.set_ylabel("тангаж, °", color=PURPLE)
    ax2.tick_params(colors=PURPLE, labelsize=9)
    ax.axhline(3, color=PINK, linestyle="--", linewidth=1)
    ax.text(0.02, 0.1, "3 м — выравнивание", transform=ax.transAxes, color=PINK, fontsize=9)
    save(fig, out, "launch_land", "test_sim_hand_launch · test_sim_auto_land")


def pitot(src, out):
    d = load(src, "pitot")
    t = d["t"]
    est = [x if x >= 0 else None for x in d["airspeed_est"]]
    fig, ax = figure(12, 4.2)
    style(ax, "Самодельная трубка Пито: BMP581 в трубке + барометр фюзеляжа", "время, с", "воздушная скорость, м/с")
    ax.plot(t, d["speed"], color=MUTED, linewidth=4, alpha=0.6, label="истинная (модель)")
    ax.plot(t, est, color=CYAN, linewidth=1.5, label="оценка прошивки по двум шумным барометрам")
    ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=TEXT, fontsize=9, loc="lower right")
    ax.text(0.01, 0.9, "ошибка < 0.5 м/с при шуме ±1 Па и сдвиге 150 Па между чипами",
            transform=ax.transAxes, color=GREEN, fontsize=9.5)
    save(fig, out, "pitot", "test_sim_real_pitot_in_the_loop")


def recovery(src, out):
    fig, axes = figure(12, 4.2, 2)
    d = load(src, "stabilize")
    ax = axes[0]
    style(ax, "STABILIZE: выход из крена 60°", "время, с", "угол, °")
    ax.plot(d["t"], d["roll"], color=CYAN, linewidth=2.2, label="крен")
    ax.plot(d["t"], d["pitch"], color=PURPLE, linewidth=1.5, label="тангаж")
    ax.axhline(0, color=GRID, linewidth=1)
    ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=TEXT, fontsize=9)
    d = load(src, "rescue")
    ax = axes[1]
    style(ax, "RESCUE: из спирали — одним тумблером", "время, с", "угол, °")
    ax.plot(d["t"], d["roll"], color=CYAN, linewidth=2.2, label="крен")
    ax.plot(d["t"], d["pitch"], color=PURPLE, linewidth=1.5, label="тангаж")
    ax.axhline(0, color=GRID, linewidth=1)
    ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=TEXT, fontsize=9)
    save(fig, out, "recovery", "test_sim_stabilize_recovers_from_upset · test_sim_rescue_from_spiral_dive")


def replay_gif(src, out, name="failsafe_rth", frames=120):
    """Анимация: пульт выключен — самолёт сам возвращается домой."""
    from matplotlib.animation import FuncAnimation, PillowWriter

    d = load(src, name)
    n = len(d["t"])
    step = max(1, n // frames)
    frames = min(frames, n)
    idx = list(range(0, n, step))
    fig = plt.figure(figsize=(6.4, 4.0), facecolor=BG)
    ax = fig.add_axes((0.03, 0.05, 0.6, 0.9))
    ax.set_facecolor(PANEL)
    ax.set_aspect("equal")
    # Квадратное поле вокруг всей траектории.
    x0, x1 = min(d["east"]), max(d["east"])
    y0, y1 = min(d["north"]), max(d["north"])
    half = max(x1 - x0, y1 - y0) / 2 + 40
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.plot(0, 0, marker="*", color=YELLOW, markersize=14)
    ax.text(8, -18, "HOME", color=YELLOW, fontsize=8)
    trail, = ax.plot([], [], color=CYAN, linewidth=2)
    plane, = ax.plot([], [], marker="o", color="#ffffff", markersize=7)
    hud = fig.text(0.66, 0.9, "", color=TEXT, fontsize=10, va="top", family="DejaVu Sans Mono")
    fig.text(0.66, 0.12, "OpenPlane · симуляция\nвся прошивка в контуре", color=MUTED, fontsize=8)

    def draw(k):
        i = idx[k]
        trail.set_data(d["east"][: i + 1], d["north"][: i + 1])
        plane.set_data([d["east"][i]], [d["north"][i]])
        mode = d["mode"][i]
        link = "ПОТЕРЯНА" if str(mode).startswith("FAILSAFE") else "ok"
        hud.set_text(
            f"t      {d['t'][i]:6.1f} с\n"
            f"режим  {mode}\n"
            f"связь  {link}\n"
            f"высота {d['height'][i]:6.1f} м\n"
            f"скор.  {d['speed'][i]:6.1f} м/с\n"
            f"дом    {max(d['home_dist'][i], 0):6.0f} м\n"
            f"газ    {d['throttle'][i] * 100:6.0f} %"
        )
        return trail, plane, hud

    anim = FuncAnimation(fig, draw, frames=len(idx), interval=80, blit=False)
    path = os.path.join(out, "replay_rth.gif")
    anim.save(path, writer=PillowWriter(fps=12), dpi=90, savefig_kwargs={"facecolor": BG})
    plt.close(fig)
    print("  ", path, os.path.getsize(path) // 1024, "KB")


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sim"
    out = sys.argv[2] if len(sys.argv) > 2 else "docs/images/sim"
    os.makedirs(out, exist_ok=True)
    plt.rcParams["font.family"] = "DejaVu Sans"
    missions(src, out)
    soaring(src, out)
    launch_land(src, out)
    pitot(src, out)
    recovery(src, out)
    replay_gif(src, out)


if __name__ == "__main__":
    main()
