#!/usr/bin/env python3
"""Выкладка веб-прошивальщика OpenPlane Flasher на GitHub Pages.

Собирает готовые прошивки STM32 (pio run), кладёт их рядом со страницей
tools/web-flasher/index.html, пишет firmware/manifest.json (размер, sha256,
чип, предел флеша до сектора настроек) и заливает всё одним коммитом в
ветку gh-pages (история ветки не копится: каждый раз --force).

Страница: https://damir-lebedev.github.io/OpenPlaneProject/

    python tools/publish_web_flasher.py            # сборка + проверка + выкладка
    python tools/publish_web_flasher.py --no-build # взять уже собранные .pio/build/*
    python tools/publish_web_flasher.py --dry-run  # собрать сайт в папку, не пушить
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Предел записи — начало сектора настроек (hal/stm32/Stm32FlashStorage.h):
# H743 — последний сектор банка 2, F405 — сектор 11.
H743_LIMIT = "0x081E0000"
F405_LIMIT = "0x080E0000"

FIRMWARE = [
    {
        "env": "matek-h743-quad", "vehicle": "copter", "chip": "h743", "limit": H743_LIMIT,
        "title": "Matek H743 (SLIM / WING / WLITE)", "title_en": "Matek H743 (SLIM / WING / WLITE)",
        "note": "квадро и другие рамы, сборка freestyle5 — раму и регуляторы потом в Studio/CLI",
        "note_en": "quad and other frames, freestyle5 build — set frame and ESCs in Studio/CLI",
    },
    {
        "env": "aocodarc-f405v3-quad", "vehicle": "copter", "chip": "f405", "limit": F405_LIMIT,
        "title": "AocodaRC F405 V3", "title_en": "AocodaRC F405 V3",
        "note": "ICM-42688-P, SPL06, OSD, флеш 16 МБ", "note_en": "ICM-42688-P, SPL06, OSD, 16 MB flash",
    },
    {
        "env": "getfun-f405v3-quad", "vehicle": "copter", "chip": "f405", "limit": F405_LIMIT,
        "title": "GETFUN F405V3 (JXFPVF405V3)", "title_en": "GETFUN F405V3 (JXFPVF405V3)",
        "note": "7\" на Li-ion, Multishot (BLHeli_S)", "note_en": "7\" Li-ion, Multishot (BLHeli_S)",
    },
    {
        "env": "stm32h743-devebox-quad", "vehicle": "copter", "chip": "h743", "limit": H743_LIMIT,
        "title": "DevEBox H743", "title_en": "DevEBox H743",
        "note": "голая плата, датчики на I2C/SPI", "note_en": "bare board, sensors on I2C/SPI",
    },
    {
        "env": "matek-h743", "vehicle": "plane", "chip": "h743", "limit": H743_LIMIT,
        "title": "Matek H743", "title_en": "Matek H743",
        "note": "самолёт, настройки по умолчанию", "note_en": "plane, default settings",
    },
    {
        "env": "stm32h743-devebox", "vehicle": "plane", "chip": "h743", "limit": H743_LIMIT,
        "title": "DevEBox H743", "title_en": "DevEBox H743",
        "note": "самолёт, настройки по умолчанию", "note_en": "plane, default settings",
    },
]


def run(cmd: list[str], cwd: Path = ROOT, capture: bool = False) -> str:
    print("$", " ".join(cmd))
    r = subprocess.run(cmd, cwd=cwd, check=True, text=True, encoding="utf-8", errors="replace",
                       capture_output=capture)
    return r.stdout.strip() if capture else ""


def pio() -> str:
    exe = shutil.which("pio")
    if exe:
        return exe
    fallback = Path.home() / ".platformio" / "penv" / ("Scripts/pio.exe" if sys.platform == "win32" else "bin/pio")
    if fallback.exists():
        return str(fallback)
    sys.exit("pio не найден: добавьте ~/.platformio/penv/Scripts в PATH")


def build_site(out: Path, build: bool) -> None:
    if build:
        cmd = [pio(), "run"]
        for f in FIRMWARE:
            cmd += ["-e", f["env"]]
        run(cmd)

    commit = run(["git", "rev-parse", "--short", "HEAD"], capture=True)
    dirty = bool(run(["git", "status", "--porcelain", "--untracked-files=no"], capture=True))
    fw_dir = out / "firmware"
    fw_dir.mkdir(parents=True)
    shutil.copy(ROOT / "tools" / "web-flasher" / "index.html", out / "index.html")
    (out / ".nojekyll").write_text("")

    entries = []
    for f in FIRMWARE:
        src = ROOT / ".pio" / "build" / f["env"] / "firmware.bin"
        if not src.exists():
            sys.exit(f"нет {src}: соберите без --no-build")
        data = src.read_bytes()
        limit = int(f["limit"], 16)
        if 0x08000000 + len(data) > limit:
            sys.exit(f"{f['env']}: {len(data)} байт залезают в сектор настроек ({f['limit']})")
        name = f"{f['env']}.bin"
        (fw_dir / name).write_bytes(data)
        entries.append({**f, "file": name, "address": "0x08000000", "size": len(data),
                        "sha256": hashlib.sha256(data).hexdigest()})
        print(f"  {name:32} {len(data):>8} B")

    manifest = {
        "commit": commit + ("+" if dirty else ""),
        "date": dt.date.today().isoformat(),
        "firmware": entries,
    }
    (fw_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")


def push(site: Path) -> None:
    remote = run(["git", "remote", "get-url", "origin"], capture=True)
    run(["git", "init", "-q", "-b", "gh-pages"], cwd=site)
    run(["git", "add", "-A"], cwd=site)
    name = run(["git", "config", "user.name"], capture=True)
    email = run(["git", "config", "user.email"], capture=True)
    run(["git", "-c", f"user.name={name}", "-c", f"user.email={email}", "commit", "-q", "-m",
         "OpenPlane Flasher: page + prebuilt firmware"], cwd=site)
    run(["git", "push", "-f", remote, "gh-pages"], cwd=site)


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-build", action="store_true", help="не запускать pio run")
    ap.add_argument("--dry-run", action="store_true", help="только собрать сайт, не пушить")
    ap.add_argument("--out", type=Path, help="папка для сайта (по умолчанию — временная)")
    args = ap.parse_args()

    out = args.out or Path(tempfile.mkdtemp(prefix="openplane-flasher-"))
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} не пуста")
    out.mkdir(parents=True, exist_ok=True)
    build_site(out, build=not args.no_build)
    print("сайт:", out)
    if not args.dry_run:
        push(out)
        print("готово: https://damir-lebedev.github.io/OpenPlaneProject/")


if __name__ == "__main__":
    main()
