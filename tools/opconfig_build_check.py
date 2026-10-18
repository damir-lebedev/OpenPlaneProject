#!/usr/bin/env python3
"""
Сборка конфигураций, которые пишет tools/opconfig.py apply, на каждой плате.

    python tools/opconfig_build_check.py                 # все платы
    python tools/opconfig_build_check.py esp32-s3 stm32h743

tools/test_opconfig.py проверяет, что apply пишет ровно то, что просили;
здесь — что записанное компилируется. Для каждой платы во временной папке:
копия исходников → apply «тяжёлой» конфигурации (все виды параметров,
свой набор датчиков, привязки всех видов, config/Project.h) → pio run.
Итог — таблица «плата: OK / FAIL», логи — в .pio/opconfig-check/.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import opconfig  # noqa: E402

DEFAULT_BOARDS = ["esp32-s3", "esp32-dev", "esp32-c3", "stm32h743"]


def heavy_config(manifest, board):
    cfg = opconfig.current_config(manifest, board)
    env = next(b for b in manifest["boards"] if b["env"] == board)
    params = cfg["params"]
    params.update({
        "RTH_ALTITUDE_M": 55.5,              # float
        "FLAPS_TRANSITION_MS": 1500,         # uint32
        "THROTTLE_LIMIT_PCT": 80,            # uint16
        "FAILSAFE_RTH": False,               # bool
        "LANDING_GEAR": "NOSE_WHEEL",        # enum
        "WIFI_AP_SSID": 'Борт "№1"',         # строка с кавычками и кириллицей
        "CAL_SWITCH": "SWD",                 # канал
        "CHANNEL_REVERSED": [False] * 8 + [True, False],  # массив
    })
    if env["outputs"]["aux1"]:
        params["AILERON_LEFT_REVERSED"] = not params["AILERON_LEFT_REVERSED"]
    cfg["sensors"] = {
        "imu": "SENSOR_IMU_LSM6DSV_SPI",
        "baro": "SENSOR_BARO_SPL06",
        "mag": "SENSOR_MAG_QMC6309",
        "airspeed": "SENSOR_AIRSPEED_PITOT_BMP581",
        "gps": "SENSOR_GPS_UBLOX_M10",
    }
    bindings = [
        {"kind": "modes", "channel": "SWC", "modes": ["MODE_MANUAL", "MODE_STABILIZE", "MODE_CRUISE"]},
        {"kind": "mode", "channel": "SWD", "mode": "MODE_RTH"},
        {"kind": "feature", "channel": "SWB", "feature": "FLAPS"},
        {"kind": "knob", "channel": "VRA", "knob": "RATES"},
        {"kind": "knob", "channel": "VRB", "knob": "LOITER_RADIUS"},
    ]
    cfg["bindings"] = bindings
    cfg["project"] = {"plane": "Проверка opconfig", "configHash": "check0000001", "commit": "check"}
    return cfg


def pio():
    scripts = os.path.join(os.path.expanduser("~"), ".platformio", "penv", "Scripts" if os.name == "nt" else "bin")
    for name in ("pio.exe", "pio", "platformio.exe", "platformio"):
        path = os.path.join(scripts, name)
        if os.path.isfile(path):
            return path
    return shutil.which("pio") or "pio"


def main(argv):
    boards = argv or DEFAULT_BOARDS
    manifest = opconfig.describe(opconfig.Source(ROOT))
    logs = os.path.join(ROOT, ".pio", "opconfig-check")
    os.makedirs(logs, exist_ok=True)
    work = tempfile.mkdtemp(prefix="opconfig-build-")
    results = {}
    try:
        for name in ("include", "src", "lib"):
            shutil.copytree(os.path.join(ROOT, name), os.path.join(work, name))
        for name in os.listdir(ROOT):
            if name.endswith((".ini", ".csv")):
                shutil.copy(os.path.join(ROOT, name), work)
        pristine = {rel: open(os.path.join(work, rel), "rb").read() for rel in opconfig.FILES.values()
                    if rel != "platformio.ini"}
        for board in boards:
            for rel, data in pristine.items():
                with open(os.path.join(work, rel), "wb") as f:
                    f.write(data)
            cfg = heavy_config(manifest, board)
            try:
                applied = opconfig.apply(opconfig.Source(work), cfg)
            except opconfig.ConfigError as e:
                results[board] = "FAIL (apply: {})".format(str(e).replace("\n", "; "))
                print("{:12} {}".format(board, results[board]), flush=True)
                continue
            log = os.path.join(logs, board + ".log")
            env = dict(os.environ, PLATFORMIO_BUILD_DIR=os.path.join(work, ".pio", "build"))
            with open(log, "w", encoding="utf-8") as f:
                f.write(json.dumps(applied, ensure_ascii=False) + "\n")
                f.flush()
                code = subprocess.call([pio(), "run", "-e", board, "-d", work], stdout=f, stderr=subprocess.STDOUT, env=env)
            results[board] = "OK" if code == 0 else "FAIL (лог: {})".format(os.path.relpath(log, ROOT))
            print("{:12} {}".format(board, results[board]), flush=True)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return 0 if all(v == "OK" for v in results.values()) else 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main(sys.argv[1:]))
