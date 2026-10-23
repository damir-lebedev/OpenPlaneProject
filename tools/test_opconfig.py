#!/usr/bin/env python3
"""
Тесты tools/opconfig.py — описания и настройки прошивки для OpenPlane Studio.

    python tools/test_opconfig.py

Каждый тест работает с копией include/ и platformio.ini во временной папке:
репозиторий не трогается. Что значения не только записываются, но и
собираются, проверяет сборка в приложении (PlatformIO); здесь — что apply
пишет ровно то, что просили, и ничего больше.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import opconfig  # noqa: E402


def read(root, rel):
    with open(os.path.join(root, rel), encoding="utf-8", newline="") as f:
        return f.read()


class OpconfigTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="opconfig-")
        shutil.copytree(os.path.join(ROOT, "include"), os.path.join(self.tmp, "include"))
        shutil.copy(os.path.join(ROOT, "platformio.ini"), self.tmp)
        self.manifest = opconfig.describe(opconfig.Source(self.tmp))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def config(self, board="stm32h743-devebox", **changes):
        cfg = opconfig.current_config(self.manifest, board)
        for key, value in changes.items():
            cfg[key] = value
        return cfg

    def apply(self, cfg, write=True):
        return opconfig.apply(opconfig.Source(self.tmp), cfg, write=write)

    def param(self, name, manifest=None, board=None):
        manifest = manifest or opconfig.describe(opconfig.Source(self.tmp))
        return next(p for p in manifest["params"] if p["name"] == name and (board is None or p["board"] == board))

    def assert_rejected(self, cfg, fragment):
        before = {rel: read(self.tmp, rel) for rel in opconfig.FILES.values()}
        with self.assertRaises(opconfig.ConfigError) as ctx:
            self.apply(cfg)
        self.assertIn(fragment, str(ctx.exception))
        for rel, text in before.items():
            self.assertEqual(text, read(self.tmp, rel), rel + " изменился, хотя apply отказал")

    # --- описание ---

    def test_check_passes_on_repository(self):
        errors, _ = opconfig.check(ROOT)
        self.assertEqual([], errors)

    def test_describe_lists_boards_modes_and_requirements(self):
        m = self.manifest
        envs = {b["env"]: b for b in m["boards"]}
        self.assertIn("stm32h743-devebox", envs)
        self.assertTrue(any(b["default"] for b in m["boards"]))
        self.assertEqual("sd", envs["stm32h743-devebox"]["blackbox"])
        self.assertEqual("mavlink", envs["stm32h743-devebox"]["telemetry"])
        self.assertEqual("none", envs["esp32-c3"]["blackbox"])
        self.assertFalse(envs["esp32-c3"]["outputs"]["rudder"])
        self.assertTrue(envs["esp32-s3"]["outputs"]["aux1"])

        modes = {x["id"]: x for x in m["modes"]}
        self.assertEqual(12, len(modes))
        self.assertEqual([], modes["MODE_MANUAL"]["needs"])
        self.assertEqual(["imu", "baro", "gps"], modes["MODE_RTH"]["needs"])
        self.assertIn("wheels", modes["MODE_AUTO_TAKEOFF"]["needs"])
        self.assertEqual("RTH", modes["MODE_RTH"]["name"])
        features = {x["id"]: x for x in m["features"]}
        self.assertEqual(["gps"], features["GEOFENCE"]["needs"])
        self.assertEqual(["esc3d"], features["MOTOR_REVERSE"]["needs"])
        self.assertEqual(8, len(m["knobs"]))

        swc = next(c for c in m["channels"]["list"] if c["name"] == "SWC")
        self.assertEqual(3, swc["positions"])
        self.assertTrue(swc["bindable"])
        self.assertFalse(next(c for c in m["channels"]["list"] if c["name"] == "SWA")["bindable"])

        self.assertIn("SENSOR_IMU_LSM6DSV", [o["id"] for o in m["sensors"]["options"]["imu"]])
        self.assertTrue(all(m["current"]["sensors"].values()))
        self.assertTrue(m["current"]["bindings"])

    def test_describe_params_have_kinds_units_and_docs(self):
        rth = self.param("RTH_ALTITUDE_M", self.manifest)
        self.assertEqual(("float", "м"), (rth["kind"], rth["unit"]))
        self.assertIn("RTH", rth["doc"])
        self.assertEqual("enum", self.param("LANDING_GEAR", self.manifest)["kind"])
        rx = self.param("RX_PROTOCOL", self.manifest)
        self.assertEqual(("enum", "RxProtocol::IBUS"), (rx["kind"], rx["value"]))
        self.assertEqual(["IBUS", "SBUS", "CRSF"], rx["options"])
        self.assertIn("ExpressLRS", rx["doc"])
        self.assertEqual("channel", self.param("CAL_SWITCH", self.manifest)["kind"])
        self.assertEqual("array", self.param("CHANNEL_REVERSED", self.manifest)["kind"])
        self.assertTrue(self.param("PWM_MIN", self.manifest)["fixed"])
        self.assertEqual(["gps"], self.param("FAILSAFE_RTH", self.manifest)["needs"])
        pin = self.param("PIN_ESC", self.manifest, board="BOARD_STM32H743")
        self.assertEqual(("pin", "Hardware pinout"), (pin["kind"], pin["group"]))
        self.assertEqual("мс", self.param("FLAPS_TRANSITION_MS", self.manifest)["unit"])
        self.assertEqual("м/с", self.param("STALL_SPEED_MS", self.manifest)["unit"])

    def test_docs_keep_line_end_dashes_and_wrapped_notes(self):
        doc = self.param("RX_FAILSAFE_THROTTLE_US", self.manifest)["doc"]
        self.assertIn("слать iBUS —\nон продолжает", doc)      # тире переноса — часть фразы
        note = self.param("MOTOR_REVERSED", self.manifest)["note"]
        self.assertTrue(note.endswith("только двунаправленный ESC — у обычного провода)"), note)
        self.assertEqual("руль высоты", self.param("ELEVATOR_REVERSED", self.manifest)["note"])

    def test_range_tags_reach_the_app_and_check_holds_values_to_them(self):
        self.assertEqual([15.0, 120.0], self.param("RTH_ALTITUDE_M", self.manifest)["range"])
        self.assertEqual([0.1, 1.0], self.param("STAB_GAIN_MIN", self.manifest)["range"])
        path = os.path.join(self.tmp, "include", "config", "Config.h")
        text = read(self.tmp, "include/config/Config.h")
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write(text.replace("RTH_ALTITUDE_M = 40.0f;", "RTH_ALTITUDE_M = 400.0f;"))
        errors, _ = opconfig.check(self.tmp)
        self.assertTrue(any("RTH_ALTITUDE_M" in e and "@range(15.0, 120.0)" in e for e in errors), errors)

    # --- запись ---

    def test_apply_current_values_changes_nothing(self):
        for board in self.manifest["boards"]:
            result = self.apply(self.config(board["env"]))
            self.assertEqual([], result["changed"], board["env"])

    def test_apply_changes_one_line_per_parameter(self):
        before = read(self.tmp, opconfig.FILES["config"])
        value = self.param("RTH_ALTITUDE_M", self.manifest)["value"] + 15
        result = self.apply(self.config(params={"RTH_ALTITUDE_M": value}))
        self.assertEqual([opconfig.FILES["config"]], result["changed"])
        after = read(self.tmp, opconfig.FILES["config"])
        changed = [(a, b) for a, b in zip(before.splitlines(), after.splitlines()) if a != b]
        self.assertEqual(1, len(changed))
        self.assertIn("RTH_ALTITUDE_M = {}f;".format(repr(float(value))), changed[0][1])
        self.assertEqual(value, self.param("RTH_ALTITUDE_M")["value"])

    def test_apply_every_kind_of_value(self):
        m = self.manifest
        left = self.param("AILERON_LEFT_REVERSED", m)["value"]
        self.apply(self.config(params={
            "AILERON_LEFT_REVERSED": not left,
            "LANDING_GEAR": "NOSE_WHEEL",
            "RX_PROTOCOL": "CRSF",
            "WIFI_AP_SSID": 'Борт "№1" \\ тест',
            "CAL_SWITCH": "SWD",
            "CHANNEL_REVERSED": [False] * 8 + [True, False],
            "FLAPS_TRANSITION_MS": 1500,
            "CAMERA_US_PER_DEG": 5.0,
        }))
        after = opconfig.describe(opconfig.Source(self.tmp))
        self.assertEqual(not left, self.param("AILERON_LEFT_REVERSED", after)["value"])
        self.assertEqual("LandingGear::NOSE_WHEEL", self.param("LANDING_GEAR", after)["value"])
        self.assertEqual("RxProtocol::CRSF", self.param("RX_PROTOCOL", after)["value"])
        self.assertEqual('Борт "№1" \\ тест', self.param("WIFI_AP_SSID", after)["value"])
        self.assertEqual("Channels::SWD", self.param("CAL_SWITCH", after)["value"])
        self.assertEqual([False] * 8 + [True, False], self.param("CHANNEL_REVERSED", after)["value"])
        self.assertEqual(1500, self.param("FLAPS_TRANSITION_MS", after)["value"])
        text = read(self.tmp, opconfig.FILES["config"])
        self.assertIn('WIFI_AP_SSID = "Борт \\"№1\\" \\\\ тест";', text)
        self.assertIn("CAMERA_US_PER_DEG = 5.0f;", text)
        self.assertIn("// CH9  — VrA", text)   # комментарии элементов массива на месте

    def test_apply_board_pins_only_in_that_board_block(self):
        esp = read(self.tmp, opconfig.FILES["config"])
        self.apply(self.config("stm32h743-devebox", params={"PIN_AUX1": "PE10"}))
        self.assertEqual("PE10", self.param("PIN_AUX1", board="BOARD_STM32H743")["value"])
        self.assertEqual(self.param("PIN_AUX1", self.manifest, board="BOARD_ESP32_S3")["value"],
                         self.param("PIN_AUX1", board="BOARD_ESP32_S3")["value"])
        diff = [l for l in read(self.tmp, opconfig.FILES["config"]).splitlines() if l not in esp.splitlines()]
        self.assertEqual(1, len(diff))
        self.apply(self.config("esp32-s3", params={"PIN_AUX1": 14}))
        self.assertEqual(14, self.param("PIN_AUX1", board="BOARD_ESP32_S3")["value"])

    def test_apply_rejects_bad_values_and_writes_nothing(self):
        cases = [
            ({"NO_SUCH_PARAM": 1}, "такого параметра нет"),
            ({"PWM_MIN": 900}, "менять нельзя"),
            ({"RTH_ALTITUDE_M": "высоко"}, "нужно число"),
            ({"THROTTLE_LIMIT_PCT": 70000}, "вне пределов"),
            ({"PIN_AUX1": "PZ9"}, "пин вида PA3"),
            ({"FAILSAFE_RTH": 1}, "true или false"),
            ({"LANDING_GEAR": "SKIDS"}, "нет варианта"),
            ({"CAL_SWITCH": "SWX"}, "нет канала"),
            ({"CHANNEL_REVERSED": [True]}, "список из 10"),
        ]
        for params, fragment in cases:
            with self.subTest(params=params):
                self.assert_rejected(self.config(params=params), fragment)
        unknown_board = self.config()
        unknown_board["board"] = "esp32-nope"
        self.assert_rejected(unknown_board, "нет окружения")

    def test_apply_rejects_bindings_the_firmware_would_not_compile(self):
        sticks = [{"kind": "feature", "channel": "SWA", "feature": "FLAPS"}]
        self.assert_rejected(self.config(bindings=sticks), "занимать нельзя")
        twice = [{"kind": "feature", "channel": "SWB", "feature": "FLAPS"},
                 {"kind": "knob", "channel": "SWB", "knob": "RATES"}]
        self.assert_rejected(self.config(bindings=twice), "уже есть привязка")
        two_switches = [{"kind": "modes", "channel": "SWC", "modes": ["MODE_MANUAL", "MODE_STABILIZE"]},
                        {"kind": "modes", "channel": "SWD", "modes": ["MODE_MANUAL", "MODE_ACRO"]}]
        self.assert_rejected(self.config(bindings=two_switches), "только один")
        self.assert_rejected(self.config(bindings=[]), "хотя бы одна")
        unknown = [{"kind": "mode", "channel": "SWD", "mode": "MODE_WARP"}]
        self.assert_rejected(self.config(bindings=unknown), "MODE_WARP")

    def test_apply_bindings_rewrites_the_table_and_keeps_ideas(self):
        old = read(self.tmp, opconfig.FILES["controls"])
        bindings = [
            {"kind": "modes", "channel": "SWC", "modes": ["MODE_STABILIZE", "MODE_CRUISE", "MODE_LOITER"]},
            {"kind": "mode", "channel": "SWD", "mode": "MODE_RTH"},
            {"kind": "feature", "channel": "SWB", "feature": "PAYLOAD_DROP"},
            {"kind": "knob", "channel": "VRA", "knob": "STAB_GAIN"},
        ]
        self.apply(self.config(bindings=bindings))
        text = read(self.tmp, opconfig.FILES["controls"])
        self.assertIn("        Bind::mode   (Channels::SWD, MODE_RTH),", text)
        self.assertIn("        Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),", text)
        self.assertIn("// Идеи — раскомментируйте", text)
        self.assertEqual(old.count("// Bind::"), text.count("// Bind::"))
        self.assertEqual(bindings, opconfig.describe(opconfig.Source(self.tmp))["current"]["bindings"])

        # Строка, которая не менялась, остаётся как была.
        vra = next(l for l in old.splitlines() if "Channels::VRA" in l and not l.strip().startswith("//"))
        self.assertIn(vra, text.splitlines())

    def test_apply_sensors_uses_a_kit_or_writes_own_block(self):
        kits = {k["id"]: k["sensors"] for k in self.manifest["sensors"]["kits"] if k["sensors"]}
        original = read(self.tmp, opconfig.FILES["sensors"])

        self.apply(self.config(sensors=kits["SENSOR_KIT_LSM6DSV_PITOT"]))
        text = read(self.tmp, opconfig.FILES["sensors"])
        self.assertIn("#define SENSOR_KIT SENSOR_KIT_LSM6DSV_PITOT", text)
        self.assertNotIn(opconfig.STUDIO_BEGIN, text)

        custom = dict(kits["SENSOR_KIT_LSM6DSV_PITOT"], gps="SENSOR_GPS_NONE", mag="SENSOR_MAG_NONE")
        self.apply(self.config(sensors=custom))
        text = read(self.tmp, opconfig.FILES["sensors"])
        self.assertIn("#define SENSOR_KIT SENSOR_KIT_CUSTOM", text)
        self.assertIn("#define SENSOR_GPS SENSOR_GPS_NONE", text)
        self.assertEqual(custom, opconfig.describe(opconfig.Source(self.tmp))["current"]["sensors"])

        self.apply(self.config(sensors=self.manifest["current"]["sensors"]))
        self.assertEqual(original, read(self.tmp, opconfig.FILES["sensors"]))
        self.assert_rejected(self.config(sensors=dict(custom, imu="SENSOR_IMU_BNO085")), "нет варианта")

    def test_board_with_own_sensor_kit(self):
        """Matek H743: набор датчиков платы — флагом env, строка SENSOR_KIT ему не указ."""
        kits = {k["id"]: k["sensors"] for k in self.manifest["sensors"]["kits"] if k["sensors"]}
        boards = {b["env"]: b for b in self.manifest["boards"]}
        self.assertEqual("SENSOR_KIT_MATEK_H743", boards["matek-h743"]["kit"])
        self.assertEqual(kits["SENSOR_KIT_MATEK_H743"], boards["matek-h743"]["sensors"])
        self.assertIsNone(boards["stm32h743-devebox"]["kit"])
        self.assertEqual(self.manifest["current"]["sensors"], boards["stm32h743-devebox"]["sensors"])
        self.assertEqual(kits["SENSOR_KIT_MATEK_H743"], self.config("matek-h743")["sensors"])
        original = read(self.tmp, opconfig.FILES["sensors"])

        self.assertEqual([], self.apply(self.config("matek-h743"))["changed"])

        # Компас на внешний разъём: набор платы + строки SENSOR_*; строка SENSOR_KIT не тронута.
        with_mag = dict(kits["SENSOR_KIT_MATEK_H743"], mag="SENSOR_MAG_QMC5883L")
        self.apply(self.config("matek-h743", sensors=with_mag))
        text = read(self.tmp, opconfig.FILES["sensors"])
        self.assertIn("#define SENSOR_MAG SENSOR_MAG_QMC5883L", text)
        self.assertIn("#define SENSOR_IMU SENSOR_IMU_ICM42688", text)
        self.assertEqual(original.count("#define SENSOR_KIT SENSOR_KIT_BENCH_GY521"),
                         text.count("#define SENSOR_KIT SENSOR_KIT_BENCH_GY521"))
        manifest = opconfig.describe(opconfig.Source(self.tmp))
        self.assertEqual(with_mag, next(b for b in manifest["boards"] if b["env"] == "matek-h743")["sensors"])

        # Чужой набор целиком — тоже строками SENSOR_*: строку SENSOR_KIT перекрыл бы флаг.
        self.apply(self.config("matek-h743", sensors=kits["SENSOR_KIT_LSM6DSV_PITOT"]))
        text = read(self.tmp, opconfig.FILES["sensors"])
        self.assertIn(opconfig.STUDIO_BEGIN, text)
        self.assertIn("#define SENSOR_IMU SENSOR_IMU_LSM6DSV", text)

        # Обратно к набору платы — файл как был.
        self.apply(self.config("matek-h743", sensors=kits["SENSOR_KIT_MATEK_H743"]))
        self.assertEqual(original, read(self.tmp, opconfig.FILES["sensors"]))

    def test_apply_writes_project_header(self):
        cfg = self.config(project={"plane": 'Astro "Cargo" \\1', "configHash": "abc123", "commit": "75d8784"})
        result = self.apply(cfg)
        self.assertIn(opconfig.PROJECT_HEADER, result["changed"])
        header = read(self.tmp, opconfig.PROJECT_HEADER)
        self.assertIn('#define OP_PLANE_NAME "Astro \\"Cargo\\" \\\\1"', header)
        self.assertIn('#define OP_BOARD_ENV "stm32h743-devebox"', header)
        self.assertIn('#define OP_CONFIG_HASH "abc123"', header)
        self.assertNotIn(opconfig.PROJECT_HEADER, self.apply(cfg)["changed"])   # то же — не перезаписывает

    def test_apply_keeps_crlf_line_endings(self):
        path = os.path.join(self.tmp, opconfig.FILES["config"])
        text = read(self.tmp, opconfig.FILES["config"]).replace("\r\n", "\n").replace("\n", "\r\n")
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        self.apply(self.config(params={"RTH_ALTITUDE_M": 77.0}))
        after = read(self.tmp, opconfig.FILES["config"])
        self.assertNotIn("\n", after.replace("\r\n", ""))
        self.assertIn("RTH_ALTITUDE_M = 77.0f;  // @range(15, 120)\r\n", after)   # метка в комментарии осталась

    def test_command_line_apply_reports_errors_as_json(self):
        cfg_path = os.path.join(self.tmp, "cfg.json")
        with open(cfg_path, "w", encoding="utf-8") as f:
            json.dump(self.config(params={"PWM_MIN": 1}), f)
        from io import StringIO
        out, old = StringIO(), sys.stdout
        sys.stdout = out
        try:
            code = opconfig.main(["apply", "--root", self.tmp, "--config", cfg_path])
        finally:
            sys.stdout = old
        self.assertEqual(2, code)
        result = json.loads(out.getvalue())
        self.assertFalse(result["ok"])
        self.assertIn("менять нельзя", result["errors"][0])



class CopterManifestTest(unittest.TestCase):
    """Сборки коптера — отдельным списком: среди плат самолёта их нет."""

    def test_copter_envs_and_builds(self):
        manifest = opconfig.describe(opconfig.Source(ROOT))
        planes = {b["env"] for b in manifest["boards"]}
        copters = {c["env"]: c for c in manifest["copters"]["envs"]}
        self.assertNotIn("getfun-f405v3-quad", planes)
        getfun = copters["getfun-f405v3-quad"]
        self.assertEqual("longrange7li", getfun["build"])
        self.assertEqual("MULTISHOT", getfun["esc"])
        self.assertEqual("dfu", getfun["uploadProtocol"])
        self.assertTrue(getfun["needsBoardDump"])   # заглушка с #error, пока нет дампа с платы
        self.assertIn("GETFUN", getfun["title"])
        self.assertFalse(copters["aocodarc-f405v3-quad"]["needsBoardDump"])
        builds = {b["id"]: b for b in manifest["copters"]["builds"]}
        self.assertEqual("QUAD_X", builds["longrange7li"]["frame"])
        self.assertIn("Li-ion", builds["longrange7li"]["title"])
        self.assertEqual('Фристайл 5", 6S', builds["freestyle5"]["title"])

if __name__ == "__main__":
    unittest.main(verbosity=1)
