# pio test на плате в ветке feature/split-headers (pre-скрипт сред ESP32
# и STM32 в platformio.ini). Тесту нужна реализация из src/core/, но не
# точка входа прошивки: у теста свои setup()/loop(). Поэтому при
# pio test фильтр исходников меняется на custom_test_src_filter среды.
Import("env")  # noqa: F821 — объект SCons, его подставляет PlatformIO

if "test" in env["BUILD_TYPE"]:  # noqa: F821
    env.Replace(SRC_FILTER=env.GetProjectOption("custom_test_src_filter"))  # noqa: F821
