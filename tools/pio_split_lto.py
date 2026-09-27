# Линковка с LTO в ветке feature/split-headers (post-скрипт сред ESP32 и
# STM32 в platformio.ini; post — потому что сборщик платформы задаёт
# LINKFLAGS заново, и флаг из pre-скрипта теряется).
#
# Код проекта компилируется с -flto (build_src_flags), и компоновщику
# нужен тот же флаг. Зачем: в header-only ветке прошивка — одна единица
# трансляции, и компилятор встраивает мелкие функции драйверов прямо в
# полётный цикл. Здесь они разнесены по .cpp, и без LTO прошивка на
# 1-2% больше (docs/SPLIT_HEADERS.md). LTO только для src/: ядро Arduino
# и FreeRTOS зовут app_main() и vTaskSwitchContext() из ассемблера, и
# LTO по всему проекту выбрасывает их как неиспользуемые.
Import("env")  # noqa: F821 — объект SCons, его подставляет PlatformIO

# GCC 12 (STM32) без =auto предупреждает о последовательной сборке;
# GCC 8 (ESP32) =auto не знает.
env.Append(LINKFLAGS=["-flto=auto" if env["PIOPLATFORM"] == "ststm32" else "-flto"])  # noqa: F821
