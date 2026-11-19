# Линковка симулятора (env:sitl). На Windows exe должен собираться статически:
# Studio запускает его не из консоли MinGW, и без -static Windows не находит
# libstdc++-6.dll, libgcc_s_seh-1.dll, libwinpthread-1.dll. Флаг в build_flags
# сюда не годится — PlatformIO отдаёт "-static" только компилятору, а не
# линковщику, поэтому он добавляется здесь, в LINKFLAGS.
import sys

Import("env")  # noqa: F821 — SCons

if sys.platform == "win32":
    env.Append(LINKFLAGS=["-static"])  # noqa: F821
