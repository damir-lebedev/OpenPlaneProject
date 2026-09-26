#!/usr/bin/env bash
# clang-tidy по всем заголовкам и исходникам прошивки (профиль —
# .clang-tidy в корне). Заголовки ESP-IDF clang под хост разобрать не
# может, поэтому анализ идёт с нативными фейками Arduino/ESP32 из
# test/native/support — как в нативных тестах (docs/TESTING.md).
# Код под STM32 (include/hal/stm32/, src/stm32/) фейками не покрыт —
# его проверяет сборка pio run -e stm32h743 (-Wall -Wextra).
#
#   tools/clang-tidy.sh            — все файлы include/ и src/
#   tools/clang-tidy.sh FILE...    — только указанные
#
# Код возврата 1, если есть замечания.
set -euo pipefail
cd "$(dirname "$0")/.."

CLANG_TIDY="${CLANG_TIDY:-clang-tidy}"
if [ "$#" -gt 0 ]; then
    files=("$@")
else
    mapfile -t files < <(find include src -path '*/stm32' -prune -o \( -name '*.h' -o -name '*.cpp' \) -print | sort)
fi

findings=0
for file in "${files[@]}"; do
    output=$("$CLANG_TIDY" "$file" --quiet -- -x c++ -std=gnu++17 -DBOARD_ESP32_S3 \
        -Iinclude -Itest/native/support 2>/dev/null | grep -E "warning:|error:" || true)
    if [ -n "$output" ]; then
        echo "$output"
        findings=$((findings + $(echo "$output" | wc -l)))
    fi
done

echo "clang-tidy: ${#files[@]} файлов, замечаний: $findings"
[ "$findings" -eq 0 ]
