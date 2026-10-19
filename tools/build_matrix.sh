#!/usr/bin/env bash
# ============================================================
# Матрица сборок: каждая плата × каждый набор датчиков.
#
# Собирает прошивку для esp32-s3, esp32-dev (38 pin), esp32-c3,
# stm32h743 и matek-h743 со всеми поддерживаемыми датчиками — готовыми
# наборами (SENSOR_KIT_*) и отдельными вариантами шин (I2C/SPI). У
# полётника с датчиками на плате (matek-h743: набор задан в его env)
# вместо чужих наборов — свой ("board-kit"), отдельные варианты — поверх
# него. ESP32 — с -Wall -Wextra -Wshadow, STM32 — с -Wall -Wextra
# (build_src_flags; -Wshadow шумит в заголовках ядра). Любое
# предупреждение в коде проекта (include/, src/) — ошибка матрицы.
#
#   tools/build_matrix.sh                 # всё
#   tools/build_matrix.sh esp32-s3        # одна плата
#   BOARDS="esp32-s3 stm32h743" tools/build_matrix.sh
#
# Итог — таблица "плата × конфигурация: OK / WARN / FAIL" и размер
# прошивки; логи — в .pio/matrix/. Сборка идёт в свой каталог
# (.pio/matrix-build): PlatformIO чистит каталог сборки целиком, когда
# меняются флаги, и общий .pio/build мешал бы тестам, идущим рядом.
# ============================================================
set -uo pipefail
cd "$(dirname "$0")/.."

BOARDS=${BOARDS:-${1:-"esp32-s3 esp32-dev esp32-c3 stm32h743 matek-h743"}}

# Платы с набором датчиков в своём env (-D SENSOR_KIT=… в platformio.ini).
KIT_BOARDS=" matek-h743 "
is_stm32() { [[ $1 == stm32* || $1 == matek-* ]]; }

# имя | флаги препроцессора
CONFIGS=(
  "bench-gy521|-DSENSOR_KIT=SENSOR_KIT_BENCH_GY521"
  "lsm6dsv-pitot|-DSENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT"
  "icm45686-pitot|-DSENSOR_KIT=SENSOR_KIT_ICM45686_PITOT"
  "lsm6dsv-spi+spl06-spi|-DSENSOR_KIT=SENSOR_KIT_CUSTOM -DSENSOR_IMU=SENSOR_IMU_LSM6DSV_SPI -DSENSOR_BARO=SENSOR_BARO_SPL06_SPI -DSENSOR_MAG=SENSOR_MAG_QMC6309 -DSENSOR_AIRSPEED=SENSOR_AIRSPEED_PITOT_BMP581 -DSENSOR_GPS=SENSOR_GPS_UBLOX_M10"
  "icm45686-spi+bmp581-spi|-DSENSOR_KIT=SENSOR_KIT_CUSTOM -DSENSOR_IMU=SENSOR_IMU_ICM45686_SPI -DSENSOR_BARO=SENSOR_BARO_BMP581_SPI -DSENSOR_MAG=SENSOR_MAG_QMC6309 -DSENSOR_AIRSPEED=SENSOR_AIRSPEED_NONE -DSENSOR_GPS=SENSOR_GPS_UBLOX_M10"
  "icm45686+bmp581-i2c|-DSENSOR_KIT=SENSOR_KIT_CUSTOM -DSENSOR_IMU=SENSOR_IMU_ICM45686 -DSENSOR_BARO=SENSOR_BARO_BMP581 -DSENSOR_MAG=SENSOR_MAG_QMC6309 -DSENSOR_AIRSPEED=SENSOR_AIRSPEED_NONE -DSENSOR_GPS=SENSOR_GPS_NONE"
  "icm42688-spi+dps310|-DSENSOR_KIT=SENSOR_KIT_CUSTOM -DSENSOR_IMU=SENSOR_IMU_ICM42688 -DSENSOR_BARO=SENSOR_BARO_DPS310 -DSENSOR_MAG=SENSOR_MAG_QMC5883L -DSENSOR_AIRSPEED=SENSOR_AIRSPEED_NONE -DSENSOR_GPS=SENSOR_GPS_UBLOX_M10"
)

mkdir -p .pio/matrix
export PLATFORMIO_BUILD_DIR="$PWD/.pio/matrix-build"
declare -A RESULT
failed=0

for board in $BOARDS; do
  configs=("${CONFIGS[@]}")
  if [[ $KIT_BOARDS == *" $board "* ]]; then
    # свой набор — без флагов; чужие наборы плате ни к чему; отдельные
    # варианты — без SENSOR_KIT (его задаёт env, строки SENSOR_* сильнее)
    configs=("board-kit|")
    for entry in "${CONFIGS[@]}"; do
      [[ $entry == *SENSOR_KIT_CUSTOM* ]] && configs+=("${entry/-DSENSOR_KIT=SENSOR_KIT_CUSTOM /}")
    done
  fi
  for entry in "${configs[@]}"; do
    name=${entry%%|*}
    flags=${entry#*|}
    log=".pio/matrix/${board}__${name}.log"
    if is_stm32 "$board"; then
      PLATFORMIO_BUILD_FLAGS="$flags" pio run -e "$board" >"$log" 2>&1
    else
      PLATFORMIO_BUILD_FLAGS="$flags" PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" \
        pio run -e "$board" >"$log" 2>&1
    fi
    status=$?
    warnings=$(grep -E "^(include|src)/.*warning:" "$log" | sort -u | wc -l)
    flash=$(grep -oE "Flash: .*used [0-9]+ bytes" "$log" | grep -oE "[0-9]+ bytes" | head -1)
    if [[ $status -ne 0 ]]; then
      RESULT["$board|$name"]="FAIL"
      failed=1
    elif [[ $warnings -gt 0 ]]; then
      RESULT["$board|$name"]="WARN($warnings)"
      failed=1
    else
      RESULT["$board|$name"]="OK ${flash}"
    fi
    printf '%-11s %-26s %s\n' "$board" "$name" "${RESULT["$board|$name"]}"
  done
done

echo
echo "Логи: .pio/matrix/. Предупреждения проекта: grep -E '^(include|src)/.*warning:' .pio/matrix/*.log"
exit $failed
