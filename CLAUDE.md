# OpenPlane — карта для работы с кодом

Короткая карта, чтобы не изучать репозиторий заново. Подробности — в
`docs/` (ARCHITECTURE, DEVELOPER_GUIDE, TESTING, reference/*). Ответы, комментарии
и документация — по-русски. Весь код — header-only C++ в `include/`, точки входа
в `src/`.

## Сборки и точки входа

| env (platformio.ini) | Что | Точка входа |
|---|---|---|
| `stm32h743-devebox` | основная плата (самолёт), USB CDC, DFU | `src/stm32/main.cpp` |
| `matek-h743` | готовый полётник, датчики на плате (`SENSOR_KIT_MATEK_H743`) | то же |
| `*-quad`, `aocodarc-f405v3-quad` | мультикоптер (`VEHICLE_MULTIROTOR`) | `src/stm32/main_copter.cpp` |
| `esp32-s3/c3/dev` | прежние платы ESP32 | `src/main.cpp` |
| `native`, `native-stm32` | вся прошивка на ПК с фейками железа | `test/native*/` |

`getfun-f405v3-quad` не собирается специально: нет карты пинов (`#error`).

## Слои (сверху вниз)

- **Склейка** `src/*/main.cpp`: создаёт всё, задачи FreeRTOS (flight 5, bbox 2,
  oled/storage 1), `setupSensors()`: IMU → калибровка → барометр → компас → GPS.
- **Управление** `include/control/`: `FlightController` (такт: приёмник →
  тумблеры → автопилот → ARM/реверс → микшер → `ThrottleRamp` → выходы;
  приоритет: потеря связи > ARM > режим > газ), `ArmingManager`, `ControlMixer`,
  `FlightOutputs` (таблица выходов, `EscSignal` для 3D-ESC), `MotorReverse`,
  `GroundCalibration` (горизонт/стоянка жестом SwB), `GyroStickCalibration`
  (ноль гироскопа стиками), `SignalPlayer` (кивки рулей).
- **Автопилот** `include/autopilot/`: `Autopilot`, режимы, `PilotSwitches` +
  `ControlBinding` (тумблеры из `config/Controls.h`), `feedback/` (взлёт/посадка).
- **Датчики** `include/sensors/`: интерфейсы `SensorInterface.h`; выбор —
  `SensorSelection.h` (`SENSOR_IMU`, `_2`, `_3` и т.д., трейты `ImuChip<id>`…,
  `SelectedSensors<Board>`); `SensorChain.h` — первый ответивший из выбранных;
  драйверы — `imu/ baro/ mag/ gps/ airspeed/`, база с калибровкой —
  `imu/ImuSensorBase.h`, `mag/MagnetometerBase.h`, `baro/BarometerBase.h`.
- **HAL** `include/hal/`: `IBoard` (шины по номеру `i2cBus(0..3)`: основная,
  вторая `PIN_I2C2_*`, внешний разъём, за IMU), `II2CBus` (замок, `recover()`),
  `RegisterDevice.h` (`I2cRegisterDevice` ищет чип на всех шинах,
  `nextPlacement()`), реализации `esp32/`, `stm32/`.
- **Телеметрия** `include/telemetry/`: `DebugConsole` (клавиши, `b` — опрос шин),
  `AppLink` (`@live` для Studio), `BlackBox*`, `MavlinkTelemetry`, `OledDisplay`.
- **Коптер** `include/copter/*` — отдельная логика (Params.h — реестр настроек).
- **Настройки** `include/config/Config.h`: блоки пинов по платам + общие
  константы. Аннотации в комментариях `@range(a,b)`, `@fixed`, `@channel` —
  их читает `tools/opconfig.py`, Studio показывает параметры сам.

## OpenPlane Studio (../openplane-studio)

- Прошивает из **своей копии исходников**: `%LOCALAPPDATA%\com.openplane.studio\
  source\OpenPlaneProject` (метка `.openplane-source.json`). После пуша прошивки
  её обновить: «Настройки → Скачать с GitHub» (берёт main, тегов нет) или
  `git archive origin/main include lib src tools platformio.ini
  partitions_blackbox.csv` туда же + метка с полным хешом.
- Конфиг самолёта: `Documents/OpenPlane/Hangar/plane-*/plane.json`, конфиг
  каждой прошивки — `history/*-flash-*.json` → `config`. `opconfig.py apply
  --config cfg.json --root <копия>` пишет его в исходники (Config.h,
  SensorSelection.h, Controls.h, Project.h). Ручная прошивка — только так, на
  копии в scratchpad.
- Studio держит COM13, пока открыт. Закрывать его самому нельзя (запрет) —
  просить пользователя.
- Логика Studio: `src/core/*.ts` (+ тесты vitest), датчики —
  `components/board/steps/SensorsStep.tsx`, проверка — `core/live.ts`,
  конфиг для прошивки — `core/project.ts:toFirmwareConfig`. Переводы:
  `src/locales/en*.json`, проверка `python tools/i18n_check.py` (формат
  файлов сохранять: en.json отступ 2, en.firmware.json — 1).

## Команды

```
export PATH="$HOME/tools/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH"
pio run -e stm32h743-devebox                      # сборка
pio test -e native -e native-stm32                # ~7 мин, все нативные тесты
python -m unittest tools.test_opconfig            # opconfig
cd ../openplane-studio && npx tsc --noEmit && npx vitest run
```

Прошивка платы: в консоли клавиша `D` → DFU, затем
`pio run -d <копия> -e stm32h743-devebox -t upload`. Консоль — COM13, нужен DTR;
pyserial есть только в `~/.platformio/penv/Scripts/python.exe`. Клавиши: `s`
датчики, `b` шины, `i` калибровка гироскопа, `l` меню лога, `D` DFU.
`\x02live\n` / `\x02live 0\n` — живые кадры (`"hz"`, `"imu"`, `"mag"`, `"ch"`).

## Ловушки

- Heredoc в Bash съедает `\n` внутри строк C — патчи писать файлом (Write)
  и запускать `python файл.py`.
- Документация: русская — источник, переводы в `docs/i18n/<lang>/` (11 языков)
  правятся вручную.
- После прошивки, которая «повесила» датчик, часто нужно передёрнуть питание
  (просить пользователя переткнуть USB): SW_RESET LSM6DSV банк sensor hub не
  сбрасывает, зависший ведомый держит шину.
- LSM6DSV: фильтры помех `IF_CFG.ASF_CTRL` обязательны (без них отваливался с
  шины). Компас QMC6309 на модуле «LSM6DSV + QMC6309» — за LSM6DSV, на его
  вспомогательной шине: `sensors/imu/Lsm6dsvAuxBus.h` (мастер sensor hub; НЕ
  использовать WRITE_ONCE — после него мастер больше не читает; pass-through
  на этом модуле не работает; QMC6309 soft reset не делать).
- Проверка «двигали» при калибровке гироскопа — по уходу среднего между
  четвертями (`GYRO_CAL_DRIFT_DPS`), а не по разбросу: вибрация в самолёте
  даёт разброс ±2 °/с при стоящем нуле.
- Коммиты: на main, сообщение по-английски, в конце строка
  `Co-Authored-By` из системного напоминания; после большой работы — README,
  docs, пуш без вопросов.
