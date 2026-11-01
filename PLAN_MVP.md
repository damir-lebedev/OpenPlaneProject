# PLAN_MVP.md — что осталось до MVP прошивки дрона

Документ для продолжения работы после сброса чата (2026-11-01). Пользователь
пишет «продолжай по PLAN_MVP.md» — идти **сверху вниз**: раздел 0 (недоделанное
из прошлого чата, лежит в рабочем дереве без коммита), потом раздел 2 по
порядку. Подробности каждого пункта BF — в `PLAN_COPTER.md` (номера те же),
протокол батчей — в его шапке. Отвечать по-русски, коммитить по путям
(airframe и `stack/` не трогать), после большого куска — README/docs и push.

**MVP дрона** = 7" GETFUN F405V3 пользователя (детали — `PLAN_QUAD7.md`):
зайти в Studio → коптер в ангаре → настроить вкладками → прошить с
Betaflight одной кнопкой → без пропеллеров проверить → полететь в ANGLE,
при потере связи — вернуться домой по GPS. Железо приезжает ~22.11.2026,
до этого — всё на симуляторе и фейках.

---

## 0. Недоделано в прошлом чате (рабочее дерево, НЕ закоммичено)

`git status` в `plane`: изменены Config.h, CopterController.h, GpsHome.h,
Params.h, RcSetup.h, SensorSelection.h, AppLink.h, BlackBoxCore.h,
BlackBoxFormat.h, BlackBoxRing.h, CopterLink.h, src/stm32/main.cpp,
main_copter.cpp, QuadHarness.h, test_copter, test_copter_sim, blackbox.py,
opconfig.py, test_opconfig.py; новые — copter/GpsRescue.h,
hal/stm32/Stm32Camera.h, sensors/camera/Ov5640.h, telemetry/CameraStream.h,
test/native/test_camera/. В нём два независимых куска — коммитить раздельно.

### 0.1 GPS Rescue (пункты PLAN_COPTER 129, 133–136) — код готов, осталось довести

Сделано: `copter/GpsRescue.h` (этапы CLIMB → FLY_HOME → DESCEND → LAND, ABORT;
MAX_ALT/FIXED_ALT/CURRENT_ALT; проверки FS_ONLY/ON: нет GPS 2 с, мало
спутников, не приближается 15 с, улетает +50 м → посадка на месте вместо
DISARM у BF; без компаса — 1 с прямо, поправка курса гироскопа по курсу GPS,
потом медленная подстройка на прямых; посадка с удержанием точки и интегралом
крена против ветра, у земли (<1 м) — ровно). В контроллере: `CopterSensors.gps`
(GpsInput), `CopterRc.gpsRescue` (режим BF 46 с тумблера), `CopterSettings.rescue`,
`status.rescuePhase/rescueFail/rescueHeadingFix`, ARM-блок `GPS` при
failsafe_procedure = GPS-RESCUE без фикса (gps_rescue_allow_arming_without_fix),
failsafe_landing_time не считает полёт домой. 17 настроек `gps_rescue_*` (группа
gps), `@live` → `"rescue"`, main_copter заполняет GpsInput (`fillGps`,
`GpsHome::offsetFromHome`). Харнесс: модель GPS (`h.gpsOn`, `h.compass`,
`h.gpsLost`, 10 Гц, шум 0.4 м, дом при ARM).

Проверено: test_copter 115/115 (юнит-тесты `test_gps_rescue_logic`,
`test_gps_rescue_params`); новые сим-тесты по отдельности зелёные: с 150 м при
боковом ветре 4 м/с — сел в 3 м от дома за 56 с; без компаса с ошибкой курса
70° — в 2.4 м; потеря GPS → GPS_LOST, посадка на месте; режим с тумблера.

Осталось:
1. Полный прогон: `pio test -e native -f native/test_copter -f native/test_copter_sim`
   (crosswind-тест теперь на трёх физиках — nominal/sluggish/twitchy — ещё не гонялся),
   `pio test -e native-stm32 -f native_stm32/test_app_stm32_copter`, сборки
   `aocodarc-f405v3-quad matek-h743-quad stm32h743-devebox-quad`.
2. Строки `gps_rescue_*` в таблицу «Настройки» docs/COPTER.md, раздел «GPS
   Rescue» там же; PLAN_COPTER: 129, 133, 134, 135, 136 → [x] с «Итог»
   (130/131/132 — отдельно: оценка положения GPS+акселерометр, компас, POSHOLD
   ещё не сделаны), `python tools/plan_to_json.py`.
3. Коммит кода (include/copter, src/stm32/main_copter.cpp, test/native/helpers,
   test_copter, test_copter_sim, include/telemetry/CopterLink.h), затем коммит плана.
4. Studio: вкладка «GPS» показывает настройки сама (группа gps); добавить на
   «Безопасность» подсказку выбрать failsafe_procedure = GPS-RESCUE для 7" и
   статус возврата из `@live.rescue` на «Подготовке».

### 0.2 Камера OV5640 на DevEBox H743 (просьба пользователя) — прошивка готова, железо пострадало

Сделано: `sensors/camera/Ov5640.h` (SCCB 0x3C на шине датчиков PB10/PB11, ID
0x5640, таблица JPEG 640×480, качество 0x4407), `hal/stm32/Stm32Camera.h`
(XCLK = MCO1 HSI48/2 на PA8, PWDN PA7, RESET PC4, DCMI AF13 D0–D7 PC6 PC7 PE0
PE1 PE4 PD3 PE5 PE6, PCLK PA6, HREF PA4, VSYNC PB7; снимок JPEG через DMA2
Stream7/DMAMUX req 75; буфер в D2 SRAM 0x30000000 — AXI занята очередью ящика),
`telemetry/CameraStream.h` (заявка «cam snap», кадр строками `@cam {...,"d":base64}`
по 384 байта, записи ящика REC_CAM 0x24 «t_us:I frame:H off:I size:I data:y»),
`BlackBoxCore::pushCameraFrame` (только при записи и если в очереди остаётся
CAMERA_BLACKBOX_RESERVE), `AppLink`: `"cam"`/`camw`/`camh`/`camn`/`camkb`/`camrec`
в `@live`, команды ПК `cam snap`, `cam diag`; main.cpp: setupCamera() до задач,
задача `camera` (кадр в ящик раз в CAMERA_BLACKBOX_PERIOD_MS = 1 с при записи).
`SENSOR_CAMERA` (NONE / OV5640) в SensorSelection.h; opconfig знает категорию
`camera` (bus "dcmi"; наборы камеру не содержат — она отдельной строкой в
блоке Studio, явная строка блока главнее умолчания), test_opconfig 19/19;
`tools/blackbox.py` понимает тип `y` и складывает кадры в `<журнал>/camera/*.jpg`.
test_camera 4/4. Распиновка — со схемы DevEBox (github.com/mcauser/MCUDEV_DEVEBOX_H7XX_M,
docs/STM32H7XX_M_schematics.pdf, разъём J6).

На железе: шлейф камеры сначала стоял наоборот (пользователь перепутал
полярность) → SCCB молчал; после правильной установки модуль «ощутимо
греется» и плата пропала с USB даже в DFU; без камеры плата жива. Вывод:
**модуль OV5640, скорее всего, повреждён (КЗ по 3,3 В) — к плате не
подключать**, нужен новый модуль. Плата прошита сборкой с камерой (конфиг
самолёта из последней прошивки Studio + SENSOR_CAMERA_OV5640, копия в
скретчпаде), висела в DFU с перемычкой BT0 — пользователю снять BT0 и нажать
RST. С новым модулем: консоль `cam diag` (такты, PWDN, ответ SCCB), снимок
`cam snap`; если SCCB отвечает, а кадра нет — подобрать полярности DCMI
(`Stm32Camera::polarityExtra`, 0x4740) и PCLK (0x3824/0x460C).

Осталось:
1. ~~Смежные тесты~~ — прошли: test_telemetry, test_blackbox, test_blackbox_scan,
   test_app, test_copter_blackbox — 71/71 с полем `cam` в `@live` и схемой CAM.
2. Матрица сборок: `bash tools/build_matrix.sh` (или хотя бы stm32h743-devebox,
   matek-h743, esp32-s3) — SensorSelection ругается `#error`, если камера не на H743.
3. Studio: категория датчика `camera` (core/manifest.ts SensorCategory и
   SENSOR_CATEGORIES, SensorsStep — заголовок «Камера», core/live.ts — поле
   `live.cam`, ожидаемый адрес 0x3C на шине датчиков в проверке @scan,
   project.ts `pick("camera")`, diff.ts SENSOR_TITLES), чип камеры в панели
   платы, кнопка «Снимок» (команда `cam snap`, Rust link.rs: добавить "cam" в
   COPTER_MSGS → событие board-msg; собрать кадр по `off` и показать `<img>`
   из base64), core/blackbox.ts — тип поля `y` пропускать как `s`; раздел «Ящик»
   — кадры камеры на таймлайне (по t_us). Перевод строк — en.json (i18n_check).
4. Документация: docs/BLACKBOX.md (запись CAM, папка camera/), docs/FC_BOARD.md
   или README (камера DevEBox, распиновка J6), docs/TESTING.md (test_camera).
5. Коммит прошивки и Studio раздельно, push.

---

## 1. Сделано в прошлом чате (для ориентира)

- Studio v0.2.0 выпущен: github.com/damir-lebedev/openplane-studio/releases/latest
  (Windows 416 МБ, macOS ~190 МБ, всё офлайн внутри); кнопки «Скачать» в начале
  README обоих репо (12 языков). Выпуск — тегом vX.Y.Z, CI сам.
- Studio: коптер как конфигуратор BF — вкладки по `@params`, проверка моторов,
  переход с Betaflight одной кнопкой (дамп → заголовок → `bl` → бэкап → заливка),
  «Вернуть Betaflight», перенос diff BF.
- Прошивка: `motor` (связь/CLI/MSP_SET_MOTOR), рама в `@params`, bf_board.py
  probe/dfu/restore; ANGLE/HORIZON как BF 4.5, ACRO TRAINER, auto_profile_cell_count.
- PLAN_COPTER: 158, 159, 163, 031, 079, 033–036 отмечены.
- Трекер-артефакт CjjoBvwmspqw8rnrWMhpVa из этой сессии недоступен — статусы
  только в PLAN_COPTER.md и tools/plan_tracker/items.json.

---

## 2. До MVP дрона — по порядку

### 2.1 Без железа (делать сейчас)

| # | Что | Пункты PLAN_COPTER | Зачем для MVP |
|---|---|---|---|
| 1 | GPS Rescue — довести и закоммитить | 0.1 выше | 7" дальнобой: связь пропала — домой |
| 2 | OSD на AT7456E (MAX7456) по SPI: драйвер, шрифт, элементы полёта, батарея, предупреждения, стрелка домой | 145–150 | на GETFUN аналоговое видео: без OSD летать вслепую по батарее |
| 3 | VTX SmartAudio/Tramp — проверить, что сделано в copter/Vtx.h, довести до пункта | 153 | мощность с тумблера, потолок 25 мВт |
| 4 | Crash recovery | 038 | удар в воздухе — выровняться |
| 5 | RC smoothing как в BF, мёртвые зоны, RSSI/LQ | 111, 112, 113 | ELRS 250 Гц, плавность и OSD LQ |
| 6 | Телеметрия CRSF в пульт (батарея, GPS, режим) — проверить готовое | 118 | TX12 показывает батарею |
| 7 | Пищалка: события и режимы, DShot-маяк | 107–109 | найти упавший 7" |
| 8 | PREARM, команды стиками, airmode с порогом | 100, 105, 098 | безопасность и привычки BF |
| 9 | Turtle mode, DShot-команды и направление моторов | 083, 084, 086 | перевернуться после падения; реверс без проводов |
| 10 | Упрощённые ползунки PID и фильтров | 032, 064 | настройка новичком в Studio |
| 11 | Studio: живые кадры коптера в панели платы (сейчас — самолётные виджеты), ящик коптера и .bfl, вкладка OSD | 119-хвост, 157 | MVP приложения |
| 12 | Переводы D13 (README/COPTER.md на 11 языков) | D13 | перед первым полётом |

Батч — по протоколу PLAN_COPTER (5 пунктов, тесты, «Итог», коммит, push).

### 2.2 С железом (после ~22.11, только с пользователем — PLAN_QUAD7 Q12, PLAN_COPTER раздел 2)

1. **178** — GETFUN на родном BF: в Studio «Перейти на OpenPlane» (дамп, бэкап,
   заголовок; кнопка проверена только на фейках). Сначала — только дамп и бэкап,
   летать на родном BF.
2. **179** — 3–5 батарей на родном BF (пилот учится; журналы BF включены).
3. **180** — Bluejay на регуляторы, `dshot_bidir`, RPM-фильтр.
4. **082 / 089** — DShot и двунаправленный DShot на нашей прошивке (стенд,
   логический анализатор/регулятор); 089 — захват ответа на F405 (bitbang/DMA2).
5. **181 → 182** — эталонные журналы BF → подгонка модели симулятора.
6. **160** — конфигуратор BF подключается к нашей прошивке (проверить вкладки).
7. **172** — первый полёт на нашей прошивке: без пропеллеров (моторы, приёмник,
   OSD, failsafe) → привязь → ANGLE низко → ALT_HOLD → GPS Rescue на малой
   дистанции → ACRO. **183** — «не хуже BF» по журналам.
8. **D14** — переводы после первого полёта.

После этого — MVP. Дальше по PLAN_COPTER: фильтры BF (054–070), DShot-обороты в
режекторы (090), POSHOLD (130–132), ESC passthrough (094) и прочее.
