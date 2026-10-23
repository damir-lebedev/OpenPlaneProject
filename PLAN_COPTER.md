# PLAN_COPTER.md — всё из Betaflight в коптерную прошивку

Рабочий план. Пользователь чистит чат и пишет «продолжай работу по плану» —
тогда выполняется **ровно этот протокол**, без чтения всего файла:

1. `grep -n "^### \[ \]" PLAN_COPTER.md | grep -v "Стенд: да" | head -5` —
   первые 5 невыполненных пунктов без стенда; прочитать только их (Read с
   offset/limit ~10 строк на пункт). Пункты «Стенд: да» — только вместе с
   пользователем, когда он скажет, что железо на столе.
2. Читать только файлы из строки «Файлы»; искать — grep, а не чтение целиком.
   Память: `copter-firmware.md` (как устроено ядро).
3. Сделать, покрыть тестами, проверить:
   `export PATH="$HOME/tools/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH"`
   `pio test -e native -f native/test_copter -f native/test_copter_sim 2>&1 | tail -3`
   `pio test -e native-stm32 -f native_stm32/test_app_stm32_copter 2>&1 | tail -3`
   `pio run -e matek-h743-quad -e stm32h743-devebox-quad 2>&1 | grep -E "error|warning: |SUCCESS|FAILED"`
   Если в батче есть пункт с номером, кратным 20, или пункт «D» — полный прогон
   `pio test -e native -e native-stm32 2>&1 | tail -3`.
4. Отметить пункты: `### [x] NNN · …` и в конце пункта строка
   `- **Итог:** что сделано одной строкой (коммит abcdef0)`.
5. Трекер <https://claude.ai/artifact/CjjoBvwmspqw8rnrWMhpVa> (страница
   `tools/plan_tracker/index.html`): ArtifactData (ToolSearch `select:ArtifactData`),
   коллекция `status`, документ `NNN` = `{status: "done"|"doing"|"blocked",
   commit, result, date}` — один batch на все 5 пунктов; документ уже есть —
   его `version` в `if_version`. Менялся текст плана — `python tools/plan_to_json.py`
   и публикация Artifact с `url` трекера, файлом index.html и `files: {"items.json":
   "tools/plan_tracker/items.json"}`.
6. Новые настройки — строкой в таблицу «Настройки» в `docs/COPTER.md` (только
   русский; переводы — пунктами «D»).
7. Коммит по путям: `git add` нужные файлы, затем
   `git commit -- README.md docs include platformio.ini src test tools PLAN_COPTER.md`
   (airframe не трогать никогда), `git push origin main`.
8. Ответ пользователю — 5–10 строк: что сделано, что проверено, что дальше.

Правила:
- **Код Betaflight не копировать** (GPL-3.0 несовместима с OpenPlane License):
  поведение — по описанию и документации BF, код — свой. Имена настроек и
  значения по умолчанию — как в BF (это не код), в BF-единицах, если они имеют
  смысл у нас.
- Каждая настройка — в реестр (пункт 001): имя BF, тип, мин/макс, дефолт,
  единицы, описание по-русски одной строкой, группа (= вкладка UI).
- Направления — только настройкой/конфигуратором ESC, никогда «переставьте провода».
- «Стенд: да» — нужен человек и железо: код и тесты на фейках делаются, проверку
  на железе — вместе с пользователем (позвать светом, память `call-user-via-lights`).
- Репо приложения — `../openplane-studio` (свой PLAN.md, свои коммиты) — для
  пунктов «Репо: studio».

Статусы: `[ ]` не начат · `[~]` в работе · `[x]` готов · `[!]` заблокирован (причина в «Итог»).

---

## 0. Основа

### [ ] 001 · Реестр настроек коптера
- **BF:** все `set имя = значение` из CLI BF — механизм
- **Что:** таблица параметров (имя BF, тип int/float/enum/bool, мин, макс, дефолт, единицы, группа, описание); указатель на поле CopterSettings/CopterTune; сохранение в KeyValueStore (флеш, как калибровки), версия схемы; «0 = из модели» для параметров, которые выводит Presets.
- **Сейчас:** CopterSettings (поля в коде), CopterConfig.h (constexpr), ничего не сохраняется.
- **Файлы:** новый include/copter/Params.h, copter/CopterController.h, config/CopterConfig.h, storage/KeyValueStore.h
- **Тест:** test_copter: set/get/границы/enum по имени/сохранение-загрузка через фейк флеша/неизвестное имя
- **UI:** основа всех вкладок — список параметров по группам

### [ ] 002 · CLI как в Betaflight
- **BF:** `#` вход в CLI, `set`, `get`, `dump`, `diff`, `save`, `defaults`, `exit`, `status`, `version`, `help`
- **Что:** строковый режим консоли коптера по `#` (как BF): разбор `set x = y`, `get` по подстроке, `diff` (только отличные от дефолта), `dump`, `save` (во флеш + перезапуск контроллера), `defaults`; ответы в формате BF (чтобы diff из BF можно было вставить).
- **Сейчас:** консоль по одной клавише (t, m, s, o, i, r, l, D).
- **Файлы:** telemetry/CopterConsole.h, copter/Params.h
- **Тест:** test_app_stm32_copter: `#` → `set p_roll = 50` → `save` → перезагрузка фейка → `get p_roll`
- **UI:** вкладка «CLI» в приложении (терминал + вставка diff)

### [ ] 003 · PID в единицах Betaflight
- **BF:** `p_roll i_roll d_roll f_roll` и то же для pitch/yaw
- **Что:** перевод BF ↔ наши (P·0.032029/1000 на °/с, I·0.244381/1000, D·0.000529/1000, F·0.013754/1000 — масштабы BF, проверить по документации); 0 = из модели; `t` показывает и BF-числа.
- **Сейчас:** AxisGains p/i/d/ff в «команда смесителя на °/с».
- **Файлы:** copter/Params.h, copter/Presets.h, telemetry/CopterConsole.h
- **Тест:** test_copter: модельный 5" в BF-единицах — порядок BF-дефолтов (P 30–60); set p_roll меняет AxisGains
- **UI:** вкладка «PID»: таблица P/I/D/F по осям, как в конфигураторе BF

### [ ] 004 · Дефолты Betaflight 4.5 как источник настройки
- **BF:** дефолтный профиль 4.5 (P/I/D/F, d_max, фильтры, TPA, AG, FF)
- **Что:** `tune_source = MODEL | BETAFLIGHT` (наша настройка): BF-дефолты целиком; симуляция всех 12 сборок на BF-дефолтах при нашем цикле 1 кГц и OneShot — где не летает, честно записать в COPTER.md.
- **Файлы:** copter/Presets.h, copter/Params.h, test_copter_sim
- **Тест:** test_copter_sim: freestyle5 висит и держит расход на BF-дефолтах
- **UI:** вкладка «PID»: переключатель «из модели / как в BF»

### [ ] 005 · Профили PID и расходов
- **BF:** `profile 0..3`, `rateprofile 0..3`, `pid_profile`, `rateprofile_name`, `profile_name`
- **Что:** 4 PID-профиля и 4 профиля расходов в реестре, переключение из CLI (и с тумблера/крутилки — пункт 122).
- **Файлы:** copter/Params.h, telemetry/CopterConsole.h
- **Тест:** test_copter: профили независимы, переключение без перезапуска
- **UI:** выпадающий список профиля на вкладках PID и Rates

### [ ] 006 · Режимы на тумблерах как в BF
- **BF:** `aux <i> <режим> <канал> <от> <до>`, режимы ARM, ANGLE, HORIZON, AIRMODE, ALTHOLD, FAILSAFE, BEEPER…
- **Что:** таблица диапазонов вместо зашитых SwA/SwC; режимы с приоритетом как в BF; `aux` в CLI.
- **Сейчас:** ARM = CH5 > 1700, режим — SwC (CopterConfig::MODE_*).
- **Файлы:** copter/Params.h, src/stm32/main_copter.cpp, config/CopterConfig.h
- **Тест:** test_app_stm32_copter: `aux` ARM на CH6, ANGLE на CH7 1300–1700
- **UI:** вкладка «Режимы»: полосы каналов с ползунками диапазонов и живой меткой положения тумблера

### [ ] 007 · Карта каналов и диапазон приёмника
- **BF:** `map`, `rx_min_usec`, `rx_max_usec`, `rxrange`, `serialrx_provider`
- **Что:** AETR/TAER… порядок, калибровка концов стиков по каналу, выбор протокола без пересборки.
- **Сейчас:** порядок — config/Channels.h, протокол — Config::RX_PROTOCOL (сборка).
- **Файлы:** rc/RcReceiver.h, copter/Params.h, src/stm32/main_copter.cpp
- **Тест:** test_app_stm32_copter: `map TAER1234` меняет газ/крен
- **UI:** вкладка «Приёмник»: живые полосы каналов, выбор порядка, протокол

### [ ] 008 · `status` и `tasks` как в BF
- **BF:** `status` (CPU, цикл, флаги запрета ARM), `tasks`
- **Что:** загрузка цикла, пик, причины запрета ARM кодами BF (RXLOSS, ANGLE, THROTTLE, CLI, MSP…) и словами.
- **Файлы:** telemetry/CopterConsole.h, copter/CopterController.h
- **Тест:** test_app_stm32_copter: вывод status содержит флаги
- **UI:** строка состояния внизу приложения (как в конфигураторе)

### [ ] 009 · Описание коптера — тоже параметры
- **BF:** — (наше; в BF это пресеты)
- **Что:** COPTER_BUILD, масса, винт, мотор, KV, батарея, рама — в реестр (`set copter_build = race5`, `set copter_mass_g = 742`), пересчёт настройки при `save`.
- **Сейчас:** только constexpr в CopterConfig.h + пересборка.
- **Файлы:** config/CopterConfig.h, copter/Params.h, src/stm32/main_copter.cpp
- **Тест:** test_app_stm32_copter: смена сборки через CLI → `t` показывает новую
- **UI:** вкладка «Коптер»: мастер «рама → винт → мотор → батарея → масса» с картинками и итогом (тяга/вес, висение, предупреждения)

### [ ] 010 · Машинный протокол параметров для приложения
- **BF:** аналог MSP-запросов настроек
- **Что:** строки `@params` (JSON-список с группами, единицами, границами), `@set`, `@save`, `@live` коптера (углы, моторы, газ, батарея, режим, причины запрета ARM) — как AppLink самолёта.
- **Файлы:** telemetry/CopterConsole.h (или новый CopterLink.h), telemetry/AppLink.h (образец)
- **Тест:** test_app_stm32_copter: `@params` разбирается как JSON, `@set` меняет параметр
- **UI:** транспорт для всех вкладок приложения

---

## 1. Регулятор (PID-профиль)

### [ ] 011 · d_max (d_min) — D растёт при резких движениях
- **BF:** `d_max_roll d_max_pitch d_max_yaw d_max_gain d_max_advance` (до 4.5 — `d_min_*`, `d_min_gain`, `d_min_advance`)
- **Что:** базовый D меньше, до d_max — по скорости изменения гироскопа/уставки.
- **Файлы:** copter/RateController.h, copter/Params.h
- **Тест:** test_copter: D растёт на ступеньке; test_copter_sim: шум моторов на висении не выше прежнего
- **UI:** вкладка «PID»: колонка D max и ползунок «D max»

### [ ] 012 · iterm_relax — тип и частота
- **BF:** `iterm_relax` (OFF/RP/RPY/RP_INC/RPY_INC), `iterm_relax_type` (GYRO/SETPOINT), `iterm_relax_cutoff`
- **Что:** BF-вариант relax по ФВЧ уставки; наш relax по ошибке — отдельной настройкой `iterm_relax_error` (наша, дефолт вкл).
- **Сейчас:** RateAxis: relax по уставке (PT1 10 Гц, 40 °/с) и по ошибке (20–80 °/с).
- **Файлы:** copter/RateController.h, copter/Params.h
- **Тест:** test_copter: типы; test_copter_sim: «вялая» гекса — перелёт ≤ 12 %
- **UI:** вкладка «PID → дополнительно»

### [ ] 013 · iterm_rotation
- **BF:** `iterm_rotation`
- **Что:** вектор интеграла поворачивается вместе с рамой при вращении (полезно в рысканье на крене).
- **Файлы:** copter/RateController.h, copter/CopterController.h
- **Тест:** test_copter: пируэт — интеграл крена переходит в тангаж
- **UI:** «PID → дополнительно», переключатель

### [ ] 014 · iterm_windup
- **BF:** `iterm_windup` (%)
- **Что:** интеграл не копится, когда моторы упираются больше заданного процента.
- **Сейчас:** anti-windup по флагу saturated смесителя.
- **Файлы:** copter/RateController.h, copter/Mixer.h
- **Тест:** test_copter: на упоре интеграл стоит
- **UI:** «PID → дополнительно»

### [ ] 015 · iterm_limit
- **BF:** `iterm_limit`
- **Что:** предел интеграла в BF-единицах (сейчас iLimit 0.3 доли смесителя).
- **Файлы:** copter/RateController.h, copter/Params.h
- **Тест:** test_copter: предел соблюдается
- **UI:** «PID → дополнительно»

### [ ] 016 · pidsum_limit, pidsum_limit_yaw
- **BF:** `pidsum_limit`, `pidsum_limit_yaw`
- **Что:** ограничение суммы PID по осям (сейчас ±1 одно на все).
- **Файлы:** copter/RateController.h
- **Тест:** test_copter: рысканье ограничено своим пределом
- **UI:** «PID → дополнительно»

### [ ] 017 · anti_gravity — как в BF 4.3+
- **BF:** `anti_gravity_gain`, `anti_gravity_cutoff_hz`, `anti_gravity_p_gain`
- **Что:** усиление I (и P) по ФВЧ газа с настраиваемой частотой.
- **Сейчас:** iBoost = 1 + min(3, |Δгаз|·30), ФНЧ 15 Гц.
- **Файлы:** copter/CopterController.h, copter/RateController.h
- **Тест:** test_copter_sim: рывок газом — нос не клюёт (тангаж < 3°)
- **UI:** вкладка «PID»: ползунок Anti Gravity

### [ ] 018 · throttle_boost
- **BF:** `throttle_boost`, `throttle_boost_cutoff`
- **Что:** добавка к газу по его ФВЧ — быстрее отклик на рывок газа.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: вертикальное ускорение после рывка быстрее
- **UI:** «PID → дополнительно»

### [ ] 019 · TPA как в BF
- **BF:** `tpa_mode` (PD/D), `tpa_rate`, `tpa_breakpoint`
- **Что:** классическое ослабление PD выше точки излома; наш TPA из модели — режим `tpa_mode = MODEL` (наш, дефолт).
- **Сейчас:** таблица tpa[11] из модели, множитель P и D.
- **Файлы:** copter/Presets.h, copter/CopterController.h, copter/Params.h
- **Тест:** test_copter: кривая TPA; test_copter_sim: полный газ без раскачки
- **UI:** вкладка «PID»: график коэффициента от газа

### [ ] 020 · tpa_low — TPA на малом газе
- **BF:** `tpa_low_rate`, `tpa_low_breakpoint`, `tpa_low_always`
- **Что:** изменение PD на малом газе (спуски, флипы на нуле).
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter: кривая; test_copter_sim: airmode на нуле газа не раскачивается
- **UI:** тот же график TPA

### [ ] 021 · feedforward_transition
- **BF:** `feedforward_transition`
- **Что:** FF ослабляется у центра стика.
- **Сейчас:** FF = 0.7/K·d(уставки)/dt через PT1 25 Гц.
- **Файлы:** copter/RateController.h, copter/CopterController.h
- **Тест:** test_copter: FF у центра меньше
- **UI:** «PID → Feedforward»

### [ ] 022 · feedforward_averaging
- **BF:** `feedforward_averaging` (OFF/2/3/4_POINT)
- **Что:** усреднение производной уставки по кадрам приёмника.
- **Файлы:** copter/RateController.h
- **Тест:** test_copter: ступени кадров 50 Гц сглажены
- **UI:** «PID → Feedforward»

### [ ] 023 · feedforward_smooth_factor
- **BF:** `feedforward_smooth_factor`
- **Что:** сглаживание FF между кадрами.
- **Файлы:** copter/RateController.h
- **Тест:** test_copter
- **UI:** «PID → Feedforward»

### [ ] 024 · feedforward_jitter_factor
- **BF:** `feedforward_jitter_factor`
- **Что:** ослабление FF при медленном дрожании стика.
- **Файлы:** copter/RateController.h
- **Тест:** test_copter: дрожание ±2 % — FF почти ноль
- **UI:** «PID → Feedforward»

### [ ] 025 · feedforward_boost
- **BF:** `feedforward_boost`
- **Что:** добавка по второй производной уставки (ускорение стика).
- **Файлы:** copter/RateController.h
- **Тест:** test_copter
- **UI:** «PID → Feedforward»

### [ ] 026 · feedforward_max_rate_limit
- **BF:** `feedforward_max_rate_limit`
- **Что:** FF гаснет у предела расхода — нет перелёта на краю.
- **Файлы:** copter/RateController.h
- **Тест:** test_copter_sim: флип полным стиком — перелёт расхода < 5 %
- **UI:** «PID → Feedforward»

### [ ] 027 · yaw_lowpass_hz
- **BF:** `yaw_lowpass_hz`
- **Что:** ФНЧ выхода рысканья настройкой (сейчас yawLpfHz из модели).
- **Файлы:** copter/Params.h, copter/CopterController.h
- **Тест:** test_copter
- **UI:** вкладка «Фильтры»

### [ ] 028 · integrated yaw
- **BF:** `use_integrated_yaw`, `integrated_yaw_relax`
- **Что:** рысканье через интеграл суммы PID — для коптеров со слабым рысканьем (большие винты).
- **Файлы:** copter/RateController.h
- **Тест:** test_copter_sim: hex15 — рысканье держит расход
- **UI:** «PID → дополнительно»

### [ ] 029 · absolute control
- **BF:** `abs_control_gain`, `abs_control_limit`, `abs_control_error_limit`, `abs_control_cutoff`
- **Что:** коррекция накопленной ошибки угла (против «уплывания» в акро на ветру).
- **Файлы:** copter/RateController.h
- **Тест:** test_copter_sim: ветер — в ACRO курс уплывает меньше
- **UI:** «PID → дополнительно»

### [ ] 030 · thrust_linear, motor_output_limit
- **BF:** `thrust_linear` (%), `motor_output_limit` (%)
- **Что:** настройкой (сейчас thrustLinear из размера винта); предел выхода моторов (батарея 6S на моторах под 4S).
- **Файлы:** copter/Mixer.h, copter/Params.h
- **Тест:** test_copter: предел 80 % — команда не выше
- **UI:** вкладка «Моторы»

### [ ] 031 · vbat_sag_compensation (%)
- **BF:** `vbat_sag_compensation`
- **Что:** процент компенсации просадки (сейчас вкл/выкл).
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: на 100 % газ висения не растёт с разрядом
- **UI:** вкладка «PID»

### [ ] 032 · Упрощённая настройка — ползунки
- **BF:** `simplified_pids_mode`, `simplified_master_multiplier`, `simplified_i_gain`, `simplified_d_gain`, `simplified_pi_gain`, `simplified_dmax_gain`, `simplified_feedforward_gain`, `simplified_pitch_d_gain`, `simplified_pitch_pi_gain`
- **Что:** множители поверх модели/BF-дефолтов — ровно как ползунки BF; запас по усилению пересчитывать и предупреждать, если < 6 дБ.
- **Файлы:** copter/Presets.h, copter/Params.h
- **Тест:** test_copter: master ×1.5 → P,I,D,F ×1.5; предупреждение запаса
- **UI:** вкладка «PID»: ползунки + полоса «запас устойчивости» из модели (зелёная/жёлтая/красная)

### [ ] 033 · auto_profile_cell_count
- **BF:** `auto_profile_cell_count`
- **Что:** выбор PID-профиля по числу банок батареи.
- **Файлы:** copter/Params.h, src/stm32/main_copter.cpp
- **Тест:** test_app_stm32_copter: 4S → профиль 2
- **UI:** вкладка «PID», поле у профиля

### [ ] 034 · Режим ANGLE как в BF 4.5
- **BF:** `angle_limit`, `angle_p_gain`, `angle_feedforward`, `angle_feedforward_smoothing_ms`, `angle_earth_ref`
- **Что:** сила выравнивания, FF по скорости стика, рысканье относительно Земли (у нас уже есть) — настройками.
- **Сейчас:** levelGain из модели, maxAngleDeg по стилю, рысканье вокруг вертикали.
- **Файлы:** copter/CopterController.h (rateSetpoints), copter/Params.h
- **Тест:** test_copter_sim: ANGLE — выход на наклон быстрее с FF
- **UI:** вкладка «PID → Angle/Horizon»

### [ ] 035 · Режим HORIZON как в BF 4.5
- **BF:** `horizon_level_strength`, `horizon_limit_degrees`, `horizon_limit_sticks`, `horizon_delay_ms`
- **Что:** смешение ANGLE/ACRO по стику и по углу, задержка выравнивания.
- **Сейчас:** horizonTransition 0.75, квадратичное смешение.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: флип в HORIZON проходит, у центра — выравнивается
- **UI:** «PID → Angle/Horizon»

### [ ] 036 · Acro trainer
- **BF:** `acro_trainer_angle_limit`, `acro_trainer_lookahead_ms`, `acro_trainer_gain`, режим ACRO TRAINER
- **Что:** ACRO с ограничением наклона — для обучения.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: полный стик — наклон не больше предела
- **UI:** «PID → Angle/Horizon» + режим на вкладке «Режимы»

### [ ] 037 · Launch control
- **BF:** `launch_control_mode` (NORMAL/PITCHONLY/FULL), `launch_trigger_allow_reset`, `launch_trigger_throttle_percent`, `launch_angle_limit`, `launch_control_gain`, режим LAUNCH CONTROL
- **Что:** гоночный старт: коптер на земле держит наклон до рывка газом.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: старт под 30°, взлёт без кувырка
- **UI:** «PID → дополнительно» + режим

### [ ] 038 · Crash recovery
- **BF:** `crash_recovery` (OFF/ON/BEEP/DISARM), `crash_dthreshold`, `crash_gthreshold`, `crash_setpoint_threshold`, `crash_time`, `crash_delay`, `crash_recovery_angle`, `crash_recovery_rate`, `crash_limit_yaw`
- **Что:** удар в воздухе → выравнивание; DISARM — как наша защита (оставить и нашу).
- **Сейчас:** переворот > 90° 0.7 с, на спине 1 с, потеря управления 30° 2 с → DISARM.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: удар (импульс момента) — выровнялся
- **UI:** вкладка «Безопасность»

### [ ] 039 · EZ landing
- **BF:** `ez_landing_threshold`, `ez_landing_limit`, `ez_landing_speed`, `mixer_type = EZLANDING`
- **Что:** мягкая посадка: ограничение разницы моторов у самой земли на малом газе.
- **Файлы:** copter/Mixer.h, copter/CopterController.h
- **Тест:** test_copter_sim: посадка без подпрыгивания
- **UI:** вкладка «Моторы»

### [ ] 040 · mixer_type LEGACY / LINEAR / DYNAMIC
- **BF:** `mixer_type`
- **Что:** способы уложить команды в 0..1 (сейчас — свой, airmode).
- **Файлы:** copter/Mixer.h
- **Тест:** test_copter: каждый тип
- **UI:** вкладка «Моторы»

### [ ] 041 · Dynamic idle
- **BF:** `dyn_idle_min_rpm`, `dyn_idle_p_gain`, `dyn_idle_i_gain`, `dyn_idle_d_gain`, `dyn_idle_max_increase`, `dyn_idle_start_increase`
- **Что:** холостые по оборотам (нужны обороты: модель сейчас, двунаправленный DShot — пункт 087).
- **Файлы:** copter/CopterController.h, copter/Mixer.h
- **Тест:** test_copter_sim: на нуле газа обороты не ниже минимума
- **UI:** вкладка «Моторы»

### [ ] 042 · Проверка BF-функций регулятора в симуляции
- **BF:** —
- **Что:** прогон всех 12 сборок × 3 физики с включёнными 011–041; таблица в COPTER.md «что дали функции BF» (перелёт, время, дрожь моторов).
- **Файлы:** test/native/test_copter_sim
- **Тест:** сам пункт
- **UI:** —

### [ ] 043 · Studio: вкладка «PID» — Репо: studio
- **BF:** вкладка PID Tuning конфигуратора
- **Что:** таблица P/I/D/Dmax/F по осям, ползунки упрощённой настройки, профили, график TPA, переключатель источника (модель/BF), полоса запаса устойчивости, «применить/сохранить».
- **Файлы:** ../openplane-studio/src (новая вкладка Copter/PID)
- **Тест:** тест компонентов приложения + живая плата на фейке (протокол 010)
- **UI:** сам пункт

### [ ] 044 · Studio: график отклика — Репо: studio
- **BF:** аналога нет (BF — только по blackbox)
- **Что:** переходный процесс угловой скорости из модели при текущих PID (и запасы) — меняешь ползунок, видишь ступеньку.
- **Файлы:** ../openplane-studio/src, расчёт — в прошивке (`@stepresponse`) или копия модели
- **Тест:** ступенька совпадает с test_copter_sim
- **UI:** сам пункт

### [ ] D01 · Документация: PID-профиль (ru + 11 языков)
- **Что:** раздел «Настройки регулятора» в COPTER.md и переводы; README — строка в разделе коптера.
- **Файлы:** docs/COPTER.md, docs/i18n/*/COPTER.md

---

## 2. Расходы (rate profile)

### [ ] 045 · rates_type BETAFLIGHT
- **BF:** `rates_type = BETAFLIGHT`, `roll_rc_rate`, `roll_expo`, `roll_srate` (и pitch/yaw)
- **Что:** формула расходов BF рядом с нашей RateCurve.
- **Сейчас:** RateCurve(center, max, expo) по стилю полёта.
- **Файлы:** copter/Presets.h (RateCurve), copter/Params.h
- **Тест:** test_copter: точки кривой как в калькуляторе BF
- **UI:** вкладка «Rates»: график кривой и 3D-превью расхода

### [ ] 046 · rates_type ACTUAL
- **BF:** `rates_type = ACTUAL` (center sensitivity, max rate, expo)
- **Что:** формула Actual (наша RateCurve ближе всего к ней).
- **Файлы:** copter/Presets.h
- **Тест:** test_copter
- **UI:** «Rates»

### [ ] 047 · rates_type KISS
- **BF:** `rates_type = KISS`
- **Файлы:** copter/Presets.h · **Тест:** test_copter · **UI:** «Rates»

### [ ] 048 · rates_type RACEFLIGHT
- **BF:** `rates_type = RACEFLIGHT`
- **Файлы:** copter/Presets.h · **Тест:** test_copter · **UI:** «Rates»

### [ ] 049 · rates_type QUICK
- **BF:** `rates_type = QUICK`, `quickrates_rc_expo`
- **Файлы:** copter/Presets.h · **Тест:** test_copter · **UI:** «Rates»

### [ ] 050 · Пределы расхода
- **BF:** `roll_rate_limit`, `pitch_rate_limit`, `yaw_rate_limit`
- **Файлы:** copter/Presets.h · **Тест:** test_copter · **UI:** «Rates»

### [ ] 051 · Кривая газа
- **BF:** `thr_mid`, `thr_expo`
- **Что:** кривая стика газа (центр — висение и т. п.).
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter: thr_mid 0.3, expo 0.5 — точки
- **UI:** «Rates»: график газа

### [ ] 052 · Ограничение газа
- **BF:** `throttle_limit_type` (OFF/SCALE/CLIP), `throttle_limit_percent`
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** «Rates»

### [ ] 053 · Studio: вкладка «Rates» — Репо: studio
- **BF:** вкладка Rates конфигуратора
- **Что:** график кривой по осям, выбор типа, профили, живая точка стика, 3D-модель коптера крутится с расходом.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

---

## 3. Фильтры

### [ ] 054 · Типы ФНЧ гироскопа
- **BF:** `gyro_lpf1_type` (PT1/BIQUAD/PT2/PT3), `gyro_lpf1_static_hz`, `gyro_lpf2_type`, `gyro_lpf2_static_hz`
- **Что:** две ступени ФНЧ с выбором типа; нужен PT3 в Filters.h; модель контура учитывает выбранные.
- **Сейчас:** один PT2 на gyroLpfHz.
- **Файлы:** copter/Filters.h, copter/GyroFilter.h, copter/Presets.h (LoopShape)
- **Тест:** test_copter: АЧХ каждого типа; запас контура пересчитан
- **UI:** вкладка «Фильтры»: график АЧХ/ФЧХ всей цепочки

### [ ] 055 · Динамический ФНЧ гироскопа
- **BF:** `gyro_lpf1_dyn_min_hz`, `gyro_lpf1_dyn_max_hz`, `gyro_lpf1_dyn_expo`
- **Что:** частота среза растёт с газом.
- **Файлы:** copter/GyroFilter.h
- **Тест:** test_copter: частота от газа; test_copter_sim: шум на полном газе ниже
- **UI:** «Фильтры»: ползунки + график частоты от газа

### [ ] 056 · Статические режекторы гироскопа
- **BF:** `gyro_notch1_hz`, `gyro_notch1_cutoff`, `gyro_notch2_hz`, `gyro_notch2_cutoff`
- **Файлы:** copter/GyroFilter.h · **Тест:** test_copter · **UI:** «Фильтры»

### [ ] 057 · Типы ФНЧ D
- **BF:** `dterm_lpf1_type`, `dterm_lpf1_static_hz`, `dterm_lpf2_type`, `dterm_lpf2_static_hz`
- **Сейчас:** один PT1 dtermLpfHz.
- **Файлы:** copter/RateController.h, copter/Presets.h · **Тест:** test_copter · **UI:** «Фильтры»

### [ ] 058 · Динамический ФНЧ D
- **BF:** `dterm_lpf1_dyn_min_hz`, `dterm_lpf1_dyn_max_hz`, `dterm_lpf1_dyn_expo`
- **Файлы:** copter/RateController.h · **Тест:** test_copter · **UI:** «Фильтры»

### [ ] 059 · Режектор D
- **BF:** `dterm_notch_hz`, `dterm_notch_cutoff`
- **Файлы:** copter/RateController.h · **Тест:** test_copter · **UI:** «Фильтры»

### [ ] 060 · Динамический режектор: анализ спектра (SDFT)
- **BF:** dynamic notch (основа) — `dyn_notch_min_hz`, `dyn_notch_max_hz`
- **Что:** скользящее ДПФ гироскопа по каждой оси в полосе min–max, поиск пиков (своя реализация).
- **Файлы:** новый copter/Spectrum.h
- **Тест:** test_copter: синус 180 Гц + шум → пик 180 ± 5 Гц; время на цикл
- **UI:** «Фильтры»: живой спектр гироскопа

### [ ] 061 · Динамический режектор: фильтры по пикам
- **BF:** `dyn_notch_count`, `dyn_notch_q`
- **Что:** N режекторов едут за найденными пиками; учитываются в модели контура как худший случай.
- **Файлы:** copter/GyroFilter.h, copter/Spectrum.h
- **Тест:** test_copter_sim: вибрация с неверной моделью оборотов (±30 %) срезана
- **UI:** «Фильтры»: метки режекторов на спектре

### [ ] 062 · Настройки RPM-фильтра
- **BF:** `rpm_filter_harmonics`, `rpm_filter_weights`, `rpm_filter_q`, `rpm_filter_min_hz`, `rpm_filter_fade_range_hz`, `rpm_filter_lpf_hz`
- **Что:** наши режекторы по оборотам — по этим настройкам (гармоники, веса, плавное выключение у min); источник оборотов — модель или DShot (090).
- **Сейчас:** 1×, 2×, лопасти×; Q 4; min = 5 полос контура.
- **Файлы:** copter/GyroFilter.h, copter/Presets.h
- **Тест:** test_copter: веса и затухание
- **UI:** «Фильтры»

### [ ] 063 · Режекторы по каждому мотору
- **BF:** RPM filter (по мотору)
- **Что:** вместо одного среднего — режекторы на каждый мотор (обороты разные при манёвре).
- **Файлы:** copter/GyroFilter.h, copter/CopterController.h
- **Тест:** test_copter_sim: манёвр — шум моторов ниже, время цикла в норме
- **UI:** «Фильтры»

### [ ] 064 · Упрощённые ползунки фильтров
- **BF:** `simplified_gyro_filter`, `simplified_gyro_filter_multiplier`, `simplified_dterm_filter`, `simplified_dterm_filter_multiplier`
- **Файлы:** copter/Presets.h · **Тест:** test_copter · **UI:** «Фильтры»: два ползунка + запас контура

### [ ] 065 · Фильтр в чипе гироскопа
- **BF:** `gyro_hardware_lpf` (NORMAL/OPTION_1/OPTION_2/EXPERIMENTAL)
- **Что:** выбор полосы UI-фильтра чипа (сейчас IMU_WIDE_BANDWIDTH в сборке); задержка — в модель.
- **Файлы:** sensors/imu/*_Sensor.h, config/Config.h
- **Тест:** test_app_stm32_copter: регистр фильтра ICM42688 по настройке
- **UI:** «Фильтры»

### [ ] 066 · ФНЧ акселерометра, калибровка, триммеры
- **BF:** `acc_lpf_hz`, `acc_calibration`, `acc_trim_roll`, `acc_trim_pitch`
- **Сейчас:** PT2 20 Гц зашит; калибровка — 3 позы ('o').
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** вкладка «Настройка платы»: кнопка калибровки, 3D-модель

### [ ] 067 · Фильтр ориентации
- **BF:** `imu_dcm_kp`, `imu_dcm_ki`
- **Сейчас:** Mahony kFlight 0.5 (τ 2 с), kGround 5.
- **Файлы:** copter/Attitude.h · **Тест:** test_copter · **UI:** «Настройка платы → дополнительно»

### [ ] 068 · Калибровка гироскопа
- **BF:** `gyro_calib_duration`, `gyro_calib_noise_limit`, `gyro_offset_yaw`
- **Файлы:** sensors/imu/ImuSensorBase.h · **Тест:** test_app_stm32_copter · **UI:** «Настройка платы»

### [ ] 069 · Yaw spin recovery
- **BF:** `yaw_spin_recovery`, `yaw_spin_threshold`, `gyro_overflow_detect`
- **Что:** гироскоп зашкалил по рысканью (> 2000 °/с) — особый режим остановки вращения.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: раскрутка 2500 °/с — остановилась
- **UI:** «Безопасность»

### [ ] 070 · Ручной поворот платы
- **BF:** `align_board_roll`, `align_board_pitch`, `align_board_yaw`, `gyro_1_align`
- **Что:** вместе с калибровкой тремя позами — числами, как в BF.
- **Файлы:** sensors/imu/ImuOrientation.h · **Тест:** test_imu · **UI:** «Настройка платы»: 3D-модель поворачивается

### [ ] 071 · Studio: вкладка «Фильтры» — Репо: studio
- **Что:** АЧХ/ФЧХ цепочки (ФНЧ, режекторы, задержки) из модели, живой спектр гироскопа (060), ползунки, запас контура.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 072 · Studio: вкладка «Настройка платы» — Репо: studio
- **Что:** 3D-коптер по живой ориентации, калибровки (гироскоп, акселерометр, 3 позы), поворот платы.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] D02 · Документация: расходы и фильтры (ru + 11 языков)

---

## 4. Моторы, регуляторы, DShot

### [ ] 073 · Протокол регуляторов без пересборки
- **BF:** `motor_pwm_protocol`, `motor_pwm_rate`, `use_unsynced_pwm`
- **Сейчас:** CopterConfig::ESC_PROTOCOL при сборке; частоты зашиты в Stm32MotorOutput.
- **Файлы:** hal/stm32/Stm32MotorOutput.h, copter/Params.h
- **Тест:** test_app_stm32_copter: `set motor_pwm_protocol = PWM` → импульс 1000 мкс
- **UI:** вкладка «Моторы»

### [ ] 074 · Коллекторные моторы (BRUSHED)
- **BF:** `motor_pwm_protocol = BRUSHED`, `motor_pwm_rate` 32 кГц
- **Что:** вупы на коллекторных моторах: ШИМ на MOSFET, модель коллекторного мотора в Propulsion.
- **Файлы:** hal/stm32/Stm32MotorOutput.h, copter/Propulsion.h
- **Тест:** test_app_stm32_copter, test_copter
- **UI:** «Моторы»

### [ ] 075 · Холостые и минимальная команда
- **BF:** `dshot_idle_value` / `motor_idle`, `min_command`
- **Сейчас:** idle из размера винта.
- **Файлы:** copter/Params.h, copter/Mixer.h · **Тест:** test_copter · **UI:** «Моторы»

### [ ] 076 · Пропеллеры наружу и порядок выходов
- **BF:** `yaw_motors_reversed`, `resource MOTOR n <пин>`
- **Сейчас:** PROPS_OUT и MOTOR_PINS — constexpr.
- **Файлы:** config/CopterConfig.h, hal/stm32/Stm32MotorOutput.h · **Тест:** test_app_stm32_copter · **UI:** «Моторы»: схема рамы с номерами и стрелками вращения, перетаскивание номеров

### [ ] 077 · Своя рама: mmix
- **BF:** `mixer CUSTOM`, `mmix <i> <газ> <крен> <тангаж> <рысканье>`
- **Что:** любые положения моторов; модель контура считает плечи из mmix.
- **Файлы:** copter/Frames.h, copter/Presets.h · **Тест:** test_copter, test_copter_sim: несимметричная рама висит · **UI:** «Моторы»: редактор рамы мышкой

### [ ] 078 · Трикоптер (мотор + серво рысканья)
- **BF:** `mixer TRI`, `tri_unarmed_servo`, сервовыходы
- **Что:** рама TRI с сервоприводом хвоста (в BF есть), модель серво в симуляторе.
- **Файлы:** copter/Frames.h, copter/Mixer.h, hal/stm32/Stm32ServoOutput.h
- **Тест:** test_copter_sim: трикоптер висит и рыскает
- **UI:** «Моторы»

### [ ] 079 · Проверка моторов из CLI
- **BF:** `motor <n> <значение>`, вкладка Motors
- **Что:** ручное управление каждым мотором с защитой (пропеллеры!), остановка при потере связи с ПК.
- **Сейчас:** 'm' — по очереди на 6 %.
- **Файлы:** telemetry/CopterConsole.h · **Тест:** test_app_stm32_copter · **UI:** вкладка «Моторы»: ползунки на каждый мотор, «я снял пропеллеры»

### [ ] 080 · DShot: кадры на таймере с DMA (STM32H7)
- **BF:** `motor_pwm_protocol = DSHOT150/300/600`
- **Что:** драйвер DShot: таймер в режиме burst DMA, буфер длительностей (EscProtocol::frameToPulses уже есть), отправка раз в цикл.
- **Файлы:** новый hal/stm32/Stm32Dshot.h
- **Тест:** test_app_stm32_copter на фейке DMA: длительности бит, CRC, частота
- **UI:** «Моторы»

### [ ] 081 · DShot: несколько таймеров и пины Matek/DevEBox
- **BF:** —
- **Что:** S1–S8 на TIM1/TIM2/TIM4 — одновременный запуск, без конфликтов DMA-потоков.
- **Файлы:** hal/stm32/Stm32Dshot.h · **Тест:** test_app_stm32_copter · **UI:** —

### [ ] 082 · DShot: проверка на стенде — Стенд: да
- **Что:** логический анализатор или ESC: кадры, взвод ESC, газ.
- **UI:** —

### [ ] 083 · DShot: команды
- **BF:** `dshotprog`, команды BEEP, ESC_INFO, SPIN_DIRECTION, 3D, SAVE, LED
- **Что:** очередь команд (повтор 6–10 раз, только при остановленных моторах).
- **Файлы:** hal/stm32/Stm32Dshot.h, copter/EscProtocol.h · **Тест:** test_app_stm32_copter · **UI:** «Моторы»: кнопки «пискнуть», «направление»

### [ ] 084 · Направление моторов командой DShot
- **BF:** мастер Motor Direction конфигуратора
- **Что:** сменить направление мотора из консоли/приложения без конфигуратора ESC.
- **Файлы:** telemetry/CopterConsole.h · **Тест:** test_app_stm32_copter · **UI:** «Моторы»: клик по мотору на схеме — сменить направление

### [ ] 085 · 3D-режим (реверс)
- **BF:** `3d_deadband_low`, `3d_deadband_high`, `3d_neutral`, `3d_deadband_throttle`, `3d_limit_low`, `3d_limit_high`, `3d_switched_mode`, режим 3D
- **Файлы:** copter/Mixer.h, copter/CopterController.h · **Тест:** test_copter · **UI:** «Моторы»

### [ ] 086 · Turtle mode (переворот после падения)
- **BF:** `crashflip_motor_percent`, `crashflip_expo`, режим FLIP OVER AFTER CRASH
- **Что:** нужен DShot-реверс; лёг на спину → переворот моторами.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim: на спине — перевернулся · **UI:** режим на вкладке «Режимы»

### [ ] 087 · Двунаправленный DShot: приём ответа (GCR)
- **BF:** `dshot_bidir`
- **Что:** переключение пина на вход, захват таймером/DMA, декодирование GCR → eRPM.
- **Файлы:** hal/stm32/Stm32Dshot.h, copter/EscProtocol.h
- **Тест:** test_copter: декодер на эталонных кадрах; test_app_stm32_copter на фейке захвата
- **UI:** «Моторы»: обороты каждого мотора живьём

### [ ] 088 · Обороты: число полюсов и проверка
- **BF:** `motor_poles`, `dshot_bidir` ошибки (%)
- **Файлы:** copter/Params.h · **Тест:** test_copter · **UI:** «Моторы»: % ошибок телеметрии

### [ ] 089 · Двунаправленный DShot: стенд — Стенд: да

### [ ] 090 · Обороты из DShot — в режекторы и модель
- **BF:** RPM filter
- **Что:** источник оборотов — телеметрия; модель уточняет kT/kQ по реальным оборотам на висении.
- **Файлы:** copter/GyroFilter.h, copter/CopterController.h · **Тест:** test_copter_sim с «телеметрией» из модели · **UI:** «Фильтры»

### [ ] 091 · Extended DShot telemetry
- **BF:** `dshot_edt`
- **Что:** температура, напряжение, ток ESC.
- **Файлы:** hal/stm32/Stm32Dshot.h · **Тест:** test_copter (декодер) · **UI:** «Моторы»

### [ ] 092 · ESC-телеметрия по UART (KISS/BLHeli32)
- **BF:** `esc_sensor` (feature ESC_SENSOR)
- **Файлы:** новый sensors/EscSensor.h · **Тест:** test_app_stm32_copter · **UI:** «Моторы»

### [ ] 093 · Dynamic idle на реальных оборотах
- **BF:** dyn_idle_* (см. 041)
- **Что:** переключить 041 на телеметрию.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim · **UI:** «Моторы»

### [ ] 094 · ESC passthrough (BLHeli 4-way)
- **BF:** passthrough для BLHeliSuite/ESC-Configurator через USB полётника
- **Что:** протокол 4-way-if, однопроводной UART на пинах моторов; требует MSP (пункт 158) или свой вход.
- **Файлы:** новый hal/stm32/Stm32EscPassthrough.h · **Тест:** test_app_stm32_copter на фейке · **UI:** кнопка «Открыть ESC-Configurator»

### [ ] 095 · ESC passthrough: стенд — Стенд: да

### [ ] 096 · Studio: вкладка «Моторы» — Репо: studio
- **Что:** схема рамы, протокол, холостые, ползунки моторов с защитой, обороты/температура, направление кликом.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] D03 · Документация: моторы и DShot (ru + 11 языков)

---

## 5. ARM, failsafe, безопасность, пищалка

### [ ] 097 · Угол для ARM и автодисарм
- **BF:** `small_angle`, `auto_disarm_delay`
- **Сейчас:** 25° зашит; автодисарма нет.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** «Безопасность»

### [ ] 098 · Airmode с порогом газа
- **BF:** `airmode_start_throttle_percent`, режим AIRMODE, feature AIRMODE
- **Сейчас:** airmode — всегда (CopterSettings).
- **Файлы:** copter/Mixer.h, copter/CopterController.h · **Тест:** test_copter · **UI:** «Безопасность» + режим

### [ ] 099 · Runaway takeoff prevention
- **BF:** `runaway_takeoff_prevention`, `runaway_takeoff_deactivate_delay`, `runaway_takeoff_deactivate_throttle_percent`
- **Что:** не та сторона мотора/неверная плата → коптер «убегает» на взлёте → DISARM.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim: перепутанные моторы → DISARM за 1 с · **UI:** «Безопасность»

### [ ] 100 · PREARM
- **BF:** режим PREARM
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** режим

### [ ] 101 · Флаги запрета ARM как в BF
- **BF:** arming disable flags (RXLOSS, FAILSAFE, ANGLE, CALIB, CLI, CMS, BOXFAILSAFE, RUNAWAY, CRASH, THROTTLE, MSP, PARALYZE, GPS, RESC, RPMFILTER, REBOOT_REQD, DSHOT_BBANG, NO_ACC_CAL, MOTOR_PROTO, ARMSWITCH)
- **Сейчас:** наши ArmBlock с текстом.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** строка состояния: значки причин

### [ ] 102 · Failsafe: этапы и задержки
- **BF:** `failsafe_delay`, `failsafe_off_delay` / `failsafe_landing_time`, `failsafe_throttle`, `failsafe_throttle_low_delay`, `failsafe_recovery_delay`, `failsafe_stick_threshold`
- **Сейчас:** охрана 0.3 с, снижение 0.7 м/с, возврат через 1 с, максимум 120 с.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim · **UI:** вкладка «Failsafe»: шкала времени этапов

### [ ] 103 · Failsafe: процедура
- **BF:** `failsafe_procedure` (AUTO-LAND / DROP / GPS-RESCUE), `failsafe_switch_mode` (STAGE1/STAGE2/KILL), режим FAILSAFE
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim: каждая процедура · **UI:** «Failsafe»

### [ ] 104 · Значения каналов при потере связи
- **BF:** `rxfail <канал> <режим> <значение>`
- **Файлы:** rc/RcReceiver.h, src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** «Failsafe»

### [ ] 105 · Команды стиками
- **BF:** ARM/DISARM стиками, калибровка гироскопа/акселерометра стиками, выбор профиля стиками, триммеры акселерометра стиками, режим STICK COMMANDS DISABLE
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** «Режимы»: картинка комбинаций стиков

### [ ] 106 · PARALYZE
- **BF:** режим PARALYZE
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** режим

### [ ] 107 · Пищалка: события
- **BF:** `beeper <условие>` (GYRO_CALIBRATED, RX_LOST, RX_LOST_LANDING, DISARMING, ARMING, ARMING_GPS_FIX, BAT_CRIT_LOW, BAT_LOW, GPS_STATUS, RX_SET, ACC_CALIBRATION, READY_BEEP, DISARM_REPEAT, ARMED, SYSTEM_INIT, ON_USB, CRASH_FLIP, …), `beeper_inversion`, `beeper_od`, `beeper_frequency`
- **Сейчас:** пищалки у коптера нет (у самолёта есть control/Beeper.h).
- **Файлы:** control/Beeper.h, src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** вкладка «Пищалка»: галочки событий, кнопка «послушать»

### [ ] 108 · Пищалка: режимы
- **BF:** BEEPER, BEEPER MUTE, BEEP GPS SATELLITE COUNT
- **Файлы:** src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** режимы

### [ ] 109 · DShot-маяк
- **BF:** `beeper_dshot_beacon_tone`, `dshot_beacon <условие>`
- **Что:** моторы пищат, чтобы найти коптер в траве (после DShot).
- **Файлы:** hal/stm32/Stm32Dshot.h · **Тест:** test_app_stm32_copter · **UI:** «Пищалка»

### [ ] 110 · Studio: вкладки «Безопасность», «Failsafe», «Режимы», «Пищалка» — Репо: studio
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] D04 · Документация: безопасность (ru + 11 языков)

---

## 6. Пульт: сглаживание, приёмники, телеметрия, крутилки

### [ ] 111 · RC smoothing как в BF
- **BF:** `rc_smoothing`, `rc_smoothing_auto_factor`, `rc_smoothing_auto_factor_throttle`, `rc_smoothing_setpoint_cutoff`, `rc_smoothing_feedforward_cutoff`, `rc_smoothing_throttle_cutoff`
- **Что:** частота кадров приёмника измеряется → частоты фильтров автоматически.
- **Сейчас:** PT1 40 Гц на стиках.
- **Файлы:** copter/CopterController.h, rc/RcReceiver.h · **Тест:** test_copter: 50/150/500 Гц кадров · **UI:** «Приёмник»: частота кадров и фильтры

### [ ] 112 · Мёртвые зоны
- **BF:** `deadband`, `yaw_deadband`
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** «Приёмник»

### [ ] 113 · RSSI и качество связи
- **BF:** `rssi_channel`, `rssi_scale`, `rssi_offset`, `rssi_src_frame_errors`, LQ CRSF
- **Файлы:** rc/RcReceiver.h · **Тест:** test_rc · **UI:** «Приёмник»: полоса RSSI/LQ

### [ ] 114 · Приёмник GHST
- **BF:** `serialrx_provider = GHST`
- **Файлы:** новый rc/GhstParser.h · **Тест:** test_rc (эталонные кадры по описанию протокола) · **UI:** «Приёмник»

### [ ] 115 · Приёмник Spektrum / SRXL2
- **BF:** `serialrx_provider = SPEKTRUM1024/2048/SRXL/SRXL2`
- **Файлы:** новый rc/SpektrumParser.h · **Тест:** test_rc · **UI:** «Приёмник»

### [ ] 116 · Приёмник FPort
- **BF:** `serialrx_provider = FPORT` (SBUS + SmartPort по одному проводу)
- **Файлы:** новый rc/FportParser.h · **Тест:** test_rc · **UI:** «Приёмник»

### [ ] 117 · Инверсия и полудуплекс UART приёмника
- **BF:** `serialrx_inverted`, `serialrx_halfduplex`
- **Файлы:** hal/stm32/Stm32UartPort.h · **Тест:** test_hal · **UI:** «Приёмник»

### [ ] 118 · Телеметрия CRSF (ExpressLRS)
- **BF:** CRSF telemetry: батарея, ориентация, режим, GPS, вариометр
- **Файлы:** новый telemetry/CrsfTelemetry.h · **Тест:** test_telemetry: кадры с CRC · **UI:** «Приёмник»: что уходит на пульт

### [ ] 119 · Телеметрия iBUS (FlySky, iBUS-SENS)
- **BF:** IBUS telemetry
- **Что:** у пользователя FS-i6: напряжение, высота, режим на экране пульта.
- **Файлы:** новый telemetry/IbusTelemetry.h · **Тест:** test_telemetry · **UI:** «Приёмник»

### [ ] 120 · Телеметрия SmartPort (FrSky)
- **BF:** SmartPort telemetry
- **Файлы:** новый telemetry/SmartPortTelemetry.h · **Тест:** test_telemetry · **UI:** «Приёмник»

### [ ] 121 · Подстройки в полёте крутилками
- **BF:** `adjrange` (RC_RATE, P/I/D по осям, профили…)
- **Что:** у пользователя VRA/VRB.
- **Файлы:** copter/Params.h, src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** вкладка «Подстройки»

### [ ] 122 · Профиль крутилкой/тумблером
- **BF:** adjustment RATE_PROFILE/PID_PROFILE, `profile` через aux
- **Файлы:** src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** «Подстройки»

### [ ] 123 · Studio: вкладки «Приёмник» и «Подстройки» — Репо: studio
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] D05 · Документация: пульт и телеметрия (ru + 11 языков)

---

## 7. Батарея и ток

### [ ] 124 · Напряжения банки
- **BF:** `vbat_max_cell_voltage`, `vbat_full_cell_voltage`, `vbat_min_cell_voltage`, `vbat_warning_cell_voltage`, `vbat_hysteresis`, `vbat_detect_cell_voltage`, `force_battery_cell_count`
- **Сейчас:** minCellVolts 3.3 для ARM; число банок — из описания.
- **Файлы:** copter/CopterController.h, src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** вкладка «Питание»

### [ ] 125 · Делитель и датчик тока
- **BF:** `vbat_scale`, `vbat_divider`, `vbat_multiplier`, `ibata_scale`, `ibata_offset`, `current_meter`, `battery_meter`
- **Сейчас:** BLACKBOX_VBAT_DIVIDER зашит, ток не меряется.
- **Файлы:** src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** «Питание»: калибровка по мультиметру

### [ ] 126 · Ёмкость и предупреждения
- **BF:** `bat_capacity`, `use_vbat_alerts`, `use_cbat_alerts`, `cbat_alert_percent`, `vbat_cutoff_percent`, `vbat_duration_for_warning`, `vbat_duration_for_critical`, `vbat_lpf_period`, `ibat_lpf_period`
- **Файлы:** новый copter/Battery.h · **Тест:** test_copter · **UI:** «Питание»

### [ ] 127 · Studio: вкладка «Питание» — Репо: studio

### [ ] D06 · Документация: питание (ru + 11 языков)

---

## 8. Высота, GPS, возврат

### [ ] 128 · Удержание высоты как в BF 2025
- **BF:** режим ALTHOLD, `alt_hold_adjust_rate`, `alt_hold_deadband`, `autopilot_*` (газ висения, PID высоты)
- **Сейчас:** свой ALT_HOLD (VerticalEstimator + AltitudeController, обучение висения).
- **Файлы:** copter/AltitudeHold.h, copter/Params.h · **Тест:** test_copter_sim · **UI:** вкладка «Автопилот»

### [ ] 129 · GPS для коптера
- **BF:** `gps_provider`, `gps_sbas_mode`, `gps_auto_config`, `gps_set_home_point_once`, `gps_use_3d_speed`
- **Что:** подключить существующий UbloxM10 к коптеру, дом при ARM.
- **Файлы:** src/stm32/main_copter.cpp, sensors/gps/* · **Тест:** test_app_stm32_copter с NAV-PVT · **UI:** вкладка «GPS»: карта, спутники

### [ ] 130 · Оценка положения и скорости (GPS + акселерометр)
- **BF:** основа POSHOLD/RESCUE
- **Файлы:** новый copter/PositionEstimator.h · **Тест:** test_copter_sim с моделью GPS (шум, задержка 0.1 с) · **UI:** «GPS»

### [ ] 131 · Компас для коптера
- **BF:** `mag_hardware`, `align_mag`, `mag_declination`, калибровка компаса
- **Файлы:** src/stm32/main_copter.cpp, sensors/mag/* · **Тест:** test_app_stm32_copter · **UI:** «Настройка платы»: калибровка компаса

### [ ] 132 · Удержание позиции
- **BF:** режим POSHOLD, `pos_hold_adjust_rate`, `pos_hold_deadband`, `pos_hold_without_mag`
- **Файлы:** новый copter/PositionHold.h · **Тест:** test_copter_sim: ветер 5 м/с — дрейф < 2 м · **UI:** «Автопилот»

### [ ] 133 · GPS Rescue: набор высоты и разворот
- **BF:** `gps_rescue_min_start_dist`, `gps_rescue_alt_mode`, `gps_rescue_initial_climb`, `gps_rescue_ascend_rate`, `gps_rescue_return_alt`
- **Файлы:** новый copter/GpsRescue.h · **Тест:** test_copter_sim · **UI:** вкладка «Возврат»

### [ ] 134 · GPS Rescue: полёт домой
- **BF:** `gps_rescue_ground_speed`, `gps_rescue_max_angle`, `gps_rescue_roll_mix`, `gps_rescue_pitch_cutoff`, `gps_rescue_yaw_p`, `gps_rescue_velocity_p/i/d`, `gps_rescue_use_mag`
- **Файлы:** copter/GpsRescue.h · **Тест:** test_copter_sim: с 300 м домой при ветре · **UI:** «Возврат»

### [ ] 135 · GPS Rescue: снижение и посадка
- **BF:** `gps_rescue_descent_dist`, `gps_rescue_descend_rate`, `gps_rescue_landing_alt`, `gps_rescue_disarm_threshold`, `gps_rescue_throttle_min/max/hover`, `gps_rescue_throttle_p/i/d`
- **Файлы:** copter/GpsRescue.h · **Тест:** test_copter_sim: посадка в 5 м от дома · **UI:** «Возврат»

### [ ] 136 · GPS Rescue: проверки
- **BF:** `gps_rescue_sanity_checks`, `gps_rescue_min_sats`, `gps_rescue_allow_arming_without_fix`, режим GPS RESCUE
- **Файлы:** copter/GpsRescue.h, copter/CopterController.h · **Тест:** test_copter_sim: потеря GPS в пути → посадка · **UI:** «Возврат»

### [ ] 137 · Studio: вкладки «Автопилот», «GPS», «Возврат» — Репо: studio
- **Что:** карта с домом и траекторией, профиль высоты возврата.
- **UI:** сам пункт

### [ ] D07 · Документация: высота и GPS (ru + 11 языков)

---

## 9. Чёрный ящик, отладка, цикл

### [ ] 138 · Чёрный ящик коптера
- **BF:** `blackbox_device`, `blackbox_mode` (NORMAL/MOTOR_TEST/ALWAYS), `blackbox_sample_rate`
- **Что:** на SD (как самолёт): гироскоп сырой/после фильтров, уставки, P/I/D/F, моторы, газ, батарея, режим.
- **Файлы:** telemetry/BlackBox*.h (образец), новый telemetry/CopterBlackBox.h · **Тест:** test_app_stm32_copter + разбор · **UI:** вкладка «Чёрный ящик»

### [ ] 139 · Формат журнала Betaflight (.BFL)
- **BF:** формат blackbox (заголовок H, кадры I/P, кодирование знаков переменной длины)
- **Что:** писать в формате BF — открывается в Blackbox Explorer и PIDtoolbox; своя реализация по описанию формата.
- **Файлы:** новый telemetry/BflEncoder.h · **Тест:** разбор нашим декодером + проверка заголовка; отдельно — открыть в Blackbox Explorer (пользователь) · **UI:** «Чёрный ящик»: «открыть в Blackbox Explorer»

### [ ] 140 · Выбор полей и debug_mode
- **BF:** `blackbox_disable_*`, `blackbox_high_resolution`, `debug_mode` (GYRO_SCALED, FFT, RPM_FILTER, FEEDFORWARD, D_MAX, RC_SMOOTHING…)
- **Файлы:** telemetry/CopterBlackBox.h · **Тест:** test_app_stm32_copter · **UI:** «Чёрный ящик»

### [ ] 141 · Быстрый цикл: IMU 8 кГц по DRDY
- **BF:** `gyro_rate`, частота гироскопа
- **Что:** ICM42688 на 8 кГц, чтение SPI по прерыванию готовности с DMA.
- **Файлы:** sensors/imu/ICM42688_Sensor.h, hal/stm32/Stm32SpiBus.h · **Тест:** test_app_stm32_copter на фейке · **UI:** «Настройка платы»: частоты

### [ ] 142 · Быстрый цикл: PID 2–4 кГц вне тика FreeRTOS
- **BF:** `pid_process_denom`
- **Что:** цикл от таймера/DRDY, консоль/барометр — в задачах ниже; модель контура пересчитывает настройку под частоту.
- **Файлы:** src/stm32/main_copter.cpp, hal/Rtos.h · **Тест:** test_app_stm32_copter; test_copter_sim: whoop на 4 кГц (уже есть) · **UI:** «Настройка платы»

### [ ] 143 · Быстрый цикл: стенд — Стенд: да
- **Что:** время цикла и джиттер на Matek H743 (`status`, `tasks`).

### [ ] 144 · Studio: вкладка «Чёрный ящик» — Репо: studio
- **Что:** скачать журнал, графики гироскопа/уставок/моторов, спектр, кнопка «открыть в Blackbox Explorer».
- **UI:** сам пункт

### [ ] D08 · Документация: чёрный ящик и цикл (ru + 11 языков)

---

## 10. OSD, VTX, LED, камера

### [ ] 145 · OSD: драйвер AT7456E (MAX7456) на Matek H743
- **BF:** `osd_*`, `max7456_*`
- **Что:** SPI-драйвер, видеостандарт, вывод символов, загрузка шрифта.
- **Файлы:** новый hal/stm32/Max7456.h · **Тест:** test_app_stm32_copter на регистровом эмуляторе · **UI:** вкладка «OSD»

### [ ] 146 · OSD: шрифт и загрузка
- **BF:** `font` / загрузка шрифта MCM
- **Файлы:** hal/stm32/Max7456.h · **Тест:** эмулятор · **UI:** «OSD»: выбор шрифта

### [ ] 147 · OSD: элементы полёта
- **BF:** `osd_vbat_pos`, `osd_rssi_pos`, `osd_flymode_pos`, `osd_throttle_pos`, `osd_crosshairs_pos`, `osd_ah_pos`, `osd_ah_sbar_pos`, `osd_tim_1_pos`, `osd_warnings_pos`, `osd_link_quality_pos`
- **Файлы:** новый telemetry/Osd.h · **Тест:** test_app_stm32_copter: экран в буфере · **UI:** «OSD»: перетаскивание элементов по экрану

### [ ] 148 · OSD: элементы навигации
- **BF:** `osd_altitude_pos`, `osd_vario_pos`, `osd_gps_speed_pos`, `osd_gps_sats_pos`, `osd_home_dir_pos`, `osd_home_dist_pos`, `osd_gps_lat_pos`, `osd_gps_lon_pos`, `osd_compass_bar_pos`
- **Файлы:** telemetry/Osd.h · **Тест:** test_app_stm32_copter · **UI:** «OSD»

### [ ] 149 · OSD: питание и моторы
- **BF:** `osd_current_pos`, `osd_mah_drawn_pos`, `osd_avg_cell_voltage_pos`, `osd_esc_tmp_pos`, `osd_esc_rpm_pos`, `osd_power_pos`
- **Файлы:** telemetry/Osd.h · **Тест:** test_app_stm32_copter · **UI:** «OSD»

### [ ] 150 · OSD: предупреждения и статистика после полёта
- **BF:** `osd_warn_*`, `osd_stat_*`, единицы, `osd_cap_alarm`, `osd_alt_alarm`
- **Файлы:** telemetry/Osd.h · **Тест:** test_app_stm32_copter · **UI:** «OSD»

### [ ] 151 · OSD: меню настроек стиками (CMS)
- **BF:** CMS
- **Что:** PID, расходы, фильтры, VTX из очков без компьютера.
- **Файлы:** новый telemetry/OsdMenu.h · **Тест:** test_app_stm32_copter · **UI:** —

### [ ] 152 · OSD: стенд — Стенд: да

### [ ] 153 · VTX: SmartAudio и Tramp
- **BF:** `vtx_band`, `vtx_channel`, `vtx_power`, `vtx_pit_mode_freq`, `vtxtable`, режим VTX PIT MODE
- **Файлы:** новый telemetry/VtxControl.h · **Тест:** test_telemetry: кадры протоколов · **UI:** вкладка «Видео»

### [ ] 154 · Цифровое видео (DJI/Walksnail/HDZero) — MSP DisplayPort
- **BF:** MSP DisplayPort OSD
- **Что:** OSD в цифровых очках; требует MSP (158).
- **Файлы:** telemetry/Osd.h · **Тест:** test_app_stm32_copter · **UI:** «OSD»

### [ ] 155 · LED-лента WS2812
- **BF:** `ledstrip_*`, `led`, `color`, `mode_color`
- **Файлы:** новый hal/stm32/Stm32LedStrip.h · **Тест:** test_app_stm32_copter · **UI:** вкладка «LED»

### [ ] 156 · Управление камерой RunCam
- **BF:** режимы CAMERA CONTROL 1–3, `camera_control_*`
- **Файлы:** новый telemetry/CameraControl.h · **Тест:** test_telemetry · **UI:** «Видео»

### [ ] 157 · Studio: вкладки «OSD», «Видео», «LED» — Репо: studio
- **Что:** редактор OSD перетаскиванием, предпросмотр шрифта.
- **UI:** сам пункт

### [ ] D09 · Документация: OSD и видео (ru + 11 языков)

---

## 11. MSP и совместимость

### [ ] 158 · MSP: транспорт и базовые сообщения
- **BF:** MSP v1/v2 (API_VERSION, FC_VARIANT, FC_VERSION, BOARD_INFO, STATUS, RAW_IMU, ATTITUDE, ALTITUDE, ANALOG, RC, MOTOR)
- **Что:** своя реализация по открытому описанию протокола.
- **Файлы:** новый telemetry/Msp.h · **Тест:** test_telemetry: кадры с CRC (v1 xor, v2 crc8 dvb-s2) · **UI:** —

### [ ] 159 · MSP: чтение и запись настроек
- **BF:** MSP_PID / SET_PID, RC_TUNING, FILTER_CONFIG, ADVANCED_CONFIG, MODE_RANGES, MOTOR_CONFIG, FAILSAFE_CONFIG, EEPROM_WRITE
- **Файлы:** telemetry/Msp.h, copter/Params.h · **Тест:** test_telemetry · **UI:** —

### [ ] 160 · Подключение конфигуратора Betaflight — Стенд: да
- **Что:** проверить, какие вкладки конфигуратора BF работают поверх 158–159; честно записать ограничения.

### [ ] 161 · MSP для ELRS Backpack и очков
- **BF:** MSP over CRSF, DisplayPort
- **Файлы:** telemetry/Msp.h · **Тест:** test_telemetry · **UI:** —

### [ ] 162 · MAVLink для коптера
- **BF:** — (у нас есть для самолёта)
- **Что:** HEARTBEAT типа QUADROTOR, ATTITUDE, позиция, режимы — QGroundControl.
- **Файлы:** telemetry/MavlinkTelemetry.h (образец) · **Тест:** test_mavlink · **UI:** —

### [ ] 163 · Импорт diff из Betaflight
- **BF:** вывод `diff all`
- **Что:** вставить diff BF в нашу CLI — понятное применяется, непонятное перечисляется списком.
- **Файлы:** telemetry/CopterConsole.h · **Тест:** test_app_stm32_copter: настоящий diff 4.5 · **UI:** «CLI»: кнопка «вставить diff из BF»

### [ ] D10 · Документация: совместимость (ru + 11 языков)

---

## 12. Режимы и мелочи BF

### [ ] 164 · HEADFREE и HEADADJ
- **BF:** режимы HEADFREE, HEADADJ
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim · **UI:** режимы

### [ ] 165 · FPV angle mix
- **BF:** `fpv_mix_degrees`, режим FPV ANGLE MIX
- **Файлы:** copter/CopterController.h · **Тест:** test_copter · **UI:** «Rates»

### [ ] 166 · Стабилизация камеры сервами
- **BF:** режим CAMSTAB, `gimbal_*`, сервы
- **Файлы:** copter/CopterController.h, hal/stm32/Stm32ServoOutput.h · **Тест:** test_app_stm32_copter · **UI:** вкладка «Сервы»

### [ ] 167 · USER1–USER4 (пины по тумблеру)
- **BF:** режимы USER1–4, `pinio_box`
- **Файлы:** src/stm32/main_copter.cpp · **Тест:** test_app_stm32_copter · **UI:** «Режимы»

### [ ] 168 · Chirp — измерение коптера в полёте
- **BF:** режим CHIRP (BF 2025), `chirp_*`
- **Что:** качание частот по оси → настоящие K и τ → уточнение модели и пересчёт настройки (наше: автонастройка по измерению).
- **Файлы:** новый copter/Chirp.h, copter/Presets.h · **Тест:** test_copter_sim: по chirp «вялой» физики восстановлены K и τ ±15 % · **UI:** вкладка «Автонастройка»: кнопка, график Боде

### [ ] 169 · Автонастройка по chirp: стенд/полёт — Стенд: да

### [ ] 170 · Studio: вкладки «Сервы» и «Автонастройка» — Репо: studio

### [ ] D11 · Документация: режимы и автонастройка (ru + 11 языков)

---

## 13. Итог

### [ ] 171 · Полная проверка: все сборки × все физики со всеми функциями
- **Что:** сводная таблица в COPTER.md; полный прогон тестов; покрытие gcovr.

### [ ] 172 · Первый полёт — Стенд: да
- **Что:** без пропеллеров → привязанный → висение в ANGLE → ALT_HOLD → ACRO; журнал в BF-формате (139) → сравнение с симуляцией.

### [ ] D12 · Документация: итог, README, дорожная карта (ru + 11 языков)
