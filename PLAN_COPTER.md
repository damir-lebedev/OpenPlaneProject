# PLAN_COPTER.md — всё из Betaflight в коптерную прошивку

Подробности каждого пункта BF. **Порядок работы — в `PLAN_MVP.md`** (с него
начинать после «продолжай работать по плану»); здесь — только описание
пунктов, которые он называет. Протокол батча:

1. Взять следующие пункты по списку PLAN_MVP.md (до ~5 за раз); прочитать
   здесь только их: `grep -n "^### \[.\] NNN" PLAN_COPTER.md`, Read с
   offset/limit. Пункты «Стенд: да» — только вместе с пользователем, когда
   железо на столе.
2. Читать только файлы из строки «Файлы»; искать — grep, а не чтение целиком.
   Память: `copter-firmware.md` (как устроено ядро).
3. Сделать прошивку **и сразу её UI в Studio** (`../openplane-studio`:
   вкладка, наглядная картинка, перевод en.json — `python tools/i18n_check.py`,
   `npx tsc --noEmit && npx vitest run`). Покрыть тестами, проверить:
   `export PATH="$HOME/tools/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH"`
   `pio test -e native -f native/test_copter -f native/test_copter_sim 2>&1 | grep -E "error:|FAIL|cases"`
   `pio test -e native-stm32 -f native_stm32/test_app_stm32_copter 2>&1 | tail -1`
   `pio run -e aocodarc-f405v3-quad -e matek-h743-quad -e stm32h743-devebox-quad 2>&1 | grep -E "RAM:|error|FAILED|succeeded"`
   (getfun-f405v3-quad не собирается без дампа платы — так и задумано.)
   Один сим-тест быстро: скопировать test_main.cpp, оставить нужные RUN_TEST,
   `g++ -std=gnu++17 -O2 -DUNITY_INCLUDE_DOUBLE -I include -I test/native -I lib -I .pio/libdeps/native/Unity/src x.cpp -x c .pio/libdeps/native/Unity/src/unity.c`.
4. Готовый пункт **удалить** отсюда и дописать номером в строку «Сделано» ниже;
   частично готовый — `[~]` и строка «Ход». Потом `PYTHONIOENCODING=utf-8 python tools/plan_to_json.py`.
5. Новые настройки — строкой в таблицу «Настройки» в `docs/COPTER.md` (только
   русский; переводы — пунктами «D»).
6. Коммит по путям (airframe и `stack/` не трогать никогда; `git stash` не
   использовать — пользователь держит airframe в индексе): прошивка и Studio —
   раздельно, затем план; `git push origin main` в обоих репо.
7. Ответ пользователю — 5–10 строк: что сделано, что проверено, что дальше.

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

Решения (прожарка плана, 2026-10-24):
- Цель — публичная прошивка на замену BF, полный паритет; ничего не выкидываем,
  ненужное 7"-у пользователя идёт позже.
- Главная плата — GETFUN F405 пользователя (7", BLHeli_S → Bluejay), затем
  популярные: H743, F722, AT32F435. Железные пункты (DShot, ящик, цикл, OSD) —
  сначала под F405.
- Железа до 22 ноября 2026 нет (DevEBox занят самолётом): раздел 1 — на
  симуляторе и фейках, раздел 2 — с железом и пользователем.
- Первые полёты — на родном BF (пилот учится, эталонные журналы); наша
  прошивка — только после подгонки модели по журналу (182), лестницей 172.
- Конфигуратор BF должен работать: MSP представляется как BTFL / BF 4.5
  (API 1.46), в имени платы и сборки — «OpenPlane …».
- «Не хуже BF» — критерии пункта 183; релиз 1.0 — раздел 13.
- Переводы — к релизам (D13–D15), не по разделам; русский — сразу.
- Разделы 14–16 — после 1.0.

Решения (2026-11-02, просьба пользователя: «самый топовый софт на рынке»):
- **Каждая фича — сразу со своим UI в Studio** (вкладка, наглядная картинка:
  график, схема, экран), а не «после 1.0». Новичку — понятные слова и
  картинки, профи — все настройки, пришедшему с BF — имена BF рядом.
- Интерфейс BF не копируем: делаем красиво и по-своему; совместимость с BF —
  только по именам настроек, diff, MSP и CLI.
- DShot2400/DShot1200 не делаем: в BF их нет, регуляторы (BLHeli_S/Bluejay,
  AM32, BLHeli_32) их не держат; потолок — DShot600 + bidir, уже через DMA
  без прерываний (Stm32Dshot.h).
- Фильтр Калмана гироскопа (как в EmuFlight/IMUF) — исследование после 1.0
  (пункт 189): берём, только если сим и журналы покажут выигрыш против
  RPM-фильтра + динамических режекторов BF.

Статусы: `[ ]` не начат · `[~]` в работе · `[!]` заблокирован (причина в «Итог»).
Готовый пункт **удаляется** из плана (строка в сводку ниже), частично
сделанный — `[~]` и строка «Ход: сделано… осталось…».

Сделано и проверено (из плана убрано, подробности — `git log`, docs/COPTER.md): 001 Реестр настроек коптера, 002 CLI как в Betaflight, 003 PID в единицах Betaflight, 004 Дефолты Betaflight 4.5 как источник настройки, 005 Профили PID и расходов, 006 Режимы на тумблерах как в BF, 007 Карта каналов и диапазон приёмника, 008 status и tasks как в BF, 009 Описание коптера, 010 Машинный протокол параметров для приложения, 097 Угол для ARM и автодисарм, 099 Runaway takeoff prevention, 101 Флаги запрета ARM как в BF, 102 Failsafe: этапы и задержки, 103 Failsafe: процедура, 104 Значения каналов при потере связи, 124 Напряжения банки, 125 Делитель и датчик тока, 126 Ёмкость и предупреждения, 080 DShot: кадры на таймере с DMA (F405 и H743), 081 DShot: несколько таймеров и пины F405 / H743, 087 Двунаправленный DShot: приём ответа (GCR), 088 Обороты: число полюсов и проверка, 177 Чёрный ящик на SPI-флеше (F405, 16 МБ), 139 Формат журнала Betaflight (.BFL), 158 MSP: транспорт и базовые сообщения, 159 MSP: чтение и запись настроек, 163 Импорт diff из Betaflight, 011 d_max (d_min), 012 iterm_relax, 013 iterm_rotation, 014 iterm_windup, 015 iterm_limit, 016 pidsum_limit, pidsum_limit_yaw, 017 anti_gravity, 018 throttle_boost, 019 TPA как в BF, 020 tpa_low, 021 feedforward_transition, 022 feedforward_averaging, 023 feedforward_smooth_factor, 024 feedforward_jitter_factor, 025 feedforward_boost, 026 feedforward_max_rate_limit, 027 yaw_lowpass_hz, 028 integrated yaw, 029 absolute control, 030 thrust_linear, motor_output_limit, 031 vbat_sag_compensation (%), 033 auto_profile_cell_count, 034 Режим ANGLE как в BF 4.5, 035 Режим HORIZON как в BF 4.5, 036 Acro trainer, 038 Crash recovery, 079 Проверка моторов из CLI, 098 Airmode с порогом газа, 100 PREARM, 111 RC smoothing как в BF, 112 Мёртвые зоны, 118 Телеметрия CRSF (ExpressLRS), 129 GPS для коптера, 133 GPS Rescue: набор высоты и разворот, 134 GPS Rescue: полёт домой, 135 GPS Rescue: снижение и посадка, 136 GPS Rescue: проверки, 138 Чёрный ящик коптера, 144 Studio: вкладка «Чёрный ящик», 148 OSD: элементы навигации, 150 OSD: предупреждения и статистика после полёта, 153 VTX: SmartAudio и Tramp, 083 DShot: команды, 084 Направление моторов командой DShot, 086 Turtle mode.

---

## 0. Основа

---

## 1. До 22 ноября: первый полёт в коде

Всё проверяется в симуляторе и на фейках, железа нет. Порядок: безопасность и батарея → DShot под F405 → чёрный ящик на флеше и журнал BF → MSP под конфигуратор BF.

### [ ] 160 · Подключение конфигуратора Betaflight — Стенд: да
- **Что:** конфигуратор BF подключается к нам как к BF 4.5 (158) и настраивает PID, Rates, приёмник, режимы, моторы, CLI; что отображается странно — записать, что не работает — чинить в 159. Можно на DevEBox, коптер не нужен.

### [ ] D13 · Переводы: до первого полёта (11 языков)
- **Что:** всё, что поменялось в русских COPTER.md и README с прошлых переводов, — в docs/i18n/<lang>/ на 11 языков.

---

### [ ] 190 · Studio: режимы «Новичок» и «Профи» — Репо: studio
- **Что:** переключатель в шапке коптера. «Новичок»: мастер по шагам (рама и винты → пульт и режимы → моторы без винтов → failsafe и возврат → батарея → OSD и видео → «можно лететь») и одна страница «Настройка полёта» с 5–6 ползунками (отзывчивость, плавность, фильтры, расходы, предел наклона) — без единого имени настройки. «Профи»: все вкладки и все настройки как сейчас, поиск по имени.
- **Файлы:** src/components/CopterView.tsx, новый src/components/copter/Wizard.tsx, core/copterParams.ts
- **Тест:** vitest: новичок не видит профи-вкладок; мастер доходит до «можно лететь» на фейковой плате
- **UI:** сам пункт

### [ ] 191 · Studio: переход с Betaflight — имена BF и поиск — Репо: studio
- **Что:** у каждой строки настройки — понятное название крупно и имя BF мелко (`p_roll`); поиск сверху по имени BF или по-русски («газ висения», `dyn_notch`) — переход на нужную вкладку с подсветкой; страница «Я с Betaflight»: какая вкладка BF где у нас, что сделано иначе и почему (наши отличия: AUTO-LAND по умолчанию, посадка вместо DISARM в GPS Rescue, airmode в воздухе…), импорт diff — уже есть.
- **Файлы:** src/components/copter/common.tsx, новый src/components/copter/Search.tsx
- **Тест:** vitest: поиск по имени BF открывает вкладку · **UI:** сам пункт

### [ ] 192 · Хранилище настроек больше 2 КБ
- **Что:** KeyValueStore — 2 КБ на всю плату, заданная руками настройка — 7 байт; большой diff BF (сотни строк, OSD-раскладка) не влезет — `save` ответит FLASH_FULL. Свой сектор флеша под настройки коптера (F405: 16 КБ, H743: 128 КБ) или второй ключ.
- **Файлы:** storage/KeyValueStore.h, copter/Params.h, hal/stm32/*
- **Тест:** test_copter: diff на 400 строк сохраняется и читается после «перезагрузки» · **UI:** «Прочее»: занято N из M

## 2. С 22 ноября: железо, эталон BF, первый полёт

Только с пользователем и железом. Сначала родной BF (сборка, пилот, эталонные журналы), потом Bluejay, модель по журналу — и лишь тогда наша прошивка.

### [ ] 178 · GETFUN F405: дамп BF, бэкап родной прошивки, заголовок платы — Стенд: да
- **Что:** шаг 1 из Q12 PLAN_QUAD7.md: `tools/bf_board.py read`, `backup`, `header` → include/config/boards/GETFUNF405V3.h без заглушки; сборка getfun-f405v3-quad. Родная прошивка BF — запасная (летаем на ней в 179–181).
- **Файлы:** tools/bf_board.py, include/config/boards/GETFUNF405V3.h, stack/

### [ ] 179 · Первые полёты на родном Betaflight (BLHeli_S) — Стенд: да
- **Что:** 3–5 аккумуляторов на родном BF: проверка сборки (направления моторов, приёмник, VTX, вибрации) и полёты пилота на настоящем коптере; журнал BF включён.

### [ ] 082 · DShot: проверка на стенде — Стенд: да
- **Что:** логический анализатор или ESC: кадры, взвод ESC, газ.
- **UI:** —

### [ ] 180 · Bluejay на регулятор — Стенд: да
- **Что:** после 179: через passthrough BF в ESC-Configurator сохранить родную BLHeli_S (`E_X_40`), прошить Bluejay `E_X_40_48_v0.21.1-RC1` (BB51 с разводкой E есть только с 0.21.1-RC1); в BF — `dshot_bidir = ON`, `motor_poles` по мотору (2810 — обычно 14, проверить), RPM-фильтр; ошибки телеметрии оборотов < 1 %. Телеметрия регулятора (EDT, UART: 091, 092) не нужна — после 1.0.

### [ ] 089 · Двунаправленный DShot: стенд — Стенд: да
- **Что:** захват ответа регулятора на железе — Stm32DshotCapture (087 сделал всё, кроме него): у F405 читать порт по таймеру умеет только DMA2, его потоки TIMx_UP заняты TIM1/TIM8 — вариант BF (bitbang: моторы как GPIO через BSRR и чтение IDR одним DMA2 на порт) или захват фронтов таймером с DMA по каналу; ошибки телеметрии < 1 % на Bluejay (180).

### [ ] 181 · Эталонные журналы BF с RPM-фильтром — Стенд: да
- **Что:** один коптер, винты 7040, одна батарея; манёвры: висение 3 мин, ступеньки по крену/тангажу/рысканию, резкий газ, флипы и роллы, пропвош на сбросе газа. Журналы — в Documents/OpenPlane/Blackbox/bf_ref/. Эталон для 182 и 183.

### [!] 182 · Модель симулятора по журналу BF
- **Что:** по журналам 181 восстановить K и τ осей, газ висения, шум по оборотам; обновить сборку longrange7li и QuadSim; пересчитать настройку и прогнать test_copter_sim. Обязательно до первого висения на нашей прошивке (172).
- **Файлы:** copter/Presets.h, test/native/helpers/QuadSim.h, новый tools/bf_log_fit.py
- **Тест:** test_copter_sim: модель из журнала повторяет ступеньку BF ±15 %
- **Итог:** ждёт журналы 181 (железо — после 22 ноября 2026)

### [ ] 172 · Первый полёт на нашей прошивке — Стенд: да
- **Что:** только после 181 и 182 (модель подогнана по журналу BF): без пропеллеров (`t`, `m`, `s`, пульт, OSD, failsafe) → на привязи → висение в ANGLE низко в открытом поле → ALT_HOLD → ACRO — только когда журнал совпал с BF по 183. Журнал включён всегда.

### [ ] 183 · «Не хуже BF»: сравнение по журналам — Стенд: да
- **Что:** манёвры 181 на нашей прошивке. Критерии: ступенька (время нарастания и перелёт) ±15 % от BF; шум гироскопа после фильтров (СКЗ 50–500 Гц) не больше BF; моторы после 3 мин висения не горячее; газ висения ±3 %; пропвош не хуже; failsafe, дизарм при крэше и runaway проверены вживую. Хуже — сначала модель (182), потом настройка. Таблица — в COPTER.md.
- **Файлы:** tools/bf_log_fit.py, docs/COPTER.md

### [ ] D14 · Переводы: после первого полёта (11 языков)
- **Что:** то же после 183: таблица «мы против BF», что поменялось по журналам.

---

## 3. Регулятор (PID-профиль)

### [ ] 032 · Упрощённая настройка — ползунки
- **BF:** `simplified_pids_mode`, `simplified_master_multiplier`, `simplified_i_gain`, `simplified_d_gain`, `simplified_pi_gain`, `simplified_dmax_gain`, `simplified_feedforward_gain`, `simplified_pitch_d_gain`, `simplified_pitch_pi_gain`
- **Что:** множители поверх модели/BF-дефолтов — ровно как ползунки BF; запас по усилению пересчитывать и предупреждать, если < 6 дБ.
- **Файлы:** copter/Presets.h, copter/Params.h
- **Тест:** test_copter: master ×1.5 → P,I,D,F ×1.5; предупреждение запаса
- **UI:** вкладка «PID»: ползунки + полоса «запас устойчивости» из модели (зелёная/жёлтая/красная)

### [ ] 037 · Launch control
- **BF:** `launch_control_mode` (NORMAL/PITCHONLY/FULL), `launch_trigger_allow_reset`, `launch_trigger_throttle_percent`, `launch_angle_limit`, `launch_control_gain`, режим LAUNCH CONTROL
- **Что:** гоночный старт: коптер на земле держит наклон до рывка газом.
- **Файлы:** copter/CopterController.h
- **Тест:** test_copter_sim: старт под 30°, взлёт без кувырка
- **UI:** «PID → дополнительно» + режим

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

---

## 4. Расходы (rate profile)

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

---

## 5. Фильтры

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

---

## 6. Моторы, регуляторы, DShot

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

### [ ] 085 · 3D-режим (реверс)
- **BF:** `3d_deadband_low`, `3d_deadband_high`, `3d_neutral`, `3d_deadband_throttle`, `3d_limit_low`, `3d_limit_high`, `3d_switched_mode`, режим 3D
- **Файлы:** copter/Mixer.h, copter/CopterController.h · **Тест:** test_copter · **UI:** «Моторы»

### [ ] 090 · Обороты из DShot — в режекторы и модель
- **BF:** RPM filter
- **Что:** источник оборотов — телеметрия; модель уточняет kT/kQ по реальным оборотам на висении.
- **Файлы:** copter/GyroFilter.h, copter/CopterController.h · **Тест:** test_copter_sim с «телеметрией» из модели · **UI:** «Фильтры»

### [ ] 093 · Dynamic idle на реальных оборотах
- **BF:** dyn_idle_* (см. 041)
- **Что:** переключить 041 на телеметрию.
- **Файлы:** copter/CopterController.h · **Тест:** test_copter_sim · **UI:** «Моторы»

### [ ] 094 · ESC passthrough (BLHeli 4-way)
- **BF:** passthrough для BLHeliSuite/ESC-Configurator через USB полётника
- **Что:** протокол 4-way-if, однопроводной UART на пинах моторов; требует MSP (пункт 158) или свой вход.
- **Файлы:** новый hal/stm32/Stm32EscPassthrough.h · **Тест:** test_app_stm32_copter на фейке · **UI:** кнопка «Открыть ESC-Configurator»

### [ ] 095 · ESC passthrough: стенд — Стенд: да

---

## 7. ARM, failsafe, безопасность, пищалка

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

---

## 8. Пульт: сглаживание, приёмники, телеметрия, крутилки

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

---

## 9. Высота, GPS, возврат

### [ ] 128 · Удержание высоты как в BF 2025
- **BF:** режим ALTHOLD, `alt_hold_adjust_rate`, `alt_hold_deadband`, `autopilot_*` (газ висения, PID высоты)
- **Сейчас:** свой ALT_HOLD (VerticalEstimator + AltitudeController, обучение висения).
- **Файлы:** copter/AltitudeHold.h, copter/Params.h · **Тест:** test_copter_sim · **UI:** вкладка «Автопилот»

### [ ] 130 · Оценка положения и скорости (GPS + акселерометр)
- **BF:** основа POSHOLD/RESCUE
- **Файлы:** новый copter/PositionEstimator.h · **Тест:** test_copter_sim с моделью GPS (шум, задержка 0.1 с) · **UI:** «GPS»

### [ ] 131 · Компас для коптера
- **BF:** `mag_hardware`, `align_mag`, `mag_declination`, калибровка компаса
- **Файлы:** src/stm32/main_copter.cpp, sensors/mag/* · **Тест:** test_app_stm32_copter · **UI:** «Настройка платы»: калибровка компаса

### [ ] 132 · Удержание позиции
- **BF:** режим POSHOLD, `pos_hold_adjust_rate`, `pos_hold_deadband`, `pos_hold_without_mag`
- **Файлы:** новый copter/PositionHold.h · **Тест:** test_copter_sim: ветер 5 м/с — дрейф < 2 м · **UI:** «Автопилот»

---

## 10. Чёрный ящик, отладка, цикл

### [ ] 140 · Выбор полей и debug_mode
- **BF:** `blackbox_disable_*`, `blackbox_high_resolution`, `debug_mode` (GYRO_SCALED, FFT, RPM_FILTER, FEEDFORWARD, D_MAX, RC_SMOOTHING…)
- **Файлы:** telemetry/CopterBlackBox.h · **Тест:** test_app_stm32_copter · **UI:** «Чёрный ящик»

### [ ] 141 · Быстрый цикл: IMU 8 кГц по DRDY
- **Платы:** F405 (GETFUN, AOCODARC) и H743.
- **BF:** `gyro_rate`, частота гироскопа
- **Что:** ICM42688 на 8 кГц, чтение SPI по прерыванию готовности с DMA.
- **Файлы:** sensors/imu/ICM42688_Sensor.h, hal/stm32/Stm32SpiBus.h · **Тест:** test_app_stm32_copter на фейке · **UI:** «Настройка платы»: частоты

### [ ] 142 · Быстрый цикл: PID 2–4 кГц вне тика FreeRTOS
- **BF:** `pid_process_denom`
- **Что:** цикл от таймера/DRDY, консоль/барометр — в задачах ниже; модель контура пересчитывает настройку под частоту.
- **Файлы:** src/stm32/main_copter.cpp, hal/Rtos.h · **Тест:** test_app_stm32_copter; test_copter_sim: whoop на 4 кГц (уже есть) · **UI:** «Настройка платы»

### [ ] 143 · Быстрый цикл: стенд — Стенд: да
- **Что:** время цикла и джиттер на F405 (GETFUN) и Matek H743 (`status`, `tasks`).

---

## 11. OSD, VTX, LED, камера

### [ ] 145 · OSD: драйвер AT7456E (MAX7456) на Matek H743
- **Сейчас:** драйвер и элементы уже есть с квадрика (PLAN_QUAD7 Q8: copter/Osd.h, hal/Max7456.h, F405); осталось — H743 и пины по заголовку платы.
- **BF:** `osd_*`, `max7456_*`
- **Что:** SPI-драйвер, видеостандарт, вывод символов, загрузка шрифта.
- **Файлы:** новый hal/stm32/Max7456.h · **Тест:** test_app_stm32_copter на регистровом эмуляторе · **UI:** вкладка «OSD»

### [ ] 146 · OSD: шрифт и загрузка
- **BF:** `font` / загрузка шрифта MCM
- **Файлы:** hal/stm32/Max7456.h · **Тест:** эмулятор · **UI:** «OSD»: выбор шрифта

### [~] 147 · OSD: элементы полёта
- **BF:** `osd_vbat_pos`, `osd_rssi_pos`, `osd_flymode_pos`, `osd_throttle_pos`, `osd_crosshairs_pos`, `osd_ah_pos`, `osd_ah_sbar_pos`, `osd_tim_1_pos`, `osd_warnings_pos`, `osd_link_quality_pos`
- **Файлы:** новый telemetry/Osd.h · **Тест:** test_app_stm32_copter: экран в буфере · **UI:** «OSD»: перетаскивание элементов по экрану
- **Ход:** Сделано (878faaf): osd_*_pos как в BF (x + 32·y, 0x0800, профили 2–3 = показан) для vbat, flymode, throttle, crosshairs, tim_2, warnings, link_quality; Studio — вкладка «OSD». Осталось: ah/ah_sbar (символы шрифта проверить на железе), tim_1, rssi

### [~] 149 · OSD: питание и моторы
- **BF:** `osd_current_pos`, `osd_mah_drawn_pos`, `osd_avg_cell_voltage_pos`, `osd_esc_tmp_pos`, `osd_esc_rpm_pos`, `osd_power_pos`
- **Файлы:** telemetry/Osd.h · **Тест:** test_app_stm32_copter · **UI:** «OSD»
- **Ход:** Сделано (878faaf): current, mah_drawn, avg_cell_voltage, power. Осталось: esc_tmp, esc_rpm — после телеметрии регуляторов (091/092)

### [ ] 151 · OSD: меню настроек стиками (CMS)
- **BF:** CMS
- **Что:** PID, расходы, фильтры, VTX из очков без компьютера.
- **Файлы:** новый telemetry/OsdMenu.h · **Тест:** test_app_stm32_copter · **UI:** —

### [ ] 152 · OSD: стенд — Стенд: да

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

---

## 12. MSP и совместимость

### [ ] 188 · Настройка с пульта: MSP через CRSF и Lua-скрипты Betaflight
- **BF:** MSP over CRSF (ELRS/Crossfire) и SmartPort; betaflight-tx-lua-scripts на EdgeTX (PID, расходы, VTX, фильтры, ползунки)
- **Что:** MSP-запросы приходят кадрами CRSF 0x7A/0x7B через приёмник, ответ — телеметрией; тот же разбор, что по USB (telemetry/CopterMsp.h). Lua-скрипты BF на пульте пользователя (RadioMaster TX12, EdgeTX) работают без правок. Позже — свой Lua-скрипт с нашими страницами (GPS Rescue, OSD).
- **Файлы:** telemetry/CopterMsp.h, copter/CrsfTelemetry.h, rc/RcReceiver.h
- **Тест:** test_copter: кадры MSP в CRSF (запрос, ответ кусками, CRC), чтение и запись PID с «пульта»
- **UI:** Studio «Приёмник»: «Настройка с пульта» — кнопка скачать Lua-скрипты на SD пульта и картинка, где их найти в EdgeTX

### [ ] 161 · MSP для ELRS Backpack и очков
- **BF:** MSP over CRSF, DisplayPort
- **Файлы:** telemetry/Msp.h · **Тест:** test_telemetry · **UI:** —

### [ ] 162 · MAVLink для коптера
- **BF:** — (у нас есть для самолёта)
- **Что:** HEARTBEAT типа QUADROTOR, ATTITUDE, позиция, режимы — QGroundControl.
- **Файлы:** telemetry/MavlinkTelemetry.h (образец) · **Тест:** test_mavlink · **UI:** —

---

## 13. Итог — релиз 1.0

Релиз 1.0 = 183 выполнен на 7" F405 + разделы 0–12. Что нельзя проверить своим железом (GHST, SRXL2, FPort, SmartPort, iBUS-телеметрия, трикоптер, коллекторные моторы), — на фейках, в COPTER.md с пометкой «не проверено в полёте».

### [ ] 171 · Полная проверка: все сборки × все физики со всеми функциями
- **Что:** сводная таблица в COPTER.md; полный прогон тестов; покрытие gcovr.

### [ ] D15 · Переводы: релиз 1.0 (11 языков)
- **Что:** COPTER.md, README, дорожная карта — к релизу 1.0.

---

## 14. После 1.0: режимы и мелочи BF

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

---

## 15. После 1.0: приложение и телеметрия регуляторов

Пока работает конфигуратор BF (158–160), вкладки Studio — удобная надстройка; делаются после 1.0. Сюда же — телеметрия регулятора (EDT, UART).

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

### [ ] 053 · Studio: вкладка «Rates» — Репо: studio
- **BF:** вкладка Rates конфигуратора
- **Что:** график кривой по осям, выбор типа, профили, живая точка стика, 3D-модель коптера крутится с расходом.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 071 · Studio: вкладка «Фильтры» — Репо: studio
- **Что:** АЧХ/ФЧХ цепочки (ФНЧ, режекторы, задержки) из модели, живой спектр гироскопа (060), ползунки, запас контура.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 072 · Studio: вкладка «Настройка платы» — Репо: studio
- **Что:** 3D-коптер по живой ориентации, калибровки (гироскоп, акселерометр, 3 позы), поворот платы.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 096 · Studio: вкладка «Моторы» — Репо: studio
- **Что:** схема рамы, протокол, холостые, ползунки моторов с защитой, обороты/температура, направление кликом.
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 110 · Studio: вкладки «Безопасность», «Failsafe», «Режимы», «Пищалка» — Репо: studio
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 123 · Studio: вкладки «Приёмник» и «Подстройки» — Репо: studio
- **Файлы:** ../openplane-studio/src
- **UI:** сам пункт

### [ ] 127 · Studio: вкладка «Питание» — Репо: studio

### [ ] 137 · Studio: вкладки «Автопилот», «GPS», «Возврат» — Репо: studio
- **Что:** карта с домом и траекторией, профиль высоты возврата.
- **UI:** сам пункт

### [~] 157 · Studio: вкладки «OSD», «Видео», «LED» — Репо: studio
- **Что:** редактор OSD перетаскиванием, предпросмотр шрифта.
- **UI:** сам пункт
- **Ход:** Сделано (studio 2ddd952): вкладка «OSD» — экран очков 30×16/13, перетаскивание, живые значения, статистика и тревоги. «Видео» уже была (VtxTab). Осталось: «LED»

### [ ] 170 · Studio: вкладки «Сервы» и «Автонастройка» — Репо: studio

### [ ] 091 · Extended DShot telemetry
- **BF:** `dshot_edt`
- **Что:** температура, напряжение, ток ESC.
- **Файлы:** hal/stm32/Stm32Dshot.h · **Тест:** test_copter (декодер) · **UI:** «Моторы»

### [ ] 092 · ESC-телеметрия по UART (KISS/BLHeli32)
- **BF:** `esc_sensor` (feature ESC_SENSOR)
- **Файлы:** новый sensors/EscSensor.h · **Тест:** test_app_stm32_copter · **UI:** «Моторы»

---

## 16. После 1.0: сверх Betaflight — просьбы пользователя

### [ ] 189 · Исследование: адаптивный фильтр гироскопа (Калман)
- **BF:** нет (есть в EmuFlight, IMUF)
- **Что:** фильтр Калмана по оси гироскопа (шум оценивается на ходу) против цепочки BF (RPM + динамические режекторы + ФНЧ): задержка, шум D, запас контура — на симуляторе с моделью вибраций и на эталонных журналах (181). Включать, только если выигрыш виден на обоих.
- **Файлы:** copter/GyroFilter.h, новый copter/GyroKalman.h
- **Тест:** test_copter_sim: тот же шум — меньше задержка при том же шуме D · **UI:** «Фильтры»: выбор цепочки и график АЧХ

### [ ] 173 · Телеметрия на телефон и в браузер
- **BF:** нет (BF шлёт телеметрию только на пульт; у ExpressLRS есть TX Backpack с Wi-Fi и MAVLink)
- **Что:** коптер → пульт (CRSF-телеметрия, 118) → телефон: через ELRS TX Backpack (Wi-Fi, MAVLink по UDP) или свой мост на ESP32 у пульта — читает телеметрию с модуля и раздаёт веб-страницу (WebSocket): батарея, режим, высота, скорость, GPS и карта, LQ/RSSI, предупреждения. FS-i6 телеметрию наружу не отдаёт — первым делом спросить пользователя про пульт/модуль и выбрать связку, решение — в «Итог». Мост — отдельный репо (как прошивка RC-джойстика).
- **Сейчас:** CRSF-телеметрия на пульт (copter/CrsfTelemetry.h), протокол приложения telemetry/CopterLink.h; на телефон — ничего.
- **Файлы:** отдельный репо моста, telemetry/CopterLink.h
- **Тест:** test_telemetry: кадры CRSF → данные страницы; страница — на записанных кадрах; на железе — с пользователем
- **UI:** веб-страница (PWA) на телефоне; тот же экран в Studio

### [ ] 174 · Режимы и настройки с телефона
- **BF:** настройки по MSP через ELRS (MSP-over-CRSF) — механизм; смены режимов с телефона в BF нет
- **Что:** через мост 173: смена режима и профиля, запуск GPS Rescue, чтение/запись настроек (реестр 001). Безопасность: ARM с телефона — никогда; команды режима принимаются, только пока включён режим-разрешение на тумблере (новый aux «PHONE»); мост пропал — всё как по тумблерам.
- **Файлы:** telemetry/CopterLink.h, copter/RcSetup.h, src/stm32/main_copter.cpp
- **Тест:** test_app_stm32_copter: команда моста меняет режим только при разрешающем тумблере; ARM — отказ; пропал мост — режим с тумблеров
- **UI:** кнопки режимов на веб-странице 173

### [ ] 175 · Follow Me — коптер летит за пилотом
- **BF:** нет (в BF — только GPS Rescue и удержание позиции; Follow Me есть в INAV и ArduPilot)
- **Что:** цель — координаты телефона пилота → мост 173 → коптер. Держит дистанцию, высоту и сторону, нос на пилота; включается тумблером (aux «FOLLOW ME»). Защиты: свежих координат цели нет > 2 с — висит на месте, > 10 с — GPS Rescue; предел скорости, минимальная высота, геозабор от точки ARM. iPhone: Safari отдаёт GPS странице только по HTTPS — нужен сертификат на мосту или приложение; выбрать в пункте. Нужны 129–132 (GPS, оценка положения, удержание позиции) и 173.
- **Файлы:** новый copter/FollowMe.h, copter/CopterController.h, copter/RcSetup.h
- **Тест:** test_copter_sim: цель идёт 5 м/с по кругу — дистанция 10 ± 3 м, нос на цель ± 15°; цель пропала — зависает, потом домой; полёт в поле — с пользователем
- **UI:** «Автопилот → Follow Me»: дистанция, высота, сторона, предел скорости

### [ ] 176 · Мелодия моторами по тумблеру и свой звук при включении
- **BF:** нет; ближайшее — DShot-маяк (109, пять фиксированных тонов) и стартовая мелодия регуляторов Bluejay (пишется в регулятор через passthrough, 094)
- **Что:** самый последний пункт, по желанию пользователя (может и не делаться). Загрузить мелодию (RTTTL) через CLI/Studio и играть по тумблеру — только в DISARM: пищалкой полётника (любые ноты) и/или моторами (DShot-тоны — грубо, 5 нот). Свой звук при включении — записать стартовую мелодию Bluejay в регуляторы через passthrough.
- **Файлы:** новый copter/Melody.h, copter/Beeper.h, hal/stm32/Stm32EscPassthrough.h
- **Тест:** test_copter: разбор RTTTL; test_app_stm32_copter: тумблер в DISARM — играет, в ARM — нет
- **UI:** «Пищалка»: загрузить мелодию, «послушать»
