# Lua-скрипт OpenPlane для EdgeTX

Настройка коптера на прошивке OpenPlane с пульта через приёмник ExpressLRS /
Crossfire (MSP через CRSF). Подробно — `docs/COPTER.md`, «Настройка с пульта».

Установка: OpenPlane Studio → коптер → «Приёмник» → «Настройка с пульта» →
«Скрипт OpenPlane на SD» (копия из исходников прошивки), или вручную: папку
`SCRIPTS` скопировать в корень SD-карты пульта (пульт по USB как накопитель). На полётнике: `serialrx_provider = CRSF`, `crsf_telemetry = ON`.

| Файл | Что |
|---|---|
| `SCRIPTS/TOOLS/OpenPlane.lua` | SYS → Tools → OpenPlane: GPS Rescue, Training, OSD, Radio, Find |
| `SCRIPTS/TELEMETRY/oplan.lua` | экран телеметрии модели: в фоне шлёт имя модели пульта (для «Перед полётом» в Studio) и запоминает точку GPS |
| `SCRIPTS/OPLANE/find.lua` | поиск упавшего: последняя точка GPS на SD (`lastgps.txt`), метры, стрелка, писк |
| `SCRIPTS/OPLANE/msp.lua` | общая часть: MSP v2 через CRSF, свои команды `0x4F00`–`0x4F02` |

Кнопки: колесо или +/− — строка, ENTER — править, ENTER ещё раз — записать
в полётник, EXIT — отменить; PAGE — страница. Записанное держится в памяти
полётника, ARM до сохранения запрещён — на выходе скрипт спросит «Save»
(ENTER). При ARM настройки не меняются.

Страница Find: координаты последней точки, «Fix lost: it is HERE», если
GPS коптера потерял фикс, метры от старта или от пилота (пульт со своим
GPS — со стрелкой), ENTER — писк «горячо-холодно». Точку пишет экран
телеметрии `oplan` в фоне — поставьте его на экран модели.

Код свой (не из Lua-скриптов Betaflight), Lua 5.2 без битовых операций.
Тесты на ПК с заглушками API EdgeTX: `python tools/test_edgetx_lua.py`
(`pip install lupa`).
