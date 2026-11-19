-- OpenPlane: «Найти коптер» с пульта EdgeTX — общая часть страницы Find
-- (Tools → OpenPlane) и экрана телеметрии oplan.lua.
--
-- Последняя точка GPS коптера приходит телеметрией CRSF (датчики GPS, Sats,
-- Alt). Прошивка после потери фикса шлёт последнюю точку с фиксом, спутники —
-- сколько видно сейчас: мало спутников = «фикс пропал, коптер лежит здесь».
-- Точка пишется на SD (lastgps.txt рядом с этим файлом) — переживает потерю
-- связи и выключение пульта; Studio читает тот же файл («Найти упавший»).
--
-- Где искать: от точки старта (первый фикс за сессию) — метры и сторона
-- света; у пульта со своим GPS (getTxGPS) — метры от пилота и стрелка «куда
-- идти» по направлению шага. Писк: чем ближе (по GPS пульта) или чем сильнее
-- сигнал приёмника (без GPS пульта) — тем чаще и выше.
-- Без битовых операций: EdgeTX — Lua 5.2.

local FILE = "/SCRIPTS/OPLANE/lastgps.txt"
local FIX_SATS = 5         -- как FIX_SATS в Studio (core/findLost.ts)
local SAVE_MOVE_M = 3      -- писать на SD, когда точка ушла дальше...
local SAVE_EVERY = 500     -- ...но не чаще раза в 5 с (getTime — сотые доли секунды)
local STEP_M = 4           -- шаг пилота, по которому видно направление ходьбы
local EARTH_R = 6371000
local RAD = 3.14159265358979 / 180

local M = {}
M.FILE = FILE

-- last: { lat, lon, sats, alt, time = "ГГГГ-ММ-ДД чч:мм:сс", at = getTime() или nil (из файла),
--         fixLost, model }
local last = nil
local start = nil          -- первый фикс за сессию: { lat, lon }
local saved = { at = -100000, lat = 0, lon = 0, fixLost = false }
local pilot = nil          -- GPS пульта: { lat, lon, sats }
local stepFrom = nil       -- откуда считаем шаг пилота
local course = nil         -- направление ходьбы пилота, ° от севера
local beepAt = 0
M.beep = false

local function atan2(y, x)
  if math.atan2 then return math.atan2(y, x) end
  if x > 0 then return math.atan(y / x) end
  if x < 0 then return math.atan(y / x) + (y >= 0 and 180 or -180) * RAD end
  if y > 0 then return 90 * RAD end
  if y < 0 then return -90 * RAD end
  return 0
end

-- Расстояние (м) и направление (° от севера) из a в b — как distanceBearing в Studio.
function M.distanceBearing(a, b)
  local north = (b.lat - a.lat) * RAD * EARTH_R
  local east = (b.lon - a.lon) * RAD * EARTH_R * math.cos((a.lat + b.lat) / 2 * RAD)
  local m = math.sqrt(north * north + east * east)
  if m == 0 then return 0, 0 end
  return m, (atan2(east, north) / RAD + 360) % 360
end

local DIRS = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" }
function M.compass(deg)
  return DIRS[math.floor(((deg % 360) + 22.5) / 45) % 8 + 1]
end

local function now()
  local d = getDateTime and getDateTime()
  if not d then return "" end
  return string.format("%04d-%02d-%02d %02d:%02d:%02d", d.year, d.mon, d.day, d.hour, d.min, d.sec)
end

-- Текст файла (key=value построчно) и обратно — чистые функции, проверяются тестом.
function M.encode(p, s)
  local t = {
    "lat=" .. string.format("%.7f", p.lat),
    "lon=" .. string.format("%.7f", p.lon),
    "sats=" .. tostring(p.sats or -1),
    "alt=" .. tostring(math.floor((p.alt or 0) + 0.5)),
    "time=" .. (p.time or ""),
    "fixlost=" .. (p.fixLost and "1" or "0"),
    "model=" .. (p.model or ""),
  }
  if s then
    t[#t + 1] = "slat=" .. string.format("%.7f", s.lat)
    t[#t + 1] = "slon=" .. string.format("%.7f", s.lon)
  end
  return table.concat(t, "\n") .. "\n"
end

function M.decode(text)
  local v = {}
  for k, x in string.gmatch(text or "", "(%w+)=([^\r\n]*)") do v[k] = x end
  local lat, lon = tonumber(v.lat), tonumber(v.lon)
  if not lat or not lon or (lat == 0 and lon == 0) then return nil end
  local p = { lat = lat, lon = lon, sats = tonumber(v.sats), alt = tonumber(v.alt) or 0,
              time = v.time or "", fixLost = v.fixlost == "1", model = v.model or "" }
  if p.sats and p.sats < 0 then p.sats = nil end
  local slat, slon = tonumber(v.slat), tonumber(v.slon)
  return p, (slat and slon) and { lat = slat, lon = slon } or nil
end

local function load()
  local f = io.open(FILE, "r")
  if not f then return end
  local text = io.read(f, 512)
  io.close(f)
  local p, s = M.decode(text)
  if p then
    last = p
    -- старт из файла — только для подсказки «от старта», новый фикс его заменит
    start = s
    saved = { at = -100000, lat = p.lat, lon = p.lon, fixLost = p.fixLost }
  end
end

local function save()
  local f = io.open(FILE, "w")
  if not f then return end
  io.write(f, M.encode(last, start))
  io.close(f)
  saved = { at = getTime(), lat = last.lat, lon = last.lon, fixLost = last.fixLost }
end

local function num(name)
  local v = getValue(name)
  return type(v) == "number" and v or nil
end

local function modelName()
  local info = model and model.getInfo and model.getInfo()
  return info and info.name or ""
end

local session = false      -- в этой сессии был фикс (старт из файла заменён)

-- Звать часто (background/run): читает телеметрию и GPS пульта, пишет точку.
function M.update()
  if not last and not M.loaded then
    M.loaded = true
    load()
  end
  local tx = getTxGPS and getTxGPS()
  if tx and tx.fix and tx.lat and tx.lon and not (tx.lat == 0 and tx.lon == 0) then
    pilot = { lat = tx.lat, lon = tx.lon, sats = tx.numsat }
    if not stepFrom then stepFrom = pilot end
    local m, deg = M.distanceBearing(stepFrom, pilot)
    if m >= STEP_M then
      course = deg
      stepFrom = pilot
    end
  else
    pilot, stepFrom, course = nil, nil, nil
  end

  local link = getRSSI and getRSSI() or 0
  if link <= 0 then return end
  local g = getValue("GPS")
  if type(g) ~= "table" or not g.lat or not g.lon or (g.lat == 0 and g.lon == 0) then return end
  local sats = num("Sats")
  local fixed = sats == nil or sats >= FIX_SATS
  if fixed and not session then
    session = true
    start = { lat = g.lat, lon = g.lon }
  end
  last = { lat = g.lat, lon = g.lon, sats = sats, alt = num("Alt") or 0, time = now(),
           at = getTime(), fixLost = not fixed, model = modelName() }
  local moved = M.distanceBearing(saved, last)
  if (moved >= SAVE_MOVE_M or last.fixLost ~= saved.fixLost) and getTime() - saved.at >= SAVE_EVERY then
    save()
  end
end

-- Сила сигнала 0..1: 1RSS (дБм, ELRS/Crossfire) или getRSSI.
function M.signal()
  local dbm = num("1RSS")
  if dbm and dbm < 0 then return math.max(0, math.min(1, (dbm + 110) / 70)), dbm end
  local r = getRSSI and getRSSI() or 0
  if r <= 0 then return nil end
  return math.max(0, math.min(1, r / 100)), nil
end

-- Писк «горячо-холодно»: период (сотые доли секунды) и частота, Гц. nil — молчать.
function M.beepPlan(distM, sig)
  if distM then
    return math.max(15, math.min(200, distM * 2)), distM < 10 and 2000 or 1400
  end
  if sig then return 200 - 170 * sig, 500 + 1500 * sig end
  return nil
end

local function tick(dist, sig)
  if not M.beep or not playTone then return end
  local period, freq = M.beepPlan(dist, sig)
  if not period or getTime() < beepAt then return end
  beepAt = getTime() + period
  playTone(freq, 60, 0, PLAY_NOW or 0)
end

function M.age()
  if not last then return nil end
  if last.at then
    local s = math.floor((getTime() - last.at) / 100)
    if s < 60 then return s .. "s ago" end
    if s < 3600 then return math.floor(s / 60) .. "m ago" end
    return math.floor(s / 3600) .. "h ago"
  end
  return string.sub(last.time or "", 6, 16)   -- «ММ-ДД чч:мм» из файла
end

function M.last() return last end

local function arrow(cx, cy, r, deg, north)
  if lcd.drawCircle then lcd.drawCircle(cx, cy, r, 0) end
  local a = deg * RAD
  local x, y = cx + r * math.sin(a), cy - r * math.cos(a)
  lcd.drawLine(cx, cy, math.floor(x + 0.5), math.floor(y + 0.5), SOLID, 0)
  for _, side in ipairs({ -150, 150 }) do
    local b = (deg + side) * RAD
    lcd.drawLine(math.floor(x + 0.5), math.floor(y + 0.5),
      math.floor(x + r * 0.45 * math.sin(b) + 0.5), math.floor(y - r * 0.45 * math.cos(b) + 0.5), SOLID, 0)
  end
  if north then lcd.drawText(cx - 2, cy - r - 1, "N", SMLSIZE) end
end

-- Страница: строки y0.. с шагом lh, шрифт font. Возвращает текст подсказки.
function M.draw(y0, lh, font)
  local w = LCD_W
  local sig, dbm = M.signal()
  if not last then
    lcd.drawText(2, y0, "No GPS point yet", font)
    lcd.drawText(2, y0 + lh, "Fly with GPS fix once:", font)
    lcd.drawText(2, y0 + 2 * lh, "point is kept on SD", font)
    tick(nil, sig)
    return
  end
  local y = y0
  lcd.drawText(2, y, string.format("%.6f %.6f", last.lat, last.lon), font)
  y = y + lh
  local s = (last.sats and ("Sats " .. last.sats .. " ") or "") .. (M.age() or "")
  lcd.drawText(2, y, s, font + (last.fixLost and BLINK or 0))
  y = y + lh
  if last.fixLost then
    lcd.drawText(2, y, "Fix lost: it is HERE", font + INVERS)
    y = y + lh
  end
  local dist, deg, rel = nil, nil, nil
  if pilot then
    dist, deg = M.distanceBearing(pilot, last)
    lcd.drawText(2, y, string.format("You: %dm %s", math.floor(dist + 0.5), M.compass(deg)), font)
    if course then rel = (deg - course + 360) % 360 end
  elseif start then
    local m, d = M.distanceBearing(start, last)
    lcd.drawText(2, y, string.format("Start: %dm %s", math.floor(m + 0.5), M.compass(d)), font)
  end
  y = y + lh
  local link = dbm and (dbm .. "dBm") or (sig and (math.floor(sig * 100) .. "%") or "no link")
  lcd.drawText(2, y, "Link " .. link .. "  Beep " .. (M.beep and "ON" or "OFF"), font)
  if last.model ~= "" and last.model ~= modelName() then
    lcd.drawText(2, y + lh, "Point of " .. last.model, font + BLINK)
  end
  local r = lh >= 20 and 26 or math.floor(lh * 1.5)
  if deg then
    -- стрелка: по ходьбе (вверх — прямо) или от севера, пока пилот стоит
    arrow(w - r - 4, LCD_H - r - 4, r, rel or deg, rel == nil)
  end
  tick(dist, sig)
end

return M
