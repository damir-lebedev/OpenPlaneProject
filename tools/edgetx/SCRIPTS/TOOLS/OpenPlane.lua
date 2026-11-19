-- OpenPlane: настройки коптера с пульта EdgeTX (System → Tools → OpenPlane).
-- Страницы: возврат домой, учебные ограничения, OSD, пульт. Связь — MSP через
-- CRSF (../OPLANE/msp.lua), настройки — по именам BF (как в CLI). Подписи
-- английские: шрифты пульта не у всех с кириллицей.
--
-- Колесо/+- — строка, ENTER — править (ENTER ещё раз — записать, EXIT —
-- отменить), PAGE — страница, EXIT — выход (несохранённое — спросит).

local msp = nil

-- { подпись, имя настройки, единицы, step = шаг, vis = показ элемента OSD (бит 0x800) }
local PAGES = {
  { title = "GPS Rescue", rows = {
    { "Return alt", "gps_rescue_return_alt", "m" },
    { "Init climb", "gps_rescue_initial_climb", "m" },
    { "Alt mode", "gps_rescue_alt_mode" },
    { "Speed", "gps_rescue_ground_speed", "cm/s", step = 50 },
    { "Climb rate", "gps_rescue_ascend_rate", "cm/s", step = 50 },
    { "Descent dist", "gps_rescue_descent_dist", "m" },
    { "Descend rate", "gps_rescue_descend_rate", "cm/s", step = 25 },
    { "Land alt", "gps_rescue_landing_alt", "m" },
    { "Min start", "gps_rescue_min_start_dist", "m" },
    { "Min sats", "gps_rescue_min_sats" },
    { "Sanity", "gps_rescue_sanity_checks" },
    { "Arm no fix", "gps_rescue_allow_arming_without_fix" },
    { "FS procedure", "failsafe_procedure" },
  } },
  { title = "Training", rows = {
    { "Limits", "training_limits" },
    { "Max alt", "training_max_alt", "m", step = 5 },
    { "Max dist", "training_max_dist", "m", step = 10 },
    { "Max angle", "training_max_angle", "deg" },
    { "Max speed", "training_max_speed", "km/h" },
    { "OSD warning", "osd_warn_training" },
  } },
  { title = "OSD", rows = {
    { "Alt alarm", "osd_alt_alarm", "m", step = 10 },
    { "mAh alarm", "osd_cap_alarm", "mAh", step = 100 },
    { "LQ alarm", "osd_link_quality_alarm", "%" },
    { "Battery V", "osd_vbat_pos", vis = true },
    { "Cell V", "osd_avg_cell_voltage_pos", vis = true },
    { "Altitude", "osd_altitude_pos", vis = true },
    { "Home dist", "osd_home_dist_pos", vis = true },
    { "Home dir", "osd_home_dir_pos", vis = true },
    { "GPS sats", "osd_gps_sats_pos", vis = true },
    { "Fly mode", "osd_flymode_pos", vis = true },
    { "Link qual", "osd_link_quality_pos", vis = true },
    { "Crosshair", "osd_crosshairs_pos", vis = true },
    { "Warnings", "osd_warnings_pos", vis = true },
    { "Preflight", "osd_preflight_pos", vis = true },
  } },
  { title = "Radio", radio = true },
}

local VISIBLE = 2048   -- OSD_VISIBLE в osd_*_pos

local page, cursor, top = 1, 1, 1
local edit = nil       -- { row, v }
local dirty = false    -- в прошивке есть несохранённое (ARM запрещён до сохранения)
local note = ""        -- строка состояния внизу
local asking = false   -- «сохранить?» при выходе
local leaving = false
local radio = { model = "", craft = nil, asked = false }

local color = LCD_W > 200
local LH = color and 24 or 8
local FONT = color and 0 or SMLSIZE

local function ev(name) return _G[name] or -1 end
local E_NEXT, E_PREV = ev("EVT_VIRTUAL_NEXT"), ev("EVT_VIRTUAL_PREV")
local E_NEXT_R, E_PREV_R = ev("EVT_VIRTUAL_NEXT_REPT"), ev("EVT_VIRTUAL_PREV_REPT")
local E_INC, E_DEC = ev("EVT_VIRTUAL_INC"), ev("EVT_VIRTUAL_DEC")
local E_INC_R, E_DEC_R = ev("EVT_VIRTUAL_INC_REPT"), ev("EVT_VIRTUAL_DEC_REPT")
local E_ENTER, E_EXIT = ev("EVT_VIRTUAL_ENTER"), ev("EVT_VIRTUAL_EXIT")
local E_PAGE_N, E_PAGE_P = ev("EVT_VIRTUAL_NEXT_PAGE"), ev("EVT_VIRTUAL_PREV_PAGE")

local function isVisible(v) return math.floor(v / VISIBLE) % 2 == 1 end

local function load()
  msp.clear()
  edit = nil
  local p = PAGES[page]
  if p.radio then
    radio.model = msp.modelName()
    radio.craft = nil
    radio.asked = true
    msp.sendModel(radio.model, function(craft)
      radio.craft = craft or false
    end)
    return
  end
  for _, row in ipairs(p.rows) do
    row.p, row.err = nil, nil
    msp.getParam(row[2], function(param)
      row.p = param
      row.err = param == nil
    end)
  end
end

local function rowCount()
  local p = PAGES[page]
  return p.rows and #p.rows or 0
end

local function shown(row)
  if row.err then return "--" end
  local p = row.p
  if not p then return "..." end
  local v = (edit and edit.row == row) and edit.v or p.value
  if row.vis then return isVisible(v) and "ON" or "OFF" end
  if p.type == 2 then return p.opts[v - p.min + 1] or tostring(v) end
  if p.type == 3 then return p.text or "" end
  local s
  if p.dec > 0 then
    s = string.format("%." .. p.dec .. "f", v / 10 ^ p.dec)
  else
    s = string.format("%d", math.floor(v))
  end
  return row[3] and (s .. " " .. row[3]) or s
end

local function step(dir)
  local row, p = edit.row, edit.row.p
  if row.vis then
    edit.v = isVisible(edit.v) and edit.v - VISIBLE or edit.v + VISIBLE
    return
  end
  if p.type == 2 then
    edit.v = edit.v + dir
    if edit.v > p.max then edit.v = p.min end
    if edit.v < p.min then edit.v = p.max end
    return
  end
  local k = (row.step or 1) * 10 ^ p.dec
  edit.v = math.max(p.min, math.min(p.max, edit.v + dir * k))
end

local function commit()
  local row, v = edit.row, edit.v
  edit = nil
  if v == row.p.value then return end
  note = "Writing..."
  msp.setParam(row[2], v, function(param, late)
    if param then
      row.p = param
      dirty = true
      note = "Not saved: ENT on exit"
    else
      note = late and "No reply" or "Refused (armed?)"
    end
  end)
end

local function save()
  note = "Saving..."
  msp.save(function(ok)
    if ok then
      dirty = false
      note = "Saved"
    else
      note = "Save failed (armed?)"
    end
    if ok and asking then leaving = true end
  end)
end

local function title(t)
  if not color and lcd.drawScreenTitle then
    lcd.drawScreenTitle(t, page, #PAGES)
  else
    lcd.drawFilledRectangle(0, 0, LCD_W, LH, SOLID)
    lcd.drawText(4, 2, t .. "  " .. page .. "/" .. #PAGES, INVERS)
  end
end

local function drawRadio()
  local y = LH + 2
  lcd.drawText(2, y, "Model", FONT)
  lcd.drawText(LCD_W - 2, y, radio.model ~= "" and radio.model or "-", FONT + RIGHT)
  y = y + LH
  lcd.drawText(2, y, "Craft", FONT)
  local craft = radio.craft == nil and "..." or (radio.craft == false and "no reply" or radio.craft)
  lcd.drawText(LCD_W - 2, y, craft ~= "" and craft or "(empty)", FONT + RIGHT)
  y = y + LH
  if type(radio.craft) == "string" then
    local same = string.upper(radio.craft) == string.upper(radio.model) and radio.craft ~= ""
    lcd.drawText(2, y, same and "Model matches craft" or "Name differs: check model!", FONT + (same and 0 or BLINK))
  end
  y = y + LH
  lcd.drawText(2, y, "ENT: send again", FONT)
end

local function draw()
  lcd.clear()
  local p = PAGES[page]
  title(p.title .. (dirty and " *" or ""))
  if asking then
    lcd.drawText(2, LH + 4, "Save to flight controller?", FONT)
    lcd.drawText(2, LH * 2 + 6, "ENT: save   EXIT: leave", FONT)
    lcd.drawText(2, LH * 3 + 8, "(unsaved: no ARM)", FONT)
    lcd.drawText(2, LCD_H - LH, note, FONT)
    return
  end
  if p.radio then
    drawRadio()
    return
  end
  local visible = math.floor((LCD_H - 2 * LH) / LH)
  if cursor < top then top = cursor end
  if cursor >= top + visible then top = cursor - visible + 1 end
  for i = top, math.min(#p.rows, top + visible - 1) do
    local row = p.rows[i]
    local y = LH + 1 + (i - top) * LH
    local sel = i == cursor
    lcd.drawText(2, y, row[1], FONT + ((sel and not edit) and INVERS or 0))
    local editing = edit and edit.row == row
    lcd.drawText(LCD_W - 2, y, shown(row), FONT + RIGHT + (editing and (INVERS + BLINK) or 0))
  end
  lcd.drawText(2, LCD_H - LH + 1, note, FONT)
end

local function init()
  msp = loadScript("/SCRIPTS/OPLANE/msp.lua")()
  radio.model = msp.modelName()
  msp.sendModel(radio.model, function(craft) radio.craft = craft or false end)
  load()
end

local function anyOf(e, ...)
  for _, x in ipairs({ ... }) do
    if e == x then return true end
  end
  return false
end

local function run(e)
  msp.poll()
  if leaving then return 1 end
  if asking then
    if e == E_ENTER then save()
    elseif e == E_EXIT then return 1 end
    draw()
    return 0
  end
  local p = PAGES[page]
  if edit then
    if anyOf(e, E_NEXT, E_NEXT_R, E_INC, E_INC_R) then step(1)
    elseif anyOf(e, E_PREV, E_PREV_R, E_DEC, E_DEC_R) then step(-1)
    elseif e == E_ENTER then commit()
    elseif e == E_EXIT then edit = nil end
  elseif e == E_EXIT then
    if not dirty then return 1 end
    asking = true
  elseif e == E_PAGE_N or e == E_PAGE_P then
    page = page + (e == E_PAGE_N and 1 or -1)
    if page > #PAGES then page = 1 end
    if page < 1 then page = #PAGES end
    cursor, top, note = 1, 1, dirty and "Not saved: ENT on exit" or ""
    load()
  elseif anyOf(e, E_NEXT, E_NEXT_R) and cursor < rowCount() then
    cursor = cursor + 1
  elseif anyOf(e, E_PREV, E_PREV_R) and cursor > 1 then
    cursor = cursor - 1
  elseif e == E_ENTER then
    if p.radio then
      load()
    else
      local row = p.rows[cursor]
      if row.p and row.p.type ~= 3 then edit = { row = row, v = row.p.value } end
    end
  end
  draw()
  return 0
end

return { init = init, run = run }
