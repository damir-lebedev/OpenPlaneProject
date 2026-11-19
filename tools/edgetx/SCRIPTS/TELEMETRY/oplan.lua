-- OpenPlane: экран телеметрии модели (Model → Telemetry → Screen → Script: oplan).
-- В фоне раз в 10 с сообщает прошивке имя модели пульта (MSP через CRSF,
-- ../OPLANE/msp.lua): Studio в «Перед полётом» сверяет его с именем коптера —
-- ловит «включил не ту модель». На экране — обе строки.

local msp = nil
local myModel, craft = "", nil
local last = -1000

local function init()
  msp = loadScript("/SCRIPTS/OPLANE/msp.lua")()
end

local function background()
  local rssi = getRSSI and getRSSI() or 0
  if rssi > 0 and not msp.busy() and getTime() - last > 1000 then
    last = getTime()
    myModel = msp.modelName()
    msp.sendModel(myModel, function(name) craft = name end)
  end
  msp.poll()
end

local function run(event)
  background()
  local small = LCD_W <= 200 and SMLSIZE or 0
  local lh = LCD_W <= 200 and 10 or 26
  lcd.clear()
  lcd.drawText(2, 2, "OpenPlane", small + INVERS)
  lcd.drawText(2, 2 + lh, "Model: " .. myModel, small)
  lcd.drawText(2, 2 + 2 * lh, "Craft: " .. (craft or "-"), small)
  if craft and craft ~= "" then
    local same = string.upper(craft) == string.upper(myModel)
    lcd.drawText(2, 2 + 3 * lh, same and "OK: same name" or "Name differs!", small + (same and 0 or BLINK))
  end
  return 0
end

return { init = init, background = background, run = run }
