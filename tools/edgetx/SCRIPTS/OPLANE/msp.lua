-- OpenPlane: MSP через CRSF для EdgeTX — общая часть скриптов OpenPlane.lua
-- (Tools) и oplan.lua (Telemetry). Свой код по открытому описанию кадров
-- (include/copter/CrsfMsp.h), не из Lua-скриптов Betaflight.
--
-- Запрос: кадр CRSF 0x7A {куда 0xC8, откуда 0xEA, состояние, кусок...};
-- состояние: номер куска 0..15, 0x10 — первый, 0x40 — MSP v2. Первый кусок
-- v2: флаг, команда u16, размер u16, данные (младший байт первым).
-- Ответ — кадры 0x7B так же, бит 0x80 состояния — ошибка.
--
-- Своих команд прошивки (MSP2, telemetry/CopterMsp.h):
--   0x4F00 настройка по имени -> тип, знаков, значение, мин, макс (i32), имена/текст
--   0x4F01 значение i32 + имя -> то же, что 0x4F00 (новое значение)
--   0x4F02 имя модели пульта -> имя коптера (craft_name)
--   250    MSP_EEPROM_WRITE — сохранить
-- Без битовых операций: EdgeTX — Lua 5.2.

local REQ, RESP = 0x7A, 0x7B
local FC, RADIO = 0xC8, 0xEA
local CHUNK = 7        -- байт запроса в кадре после состояния — как шлют Lua-скрипты BF
local TIMEOUT = 150    -- 1.5 с (getTime — сотые доли секунды)
local TRIES = 3

local M = {}
local txSeq = 0
local cur = nil        -- запрос в работе: cmd, body, at, sent, tries, cb
local queue = {}
local rx = nil         -- собираемый ответ: cmd, size, data, seq, err

local function floor(x) return math.floor(x) end

function M.bytes(s)
  local t = {}
  for i = 1, #s do t[i] = string.byte(s, i) end
  return t
end

function M.putI32(t, v)
  if v < 0 then v = v + 4294967296 end
  for _ = 1, 4 do
    t[#t + 1] = v % 256
    v = floor(v / 256)
  end
  return t
end

function M.i32(d, at)
  local v = d[at] + d[at + 1] * 256 + d[at + 2] * 65536 + d[at + 3] * 16777216
  if v >= 2147483648 then v = v - 4294967296 end
  return v
end

function M.text(d, from)
  local s = {}
  for i = from, #d do s[#s + 1] = string.char(d[i]) end
  return table.concat(s)
end

-- cb(ok, data, timedOut): data — байты ответа без заголовка MSP.
function M.request(cmd, data, cb)
  queue[#queue + 1] = { cmd = cmd, data = data or {}, cb = cb }
end

function M.busy() return cur ~= nil or #queue > 0 end

function M.clear()
  queue = {}
  cur = nil
  rx = nil
end

local function start(r)
  local n = #r.data
  local body = { 0, r.cmd % 256, floor(r.cmd / 256), n % 256, floor(n / 256) }
  for i = 1, n do body[#body + 1] = r.data[i] end
  r.body = body
  r.at = 1
  r.sent = nil
  r.tries = (r.tries or 0) + 1
  cur = r
end

-- По куску за вызов: очередь CRSF к модулю короткая.
local function pushNext()
  local frame = { FC, RADIO, txSeq + 0x40 + (cur.at == 1 and 0x10 or 0) }
  local last = math.min(cur.at + CHUNK - 1, #cur.body)
  for i = cur.at, last do frame[#frame + 1] = cur.body[i] end
  if crossfireTelemetryPush(REQ, frame) then
    txSeq = (txSeq + 1) % 16
    cur.at = last + 1
    if cur.at > #cur.body then cur.sent = getTime() end
  end
end

local function finish()
  local r = rx
  rx = nil
  if not cur or not cur.sent or r.cmd ~= cur.cmd then return end
  local cb = cur.cb
  cur = nil
  local data = {}
  for i = 1, r.size do data[i] = r.data[i] end
  if cb then cb(not r.err, data, false) end
end

local function receive()
  for _ = 1, 8 do
    local cmd, d = crossfireTelemetryPop()
    if cmd == nil then return end
    -- d: куда (0xEA), откуда (0xC8), состояние, кусок...
    if cmd == RESP and d and #d >= 3 and d[2] == FC then
      local st = d[3]
      local seq = st % 16
      local from = nil
      if floor(st / 16) % 2 == 1 then
        if #d >= 8 then
          rx = { cmd = d[5] + d[6] * 256, size = d[7] + d[8] * 256, data = {}, seq = seq, err = st >= 128 }
          from = 9
        else
          rx = nil
        end
      elseif rx and seq == (rx.seq + 1) % 16 then
        rx.seq = seq
        from = 4
      else
        rx = nil   -- кусок пропал: ждём повтора по тайм-ауту
      end
      if from then
        for i = from, #d do rx.data[#rx.data + 1] = d[i] end
        if #rx.data >= rx.size then finish() end
      end
    end
  end
end

-- Звать из run/background каждый раз.
function M.poll()
  receive()
  if not cur then
    if #queue == 0 then return end
    start(table.remove(queue, 1))
  end
  if not cur.sent then
    pushNext()
  elseif getTime() - cur.sent > TIMEOUT then
    if cur.tries >= TRIES then
      local cb = cur.cb
      cur = nil
      if cb then cb(false, nil, true) end
    else
      start(cur)
    end
  end
end

-- Настройка: { type = 0 INT / 1 FLOAT / 2 ENUM / 3 STRING, dec, value, min, max,
-- opts = { имена ENUM }, text = STRING } — value/min/max в 10^dec долях.
local function parseParam(d)
  if not d or #d < 14 then return nil end
  local p = { type = d[1], dec = d[2], value = M.i32(d, 3), min = M.i32(d, 7), max = M.i32(d, 11) }
  local extra = M.text(d, 15)
  if p.type == 2 then
    p.opts = {}
    for name in string.gmatch(extra, "[^,]+") do p.opts[#p.opts + 1] = name end
  elseif p.type == 3 then
    p.text = extra
  end
  return p
end

-- cb(param или nil, timedOut)
function M.getParam(name, cb)
  M.request(0x4F00, M.bytes(name), function(ok, d, late) cb(ok and parseParam(d) or nil, late) end)
end

function M.setParam(name, value, cb)
  local data = M.putI32({}, value)
  for _, b in ipairs(M.bytes(name)) do data[#data + 1] = b end
  M.request(0x4F01, data, function(ok, d, late) cb(ok and parseParam(d) or nil, late) end)
end

-- cb(ok) — прошивка сохранила и применила (нельзя при ARM)
function M.save(cb)
  M.request(250, {}, function(ok) cb(ok) end)
end

-- Имя модели пульта -> прошивке (строка «пульт — та модель» в Studio); cb(имя коптера или nil)
function M.sendModel(name, cb)
  M.request(0x4F02, M.bytes(string.sub(name or "", 1, 16)), function(ok, d) cb(ok and M.text(d, 1) or nil) end)
end

function M.modelName()
  local info = model and model.getInfo and model.getInfo()
  return info and info.name or ""
end

return M
