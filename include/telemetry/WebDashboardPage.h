#pragma once
#include <Arduino.h>

// ============================================================
// WEB DASHBOARD PAGE — HTML/JS дашборда (GET /)
//
// Статическая страница: всё динамическое строится в браузере по
// JSON из GET /api/status (опрос каждые 200 мс). Строки RC-каналов,
// выходов и датчиков создаются по ключам JSON — новый выход или
// датчик появится на странице без правки этого файла (достаточно
// подписи в OUTPUT_NAMES/SENSORS, иначе покажется ключ).
// ============================================================

namespace WebDashboardPage
{
    static const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="ru"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenPlane Debug</title>
<style>
body{font-family:Arial,sans-serif;background:#667eea;margin:0;padding:16px}
.container{max-width:900px;margin:0 auto}
h1{color:#fff;text-align:center;margin:12px 0 20px}
.card{background:#fff;border-radius:10px;padding:16px 20px;margin:10px 0;box-shadow:0 4px 8px rgba(0,0,0,.2)}
.card h2{color:#667eea;border-bottom:2px solid #667eea;padding-bottom:8px;margin:0 0 8px;font-size:18px}
.row{display:flex;align-items:center;gap:10px;padding:5px 0;flex-wrap:wrap}
.label{font-weight:bold;color:#333;min-width:150px}
.value{color:#667eea;font-weight:bold}
.badge{padding:3px 10px;border-radius:12px;font-size:12px;font-weight:bold;color:#fff;background:#999}
.badge:empty{display:none}
.ok{background:#4CAF50}.bad{background:#f44336}
.bar{flex:1;min-width:80px;height:10px;background:#eee;border-radius:5px;overflow:hidden}
.fill{height:100%;background:#667eea;width:50%}
button{background:#667eea;color:#fff;border:none;padding:8px 16px;margin:4px;border-radius:5px;cursor:pointer}
button:hover{background:#764ba2}
button.manual{background:#f44336}
input[type=number]{width:64px}
</style></head><body><div class="container">
<h1>OpenPlane Debug Dashboard</h1>

<div class="card"><h2>Состояние</h2><div class="row">
<span class="label">Связь</span><span class="badge" id="rx">--</span>
<span class="label">ARM</span><span class="badge" id="arm">--</span>
<span class="label">Закрылки</span><span class="value" id="flaps">--</span>
</div></div>

<div class="card"><h2>RC каналы</h2><div id="rc"></div></div>
<div class="card"><h2>Выходы</h2><div id="outputs"></div></div>
<div class="card"><h2>Датчики</h2><div id="sensors"></div></div>

<div class="card"><h2>Автопилот</h2>
<div class="row"><span class="label">Режим</span><span class="value" id="mode">--</span></div>
<div class="row"><span class="label">Цели</span><span class="value" id="target">--</span></div>
<div class="row"><span class="label">Коррекции</span><span class="value" id="corr">--</span></div>
<div class="row"><span class="label">Функции</span><span class="value" id="features">--</span></div>
<div class="row" id="modes"></div></div>

<div class="card"><h2>Навигация</h2>
<div class="row"><span class="label">GPS / дом</span><span class="badge" id="gpsState">--</span><span class="badge" id="homeState">--</span>
<span class="badge" id="fence"></span><span class="badge" id="stall"></span></div>
<div class="row"><span class="label">До дома</span><span class="value" id="home">--</span></div>
<div class="row"><span class="label">Курс / цель</span><span class="value" id="course">--</span></div>
<div class="row"><span class="label">Скорость</span><span class="value" id="speed">--</span></div>
</div>

<div class="card"><h2>PID (крен / тангаж)</h2>
<div class="row">Крен: Kp<input type="number" step="0.01" id="kpRoll"> Ki<input type="number" step="0.01" id="kiRoll"> Kd<input type="number" step="0.01" id="kdRoll"></div>
<div class="row">Тангаж: Kp<input type="number" step="0.01" id="kpPitch"> Ki<input type="number" step="0.01" id="kiPitch"> Kd<input type="number" step="0.01" id="kdPitch"></div>
<div class="row"><button onclick="applyPid()">Применить</button></div>
</div>

</div><script>
const OUTPUT_NAMES = {aileronLeft:'Элерон L', aileronRight:'Элерон R', elevator:'Руль высоты', rudder:'Руль направления', esc:'ESC (газ)', aux1:'Груз (AUX1)', aux2:'Камера (AUX2)'};
const SENSORS = [['imu','IMU',['roll','pitch','yaw']], ['baro','Барометр',['altitude','climb']], ['mag','Компас',['heading']], ['gps','GPS',['fix','numSV','lat','lon']], ['airspeed','Трубка Пито',['ias','tas','dp']]];
// Номер = AutopilotMode (autopilot/AutopilotTypes.h).
const MODES = ['Manual','Stabilize','Takeoff','Alt Hold','Acro','Cruise','Loiter','RTH','Launch','Land','Soaring','Rescue'];
const PID_FIELDS = ['kpRoll','kiRoll','kdRoll','kpPitch','kiPitch','kdPitch'];
const $ = id => document.getElementById(id);

function badge(el, good, okText, badText) {
  el.textContent = good ? okText : badText;
  el.className = 'badge ' + (good ? 'ok' : 'bad');
}

// Строка создаётся один раз и дальше только обновляется.
function row(container, id, label, withBar) {
  let r = $(id);
  if (r) return r;
  r = document.createElement('div');
  r.className = 'row';
  r.id = id;
  r.innerHTML = '<span class="label">' + label + '</span>' +
    (withBar ? '<div class="bar"><div class="fill"></div></div>' : '') +
    '<span class="value"></span><span class="badge"></span>';
  container.appendChild(r);
  return r;
}

function render(s) {
  badge($('rx'), !s.failsafe, 'OK', 'LOST');
  badge($('arm'), s.armed, 'ARMED', 'DISARMED');
  $('flaps').textContent = s.flapsUs > 0 ? ('выпущены, ' + s.flapsUs + ' мкс') : 'убраны';

  s.rc.forEach((v, i) => {
    const r = row($('rc'), 'rc' + i, 'CH' + (i + 1), true);
    r.querySelector('.fill').style.width = Math.max(0, Math.min(100, (v - 1000) / 10)) + '%';
    r.querySelector('.value').textContent = v;
  });

  for (const key in s.outputs) {
    const o = s.outputs[key];
    const r = row($('outputs'), 'out-' + key, OUTPUT_NAMES[key] || key, false);
    r.querySelector('.value').textContent = o.us + ' мкс';
    badge(r.querySelector('.badge'), o.attached, 'OK', 'НЕ ПОДКЛЮЧЁН');
  }

  SENSORS.forEach(([key, label, fields]) => {
    const d = s[key];
    const r = row($('sensors'), 'sens-' + key, label, false);
    const v = r.querySelector('.value');
    const b = r.querySelector('.badge');
    if (!d.attached) { badge(b, false, '', 'НЕТ В СБОРКЕ'); v.textContent = '--'; return; }
    if (!d.available) { badge(b, false, '', 'НЕ ОТВЕЧАЕТ'); v.textContent = '--'; return; }
    badge(b, true, 'OK', '');
    v.textContent = fields.map(f => f + '=' + Number(d[f]).toFixed(2)).join('  ');
  });

  const a = s.autopilot;
  if (!a.attached) { $('mode').textContent = 'НЕТ АВТОПИЛОТА'; return; }
  $('mode').textContent = a.modeName;
  $('target').textContent = 'крен ' + a.desiredRoll.toFixed(1) + '°, тангаж ' + a.desiredPitch.toFixed(1) + '°, высота ' + a.targetAlt.toFixed(1) + ' м';
  $('corr').textContent = 'крен ' + a.rollCorr.toFixed(0) + ' мкс, тангаж ' + a.pitchCorr.toFixed(0) + ' мкс, газ ' + a.throttleCorr.toFixed(0) + '%';
  $('features').textContent = a.features.length ? a.features.join(', ') : 'нет';
  const n = a.nav;
  badge($('gpsState'), n.gps, 'GPS OK', 'НЕТ GPS');
  badge($('homeState'), n.home, 'ДОМ', 'ДОМА НЕТ');
  $('fence').textContent = n.fence ? 'ГЕОЗАБОР' : ''; $('fence').className = 'badge bad';
  $('stall').textContent = n.stall ? 'МАЛАЯ СКОРОСТЬ' : ''; $('stall').className = 'badge bad';
  $('home').textContent = n.homeDist < 0 ? '--' : (n.homeDist.toFixed(0) + ' м, пеленг ' + n.homeBearing.toFixed(0) + '°');
  $('course').textContent = n.course.toFixed(0) + '° → ' + n.targetCourse.toFixed(0) + '°';
  $('speed').textContent = n.speed.toFixed(1) + ' м/с';
  PID_FIELDS.forEach(f => { const el = $(f); if (!el.dataset.touched) el.value = a[f]; });
}

async function poll() {
  try { render(await (await fetch('/api/status')).json()); } catch (e) { console.error(e); }
}

async function post(url, body) {
  try {
    await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  } catch (e) { console.error(e); }
}

function setMode(m) { post('/api/setmode', {mode:m}); }

function applyPid() {
  const body = {};
  PID_FIELDS.forEach(f => { body[f] = parseFloat($(f).value) || 0; });
  post('/api/setpid', body);
}

// Поле, которое пользователь начал править, опрос больше не перезаписывает.
PID_FIELDS.forEach(f => $(f).addEventListener('input', () => { $(f).dataset.touched = '1'; }));

MODES.forEach((name, m) => {
  const b = document.createElement('button');
  b.textContent = name;
  if (m === 0) b.className = 'manual';
  b.onclick = () => setMode(m);
  $('modes').appendChild(b);
});

setInterval(poll, 200);
poll();
</script></body></html>)HTML";
}
