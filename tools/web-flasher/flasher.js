"use strict";
// OpenPlane Flasher — прошивка STM32 по USB из браузера (WebUSB / Web Serial, DfuSe) и отчёт об ошибке.
// Исходник: tools/web-flasher/, выкладка — tools/publish_web_flasher.py.
// ---------------- тексты ----------------
const EN = {
  tagline: "Flash the flight controller from your phone — one cable",
  unsupported: "This browser has no USB access. On Android open the page in <b>Chrome</b> (not inside Telegram/WhatsApp); on a computer use Chrome or Edge.",
  s1h: "Firmware", s1p: "Pick your board. If it already runs OpenPlane, the page will recognise it.",
  loading: "Loading list…",
  s2h: "Connect and flash",
  s2p: "Props off, battery unplugged. Cable from the phone into the flight controller USB. Tap the button and pick the board.",
  go: "Connect and flash", pickdfu: "Allow the bootloader (STM32 BOOTLOADER)", logbtn: "Show log",
  settingsNote: "Copter settings (PID, switches, calibrations) live in a separate flash sector — flashing keeps them. Prebuilt plane firmware uses default settings: flash a plane with its own sensors and switches from OpenPlane Studio.",
  helpH: "Not working?",
  helpL: "<li><b>The phone doesn't see the board.</b> Some flight controllers lack the USB-C resistors that tell the phone to power them. Use an OTG adapter (USB-C → USB-A) plus a USB-A → USB-C cable; enable “OTG” in Android settings if present.</li><li><b>The board won't enter the bootloader</b> (no OpenPlane yet, or Betaflight/INAV). Unplug USB, hold <b>BOOT</b>, plug in, release. Then tap “Connect and flash” and pick <b>STM32 BOOTLOADER</b>. DevEBox has no button — jumper BT0 → 3V3 and press RST.</li><li><b>Armed</b> — disarm first.</li><li><b>Windows:</b> the bootloader needs the WinUSB driver (Zadig → “STM32 BOOTLOADER” → WinUSB). Android and Linux need no drivers.</li><li><b>Flashing interrupted</b> — no harm: the STM32 bootloader is in ROM. Hold BOOT and flash again.</li>",
  howH: "How it works",
  howL: "<li>Chrome talks to the board directly via WebUSB — no apps, no drivers.</li><li>The page reads the firmware passport (<code>@id</code>), picks the matching build and sends the <code>D</code> key: the board reboots into the STM32 system bootloader.</li><li>Only the firmware sectors are erased, then written block by block, verified by reading back and started.</li>",
};
Object.assign(EN, {
  bugB: "Found a bug or something's unclear?", bugS: "Write here — it reaches the author right away.", bugBtn: "Report",
  rMsgL: "What happened", rMsgP: "What you did and what went wrong: board, step, what's on the screen",
  rNickL: "Nickname — for the testers' thank-you list", rNickP: "optional",
  rContactL: "Telegram, e-mail or phone — so we can reply", rContactP: "optional",
  rConsent: 'I give <a href="https://lebedev-visit.ru/soglasie" target="_blank" rel="noopener">consent to personal data processing</a> so that I get a reply about this report.',
  rNews: "Tell me about new OpenPlane versions and offers at this contact",
  rAttach: "Attach the flashing log and board data", rWhat: "What gets attached",
  rSend: "Send", rCancel: "Close", reportNow: "Send an error report",
  showH: "What's inside the firmware", showP: "Same firmware you just flashed — here is what it can do and how it was checked.",
  fenceH: "Geofence: a drone that won't fly away",
  fenceP: "Draw a circle and a ceiling. In any mode, even ACRO at full speed, the copter brakes near the edge, and at the edge the autopilot takes over and brings it back — sticks are ignored. GPS lost — it flies home on its own sensors or lands.",
  fenceC1: "OpenPlane Studio → Safety: radius, ceiling, what to do without GPS; next to the circle — the radar from the goggles.",
  fenceC2: "The real trajectory from a firmware test: a robot pilot pushes outward for a minute.",
  simH: "Simulator: fly before you buy",
  simP: "The same copter firmware built for a PC, with a flight model instead of sensors. Radio over USB, gamepad or keyboard; chase, FPV or ground camera; crashed — press R. Free, inside OpenPlane Studio.",
  linkStudio: "OpenPlane Studio<small>configure, flash, simulator — Windows and macOS</small>",
  linkGit: "Source code<small>github.com/damir-lebedev/OpenPlaneProject</small>",
  linkSite: "Lebedev Space<small>planes, pilot school, news</small>",
});
const RU = {};
const MSG = {
  ru: {
    custom: "Свой файл .bin…", choose: "— выберите плату —",
    noList: "Список прошивок не загрузился — можно выбрать свой .bin",
    pickFirst: "Сначала выберите прошивку", connecting: "Подключение…",
    idOk: (id) => `Плата: ${id.board}${id.vehicle === "copter" ? " (коптер)" : ""}, прошивка ${id.commit}`,
    armed: "Плата заармлена — снимите ARM и повторите", toDfu: "Перезагрузка в загрузчик…",
    waitDfu: "Ждём загрузчик STM32…", needTap: "Нажмите «Разрешить загрузчик» и выберите STM32 BOOTLOADER",
    noDfu: "Загрузчик не появился. Зажмите BOOT, переподключите USB и нажмите кнопку ещё раз.",
    chip: (c) => `Загрузчик: ${c}`, wrongChip: (a, b) => `Эта прошивка для ${a}, а плата — ${b}. Выберите другую.`,
    tooBig: "Прошивка залезает в сектор настроек — отказ.",
    erase: (i, n) => `Стирание ${i}/${n}…`, write: (p) => `Запись ${p}%`, verify: (p) => `Сверка ${p}%`,
    verifyBad: (a) => `Сверка не сошлась по адресу 0x${a.toString(16)} — прошейте ещё раз`,
    done: "Готово! Плата перезапущена с новой прошивкой ✈", fail: "Ошибка: ",
    hideLog: "Скрыть журнал", showLog: "Показать журнал", cancelled: "Выбор устройства отменён",
    mismatch: (b) => `Внимание: плата сообщила «${b}», а выбрана другая прошивка.`,
    updated: "собрано", plane: "самолёт", copter: "коптер", kb: "КБ",
    rNeedMsg: "Опишите, что случилось (хотя бы пару слов)", rNeedConsent: "Чтобы оставить контакт или ник, нужно согласие на обработку данных",
    rBadContact: "Контакт не похож на Telegram (@ник), почту или телефон", rSending: "Отправка…",
    rOk: "Отправлено — спасибо! Автор уже видит отчёт.", rFail: "Не отправилось. Скопируйте отчёт и пришлите в Telegram @od1ndva4:",
    rCopy: "Скопировать отчёт", rCopied: "Скопировано", rNothing: "(журнал пуст — прошивку ещё не запускали)",
  },
  en: {
    custom: "Custom .bin file…", choose: "— pick a board —",
    noList: "Firmware list did not load — you can pick your own .bin",
    pickFirst: "Pick the firmware first", connecting: "Connecting…",
    idOk: (id) => `Board: ${id.board}${id.vehicle === "copter" ? " (copter)" : ""}, firmware ${id.commit}`,
    armed: "The board is armed — disarm and retry", toDfu: "Rebooting into the bootloader…",
    waitDfu: "Waiting for the STM32 bootloader…", needTap: "Tap “Allow the bootloader” and pick STM32 BOOTLOADER",
    noDfu: "The bootloader did not show up. Hold BOOT, replug USB and tap the button again.",
    chip: (c) => `Bootloader: ${c}`, wrongChip: (a, b) => `This firmware is for ${a}, the board is ${b}. Pick another one.`,
    tooBig: "Firmware would overlap the settings sector — refused.",
    erase: (i, n) => `Erasing ${i}/${n}…`, write: (p) => `Writing ${p}%`, verify: (p) => `Verifying ${p}%`,
    verifyBad: (a) => `Verify mismatch at 0x${a.toString(16)} — flash again`,
    done: "Done! The board restarted with the new firmware ✈", fail: "Error: ",
    hideLog: "Hide log", showLog: "Show log", cancelled: "Device selection cancelled",
    mismatch: (b) => `Note: the board reported “${b}”, but a different firmware is selected.`,
    updated: "built", plane: "plane", copter: "copter", kb: "KB",
    rNeedMsg: "Describe what happened (a few words is fine)", rNeedConsent: "To leave a contact or a nickname, consent to data processing is required",
    rBadContact: "The contact doesn't look like Telegram (@nick), e-mail or phone", rSending: "Sending…",
    rOk: "Sent — thank you! The author already sees the report.", rFail: "Couldn't send. Copy the report and send it on Telegram @od1ndva4:",
    rCopy: "Copy the report", rCopied: "Copied", rNothing: "(log is empty — flashing hasn't been started yet)",
  },
};
let lang = (localStorage.getItem("op-lang") || (navigator.language || "ru").slice(0, 2)) === "ru" ? "ru" : "en";
const $ = (id) => document.getElementById(id);
const M = () => MSG[lang];
function applyLang() {
  document.documentElement.lang = lang;
  document.querySelectorAll("[data-t]").forEach((el) => {
    const k = el.dataset.t;
    if (!(k in RU)) RU[k] = el.innerHTML;
    el.innerHTML = lang === "en" && EN[k] ? EN[k] : RU[k];
  });
  document.querySelectorAll("[data-tp]").forEach((el) => {
    const k = el.dataset.tp;
    if (!(k in RU)) RU[k] = el.getAttribute("placeholder") || "";
    el.setAttribute("placeholder", lang === "en" && EN[k] ? EN[k] : RU[k]);
  });
  document.querySelectorAll("img[data-svg]").forEach((el) => { el.src = `img/${el.dataset.svg}${lang === "en" ? ".en" : ""}.svg`; });
  $("lang").textContent = lang === "ru" ? "EN" : "RU";
  $("logbtn").textContent = $("log").classList.contains("on") ? M().hideLog : M().showLog;
  fillList();
}
$("lang").onclick = () => { lang = lang === "ru" ? "en" : "ru"; try { localStorage.setItem("op-lang", lang); } catch (e) {} applyLang(); };

// ---------------- журнал и статус ----------------
function log(s) { const l = $("log"); l.textContent += s + "\n"; l.scrollTop = l.scrollHeight; console.log(s); }
function status(s, cls = "") { const e = $("status"); e.textContent = s; e.className = "status " + cls; log("» " + s); }
function progress(f) { $("bar").style.width = Math.round(Math.max(0, Math.min(1, f)) * 100) + "%"; }
$("logbtn").onclick = () => { const l = $("log"); l.classList.toggle("on"); $("logbtn").textContent = l.classList.contains("on") ? M().hideLog : M().showLog; };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---------------- список прошивок ----------------
let manifest = null, customBin = null;
async function loadManifest() {
  try {
    const r = await fetch("firmware/manifest.json", { cache: "no-cache" });
    manifest = await r.json();
    $("build").textContent = `${M().updated} ${manifest.date} · ${manifest.commit}`;
  } catch (e) { manifest = { firmware: [] }; status(M().noList, "warn"); }
  fillList();
}
function fillList() {
  if (!manifest) return;
  const sel = $("fw"), keep = sel.value;
  sel.innerHTML = "";
  const o0 = new Option(M().choose, ""); sel.add(o0);
  for (const f of manifest.firmware) {
    const kind = f.vehicle === "copter" ? M().copter : M().plane;
    sel.add(new Option(`${lang === "en" ? f.title_en : f.title} — ${kind}`, f.env));
  }
  sel.add(new Option(M().custom, "__file"));
  sel.value = keep;
  showInfo();
}
function selected() { return manifest && manifest.firmware.find((f) => f.env === $("fw").value); }
function showInfo() {
  const f = selected(), i = $("fwinfo");
  if ($("fw").value === "__file") { i.textContent = customBin ? `${customBin.name} · ${(customBin.data.byteLength / 1024).toFixed(0)} ${M().kb}` : ""; return; }
  i.textContent = f ? `${lang === "en" ? f.note_en : f.note} · ${(f.size / 1024).toFixed(0)} ${M().kb} · ${f.chip.toUpperCase()}` : "";
}
$("fw").onchange = () => { if ($("fw").value === "__file") $("file").click(); showInfo(); };
$("file").onchange = async () => {
  const f = $("file").files[0]; if (!f) return;
  customBin = { name: f.name, data: new Uint8Array(await f.arrayBuffer()) }; showInfo();
};

// ---------------- консоль платы: WebUSB CDC (Android) или Web Serial (ПК) ----------------
const ST_VID = 0x0483, DFU_PID = 0xdf11;
const enc = new TextEncoder(), dec = new TextDecoder();

async function cdcOverUsb(dev) {
  await dev.open();
  if (!dev.configuration) await dev.selectConfiguration(1);
  const ifs = dev.configuration.interfaces;
  const comm = ifs.find((i) => i.alternates[0].interfaceClass === 0x02);
  const data = ifs.find((i) => i.alternates[0].interfaceClass === 0x0a);
  if (!data) throw new Error("no CDC data interface");
  if (comm) { try { await dev.claimInterface(comm.interfaceNumber); } catch (e) { log("comm: " + e.message); } }
  await dev.claimInterface(data.interfaceNumber);
  const eps = data.alternates[0].endpoints;
  const epIn = eps.find((e) => e.direction === "in").endpointNumber;
  const epOut = eps.find((e) => e.direction === "out").endpointNumber;
  if (comm) {
    const idx = comm.interfaceNumber;
    const lc = new Uint8Array([0x00, 0xc2, 0x01, 0x00, 0, 0, 8]); // 115200 8N1
    try { await dev.controlTransferOut({ requestType: "class", recipient: "interface", request: 0x20, value: 0, index: idx }, lc); } catch (e) {}
    try { await dev.controlTransferOut({ requestType: "class", recipient: "interface", request: 0x22, value: 0x03, index: idx }); } catch (e) {}
  }
  let buf = "", alive = true;
  (async () => {
    while (alive) {
      try { const r = await dev.transferIn(epIn, 512); if (r.data) buf += dec.decode(r.data); }
      catch (e) { alive = false; }
    }
  })();
  return {
    write: (s) => dev.transferOut(epOut, enc.encode(s)),
    take: () => { const b = buf; buf = ""; return b; },
    close: async () => { alive = false; try { await dev.close(); } catch (e) {} },
  };
}

async function cdcOverSerial() {
  const port = await navigator.serial.requestPort({ filters: [{ usbVendorId: ST_VID }] });
  await port.open({ baudRate: 115200 });
  try { await port.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch (e) {}
  const reader = port.readable.getReader(), writer = port.writable.getWriter();
  let buf = "", alive = true;
  (async () => {
    while (alive) {
      try { const { value, done } = await reader.read(); if (done) break; buf += dec.decode(value); }
      catch (e) { break; }
    }
  })();
  return {
    write: (s) => writer.write(enc.encode(s)),
    take: () => { const b = buf; buf = ""; return b; },
    close: async () => { alive = false; try { reader.releaseLock(); writer.releaseLock(); await port.close(); } catch (e) {} },
  };
}

// Паспорт прошивки: STX "id" \n → "@id {json}" (telemetry/FirmwareIdentity.h, CopterLink.h)
async function readIdentity(con) {
  con.take();
  for (let attempt = 0; attempt < 3; attempt++) {
    await con.write("\x02id\n");
    let acc = "";
    for (let t = 0; t < 10; t++) {
      await sleep(100);
      acc += con.take();
      const m = acc.match(/@id (\{[^\n]*\})/);
      if (m) { try { return JSON.parse(m[1]); } catch (e) {} }
    }
  }
  return null;
}

// Паспорт → env сборки
function envFromIdentity(id) {
  const copter = id.vehicle === "copter";
  const b = id.board || "";
  if (b.endsWith("-quad")) return b;
  if (b === "matek-h743") return copter ? "matek-h743-quad" : "matek-h743";
  if (b === "stm32h743" || b === "stm32h743-devebox") return copter ? "stm32h743-devebox-quad" : "stm32h743-devebox";
  return b;
}

// ---------------- DfuSe (системный загрузчик STM32) ----------------
const DFU = { DNLOAD: 1, UPLOAD: 2, GETSTATUS: 3, CLRSTATUS: 4, GETSTATE: 5, ABORT: 6 };
const S = { dfuIDLE: 2, dfuDNBUSY: 4, dfuDNLOAD_IDLE: 5, dfuMANIFEST: 7, dfuUPLOAD_IDLE: 9, dfuERROR: 10 };

function parseLayout(desc) {
  // "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg"
  const mult = { " ": 1, B: 1, K: 1024, M: 1048576 };
  const segs = [];
  const re = /\/\s*(0x[0-9a-fA-F]{1,8})\s*\/((?:\s*\d+\s*\*\s*\d+\s?[ BKM]\s*[a-g]\s*,?)+)/g;
  let m;
  while ((m = re.exec(desc))) {
    let addr = parseInt(m[1], 16);
    const sr = /(\d+)\s*\*\s*(\d+)\s?([ BKM])\s*([a-g])/g; let s;
    while ((s = sr.exec(m[2]))) {
      const count = parseInt(s[1], 10), size = parseInt(s[2], 10) * mult[s[3]];
      const p = s[4].charCodeAt(0) - 96;
      for (let i = 0; i < count; i++) { segs.push({ start: addr, size, erasable: !!(p & 2), writable: !!(p & 4) }); addr += size; }
    }
  }
  return segs;
}
function chipFromLayout(sectors) {
  const total = sectors.reduce((a, s) => a + s.size, 0);
  if (sectors.length && sectors[0].size === 16384 && total <= 1048576) return "f405";
  if (sectors.length && sectors[0].size === 131072 && total === 2097152) return "h743";
  return "unknown";
}

class Dfu {
  constructor(dev) { this.dev = dev; }
  async open() {
    const d = this.dev;
    await d.open();
    if (!d.configuration) await d.selectConfiguration(1);
    // интерфейс DFU, альтернатива "@Internal Flash"
    let pick = null;
    for (const intf of d.configuration.interfaces) {
      for (const alt of intf.alternates) {
        if (alt.interfaceClass === 0xfe && alt.interfaceSubclass === 0x01) {
          const name = alt.interfaceName || (await this.stringFor(intf.interfaceNumber, alt.alternateSetting));
          log(`alt ${alt.alternateSetting}: ${name}`);
          if (!pick && /internal flash/i.test(name || "")) pick = { intf: intf.interfaceNumber, alt: alt.alternateSetting, name };
        }
      }
    }
    if (!pick) throw new Error("no '@Internal Flash' DFU alt setting");
    this.intf = pick.intf;
    await d.claimInterface(this.intf);
    await d.selectAlternateInterface(this.intf, pick.alt);
    this.sectors = parseLayout(pick.name);
    this.chip = chipFromLayout(this.sectors);
    this.xfer = (await this.readTransferSize()) || 1024;
    log(`chip ${this.chip}, xfer ${this.xfer}, sectors ${this.sectors.length}`);
  }
  async rawDesc(type, index, len, langId = 0) {
    const r = await this.dev.controlTransferIn({ requestType: "standard", recipient: "device", request: 6, value: (type << 8) | index, index: langId }, len);
    return new DataView(r.data.buffer);
  }
  async configDesc() {
    if (this._cfg) return this._cfg;
    const head = await this.rawDesc(2, 0, 9);
    this._cfg = await this.rawDesc(2, 0, head.getUint16(2, true));
    return this._cfg;
  }
  async stringFor(intfNum, altNum) {
    try {
      const c = await this.configDesc();
      for (let o = 0; o < c.byteLength; o += c.getUint8(o)) {
        if (c.getUint8(o + 1) === 4 && c.getUint8(o + 2) === intfNum && c.getUint8(o + 3) === altNum) {
          const idx = c.getUint8(o + 8); if (!idx) return "";
          const s = await this.rawDesc(3, idx, 255, 0x0409);
          let out = ""; for (let i = 2; i < s.getUint8(0); i += 2) out += String.fromCharCode(s.getUint16(i, true));
          return out;
        }
        if (!c.getUint8(o)) break;
      }
    } catch (e) { log("string desc: " + e.message); }
    return "";
  }
  async readTransferSize() {
    try {
      const c = await this.configDesc();
      for (let o = 0; o < c.byteLength && c.getUint8(o); o += c.getUint8(o)) {
        if (c.getUint8(o + 1) === 0x21 && c.getUint8(o) >= 7) return c.getUint16(o + 5, true);
      }
    } catch (e) { log("func desc: " + e.message); }
    return 0;
  }
  out(req, value, data) {
    return this.dev.controlTransferOut({ requestType: "class", recipient: "interface", request: req, value, index: this.intf }, data);
  }
  async in(req, value, len) {
    const r = await this.dev.controlTransferIn({ requestType: "class", recipient: "interface", request: req, value, index: this.intf }, len);
    if (r.status !== "ok") throw new Error("control in: " + r.status);
    return r.data;
  }
  async getStatus() {
    const d = await this.in(DFU.GETSTATUS, 0, 6);
    return { status: d.getUint8(0), poll: d.getUint8(1) | (d.getUint8(2) << 8) | (d.getUint8(3) << 16), state: d.getUint8(4) };
  }
  async pollUntil(pred) {
    let st = await this.getStatus();
    while (!pred(st.state) && st.state !== S.dfuERROR) { await sleep(Math.max(st.poll, 5)); st = await this.getStatus(); }
    return st;
  }
  async toIdle() {
    let st = await this.getStatus();
    if (st.state === S.dfuERROR) { await this.out(DFU.CLRSTATUS, 0); st = await this.getStatus(); }
    if (st.state !== S.dfuIDLE) { await this.out(DFU.ABORT, 0); st = await this.getStatus(); }
    if (st.state === S.dfuERROR) { await this.out(DFU.CLRSTATUS, 0); st = await this.getStatus(); }
    if (st.state !== S.dfuIDLE) throw new Error("DFU not idle, state " + st.state);
  }
  async command(cmd, addr) {
    const p = new Uint8Array([cmd, addr & 0xff, (addr >>> 8) & 0xff, (addr >>> 16) & 0xff, (addr >>> 24) & 0xff]);
    await this.out(DFU.DNLOAD, 0, p);
    const st = await this.pollUntil((s) => s !== S.dfuDNBUSY);
    if (st.status !== 0) throw new Error(`DfuSe 0x${cmd.toString(16)} @0x${addr.toString(16)}: status ${st.status}`);
  }
  async flash(start, data, onPhase) {
    await this.toIdle();
    const end = start + data.length;
    const touched = this.sectors.filter((s) => s.start < end && s.start + s.size > start);
    if (!touched.length || touched.some((s) => !s.erasable || !s.writable)) throw new Error("address range not writable");
    for (let i = 0; i < touched.length; i++) {
      onPhase("erase", i / touched.length, i + 1, touched.length);
      await this.command(0x41, touched[i].start);
    }
    for (let off = 0; off < data.length; off += this.xfer) {
      onPhase("write", off / data.length);
      await this.command(0x21, start + off);
      await this.out(DFU.DNLOAD, 2, data.subarray(off, Math.min(off + this.xfer, data.length)));
      const st = await this.pollUntil((s) => s === S.dfuDNLOAD_IDLE);
      if (st.status !== 0) throw new Error(`write @0x${(start + off).toString(16)}: status ${st.status}`);
    }
    onPhase("write", 1);
  }
  async verify(start, data, onPhase) {
    await this.toIdle();
    await this.command(0x21, start);
    await this.out(DFU.ABORT, 0);
    for (let off = 0, block = 2; off < data.length; off += this.xfer, block++) {
      onPhase("verify", off / data.length);
      const want = Math.min(this.xfer, data.length - off);
      const got = new Uint8Array((await this.in(DFU.UPLOAD, block, this.xfer)).buffer).subarray(0, want);
      for (let i = 0; i < want; i++) if (got[i] !== data[off + i]) return start + off + i;
    }
    await this.out(DFU.ABORT, 0);
    onPhase("verify", 1);
    return -1;
  }
  async leave(start) {
    await this.toIdle();
    await this.command(0x21, start);
    await this.out(DFU.DNLOAD, 0, new Uint8Array(0));
    try { await this.pollUntil((s) => s === S.dfuMANIFEST); } catch (e) { /* плата ушла в прошивку */ }
    try { await this.dev.close(); } catch (e) {}
  }
}

// ---------------- ход прошивки ----------------
let busy = false, dfuResolve = null, lastId = null, lastError = "";
const isDfu = (d) => d.vendorId === ST_VID && d.productId === DFU_PID;

async function firmwareBytes() {
  if ($("fw").value === "__file") { if (!customBin) throw new Error(M().pickFirst); return { data: customBin.data, chip: null, env: null, base: 0x08000000, limit: null }; }
  const f = selected(); if (!f) throw new Error(M().pickFirst);
  const r = await fetch("firmware/" + f.file, { cache: "no-cache" });
  if (!r.ok) throw new Error("download " + f.file + ": " + r.status);
  const data = new Uint8Array(await r.arrayBuffer());
  if (f.sha256 && crypto.subtle) {
    const h = Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256", data))).map((b) => b.toString(16).padStart(2, "0")).join("");
    if (h !== f.sha256) throw new Error("sha256 mismatch");
  }
  return { data, chip: f.chip, env: f.env, base: parseInt(f.address, 16), limit: f.limit ? parseInt(f.limit, 16) : null };
}

async function waitForDfu(ms) {
  const known = (await navigator.usb.getDevices()).find(isDfu);
  if (known) return known;
  return new Promise((resolve) => {
    const t = setTimeout(() => { dfuResolve = null; resolve(null); }, ms);
    dfuResolve = (d) => { clearTimeout(t); dfuResolve = null; resolve(d); };
  });
}
if ("usb" in navigator) navigator.usb.addEventListener("connect", (e) => { if (isDfu(e.device) && dfuResolve) dfuResolve(e.device); });

async function run() {
  if (busy) return; busy = true; $("go").disabled = true; $("go").classList.remove("pulse"); progress(0);
  $("s1").classList.remove("done"); $("s2").classList.remove("done"); $("reportnow").classList.add("hide");
  try {
    let dev = null;
    if ("usb" in navigator) {
      status(M().connecting);
      try { dev = await navigator.usb.requestDevice({ filters: [{ vendorId: ST_VID }] }); }
      catch (e) { if (e.name === "NotFoundError") throw new Error(M().cancelled); throw e; }
    }
    if (!dev || !isDfu(dev)) {
      // консоль работающей прошивки
      let con = null;
      if (dev) { try { con = await cdcOverUsb(dev); } catch (e) { log("WebUSB CDC: " + e.message); try { await dev.close(); } catch (_) {} } }
      if (!con && "serial" in navigator) con = await cdcOverSerial();
      if (!con) throw new Error("no access to the board console");
      const id = await readIdentity(con);
      if (id) {
        lastId = id;
        log("@id " + JSON.stringify(id));
        $("s1").classList.add("done");
        status(M().idOk(id));
        if (id.armed) { await con.close(); throw new Error(M().armed); }
        const env = envFromIdentity(id);
        if (manifest && manifest.firmware.some((f) => f.env === env)) {
          if (!$("fw").value) { $("fw").value = env; showInfo(); }
          else if ($("fw").value !== "__file" && $("fw").value !== env) log(M().mismatch(env));
        }
      }
      if (!$("fw").value || ($("fw").value === "__file" && !customBin)) { await con.close(); $("fw").focus(); throw new Error(M().pickFirst); }
      status(M().toDfu);
      await con.write("D");
      await sleep(300);
      await con.close();
      status(M().waitDfu);
      dev = await waitForDfu(4000);
      if (!dev) {
        status(M().needTap, "warn");
        $("pickdfu").classList.remove("hide");
        dev = await new Promise((resolve) => {
          $("pickdfu").onclick = async () => {
            try { resolve(await navigator.usb.requestDevice({ filters: [{ vendorId: ST_VID, productId: DFU_PID }] })); }
            catch (e) { resolve(null); }
          };
        });
        $("pickdfu").classList.add("hide");
        if (!dev) throw new Error(M().noDfu);
      }
    }
    if (!$("fw").value) { $("fw").focus(); throw new Error(M().pickFirst); }
    const fw = await firmwareBytes();
    let data = fw.data;
    if (data.length % 32) { const p = new Uint8Array(Math.ceil(data.length / 32) * 32).fill(0xff); p.set(data); data = p; }
    const dfu = new Dfu(dev);
    await dfu.open();
    status(M().chip(dfu.chip.toUpperCase()));
    if (fw.chip && dfu.chip !== "unknown" && fw.chip !== dfu.chip) throw new Error(M().wrongChip(fw.chip.toUpperCase(), dfu.chip.toUpperCase()));
    const limit = fw.limit || (dfu.chip === "f405" ? 0x080e0000 : dfu.chip === "h743" ? 0x081e0000 : null);
    if (limit && fw.base + data.length > limit) throw new Error(M().tooBig);
    const phase = (p, f, i, n) => {
      if (p === "erase") { status(M().erase(i, n)); progress(0.15 * f); }
      if (p === "write") { $("status").textContent = M().write(Math.round(f * 100)); progress(0.15 + 0.65 * f); }
      if (p === "verify") { $("status").textContent = M().verify(Math.round(f * 100)); progress(0.8 + 0.2 * f); }
    };
    const t0 = performance.now();
    await dfu.flash(fw.base, data, phase);
    const bad = await dfu.verify(fw.base, data, phase);
    if (bad >= 0) throw new Error(M().verifyBad(bad));
    log(`${data.length} B, ${((performance.now() - t0) / 1000).toFixed(1)} s`);
    await dfu.leave(fw.base);
    progress(1);
    $("s2").classList.add("done");
    status(M().done, "ok");
  } catch (e) {
    status(M().fail + (e.message || e), "bad");
    lastError = String(e.message || e);
    $("reportnow").classList.remove("hide");
  } finally {
    busy = false; $("go").disabled = false;
  }
}
$("go").onclick = run;

if (!("usb" in navigator) && !("serial" in navigator)) { $("unsupported").classList.remove("hide"); $("go").disabled = true; }
applyLang();
loadManifest();

// ---------------- отчёт об ошибке ----------------
// Уходит на сервер Lebedev Space (/api/flash-report): сразу в Telegram автору; с контактом — ещё и в CRM.
// С GitHub Pages — на тот же сервер (CORS разрешён только для этого адреса).
const REPORT_API = /(^|\.)github\.io$/.test(location.hostname) || location.protocol === "file:"
  ? "https://lebedev-space.ru/api/flash-report" : "/api/flash-report";
const CONTACT_RE = /^(@?[A-Za-z0-9_]{4,32}|https?:\/\/t\.me\/[A-Za-z0-9_]{4,32}|[^\s@]{1,64}@[^\s@]{1,190}\.[^\s@]{2,}|\+?[0-9\s()\-]{10,18})$/;

function attachment() {
  const journal = $("log").textContent.slice(-16000);
  return {
    board: lastId ? { board: lastId.board || "", vehicle: lastId.vehicle || "", commit: lastId.commit || "", rx: lastId.rx || "", armed: !!lastId.armed } : null,
    firmware: $("fw").value === "__file" ? (customBin ? "file:" + customBin.name : "file") : $("fw").value,
    build: manifest ? `${manifest.date} ${manifest.commit}` : "",
    status: $("status").textContent.slice(0, 300),
    error: lastError.slice(0, 300),
    ua: navigator.userAgent.slice(0, 300),
    usb: "usb" in navigator, serial: "serial" in navigator,
    journal,
  };
}
function attachmentText(a) {
  return [
    `firmware: ${a.firmware || "—"} · build ${a.build || "—"}`,
    a.board ? `board: ${a.board.board} ${a.board.vehicle} ${a.board.commit} rx=${a.board.rx} armed=${a.board.armed}` : "board: —",
    `status: ${a.status || "—"}`, `browser: ${a.ua}`, `webusb=${a.usb} webserial=${a.serial}`, "",
    a.journal || M().rNothing,
  ].join("\n");
}
function refreshForm() {
  const hasContact = $("rcontact").value.trim() !== "";
  $("rnewsrow").classList.toggle("off", !hasContact);
  $("rnews").disabled = !hasContact;
  if (!hasContact) $("rnews").checked = false;
  $("rattachtext").textContent = $("rattach").checked ? attachmentText(attachment()) : "—";
}
function openReport(prefill) {
  $("report").classList.remove("hide");
  $("bugbtn").classList.add("hide");
  if (prefill && !$("rmsg").value) $("rmsg").value = prefill;
  $("rsent").textContent = ""; $("rsent").className = "sent"; $("rcopy").classList.add("hide");
  refreshForm();
  $("report").scrollIntoView({ behavior: "smooth", block: "start" });
  setTimeout(() => $("rmsg").focus(), 300);
}
function closeReport() { $("report").classList.add("hide"); $("bugbtn").classList.remove("hide"); }
$("bugbtn").onclick = () => openReport();
$("rcancel").onclick = closeReport;
$("reportnow").onclick = () => openReport(lastError ? `${M().fail}${lastError}\n\n` : "");
["rcontact", "rattach"].forEach((id) => $(id).addEventListener("input", refreshForm));
$("rattach").addEventListener("change", refreshForm);

async function sendReport(ev) {
  ev.preventDefault();
  const sent = $("rsent");
  const message = $("rmsg").value.trim(), nick = $("rnick").value.trim(), contact = $("rcontact").value.trim();
  const fail = (t) => { sent.textContent = t; sent.className = "sent bad"; };
  if (message.length < 3) { $("rmsg").focus(); return fail(M().rNeedMsg); }
  if (contact && !CONTACT_RE.test(contact)) { $("rcontact").focus(); return fail(M().rBadContact); }
  if ((contact || nick) && !$("rconsent").checked) return fail(M().rNeedConsent);
  const body = {
    message, nick, contact, consent: $("rconsent").checked, news: !!contact && $("rnews").checked,
    lang, page: location.href.slice(0, 300), website: $("rtrap").value,
    attach: $("rattach").checked ? attachment() : null,
  };
  $("rsend").disabled = true; sent.textContent = M().rSending; sent.className = "sent";
  try {
    const r = await fetch(REPORT_API, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.message || "HTTP " + r.status);
    sent.textContent = M().rOk; sent.className = "sent ok";
    $("rmsg").value = "";
  } catch (e) {
    log("report: " + e.message);
    fail(`${M().rFail} (${e.message})`);
    const btn = $("rcopy"); btn.classList.remove("hide"); btn.textContent = M().rCopy;
    btn.onclick = async () => {
      const text = `${message}\n\n${nick ? "nick: " + nick + "\n" : ""}${contact ? "contact: " + contact + "\n" : ""}\n${body.attach ? attachmentText(body.attach) : ""}`;
      try { await navigator.clipboard.writeText(text); btn.textContent = M().rCopied; } catch (_) { /* старый браузер */ }
      location.href = "https://t.me/od1ndva4";
    };
  } finally { $("rsend").disabled = false; }
}
$("report").addEventListener("submit", sendReport);
