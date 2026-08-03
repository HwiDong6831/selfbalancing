"use strict";

// WebSocket 연결 (자동 재접속)
const connEl = document.getElementById("conn");
let ws = null;

function wsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws/telemetry`;
}

const ALIVE_TIMEOUT_MS = 1000;   // 이 시간 넘게 수신이 없으면 ESP 가 끊긴 것으로 본다
let sockOpen = false;
let lastRxAt = 0;

function connect() {
  ws = new WebSocket(wsUrl());
  ws.onopen = () => { sockOpen = true; updateConn(); };
  ws.onclose = () => { sockOpen = false; updateConn(); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    lastRxAt = Date.now();
    updateConn();
    if (typeof msg.log === "string") appendLog(msg.log);
    else render(msg);
  };
}

function updateConn() {
  let cls, text;
  if (!sockOpen) {
    cls = "off";  text = "서버 연결 끊김";
  } else if (Date.now() - lastRxAt > ALIVE_TIMEOUT_MS) {
    cls = "warn"; text = "ESP 응답 없음";
  } else {
    cls = "on";   text = "연결됨";
  }
  connEl.textContent = text;
  connEl.className = "conn conn--" + cls;
}

setInterval(updateConn, 250);

// 차트: angle/rate/uq 링버퍼
const CAP = 240;
const buf = { angle: [], rate: [], uq: [] };
const canvas = document.getElementById("chart");
const ctx = canvas.getContext("2d");

const SERIES = [
  { key: "angle", color: "#4da3ff" },
  { key: "rate",  color: "#ffb454" },
  { key: "uq",    color: "#7ee787" },
];

function push(b) {
  buf.angle.push(b.angle);
  buf.rate.push(b.rate);
  buf.uq.push(b.uq);
  for (const k in buf) if (buf[k].length > CAP) buf[k].shift();
}

function drawChart() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;

  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  let max = 1;
  for (const s of SERIES) for (const v of buf[s.key]) max = Math.max(max, Math.abs(v));
  max *= 1.15;

  const mid = h / 2;   // 0 기준선
  ctx.strokeStyle = "#2a323d";
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(w, mid); ctx.stroke();

  const n = buf.angle.length;
  if (n < 2) return;
  const dx = w / (CAP - 1);
  const off = CAP - n;   // 최신이 우측

  for (const s of SERIES) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const arr = buf[s.key];
    for (let i = 0; i < n; i++) {
      const x = (off + i) * dx;
      const y = mid - (arr[i] / max) * (h / 2 - 6);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

const $ = (id) => document.getElementById(id);

// 부호 자리 고정용 문자. 둘 다 숫자 한 칸 폭이다 (U+2212, U+2007)
const MINUS = "−";
const FIGSP = " ";
function fmt(v, d = 2) {
  const n = Number(v);
  if (v == null || Number.isNaN(n)) return "–";
  const mag = Math.abs(n).toFixed(d);
  const neg = n < 0 && parseFloat(mag) !== 0;
  return (neg ? MINUS : FIGSP) + mag;
}

function render(frame) {
  const b = frame.balance || {};
  $("b-angle").textContent = fmt(b.angle);
  $("b-rate").textContent  = fmt(b.rate, 1);
  $("b-set").textContent   = fmt(b.setpoint);
  $("b-uq").textContent    = fmt(b.uq);
  $("enc").textContent     = fmt(frame.encoder ? frame.encoder.angle : null, 1);

  const v = frame.voting || {};
  const chans = (frame.sensors || []).map((s) => s.ch);
  renderVote("accel", v.accel, chans);
  renderVote("gyro", v.gyro, chans);

  renderSensors(frame.sensors || []);
  syncFaultRows(frame.sensors || []);
  recordVote(frame);
  push(b);
  if (window.robot3d) window.robot3d.set(b.angle, frame.encoder ? frame.encoder.angle : 0);
}

// 영문 코드 → 화면 표기. CSS 클래스는 코드 그대로 쓴다.
const FAULT_LABEL = { none: "정상", dropout: "끊김", freeze: "고정", drift: "드리프트" };
const VOTE_LABEL  = { ok: "정상", degraded: "일부 이상", single: "단일 센서", fail: "실패" };

// 판정 한 덩이를 배지와 채널 카드로 그린다.
function renderVote(kind, v, chans) {
  v = v || {};
  const badge = $(`vote-${kind}-result`);
  badge.textContent = VOTE_LABEL[v.result] || "–";
  badge.className = "badge badge--" +
    (v.result === "ok" ? "ok" : v.result === "fail" ? "fail" : "warn");

  const host = $(`vote-${kind}-chs`);
  while (host.childElementCount < chans.length) {
    host.appendChild(document.createElement("span"));
  }
  const used = v.used || [];
  chans.forEach((ch, i) => {
    const cls = v.result === "fail" ? "fail" : used.includes(ch) ? "ok" : "warn";
    const el = host.children[i];
    el.className = "ch-card ch-card--" + cls;
    el.textContent = "ch " + ch;
    el.title = cls === "ok" ? "채택" : cls === "fail" ? "검증 불가" : "배제";
  });
}

// 이상 이력. 같은 상태가 이어지는 동안은 한 행으로 묶고 지속 시간만 갱신한다.
const VH_CAP = 200;          // 표에 쌓아 두는 최대 행 수
const VH_SIG = {
  accel: { unit: "g",   digits: 4, axes: ["ax", "ay"] },
  gyro:  { unit: "°/s", digits: 2, axes: ["gz"] },
};
const vh = { accel: { key: "ok" }, gyro: { key: "ok" } };

const vhChs = new Set();     // 채널 필터. 비어 있으면 전부 보여준다

$("vh-clear").onclick = () => {
  for (const sig of ["accel", "gyro"]) {
    $("vh-" + sig).textContent = "";
    vh[sig] = { key: "ok" };
  }
  updateEmpty();
};

for (const tab of document.querySelectorAll(".vh-tab")) {
  tab.onclick = () => {
    const sig = tab.dataset.sig;
    for (const t of document.querySelectorAll(".vh-tab")) t.classList.toggle("is-on", t === tab);
    $("vh-accel").hidden = sig !== "accel";
    $("vh-gyro").hidden  = sig !== "gyro";
    $("vh-table").dataset.sig = sig;
    updateEmpty();
  };
}

const activeSig = () => $("vh-table").dataset.sig;
const rowShown = (tr) => tr.dataset.chs.split(",").some((c) => vhChs.has(c));

function updateEmpty() {
  const body = $("vh-" + activeSig());
  const el = $("vh-empty");
  el.hidden = [...body.children].some((tr) => !tr.hidden);
  el.textContent = body.childElementCount ? "선택한 채널에 해당하는 이상 없음" : "이상 없음";
}

function applyFilter() {
  for (const sig of ["accel", "gyro"]) {
    for (const tr of $("vh-" + sig).children) tr.hidden = !rowShown(tr);
  }
  updateEmpty();
}

// 프레임에 실려 온 채널로 필터 체크박스를 만든다.
function syncChannelFilter(sensors) {
  const host = $("vh-chs");
  for (const s of sensors) {
    const ch = String(s.ch);
    if (vhChs.has(ch) || host.querySelector(`[data-ch="${ch}"]`)) continue;
    vhChs.add(ch);

    const box = document.createElement("input");
    box.type = "checkbox";
    box.checked = true;
    box.dataset.ch = ch;
    box.onchange = () => {
      if (box.checked) vhChs.add(ch);
      else vhChs.delete(ch);
      applyFilter();
    };

    const label = document.createElement("label");
    label.className = "vh-ch";
    label.append(box, "ch" + ch);
    host.appendChild(label);
  }
}

function recordVote(frame) {
  const v = frame.voting || {};
  if (v.tol) {
    $("vh-tol").textContent =
      `임계값  가속도 ${fmt(v.tol.accel, 4)} g　자이로 ${fmt(v.tol.gyro, 2)} °/s`;
  }
  syncChannelFilter(frame.sensors || []);
  for (const sig of ["accel", "gyro"]) track(sig, v[sig], frame.sensors || []);
}

function track(sig, vote, sensors) {
  const st = vh[sig];
  const result = (vote && vote.result) || "ok";
  const rejected = (vote && vote.rejected) || [];

  // 같은 사건인지 가리는 열쇠. 배제 채널이 바뀌면 판정이 같아도 다른 사건이다.
  const key = result === "ok" ? "ok" : result + ":" + rejected.join(",");
  if (key === st.key) {
    if (result !== "ok") {
      st.frames++;
      st.dur.textContent = duration(st);
    }
    return;
  }
  st.key = key;
  if (result === "ok") return;

  st.t0 = Date.now();
  st.frames = 1;
  addRow(sig, result, vote, rejected, sensors, st);
  updateEmpty();
}

const duration = (st) =>
  `${st.frames}프레임 / ${((Date.now() - st.t0) / 1000).toFixed(2)}초`;

function addRow(sig, result, vote, rejected, sensors, st) {
  const { unit, digits, axes } = VH_SIG[sig];

  const bad = rejected.length === 1 ? sensors.find((s) => s.ch === rejected[0]) : null;
  let badTxt = "–", okTxt = "–", gapTxt = "–";
  if (bad) {
    const bv = axes.map((k) => Number(bad[k]));
    const ov = (vote.val || []).map(Number);
    badTxt = bv.map((x) => fmt(x, digits)).join(" ");
    okTxt  = ov.map((x) => fmt(x, digits)).join(" ");
    let gap = 0;
    for (let i = 0; i < bv.length && i < ov.length; i++) {
      gap = Math.max(gap, Math.abs(bv[i] - ov[i]));
    }
    gapTxt = fmt(gap, digits) + " " + unit;
    if (bad.fault && bad.fault !== "none") badTxt = FAULT_LABEL[bad.fault] || bad.fault;
  }

  const t = new Date(st.t0);
  const hhmmss = t.toTimeString().slice(0, 8) +
                 "." + String(t.getMilliseconds()).padStart(3, "0");

  const tr = document.createElement("tr");
  tr.className = "vh-" + (result === "fail" ? "fail" : "warn");
  tr.dataset.chs = rejected.join(",");
  tr.hidden = !rowShown(tr);
  const cells = [hhmmss, VOTE_LABEL[result] || result,
                 rejected.length ? rejected.map((c) => "ch" + c).join(", ") : "–",
                 badTxt, okTxt, gapTxt, ""];
  for (const text of cells) {
    const td = document.createElement("td");
    td.textContent = text;
    tr.appendChild(td);
  }
  st.dur = tr.lastChild;
  st.dur.textContent = duration(st);

  const body = $("vh-" + sig);
  body.insertBefore(tr, body.firstChild);   // 최신이 위
  while (body.childElementCount > VH_CAP) body.removeChild(body.lastChild);
}

// 결함 주입. 브라우저 → 서버 → ESP32 로 내려간다.
const FI_MODES = [
  ["none",    "정상"],
  ["dropout", "누락"],
  ["freeze",  "값 고정"],
  ["drift",   "점진적 변형"],
];
const fiMode = new Map();   // ch → mode

function sendFault(ch, mode) {
  fiMode.set(ch, mode);
  const rate = Number($(`fi-rate-${ch}`).value) || 0;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ cmd: "fault", ch, mode, rate }));
  }
  for (const b of document.querySelectorAll(`[data-fi-ch="${ch}"]`)) {
    b.classList.toggle("is-on", b.dataset.fiMode === mode);
  }
}

$("fi-clear").onclick = () => {
  for (const ch of fiMode.keys()) sendFault(ch, "none");
};

function syncFaultRows(sensors) {
  const host = $("fi-rows");
  for (const s of sensors) {
    if (fiMode.has(s.ch)) continue;
    fiMode.set(s.ch, "none");

    const row = document.createElement("div");
    row.className = "fi-row";
    row.innerHTML = `<span class="fi-ch">ch ${s.ch}</span>`;

    for (const [mode, label] of FI_MODES) {
      const b = document.createElement("button");
      b.type = "button";
      b.className = "fi-btn" + (mode === "none" ? " is-on" : "");
      b.textContent = label;
      b.dataset.fiCh = s.ch;
      b.dataset.fiMode = mode;
      b.onclick = () => sendFault(s.ch, mode);
      row.appendChild(b);
    }

    const rate = document.createElement("input");
    rate.type = "number";
    rate.id = `fi-rate-${s.ch}`;
    rate.className = "fi-rate";
    rate.value = "0.5";
    rate.step = "0.1";
    rate.min = "0";
    rate.title = "점진적 변형 속도 — 초당 임계값의 몇 배";
    rate.onchange = () => {
      if (fiMode.get(s.ch) === "drift") sendFault(s.ch, "drift");
    };
    row.append(rate, Object.assign(document.createElement("span"),
                                   { className: "fi-unit", textContent: "배/초" }));
    host.appendChild(row);
  }
}

function renderSensors(sensors) {
  const host = $("sensors");
  while (host.children.length < sensors.length) {
    const card = document.createElement("div");
    card.className = "sensor-card";
    card.innerHTML =
      `<h3><span class="ch"></span><span class="fault"></span></h3>
       <table><tbody>
         <tr><td class="k">ax</td><td class="v ax"></td><td class="k">gx</td><td class="v gx"></td></tr>
         <tr><td class="k">ay</td><td class="v ay"></td><td class="k">gy</td><td class="v gy"></td></tr>
         <tr><td class="k">az</td><td class="v az"></td><td class="k">gz</td><td class="v gz"></td></tr>
       </tbody></table>
       <div class="zero"></div>`;
    host.appendChild(card);
  }
  sensors.forEach((s, i) => {
    const c = host.children[i];
    c.querySelector(".ch").textContent = "ch " + s.ch;
    const f = c.querySelector(".fault");
    const code = s.fault || "none";
    f.textContent = FAULT_LABEL[code] || code;
    f.className = "fault fault--" + code;
    for (const k of ["ax", "ay", "az"]) c.querySelector("." + k).textContent = fmt(s[k], 3);
    for (const k of ["gx", "gy", "gz"]) c.querySelector("." + k).textContent = fmt(s[k], 1);
    c.querySelector(".zero").textContent =
      `영점  ax ${fmt(s.ax0, 4)}  ay ${fmt(s.ay0, 4)}  gz ${fmt(s.gz0, 2)}`;
  });
}

// 시리얼 로그
const LOG_CAP = 400;
const logEl = $("log");
const followEl = $("log-follow");

$("log-clear").onclick = () => { logEl.textContent = ""; };

function appendLog(line) {
  // ESP-IDF 형식 "I (1234) TAG: 본문" 에서 첫 글자가 레벨이다
  const lv = /^[IWED] \(/.test(line) ? line[0] : "";
  const row = document.createElement("div");
  if (lv) row.className = "lv-" + lv;
  row.textContent = line;
  logEl.appendChild(row);

  while (logEl.childElementCount > LOG_CAP) logEl.removeChild(logEl.firstChild);
  if (followEl.checked) logEl.scrollTop = logEl.scrollHeight;
}

// summary 안의 조작 요소는 눌러도 섹션이 접히지 않게 한다.
for (const el of document.querySelectorAll("summary .no-toggle")) {
  el.addEventListener("click", (e) => e.stopPropagation());
}

// 차트 렌더 루프
function loop() { drawChart(); requestAnimationFrame(loop); }

connect();
requestAnimationFrame(loop);
