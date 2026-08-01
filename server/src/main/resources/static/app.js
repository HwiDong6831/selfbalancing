"use strict";

// WebSocket 연결 (자동 재접속)
const connEl = document.getElementById("conn");
let ws = null;

function wsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws/telemetry`;
}

/*
 * 연결은 두 개다.  브라우저 ──(/ws/telemetry)── 서버 ──(/ws/esp)── ESP32
 * 소켓만 보면 ESP 가 빠져도 "연결됨" 으로 남으므로 수신 여부를 같이 본다.
 * 로그도 ESP 에서 오니 생존 신호로 센다 (부팅 직후 6초는 로그만 오고 프레임이 없다).
 */
const ALIVE_TIMEOUT_MS = 1000;
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
    // 프레임과 시리얼 로그가 같은 소켓으로 온다
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

// 수신이 끊긴 것은 이벤트가 아니라 시간 경과로만 알 수 있다
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
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  // 세 시리즈 절대 최대값 기준 대칭 스케일
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

// 부호 자리 고정용: 음수는 U+2212, 양수는 U+2007(figure space) — 둘 다 숫자 1개 폭
const MINUS = "−";
const FIGSP = " ";
function fmt(v, d = 2) {
  const n = Number(v);
  if (v == null || Number.isNaN(n)) return "–";
  const mag = Math.abs(n).toFixed(d);
  const neg = n < 0 && parseFloat(mag) !== 0;   // -0.x → 0 이면 부호 제거
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
  renderVote("accel", v.accel);
  renderVote("gyro", v.gyro);

  renderSensors(frame.sensors || []);
  push(b);
}

// 서버 계약은 영문 코드로 오고 화면에만 우리말로 바꾼다. CSS 클래스는 코드 그대로 쓴다.
const FAULT_LABEL = { none: "정상", dropout: "끊김", freeze: "고정", drift: "드리프트" };
const VOTE_LABEL  = { ok: "정상", degraded: "일부 이상", fail: "실패" };

// 가속도·자이로 판정 한 덩이
function renderVote(kind, v) {
  v = v || {};
  const badge = $(`vote-${kind}-result`);
  badge.textContent = VOTE_LABEL[v.result] || "–";
  badge.className = "badge badge--" +
    (v.result === "ok" ? "ok" : v.result === "fail" ? "fail" : "warn");
  $(`vote-${kind}-detail`).textContent =
    `used: [${(v.used || []).join(", ")}]　rejected: [${(v.rejected || []).join(", ")}]`;
}

function renderSensors(sensors) {
  const host = $("sensors");
  while (host.children.length < sensors.length) {   // 카드 부족하면 생성
    const card = document.createElement("div");
    card.className = "sensor-card";
    card.innerHTML =
      `<h3><span class="ch"></span><span class="fault"></span></h3>
       <table><tbody>
         <tr><td class="k">ax</td><td class="v ax"></td><td class="k">gx</td><td class="v gx"></td></tr>
         <tr><td class="k">ay</td><td class="v ay"></td><td class="k">gy</td><td class="v gy"></td></tr>
         <tr><td class="k">az</td><td class="v az"></td><td class="k">gz</td><td class="v gz"></td></tr>
       </tbody></table>`;
    host.appendChild(card);
  }
  sensors.forEach((s, i) => {
    const c = host.children[i];
    c.querySelector(".ch").textContent = "ch " + s.ch;
    const f = c.querySelector(".fault");
    const code = s.fault || "none";
    f.textContent = FAULT_LABEL[code] || code;
    f.className = "fault fault--" + code;
    // 가속도는 g(±2), 각속도는 deg/s(±250) 라 유효자리가 다르다.
    for (const k of ["ax", "ay", "az"]) c.querySelector("." + k).textContent = fmt(s[k], 3);
    for (const k of ["gx", "gy", "gz"]) c.querySelector("." + k).textContent = fmt(s[k], 1);
  });
}

// 시리얼 로그
const LOG_CAP = 400;
const logEl = $("log");
const followEl = $("log-follow");

$("log-clear").onclick = () => { logEl.textContent = ""; };

function appendLog(line) {
  // ESP-IDF 형식 "I (1234) TAG: 본문" 의 첫 글자가 레벨
  const lv = /^[IWED] \(/.test(line) ? line[0] : "";
  const row = document.createElement("div");
  if (lv) row.className = "lv-" + lv;
  row.textContent = line;
  logEl.appendChild(row);

  while (logEl.childElementCount > LOG_CAP) logEl.removeChild(logEl.firstChild);
  if (followEl.checked) logEl.scrollTop = logEl.scrollHeight;
}

// 차트 렌더 루프
function loop() { drawChart(); requestAnimationFrame(loop); }

connect();
requestAnimationFrame(loop);
