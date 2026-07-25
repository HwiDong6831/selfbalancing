"use strict";

// WebSocket 연결 (자동 재접속)
const connEl = document.getElementById("conn");
let ws = null;

function wsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws/telemetry`;
}

function connect() {
  ws = new WebSocket(wsUrl());
  ws.onopen = () => setConn(true);
  ws.onclose = () => { setConn(false); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let frame;
    try { frame = JSON.parse(ev.data); } catch { return; }
    render(frame);
  };
}

function setConn(on) {
  connEl.textContent = on ? "연결됨" : "연결 끊김";
  connEl.className = "conn " + (on ? "conn--on" : "conn--off");
}

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
  const vr = $("vote-result");
  vr.textContent = v.result || "–";
  vr.className = "badge badge--" + (v.result === "ok" ? "ok" : v.result === "fail" ? "fail" : "warn");
  $("vote-detail").textContent =
    `used: [${(v.used || []).join(", ")}]　rejected: [${(v.rejected || []).join(", ")}]`;

  renderSensors(frame.sensors || []);
  push(b);
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
    f.textContent = s.fault;
    f.className = "fault fault--" + s.fault;
    // 가속도는 g(±2), 각속도는 deg/s(±250) 라 유효자리가 다르다.
    for (const k of ["ax", "ay", "az"]) c.querySelector("." + k).textContent = fmt(s[k], 3);
    for (const k of ["gx", "gy", "gz"]) c.querySelector("." + k).textContent = fmt(s[k], 1);
  });
}

// 차트 렌더 루프
function loop() { drawChart(); requestAnimationFrame(loop); }

connect();
requestAnimationFrame(loop);
