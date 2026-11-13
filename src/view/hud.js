// จอแสดงผลการบิน (วาดบน canvas 2 มิติทับภาพ 3 มิติ): ความเร็ว ความสูง ทิศ ท่าทาง เครื่องยนต์ น้ำมัน แผนที่ คำเตือน
import { DEG, KMH, G, clamp, fuelTotal } from '../sim/core.js';

const FONT = '"Leelawadee UI","Noto Sans Thai","Segoe UI",sans-serif';
const GREEN = '#7dffb0', AMBER = '#ffcc55', RED = '#ff5a4d', DIM = 'rgba(8,16,24,0.55)';

export class Hud {
  constructor(canvas, data) { this.c = canvas; this.g = canvas.getContext('2d'); this.data = data; this.trail = []; this.trailT = 0; this.resize(); }
  resize() {
    const r = Math.min(window.devicePixelRatio || 1, 2);
    this.w = window.innerWidth; this.h = window.innerHeight;
    this.c.width = Math.round(this.w * r); this.c.height = Math.round(this.h * r);
    this.g.setTransform(r, 0, 0, r, 0, 0);
  }
  resetTrail() { this.trail = []; this.trailT = 0; }
  text(s, x, y, size = 14, color = GREEN, align = 'left', bold = false) {
    const g = this.g; g.font = `${bold ? '600 ' : ''}${size}px ${FONT}`; g.textAlign = align; g.textBaseline = 'middle';
    g.fillStyle = 'rgba(0,0,0,0.6)'; g.fillText(s, x + 1, y + 1); g.fillStyle = color; g.fillText(s, x, y);
  }
  panel(x, y, w, h) { const g = this.g; g.fillStyle = DIM; g.beginPath(); g.roundRect(x, y, w, h, 8); g.fill(); }
  bar(x, y, w, h, frac, color, label, value) {
    const g = this.g; g.fillStyle = 'rgba(255,255,255,0.12)'; g.fillRect(x, y, w, h);
    g.fillStyle = color; g.fillRect(x, y, w * clamp(frac, 0, 1), h);
    if (label) this.text(label, x - 6, y + h / 2, 12, '#cfe', 'right');
    if (value) this.text(value, x + w + 6, y + h / 2, 12, '#fff', 'left');
  }

  /** s = ข้อมูลสรุปจากตัวเกม (ดู main.js: hudState) */
  draw(ac, ap, s, dt) {
    const g = this.g, w = this.w, h = this.h, D = this.data;
    g.clearRect(0, 0, w, h);
    if (s.hidden) return;
    const e = ac.euler(), V = ac.air.V * KMH, alt = ac.alt, vn = ac.vNED(), vs = -vn[2];
    const hdg = ((e.yaw / DEG) % 360 + 360) % 360;

    // ---------- ขั้นบันได pitch + เส้นขอบฟ้า (มุมมองจากหัวเครื่อง) หรือ จอท่าทางเล็ก ----------
    if (s.fpv) this.ladder(w / 2, h / 2, Math.min(w, h) * 0.9, e, ac, s.fovDeg);
    else this.adi(w / 2, h - 96, 62, e);

    // ---------- ความเร็ว (ซ้าย) / ความสูง (ขวา) ----------
    const cy = h / 2, lx = Math.max(150, w * 0.2), rx = w - lx;
    this.tape(lx, cy, V, 10, 5, 'km/h', true, [[s.vsKmh, AMBER, 'Vs'], [D.limits.V_NE_kmh, RED, 'VNE']]);
    this.tape(rx, cy, alt, 20, 5, 'm', false, []);
    this.text(`AGL ${ac.agl.toFixed(0)} m`, rx + 6, cy + 96, 13, ac.agl < 60 && ac.phase === 'air' ? AMBER : '#cfe', 'center');
    this.text(`${vs >= 0 ? '▲' : '▼'} ${Math.abs(vs).toFixed(1)} m/s`, rx + 6, cy + 114, 13, '#cfe', 'center');
    this.text(`GS ${(Math.hypot(vn[0], vn[1]) * KMH).toFixed(0)}`, lx - 6, cy + 96, 13, '#cfe', 'center');
    this.text(`α ${(ac.air.alpha / DEG).toFixed(1)}°  n ${ac.nz.toFixed(2)} g`, lx - 6, cy + 114, 13, Math.abs(ac.nz) > D.limits.n_limit ? RED : '#cfe', 'center');

    // ---------- แถบทิศ (บน) ----------
    this.heading(w / 2, 34, hdg, s.homeBrg);

    // ---------- โหมด / สถานะ (ซ้ายบน) ----------
    this.panel(12, 12, 250, 112);
    this.text(s.modeName, 24, 32, 20, '#fff', 'left', true);
    this.text(s.phaseText, 24, 56, 13, '#bde');
    this.text(`เวลา ${fmtT(s.tFlight)}   ระยะ ${(s.dist / 1000).toFixed(1)} km`, 24, 76, 13, '#bde');
    this.text(`มวล ${ac.mp.m.toFixed(1)} kg   SM ${(ac.mp.SM * 100).toFixed(1)} %`, 24, 96, 13, '#bde');
    this.text(s.timeScale > 1 ? `×${s.timeScale}` : '', 240, 32, 16, AMBER, 'right', true);
    if (s.target) this.text(s.target, 24, 114, 12, AMBER);

    // ---------- เครื่องยนต์ + น้ำมัน (ซ้ายล่าง) ----------
    const eng = ac.engine, px = 12, py = h - 214;
    this.panel(px, py, 290, 202);
    this.text(`${D.engine.name}`, px + 12, py + 16, 13, '#fff', 'left', true);
    const stTxt = { off: 'ดับ', start: 'สตาร์ต: ' + eng.startStage, run: 'ทำงาน', cool: eng.flameout ? 'ดับ (น้ำมันหมด)' : 'กำลังดับ' }[eng.state];
    this.text(stTxt, px + 278, py + 16, 12, eng.state === 'run' ? GREEN : AMBER, 'right');
    this.bar(px + 70, py + 32, 150, 10, ac.ctl.throttle, '#5aa9ff', 'คันเร่ง', `${(ac.ctl.throttle * 100).toFixed(0)} %`);
    this.bar(px + 70, py + 50, 150, 10, eng.n, GREEN, 'รอบ', `${(eng.rpm / 1000).toFixed(0)}k`);
    this.bar(px + 70, py + 68, 150, 10, eng.thrust / 125, GREEN, 'แรงขับ', `${eng.thrust.toFixed(0)} N`);
    this.bar(px + 70, py + 86, 150, 10, (eng.egt - 35) / 800, eng.egt > D.engine.egt_range_C[1] ? RED : AMBER, 'EGT', `${eng.egt.toFixed(0)} °C`);
    const ft = fuelTotal(ac.fuel), ff = eng.fuelFlow * 60000;
    this.text(`น้ำมัน ${ft.toFixed(2)} kg  ·  ${ff.toFixed(0)} g/min  ·  เหลือ ${ff > 1 ? fmtT(ft / eng.fuelFlow) : '—'}`, px + 12, py + 110, 12, ft < 1.5 ? RED : '#cfe');
    let i = 0;
    for (const t of D.mass.tanks) this.bar(px + 70, py + 124 + i++ * 17, 150, 9, ac.fuel[t.name] / t.kg_cap, '#e8a23a', t.name, `${ac.fuel[t.name].toFixed(2)}`);
    this.text(`elevon  L ${ac.dL.toFixed(1)}°  R ${ac.dR.toFixed(1)}°`, px + 12, py + 192, 12, '#bde');

    // ---------- แผนที่ (ขวาล่าง) ----------
    this.map(w - 190, h - 190, 170, ac, s, dt);

    // ---------- คำเตือน ----------
    let wy = 96;
    const blink = Math.floor(performance.now() / 350) % 2 === 0;
    for (const [txt, level] of s.warnings) { if (level < 2 || blink) this.text(txt, w / 2, wy, 22, level >= 2 ? RED : AMBER, 'center', true); wy += 30; }
    if (s.banner) { this.panel(w / 2 - 260, h * 0.24 - 20, 520, 40 + 22 * (s.bannerLines?.length || 0)); this.text(s.banner, w / 2, h * 0.24, 18, '#fff', 'center', true); (s.bannerLines || []).forEach((t, k) => this.text(t, w / 2, h * 0.24 + 26 + 22 * k, 14, '#cfe', 'center')); }
    if (s.toast) this.text(s.toast, w / 2, h - 190, 16, '#fff', 'center', true);
    this.text(s.hint, w / 2, h - 14, 12, 'rgba(255,255,255,0.75)', 'center');
  }

  tape(x, y, val, major, per, unit, left, marks) {
    const g = this.g, H = 150, W = 74, pxPer = 3.2 * (10 / major);
    this.panel(x - W / 2 - (left ? 6 : -6) - 0, y - H / 2 - 8, W, H + 16);
    const x0 = x + (left ? W / 2 - 6 : -W / 2 + 6);
    g.save(); g.beginPath(); g.rect(x - W, y - H / 2, 2 * W, H); g.clip();
    g.strokeStyle = GREEN; g.lineWidth = 1.2;
    const v0 = Math.floor((val - H / 2 / pxPer) / major) * major;
    for (let v = v0; v < val + H / 2 / pxPer + major; v += major) {
      const yy = y - (v - val) * pxPer, big = Math.round(v / major) % per === 0;
      g.beginPath(); g.moveTo(x0, yy); g.lineTo(x0 + (left ? -1 : 1) * (big ? 12 : 6), yy); g.stroke();
      if (big && v >= 0) this.text(String(v), x0 + (left ? -16 : 16), yy, 12, GREEN, left ? 'right' : 'left');
    }
    for (const [mv, col, lab] of marks) { const yy = y - (mv - val) * pxPer; g.strokeStyle = col; g.lineWidth = 3; g.beginPath(); g.moveTo(x0, yy); g.lineTo(x0 + (left ? 8 : -8), yy); g.stroke(); this.text(lab, x0 + (left ? 12 : -12), yy, 10, col, left ? 'left' : 'right'); }
    g.restore();
    g.fillStyle = 'rgba(0,0,0,0.85)'; g.strokeStyle = '#fff'; g.lineWidth = 1.5;
    g.beginPath(); g.roundRect(x - W / 2 + (left ? -10 : 4), y - 15, W + 6, 30, 5); g.fill(); g.stroke();
    this.text(val.toFixed(0), x + (left ? -7 : 7), y - 1, 20, '#fff', 'center', true);
    this.text(unit, x + (left ? -7 : 7), y - H / 2 - 20, 12, '#cfe', 'center');
  }

  heading(x, y, hdg, homeBrg) {
    const g = this.g, W = 360, px = 4;
    this.panel(x - W / 2, y - 22, W, 44);
    g.save(); g.beginPath(); g.rect(x - W / 2 + 4, y - 22, W - 8, 44); g.clip();
    g.strokeStyle = GREEN; g.lineWidth = 1.2;
    for (let d = Math.floor((hdg - 50) / 5) * 5; d <= hdg + 50; d += 5) {
      const xx = x + (d - hdg) * px, dd = ((d % 360) + 360) % 360, big = dd % 30 === 0;
      g.beginPath(); g.moveTo(xx, y + 20); g.lineTo(xx, y + (big ? 6 : 13)); g.stroke();
      if (big) this.text({ 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[dd] ?? String(dd / 10), xx, y - 4, 13, dd % 90 === 0 ? '#fff' : GREEN, 'center', dd % 90 === 0);
    }
    if (homeBrg !== null) { let d = ((homeBrg - hdg + 540) % 360) - 180; d = clamp(d, -43, 43); this.text('⌂', x + d * px, y + 12, 16, AMBER, 'center'); }
    g.restore();
    g.fillStyle = '#fff'; g.beginPath(); g.moveTo(x, y + 22); g.lineTo(x - 6, y + 31); g.lineTo(x + 6, y + 31); g.fill();
    this.text(hdg.toFixed(0).padStart(3, '0') + '°', x, y + 42, 15, '#fff', 'center', true);
  }

  adi(x, y, r, e) {                 // จอท่าทางเล็ก (มุมมองตามหลัง)
    const g = this.g, pp = r / (30 * DEG);
    g.save(); g.beginPath(); g.arc(x, y, r, 0, 6.283); g.clip();
    g.translate(x, y); g.rotate(-e.roll); g.translate(0, e.pitch * pp);
    g.fillStyle = '#3f7fc4'; g.fillRect(-2 * r, -4 * r, 4 * r, 4 * r); g.fillStyle = '#7a5a36'; g.fillRect(-2 * r, 0, 4 * r, 4 * r);
    g.strokeStyle = '#fff'; g.lineWidth = 1.5; g.beginPath(); g.moveTo(-2 * r, 0); g.lineTo(2 * r, 0); g.stroke();
    g.lineWidth = 1; g.font = `10px ${FONT}`; g.fillStyle = '#fff'; g.textAlign = 'center';
    for (let p = -60; p <= 60; p += 10) if (p) { const yy = -p * DEG * pp; g.beginPath(); g.moveTo(-14, yy); g.lineTo(14, yy); g.stroke(); g.fillText(String(Math.abs(p)), 24, yy + 3); }
    g.restore();
    g.strokeStyle = '#fff'; g.lineWidth = 2; g.beginPath(); g.arc(x, y, r, 0, 6.283); g.stroke();
    g.strokeStyle = AMBER; g.lineWidth = 3; g.beginPath(); g.moveTo(x - 30, y); g.lineTo(x - 10, y); g.lineTo(x, y + 7); g.lineTo(x + 10, y); g.lineTo(x + 30, y); g.stroke();
  }

  ladder(x, y, size, e, ac, fovDeg) {      // HUD มุมมองหัวเครื่อง: ขั้นบันได pitch + จุดแนวบิน
    const g = this.g, pp = (this.h / 2) / Math.tan(fovDeg * DEG / 2);       // พิกเซลต่อเรเดียนใกล้กลางจอ
    g.save(); g.beginPath(); g.rect(x - size * 0.3, y - size * 0.34, size * 0.6, size * 0.68); g.clip();
    g.translate(x, y); g.rotate(-e.roll);
    g.strokeStyle = GREEN; g.fillStyle = GREEN; g.lineWidth = 1.6; g.font = `13px ${FONT}`; g.textAlign = 'left'; g.textBaseline = 'middle';
    for (let p = -90; p <= 90; p += 5) {
      const yy = (e.pitch - p * DEG) * pp;
      if (Math.abs(yy) > size * 0.5) continue;
      const half = p === 0 ? size * 0.3 : p % 10 === 0 ? 70 : 35;
      g.setLineDash(p < 0 ? [8, 6] : []); g.beginPath(); g.moveTo(-half, yy); g.lineTo(-22, yy); g.moveTo(22, yy); g.lineTo(half, yy); g.stroke();
      if (p % 10 === 0 && p !== 0) g.fillText(String(p), half + 6, yy);
    }
    g.setLineDash([]); g.restore();
    // จุดแนวบิน (ทิศที่เครื่องกำลังไปจริง)
    const fx = x + Math.tan(ac.air.beta) * pp, fy = y + Math.tan(ac.air.alpha) * pp;
    g.strokeStyle = GREEN; g.lineWidth = 2; g.beginPath(); g.arc(fx, fy, 8, 0, 6.283); g.moveTo(fx - 20, fy); g.lineTo(fx - 8, fy); g.moveTo(fx + 8, fy); g.lineTo(fx + 20, fy); g.moveTo(fx, fy - 8); g.lineTo(fx, fy - 15); g.stroke();
    g.beginPath(); g.moveTo(x - 14, y); g.lineTo(x - 5, y); g.moveTo(x + 5, y); g.lineTo(x + 14, y); g.moveTo(x, y - 5); g.lineTo(x, y + 5); g.stroke();
  }

  map(x, y, size, ac, s, dt) {
    const g = this.g, R = size / 2, cx = x + R, cy = y + R, range = s.mapRange, k = R / range;
    this.trailT += dt;
    if (ac.phase === 'air' && this.trailT > 1) { this.trailT = 0; this.trail.push([ac.s.pos[0], ac.s.pos[1]]); if (this.trail.length > 900) this.trail.shift(); }
    g.save(); g.beginPath(); g.arc(cx, cy, R, 0, 6.283); g.clip();
    g.fillStyle = 'rgba(10,30,40,0.7)'; g.fillRect(x, y, size, size);
    const N = ac.s.pos[0], E = ac.s.pos[1], T = s.terrain, st = size / 26, base = T.real ? T.siteAlt : 0;   // แผนที่จริง: สีตามความสูงเหนือลานปล่อย
    for (let j = 0; j < 26; j++) for (let i = 0; i < 26; i++) {       // ภูมิประเทศย่อ (เหนืออยู่บน)
      const e = E + ((i + 0.5) * st - R) / k, n = N - ((j + 0.5) * st - R) / k, hh = T.height(n, e) - base;
      g.fillStyle = hh < 0 && !T.real ? 'rgba(30,90,130,0.8)' : hh < 150 ? 'rgba(70,110,60,0.8)' : hh < 500 ? 'rgba(110,110,70,0.85)' : 'rgba(150,140,130,0.9)';
      g.fillRect(x + i * st, y + j * st, st + 0.5, st + 0.5);
    }
    g.strokeStyle = 'rgba(255,255,255,0.8)'; g.lineWidth = 1.2; g.beginPath();
    this.trail.forEach(([n, e], i) => { const px = cx + (e - E) * k, py = cy - (n - N) * k; if (i) g.lineTo(px, py); else g.moveTo(px, py); });
    g.stroke();
    if (s.route && s.route.length) {                                   // เส้นทางภารกิจ: ฐาน → จุด 1 … (จุดเป้าปัจจุบันทึบ)
      g.strokeStyle = 'rgba(255,192,74,0.9)'; g.lineWidth = 1.5; g.beginPath(); g.moveTo(cx - E * k, cy + N * k);
      for (const p of s.route) g.lineTo(cx + (p.e - E) * k, cy - (p.n - N) * k);
      g.stroke();
      s.route.forEach((p, i) => { const px = cx + (p.e - E) * k, py = cy - (p.n - N) * k; g.beginPath(); g.arc(px, py, 4, 0, 6.283); g.fillStyle = i === s.routeIdx ? AMBER : 'rgba(10,20,28,0.9)'; g.fill(); g.stroke(); });
    }
    const hx = cx + (0 - E) * k, hy = cy - (0 - N) * k;
    g.fillStyle = AMBER; g.font = `16px ${FONT}`; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText('⌂', clamp(hx, x + 8, x + size - 8), clamp(hy, y + 8, y + size - 8));
    const yaw = ac.euler().yaw;
    g.translate(cx, cy); g.rotate(yaw); g.fillStyle = '#fff'; g.beginPath(); g.moveTo(0, -9); g.lineTo(6, 7); g.lineTo(0, 3); g.lineTo(-6, 7); g.fill();
    g.restore();
    g.strokeStyle = 'rgba(255,255,255,0.6)'; g.lineWidth = 1.5; g.beginPath(); g.arc(cx, cy, R, 0, 6.283); g.stroke();
    this.text('N', cx, y + 9, 12, '#fff', 'center', true);
    this.text(`${(range * 2 / 1000).toFixed(0)} km`, x + 4, y + size + 0, 11, '#cfe');
    this.text(`⌂ ${(s.homeDist / 1000).toFixed(2)} km`, cx, y - 12, 13, AMBER, 'center');
    if (s.coord) this.text(s.coord, cx, y - 27, 11, '#cfe', 'center');
    // ลม
    const wa = (s.wind.fromDeg + 180) * DEG;
    g.save(); g.translate(x - 26, cy); g.rotate(wa); g.strokeStyle = '#9fd4ff'; g.lineWidth = 2; g.beginPath(); g.moveTo(0, 12); g.lineTo(0, -12); g.lineTo(-5, -5); g.moveTo(0, -12); g.lineTo(5, -5); g.stroke(); g.restore();
    this.text(`ลม ${s.wind.speed.toFixed(0)} m/s`, x - 26, cy + 26, 11, '#9fd4ff', 'center');
    this.text(`จาก ${s.wind.fromDeg.toFixed(0)}°`, x - 26, cy + 40, 11, '#9fd4ff', 'center');
  }
}
export function fmtT(sec) { if (!isFinite(sec)) return '—'; const m = Math.floor(sec / 60), s = Math.floor(sec % 60); return `${m}:${String(s).padStart(2, '0')}`; }
void G;
