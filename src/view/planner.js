// หน้าวางแผนภารกิจ (เหมือนโปรแกรมสถานีภาคพื้น): แผนที่เลื่อน/ซูมได้ คลิกวางจุดบิน ตั้งความสูงรายจุด
// แสดงระยะ ทิศ เวลา น้ำมัน และหน้าตัดภูมิประเทศตามเส้นทาง (เตือนช่วงที่พ้นพื้นน้อยกว่า 60 m)
// จุดบินเก็บเป็น { n, e, h } โดย h = ความสูงเหนือลานปล่อย (m) · mission() แปลงเป็นความสูงเหนือระดับน้ำทะเลให้ autopilot
import { missionStats } from '../sim/mission.js';

const $ = id => document.getElementById(id);
const MIN_CLEAR = 60, AMBER = '#ffc04a', RED = '#ff5d4d';

export class Planner {
  /** onChange() เรียกทุกครั้งที่แผนเปลี่ยน · estimate(speedKmh, distM) → { min, fuelKg, fuelAvailKg } */
  constructor({ onChange, estimate }) {
    this.onChange = onChange; this.estimate = estimate;
    this.el = $('planner'); this.cv = $('planMap'); this.pf = $('planProf');
    this.wps = []; this.spd = 200; this.T = null; this.home = { n: 0, e: 0, alt: 0 };
    this.view = { n: 0, e: 0, k: 0.02 }; this.imgs = new Map(); this.acPos = null; this.stats = null; this.opened = false;
    let down = null;
    const at = ev => { const r = this.cv.getBoundingClientRect(); return [ev.clientX - r.left, ev.clientY - r.top]; };
    const hit = (x, y) => this.wps.findIndex(w => Math.hypot(this.X(w.e) - x, this.Y(w.n) - y) < 15);
    this.cv.addEventListener('pointerdown', ev => { const [x, y] = at(ev); down = { x, y, wp: hit(x, y), moved: 0, btn: ev.button }; this.cv.setPointerCapture(ev.pointerId); });
    this.cv.addEventListener('pointermove', ev => {
      if (!down || down.btn !== 0) return;
      const [x, y] = at(ev), dx = x - down.x, dy = y - down.y; down.moved += Math.abs(dx) + Math.abs(dy); down.x = x; down.y = y;
      if (down.wp >= 0) { const w = this.wps[down.wp]; w.e += dx / this.view.k; w.n -= dy / this.view.k; this.refresh(false); }
      else { this.view.e -= dx / this.view.k; this.view.n += dy / this.view.k; this.draw(); }
    });
    this.cv.addEventListener('pointerup', ev => {
      if (!down) return;
      const [x, y] = at(ev), d = down; down = null;
      if (d.btn === 0 && d.wp < 0 && d.moved < 5) this.wps.push({ n: this.N(y), e: this.E(x), h: this.wps.length ? this.wps[this.wps.length - 1].h : 300 });
      if (d.btn === 0 && (d.wp >= 0 || d.moved < 5)) this.refresh(true);
    });
    this.cv.addEventListener('contextmenu', ev => { ev.preventDefault(); const [x, y] = at(ev), i = hit(x, y); if (i >= 0) { this.wps.splice(i, 1); this.refresh(true); } });
    this.cv.addEventListener('wheel', ev => {
      ev.preventDefault(); ev.stopPropagation();
      const [x, y] = at(ev), n = this.N(y), e = this.E(x);
      this.view.k = Math.min(Math.max(this.view.k * (ev.deltaY > 0 ? 0.8 : 1.25), 0.0015), 0.6);
      this.view.e = e - (x - this.w / 2) / this.view.k; this.view.n = n + (y - this.h / 2) / this.view.k; this.draw();
    }, { passive: false });
    $('planList').addEventListener('input', ev => { const i = +ev.target.dataset.i; if (ev.target.matches('input') && this.wps[i]) { this.wps[i].h = Math.min(Math.max(+ev.target.value || 0, 30), 4000); this.refresh(false); this.save(); this.onChange(); } });
    $('planList').addEventListener('click', ev => { if (ev.target.matches('button[data-del]')) { this.wps.splice(+ev.target.dataset.del, 1); this.refresh(true); } });
    $('planSpd').addEventListener('input', () => { this.spd = +$('planSpd').value; this.info(); this.save(); this.onChange(); });
    $('planClear').addEventListener('click', () => { this.wps = []; this.refresh(true); });
    $('planFit').addEventListener('click', () => { this.fit(); this.draw(); });
    window.addEventListener('resize', () => { if (this.opened) { this.resize(); this.draw(); this.profile(); } });
  }

  X(e) { return this.w / 2 + (e - this.view.e) * this.view.k; }
  Y(n) { return this.h / 2 - (n - this.view.n) * this.view.k; }
  E(x) { return this.view.e + (x - this.w / 2) / this.view.k; }
  N(y) { return this.view.n - (y - this.h / 2) / this.view.k; }
  key() { return 'fw57jb.mission.' + (this.T.meta?.id ?? 'procedural'); }

  /** จุดบินสำหรับ autopilot: ความสูงเหนือระดับน้ำทะเล */
  mission() { return this.wps.map(w => ({ n: w.n, e: w.e, alt: this.home.alt + w.h })); }

  setTerrain(T) {
    this.T = T; this.home = { n: 0, e: 0, alt: T.siteAlt }; this.imgs.clear(); this.wps = []; this.spd = 200; this.fitted = false;
    try { const s = JSON.parse(localStorage.getItem(this.key()) || 'null'); if (s && Array.isArray(s.wps)) { this.wps = s.wps.filter(w => isFinite(w.n) && isFinite(w.e) && isFinite(w.h)); this.spd = s.spd || 200; } } catch { /* ไม่มีที่เก็บ = เริ่มแผนว่าง */ }
    $('planSpd').value = this.spd; this.stats = missionStats(this.mission(), this.home, T.height);
  }
  save() { try { localStorage.setItem(this.key(), JSON.stringify({ spd: this.spd, wps: this.wps })); } catch { /* เก็บไม่ได้ก็ยังใช้ในรอบนี้ได้ */ } }

  open(acPos = null) {
    this.acPos = acPos; this.opened = true; this.el.classList.remove('hide'); this.resize();
    if (!this.fitted) { this.fit(); this.fitted = true; }
    this.refresh(false); this.list();
  }
  close() { this.opened = false; this.el.classList.add('hide'); }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    for (const c of [this.cv, this.pf]) { const r = c.getBoundingClientRect(); c.width = Math.max(2, Math.round(r.width * dpr)); c.height = Math.max(2, Math.round(r.height * dpr)); c.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0); }
    const r = this.cv.getBoundingClientRect(); this.w = r.width; this.h = r.height;
  }
  fit() {
    const pts = [this.home, ...this.wps, ...(this.acPos ? [{ n: this.acPos[0], e: this.acPos[1] }] : [])];
    let n0 = 1e9, n1 = -1e9, e0 = 1e9, e1 = -1e9;
    for (const p of pts) { n0 = Math.min(n0, p.n); n1 = Math.max(n1, p.n); e0 = Math.min(e0, p.e); e1 = Math.max(e1, p.e); }
    const span = Math.max(n1 - n0, e1 - e0, this.wps.length ? 4000 : 30000) * 1.3;
    this.view = { n: (n0 + n1) / 2, e: (e0 + e1) / 2, k: Math.min(this.w, this.h) / span };
  }

  /** คำนวณสรุปใหม่แล้ววาด · structural = จำนวน/ลำดับจุดเปลี่ยน (สร้างรายการใหม่ บันทึก แจ้งตัวเกม) */
  refresh(structural) {
    this.stats = missionStats(this.mission(), this.home, this.T.height);
    if (structural) { this.list(); this.save(); this.onChange(); } else this.legCells();
    this.info(); this.draw(); this.profile();
  }

  list() {
    $('planList').innerHTML = this.wps.length ? this.wps.map((w, i) => `<div class="wp"><b>${i + 1}</b><label>สูงจากลาน <input type="number" data-i="${i}" min="30" max="4000" step="50" value="${Math.round(w.h)}"> m</label><span id="leg${i}"></span><button class="ghost" data-del="${i}" title="ลบจุดนี้">✕</button></div>`).join('')
      : '<p class="note">ยังไม่มีจุดบิน — คลิกบนแผนที่เพื่อวางจุดแรก</p>';
    this.legCells();
  }
  legCells() {
    this.stats.legs.slice(0, this.wps.length).forEach((l, i) => {
      const el = $('leg' + i); if (!el) return;
      el.textContent = `${(l.dist / 1000).toFixed(1)} km · ${l.brg.toFixed(0).padStart(3, '0')}° · พ้นพื้น ${isFinite(l.minClear) ? l.minClear.toFixed(0) + ' m' : '—'}`;
      el.className = l.minClear < MIN_CLEAR ? 'bad' : '';
    });
  }
  info() {
    const st = this.stats, row = (a, b, bad) => `<div><b>${a}</b><span class="${bad ? 'bad' : ''}">${b}</span></div>`;
    $('planSpdV').textContent = this.spd + ' km/h';
    if (!this.wps.length) { $('planInfo').innerHTML = ''; return; }
    const est = this.estimate(this.spd, st.total), back = st.legs[st.legs.length - 1], top = Math.max(...this.wps.map(w => w.h)) + this.home.alt;
    $('planInfo').innerHTML = row('ระยะรวม (ไป–กลับฐาน)', (st.total / 1000).toFixed(1) + ' km')
      + row('ขากลับฐาน', `${(back.dist / 1000).toFixed(1)} km · ${back.brg.toFixed(0).padStart(3, '0')}° · พ้นพื้น ${isFinite(back.minClear) ? back.minClear.toFixed(0) + ' m' : '—'}`, back.minClear < MIN_CLEAR)
      + row('เวลาบินประมาณ', `${Math.floor(est.min)} นาที ที่ ${this.spd} km/h`)
      + row('น้ำมันที่ใช้ประมาณ', `${est.fuelKg.toFixed(1)} kg จาก ${est.fuelAvailKg.toFixed(1)} kg`, est.fuelKg > 0.85 * est.fuelAvailKg)
      + row('พ้นพื้นต่ำสุด', isFinite(st.minClear) ? st.minClear.toFixed(0) + ' m' + (st.minClear < MIN_CLEAR ? ' — ต่ำเกินไป เพิ่มความสูง' : '') : '—', st.minClear < MIN_CLEAR)
      + row('จุดสูงสุดของแผน', top.toFixed(0) + ' m เหนือระดับน้ำทะเล');
  }

  // ---------- แผนที่ ----------
  tileImg(L, ix, iz) {
    const key = L + '/' + ix + '_' + iz; let im = this.imgs.get(key);
    if (!im) { if (this.imgs.size > 400) this.imgs.clear(); im = new Image(); im.onload = () => { if (this.opened) this.draw(); }; im.src = `${this.T.meta.url}/${this.T.meta.img.path}/${key}.jpg`; this.imgs.set(key, im); }
    return im;
  }
  base(g) {
    const T = this.T, { k } = this.view, w = this.w, h = this.h;
    g.fillStyle = '#0d1a22'; g.fillRect(0, 0, w, h);
    if (T.real) {
      const im = T.meta.img, lv = im.levels;
      let L = Math.min(Math.max(Math.ceil(Math.log2(64 * k)), 0), lv.length - 1);
      const count = l => { const S = im.root / (1 << l); return (w / k / S + 1) * (h / k / S + 1); };
      while (L > 0 && count(L) > 140) L--;
      for (let l = 0; l <= L; l += Math.max(L, 1)) {                 // ชั้นหยาบสุดรองพื้น แล้วทับด้วยชั้นที่เลือก
        const S = im.root / (1 << l), r = lv[l];
        const x0 = Math.max(Math.floor(this.E(0) / S), r.x0), x1 = Math.min(Math.floor(this.E(w) / S), r.x1);
        const z0 = Math.max(Math.floor(-this.N(0) / S), r.z0), z1 = Math.min(Math.floor(-this.N(h) / S), r.z1);
        for (let iz = z0; iz <= z1; iz++) for (let ix = x0; ix <= x1; ix++) {
          const t = this.tileImg(l, ix, iz);
          if (t.complete && t.naturalWidth) g.drawImage(t, this.X(ix * S), this.Y(-iz * S), S * k + 0.6, S * k + 0.6);
        }
      }
      g.strokeStyle = 'rgba(255,255,255,0.5)'; g.setLineDash([6, 6]); g.strokeRect(this.X(-T.half), this.Y(T.half), 2 * T.half * k, 2 * T.half * k); g.setLineDash([]);
    } else {
      const c = 8, base = T.siteAlt;
      for (let y = 0; y < h; y += c) for (let x = 0; x < w; x += c) {
        const hh = T.height(this.N(y + c / 2), this.E(x + c / 2));
        const v = Math.min(1, Math.max(0, (hh - base) / 1500));
        g.fillStyle = hh < 0 ? '#1d5e7e' : hh < 2.5 ? '#c9b97f' : `rgb(${70 + 110 * v | 0},${118 - 10 * v | 0},${62 + 80 * v | 0})`;
        g.fillRect(x, y, c, c);
      }
    }
  }
  draw() {
    if (!this.opened) return;
    const g = this.cv.getContext('2d'), w = this.w, h = this.h, { k } = this.view, st = this.stats;
    this.base(g);
    const txt = (s, x, y, color = '#fff', size = 12, align = 'center') => { g.font = `600 ${size}px system-ui, sans-serif`; g.textAlign = align; g.textBaseline = 'middle'; g.lineWidth = 3; g.strokeStyle = 'rgba(0,0,0,0.75)'; g.strokeText(s, x, y); g.fillStyle = color; g.fillText(s, x, y); };
    const hx = this.X(0), hy = this.Y(0);
    for (const km of [10, 25, 50]) {                                   // วงระยะจากลานปล่อย
      const r = km * 1000 * k; if (r < 30 || r > 3 * Math.max(w, h)) continue;
      g.strokeStyle = 'rgba(255,255,255,0.35)'; g.lineWidth = 1; g.setLineDash([4, 6]); g.beginPath(); g.arc(hx, hy, r, 0, 6.283); g.stroke(); g.setLineDash([]);
      txt(km + ' km', hx, hy - r - 8, 'rgba(255,255,255,0.8)', 11);
    }
    const pts = [this.home, ...this.wps, this.home];
    if (this.wps.length) for (let i = 0; i + 1 < pts.length; i++) {      // ช่วงแดง = พ้นพื้นน้อยกว่า 60 m
      const a = pts[i], b = pts[i + 1], leg = st.legs[i], ret = i === pts.length - 2;
      g.strokeStyle = leg && leg.minClear < MIN_CLEAR ? RED : ret ? 'rgba(255,255,255,0.75)' : AMBER; g.lineWidth = ret ? 1.5 : 2.5; g.setLineDash(ret ? [8, 6] : []);
      g.beginPath(); g.moveTo(this.X(a.e), this.Y(a.n)); g.lineTo(this.X(b.e), this.Y(b.n)); g.stroke(); g.setLineDash([]);
    }
    this.wps.forEach((p, i) => {
      const x = this.X(p.e), y = this.Y(p.n);
      g.fillStyle = 'rgba(10,20,28,0.85)'; g.strokeStyle = AMBER; g.lineWidth = 2; g.beginPath(); g.arc(x, y, 11, 0, 6.283); g.fill(); g.stroke();
      txt(String(i + 1), x, y + 0.5, '#fff', 12); txt(`+${Math.round(p.h)} m`, x, y + 22, AMBER, 11);
    });
    txt('⌂', hx, hy, '#7fe3ff', 20);
    if (this.acPos) { const x = this.X(this.acPos[1]), y = this.Y(this.acPos[0]); g.save(); g.translate(x, y); g.rotate(this.acPos[2] || 0); g.fillStyle = '#fff'; g.strokeStyle = '#000'; g.lineWidth = 1; g.beginPath(); g.moveTo(0, -11); g.lineTo(7, 8); g.lineTo(0, 4); g.lineTo(-7, 8); g.closePath(); g.fill(); g.stroke(); g.restore(); }
    // มาตราส่วน + ทิศเหนือ + ที่มาภาพ
    const nice = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000].find(m => m * k > 70) ?? 50000;
    g.strokeStyle = '#fff'; g.lineWidth = 2; g.beginPath(); g.moveTo(14, h - 16); g.lineTo(14 + nice * k, h - 16); g.stroke();
    txt(nice >= 1000 ? nice / 1000 + ' km' : nice + ' m', 14 + nice * k / 2, h - 28, '#fff', 11); txt('N ↑', w - 26, 18, '#fff', 13);
    if (this.T.real) txt('ภาพ: Sentinel-2 cloudless 2023 © EOX (CC BY-NC-SA 4.0)', w - 8, h - 10, 'rgba(255,255,255,0.8)', 10, 'right');
  }

  // ---------- หน้าตัดเส้นทาง: พื้น (ทึบ) กับความสูงที่วางแผน (เส้น) ----------
  profile() {
    const g = this.pf.getContext('2d'), r = this.pf.getBoundingClientRect(), w = r.width, h = r.height, p = this.stats.profile;
    g.clearRect(0, 0, w, h); g.fillStyle = '#0d1a22'; g.fillRect(0, 0, w, h);
    g.font = '11px system-ui, sans-serif'; g.textBaseline = 'middle';
    if (p.length < 2) { g.fillStyle = '#8aa'; g.textAlign = 'center'; g.fillText('หน้าตัดเส้นทางจะแสดงเมื่อมีจุดบิน', w / 2, h / 2); return; }
    let lo = 1e9, hi = -1e9; for (const q of p) { lo = Math.min(lo, q.ground); hi = Math.max(hi, q.ground, q.alt); }
    lo = Math.max(lo - 30, -50); hi += 60;
    const X = s => 44 + (w - 54) * s / this.stats.total, Y = a => h - 14 - (h - 24) * (a - lo) / (hi - lo);
    g.fillStyle = '#3d5a3a'; g.beginPath(); g.moveTo(X(0), Y(lo)); for (const q of p) g.lineTo(X(q.s), Y(q.ground)); g.lineTo(X(this.stats.total), Y(lo)); g.fill();
    for (let i = 1; i < p.length; i++) { g.strokeStyle = Math.min(p[i].alt - p[i].ground, p[i - 1].alt - p[i - 1].ground) < MIN_CLEAR ? RED : AMBER; g.lineWidth = 2; g.beginPath(); g.moveTo(X(p[i - 1].s), Y(p[i - 1].alt)); g.lineTo(X(p[i].s), Y(p[i].alt)); g.stroke(); }
    let s = 0; g.textAlign = 'center';
    this.stats.legs.forEach((l, i) => { s += l.dist; const x = X(s); g.strokeStyle = 'rgba(255,255,255,0.3)'; g.lineWidth = 1; g.beginPath(); g.moveTo(x, 8); g.lineTo(x, h - 14); g.stroke(); g.fillStyle = '#fff'; g.fillText(i < this.wps.length ? String(i + 1) : '⌂', x, 7); });
    g.fillStyle = '#9bb'; g.textAlign = 'right'; g.fillText(hi.toFixed(0) + ' m', 40, 12); g.fillText(lo.toFixed(0) + ' m', 40, h - 16);
    g.textAlign = 'center'; g.fillText((this.stats.total / 1000).toFixed(1) + ' km', w - 30, h - 6); g.fillText('0', 44, h - 6);
  }
}
