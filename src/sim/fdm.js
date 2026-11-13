// แบบจำลองการบิน 6-DOF ของ FW57J-B: อากาศพลศาสตร์ + เครื่องยนต์ + รางปล่อย + ร่ม + พื้น
// ข้อมูลทั้งหมดมาจาก data/fw57jb_sim.json (ส่งออกจาก spec) · ไม่ขึ้นกับเบราว์เซอร์
import { G, DEG, KMH, clamp, qrot, qrotInv, qnormalize, qFromEuler, eulerFromQ, cross, density, makeRng, fillFuel, fuelTotal, burnFuel, massProps } from './core.js';

// ======================= อากาศพลศาสตร์ =======================
/** แรง/โมเมนต์อากาศพลศาสตร์ในแกนตัวเครื่อง รอบ CG · va = ความเร็วเทียบอากาศ (แกนตัวเครื่อง) · dL,dR = มุม elevon (องศา, + = ชายหลังลง) */
export function aeroForces(data, mp, va, om, dL, dR, rho) {
  const A = data.aero;
  const u = va[0], v = va[1], w = va[2];
  const V = Math.hypot(u, v, w);
  const o = { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, alpha: 0, beta: 0, V, qbar: 0, CL: 0, CD: 0, sigma: 0 };
  if (V < 0.3) return o;
  const alpha = Math.atan2(w, u), beta = Math.asin(clamp(v / V, -1, 1));
  const qbar = 0.5 * rho * V * V, Vd = Math.max(V, 8);
  const ph = om[0] * A.b / (2 * Vd), qh = om[1] * A.MAC / (2 * Vd), rh = om[2] * A.b / (2 * Vd);
  const de = (dL + dR) / 2, da = (dL - dR) / 2;

  // ผสมช่วงกระแสเกาะผิว (เชิงเส้นจาก OpenVSP) กับช่วงหลังร่วงหล่น (แผ่นเรียบ)
  const M = A.stall_blend_M, as = A.alpha_stall_deg * DEG, an = A.alpha_stall_neg_deg * DEG;
  const sp = 1 / (1 + Math.exp(-M * (alpha - as))), sn = 1 / (1 + Math.exp(M * (alpha - an)));
  const sig = clamp(sp + sn - sp * sn, 0, 1);
  const sa = Math.sin(alpha), ca = Math.cos(alpha), sb = Math.sin(beta);
  const CLwbLin = A.CL0 + A.CLa * alpha;
  const CLwb = (1 - sig) * CLwbLin + sig * A.post_stall_CL_k * Math.sin(2 * alpha);
  const ctlEff = 1 - 0.5 * sig;
  const CLatt = CLwbLin + A.CLq * qh + A.CLde_per_deg * de;
  const CL = CLwb + (1 - sig) * A.CLq * qh + A.CLde_per_deg * de * ctlEff;
  const CD = (1 - sig) * (A.CD0 + A.K * CLatt * CLatt + A.CDde_per_deg2 * (dL * dL + dR * dR) / 2)
    + sig * (A.CD0 + A.post_stall_CD_k * sa * sa) + 0.6 * sb * sb;

  const arm = (A.NP_x - mp.x) / A.MAC;
  const CN = CLwb * ca + CD * sa;
  const Cm = (1 - sig) * A.Cm_ac + sig * A.post_stall_Cm * Math.sign(alpha) - CN * arm
    + (A.Cmde_per_deg + A.CLde_per_deg * (mp.x - A.x_ref) / A.MAC) * de * ctlEff + A.Cmq * qh;

  // ทางข้าง: Cnβ Clβ CYβ ขึ้นกับ CL (2 จุดจาก OpenVSP/CFD: บินเดินทาง กับ หลุดราง)
  const T0 = A.lat_table[0], T1 = A.lat_table[1];
  const CLc = clamp(CL, A.lat_CL_clamp[0], A.lat_CL_clamp[1]);
  const tt = (CLc - T0.CL) / (T1.CL - T0.CL);
  const Cnb = T0.Cnb + (T1.Cnb - T0.Cnb) * tt, Clb = T0.Clb + (T1.Clb - T0.Clb) * tt, CYb = T0.CYb + (T1.CYb - T0.CYb) * tt;
  const CY = CYb * beta + A.CYp * ph + A.CYr * rh;
  const Cl = Clb * beta + A.Clp * (1 - 0.8 * sig) * ph + A.Clr_per_CL * CLc * rh + A.Clda_per_deg * da * (1 - 0.6 * sig);
  const Cn = Cnb * beta + A.Cnp_per_CL * CLc * ph + (A.Cnr0 + A.Cnr_per_CL2 * CLc * CLc) * rh + A.Cnda_per_CL * CLc * A.Clda_per_deg * da;

  const qS = qbar * A.S;
  o.Fx = qS * (-CD * u / V + CL * sa);
  o.Fy = qS * (-CD * v / V + CY);
  o.Fz = qS * (-CD * w / V - CL * ca);
  o.Mx = qS * A.b * Cl; o.My = qS * A.MAC * Cm; o.Mz = qS * A.b * Cn;
  o.alpha = alpha; o.beta = beta; o.qbar = qbar; o.CL = CL; o.CD = CD; o.sigma = sig;
  return o;
}

// ======================= เครื่องยนต์ SW140B =======================
export function thrustFull(data, Vms) {
  const tb = data.engine.thrust_full_table_kmh_N, v = clamp(Vms * KMH, tb[0][0], tb[tb.length - 1][0]);
  for (let i = 1; i < tb.length; i++) if (v <= tb[i][0]) { const t = (v - tb[i - 1][0]) / (tb[i][0] - tb[i - 1][0]); return tb[i - 1][1] + (tb[i][1] - tb[i - 1][1]) * t; }
  return tb[tb.length - 1][1];
}

export class Engine {
  constructor(data) {
    this.E = data.engine;
    this.kExp = Math.log(this.E.idle_thrust_frac) / Math.log(this.E.n_idle);   // แรงขับ ∝ n^k
    this.state = 'off';            // off | start | run | cool
    this.n = 0; this.egt = 35; this.thrust = 0; this.thrustFrac = 0; this.fuelFlow = 0;   // fuelFlow kg/s
    this.tStart = 0; this.startTime = this.E.start_time_s; this.groundFullTime = 0; this.flameout = false;
  }
  get rpm() { return this.n * this.E.rpm_max; }
  get startStage() {
    if (this.state !== 'start') return '';
    const f = this.tStart / this.startTime;
    return f < 0.25 ? 'มอเตอร์สตาร์ตหมุน' : f < 0.5 ? 'จุดระเบิด' : 'เร่งเข้ารอบเดินเบา';
  }
  start(quick = false) {
    if (this.state === 'off' || this.state === 'cool') { this.state = 'start'; this.tStart = 0; this.startTime = quick ? this.E.quick_start_time_s : this.E.start_time_s; this.flameout = false; }
  }
  shutdown() { if (this.state !== 'off') this.state = 'cool'; }
  forceRunning(throttle = 0) { this.state = 'run'; this.n = this.E.n_idle + (1 - this.E.n_idle) * clamp(throttle, 0, 1); this.egt = 450; this.flameout = false; }
  /** คันเร่งที่ให้แรงขับ T ที่ความเร็ว V (ใช้ตั้งสมดุลและ feedforward ของ autopilot) */
  throttleFor(T, Tfull) {
    const f = clamp(T / Tfull, this.E.idle_thrust_frac, 1);
    return clamp((Math.pow(f, 1 / this.kExp) - this.E.n_idle) / (1 - this.E.n_idle), 0, 1);
  }
  update(dt, throttle, Vms, rhoRatio, hasFuel, oatC, Tfull) {
    const E = this.E;
    if ((this.state === 'run' || this.state === 'start') && !hasFuel) { this.state = 'cool'; this.flameout = true; }
    let egtT = oatC, tau = 8;
    if (this.state === 'start') {
      this.tStart += dt;
      const f = clamp(this.tStart / this.startTime, 0, 1);
      this.n = E.n_idle * f * f * (3 - 2 * f);
      egtT = f < 0.25 ? oatC : f < 0.6 ? 620 : 480; tau = 1.5;
      if (f >= 1) this.state = 'run';
    } else if (this.state === 'run') {
      const nt = E.n_idle + (1 - E.n_idle) * clamp(throttle, 0, 1);
      this.n += clamp((nt - this.n) / E.spool_tau_s, -E.spool_down_per_s, E.spool_up_per_s) * dt;
      tau = 2.0;
    } else {
      const nWind = clamp(0.03 * Vms / 50, 0, 0.06);            // ใบพัดหมุนตามลม
      this.n += clamp(nWind - this.n, -0.15 * dt, 0.15 * dt);
      if (this.state === 'cool' && this.n <= nWind + 1e-3) this.state = 'off';
    }
    const running = this.state === 'run';
    const f = running ? Math.pow(this.n, this.kExp) : 0;
    this.thrustFrac = f;
    this.thrust = f * Tfull * rhoRatio;
    if (running) {
      const ff = E.fuel_flow_idle_g_min + (E.fuel_flow_max_g_min - E.fuel_flow_idle_g_min) * clamp((f - E.idle_thrust_frac) / (1 - E.idle_thrust_frac), 0, 1);
      this.fuelFlow = ff / 60000;
      egtT = E.egt_range_C[0] + 20 + (E.egt_range_C[1] - E.egt_range_C[0] - 20) * Math.pow(f, 0.6);
    } else if (this.state === 'start' && this.tStart / this.startTime > 0.4) {
      this.fuelFlow = 0.6 * E.fuel_flow_idle_g_min / 60000;
    } else this.fuelFlow = 0;
    this.egt += (egtT - this.egt) * clamp(dt / tau, 0, 1);
  }
}

// ======================= รางปล่อยลมอัด L6 =======================
/** โปรไฟล์ความเร่งบนราง: วาล์วเปิด 0.25 s ถึงค่าสูงสุด แล้วลดตามระยะ (ลมขยายตัว) ให้ความเร็วออกรางตรงตามตาราง spec */
export function railProfile(data, tow) {
  const R = data.rail, tb = R.table;
  const Vexit = (R.V_launch_ref_kmh / KMH) * Math.sqrt(tow / R.tow_ref_kg);
  let i = 1; while (i < tb.length - 1 && tow > tb[i].tow) i++;
  const t = (tow - tb[i - 1].tow) / (tb[i].tow - tb[i - 1].tow);
  const aPeak = Math.max(3, tb[i - 1].peak_g + (tb[i].peak_g - tb[i - 1].peak_g) * t) * G;
  const sAcc = R.s_accel_m, tv = R.valve_open_s;
  const run = k => { let s = 0, v = 0, tt = 0; const dt = 2e-4; while (s < sAcc && tt < 5) { v += aPeak * Math.min(1, tt / tv) * Math.max(0.05, 1 - k * s / sAcc) * dt; s += v * dt; tt += dt; } return { v, t: tt }; };
  let lo = -1, hi = 0.95;
  for (let n = 0; n < 40; n++) { const mid = (lo + hi) / 2; if (run(mid).v > Vexit) lo = mid; else hi = mid; }
  const k = (lo + hi) / 2;
  return { Vexit, aPeak, k, tExit: run(k).t, accel: (s, tt) => aPeak * Math.min(1, tt / tv) * Math.max(0.05, 1 - k * s / sAcc) };
}

// ======================= สมดุลบินระดับ =======================
/** หา α, elevon, แรงขับ ที่บินระดับความเร็ว V · ok=false ถ้ายกไม่พอ (ต่ำกว่าความเร็วร่วงหล่น) หรือ elevon สุด */
export function trimLevel(data, mp, V, { rho }) {
  const W = mp.m * G, zt = mp.z - data.engine.thrust_axis_z, maxDe = data.aero.elevon_max_deg;
  const evalAt = (alpha, de) => {
    const a = aeroForces(data, mp, [V * Math.cos(alpha), 0, V * Math.sin(alpha)], [0, 0, 0], de, de, rho);
    const T = -a.Fx + W * Math.sin(alpha);
    return { a, T, My: a.My + T * zt, rz: a.Fz + W * Math.cos(alpha) };
  };
  const solveDe = alpha => {
    let de = 0;
    for (let i = 0; i < 14; i++) {
      const e0 = evalAt(alpha, de).My, slope = (evalAt(alpha, de + 0.1).My - e0) / 0.1;
      if (Math.abs(slope) < 1e-9) break;
      const step = clamp(-e0 / slope, -6, 6);
      de = clamp(de + step, -maxDe - 8, maxDe + 8);
      if (Math.abs(step) < 1e-6) break;
    }
    return de;
  };
  const resid = alpha => { const de = solveDe(alpha); const e = evalAt(alpha, de); return { r: e.rz, de, e, alpha }; };
  let prev = resid(-6 * DEG);
  for (let ad = -5.75; ad <= 32; ad += 0.25) {
    const cur = resid(ad * DEG);
    if (prev.r > 0 && cur.r <= 0) {
      let lo = prev, hi = cur;
      for (let i = 0; i < 30; i++) { const mid = resid((lo.alpha + hi.alpha) / 2); if (mid.r > 0) lo = mid; else hi = mid; }
      const a = hi.e.a;
      return { ok: Math.abs(hi.de) <= maxDe, alpha: hi.alpha, de: hi.de, thrust: hi.e.T, drag: a.qbar * data.aero.S * a.CD, CL: a.CL };
    }
    prev = cur;
  }
  return { ok: false };
}

// ======================= เครื่องบินทั้งลำ =======================
export class Aircraft {
  constructor(data, opts = {}) {
    this.data = data;
    this.cargoKg = opts.cargoKg ?? 0;
    this.oatC = opts.oatC ?? data.atmos.oat_C;
    this.fuel = fillFuel(data, this.cargoKg, opts.fuelFrac ?? 1);
    this.mp = massProps(data, this.cargoKg, this.fuel);
    this.wind = { speed: 0, fromDeg: 0, turb: 0, ...(opts.wind || {}) };
    this.terrain = opts.terrain ?? (() => 0);
    this.rng = makeRng(opts.seed ?? 7);
    this.engine = new Engine(data);
    this.ctl = { dL: 0, dR: 0, throttle: 0 };          // คำสั่ง (องศา, 0–1)
    this.dL = 0; this.dR = 0;                          // มุม elevon จริง (หลัง servo)
    this.s = { pos: [0, 0, 0], vb: [0, 0, 0], q: [1, 0, 0, 0], om: [0, 0, 0] };
    this.chute = { state: 'stowed', t: 0, tFill: 0, cds: 0, force: 0, forceMax: 0 };
    this.phase = 'rail';                               // rail | air | landed | crashed
    this.rail = null;
    this.t = 0; this.tAir = 0;
    this.gust = [0, 0, 0]; this.windNow = [0, 0, 0];
    this.air = { V: 0, alpha: 0, beta: 0, qbar: 0, rho: density(0, this.oatC), CL: 0, sigma: 0 };
    this.nz = 1; this.nx = 0; this.touchdown = null; this.crashReason = null;
    this._overT = 0; this._mpTick = 0; this._surf = 0;
    this._y = new Float64Array(13); this._k = [0, 1, 2, 3].map(() => new Float64Array(13)); this._yt = new Float64Array(13);
    this.initRail({});
  }
  get alt() { return -this.s.pos[2]; }
  get agl() { return this.alt - this._surf; }
  euler() { return eulerFromQ(this.s.q); }
  vNED() { return qrot(this.s.q, this.s.vb); }
  airdata() {
    const wB = qrotInv(this.s.q, this.windNow), vb = this.s.vb;
    const va = [vb[0] - wB[0], vb[1] - wB[1], vb[2] - wB[2]], V = Math.hypot(va[0], va[1], va[2]);
    return { V, alpha: V > 0.3 ? Math.atan2(va[2], va[0]) : 0, beta: V > 0.3 ? Math.asin(clamp(va[1] / V, -1, 1)) : 0 };
  }

  // ---------- ตั้งสถานะเริ่ม ----------
  initRail({ heading = 0 } = {}) {
    const R = this.data.rail, e = R.elev_deg * DEG, h = heading * DEG;
    const ground = Math.max(this.terrain(0, 0), 0);
    this._surf = ground;
    this.s.pos = [0, 0, -(ground + R.carriage_height_m)];
    this.s.vb = [0, 0, 0]; this.s.om = [0, 0, 0];
    this.s.q = qFromEuler(0, e, h);
    this.rail = { launched: false, s: 0, v: 0, t: 0, p0: [...this.s.pos], dir: [Math.cos(e) * Math.cos(h), Math.cos(e) * Math.sin(h), -Math.sin(e)], profile: null, heading, peakG: 0 };
    this.phase = 'rail';
    this.dL = this.dR = this.ctl.dL = this.ctl.dR = R.elevon_preset_deg;
    this.nz = Math.cos(e); this.nx = Math.sin(e);
  }
  launch() {
    if (this.phase !== 'rail' || this.rail.launched) return false;
    this.mp = massProps(this.data, this.cargoKg, this.fuel);
    this.rail.profile = railProfile(this.data, this.mp.m);
    this.rail.launched = true;
    return true;
  }
  initAir({ V, alt, heading = 0, gamma = 0 }) {
    const rho = density(alt, this.oatC);
    const tr = trimLevel(this.data, this.mp, V, { rho });
    const alpha = tr.ok ? tr.alpha : 8 * DEG, de = tr.ok ? tr.de : -3;
    this.s.pos = [0, 0, -alt];
    this.s.vb = [V * Math.cos(alpha), 0, V * Math.sin(alpha)];
    this.s.om = [0, 0, 0];
    this.s.q = qFromEuler(0, alpha + gamma * DEG, heading * DEG);
    this.dL = this.dR = this.ctl.dL = this.ctl.dR = de;
    const Tf = thrustFull(this.data, V) * rho / density(0, this.oatC);
    this.ctl.throttle = this.engine.throttleFor(tr.ok ? tr.thrust : 60, Tf);
    this.engine.forceRunning(this.ctl.throttle);
    this.phase = 'air'; this.tAir = 0; this.rail = null;
    this._surf = Math.max(this.terrain(0, 0), 0);
    this.air = { ...this.air, ...this.airdata(), rho };
    return tr;
  }
  deployChute() {
    if (this.phase !== 'air' || this.chute.state !== 'stowed') return false;
    const C = this.data.chute;
    this.chute.state = 'inflating'; this.chute.t = 0;
    this.chute.tFill = clamp(C.fill_const_m / Math.max(this.air.V, 5), 1.2, 6);
    this.chute.deployV = this.air.V;
    return true;
  }

  // ---------- ก้าวเวลา ----------
  step(dt) {
    this.t += dt;
    if (this.phase === 'landed' || this.phase === 'crashed') { this.engine.update(dt, 0, 0, 1, false, this.oatC, 0); return; }
    this._surf = Math.max(this.terrain(this.s.pos[0], this.s.pos[1]), 0);
    // servo elevon: จำกัดอัตรา 160°/s และมุม ±22.5°
    const mx = this.data.aero.elevon_max_deg, rate = 160 * dt;
    this.dL += clamp(clamp(this.ctl.dL, -mx, mx) - this.dL, -rate, rate);
    this.dR += clamp(clamp(this.ctl.dR, -mx, mx) - this.dR, -rate, rate);
    this._updateWind(dt);
    const rho = density(this.alt, this.oatC), rho0 = density(0, this.oatC);
    const eng = this.engine;
    eng.update(dt, this.ctl.throttle, this.air.V, rho / rho0, fuelTotal(this.fuel) > 1e-6, this.oatC, thrustFull(this.data, this.air.V));
    if (eng.fuelFlow > 0) {
      burnFuel(this.data, this.fuel, eng.fuelFlow * dt);
      if (++this._mpTick >= 48) { this._mpTick = 0; this.mp = massProps(this.data, this.cargoKg, this.fuel); }
    }
    if (this.phase === 'rail') {
      if (eng.thrustFrac > 0.9) eng.groundFullTime += dt;
      this._stepRail(dt, rho);
    } else this._stepAir(dt, rho);
  }

  _updateWind(dt) {
    const w = this.wind, f = clamp(Math.pow(Math.max(this.agl, 2) / 10, 0.143), 0.7, 1.6), d = w.fromDeg * DEG;
    if (w.turb > 0) {
      const tau = clamp(150 / Math.max(this.air.V, 10), 0.5, 8), k = Math.sqrt(2 * dt / tau) * w.turb;
      for (let i = 0; i < 3; i++) this.gust[i] += -this.gust[i] * dt / tau + k * this.rng.normal() * (i === 2 ? 0.7 : 1);
    } else this.gust[0] = this.gust[1] = this.gust[2] = 0;
    this.windNow = [-w.speed * f * Math.cos(d) + this.gust[0], -w.speed * f * Math.sin(d) + this.gust[1], this.gust[2]];
  }

  _stepRail(dt, rho) {
    const r = this.rail, R = this.data.rail;
    if (r.launched) {
      const a = r.profile.accel(r.s, r.t);
      r.v += a * dt; r.s += r.v * dt; r.t += dt;
      r.peakG = Math.max(r.peakG, a / G);
      this.nx = a / G + Math.sin(R.elev_deg * DEG);
      this.s.pos = [r.p0[0] + r.dir[0] * r.s, r.p0[1] + r.dir[1] * r.s, r.p0[2] + r.dir[2] * r.s];
      this.s.vb = [r.v, 0, 0];
      if (r.s >= R.s_accel_m) { this.phase = 'air'; this.tAir = 0; }
    }
    const ad = this.airdata();
    this.air = { ...this.air, ...ad, rho, qbar: 0.5 * rho * ad.V * ad.V };
  }

  _deriv(y, dy, rho, rec) {
    const q = [y[6], y[7], y[8], y[9]], vb = [y[3], y[4], y[5]], om = [y[10], y[11], y[12]], mp = this.mp;
    const wB = qrotInv(q, this.windNow);
    const va = [vb[0] - wB[0], vb[1] - wB[1], vb[2] - wB[2]];
    const A = aeroForces(this.data, mp, va, om, this.dL, this.dR, rho);
    const T = this.engine.thrust;
    let Fx = A.Fx + T, Fy = A.Fy, Fz = A.Fz;
    let Mx = A.Mx, My = A.My + T * (mp.z - this.data.engine.thrust_axis_z), Mz = A.Mz;
    let Fc = 0;
    if (this.chute.cds > 0) {
      // ร่ม: แรงต้านตามทิศลมสัมพัทธ์ที่จุดรวมสายเหนือ CG + หน่วงการแกว่งจากสายโยง
      const r = [0, 0, -this.data.chute.attach_above_cg_m];
      const wxr = cross(om, r), vt = [va[0] + wxr[0], va[1] + wxr[1], va[2] + wxr[2]];
      const sp = Math.hypot(vt[0], vt[1], vt[2]), k = -0.5 * rho * sp * this.chute.cds;
      const F = [k * vt[0], k * vt[1], k * vt[2]];
      Fc = Math.hypot(F[0], F[1], F[2]);
      const Mc = cross(r, F), cd = 0.12 * Fc;
      Fx += F[0]; Fy += F[1]; Fz += F[2];
      Mx += Mc[0] - cd * om[0]; My += Mc[1] - cd * om[1]; Mz += Mc[2] - cd * om[2];
    }
    const gb = qrotInv(q, [0, 0, G]);
    dy[3] = Fx / mp.m + gb[0] - (om[1] * vb[2] - om[2] * vb[1]);
    dy[4] = Fy / mp.m + gb[1] - (om[2] * vb[0] - om[0] * vb[2]);
    dy[5] = Fz / mp.m + gb[2] - (om[0] * vb[1] - om[1] * vb[0]);
    dy[10] = (Mx - (mp.Izz - mp.Iyy) * om[1] * om[2]) / mp.Ixx;
    dy[11] = (My - (mp.Ixx - mp.Izz) * om[2] * om[0]) / mp.Iyy;
    dy[12] = (Mz - (mp.Iyy - mp.Ixx) * om[0] * om[1]) / mp.Izz;
    const vn = qrot(q, vb);
    dy[0] = vn[0]; dy[1] = vn[1]; dy[2] = vn[2];
    dy[6] = 0.5 * (-q[1] * om[0] - q[2] * om[1] - q[3] * om[2]);
    dy[7] = 0.5 * (q[0] * om[0] + q[2] * om[2] - q[3] * om[1]);
    dy[8] = 0.5 * (q[0] * om[1] + q[3] * om[0] - q[1] * om[2]);
    dy[9] = 0.5 * (q[0] * om[2] + q[1] * om[1] - q[2] * om[0]);
    if (rec) { rec.A = A; rec.Fc = Fc; rec.nz = -Fz / (mp.m * G); rec.nx = Fx / (mp.m * G); }
  }

  _stepAir(dt, rho) {
    const s = this.s, y = this._y, k = this._k, yt = this._yt, C = this.data.chute, ch = this.chute;
    this.tAir += dt;
    if (ch.state === 'inflating') {
      ch.t += dt;
      ch.cds = C.CDS_m2 * Math.min(1, (ch.t / ch.tFill) ** 2);
      if (ch.t >= ch.tFill) ch.state = 'open';
    }
    y.set(s.pos, 0); y.set(s.vb, 3); y.set(s.q, 6); y.set(s.om, 10);
    const rec = {};
    this._deriv(y, k[0], rho, rec);
    for (let i = 0; i < 13; i++) yt[i] = y[i] + 0.5 * dt * k[0][i];
    this._deriv(yt, k[1], rho);
    for (let i = 0; i < 13; i++) yt[i] = y[i] + 0.5 * dt * k[1][i];
    this._deriv(yt, k[2], rho);
    for (let i = 0; i < 13; i++) yt[i] = y[i] + dt * k[2][i];
    this._deriv(yt, k[3], rho);
    for (let i = 0; i < 13; i++) y[i] += dt / 6 * (k[0][i] + 2 * k[1][i] + 2 * k[2][i] + k[3][i]);
    s.pos = [y[0], y[1], y[2]]; s.vb = [y[3], y[4], y[5]]; s.q = qnormalize([y[6], y[7], y[8], y[9]]); s.om = [y[10], y[11], y[12]];

    const A = rec.A;
    this.air = { V: A.V, alpha: A.alpha, beta: A.beta, qbar: A.qbar, rho, CL: A.CL, sigma: A.sigma };
    this.nz = rec.nz; this.nx = rec.nx;
    ch.force = rec.Fc; ch.forceMax = Math.max(ch.forceMax, rec.Fc);
    if (rec.Fc > C.F_ult_N && ch.state !== 'failed') { ch.state = 'failed'; ch.cds = 0; }

    // โครงสร้าง: เกิน 1.5 × n_limit ต่อเนื่อง → เสียหาย (ไม่นับช่วงร่มกระตุก)
    if (ch.state === 'stowed' && Math.abs(this.nz) > this.data.limits.n_ultimate) {
      this._overT += dt;
      if (this._overT > 0.15) return this._end('crashed', 'โครงสร้างรับแรงเกินค่าประลัย');
    } else this._overT = 0;

    // แตะพื้น (ไม่มีล้อ: ลงด้วยร่มเท่านั้น · ลงท้องได้ถ้านุ่มมาก)
    const ground = this.terrain(s.pos[0], s.pos[1]), surf = Math.max(ground, 0);
    this._surf = surf;
    if (this.alt - surf <= 0.12) {
      const vn = this.vNED(), vz = vn[2], vh = Math.hypot(vn[0], vn[1]), e = this.euler();
      const chuteOK = ch.state === 'open' || (ch.state === 'inflating' && ch.cds > 0.5 * C.CDS_m2);
      this.touchdown = { vz, vh, water: ground < 0, chute: chuteOK, roll: e.roll, pitch: e.pitch };
      s.pos[2] = -(surf + 0.12);
      if (chuteOK && vz < 8) { this.touchdown.type = 'chute'; return this._end('landed'); }
      if (vz < 2.5 && vh < 45 && Math.abs(e.roll) < 20 * DEG && e.pitch > -8 * DEG && e.pitch < 20 * DEG) { this.touchdown.type = 'belly'; return this._end('landed'); }
      return this._end('crashed', ground < 0 ? 'ตกน้ำ' : 'ชนพื้น');
    }
  }

  _end(phase, reason = null) {
    this.phase = phase; this.crashReason = reason;
    this.s.vb = [0, 0, 0]; this.s.om = [0, 0, 0];
    this.engine.shutdown(); this.ctl.throttle = 0;
    if (this.chute.state === 'open' || this.chute.state === 'inflating') this.chute.state = 'collapsed';
    this.chute.cds = 0; this.chute.force = 0;
    this.air = { ...this.air, V: 0, alpha: 0, beta: 0, qbar: 0, sigma: 0 };
    this.nz = 1; this.nx = 0;
  }
}
