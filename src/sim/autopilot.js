// ระบบควบคุมการบิน (เลียนแบบโหมดของ autopilot UAV ปีกตรึง)
//   MANUAL  คันบังคับ → elevon ตรง ๆ (มี trim)
//   FBWA    คันบังคับ → มุมเอียง/มุมก้มเงยเป้า · ปล่อยมือ = ปีกระดับ บินระดับโดยประมาณ · มีกันร่วงหล่นและจำกัด g
//   CRUISE  รักษาความสูง ทิศ ความเร็ว · คันบังคับเลื่อนค่าเป้า · คันเร่งอัตโนมัติ
//   AUTO    บินตามจุดในภารกิจตามลำดับ (เกาะเส้นระหว่างจุด เลี้ยวล่วงหน้า ไล่ความสูงตามช่วง) จบแล้วเข้า RTL
//   RTL     กลับฐาน: รักษาความสูงให้พ้นภูมิประเทศตามทาง → วนรอเหนือลานปล่อยที่ +150 m รัศมี 250 m รอกางร่ม
//   TAKEOFF ออกราง: ตั้ง elevon ล่วงหน้า −4.8° → หลุดรางแล้วคุมปีกระดับ ไต่ ~6° → ส่งต่อ FBWA
import { G, DEG, KMH, clamp, wrapPi } from './core.js';
import { thrustFull } from './fdm.js';
import { legGuidance, loiterGuidance, turnLead, RTL_ALT, LOITER_R, LOITER_V, ACCEPT_R } from './mission.js';

export const MODES = ['MANUAL', 'FBWA', 'CRUISE'];
const VREF = 200 / KMH;

export class Autopilot {
  constructor(data) {
    this.data = data;
    this.mode = 'TAKEOFF';
    this.qInt = 0;                 // อินทิกรัลของลูปอัตราก้มเงย (องศา elevon)
    this.trimDe = data.rail.elevon_preset_deg;   // trim ของโหมด MANUAL
    this.altT = 0; this.hdgT = 0; this.spdT = VREF; this.vsInt = 0; this.thrInt = 0;
    this.rollT = 0; this.pitchT = 0;
    this.tAirTakeoff = 0;
    this.msg = '';
    // ภารกิจ: จุดบิน { n, e, alt } · ฐาน = ลานปล่อย · terrainFn (ถ้ามี) ใช้ตั้งความสูงกลับฐานให้พ้นเขา
    this.mission = []; this.wpIdx = 0; this.navFrom = null; this.missionSpd = VREF;
    this.home = { n: 0, e: 0, alt: 0 }; this.terrainFn = null; this.rtlAlt = 0; this.rtlArrived = false;
    this.afterTakeoff = 'FBWA';    // 'AUTO' = เริ่มภารกิจเองเมื่อพ้นราง
  }

  setMode(mode, ac) {
    this.mode = mode;
    if (ac) {
      const e = ac.euler();
      this.qInt = -(ac.dL + ac.dR) / 2;                 // ต่อเนื่อง ไม่กระตุก
      this.trimDe = (ac.dL + ac.dR) / 2;
      this.altT = ac.alt; this.hdgT = this.course(ac, e); this.vsInt = 0; this.thrInt = 0;
      this.spdT = clamp(ac.air.V, 110 / KMH, this.data.limits.V_C_kmh / KMH);
    } else this.qInt = -this.data.rail.elevon_preset_deg;
    this.tAirTakeoff = 0;
    if (ac && mode === 'AUTO') {                        // เริ่ม/ทำต่อจากจุดที่ค้าง · จบแล้วเริ่มใหม่จากจุดแรก
      if (this.wpIdx >= this.mission.length) this.wpIdx = 0;
      this.navFrom = { n: ac.s.pos[0], e: ac.s.pos[1], alt: ac.alt, direct: true };
    }
    if (ac && mode === 'RTL') {
      const p = ac.s.pos, h = this.home, d = Math.hypot(p[0] - h.n, p[1] - h.e), k = Math.max(1, Math.ceil(d / 150));
      let top = -Infinity;
      if (this.terrainFn) for (let j = 0; j <= k; j++) top = Math.max(top, this.terrainFn(p[0] + (h.n - p[0]) * j / k, p[1] + (h.e - p[1]) * j / k));
      this.rtlAlt = Math.max(ac.alt, h.alt + RTL_ALT, top + 120); this.rtlArrived = false;
    }
  }

  /** มุมปะทะที่ต้องใช้บินด้วยโหลดแฟกเตอร์ n ที่ความเร็วปัจจุบัน */
  alphaReq(ac, n = 1) {
    const A = this.data.aero, q = Math.max(ac.air.qbar, 50);
    return clamp((n * ac.mp.m * G / (q * A.S) - A.CL0) / A.CLa, -2 * DEG, 14 * DEG);
  }

  /** แนวบินเทียบพื้น (course) — ใช้แทนทิศหัวเครื่อง เพราะหัวเครื่องส่ายตาม Dutch roll แล้วลูปรักษาทิศจะไปกระตุ้นซ้ำ */
  course(ac, e) { const v = ac.vNED(); return Math.hypot(v[0], v[1]) > 5 ? Math.atan2(v[1], v[0]) : e.yaw; }

  /** มุมเอียงสูงสุดที่ยังเหลือความเร็วเผื่อร่วงหล่น 20 % (ยิ่งช้ายิ่งเอียงได้น้อย) */
  bankLimit(ac) {
    const A = this.data.aero, vs = Math.sqrt(2 * ac.mp.m * G / (ac.air.rho * A.S * A.CLmax_design));
    const r = (1.2 * vs / Math.max(ac.air.V, vs)) ** 2;
    return r >= 1 ? 10 * DEG : clamp(Math.acos(r), 10 * DEG, 55 * DEG);
  }

  update(ac, input, dt) {
    const mx = this.data.aero.elevon_max_deg;
    if (ac.phase === 'rail') {                          // บนราง: elevon ตั้งล่วงหน้า คันเร่งตามมือ
      ac.ctl.dL = ac.ctl.dR = this.data.rail.elevon_preset_deg;
      ac.ctl.throttle = input.throttle;
      this.qInt = -this.data.rail.elevon_preset_deg;
      return;
    }
    if (ac.phase !== 'air') return;
    if (ac.chute.state !== 'stowed') { ac.ctl.dL = ac.ctl.dR = 0; ac.ctl.throttle = 0; return; }

    const e = ac.euler();
    switch (this.mode) {
      case 'MANUAL': {
        const de = this.trimDe - input.pitch * 14, da = input.roll * 9;
        ac.ctl.dL = clamp(de + da, -mx, mx); ac.ctl.dR = clamp(de - da, -mx, mx);
        ac.ctl.throttle = input.throttle;
        return;
      }
      case 'TAKEOFF': {
        this.tAirTakeoff += dt;
        this._attitude(ac, e, 0, 6 * DEG + this.alphaReq(ac, 1), dt);
        ac.ctl.throttle = input.throttle;
        if (this.tAirTakeoff > 6 && ac.air.V > 1.05 * (ac.rail?.profile?.Vexit ?? 25)) { const m = this.afterTakeoff === 'AUTO' && this.mission.length ? 'AUTO' : 'FBWA'; this.setMode(m, ac); this.msg = 'พ้นราง → โหมด ' + m; }
        return;
      }
      case 'FBWA': {
        const bl = this.bankLimit(ac);
        const rollT = clamp(input.roll * 55 * DEG, -bl, bl);
        const n = 1 / Math.max(Math.cos(e.roll), 0.45);
        const pitchT = this.alphaReq(ac, n) * Math.cos(e.roll) + input.pitch * 25 * DEG;
        this._attitude(ac, e, rollT, pitchT, dt);
        ac.ctl.throttle = input.throttle;
        return;
      }
      case 'CRUISE': {
        this.hdgT = wrapPi(this.hdgT + input.roll * 20 * DEG * dt);
        this.altT += input.pitch * 8 * dt;
        if (input.speedAdjust) this.spdT = clamp(this.spdT + input.speedAdjust * dt, 100 / KMH, this.data.limits.V_C_kmh / KMH);
        this._cruise(ac, e, dt, 0);
        return;
      }
      case 'AUTO': {
        const wps = this.mission, to = wps[this.wpIdx];
        if (!to) { this.setMode('RTL', ac); this.msg = wps.length ? 'จบภารกิจ → RTL กลับฐาน' : 'ไม่มีจุดบิน → RTL กลับฐาน'; return; }
        const V = Math.max(ac.air.V, 15), from = this.navFrom ?? { n: ac.s.pos[0], e: ac.s.pos[1], alt: ac.alt, direct: true };
        const g = legGuidance(from, to, ac.s.pos, V), next = wps[this.wpIdx + 1];
        if (input.speedAdjust) this.missionSpd = clamp(this.missionSpd + input.speedAdjust * dt, 100 / KMH, this.data.limits.V_C_kmh / KMH);
        this.hdgT = g.course; this.spdT = this.missionSpd;
        // ระหว่างจุดกับจุด: ไล่ความสูงตามระยะ · ช่วงแรกจากตำแหน่งที่เข้าโหมด: ไต่/ลดไปความสูงของจุดเป้าทันที
        this.altT = from.direct ? to.alt : from.alt + (to.alt - from.alt) * clamp(g.along / g.len, 0, 1);
        const lead = next ? turnLead(V, wrapPi(Math.atan2(next.e - to.e, next.n - to.n) - g.track)) : 0;
        if (g.len - g.along < Math.max(ACCEPT_R, lead)) {                 // ถึงจุด (หรือถึงระยะเริ่มเลี้ยว) → ไปจุดถัดไป
          this.navFrom = to; this.wpIdx++;
          if (next) this.msg = `ผ่านจุด ${this.wpIdx} → ไปจุด ${this.wpIdx + 1}`;
          else { this.setMode('RTL', ac); this.msg = 'จบภารกิจ → RTL กลับฐาน'; return; }
        }
        this._cruise(ac, e, dt, 0);
        return;
      }
      case 'RTL': {
        const V = Math.max(ac.air.V, 15), h = this.home, g = loiterGuidance(h, ac.s.pos, LOITER_R, V), near = g.d < 2500;
        this.hdgT = g.course; this.altT = near ? h.alt + RTL_ALT : this.rtlAlt; this.spdT = near ? LOITER_V : VREF;
        if (!this.rtlArrived && g.d < LOITER_R * 1.5) { this.rtlArrived = true; this.msg = 'ถึงฐาน — วนรอเหนือลานปล่อย · กด P สองครั้งเพื่อกางร่ม'; }
        this._cruise(ac, e, dt, g.rollFF);
        return;
      }
    }
  }

  // รักษาแนวบิน hdgT ความสูง altT ความเร็ว spdT (ใช้ร่วมกันโดย CRUISE / AUTO / RTL) · rollFF = มุมเอียงล่วงหน้าของวงเลี้ยว
  _cruise(ac, e, dt, rollFF) {
    const V = Math.max(ac.air.V, 15);
    const hErr = wrapPi(this.hdgT - this.course(ac, e));
    const bl = Math.min(45 * DEG, this.bankLimit(ac));
    const rollT = clamp(Math.atan(V * 0.35 * hErr / G) + rollFF, -bl, bl);
    const vs = -ac.vNED()[2];
    // ความเร็วมาก่อนการไต่: ต่ำกว่า 75 % ของความเร็วเป้าไต่ได้แค่ 1.5 m/s (เร่งก่อน) → ไต่เต็ม 8 m/s เมื่อถึง 90 % · ใกล้ร่วงหล่นไม่สั่งไต่
    const A0 = this.data.aero, vStall = Math.sqrt(2 * ac.mp.m * G / (ac.air.rho * A0.S * A0.CLmax_design));
    const vsMax = V < 1.15 * vStall ? 0 : 1.5 + 6.5 * clamp((V - 0.75 * this.spdT) / (0.15 * this.spdT), 0, 1);
    const vsCmd = clamp(0.3 * (this.altT - ac.alt), -8, vsMax);
    this.vsInt = clamp(this.vsInt + 0.004 * (vsCmd - vs) * dt, -0.1, 0.1);
    const n = 1 / Math.max(Math.cos(e.roll), 0.6);
    const pitchT = Math.asin(clamp(vsCmd / V, -0.3, 0.3)) + this.alphaReq(ac, n) * Math.cos(e.roll) + 0.012 * (vsCmd - vs) + this.vsInt;
    this._attitude(ac, e, rollT, clamp(pitchT, -20 * DEG, 25 * DEG), dt);
    // คันเร่งอัตโนมัติ: feedforward จากแรงต้านโดยประมาณ + PI ความเร็ว
    const A = this.data.aero, W = ac.mp.m * G, CL = n * W / (Math.max(ac.air.qbar, 50) * A.S);
    const Treq = ac.air.qbar * A.S * (A.CD0 + A.K * CL * CL) + W * clamp(vsCmd / V, -0.3, 0.3);
    const ff = ac.engine.throttleFor(Treq, thrustFull(this.data, V) * ac.air.rho / 1.1455);
    const err = this.spdT - ac.air.V;
    this.thrInt = clamp(this.thrInt + 0.02 * err * dt, -0.4, 0.4);
    ac.ctl.throttle = clamp(ff + 0.08 * err + this.thrInt, 0, 1);
  }

  // ลูปท่าทาง: มุมเอียง/ก้มเงยเป้า → อัตราหมุนเป้า → elevon (ปรับเกนตามความดันพลวัต)
  _attitude(ac, e, rollT, pitchT, dt) {
    const A = this.data.aero, L = this.data.limits, mx = A.elevon_max_deg;
    const V = Math.max(ac.air.V, 15), [p, q] = ac.s.om, gs = clamp((VREF / V) ** 2, 0.2, 5);
    pitchT = Math.min(pitchT, e.pitch + (12 * DEG - ac.air.alpha));          // ตัวจำกัดมุมปะทะ: ไม่สั่งเงยเกิน α 12°
    this.rollT = rollT; this.pitchT = pitchT;

    // --- เอียง ---
    const pCmd = clamp(4.0 * (rollT - e.roll), -1.6, 1.6);
    const pss = (A.Clda_per_deg / Math.abs(A.Clp)) * (2 * V / A.b);         // อัตราหมุนคงตัวต่อองศา δa
    const daMax = mx * Math.min(1, (L.roll_full_deflection_up_to_kmh / (V * KMH)) ** 2);   // ตารางจำกัดโครงสร้างที่ความเร็วสูง
    const da = clamp(pCmd / pss + 1.0 * (pCmd - p) * gs, -daMax, daMax);

    // --- ก้มเงย ---
    const cr = Math.cos(e.roll), phi = clamp(e.roll, -70 * DEG, 70 * DEG);
    let qCmd = clamp(3.0 * (pitchT - e.pitch), -0.7, 0.7) * Math.max(cr, 0) + G * Math.tan(phi) * Math.sin(phi) / V;
    const nGeo = Math.cos(e.pitch) * cr;
    qCmd = clamp(qCmd, (-1.0 - nGeo) * G / V, (0.95 * L.n_limit - nGeo) * G / V);          // จำกัด g
    const aProt = 14.5 * DEG;
    if (ac.air.alpha > aProt) qCmd -= 5 * (ac.air.alpha - aProt);                          // กันร่วงหล่น
    const err = qCmd - q;
    const raw = -(4.0 * err * gs + this.qInt);
    if (!(raw <= -mx && err > 0) && !(raw >= mx && err < 0)) this.qInt = clamp(this.qInt + 15 * err * gs * dt, -mx, mx);   // anti-windup
    const de = clamp(-(4.0 * err * gs + this.qInt), -mx, mx);

    // ผสม elevon: ให้ก้มเงยมาก่อน เอียงใช้ส่วนที่เหลือ
    const room = mx - Math.abs(de), daL = clamp(da, -room, room);
    ac.ctl.dL = de + daL; ac.ctl.dR = de - daL;
  }
}
