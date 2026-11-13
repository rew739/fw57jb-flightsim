// ชุดทดสอบฟิสิกส์ — เทียบผลจำลองกับตัวเลขใน spec v49 (data/fw57jb_sim.json → "check")
// รัน:  node --test 10_FlightSim_Game/tests/
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { density, fillFuel, massProps } from '../src/sim/core.js';
import { Aircraft, trimLevel, railProfile } from '../src/sim/fdm.js';
import { Autopilot } from '../src/sim/autopilot.js';

const data = JSON.parse(readFileSync(fileURLToPath(new URL('../data/fw57jb_sim.json', import.meta.url)), 'utf8'));
const G = 9.80665, KMH = 3.6, DEG = 180 / Math.PI;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ได้ ${a.toFixed(4)} ต้องการ ${b} ± ${tol}`);
const sumFuel = f => Object.values(f).reduce((a, b) => a + b, 0);

function run(ac, ap, seconds, input = { roll: 0, pitch: 0, throttle: 1 }, each) {
  const dt = 1 / 240;
  for (let t = 0; t < seconds; t += dt) {
    if (ap) ap.update(ac, typeof input === 'function' ? input(t) : input, dt);
    ac.step(dt);
    if (each) each(t);
  }
}

test('อากาศ 35 °C ระดับน้ำทะเล มีความหนาแน่น 1.145', () => {
  near(density(0, 35), 1.1455, 0.002, 'ρ');
});

test('เติมเต็มที่ cargo 20 kg ได้มวลขึ้นเท่าขีดราง 66.97 kg โดยลดที่ถัง pod', () => {
  const f = fillFuel(data, 20, 1);
  const mp = massProps(data, 20, f);
  near(mp.m, 66.97, 0.01, 'TOW');
  near(f.POD, 3.5235, 0.002, 'น้ำมัน pod');
  near(f.WING, 8.8452, 0.001, 'น้ำมันปีก');
});

test('CG ทุกกรณีตรงกับ spec (เต็ม/หมด × cargo 0/10/20)', () => {
  for (const [key, x] of Object.entries(data.check.cg_x_cases)) {
    const [c, st] = key.split('_');
    const cargo = Number(c);
    const fuel = st === 'เต็ม' ? fillFuel(data, cargo, 1) : fillFuel(data, cargo, 0);
    near(massProps(data, cargo, fuel).x, x, 0.0005, `x CG กรณี ${key}`);
  }
});

test('ความเฉื่อยที่ MTOW ตรงกับ spec ภายใน 1 %', () => {
  const mp = massProps(data, 20, fillFuel(data, 20, 1));
  const I = data.mass.I_mtow_spec;
  near(mp.Ixx / I.Ixx_roll, 1, 0.01, 'Ixx'); near(mp.Iyy / I.Iyy_pitch, 1, 0.01, 'Iyy'); near(mp.Izz / I.Izz_yaw, 1, 0.01, 'Izz');
});

test('แรงต้านบินระดับ 200 km/h ที่ MTOW ≈ 51.6 N (spec D_200_N)', () => {
  const mp = massProps(data, 20, fillFuel(data, 20, 1));
  const tr = trimLevel(data, mp, 200 / KMH, { rho: density(0, 35) });
  assert.ok(tr.ok, 'ต้อง trim ได้');
  near(tr.drag, data.check.D_200_N, 0.03 * data.check.D_200_N, 'แรงต้าน');
  near(tr.thrust, tr.drag, 0.02 * tr.drag, 'แรงขับ = แรงต้าน');
});

test('ความเร็วร่วงหล่นตอน trim ที่ MTOW ≈ 75.2 km/h (CLmax 1.19)', () => {
  const mp = massProps(data, 20, fillFuel(data, 20, 1));
  const rho = density(0, 35);
  let vs = null;
  for (let v = 90; v > 55; v -= 0.25) { if (!trimLevel(data, mp, v / KMH, { rho }).ok) { vs = v + 0.25; break; } }
  assert.ok(vs !== null, 'ต้องมีความเร็วที่ trim ไม่ได้');
  near(vs, data.check.Vs_CLmax119_MTOW_kmh, 2.0, 'Vs');
});

test('อัตราร่อนสูงสุดใกล้ค่าทฤษฎี 1/(2√(CD0·K)) ≈ 17', () => {
  const mp = massProps(data, 20, fillFuel(data, 20, 1));
  const tr = trimLevel(data, mp, 37.2, { rho: density(0, 35) });
  const LD = mp.m * G / tr.drag;
  assert.ok(LD > 15.3 && LD < 17.2, `L/D ${LD.toFixed(2)}`);
});

test('รางปล่อย L6 ที่ MTOW: ออกราง 100.6 km/h เร่งสูงสุด 9.88 g ในระยะ 4.6 m', () => {
  const rp = railProfile(data, 66.97);
  near(rp.Vexit * KMH, data.check.V_launch_MTOW_kmh, 0.3, 'V ออกราง');
  near(rp.aPeak / G, data.check.rail_peak_g_MTOW, 0.02, 'g สูงสุด');
  // อินทิเกรตโปรไฟล์จริง
  let s = 0, v = 0, t = 0, amax = 0; const dt = 1e-4;
  while (s < data.rail.s_accel_m) { const a = rp.accel(s, t); amax = Math.max(amax, a); v += a * dt; s += v * dt; t += dt; }
  near(v * KMH, data.check.V_launch_MTOW_kmh, 0.5, 'V จากการอินทิเกรต');
  assert.ok(amax / G <= 9.9, `g สูงสุด ${amax / G}`);
});

test('เครื่องบินเบากว่าออกรางช้ากว่าและ g น้อยกว่า', () => {
  const a = railProfile(data, 47.38), b = railProfile(data, 66.97);
  assert.ok(a.Vexit < b.Vexit && a.aPeak < b.aPeak);
  near(a.Vexit * KMH, 84.6, 0.6, 'V ออกราง cargo 0');
});

test('เสถียรตามยาว: ถูกรบกวนมุมปะทะแล้วกลับเข้าสมดุลเอง (คุมนิ่ง)', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  ac.initAir({ V: 200 / KMH, alt: 500 });
  const a0 = ac.air.alpha;
  ac.s.vb[2] += 3.0;                                   // เพิ่มความเร็วแนวดิ่ง → α เพิ่ม ~3°
  run(ac, null, 4);
  near(ac.air.alpha * DEG, a0 * DEG, 0.4, 'α หลัง 4 s');
  assert.ok(Math.abs(ac.s.om[1]) < 0.02, `อัตราก้มเงยเหลือ ${ac.s.om[1]}`);
});

test('เสถียรทางข้าง: ถูกรบกวนมุมไถลแล้วมุมไถลลดลง (winglet)', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  ac.initAir({ V: 200 / KMH, alt: 500 });
  ac.s.vb[1] += 4.0;                                   // β ≈ 4°
  const b0 = Math.abs(ac.airdata().beta);
  let bmax = 0;
  run(ac, null, 6, undefined, t => { if (t > 5) bmax = Math.max(bmax, Math.abs(ac.air.beta)); });
  assert.ok(bmax < 0.6 * b0, `β ช่วงท้าย ${bmax * DEG}° จากเริ่ม ${b0 * DEG}°`);
});

test('เต็มคันเร่งอยู่กับที่: แรงขับ 125 N ใช้น้ำมัน 380 g/min จากถัง pod ก่อน', () => {
  const ac = new Aircraft(data, { cargoKg: 0, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  ac.initRail({ heading: 0 });
  ac.engine.forceRunning(1.0);
  const f0 = { ...ac.fuel };
  ac.ctl.throttle = 1;
  run(ac, null, 60);
  near(ac.engine.thrust, 125.0, 1.5, 'แรงขับ');
  near((sumFuel(f0) - sumFuel(ac.fuel)) * 1000, 380, 8, 'น้ำมันที่ใช้ใน 60 s (g)');
  near(ac.fuel.WING, f0.WING, 1e-9, 'ถังปีกยังไม่ถูกใช้');
  assert.equal(ac.phase, 'rail');
});

test('เครื่องยนต์เร่งจากเดินเบาถึงเต็มใช้เวลาราว 2–5 s (ไม่ทันที)', () => {
  const ac = new Aircraft(data, { cargoKg: 0, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  ac.initRail({ heading: 0 });
  ac.engine.forceRunning(0.0);
  ac.ctl.throttle = 1;
  let t95 = null;
  run(ac, null, 8, undefined, t => { if (t95 === null && ac.engine.thrust > 0.95 * 125) t95 = t; });
  assert.ok(t95 > 2 && t95 < 5, `t95 = ${t95}`);
});

test('ออกรางที่ MTOW ด้วยโหมด TAKEOFF: ไม่จมต่ำกว่าปลายราง ความเร็วไม่ต่ำกว่า 25 m/s แล้วไต่ขึ้น', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  const ap = new Autopilot(data);
  ac.initRail({ heading: 0 });
  ac.engine.forceRunning(1.0);
  ap.setMode('TAKEOFF');
  ac.launch();
  let hExit = null, hMin = 1e9, vMin = 1e9, nMax = 0;
  run(ac, ap, 8, { roll: 0, pitch: 0, throttle: 1 }, () => {
    if (ac.phase === 'air') {
      if (hExit === null) hExit = ac.alt;
      hMin = Math.min(hMin, ac.alt); vMin = Math.min(vMin, ac.air.V); nMax = Math.max(nMax, ac.nz);
    }
  });
  assert.equal(ac.phase, 'air');
  assert.ok(hMin >= hExit - 0.3, `จม ${(hExit - hMin).toFixed(2)} m ใต้ปลายราง`);
  assert.ok(vMin >= 25, `V ต่ำสุด ${vMin.toFixed(2)} m/s`);
  assert.ok(ac.alt - hExit > 15, `ไต่ได้ ${(ac.alt - hExit).toFixed(1)} m ใน 8 s`);
  assert.ok(nMax < 3.0, `n สูงสุด ${nMax.toFixed(2)}`);
});

test('ออกรางที่ MTOW จากลานสูง 300 m (แผนที่เชียงใหม่ อากาศบางกว่า 3 %): ยังไม่จมและไต่ขึ้นได้', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 }, terrain: () => 300 });
  const ap = new Autopilot(data);
  ac.initRail({ heading: 0 }); ac.engine.forceRunning(1.0); ap.setMode('TAKEOFF'); ac.launch();
  let hExit = null, hMin = 1e9, vMin = 1e9, aMax = 0;
  run(ac, ap, 8, { roll: 0, pitch: 0, throttle: 1 }, () => {
    if (ac.phase !== 'air') return;
    if (hExit === null) hExit = ac.alt;
    hMin = Math.min(hMin, ac.alt); vMin = Math.min(vMin, ac.air.V); aMax = Math.max(aMax, ac.air.alpha * DEG);
  });
  assert.equal(ac.phase, 'air');
  assert.ok(hExit > 300 && hExit < 306, `ปลายรางอยู่ที่ ${hExit.toFixed(1)} m`);
  assert.ok(hMin >= hExit - 0.3, `จม ${(hExit - hMin).toFixed(2)} m ใต้ปลายราง`);
  assert.ok(vMin >= 25, `V ต่ำสุด ${vMin.toFixed(2)} m/s`);
  assert.ok(aMax < 14, `α สูงสุด ${aMax.toFixed(1)}°`);
  assert.ok(ac.alt - hExit > 12, `ไต่ได้ ${(ac.alt - hExit).toFixed(1)} m ใน 8 s`);
});

test('โหมด CRUISE รักษาความสูง ความเร็ว และทิศ ได้ 40 s', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  const ap = new Autopilot(data);
  ac.initAir({ V: 200 / KMH, alt: 500, heading: 90 });
  ap.setMode('CRUISE', ac);
  run(ac, ap, 40, { roll: 0, pitch: 0, throttle: 0.5 });
  near(ac.alt, 500, 5, 'ความสูง');
  near(ac.air.V * KMH, 200, 5, 'ความเร็ว');
  near(ac.euler().yaw * DEG, 90, 3, 'ทิศ');
  assert.ok(Math.abs(ac.euler().roll * DEG) < 2, 'ปีกระดับ');
});

test('โหมด FBWA: โยกเอียงเต็ม → เอียงราว 55° ใน 3 s ไม่เกินเลย และไม่เสียความสูงมาก', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  const ap = new Autopilot(data);
  ac.initAir({ V: 200 / KMH, alt: 500 });
  ap.setMode('FBWA', ac);
  let rmax = 0;
  run(ac, ap, 6, { roll: 1, pitch: 0, throttle: 0.7 }, () => { rmax = Math.max(rmax, ac.euler().roll * DEG); });
  near(ac.euler().roll * DEG, 55, 5, 'มุมเอียงคงตัว');
  assert.ok(rmax < 63, `เอียงเกิน ${rmax.toFixed(1)}°`);
  assert.ok(Math.abs(ac.alt - 500) < 25, `ความสูงเปลี่ยน ${(ac.alt - 500).toFixed(1)} m`);
  assert.ok(ac.nz < 3.05, `n ${ac.nz.toFixed(2)}`);
});

// แรงกระตุก: ค่าคำนวณ 1 มิติของโครงการ = 1119 N (เครื่องพุ่งตรง) · ในแบบจำลอง 6-DOF เครื่องเชิดหัวขึ้นใต้ร่ม ปีกตั้งชันช่วยเบรก
// จึงได้ค่าต่ำกว่า → ตรวจว่าอยู่ในช่วง 0.5–1.3 เท่าของค่า 1 มิติ และไม่เกินขีดจำกัดโครงสร้าง
test('กางร่มที่ 100 km/h: แรงกระตุกไม่เกินขีดโครงสร้าง แล้วร่อนลง 5 m/s ลำตัวเกือบระดับ', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  ac.initAir({ V: 100 / KMH, alt: 600 });
  ac.ctl.throttle = 0; ac.engine.shutdown();
  ac.deployChute();
  let fmax = 0;
  run(ac, null, 25, undefined, () => { fmax = Math.max(fmax, ac.chute.force); });
  const F1D = data.chute.open_ref.F_peak_N;
  assert.ok(fmax > 0.5 * F1D && fmax < 1.3 * F1D && fmax < data.chute.F_limit_N, `แรงกระตุก ${fmax.toFixed(0)} N (ค่า 1 มิติ ${F1D.toFixed(0)})`);
  const vs = ac.vNED()[2];
  near(vs, data.check.chute_vd_full_ms, 0.5, 'อัตราร่วง');
  assert.ok(Math.abs(ac.euler().pitch * DEG) < 25 && Math.abs(ac.euler().roll * DEG) < 25, `ท่า pitch ${(ac.euler().pitch * DEG).toFixed(1)}° roll ${(ac.euler().roll * DEG).toFixed(1)}°`);
  assert.equal(ac.chute.state, 'open');
});

test('กางร่มเร็วเกิน (300 km/h) สายร่มขาดเพราะแรงเกินค่าประลัย', () => {
  const ac = new Aircraft(data, { cargoKg: 20, fuelFrac: 1, wind: { speed: 0, turb: 0 } });
  ac.initAir({ V: 300 / KMH, alt: 800 });
  ac.deployChute();
  run(ac, null, 4);
  assert.equal(ac.chute.state, 'failed');
});

test('ลงถึงพื้นด้วยร่ม = ลงจอดสำเร็จ · ชนพื้นโดยไม่มีร่ม = ตก', () => {
  const a = new Aircraft(data, { cargoKg: 0, fuelFrac: 0.2, wind: { speed: 0, turb: 0 }, terrain: () => 0 });
  a.initAir({ V: 95 / KMH, alt: 80 });
  a.engine.shutdown(); a.deployChute();
  run(a, null, 40);
  assert.equal(a.phase, 'landed');
  assert.ok(a.touchdown.vz < 6, `ความเร็วแตะพื้น ${a.touchdown.vz}`);
  const b = new Aircraft(data, { cargoKg: 0, fuelFrac: 0.2, wind: { speed: 0, turb: 0 }, terrain: () => 0 });
  b.initAir({ V: 150 / KMH, alt: 30, gamma: -20 });
  run(b, null, 10);
  assert.equal(b.phase, 'crashed');
});

// ---------- ระบบกันร่วงหล่นของ FBWA (พบจากการทดสอบเล่น: ดึงคันค้างแล้วเครื่องตก) ----------
function launched(cargoKg, turb) {
  const ac = new Aircraft(data, { cargoKg, fuelFrac: 1, wind: { speed: 3, fromDeg: 40, turb }, terrain: () => 12, seed: 3 });
  const ap = new Autopilot(data);
  ac.initRail({ heading: 40 }); ac.engine.forceRunning(1); ap.setMode('TAKEOFF'); ac.launch();
  run(ac, ap, 8.8, { roll: 0, pitch: 0, throttle: 1 });
  return { ac, ap };
}

test('FBWA: ดึงคันบังคับค้าง 40 s หลังออกราง เครื่องไม่ร่วงหล่น ไม่พลิก และยังไต่ขึ้น', () => {
  const { ac, ap } = launched(10, 0.5);
  const h0 = ac.alt; let aMax = 0, rMax = 0;
  run(ac, ap, 40, { roll: 0, pitch: 1, throttle: 1 }, () => { aMax = Math.max(aMax, ac.air.alpha * DEG); rMax = Math.max(rMax, Math.abs(ac.euler().roll * DEG)); });
  assert.equal(ac.phase, 'air');
  assert.ok(aMax < 16, `α สูงสุด ${aMax.toFixed(1)}°`);
  assert.ok(rMax < 15, `เอียงสูงสุด ${rMax.toFixed(1)}°`);
  assert.ok(ac.alt > h0 + 40, `ไต่ได้ ${(ac.alt - h0).toFixed(0)} m`);
});

test('FBWA: ดึงคันค้างพร้อมโยกเอียงเต็ม ที่คันเร่งเดินเบา (ความเร็วตก) เครื่องยังคุมได้ ไม่หมุนควง', () => {
  const { ac, ap } = launched(20, 0.5);
  run(ac, ap, 10, { roll: 0, pitch: 0.4, throttle: 1 });
  let aMax = 0, rMax = 0;
  run(ac, ap, 25, { roll: 1, pitch: 1, throttle: 0 }, () => { aMax = Math.max(aMax, ac.air.alpha * DEG); rMax = Math.max(rMax, Math.abs(ac.euler().roll * DEG)); });
  assert.ok(aMax < 17, `α สูงสุด ${aMax.toFixed(1)}°`);
  assert.ok(rMax < 65, `เอียงสูงสุด ${rMax.toFixed(1)}°`);
  assert.ok(ac.air.sigma < 0.2, 'ไม่อยู่ในสภาพร่วงหล่น');
});

// พบจากการทดสอบเล่น: CRUISE ที่ความเร็วต่ำ (~110 km/h) ลูปรักษาทิศทำให้ส่ายทางข้างเพิ่มขึ้นเรื่อย ๆ (ตัวเครื่องเองเสถียร)
test('โหมด CRUISE ที่ความเร็วต่ำ 110 km/h มีลมปั่นป่วนเบา: ไม่ส่ายทางข้าง รักษาทิศได้ 60 s', () => {
  const { ac, ap } = launched(10, 0.5);
  run(ac, ap, 12, { roll: 0, pitch: 0.5, throttle: 1 });
  ac.ctl.throttle = 0.45;
  run(ac, ap, 10, { roll: 0, pitch: 0, throttle: 0.45 });
  ap.setMode('CRUISE', ac); ap.spdT = 110 / KMH;
  const trk0 = Math.atan2(ac.vNED()[1], ac.vNED()[0]) * DEG;
  let bMax = 0;
  run(ac, ap, 60, { roll: 0, pitch: 0, throttle: 0 }, t => { if (t > 5) bMax = Math.max(bMax, Math.abs(ac.air.beta * DEG)); });
  const trk = Math.atan2(ac.vNED()[1], ac.vNED()[0]) * DEG;
  assert.equal(ac.phase, 'air');
  assert.ok(bMax < 6, `มุมไถลสูงสุด ${bMax.toFixed(1)}°`);
  assert.ok(Math.abs(trk - trk0) < 6, `แนวบินเปลี่ยน ${(trk - trk0).toFixed(1)}°`);
  near(ac.air.V * KMH, 110, 8, 'ความเร็ว');
});

// ---------- ไฟล์ข้อมูลของเกมต้องตรงกับ spec ล่าสุด (ถ้า spec เปลี่ยน ให้รัน tools/export_from_spec.py ใหม่) ----------
test('data/fw57jb_sim.json ตรงกับ spec ปัจจุบัน (มวล CG อากาศพลศาสตร์ ราง)', () => {
  const spec = JSON.parse(readFileSync(fileURLToPath(new URL('../../06_FlyingWing_Design/FW57J_B_BG25_spec_for_sim.json', import.meta.url)), 'utf8'));
  near(data.mass.empty_kg, spec.mass_kg.empty, 1e-6, 'มวลเปล่า');
  near(data.mass.tow_max_rail_kg, spec.mass_kg.MTOW, 1e-6, 'MTOW');
  near(data.mass.fuel_cap_kg, spec.mass_kg.fuel_capacity_kg, 1e-6, 'ความจุน้ำมัน');
  near(data.aero.NP_x, spec.cg_m_from_CAD_origin.NP, 1e-9, 'NP');
  near(data.aero.CLa, spec.aero_OpenVSP.CLa, 1e-9, 'CLa');
  near(data.aero.CD0, spec.polar_design.CD0, 1e-9, 'CD0');
  near(data.mass.I_mtow_spec.Izz_yaw, spec.inertia_kgm2_about_CG.Izz_yaw, 1e-9, 'Izz');
  near(data.rail.V_launch_ref_kmh, spec.launcher.V_launch_kmh, 1e-9, 'V ออกราง');
  near(data.limits.V_NE_kmh, spec.limits.V_NE_kmh, 1e-9, 'V_NE');
});
