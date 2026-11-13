// ทดสอบการบินตามจุด (AUTO) และกลับฐาน (RTL): node --test 10_FlightSim_Game/tests/mission.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { Aircraft } from '../src/sim/fdm.js';
import { Autopilot } from '../src/sim/autopilot.js';
import { legGuidance, loiterGuidance, turnLead, missionStats, RTL_ALT, LOITER_R, LOITER_V } from '../src/sim/mission.js';

const data = JSON.parse(readFileSync(fileURLToPath(new URL('../data/fw57jb_sim.json', import.meta.url)), 'utf8'));
const G = 9.80665, KMH = 3.6, RAD = Math.PI / 180;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ได้ ${(+a).toFixed(3)} ต้องการ ${b} ± ${tol}`);
const NOIN = { roll: 0, pitch: 0, throttle: 0.6, speedAdjust: 0 };
function run(ac, ap, seconds, each) {
  const dt = 1 / 240;
  for (let t = 0; t < seconds && ac.phase === 'air'; t += dt) { ap.update(ac, NOIN, dt); ac.step(dt); if (each) each(t); }
}

test('นำทางตามเส้น: อยู่บนเส้น = บินตามทิศเส้น · หลุดไปทางขวา = หันกลับทางซ้าย', () => {
  const a = { n: 0, e: 0 }, b = { n: 0, e: 4000 };                             // เส้นไปทางตะวันออก (090°)
  const on = legGuidance(a, b, [0, 1000], 55);
  near(on.track, 90 * RAD, 1e-9, 'ทิศเส้น'); near(on.course, 90 * RAD, 1e-9, 'แนวบินเป้า'); near(on.along, 1000, 1e-9, 'ระยะตามเส้น'); near(on.len, 4000, 1e-9, 'ยาว'); near(on.xtk, 0, 1e-9, 'เบี่ยง');
  const right = legGuidance(a, b, [-100, 1000], 55);                           // ใต้เส้น = ขวาของเส้นที่มุ่งตะวันออก
  near(right.xtk, 100, 1e-9, 'เบี่ยงขวา +'); assert.ok(right.course < 90 * RAD && right.course > 45 * RAD, `หันซ้ายเข้าเส้น ${right.course / RAD}°`);
  const far = legGuidance(a, b, [5000, 1000], 55);                              // ไกลมาก: มุมเข้าเส้นไม่เกิน 60°
  near(far.course, 150 * RAD, 1e-6, 'จำกัดมุมเข้าเส้น 60°');
});

test('ระยะเลี้ยวล่วงหน้าโตตามความเร็วและมุมเลี้ยว และไม่เกิน 600 m', () => {
  assert.equal(turnLead(55, 0), 0);
  const l90 = turnLead(55, 90 * RAD), l30 = turnLead(55, 30 * RAD);
  near(l90, 55 * 55 / (G * Math.tan(35 * RAD)), 1, 'เลี้ยว 90° = รัศมีเลี้ยวที่เอียง 35°');
  assert.ok(l30 < l90 && l30 > 50, `30° → ${l30}`);
  assert.equal(turnLead(90, 170 * RAD), 600);
});

test('วนรอ: ไกล = มุ่งเข้าศูนย์กลาง · อยู่บนวง = บินตามเส้นสัมผัสตามเข็ม พร้อมมุมเอียงล่วงหน้า', () => {
  const c = { n: 0, e: 0 };
  const far = loiterGuidance(c, [-6000, 0], 250, 36);                           // อยู่ใต้ศูนย์กลาง 6 km
  near(far.course, 0, 0.06, 'มุ่งเหนือเข้าหาศูนย์กลาง'); near(far.rollFF, 0, 1e-9, 'ยังไม่เอียงล่วงหน้า');
  const on = loiterGuidance(c, [-250, 0], 250, 36);                             // บนวง ใต้ศูนย์กลาง → ตามเข็ม = มุ่งตะวันตก
  near(on.course, -90 * RAD, 1e-9, 'เส้นสัมผัส'); near(on.rollFF, Math.atan(36 * 36 / (G * 250)), 1e-9, 'มุมเอียงวงเลี้ยวคงตัว');
  const inside = loiterGuidance(c, [-100, 0], 250, 36);
  assert.ok(inside.course < -90 * RAD && inside.course > -180 * RAD, `อยู่ในวง → หันออก ${inside.course / RAD}°`);
});

test('สรุปภารกิจ: ระยะ ทิศ ระยะพ้นพื้นต่ำสุดของแต่ละช่วง รวมขากลับฐาน', () => {
  const home = { n: 0, e: 0, alt: 300 };
  const ridge = (n, e) => (e > 5000 && e < 6000 ? 800 : 300);                   // สันเขาสูง 800 m ขวางทางตะวันออก
  const wps = [{ n: 0, e: 4000, alt: 700 }, { n: 0, e: 9000, alt: 700 }, { n: 3000, e: 9000, alt: 1000 }];
  const st = missionStats(wps, home, ridge);
  assert.equal(st.legs.length, 4, 'ฐาน→1, 1→2, 2→3, 3→ฐาน');
  near(st.legs[0].dist, 4000, 1e-6, 'ช่วง 1'); near(st.legs[0].brg, 90, 1e-6, 'ทิศช่วง 1'); near(st.legs[2].brg, 0, 1e-6, 'ทิศช่วง 3');
  near(st.legs[1].minClear, -100, 1e-6, 'ช่วง 2 ชนสันเขา (700 − 800)');
  near(st.legs[2].minClear, 400, 1e-6, 'ช่วง 3');
  near(st.total, 4000 + 5000 + 3000 + Math.hypot(3000, 9000), 1e-6, 'ระยะรวม');
  near(st.minClear, -100, 1e-6, 'ต่ำสุดทั้งเส้นทาง');
  near(st.legs[0].minClear, 120, 1e-6, 'ช่วงแรกไต่ทันทีด้วยความชัน 8 % จากความสูงลาน ไม่นับ 1.5 km แรก (ที่ 1.5 km สูง 420 m)');
  near(st.legs[3].minClear, 1000 - 800, 1e-6, 'ขากลับใช้ความสูงจุดสุดท้าย ไม่นับเขตวนรอ 2.5 km รอบฐาน');
  assert.ok(st.profile.length > 100 && st.profile[0].s === 0, 'มีหน้าตัดเส้นทาง');
  assert.deepEqual(missionStats([], home, ridge), { legs: [], total: 0, minClear: Infinity, profile: [] });
});

function airborne(opts = {}) {
  const ac = new Aircraft(data, { cargoKg: 10, fuelFrac: 1, wind: { speed: 3, fromDeg: 40, turb: 0.5 }, terrain: () => 300, seed: 5, ...opts });
  const ap = new Autopilot(data);
  ap.home = { n: 0, e: 0, alt: 300 };
  ac.initAir({ V: 200 / KMH, alt: 700, heading: 0 });
  return { ac, ap };
}

test('AUTO: บินผ่านจุด 3 จุดตามลำดับ เกาะเส้นทาง ถึงความสูงที่ตั้ง แล้วเข้า RTL เอง', () => {
  const { ac, ap } = airborne();
  ap.mission = [{ n: 2500, e: 0, alt: 700 }, { n: 2500, e: 3000, alt: 850 }, { n: -500, e: 3000, alt: 750 }];
  ap.setMode('AUTO', ac);
  const dmin = [1e9, 1e9, 1e9]; let xtkMid = 0, altAt2 = 0, aMax = 0, seq = [];
  run(ac, ap, 200, () => {
    const p = ac.s.pos;
    ap.mission.forEach((w, i) => { const d = Math.hypot(p[0] - w.n, p[1] - w.e); if (d < dmin[i]) { dmin[i] = d; if (i === 1) altAt2 = ac.alt; } });
    if (ap.mode === 'AUTO' && ap.wpIdx === 1 && p[1] > 1200 && p[1] < 2200) xtkMid = Math.max(xtkMid, Math.abs(p[0] - 2500));
    if (seq[seq.length - 1] !== ap.mode + ap.wpIdx) seq.push(ap.mode + ap.wpIdx);
    aMax = Math.max(aMax, ac.air.alpha / RAD);
  });
  assert.equal(ac.phase, 'air');
  assert.deepEqual(seq.slice(0, 4), ['AUTO0', 'AUTO1', 'AUTO2', 'RTL3'], `ลำดับ ${seq}`);
  dmin.forEach((d, i) => assert.ok(d < 350, `ผ่านจุด ${i + 1} ห่าง ${d.toFixed(0)} m (เลี้ยวตัดมุมได้ไม่เกิน 350 m)`));
  assert.ok(xtkMid < 40, `กลางช่วง 2 เบี่ยงจากเส้น ${xtkMid.toFixed(1)} m`);
  near(altAt2, 850, 30, 'ความสูงที่จุด 2');
  assert.ok(aMax < 12.5, `α สูงสุด ${aMax.toFixed(1)}°`);
});

test('AUTO ช่วงแรก (จากจุดที่เข้าโหมด): ไต่ตรงไปความสูงของจุดแรกทันที ไม่ไล่ระดับตามระยะ', () => {
  const { ac, ap } = airborne();
  ac.initAir({ V: 200 / KMH, alt: 400, heading: 0 });
  ap.mission = [{ n: 8000, e: 0, alt: 900 }, { n: 8000, e: 4000, alt: 900 }];
  ap.setMode('AUTO', ac);
  run(ac, ap, 70);
  assert.equal(ap.wpIdx, 0, 'ยังไม่ถึงจุดแรก');
  assert.ok(ac.alt > 760, `หลัง 70 s (ไปได้ราวครึ่งทาง) สูง ${ac.alt.toFixed(0)} m — ถ้าไล่ระดับตามระยะจะได้ราว 640 m`);
});

test('ไต่ขึ้นในโหมดอัตโนมัติ: เร่งความเร็วก่อนแล้วค่อยไต่ ไม่ค้างอยู่ใกล้ความเร็วร่วงหล่น', () => {
  const ac = new Aircraft(data, { cargoKg: 10, fuelFrac: 1, wind: { speed: 0, turb: 0 }, terrain: () => 300 });
  const ap = new Autopilot(data);
  ap.home = { n: 0, e: 0, alt: 300 }; ap.mission = [{ n: 9000, e: 0, alt: 800 }, { n: 9000, e: 5000, alt: 800 }]; ap.afterTakeoff = 'AUTO';
  ac.initRail({ heading: 0 }); ac.engine.forceRunning(1); ap.setMode('TAKEOFF'); ac.launch();
  const dt = 1 / 240, at = {}; let hMinAfter = 1e9;
  for (let t = 0; t < 110; t += dt) {
    ap.update(ac, { roll: 0, pitch: 0, throttle: 1, speedAdjust: 0 }, dt); ac.step(dt);
    for (const k of [30, 60, 110]) if (!at[k] && t >= k - dt) at[k] = { V: ac.air.V * KMH, alt: ac.alt };
    if (t > 8 && ac.phase === 'air') hMinAfter = Math.min(hMinAfter, ac.alt);
  }
  assert.equal(ac.phase, 'air'); assert.equal(ap.mode, 'AUTO');
  assert.ok(at[30].V > 150, `ที่ 30 s ความเร็ว ${at[30].V.toFixed(0)} km/h (ต้องเร่งพ้นช่วงใกล้ร่วงหล่นแล้ว)`);
  assert.ok(at[60].V > 170, `ที่ 60 s ความเร็ว ${at[60].V.toFixed(0)} km/h`);
  assert.ok(hMinAfter > 315, `หลังพ้นราง 8 s ไม่ลดต่ำกว่า 15 m เหนือลาน (ต่ำสุด ${hMinAfter.toFixed(0)} m)`);
  assert.ok(at[110].alt > 780, `ที่ 110 s ถึงความสูงเป้า 800 m (ได้ ${at[110].alt.toFixed(0)} m)`);
});

test('RTL: กลับมาวนเหนือลานปล่อยที่ความสูงลาน + 150 m รัศมีราว 250 m ความเร็ว 110 km/h', () => {
  const { ac, ap } = airborne();
  ac.initAir({ V: 200 / KMH, alt: 700, heading: 90 }); ac.s.pos[0] = 3000; ac.s.pos[1] = 4000;      // ห่างฐาน 5 km
  ap.setMode('RTL', ac);
  let rMin = 1e9, rMax = 0, hMin = 1e9, hMax = 0, vMin = 1e9, vMax = 0, bankMax = 0, turned = 0, lastC = null;
  run(ac, ap, 330, t => {
    if (t < 230) return;                                                        // 100 s สุดท้าย = วนรอแล้ว
    const p = ac.s.pos, r = Math.hypot(p[0], p[1]), v = ac.vNED(), c = Math.atan2(v[1], v[0]);
    rMin = Math.min(rMin, r); rMax = Math.max(rMax, r); hMin = Math.min(hMin, ac.alt); hMax = Math.max(hMax, ac.alt);
    vMin = Math.min(vMin, ac.air.V * KMH); vMax = Math.max(vMax, ac.air.V * KMH); bankMax = Math.max(bankMax, Math.abs(ac.euler().roll) / RAD);
    if (lastC !== null) { let d = c - lastC; if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI; turned += d; } lastC = c;
  });
  assert.equal(ac.phase, 'air'); assert.equal(ap.mode, 'RTL');
  assert.ok(rMin > LOITER_R - 70 && rMax < LOITER_R + 70, `รัศมีวน ${rMin.toFixed(0)}–${rMax.toFixed(0)} m (เป้า ${LOITER_R})`);
  assert.ok(hMin > 300 + RTL_ALT - 20 && hMax < 300 + RTL_ALT + 20, `ความสูง ${hMin.toFixed(0)}–${hMax.toFixed(0)} m (เป้า ${300 + RTL_ALT})`);
  assert.ok(vMin > LOITER_V * KMH - 12 && vMax < LOITER_V * KMH + 12, `ความเร็ว ${vMin.toFixed(0)}–${vMax.toFixed(0)} km/h`);
  assert.ok(bankMax < 46, `เอียงสูงสุด ${bankMax.toFixed(0)}°`);
  assert.ok(turned > 2 * Math.PI, `วนตามเข็มได้ ${(turned / RAD).toFixed(0)}° ใน 100 s`);
});

test('RTL รู้จักภูมิประเทศ: มีเขาขวางทางกลับ → ตั้งความสูงกลับฐานให้พ้นยอด 120 m', () => {
  const ridge = (n, e) => (e > 2000 && e < 3000 ? 900 : 300);
  const { ac, ap } = airborne({ terrain: ridge });
  ap.terrainFn = ridge;
  ac.initAir({ V: 200 / KMH, alt: 600, heading: 270 }); ac.s.pos[0] = 0; ac.s.pos[1] = 9000;      // ห่างสันเขา 6 km พอให้ไต่ 420 m
  ap.setMode('RTL', ac);
  near(ap.rtlAlt, 1020, 1e-6, 'ความสูงกลับฐาน');
  run(ac, ap, 220);
  assert.equal(ac.phase, 'air', 'ข้ามสันเขาได้ ไม่ชน');
  assert.ok(Math.hypot(ac.s.pos[0], ac.s.pos[1]) < 1500, 'ถึงบริเวณฐาน');
});

test('AUTO ไม่มีจุดบิน → เข้า RTL · AUTO หลังจบภารกิจเริ่มใหม่จากจุดแรก', () => {
  const { ac, ap } = airborne();
  ap.mission = []; ap.setMode('AUTO', ac);
  run(ac, ap, 0.1);
  assert.equal(ap.mode, 'RTL');
  ap.mission = [{ n: 2000, e: 0, alt: 700 }]; ap.wpIdx = 1; ap.setMode('AUTO', ac);
  assert.equal(ap.wpIdx, 0);
});
