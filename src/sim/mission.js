// การนำทางตามจุด (ใช้กับโหมด AUTO / RTL ของ autopilot) และสรุปภารกิจสำหรับหน้าวางแผน — คณิตล้วน ไม่ขึ้นกับเบราว์เซอร์
// จุดบิน = { n, e, alt } เมตร (เหนือ, ตะวันออก จากลานปล่อย · alt เหนือระดับน้ำทะเล) · มุมเป็นเรเดียน วัดจากเหนือตามเข็ม
import { G, clamp } from './core.js';

export const RTL_ALT = 150;            // ความสูงกลับฐาน/วนรอ เหนือลานปล่อย (m)
export const LOITER_R = 250;           // รัศมีวนรอ (m)
export const LOITER_V = 110 / 3.6;     // ความเร็ววนรอ (m/s) — เท่าความเร็วสูงสุดที่แนะนำให้กางร่ม
export const ACCEPT_R = 40;            // ถือว่าถึงจุดเมื่อเหลือระยะตามเส้นน้อยกว่านี้ (m)
const HOME_CLIMB_SKIP = 1500, HOME_LOITER_SKIP = 2500;
export const CLIMB_GRAD = 0.08;        // ความชันไต่ที่ใช้ประมาณช่วงแรกในหน้าวางแผน (ราว 4.4 m/s ที่ 200 km/h)

/** นำทางตามเส้น a → b: คืนทิศเส้น ระยะตามเส้น ระยะเบี่ยง (+ = อยู่ขวาของเส้น) และแนวบินเป้าที่พาเข้าหาเส้น (มุมเข้าไม่เกิน 60°) */
export function legGuidance(a, b, pos, V) {
  const dn = b.n - a.n, de = b.e - a.e, len = Math.hypot(dn, de) || 1e-6, tn = dn / len, te = de / len;
  const pn = pos[0] - a.n, pe = pos[1] - a.e;
  const along = pn * tn + pe * te, xtk = pe * tn - pn * te, track = Math.atan2(de, dn);
  const L1 = clamp(V * 5, 150, 400);
  return { track, len, along, xtk, course: track - clamp(Math.atan(xtk / L1), -Math.PI / 3, Math.PI / 3) };
}

/** ระยะที่ต้องเริ่มเลี้ยวก่อนถึงจุด เพื่อเลี้ยวเข้าช่วงถัดไปได้พอดี (เอียง 35°) · ไม่เกิน 600 m */
export function turnLead(V, dPsi, bankDeg = 35) {
  const R = V * V / (G * Math.tan(bankDeg * Math.PI / 180));
  return Math.min(R * Math.tan(Math.min(Math.abs(dPsi), Math.PI * 0.94) / 2), 600);
}

/** วนรอบจุด c รัศมี R ตามเข็มนาฬิกา: แนวบินเป้า + มุมเอียงล่วงหน้าของวงเลี้ยวคงตัว (จางเมื่ออยู่ห่างวง) */
export function loiterGuidance(c, pos, R, V) {
  const dn = c.n - pos[0], de = c.e - pos[1], d = Math.hypot(dn, de), toC = Math.atan2(de, dn);
  const course = toC - (Math.PI / 2 - Math.atan((d - R) / (0.6 * R)));
  return { d, course, rollFF: Math.atan(V * V / (G * R)) * clamp(1 - Math.abs(d - R) / R, 0, 1) };
}

/** สรุปภารกิจ: ฐาน → จุด 1 … จุดสุดท้าย → ฐาน · ระยะ ทิศ (องศา) ระยะพ้นพื้นต่ำสุด และหน้าตัดเส้นทาง
 *  ช่วงแรกไต่ทันทีด้วยความชัน 8 % จากความสูงลานจนถึงความสูงจุดแรก (ไม่นับ 1.5 km แรก) · ขากลับคงความสูงจุดสุดท้ายหรือลาน + 150 m (ไม่นับเขตวนรอ 2.5 km) */
export function missionStats(wps, home, height, step = 100) {
  if (!wps.length) return { legs: [], total: 0, minClear: Infinity, profile: [] };
  const last = wps[wps.length - 1], back = Math.max(last.alt, home.alt + RTL_ALT);
  const pts = [{ n: home.n, e: home.e, alt: home.alt }, ...wps, { n: home.n, e: home.e, alt: back }];
  const legs = [], profile = [];
  let total = 0, minClear = Infinity;
  for (let i = 0; i + 1 < pts.length; i++) {
    const a = pts[i], b = pts[i + 1], dist = Math.hypot(b.n - a.n, b.e - a.e), ret = i === pts.length - 2;
    const k = Math.max(1, Math.ceil(dist / step)), skip = i === 0 ? HOME_CLIMB_SKIP : ret ? HOME_LOITER_SKIP : 0;
    let clear = Infinity;
    for (let j = 0; j <= k; j++) {
      const t = j / k, n = a.n + (b.n - a.n) * t, e = a.e + (b.e - a.e) * t, ground = height(n, e);
      const alt = ret ? back : i === 0 ? (b.alt >= a.alt ? Math.min(b.alt, a.alt + CLIMB_GRAD * dist * t) : b.alt) : a.alt + (b.alt - a.alt) * t;
      if (j || !i) profile.push({ s: total + dist * t, ground, alt });       // จุดต้นช่วงซ้ำกับจุดปลายช่วงก่อน
      if (Math.hypot(n - home.n, e - home.e) >= skip) clear = Math.min(clear, alt - ground);
    }
    legs.push({ dist, brg: ((Math.atan2(b.e - a.e, b.n - a.n) * 180 / Math.PI) % 360 + 360) % 360, minClear: clear });
    total += dist; minClear = Math.min(minClear, clear);
  }
  return { legs, total, minClear, profile };
}
