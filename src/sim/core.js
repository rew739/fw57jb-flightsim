// คณิตศาสตร์พื้นฐาน + บรรยากาศ + มวล/CG/ความเฉื่อย (ไม่ขึ้นกับเบราว์เซอร์ ใช้ใน Node ได้)
// แกนตัวเครื่อง: x ไปหน้า, y ไปขวา, z ลง · แกนโลก: NED (เหนือ, ตะวันออก, ลง) · มุมเป็นเรเดียนเว้นแต่บอกไว้

export const G = 9.80665;
export const DEG = Math.PI / 180;
export const KMH = 3.6;
export const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
export const lerp = (a, b, t) => a + (b - a) * t;

// ---------- quaternion [w, x, y, z] : หมุนเวกเตอร์จากแกนตัวเครื่อง → NED ----------
export function qmul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}
export function qrot(q, v) {            // body → NED
  const [w, x, y, z] = q, [a, b, c] = v;
  const tx = 2 * (y * c - z * b), ty = 2 * (z * a - x * c), tz = 2 * (x * b - y * a);
  return [a + w * tx + (y * tz - z * ty), b + w * ty + (z * tx - x * tz), c + w * tz + (x * ty - y * tx)];
}
export function qrotInv(q, v) {         // NED → body
  return qrot([q[0], -q[1], -q[2], -q[3]], v);
}
export function qnormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
export function qFromEuler(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2), cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2), cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}
export function eulerFromQ(q) {
  const [w, x, y, z] = q;
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.asin(clamp(2 * (w * y - z * x), -1, 1)),
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  };
}
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const wrapPi = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

// ---------- บรรยากาศ: ISA เลื่อนอุณหภูมิ (โครงการใช้ 35 °C ที่ระดับน้ำทะเล → ρ 1.145) ----------
export function density(altM, oatC = 35) {
  const T0 = oatC + 273.15, L = 0.0065, R = 287.05, h = clamp(altM, -200, 11000);
  const T = T0 - L * h;
  const p = 101325 * Math.pow(T / T0, G / (R * L));
  return p / (R * T);
}

// ---------- ตัวสุ่มที่ทำซ้ำได้ ----------
export function makeRng(seed = 12345) {
  let a = seed >>> 0;
  const u = () => { a |= 0; a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  return { uniform: u, normal: () => Math.sqrt(-2 * Math.log(u() + 1e-12)) * Math.cos(2 * Math.PI * u()) };
}

// ---------- น้ำมัน ----------
/** เติมน้ำมัน frac (0–1) ของปริมาณที่เติมได้มากสุดที่ cargo นี้ · เติมไม่เต็มให้ลดจากถังที่ใช้ก่อน (pod) ตามกฎใน spec */
export function fillFuel(data, cargoKg, frac) {
  const M = data.mass;
  const fuel = {};
  for (const t of M.tanks) fuel[t.name] = t.kg_cap;
  const cap = Math.min(M.fuel_cap_kg, M.tow_max_rail_kg - M.empty_kg - cargoKg);
  let cut = M.fuel_cap_kg - clamp(frac, 0, 1) * Math.max(cap, 0);
  for (const k of M.burn_order) { const d = Math.min(cut, fuel[k]); fuel[k] -= d; cut -= d; if (cut <= 1e-12) break; }
  return fuel;
}
export const fuelTotal = fuel => { let s = 0; for (const k in fuel) s += fuel[k]; return s; };
/** ใช้น้ำมัน kg ตามลำดับถังใน spec (POD → WING → LE → A3) · คืนปริมาณที่ใช้ได้จริง */
export function burnFuel(data, fuel, kg) {
  let need = kg;
  for (const k of data.mass.burn_order) { const d = Math.min(need, fuel[k]); fuel[k] -= d; need -= d; if (need <= 0) break; }
  return kg - need;
}

// ---------- มวล / CG / ความเฉื่อย ----------
export function massProps(data, cargoKg, fuel) {
  const M = data.mass;
  let m = M.empty_kg + cargoKg, mx = M.empty_kg * M.x_empty + cargoKg * M.x_cargo, mz = M.empty_kg * M.z_empty + cargoKg * M.z_cargo;
  const kc = cargoKg / 20;
  let Ixx = M.I_empty.Ixx + kc * M.I_cargo20.Ixx, Iyy = M.I_empty.Iyy + kc * M.I_cargo20.Iyy, Izz = M.I_empty.Izz + kc * M.I_cargo20.Izz;
  for (const t of M.tanks) {
    const f = fuel[t.name] || 0, r = f / t.I_at_ref.kg;
    m += f; mx += f * t.x; mz += f * M.z_fuel;
    Ixx += r * t.I_at_ref.Ixx; Iyy += r * t.I_at_ref.Iyy; Izz += r * t.I_at_ref.Izz;
  }
  const x = mx / m;
  return { m, x, z: mz / m, Ixx, Iyy, Izz, SM: (data.aero.NP_x - x) / data.aero.MAC, fuelKg: fuelTotal(fuel), cargoKg };
}
