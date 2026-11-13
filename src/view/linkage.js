// กลไกบังคับ elevon: แขนเซอร์โว → ก้านดัน (หัวบอลสองปลาย) → horn บน elevon
// คณิตล้วน ไม่ใช้ three.js → ทดสอบใน Node ได้ · หน่วยเมตร/เรเดียน · แกนเดียวกับโมเดล (X ขวา, Y ขึ้น, Z ไปท้าย)

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = a => mul(a, 1 / (Math.hypot(a[0], a[1], a[2]) || 1));

/** หมุนจุด v รอบแกน (ผ่านจุด p ทิศ axis หนึ่งหน่วย) ตามกฎมือขวา */
export function rotateAbout(v, p, axis, ang) {
  const r = sub(v, p), c = Math.cos(ang), s = Math.sin(ang), k = dot(axis, r) * (1 - c), x = cross(axis, r);
  return [p[0] + r[0] * c + x[0] * s + axis[0] * k, p[1] + r[1] * c + x[1] * s + axis[1] * k, p[2] + r[2] * c + x[2] * s + axis[2] * k];
}

/** quaternion [x, y, z, w] ของการหมุนสั้นที่สุดจากทิศ a ไปทิศ b (เวกเตอร์หนึ่งหน่วย) */
function shortestArc(a, b) {
  const c = cross(a, b), w = 1 + dot(a, b), l = Math.hypot(c[0], c[1], c[2], w);
  return l < 1e-12 ? [0, 0, 0, 1] : [c[0] / l, c[1] / l, c[2] / l, w / l];
}

/**
 * แก้กลไก 1 สถานี: รู้มุม elevon → หามุมแขนเซอร์โวที่ทำให้ความยาวก้านดันคงที่
 * st = { S, s: จุดบนแกนเซอร์โว + ทิศ · A0: ศูนย์บอลปลายแขน · H0: ศูนย์บอลบน horn · hinge: { p, axis } } (ท่า elevon 0°)
 * คืน { theta: มุมแขนรอบ s, A, H: ตำแหน่งบอลใหม่, q: การหมุนของก้านรอบบอลปลายแขน, locked }
 */
export function solveStation(st, delta) {
  const H = rotateAbout(st.H0, st.hinge.p, st.hinge.axis, delta), L = Math.hypot(...sub(st.A0, st.H0));
  // บอลปลายแขนเดินบนวงกลม: A(θ) = S + r∥ + r⊥·cos θ + (s × r⊥)·sin θ → |A − H|² = L² เป็นสมการ P·cos θ + Q·sin θ = K
  const r = sub(st.A0, st.S), rPar = mul(st.s, dot(st.s, r)), rPerp = sub(r, rPar), t = cross(st.s, rPerp);
  const d = sub(sub(H, st.S), rPar), P = dot(d, rPerp), Q = dot(d, t), K = (dot(rPerp, rPerp) + dot(d, d) - L * L) / 2;
  const R = Math.hypot(P, Q), x = R > 1e-15 ? K / R : 1, locked = Math.abs(x) > 1;
  const base = Math.atan2(Q, P), sw = Math.acos(Math.max(-1, Math.min(1, x)));
  const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
  const t1 = wrap(base + sw), t2 = wrap(base - sw), theta = Math.abs(t1) <= Math.abs(t2) ? t1 : t2;      // กิ่งที่ต่อเนื่องจากท่า 0°
  const A = add(add(st.S, rPar), add(mul(rPerp, Math.cos(theta)), mul(t, Math.sin(theta))));
  return { theta, A, H, q: shortestArc(unit(sub(st.H0, st.A0)), unit(sub(H, A))), locked };
}
