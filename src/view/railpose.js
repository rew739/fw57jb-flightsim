// ท่าของชิ้นขยับบนรางปล่อย (ใช้กับภาพเท่านั้น ไม่กระทบฟิสิกส์) · คณิตล้วน ทดสอบใน Node ได้
// ac = { onRail, s, v }: เครื่องยังอยู่บนรางไหม + ระยะ/ความเร็วตามราง · prev = ผลรอบก่อน · K = { s_accel, brake, tackle }
// คืน { carriage: ระยะแคร่ตามราง (m), piston: ระยะลูกสูบ (m), v: ความเร็วแคร่, rope: ยังแสดงเชือกท่าเริ่มไหม }
export function railPose(ac, prev, K, dt) {
  let c = prev?.carriage || 0, v = prev?.v || 0;
  const end = K.s_accel + K.brake;
  if (ac.onRail) { c = Math.min(ac.s || 0, end); v = ac.v || 0; }
  else if (v > 0) {                                   // เครื่องหลุดแล้ว: แคร่ถูกเบรกด้วยความหน่วงคงที่จนหยุดที่สุดระยะเบรก
    const left = end - c;
    if (left < 1e-3 || v < 0.05) v = 0;
    else { const a = v * v / (2 * left); c = Math.min(end, c + v * dt - 0.5 * a * dt * dt); v = Math.max(0, v - a * dt); if (v < 0.05) v = 0; }
  }
  return { carriage: c, v, piston: Math.min(c, K.s_accel) / K.tackle, rope: c < 0.1 };
}
