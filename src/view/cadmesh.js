// ถอดรหัสไฟล์โมเดลที่ส่งออกจาก SolidWorks (models/fw57jb/model.json + model.bin สร้างด้วย tools/build_model.py)
// ไม่ใช้ three.js → ทดสอบใน Node ได้ · แกนโมเดล: X = ขวา, Y = ขึ้น, Z = ไปท้าย · จุดกำเนิด = CG อ้างอิง

/** คืนรายการ mesh: ข้อมูลใน meta + position/normal (Float32Array) + index */
export function decodeModel(meta, buf) {
  return meta.meshes.map(m => {
    const nI = m.ntri * 3, I = m.idxType === 'u32' ? Uint32Array : Uint16Array;
    const end = Math.max(m.pos + m.nvert * 6, m.nrm + m.nvert * 3, m.idx + nI * I.BYTES_PER_ELEMENT);
    if (end > buf.byteLength) throw new Error(`model.bin สั้นกว่าที่ model.json ระบุ (ต้องการ ${end} ไบต์ มี ${buf.byteLength})`);
    const q = new Uint16Array(buf, m.pos, m.nvert * 3), n8 = new Int8Array(buf, m.nrm, m.nvert * 3);
    const position = new Float32Array(m.nvert * 3), normal = new Float32Array(m.nvert * 3);
    for (let i = 0; i < position.length; i++) position[i] = m.min[i % 3] + q[i] / 65535 * m.ext[i % 3];
    for (let i = 0; i < normal.length; i += 3) {
      const x = n8[i], y = n8[i + 1], z = n8[i + 2], l = Math.hypot(x, y, z) || 1;
      normal[i] = x / l; normal[i + 1] = y / l; normal[i + 2] = z / l;
    }
    return { ...m, position, normal, index: new I(buf, m.idx, nI) };
  });
}

/** quaternion [x, y, z, w] ของการหมุนรอบแกนบานพับ (เวกเตอร์หนึ่งหน่วย ชี้ไปทางขวา) · มุมบวก = ชายหลังลง */
export function hingeRotation(axis, angle) {
  const s = Math.sin(angle / 2);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)];
}
