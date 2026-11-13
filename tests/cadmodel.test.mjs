// ทดสอบโมเดลเครื่องบินที่ส่งออกจาก SolidWorks: node --test 10_FlightSim_Game/tests/cadmodel.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { decodeModel, hingeRotation } from '../src/view/cadmesh.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ได้ ${a} ควรเป็น ${b} ± ${tol}`);

// ---------- ตัวถอดรหัส (ข้อมูลสังเคราะห์) ----------
function synthetic() {
  // สามเหลี่ยม 1 รูป: จุด (1,2,3) (2,2,3) (1,4,3) · normal +Z
  const buf = new ArrayBuffer(40), u16 = new Uint16Array(buf, 0, 9), i8 = new Int8Array(buf, 20, 9), ix = new Uint16Array(buf, 32, 3);
  u16.set([0, 0, 0, 65535, 0, 0, 0, 65535, 0]); i8.set([0, 0, 127, 0, 0, 127, 0, 0, 127]); ix.set([0, 1, 2]);
  const meta = { version: 1, meshes: [{ group: 'static', skin: true, color: [0.1, 0.2, 0.3], opacity: 1, nvert: 3, ntri: 1, min: [1, 2, 3], ext: [1, 2, 0], pos: 0, nrm: 20, idx: 32, idxType: 'u16' }] };
  return { meta, buf };
}

test('ถอดรหัสตำแหน่งจาก uint16 กลับเป็นเมตรตามกรอบของ mesh', () => {
  const { meta, buf } = synthetic(), [m] = decodeModel(meta, buf);
  assert.deepEqual(Array.from(m.position), [1, 2, 3, 2, 2, 3, 1, 4, 3]);
});

test('ถอดรหัส normal จาก int8 เป็นเวกเตอร์หนึ่งหน่วย และดัชนีสามเหลี่ยมครบ', () => {
  const { meta, buf } = synthetic(), [m] = decodeModel(meta, buf);
  assert.deepEqual(Array.from(m.normal), [0, 0, 1, 0, 0, 1, 0, 0, 1]);
  assert.deepEqual(Array.from(m.index), [0, 1, 2]);
  assert.equal(m.group, 'static');
});

test('ไฟล์ข้อมูลสั้นกว่าที่ meta บอก → แจ้งข้อผิดพลาด ไม่คืนโมเดลพัง', () => {
  const { meta, buf } = synthetic();
  assert.throws(() => decodeModel(meta, buf.slice(0, 30)), /model\.bin/);
});

test('หมุน elevon รอบแกนบานพับที่ชี้ไปทางขวา: มุมบวก = ชายหลังลง', () => {
  const q = hingeRotation([1, 0, 0], 10 * Math.PI / 180);          // quaternion [x, y, z, w]
  // จุดชายหลัง (0, 0, 1) หมุนด้วย q → y ต้องติดลบ
  const [x, y, z, w] = q, v = [0, 0, 1];
  const t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
  const r = [v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0])];
  near(r[1], -Math.sin(10 * Math.PI / 180), 1e-9, 'y ของชายหลัง');
  near(r[2], Math.cos(10 * Math.PI / 180), 1e-9, 'z ของชายหลัง');
});

// ---------- ไฟล์จริงที่สร้างจาก CAD ----------
const DIR = new URL('../models/fw57jb/', import.meta.url);
const meta = JSON.parse(fs.readFileSync(new URL('model.json', DIR), 'utf8'));
const data = JSON.parse(fs.readFileSync(new URL('../data/fw57jb_sim.json', import.meta.url), 'utf8'));
const bin = fs.readFileSync(new URL(meta.bin, DIR));
const meshes = decodeModel(meta, bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));

function bbox(ms) {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const m of ms) for (let i = 0; i < m.position.length; i++) { const k = i % 3, v = m.position[i]; if (v < lo[k]) lo[k] = v; if (v > hi[k]) hi[k] = v; }
  return { lo, hi };
}

test('โมเดลจาก CAD: กางปีกตรงกับ spec และซ้าย-ขวาสมมาตร (แกน X = ขวา)', () => {
  const { lo, hi } = bbox(meshes);
  near(hi[0] - lo[0], data.geom.b, 0.06, 'กางปีก m');
  near(hi[0] + lo[0], 0, 0.01, 'สมมาตรซ้ายขวา m');
});

test('โมเดลจาก CAD: หัวชี้ −Z ท้ายอยู่ +Z และจุดกำเนิดอยู่ที่ CG อ้างอิง x = 0.5 m', () => {
  const { lo, hi } = bbox(meshes.filter(m => m.skin));
  near(hi[2], data.geom.pod.x_tail - 0.5, 0.02, 'ปลายท่อท้าย m');
  assert.ok(lo[2] < -0.7 && lo[2] > -1.2, `หัวเครื่อง z = ${lo[2]}`);
  assert.ok(hi[1] > 0.5 && hi[1] < 0.8, `ยอด winglet y = ${hi[1]}`);
});

test('โมเดลจาก CAD: ละเอียดระดับ CAD (สามเหลี่ยมเกิน 150,000 รูป) และดัชนีไม่ชี้เกินจำนวนจุด', () => {
  let ntri = 0;
  for (const m of meshes) {
    ntri += m.index.length / 3;
    let mx = 0; for (const i of m.index) if (i > mx) mx = i;
    assert.ok(mx < m.position.length / 3, `ดัชนีเกินใน mesh ${m.group}`);
    assert.equal(m.normal.length, m.position.length);
  }
  assert.ok(ntri > 150000, `สามเหลี่ยม ${ntri}`);
  assert.equal(ntri, meta.ntri);
});

test('โมเดลจาก CAD: มีชุด elevon ซ้าย/ขวาแยกจากลำตัว อยู่ถูกข้าง และแกนบานพับผ่านชุดนั้น', () => {
  for (const [side, sgn] of [['R', 1], ['L', -1]]) {
    const ms = meshes.filter(m => m.group === 'elevon_' + side);
    assert.ok(ms.length > 0, 'ไม่มี elevon_' + side);
    const { lo, hi } = bbox(ms), h = meta.hinges[side];
    assert.ok(sgn * (lo[0] + hi[0]) > 0, `elevon ${side} อยู่ผิดข้าง`);
    near(lo[0] * sgn > hi[0] * sgn ? hi[0] * sgn : lo[0] * sgn, data.geom.elevon_y[0], 0.08, `ขอบใน elevon ${side}`);
    near(Math.hypot(...h.axis), 1, 1e-6, 'ความยาวแกนบานพับ');
    assert.ok(h.axis[0] > 0.8, `แกนบานพับ ${side} ต้องชี้ไปทางขวา`);
    assert.ok(h.p[0] > lo[0] && h.p[0] < hi[0] && h.p[2] > lo[2] - 0.05 && h.p[2] < hi[2], `จุดบนแกนบานพับ ${h.p} อยู่นอกชุด elevon`);
  }
});

test('โมเดลจาก CAD: normal ที่เก็บไว้ไปทางเดียวกับด้านหน้าของสามเหลี่ยม (ลำดับจุดถูกต้อง)', () => {
  let ok = 0, n = 0;
  for (const m of meshes) {
    const P = m.position, N = m.normal, I = m.index;
    for (let t = 0; t < I.length; t += 3) {
      const a = I[t] * 3, b = I[t + 1] * 3, c = I[t + 2] * 3;
      const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2], vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
      const gx = uy * vz - uz * vy, gy = uz * vx - ux * vz, gz = ux * vy - uy * vx;
      if (gx * gx + gy * gy + gz * gz < 1e-16) continue;            // สามเหลี่ยมลีบ
      n++; if (gx * (N[a] + N[b] + N[c]) + gy * (N[a + 1] + N[b + 1] + N[c + 1]) + gz * (N[a + 2] + N[b + 2] + N[c + 2]) > 0) ok++;
    }
  }
  assert.ok(ok / n > 0.995, `สามเหลี่ยมที่ normal ตรงด้านหน้า ${(100 * ok / n).toFixed(2)} %`);
});

test('โมเดลจาก CAD: ผิวหันออกนอกชิ้น (ปริมาตรมีเครื่องหมายของทุก mesh เป็นบวก)', () => {
  // ทุก mesh = ชิ้นตันปิดหลายชิ้นรวมกัน → ถ้าลำดับจุดหันออก ผลรวม (a · b × c)/6 ต้องเป็นบวก
  let total = 0;
  for (const m of meshes) {
    const P = m.position, I = m.index; let v = 0;
    for (let t = 0; t < I.length; t += 3) {
      const a = I[t] * 3, b = I[t + 1] * 3, c = I[t + 2] * 3;
      v += P[a] * (P[b + 1] * P[c + 2] - P[b + 2] * P[c + 1]) + P[a + 1] * (P[b + 2] * P[c] - P[b] * P[c + 2]) + P[a + 2] * (P[b] * P[c + 1] - P[b + 1] * P[c]);
    }
    assert.ok(v > 0, `mesh ${m.group} สี ${m.color} ปริมาตร ${(v / 6 * 1e6).toFixed(1)} cm³ ติดลบ = ผิวกลับด้าน`);
    total += v / 6;
  }
  assert.ok(total > 0.01 && total < 0.2, `ปริมาตรเนื้อวัสดุรวม ${(total * 1000).toFixed(1)} ลิตร`);
});
