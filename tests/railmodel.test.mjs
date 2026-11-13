// ทดสอบโมเดลรางปล่อยที่ส่งออกจาก SolidWorks (models/lr2/ สร้างด้วย tools/export_cad_mesh.py --asm … + tools/build_rail.py)
// แกนของโมเดลราง: X = ขวา, Y = ขึ้นจากพื้น, −Z = ทิศปล่อยในแนวราบ · จุด (0, h, 0) = CG อ้างอิงของเครื่องตอนอยู่บนราง
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { decodeModel } from '../src/view/cadmesh.js';
import { railPose } from '../src/view/railpose.js';

const dir = new URL('../models/lr2/', import.meta.url);
const meta = JSON.parse(fs.readFileSync(new URL('model.json', dir)));
const bin = fs.readFileSync(new URL(meta.bin, dir));
const buf = bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength);
const meshes = decodeModel(meta, buf);
const sim = JSON.parse(fs.readFileSync(new URL('../data/fw57jb_sim.json', import.meta.url))).rail;
const DEG = Math.PI / 180;
const bbox = ms => { const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9]; for (const m of ms) for (let i = 0; i < m.position.length; i++) { const v = m.position[i], k = i % 3; if (v < lo[k]) lo[k] = v; if (v > hi[k]) hi[k] = v; } return { lo, hi }; };
const grp = g => meshes.filter(m => m.group === g);

test('ไฟล์โมเดลรางถอดรหัสได้ และมีครบ 4 ชุด: โครง แคร่ ลูกสูบ เชือก', () => {
  for (const g of ['static', 'carriage', 'piston', 'rope']) assert.ok(grp(g).length > 0, 'ไม่มีชุด ' + g);
  assert.ok(meta.ntri > 20000, `สามเหลี่ยม ${meta.ntri}`);
});

test('ขารางวางบนพื้น (y ต่ำสุด = 0) และรางสมมาตรซ้ายขวา', () => {
  const b = bbox(meshes);
  assert.ok(Math.abs(b.lo[1]) < 0.002, `y ต่ำสุด ${b.lo[1]}`);
  assert.ok(Math.abs(b.lo[0] + b.hi[0]) < 0.05, `x ${b.lo[0]} … ${b.hi[0]}`);
});

test('มุมยกและความยาวรางตรงกับค่าที่ฟิสิกส์ใช้', () => {
  const e = sim.elev_deg * DEG;
  assert.ok(Math.abs(meta.dir[0]) < 1e-3 && Math.abs(meta.dir[1] - Math.sin(e)) < 2e-3 && Math.abs(meta.dir[2] + Math.cos(e)) < 2e-3, `ทิศราง ${meta.dir}`);
  assert.ok(Math.abs(meta.rail_length - sim.length_m) < 0.05, `รางยาว ${meta.rail_length} m (ฟิสิกส์ ${sim.length_m})`);
});

test('ความสูงเครื่องบนรางใน CAD ตรงกับที่ฟิสิกส์ใช้ตั้งต้น', () => {
  assert.ok(Math.abs(meta.start[0]) < 1e-3 && Math.abs(meta.start[2]) < 1e-3);
  assert.ok(Math.abs(meta.start[1] - sim.carriage_height_m) < 0.005, `CAD ${meta.start[1]} m · ฟิสิกส์ ${sim.carriage_height_m} m`);
});

test('แคร่อยู่ใต้เครื่องตอนเริ่ม', () => {
  const b = bbox(grp('carriage')), c = [0, 1, 2].map(k => (b.lo[k] + b.hi[k]) / 2);
  assert.ok(c[1] < meta.start[1] && Math.hypot(c[0], c[1] - meta.start[1], c[2]) < 0.7, `ศูนย์แคร่ ${c}`);
});

test('แคร่วิ่งสุดระยะเร่ง + เบรกแล้วยังไม่หลุดปลายราง และลูกสูบไม่ชนฝากระบอก', () => {
  const along = p => p[1] * meta.dir[1] + p[2] * meta.dir[2];
  const b = bbox(grp('carriage')), front = Math.max(along([0, b.lo[1], b.lo[2]]), along([0, b.hi[1], b.lo[2]]));
  assert.ok(front + sim.s_accel_m + meta.brake_m <= meta.rail_front + 0.05, `หน้าแคร่ ${front} + ${sim.s_accel_m} + ${meta.brake_m} > ปลายราง ${meta.rail_front}`);
  assert.ok(sim.s_accel_m / meta.tackle <= meta.piston_room, `ช่วงชักที่ต้องใช้ ${sim.s_accel_m / meta.tackle} m · มี ${meta.piston_room} m`);
});

test('ผิวทุก mesh หันออก (ปริมาตรมีเครื่องหมายเป็นบวก)', () => {
  for (const m of meshes) {
    let v = 0; const p = m.position, I = m.index;
    for (let i = 0; i < I.length; i += 3) {
      const a = I[i] * 3, b = I[i + 1] * 3, c = I[i + 2] * 3;
      v += p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1]) - p[a + 1] * (p[b] * p[c + 2] - p[b + 2] * p[c]) + p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c]);
    }
    assert.ok(v > 0, `${m.group} สี ${m.color}: ปริมาตร ${v}`);
  }
});

test('railPose: ก่อนปล่อยทุกอย่างอยู่ที่เดิมและเห็นเชือก', () => {
  const r = railPose({ s: 0, v: 0, onRail: true }, 0, { s_accel: 4.6, brake: 1.2, tackle: 3 }, 0.02);
  assert.equal(r.carriage, 0); assert.equal(r.piston, 0); assert.equal(r.rope, true);
});

test('railPose: ระหว่างเร่ง แคร่ตามเครื่อง ลูกสูบเดิน 1/3 ของแคร่', () => {
  const r = railPose({ s: 3, v: 20, onRail: true }, { carriage: 2.9, v: 19 }, { s_accel: 4.6, brake: 1.2, tackle: 3 }, 0.02);
  assert.equal(r.carriage, 3); assert.ok(Math.abs(r.piston - 1) < 1e-9); assert.equal(r.rope, false);
});

test('railPose: เครื่องหลุดแล้วแคร่หน่วงจนหยุดภายในระยะเบรก ลูกสูบหยุดที่สุดระยะเร่ง', () => {
  const K = { s_accel: 4.6, brake: 1.2, tackle: 3 };
  let st = { carriage: 4.6, v: 28 };
  for (let i = 0; i < 400; i++) st = railPose({ onRail: false }, st, K, 0.01);
  assert.ok(st.v === 0 && st.carriage > 4.6 + 0.9 * 1.2 && st.carriage <= 4.6 + 1.2 + 1e-6, `แคร่หยุดที่ ${st.carriage}`);
  assert.ok(Math.abs(st.piston - 4.6 / 3) < 1e-9);
});
