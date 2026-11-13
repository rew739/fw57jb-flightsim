// ทดสอบกลไกบังคับ elevon (แขนเซอร์โว → ก้านดัน → horn): node --test 10_FlightSim_Game/tests/linkage.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { rotateAbout, solveStation } from '../src/view/linkage.js';
import { decodeModel } from '../src/view/cadmesh.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ได้ ${a} ควรเป็น ${b} ± ${tol}`);
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const D = Math.PI / 180;

test('หมุนจุดรอบแกน: จุดชายหลังหมุนรอบแกนที่ชี้ขวาด้วยมุมบวก → ลงล่าง', () => {
  const r = rotateAbout([0, 0, 1], [0, 0, 0], [1, 0, 0], 90 * D);
  near(r[1], -1, 1e-12, 'y'); near(r[2], 0, 1e-12, 'z');
});

test('กลไกสี่เหลี่ยมด้านขนาน (แขน = horn, แกนขนานกัน): มุมแขนเท่ามุม elevon และก้านไม่เอียง', () => {
  const st = { S: [0, 0, 0], s: [1, 0, 0], A0: [0, -1, 0], H0: [0, -1, 5], hinge: { p: [0, 0, 5], axis: [1, 0, 0] } };
  const r = solveStation(st, 0.3);
  near(r.theta, 0.3, 1e-8, 'มุมแขน');
  near(dist(r.A, r.H), 5, 1e-9, 'ความยาวก้าน');
  near(Math.abs(r.q[3]), 1, 1e-9, 'ก้านต้องเลื่อนขนาน ไม่หมุน');
});

test('horn ยาวสองเท่าของแขน: แขนหมุนประมาณสองเท่าของ elevon ที่มุมเล็ก', () => {
  const st = { S: [0, 0, 0], s: [1, 0, 0], A0: [0, -1, 0], H0: [0, -2, 5], hinge: { p: [0, 0, 5], axis: [1, 0, 0] } };
  near(solveStation(st, 0.01).theta, 0.02, 2e-4, 'มุมแขน');
});

test('มุม elevon เป็นศูนย์ → ทุกชิ้นอยู่ท่าเดิม', () => {
  const st = { S: [0.1, 0, 0], s: [0, 0, 1], A0: [0.1, -0.02, 0], H0: [0.18, -0.04, 0.001], hinge: { p: [0.19, 0, 0], axis: [0.96, 0, 0.28] } };
  const r = solveStation(st, 0);
  near(r.theta, 0, 1e-9, 'มุมแขน'); assert.deepEqual(r.q.map(v => +v.toFixed(9)), [0, 0, 0, 1]);
});

test('กลไกล็อก (เกินระยะเอื้อม) → คืนมุมล่าสุดที่ทำได้ ไม่คืน NaN', () => {
  const st = { S: [0, 0, 0], s: [1, 0, 0], A0: [0, -0.2, 0], H0: [0, -3, 5], hinge: { p: [0, 0, 5], axis: [1, 0, 0] } };
  const r = solveStation(st, 1.4);
  assert.ok(Number.isFinite(r.theta) && r.q.every(Number.isFinite), 'ต้องเป็นตัวเลข');
});

// ---------- ไฟล์จริงจาก CAD ----------
const DIR = new URL('../models/fw57jb/', import.meta.url);
const meta = JSON.parse(fs.readFileSync(new URL('model.json', DIR), 'utf8'));
const data = JSON.parse(fs.readFileSync(new URL('../data/fw57jb_sim.json', import.meta.url), 'utf8'));
const KEYS = ['R0', 'R1', 'L0', 'L1'];
const station = k => ({ ...meta.linkage[k], hinge: meta.hinges[k[0]] });

test('โมเดลจาก CAD: มีข้อมูลกลไก 4 สถานี และชุดชิ้นขยับ (แขน + ก้าน) แยกออกมาครบ', () => {
  const bin = fs.readFileSync(new URL(meta.bin, DIR));
  const groups = new Set(decodeModel(meta, bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength)).map(m => m.group));
  for (const k of KEYS) {
    assert.ok(meta.linkage?.[k], 'ไม่มี linkage ' + k);
    assert.ok(groups.has('arm_' + k) && groups.has('rod_' + k), 'ไม่มีชุดชิ้นของ ' + k);
    assert.ok((k[0] === 'R' ? 1 : -1) * meta.linkage[k].S[0] > 0.5, `เซอร์โว ${k} อยู่ผิดข้าง`);
  }
});

test('โมเดลจาก CAD: ตลอดช่วง elevon ±22.5° กลไกแก้ได้ ความยาวก้านคงที่ และแขนไม่พลิกเกิน 80°', () => {
  for (const k of KEYS) {
    const st = station(k), L = dist(st.A0, st.H0);
    assert.ok(L > 0.05 && L < 0.15, `ความยาวก้าน ${k} = ${L}`);
    for (let d = -22.5; d <= 22.5; d += 2.5) {
      const r = solveStation(st, d * D);
      near(dist(r.A, r.H), L, 1e-7, `ก้าน ${k} ที่ ${d}°`);
      assert.ok(Math.abs(r.theta) < 80 * D, `แขน ${k} ที่ ${d}° หมุน ${(r.theta / D).toFixed(1)}°`);
    }
  }
});

test('โมเดลจาก CAD: อัตราทดแขนต่อ elevon ที่มุมเล็กใกล้ค่าออกแบบ 2.1 : 1 และซ้ายขวาเท่ากัน', () => {
  const ratio = k => Math.abs(solveStation(station(k), 1 * D).theta - solveStation(station(k), -1 * D).theta) / (2 * D);
  for (const k of KEYS) assert.ok(ratio(k) > 1.7 && ratio(k) < 2.6, `อัตราทด ${k} = ${ratio(k).toFixed(2)}`);
  near(ratio('R0'), ratio('L0'), 0.02, 'สถานีใน'); near(ratio('R1'), ratio('L1'), 0.02, 'สถานีนอก');
});
