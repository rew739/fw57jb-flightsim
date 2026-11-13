// ทดสอบภูมิประเทศจากแผนที่จริง: node --test 10_FlightSim_Game/tests/realterrain.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { makeRealTerrain, neToLatLon, latLonToNE, imageTile, terrainAhead } from '../src/sim/realterrain.js';

// ตารางสังเคราะห์ 9 × 9 จุด ห่างกัน 100 m (±400 m) · h = 500 + 0.2·N − 0.1·E
const META = {
  id: 'synthetic', lat0: 18.7, lon0: 99.0, m_per_deg_lat: 110700, m_per_deg_lon: 105400,
  grid: { n: 9, step: 100, half: 400, scale: 0.1 }, site_alt: 500, site_flat_m: [0, 0],
  img: { root: 16384, px: 256, levels: [{ L: 0, x0: -6, x1: 5, z0: -6, z1: 5 }, { L: 1, x0: -12, x1: 11, z0: -12, z1: 11 }, { L: 2, x0: -24, x1: 23, z0: -24, z1: 23 }, { L: 3, x0: -16, x1: 15, z0: -16, z1: 15 }] },
};
const plane = (n, e) => 500 + 0.2 * n - 0.1 * e;
function grid(fn, m = META) {
  const { n, step, half, scale } = m.grid, a = new Int16Array(n * n);
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) a[j * n + i] = Math.round(fn(half - j * step, -half + i * step) / scale);
  return a;
}
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ได้ ${a} ควรเป็น ${b} ± ${tol}`);

test('ความสูงที่จุดตารางตรงกับค่าที่เก็บ', () => {
  const T = makeRealTerrain(META, grid(plane));
  near(T.height(400, -400), plane(400, -400), 0.06, 'มุมตะวันตกเฉียงเหนือ');
  near(T.height(-200, 300), plane(-200, 300), 0.06, 'จุดกลางตาราง');
});

test('ระหว่างจุดตาราง พื้นเอียงเรียบได้ค่าตรงตามระนาบ', () => {
  const T = makeRealTerrain(META, grid(plane));
  for (const [n, e] of [[37, -251], [-123.4, 88.8], [250, 250], [-310, -15]]) near(T.height(n, e), plane(n, e), 0.08, `(${n},${e})`);
});

test('ยอดเขาเดี่ยวไม่ถูกยืดสูงเกินจริงเกิน 15 %', () => {
  const T = makeRealTerrain(META, grid((n, e) => 500 + (n === 0 && e === 0 ? 100 : 0)));
  let hi = -1e9, lo = 1e9;
  for (let n = -150; n <= 150; n += 10) for (let e = -150; e <= 150; e += 10) { const h = T.height(n, e); hi = Math.max(hi, h); lo = Math.min(lo, h); }
  near(hi, 600, 0.06, 'ยอด'); assert.ok(lo > 500 - 15, `ต่ำสุดรอบยอด ${lo}`);
});

test('นอกขอบแผนที่ใช้ความสูงที่ขอบ และ inside() บอกว่าอยู่นอก', () => {
  const T = makeRealTerrain(META, grid(plane));
  near(T.height(5000, 0), plane(400, 0), 0.06, 'เหนือขอบ');
  near(T.height(0, -9000), plane(0, -400), 0.06, 'ตะวันตกขอบ');
  assert.equal(T.inside(399, -399), true); assert.equal(T.inside(401, 0), false); assert.equal(T.inside(0, 1e6), false);
});

test('รอบลานปล่อยปรับเรียบเท่าความสูงลาน แล้วค่อย ๆ กลับเป็นพื้นจริง', () => {
  const m = { ...META, grid: { n: 81, step: 100, half: 4000, scale: 0.1 }, site_flat_m: [450, 2200], site_alt: 500 };
  const T = makeRealTerrain(m, grid(plane, m));
  assert.equal(T.siteAlt, 500);
  assert.equal(T.height(0, 0), 500); assert.equal(T.height(300, -300), 500);
  near(T.height(2300, 0), plane(2300, 0), 0.08, 'นอกรัศมีปรับเรียบ');
  const a = T.height(1000, 0); assert.ok(a > 500 && a < plane(1000, 0), `ช่วงเปลี่ยนผ่าน ${a}`);
  near(T.height(2199, 0), T.height(2201, 0), 0.6, 'ต่อเนื่องที่ขอบนอก');
});

test('แปลงตำแหน่ง ↔ ละติจูด/ลองจิจูด ไปกลับได้ค่าเดิม', () => {
  const ll = neToLatLon(META, 0, 0); near(ll.lat, 18.7, 1e-12); near(ll.lon, 99.0, 1e-12);
  const up = neToLatLon(META, 11070, 0); near(up.lat, 18.8, 1e-9, 'ขึ้นเหนือ 11.07 km = 0.1°');
  const p = latLonToNE(META, 18.588, 98.487), back = neToLatLon(META, p.n, p.e);
  near(back.lat, 18.588, 1e-9); near(back.lon, 98.487, 1e-9);
  assert.ok(p.n < 0 && p.e < 0, 'ดอยอินทนนท์อยู่ทางตะวันตกเฉียงใต้');
});

test('หน้ากากป่าอ่านค่าเป็น 0–1 และไม่มีหน้ากาก = 0', () => {
  const f = new Uint8Array(81).fill(0); f[4 * 9 + 4] = 255;                   // จุดกลาง (0, 0)
  const T = makeRealTerrain(META, grid(plane), f);
  near(T.forest(0, 0), 1, 1e-9); near(T.forest(0, 50), 0.5, 1e-9); near(T.forest(0, 100), 0, 1e-9);
  assert.equal(makeRealTerrain(META, grid(plane)).forest(0, 0), 0);
});

test('เลือกแผ่นภาพ: ใช้ระดับละเอียดสุดที่มีและไม่ละเอียดเกินขนาดชิ้นผิวดิน', () => {
  assert.deepEqual(imageTile(META, 0, 0, 16384), { L: 0, ix: 0, iz: 0, size: 16384 });
  assert.deepEqual(imageTile(META, -4096, 8192, 4096), { L: 2, ix: -1, iz: 2, size: 4096 });
  assert.deepEqual(imageTile(META, 1024, -512, 512), { L: 3, ix: 0, iz: -1, size: 2048 }, 'ชิ้นเล็กกว่าแผ่นละเอียดสุด → ใช้แผ่นที่ครอบ');
  assert.deepEqual(imageTile(META, 40960, 0, 512), { L: 2, ix: 10, iz: 0, size: 4096 }, 'นอกเขตภาพละเอียด → ถอยไประดับหยาบกว่า');
  assert.equal(imageTile(META, 6 * 16384, 0, 16384), null, 'นอกแผนที่ไม่มีภาพ');
});

test('เตือนภูมิประเทศข้างหน้า: บอกเวลาที่พื้นจะสูงถึงระดับบิน (เผื่อ 40 m) หรือ null ถ้าพ้น', () => {
  const wall = (n) => n > 1000 ? 600 : 300;                                   // หน้าผาสูง 600 m อยู่เหนือขึ้นไป 1 km
  assert.equal(terrainAhead(() => 300, [0, 0, -500], [55, 0, 0]), null, 'พื้นราบ บินระดับสูง 200 m');
  assert.equal(terrainAhead(wall, [0, 0, -500], [55, 0, 0]), 20, 'บินเข้าหาหน้าผาที่ 55 m/s → ถึงใน 18.2 s ปัดขึ้นเป็นช่วง 4 s');
  assert.equal(terrainAhead(wall, [0, 0, -500], [0, 55, 0]), null, 'บินขนานหน้าผา');
  assert.equal(terrainAhead(wall, [0, 0, -500], [55, 0, -9]), null, 'ไต่ 9 m/s ข้ามพ้น (ถึงหน้าผาที่ความสูง 680 m)');
  assert.equal(terrainAhead(wall, [0, 0, -500], [55, 0, -6]), 20, 'ไต่ 6 m/s เหลือระยะไม่ถึง 40 m');
});

// ---------- ข้อมูลจริง (ข้ามถ้ายังไม่ได้ดาวน์โหลดแผนที่) ----------
const dir = new URL('../maps/chiangmai/', import.meta.url);
const have = fs.existsSync(new URL('meta.json', dir)) && fs.existsSync(new URL('elev.bin', dir));
test('แผนที่เชียงใหม่: ความสูงตรงกับสถานที่จริง', { skip: !have && 'ยังไม่มี maps/chiangmai (รัน tools/fetch_map.py)' }, () => {
  const meta = JSON.parse(fs.readFileSync(new URL('meta.json', dir), 'utf8'));
  const buf = fs.readFileSync(new URL('elev.bin', dir));
  const elev = new Int16Array(buf.buffer, buf.byteOffset, buf.byteLength / 2);
  assert.equal(elev.length, meta.grid.n * meta.grid.n, 'ขนาดไฟล์ความสูงตรงกับ meta');
  const T = makeRealTerrain(meta, elev);
  const peak = (lat, lon, r) => { const c = latLonToNE(meta, lat, lon); let m = -1e9; for (let dn = -r; dn <= r; dn += 40) for (let de = -r; de <= r; de += 40) m = Math.max(m, T.height(c.n + dn, c.e + de)); return m; };
  const inth = peak(18.5880, 98.4870, 1500), pui = peak(18.8322, 98.8847, 3000);
  assert.ok(inth > 2480 && inth < 2590, `ดอยอินทนนท์ (จริง 2,565 m) ได้ ${inth.toFixed(0)}`);
  assert.ok(pui > 1600 && pui < 1710, `ดอยปุย (จริง 1,685 m) ได้ ${pui.toFixed(0)}`);
  const city = latLonToNE(meta, 18.7883, 98.9853), hc = T.height(city.n, city.e);
  assert.ok(hc > 295 && hc < 325, `กลางเมืองเชียงใหม่ (จริงราว 310 m) ได้ ${hc.toFixed(0)}`);
  assert.ok(Math.abs(T.siteAlt - meta.site_alt) < 1e-9 && T.height(0, 0) === T.siteAlt, 'ลานปล่อยอยู่ที่ความสูงลาน');
  let rough = 0; for (let a = 0; a < 6.28; a += 0.3) rough = Math.max(rough, Math.abs(T.height(2500 * Math.cos(a), 2500 * Math.sin(a)) - T.siteAlt));
  assert.ok(rough < 25, `รอบลานปล่อย 2.5 km ต้องเป็นที่ราบ (ต่างสูงสุด ${rough.toFixed(1)} m)`);
});
