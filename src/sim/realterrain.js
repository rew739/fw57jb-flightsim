// ภูมิประเทศจากแผนที่จริง: ตารางความสูง (จาก tools/fetch_map.py) → ฟังก์ชัน height(เหนือ, ตะวันออก) แบบเดียวกับ terrainfn.js
// พิกัด: จุดกำเนิด = ลานปล่อย (lat0, lon0) · ระนาบสัมผัสแบบแบน: N = (lat − lat0)·m_per_deg_lat, E = (lon − lon0)·m_per_deg_lon
// ตาราง elev[j·n + i]: i ไปทางตะวันออก (E = −half + i·step), j ไปทางใต้ (N = half − j·step) · ค่า int16 × grid.scale = เมตร

export const neToLatLon = (meta, n, e) => ({ lat: meta.lat0 + n / meta.m_per_deg_lat, lon: meta.lon0 + e / meta.m_per_deg_lon });
export const latLonToNE = (meta, lat, lon) => ({ n: (lat - meta.lat0) * meta.m_per_deg_lat, e: (lon - meta.lon0) * meta.m_per_deg_lon });

/** แผ่นภาพที่ใช้กับชิ้นผิวดิน (มุมต่ำสุด x = ตะวันออก, z = −เหนือ, ขนาด size) → { L, ix, iz, size } หรือ null ถ้านอกแผนที่ */
export function imageTile(meta, x, z, size) {
  const im = meta.img, lv = im.levels;
  for (let l = Math.min(Math.round(Math.log2(im.root / size)), lv.length - 1); l >= 0; l--) {
    const S = im.root / (1 << l), ix = Math.floor(x / S), iz = Math.floor(z / S), r = lv[l];
    if (ix >= r.x0 && ix <= r.x1 && iz >= r.z0 && iz <= r.z1) return { L: l, ix, iz, size: S };
  }
  return null;
}

/** มองไปข้างหน้าตามแนวบิน: คืนเวลา (s) ที่พื้นจะสูงถึงระดับบินโดยเผื่อ margin เมตร หรือ null ถ้าพ้นตลอด horizon วินาที
 *  pos, vNED = [เหนือ, ตะวันออก, ลง] (m, m/s) · ใช้ได้กับภูมิประเทศทั้งสองแบบ */
export function terrainAhead(height, pos, vNED, { horizon = 24, step = 4, margin = 40 } = {}) {
  for (let t = step; t <= horizon; t += step) if (height(pos[0] + vNED[0] * t, pos[1] + vNED[1] * t) > -pos[2] - vNED[2] * t - margin) return t;
  return null;
}

export function makeRealTerrain(meta, elev, forestMask = null) {
  const { n: N, step, half, scale } = meta.grid, last = N - 1;
  const siteAlt = meta.site_alt, [r0, r1] = meta.site_flat_m ?? [450, 2200];
  // Catmull-Rom 1 มิติ (ผ่านจุดข้อมูลทุกจุด ต่อเนื่องถึงความชัน → ผิวดินไม่เป็นเหลี่ยมตามตาราง)
  const cr = (p0, p1, p2, p3, t) => p1 + 0.5 * t * (p2 - p0 + t * (2 * p0 - 5 * p1 + 4 * p2 - p3 + t * (3 * (p1 - p2) + p3 - p0)));
  // แถว j (0…last) ที่ตำแหน่ง i + t · จุดนอกขอบตารางใช้การต่อเส้นตรงจากสองจุดริม
  const row = (j, i, t) => {
    const o = j * N, p1 = elev[o + i], p2 = elev[o + i + 1];
    return cr(i > 0 ? elev[o + i - 1] : 2 * p1 - p2, p1, p2, i + 2 <= last ? elev[o + i + 2] : 2 * p2 - p1, t);
  };
  const raw = (n, e) => {
    const fx = Math.min(Math.max((e + half) / step, 0), last), fz = Math.min(Math.max((half - n) / step, 0), last);
    const i = Math.min(Math.floor(fx), last - 1), j = Math.min(Math.floor(fz), last - 1), tx = fx - i, tz = fz - j;
    const q1 = row(j, i, tx), q2 = row(j + 1, i, tx);
    return scale * cr(j > 0 ? row(j - 1, i, tx) : 2 * q1 - q2, q1, q2, j + 2 <= last ? row(j + 2, i, tx) : 2 * q2 - q1, tz);
  };
  /** ความสูงพื้น (m เหนือระดับน้ำทะเล) · รอบลานปล่อยปรับเรียบเท่า siteAlt */
  const height = (n, e) => {
    if (r1 <= 0) return raw(n, e);
    const d = Math.hypot(n, e);
    if (d < r0) return siteAlt;
    const h = raw(n, e);
    if (d > r1) return h;
    const t = (r1 - d) / (r1 - r0), w = t * t * (3 - 2 * t);
    return h + (siteAlt - h) * w;
  };
  const inside = (n, e) => Math.abs(n) <= half && Math.abs(e) <= half;
  /** ความหนาแน่นป่า 0–1 จากหน้ากากที่ประมาณจากสีภาพดาวเทียม (ใช้วางต้นไม้) */
  const forest = (n, e) => {
    if (!forestMask) return 0;
    const fx = Math.min(Math.max((e + half) / step, 0), last), fz = Math.min(Math.max((half - n) / step, 0), last);
    const i = Math.min(Math.floor(fx), last - 1), j = Math.min(Math.floor(fz), last - 1), u = fx - i, v = fz - j, o = j * N + i;
    const a = forestMask[o], b = forestMask[o + 1], c = forestMask[o + N], d = forestMask[o + N + 1];
    return (a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v) / 255;
  };
  const hash = (ix, iy, s) => {
    let h = (Math.imul(ix, 374761393) + Math.imul(iy, 668265263) + Math.imul(s + 977, 1442695041)) | 0;
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
  };
  return { height, forest, dryness: () => 0.5, hash, inside, siteAlt, real: true, meta, half };
}
