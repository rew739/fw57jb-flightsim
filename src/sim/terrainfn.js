// ภูมิประเทศสร้างอัตโนมัติแบบไม่มีขอบ: ความสูง (m เหนือระดับน้ำทะเล) เป็นฟังก์ชันของตำแหน่ง (เหนือ, ตะวันออก)
// ใช้ร่วมกันทั้งฟิสิกส์ (ตรวจแตะพื้น) และภาพ (สร้างผิวดิน) → พื้นที่เห็นกับพื้นที่ชนคือที่เดียวกัน
export const AIRFIELD_ALT = 12;          // ความสูงลานปล่อย (m)

export function makeTerrain(seed = 20261112) {
  const hash = (ix, iy, s) => {
    let h = (Math.imul(ix, 374761393) + Math.imul(iy, 668265263) + Math.imul(s + seed, 1442695041)) | 0;
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
  };
  // value noise −1…1 (ต่อเนื่องถึงอนุพันธ์อันดับสอง)
  const noise = (x, y, s = 0) => {
    const ix = Math.floor(x), iy = Math.floor(y), fx = x - ix, fy = y - iy;
    const u = fx * fx * fx * (fx * (fx * 6 - 15) + 10), v = fy * fy * fy * (fy * (fy * 6 - 15) + 10);
    const a = hash(ix, iy, s), b = hash(ix + 1, iy, s), c = hash(ix, iy + 1, s), d = hash(ix + 1, iy + 1, s);
    return 2 * (a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v) - 1;
  };
  const fbm = (x, y, oct, s) => { let a = 0, amp = 1, f = 1, n = 0; for (let i = 0; i < oct; i++) { a += amp * noise(x * f, y * f, s + i); n += amp; amp *= 0.5; f *= 2.03; } return a / n; };
  const smooth = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

  const raw = (n, e) => {
    const cont = noise(n / 26000 + 3.1, e / 26000 - 1.7, 1) + 0.45 * noise(n / 9000, e / 9000, 2);      // แผ่นดิน/ทะเล
    let h = 45 + 110 * cont;
    const mMask = Math.max(0, noise(n / 17000 - 5.2, e / 17000 + 8.3, 3) + 0.15);                         // แนวเทือกเขา
    const ridge = 1 - Math.abs(noise(n / 5200, e / 5200, 4));
    h += mMask * mMask * ridge * ridge * 1500;
    const land = smooth(-10, 60, h);
    h += 38 * fbm(n / 2600, e / 2600, 4, 10) * (0.35 + 0.65 * land) * (1 + 2.5 * mMask);                  // เนินเขา
    h += (5 * noise(n / 310, e / 310, 20) + 1.6 * noise(n / 95, e / 95, 21)) * land;                      // รายละเอียด
    return h < 0 ? h * 1.6 : h;
  };
  /** ความสูงพื้น (ติดลบ = ใต้ทะเล) · รอบจุดกำเนิดปรับเรียบเป็นลานปล่อย */
  const height = (n, e) => {
    const d = Math.hypot(n, e);
    if (d < 450) return AIRFIELD_ALT;
    const h = raw(n, e);
    if (d > 2200) return h;
    const w = smooth(2200, 450, d);
    return h + (AIRFIELD_ALT - h) * w;
  };
  /** ความหนาแน่นป่า 0–1 (ใช้ทั้งสีพื้นและการวางต้นไม้) */
  const forest = (n, e) => smooth(-0.05, 0.35, noise(n / 1400 + 11.3, e / 1400 - 4.4, 30) + 0.35 * noise(n / 330, e / 330, 31));
  const dryness = (n, e) => 0.5 + 0.5 * noise(n / 3100 - 2.2, e / 3100 + 6.1, 32);
  return { height, forest, dryness, noise, hash, siteAlt: AIRFIELD_ALT, real: false, inside: () => true };
}
