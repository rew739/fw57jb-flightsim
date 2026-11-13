// โลกเปิด: ผิวดินแบบ quadtree LOD (สร้าง/ทิ้งตามตำแหน่งกล้อง) + ทะเล + ท้องฟ้า + เมฆ + ต้นไม้ + ลานปล่อย
// แกนภาพ (three.js): x = ตะวันออก, y = ขึ้น, z = ใต้ (เหนือ = −z)
import * as THREE from 'three';
import { imageTile } from '../sim/realterrain.js';
import { decodeModel } from './cadmesh.js';
import { railPose } from './railpose.js';

const ROOT = 16384, MIN = 512, SEG = 32, SPLIT = 1.7;
const SUN_DIR = new THREE.Vector3(0.45, 0.72, 0.53).normalize();
const HORIZON = new THREE.Color(0xbdd3e6);

function mergeGeoms(parts) {            // รวม geometry เล็ก ๆ (position/normal/color) เป็นก้อนเดียว
  let nv = 0, ni = 0;
  for (const p of parts) { nv += p.g.attributes.position.count; ni += p.g.index.count; }
  const pos = new Float32Array(nv * 3), nor = new Float32Array(nv * 3), col = new Float32Array(nv * 3), idx = new Uint32Array(ni);
  let ov = 0, oi = 0;
  for (const p of parts) {
    const g = p.g, n = g.attributes.position.count;
    pos.set(g.attributes.position.array, ov * 3); nor.set(g.attributes.normal.array, ov * 3);
    for (let i = 0; i < n; i++) { col[(ov + i) * 3] = p.c.r; col[(ov + i) * 3 + 1] = p.c.g; col[(ov + i) * 3 + 2] = p.c.b; }
    for (let i = 0; i < g.index.count; i++) idx[oi + i] = g.index.array[i] + ov;
    ov += n; oi += g.index.count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3)); out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  out.setAttribute('color', new THREE.BufferAttribute(col, 3)); out.setIndex(new THREE.BufferAttribute(idx, 1));
  return out;
}

function detailTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256;
  const g = c.getContext('2d'), im = g.createImageData(256, 256);
  let s = 1234567;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) | 0) >>> 0) / 4294967296;
  const base = new Float32Array(256 * 256);
  for (let o = 0; o < 4; o++) {           // สัญญาณรบกวนหลายความถี่ ต่อเนื่องที่ขอบ
    const n = 8 << o, grid = new Float32Array(n * n).map(rnd), amp = 1 / (1 << o);
    for (let y = 0; y < 256; y++) for (let x = 0; x < 256; x++) {
      const fx = x / 256 * n, fy = y / 256 * n, ix = Math.floor(fx), iy = Math.floor(fy), u = fx - ix, v = fy - iy;
      const a = grid[iy * n + ix], b = grid[iy * n + (ix + 1) % n], cc = grid[((iy + 1) % n) * n + ix], d = grid[((iy + 1) % n) * n + (ix + 1) % n];
      base[y * 256 + x] += amp * (a + (b - a) * u + (cc - a) * v + (a - b - cc + d) * u * v);
    }
  }
  for (let i = 0; i < 256 * 256; i++) { const v = Math.round(255 * (0.74 + 0.26 * base[i] / 1.875)); im.data[i * 4] = im.data[i * 4 + 1] = im.data[i * 4 + 2] = v; im.data[i * 4 + 3] = 255; }
  g.putImageData(im, 0, 0);
  const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.anisotropy = 8; t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function puffTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 128;
  const g = c.getContext('2d'), gr = g.createRadialGradient(64, 64, 4, 64, 64, 64);
  gr.addColorStop(0, 'rgba(255,255,255,0.95)'); gr.addColorStop(0.5, 'rgba(250,252,255,0.55)'); gr.addColorStop(1, 'rgba(245,250,255,0)');
  g.fillStyle = gr; g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

class Node {
  constructor(x, z, size) { this.x = x; this.z = z; this.size = size; this.mesh = null; this.trees = null; this.kids = null; this.queued = false; }
}

export class World {
  /** terrain = makeTerrain() · พิกัดจำลอง (เหนือ n, ตะวันออก e) ↔ ภาพ (x = e, z = −n) */
  constructor(scene, terrain) {
    this.scene = scene; this.T = terrain;
    this.roots = new Map(); this.queue = [];
    this.group = new THREE.Group(); scene.add(this.group);
    this.detail = detailTexture();
    this.mat = new THREE.MeshLambertMaterial({ vertexColors: true, map: this.detail, side: THREE.DoubleSide });
    this.tiles = new Map(); this.idle = []; this.texLoader = new THREE.TextureLoader();     // แผ่นภาพดาวเทียม (เฉพาะแผนที่จริง)
    this.treeMat = new THREE.MeshLambertMaterial({ vertexColors: true });
    const cone = new THREE.ConeGeometry(3.4, 9, 6); cone.translate(0, 8, 0);
    const cone2 = new THREE.ConeGeometry(2.4, 6, 6); cone2.translate(0, 12, 0);
    const trunk = new THREE.CylinderGeometry(0.35, 0.5, 4, 5); trunk.translate(0, 2, 0);
    this.treeGeo = mergeGeoms([{ g: cone, c: new THREE.Color(0x2c5a26) }, { g: cone2, c: new THREE.Color(0x356b2c) }, { g: trunk, c: new THREE.Color(0x4f3a26) }]);
    this._sky(); this._water(); this._clouds();
    this.site = null;
    this.water.visible = !terrain.real;
  }

  /** สลับแผนที่: ทิ้งผิวดินและแผ่นภาพเดิมทั้งหมด แล้วสร้างใหม่จากภูมิประเทศ t */
  setTerrain(t) {
    for (const node of this.roots.values()) this._free(node);
    this.roots.clear(); this.queue.length = 0;
    for (const tile of this.tiles.values()) this._dropTile(tile);
    this.tiles.clear(); this.idle.length = 0;
    this.T = t; this.water.visible = !t.real;
  }

  // ---------- แผ่นภาพดาวเทียม: โหลดตามที่ผิวดินต้องใช้ ระหว่างรอใช้แผ่นหยาบกว่าที่โหลดแล้ว ----------
  _tile(t) {
    const key = t.L + '/' + t.ix + '_' + t.iz;
    let tile = this.tiles.get(key);
    if (tile) return tile;
    tile = { key, L: t.L, x0: t.ix * t.size, z0: t.iz * t.size, size: t.size, ready: false, failed: false, users: new Set(), waiters: new Set(), mat: null, T: this.T };
    this.tiles.set(key, tile);
    const m = this.T.meta;
    this.texLoader.load(`${m.url}/${m.img.path}/${key}.jpg`, tex => {
      if (tile.T !== this.T || this.tiles.get(key) !== tile) { tex.dispose(); return; }        // สลับแผนที่ไปแล้ว
      tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 8;
      const mat = new THREE.MeshLambertMaterial({ map: tex, side: THREE.DoubleSide });
      // ลายพื้นละเอียดซ้อนบนภาพดาวเทียม (ภาพจริงละเอียดสุด 8 m/จุด มองใกล้พื้นจะเบลอ) จางหายเมื่อไกล 2.5 km
      mat.onBeforeCompile = sh => {
        sh.uniforms.detailMap = { value: this.detail };
        sh.vertexShader = sh.vertexShader.replace('#include <common>', '#include <common>\nvarying vec2 vDetailUv;')
          .replace('#include <begin_vertex>', '#include <begin_vertex>\nvDetailUv = (modelMatrix * vec4(position, 1.0)).xz / 23.0;');
        sh.fragmentShader = sh.fragmentShader.replace('#include <common>', '#include <common>\nuniform sampler2D detailMap; varying vec2 vDetailUv;')
          .replace('#include <map_fragment>', `#include <map_fragment>
            float dFar = clamp(length(vViewPosition) / 2500.0, 0.0, 1.0);
            float dTex = texture2D(detailMap, vDetailUv).r * 0.6 + texture2D(detailMap, vDetailUv * 0.131).r * 0.4;
            diffuseColor.rgb *= mix(0.58 + 0.66 * dTex, 1.0, dFar);`);
      };
      mat.customProgramCacheKey = () => 'sat-detail';
      tile.mat = mat; tile.ready = true;
      for (const node of tile.waiters) if (node.mesh && (!node.tile || node.tile.L < tile.L)) this._bind(node, tile);
      tile.waiters.clear(); this._idleCheck(tile);
    }, undefined, () => { tile.failed = true; tile.waiters.clear(); });
    return tile;
  }
  _dropTile(tile) { if (tile.mat) { tile.mat.map.dispose(); tile.mat.dispose(); tile.mat = null; } tile.ready = false; }
  _idleCheck(tile) {                      // แผ่นที่ไม่มีใครใช้: เก็บไว้ 48 แผ่นล่าสุด เกินนั้นคืนหน่วยความจำ
    if (tile.users.size || tile.waiters.size || !tile.ready) return;
    this.idle.push(tile);
    while (this.idle.length > 48) { const old = this.idle.shift(); if (!old.users.size && !old.waiters.size && this.tiles.get(old.key) === old) { this._dropTile(old); this.tiles.delete(old.key); } }
  }
  _bind(node, tile) {                     // ให้ชิ้นผิวดินใช้แผ่นภาพ: เขียน uv ใหม่ตามตำแหน่งในแผ่น
    const g = node.mesh.geometry, pos = g.attributes.position.array, uv = g.attributes.uv.array;
    const cx = node.x + node.size / 2 - tile.x0, cz = node.z + node.size / 2 - tile.z0, k = 1 / tile.size;
    for (let i = 0, n = uv.length / 2; i < n; i++) { uv[i * 2] = (pos[i * 3] + cx) * k; uv[i * 2 + 1] = 1 - (pos[i * 3 + 2] + cz) * k; }
    g.attributes.uv.needsUpdate = true;
    if (node.tile) { node.tile.users.delete(node); this._idleCheck(node.tile); }
    node.tile = tile; tile.users.add(node); node.mesh.material = tile.mat;
  }
  _texture(node) {                        // เรียกหลังสร้างชิ้นผิวดิน (แผนที่จริงเท่านั้น)
    const meta = this.T.meta, want = imageTile(meta, node.x, node.z, node.size);
    if (!want) return;                    // นอกแผนที่: ใช้สีพื้นธรรมดา
    const best = this._tile(want);
    if (best.ready) { this._bind(node, best); return; }
    if (!best.failed) best.waiters.add(node);
    for (let L = want.L - 1; L >= 0; L--) {             // ระหว่างรอ: แผ่นหยาบกว่าที่พร้อมแล้ว (ขอแผ่นหยาบสุดไว้เสมอ)
      const S = meta.img.root / (1 << L), ix = Math.floor(node.x / S), iz = Math.floor(node.z / S);
      const t = L === 0 ? this._tile({ L, ix, iz, size: S }) : this.tiles.get(L + '/' + ix + '_' + iz);
      if (t?.ready) { this._bind(node, t); return; }
      if (L === 0 && !t.failed) t.waiters.add(node);
    }
  }

  _sky() {
    const s = this.scene;
    s.fog = new THREE.FogExp2(HORIZON.getHex(), 0.000042);
    s.background = HORIZON.clone();
    const geo = new THREE.SphereGeometry(70000, 32, 16);
    const mat = new THREE.ShaderMaterial({
      side: THREE.BackSide, depthWrite: false, fog: false,
      uniforms: { top: { value: new THREE.Color(0x2f6fbf) }, hor: { value: HORIZON.clone() }, sun: { value: SUN_DIR } },
      vertexShader: 'varying vec3 d; void main(){ d = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
      fragmentShader: `varying vec3 d; uniform vec3 top; uniform vec3 hor; uniform vec3 sun;
        void main(){ float h = clamp(d.y, 0.0, 1.0); vec3 c = mix(hor, top, pow(h, 0.55));
          float s = max(dot(normalize(d), sun), 0.0); c += vec3(1.0,0.95,0.8) * (pow(s, 600.0) * 2.0 + pow(s, 12.0) * 0.18);
          gl_FragColor = vec4(c, 1.0); }`,
    });
    this.sky = new THREE.Mesh(geo, mat); this.sky.renderOrder = -10; this.sky.frustumCulled = false; s.add(this.sky);
    s.add(new THREE.HemisphereLight(0xd6e6ff, 0x7a7560, 1.15));
    const sun = new THREE.DirectionalLight(0xfff1d8, 2.2); sun.position.copy(SUN_DIR).multiplyScalar(1000); s.add(sun);
  }

  _water() {
    const mat = new THREE.MeshPhongMaterial({ color: 0x1d5e7e, specular: 0x9fb8c8, shininess: 70 });
    this.water = new THREE.Mesh(new THREE.PlaneGeometry(300000, 300000), mat);
    this.water.rotation.x = -Math.PI / 2; this.scene.add(this.water);
  }

  _clouds() {
    const mat = new THREE.SpriteMaterial({ map: puffTexture(), transparent: true, depthWrite: false, opacity: 0.9 });
    this.clouds = []; this.cloudSpan = 70000;
    let s = 99;
    const rnd = () => ((s = (Math.imul(s, 1103515245) + 12345) | 0) >>> 0) / 4294967296;
    for (let i = 0; i < 70; i++) {
      const cx = (rnd() - 0.5) * this.cloudSpan, cz = (rnd() - 0.5) * this.cloudSpan, cy = 1500 + rnd() * 1100, big = 500 + rnd() * 900;
      for (let j = 0; j < 5; j++) {
        const sp = new THREE.Sprite(mat);
        sp.userData.base = new THREE.Vector3(cx + (rnd() - 0.5) * big * 1.8, cy + (rnd() - 0.3) * big * 0.25, cz + (rnd() - 0.5) * big * 1.8);
        const w = big * (0.7 + rnd() * 0.7); sp.scale.set(w, w * 0.55, 1);
        this.scene.add(sp); this.clouds.push(sp);
      }
    }
  }

  /** เสาแสงบอกตำแหน่งจุดบินในฉาก 3 มิติ (วงแหวน = ความสูงที่วางแผน) */
  setRoute(wps) {
    if (this.route) { this.scene.remove(this.route); this.route.traverse(o => { if (o.geometry) o.geometry.dispose(); }); }
    this.route = new THREE.Group();
    this.routeMat ??= new THREE.MeshBasicMaterial({ color: 0xffc04a, transparent: true, opacity: 0.45, depthWrite: false, fog: false, side: THREE.DoubleSide });
    for (const p of wps) {
      const ground = Math.max(this.T.height(p.n, p.e), 0), hgt = Math.max(p.alt - ground, 20);
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(7, 7, hgt, 8, 1, true), this.routeMat); pole.position.set(p.e, ground + hgt / 2, -p.n);
      const ring = new THREE.Mesh(new THREE.TorusGeometry(60, 5, 6, 28), this.routeMat); ring.rotation.x = Math.PI / 2; ring.position.set(p.e, p.alt, -p.n);
      this.route.add(pole, ring);
    }
    this.scene.add(this.route);
  }

  // ---------- รางปล่อยจาก CAD (models/lr2/ — tools/build_rail.py) ----------
  /** โหลดรูปทรงรางจริงจาก SolidWorks แล้วแทนรางกล่องสำรอง · คืน meta (หรือ null ถ้าไม่มีไฟล์) */
  async loadRail(dir = 'models/lr2/') {
    let meta, buf;
    try {
      const r = await fetch(dir + 'model.json'); if (!r.ok) return null;
      meta = await r.json();
      const rb = await fetch(dir + meta.bin); if (!rb.ok) return null;
      buf = await rb.arrayBuffer();
    } catch (e) { return null; }
    const parts = decodeModel(meta, buf).map(m => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(m.position, 3)); g.setAttribute('normal', new THREE.BufferAttribute(m.normal, 3));
      g.setIndex(new THREE.BufferAttribute(m.index, 1));
      const col = new THREE.Color().setRGB(m.color[0], m.color[1], m.color[2], THREE.SRGBColorSpace);
      const mat = new THREE.MeshPhongMaterial({ color: col, emissive: col.clone().multiplyScalar(0.25), specular: new THREE.Color().setScalar(0.35 * m.specular + 0.04), shininess: 18 + 90 * m.shininess });
      return { group: m.group, g, mat };
    });
    this.railCad = { meta, parts };
    if (this.siteArgs) this.buildSite(...this.siteArgs);
    return meta;
  }

  _addCadRail(rg) {                                    // แกนโมเดลราง = แกนของกลุ่ม rg (−z = ทิศปล่อย, y = ขึ้นจากพื้น)
    const gs = {};
    for (const p of this.railCad.parts) { const g = gs[p.group] ||= new THREE.Group(); g.add(new THREE.Mesh(p.g, p.mat)); }
    for (const g of Object.values(gs)) rg.add(g);
    this.railParts = gs;
  }

  /** ขยับแคร่ + ลูกสูบตามการปล่อย (ภาพเท่านั้น) · ac = เครื่องบินของฟิสิกส์ */
  updateRail(ac, dt, rail) {
    const m = this.railCad?.meta, K = { s_accel: rail.s_accel_m, brake: m?.brake_m ?? 0.9, tackle: m?.tackle ?? 3 };
    const st = this.railState = railPose({ onRail: ac.phase === 'rail', s: ac.rail?.s, v: ac.rail?.v }, this.railState, K, dt);
    if (this.railParts) {
      const d = m.dir, P = this.railParts;
      P.carriage?.position.set(d[0] * st.carriage, d[1] * st.carriage, d[2] * st.carriage);
      P.piston?.position.set(d[0] * st.piston, d[1] * st.piston, d[2] * st.piston);
      if (P.rope) P.rope.visible = st.rope;
    } else if (this.carriage) this.carriage.position.z = -st.carriage;
  }

  // ---------- ลานปล่อย: ราง ถังลม เต็นท์สถานีควบคุม รถ ถุงลม ----------
  buildSite(headingDeg, rail) {
    if (this.site) this.scene.remove(this.site);
    this.siteArgs = [headingDeg, rail]; this.railState = null; this.railParts = null; this.carriage = null;
    const g = new THREE.Group(), y0 = this.T.siteAlt;
    const lam = c => new THREE.MeshLambertMaterial({ color: c });
    const box = (w, h, d, c, x, y, z, parent = g) => { const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), lam(c)); m.position.set(x, y, z); parent.add(m); return m; };
    // กลุ่มรางหมุนตามทิศปล่อย (แกน −z ของกลุ่ม = ทิศราง)
    const rg = new THREE.Group(); rg.position.set(0, y0, 0); rg.rotation.y = -headingDeg * Math.PI / 180; g.add(rg);
    box(60, 0.12, 170, 0x8f8a7c, 0, -0.05, -45, rg);                      // ลานกรวด + เส้นแนวปล่อย (หันตามราง) · ผิวลานสูง 1 cm ให้ขารางจาก CAD วางพอดี
    box(1.2, 0.02, 150, 0xd8d4c4, 0, 0.012, -55, rg);
    if (this.railCad) this._addCadRail(rg); else {
    const e = rail.elev_deg * Math.PI / 180, L = rail.length_m, hc = rail.carriage_height_m;
    const beam = new THREE.Group(); beam.position.set(0, hc - 0.32, 0); beam.rotation.x = e; rg.add(beam);
    box(0.22, 0.26, L, 0x55606b, 0, 0, -(L / 2 - 0.9), beam);
    box(0.5, 0.05, L, 0x8a939c, 0, 0.15, -(L / 2 - 0.9), beam);
    const cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 1.9, 12), lam(0xb9bfc6)); cyl.rotation.x = Math.PI / 2; cyl.position.set(0, -0.3, -0.3); beam.add(cyl);
    this.carriage = box(0.55, 0.16, 0.8, 0xd9822b, 0, 0.26, 0, beam);
    for (const s of [0.3, L * 0.5, L - 1.3]) {             // ขารับราง
      const yTop = hc - 0.4 + Math.sin(e) * s;
      for (const sx of [-1, 1]) { const leg = box(0.08, yTop, 0.08, 0x4a525a, sx * 0.55, yTop / 2, -Math.cos(e) * s, rg); leg.rotation.z = -sx * 0.18; }
    }
    const tank = new THREE.Mesh(new THREE.CylinderGeometry(0.33, 0.33, 2.4, 16), lam(0x2f6da8)); tank.rotation.z = Math.PI / 2; tank.position.set(0, 0.45, 1.8); rg.add(tank);
    box(1.6, 0.9, 1.0, 0x6b7178, 0, 0.45, 3.4, rg);
    }
    // สถานีควบคุม + รถ + คน (ให้เห็นสเกล)
    box(4, 2.4, 3, 0xe7e3d6, 14, y0 + 1.2, 18);
    const roof = new THREE.Mesh(new THREE.ConeGeometry(3.1, 1.2, 4), lam(0x3f7fb5)); roof.position.set(14, y0 + 3.0, 18); roof.rotation.y = Math.PI / 4; g.add(roof);
    box(1.9, 0.9, 4.6, 0xc8c8c8, 21, y0 + 0.75, 20); box(1.8, 0.8, 1.9, 0xb5b5b5, 21, y0 + 1.6, 19.4);
    for (const [px, pz] of [[11, 14], [12.2, 14.6], [-7, 9]]) { const p = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.7, 8), lam(0x2d3e55)); p.position.set(px, y0 + 0.85, pz); g.add(p); }
    const mast = box(0.08, 6, 0.08, 0x888888, 16.5, y0 + 3, 16);
    // ถุงลม
    const pole = box(0.07, 5, 0.07, 0xdddddd, -14, y0 + 2.5, 6);
    const sock = new THREE.Mesh(new THREE.ConeGeometry(0.35, 2.2, 10, 1, true), new THREE.MeshLambertMaterial({ color: 0xf07a1a, side: THREE.DoubleSide }));
    sock.geometry.rotateX(-Math.PI / 2); sock.geometry.translate(0, 0, 1.1);
    this.sock = new THREE.Group(); this.sock.position.set(-14, y0 + 5, 6); this.sock.add(sock); g.add(this.sock);
    this.site = g; this.scene.add(g);
    void mast; void pole;
  }
  setWind(fromDeg, speed) {          // ถุงลมชี้ตามลม (ทิศที่ลมพัดไป) และยกขึ้นตามความแรง
    if (!this.sock) return;
    this.sock.rotation.set(0, 0, 0);
    this.sock.rotation.y = -((fromDeg + 180) * Math.PI / 180) + Math.PI;
    this.sock.children[0].rotation.x = Math.max(0, 1.2 - speed * 0.15);
  }

  // ---------- ผิวดิน ----------
  _color(h, slope, n, e, out) {
    const T = this.T;
    let r, g, b;
    if (T.real) {                           // แผนที่จริง: สีสำรองระหว่างรอภาพ / นอกขอบแผนที่
      const f = T.forest(n, e), rk = Math.min(1, Math.max(0, (slope - 0.7) / 0.5));
      r = 0.40 + (0.15 - 0.40) * f; g = 0.44 + (0.26 - 0.44) * f; b = 0.29 + (0.13 - 0.29) * f;
      r += (0.42 - r) * rk; g += (0.40 - g) * rk; b += (0.37 - b) * rk;
    }
    else if (h < -0.5) { r = 0.30; g = 0.38; b = 0.36; }
    else if (h < 2.2) { r = 0.78; g = 0.72; b = 0.52; }
    else {
      const d = T.dryness(n, e), f = T.forest(n, e);
      r = 0.30 + 0.22 * d; g = 0.47 + 0.05 * d; b = 0.19 + 0.08 * d;                 // ทุ่งหญ้า เขียว ↔ แห้ง
      const ff = f * (h > 6 ? 1 : 0.3);
      r += (0.12 - r) * ff; g += (0.29 - g) * ff; b += (0.11 - b) * ff;               // ป่า
      const rock = Math.min(1, Math.max(0, (slope - 0.45) / 0.35)) + Math.min(1, Math.max(0, (h - 900) / 500)) * 0.7;
      const rk = Math.min(1, rock);
      r += (0.46 - r) * rk; g += (0.43 - g) * rk; b += (0.40 - b) * rk;               // หิน
      if (h < 5) { const s = (5 - h) / 2.8; r += (0.78 - r) * s; g += (0.72 - g) * s; b += (0.52 - b) * s; }
    }
    const v = 0.94 + 0.12 * T.hash(Math.round(n), Math.round(e), 77);
    out[0] = r * v; out[1] = g * v; out[2] = b * v;
  }

  _build(node) {
    const T = this.T, S = node.size, n1 = SEG + 1, step = S / SEG, N = n1 + 2;
    const cx = node.x + S / 2, cz = node.z + S / 2;
    const hh = new Float32Array(N * N);
    for (let j = 0; j < N; j++) for (let i = 0; i < N; i++) hh[j * N + i] = T.height(-(node.z + (j - 1) * step), node.x + (i - 1) * step);
    const nv = n1 * n1 + 4 * n1;
    const pos = new Float32Array(nv * 3), nor = new Float32Array(nv * 3), col = new Float32Array(nv * 3), uv = new Float32Array(nv * 2);
    const c = [0, 0, 0];
    let hmin = 1e9, hmax = -1e9;
    for (let j = 0; j < n1; j++) for (let i = 0; i < n1; i++) {
      const k = j * n1 + i, h = hh[(j + 1) * N + i + 1];
      const dx = (hh[(j + 1) * N + i + 2] - hh[(j + 1) * N + i]) / (2 * step), dz = (hh[(j + 2) * N + i + 1] - hh[j * N + i + 1]) / (2 * step);
      const il = 1 / Math.hypot(dx, 1, dz);
      pos[k * 3] = i * step - S / 2; pos[k * 3 + 1] = h; pos[k * 3 + 2] = j * step - S / 2;
      nor[k * 3] = -dx * il; nor[k * 3 + 1] = il; nor[k * 3 + 2] = -dz * il;
      this._color(h, Math.hypot(dx, dz), -(node.z + j * step), node.x + i * step, c);
      col[k * 3] = c[0]; col[k * 3 + 1] = c[1]; col[k * 3 + 2] = c[2];
      uv[k * 2] = i * step / 32; uv[k * 2 + 1] = j * step / 32;
      if (h < hmin) hmin = h; if (h > hmax) hmax = h;
    }
    // ชายกระโปรงปิดรอยต่อระหว่างระดับความละเอียด
    const idx = [];
    for (let j = 0; j < SEG; j++) for (let i = 0; i < SEG; i++) { const a = j * n1 + i, b = a + 1, d = a + n1, e = d + 1; idx.push(a, d, b, b, d, e); }
    let k = n1 * n1;
    const drop = S * 0.06 + 4;
    const edge = (get, flip) => {
      const start = k;
      for (let t = 0; t < n1; t++, k++) {
        const s = get(t);
        pos[k * 3] = pos[s * 3]; pos[k * 3 + 1] = pos[s * 3 + 1] - drop; pos[k * 3 + 2] = pos[s * 3 + 2];
        nor[k * 3] = nor[s * 3]; nor[k * 3 + 1] = nor[s * 3 + 1]; nor[k * 3 + 2] = nor[s * 3 + 2];
        col[k * 3] = col[s * 3]; col[k * 3 + 1] = col[s * 3 + 1]; col[k * 3 + 2] = col[s * 3 + 2];
        uv[k * 2] = uv[s * 2]; uv[k * 2 + 1] = uv[s * 2 + 1];
      }
      for (let t = 0; t < SEG; t++) { const a = get(t), b = get(t + 1), d = start + t, e = start + t + 1; if (flip) idx.push(a, b, d, b, e, d); else idx.push(a, d, b, b, d, e); }
    };
    edge(t => t, true); edge(t => SEG * n1 + t, false); edge(t => t * n1, false); edge(t => t * n1 + SEG, true);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3)); geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3)); geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    geo.setIndex(idx);
    geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, (hmin + hmax) / 2, 0), Math.hypot(S * 0.71, (hmax - hmin) / 2 + drop));
    const mesh = new THREE.Mesh(geo, this.mat);
    mesh.position.set(cx, 0, cz); mesh.visible = false; mesh.matrixAutoUpdate = false; mesh.updateMatrix();
    this.group.add(mesh); node.mesh = mesh; node.hmid = (hmin + hmax) / 2;
    if (S === MIN && hmax > 2) this._trees(node);
    if (T.real) this._texture(node);
  }

  _trees(node) {
    const T = this.T, cell = 26, m = Math.floor(node.size / cell), mats = [];
    const M = new THREE.Matrix4(), q = new THREE.Quaternion(), up = new THREE.Vector3(0, 1, 0), p = new THREE.Vector3(), sc = new THREE.Vector3();
    for (let j = 0; j < m; j++) for (let i = 0; i < m; i++) {
      const gx = Math.floor(node.x / cell) + i, gz = Math.floor(node.z / cell) + j;
      const x = (gx + T.hash(gx, gz, 41)) * cell, z = (gz + T.hash(gx, gz, 42)) * cell, n = -z, e = x;
      if (Math.hypot(n, e) < 520) continue;
      if (T.hash(gx, gz, 43) > T.forest(n, e) * 0.95 + (T.real ? 0 : 0.03)) continue;
      const h = T.height(n, e);
      if (h < 3 || (h > 1000 && !T.real)) continue;      // โลกสร้างอัตโนมัติมีแนวต้นไม้ที่ 1000 m · แผนที่จริงใช้หน้ากากป่า
      if (Math.abs(T.height(n + 6, e) - h) + Math.abs(T.height(n, e + 6) - h) > 6) continue;
      const s = 0.7 + 0.9 * T.hash(gx, gz, 44);
      q.setFromAxisAngle(up, T.hash(gx, gz, 45) * 6.283); p.set(x - node.x - node.size / 2, h - 0.3, z - node.z - node.size / 2); sc.set(s, s * (0.85 + 0.4 * T.hash(gx, gz, 46)), s);
      mats.push(M.compose(p, q, sc).clone());
    }
    if (!mats.length) return;
    const inst = new THREE.InstancedMesh(this.treeGeo, this.treeMat, mats.length);
    mats.forEach((mm, i) => inst.setMatrixAt(i, mm));
    inst.position.set(node.x + node.size / 2, 0, node.z + node.size / 2);
    inst.frustumCulled = false; inst.visible = false;
    this.group.add(inst); node.trees = inst;
  }

  _free(node) {
    if (node.mesh) { this.group.remove(node.mesh); node.mesh.geometry.dispose(); node.mesh = null; }
    if (node.trees) { this.group.remove(node.trees); node.trees.dispose(); node.trees = null; }
    if (node.tile) { const t = node.tile; node.tile = null; t.users.delete(node); this._idleCheck(t); }
    if (node.kids) { node.kids.forEach(k => this._free(k)); node.kids = null; }
    node.queued = false; node.dead = true;
  }
  _request(node, d) { node.dead = false; if (!node.mesh && !node.queued) { node.queued = true; node.prio = d / node.size; this.queue.push(node); } }
  _show(node, v) { if (node.mesh) node.mesh.visible = v; if (node.trees) node.trees.visible = v; }
  _hideTree(node) { this._show(node, false); if (node.kids) node.kids.forEach(k => this._hideTree(k)); }

  _visit(node, cam) {
    const S = node.size;
    const dx = Math.max(node.x - cam.x, 0, cam.x - node.x - S), dz = Math.max(node.z - cam.z, 0, cam.z - node.z - S);
    const dy = cam.y - (node.hmid ?? 0);
    const d = Math.hypot(dx, dz, Math.max(0, Math.abs(dy) - S * 0.2));
    if (S > MIN && d < S * SPLIT) {
      if (!node.kids) { const h = S / 2; node.kids = [new Node(node.x, node.z, h), new Node(node.x + h, node.z, h), new Node(node.x, node.z + h, h), new Node(node.x + h, node.z + h, h)]; }
      if (node.kids.every(k => k.mesh)) { this._show(node, false); node.kids.forEach(k => this._visit(k, cam)); return; }
      node.kids.forEach(k => this._request(k, d));
    } else if (node.kids) { node.kids.forEach(k => this._free(k)); node.kids = null; }
    if (node.mesh) { this._show(node, true); if (node.kids) node.kids.forEach(k => this._hideTree(k)); }
    else this._request(node, d);
  }

  /** เรียกทุกเฟรม · budgetMs = เวลาสร้างผิวดินสูงสุดต่อเฟรม (ใส่ Infinity ตอนโหลดครั้งแรก) */
  update(cam, budgetMs = 5) {
    const rx = Math.floor(cam.x / ROOT), rz = Math.floor(cam.z / ROOT), keep = new Set();
    for (let j = -2; j <= 2; j++) for (let i = -2; i <= 2; i++) {
      const key = (rx + i) + ',' + (rz + j); keep.add(key);
      if (!this.roots.has(key)) this.roots.set(key, new Node((rx + i) * ROOT, (rz + j) * ROOT, ROOT));
    }
    for (const [key, node] of this.roots) if (!keep.has(key)) { this._free(node); this.roots.delete(key); }
    for (const node of this.roots.values()) this._visit(node, cam);
    if (this.queue.length) {
      this.queue.sort((a, b) => b.prio - a.prio);          // ท้ายคิว = ใกล้/สำคัญสุด
      const t0 = performance.now();
      while (this.queue.length && performance.now() - t0 < budgetMs) { const n = this.queue.pop(); n.queued = false; if (!n.mesh && !n.dead) this._build(n); }
    }
    this.sky.position.copy(cam); this.water.position.set(cam.x, 0, cam.z);
    const span = this.cloudSpan;
    for (const sp of this.clouds) {
      const b = sp.userData.base;
      sp.position.set(b.x + Math.round((cam.x - b.x) / span) * span, b.y + this.T.siteAlt, b.z + Math.round((cam.z - b.z) / span) * span);
    }
    return this.queue.length;
  }
  /** โหลดผิวดินรอบจุดเริ่มให้ครบก่อนเริ่มเล่น */
  preload(cam) { for (let i = 0; i < 40; i++) if (this.update(cam, Infinity) === 0 && i > 6) break; }
}
