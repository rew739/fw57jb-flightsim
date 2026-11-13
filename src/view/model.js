// โมเดล 3D ของ FW57J-B: รูปทรงจริงจาก SolidWorks (models/fw57jb/ — loadCad) + elevon ขยับได้ + เปลวไอพ่น + ร่ม
// ถ้าไม่มีไฟล์โมเดลจาก CAD ใช้โมเดลสำรองที่สร้างจากขนาดใน spec (ปีกลู่ 27° + winglet + pod)
// แกนโมเดล: X = ขวา, Y = ขึ้น, Z = ไปท้าย (หัวชี้ −Z) · จุดกำเนิด = CG
import * as THREE from 'three';
import { decodeModel, hingeRotation } from './cadmesh.js';
import { solveStation } from './linkage.js';

const XS = [0, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.9, 1];
const thick = (x, t) => 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
const camber = x => 0.018 * Math.sin(Math.PI * Math.min(x / 0.6, 1)) * (1 - x) - 0.006 * Math.sin(Math.PI * x) * x;   // โค้งน้อย + reflex ท้าย

function geomFrom(pos, idx) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setIndex(idx); g.computeVertexNormals();
  return g;
}

/** ผิวปีกระหว่างหน้าตัด 2 สถานี · sec = {X, Y, zle, c, twist, cf (สัดส่วนคอร์ดที่ลากถึง)} */
function loft(a, b) {
  const pos = [], idx = [];
  const ring = s => {
    const xs = XS.filter(x => x <= s.cf + 1e-9), pts = [];
    for (const x of xs) pts.push([x, camber(x) + thick(x, 0.105)]);              // ผิวบน หน้า → ท้าย
    for (let i = xs.length - 1; i >= 0; i--) pts.push([xs[i], camber(xs[i]) - thick(xs[i], 0.105)]);   // ผิวล่าง ท้าย → หน้า
    const ct = Math.cos(s.twist), st = Math.sin(s.twist);
    return pts.map(([x, y]) => [s.X, s.Y + (y * ct - x * st) * s.c, s.zle + (x * ct + y * st) * s.c]);
  };
  const ra = ring(a), rb = ring(b), n = ra.length;
  for (const p of ra) pos.push(...p); for (const p of rb) pos.push(...p);
  for (let i = 0; i < n; i++) { const j = (i + 1) % n; idx.push(i, n + i, j, j, n + i, n + j); }
  return geomFrom(pos, idx);
}

function elevonGeom(len, c0, c1) {       // ลิ่มในแกนบานพับ: x ตามแนวบานพับ, z ไปชายหลัง, y ขึ้น
  const t0 = 0.05 * c0 * 4, t1 = 0.05 * c1 * 4;
  const pos = [0, t0 / 2, 0, 0, -t0 / 2, 0, 0, 0, c0, len, t1 / 2, 0, len, -t1 / 2, 0, len, 0, c1];
  const g = new THREE.BufferGeometry();
  const tri = [0, 3, 2, 3, 5, 2, 1, 2, 4, 4, 2, 5, 0, 1, 3, 3, 1, 4, 0, 2, 1, 3, 4, 5];
  const flat = []; for (const i of tri) flat.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
  g.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3)); g.computeVertexNormals();
  return g;
}

function podGeom(P, yc, zc) {
  const L = P.x_tail - P.x_nose, N = 26, R = 18, pos = [], idx = [];
  for (let i = 0; i <= N; i++) {
    const s = i / N, x = P.x_nose + s * L;
    const fn = Math.min(1, s / 0.24), nose = Math.sqrt(Math.max(0, 1 - (1 - fn) ** 2.2));
    const ta = Math.max(0, (s - 0.62) / 0.38), tail = ta * ta * (3 - 2 * ta);
    const hw = (P.half_w * nose) * (1 - tail) + P.r_exhaust * tail, hh = (P.half_h * nose) * (1 - tail) + P.r_exhaust * tail;
    for (let k = 0; k < R; k++) {
      const a = k / R * 2 * Math.PI, ca = Math.cos(a), sa = Math.sin(a), e = 2 / 2.6;
      pos.push(Math.sign(ca) * Math.abs(ca) ** e * hw, yc + Math.sign(sa) * Math.abs(sa) ** e * hh, x - zc);
    }
  }
  for (let i = 0; i < N; i++) for (let k = 0; k < R; k++) { const a = i * R + k, b = i * R + (k + 1) % R, c = a + R, d = b + R; idx.push(a, b, c, b, d, c); }
  return geomFrom(pos, idx);
}

export class AircraftModel {
  constructor(data) {
    const Gm = data.geom, half = Gm.b / 2, xcg = 0.5, zcg = 0.04, tanS = Math.tan(Gm.sweep_LE_deg * Math.PI / 180);
    this.group = new THREE.Group();
    const white = new THREE.MeshPhongMaterial({ color: 0xf1f1ee, shininess: 40, side: THREE.DoubleSide });
    const orange = new THREE.MeshPhongMaterial({ color: 0xe8691b, shininess: 30, side: THREE.DoubleSide });
    const carbon = new THREE.MeshPhongMaterial({ color: 0x33383d, shininess: 60, side: THREE.DoubleSide });
    const sec = (y, cf = 1, sgn = 1) => ({ X: sgn * y, Y: -zcg, zle: Gm.x_le_root + tanS * y - xcg, c: Gm.cr - (Gm.cr - Gm.ct) * y / half, twist: Gm.twist_tip_deg * Math.PI / 180 * y / half, cf });
    const [y0, y1] = Gm.elevon_y, hf = 1 - Gm.elevon_chord_frac;
    const body = this.proc = new THREE.Group(); this.group.add(body);       // โมเดลสำรองสร้างจากขนาดใน spec (ใช้เมื่อไม่มีไฟล์โมเดลจาก CAD)
    this.elev = {};
    for (const sgn of [1, -1]) {
      body.add(new THREE.Mesh(loft(sec(0.05, 1, sgn), sec(y0, 1, sgn)), white));
      body.add(new THREE.Mesh(loft(sec(y0, hf, sgn), sec(y1, hf, sgn)), white));
      body.add(new THREE.Mesh(loft(sec(y1, 1, sgn), sec(half, 1, sgn)), orange));
      // elevon: กลุ่มวางตามแนวบานพับ แล้วหมุนรอบแกน x ของกลุ่ม (+ = ชายหลังลง)
      const hp = y => { const s = sec(y, 1, sgn); return new THREE.Vector3(s.X, s.Y + camber(hf) * s.c - Math.sin(s.twist) * hf * s.c, s.zle + hf * s.c); };
      const A = hp(y0), B = hp(y1), start = sgn > 0 ? A : B, end = sgn > 0 ? B : A;
      const ex = end.clone().sub(start), len = ex.length(); ex.normalize();
      const ey = new THREE.Vector3(0, 1, 0), ez = new THREE.Vector3().crossVectors(ex, ey).normalize(); ey.crossVectors(ez, ex);
      const frame = new THREE.Group(); frame.position.copy(start);
      frame.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(ex, ey, ez));
      const cA = Gm.elevon_chord_frac * sec(y0).c * 0.95, cB = Gm.elevon_chord_frac * sec(y1).c * 0.95;
      const m = new THREE.Mesh(elevonGeom(len, sgn > 0 ? cA : cB, sgn > 0 ? cB : cA), orange);
      frame.add(m); body.add(frame); this.elev[sgn > 0 ? 'R' : 'L'] = m;
      // winglet
      const tip = sec(half, 1, sgn), h = Gm.winglet_h, pos = [], idx = [];
      const ring = (yy, zle, c) => { const pts = []; for (const x of XS) pts.push([x, thick(x, 0.08)]); for (let i = XS.length - 1; i >= 0; i--) pts.push([XS[i], -thick(XS[i], 0.08)]); return pts.map(([x, t]) => [tip.X + t * c, tip.Y + yy, zle + x * c]); };
      const r0 = ring(-0.02, tip.zle, tip.c), r1 = ring(h, tip.zle + 0.55 * h, tip.c * 0.5), n = r0.length;
      for (const p of r0) pos.push(...p); for (const p of r1) pos.push(...p);
      for (let i = 0; i < n; i++) { const j = (i + 1) % n; idx.push(i, n + i, j, j, n + i, n + j); }
      body.add(new THREE.Mesh(geomFrom(pos, idx), orange));
    }
    body.add(new THREE.Mesh(podGeom(Gm.pod, -zcg, xcg), carbon));
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.012, 0.9), orange); stripe.position.set(0, Gm.pod.half_h - zcg - 0.004, -0.25); body.add(stripe);
    const noz = new THREE.Mesh(new THREE.CircleGeometry(Gm.pod.r_exhaust * 0.85, 16), new THREE.MeshBasicMaterial({ color: 0x0a0a0a, side: THREE.DoubleSide }));
    noz.position.set(0, -zcg, Gm.pod.x_tail - xcg + 0.002); body.add(noz);
    // เปลวไอพ่น
    const fg = new THREE.ConeGeometry(Gm.pod.r_exhaust * 0.75, 1, 12, 1, true); fg.rotateX(Math.PI / 2); fg.translate(0, 0, 0.5);
    this.flame = new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ color: 0xffa040, transparent: true, opacity: 0.55, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide }));
    this.flame.position.set(0, -zcg, Gm.pod.x_tail - xcg); this.group.add(this.flame); this.flameK = 1; this.rExh = Gm.pod.r_exhaust;

    // ร่ม (อยู่ในฉาก ไม่ได้เป็นลูกของตัวเครื่อง)
    this.chute = new THREE.Group(); this.chute.visible = false;
    const R = 0.33 * data.chute.D0_m, dome = new THREE.SphereGeometry(R, 20, 8, 0, Math.PI * 2, 0, Math.PI / 2);
    const cols = [], pa = dome.attributes.position;
    for (let i = 0; i < pa.count; i++) { const a = Math.atan2(pa.getZ(i), pa.getX(i)), k = Math.floor((a + Math.PI) / (Math.PI / 5)) % 2; cols.push(...(k ? [0.95, 0.42, 0.1] : [0.96, 0.96, 0.94])); }
    dome.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
    this.canopy = new THREE.Mesh(dome, new THREE.MeshLambertMaterial({ vertexColors: true, side: THREE.DoubleSide }));
    this.chute.add(this.canopy);
    this.lines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x222222 }));
    this.lines.frustumCulled = false; this.chute.add(this.lines);
    this.chuteR = R; this.chuteDir = new THREE.Vector3(0, 1, 0);
    // เงาใต้เครื่อง
    this.shadow = new THREE.Mesh(new THREE.CircleGeometry(1, 20), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.35, depthWrite: false }));
    this.shadow.rotation.x = -Math.PI / 2; this.shadow.scale.set(Gm.b / 2, 0.75, 1);
  }
  addTo(scene) { scene.add(this.group, this.chute, this.shadow); }

  /** โหลดรูปทรงจริงจาก CAD แล้วแทนโมเดลสำรอง · คืน meta (หรือ null ถ้าไม่มีไฟล์) */
  async loadCad(dir = 'models/fw57jb/') {
    let meta, buf;
    try {
      const r = await fetch(dir + 'model.json'); if (!r.ok) return null;
      meta = await r.json();
      const rb = await fetch(dir + meta.bin); if (!rb.ok) return null;
      buf = await rb.arrayBuffer();
    } catch (e) { return null; }
    const cad = new THREE.Group(), piv = {}, inner = {};
    for (const side of ['R', 'L']) {                       // elevon: จุดหมุนอยู่บนแกนบานพับจาก CAD
      const h = meta.hinges[side], g = new THREE.Group(), q = new THREE.Group();
      g.position.fromArray(h.p); q.position.fromArray(h.p).negate(); g.add(q); cad.add(g); piv[side] = g; inner[side] = q;
    }
    // กลไกบังคับ elevon ต่อสถานี: แขนเซอร์โวหมุนรอบแกนเซอร์โว · ก้านดันยึดที่บอลปลายแขนแล้วชี้ไปบอลบน horn
    const stations = [];
    for (const [k, L] of Object.entries(meta.linkage || {})) {
      const arm = new THREE.Group(), armIn = new THREE.Group(), rod = new THREE.Group(), rodIn = new THREE.Group();
      arm.position.fromArray(L.S); armIn.position.fromArray(L.S).negate(); arm.add(armIn);
      rod.position.fromArray(L.A0); rodIn.position.fromArray(L.A0).negate(); rod.add(rodIn);
      cad.add(arm, rod); inner['arm_' + k] = armIn; inner['rod_' + k] = rodIn;
      stations.push({ side: k[0], st: { ...L, hinge: meta.hinges[k[0]] }, arm, rod });
    }
    this.skinMats = [];
    for (const m of decodeModel(meta, buf)) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(m.position, 3)); g.setAttribute('normal', new THREE.BufferAttribute(m.normal, 3));
      g.setIndex(new THREE.BufferAttribute(m.index, 1));
      const clear = m.opacity < 0.99;
      const col = new THREE.Color().setRGB(m.color[0], m.color[1], m.color[2], THREE.SRGBColorSpace);
      const mat = new THREE.MeshPhongMaterial({
        color: col, emissive: col.clone().multiplyScalar(0.3),          // แสงรอบตัวแบบ SolidWorks: ด้านที่ไม่โดนแดด (ใต้ท้อง) ยังเห็นรายละเอียด
        specular: new THREE.Color().setScalar(0.35 * m.specular + 0.04), shininess: 18 + 90 * m.shininess,
        transparent: clear, opacity: m.opacity, depthWrite: !clear, side: clear ? THREE.DoubleSide : THREE.FrontSide,
      });
      mat.userData.opacity0 = m.opacity;
      if (m.skin) this.skinMats.push(mat);
      const mesh = new THREE.Mesh(g, mat); mesh.renderOrder = clear ? 2 : m.skin ? 1 : 0;
      (m.group === 'elevon_R' ? inner.R : m.group === 'elevon_L' ? inner.L : inner[m.group] || cad).add(mesh);
    }
    this.group.remove(this.proc); this.group.add(cad); this.cad = cad; this.cadMeta = meta; this.pivot = piv; this.stations = stations;
    this.flame.position.fromArray(meta.nozzle.p); this.flame.scale.x = this.flame.scale.y = this.flameK = meta.nozzle.r / (0.75 * this.rExh);
    return meta;
  }

  /** มุมมองโปร่ง: ผิวนอกใสให้เห็นโครง ถัง เครื่องยนต์ อุปกรณ์ข้างใน */
  setXray(on) {
    this.xray = !!on && !!this.skinMats;
    for (const m of this.skinMats || []) {
      m.transparent = this.xray || m.userData.opacity0 < 0.99; m.opacity = this.xray ? 0.16 : m.userData.opacity0;
      m.depthWrite = !m.transparent; m.side = m.transparent ? THREE.DoubleSide : THREE.FrontSide; m.needsUpdate = true;
    }
    return this.xray;
  }

  setElevon(dL, dR) {                                      // เรเดียน · บวก = ชายหลังลง
    if (this.pivot) {
      this.pivot.L.quaternion.fromArray(hingeRotation(this.cadMeta.hinges.L.axis, dL));
      this.pivot.R.quaternion.fromArray(hingeRotation(this.cadMeta.hinges.R.axis, dR));
      for (const s of this.stations) {
        const r = solveStation(s.st, s.side === 'R' ? dR : dL);
        s.arm.quaternion.fromArray(hingeRotation(s.st.s, r.theta));
        s.rod.position.fromArray(r.A); s.rod.quaternion.fromArray(r.q);
      }
    } else { this.elev.L.rotation.x = dL; this.elev.R.rotation.x = dR; }
  }

  update(ac, dt) {
    const s = ac.s, q = s.q, D = Math.PI / 180;
    this.group.position.set(s.pos[1], -s.pos[2], -s.pos[0]);
    this.group.quaternion.set(q[2], -q[3], -q[1], q[0]);
    this.setElevon(ac.dL * D, ac.dR * D);
    const f = ac.engine.thrustFrac;
    this.flame.visible = f > 0.02; this.flame.scale.set(this.flameK, this.flameK, 0.25 + 1.1 * f); this.flame.material.opacity = 0.25 + 0.4 * f;
    // เงา
    const surf = ac._surf, agl = ac.alt - surf;
    this.shadow.visible = agl < 400 && agl > -1;
    if (this.shadow.visible) {
      this.shadow.position.set(s.pos[1], surf + 0.25, -s.pos[0]);
      this.shadow.rotation.z = -ac.euler().yaw;
      this.shadow.material.opacity = 0.38 * Math.max(0, 1 - agl / 400);
    }
    // ร่ม: ลอยตามทิศลมสัมพัทธ์เหนือเครื่อง
    const ch = ac.chute, show = ch.state === 'inflating' || ch.state === 'open';
    this.chute.visible = show;
    if (show) {
      const vn = ac.vNED(), w = ac.windNow;
      const rel = new THREE.Vector3(-(vn[1] - w[1]), (vn[2] - w[2]), (vn[0] - w[0]));     // ทิศตรงข้ามความเร็วเทียบอากาศ (แกนภาพ)
      if (rel.lengthSq() > 0.5) this.chuteDir.lerp(rel.normalize(), Math.min(1, 4 * dt)).normalize();
      const frac = Math.sqrt(Math.max(ch.cds / ac.data.chute.CDS_m2, 0.02)), Lr = 8.5;
      const base = this.group.position.clone().add(new THREE.Vector3(0, 0.2, 0));
      const top = base.clone().addScaledVector(this.chuteDir, Lr);
      this.canopy.position.copy(top); this.canopy.scale.set(frac, 0.45 + 0.55 * frac, frac);
      this.canopy.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), this.chuteDir);
      const pts = [], r = this.chuteR * frac;
      for (let i = 0; i < 10; i++) {
        const a = i / 10 * Math.PI * 2, p = new THREE.Vector3(Math.cos(a) * r, 0, Math.sin(a) * r).applyQuaternion(this.canopy.quaternion).add(top);
        pts.push(base.x, base.y, base.z, p.x, p.y, p.z);
      }
      this.lines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    }
  }
}
