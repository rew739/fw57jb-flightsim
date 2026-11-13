# -*- coding: utf-8 -*-
"""ขั้น 2/2 ของโมเดลรางปล่อยในเกม: ผลดิบจาก export_cad_mesh.py (LR2_assembly) → models/lr2/model.json + model.bin

- แกนของโมเดลราง: X = ขวา, Y = ขึ้นจากพื้น, −Z = ทิศปล่อยในแนวราบ · จุด (0, h, 0) = CG อ้างอิงของเครื่องตอนอยู่บนราง
  (assembly ราง: x = ทิศปล่อยในแนวราบ, y ขึ้น, z = ขวา · พื้น = จุดต่ำสุดของโครง)
- ตำแหน่งเครื่อง: อ่านจาก component FW57J_B-1 ใน assembly ราง + จุดอ้างอิง CG จาก models/fw57jb/model.json
- 4 ชุด: static (โครง ถังลม กระบอก เบรก), carriage (แคร่ ขยับตามราง), piston (ลูกสูบ + รอกเคลื่อน), rope (เชือกท่าเริ่ม)
ใช้:  python 10_FlightSim_Game/tools/export_cad_mesh.py --asm 04_CAD/Launcher/1_current/cad/LR2_assembly.SLDASM --skip FW57J_jet --cache C:/tmp/lr2_mesh
      python 10_FlightSim_Game/tools/build_rail.py [--cache C:/tmp/lr2_mesh]"""
import os, sys, io, json, argparse
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", line_buffering=True)
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); GAME = os.path.dirname(HERE)
ap = argparse.ArgumentParser(); ap.add_argument("--cache", default="C:/tmp/lr2_mesh"); A = ap.parse_args()
OUT = os.path.join(GAME, "models", "lr2"); os.makedirs(OUT, exist_ok=True)
BRAKE_M, TACKLE = 1.2, 3                     # ระยะเบรกแคร่และอัตราทดรอกของราง L6 (PhaseL6)

raw = json.load(open(os.path.join(A.cache, "raw.json"), encoding="utf-8"))
P_all = np.fromfile(os.path.join(A.cache, "raw.f32"), dtype="<f4").reshape(-1, 3)
N_all = np.fromfile(os.path.join(A.cache, "raw_n.f32"), dtype="<f4").reshape(-1, 3)
bodies = [b for b in raw["bodies"] if not b["envelope"] and not b["hidden"]]
def body(name): return next(b for b in bodies if b["body"] == name)
def cen(b): bx = np.array(b["box"]); return (bx[:3] + bx[3:]) / 2

# ตำแหน่งเครื่องบนราง: จุดอ้างอิง CG ของเครื่อง (พิกัด assembly เครื่อง) → พิกัด assembly ราง
ac = next(a for a in raw["assemblies"] if a["path"].upper().endswith("FW57J_B.SLDASM")); T = ac["T"]
R = np.array(T[:9]).reshape(3, 3); tr = np.array(T[9:12])
ref = json.load(open(os.path.join(GAME, "models", "fw57jb", "model.json"), encoding="utf-8"))["ref"]
cg = np.array([ref["x_cg"], ref["y_pod_axis_cad"] + ref["z_cg_above_pod_axis"], 0.0]) @ R * T[12] + tr
y_ground = min(b["box"][1] for b in bodies if b["part"].upper().endswith("LR2_FRAME"))
fwd = np.array([-1.0, 0, 0]) @ R                                       # ทิศหัวเครื่อง = ทิศราง (พิกัด assembly ราง)
assert abs(fwd[2]) < 1e-6 and fwd[0] > 0, f"เครื่องไม่ได้หันตามแกน x ของ assembly ราง: {fwd}"
def to_site(P):  return np.stack([P[:, 2] - cg[2], P[:, 1] - y_ground, -(P[:, 0] - cg[0])], 1)
def to_siteN(N): return np.stack([N[:, 2], N[:, 1], -N[:, 0]], 1)
d_site = to_siteN(fwd[None, :])[0]

GROUP = {"LR2_FRAME": "static", "LR2_CARRIAGE": "carriage", "LR2_PISTON": "piston"}
def group_of(b):
    p = b["part"].upper()
    return "rope" if "ROPE" in p else GROUP.get(p, "static")

buckets, used = {}, 0
for b in bodies:
    i0 = b["offset"] // 12; n = b["ntri"] * 3
    P = to_site(P_all[i0:i0 + n].astype(np.float64)); N = to_siteN(N_all[i0:i0 + n].astype(np.float64))
    if b.get("det", 1.0) < 0: P = P.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3); N = N.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3)
    Tt = P.reshape(-1, 3, 3); G = np.cross(Tt[:, 1] - Tt[:, 0], Tt[:, 2] - Tt[:, 0]); gl = np.linalg.norm(G, axis=1, keepdims=True)
    Gn = np.repeat(G / np.maximum(gl, 1e-30), 3, 0)
    ln = np.linalg.norm(N, axis=1, keepdims=True); N = np.where(ln > 1e-6, N / np.maximum(ln, 1e-30), Gn)
    N = np.where((N * Gn).sum(1, keepdims=True) < -0.2, -N, N)
    c = b["color"] or [0.5, 0.5, 0.5, 1, 1, 0.2, 0.3, 0, 0]
    col = np.tile(np.array(c[:3]), (len(P), 1)); alpha = 1.0 - c[7]
    for k, cnt, rgb, _ in b["face_colors"]: col[k * 3:(k + cnt) * 3] = rgb
    for rgb in np.unique(col.round(3), axis=0):
        sel = (col.round(3) == rgb).all(1)
        buckets.setdefault((group_of(b), tuple(rgb.tolist()), round(alpha, 2), round(c[5], 2), round(c[6], 2)), []).append((P[sel], N[sel]))
    used += 1

blob, meshes, ntri, nvert = bytearray(), [], 0, 0
def put(arr):
    while len(blob) % 4: blob.append(0)
    o = len(blob); blob.extend(arr.tobytes()); return o
lo_all, hi_all = np.full(3, np.inf), np.full(3, -np.inf)
for key in sorted(buckets):
    grp, rgb, alpha, spec, shine = key
    P = np.concatenate([p for p, _ in buckets[key]]); N = np.concatenate([n for _, n in buckets[key]])
    lo, hi = P.min(0), P.max(0); ext = np.maximum(hi - lo, 1e-9)
    q = np.round((P - lo) / ext * 65535).astype("<u2"); n8 = np.clip(np.round(N * 127), -127, 127).astype("i1")
    rec = np.zeros(len(q), dtype=[("p", "<u2", 3), ("n", "i1", 3)]); rec["p"] = q; rec["n"] = n8
    uniq, inv = np.unique(rec, return_inverse=True)
    idx = inv.reshape(-1, 3); a, b_, c_ = uniq["p"][idx[:, 0]], uniq["p"][idx[:, 1]], uniq["p"][idx[:, 2]]
    idx = idx[~((a == b_).all(1) | (b_ == c_).all(1) | (a == c_).all(1))]
    it = "u16" if len(uniq) <= 65535 else "u32"
    meshes.append(dict(group=grp, skin=False, color=list(rgb), opacity=alpha, specular=spec, shininess=shine, nvert=int(len(uniq)), ntri=int(len(idx)),
                       min=lo.round(6).tolist(), ext=ext.round(6).tolist(), pos=put(np.ascontiguousarray(uniq["p"])), nrm=put(np.ascontiguousarray(uniq["n"])),
                       idx=put(idx.astype("<u2" if it == "u16" else "<u4")), idxType=it))
    ntri += len(idx); nvert += len(uniq); lo_all = np.minimum(lo_all, lo); hi_all = np.maximum(hi_all, hi)

# ตัวเลขของราง: ความยาว ปลายหน้า (ตามทิศราง วัดจากจุดอ้างอิง CG) ช่วงชักที่ลูกสูบเดินได้
def corners(b): bx = np.array(b["box"]); return np.array([[bx[i], bx[j], bx[k]] for i in (0, 3) for j in (1, 4) for k in (2, 5)])
rail = body("Rail_R_L6"); dx = rail["box"][3] - rail["box"][0]
rail_len = dx / fwd[0] - 0.0                                           # กล่องรางเอียง: ยาวจริง ≈ ช่วง x / cos(มุมยก) (หักความหนาเล็กน้อยด้านล่าง)
rail_front = float(((corners(rail) - cg) @ fwd).max())
room = float((cen(body("Cyl_cap_front_rodhole")) - cen(body("Piston_head_200"))) @ fwd) - 0.5 * 0.07 - 0.5 * 0.132
out_dir = float((cen(body("Drum_mov_axle")) - cen(body("Sheave_fixed_axle_R"))) @ fwd)
assert out_dir > 0 and room > 0, "ทิศชักลูกสูบไม่ใช่ไปทางหน้าราง — ตรวจ CAD"

meta = dict(version=1, source=dict(asm=raw["asm"], asm_mtime=raw["asm_mtime"], exported=raw["exported"]),
            frame="X ขวา, Y ขึ้นจากพื้น, −Z ทิศปล่อยแนวราบ (m) · (0, start.y, 0) = CG อ้างอิงของเครื่องบนราง",
            start=[0.0, round(float(cg[1] - y_ground), 4), 0.0], dir=[round(float(v), 6) for v in d_site],
            elev_deg=round(float(np.degrees(np.arcsin(d_site[1]))), 3), rail_length=round(float(rail_len), 3), rail_front=round(rail_front, 3),
            brake_m=BRAKE_M, tackle=TACKLE, piston_room=round(room, 3),
            bbox=dict(min=lo_all.round(4).tolist(), max=hi_all.round(4).tolist()), nbody=used, ntri=int(ntri), nvert=int(nvert),
            bin="model.bin", bytes=len(blob), meshes=meshes)
open(os.path.join(OUT, "model.bin"), "wb").write(blob)
json.dump(meta, open(os.path.join(OUT, "model.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print(f"body {used} → mesh {len(meshes)} · สามเหลี่ยม {ntri:,} · จุด {nvert:,} · model.bin {len(blob) / 1e6:.2f} MB")
print(f"กรอบ (m): {lo_all.round(3)} … {hi_all.round(3)}")
print(f"CG อ้างอิงของเครื่องบนราง สูงจากพื้น {meta['start'][1]} m · มุมยก {meta['elev_deg']}° · รางยาว {meta['rail_length']} m · ปลายหน้า {meta['rail_front']} m · ช่วงชักว่าง {meta['piston_room']} m")
for g in sorted({m["group"] for m in meshes}):
    ms = [m for m in meshes if m["group"] == g]; print(f"  {g}: {len(ms)} mesh · {sum(m['ntri'] for m in ms):,} สามเหลี่ยม")
