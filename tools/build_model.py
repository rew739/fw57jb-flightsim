# -*- coding: utf-8 -*-
"""ขั้น 2/2 ของโมเดลเครื่องบินในเกม: ผลดิบจาก export_cad_mesh.py → models/fw57jb/model.json + model.bin

- หมุนเข้าแกนของเกม: X = ขวา, Y = ขึ้น, Z = ไปท้าย · จุดกำเนิด = CG อ้างอิง (x 0.5 m, สูงกว่าแกน pod 0.04 m)
  (CAD: x ไปท้าย, y ขึ้น, z ไปทางซ้าย)
- รวม body ที่สี/ความโปร่งเดียวกันเป็น mesh เดียว แยก 3 ชุด: static, elevon_R, elevon_L · แยกผิวนอก (skin) ไว้ทำมุมมองโปร่ง
- รวมจุดซ้ำ (ตำแหน่ง + normal) · เก็บตำแหน่ง uint16 ต่อกรอบ mesh, normal int8
- แกนบานพับ elevon หาจากสลักบานพับใน CAD (HGP_PinR)
ใช้:  python 10_FlightSim_Game/tools/build_model.py [--cache C:/tmp/fw57jb_mesh]"""
import os, sys, io, json, re, argparse
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", line_buffering=True)
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); GAME = os.path.dirname(HERE)
ap = argparse.ArgumentParser(); ap.add_argument("--cache", default="C:/tmp/fw57jb_mesh"); A = ap.parse_args()
OUT = os.path.join(GAME, "models", "fw57jb"); os.makedirs(OUT, exist_ok=True)

X_CG, Z_CG = 0.5, 0.04                       # เท่ากับที่ src/view/model.js ใช้กับโมเดลเดิม
SKIP_PART = re.compile(r"CARGO_ENVELOPE", re.I)                         # ปริมาตรอ้างอิงช่องสัมภาระ ไม่ใช่ชิ้นจริง
SKIN = re.compile(r"^(WP_Wing[RL]_s|PodSkin_|Winglet[RL]$|Elevon[RL]$|CargoDoor_h|HatchZ6_h|ChuteHatch|TAIL_SKIN_|Shell1$|SV_R\w+_Fair(W|Ec)$|HGW_Cove)")

raw = json.load(open(os.path.join(A.cache, "raw.json"), encoding="utf-8"))
P_all = np.fromfile(os.path.join(A.cache, "raw.f32"), dtype="<f4").reshape(-1, 3)
N_all = np.fromfile(os.path.join(A.cache, "raw_n.f32"), dtype="<f4").reshape(-1, 3)
assert len(P_all) == len(N_all), "raw.f32 กับ raw_n.f32 ไม่เท่ากัน — รัน export_cad_mesh.py ใหม่"

def base_name(n):                                 # body ของ part ซ้าย (mirror): "<PART>-<ชื่อเดิม>"
    m = re.match(r"^<[^>]+>-<(.+)>$", n); return m.group(1) if m else n
def group_of(b):
    """ชุดที่ขยับด้วยกัน: elevon_R/L · arm_<ข้าง><สถานี> (แขนเซอร์โว) · rod_<ข้าง><สถานี> (ก้านดัน) · สถานี 0 = ใน, 1 = นอก"""
    p = b["part"].upper(); zc = 0.5 * (b["box"][2] + b["box"][5]); side = "R" if zc < 0 else "L"      # CAD +z = ซ้าย
    if re.search(r"(^|_)ELEVON_R$", p): return "elevon_R"
    if re.search(r"(^|_)ELEVON_L$", p): return "elevon_L"
    if p.endswith("SERVO_ARM"): return f"arm_{side}{0 if abs(zc) < 1.25 else 1}"
    if p.endswith("PUSHROD_IN"): return f"rod_{side}0"
    if p.endswith("PUSHROD_OUT"): return f"rod_{side}1"
    return "static"

pod = next(b for b in raw["bodies"] if b["body"].startswith("PodSkin_"))
y_axis = 0.5 * (pod["box"][1] + pod["box"][4]); y0 = y_axis + Z_CG
def to_game(P):  return np.stack([-P[:, 2], P[:, 1] - y0, P[:, 0] - X_CG], 1)
def to_gameN(N): return np.stack([-N[:, 2], N[:, 1], N[:, 0]], 1)

buckets, hinge_pts, used, skipped = {}, {"R": [], "L": []}, 0, []
link = {}                                         # กลไก elevon ต่อสถานี: A0 บอลปลายแขน · H0 บอลบน horn · S, s แกนเซอร์โว
def center(b): bx = np.array(b["box"]); return to_game(((bx[:3] + bx[3:]) / 2)[None, :])[0]
shafts = []
for b in raw["bodies"]:
    if b["envelope"] or b["hidden"] or SKIP_PART.search(b["part"]): skipped.append(f'{b["inst"]}/{b["body"]}'); continue
    i0 = b["offset"] // 12; n = b["ntri"] * 3
    P = to_game(P_all[i0:i0 + n].astype(np.float64)); N = to_gameN(N_all[i0:i0 + n].astype(np.float64))
    if b.get("det", 1.0) < 0: P = P.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3); N = N.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3)
    # normal: ถ้าไม่มี (ศูนย์) หรือสวนกับด้านหน้าของสามเหลี่ยม → ใช้/กลับตาม normal ของสามเหลี่ยม
    T = P.reshape(-1, 3, 3); G = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); gl = np.linalg.norm(G, axis=1, keepdims=True)
    Gn = np.repeat(G / np.maximum(gl, 1e-30), 3, 0)
    ln = np.linalg.norm(N, axis=1, keepdims=True); N = np.where(ln > 1e-6, N / np.maximum(ln, 1e-30), Gn)
    N = np.where((N * Gn).sum(1, keepdims=True) < -0.2, -N, N)
    name = base_name(b["body"]); grp = group_of(b); skin = bool(SKIN.match(name))
    if grp.startswith("arm_"):
        L = link.setdefault(grp[4:], {})
        if "ArmBall" in name: L["A0"] = center(b)
        if "ArmPin" in name:                                         # สลักแขนขนานกับแกนเซอร์โว → ใช้หาทิศแกน
            _, _, vt = np.linalg.svd(P - P.mean(0), full_matrices=False); L["s"] = vt[0] * (1 if vt[0][0] > 0 else -1)
    if grp.startswith("elevon_") and re.match(r"SV_R(0|1b)_BallH$", name):
        link.setdefault(grp[-1] + ("0" if "_R0_" in name else "1"), {})["H0"] = center(b)
    if b["part"].upper().find("SERVO_FIXED") >= 0 and re.search(r"Shaft$", name): shafts.append(center(b))
    c = b["color"] or [0.5, 0.5, 0.5, 1, 1, 0.2, 0.3, 0, 0]
    col = np.tile(np.array(c[:3]), (len(P), 1)); alpha = 1.0 - c[7]
    for k, cnt, rgb, tr in b["face_colors"]:                         # ผิวที่ตั้งสีเอง (ความโปร่งใช้ของ body)
        col[k * 3:(k + cnt) * 3] = rgb
    if name == "HGP_PinR": hinge_pts["L" if grp.endswith("_L") else "R"].append(P)
    for rgb in np.unique(col.round(3), axis=0):
        sel = (col.round(3) == rgb).all(1)
        key = (grp, skin, tuple(rgb.tolist()), round(alpha, 2), round(c[5], 2), round(c[6], 2))
        buckets.setdefault(key, []).append((P[sel], N[sel]))
    used += 1

hinges = {}
for side, pts in hinge_pts.items():
    assert pts, f"ไม่พบสลักบานพับ HGP_PinR ฝั่ง {side}"
    Q = np.concatenate(pts); cen = Q.mean(0); _, _, vt = np.linalg.svd(Q - cen, full_matrices=False); ax = vt[0] * (1 if vt[0][0] > 0 else -1)
    hinges[side] = dict(p=cen.round(5).tolist(), axis=(ax / np.linalg.norm(ax)).tolist(), length=float(np.ptp((Q - cen) @ ax)))

nz = next(b for b in raw["bodies"] if b["body"].endswith("E140D_Nozzle")); nb = np.array(nz["box"])
nozzle = dict(p=[round(float(-0.5 * (nb[2] + nb[5])), 4), round(float(0.5 * (nb[1] + nb[4]) - y0), 4), round(float(nb[3] - X_CG), 4)], r=round(float(0.25 * (nb[4] - nb[1] + nb[5] - nb[2])), 4))
parts = sorted({b["part"].upper() for b in raw["bodies"] if not SKIP_PART.search(b["part"])})

linkage = {}
for k in ("R0", "R1", "L0", "L1"):
    L = link.get(k, {}); assert all(q in L for q in ("A0", "H0", "s")), f"ข้อมูลกลไกสถานี {k} ไม่ครบ: {sorted(L)}"
    S = min(shafts, key=lambda c: np.linalg.norm(c - L["A0"])); assert np.linalg.norm(S - L["A0"]) < 0.05, f"หาเพลาเซอร์โวของ {k} ไม่เจอ"
    S = L["A0"] + (S - L["A0"]) - L["s"] * np.dot(S - L["A0"], L["s"])       # จุดบนแกนในระนาบเดียวกับบอลปลายแขน
    linkage[k] = dict(S=S.round(5).tolist(), s=L["s"].tolist(), A0=L["A0"].round(5).tolist(), H0=L["H0"].round(5).tolist(),
                      arm=float(np.linalg.norm(S - L["A0"])), rod=float(np.linalg.norm(L["H0"] - L["A0"])))

blob, meshes, ntri, nvert = bytearray(), [], 0, 0
def put(arr):
    while len(blob) % 4: blob.append(0)
    o = len(blob); blob.extend(arr.tobytes()); return o
lo_all, hi_all = np.full(3, np.inf), np.full(3, -np.inf)
for key in sorted(buckets, key=lambda k: (k[0], not k[1], -k[3], k[2])):
    grp, skin, rgb, alpha, spec, shine = key
    P = np.concatenate([p for p, _ in buckets[key]]); N = np.concatenate([n for _, n in buckets[key]])
    lo, hi = P.min(0), P.max(0); ext = np.maximum(hi - lo, 1e-9)
    q = np.round((P - lo) / ext * 65535).astype("<u2"); n8 = np.clip(np.round(N * 127), -127, 127).astype("i1")
    rec = np.zeros(len(q), dtype=[("p", "<u2", 3), ("n", "i1", 3)]); rec["p"] = q; rec["n"] = n8
    uniq, inv = np.unique(rec, return_inverse=True)
    idx = inv.reshape(-1, 3); a, b_, c_ = uniq["p"][idx[:, 0]], uniq["p"][idx[:, 1]], uniq["p"][idx[:, 2]]
    keep = ~((a == b_).all(1) | (b_ == c_).all(1) | (a == c_).all(1)); idx = idx[keep]       # ตัดสามเหลี่ยมลีบหลังปัดค่า
    it = "u16" if len(uniq) <= 65535 else "u32"
    meshes.append(dict(group=grp, skin=skin, color=list(rgb), opacity=alpha, specular=spec, shininess=shine, nvert=int(len(uniq)), ntri=int(len(idx)),
                       min=lo.round(6).tolist(), ext=ext.round(6).tolist(),
                       pos=put(np.ascontiguousarray(uniq["p"])), nrm=put(np.ascontiguousarray(uniq["n"])),
                       idx=put(idx.astype("<u2" if it == "u16" else "<u4")), idxType=it))
    ntri += len(idx); nvert += len(uniq); lo_all = np.minimum(lo_all, lo); hi_all = np.maximum(hi_all, hi)

wing = [b for b in raw["bodies"] if base_name(b["body"]).startswith("WP_Wing")]
meta = dict(version=1, source=dict(asm=raw["asm"], asm_mtime=raw["asm_mtime"], exported=raw["exported"]),
            frame="X ขวา, Y ขึ้น, Z ไปท้าย (m) · จุดกำเนิด = CG อ้างอิง", ref=dict(x_cg=X_CG, y_pod_axis_cad=round(y_axis, 5), z_cg_above_pod_axis=Z_CG),
            wing_mid_y=round(float(np.mean([0.5 * (b["box"][1] + b["box"][4]) for b in wing])) - y0, 4),
            bbox=dict(min=lo_all.round(4).tolist(), max=hi_all.round(4).tolist()), nbody=used, ntri=int(ntri), nvert=int(nvert),
            hinges=hinges, linkage=linkage, nozzle=nozzle, parts=parts, bin="model.bin", bytes=len(blob), meshes=meshes)
open(os.path.join(OUT, "model.bin"), "wb").write(blob)
json.dump(meta, open(os.path.join(OUT, "model.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print(f"body {used} (ข้าม {len(skipped)}: {skipped[:4]}) → mesh {len(meshes)} · สามเหลี่ยม {ntri:,} · จุด {nvert:,} · model.bin {len(blob) / 1e6:.2f} MB")
print(f"กรอบ (m): {lo_all.round(3)} … {hi_all.round(3)} · แกน pod ใน CAD y = {y_axis:.4f}")
for s, h in hinges.items(): print(f"บานพับ {s}: จุด {np.round(h['p'], 3)} แกน {np.round(h['axis'], 4)} ยาว {h['length']:.3f} m")
for k, L in linkage.items(): print(f"กลไก {k}: แขน {L['arm'] * 1000:.1f} mm · ก้าน {L['rod'] * 1000:.1f} mm · แกนเซอร์โว {np.round(L['s'], 3)}")
for g in sorted({m["group"] for m in meshes}):
    ms = [m for m in meshes if m["group"] == g]; print(f"  {g}: {len(ms)} mesh · {sum(m['ntri'] for m in ms):,} สามเหลี่ยม · ผิวนอก {sum(m['ntri'] for m in ms if m['skin']):,}")
