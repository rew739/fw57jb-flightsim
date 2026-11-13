# -*- coding: utf-8 -*-
"""ขั้น 1/2 ของโมเดลเครื่องบินในเกม: ดึงผิวสามเหลี่ยม (tessellation ที่ SolidWorks ใช้แสดงผล) ของทุก body
ใน 04_CAD/FW57J_jet/1_current/cad/FW57J_B.SLDASM ออกมาในพิกัด assembly พร้อมสี/ความโปร่งของ body

- เปิด assembly แบบอ่านอย่างเดียว ไม่แก้ ไม่บันทึก CAD · ถ้าผู้ใช้เปิดไฟล์นี้อยู่แล้วจะใช้เอกสารนั้นและไม่ปิด
- ผลดิบ (ไฟล์ใหญ่) ลงโฟลเดอร์ cache นอกโปรเจกต์: <cache>/raw.json + raw.f32 (จุด) + raw_n.f32 (normal ต่อจุด)  → ขั้น 2: build_model.py
ใช้:  python 10_FlightSim_Game/tools/export_cad_mesh.py [--cache C:/tmp/fw57jb_mesh]
รางปล่อย (ข้ามชิ้นของเครื่องบินที่อยู่ใน assembly ราง แต่จดตำแหน่งเครื่องไว้ใน "assemblies" → ขั้น 2: build_rail.py):
      python 10_FlightSim_Game/tools/export_cad_mesh.py --asm 04_CAD/Launcher/1_current/cad/LR2_assembly.SLDASM --skip FW57J_jet --cache C:/tmp/lr2_mesh
ต้องเปิด SolidWorks ก่อน (นอกมหาวิทยาลัยต่อ VPN)"""
import os, sys, io, json, time, argparse, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", line_buffering=True)
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "06_FlyingWing_Design", "calc", "fea_com"))
import numpy as np
import win32com.client as w, sldworks_tlb as sw
from swconst_tlb import constants as c

ap = argparse.ArgumentParser(); ap.add_argument("--cache", default="C:/tmp/fw57jb_mesh")
ap.add_argument("--asm", default="04_CAD/FW57J_jet/1_current/cad/FW57J_B.SLDASM", help="พาธ assembly นับจากรากโปรเจกต์")
ap.add_argument("--skip", default="", help="regex ของพาธไฟล์ชิ้นที่ไม่ต้องดึงผิว"); A = ap.parse_args()
ASM = os.path.join(ROOT, *A.asm.replace("\\", "/").split("/"))
T0 = time.time()
def t(): return f"[{time.time() - T0:5.0f} s]"

app = sw.ISldWorks(w.GetActiveObject("SldWorks.Application")._oleobj_)
mine = False
m_ = app.GetOpenDocumentByName(ASM)
if m_ is None:
    r = app.OpenDoc6(ASM, 2, 1 | 2, "", 0, 0)                      # 1 = เงียบ, 2 = อ่านอย่างเดียว
    m_ = r[0] if isinstance(r, tuple) else r; mine = True
assert m_ is not None, "เปิด assembly ไม่ได้"
d = sw.IModelDoc2(m_._oleobj_); asm = sw.IAssemblyDoc(m_._oleobj_)
print(t(), "เปิดแล้ว", d.GetTitle(), "(สคริปต์เปิดเอง)" if mine else "(ผู้ใช้เปิดอยู่)")
asm.ResolveAllLightWeightComponents(False)

def mpv(x):
    try:
        v = x() if callable(x) else x
        return [float(q) for q in v] if v is not None and len(v) >= 8 else None
    except Exception: return None

def chain(cc):
    ch, p = [], cc.GetParent()
    while p is not None:
        p = sw.IComponent2(p._oleobj_); ch.append(p); p = p.GetParent()
    return ch

os.makedirs(A.cache, exist_ok=True)
recs, asms, off, ntri = [], [], 0, 0
with open(os.path.join(A.cache, "raw.f32"), "wb") as fo, open(os.path.join(A.cache, "raw_n.f32"), "wb") as fn:
    comps = [sw.IComponent2(x._oleobj_) for x in (asm.GetComponents(False) or [])]
    for ic, cc in enumerate(comps):
        pth = cc.GetPathName() or ""
        if pth.lower().endswith(".sldasm"):                      # assembly ย่อย: จดตำแหน่งไว้ (ใช้โยงพิกัดรางกับเครื่องบิน)
            asms.append(dict(inst=cc.Name2, path=os.path.relpath(pth, ROOT).replace("\\", "/"), T=list(sw.IMathTransform(cc.GetTotalTransform(False)._oleobj_).ArrayData)[:13]))
        if not pth.lower().endswith(".sldprt"): continue
        if A.skip and re.search(A.skip, pth.replace("\\", "/"), re.I): continue
        ch = chain(cc)
        if cc.IsSuppressed() or any(p.IsSuppressed() for p in ch): continue
        env = bool(cc.IsEnvelope()) or any(bool(p.IsEnvelope()) for p in ch)
        hid = bool(cc.IsHidden(False))
        T = list(sw.IMathTransform(cc.GetTotalTransform(False)._oleobj_).ArrayData)
        R = np.array(T[:9]).reshape(3, 3); tr = np.array(T[9:12]); sc = T[12]
        md = cc.GetModelDoc2(); md = sw.IModelDoc2(md._oleobj_) if md is not None else None
        ccol = mpv(lambda: cc.MaterialPropertyValues); pcol = mpv(lambda: md.MaterialPropertyValues) if md else None
        bs = cc.GetBodies3(c.swSolidBody, None)
        if isinstance(bs, tuple) and bs and isinstance(bs[0], tuple): bs = bs[0]
        nb = 0
        for b in bs or []:
            if b is None: continue
            b = sw.IBody2(b._oleobj_)
            bcol = mpv(lambda: b.MaterialPropertyValues2)
            tris, nrms, fcols = [], [], []
            for f in (b.GetFaces() or []):
                f = sw.IFace2(f._oleobj_)
                tt = f.GetTessTriangles(True)
                if not tt: continue
                fc = mpv(lambda: f.MaterialPropertyValues)
                nn = f.GetTessNorms()
                if not nn or len(nn) != len(tt): nn = [0.0] * len(tt)          # ไม่มี normal → ขั้น 2 ใช้ normal ของสามเหลี่ยมแทน
                fcols.append((len(tt) // 9, fc)); tris.append(np.asarray(tt, dtype=np.float64)); nrms.append(np.asarray(nn, dtype=np.float64))
            if not tris: continue
            P = np.concatenate(tris).reshape(-1, 3)
            P = (P @ R) * sc + tr                                   # SolidWorks: p' = p·R + t
            P32 = P.astype("<f4"); fo.write(P32.tobytes())
            fn.write((np.concatenate(nrms).reshape(-1, 3) @ R).astype("<f4").tobytes())
            n = len(P32) // 3
            # ช่วงสีรายผิว (เฉพาะผิวที่ตั้งสีเอง)
            fr, k = [], 0
            for cnt, fc in fcols:
                if fc is not None: fr.append([k, cnt, fc[:3], fc[7]])
                k += cnt
            recs.append(dict(inst=cc.Name2, part=os.path.splitext(os.path.basename(pth))[0], body=b.Name, envelope=env, hidden=hid,
                             color=bcol or ccol or pcol, color_src="body" if bcol else "comp" if ccol else "part" if pcol else "none",
                             face_colors=fr, det=float(np.linalg.det(R)), offset=off, ntri=n,
                             box=[*P.min(0).round(5).tolist(), *P.max(0).round(5).tolist()]))
            off += P32.nbytes; ntri += n; nb += 1
        print(t(), f"{ic + 1}/{len(comps)} {cc.Name2}: {nb} body · รวม {ntri:,} สามเหลี่ยม" + (" [envelope]" if env else "") + (" [hidden]" if hid else ""))
json.dump(dict(asm=os.path.relpath(ASM, ROOT).replace("\\", "/"), asm_mtime=os.path.getmtime(ASM), exported=time.strftime("%Y-%m-%d %H:%M"),
               units="m", assemblies=asms, bodies=recs), open(os.path.join(A.cache, "raw.json"), "w", encoding="utf-8"), ensure_ascii=False)
print(t(), f"เสร็จ: {len(recs)} body · {ntri:,} สามเหลี่ยม · {off / 1e6:.1f} MB → {A.cache}")
if mine:
    app.CloseDoc(d.GetTitle()); print(t(), "ปิด assembly ที่สคริปต์เปิดเอง (ไม่บันทึก)")
