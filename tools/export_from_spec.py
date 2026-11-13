# -*- coding: utf-8 -*-
"""ส่งออกข้อมูลเครื่อง FW57J-B จาก spec + ผลคำนวณ → data/fw57jb_sim.json ให้เกมจำลองใช้

รัน:  python 10_FlightSim_Game/tools/export_from_spec.py
อ่าน (ไม่แก้):
  06_FlyingWing_Design/FW57J_B_BG25_spec_for_sim.json   มวล CG อากาศพลศาสตร์ แรงขับ ราง ขีดจำกัด
  06_FlyingWing_Design/calc/phaseFT2_spec_v36.json      ตำแหน่งถังน้ำมัน
  06_FlyingWing_Design/calc/phaseDFA6_f2_inertia.json   ความเฉื่อยเครื่องเปล่า + ส่วนของน้ำมัน/cargo
  06_FlyingWing_Design/calc/phaseL4_parachute.json      ร่มหลัก
ค่าที่ spec ไม่มี (อนุพันธ์เชิงอัตราหมุน เวลาเร่งรอบเครื่อง ฯลฯ) ประมาณด้วยสูตรตำรา และระบุชื่อไว้ใน "estimated"
"""
import json, math, os, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
D06 = os.path.join(ROOT, "06_FlyingWing_Design")


def load(*p):
    with open(os.path.join(D06, *p), encoding="utf-8") as f:
        return json.load(f)


S = load("FW57J_B_BG25_spec_for_sim.json")
FT = load("calc", "phaseFT2_spec_v36.json")
IN = load("calc", "phaseDFA6_f2_inertia.json")
CH = load("calc", "phaseL4_parachute.json")

G0 = 9.80665
geo, mk, cg = S["geometry"], S["mass_kg"], S["cg_m_from_CAD_origin"]
av, pol, lim, prop = S["aero_OpenVSP"], S["polar_design"], S["limits"], S["propulsion"]
b, Sref, mac, cr, ct = geo["b"], geo["S"], geo["MAC"], geo["cr"], geo["ct"]
half = b / 2
lam = ct / cr

# ---------- มวล / CG / ความเฉื่อย ----------
order = S["loading_rule"]["fuel_burn_order"]
cat = IN["categories"]
fuel_cat = {"POD": "น้ำมัน (pod)", "WING": "น้ำมัน (ปีก)", "LE": "น้ำมัน (ถังขอบหน้า FT)", "A3": "น้ำมัน (ถัง A3 FT)"}
tanks = []
for k in order:
    c = cat[fuel_cat[k]]
    tanks.append(dict(name=k, kg_cap=FT["tanks"][k]["kg"], x=FT["tanks"][k]["x"], y_span=FT["tanks"][k]["z"],
                      I_at_ref=dict(kg=c["kg"], Ixx=c["Ixx"], Iyy=c["Iyy"], Izz=c["Izz"])))
st0 = IN["states"]["0_หมด"]
m_e, x_e = mk["empty"], FT["X_E1"]
zc = cg["z_above_pod_axis_all_cases"]
z_e = zc["0_หมด"]
m_full0 = m_e + mk["fuel_capacity_kg"]
z_fuel = (m_full0 * zc["0_เต็ม"] - m_e * z_e) / mk["fuel_capacity_kg"]
z_cargo = ((m_e + 20) * zc["20_หมด"] - m_e * z_e) / 20.0
# ความเฉื่อยของ cargo 20 kg = ส่วนที่เหลือ เพื่อให้สถานะ MTOW ตรงค่า spec (spec เฉลี่ยกรณี cargo อัดแน่น/เต็มช่อง)
Isp = S["inertia_kgm2_about_CG"]
I_cargo20 = dict(
    Ixx=Isp["Ixx_roll"] - st0["Ixx_roll"] - sum(t["I_at_ref"]["Ixx"] for t in tanks),
    Iyy=Isp["Iyy_pitch"] - st0["Iyy_pitch"] - sum(t["I_at_ref"]["Iyy"] for t in tanks),
    Izz=Isp["Izz_yaw"] - st0["Izz_yaw"] - sum(t["I_at_ref"]["Izz"] for t in tanks))
mass = dict(
    empty_kg=m_e, x_empty=x_e, z_empty=z_e,
    cargo_max_kg=20.0, x_cargo=FT.get("X_CARGO", 0.5) if isinstance(FT.get("X_CARGO"), float) else 0.5, z_cargo=z_cargo,
    z_fuel=z_fuel, fuel_rho_kg_per_L=FT["RHO"], fuel_cap_kg=mk["fuel_capacity_kg"],
    tow_max_rail_kg=FT["M_RAIL"], tanks=tanks, burn_order=order,
    I_empty=dict(Ixx=st0["Ixx_roll"], Iyy=st0["Iyy_pitch"], Izz=st0["Izz_yaw"]),
    I_cargo20=I_cargo20,
    I_mtow_spec=S["inertia_kgm2_about_CG"],
    cg_window_x=S["loading_rule"]["cg_window_x"],
)

# ---------- อากาศพลศาสตร์ ----------
NP = cg["NP"]
x_ref = 0.510                                    # จุดอ้างโมเมนต์ของผล OpenVSP
VSP = load("openvsp", "results", "fw57jb_aero.json")
Cm0_ref = VSP["long"]["Cm0"]
# โมเมนต์รอบ NP (ไม่ขึ้นกับ α): Cm_NP = Cm0 + CL0·(NP_wing − x_ref)/MAC
Cm_ac = Cm0_ref + av["CL0"] * (av["NP_wing"] - x_ref) / mac
lat_lo, lat_hi = VSP["lat"]["cruise 200 km/h"], VSP["lat"]["หลุดราง 26.8 m/s"]

# elevon: จุดศูนย์กลางแรงยก (ถ่วงตามคอร์ด) ของช่วง y 0.729–1.788
y0, y1 = 0.729, 1.788
chord = lambda y: cr - (cr - ct) * y / half
n = 200
ys = [y0 + (y1 - y0) * (i + 0.5) / n for i in range(n)]
y_e = sum(y * chord(y) for y in ys) / sum(chord(y) for y in ys)
CLde = av["CLde_per_deg"]
Clda = CLde * y_e / b                             # ต่อองศาของ δa = (δซ้าย − δขวา)/2

CLa = av["CLa"]
est = {}
est["Clp"] = -0.85 * CLa * (1 + 3 * lam) / (12 * (1 + lam))        # strip theory × 0.85 (ปีกลู่ AR 6)
est["Cnp_per_CL"] = -0.125                                         # Cnp ≈ −CL/8
est["Clr_per_CL"] = 0.25                                           # Clr ≈ CL/4
est["Cnr0"] = -0.022                                               # winglet + แรงต้านปีก
est["Cnr_per_CL2"] = -0.020
est["CYr"] = 0.05
est["CYp"] = 0.0
est["CLq"] = 2.5
est["Cnda_per_CL"] = -0.08                                         # adverse yaw: Cn_δa = −0.08·CL·Cl_δa
est["CDde_per_deg2"] = 3.0e-5                                      # แรงต้านจาก elevon โก่ง
est["post_stall_CL_k"] = 0.9                                       # CL = k·sin 2α หลังร่วงหล่น
est["post_stall_CD_k"] = 1.25                                      # CD = CD0 + k·sin²α
est["post_stall_Cm"] = -0.10                                       # หัวทิ่มหลังร่วงหล่น (โคนปีกร่วงก่อน y 0.655)
est["stall_blend_M"] = 40.0

aero = dict(
    S=Sref, b=b, MAC=mac, NP_x=NP, x_ref=x_ref,
    CL0=av["CL0"], CLa=CLa, CLmax_design=av["CLmax_design"],
    alpha_stall_deg=22.0,                          # ปรับให้ความเร็วร่วงหล่นตอน trim ที่ MTOW = 75.2 km/h (CLmax ออกแบบ 1.19) — ดู tests
    alpha_stall_neg_deg=-12.0,
    CD0=pol["CD0"], K=pol["K"],
    Cm_ac=Cm_ac, Cmq=S["aero_XFLR5"]["Cmq"],
    CLde_per_deg=CLde, Cmde_per_deg=av["CMde_per_deg"],
    Clda_per_deg=Clda, elevon_y_centroid=y_e, elevon_max_deg=lim["elevon_travel_deg"],
    lat_table=[
        dict(CL=lat_lo["CL"], Cnb=av["Cnb_cruise"], Clb=lat_lo["Clb"], CYb=lat_lo["CYb"]),
        dict(CL=lat_hi["CL"], Cnb=av["Cnb_rail_exit"], Clb=lat_hi["Clb"], CYb=lat_hi["CYb"]),
    ],
    lat_CL_clamp=[lat_lo["CL"], lat_hi["CL"]],     # ไม่ประมาณนอกช่วงข้อมูล (CL 0.155–0.665) — ที่ CL สูงกว่านี้ยังไม่มีผล OpenVSP/CFD
    **est,
)

# ---------- เครื่องยนต์ ----------
tv = sorted((float(k), v) for k, v in prop["thrust_N_full_vs_V"].items())
engine = dict(
    name=prop["engine"],
    thrust_full_table_kmh_N=[[k, v] for k, v in tv],
    thrust_axis_z=prop["thrust_axis_z_above_pod_axis"],
    fuel_flow_idle_g_min=prop["fuel_flow_g_min"]["idle"], fuel_flow_max_g_min=prop["fuel_flow_g_min"]["max"],
    egt_range_C=lim["engine_EGT_range_C"], ground_full_throttle_max_s=lim["engine_ground_full_throttle_max_s"],
    # ค่าประมาณ (ไม่มีใน spec): รอบเดินเบา/สูงสุด, เวลาเร่งรอบ, ลำดับสตาร์ต
    n_idle=0.27, idle_thrust_frac=0.04, spool_up_per_s=0.28, spool_down_per_s=0.40, spool_tau_s=0.6,
    rpm_max=125000, start_time_s=20.0, quick_start_time_s=3.0,
)

# ---------- รางปล่อย ----------
L = S["launcher"]
rail = dict(
    length_m=L["rail_length_m"], s_accel_m=L["s_accel_m"], valve_open_s=L["valve_open_s"], elev_deg=12.0,
    elevon_preset_deg=-4.8, carriage_height_m=1.093,   # CG อ้างอิงสูงจากพื้นตอนอยู่บนราง: จาก CAD ราง (models/lr2/model.json start · tools/build_rail.py)
    V_launch_ref_kmh=L["V_launch_kmh"], tow_ref_kg=FT["M_RAIL"],
    table=[dict(tow=r["TOW"], peak_g=r["rail_g"], V_kmh=r["V_launch"], bar=r["rail_bar"]) for r in S["loading_rule"]["payload_range"]],
)

# ---------- ร่ม ----------
cv = CH["canopies"]["cross"]
ac = cv["aircraft"]["FW57J-B"]
chute = dict(
    CDS_m2=cv["size"]["CDS"], D0_m=cv["size"]["D0"], vd_design_ms=CH["VD"],
    fill_const_m=3.5253 * (100 / 3.6),            # t_fill = const / V0 [m/s] (จากผล 100 km/h: 3.525 s)
    F_limit_N=CH["F_lim_FEA"], F_ult_N=CH["F_ult_FEA"],
    attach_above_cg_m=0.40, open_ref=dict(V_kmh=100, F_peak_N=ac["open"]["100"]["F"], h_loss_m=ac["open"]["100"]["h_loss"]),
)

out = dict(
    meta=dict(aircraft="FW57J-B", spec_name=S["name"][:60], generated=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
              source="tools/export_from_spec.py", estimated=sorted(est.keys()) + ["engine.n_idle", "engine.spool_*", "engine.start_time_s", "chute.attach_above_cg_m"]),
    atmos=dict(oat_C=35.0, p0_Pa=101325.0, g=G0, note="โครงการใช้อากาศ 35 °C ที่ระดับน้ำทะเล (ρ 1.145)"),
    geom=dict(S=Sref, b=b, MAC=mac, cr=cr, ct=ct, sweep_LE_deg=geo["sweep_LE_deg"], twist_tip_deg=geo["twist_tip_deg"],
              winglet_h=geo["winglet_h"], x_le_root=-0.031, elevon_y=[y0, y1], elevon_chord_frac=0.25,
              pod=dict(x_nose=-0.30, x_tail=1.30, half_w=0.15, half_h=0.17, r_exhaust=0.068)),
    mass=mass, aero=aero, engine=engine, rail=rail, chute=chute,
    limits=dict(V_NE_kmh=lim["V_NE_kmh"], V_C_kmh=lim["V_C_kmh"], n_limit=lim["n_limit"], n_ultimate=1.5 * lim["n_limit"],
                roll_full_deflection_up_to_kmh=S["elevon_actuation"]["autopilot_limits"]["roll_full_deflection_up_to_kmh"],
                V_cruise_design_kmh=S["performance"]["V_cruise_design_kmh"]),
    check=dict(D_200_N=S["performance"]["D_200_N"], Vs_CLmax119_MTOW_kmh=75.2, V_launch_MTOW_kmh=L["V_launch_kmh"],
               rail_peak_g_MTOW=L["a_peak_g"], chute_vd_full_ms=ac["vd_full"], range_200_cargo20_km=S["performance"]["range_by_cargo_200_km"]["20"],
               cg_x_cases=cg["x_all_cases"], SM_cases=cg["SM_all_cases"]),
)
dst = os.path.join(os.path.dirname(HERE), "data", "fw57jb_sim.json")
with open(dst, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("เขียน", dst)
print(f"  มวลเปล่า {m_e:.3f} kg · x {x_e:.4f} · I เปล่า {st0['Ixx_roll']:.2f}/{st0['Iyy_pitch']:.2f}/{st0['Izz_yaw']:.2f}")
print(f"  Cm_ac {Cm_ac:+.5f} · y_e {y_e:.3f} m · Clda {Clda:.5f}/deg · Clp {est['Clp']:.3f}")
print(f"  z_fuel {z_fuel:.4f} · z_cargo {z_cargo:.4f} · CDS ร่ม {chute['CDS_m2']:.2f} m²")
