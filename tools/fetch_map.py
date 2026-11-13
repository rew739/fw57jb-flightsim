# -*- coding: utf-8 -*-
"""ดึงแผนที่จริงมาให้เกมใช้: ความสูงพื้น + ภาพถ่ายดาวเทียม → maps/<ชื่อ>/
รัน:  python 10_FlightSim_Game/tools/fetch_map.py chiangmai            (ทั้งหมด ~3,000 ไฟล์ต้นทาง)
      python 10_FlightSim_Game/tools/fetch_map.py chiangmai --stage elev   (เฉพาะความสูง)
ต้องมี numpy + Pillow · ต่ออินเทอร์เน็ตเฉพาะตอนรันสคริปต์นี้ ตัวเกมเล่นได้โดยไม่ต่อเน็ต

แหล่งข้อมูล (ใช้ฟรี ต้องแสดงที่มา):
  ความสูง  Mapzen Terrain Tiles บน AWS Open Data (terrarium, ซูม 11 ≈ 72 m/จุด · ข้อมูลหลัก SRTM)
  ภาพ      Sentinel-2 cloudless 2023 ของ EOX (https://s2maps.eu) สัญญาอนุญาต CC BY-NC-SA 4.0 (ห้ามใช้เชิงพาณิชย์)

ผลลัพธ์ (พิกัดเดียวกับเกม: จุดกำเนิด = ลานปล่อย, x = ตะวันออก, z = −เหนือ):
  meta.json   ตำแหน่ง ขนาดตาราง ระดับภาพที่มี ที่มาข้อมูล
  elev.bin    int16 × 0.1 m ตาราง n × n จุด ห่าง 64 m (แถวแรก = ขอบเหนือ)
  forest.bin  uint8 0–255 ความหนาแน่นป่าโดยประมาณจากสีภาพ (ตารางเดียวกับ elev)
  img/<L>/<ix>_<iz>.jpg   แผ่นภาพ 256 px ขนาดพื้น 16384 / 2^L m (L0–L2 ทั้งแผนที่, L3 เฉพาะรอบลานปล่อย)
"""
import argparse, io, json, math, os, shutil, sys, time, urllib.request
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from PIL import Image

GAME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT, PX = 16384, 256
HALF_ROOTS, FINE_ROOTS = 6, 2          # แผนที่ ±6 ช่องราก (±98.3 km) · ภาพละเอียด L3 ±2 ช่องราก (±32.8 km)
STEP, SCALE = 64, 0.1                  # ตารางความสูง: ระยะจุด (m), หน่วยที่เก็บ (m)
Z_ELEV, Z_IMG, Z_FINE = 11, 13, 14
ELEV_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
IMG_URL = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2023_3857/default/g/{z}/{y}/{x}.jpg"
UA = {"User-Agent": "flightsim-map-fetch/1.0 (python-urllib)"}

# ลานปล่อย = จุดกำเนิดของเกม (เลือกที่โล่งราบในแอ่งเชียงใหม่–ลำพูน ไม่ใช่สนามบินจริง)
MAPS = {
    "chiangmai": {"name": "เชียงใหม่ (แผนที่จริง)", "lat0": 18.6850, "lon0": 99.0880},
}


def m_per_deg(lat0):
    p = math.radians(lat0)
    return (111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p),
            111412.84 * math.cos(p) - 93.5 * math.cos(3 * p))


class Area:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.mlat, self.mlon = m_per_deg(lat0)

    def global_px(self, n, e, z):
        """ตำแหน่ง (เหนือ, ตะวันออก) เมตร → พิกัดจุดภาพรวมของ Web Mercator ที่ซูม z"""
        lat = np.radians(self.lat0 + np.asarray(n, float) / self.mlat)
        lon = self.lon0 + np.asarray(e, float) / self.mlon
        s = PX * 2 ** z
        return (lon + 180.0) / 360.0 * s, (1.0 - np.arcsinh(np.tan(lat)) / math.pi) / 2.0 * s

    def tile_range(self, half, z):
        gx, gy = self.global_px([half + 200, -half - 200], [-half - 200, half + 200], z)
        return int(gx[0] // PX), int(gx[1] // PX), int(gy[0] // PX), int(gy[1] // PX)


class Source:
    """แผ่นต้นทาง (เก็บสำรองบนดิสก์ + จำในหน่วยความจำจำนวนจำกัด)"""

    def __init__(self, url, ext, cache, kind):
        self.url, self.ext, self.dir, self.kind, self.mem = url, ext, os.path.join(cache, kind), kind, {}

    def path(self, z, x, y):
        return os.path.join(self.dir, str(z), f"{x}_{y}.{self.ext}")

    def download(self, z, tiles):
        todo = [(x, y) for x, y in tiles if not os.path.exists(self.path(z, x, y))]
        print(f"  {self.kind} ซูม {z}: ต้องใช้ {len(tiles)} แผ่น · มีแล้ว {len(tiles) - len(todo)} · ดาวน์โหลด {len(todo)}", flush=True)
        os.makedirs(os.path.join(self.dir, str(z)), exist_ok=True)
        done, size, t0 = [0], [0], time.time()

        def get(xy):
            x, y = xy
            for attempt in range(5):
                try:
                    req = urllib.request.Request(self.url.format(z=z, x=x, y=y), headers=UA)
                    with urllib.request.urlopen(req, timeout=40) as r:
                        b = r.read()
                    tmp = self.path(z, x, y) + ".part"
                    with open(tmp, "wb") as f:
                        f.write(b)
                    os.replace(tmp, self.path(z, x, y))
                    done[0] += 1; size[0] += len(b)
                    if done[0] % 200 == 0:
                        print(f"    {done[0]}/{len(todo)} · {size[0] / 1e6:.1f} MB · {time.time() - t0:.0f} s", flush=True)
                    return None
                except Exception as ex:          # ลองใหม่ (เครือข่ายสะดุด / เซิร์ฟเวอร์ให้รอ)
                    err = f"{z}/{x}/{y}: {ex}"
                    time.sleep(1.5 * (attempt + 1))
            return err

        with ThreadPoolExecutor(6) as pool:
            errs = [e for e in pool.map(get, todo) if e]
        if errs:
            print(f"  ❌ ดาวน์โหลดไม่สำเร็จ {len(errs)} แผ่น เช่น {errs[0]}", flush=True)
            sys.exit(1)
        return size[0]

    def tile(self, z, x, y):
        k = (z, x, y)
        if k not in self.mem:
            if len(self.mem) > 80:
                self.mem.clear()
            a = np.asarray(Image.open(self.path(z, x, y)).convert("RGB"), dtype=np.float32)
            if self.kind == "elev":
                a = (a[:, :, 0] * 256.0 + a[:, :, 1] + a[:, :, 2] / 256.0 - 32768.0)[:, :, None]
            self.mem[k] = a
        return self.mem[k]

    def sample(self, z, gx, gy):
        """สุ่มค่าแบบ bilinear ที่พิกัดจุดภาพรวม (gx, gy) → อาร์เรย์ [..., ช่องสี]"""
        x, y = gx - 0.5, gy - 0.5
        x0, y0 = np.floor(x).astype(np.int64), np.floor(y).astype(np.int64)
        fx, fy = (x - x0)[..., None].astype(np.float32), (y - y0)[..., None].astype(np.float32)
        tx0, tx1, ty0, ty1 = x0.min() // PX, (x0.max() + 1) // PX, y0.min() // PX, (y0.max() + 1) // PX
        mos = np.concatenate([np.concatenate([self.tile(z, tx, ty) for tx in range(tx0, tx1 + 1)], axis=1) for ty in range(ty0, ty1 + 1)], axis=0)
        ix, iy = x0 - tx0 * PX, y0 - ty0 * PX
        a, b, c, d = mos[iy, ix], mos[iy, ix + 1], mos[iy + 1, ix], mos[iy + 1, ix + 1]
        return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy


def build_elev(area, src, out):
    half, n = HALF_ROOTS * ROOT, 2 * HALF_ROOTS * ROOT // STEP + 1
    elev = np.zeros((n, n), np.float32)
    for j0 in range(0, n, 512):
        for i0 in range(0, n, 512):
            jj, ii = np.arange(j0, min(j0 + 512, n)), np.arange(i0, min(i0 + 512, n))
            N, E = np.meshgrid(half - jj * STEP, -half + ii * STEP, indexing="ij")
            elev[j0:j0 + len(jj), i0:i0 + len(ii)] = src.sample(Z_ELEV, *area.global_px(N, E, Z_ELEV))[..., 0]
    fixed = despike(elev) + despike(elev, win=3, limit=60.0)      # รอบสอง: จุดเดี่ยวที่โดดจาก 8 จุดรอบตัวเกิน 60 m
    np.clip(np.round(elev / SCALE), -32768, 32767).astype("<i2").tofile(os.path.join(out, "elev.bin"))
    return elev, n, fixed


def despike(elev, win=5, limit=150.0, passes=3):
    """ลบจุดเสียในข้อมูลต้นทาง (ยอดแหลม/หลุมลึกหลายพันเมตรตามขอบอ่างเก็บน้ำ): จุดที่ต่างจากค่ากลางของ 5 × 5 จุดรอบตัว
    (รัศมี 128 m) เกิน 150 m ถูกแทนด้วยค่ากลาง · ยอดเขาจริงชันไม่เกินราว 55° จะไม่ถูกแตะ"""
    from numpy.lib.stride_tricks import sliding_window_view
    r, total = win // 2, 0
    for _ in range(passes):
        pad, bad_n = np.pad(elev, r, mode="edge"), 0
        for j0 in range(0, elev.shape[0], 256):
            blk = pad[j0:j0 + 256 + 2 * r]
            med = np.median(sliding_window_view(blk, (win, win)).reshape(blk.shape[0] - 2 * r, elev.shape[1], win * win), axis=2)
            cur = elev[j0:j0 + med.shape[0]]
            bad = np.abs(cur - med) > limit
            cur[bad] = med[bad]; bad_n += int(bad.sum())
        total += bad_n
        if not bad_n:
            break
    return total


def forest_from_rgb(rgb):
    """ประมาณความหนาแน่นป่าจากสีจริง: เขียวเข้ม = ป่า · สว่าง/น้ำตาล/เทา = นา เมือง ดินโล่ง"""
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    lum = 0.3 * r + 0.59 * g + 0.11 * b
    green = (g - np.maximum(r, b)) / np.maximum(lum, 1.0)          # เขียวเด่นแค่ไหน
    # ปรับเทียบกับภาพเชียงใหม่: ป่าบนเขา ความสว่างกลาง ~30 เขียวเด่น ≥ 0.3 · นา/สวน ความสว่างกลาง 64–74 เขียวเด่น 0.1–0.2
    f = np.clip((green - 0.22) / 0.20, 0, 1) * np.clip((58.0 - lum) / 18.0, 0, 1)
    return f


def build_forest(area, src, out, n):
    half, mask = HALF_ROOTS * ROOT, np.zeros((n, n), np.uint8)
    for j0 in range(0, n, 256):
        for i0 in range(0, n, 256):
            jj, ii = np.arange(j0, min(j0 + 256, n)), np.arange(i0, min(i0 + 256, n))
            N, E = np.meshgrid(half - jj * STEP, -half + ii * STEP, indexing="ij")
            mask[j0:j0 + len(jj), i0:i0 + len(ii)] = np.round(255 * forest_from_rgb(src.sample(Z_IMG, *area.global_px(N, E, Z_IMG))))
    mask.tofile(os.path.join(out, "forest.bin"))
    return float(mask.mean()) / 255


def save_jpg(arr, path):
    Image.fromarray(np.clip(np.round(arr), 0, 255).astype(np.uint8)).save(path, quality=86, optimize=True)


def build_tiles(area, src, out, L, z, r):
    """แผ่นภาพระดับ L จากต้นทางซูม z · ดัชนี −r … r−1 ทั้งสองแกน"""
    S, d = ROOT / 2 ** L, os.path.join(out, "img", str(L))
    os.makedirs(d, exist_ok=True)
    c = (np.arange(PX) + 0.5) / PX
    for iz in range(-r, r):
        for ix in range(-r, r):
            Z, X = np.meshgrid((iz + c) * S, (ix + c) * S, indexing="ij")
            save_jpg(src.sample(z, *area.global_px(-Z, X, z)), os.path.join(d, f"{ix}_{iz}.jpg"))
        if (iz + r) % 8 == 7:
            print(f"    L{L}: แถว {iz + r + 1}/{2 * r}", flush=True)
    return {"L": L, "x0": -r, "x1": r - 1, "z0": -r, "z1": r - 1}


def build_coarse(out, L, r):
    """แผ่นระดับ L ย่อจาก 4 แผ่นของระดับ L + 1"""
    d, dn = os.path.join(out, "img", str(L)), os.path.join(out, "img", str(L + 1))
    os.makedirs(d, exist_ok=True)
    for iz in range(-r, r):
        for ix in range(-r, r):
            big = Image.new("RGB", (2 * PX, 2 * PX))
            for a in (0, 1):
                for b in (0, 1):
                    big.paste(Image.open(os.path.join(dn, f"{2 * ix + a}_{2 * iz + b}.jpg")), (a * PX, b * PX))
            big.resize((PX, PX), Image.BOX).save(os.path.join(d, f"{ix}_{iz}.jpg"), quality=86, optimize=True)
    return {"L": L, "x0": -r, "x1": r - 1, "z0": -r, "z1": r - 1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map", choices=sorted(MAPS))
    ap.add_argument("--stage", choices=["elev", "all"], default="all")
    ap.add_argument("--cache", default=None, help="โฟลเดอร์เก็บแผ่นต้นทาง (ค่าเริ่มต้น maps/<ชื่อ>/_cache ลบทิ้งเมื่อเสร็จ)")
    ap.add_argument("--keep-cache", action="store_true")
    ap.add_argument("--lat", type=float); ap.add_argument("--lon", type=float)
    a = ap.parse_args()
    cfg = dict(MAPS[a.map])
    if a.lat is not None: cfg["lat0"] = a.lat
    if a.lon is not None: cfg["lon0"] = a.lon
    out = os.path.join(GAME, "maps", a.map)
    cache = a.cache or os.path.join(out, "_cache")
    os.makedirs(out, exist_ok=True)
    area, half = Area(cfg["lat0"], cfg["lon0"]), HALF_ROOTS * ROOT
    t0 = time.time()
    print(f"แผนที่ {a.map}: ลานปล่อย {cfg['lat0']:.4f}°N {cfg['lon0']:.4f}°E · ครอบคลุม ±{half / 1000:.1f} km", flush=True)

    elev_src = Source(ELEV_URL, "png", cache, "elev")
    x0, x1, y0, y1 = area.tile_range(half, Z_ELEV)
    elev_src.download(Z_ELEV, [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)])
    elev, n, fixed = build_elev(area, elev_src, out)
    print(f"  แก้จุดเสียในข้อมูลความสูงต้นทาง {fixed} จุด", flush=True)
    c = n // 2
    site = elev[c - 5:c + 6, c - 5:c + 6]
    ring = elev[c - 40:c + 41, c - 40:c + 41]
    site_alt = round(float(site.mean()), 1)
    print(f"  ความสูง: ตาราง {n} × {n} · ต่ำสุด {elev.min():.0f} m · สูงสุด {elev.max():.0f} m", flush=True)
    print(f"  ลานปล่อย {site_alt} m · รอบ 2.5 km ต่างกัน {ring.min() - site_alt:+.1f} ถึง {ring.max() - site_alt:+.1f} m", flush=True)

    meta = {
        "id": a.map, "name": cfg["name"], "lat0": cfg["lat0"], "lon0": cfg["lon0"],
        "m_per_deg_lat": area.mlat, "m_per_deg_lon": area.mlon,
        "grid": {"n": n, "step": STEP, "half": half, "scale": SCALE},
        "site_alt": site_alt, "site_flat_m": [450, 2200], "elev_max": round(float(elev.max()), 1), "elev_min": round(float(elev.min()), 1), "elev_despiked": fixed,
        "img": {"root": ROOT, "px": PX, "path": "img", "levels": []},
        "attribution": [
            "ความสูง: Mapzen Terrain Tiles (AWS Open Data) — ข้อมูล SRTM ของ NASA/USGS และแหล่งอื่น",
            "ภาพ: Sentinel-2 cloudless 2023 — https://s2maps.eu โดย EOX IT Services GmbH (มีข้อมูล Copernicus Sentinel ที่ดัดแปลงแล้ว ปี 2023) · CC BY-NC-SA 4.0",
        ],
        "built": time.strftime("%Y-%m-%d"),
    }
    if a.stage == "all":
        img_src = Source(IMG_URL, "jpg", cache, "img")
        x0, x1, y0, y1 = area.tile_range(half, Z_IMG)
        img_src.download(Z_IMG, [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)])
        x0, x1, y0, y1 = area.tile_range(FINE_ROOTS * ROOT, Z_FINE)
        img_src.download(Z_FINE, [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)])
        if os.path.isdir(os.path.join(out, "img")):
            shutil.rmtree(os.path.join(out, "img"))
        print("  สร้างแผ่นภาพตามพิกัดเกม …", flush=True)
        l2 = build_tiles(area, img_src, out, 2, Z_IMG, HALF_ROOTS * 4)
        l3 = build_tiles(area, img_src, out, 3, Z_FINE, FINE_ROOTS * 8)
        l1 = build_coarse(out, 1, HALF_ROOTS * 2)
        l0 = build_coarse(out, 0, HALF_ROOTS)
        meta["img"]["levels"] = [l0, l1, l2, l3]
        meta["forest"] = "forest.bin"
        print(f"  ป่าปกคลุมโดยประมาณ {100 * build_forest(area, img_src, out, n):.0f} % ของพื้นที่", flush=True)
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)

    if a.stage == "all":
        idx_path = os.path.join(GAME, "maps", "index.json")
        idx = json.load(open(idx_path, encoding="utf-8")) if os.path.exists(idx_path) else []
        idx = [m for m in idx if m["id"] != a.map] + [{"id": a.map, "name": cfg["name"]}]
        with open(idx_path, "w", encoding="utf-8") as f:
            json.dump(idx, f, ensure_ascii=False, indent=1)
        if not a.keep_cache and a.cache is None:
            shutil.rmtree(cache, ignore_errors=True)
    files = sum(len(fs) for d, _, fs in os.walk(out) if "_cache" not in d)
    size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(out) if "_cache" not in d for f in fs)
    print(f"เสร็จ: {files} ไฟล์ · {size / 1e6:.1f} MB ใน maps/{a.map} · ใช้เวลา {time.time() - t0:.0f} s", flush=True)


if __name__ == "__main__":
    main()
