# -*- coding: utf-8 -*-
"""เซิร์ฟเวอร์เล็กสำหรับเปิดเกม (เกมใช้ ES module จึงเปิดจากไฟล์ตรง ๆ ไม่ได้)
รัน:  python 10_FlightSim_Game/tools/serve.py [พอร์ต] [--open]
"""
import http.server, os, sys, functools, webbrowser

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = next((int(a) for a in sys.argv[1:] if a.isdigit()), 8757)


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")      # แก้ไฟล์แล้วเห็นผลทันที
        super().end_headers()

    def log_message(self, *a):
        pass


Handler.extensions_map = {**Handler.extensions_map, ".js": "text/javascript", ".mjs": "text/javascript", ".json": "application/json"}
srv = http.server.ThreadingHTTPServer(("127.0.0.1", PORT), functools.partial(Handler, directory=ROOT))
url = f"http://localhost:{PORT}/"
print("FW57J-B Flight Simulator:", url, "(ปิดหน้าต่างนี้เพื่อหยุด)", flush=True)
if "--open" in sys.argv:
    webbrowser.open(url)
srv.serve_forever()
