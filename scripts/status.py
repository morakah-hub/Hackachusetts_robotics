"""Print a short summary of the running server's state: .venv\\Scripts\\python.exe scripts\\status.py"""
import sys

import httpx

base = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8765"
s = httpx.get(f"{base}/api/state", timeout=5).json()
print("status", s.get("status"), "phase", s.get("phase"), "rev", s.get("scene_revision"), "sim", round(s.get("sim_time_s", 0), 2),
      "rtf", s.get("realtime_factor"))
r = s.get("result")
if r:
    print("result", r["outcome"], r["failure_stage"], r["reason"])
for e in s.get("events", [])[-10:]:
    print(" ", e["t_sim"], e["level"], e["msg"])
