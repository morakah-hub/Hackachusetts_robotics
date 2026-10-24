"""Smoke test against a running server: health, visuals, camera, edit, run to completion.

    .\\.venv\\Scripts\\python.exe scripts\\smoke_api.py [base_url]
"""
import json
import sys
import time

import httpx

base = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8765"
c = httpx.Client(base_url=base, timeout=20)
print("health", c.get("/api/health").json())
vis = c.get("/api/scene/visuals").json()
blob = c.get("/api/scene/meshes.bin").content
print("visual geoms", len(vis["geoms"]), "mesh bytes", len(blob), "declared", vis["mesh_bytes"])
r = c.get("/api/camera/rgbd/rgb")
print("rgb", r.status_code, r.headers.get("content-type"), len(r.content), "frame", r.headers.get("x-frame-id"))
html = c.get("/")
print("index", html.status_code, "Armstrong" in html.text)
s = c.get("/api/state").json()
rev = s["scene_revision"]
print("status", s["status"], "rev", rev)
e = c.post("/api/scene/edit", json={"block": {"x": 0.45, "y": 0.15}, "expected_revision": rev}).json()
print("edit", e)
stale = c.post("/api/run", json={"perception_mode": "camera_rgbd", "scene_revision": rev})
print("stale run ->", stale.status_code, stale.json()["error"]["code"])
r = c.post("/api/run", json={"perception_mode": "camera_rgbd", "scene_revision": e["scene_revision"]}).json()
print("run", r)
t0 = time.time()
while time.time() - t0 < 60:
    s = c.get("/api/state").json()
    if s["status"] in ("completed", "failed", "stopped"):
        break
    time.sleep(0.5)
res = s["result"]
print("final", s["status"], res["outcome"], res["failure_stage"], res["reason"])
print(json.dumps({k: res["metrics"][k] for k in ("placement_error_xy_m", "estimate_error_xy_m", "max_lift_m")}))
print("wall", round(time.time() - t0, 1), "sim", round(res["sim_duration_s"], 1), "rtf", s["realtime_factor"])
