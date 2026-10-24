"""Headless pick-and-place run (same worker code path as the app, stepped as fast as possible).

    .\\.venv\\Scripts\\python.exe scripts\\headless_run.py [camera_rgbd|known_world] [x y]
"""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend"))

from armstrong.sim import SimWorker  # noqa: E402

mode = sys.argv[1] if len(sys.argv) > 1 else "camera_rgbd"
w = SimWorker(realtime=False)
w.setup_sync()
if len(sys.argv) >= 4:
    w.command_sync("edit", block={"x": float(sys.argv[2]), "y": float(sys.argv[3])})
t0 = time.perf_counter()
res = w.run_sync({"perception_mode": mode, "scene_revision": w.scene_revision})
print(f"wall {time.perf_counter() - t0:.1f}s")
keep = {k: res[k] for k in ("outcome", "failure_stage", "failure_code", "reason", "sim_duration_s")}
print(json.dumps(keep, indent=1))
mt = res["metrics"]
print(json.dumps({k: mt[k] for k in ("placement_error_xy_m", "placement_error_z_m", "final_tilt_deg", "max_lift_m",
                                    "object_slip_m", "forbidden_contacts", "first_forbidden_contact", "grasp_width_m",
                                    "estimate_error_xy_m", "torque_saturation_steps")}, indent=1))
if res.get("estimate"):
    e = res["estimate"]
    print("estimate", {k: e.get(k) for k in ("valid", "position_m", "yaw_rad", "pixels", "top_points", "top_extent_m", "reasons")})
for p in res["phases"]:
    print(f"  {p['phase']:10s} {p['t_start']:.2f} -> {p['t_end']}")
for e in list(w.events)[-8:]:
    print("  ", e["msg"])
