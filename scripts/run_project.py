"""Headless run of an example or user project (same worker code path as the app, stepped as fast as possible).

    .\\.venv\\Scripts\\python.exe scripts\\run_project.py two_part_kit [camera_rgbd|known_world] [example|user]
"""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend"))

from armstrong import projects as pj  # noqa: E402
from armstrong.sim import SimWorker  # noqa: E402

pid = sys.argv[1] if len(sys.argv) > 1 else "two_part_kit"
mode = sys.argv[2] if len(sys.argv) > 2 else "camera_rgbd"
source = sys.argv[3] if len(sys.argv) > 3 else "example"
w = SimWorker(realtime=False)
w.setup_sync()
w.command_sync("load_project", project=pj.load(source, pid))
print("recipe status", w.recipe_status)
t0 = time.perf_counter()
res = w.run_sync({"perception_mode": mode, "scene_revision": w.scene_revision})
print(f"wall {time.perf_counter() - t0:.1f}s  sim {res['sim_duration_s']:.1f}s")
print(json.dumps({k: res[k] for k in ("outcome", "failure_stage", "failure_code", "reason", "failed_step")}, indent=1))
for s in res["steps"]:
    m = s.get("metrics") or {}
    e = s.get("estimate") or {}
    pl = s.get("plan") or {}

    def mm(v):
        return None if v is None else round(1e3 * v, 2)
    print(f" step {s['index']} {s['part_id']} -> {s.get('dest_name')}: {s['outcome']} | {s.get('reason')}")
    print(f"    est_err {mm(m.get('estimate_error_xy_m'))} mm, place_err {mm(m.get('placement_error_xy_m'))} mm, "
          f"lift {mm(m.get('max_lift_m'))} mm, slip {mm(m.get('object_slip_m'))} mm, yaw_err {m.get('yaw_error_deg')}, "
          f"grasp_w {mm(m.get('grasp_width_m'))} mm, sim {s.get('sim_duration_s', 0):.1f}s")
    if pl:
        print(f"    plan: part yaw {pl.get('part_yaw_rad'):.3f} grasp yaw {pl.get('grasp_yaw_rad'):.3f} place yaw "
              f"{pl.get('place_yaw_rad'):.3f} final {pl.get('final_part_yaw_rad'):.3f} width {pl.get('grasp_width_m')}")
    if e:
        print(f"    estimate valid {e.get('valid')} pos {e.get('position_m')} yaw {e.get('yaw_rad')} ext {e.get('top_extent_m')}")
print("metrics", {k: res["metrics"].get(k) for k in ("steps_total", "steps_completed", "forbidden_contacts",
                                                      "first_forbidden_contact")})
for ev in list(w.events)[-6:]:
    print("  ", ev["msg"])
