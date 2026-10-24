"""Diagnose camera estimate bias over a grid of block positions (truth used for evaluation only)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend"))

import numpy as np  # noqa: E402

from armstrong import scene as sc  # noqa: E402
from armstrong.camera import backproject  # noqa: E402
from armstrong.perception import estimate_block, orange_mask  # noqa: E402
from armstrong.sim import SimWorker  # noqa: E402

w = SimWorker(realtime=False)
w.setup_sync()
cat = {"id": "block", "color_class": "orange", "size_m": list(w.cfg.block.size), "support_z_m": sc.TABLE_TOP_Z}
errs = []
for x in (0.38, 0.5, 0.62):
    for y in (-0.1, 0.1, 0.25):
        for yaw in (0.0, 0.4):
            w.command_sync("edit", block={"x": x, "y": y, "yaw": yaw})
            obs = w.observe()
            e = estimate_block(obs, cat, w.scene_revision)
            if not e["valid"]:
                print(x, y, yaw, "INVALID", e["reasons"])
                continue
            d = np.asarray(e["position_m"]) - w.data.xpos[w.ids.block_body]
            errs.append(d)
            print(f"{x:.2f} {y:+.2f} yaw {yaw:.1f}: err x {1e3*d[0]:+.1f} y {1e3*d[1]:+.1f} z {1e3*d[2]:+.1f} mm  "
                  f"yaw_est {np.degrees(e['yaw_rad']):.1f} ext {np.round(np.array(e['top_extent_m'])*1e3,1)}")
errs = np.array(errs)
print("mean err mm", np.round(1e3 * errs.mean(0), 2), "max |xy| mm", round(1e3 * np.linalg.norm(errs[:, :2], axis=1).max(), 2))
# raw top-face x-range for the default pose
w.command_sync("edit", block={"x": 0.5, "y": 0.2, "yaw": 0.0})
obs = w.observe()
m = orange_mask(obs.rgb) & (obs.depth_m > 0)
vs, us = np.nonzero(m)
pts = backproject(obs.depth_m, obs.K, obs.T_world_camera, us, vs)
top = pts[pts[:, 2] > sc.TABLE_TOP_Z + 0.06 - 0.003]
print("raw top pts", len(top), "x range", top[:, 0].min().round(4), top[:, 0].max().round(4), "y range",
      top[:, 1].min().round(4), top[:, 1].max().round(4), "z", np.round(np.percentile(top[:, 2], [5, 50, 95]), 4))
