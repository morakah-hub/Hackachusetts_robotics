"""Camera estimate accuracy over a grid of poses for a square block and a 40 x 60 mm part (truth = evaluation only)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend"))

import numpy as np  # noqa: E402

from armstrong import projects as pj  # noqa: E402
from armstrong.perception import SceneObserver, wrap  # noqa: E402
from armstrong.sim import SimWorker  # noqa: E402
from armstrong.task import catalog_entry, table_info  # noqa: E402

w = SimWorker(realtime=False)
w.setup_sync()
w.command_sync("load_project", project=pj.load("example", "two_part_kit"))
for pid, other_pose in (("block", {"x": 0.62, "y": -0.25}), ("spacer", {"x": 0.40, "y": -0.30})):
    other = "spacer" if pid == "block" else "block"
    w.command_sync("edit", parts={other: other_pose})
    errs, yerrs = [], []
    part = w.cfg.part(pid)
    for x in (0.38, 0.50, 0.62):
        for y in (-0.05, 0.10, 0.25):
            for yaw in (0.0, 0.4, -1.1):
                w.command_sync("edit", parts={pid: {"x": x, "y": y, "yaw": yaw}})
                e = SceneObserver(w.observe(), table_info()).estimate(catalog_entry(part), w.scene_revision)
                if not e["valid"]:
                    print(pid, x, y, yaw, "INVALID", e["reasons"])
                    continue
                t = w.data.xpos[w.ids.parts[pid].body]
                errs.append(np.asarray(e["position_m"]) - t)
                yerrs.append(abs(np.degrees(wrap(e["yaw_rad"] - yaw, e["yaw_period_rad"]))))
    errs = np.array(errs)
    print(f"{pid}: {len(errs)} valid poses, max |xy| {1e3 * np.linalg.norm(errs[:, :2], axis=1).max():.2f} mm, "
          f"mean xy {np.round(1e3 * errs[:, :2].mean(0), 2)} mm, max |z| {1e3 * np.abs(errs[:, 2]).max():.2f} mm, "
          f"max yaw err {max(yerrs):.2f} deg")
    w.command_sync("reset")
