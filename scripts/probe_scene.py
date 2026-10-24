"""Development probe: compile the scene, render one RGB-D frame and check basic geometry."""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from armstrong import scene as sc  # noqa: E402
from armstrong.camera import RGBDCamera, project  # noqa: E402
from armstrong.kinematics import Kinematics  # noqa: E402
from armstrong.robot import HOME_Q  # noqa: E402

t0 = time.perf_counter()
cfg = sc.SceneConfig()
m, ids = sc.build_model(cfg)
d = mujoco.MjData(m)
sc.set_initial_state(m, d, ids, cfg)
print(f"compiled in {time.perf_counter() - t0:.2f}s nq={m.nq} nv={m.nv} nu={m.nu} ngeom={m.ngeom} nmesh={m.nmesh}")
print("mesh verts", m.nmeshvert, "faces", m.nmeshface)
kin = Kinematics(m, ids)
kin.load(d.qpos)
p, R = kin.fk(HOME_Q)
print("home ee", p.round(4))
print("home R\n", R.round(3))
print("contacts at home:", d.ncon, [(m.geom(c.geom1).name, m.geom(c.geom2).name, round(c.dist, 4)) for c in d.contact[:d.ncon]])
cam = RGBDCamera(m, cfg.camera, ids.camera)
obs = cam.capture(d, 1)
print("K", obs.K.round(2).tolist())
top = np.array([cfg.block.x, cfg.block.y, sc.TABLE_TOP_Z + cfg.block.size[2]])
uv, z = project(obs.T_world_camera, obs.K, top)
print("block top center projects to", uv.round(2), "depth", round(z, 4))
u, v = int(round(uv[0])), int(round(uv[1]))
print("rendered depth there", obs.depth_m[v, u], "rgb", obs.rgb[v, u])
out = ROOT / "runtime"
out.mkdir(exist_ok=True)
Image.fromarray(obs.rgb).save(out / "probe_rgb.png")
dm = obs.depth_m
vis = np.clip((dm - 0.5) / 1.2, 0, 1)
Image.fromarray((255 * (1 - vis) * (dm > 0)).astype(np.uint8)).save(out / "probe_depth.png")
# Let physics run 1 s with the arm torque-free to check the block rests stably.
for _ in range(500):
    mujoco.mj_step(m, d)
a = ids.block_qadr
print("block after 1 s passive:", d.qpos[a:a + 3].round(4))
print("ok")
