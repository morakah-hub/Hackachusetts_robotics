"""Development probe: confirm the MjSpec user-mesh API and free-body/inertial handling on this MuJoCo."""
import sys
from pathlib import Path

import mujoco
import numpy as np

print("mujoco", mujoco.__version__)
xml = """
<mujoco>
  <worldbody>
    <body name="part_a" pos="0 0 0.5">
      <freejoint name="part_a_free"/>
      <inertial pos="0 0 0" mass="0.3" diaginertia="1e-4 1e-4 1e-4"/>
      <geom name="part_a_box" type="box" size="0.02 0.03 0.025" group="3" rgba="1 0 0 0"/>
      <geom name="part_a_vis" type="mesh" mesh="mesh_a" contype="0" conaffinity="0" group="2"/>
    </body>
    <body name="fix" mocap="true" pos="0.3 0 0.1">
      <geom name="fix_box" type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>"""
spec = mujoco.MjSpec.from_string(xml)
# open box (no bottom) offset from origin to check recentering and shell inertia
v = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0], [-1, -1, 2], [1, -1, 2], [1, 1, 2], [-1, 1, 2]], float)
v = v * [0.02, 0.03, 0.025] + [0.01, 0.0, -0.025]
f = np.array([[4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
mesh = spec.add_mesh()
mesh.name = "mesh_a"
mesh.uservert = v.ravel().tolist()
mesh.userface = f.ravel().tolist()
print("inertia enum", list(mujoco.mjtMeshInertia.__members__))
mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
m = spec.compile()
d = mujoco.MjData(m)
g = m.geom("part_a_vis").id
print("body mass", m.body_mass[m.body("part_a").id], "inertia", m.body_inertia[m.body("part_a").id])
print("vis geom pos", m.geom_pos[g], "quat", m.geom_quat[g])
mid = m.geom_dataid[g]
vv = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
print("compiled mesh verts min/max", vv.min(0).round(4), vv.max(0).round(4))
mujoco.mj_forward(m, d)
R = d.geom_xmat[g].reshape(3, 3)
world = vv @ R.T + d.geom_xpos[g]
print("mesh world bbox", world.min(0).round(4), world.max(0).round(4), "(expect x -0.01..0.03, z 0.475..0.525)")
print("mocap ids", m.body_mocapid[m.body("fix").id], "nmocap", m.nmocap)
print("ok")
