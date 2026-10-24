"""Workcell scene: floor, table, Panda on a fixed mount, one rigid block, a destination pad
and one calibrated RGB-D camera.

Conventions: meters, radians, right-handed world with +Z up. Quaternions are stored
[w, x, y, z] (MuJoCo order) everywhere in the backend; the browser converts explicitly.

The model is compiled once. Stopped-scene edits move the block through its free-joint qpos
(state edit while idle, allowed by the spec) and the pad through its mocap pose (visual-only,
non-colliding). During a run nothing writes the block state: it moves only through contact.
"""

from __future__ import annotations

import warnings
from dataclasses import asdict, dataclass, field

import mujoco
import numpy as np

from . import robot as rb

TABLE_TOP_Z = 0.40
TABLE_CENTER_XY = (0.35, 0.0)
TABLE_HALF_XY = (0.55, 0.50)
TABLE_THICKNESS = 0.04
MOUNT_POS = (0.0, 0.0, TABLE_TOP_Z)
# Validated tabletop workspace for the block and pad (radius from the robot base axis).
REACH_MIN, REACH_MAX = 0.30, 0.72
WORKSPACE_Y = 0.42


@dataclass
class BlockSpec:
    x: float = 0.50
    y: float = 0.20
    yaw: float = 0.0
    size: tuple[float, float, float] = (0.05, 0.05, 0.06)   # full extents (m)
    mass: float = 0.4
    friction: tuple[float, float, float] = (1.0, 0.005, 0.0001)
    color: tuple[float, float, float] = (0.95, 0.42, 0.06)  # saturated orange, distinct from scene


@dataclass
class PadSpec:
    x: float = 0.50
    y: float = -0.20
    size: tuple[float, float] = (0.08, 0.08)
    color: tuple[float, float, float] = (0.10, 0.62, 0.60)


@dataclass
class CameraSpec:
    id: str = "rgbd"
    position: tuple[float, float, float] = (1.18, 0.0, 1.02)
    target: tuple[float, float, float] = (0.50, 0.0, 0.40)
    vertical_fov_deg: float = 45.0
    width: int = 320
    height: int = 240
    rate_hz: float = 5.0


@dataclass
class SceneConfig:
    block: BlockSpec = field(default_factory=BlockSpec)
    pad: PadSpec = field(default_factory=PadSpec)
    camera: CameraSpec = field(default_factory=CameraSpec)

    def to_dict(self) -> dict:
        return asdict(self)


def look_at_quat(position, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """MuJoCo camera quaternion [w,x,y,z]: camera looks along its -Z, +Y is image-up."""
    p, t = np.asarray(position, float), np.asarray(target, float)
    f = t - p
    f /= np.linalg.norm(f)
    x = np.cross(f, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    R = np.column_stack([x, y, -f])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.ravel())
    return q


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def _env_spec(cfg: SceneConfig) -> mujoco.MjSpec:
    b, p, c = cfg.block, cfg.pad, cfg.camera
    tx, ty = TABLE_CENTER_XY
    hx, hy = TABLE_HALF_XY
    tz = TABLE_TOP_Z - TABLE_THICKNESS / 2
    leg_h = (TABLE_TOP_Z - TABLE_THICKNESS) / 2
    legs = "".join(
        f'<geom name="table_leg{i}" type="box" size="0.025 0.025 {leg_h:.4f}" '
        f'pos="{sx * (hx - 0.05):.3f} {sy * (hy - 0.05):.3f} {leg_h:.4f}" material="table_leg"/>'
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)), start=1))
    xml = f"""
<mujoco model="armstrong_workcell">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <statistic center="0.4 0 0.5" extent="1.0"/>
  <visual>
    <global offwidth="640" offheight="480"/>
    <map znear="0.01" zfar="20"/>
    <quality shadowsize="2048"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.05 0.05 0.05"/>
  </visual>
  <asset>
    <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.16 0.16 0.17" rgb2="0.19 0.19 0.20"
      width="256" height="256"/>
    <material name="floor" texture="floor_tex" texrepeat="8 8" reflectance="0.05"/>
    <material name="table_top" rgba="0.46 0.44 0.41 1" specular="0.2"/>
    <material name="table_leg" rgba="0.22 0.22 0.23 1"/>
    <material name="block" rgba="{b.color[0]} {b.color[1]} {b.color[2]} 1" specular="0.3"/>
    <material name="pad" rgba="{p.color[0]} {p.color[1]} {p.color[2]} 1"/>
    <material name="mount" rgba="0.20 0.21 0.22 1"/>
  </asset>
  <worldbody>
    <light name="key" pos="1.2 -0.8 2.4" dir="-0.45 0.3 -1" directional="true" diffuse="0.55 0.55 0.55"
      castshadow="true"/>
    <light name="fill" pos="-0.5 1.2 2.0" dir="0.3 -0.5 -1" directional="true" diffuse="0.2 0.2 0.22"
      castshadow="false"/>
    <geom name="floor" type="plane" size="4 4 0.05" material="floor"/>
    <body name="table" pos="{tx} {ty} 0">
      <geom name="table_top" type="box" size="{hx} {hy} {TABLE_THICKNESS / 2}" pos="0 0 {tz}"
        material="table_top"/>
      {legs}
    </body>
    <body name="block" pos="{b.x} {b.y} {TABLE_TOP_Z + b.size[2] / 2}">
      <freejoint name="block_free"/>
      <geom name="block_geom" type="box" size="{b.size[0] / 2} {b.size[1] / 2} {b.size[2] / 2}"
        mass="{b.mass}" friction="{b.friction[0]} {b.friction[1]} {b.friction[2]}" condim="4"
        material="block"/>
    </body>
    <body name="pad" mocap="true" pos="{p.x} {p.y} {TABLE_TOP_Z}">
      <geom name="pad_geom" type="box" size="{p.size[0] / 2} {p.size[1] / 2} 0.0006" pos="0 0 0.0006"
        material="pad" contype="0" conaffinity="0"/>
    </body>
    <camera name="{c.id}" pos="{c.position[0]} {c.position[1]} {c.position[2]}" fovy="{c.vertical_fov_deg}"/>
  </worldbody>
</mujoco>
"""
    spec = mujoco.MjSpec.from_string(xml)
    spec.camera(c.id).quat = look_at_quat(c.position, c.target)
    return spec


@dataclass
class SceneIds:
    arm_qadr: np.ndarray
    arm_dadr: np.ndarray
    arm_act: np.ndarray
    finger_qadr: np.ndarray
    grip_act: int
    ee_site: int
    hand: int
    block_body: int
    block_geom: int
    block_qadr: int
    block_dadr: int
    pad_body: int
    pad_mocap: int
    camera: int
    table_body: int
    floor_geom: int
    robot_bodies: np.ndarray
    finger_bodies: np.ndarray


def build_model(cfg: SceneConfig) -> tuple[mujoco.MjModel, SceneIds]:
    env = _env_spec(cfg)
    frame = env.worldbody.add_frame(pos=list(MOUNT_POS))
    panda = rb.load_panda_spec()
    frame.attach_body(panda.body(rb.BASE_BODY), "", "")
    env.add_exclude(bodyname1=rb.BASE_BODY, bodyname2="table")
    with warnings.catch_warnings():   # empty attach prefix: names stay as in the Menagerie model
        warnings.simplefilter("ignore")
        model = env.compile()
    return model, scene_ids(model)


def scene_ids(m: mujoco.MjModel) -> SceneIds:
    def jq(name):
        return m.jnt_qposadr[m.joint(name).id]

    def jd(name):
        return m.jnt_dofadr[m.joint(name).id]

    base = m.body(rb.BASE_BODY).id
    robot = []
    for b in range(m.nbody):
        a = b
        while a not in (0, base):
            a = m.body_parentid[a]
        if a == base:
            robot.append(b)
    block = m.body("block").id
    pad = m.body("pad").id
    return SceneIds(
        arm_qadr=np.array([jq(j) for j in rb.ARM_JOINTS]),
        arm_dadr=np.array([jd(j) for j in rb.ARM_JOINTS]),
        arm_act=np.array([m.actuator(a).id for a in rb.ARM_ACTUATORS]),
        finger_qadr=np.array([jq(j) for j in rb.FINGER_JOINTS]),
        grip_act=m.actuator(rb.GRIPPER_ACTUATOR).id,
        ee_site=m.site(rb.EE_SITE).id,
        hand=m.body(rb.HAND_BODY).id,
        block_body=block,
        block_geom=m.geom("block_geom").id,
        block_qadr=m.jnt_qposadr[m.body_jntadr[block]],
        block_dadr=m.jnt_dofadr[m.body_jntadr[block]],
        pad_body=pad,
        pad_mocap=m.body_mocapid[pad],
        camera=m.camera(0).id,
        table_body=m.body("table").id,
        floor_geom=m.geom("floor").id,
        robot_bodies=np.array(robot),
        finger_bodies=np.array([m.body(n).id for n in rb.FINGER_BODIES]),
    )


def set_initial_state(m: mujoco.MjModel, d: mujoco.MjData, ids: SceneIds, cfg: SceneConfig,
                      arm_q: np.ndarray | None = None) -> None:
    """Initialization / stopped-scene edit: arm at home (or arm_q), gripper open, block and pad placed."""
    mujoco.mj_resetData(m, d)
    d.qpos[ids.arm_qadr] = rb.HOME_Q if arm_q is None else arm_q
    d.qpos[ids.finger_qadr] = 0.04
    d.ctrl[ids.grip_act] = rb.GRIPPER_OPEN
    place_block(m, d, ids, cfg.block)
    place_pad(d, ids, cfg.pad)
    mujoco.mj_forward(m, d)


def place_block(m: mujoco.MjModel, d: mujoco.MjData, ids: SceneIds, b: BlockSpec) -> None:
    a = ids.block_qadr
    d.qpos[a:a + 3] = [b.x, b.y, TABLE_TOP_Z + b.size[2] / 2]
    d.qpos[a + 3:a + 7] = yaw_quat(b.yaw)
    d.qvel[ids.block_dadr:ids.block_dadr + 6] = 0.0


def place_pad(d: mujoco.MjData, ids: SceneIds, p: PadSpec) -> None:
    d.mocap_pos[ids.pad_mocap] = [p.x, p.y, TABLE_TOP_Z]
    d.mocap_quat[ids.pad_mocap] = [1.0, 0.0, 0.0, 0.0]


def workspace_check(x: float, y: float, what: str) -> str | None:
    """None if (x, y) is inside the validated tabletop workspace, else a human-readable reason."""
    r = float(np.hypot(x, y))
    if r < REACH_MIN:
        return f"{what} at ({x:.3f}, {y:.3f}) m is {r:.2f} m from the robot base; minimum is {REACH_MIN:.2f} m"
    if r > REACH_MAX:
        return f"{what} at ({x:.3f}, {y:.3f}) m is {r:.2f} m from the robot base; maximum validated reach is {REACH_MAX:.2f} m"
    if abs(y) > WORKSPACE_Y:
        return f"{what} y = {y:.3f} m is outside the validated band |y| <= {WORKSPACE_Y:.2f} m"
    return None


def on_table(x: float, y: float, half: float) -> bool:
    tx, ty = TABLE_CENTER_XY
    hx, hy = TABLE_HALF_XY
    return (tx - hx + half <= x <= tx + hx - half) and (ty - hy + half <= y <= ty + hy - half)
