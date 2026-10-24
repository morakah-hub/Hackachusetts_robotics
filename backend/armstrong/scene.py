"""Workcell scene: floor, table, Panda on a fixed mount, a small collection of parts, named destinations and
one calibrated RGB-D camera.

Conventions: meters, radians, right-handed world with +Z up. Quaternions are [w, x, y, z] in the backend.

Parts have stable ids and one of three roles:
* movable  - free body (box collision proxy + optional uploaded visual mesh), moved only by contact;
* fixture  - static known obstacle (box collision proxy), labelled "known geometry" in plans;
* visual   - visual reference only (no collision, no mass).
Destinations are visual-only pads (mocap bodies) with a name, an optional requested yaw and an optional zone.

MuJoCo names: body part_<id>, collision geom part_<id>_col, mesh geom part_<id>_vis, joint part_<id>_free,
destination body dest_<id>. All addresses are resolved by name after compilation (free bodies change the
qpos/qvel layout). Structural changes recompile the model; pose edits while idle write qpos / mocap only.
"""

from __future__ import annotations

import math
import re
import warnings
from dataclasses import asdict, dataclass, field

import mujoco
import numpy as np

from . import assets as am
from . import robot as rb

TABLE_TOP_Z = 0.40
TABLE_CENTER_XY = (0.35, 0.0)
TABLE_HALF_XY = (0.55, 0.50)
TABLE_THICKNESS = 0.04
MOUNT_POS = (0.0, 0.0, TABLE_TOP_Z)
# Validated tabletop workspace for parts and destinations (radius from the robot base axis).
REACH_MIN, REACH_MAX = 0.30, 0.72
WORKSPACE_Y = 0.42
BASE_KEEPOUT = 0.16
MAX_PARTS = 6
MAX_MOVABLE = 4
MAX_DESTINATIONS = 8

PALETTE = {"orange": (0.95, 0.42, 0.06), "blue": (0.12, 0.32, 0.92), "green": (0.20, 0.70, 0.22),
           "magenta": (0.80, 0.15, 0.65)}
NEUTRAL = {"fixture": (0.36, 0.39, 0.43), "visual": (0.74, 0.70, 0.62)}
DEST_RGB = (0.10, 0.62, 0.60)
ROLES = ("movable", "fixture", "visual")
ID_RE = re.compile(r"^[a-z][a-z0-9_]{0,23}$")


class SceneError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code, self.message = code, message


@dataclass
class PartSpec:
    id: str
    name: str
    role: str = "movable"
    asset_id: str | None = None          # uploaded canonical mesh (visual shape); None = primitive box
    size: list = field(default_factory=lambda: [0.05, 0.05, 0.06])   # full proxy extents (m), part frame
    mass: float = 0.4
    friction: float = 1.0
    material: str | None = None
    color: str = "orange"                 # palette key (movable parts); fixtures/visuals use neutral colors
    category: str = ""
    x: float = 0.5
    y: float = 0.2
    yaw: float = 0.0

    @property
    def z(self) -> float:
        return TABLE_TOP_Z + self.size[2] / 2

    def rgb(self) -> tuple:
        return PALETTE.get(self.color, PALETTE["orange"]) if self.role == "movable" else NEUTRAL[self.role]


@dataclass
class DestinationSpec:
    id: str
    name: str
    x: float
    y: float
    yaw: float | None = None              # requested final part yaw; None = keep the part's yaw
    zone: str = ""
    size: list = field(default_factory=lambda: [0.07, 0.07])


@dataclass
class CameraSpec:
    id: str = "rgbd"
    position: tuple = (1.18, 0.0, 1.02)
    target: tuple = (0.50, 0.0, 0.40)
    vertical_fov_deg: float = 45.0
    width: int = 320
    height: int = 240
    rate_hz: float = 5.0


def default_parts() -> list[PartSpec]:
    return [PartSpec(id="block", name="Orange block", size=[0.05, 0.05, 0.06], mass=0.4, friction=1.0,
                     color="orange", x=0.50, y=0.20)]


def default_destinations() -> list[DestinationSpec]:
    return [DestinationSpec(id="pad", name="Destination pad", x=0.50, y=-0.20, size=[0.08, 0.08])]


@dataclass
class SceneConfig:
    name: str = "Single block (original demo)"
    parts: list = field(default_factory=default_parts)
    destinations: list = field(default_factory=default_destinations)
    camera: CameraSpec = field(default_factory=CameraSpec)

    def part(self, pid: str) -> PartSpec | None:
        return next((p for p in self.parts if p.id == pid), None)

    def dest(self, did: str) -> DestinationSpec | None:
        return next((d for d in self.destinations if d.id == did), None)

    def movable(self) -> list[PartSpec]:
        return [p for p in self.parts if p.role == "movable"]

    def to_dict(self) -> dict:
        return {"version": 1, "name": self.name, "parts": [asdict(p) for p in self.parts],
                "destinations": [asdict(d) for d in self.destinations], "camera": asdict(self.camera)}

    @classmethod
    def from_dict(cls, d: dict) -> "SceneConfig":
        if not isinstance(d, dict):
            raise SceneError("INVALID_SCENE", "scene must be an object")
        try:
            parts = [PartSpec(**_pick(p, PartSpec)) for p in d.get("parts", [])]
            dests = [DestinationSpec(**_pick(x, DestinationSpec)) for x in d.get("destinations", [])]
        except TypeError as e:
            raise SceneError("INVALID_SCENE", f"malformed part or destination: {e}") from None
        cfg = cls(name=str(d.get("name", "Untitled"))[:80], parts=parts, destinations=dests)
        validate_config(cfg)
        return cfg


def _pick(d: dict, klass) -> dict:
    if not isinstance(d, dict):
        raise SceneError("INVALID_SCENE", "part/destination entries must be objects")
    names = set(klass.__dataclass_fields__)
    return {k: v for k, v in d.items() if k in names}


def _finite(*vals) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def footprint(size, yaw: float) -> np.ndarray:
    """Corners (4, 2) of a centered rectangle of full size (sx, sy) rotated by yaw."""
    hx, hy = size[0] / 2, size[1] / 2
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
    return pts @ np.array([[c, s], [-s, c]])


def rects_overlap(c1, s1, yaw1, c2, s2, yaw2, margin: float = 0.0) -> bool:
    """Separating-axis test for two rotated rectangles (centers c, full sizes s), with a clearance margin."""
    a = footprint([s1[0] + 2 * margin, s1[1] + 2 * margin], yaw1) + np.asarray(c1)[:2]
    b = footprint(s2, yaw2) + np.asarray(c2)[:2]
    for poly in (a, b):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def movable_issues(p: PartSpec) -> list[str]:
    """Reasons a part cannot be handled by the validated top-down parallel-jaw skill (empty = eligible)."""
    out = []
    short, long_ = sorted(p.size[:2])
    if not (am.GRASP_WIDTH_MIN <= short <= am.GRASP_WIDTH_MAX):
        out.append(f"grasp width (short side) {1e3 * short:.0f} mm is outside {1e3 * am.GRASP_WIDTH_MIN:.0f}-"
                   f"{1e3 * am.GRASP_WIDTH_MAX:.0f} mm")
    if long_ > am.LONG_SIDE_MAX:
        out.append(f"long side {1e3 * long_:.0f} mm exceeds {1e3 * am.LONG_SIDE_MAX:.0f} mm")
    if not (am.HEIGHT_MIN <= p.size[2] <= am.HEIGHT_MAX):
        out.append(f"height {1e3 * p.size[2]:.0f} mm is outside {1e3 * am.HEIGHT_MIN:.0f}-"
                   f"{1e3 * am.HEIGHT_MAX:.0f} mm")
    if not (0.02 <= p.mass <= am.MASS_MAX):
        out.append(f"mass {p.mass:.3f} kg is outside 0.020-{am.MASS_MAX:.1f} kg")
    if not (0.3 <= p.friction <= 1.5):
        out.append(f"friction {p.friction:.2f} is outside 0.3-1.5")
    if p.asset_id:
        try:
            meta = am.asset_meta(p.asset_id)
            out += meta["eligible"]["movable"]["reasons"]
        except am.AssetError as e:
            out.append(str(e))
    return out


def validate_config(cfg: SceneConfig) -> None:
    ids = set()
    if len(cfg.parts) > MAX_PARTS:
        raise SceneError("INVALID_SCENE", f"at most {MAX_PARTS} parts are supported")
    if len(cfg.movable()) > MAX_MOVABLE:
        raise SceneError("INVALID_SCENE", f"at most {MAX_MOVABLE} movable parts are supported")
    if len(cfg.destinations) > MAX_DESTINATIONS:
        raise SceneError("INVALID_SCENE", f"at most {MAX_DESTINATIONS} destinations are supported")
    colors = {}
    for p in cfg.parts:
        if not isinstance(p.id, str) or not ID_RE.match(p.id) or p.id in ids:
            raise SceneError("INVALID_SCENE", f"invalid or duplicate part id {p.id!r}")
        ids.add(p.id)
        p.name = (str(p.name).strip() or p.id)[:40]
        p.category = str(p.category or "").strip()[:24]
        if p.role not in ROLES:
            raise SceneError("INVALID_SCENE", f"part {p.name}: role must be one of {', '.join(ROLES)}")
        if not (isinstance(p.size, (list, tuple)) and len(p.size) == 3 and _finite(*p.size)):
            raise SceneError("INVALID_SCENE", f"part {p.name}: size must be three finite numbers")
        p.size = [float(s) for s in p.size]
        if min(p.size) < 0.005 or max(p.size) > am.FIXTURE_MAX_SIDE:
            raise SceneError("INVALID_SCENE", f"part {p.name}: dimensions must be 5-{1e3 * am.FIXTURE_MAX_SIDE:.0f} mm")
        if not _finite(p.x, p.y, p.yaw, p.mass, p.friction):
            raise SceneError("INVALID_SCENE", f"part {p.name}: pose, mass and friction must be finite numbers")
        if p.asset_id is not None:
            am.asset_path(p.asset_id)   # raises AssetError for invalid / missing ids
        if p.role == "movable":
            if p.color not in PALETTE:
                raise SceneError("INVALID_SCENE", f"part {p.name}: color must be one of {', '.join(PALETTE)}")
            if p.color in colors:
                raise SceneError("INVALID_SCENE", f"{p.name} and {colors[p.color]} are both {p.color}: the camera "
                                                  f"identifies movable parts by color, so each needs its own color")
            colors[p.color] = p.name
            if p.mass <= 0:
                raise SceneError("INVALID_SCENE", f"part {p.name}: mass must be positive")
    for d in cfg.destinations:
        if not isinstance(d.id, str) or not ID_RE.match(d.id) or d.id in ids:
            raise SceneError("INVALID_SCENE", f"invalid or duplicate destination id {d.id!r}")
        ids.add(d.id)
        d.name = (str(d.name).strip() or d.id)[:40]
        d.zone = str(d.zone or "").strip()[:24]
        if not _finite(d.x, d.y) or (d.yaw is not None and not _finite(d.yaw)):
            raise SceneError("INVALID_SCENE", f"destination {d.name}: pose must be finite numbers")
        if not (isinstance(d.size, (list, tuple)) and len(d.size) == 2 and _finite(*d.size)
                and 0.03 <= min(d.size) and max(d.size) <= 0.2):
            raise SceneError("INVALID_SCENE", f"destination {d.name}: pad size must be 30-200 mm")
        d.size = [float(s) for s in d.size]


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


def _f(v) -> str:
    return " ".join(f"{float(x):.6g}" for x in v)


def _env_spec(cfg: SceneConfig) -> mujoco.MjSpec:
    c = cfg.camera
    tx, ty = TABLE_CENTER_XY
    hx, hy = TABLE_HALF_XY
    tz = TABLE_TOP_Z - TABLE_THICKNESS / 2
    leg_h = (TABLE_TOP_Z - TABLE_THICKNESS) / 2
    legs = "".join(
        f'<geom name="table_leg{i}" type="box" size="0.025 0.025 {leg_h:.4f}" '
        f'pos="{sx * (hx - 0.05):.3f} {sy * (hy - 0.05):.3f} {leg_h:.4f}" material="table_leg"/>'
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)), start=1))
    mats, bodies = [], []
    for p in cfg.parts:
        mats.append(f'<material name="mat_{p.id}" rgba="{_f(p.rgb())} 1" specular="0.3"/>')
        half = [s / 2 for s in p.size]
        has_mesh = p.asset_id is not None
        vis = (f'<geom name="part_{p.id}_vis" type="mesh" mesh="mesh_{p.asset_id}" contype="0" conaffinity="0" '
               f'group="2" material="mat_{p.id}"/>' if has_mesh else "")
        col_vis = 'group="3" rgba="0.5 0.5 0.5 0"' if has_mesh else f'group="0" material="mat_{p.id}"'
        pose = f'pos="{p.x:.6g} {p.y:.6g} {p.z:.6g}" quat="{_f(yaw_quat(p.yaw))}"'
        if p.role == "movable":
            m = p.mass
            ixx = m / 12 * (p.size[1] ** 2 + p.size[2] ** 2)
            iyy = m / 12 * (p.size[0] ** 2 + p.size[2] ** 2)
            izz = m / 12 * (p.size[0] ** 2 + p.size[1] ** 2)
            bodies.append(
                f'<body name="part_{p.id}" {pose}><freejoint name="part_{p.id}_free"/>'
                f'<inertial pos="0 0 0" mass="{m:.6g}" diaginertia="{ixx:.6g} {iyy:.6g} {izz:.6g}"/>'
                f'<geom name="part_{p.id}_col" type="box" size="{_f(half)}" friction="{p.friction:.4g} 0.005 0.0001" '
                f'condim="4" {col_vis}/>{vis}</body>')
        elif p.role == "fixture":
            bodies.append(f'<body name="part_{p.id}" mocap="true" {pose}>'
                          f'<geom name="part_{p.id}_col" type="box" size="{_f(half)}" friction="{p.friction:.4g} 0.005 0.0001" '
                          f'{col_vis}/>{vis}</body>')
        else:
            inner = vis or (f'<geom name="part_{p.id}_vis" type="box" size="{_f(half)}" contype="0" conaffinity="0" '
                            f'group="2" material="mat_{p.id}"/>')
            bodies.append(f'<body name="part_{p.id}" mocap="true" {pose}>{inner}</body>')
    for d in cfg.destinations:
        bodies.append(
            f'<body name="dest_{d.id}" mocap="true" pos="{d.x:.6g} {d.y:.6g} {TABLE_TOP_Z}" quat="{_f(yaw_quat(d.yaw or 0.0))}">'
            f'<geom name="dest_{d.id}_geom" type="box" size="{d.size[0] / 2:.4g} {d.size[1] / 2:.4g} 0.0006" '
            f'pos="0 0 0.0006" material="dest" contype="0" conaffinity="0"/></body>')
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
    <material name="dest" rgba="{_f(DEST_RGB)} 1"/>
    {''.join(mats)}
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
    {''.join(bodies)}
    <camera name="{c.id}" pos="{_f(c.position)}" fovy="{c.vertical_fov_deg}"/>
  </worldbody>
</mujoco>
"""
    spec = mujoco.MjSpec.from_string(xml)
    spec.camera(c.id).quat = look_at_quat(c.position, c.target)
    for aid in sorted({p.asset_id for p in cfg.parts if p.asset_id}):
        verts, faces = am.mesh_from_triangles(am.load_triangles(aid))
        mesh = spec.add_mesh()
        mesh.name = f"mesh_{aid}"
        mesh.uservert = verts.ravel().tolist()
        mesh.userface = faces.ravel().tolist()
        mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL   # visual only; body inertia is explicit
    return spec


@dataclass
class PartIds:
    body: int
    geom: int      # collision geom, -1 for visual references
    qadr: int      # free-joint qpos address, -1 if not movable
    dadr: int
    mocap: int     # mocap index, -1 for movable parts


@dataclass
class SceneIds:
    arm_qadr: np.ndarray
    arm_dadr: np.ndarray
    arm_act: np.ndarray
    finger_qadr: np.ndarray
    grip_act: int
    ee_site: int
    hand: int
    camera: int
    table_body: int
    floor_geom: int
    robot_bodies: np.ndarray
    finger_bodies: np.ndarray
    parts: dict
    dests: dict


def build_model(cfg: SceneConfig) -> tuple[mujoco.MjModel, SceneIds]:
    validate_config(cfg)
    env = _env_spec(cfg)
    frame = env.worldbody.add_frame(pos=list(MOUNT_POS))
    panda = rb.load_panda_spec()
    frame.attach_body(panda.body(rb.BASE_BODY), "", "")
    env.add_exclude(bodyname1=rb.BASE_BODY, bodyname2="table")
    with warnings.catch_warnings():   # empty attach prefix: names stay as in the Menagerie model
        warnings.simplefilter("ignore")
        model = env.compile()
    return model, scene_ids(model, cfg)


def scene_ids(m: mujoco.MjModel, cfg: SceneConfig) -> SceneIds:
    def jq(name):
        return int(m.jnt_qposadr[m.joint(name).id])

    def jd(name):
        return int(m.jnt_dofadr[m.joint(name).id])

    base = m.body(rb.BASE_BODY).id
    robot = []
    for b in range(m.nbody):
        a = b
        while a not in (0, base):
            a = m.body_parentid[a]
        if a == base:
            robot.append(b)
    parts = {}
    for p in cfg.parts:
        b = m.body(f"part_{p.id}").id
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"part_{p.id}_col")
        if p.role == "movable":
            j = m.body_jntadr[b]
            parts[p.id] = PartIds(b, g, int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]), -1)
        else:
            parts[p.id] = PartIds(b, g, -1, -1, int(m.body_mocapid[b]))
    dests = {d.id: int(m.body_mocapid[m.body(f"dest_{d.id}").id]) for d in cfg.destinations}
    return SceneIds(
        arm_qadr=np.array([jq(j) for j in rb.ARM_JOINTS]),
        arm_dadr=np.array([jd(j) for j in rb.ARM_JOINTS]),
        arm_act=np.array([m.actuator(a).id for a in rb.ARM_ACTUATORS]),
        finger_qadr=np.array([jq(j) for j in rb.FINGER_JOINTS]),
        grip_act=m.actuator(rb.GRIPPER_ACTUATOR).id,
        ee_site=m.site(rb.EE_SITE).id,
        hand=m.body(rb.HAND_BODY).id,
        camera=m.camera(0).id,
        table_body=m.body("table").id,
        floor_geom=m.geom("floor").id,
        robot_bodies=np.array(robot),
        finger_bodies=np.array([m.body(n).id for n in rb.FINGER_BODIES]),
        parts=parts, dests=dests,
    )


def set_initial_state(m: mujoco.MjModel, d: mujoco.MjData, ids: SceneIds, cfg: SceneConfig,
                      arm_q: np.ndarray | None = None) -> None:
    """Initialization / reset: arm at home (or arm_q), gripper open, parts and destinations at configured poses."""
    mujoco.mj_resetData(m, d)
    d.qpos[ids.arm_qadr] = rb.HOME_Q if arm_q is None else arm_q
    d.qpos[ids.finger_qadr] = 0.04
    d.ctrl[ids.grip_act] = rb.GRIPPER_OPEN
    for p in cfg.parts:
        place_part(d, ids, p)
    for dst in cfg.destinations:
        place_dest(d, ids, dst)
    mujoco.mj_forward(m, d)


def place_part(d: mujoco.MjData, ids: SceneIds, p: PartSpec) -> None:
    pi = ids.parts[p.id]
    if pi.qadr >= 0:
        a = pi.qadr
        d.qpos[a:a + 3] = [p.x, p.y, p.z]
        d.qpos[a + 3:a + 7] = yaw_quat(p.yaw)
        d.qvel[pi.dadr:pi.dadr + 6] = 0.0
    else:
        d.mocap_pos[pi.mocap] = [p.x, p.y, p.z]
        d.mocap_quat[pi.mocap] = yaw_quat(p.yaw)


def place_dest(d: mujoco.MjData, ids: SceneIds, dst: DestinationSpec) -> None:
    k = ids.dests[dst.id]
    d.mocap_pos[k] = [dst.x, dst.y, TABLE_TOP_Z]
    d.mocap_quat[k] = yaw_quat(dst.yaw or 0.0)


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
