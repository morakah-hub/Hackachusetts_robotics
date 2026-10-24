"""Arm kinematics on private scratch data: FK, full-pose IK and collision checks.

Pose IK is damped least squares on the 6 x 7 site Jacobian with a world-frame error
e = [p* - p ; w * log(R* R^T)], a null-space pull toward a rest posture and joint-limit clamping.

Collision checks run on a private *planning world*: arm and finger joints from proprioception, movable parts
at the poses supplied by the caller (camera estimates in camera mode), fixtures at their known poses. Nothing
here writes the live simulation state. Rules (every other contact is ignored, e.g. parts resting on the table):
* robot vs anything outside the robot (table, floor, fixtures, parts) is a collision, except the hand/fingers
  vs the carried part while carrying;
* robot self-contact is a collision;
* the carried part vs anything other than the hand is a collision, except the table while lifting off / lowering.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .scene import SceneIds


def rot_error(R_target: np.ndarray, R: np.ndarray) -> np.ndarray:
    """World-frame rotation vector that rotates R onto R_target."""
    rel = R_target @ R.T
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rel.ravel())
    if q[0] < 0:
        q = -q
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    pos_err: float
    rot_err: float
    reason: str = ""


class Kinematics:
    def __init__(self, model: mujoco.MjModel, ids: SceneIds, limit_margin: float = 0.03):
        self.m = model
        self.ids = ids
        self.d = mujoco.MjData(model)
        lo = model.jnt_range[[model.dof_jntid[a] for a in ids.arm_dadr], 0]
        hi = model.jnt_range[[model.dof_jntid[a] for a in ids.arm_dadr], 1]
        self.q_min, self.q_max = lo + limit_margin, hi - limit_margin
        self.jac_p = np.zeros((3, model.nv))
        self.jac_r = np.zeros((3, model.nv))
        self.robot_body = np.zeros(model.nbody, dtype=bool)
        self.robot_body[ids.robot_bodies] = True
        self.hand_side = np.zeros(model.nbody, dtype=bool)
        self.hand_side[ids.finger_bodies] = True
        self.hand_side[ids.hand] = True
        self.body_part = {pi.body: pid for pid, pi in ids.parts.items()}
        self.table_body = ids.table_body

    # ------------------------------------------------------------------ planning world

    def set_world(self, data: mujoco.MjData, part_poses: dict) -> None:
        """Proprioception (arm/finger joints) + known static geometry (mocap) from `data`; every movable part
        pose from `part_poses` {part_id: (position (3,), yaw)}. Raises KeyError if a movable part is missing."""
        d = self.d
        d.qpos[:] = 0.0
        a = self.ids.arm_qadr
        d.qpos[a] = data.qpos[a]
        f = self.ids.finger_qadr
        d.qpos[f] = data.qpos[f]
        d.mocap_pos[:] = data.mocap_pos
        d.mocap_quat[:] = data.mocap_quat
        for pid, pi in self.ids.parts.items():
            if pi.qadr < 0:
                continue
            pos, yaw = part_poses[pid]
            d.qpos[pi.qadr:pi.qadr + 3] = pos
            d.qpos[pi.qadr + 3:pi.qadr + 7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.d.qpos[self.ids.arm_qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        s = self.ids.ee_site
        return self.d.site_xpos[s].copy(), self.d.site_xmat[s].reshape(3, 3).copy()

    def hand_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.d.qpos[self.ids.arm_qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        h = self.ids.hand
        return self.d.xpos[h].copy(), self.d.xmat[h].reshape(3, 3).copy()

    def jac(self, q: np.ndarray) -> np.ndarray:
        self.d.qpos[self.ids.arm_qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        mujoco.mj_jacSite(self.m, self.d, self.jac_p, self.jac_r, self.ids.ee_site)
        a = self.ids.arm_dadr
        return np.vstack([self.jac_p[:, a], self.jac_r[:, a]])

    def solve(self, p_target: np.ndarray, R_target: np.ndarray, seed: np.ndarray, rest: np.ndarray | None = None,
              tol: float = 2e-4, rot_tol: float = 2e-3, max_iter: int = 400, rot_w: float = 0.3,
              damping: float = 0.03, null_gain: float = 0.05) -> IKResult:
        q = np.clip(np.asarray(seed, float), self.q_min, self.q_max)
        rest = q.copy() if rest is None else np.asarray(rest, float)
        W = np.r_[np.ones(3), np.full(3, rot_w)]
        pe = re = np.inf
        for _ in range(max_iter):
            p, R = self.fk(q)
            ep, er = p_target - p, rot_error(R_target, R)
            pe, re = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
            if pe < tol and re < rot_tol:
                return IKResult(q, True, pe, re)
            J = W[:, None] * self.jac(q)
            e = W * np.r_[ep, er]
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + damping**2 * np.eye(6), e)
            Jp = J.T @ np.linalg.inv(JJt + 1e-9 * np.eye(6))
            dq += null_gain * (np.eye(7) - Jp @ J) @ (rest - q)
            step = np.abs(dq).max()
            if step > 0.2:
                dq *= 0.2 / step
            q = np.clip(q + dq, self.q_min, self.q_max)
        return IKResult(q, False, pe, re, f"no IK solution: closest pose is {1e3 * pe:.1f} mm / "
                                          f"{np.degrees(re):.1f} deg from the target")

    def solve_multi(self, p_target, R_target, seeds, rest=None) -> IKResult:
        best = None
        for s in seeds:
            r = self.solve(p_target, R_target, s, rest)
            if r.ok:
                return r
            if best is None or r.pos_err < best.pos_err:
                best = r
        return best

    # ------------------------------------------------------------------ collision checks

    def _name(self, b: int) -> str:
        if b == 0:
            return "floor"
        pid = self.body_part.get(b)
        return f"part '{pid}'" if pid else self.m.body(b).name

    def contacts_at(self, q: np.ndarray, carried: int | None = None, allow_carried_table: bool = False) -> str | None:
        """First disallowed contact at arm configuration q in the planning world, or None."""
        self.d.qpos[self.ids.arm_qadr] = q
        mujoco.mj_forward(self.m, self.d)
        m, d = self.m, self.d
        for c in d.contact[: d.ncon]:
            if c.dist > 0:
                continue
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            r1, r2 = self.robot_body[b1], self.robot_body[b2]
            if r1 and r2:
                return f"self-collision {m.body(b1).name}-{m.body(b2).name}"
            if r1 or r2:
                mine, other = (b1, b2) if r1 else (b2, b1)
                if carried is not None and other == carried and self.hand_side[mine]:
                    continue
                return f"{m.body(mine).name} would hit {self._name(other)}"
            if carried is not None and carried in (b1, b2):
                other = b2 if b1 == carried else b1
                if other == self.table_body and allow_carried_table:
                    continue
                return f"carried {self._name(carried)} would hit {self._name(other)}"
        return None

    def path_collision(self, qs, carried_part: str | None = None, carry=None, allow_carried_table: bool = False) -> str | None:
        """Check sampled joint configurations. `carry` = (part offset in hand frame, part rotation in hand frame):
        the planning copy of the carried part moves with the hand (prediction only; the live part is never attached)."""
        carried = None
        if carried_part is not None:
            pi = self.ids.parts[carried_part]
            carried = pi.body
        for q in qs:
            if carry is not None and carried is not None:
                h, Rh = self.hand_pose(q)
                off, Rrel = carry
                a = self.ids.parts[carried_part].qadr
                self.d.qpos[a:a + 3] = h + Rh @ off
                qq = np.zeros(4)
                mujoco.mju_mat2Quat(qq, (Rh @ Rrel).ravel())
                self.d.qpos[a + 3:a + 7] = qq
            hit = self.contacts_at(q, carried, allow_carried_table)
            if hit:
                return hit
        return None
