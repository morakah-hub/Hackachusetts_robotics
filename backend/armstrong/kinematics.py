"""Arm kinematics on private scratch data: FK, full-pose IK and collision checks.

Pose IK is damped least squares on the 6 x 7 site Jacobian with a world-frame error
e = [p* - p ; w * log(R* R^T)], a null-space pull toward a rest posture and joint-limit
clamping. Nothing here writes the live simulation state.
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
        robot = set(ids.robot_bodies.tolist())
        self.robot_body = np.zeros(model.nbody, dtype=bool)
        self.robot_body[list(robot)] = True
        self.finger_body = np.zeros(model.nbody, dtype=bool)
        self.finger_body[ids.finger_bodies] = True
        self.hand_side = self.finger_body.copy()
        self.hand_side[ids.hand] = True

    def load(self, qpos_full: np.ndarray, q: np.ndarray | None = None) -> None:
        self.d.qpos[:] = qpos_full
        if q is not None:
            self.d.qpos[self.ids.arm_qadr] = q

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

    def contacts_at(self, q: np.ndarray, allow_hand_block: bool, allow_block_table: bool,
                    margin: float = 0.0) -> str | None:
        """First disallowed robot contact at q with the scratch state (block where loaded), or None."""
        self.d.qpos[self.ids.arm_qadr] = q
        mujoco.mj_forward(self.m, self.d)
        m, d = self.m, self.d
        for c in d.contact[: d.ncon]:
            if c.dist > -margin:
                continue
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            r1, r2 = self.robot_body[b1], self.robot_body[b2]
            blk = self.ids.block_body
            if r1 or r2:
                other = b2 if r1 else b1
                mine = b1 if r1 else b2
                if self.robot_body[other]:
                    return f"self-collision {m.body(b1).name}-{m.body(b2).name}"
                if other == blk and allow_hand_block and self.hand_side[mine]:
                    continue
                return f"{m.body(mine).name} would hit {m.body(other).name if other else 'floor'}"
            if blk in (b1, b2) and not allow_block_table:
                other = b2 if b1 == blk else b1
                return f"block would hit {m.body(other).name if other else 'floor'}"
        return None

    def path_collision(self, qs, allow_hand_block=False, allow_block_table=True, carry=None,
                       margin: float = 0.0) -> str | None:
        """Check sampled joint configurations. `carry` = (block offset in hand frame, block quat in hand frame):
        the planning copy of the block moves with the hand (prediction only; the live block is never attached)."""
        a = self.ids.block_qadr
        for q in qs:
            if carry is not None:
                h, Rh = self.hand_pose(q)
                off, Rrel = carry
                self.d.qpos[a:a + 3] = h + Rh @ off
                qq = np.zeros(4)
                mujoco.mju_mat2Quat(qq, (Rh @ Rrel).ravel())
                self.d.qpos[a + 3:a + 7] = qq
            hit = self.contacts_at(q, allow_hand_block, allow_block_table, margin)
            if hit:
                return hit
        return None
