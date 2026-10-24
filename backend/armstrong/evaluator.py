"""Ground-truth outcome evaluator.

Reads simulator truth (part poses, contacts) to score each workflow step and to display the world. It is never an
input to perception or to the task supervisor's decisions.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .perception import wrap
from .scene import TABLE_TOP_Z, SceneConfig, SceneIds

PLACE_TOL_XY = 0.015
PLACE_TOL_Z = 0.006
TILT_TOL_DEG = 10.0
YAW_TOL_DEG = 8.0
SETTLE_SPEED = 0.005
CARRY_PHASES = ("lift", "transport", "lower")


def part_yaw(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


class Evaluator:
    def __init__(self, model: mujoco.MjModel, ids: SceneIds, cfg: SceneConfig):
        self.m, self.ids, self.cfg = model, ids, cfg
        self.robot = np.zeros(model.nbody, bool)
        self.robot[ids.robot_bodies] = True
        self.fingers = np.zeros(model.nbody, bool)
        self.fingers[ids.finger_bodies] = True
        self.body_part = {pi.body: pid for pid, pi in ids.parts.items()}
        self.reset()

    def reset(self) -> None:
        self.forbidden = 0
        self.forbidden_first: str | None = None
        self.target: str | None = None
        self.z0 = None
        self.max_lift = 0.0
        self.carry_ref = None
        self.max_slip = 0.0

    def pose(self, d: mujoco.MjData, pid: str):
        b = self.ids.parts[pid].body
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()

    def begin_step(self, d: mujoco.MjData, pid: str) -> None:
        self.target = pid
        self.start_pos = self.pose(d, pid)[0]
        self.z0 = float(self.start_pos[2])
        self.max_lift = 0.0
        self.carry_ref = None
        self.max_slip = 0.0

    def _name(self, b: int) -> str:
        if b == 0:
            return "floor"
        pid = self.body_part.get(b)
        return f"part '{pid}'" if pid else self.m.body(b).name

    def step(self, d: mujoco.MjData, phase: str) -> None:
        m = self.m
        tb = None
        if self.target is not None:
            tb = self.ids.parts[self.target].body
            p, _ = self.pose(d, self.target)
            self.max_lift = max(self.max_lift, float(p[2] - self.z0))
            if phase in CARRY_PHASES:
                h = self.ids.hand
                rel = d.xmat[h].reshape(3, 3).T @ (p - d.xpos[h])
                if self.carry_ref is None:
                    self.carry_ref = rel
                self.max_slip = max(self.max_slip, float(np.linalg.norm(rel - self.carry_ref)))
        for c in d.contact[: d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            bad = None
            for a, b in ((b1, b2), (b2, b1)):
                if self.robot[a] and not self.robot[b]:
                    if b == tb and self.fingers[a]:
                        continue
                    bad = f"{m.body(a).name} touched {self._name(b)}"
                elif a in self.body_part and b == 0:
                    bad = f"{self._name(a)} touched the floor"
                elif a == tb and b in self.body_part and phase in CARRY_PHASES:
                    bad = f"carried {self._name(a)} touched {self._name(b)}"
            if bad:
                self.forbidden += 1
                if self.forbidden_first is None:
                    self.forbidden_first = f"{bad} during {phase} (t = {d.time:.2f} s)"
                break

    def estimate_error(self, est: dict | None) -> float | None:
        """Evaluation only: xy distance between a planning estimate and the part's true pose at step start."""
        if not est or not est.get("valid") or self.target is None:
            return None
        return float(np.linalg.norm(np.asarray(est["position_m"][:2]) - self.start_pos[:2]))

    def touching_fingers(self, d: mujoco.MjData, pid: str) -> bool:
        m, b = self.m, self.ids.parts[pid].body
        for c in d.contact[: d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b in (b1, b2) and (self.fingers[b1] or self.fingers[b2]):
                return True
        return False

    def end_step(self, d: mujoco.MjData, pid: str, target_xy, requested_yaw: float | None, period: float) -> dict:
        part = self.cfg.part(pid)
        p, R = self.pose(d, pid)
        tilt = float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), -1, 1))))
        target = np.array([target_xy[0], target_xy[1], TABLE_TOP_Z + part.size[2] / 2])
        err = p - target
        exy = float(np.linalg.norm(err[:2]))
        dofs = self.ids.parts[pid].dadr
        speed = float(np.linalg.norm(d.qvel[dofs:dofs + 3]))
        yaw = part_yaw(R)
        yaw_err = None if requested_yaw is None else abs(np.degrees(wrap(yaw - requested_yaw, period)))
        reasons = []
        if exy > PLACE_TOL_XY:
            reasons.append(f"{part.name} center is {1e3 * exy:.1f} mm from the destination center "
                           f"(tolerance {1e3 * PLACE_TOL_XY:.0f} mm)")
        if abs(err[2]) > PLACE_TOL_Z:
            reasons.append(f"{part.name} height error {1e3 * err[2]:.1f} mm (not resting on the table)")
        if tilt > TILT_TOL_DEG:
            reasons.append(f"{part.name} tilted {tilt:.1f} deg")
        if yaw_err is not None and yaw_err > YAW_TOL_DEG:
            reasons.append(f"{part.name} yaw is {yaw_err:.1f} deg from the requested orientation")
        if speed > SETTLE_SPEED:
            reasons.append(f"{part.name} still moving ({1e3 * speed:.1f} mm/s)")
        if self.touching_fingers(d, pid):
            reasons.append(f"{part.name} still touching the gripper")
        return {"placed": not reasons, "reasons": reasons, "placement_error_m": float(np.linalg.norm(err)),
                "placement_error_xy_m": exy, "placement_error_z_m": float(err[2]), "final_tilt_deg": tilt,
                "final_yaw_rad": yaw, "yaw_error_deg": yaw_err, "final_position_m": p.tolist(),
                "max_lift_m": self.max_lift, "object_slip_m": self.max_slip if self.carry_ref is not None else None,
                "tolerances": {"xy_m": PLACE_TOL_XY, "z_m": PLACE_TOL_Z, "tilt_deg": TILT_TOL_DEG,
                               "yaw_deg": YAW_TOL_DEG}}
