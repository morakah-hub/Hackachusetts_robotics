"""Ground-truth outcome evaluator.

Reads simulator truth (block pose, contacts) to score a run and to display the world. It is
never an input to perception or to the task supervisor's decisions.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .scene import TABLE_TOP_Z, SceneIds

PLACE_TOL_XY = 0.015
PLACE_TOL_Z = 0.006
TILT_TOL_DEG = 10.0
SETTLE_SPEED = 0.005


class Evaluator:
    def __init__(self, model: mujoco.MjModel, ids: SceneIds, block_half_z: float):
        self.m, self.ids = model, ids
        self.half_z = block_half_z
        robot = np.zeros(model.nbody, bool)
        robot[ids.robot_bodies] = True
        fingers = np.zeros(model.nbody, bool)
        fingers[ids.finger_bodies] = True
        self.robot, self.fingers = robot, fingers
        self.reset(None)

    def reset(self, data: mujoco.MjData | None) -> None:
        self.z0 = None if data is None else float(self.block_pos(data)[2])
        self.max_lift = 0.0
        self.forbidden = 0
        self.forbidden_first: str | None = None
        self.carry_ref = None
        self.max_slip = 0.0

    def block_pos(self, d: mujoco.MjData) -> np.ndarray:
        return d.xpos[self.ids.block_body].copy()

    def block_speed(self, d: mujoco.MjData) -> float:
        a = self.ids.block_dadr
        return float(np.linalg.norm(d.qvel[a:a + 3]))

    def block_in_hand(self, d: mujoco.MjData) -> np.ndarray:
        h = self.ids.hand
        return d.xmat[h].reshape(3, 3).T @ (self.block_pos(d) - d.xpos[h])

    def step(self, d: mujoco.MjData, phase: str) -> None:
        p = self.block_pos(d)
        if self.z0 is None:
            self.z0 = float(p[2])
        self.max_lift = max(self.max_lift, float(p[2] - self.z0))
        if phase in ("lift", "transport", "lower"):
            rel = self.block_in_hand(d)
            if self.carry_ref is None:
                self.carry_ref = rel
            self.max_slip = max(self.max_slip, float(np.linalg.norm(rel - self.carry_ref)))
        m = self.m
        for c in d.contact[: d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            bad = None
            for a, b in ((b1, b2), (b2, b1)):
                if self.robot[a] and not self.robot[b]:
                    if b == self.ids.block_body and self.fingers[a]:
                        continue
                    bad = f"{m.body(a).name} touched {m.body(b).name or 'floor'}"
                if a == self.ids.block_body and b == 0:
                    bad = "block touched the floor"
            if bad:
                self.forbidden += 1
                if self.forbidden_first is None:
                    self.forbidden_first = f"{bad} during {phase} (t = {d.time:.2f} s)"
                break

    def touching_fingers(self, d: mujoco.MjData) -> bool:
        m = self.m
        for c in d.contact[: d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if self.ids.block_body in (b1, b2) and (self.fingers[b1] or self.fingers[b2]):
                return True
        return False

    def final(self, d: mujoco.MjData, pad_xy: np.ndarray) -> dict:
        p = self.block_pos(d)
        R = d.xmat[self.ids.block_body].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), -1, 1))))
        target = np.array([pad_xy[0], pad_xy[1], TABLE_TOP_Z + self.half_z])
        err = p - target
        exy = float(np.linalg.norm(err[:2]))
        speed = self.block_speed(d)
        reasons = []
        if exy > PLACE_TOL_XY:
            reasons.append(f"block center is {1e3 * exy:.1f} mm from the pad center (tolerance {1e3 * PLACE_TOL_XY:.0f} mm)")
        if abs(err[2]) > PLACE_TOL_Z:
            reasons.append(f"block height error {1e3 * err[2]:.1f} mm (not resting on the table)")
        if tilt > TILT_TOL_DEG:
            reasons.append(f"block tilted {tilt:.1f} deg")
        if speed > SETTLE_SPEED:
            reasons.append(f"block still moving ({1e3 * speed:.1f} mm/s)")
        if self.touching_fingers(d):
            reasons.append("block still touching the gripper")
        return {"placed": not reasons, "reasons": reasons, "placement_error_m": float(np.linalg.norm(err)),
                "placement_error_xy_m": exy, "placement_error_z_m": float(err[2]), "final_tilt_deg": tilt,
                "final_block_position_m": p.tolist(), "max_lift_m": self.max_lift,
                "object_slip_m": self.max_slip if self.carry_ref is not None else None,
                "forbidden_contacts": self.forbidden, "first_forbidden_contact": self.forbidden_first,
                "tolerances": {"xy_m": PLACE_TOL_XY, "z_m": PLACE_TOL_Z, "tilt_deg": TILT_TOL_DEG}}
