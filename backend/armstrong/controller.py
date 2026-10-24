"""Computed-torque control (CTC) of the 7 arm joints.

    tau = ID(q, qd, qdd_ref + Kp (q_ref - q) + Kd (qd_ref - qd))

ID is MuJoCo inverse dynamics (mj_inverse) on a private copy of the model with contacts
disabled and the block at a fixed nominal mass, so the controller never sees payload or
contact forces through copied state. The payload is an unmodelled disturbance handled by
the feedback terms and a small, clamped integral term. Torques are clipped to the Panda
actuator limits (87 Nm joints 1-4, 12 Nm joints 5-7).
"""

from __future__ import annotations

import copy

import mujoco
import numpy as np

from .scene import SceneIds


class ComputedTorqueController:
    name = "Computed torque (CTC)"
    key = "ctc"

    def __init__(self, model: mujoco.MjModel, ids: SceneIds, kp: float = 900.0, kd: float = 60.0,
                 ki: float = 600.0, i_limit: float = 0.02):
        self.m = copy.deepcopy(model)
        self.m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        self.d = mujoco.MjData(self.m)
        self.ids = ids
        self.kp = np.full(7, kp)
        self.kd = np.full(7, kd)
        self.ki = np.full(7, ki)
        self.i_limit = i_limit
        self.integral = np.zeros(7)
        self.tau_limit = np.abs(model.actuator_forcerange[ids.arm_act]).max(axis=1)
        self.saturated_steps = 0

    def gains(self) -> dict:
        return {"kp": float(self.kp[0]), "kd": float(self.kd[0]), "ki": float(self.ki[0]),
                "integral_limit_rad_s": self.i_limit}

    def reset(self) -> None:
        self.integral[:] = 0.0

    def compute(self, data: mujoco.MjData, q_ref, qd_ref, qdd_ref, dt: float) -> np.ndarray:
        a, v = self.ids.arm_qadr, self.ids.arm_dadr
        q, qd = data.qpos[a], data.qvel[v]
        e, ed = q_ref - q, qd_ref - qd
        self.integral = np.clip(self.integral + e * dt, -self.i_limit, self.i_limit)
        acc = qdd_ref + self.kp * e + self.kd * ed + self.ki * self.integral
        d = self.d
        d.qpos[:] = data.qpos
        d.qvel[:] = data.qvel
        d.qacc[:] = 0.0
        d.qacc[v] = acc
        mujoco.mj_inverse(self.m, d)
        tau = d.qfrc_inverse[v].copy()
        clipped = np.clip(tau, -self.tau_limit, self.tau_limit)
        if np.any(clipped != tau):
            self.saturated_steps += 1
        return clipped
