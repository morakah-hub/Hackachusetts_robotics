"""Pick-and-place task supervisor: a bounded, event-driven state machine.

    observe -> plan -> approach -> descend -> close -> lift -> transport -> lower -> release
            -> retreat -> verify

`program()` is a generator; each `yield` lets the simulation worker take one physics step.
Phases end on measured conditions (arm at rest, finger width, camera observation) or on a
bounded timeout that fails the run with a stage and a reason.

Information boundary. In camera mode the block pose comes only from `estimate_block` on a
fresh RGB-D frame; the supervisor's other inputs are proprioception (arm joints, finger width),
the catalog (block size/color) and the user-selected destination pad. Known-world mode reads
the simulator pose and is labelled as such. The live block is moved only by contact: nothing
here writes its state or attaches it to the hand.
"""

from __future__ import annotations

import numpy as np

from . import scene as sc
from .kinematics import rot_z
from .perception import estimate_block
from .robot import GRIPPER_CLOSED, GRIPPER_OPEN, HOME_Q
from .trajectory import JointMove, LineInfeasible, cartesian_line

PHASES = ("observe", "plan", "approach", "descend", "close", "lift", "transport", "lower", "release",
          "retreat", "verify")
GRASP_ABOVE_CENTER = 0.015
APPROACH_HEIGHT = 0.10
LIFT_HEIGHT = 0.10
PLACE_CLEARANCE = 0.002
WIDTH_TOL = 0.006
VERIFY_TOL_XY = 0.015


class TaskFailure(Exception):
    def __init__(self, stage: str, code: str, reason: str):
        super().__init__(f"{stage}: {reason}")
        self.stage, self.code, self.reason = stage, code, reason


class PickPlaceTask:
    def __init__(self, sim, request: dict):
        self.sim = sim
        self.mode = request.get("perception_mode", "camera_rgbd")
        if self.mode not in ("camera_rgbd", "known_world"):
            raise TaskFailure("plan", "UNSUPPORTED_TASK", f"unknown perception mode '{self.mode}'")
        cfg = sim.cfg
        self.block_size = np.asarray(cfg.block.size, float)
        self.pad_xy = np.array([cfg.pad.x, cfg.pad.y])   # user-defined goal location (not perceived)
        self.catalog = {"id": "block", "color_class": "orange", "size_m": self.block_size.tolist(),
                        "support_z_m": sc.TABLE_TOP_Z}
        self.phase = "observe"
        self.phase_log: list[dict] = []
        self.estimate: dict | None = None
        self.verify_estimate: dict | None = None
        self.plan_info: dict = {}
        self.grasp_width: float | None = None
        _, self.R_home = sim.kin.fk(HOME_Q)

    # ------------------------------------------------------------------ helpers

    def _enter(self, phase: str, note: str = "") -> None:
        t = self.sim.data.time
        if self.phase_log:
            self.phase_log[-1]["t_end"] = t
        self.phase = phase
        self.phase_log.append({"phase": phase, "t_start": t, "t_end": None, "note": note})
        self.sim.log(f"phase {phase}" + (f": {note}" if note else ""))

    def _wait(self, cond, timeout: float, stage: str, code: str, reason: str, check=None):
        deadline = self.sim.data.time + timeout
        while not cond():
            if self.sim.data.time >= deadline:
                raise TaskFailure(stage, code, reason)
            if check is not None:
                check()
            yield

    def _execute(self, seg, check=None):
        sim = self.sim
        sim.set_reference(seg)
        n = {"k": 0}

        def at_rest():
            if sim.reference_finished() and np.abs(sim.arm_qvel()).max() < 0.03:
                n["k"] += 1
            else:
                n["k"] = 0
            return n["k"] >= 20
        yield from self._wait(at_rest, seg.duration + 2.0, self.phase, "CONTROL_TIMEOUT",
                              "arm did not come to rest within 2 s after the planned motion", check)

    def _fingers_settled(self):
        st = {"last": self.sim.finger_width(), "n": 0, "t0": self.sim.data.time}

        def cond():
            w = self.sim.finger_width()
            st["n"] = st["n"] + 1 if abs(w - st["last"]) < 2e-5 else 0
            st["last"] = w
            return self.sim.data.time - st["t0"] > 0.15 and st["n"] >= 25
        return cond

    def _ik(self, p, R, what: str, seeds=None):
        kin = self.sim.kin
        seeds = seeds or [self.sim.arm_q(), HOME_Q]
        r = kin.solve_multi(p, R, seeds, rest=HOME_Q)
        if not r.ok:
            raise TaskFailure("plan", "PLAN_FAILED", f"{what} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m is "
                                                     f"unreachable with a top-down grasp: {r.reason}")
        return r.q

    def _locate(self, stage: str) -> dict:
        """Block pose for planning or verification, from the camera (or labelled simulator truth)."""
        sim = self.sim
        if self.mode == "known_world":
            p, yaw = sim.truth_block_pose()
            return {"id": "block", "valid": True, "reasons": [], "position_m": p.tolist(), "yaw_rad": yaw,
                    "source": "known_world (simulator pose, development mode)"}
        obs = sim.observe()
        est = estimate_block(obs, self.catalog, current_revision=sim.scene_revision)
        sim.publish_estimate(est)
        if not est["valid"]:
            raise TaskFailure(stage, "PERCEPTION_UNAVAILABLE",
                              "camera observation invalid, motion not started: " + "; ".join(est["reasons"]))
        return est

    # ------------------------------------------------------------------ the task

    def program(self):
        sim = self.sim
        h = self.block_size[2]

        self._enter("observe", "camera RGB-D frame" if self.mode == "camera_rgbd" else "known-world pose")
        n = {"k": 0}

        def still():
            n["k"] = n["k"] + 1 if np.abs(sim.arm_qvel()).max() < 0.02 else 0
            return n["k"] >= 10
        yield from self._wait(still, 2.0, "observe", "CONTROL_TIMEOUT", "arm not at rest before observing")
        est = self._locate("observe")
        self.estimate = est
        c = np.asarray(est["position_m"], float)
        yaw = float(est["yaw_rad"] or 0.0)
        yield

        self._enter("plan")
        for what, (x, y) in (("block", c[:2]), ("destination pad", self.pad_xy)):
            why = sc.workspace_check(float(x), float(y), what)
            if why:
                raise TaskFailure("plan", "OUT_OF_WORKSPACE", why)
        if np.linalg.norm(c[:2] - self.pad_xy) < 0.07:
            raise TaskFailure("plan", "UNSUPPORTED_TASK", "block is already on (or overlapping) the destination pad")
        R = rot_z(yaw) @ self.R_home
        p_grasp = c + [0, 0, GRASP_ABOVE_CENTER]
        p_pre = p_grasp + [0, 0, APPROACH_HEIGHT]
        place_center = np.array([self.pad_xy[0], self.pad_xy[1], sc.TABLE_TOP_Z + h / 2 + PLACE_CLEARANCE])
        p_place = place_center + [0, 0, GRASP_ABOVE_CENTER]
        p_preplace = p_place + [0, 0, LIFT_HEIGHT]
        q_pre = self._ik(p_pre, R, "pre-grasp pose")
        q_grasp = self._ik(p_grasp, R, "grasp pose", seeds=[q_pre])
        q_preplace = self._ik(p_preplace, R, "pre-place pose", seeds=[q_pre, HOME_Q])
        self._ik(p_place, R, "place pose", seeds=[q_preplace])
        self.plan_info = {"grasp_yaw_rad": yaw, "block_estimate_m": c.tolist(), "pre_grasp_m": p_pre.tolist(),
                          "grasp_m": p_grasp.tolist(), "place_m": p_place.tolist(),
                          "destination_xy_m": self.pad_xy.tolist(), "q_grasp": q_grasp.tolist(),
                          "planning_world": "block from " + est.get("source", "camera_rgbd") +
                                            "; table/floor geometry known from the scene file (not perceived)"}
        sim.publish_plan(self.plan_info)
        # Planning copy of the world: block at the *estimated* pose.
        kin = sim.kin
        kin.load(sim.data.qpos)
        bq = sim.ids.block_qadr
        kin.d.qpos[bq:bq + 3] = c
        kin.d.qpos[bq + 3:bq + 7] = sc.yaw_quat(yaw)
        approach = JointMove(sim.arm_q(), q_pre, min_time=1.5)
        hit = kin.path_collision(approach.samples(50), allow_hand_block=False)
        if hit:
            raise TaskFailure("plan", "PLAN_FAILED", f"direct approach path is not collision-free: {hit}")
        yield

        self._enter("approach")
        yield from self._execute(approach)

        self._enter("descend")
        try:
            seg = cartesian_line(kin, sim.reference_end(), p_grasp, R, speed=0.06, min_time=1.4)
        except LineInfeasible as exc:
            raise TaskFailure("descend", "PLAN_FAILED", str(exc)) from None
        hit = kin.path_collision(seg.samples(), allow_hand_block=False)
        if hit:
            raise TaskFailure("descend", "PLAN_FAILED", f"open fingers would collide on the way down: {hit}")
        yield from self._execute(seg)

        self._enter("close")
        sim.set_gripper(GRIPPER_CLOSED)
        yield from self._wait(self._fingers_settled(), 2.0, "close", "GRASP_FAILED", "fingers did not come to rest")
        width = sim.finger_width()
        self.grasp_width = width
        expected = float(self.block_size[0])
        if width > expected + WIDTH_TOL:
            raise TaskFailure("close", "GRASP_FAILED", f"gripper did not close: fingers at {1e3 * width:.1f} mm "
                                                       f"(block is {1e3 * expected:.0f} mm wide)")
        if width < expected - WIDTH_TOL:
            raise TaskFailure("close", "GRASP_FAILED", f"fingers closed to {1e3 * width:.1f} mm: block not between "
                                                       f"the fingers (expected {1e3 * expected:.0f} mm)")

        def holding():
            w = sim.finger_width()
            if w < expected - WIDTH_TOL:
                raise TaskFailure(self.phase, "GRASP_FAILED", f"finger width dropped to {1e3 * w:.1f} mm: "
                                                              f"block slipped out of the gripper")

        self._enter("lift")
        p_now, _ = kin.fk(sim.reference_end())
        try:
            seg = cartesian_line(kin, sim.reference_end(), p_now + [0, 0, LIFT_HEIGHT], R, speed=0.08)
        except LineInfeasible as exc:
            raise TaskFailure("lift", "PLAN_FAILED", str(exc)) from None
        yield from self._execute(seg, holding)

        self._enter("transport")
        # Grasp offset assumed from the estimate: the block center sits GRASP_ABOVE_CENTER below the
        # fingertip point, aligned with the hand (no simulator truth used).
        q_top = sim.reference_end()
        hp, hR = kin.hand_pose(q_top)
        ep, _ = kin.fk(q_top)
        off = hR.T @ (ep - [0, 0, GRASP_ABOVE_CENTER] - hp)
        Rrel = hR.T @ rot_z(yaw)
        move = JointMove(q_top, q_preplace, min_time=1.5)
        hit = kin.path_collision(move.samples(50), allow_hand_block=True, allow_block_table=False,
                                 carry=(off, Rrel))
        if hit:
            raise TaskFailure("transport", "PLAN_FAILED", f"transport path is not collision-free: {hit}")
        yield from self._execute(move, holding)

        self._enter("lower")
        try:
            seg = cartesian_line(kin, sim.reference_end(), p_place, R, speed=0.06, min_time=1.4)
        except LineInfeasible as exc:
            raise TaskFailure("lower", "PLAN_FAILED", str(exc)) from None
        yield from self._execute(seg, holding)

        self._enter("release")
        sim.set_gripper(GRIPPER_OPEN)
        yield from self._wait(self._fingers_settled(), 2.0, "release", "GRASP_FAILED", "fingers did not open")
        if sim.finger_width() < expected + 0.015:
            raise TaskFailure("release", "GRASP_FAILED", f"fingers opened only to {1e3 * sim.finger_width():.1f} mm")

        self._enter("retreat")
        p_now, _ = kin.fk(sim.reference_end())
        try:
            seg = cartesian_line(kin, sim.reference_end(), p_now + [0, 0, LIFT_HEIGHT], R, speed=0.08)
        except LineInfeasible as exc:
            raise TaskFailure("retreat", "PLAN_FAILED", str(exc)) from None
        yield from self._execute(seg)
        yield from self._execute(JointMove(sim.reference_end(), HOME_Q, min_time=1.5))

        self._enter("verify", "re-observe the placed block")
        t_end = sim.data.time + 0.4
        yield from self._wait(lambda: sim.data.time >= t_end, 1.0, "verify", "CONTROL_TIMEOUT", "never")
        est = self._locate("verify")
        self.verify_estimate = est
        err = float(np.linalg.norm(np.asarray(est["position_m"][:2]) - self.pad_xy))
        est["placement_error_xy_m"] = err
        if err > VERIFY_TOL_XY:
            raise TaskFailure("verify", "PLACE_FAILED", f"{'camera' if self.mode == 'camera_rgbd' else 'known-world'} "
                                                        f"check: block is {1e3 * err:.1f} mm from the pad center")
        self.phase_log[-1]["t_end"] = sim.data.time
