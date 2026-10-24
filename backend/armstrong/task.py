"""Workflow executor: a recipe of steps, each executed by the same physical pick-and-place skill.

Per step (bounded state machine; each `yield` lets the worker take one physics step):
    observe -> plan -> approach -> descend -> close -> lift -> transport -> lower -> release -> retreat -> verify

* observe: a fresh RGB-D frame of the current scene revision; every movable part is localized from the image
  (camera mode) or read from the simulator (known-world development mode, labelled).
* plan: destination resolution (fixed, or first free slot of a sort zone), workspace / eligibility / occupancy
  checks, grasp axis (across the part's short side), requested final yaw, full-pose IK for every key pose and a
  collision check of *every* motion segment (arm, and the carried part during lift/transport/lower) in a planning
  world built from proprioception, the estimates and known static geometry. A blocked route is rejected before
  any motion starts.
* verify: re-observe; the part must be on the destination (and at the requested yaw), other parts undisturbed.
  The ground-truth evaluator then scores the step independently; a failing evaluation stops the sequence and is
  labelled as an evaluator result.

The world is never reset between steps; placed parts stay where physics left them. Nothing here writes a part's
state or attaches it to the hand: parts move only through finger contact and friction.
"""

from __future__ import annotations

import time

import numpy as np

from . import scene as sc
from .kinematics import rot_z
from .perception import SQUARE_TOL_M, SceneObserver, wrap
from .robot import GRIPPER_CLOSED, GRIPPER_OPEN, HOME_Q
from .trajectory import JointMove, LineInfeasible, cartesian_line

PHASES = ("observe", "plan", "approach", "descend", "close", "lift", "transport", "lower", "release",
          "retreat", "verify")
ACTIONS = ("place", "sort")
MAX_STEPS = 8
GRASP_BELOW_TOP = 0.015     # fingertip midpoint 15 mm below the part's top face (validated v0.1 setting)
APPROACH_HEIGHT = 0.10
LIFT_HEIGHT = 0.10
PLACE_CLEARANCE = 0.002
WIDTH_TOL = 0.006
VERIFY_TOL_XY = 0.015
VERIFY_TOL_YAW_DEG = 8.0
OCCUPANCY_MARGIN = 0.008
DISTURB_TOL = 0.010


class TaskFailure(Exception):
    def __init__(self, stage: str, code: str, reason: str):
        super().__init__(f"{stage}: {reason}")
        self.stage, self.code, self.reason = stage, code, reason


def table_info() -> dict:
    return {"top_z": sc.TABLE_TOP_Z, "center_xy": sc.TABLE_CENTER_XY, "half_xy": sc.TABLE_HALF_XY}


def catalog_entry(p: sc.PartSpec) -> dict:
    """Perception priors for one part: identity, color class and dimensions. No pose information."""
    return {"id": p.id, "name": p.name, "color_class": p.color, "size_m": list(p.size), "support_z_m": sc.TABLE_TOP_Z}


def default_recipe(cfg: sc.SceneConfig) -> dict:
    mov = cfg.movable()
    steps = []
    if mov and cfg.destinations:
        steps = [{"id": "s1", "part": mov[0].id, "action": "place", "dest": cfg.destinations[0].id}]
    return {"version": 1, "name": "Move part", "steps": steps, "sort_rules": []}


def compile_recipe(cfg: sc.SceneConfig, recipe: dict) -> tuple[list[dict], dict]:
    """Validate a recipe against the scene. Returns (normalized steps, zone map {category: zone})."""
    if not isinstance(recipe, dict):
        raise TaskFailure("plan", "INVALID_RECIPE", "workflow must be an object")
    steps = recipe.get("steps") or []
    if not isinstance(steps, list) or not steps:
        raise TaskFailure("plan", "INVALID_RECIPE", "the workflow has no steps")
    if len(steps) > MAX_STEPS:
        raise TaskFailure("plan", "INVALID_RECIPE", f"at most {MAX_STEPS} steps are supported")
    zones = {}
    for r in recipe.get("sort_rules") or []:
        if isinstance(r, dict) and r.get("category") and r.get("zone"):
            zones[str(r["category"])] = str(r["zone"])
    out, used_parts, used_dests = [], set(), set()
    for i, s in enumerate(steps, start=1):
        if not isinstance(s, dict):
            raise TaskFailure("plan", "INVALID_RECIPE", f"step {i} is malformed")
        part = cfg.part(str(s.get("part")))
        if part is None:
            raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: unknown part {s.get('part')!r}")
        if part.role != "movable":
            raise TaskFailure("plan", "UNSUPPORTED_PART", f"step {i}: {part.name} is a {part.role}, not a movable part")
        issues = sc.movable_issues(part)
        if issues:
            raise TaskFailure("plan", "UNSUPPORTED_PART", f"step {i}: {part.name} cannot be grasped by the validated "
                                                          f"skill: " + "; ".join(issues))
        if part.id in used_parts:
            raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: {part.name} already appears in an earlier step")
        used_parts.add(part.id)
        action = s.get("action", "place")
        if action not in ACTIONS:
            raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: unsupported action {action!r}")
        step = {"id": str(s.get("id") or f"s{i}")[:16], "index": i, "part": part.id, "action": action, "dest": None,
                "zone": None}
        if action == "place":
            dest = cfg.dest(str(s.get("dest")))
            if dest is None:
                raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: choose a destination for {part.name}")
            if dest.id in used_dests:
                raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: {dest.name} is already the target of an "
                                                            f"earlier step")
            used_dests.add(dest.id)
            why = sc.workspace_check(dest.x, dest.y, f"destination {dest.name}")
            if why:
                raise TaskFailure("plan", "OUT_OF_WORKSPACE", f"step {i}: {why}")
            step["dest"] = dest.id
        else:
            if not part.category:
                raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: {part.name} has no category to sort by")
            zone = zones.get(part.category, part.category)
            slots = [d for d in cfg.destinations if d.zone == zone]
            if not slots:
                raise TaskFailure("plan", "INVALID_RECIPE", f"step {i}: no destination belongs to zone '{zone}' "
                                                            f"(category '{part.category}')")
            step["zone"] = zone
        out.append(step)
    return out, zones


class WorkflowTask:
    def __init__(self, sim, request: dict):
        self.sim = sim
        self.mode = request.get("perception_mode", "camera_rgbd")
        if self.mode not in ("camera_rgbd", "known_world"):
            raise TaskFailure("plan", "UNSUPPORTED_TASK", f"unknown perception mode '{self.mode}'")
        self.cfg = sim.cfg
        self.recipe = request["recipe"]
        self.steps, self.zones = compile_recipe(self.cfg, self.recipe)
        self.phase = "observe"
        self.phase_log: list[dict] = []
        self.records: list[dict] = []
        self.current: int | None = None
        self.estimate: dict | None = None
        self.verify_estimate: dict | None = None
        self.plan_info: dict = {}
        self.grasp_width: float | None = None
        _, self.R_home = sim.kin.fk(HOME_Q)

    # ------------------------------------------------------------------ helpers

    def _enter(self, phase: str, note: str = "") -> None:
        t = self.sim.data.time
        if self.phase_log and self.phase_log[-1]["t_end"] is None:
            self.phase_log[-1]["t_end"] = t
        self.phase = phase
        self.phase_log.append({"phase": phase, "t_start": t, "t_end": None, "note": note,
                               "step": None if self.current is None else self.current + 1})
        self.sim.log(f"step {self.current + 1}: {phase}" + (f" ({note})" if note else ""))

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

    def _ik(self, p, R, what: str, seeds):
        r = self.sim.kin.solve_multi(p, R, seeds, rest=HOME_Q)
        if not r.ok:
            raise TaskFailure("plan", "PLAN_FAILED", f"{what} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m is "
                                                     f"unreachable with a top-down grasp: {r.reason}")
        return r.q

    @staticmethod
    def _wrist_seed(yaw: float) -> np.ndarray:
        q = HOME_Q.copy()
        q[6] = np.clip(HOME_Q[6] - yaw, -2.85, 2.85)
        return q

    def _line(self, q0, p_end, R, what: str, speed=0.08, min_time=1.0):
        try:
            return cartesian_line(self.sim.kin, q0, p_end, R, speed=speed, min_time=min_time)
        except LineInfeasible as exc:
            raise TaskFailure("plan", "PLAN_FAILED", f"{what}: {exc}") from None

    def _observe_all(self, stage: str) -> dict:
        """Estimates of every movable part from one camera frame (or labelled simulator truth)."""
        sim = self.sim
        out = {}
        if self.mode == "known_world":
            for p in self.cfg.movable():
                pos, yaw = sim.truth_part_pose(p.id)
                period = np.pi / 2 if abs(p.size[0] - p.size[1]) < SQUARE_TOL_M else np.pi
                out[p.id] = {"id": p.id, "valid": True, "reasons": [], "position_m": pos.tolist(),
                             "yaw_rad": wrap(yaw, period), "yaw_period_rad": period,
                             "source": "known_world (simulator pose, development mode)"}
        else:
            obs = sim.observe()
            observer = SceneObserver(obs, table_info())
            for p in self.cfg.movable():
                out[p.id] = observer.estimate(catalog_entry(p), current_revision=sim.scene_revision)
        sim.publish_estimates(out)
        return out

    def _require(self, est: dict, stage: str, what: str) -> None:
        bad = [f"{self.cfg.part(pid).name}: " + "; ".join(e["reasons"]) for pid, e in est.items() if not e["valid"]]
        if bad:
            raise TaskFailure(stage, "PERCEPTION_UNAVAILABLE", f"camera observation invalid, {what}: " + " | ".join(bad))

    def _occupant(self, est: dict, pid: str, center, size, yaw) -> str | None:
        for p in self.cfg.parts:
            if p.id == pid or p.role == "visual":
                continue
            if p.role == "movable":
                e = est[p.id]
                c2, y2 = e["position_m"], e["yaw_rad"]
            else:
                c2, y2 = (p.x, p.y), p.yaw
            if sc.rects_overlap(center, size[:2], yaw, c2, p.size[:2], y2, OCCUPANCY_MARGIN):
                return p.name
        return None

    def _resolve_dest(self, step, part, est, final_yaw_fn):
        if step["action"] == "place":
            d = self.cfg.dest(step["dest"])
            occ = self._occupant(est, part.id, (d.x, d.y), part.size, final_yaw_fn(d))
            if occ:
                raise TaskFailure("plan", "DESTINATION_OCCUPIED", f"{d.name} is occupied by {occ}; clear it or "
                                                                  f"choose another destination")
            return d
        slots = [d for d in self.cfg.destinations if d.zone == step["zone"]]
        for d in slots:
            if sc.workspace_check(d.x, d.y, "slot"):
                continue
            if not self._occupant(est, part.id, (d.x, d.y), part.size, final_yaw_fn(d)):
                return d
        raise TaskFailure("plan", "NO_FREE_SLOT", f"zone '{step['zone']}' has no free reachable slot for {part.name} "
                                                  f"({len(slots)} slot(s), all occupied or out of reach)")

    # ------------------------------------------------------------------ program

    def program(self):
        for i, step in enumerate(self.steps):
            self.current = i
            part = self.cfg.part(step["part"])
            rec = {"index": i + 1, "step_id": step["id"], "part_id": part.id, "part_name": part.name,
                   "action": step["action"], "dest_id": step["dest"], "dest_name": None, "zone": step["zone"],
                   "perception_mode": self.mode, "outcome": "running", "failure_stage": None, "failure_code": None,
                   "reason": None, "estimate": None, "verify_estimate": None, "plan": None, "metrics": None,
                   "sim_t0": float(self.sim.data.time), "wall_t0": time.perf_counter()}
            self.records.append(rec)
            try:
                yield from self._step(step, part, rec)
            except TaskFailure as f:
                rec.update(outcome="failure", failure_stage=f.stage, failure_code=f.code, reason=f.reason)
                self._close(rec)
                raise
            self._close(rec)
        if self.phase_log and self.phase_log[-1]["t_end"] is None:
            self.phase_log[-1]["t_end"] = self.sim.data.time

    def _close(self, rec: dict) -> None:
        rec["sim_duration_s"] = float(self.sim.data.time) - rec.pop("sim_t0", self.sim.data.time)
        rec["wall_duration_s"] = time.perf_counter() - rec.pop("wall_t0", time.perf_counter())
        if rec.get("metrics") is None and self.sim.evaluator.target == rec["part_id"]:
            rec["metrics"] = {"max_lift_m": self.sim.evaluator.max_lift}

    def _step(self, step: dict, part: sc.PartSpec, rec: dict):
        sim, kin = self.sim, self.sim.kin
        h = part.size[2]
        sim.evaluator.begin_step(sim.data, part.id)

        self._enter("observe", "camera RGB-D frame" if self.mode == "camera_rgbd" else "known-world poses")
        n = {"k": 0}

        def still():
            n["k"] = n["k"] + 1 if np.abs(sim.arm_qvel()).max() < 0.02 else 0
            return n["k"] >= 10
        yield from self._wait(still, 2.0, "observe", "CONTROL_TIMEOUT", "arm not at rest before observing")
        est = self._observe_all("observe")
        self._require(est, "observe", "motion not started")
        e = est[part.id]
        self.estimate = rec["estimate"] = e
        c = np.asarray(e["position_m"], float)
        psi = float(e["yaw_rad"] or 0.0)
        period = float(e.get("yaw_period_rad") or np.pi / 2)
        yield

        self._enter("plan", part.name)
        why = sc.workspace_check(float(c[0]), float(c[1]), part.name)
        if why:
            raise TaskFailure("plan", "OUT_OF_WORKSPACE", why)
        sx, sy = part.size[0], part.size[1]
        if abs(sx - sy) < SQUARE_TOL_M:
            g, grip_width = wrap(psi, np.pi / 2), sx
        elif sx < sy:
            g, grip_width = wrap(psi, np.pi), sx
        else:
            g, grip_width = wrap(psi + np.pi / 2, np.pi), sy
        delta = psi - g

        def final_yaw(d):
            if d.yaw is None:
                return psi
            return wrap(g + wrap(d.yaw - psi, period), np.pi) + delta

        dest = self._resolve_dest(step, part, est, final_yaw)
        rec["dest_id"], rec["dest_name"] = dest.id, dest.name
        why = sc.workspace_check(dest.x, dest.y, f"destination {dest.name}")
        if why:
            raise TaskFailure("plan", "OUT_OF_WORKSPACE", why)
        if dest.yaw is None:
            g2 = g
        else:
            g2 = wrap(g + wrap(dest.yaw - psi, period), np.pi)
        psi_f = g2 + delta
        R1, R2 = rot_z(g) @ self.R_home, rot_z(g2) @ self.R_home
        p_grasp = c + [0, 0, h / 2 - GRASP_BELOW_TOP]
        p_pre = p_grasp + [0, 0, APPROACH_HEIGHT]
        place_center = np.array([dest.x, dest.y, sc.TABLE_TOP_Z + h / 2 + PLACE_CLEARANCE])
        p_place = place_center + [0, 0, h / 2 - GRASP_BELOW_TOP]
        p_preplace = p_place + [0, 0, LIFT_HEIGHT]
        kin.set_world(sim.data, {pid: (np.asarray(x["position_m"]), float(x["yaw_rad"])) for pid, x in est.items()})
        q_now = sim.arm_q()
        q_pre = self._ik(p_pre, R1, f"pre-grasp pose for {part.name}", [q_now, self._wrist_seed(g), HOME_Q])
        approach = JointMove(q_now, q_pre, min_time=1.5)
        descend = self._line(q_pre, p_grasp, R1, "descent", speed=0.06, min_time=1.4)
        lift = self._line(descend.end, p_grasp + [0, 0, LIFT_HEIGHT], R1, "lift")
        q_preplace = self._ik(p_preplace, R2, f"pre-place pose over {dest.name}",
                              [lift.end, self._wrist_seed(g2), HOME_Q])
        transport = JointMove(lift.end, q_preplace, min_time=1.5)
        lower = self._line(q_preplace, p_place, R2, "lowering", speed=0.06, min_time=1.4)
        retreat = self._line(lower.end, p_preplace, R2, "retreat")
        ret = JointMove(retreat.end, HOME_Q, min_time=1.5)
        # Route checks in the planning world (estimates + proprioception + known static geometry).
        checks = [("approach", approach.samples(50), {}), ("descent", descend.samples(), {})]
        hp, hR = kin.hand_pose(descend.end)
        carry = (hR.T @ (c - hp), hR.T @ rot_z(psi))
        checks += [("lift", lift.samples(), {"carried_part": part.id, "carry": carry, "allow_carried_table": True}),
                   ("transport", transport.samples(60), {"carried_part": part.id, "carry": carry}),
                   ("lowering", lower.samples(), {"carried_part": part.id, "carry": carry, "allow_carried_table": True})]
        for name, qs, kw in checks:
            hit = kin.path_collision(qs, **kw)
            if hit:
                raise TaskFailure("plan", "ROUTE_BLOCKED", f"{name} route is not collision-free: {hit}")
        a = sim.ids.parts[part.id].qadr
        kin.d.qpos[a:a + 3] = place_center
        kin.d.qpos[a + 3:a + 7] = sc.yaw_quat(psi_f)
        for name, qs in (("retreat", retreat.samples()), ("return", ret.samples(50))):
            hit = kin.path_collision(qs)
            if hit:
                raise TaskFailure("plan", "ROUTE_BLOCKED", f"{name} route is not collision-free: {hit}")
        self.plan_info = rec["plan"] = {
            "step": step["index"], "part": part.id, "destination": dest.id, "estimate_m": c.tolist(),
            "part_yaw_rad": psi, "grasp_yaw_rad": g, "place_yaw_rad": g2, "final_part_yaw_rad": psi_f,
            "requested_yaw_rad": dest.yaw, "grasp_width_m": grip_width, "grasp_m": p_grasp.tolist(),
            "place_m": p_place.tolist(), "destination_xy_m": [dest.x, dest.y],
            "checked_segments": ["approach", "descent", "lift", "transport", "lowering", "retreat", "return"],
            "planning_world": ("movable parts from " + e.get("source", "camera_rgbd") + "; table, floor and fixtures "
                               "are known geometry from the scene file (not perceived)")}
        sim.publish_plan(self.plan_info)
        yield

        self._enter("approach")
        yield from self._execute(approach)
        self._enter("descend")
        yield from self._execute(descend)

        self._enter("close")
        sim.set_gripper(GRIPPER_CLOSED)
        yield from self._wait(self._fingers_settled(), 2.0, "close", "GRASP_FAILED", "fingers did not come to rest")
        width = sim.finger_width()
        self.grasp_width = width
        if width > grip_width + WIDTH_TOL:
            raise TaskFailure("close", "GRASP_FAILED", f"gripper did not close: fingers at {1e3 * width:.1f} mm "
                                                       f"({part.name} is {1e3 * grip_width:.0f} mm across the grasp)")
        if width < grip_width - WIDTH_TOL:
            raise TaskFailure("close", "GRASP_FAILED", f"fingers closed to {1e3 * width:.1f} mm: {part.name} is not "
                                                       f"between the fingers (expected {1e3 * grip_width:.0f} mm)")

        def holding():
            w = sim.finger_width()
            if w < grip_width - WIDTH_TOL:
                raise TaskFailure(self.phase, "GRASP_FAILED", f"finger width dropped to {1e3 * w:.1f} mm: "
                                                              f"{part.name} slipped out of the gripper")

        self._enter("lift")
        yield from self._execute(lift, holding)
        self._enter("transport", dest.name)
        yield from self._execute(transport, holding)
        self._enter("lower")
        yield from self._execute(lower, holding)

        self._enter("release")
        sim.set_gripper(GRIPPER_OPEN)
        yield from self._wait(self._fingers_settled(), 2.0, "release", "GRASP_FAILED", "fingers did not open")
        if sim.finger_width() < grip_width + 0.015:
            raise TaskFailure("release", "GRASP_FAILED", f"fingers opened only to {1e3 * sim.finger_width():.1f} mm")

        self._enter("retreat")
        yield from self._execute(retreat)
        yield from self._execute(ret)

        self._enter("verify", "re-observe the scene")
        t_end = sim.data.time + 0.4
        yield from self._wait(lambda: sim.data.time >= t_end, 1.0, "verify", "CONTROL_TIMEOUT", "never")
        after = self._observe_all("verify")
        ve = after[part.id]
        self.verify_estimate = rec["verify_estimate"] = ve
        if not ve["valid"]:
            raise TaskFailure("verify", "PLACE_FAILED", f"could not confirm {part.name} on {dest.name}: "
                              + "; ".join(ve["reasons"]))
        err = float(np.linalg.norm(np.asarray(ve["position_m"][:2]) - [dest.x, dest.y]))
        ve["placement_error_xy_m"] = err
        src = "camera" if self.mode == "camera_rgbd" else "known-world"
        if err > VERIFY_TOL_XY:
            raise TaskFailure("verify", "PLACE_FAILED", f"{src} check: {part.name} is {1e3 * err:.1f} mm from the "
                                                        f"center of {dest.name}")
        if dest.yaw is not None:
            yerr = abs(np.degrees(wrap(ve["yaw_rad"] - dest.yaw, period)))
            ve["yaw_error_deg"] = yerr
            if yerr > VERIFY_TOL_YAW_DEG:
                raise TaskFailure("verify", "PLACE_FAILED", f"{src} check: {part.name} is {yerr:.1f} deg from the "
                                                            f"requested orientation")
        for pid, b in est.items():
            if pid == part.id or not after[pid]["valid"]:
                continue
            moved = float(np.linalg.norm(np.subtract(after[pid]["position_m"][:2], b["position_m"][:2])))
            if moved > DISTURB_TOL:
                raise TaskFailure("verify", "DISTURBED", f"{src} check: {self.cfg.part(pid).name} moved "
                                                         f"{1e3 * moved:.0f} mm during this step")
        ev = sim.evaluator.end_step(sim.data, part.id, (dest.x, dest.y), dest.yaw, period)
        ev["estimate_error_xy_m"] = sim.evaluator.estimate_error(e)
        ev["grasp_width_m"] = width
        rec["metrics"] = ev
        if not ev["placed"]:
            raise TaskFailure("evaluation", "PLACE_FAILED", "evaluator (ground truth): " + "; ".join(ev["reasons"]))
        rec["outcome"] = "success"
        rec["reason"] = f"{part.name} placed on {dest.name}"
        self.phase_log[-1]["t_end"] = sim.data.time
