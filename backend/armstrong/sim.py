"""Simulation worker: the single owner of MjModel/MjData, the controller and the renderer contexts.

HTTP handlers never touch MuJoCo. They submit commands through a bounded queue and read the latest published
snapshot / camera frame / geometry (replaced, never queued). One run at a time.

Status: loading -> idle; idle/completed/failed/stopped -> running <-> paused; running -> completed | failed | stopped.
Physics keeps stepping in every status except paused; pause freezes physics time, `step` advances it by a bounded
number of steps.

Scene changes are accepted only while idle. Pose-only edits write qpos / mocap. Structural changes (parts added or
removed, dimensions, mesh, mass, friction, role, color, destinations) build a complete new model bundle (model,
data, kinematics, controller, evaluator, renderers, exported geometry) and swap it in one step on this thread; if
anything fails the previous scene stays active. Every accepted change increments the scene revision and clears
estimates, plans and recipe validation.
"""

from __future__ import annotations

import copy
import io
import platform
import queue
import threading
import time
import traceback
import uuid
from collections import deque
from concurrent.futures import Future
from dataclasses import asdict

import mujoco
import numpy as np
from PIL import Image

from . import __version__
from . import assets as am
from . import scene as sc
from .camera import RGBDCamera
from .controller import ComputedTorqueController
from .evaluator import Evaluator, part_yaw
from .kinematics import Kinematics
from .perception import SceneObserver
from .robot import GRIPPER_OPEN, HOME_Q
from .task import TaskFailure, WorkflowTask, catalog_entry, compile_recipe, default_recipe, table_info
from .trajectory import Hold

IDLE_STATES = ("idle", "completed", "failed", "stopped")
STRUCTURAL_PART = ("role", "asset_id", "size", "mass", "friction", "color")


class CommandError(Exception):
    def __init__(self, code: str, message: str, details: dict | None = None):
        super().__init__(message)
        self.code, self.message, self.details = code, message, details or {}


def _depth_colormap(depth: np.ndarray, near=0.5, far=1.6) -> np.ndarray:
    t = np.clip((depth - near) / (far - near), 0, 1)
    stops = np.array([[250, 236, 200], [240, 170, 70], [40, 160, 160], [20, 50, 80], [10, 14, 22]], float)
    x = t * (len(stops) - 1)
    i = np.clip(x.astype(int), 0, len(stops) - 2)
    f = (x - i)[..., None]
    rgb = stops[i] * (1 - f) + stops[i + 1] * f
    rgb[depth <= 0] = 0
    return rgb.astype(np.uint8)


class Bundle:
    """Everything that depends on the compiled model. Built off to the side, then swapped in."""

    def __init__(self, cfg: sc.SceneConfig, render: bool):
        self.cfg = cfg
        self.model, self.ids = sc.build_model(cfg)
        self.data = mujoco.MjData(self.model)
        sc.set_initial_state(self.model, self.data, self.ids, cfg)
        self.kin = Kinematics(self.model, self.ids)
        self.ctrl = ComputedTorqueController(self.model, self.ids)
        self.evaluator = Evaluator(self.model, self.ids, cfg)
        self.camera = RGBDCamera(self.model, cfg.camera, self.ids.camera) if render else None
        if self.camera is not None:
            self.camera.capture(self.data, 0)   # fail now (not later) if the renderer cannot handle the scene

    def close(self) -> None:
        if self.camera is not None:
            self.camera.close()


class SimWorker:
    def __init__(self, cfg: sc.SceneConfig | None = None, realtime: bool = True, render: bool = True):
        self.cfg = cfg or sc.SceneConfig()
        self.realtime = realtime
        self.render_enabled = render
        self.commands: queue.Queue = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot: dict = {"status": "loading"}
        self._frame: dict | None = None
        self.status = "loading"
        self.events: deque = deque(maxlen=80)
        self.thread: threading.Thread | None = None
        self.ready = threading.Event()
        self.error: str | None = None
        self.geometry = None
        self.geometry_rev = 0

    # ------------------------------------------------------------------ setup / rebuild (worker thread)

    def _install(self, b: Bundle) -> None:
        self.bundle = b
        self.cfg = b.cfg
        self.model, self.ids, self.data = b.model, b.ids, b.data
        self.kin, self.ctrl, self.evaluator, self.camera = b.kin, b.ctrl, b.evaluator, b.camera
        self.kin.set_world(self.data, {p.id: (np.array([p.x, p.y, p.z]), p.yaw) for p in self.cfg.movable()})
        self._compute_visual_bodies()
        geometry = self.build_geometry()
        with self._lock:
            self.geometry = geometry
            self.geometry_rev += 1
        self._set_hold(HOME_Q)
        self.gripper_cmd = GRIPPER_OPEN

    def _setup(self) -> None:
        t0 = time.perf_counter()
        self.faults = {"camera_masked": False, "gripper_disabled": False}
        self.paused = False
        self.task: WorkflowTask | None = None
        self.task_iter = None
        self.run_id: str | None = None
        self.run_request: dict | None = None
        self.run_wall0 = self.run_sim0 = 0.0
        self.result: dict | None = None
        self.results: dict[str, dict] = {}
        self.trace: list = []
        self.estimates: dict = {}
        self.plan: dict | None = None
        self.last_obs = None
        self.camera_loss_events = 0
        self.scene_revision = 1
        self.step_ms = deque(maxlen=200)
        self.rtf_window: deque = deque(maxlen=60)
        self._install(Bundle(self.cfg, self.render_enabled))
        self.recipe = default_recipe(self.cfg)
        self._validate_recipe()
        self.build_s = time.perf_counter() - t0
        self.status = "idle"
        self.log(f"scene '{self.cfg.name}' compiled in {self.build_s:.2f} s (MuJoCo {mujoco.__version__})")

    def _rebuild(self, cfg: sc.SceneConfig, why: str) -> None:
        t0 = time.perf_counter()
        try:
            b = Bundle(cfg, self.render_enabled)
        except am.AssetError as e:
            raise CommandError("UNSUPPORTED_ASSET", str(e)) from None
        except sc.SceneError as e:
            raise CommandError(e.code, e.message) from None
        except Exception as e:  # compile/renderer failure: keep the previous scene
            raise CommandError("INVALID_SCENE", f"scene could not be built ({type(e).__name__}: {e}); "
                                                f"the previous scene is still active") from None
        old = self.bundle
        self._install(b)
        old.close()
        self.build_s = time.perf_counter() - t0
        self._invalidate()
        self.log(f"scene rebuilt in {self.build_s:.2f} s ({why}); revision {self.scene_revision}")

    def _invalidate(self) -> None:
        self.scene_revision += 1
        self.estimates = {}
        self.plan = None
        self.status = "idle"
        self._validate_recipe()
        if self.camera is not None:
            self.observe()

    def _validate_recipe(self) -> None:
        try:
            steps, _ = compile_recipe(self.cfg, self.recipe)
            self.recipe_status = {"ok": True, "error": None, "code": None, "steps": len(steps),
                                  "revision": self.scene_revision}
        except TaskFailure as f:
            self.recipe_status = {"ok": False, "error": f.reason, "code": f.code, "steps": 0,
                                  "revision": self.scene_revision}

    # ------------------------------------------------------------------ helpers used by the task

    def log(self, msg: str, level: str = "info") -> None:
        t = float(self.data.time) if hasattr(self, "data") else 0.0
        self.events.append({"t_sim": round(t, 3), "t_wall": time.time(), "level": level, "msg": msg})

    def arm_q(self) -> np.ndarray:
        return self.data.qpos[self.ids.arm_qadr].copy()

    def arm_qvel(self) -> np.ndarray:
        return self.data.qvel[self.ids.arm_dadr].copy()

    def finger_width(self) -> float:
        return float(self.data.qpos[self.ids.finger_qadr].sum())

    def set_gripper(self, value: float) -> None:
        if self.faults["gripper_disabled"] and value < GRIPPER_OPEN:
            self.log("fault injection: gripper close command ignored (gripper disabled)", "warn")
            return
        self.gripper_cmd = float(value)

    def _set_hold(self, q) -> None:
        self.segment = Hold(q)
        self.seg_t0 = float(self.data.time)

    def set_reference(self, seg) -> None:
        self.segment = seg
        self.seg_t0 = float(self.data.time)

    def reference_finished(self) -> bool:
        return self.data.time - self.seg_t0 >= self.segment.duration

    def reference_end(self) -> np.ndarray:
        return np.asarray(self.segment.end, float).copy()

    def observe(self):
        if self.camera is None:
            raise TaskFailure("observe", "PERCEPTION_UNAVAILABLE", "rendering disabled: no camera available")
        self.camera.masked = self.faults["camera_masked"]
        obs = self.camera.capture(self.data, self.scene_revision)
        self._publish_frame(obs)
        if obs.masked and self.task_iter is not None:
            self.camera_loss_events += 1
        return obs

    def truth_part_pose(self, pid: str):
        """Simulator truth: known-world development mode, display and evaluation only."""
        b = self.ids.parts[pid].body
        return self.data.xpos[b].copy(), part_yaw(self.data.xmat[b].reshape(3, 3))

    def publish_estimates(self, est: dict) -> None:
        self.estimates = est

    def publish_plan(self, plan: dict) -> None:
        self.plan = plan

    # ------------------------------------------------------------------ physics

    def _physics_step(self) -> None:
        t0 = time.perf_counter()
        d = self.data
        if self.task_iter is not None:
            try:
                next(self.task_iter)
            except StopIteration:
                self._finish_run(None)
            except TaskFailure as f:
                self._finish_run(f)
            except Exception as exc:  # keep the worker alive and report the bug as a failed run
                self.log(traceback.format_exc(limit=4), "error")
                self._finish_run(TaskFailure(self.task.phase if self.task else "run", "BACKEND_ERROR",
                                             f"{type(exc).__name__}: {exc}"))
        q_ref, qd_ref, qdd_ref = self.segment.sample(d.time - self.seg_t0)
        tau = self.ctrl.compute(d, q_ref, qd_ref, qdd_ref, self.model.opt.timestep)
        d.ctrl[self.ids.arm_act] = tau
        d.ctrl[self.ids.grip_act] = self.gripper_cmd
        mujoco.mj_step(self.model, d)
        if self.task is not None and self.task_iter is not None:
            self.evaluator.step(d, self.task.phase)
            if len(self.trace) < 40000 and int(round(d.time / self.model.opt.timestep)) % 25 == 0:
                row = [round(float(d.time), 3), self.task.current, self.task.phase,
                       *np.round(self.arm_q(), 5).tolist(), round(self.finger_width(), 5)]
                for p in self.cfg.movable():
                    row += np.round(d.xpos[self.ids.parts[p.id].body], 5).tolist()
                self.trace.append(row)
        self.step_ms.append(1e3 * (time.perf_counter() - t0))

    # ------------------------------------------------------------------ run lifecycle

    def _request_recipe(self, request: dict) -> dict:
        if request.get("recipe") is not None:
            return request["recipe"]
        src, dst = request.get("source_object_ids"), request.get("target_object_id")
        if src is not None or dst is not None:   # v0.1 request format: one source, one destination
            if not isinstance(src, list) or len(src) != 1 or not dst:
                raise CommandError("UNSUPPORTED_TASK", "a single pick-and-place needs exactly one source and one "
                                                       "destination")
            return {"version": 1, "name": "Move part", "steps": [{"id": "s1", "part": src[0], "action": "place",
                                                                  "dest": dst}]}
        return self.recipe

    def _start_run(self, request: dict) -> dict:
        if self.status not in IDLE_STATES:
            raise CommandError("BUSY", f"a run is already {self.status}")
        if request.get("type", "pick_place") not in ("pick_place", "workflow"):
            raise CommandError("UNSUPPORTED_TASK", f"task type '{request.get('type')}' is not implemented")
        rev = request.get("scene_revision")
        if rev is not None and int(rev) != self.scene_revision:
            raise CommandError("STALE_SCENE", f"task refers to scene revision {rev}; current revision is "
                                              f"{self.scene_revision}", {"current_revision": self.scene_revision})
        mode = request.get("perception_mode", "camera_rgbd")
        if mode not in ("camera_rgbd", "known_world"):
            raise CommandError("UNSUPPORTED_TASK", f"perception mode '{mode}' is not implemented")
        recipe = self._request_recipe(request)
        try:
            self.task = WorkflowTask(self, {"perception_mode": mode, "recipe": recipe})
        except TaskFailure as f:
            raise CommandError(f.code, f.reason) from None
        self.task_iter = self.task.program()
        self.run_id = uuid.uuid4().hex[:8]
        self.run_request = {"perception_mode": mode, "scene_revision": self.scene_revision,
                            "recipe": copy.deepcopy(recipe), "type": "workflow"}
        self.run_wall0, self.run_sim0 = time.perf_counter(), float(self.data.time)
        self.evaluator.reset()
        self.ctrl.saturated_steps = 0
        self.camera_loss_events = 0
        self.trace = []
        self.estimates, self.plan = {}, None
        self.result = None
        self.paused = False
        self.status = "running"
        n = len(self.task.steps)
        self.log(f"run {self.run_id} started: {n} step{'s' * (n != 1)} "
                 f"({'camera RGB-D' if mode == 'camera_rgbd' else 'KNOWN-WORLD dev'} mode)")
        return {"run_id": self.run_id}

    def _finish_run(self, failure: TaskFailure | None, stopped: bool = False) -> None:
        task = self.task
        if task is None:
            return
        self.task_iter = None
        self._set_hold(self.arm_q() if (failure or stopped) else self.reference_end())
        if task.phase_log and task.phase_log[-1]["t_end"] is None:
            task.phase_log[-1]["t_end"] = float(self.data.time)
        recs = task.records
        if stopped and recs and recs[-1]["outcome"] == "running":
            recs[-1].update(outcome="stopped", failure_stage=task.phase, failure_code="STOPPED",
                            reason="stopped by the user")
            task._close(recs[-1])
        done = {r["index"] for r in recs}
        steps = [*recs] + [{"index": s["index"], "step_id": s["id"], "part_id": s["part"],
                            "part_name": self.cfg.part(s["part"]).name, "action": s["action"], "dest_id": s["dest"],
                            "dest_name": self.cfg.dest(s["dest"]).name if s["dest"] else None, "zone": s["zone"],
                            "outcome": "not_run", "reason": "not started (an earlier step did not succeed)"}
                           for s in task.steps if s["index"] not in done]
        ok = sum(1 for r in recs if r["outcome"] == "success")
        if stopped:
            outcome, stage, code, reason = "stopped", task.phase, "STOPPED", "stopped by the user"
        elif failure is not None:
            outcome, stage, code, reason = "failure", failure.stage, failure.code, failure.reason
        elif self.evaluator.forbidden:
            outcome, stage, code = "failure", "evaluation", "CONTACT_VIOLATION"
            reason = (f"evaluator: {self.evaluator.forbidden} steps with forbidden contacts "
                      f"(first: {self.evaluator.forbidden_first})")
        else:
            outcome, stage, code = "success", None, None
            reason = f"all {len(task.steps)} step{'s' * (len(task.steps) != 1)} completed"
        last = recs[-1] if recs else {}
        metrics = {**(last.get("metrics") or {}), "steps_total": len(task.steps), "steps_completed": ok,
                   "forbidden_contacts": self.evaluator.forbidden,
                   "first_forbidden_contact": self.evaluator.forbidden_first,
                   "camera_loss_events": self.camera_loss_events, "torque_saturation_steps": self.ctrl.saturated_steps,
                   "mpc_fallbacks": None}
        metrics.setdefault("grasp_width_m", task.grasp_width)
        res = {
            "run_id": self.run_id, "outcome": outcome, "failure_stage": stage, "failure_code": code, "reason": reason,
            "failed_step": next((r["index"] for r in recs if r["outcome"] in ("failure", "stopped")), None),
            "task_request": self.run_request, "scene_revision": self.scene_revision, "scene_name": self.cfg.name,
            "perception_mode": task.mode, "actual_controller": self.ctrl.name, "controller_fallbacks": None,
            "versions": {"armstrong": __version__, "mujoco": mujoco.__version__, "python": platform.python_version()},
            "wall_duration_s": time.perf_counter() - self.run_wall0,
            "sim_duration_s": float(self.data.time) - self.run_sim0,
            "steps": steps, "phases": task.phase_log, "estimate": last.get("estimate"),
            "verify_estimate": last.get("verify_estimate"), "plan": last.get("plan"), "metrics": metrics,
            "faults": dict(self.faults),
            "timing_note": "simulated durations of this model and controller, not a real-cell throughput claim",
        }
        self.result = res
        cols = ["t", "step", "phase", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "gripper_width_m"]
        for p in self.cfg.movable():
            cols += [f"{p.id}_truth_x", f"{p.id}_truth_y", f"{p.id}_truth_z"]
        self.results[self.run_id] = {"result": res, "trace": self.trace, "trace_columns": cols,
                                     "scene": self.cfg.to_dict()}
        while len(self.results) > 20:
            self.results.pop(next(iter(self.results)))
        self.status = {"success": "completed", "failure": "failed", "stopped": "stopped"}[outcome]
        self.paused = False
        lvl = "info" if outcome == "success" else "warn"
        self.log(f"run {self.run_id} {outcome}: {ok}/{len(task.steps)} steps" +
                 (f"; stopped at step {res['failed_step']} {stage}: {reason}" if outcome != "success" else ""), lvl)

    # ------------------------------------------------------------------ commands

    def submit(self, name: str, **kwargs) -> Future:
        fut: Future = Future()
        try:
            self.commands.put_nowait((name, kwargs, fut))
        except queue.Full:
            fut.set_exception(CommandError("BUSY", "command queue full"))
        return fut

    def _require_idle(self, what: str) -> None:
        if self.status not in IDLE_STATES:
            raise CommandError("BUSY", f"{what} accepted only while stopped (status: {self.status})")

    def _check_revision(self, exp) -> None:
        if exp is not None and int(exp) != self.scene_revision:
            raise CommandError("STALE_SCENE", f"change based on revision {exp}; current is {self.scene_revision}",
                               {"current_revision": self.scene_revision})

    def _handle(self, name: str, kw: dict):
        if name == "run":
            return self._start_run(kw.get("request") or {})
        if name == "pause":
            if self.status != "running":
                raise CommandError("BUSY", f"cannot pause while {self.status}")
            self.paused, self.status = True, "paused"
            self.log("paused (physics time frozen)")
            return {}
        if name == "resume":
            if self.status != "paused":
                raise CommandError("BUSY", f"cannot resume while {self.status}")
            self.paused, self.status = False, "running"
            self.log("resumed")
            return {}
        if name == "step":
            if self.status != "paused":
                raise CommandError("BUSY", "single-stepping is available while paused")
            n = int(np.clip(int(kw.get("steps", 25)), 1, 500))
            for _ in range(n):
                self._physics_step()
                if self.status != "paused":
                    break
            return {"steps": n, "sim_time_s": float(self.data.time)}
        if name == "stop":
            if self.status in ("running", "paused"):
                self._finish_run(None, stopped=True)
            else:
                self._set_hold(self.arm_q())
            return {}
        if name == "reset":
            if kw.get("default_scene"):
                self._abort_if_running()
                self._rebuild(sc.SceneConfig(), "demo scene loaded")
                self.recipe = default_recipe(self.cfg)
                self._validate_recipe()
                return {"scene_revision": self.scene_revision}
            self._reset_scene()
            return {"scene_revision": self.scene_revision}
        if name == "edit":
            return self._edit(kw)
        if name == "set_scene":
            self._require_idle("scene changes are")
            self._check_revision(kw.get("expected_revision"))
            try:
                cfg = sc.SceneConfig.from_dict(kw.get("scene"))
            except sc.SceneError as e:
                raise CommandError(e.code, e.message) from None
            except am.AssetError as e:
                raise CommandError("UNSUPPORTED_ASSET", str(e)) from None
            self._check_layout(cfg)
            self._rebuild(cfg, kw.get("why") or "scene updated")
            return {"scene_revision": self.scene_revision}
        if name == "set_recipe":
            self._require_idle("workflow changes are")
            rec = kw.get("recipe")
            if not isinstance(rec, dict) or not isinstance(rec.get("steps", []), list):
                raise CommandError("INVALID_RECIPE", "workflow must be an object with a list of steps")
            self.recipe = {"version": 1, "name": str(rec.get("name") or "Workflow")[:60],
                           "steps": rec.get("steps") or [], "sort_rules": rec.get("sort_rules") or []}
            self._validate_recipe()
            return {"recipe_status": self.recipe_status}
        if name == "load_project":
            self._require_idle("projects can be opened")
            proj = kw["project"]
            try:
                cfg = sc.SceneConfig.from_dict(proj.get("scene"))
            except sc.SceneError as e:
                raise CommandError(e.code, e.message) from None
            except am.AssetError as e:
                raise CommandError("UNSUPPORTED_ASSET", str(e)) from None
            self._check_layout(cfg)
            self._rebuild(cfg, f"project '{cfg.name}' opened")
            self.recipe = proj.get("recipe") or default_recipe(cfg)
            self._validate_recipe()
            return {"scene_revision": self.scene_revision, "recipe_status": self.recipe_status}
        if name == "faults":
            for k in ("camera_masked", "gripper_disabled"):
                if k in kw and kw[k] is not None:
                    self.faults[k] = bool(kw[k])
            self.log(f"faults: {self.faults}", "warn" if any(self.faults.values()) else "info")
            return dict(self.faults)
        if name == "observe":
            if self.status in ("running", "paused"):
                raise CommandError("BUSY", "observation preview is available while idle")
            obs = self.observe()
            observer = SceneObserver(obs, table_info())
            self.estimates = {p.id: observer.estimate(catalog_entry(p), self.scene_revision)
                              for p in self.cfg.movable()}
            return {"estimates": self.estimates}
        raise CommandError("BACKEND_ERROR", f"unknown command {name}")

    def _abort_if_running(self):
        if self.status in ("running", "paused"):
            self._finish_run(None, stopped=True)

    def _reset_scene(self) -> None:
        self._abort_if_running()
        sc.set_initial_state(self.model, self.data, self.ids, self.cfg)
        self.ctrl.reset()
        self._set_hold(HOME_Q)
        self.gripper_cmd = GRIPPER_OPEN
        self._invalidate()
        self.log(f"scene reset (revision {self.scene_revision})")

    def _check_layout(self, cfg: sc.SceneConfig, only: set | None = None) -> None:
        """Every collidable part on the table, clear of the robot base and of other parts."""
        solid = [p for p in cfg.parts if p.role != "visual"]
        for p in cfg.parts:
            if only is not None and p.id not in only:
                continue
            half = 0.5 * float(np.hypot(p.size[0], p.size[1]))
            if not sc.on_table(p.x, p.y, min(half, 0.04)):
                raise CommandError("OUT_OF_WORKSPACE", f"{p.name} at ({p.x:.3f}, {p.y:.3f}) m is not on the table")
            if p.role != "visual" and np.hypot(p.x, p.y) < sc.BASE_KEEPOUT + half:
                raise CommandError("OUT_OF_WORKSPACE", f"{p.name} would overlap the robot base")
        for i, a in enumerate(solid):
            for b in solid[i + 1:]:
                if only is not None and a.id not in only and b.id not in only:
                    continue
                if sc.rects_overlap((a.x, a.y), a.size, a.yaw, (b.x, b.y), b.size, b.yaw, 0.002):
                    raise CommandError("INVALID_SCENE", f"{a.name} and {b.name} would overlap")
        for d in cfg.destinations:
            if not sc.on_table(d.x, d.y, 0.03):
                raise CommandError("OUT_OF_WORKSPACE", f"destination {d.name} is not on the table")

    def _edit(self, kw: dict) -> dict:
        """Pose-only edits while stopped: {"parts": {id: {x, y, yaw}}, "dests": {id: {x, y, yaw}}}.
        The v0.1 form {"block": {...}, "pad": {...}} addresses the first movable part / first destination."""
        self._require_idle("scene edits are")
        self._check_revision(kw.get("expected_revision"))
        parts = dict(kw.get("parts") or {})
        dests = dict(kw.get("dests") or {})
        if kw.get("block") and self.cfg.movable():
            parts[self.cfg.movable()[0].id] = kw["block"]
        if kw.get("pad") and self.cfg.destinations:
            dests[self.cfg.destinations[0].id] = kw["pad"]
        if not parts and not dests:
            raise CommandError("INVALID_SCENE", "nothing to edit")
        cfg = copy.deepcopy(self.cfg)
        for pid, v in parts.items():
            p = cfg.part(pid)
            if p is None:
                raise CommandError("INVALID_SCENE", f"unknown part {pid!r}")
            p.x, p.y, p.yaw = float(v.get("x", p.x)), float(v.get("y", p.y)), float(v.get("yaw", p.yaw))
            if not all(np.isfinite([p.x, p.y, p.yaw])):
                raise CommandError("INVALID_SCENE", f"{p.name}: pose must be finite numbers")
        for did, v in dests.items():
            d = cfg.dest(did)
            if d is None:
                raise CommandError("INVALID_SCENE", f"unknown destination {did!r}")
            d.x, d.y = float(v.get("x", d.x)), float(v.get("y", d.y))
            if "yaw" in v:
                d.yaw = None if v["yaw"] is None else float(v["yaw"])
            if not all(np.isfinite([d.x, d.y])) or (d.yaw is not None and not np.isfinite(d.yaw)):
                raise CommandError("INVALID_SCENE", f"destination {d.name}: pose must be finite numbers")
        self._check_layout(cfg, only=set(parts) | set(dests))
        self.cfg.parts, self.cfg.destinations = cfg.parts, cfg.destinations
        self.bundle.cfg = self.evaluator.cfg = self.cfg
        for pid in parts:
            sc.place_part(self.data, self.ids, self.cfg.part(pid))
        for did in dests:
            sc.place_dest(self.data, self.ids, self.cfg.dest(did))
        mujoco.mj_forward(self.model, self.data)
        self._invalidate()
        moved = ", ".join([self.cfg.part(p).name for p in parts] + [self.cfg.dest(d).name for d in dests])
        self.log(f"scene edited (revision {self.scene_revision}): {moved}")
        return {"scene_revision": self.scene_revision}

    def _drain(self) -> None:
        while True:
            try:
                name, kw, fut = self.commands.get_nowait()
            except queue.Empty:
                return
            if fut.set_running_or_notify_cancel():
                try:
                    res = self._handle(name, kw)
                    self._publish_snapshot()   # clients reading /api/state after the reply see the new state
                    fut.set_result(res)
                except CommandError as e:
                    fut.set_exception(e)
                except Exception as e:
                    self.log(f"command {name} failed: {e}", "error")
                    fut.set_exception(CommandError("BACKEND_ERROR", f"{type(e).__name__}: {e}"))

    # ------------------------------------------------------------------ publishing

    def _publish_frame(self, obs) -> None:
        buf = io.BytesIO()
        Image.fromarray(obs.rgb).save(buf, format="JPEG", quality=88)
        dbuf = io.BytesIO()
        Image.fromarray(_depth_colormap(obs.depth_m)).save(dbuf, format="PNG")
        self.last_obs = obs
        with self._lock:
            self._frame = {"meta": obs.describe(), "rgb": buf.getvalue(), "depth": dbuf.getvalue(),
                           "captured_monotonic_s": obs.captured_monotonic_s}

    def _publish_snapshot(self) -> None:
        d, m = self.data, self.model
        poses = {}
        for b in self._visual_bodies:
            poses[m.body(b).name] = [*np.round(d.xpos[b], 5).tolist(), *np.round(d.xquat[b], 6).tolist()]
        truth = {}
        for p in self.cfg.movable():
            tp, ty = self.truth_part_pose(p.id)
            truth[p.id] = {"position_m": tp.tolist(), "yaw_rad": ty}
        frame_meta = None
        with self._lock:
            if self._frame is not None:
                frame_meta = {**self._frame["meta"], "age_s": time.monotonic() - self._frame["captured_monotonic_s"]}
        running = self.task_iter is not None and self.task is not None
        progress = None
        if self.task is not None:
            progress = {"current_step": (self.task.current + 1) if (running and self.task.current is not None) else None,
                        "total": len(self.task.steps),
                        "completed": sum(1 for r in self.task.records if r["outcome"] == "success"),
                        "records": [{k: r.get(k) for k in ("index", "part_id", "part_name", "dest_id", "dest_name",
                                                           "outcome", "reason")} for r in self.task.records]}
        snap = {
            "status": self.status, "phase": self.task.phase if running else None,
            "run_id": self.run_id, "scene_revision": self.scene_revision, "geometry_rev": self.geometry_rev,
            "sim_time_s": float(d.time), "realtime_factor": self._rtf(),
            "controller": {"name": self.ctrl.name, "key": self.ctrl.key, "gains": self.ctrl.gains(),
                           "rate_hz": 1.0 / m.opt.timestep, "fallback": None,
                           "model": "contact-free inverse dynamics (payload unmodelled)"},
            "q": np.round(self.arm_q(), 5).tolist(), "qdot": np.round(self.arm_qvel(), 4).tolist(),
            "gripper_width_m": self.finger_width(), "gripper_command": self.gripper_cmd,
            "poses": poses, "truth": truth, "truth_label": "simulator truth (display/evaluation only)",
            "scene": self.cfg.to_dict(), "faults": dict(self.faults),
            "estimates": self.estimates, "plan": self.plan, "camera": frame_meta,
            "recipe": self.recipe, "recipe_status": self.recipe_status, "progress": progress,
            "events": list(self.events)[-40:], "result": self.result,
            "timing": {"physics_dt_s": m.opt.timestep, "step_ms_mean": float(np.mean(self.step_ms)) if self.step_ms else None,
                       "build_s": self.build_s},
            "phases": self.task.phase_log[-14:] if self.task else [],
        }
        with self._lock:
            self._snapshot = snap

    def _rtf(self):
        w = self.rtf_window
        if len(w) < 2 or w[-1][0] - w[0][0] < 0.2:
            return None
        return (w[-1][1] - w[0][1]) / (w[-1][0] - w[0][0])

    def snapshot(self) -> dict:
        with self._lock:
            return self._snapshot

    def frame(self) -> dict | None:
        with self._lock:
            return self._frame

    def geometry_pair(self):
        with self._lock:
            return self.geometry, self.geometry_rev

    # ------------------------------------------------------------------ geometry export

    def _compute_visual_bodies(self):
        m = self.model
        vis = set()
        for g in range(m.ngeom):
            if self._is_visual(g) or self._is_proxy(g):
                vis.add(int(m.geom_bodyid[g]))
        vis.discard(0)
        self._visual_bodies = sorted(vis)

    def _is_visual(self, g: int) -> bool:
        m = self.model
        group = m.geom_group[g]
        if m.geom_bodyid[g] in set(self.ids.robot_bodies.tolist()):
            return group == 2
        return group in (0, 1, 2) and (m.geom_rgba[g][3] > 0 or m.geom_matid[g] >= 0)

    def _is_proxy(self, g: int) -> bool:
        m = self.model
        return m.geom_group[g] == 3 and m.geom(g).name.startswith("part_")

    def build_geometry(self) -> tuple[dict, bytes]:
        """Visual geoms (plus part collision proxies, flagged) in MuJoCo's compiled frames + one binary blob of
        non-indexed mesh triangles (float32 positions then float32 normals per mesh, mesh frame = compiled geom frame)."""
        m = self.model
        geoms, meshes, chunks = [], {}, []
        offset = 0
        for g in range(m.ngeom):
            proxy = self._is_proxy(g)
            if not (self._is_visual(g) or proxy):
                continue
            mat = m.geom_matid[g]
            rgba = (m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]).tolist()
            gtype = int(m.geom_type[g])
            kind = {mujoco.mjtGeom.mjGEOM_BOX: "box", mujoco.mjtGeom.mjGEOM_PLANE: "plane",
                    mujoco.mjtGeom.mjGEOM_SPHERE: "sphere", mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
                    mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule", mujoco.mjtGeom.mjGEOM_MESH: "mesh"}.get(gtype)
            if kind is None:
                continue
            item = {"id": m.geom(g).name or f"geom{g}", "body": m.body(m.geom_bodyid[g]).name, "kind": kind,
                    "size": m.geom_size[g].tolist(), "pos": m.geom_pos[g].tolist(), "quat_wxyz": m.geom_quat[g].tolist(),
                    "rgba": rgba, "specular": float(m.mat_specular[mat]) if mat >= 0 else 0.3, "proxy": proxy}
            if kind == "mesh":
                mid = int(m.geom_dataid[g])
                if mid not in meshes:
                    va, fa, fn = m.mesh_vertadr[mid], m.mesh_faceadr[mid], m.mesh_facenum[mid]
                    verts = m.mesh_vert[va:va + m.mesh_vertnum[mid]]
                    faces = m.mesh_face[fa:fa + fn]
                    na = m.mesh_normaladr[mid]
                    normals = m.mesh_normal[na:na + m.mesh_normalnum[mid]]
                    fnorm = m.mesh_facenormal[fa:fa + fn]
                    pos = verts[faces.reshape(-1)].astype(np.float32)
                    nor = normals[fnorm.reshape(-1)].astype(np.float32)
                    blob = pos.tobytes() + nor.tobytes()
                    meshes[mid] = {"offset": offset, "count": int(len(pos))}
                    chunks.append(blob)
                    offset += len(blob)
                item["mesh"] = meshes[mid]
            geoms.append(item)
        parts = [{"id": p.id, "name": p.name, "role": p.role, "body": f"part_{p.id}", "size_m": p.size,
                  "asset_id": p.asset_id, "color": p.color, "category": p.category,
                  "approximation": ("box collision proxy of the uploaded mesh (approximate physics)" if p.asset_id
                                    else "primitive box") if p.role != "visual" else "visual only (no collision)"}
                 for p in self.cfg.parts]
        dests = [{"id": d.id, "name": d.name, "body": f"dest_{d.id}", "zone": d.zone, "yaw": d.yaw}
                 for d in self.cfg.destinations]
        return {"geoms": geoms, "bodies": [m.body(b).name for b in self._visual_bodies], "parts": parts,
                "destinations": dests, "units": "m", "up": "+Z", "quaternion_order": "wxyz",
                "geometry_rev": self.geometry_rev + 1, "scene_revision": getattr(self, "scene_revision", 1),
                "camera": {"id": self.cfg.camera.id, "position": list(self.cfg.camera.position),
                           "target": list(self.cfg.camera.target), "vertical_fov_deg": self.cfg.camera.vertical_fov_deg},
                "table": {"top_z": sc.TABLE_TOP_Z, "center_xy": list(sc.TABLE_CENTER_XY), "half_xy": list(sc.TABLE_HALF_XY)},
                "workspace": {"reach_min": sc.REACH_MIN, "reach_max": sc.REACH_MAX, "abs_y_max": sc.WORKSPACE_Y},
                "mesh_bytes": offset}, b"".join(chunks)

    # ------------------------------------------------------------------ main loop

    def run_forever(self) -> None:
        try:
            self._setup()
            if self.camera is not None:
                self.observe()
            self._publish_snapshot()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.status = "error"
            with self._lock:
                self._snapshot = {"status": "error", "error": self.error}
            self.ready.set()
            raise
        self.ready.set()
        dt = self.model.opt.timestep
        anchor_wall, anchor_sim = time.perf_counter(), float(self.data.time)
        last_cam = last_snap = 0.0
        cam_period = 1.0 / self.cfg.camera.rate_hz
        try:
            while not self._stop.is_set():
                model_before = self.model
                self._drain()
                now = time.perf_counter()
                if self.model is not model_before:   # rebuilt: new data, new clock
                    anchor_wall, anchor_sim = now, float(self.data.time)
                if self.paused:
                    anchor_wall, anchor_sim = now, float(self.data.time)
                else:
                    target = anchor_sim + (now - anchor_wall)
                    if target - self.data.time > 0.1:      # fell behind (planning): re-anchor, never fast-forward
                        anchor_wall, anchor_sim = now, float(self.data.time)
                        target = anchor_sim + 0.002
                    n = 0
                    while self.data.time + 0.5 * dt < target and n < 60 and not self.paused:
                        self._physics_step()
                        n += 1
                self.rtf_window.append((time.perf_counter(), float(self.data.time)))
                if self.camera is not None and now - last_cam >= cam_period:
                    last_cam = now
                    self.observe()
                if now - last_snap >= 1 / 30:
                    last_snap = now
                    self._publish_snapshot()
                time.sleep(0.002)
        finally:
            self.bundle.close()

    def start_thread(self) -> None:
        self.thread = threading.Thread(target=self.run_forever, name="armstrong-sim", daemon=True)
        self.thread.start()
        self.ready.wait(60)

    def shutdown(self) -> None:
        self._stop.set()
        if self.thread is not None:
            self.thread.join(5)

    # ------------------------------------------------------------------ synchronous driver (tests/CLI)

    def setup_sync(self) -> None:
        self._setup()

    def run_sync(self, request: dict, max_sim_s: float = 240.0) -> dict:
        """Start a run and step as fast as possible until it ends (same code path as the live loop)."""
        self._start_run(request)
        t_end = self.data.time + max_sim_s
        while self.task_iter is not None and self.data.time < t_end:
            self._physics_step()
        if self.task_iter is not None:
            self._finish_run(TaskFailure(self.task.phase, "CONTROL_TIMEOUT", f"run exceeded {max_sim_s:.0f} s"))
        return self.result

    def command_sync(self, name: str, **kw):
        return self._handle(name, kw)
