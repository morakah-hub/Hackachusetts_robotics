"""Simulation worker: the single owner of MjModel/MjData, the controller and the renderer contexts.

HTTP handlers never touch MuJoCo. They submit commands through a bounded queue and read the
latest published snapshot / camera frame (both replaced, never queued). One run at a time.

Status: loading -> idle; idle/completed/failed/stopped -> running <-> paused; running -> completed |
failed | stopped. Physics keeps stepping in every status except paused (the arm holds its last
reference); pause freezes physics time, `step` advances it by a bounded number of steps.
"""

from __future__ import annotations

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
from . import scene as sc
from .camera import RGBDCamera
from .controller import ComputedTorqueController
from .evaluator import Evaluator
from .kinematics import Kinematics
from .perception import estimate_block
from .robot import GRIPPER_CLOSED, GRIPPER_OPEN, HOME_Q
from .task import PickPlaceTask, TaskFailure
from .trajectory import Hold

IDLE_STATES = ("idle", "completed", "failed", "stopped")


class CommandError(Exception):
    def __init__(self, code: str, message: str, details: dict | None = None):
        super().__init__(message)
        self.code, self.message, self.details = code, message, details or {}


def _depth_colormap(depth: np.ndarray, near=0.5, far=1.6) -> np.ndarray:
    t = np.clip((depth - near) / (far - near), 0, 1)
    # compact teal-to-amber ramp (near = bright)
    stops = np.array([[250, 236, 200], [240, 170, 70], [40, 160, 160], [20, 50, 80], [10, 14, 22]], float)
    x = t * (len(stops) - 1)
    i = np.clip(x.astype(int), 0, len(stops) - 2)
    f = (x - i)[..., None]
    rgb = stops[i] * (1 - f) + stops[i + 1] * f
    rgb[depth <= 0] = 0
    return rgb.astype(np.uint8)


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
        self.events: deque = deque(maxlen=60)
        self.thread: threading.Thread | None = None
        self.ready = threading.Event()
        self.error: str | None = None

    # ------------------------------------------------------------------ setup (worker thread)

    def _setup(self) -> None:
        t0 = time.perf_counter()
        self.model, self.ids = sc.build_model(self.cfg)
        self.data = mujoco.MjData(self.model)
        sc.set_initial_state(self.model, self.data, self.ids, self.cfg)
        self.kin = Kinematics(self.model, self.ids)
        self.kin.load(self.data.qpos)
        self.ctrl = ComputedTorqueController(self.model, self.ids)
        self.evaluator = Evaluator(self.model, self.ids, self.cfg.block.size[2] / 2)
        self.camera = RGBDCamera(self.model, self.cfg.camera, self.ids.camera) if self.render_enabled else None
        self.scene_revision = 1
        self.faults = {"camera_masked": False, "gripper_disabled": False}
        self.paused = False
        self.task: PickPlaceTask | None = None
        self.task_iter = None
        self.run_id: str | None = None
        self.run_request: dict | None = None
        self.run_wall0 = self.run_sim0 = 0.0
        self.result: dict | None = None
        self.results: dict[str, dict] = {}
        self.trace: list = []
        self.estimate: dict | None = None
        self.plan: dict | None = None
        self.last_obs = None
        self.camera_loss_events = 0
        self._set_hold(HOME_Q)
        self.gripper_cmd = GRIPPER_OPEN
        self.step_ms = deque(maxlen=200)
        self.rtf_window: deque = deque(maxlen=60)
        self.build_s = time.perf_counter() - t0
        self.status = "idle"
        self.log(f"scene compiled in {self.build_s:.2f} s (MuJoCo {mujoco.__version__})")

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
        if obs.masked:
            self.camera_loss_events += 1
        return obs

    def truth_block_pose(self):
        """Simulator truth: known-world development mode, display and evaluation only."""
        p = self.data.xpos[self.ids.block_body].copy()
        R = self.data.xmat[self.ids.block_body].reshape(3, 3)
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        yaw = (yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
        return p, yaw

    def publish_estimate(self, est: dict) -> None:
        self.estimate = est

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
                self._finish_run(TaskFailure(self.task.phase if self.task else "run", "BACKEND_ERROR",
                                             f"{type(exc).__name__}: {exc}"))
                self.log(traceback.format_exc(limit=3), "error")
        q_ref, qd_ref, qdd_ref = self.segment.sample(d.time - self.seg_t0)
        tau = self.ctrl.compute(d, q_ref, qd_ref, qdd_ref, self.model.opt.timestep)
        d.ctrl[self.ids.arm_act] = tau
        d.ctrl[self.ids.grip_act] = self.gripper_cmd
        mujoco.mj_step(self.model, d)
        if self.task is not None and self.task_iter is not None:
            self.evaluator.step(d, self.task.phase)
            if len(self.trace) < 20000 and int(round(d.time / self.model.opt.timestep)) % 25 == 0:
                bp = d.xpos[self.ids.block_body]
                self.trace.append([round(float(d.time), 3), self.task.phase, *np.round(self.arm_q(), 5).tolist(),
                                   round(self.finger_width(), 5), *np.round(bp, 5).tolist()])
        self.step_ms.append(1e3 * (time.perf_counter() - t0))

    # ------------------------------------------------------------------ run lifecycle

    def _start_run(self, request: dict) -> dict:
        if self.status not in IDLE_STATES:
            raise CommandError("BUSY", f"a run is already {self.status}")
        if request.get("type", "pick_place") != "pick_place":
            raise CommandError("UNSUPPORTED_TASK", f"task type '{request.get('type')}' is not implemented")
        if request.get("source_object_ids", ["block"]) != ["block"] or request.get("target_object_id", "pad") != "pad":
            raise CommandError("UNSUPPORTED_TASK", "this scene supports exactly one source (block) and one "
                                                   "destination (pad)")
        rev = request.get("scene_revision")
        if rev is not None and int(rev) != self.scene_revision:
            raise CommandError("STALE_SCENE", f"task refers to scene revision {rev}; current revision is "
                                              f"{self.scene_revision}", {"current_revision": self.scene_revision})
        mode = request.get("perception_mode", "camera_rgbd")
        if mode not in ("camera_rgbd", "known_world"):
            raise CommandError("UNSUPPORTED_TASK", f"perception mode '{mode}' is not implemented")
        self.task = PickPlaceTask(self, {**request, "perception_mode": mode})
        self.task_iter = self.task.program()
        self.run_id = uuid.uuid4().hex[:8]
        self.run_request = {**request, "perception_mode": mode, "scene_revision": self.scene_revision}
        self.run_wall0, self.run_sim0 = time.perf_counter(), float(self.data.time)
        self.evaluator.reset(self.data)
        self.block_start = self.data.xpos[self.ids.block_body].copy()  # evaluation only
        self.ctrl.saturated_steps = 0
        self.camera_loss_events = 0
        self.trace = []
        self.estimate = self.plan = None
        self.result = None
        self.paused = False
        self.status = "running"
        self.log(f"run {self.run_id} started ({'camera RGB-D' if mode == 'camera_rgbd' else 'KNOWN-WORLD dev'} mode)")
        return {"run_id": self.run_id}

    def _finish_run(self, failure: TaskFailure | None, stopped: bool = False) -> None:
        task = self.task
        if task is None:
            return
        self.task_iter = None
        self._set_hold(self.arm_q() if (failure or stopped) else self.reference_end())
        if task.phase_log and task.phase_log[-1]["t_end"] is None:
            task.phase_log[-1]["t_end"] = float(self.data.time)
        ev = self.evaluator.final(self.data, task.pad_xy)
        if stopped:
            outcome, stage, code, reason = "stopped", task.phase, "STOPPED", "stopped by the user"
        elif failure is not None:
            outcome, stage, code, reason = "failure", failure.stage, failure.code, failure.reason
        elif ev["forbidden_contacts"]:
            outcome, stage, code = "failure", "evaluation", "CONTACT_VIOLATION"
            reason = f"evaluator: {ev['forbidden_contacts']} steps with forbidden contacts (first: {ev['first_forbidden_contact']})"
        elif not ev["placed"]:
            outcome, stage, code = "failure", "evaluation", "PLACE_FAILED"
            reason = "evaluator: " + "; ".join(ev["reasons"])
        else:
            outcome, stage, code, reason = "success", None, None, "block placed on the destination pad"
        res = {
            "run_id": self.run_id, "outcome": outcome, "failure_stage": stage, "failure_code": code, "reason": reason,
            "task_request": self.run_request, "scene_revision": self.scene_revision,
            "perception_mode": task.mode, "actual_controller": self.ctrl.name, "controller_fallbacks": None,
            "versions": {"armstrong": __version__, "mujoco": mujoco.__version__, "python": platform.python_version()},
            "wall_duration_s": time.perf_counter() - self.run_wall0, "sim_duration_s": float(self.data.time) - self.run_sim0,
            "phases": task.phase_log, "estimate": task.estimate, "verify_estimate": task.verify_estimate,
            "plan": task.plan_info,
            "metrics": {**ev, "grasp_width_m": task.grasp_width, "camera_loss_events": self.camera_loss_events,
                        "torque_saturation_steps": self.ctrl.saturated_steps, "mpc_fallbacks": None,
                        "estimate_error_xy_m": self._estimate_error(task)},
            "faults": dict(self.faults),
        }
        self.result = res
        self.results[self.run_id] = {"result": res, "trace": self.trace, "scene": self.cfg.to_dict()}
        while len(self.results) > 20:
            self.results.pop(next(iter(self.results)))
        self.status = {"success": "completed", "failure": "failed", "stopped": "stopped"}[outcome]
        self.paused = False
        lvl = "info" if outcome == "success" else "warn"
        self.log(f"run {self.run_id} {outcome}" + (f" at {stage}: {reason}" if outcome != "success" else
                                                     f": placement error {1e3 * ev['placement_error_xy_m']:.1f} mm"), lvl)

    def _estimate_error(self, task) -> float | None:
        """Evaluation only: distance between the planning estimate and the block's true start position."""
        if not task.estimate or not task.estimate.get("valid") or self.block_start is None:
            return None
        return float(np.linalg.norm(np.asarray(task.estimate["position_m"][:2]) - self.block_start[:2]))

    # ------------------------------------------------------------------ commands

    def submit(self, name: str, **kwargs) -> Future:
        fut: Future = Future()
        try:
            self.commands.put_nowait((name, kwargs, fut))
        except queue.Full:
            fut.set_exception(CommandError("BUSY", "command queue full"))
        return fut

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
            if self.status not in ("paused",):
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
                self.cfg = sc.SceneConfig()
            self._reset_scene()
            return {"scene_revision": self.scene_revision}
        if name == "edit":
            return self._edit(kw)
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
            cat = {"id": "block", "color_class": "orange", "size_m": list(self.cfg.block.size),
                   "support_z_m": sc.TABLE_TOP_Z}
            self.estimate = estimate_block(obs, cat, self.scene_revision)
            return self.estimate
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
        self.scene_revision += 1
        self.estimate = self.plan = None
        self.status = "idle"
        self.log(f"scene reset (revision {self.scene_revision})")

    def _edit(self, kw: dict) -> dict:
        if self.status not in IDLE_STATES:
            raise CommandError("BUSY", f"scene edits are accepted only while stopped (status: {self.status})")
        exp = kw.get("expected_revision")
        if exp is not None and int(exp) != self.scene_revision:
            raise CommandError("STALE_SCENE", f"edit based on revision {exp}; current is {self.scene_revision}",
                               {"current_revision": self.scene_revision})
        blk, pad = kw.get("block"), kw.get("pad")
        b = sc.BlockSpec(**{**asdict(self.cfg.block)})
        p = sc.PadSpec(**{**asdict(self.cfg.pad)})
        if blk:
            b.x, b.y = float(blk.get("x", b.x)), float(blk.get("y", b.y))
            b.yaw = float(blk.get("yaw", b.yaw))
            if not (np.isfinite(b.x) and np.isfinite(b.y) and np.isfinite(b.yaw)):
                raise CommandError("INVALID_SCENE", "block pose must be finite numbers")
            if not sc.on_table(b.x, b.y, 0.04):
                raise CommandError("OUT_OF_WORKSPACE", f"block at ({b.x:.3f}, {b.y:.3f}) m is not fully on the table")
            if np.hypot(b.x, b.y) < 0.16:
                raise CommandError("OUT_OF_WORKSPACE", "block would overlap the robot base")
        if pad:
            p.x, p.y = float(pad.get("x", p.x)), float(pad.get("y", p.y))
            if not (np.isfinite(p.x) and np.isfinite(p.y)):
                raise CommandError("INVALID_SCENE", "pad position must be finite numbers")
            if not sc.on_table(p.x, p.y, 0.04):
                raise CommandError("OUT_OF_WORKSPACE", f"pad at ({p.x:.3f}, {p.y:.3f}) m is not fully on the table")
            if np.hypot(p.x, p.y) < 0.16:
                raise CommandError("OUT_OF_WORKSPACE", "pad would overlap the robot base")
        self.cfg.block, self.cfg.pad = b, p
        if blk:
            sc.place_block(self.model, self.data, self.ids, b)
        if pad:
            sc.place_pad(self.data, self.ids, p)
        mujoco.mj_forward(self.model, self.data)
        self.scene_revision += 1
        self.estimate = self.plan = None
        self.status = "idle"
        self.log(f"scene edited (revision {self.scene_revision}): block ({b.x:.3f}, {b.y:.3f}, yaw "
                 f"{np.degrees(b.yaw):.0f} deg), pad ({p.x:.3f}, {p.y:.3f})")
        return {"scene_revision": self.scene_revision}

    def _drain(self) -> None:
        while True:
            try:
                name, kw, fut = self.commands.get_nowait()
            except queue.Empty:
                return
            if fut.set_running_or_notify_cancel():
                try:
                    fut.set_result(self._handle(name, kw))
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
        tp, tyaw = self.truth_block_pose()
        frame_meta = None
        with self._lock:
            if self._frame is not None:
                frame_meta = {**self._frame["meta"], "age_s": time.monotonic() - self._frame["captured_monotonic_s"]}
        snap = {
            "status": self.status, "phase": self.task.phase if self.task_iter is not None and self.task else None,
            "run_id": self.run_id, "scene_revision": self.scene_revision, "sim_time_s": float(d.time),
            "realtime_factor": self._rtf(),
            "controller": {"name": self.ctrl.name, "key": self.ctrl.key, "gains": self.ctrl.gains(),
                           "rate_hz": 1.0 / m.opt.timestep, "fallback": None,
                           "model": "contact-free inverse dynamics (payload unmodelled)"},
            "q": np.round(self.arm_q(), 5).tolist(), "qdot": np.round(self.arm_qvel(), 4).tolist(),
            "gripper_width_m": self.finger_width(), "gripper_command": self.gripper_cmd,
            "poses": poses,
            "truth": {"block_position_m": tp.tolist(), "block_yaw_rad": tyaw,
                      "label": "simulator truth (display/evaluation only)"},
            "scene": self.cfg.to_dict(), "faults": dict(self.faults),
            "estimate": self.estimate, "plan": self.plan, "camera": frame_meta,
            "events": list(self.events)[-30:], "result": self.result,
            "timing": {"physics_dt_s": m.opt.timestep, "step_ms_mean": float(np.mean(self.step_ms)) if self.step_ms else None,
                       "build_s": self.build_s},
            "phases": self.task.phase_log if self.task else [],
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

    # ------------------------------------------------------------------ geometry export

    def _compute_visual_bodies(self):
        m = self.model
        vis = set()
        for g in range(m.ngeom):
            if self._is_visual(g):
                vis.add(int(m.geom_bodyid[g]))
        vis.discard(0)
        self._visual_bodies = sorted(vis)

    def _is_visual(self, g: int) -> bool:
        m = self.model
        group = m.geom_group[g]
        if m.geom_bodyid[g] in set(self.ids.robot_bodies.tolist()):
            return group == 2
        return group in (0, 1, 2) and m.geom_rgba[g][3] > 0

    def build_geometry(self) -> tuple[dict, bytes]:
        """Visual geoms in MuJoCo's compiled frames + one binary blob of non-indexed mesh triangles
        (float32 positions then float32 normals, per mesh, mesh frame = compiled geom frame)."""
        m = self.model
        geoms, meshes, chunks = [], {}, []
        offset = 0
        for g in range(m.ngeom):
            if not self._is_visual(g):
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
                    "rgba": rgba, "specular": float(m.mat_specular[mat]) if mat >= 0 else 0.3}
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
        return {"geoms": geoms, "bodies": [m.body(b).name for b in self._visual_bodies],
                "units": "m", "up": "+Z", "quaternion_order": "wxyz",
                "camera": {"id": self.cfg.camera.id, "position": list(self.cfg.camera.position),
                           "target": list(self.cfg.camera.target), "vertical_fov_deg": self.cfg.camera.vertical_fov_deg},
                "table": {"top_z": sc.TABLE_TOP_Z, "center_xy": list(sc.TABLE_CENTER_XY), "half_xy": list(sc.TABLE_HALF_XY)},
                "workspace": {"reach_min": sc.REACH_MIN, "reach_max": sc.REACH_MAX, "abs_y_max": sc.WORKSPACE_Y},
                "mesh_bytes": offset}, b"".join(chunks)

    # ------------------------------------------------------------------ main loop

    def run_forever(self) -> None:
        try:
            self._setup()
            self._compute_visual_bodies()
            self.geometry = self.build_geometry()
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
                self._drain()
                now = time.perf_counter()
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
            if self.camera is not None:
                self.camera.close()

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
        self._compute_visual_bodies()

    def run_sync(self, request: dict, max_sim_s: float = 60.0) -> dict:
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
