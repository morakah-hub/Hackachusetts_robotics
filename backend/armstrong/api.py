"""Local HTTP/WebSocket API. Handlers enqueue commands and read published snapshots only."""

from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from . import __version__
from .sim import CommandError, SimWorker

DIST = Path(__file__).resolve().parents[2] / "frontend" / "dist"


def _clean(obj):
    """JSON-safe: non-finite floats become null; numpy scalars/arrays become Python types."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if hasattr(obj, "tolist"):
        return _clean(obj.tolist())
    if hasattr(obj, "item"):
        return _clean(obj.item())
    return obj


def _err(code: str, message: str, status: int, details: dict | None = None) -> JSONResponse:
    return JSONResponse({"error": {"code": code, "message": message, "details": details or {}}}, status_code=status)


STATUS_FOR = {"BUSY": 409, "STALE_SCENE": 409, "OUT_OF_WORKSPACE": 422, "INVALID_SCENE": 422,
              "UNSUPPORTED_TASK": 422, "BACKEND_ERROR": 500}


def create_app(worker: SimWorker) -> FastAPI:
    app = FastAPI(title="Armstrong", version=__version__)
    app.add_middleware(GZipMiddleware, minimum_size=2048)

    async def command(name: str, **kw):
        fut = worker.submit(name, **kw)
        try:
            res = await asyncio.wait_for(asyncio.wrap_future(fut), timeout=15)
        except CommandError as e:
            return _err(e.code, e.message, STATUS_FOR.get(e.code, 400), e.details)
        except asyncio.TimeoutError:
            return _err("BACKEND_ERROR", "simulation worker did not answer within 15 s", 504)
        return JSONResponse(_clean({"ok": True, **(res or {})}))

    async def body(request: Request) -> dict:
        try:
            data = await request.json()
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}

    @app.get("/api/health")
    async def health():
        snap = worker.snapshot()
        return {"name": "Armstrong", "version": __version__, "status": snap.get("status"),
                "ready": worker.ready.is_set() and worker.error is None, "error": worker.error}

    @app.get("/api/capabilities")
    async def capabilities():
        return {"robots": [{"id": "panda", "name": "Franka Emika Panda", "available": True},
                           {"id": "ur5e", "name": "Universal Robots UR5e", "available": False, "status": "planned"},
                           {"id": "xarm7", "name": "UFACTORY xArm 7", "available": False, "status": "planned"}],
                "tasks": ["pick_place"], "perception_modes": ["camera_rgbd", "known_world"],
                "controllers": [{"id": "ctc", "name": "Computed torque (CTC)", "available": True},
                                {"id": "torque_mpc", "name": "Torque MPC", "available": False, "status": "planned"}],
                "objects": {"type": "upright cuboid", "size_m": [0.05, 0.05, 0.06], "color": "orange", "count": 1},
                "imports": [], "obstacle_perception": False}

    @app.get("/api/state")
    async def state():
        return JSONResponse(_clean(worker.snapshot()))

    @app.websocket("/api/ws")
    async def ws(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_text(json.dumps(_clean(worker.snapshot())))
                await asyncio.sleep(1 / 30)
        except (WebSocketDisconnect, RuntimeError):
            return
        except Exception:
            return

    @app.get("/api/scene/visuals")
    async def visuals():
        if not worker.ready.is_set() or worker.error:
            return _err("BACKEND_ERROR", worker.error or "loading", 503)
        meta, _ = worker.geometry
        return JSONResponse(_clean(meta))

    @app.get("/api/scene/meshes.bin")
    async def meshes():
        if not worker.ready.is_set() or worker.error:
            return _err("BACKEND_ERROR", worker.error or "loading", 503)
        _, blob = worker.geometry
        return Response(blob, media_type="application/octet-stream", headers={"Cache-Control": "no-cache"})

    @app.post("/api/scene/edit")
    async def edit(request: Request):
        b = await body(request)
        return await command("edit", block=b.get("block"), pad=b.get("pad"), expected_revision=b.get("expected_revision"))

    @app.post("/api/run")
    async def run(request: Request):
        return await command("run", request=await body(request))

    @app.post("/api/control")
    async def control(request: Request):
        b = await body(request)
        action = b.get("action")
        if action not in ("pause", "resume", "stop", "reset", "step"):
            return _err("BACKEND_ERROR", f"unknown action {action!r}", 400)
        kw = {}
        if action == "step":
            kw["steps"] = b.get("steps", 25)
        if action == "reset":
            kw["default_scene"] = bool(b.get("default_scene", False))
        return await command(action, **kw)

    @app.post("/api/faults")
    async def faults(request: Request):
        b = await body(request)
        return await command("faults", camera_masked=b.get("camera_masked"), gripper_disabled=b.get("gripper_disabled"))

    @app.post("/api/perception/observe")
    async def observe():
        return await command("observe")

    def _frame_response(kind: str):
        fr = worker.frame()
        if fr is None:
            return _err("PERCEPTION_UNAVAILABLE", "no camera frame yet", 503)
        meta = fr["meta"]
        media = "image/jpeg" if kind == "rgb" else "image/png"
        return Response(fr[kind], media_type=media, headers={
            "Cache-Control": "no-store", "X-Frame-Id": str(meta["frame_id"]),
            "X-Scene-Revision": str(meta["scene_revision"]), "X-Sim-Time": f"{meta['sim_time_s']:.3f}"})

    @app.get("/api/camera/{camera_id}/rgb")
    async def cam_rgb(camera_id: str):
        return _frame_response("rgb")

    @app.get("/api/camera/{camera_id}/depth")
    async def cam_depth(camera_id: str):
        return _frame_response("depth")

    @app.get("/api/camera/{camera_id}/meta")
    async def cam_meta(camera_id: str):
        fr = worker.frame()
        return JSONResponse(_clean(fr["meta"] if fr else None))

    @app.get("/api/runs/{run_id}")
    async def run_result(run_id: str):
        rec = worker.results.get(run_id)
        if rec is None:
            return _err("BACKEND_ERROR", f"unknown run {run_id}", 404)
        return JSONResponse(_clean(rec["result"]))

    @app.get("/api/runs/{run_id}/export")
    async def run_export(run_id: str):
        rec = worker.results.get(run_id)
        if rec is None:
            return _err("BACKEND_ERROR", f"unknown run {run_id}", 404)
        payload = {**rec, "trace_columns": ["t", "phase", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "gripper_width_m",
                                            "block_truth_x", "block_truth_y", "block_truth_z"],
                   "labels": {"block_truth_*": "simulator truth (evaluation)", "result.estimate": "camera estimate"}}
        return Response(json.dumps(_clean(payload), indent=1), media_type="application/json",
                        headers={"Content-Disposition": f'attachment; filename="armstrong_run_{run_id}.json"'})

    if DIST.is_dir():
        app.mount("/assets", StaticFiles(directory=DIST / "assets"), name="assets")

        @app.get("/")
        async def index():
            return FileResponse(DIST / "index.html")

        @app.get("/{path:path}")
        async def spa(path: str):
            f = DIST / path
            if path and f.is_file() and DIST in f.resolve().parents:
                return FileResponse(f)
            return FileResponse(DIST / "index.html")
    else:
        @app.get("/")
        async def no_frontend():
            return JSONResponse({"message": "frontend not built; run start.ps1 (it builds frontend/dist)"})

    return app
