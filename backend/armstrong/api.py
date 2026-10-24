"""Local HTTP/WebSocket API. Handlers enqueue commands and read published snapshots only; MuJoCo is touched only
by the simulation worker. STL parsing and project files are handled here (no MuJoCo involved)."""

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
from . import assets as am
from . import projects as pj
from . import scene as sc
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
              "UNSUPPORTED_TASK": 422, "INVALID_RECIPE": 422, "UNSUPPORTED_PART": 422, "UNSUPPORTED_ASSET": 422,
              "BACKEND_ERROR": 500}


def create_app(worker: SimWorker) -> FastAPI:
    app = FastAPI(title="Armstrong", version=__version__)
    app.add_middleware(GZipMiddleware, minimum_size=2048)

    async def command(name: str, timeout: float = 20, **kw):
        fut = worker.submit(name, **kw)
        try:
            res = await asyncio.wait_for(asyncio.wrap_future(fut), timeout=timeout)
        except CommandError as e:
            return _err(e.code, e.message, STATUS_FOR.get(e.code, 400), e.details)
        except asyncio.TimeoutError:
            return _err("BACKEND_ERROR", f"simulation worker did not answer within {timeout:.0f} s", 504)
        return JSONResponse(_clean({"ok": True, **(res or {})}))

    async def raw_command(name: str, **kw):
        fut = worker.submit(name, **kw)
        return await asyncio.wait_for(asyncio.wrap_future(fut), timeout=20)

    async def body(request: Request) -> dict:
        try:
            data = await request.json()
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}

    async def raw_body(request: Request, limit: int) -> bytes | None:
        n = request.headers.get("content-length")
        if n is not None and n.isdigit() and int(n) > limit:
            return None
        data = await request.body()
        return None if len(data) > limit else data

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
                "tasks": ["place", "kit", "arrange", "sort"], "perception_modes": ["camera_rgbd", "known_world"],
                "controllers": [{"id": "ctc", "name": "Computed torque (CTC)", "available": True},
                                {"id": "torque_mpc", "name": "Torque MPC", "available": False, "status": "planned"}],
                "parts": {"max_parts": sc.MAX_PARTS, "max_movable": sc.MAX_MOVABLE,
                          "max_destinations": sc.MAX_DESTINATIONS, "colors": list(sc.PALETTE),
                          "grasp_width_m": [am.GRASP_WIDTH_MIN, am.GRASP_WIDTH_MAX], "height_m": [am.HEIGHT_MIN, am.HEIGHT_MAX],
                          "long_side_max_m": am.LONG_SIDE_MAX, "mass_max_kg": am.MASS_MAX},
                "imports": {"formats": ["stl (binary)", "stl (ascii)"], "max_bytes": am.MAX_UPLOAD_BYTES,
                            "max_triangles": am.MAX_TRIANGLES, "units": list(am.UNITS),
                            "collision": "box proxy for boxlike parts; concave shapes visual-only"},
                "obstacle_perception": False, "assembly": False}

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
        geo, rev = worker.geometry_pair()
        if geo is None:
            return _err("BACKEND_ERROR", worker.error or "loading", 503)
        return JSONResponse(_clean({**geo[0], "geometry_rev": rev}))

    @app.get("/api/scene/meshes.bin")
    async def meshes():
        geo, rev = worker.geometry_pair()
        if geo is None:
            return _err("BACKEND_ERROR", worker.error or "loading", 503)
        return Response(geo[1], media_type="application/octet-stream",
                        headers={"Cache-Control": "no-cache", "X-Geometry-Rev": str(rev)})

    @app.get("/api/scene")
    async def get_scene():
        snap = worker.snapshot()
        return JSONResponse(_clean({"scene": snap.get("scene"), "scene_revision": snap.get("scene_revision")}))

    @app.post("/api/scene")
    async def set_scene(request: Request):
        b = await body(request)
        return await command("set_scene", scene=b.get("scene"), expected_revision=b.get("expected_revision"),
                             why=str(b.get("why") or "scene updated")[:80])

    @app.post("/api/scene/edit")
    async def edit(request: Request):
        b = await body(request)
        return await command("edit", block=b.get("block"), pad=b.get("pad"), parts=b.get("parts"),
                             dests=b.get("dests"), expected_revision=b.get("expected_revision"))

    @app.post("/api/recipe")
    async def set_recipe(request: Request):
        b = await body(request)
        return await command("set_recipe", recipe=b.get("recipe"))

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

    # ------------------------------------------------------------------ assets (STL import)

    @app.get("/api/materials")
    async def materials():
        return {"presets": am.MATERIALS, "note": am.MATERIAL_NOTE}

    @app.post("/api/assets")
    async def upload_asset(request: Request):
        q = request.query_params
        data = await raw_body(request, am.MAX_UPLOAD_BYTES)
        if data is None:
            return _err("UNSUPPORTED_ASSET", f"file is larger than {am.MAX_UPLOAD_BYTES // (1024 * 1024)} MB", 413)
        try:
            scale = float(q.get("scale", "1"))
            rot = tuple(int(q.get(k, "0")) for k in ("rx", "ry", "rz"))
            meta = await asyncio.to_thread(am.import_stl, data, q.get("filename", "part.stl"), q.get("units", "mm"),
                                           scale, rot)
        except ValueError as e:      # AssetError is a ValueError; also bad numbers in the query
            return _err("UNSUPPORTED_ASSET", str(e), 422)
        mass, fric = am.material_mass("pla_20", meta, meta["dims_m"])
        return JSONResponse(_clean({**meta, "suggested": {"material": "pla_20", "mass_kg": mass, "friction": fric}}))

    @app.get("/api/assets/{asset_id}")
    async def asset_info(asset_id: str):
        try:
            return JSONResponse(_clean(am.asset_meta(asset_id)))
        except am.AssetError as e:
            return _err("UNSUPPORTED_ASSET", str(e), 404)

    @app.get("/api/assets/{asset_id}/mesh")
    async def asset_mesh(asset_id: str):
        try:
            return Response(am.asset_path(asset_id).read_bytes(), media_type="model/stl")
        except am.AssetError as e:
            return _err("UNSUPPORTED_ASSET", str(e), 404)

    # ------------------------------------------------------------------ projects

    @app.get("/api/projects")
    async def projects_list():
        return {"projects": pj.list_projects()}

    async def open_project(project: dict):
        return await command("load_project", timeout=40, project=project)

    @app.post("/api/projects/open")
    async def projects_open(request: Request):
        b = await body(request)
        try:
            p = pj.load(str(b.get("source")), str(b.get("id")))
        except (pj.ProjectError, ValueError) as e:
            return _err("INVALID_PROJECT", str(e), 404)
        return await open_project(p)

    async def current_project(name: str, description: str = "") -> dict:
        snap = worker.snapshot()
        return pj.make_project(name or snap["scene"].get("name", "Project"), snap["scene"], snap["recipe"],
                               {"perception_mode": "camera_rgbd"}, description)

    @app.post("/api/projects/save")
    async def projects_save(request: Request):
        b = await body(request)
        name = str(b.get("name") or "").strip()[:80]
        if not name:
            return _err("INVALID_PROJECT", "give the project a name", 422)
        p = await current_project(name, str(b.get("description") or ""))
        p["scene"]["name"] = name
        try:
            pid = pj.save_user(p)
        except (pj.ProjectError, OSError) as e:
            return _err("INVALID_PROJECT", str(e), 422)
        return {"ok": True, "id": pid, "source": "user", "name": name}

    @app.get("/api/projects/export")
    async def projects_export(name: str = ""):
        p = await current_project(name)
        try:
            data = pj.export_bundle(p)
        except (pj.ProjectError, am.AssetError) as e:
            return _err("INVALID_PROJECT", str(e), 422)
        fname = pj.slug(p["name"]) + ".armstrong.zip"
        return Response(data, media_type="application/zip",
                        headers={"Content-Disposition": f'attachment; filename="{fname}"'})

    @app.post("/api/projects/import")
    async def projects_import(request: Request):
        data = await raw_body(request, pj.MAX_BUNDLE_BYTES)
        if data is None:
            return _err("INVALID_PROJECT", "bundle is too large", 413)
        try:
            p = await asyncio.to_thread(pj.import_bundle, data)
            pid = pj.save_user(p)
        except (pj.ProjectError, am.AssetError) as e:
            return _err("INVALID_PROJECT", str(e), 422)
        res = await open_project(p)
        if res.status_code == 200:
            payload = json.loads(res.body)
            return JSONResponse({**payload, "id": pid, "source": "user"})
        return res

    # ------------------------------------------------------------------ runs

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
        payload = {**rec, "labels": {"*_truth_*": "simulator truth (evaluation)",
                                     "result.steps[].estimate": "camera estimate used for planning"}}
        return Response(json.dumps(_clean(payload), indent=1), media_type="application/json",
                        headers={"Content-Disposition": f'attachment; filename="armstrong_run_{run_id}.json"'})

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

    if DIST.is_dir():
        app.mount("/assets", StaticFiles(directory=DIST / "assets"), name="assets")

        @app.get("/")
        async def index():
            return FileResponse(DIST / "index.html")

        @app.get("/{path:path}")
        async def spa(path: str):
            if path.startswith("api/"):
                return _err("BACKEND_ERROR", f"unknown API path /{path[:80]}", 404)
            f = (DIST / path).resolve()
            if path and f.is_file() and DIST.resolve() in f.parents:
                return FileResponse(f)
            return FileResponse(DIST / "index.html")
    else:
        @app.get("/")
        async def no_frontend():
            return JSONResponse({"message": "frontend not built; run start.ps1 (it builds frontend/dist)"})

    return app
