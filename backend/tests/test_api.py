"""HTTP/WebSocket integration with the worker running on its own thread (as in the app)."""
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from armstrong import assets as am
from armstrong import projects as pj
from armstrong.api import create_app
from armstrong.sim import SimWorker

EX = Path(__file__).resolve().parents[2] / "examples" / "stl"


@pytest.fixture(scope="module")
def client(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("user_data")
    mp = pytest.MonkeyPatch()
    mp.setattr(am, "USER_ASSETS", tmp / "assets")
    mp.setattr(pj, "USER_PROJECTS", tmp / "projects")
    w = SimWorker()
    w.start_thread()
    assert w.error is None, w.error
    with TestClient(create_app(w)) as c:
        yield c
    w.shutdown()
    mp.undo()


def wait_status(c, statuses, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = c.get("/api/state").json()
        if s["status"] in statuses:
            return s
        time.sleep(0.05)
    raise AssertionError(f"status never reached {statuses}")


def test_health_and_capabilities(client):
    h = client.get("/api/health").json()
    assert h["ready"] and h["name"] == "Armstrong"
    caps = client.get("/api/capabilities").json()
    assert [r["id"] for r in caps["robots"] if r["available"]] == ["panda"]
    assert "sort" in caps["tasks"] and caps["parts"]["max_movable"] == 4


def test_visuals_match_binary_and_poses(client):
    vis = client.get("/api/scene/visuals").json()
    r = client.get("/api/scene/meshes.bin")
    assert len(r.content) == vis["mesh_bytes"] > 0
    assert r.headers["x-geometry-rev"] == str(vis["geometry_rev"])
    assert vis["up"] == "+Z" and vis["quaternion_order"] == "wxyz"
    s = client.get("/api/state").json()
    assert set(vis["bodies"]) <= set(s["poses"])
    assert {"hand", "part_block", "dest_pad", "link7"} <= set(s["poses"])


def test_camera_endpoints_and_websocket(client):
    r = client.get("/api/camera/rgbd/rgb")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg"
    assert client.get("/api/camera/rgbd/depth").headers["content-type"] == "image/png"
    with client.websocket_connect("/api/ws") as ws:
        msg = ws.receive_json()
        assert len(msg["q"]) == 7 and "recipe" in msg


def test_stl_upload_and_rejection(client):
    data = (EX / "spacer_40x60x45_mm.stl").read_bytes()
    r = client.post("/api/assets?filename=spacer.stl&units=mm&scale=1", content=data)
    assert r.status_code == 200, r.text
    meta = r.json()
    assert meta["eligible"]["movable"]["ok"] and abs(meta["dims_m"][1] - 0.06) < 1e-6
    assert client.get(f"/api/assets/{meta['id']}/mesh").content[:5] == b"Armst"
    bad = client.post("/api/assets?filename=x.stl&units=mm", content=b"garbage" * 20)
    assert bad.status_code == 422 and bad.json()["error"]["code"] == "UNSUPPORTED_ASSET"
    assert client.get("/api/assets/..%2F..%2Fsecret/mesh").status_code in (404, 422)


def test_structural_edit_rebuilds_geometry_and_revision(client):
    s = wait_status(client, ["idle", "completed", "failed", "stopped"])
    vis0 = client.get("/api/scene/visuals").json()
    meta = client.post("/api/assets?filename=cube.stl&units=mm", content=(EX / "cube_45_ascii_mm.stl").read_bytes()).json()
    scene = s["scene"]
    scene["parts"].append({"id": "cube", "name": "Blue cube", "role": "movable", "asset_id": meta["id"], "size": meta["dims_m"],
                           "mass": 0.05, "friction": 0.8, "material": "pla_20", "color": "blue", "category": "", "x": 0.42, "y": 0.0, "yaw": 0.0})
    r = client.post("/api/scene", json={"scene": scene, "expected_revision": s["scene_revision"]})
    assert r.status_code == 200, r.text
    vis1 = client.get("/api/scene/visuals").json()
    assert vis1["geometry_rev"] > vis0["geometry_rev"]
    assert any(g["body"] == "part_cube" and g["kind"] == "mesh" for g in vis1["geoms"])
    assert any(g["body"] == "part_cube" and g["proxy"] for g in vis1["geoms"])
    s2 = wait_status(client, ["idle"])
    assert s2["scene_revision"] == r.json()["scene_revision"] and "part_cube" in s2["poses"]
    stale = client.post("/api/scene", json={"scene": scene, "expected_revision": s["scene_revision"]})
    assert stale.status_code == 409


def test_edit_revision_run_pause_stop_reset(client):
    s = wait_status(client, ["idle", "completed", "failed", "stopped"])
    rev = s["scene_revision"]
    e = client.post("/api/scene/edit", json={"parts": {"block": {"x": 0.47, "y": 0.18}}, "expected_revision": rev})
    assert e.status_code == 200 and e.json()["scene_revision"] == rev + 1
    stale = client.post("/api/run", json={"scene_revision": rev})
    assert stale.status_code == 409 and stale.json()["error"]["code"] == "STALE_SCENE"
    bad = client.post("/api/scene/edit", json={"parts": {"block": {"x": 2.0, "y": 0}}})
    assert bad.status_code == 422 and bad.json()["error"]["code"] == "OUT_OF_WORKSPACE"
    r = client.post("/api/run", json={"perception_mode": "camera_rgbd", "scene_revision": rev + 1})
    assert r.status_code == 200, r.text
    wait_status(client, ["running"])
    assert client.post("/api/scene/edit", json={"parts": {"block": {"x": 0.4, "y": 0.1}}}).status_code == 409
    time.sleep(1.0)
    assert client.post("/api/control", json={"action": "pause"}).status_code == 200
    s1 = wait_status(client, ["paused"])
    time.sleep(0.3)
    assert abs(client.get("/api/state").json()["sim_time_s"] - s1["sim_time_s"]) < 1e-9, "pause freezes physics"
    assert client.post("/api/control", json={"action": "step", "steps": 25}).status_code == 200
    assert client.post("/api/control", json={"action": "resume"}).status_code == 200
    assert client.post("/api/control", json={"action": "stop"}).status_code == 200
    s = wait_status(client, ["stopped"])
    assert s["result"]["outcome"] == "stopped"
    exp = client.get(f"/api/runs/{s['result']['run_id']}/export")
    assert exp.status_code == 200 and "trace" in exp.json()
    assert client.post("/api/control", json={"action": "reset"}).json()["scene_revision"] > rev + 1
    wait_status(client, ["idle"])


def test_projects_open_save_export_import(client):
    lst = client.get("/api/projects").json()["projects"]
    assert {"two_part_kit", "single_block", "sort_by_category", "grid_2x2"} <= {p["id"] for p in lst if p["source"] == "example"}
    r = client.post("/api/projects/open", json={"source": "example", "id": "two_part_kit"})
    assert r.status_code == 200, r.text
    s = wait_status(client, ["idle"])
    assert s["scene"]["name"] == "Two-part kit" and len(s["recipe"]["steps"]) == 2 and s["recipe_status"]["ok"]
    sv = client.post("/api/projects/save", json={"name": "Judge kit copy"}).json()
    assert sv["ok"] and sv["source"] == "user"
    bundle = client.get("/api/projects/export?name=Judge kit copy")
    assert bundle.status_code == 200 and bundle.headers["content-type"] == "application/zip"
    client.post("/api/projects/open", json={"source": "example", "id": "single_block"})
    wait_status(client, ["idle"])
    imp = client.post("/api/projects/import", content=bundle.content)
    assert imp.status_code == 200, imp.text
    s = wait_status(client, ["idle"])
    spacer = next(p for p in s["scene"]["parts"] if p["id"] == "spacer")
    assert spacer["asset_id"] and abs(spacer["size"][1] - 0.06) < 1e-6 and s["recipe"]["steps"][1]["dest"] == "slot_b"
    assert client.post("/api/projects/open", json={"source": "user", "id": "../x"}).status_code == 404
    assert client.post("/api/projects/import", content=b"not a zip").status_code == 422
