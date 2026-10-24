"""HTTP/WebSocket integration with the worker running on its own thread (as in the app)."""
import time

import pytest
from fastapi.testclient import TestClient

from armstrong.api import create_app
from armstrong.sim import SimWorker


@pytest.fixture(scope="module")
def client():
    w = SimWorker()
    w.start_thread()
    assert w.error is None, w.error
    with TestClient(create_app(w)) as c:
        yield c
    w.shutdown()


def wait_status(c, statuses, timeout=10):
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


def test_visuals_match_binary_and_poses(client):
    vis = client.get("/api/scene/visuals").json()
    blob = client.get("/api/scene/meshes.bin").content
    assert len(blob) == vis["mesh_bytes"] > 0
    assert vis["up"] == "+Z" and vis["quaternion_order"] == "wxyz"
    s = client.get("/api/state").json()
    assert set(vis["bodies"]) <= set(s["poses"])
    assert {"hand", "block", "pad", "link7"} <= set(s["poses"])


def test_camera_endpoints(client):
    r = client.get("/api/camera/rgbd/rgb")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg"
    assert int(r.headers["x-frame-id"]) >= 1
    d = client.get("/api/camera/rgbd/depth")
    assert d.status_code == 200 and d.headers["content-type"] == "image/png"


def test_websocket_streams_state(client):
    with client.websocket_connect("/api/ws") as ws:
        msg = ws.receive_json()
        assert msg["status"] in ("idle", "completed", "failed", "stopped")
        assert len(msg["q"]) == 7


def test_edit_revision_run_pause_stop_reset(client):
    s = wait_status(client, ["idle", "completed", "failed", "stopped"])
    rev = s["scene_revision"]
    e = client.post("/api/scene/edit", json={"block": {"x": 0.47, "y": 0.18}, "expected_revision": rev})
    assert e.status_code == 200 and e.json()["scene_revision"] == rev + 1
    stale = client.post("/api/run", json={"scene_revision": rev})
    assert stale.status_code == 409 and stale.json()["error"]["code"] == "STALE_SCENE"
    bad = client.post("/api/scene/edit", json={"block": {"x": 2.0, "y": 0}})
    assert bad.status_code == 422 and bad.json()["error"]["code"] == "OUT_OF_WORKSPACE"
    r = client.post("/api/run", json={"perception_mode": "camera_rgbd", "scene_revision": rev + 1})
    assert r.status_code == 200, r.text
    wait_status(client, ["running"])
    busy = client.post("/api/scene/edit", json={"block": {"x": 0.4, "y": 0.1}})
    assert busy.status_code == 409
    time.sleep(1.0)
    assert client.post("/api/control", json={"action": "pause"}).status_code == 200
    s1 = wait_status(client, ["paused"])
    time.sleep(0.3)
    s2 = client.get("/api/state").json()
    assert abs(s2["sim_time_s"] - s1["sim_time_s"]) < 1e-9, "pause freezes physics time"
    assert client.post("/api/control", json={"action": "step", "steps": 25}).status_code == 200
    assert client.post("/api/control", json={"action": "resume"}).status_code == 200
    assert client.post("/api/control", json={"action": "stop"}).status_code == 200
    s = wait_status(client, ["stopped"])
    assert s["result"]["outcome"] == "stopped"
    exp = client.get(f"/api/runs/{s['result']['run_id']}/export")
    assert exp.status_code == 200 and "trace" in exp.json()
    rr = client.post("/api/control", json={"action": "reset"}).json()
    assert rr["scene_revision"] > rev + 1
    wait_status(client, ["idle"])
