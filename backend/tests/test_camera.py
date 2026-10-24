"""Camera calibration conventions and image-only localization."""
import numpy as np

from armstrong import scene as sc
from armstrong.camera import backproject, project
from armstrong.perception import estimate_block


def catalog(w):
    return {"id": "block", "color_class": "orange", "size_m": list(w.cfg.block.size), "support_z_m": sc.TABLE_TOP_Z}


def test_projection_convention_matches_rendered_pixels(worker):
    obs = worker.observe()
    b = worker.cfg.block
    top = np.array([b.x, b.y, sc.TABLE_TOP_Z + b.size[2]])
    uv, z = project(obs.T_world_camera, obs.K, top)
    u, v = int(round(uv[0])), int(round(uv[1]))
    assert 0 <= u < obs.rgb.shape[1] and 0 <= v < obs.rgb.shape[0]
    r, g, bl = obs.rgb[v, u].astype(int)
    assert r > 150 and r - bl > 100, "known block-top point must project onto orange pixels"
    assert abs(float(obs.depth_m[v, u]) - z) < 0.004, "depth must be optical-axis Z in meters"
    # +Y world (block at y=+0.2) appears in the right half of the image: optical +X = image right
    assert u > obs.rgb.shape[1] / 2
    # back-projection round trip
    p = backproject(obs.depth_m, obs.K, obs.T_world_camera, np.array([u]), np.array([v]))[0]
    assert np.linalg.norm(p - top) < 0.006


def test_optical_frame_is_right_handed_and_looks_at_table(worker):
    obs = worker.observe()
    R = obs.T_world_camera[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
    forward = R[:, 2]
    tgt = np.asarray(worker.cfg.camera.target) - np.asarray(worker.cfg.camera.position)
    assert np.dot(forward, tgt / np.linalg.norm(tgt)) > 0.999
    assert R[2, 1] < 0, "image-down (+Y optical) must point downward in the world"


def test_estimate_uses_image_only_and_is_accurate(worker):
    obs = worker.observe()
    est = estimate_block(obs, catalog(worker), worker.scene_revision)
    assert est["valid"], est["reasons"]
    truth = worker.data.xpos[worker.ids.block_body]
    assert np.linalg.norm(np.asarray(est["position_m"][:2]) - truth[:2]) < 0.008
    assert abs(est["position_m"][2] - truth[2]) < 0.006


def test_moved_block_changes_observation(worker):
    e1 = estimate_block(worker.observe(), catalog(worker), worker.scene_revision)
    worker.command_sync("edit", block={"x": 0.42, "y": 0.05, "yaw": 0.3})
    e2 = estimate_block(worker.observe(), catalog(worker), worker.scene_revision)
    assert e1["valid"] and e2["valid"]
    assert np.linalg.norm(np.subtract(e1["position_m"][:2], e2["position_m"][:2])) > 0.1
    assert np.linalg.norm(np.subtract(e2["position_m"][:2], [0.42, 0.05])) < 0.008
    assert abs(e2["yaw_rad"] - 0.3) < np.radians(4)


def test_stale_and_masked_frames_are_invalid(worker):
    obs = worker.observe()
    worker.command_sync("edit", block={"x": 0.45, "y": 0.1})
    stale = estimate_block(obs, catalog(worker), worker.scene_revision)
    assert not stale["valid"] and "stale" in stale["reasons"][0]
    worker.command_sync("faults", camera_masked=True)
    masked = estimate_block(worker.observe(), catalog(worker), worker.scene_revision)
    assert not masked["valid"] and "masked" in masked["reasons"][0]
