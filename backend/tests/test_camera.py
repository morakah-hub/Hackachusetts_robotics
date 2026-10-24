"""Camera calibration conventions and image-only localization (multi-color, non-square parts)."""
import numpy as np

from armstrong import projects as pj
from armstrong import scene as sc
from armstrong.camera import backproject, project
from armstrong.perception import SceneObserver, estimate_part, wrap
from armstrong.task import catalog_entry, table_info


def entry(w, pid="block"):
    return catalog_entry(w.cfg.part(pid))


def truth(w, pid="block"):
    return w.data.xpos[w.ids.parts[pid].body].copy()


def test_projection_convention_matches_rendered_pixels(worker):
    obs = worker.observe()
    b = worker.cfg.part("block")
    top = np.array([b.x, b.y, sc.TABLE_TOP_Z + b.size[2]])
    uv, z = project(obs.T_world_camera, obs.K, top)
    u, v = int(round(uv[0])), int(round(uv[1]))
    assert 0 <= u < obs.rgb.shape[1] and 0 <= v < obs.rgb.shape[0]
    r, g, bl = obs.rgb[v, u].astype(int)
    assert r > 150 and r - bl > 100, "known block-top point must project onto orange pixels"
    assert abs(float(obs.depth_m[v, u]) - z) < 0.004, "depth must be optical-axis Z in meters"
    assert u > obs.rgb.shape[1] / 2, "+Y world (block at y=+0.2) appears in the right half of the image"
    p = backproject(obs.depth_m, obs.K, obs.T_world_camera, np.array([u]), np.array([v]))[0]
    assert np.linalg.norm(p - top) < 0.006


def test_optical_frame_is_right_handed_and_looks_at_table(worker):
    obs = worker.observe()
    R = obs.T_world_camera[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
    tgt = np.asarray(worker.cfg.camera.target) - np.asarray(worker.cfg.camera.position)
    assert np.dot(R[:, 2], tgt / np.linalg.norm(tgt)) > 0.999
    assert R[2, 1] < 0, "image-down (+Y optical) must point downward in the world"


def test_estimate_uses_image_only_and_is_accurate(worker):
    est = estimate_part(worker.observe(), entry(worker), worker.scene_revision, table=table_info())
    assert est["valid"], est["reasons"]
    t = truth(worker)
    assert np.linalg.norm(np.asarray(est["position_m"][:2]) - t[:2]) < 0.004
    assert abs(est["position_m"][2] - t[2]) < 0.006


def test_moved_block_changes_observation(worker):
    e1 = estimate_part(worker.observe(), entry(worker), worker.scene_revision, table=table_info())
    worker.command_sync("edit", parts={"block": {"x": 0.42, "y": 0.05, "yaw": 0.3}})
    e2 = estimate_part(worker.observe(), entry(worker), worker.scene_revision, table=table_info())
    assert e1["valid"] and e2["valid"]
    assert np.linalg.norm(np.subtract(e1["position_m"][:2], e2["position_m"][:2])) > 0.1
    assert np.linalg.norm(np.subtract(e2["position_m"][:2], [0.42, 0.05])) < 0.004
    assert abs(e2["yaw_rad"] - 0.3) < np.radians(4)


def test_stale_and_masked_frames_are_invalid(worker):
    obs = worker.observe()
    worker.command_sync("edit", parts={"block": {"x": 0.45, "y": 0.1}})
    stale = estimate_part(obs, entry(worker), worker.scene_revision, table=table_info())
    assert not stale["valid"] and "stale" in stale["reasons"][0]
    worker.command_sync("faults", camera_masked=True)
    masked = estimate_part(worker.observe(), entry(worker), worker.scene_revision, table=table_info())
    assert not masked["valid"] and "masked" in masked["reasons"][0]


def test_two_colors_non_square_part_and_side_assignment(worker):
    worker.command_sync("load_project", project=pj.load("example", "two_part_kit"))
    for yaw in (0.5, -1.2, 1.4):
        worker.command_sync("edit", parts={"spacer": {"x": 0.42, "y": 0.06, "yaw": yaw}})
        ob = SceneObserver(worker.observe(), table_info())
        es = ob.estimate(entry(worker, "spacer"), worker.scene_revision)
        eb = ob.estimate(entry(worker, "block"), worker.scene_revision)
        assert es["valid"] and eb["valid"], (es["reasons"], eb["reasons"])
        assert abs(es["yaw_period_rad"] - np.pi) < 1e-9, "a 40 x 60 mm part has 180-degree symmetry only"
        assert abs(np.degrees(wrap(es["yaw_rad"] - yaw, np.pi))) < 4, "long side must be identified"
        assert abs(es["top_extent_m"][0] - 0.040) < 0.006 and abs(es["top_extent_m"][1] - 0.060) < 0.006
        assert np.linalg.norm(np.subtract(es["position_m"][:2], truth(worker, "spacer")[:2])) < 0.004
        assert np.linalg.norm(np.subtract(eb["position_m"][:2], truth(worker, "block")[:2])) < 0.004


def test_same_color_twice_is_rejected_and_ambiguity_reported(worker):
    import pytest

    from armstrong.sim import CommandError
    scene = worker.cfg.to_dict()
    scene["parts"].append({**scene["parts"][0], "id": "block2", "name": "Second orange", "x": 0.45, "y": -0.05})
    with pytest.raises(CommandError) as e:
        worker.command_sync("set_scene", scene=scene)
    assert "both orange" in e.value.message
    # A same-colored fixture (not a cataloged movable part) in view makes the orange detection ambiguous.
    scene["parts"][-1].update(role="fixture", id="decoy", name="Orange decoy")
    worker.command_sync("set_scene", scene=scene)
    obs = worker.observe()
    # fixtures are rendered in a neutral color, so the image stays unambiguous: identity comes from color only
    est = estimate_part(obs, entry(worker), worker.scene_revision, table=table_info())
    assert est["valid"], est["reasons"]
