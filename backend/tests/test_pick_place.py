"""Physical pick-and-place acceptance cases (same worker code path as the live app)."""
import mujoco
import numpy as np
import pytest

from armstrong.sim import CommandError


def block_pos(w):
    return w.data.xpos[w.ids.block_body].copy()


def test_no_hidden_attachment_in_model(worker):
    m = worker.model
    # the only equality constraint is the Menagerie finger coupling; nothing welds/attaches the block
    for i in range(m.neq):
        assert m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT
    blk = worker.ids.block_geom
    assert m.geom_contype[blk] != 0 and m.geom_conaffinity[blk] != 0


def test_camera_guided_physical_pick_place(worker):
    start = block_pos(worker)
    res = worker.run_sync({"perception_mode": "camera_rgbd", "scene_revision": worker.scene_revision})
    assert res["outcome"] == "success", res["reason"]
    m = res["metrics"]
    assert m["max_lift_m"] > 0.08                  # physically lifted by the fingers
    assert m["placement_error_xy_m"] < 0.015
    assert abs(m["placement_error_z_m"]) < 0.006    # settled on the table
    assert m["final_tilt_deg"] < 5
    assert m["forbidden_contacts"] == 0
    assert 0.044 < m["grasp_width_m"] < 0.056
    assert res["estimate"]["source"] == "camera_rgbd" and res["estimate"]["valid"]
    end = block_pos(worker)
    assert np.linalg.norm(end[:2] - [worker.cfg.pad.x, worker.cfg.pad.y]) < 0.015
    assert np.linalg.norm(end[:2] - start[:2]) > 0.3


def test_new_position_gives_new_observation_and_plan(worker):
    r1 = worker.run_sync({"perception_mode": "camera_rgbd", "scene_revision": worker.scene_revision})
    assert r1["outcome"] == "success", r1["reason"]
    worker.command_sync("edit", block={"x": 0.40, "y": 0.10, "yaw": 0.25})
    r2 = worker.run_sync({"perception_mode": "camera_rgbd", "scene_revision": worker.scene_revision})
    assert r2["outcome"] == "success", r2["reason"]
    e1, e2 = np.array(r1["estimate"]["position_m"]), np.array(r2["estimate"]["position_m"])
    assert r2["estimate"]["frame_id"] > r1["estimate"]["frame_id"]
    assert np.linalg.norm(e1[:2] - e2[:2]) > 0.1
    assert np.linalg.norm(np.subtract(r1["plan"]["grasp_m"], r2["plan"]["grasp_m"])) > 0.1
    assert abs(r2["plan"]["grasp_yaw_rad"] - 0.25) < np.radians(4)


def test_masked_camera_blocks_camera_guided_task(worker):
    worker.command_sync("faults", camera_masked=True)
    q0, b0 = worker.arm_q(), block_pos(worker)
    res = worker.run_sync({"perception_mode": "camera_rgbd", "scene_revision": worker.scene_revision})
    assert res["outcome"] == "failure"
    assert res["failure_stage"] == "observe" and res["failure_code"] == "PERCEPTION_UNAVAILABLE"
    assert res["metrics"]["camera_loss_events"] >= 1
    assert np.abs(worker.arm_q() - q0).max() < 0.01, "no blind motion may start"
    assert np.linalg.norm(block_pos(worker) - b0) < 1e-3


def test_gripper_that_cannot_close_cannot_succeed(worker):
    worker.command_sync("faults", gripper_disabled=True)
    res = worker.run_sync({"perception_mode": "camera_rgbd", "scene_revision": worker.scene_revision})
    assert res["outcome"] == "failure"
    assert res["failure_stage"] == "close" and res["failure_code"] == "GRASP_FAILED"
    assert "did not close" in res["reason"]
    assert res["metrics"]["max_lift_m"] < 0.01


def test_unreachable_target_returns_clear_failure(worker):
    worker.command_sync("edit", block={"x": 0.86, "y": 0.0})
    q0 = worker.arm_q()
    res = worker.run_sync({"perception_mode": "camera_rgbd", "scene_revision": worker.scene_revision})
    assert res["outcome"] == "failure" and res["failure_code"] == "OUT_OF_WORKSPACE"
    assert "reach" in res["reason"]
    assert np.abs(worker.arm_q() - q0).max() < 0.01


def test_ik_reports_unreachable_pose(worker):
    from armstrong.robot import HOME_Q
    _, R = worker.kin.fk(HOME_Q)
    r = worker.kin.solve(np.array([1.3, 0.0, 0.45]), R, HOME_Q)
    assert not r.ok and "no IK solution" in r.reason


def test_known_world_mode_is_labelled(worker):
    res = worker.run_sync({"perception_mode": "known_world", "scene_revision": worker.scene_revision})
    assert res["outcome"] == "success", res["reason"]
    assert res["perception_mode"] == "known_world"
    assert res["estimate"]["source"].startswith("known_world")


def test_controls_and_revisions(worker):
    rev = worker.scene_revision
    with pytest.raises(CommandError) as e:
        worker.command_sync("run", request={"scene_revision": rev - 1})
    assert e.value.code == "STALE_SCENE"
    worker.command_sync("run", request={"scene_revision": rev})
    with pytest.raises(CommandError) as e:
        worker.command_sync("edit", block={"x": 0.4, "y": 0.1})
    assert e.value.code == "BUSY"
    with pytest.raises(CommandError) as e:
        worker.command_sync("run", request={"scene_revision": rev})
    assert e.value.code == "BUSY"
    for _ in range(600):
        worker._physics_step()
    worker.command_sync("pause")
    t = worker.data.time
    worker.command_sync("step", steps=10)
    assert np.isclose(worker.data.time - t, 10 * worker.model.opt.timestep)
    worker.command_sync("resume")
    worker.command_sync("stop")
    assert worker.status == "stopped" and worker.result["outcome"] == "stopped"
    worker.command_sync("reset")
    assert worker.scene_revision == rev + 1 and worker.status == "idle"
    with pytest.raises(CommandError) as e:
        worker.command_sync("edit", block={"x": 1.5, "y": 0.0})
    assert e.value.code == "OUT_OF_WORKSPACE"
