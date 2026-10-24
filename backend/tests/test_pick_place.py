"""Physical pick-and-place and workflow acceptance cases (same worker code path as the live app)."""
import mujoco
import numpy as np
import pytest

from armstrong import projects as pj
from armstrong.sim import CommandError


def part_pos(w, pid="block"):
    return w.data.xpos[w.ids.parts[pid].body].copy()


def run(w, mode="camera_rgbd", **kw):
    return w.run_sync({"perception_mode": mode, "scene_revision": w.scene_revision, **kw})


def test_no_hidden_attachment_in_model(worker):
    m = worker.model
    for i in range(m.neq):   # the only equality constraint is the Menagerie finger coupling
        assert m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT
    g = worker.ids.parts["block"].geom
    assert m.geom_contype[g] != 0 and m.geom_conaffinity[g] != 0


def test_camera_guided_physical_pick_place(worker):
    start = part_pos(worker)
    res = run(worker)
    assert res["outcome"] == "success", res["reason"]
    m = res["metrics"]
    assert m["max_lift_m"] > 0.08
    assert m["placement_error_xy_m"] < 0.015
    assert abs(m["placement_error_z_m"]) < 0.006
    assert m["final_tilt_deg"] < 5
    assert m["forbidden_contacts"] == 0
    assert 0.044 < m["grasp_width_m"] < 0.056
    assert res["estimate"]["source"] == "camera_rgbd" and res["estimate"]["valid"]
    pad = worker.cfg.dest("pad")
    end = part_pos(worker)
    assert np.linalg.norm(end[:2] - [pad.x, pad.y]) < 0.015
    assert np.linalg.norm(end[:2] - start[:2]) > 0.3


def test_new_position_gives_new_observation_and_plan(worker):
    r1 = run(worker)
    assert r1["outcome"] == "success", r1["reason"]
    worker.command_sync("edit", parts={"block": {"x": 0.40, "y": 0.10, "yaw": 0.25}})
    r2 = run(worker)
    assert r2["outcome"] == "success", r2["reason"]
    e1, e2 = np.array(r1["estimate"]["position_m"]), np.array(r2["estimate"]["position_m"])
    assert r2["estimate"]["frame_id"] > r1["estimate"]["frame_id"]
    assert np.linalg.norm(e1[:2] - e2[:2]) > 0.1
    assert np.linalg.norm(np.subtract(r1["plan"]["grasp_m"], r2["plan"]["grasp_m"])) > 0.1
    assert abs(r2["plan"]["grasp_yaw_rad"] - 0.25) < np.radians(4)


def test_masked_camera_blocks_camera_guided_task(worker):
    worker.command_sync("faults", camera_masked=True)
    q0, b0 = worker.arm_q(), part_pos(worker)
    res = run(worker)
    assert res["outcome"] == "failure"
    assert res["failure_stage"] == "observe" and res["failure_code"] == "PERCEPTION_UNAVAILABLE"
    assert res["metrics"]["camera_loss_events"] >= 1
    assert np.abs(worker.arm_q() - q0).max() < 0.01, "no blind motion may start"
    assert np.linalg.norm(part_pos(worker) - b0) < 1e-3


def test_gripper_that_cannot_close_cannot_succeed(worker):
    worker.command_sync("faults", gripper_disabled=True)
    res = run(worker)
    assert res["outcome"] == "failure"
    assert res["failure_stage"] == "close" and res["failure_code"] == "GRASP_FAILED"
    assert "did not close" in res["reason"]
    assert res["metrics"]["max_lift_m"] < 0.01


def test_unreachable_target_returns_clear_failure(worker):
    worker.command_sync("edit", parts={"block": {"x": 0.86, "y": 0.0}})
    q0 = worker.arm_q()
    res = run(worker)
    assert res["outcome"] == "failure" and res["failure_code"] == "OUT_OF_WORKSPACE"
    assert "reach" in res["reason"]
    assert np.abs(worker.arm_q() - q0).max() < 0.01


def test_ik_reports_unreachable_pose(worker):
    from armstrong.robot import HOME_Q
    _, R = worker.kin.fk(HOME_Q)
    r = worker.kin.solve(np.array([1.3, 0.0, 0.45]), R, HOME_Q)
    assert not r.ok and "no IK solution" in r.reason


def test_known_world_mode_is_labelled(worker):
    res = run(worker, "known_world")
    assert res["outcome"] == "success", res["reason"]
    assert res["perception_mode"] == "known_world"
    assert res["estimate"]["source"].startswith("known_world")


def test_two_part_kit_in_sequence_without_reset(worker):
    worker.command_sync("load_project", project=pj.load("example", "two_part_kit"))
    res = run(worker)
    assert res["outcome"] == "success", res["reason"]
    assert [s["outcome"] for s in res["steps"]] == ["success", "success"]
    assert [s["part_id"] for s in res["steps"]] == ["block", "spacer"]
    assert res["steps"][1]["estimate"]["frame_id"] > res["steps"][0]["verify_estimate"]["frame_id"] - 1
    for pid, did in (("block", "slot_a"), ("spacer", "slot_b")):
        d = worker.cfg.dest(did)
        assert np.linalg.norm(part_pos(worker, pid)[:2] - [d.x, d.y]) < 0.015, f"{pid} must stay on {did}"
    m2 = res["steps"][1]["metrics"]
    assert m2["yaw_error_deg"] is not None and m2["yaw_error_deg"] < 8, "requested slot yaw must be achieved"
    assert 0.034 < m2["grasp_width_m"] < 0.046, "the 40 x 60 spacer is grasped across its 40 mm side"
    assert res["metrics"]["forbidden_contacts"] == 0


def test_occupied_destination_is_rejected_before_motion(worker):
    worker.command_sync("load_project", project=pj.load("example", "two_part_kit"))
    d = worker.cfg.dest("slot_a")
    worker.command_sync("edit", parts={"spacer": {"x": d.x, "y": d.y, "yaw": 0.0}})
    q0 = worker.arm_q()
    res = run(worker)
    assert res["outcome"] == "failure" and res["failure_code"] == "DESTINATION_OCCUPIED"
    assert "Blue spacer" in res["reason"] and res["failed_step"] == 1
    assert [s["outcome"] for s in res["steps"]] == ["failure", "not_run"]
    assert np.abs(worker.arm_q() - q0).max() < 0.01


def add_wall(w, x, height, y=0.20):
    """Static fixture (known geometry, neutral color) 20 mm thick along x, 120 mm long along y."""
    scene = w.cfg.to_dict()
    scene["parts"] = [p for p in scene["parts"] if p["id"] != "wall"]
    scene["parts"].append({"id": "wall", "name": "Wall fixture", "role": "fixture", "asset_id": None,
                           "size": [0.02, 0.12, height], "mass": 1.0, "friction": 1.0, "material": None,
                           "color": "orange", "category": "", "x": x, "y": y, "yaw": 0.0})
    w.command_sync("set_scene", scene=scene)


def test_occluded_part_blocks_motion(worker):
    # The camera looks from +x. A 250 mm wall between the camera and the block (6 mm from its +x face) hides it.
    add_wall(worker, x=0.541, height=0.25)
    q0, b0 = worker.arm_q(), part_pos(worker)
    res = run(worker)
    assert res["outcome"] == "failure"
    assert res["failure_stage"] == "observe" and res["failure_code"] == "PERCEPTION_UNAVAILABLE", res["reason"]
    assert "Orange block" in res["reason"] and "visible" in res["reason"], res["reason"]
    step = res["steps"][0]
    assert step["estimate"] is None and step["plan"] is None, "no plan may be made from an occluded view"
    assert np.abs(worker.arm_q() - q0).max() < 0.01 and np.linalg.norm(part_pos(worker) - b0) < 1e-3


def test_blocked_route_is_rejected_before_motion(worker):
    # A wall *behind* the block as seen from the camera cannot occlude it, but it sits 6 mm from the block's -x face,
    # inside the volume swept by the open -x finger during the descent (fingers close along world x at yaw 0).
    add_wall(worker, x=0.459, height=0.06)
    q0, b0 = worker.arm_q(), part_pos(worker)
    res = run(worker)
    step = res["steps"][0]
    est = step["estimate"]
    assert est is not None and est["valid"] and est["source"] == "camera_rgbd", "perception must succeed here"
    assert np.linalg.norm(np.subtract(est["position_m"][:2], b0[:2])) < 0.004, "estimate must be accurate"
    assert res["outcome"] == "failure"
    assert res["failure_stage"] == "plan" and res["failure_code"] == "ROUTE_BLOCKED", res["reason"]
    assert "route is not collision-free" in res["reason"] and "part 'wall'" in res["reason"], res["reason"]
    assert step["plan"] is None, "a rejected route is never published as a plan"
    assert np.abs(worker.arm_q() - q0).max() < 0.01, "a blocked route never starts moving"
    assert np.linalg.norm(part_pos(worker) - b0) < 1e-3
    # Control: the same wall 60 mm further away leaves the route clear, and the run succeeds physically.
    add_wall(worker, x=0.399, height=0.06)
    ok = run(worker)
    assert ok["outcome"] == "success", ok["reason"]
    assert ok["metrics"]["forbidden_contacts"] == 0


def test_sorting_rules_fill_zone_slots(worker):
    worker.command_sync("load_project", project=pj.load("example", "sort_by_category"))
    res = run(worker, "known_world")   # fast check of the rule logic; camera mode is covered above
    assert res["outcome"] == "success", res["reason"]
    dests = {s["part_id"]: s["dest_id"] for s in res["steps"]}
    assert dests == {"p_orange": "bin_l1", "p_blue": "bin_s1", "p_green": "bin_l2"}


def test_controls_and_revisions(worker):
    rev = worker.scene_revision
    with pytest.raises(CommandError) as e:
        worker.command_sync("run", request={"scene_revision": rev - 1})
    assert e.value.code == "STALE_SCENE"
    worker.command_sync("run", request={"scene_revision": rev})
    with pytest.raises(CommandError) as e:
        worker.command_sync("edit", parts={"block": {"x": 0.4, "y": 0.1}})
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
    assert worker.result["steps"][0]["outcome"] == "stopped"
    worker.command_sync("reset")
    assert worker.scene_revision == rev + 1 and worker.status == "idle"
    with pytest.raises(CommandError) as e:
        worker.command_sync("edit", parts={"block": {"x": 1.5, "y": 0.0}})
    assert e.value.code == "OUT_OF_WORKSPACE"


def test_failed_rebuild_keeps_previous_scene(worker):
    rev, model = worker.scene_revision, worker.model
    bad = worker.cfg.to_dict()
    bad["parts"][0]["asset_id"] = "a_0000000000000000"
    with pytest.raises(CommandError) as e:
        worker.command_sync("set_scene", scene=bad)
    assert e.value.code == "UNSUPPORTED_ASSET"
    assert worker.model is model and worker.scene_revision == rev
    res = run(worker, "known_world")
    assert res["outcome"] == "success", res["reason"]
