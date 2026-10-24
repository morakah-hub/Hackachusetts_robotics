"""Franka Emika Panda, loaded from the vendored MuJoCo Menagerie model (Apache-2.0).

The MJCF in backend/assets/robots/franka_emika_panda is used unmodified. Changes needed by
Armstrong are made on the loaded MjSpec only:

* an end-effector site midway between the fingertips (hand frame z = 0.1034 m);
* arm actuators 1-7 turned into joint torque motors (same +-87 / +-12 Nm limits), driven by
  the computed-torque controller in controller.py;
* a stiffer gripper servo (1500 N/m, 20 N s/m on the "split" tendon, ctrl 0 = closed,
  255 = open). The Menagerie default (100 N/m) squeezes a 50 mm block with ~2.5 N only.
  The stiffer value follows the reference project's validated v0.6 manipulation settings.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

ASSET_DIR = Path(__file__).resolve().parent.parent / "assets" / "robots" / "franka_emika_panda"
MODEL_XML = ASSET_DIR / "panda.xml"

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
ARM_ACTUATORS = tuple(f"actuator{i}" for i in range(1, 8))
FINGER_JOINTS = ("finger_joint1", "finger_joint2")
GRIPPER_ACTUATOR = "actuator8"
GRIPPER_OPEN = 255.0
GRIPPER_CLOSED = 0.0
BASE_BODY = "link0"
HAND_BODY = "hand"
FINGER_BODIES = ("left_finger", "right_finger")
EE_SITE = "ee_site"
EE_SITE_OFFSET = (0.0, 0.0, 0.1034)
HOME_Q = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])
# Franka datasheet joint speed limits (rad/s). The trajectories below use a fraction of them.
VELOCITY_LIMITS = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
GRIPPER_STIFFNESS = 1500.0
GRIPPER_DAMPING = 20.0


def load_panda_spec() -> mujoco.MjSpec:
    if not MODEL_XML.is_file():
        raise FileNotFoundError(f"Panda model missing: {MODEL_XML}")
    spec = mujoco.MjSpec.from_file(str(MODEL_XML))
    spec.body(HAND_BODY).add_site(name=EE_SITE, pos=EE_SITE_OFFSET, size=[0.004, 0, 0],
                                  rgba=[0.1, 0.9, 0.8, 0.0], group=4)
    for name in ARM_ACTUATORS:
        act = spec.actuator(name)
        limit = np.array(act.forcerange, dtype=float)
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.gainprm = np.r_[1.0, np.zeros(len(act.gainprm) - 1)]
        act.biastype = mujoco.mjtBias.mjBIAS_NONE
        act.biasprm = np.zeros(len(act.biasprm))
        act.ctrlrange = limit
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    grip = spec.actuator(GRIPPER_ACTUATOR)
    gain = np.array(grip.gainprm, dtype=float)
    bias = np.array(grip.biasprm, dtype=float)
    gain[0] = GRIPPER_STIFFNESS * 0.04 / 255.0
    bias[:3] = [0.0, -GRIPPER_STIFFNESS, -GRIPPER_DAMPING]
    grip.gainprm, grip.biasprm = gain, bias
    # The keyframe refers to the standalone model's qpos layout; Armstrong sets the state by name.
    for key in list(spec.keys):
        spec.delete(key)
    return spec
