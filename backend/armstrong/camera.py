"""Simulated RGB-D camera.

Optical frame convention (OpenCV): +X image-right, +Y image-down, +Z forward. MuJoCo cameras
look along their local -Z with +Y up, so R_world_optical = R_world_mjcam @ diag(1, -1, -1).
Pixel (u, v) = (column, row) of the returned arrays, row 0 at the top of the image, and
integer indices address pixel centers. K = [[f, 0, W/2 - 0.5], [0, f, H/2 - 0.5], [0, 0, 1]]
with f = (H / 2) / tan(fovy / 2). Depth is metric optical-axis Z (not Euclidean range);
invalid pixels are 0. tests/test_camera.py checks the projection on a synthetic known point.

Renderer objects own OpenGL contexts: create, use and close them on the simulation thread only.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import mujoco
import numpy as np

from .scene import CameraSpec

OPTICAL_FROM_MJCAM = np.diag([1.0, -1.0, -1.0])


@dataclass
class CameraObservation:
    camera_id: str
    frame_id: int
    scene_revision: int
    sim_time_s: float
    captured_monotonic_s: float
    rgb: np.ndarray            # (H, W, 3) uint8
    depth_m: np.ndarray        # (H, W) float32, optical Z in meters, 0 = invalid
    K: np.ndarray              # (3, 3)
    T_world_camera: np.ndarray  # (4, 4) optical frame -> world
    masked: bool = False
    meta: dict = field(default_factory=dict)

    def age_s(self) -> float:
        return time.monotonic() - self.captured_monotonic_s

    def describe(self) -> dict:
        return {"camera_id": self.camera_id, "frame_id": self.frame_id, "scene_revision": self.scene_revision,
                "sim_time_s": self.sim_time_s, "width": int(self.rgb.shape[1]), "height": int(self.rgb.shape[0]),
                "K": self.K.tolist(), "T_world_camera": self.T_world_camera.tolist(), "masked": self.masked,
                "depth_convention": "optical-axis Z, meters, 0 = invalid",
                "optical_axes": "+X right, +Y down, +Z forward"}


def intrinsics(spec: CameraSpec) -> np.ndarray:
    f = (spec.height / 2.0) / np.tan(np.radians(spec.vertical_fov_deg) / 2.0)
    return np.array([[f, 0.0, spec.width / 2.0 - 0.5], [0.0, f, spec.height / 2.0 - 0.5], [0.0, 0.0, 1.0]])


def extrinsics(model: mujoco.MjModel, data: mujoco.MjData, cam_id: int) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = data.cam_xmat[cam_id].reshape(3, 3) @ OPTICAL_FROM_MJCAM
    T[:3, 3] = data.cam_xpos[cam_id]
    return T


def project(T_world_camera: np.ndarray, K: np.ndarray, p_world: np.ndarray) -> tuple[np.ndarray, float]:
    """World point -> (u, v) pixel coordinates and optical depth Z."""
    T_cw = np.linalg.inv(T_world_camera)
    pc = T_cw[:3, :3] @ p_world + T_cw[:3, 3]
    uv = (K @ (pc / pc[2]))[:2]
    return uv, float(pc[2])


def backproject(depth: np.ndarray, K: np.ndarray, T_world_camera: np.ndarray, us: np.ndarray, vs: np.ndarray):
    """Pixels with depth -> world points (N, 3)."""
    z = depth[vs, us].astype(float)
    x = (us - K[0, 2]) * z / K[0, 0]
    y = (vs - K[1, 2]) * z / K[1, 1]
    pc = np.stack([x, y, z], axis=1)
    return pc @ T_world_camera[:3, :3].T + T_world_camera[:3, 3]


class RGBDCamera:
    def __init__(self, model: mujoco.MjModel, spec: CameraSpec, cam_id: int):
        self.model = model
        self.spec = spec
        self.cam_id = cam_id
        self.K = intrinsics(spec)
        self._rgb = mujoco.Renderer(model, spec.height, spec.width)
        self._depth = mujoco.Renderer(model, spec.height, spec.width)
        self._depth.enable_depth_rendering()
        self.frame_id = 0
        self.masked = False     # simulated lens cover (developer fault injection)
        self._opt = mujoco.MjvOption()

    def capture(self, data: mujoco.MjData, scene_revision: int) -> CameraObservation:
        self._rgb.update_scene(data, camera=self.cam_id, scene_option=self._opt)
        rgb = self._rgb.render().copy()
        self._depth.update_scene(data, camera=self.cam_id, scene_option=self._opt)
        depth = self._depth.render().astype(np.float32).copy()
        far = 0.99 * self.model.vis.map.zfar * self.model.stat.extent
        depth[(depth <= 0) | (depth >= far) | ~np.isfinite(depth)] = 0.0
        if self.masked:
            rgb[:] = 0
            depth[:] = 0.0
        self.frame_id += 1
        return CameraObservation(self.spec.id, self.frame_id, scene_revision, float(data.time), time.monotonic(),
                                 rgb, depth, self.K.copy(), extrinsics(self.model, data, self.cam_id),
                                 masked=self.masked)

    def close(self) -> None:
        # Renderer.close() frees the MjrContext *before* its GL context and without making that context current.
        # If another renderer's context is current (scene rebuilds create the new renderers first), the freed
        # OpenGL object ids would be deleted in the new context. Make each renderer's own context current first.
        for r in (self._rgb, self._depth):
            try:
                ctx = getattr(r, "_gl_context", None)
                if ctx is not None:
                    ctx.make_current()
                r.close()
            except Exception:
                pass
