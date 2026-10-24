"""Image-derived localization of one upright, known-size, known-color cuboid.

Inputs: a CameraObservation (RGB, metric depth, K, T_world_camera) and a catalog entry with
the block's color class and full dimensions, plus the known support height of the table.
This module never receives MjData, simulator poses, editor coordinates or segmentation IDs.

Pipeline: color threshold -> connected components (reject none / ambiguous)
-> back-projection of depth pixels to world points -> top-face points (highest surface) ->
height consistency check against the known block height -> rectangle fit on the top-face
points (yaw from convex-hull edge directions) -> center = rectangle midpoint, z = top - height / 2.
Orientation is constrained to an upright block (roll = pitch = 0); yaw is reported modulo 90 deg.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from .camera import CameraObservation, backproject

MIN_PIXELS = 25
MIN_TOP_POINTS = 12
TOP_BAND_M = 0.003   # wider bands pull in near side-face pixels and bias the center toward the camera
HEIGHT_TOL_M = 0.012
EXTENT_TOL_M = 0.014
MAX_AGE_S = 2.0


def orange_mask(rgb: np.ndarray) -> np.ndarray:
    r, g, b = (rgb[..., i].astype(int) for i in range(3))
    return (r > 90) & (r - g > 40) & (r - b > 70) & (g > b)


COLOR_CLASSES = {"orange": orange_mask}


def _fit_rectangle(xy: np.ndarray):
    """Yaw (mod 90 deg), side extents and center of the top-face point set.

    Yaw: length-weighted circular mean of 4*theta over convex-hull edges (a sampled square's hull
    is dominated by edges parallel to its sides; short corner edges carry little weight).
    Extents/center: robust min/max of the points in the yaw-aligned frame."""
    from scipy.spatial import ConvexHull

    hull = xy[ConvexHull(xy).vertices]
    e = np.roll(hull, -1, axis=0) - hull
    L = np.linalg.norm(e, axis=1)
    th = np.arctan2(e[:, 1], e[:, 0])
    yaw = float(np.arctan2(np.sum(L * np.sin(4 * th)), np.sum(L * np.cos(4 * th))) / 4)
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s], [-s, c]])              # world -> block frame
    p = xy @ R.T
    lo, hi = np.percentile(p, 0.5, axis=0), np.percentile(p, 99.5, axis=0)
    return yaw, hi - lo, R.T @ ((lo + hi) / 2)


def estimate_block(obs: CameraObservation, catalog: dict, current_revision: int | None = None) -> dict:
    """Return an ObservedObject dict: valid, reasons, pose (position, yaw, quaternion_wxyz) and diagnostics."""
    reasons: list[str] = []
    size = np.asarray(catalog["size_m"], float)
    support_z = float(catalog["support_z_m"])
    out = {"id": catalog["id"], "frame_id": obs.frame_id, "scene_revision": obs.scene_revision,
           "sim_time_s": obs.sim_time_s, "valid": False, "reasons": reasons, "position_m": None, "yaw_rad": None,
           "quaternion_wxyz": None, "size_m": size.tolist(), "pixels": 0, "top_points": 0,
           "top_z_m": None, "top_extent_m": None, "source": "camera_rgbd"}
    if current_revision is not None and obs.scene_revision != current_revision:
        reasons.append(f"stale frame: scene revision {obs.scene_revision}, current {current_revision}")
        return out
    if obs.age_s() > MAX_AGE_S:
        reasons.append(f"stale frame: {obs.age_s():.1f} s old")
        return out
    depth = obs.depth_m
    valid_depth = depth > 0
    if valid_depth.mean() < 0.5:
        reasons.append("camera image unavailable: less than half of the depth pixels are valid "
                       "(camera masked or covered)")
        return out
    mask = COLOR_CLASSES[catalog["color_class"]](obs.rgb) & valid_depth
    labels, n = ndimage.label(mask)
    if n == 0:
        reasons.append(f"no {catalog['color_class']} block visible in the camera image")
        return out
    sizes = ndimage.sum(mask, labels, index=np.arange(1, n + 1))
    order = np.argsort(sizes)[::-1]
    largest = float(sizes[order[0]])
    out["pixels"] = int(largest)
    if largest < MIN_PIXELS:
        reasons.append(f"block region too small ({int(largest)} px < {MIN_PIXELS}); occluded or out of view")
        return out
    if n > 1 and sizes[order[1]] >= max(MIN_PIXELS, 0.3 * largest):
        reasons.append(f"ambiguous: {int(np.sum(sizes >= max(MIN_PIXELS, 0.3 * largest)))} separate "
                       f"{catalog['color_class']} regions")
        return out
    region = labels == (order[0] + 1)
    vs, us = np.nonzero(region)
    pts = backproject(depth, obs.K, obs.T_world_camera, us, vs)
    z_hi = np.percentile(pts[:, 2], 95)
    # Top-face points: the tight height band already rejects mixed edge pixels (their depth belongs to
    # the side face or the table). No pixel erosion: under perspective a far-edge pixel covers more
    # table distance than a near-edge pixel, so eroding biases the center toward the camera (~2-3 mm).
    top = pts[np.abs(pts[:, 2] - z_hi) < TOP_BAND_M]
    out["top_points"] = int(len(top))
    if len(top) < MIN_TOP_POINTS:
        reasons.append(f"top face not visible enough ({len(top)} points)")
        return out
    top_z = float(np.median(top[:, 2]))
    out["top_z_m"] = top_z
    expected_top = support_z + size[2]
    if abs(top_z - expected_top) > HEIGHT_TOL_M:
        reasons.append(f"visible top surface at z = {top_z:.3f} m, expected {expected_top:.3f} m for an upright "
                       f"block on the table (tilted, stacked, lifted or partly occluded)")
        return out
    yaw, ext, center = _fit_rectangle(top[:, :2])
    out["top_extent_m"] = [float(ext[0]), float(ext[1])]
    nominal = size[:2]
    if np.any(np.abs(np.sort(ext) - np.sort(nominal)) > EXTENT_TOL_M):
        reasons.append(f"top face measures {1e3 * ext[0]:.0f} x {1e3 * ext[1]:.0f} mm, expected "
                       f"{1e3 * nominal[0]:.0f} x {1e3 * nominal[1]:.0f} mm (partly occluded or not the block)")
        return out
    pos = np.array([center[0], center[1], top_z - size[2] / 2])
    out.update(valid=True, position_m=pos.tolist(), yaw_rad=float(yaw),
               quaternion_wxyz=[float(np.cos(yaw / 2)), 0.0, 0.0, float(np.sin(yaw / 2))])
    return out
