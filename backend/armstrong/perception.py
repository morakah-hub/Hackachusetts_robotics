"""Image-derived localization of upright, known-size cuboid parts with distinct catalog colors.

Inputs: a CameraObservation (RGB, metric depth, K, T_world_camera), a catalog entry per part (id, color class,
full proxy dimensions in the part frame, support height) and the known static table geometry. The estimator
never receives MjData, simulator poses, editor coordinates or segmentation IDs.

Per part: color mask (HSV class) inside the known tabletop volume -> connected components (none / too small /
ambiguous rejected) -> back-projected depth points -> top-face points (highest surface) -> height check against the
known part height -> rectangle fit (yaw from convex-hull edge directions) -> side assignment for non-square parts
(measured extents vs known dimensions) -> pose. Orientation is constrained to upright (roll = pitch = 0). Yaw is
reported modulo 90 deg for square footprints and modulo 180 deg otherwise (a box is symmetric under those turns).
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull

from .camera import CameraObservation, backproject

MIN_PIXELS = 25
MIN_TOP_POINTS = 12
TOP_BAND_M = 0.003   # wider bands pull in near side-face pixels and bias the center toward the camera
HEIGHT_TOL_M = 0.012
EXTENT_TOL_M = 0.014
SQUARE_TOL_M = 0.008
MAX_AGE_S = 2.0

# Hue ranges (degrees), minimum saturation and value of each supported part color.
COLOR_CLASSES = {
    "orange": ((12.0, 42.0), 0.55, 0.30),
    "blue": ((205.0, 250.0), 0.45, 0.25),
    "green": ((90.0, 150.0), 0.45, 0.25),
    "magenta": ((285.0, 335.0), 0.45, 0.25),
}


def hsv(rgb: np.ndarray):
    x = rgb.astype(np.float32) / 255.0
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    mx, mn = x.max(-1), x.min(-1)
    d = mx - mn
    h = np.zeros_like(mx)
    nz = d > 1e-6
    rm = nz & (mx == r)
    gm = nz & (mx == g) & ~rm
    bm = nz & ~rm & ~gm
    h[rm] = (60 * ((g[rm] - b[rm]) / d[rm])) % 360
    h[gm] = 60 * ((b[gm] - r[gm]) / d[gm]) + 120
    h[bm] = 60 * ((r[bm] - g[bm]) / d[bm]) + 240
    s = np.divide(d, mx, out=np.zeros_like(d), where=mx > 1e-6)
    return h, s, mx


def color_mask(rgb: np.ndarray, color: str, hsv_cache=None) -> np.ndarray:
    (lo, hi), smin, vmin = COLOR_CLASSES[color]
    h, s, v = hsv_cache if hsv_cache is not None else hsv(rgb)
    return (h >= lo) & (h <= hi) & (s >= smin) & (v >= vmin)


def orange_mask(rgb: np.ndarray) -> np.ndarray:   # kept for scripts written against v0.1
    return color_mask(rgb, "orange")


def _fit_rectangle(xy: np.ndarray):
    """Yaw (mod 90 deg), extents along (yaw, yaw + 90 deg) and center of the top-face point set.

    Yaw: length-weighted circular mean of 4*theta over convex-hull edges (a sampled rectangle's hull is dominated
    by edges parallel to its sides; short corner edges carry little weight)."""
    hull = xy[ConvexHull(xy).vertices]
    e = np.roll(hull, -1, axis=0) - hull
    L = np.linalg.norm(e, axis=1)
    th = np.arctan2(e[:, 1], e[:, 0])
    yaw = float(np.arctan2(np.sum(L * np.sin(4 * th)), np.sum(L * np.cos(4 * th))) / 4)
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s], [-s, c]])              # world -> estimated frame
    p = xy @ R.T
    lo, hi = np.percentile(p, 0.5, axis=0), np.percentile(p, 99.5, axis=0)
    return yaw, hi - lo, R.T @ ((lo + hi) / 2)


def wrap(a: float, period: float) -> float:
    """Wrap an angle into (-period/2, period/2]."""
    return float(-((-a + period / 2) % period - period / 2))


class SceneObserver:
    """Shared per-frame work (HSV conversion, world points, tabletop volume) for several part estimates."""

    def __init__(self, obs: CameraObservation, table: dict):
        self.obs = obs
        self.hsv = hsv(obs.rgb)
        H, W = obs.depth_m.shape
        vs, us = np.mgrid[0:H, 0:W]
        valid = obs.depth_m > 0
        self.valid_fraction = float(valid.mean())
        pts = np.zeros((H, W, 3))
        if valid.any():
            pts[valid] = backproject(obs.depth_m, obs.K, obs.T_world_camera, us[valid], vs[valid])
        self.world = pts
        tz = table["top_z"]
        (cx, cy), (hx, hy) = table["center_xy"], table["half_xy"]
        self.volume = (valid & (pts[..., 2] > tz + 0.004) & (pts[..., 2] < tz + 0.16)
                       & (np.abs(pts[..., 0] - cx) < hx) & (np.abs(pts[..., 1] - cy) < hy))

    def estimate(self, entry: dict, current_revision: int | None = None) -> dict:
        return estimate_part(self.obs, entry, current_revision, observer=self)


def _result(obs, entry, size):
    return {"id": entry["id"], "frame_id": obs.frame_id, "scene_revision": obs.scene_revision,
            "sim_time_s": obs.sim_time_s, "valid": False, "reasons": [], "position_m": None, "yaw_rad": None,
            "yaw_period_rad": None, "quaternion_wxyz": None, "size_m": size.tolist(), "color": entry["color_class"],
            "pixels": 0, "top_points": 0, "top_z_m": None, "top_extent_m": None, "source": "camera_rgbd"}


def estimate_part(obs: CameraObservation, entry: dict, current_revision: int | None = None,
                  observer: SceneObserver | None = None, table: dict | None = None) -> dict:
    """Return an observed-object dict for one catalog entry: valid, reasons, pose and diagnostics."""
    size = np.asarray(entry["size_m"], float)
    out = _result(obs, entry, size)
    reasons = out["reasons"]
    name = entry.get("name", entry["id"])
    if current_revision is not None and obs.scene_revision != current_revision:
        reasons.append(f"stale frame: scene revision {obs.scene_revision}, current {current_revision}")
        return out
    if obs.age_s() > MAX_AGE_S:
        reasons.append(f"stale frame: {obs.age_s():.1f} s old")
        return out
    if observer is None:
        observer = SceneObserver(obs, table or entry["table"])
    if observer.valid_fraction < 0.5:
        reasons.append("camera image unavailable: less than half of the depth pixels are valid "
                       "(camera masked or covered)")
        return out
    support_z = float(entry["support_z_m"])
    mask = color_mask(obs.rgb, entry["color_class"], observer.hsv) & observer.volume
    labels, n = ndimage.label(mask)
    if n == 0:
        reasons.append(f"no {entry['color_class']} part ({name}) visible on the table")
        return out
    sizes = ndimage.sum(mask, labels, index=np.arange(1, n + 1))
    order = np.argsort(sizes)[::-1]
    largest = float(sizes[order[0]])
    out["pixels"] = int(largest)
    if largest < MIN_PIXELS:
        reasons.append(f"{name}: visible region too small ({int(largest)} px < {MIN_PIXELS}); occluded or out of view")
        return out
    big = int(np.sum(sizes >= max(MIN_PIXELS, 0.3 * largest)))
    if big > 1:
        reasons.append(f"ambiguous: {big} separate {entry['color_class']} regions on the table for {name}")
        return out
    region = labels == (order[0] + 1)
    pts = observer.world[region]
    z_hi = np.percentile(pts[:, 2], 95)
    top = pts[np.abs(pts[:, 2] - z_hi) < TOP_BAND_M]
    out["top_points"] = int(len(top))
    if len(top) < MIN_TOP_POINTS:
        reasons.append(f"{name}: top face not visible enough ({len(top)} points)")
        return out
    top_z = float(np.median(top[:, 2]))
    out["top_z_m"] = top_z
    expected_top = support_z + size[2]
    if abs(top_z - expected_top) > HEIGHT_TOL_M:
        reasons.append(f"{name}: visible top surface at z = {top_z:.3f} m, expected {expected_top:.3f} m for an "
                       f"upright part on the table (tilted, stacked, lifted or partly occluded)")
        return out
    nominal = size[:2]
    spread = np.ptp(top[:, :2], axis=0)
    if min(spread) < 0.5 * min(nominal):
        reasons.append(f"{name}: visible top face is only {1e3 * spread[0]:.0f} x {1e3 * spread[1]:.0f} mm "
                       f"(partly occluded)")
        return out
    try:
        yaw, ext, center = _fit_rectangle(top[:, :2])
    except Exception as e:   # degenerate point set (qhull)
        reasons.append(f"{name}: top face could not be fitted ({type(e).__name__}); partly occluded")
        return out
    if np.any(np.abs(np.sort(ext) - np.sort(nominal)) > EXTENT_TOL_M):
        reasons.append(f"{name}: top face measures {1e3 * ext[0]:.0f} x {1e3 * ext[1]:.0f} mm, expected "
                       f"{1e3 * nominal[0]:.0f} x {1e3 * nominal[1]:.0f} mm (partly occluded or another object)")
        out["top_extent_m"] = [float(ext[0]), float(ext[1])]
        return out
    square = abs(nominal[0] - nominal[1]) < SQUARE_TOL_M
    if square:
        period = np.pi / 2
    else:
        # Which measured side is the part's x side? Compare against the known dimensions.
        direct = abs(ext[0] - nominal[0]) + abs(ext[1] - nominal[1])
        swapped = abs(ext[0] - nominal[1]) + abs(ext[1] - nominal[0])
        if swapped < direct:
            yaw += np.pi / 2
            ext = ext[::-1]
        period = np.pi
    yaw = wrap(yaw, period)
    out["top_extent_m"] = [float(ext[0]), float(ext[1])]
    pos = np.array([center[0], center[1], top_z - size[2] / 2])
    out.update(valid=True, position_m=pos.tolist(), yaw_rad=float(yaw), yaw_period_rad=float(period),
               quaternion_wxyz=[float(np.cos(yaw / 2)), 0.0, 0.0, float(np.sin(yaw / 2))])
    return out


def estimate_block(obs: CameraObservation, catalog: dict, current_revision: int | None = None) -> dict:
    """v0.1 single-part interface (kept for scripts): catalog has id, color_class, size_m, support_z_m."""
    from .scene import TABLE_CENTER_XY, TABLE_HALF_XY, TABLE_TOP_Z
    table = {"top_z": TABLE_TOP_Z, "center_xy": TABLE_CENTER_XY, "half_xy": TABLE_HALF_XY}
    return estimate_part(obs, catalog, current_revision, table=table)
