"""STL import: bounded parsing, canonical meshes, eligibility analysis and a content-addressed asset store.

Accepted input: binary STL (size must equal 84 + 50 * triangle count) or ASCII STL ('solid' ... 'vertex x y z').
Limits: MAX_UPLOAD_BYTES, MAX_TRIANGLES; every coordinate must be finite. Degenerate triangles are dropped.

Canonical asset: the uploaded triangles scaled to meters (explicit units * scale), rotated by the user's 90-degree
orientation correction, and translated so the bounding-box center is the origin. That origin is the part's body
frame and the center of its box collision proxy. The canonical mesh is stored as a binary STL whose SHA-256 gives
the asset id ("a_" + 16 hex), so the browser and MuJoCo always use the same geometry and user file names never
become paths. The original file name is kept only as display metadata.
"""

from __future__ import annotations

import hashlib
import json
import re
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
USER_DIR = ROOT / "user_data"
USER_ASSETS = USER_DIR / "assets"
EXAMPLE_ASSETS = ROOT / "examples" / "assets"

MAX_UPLOAD_BYTES = 20 * 1024 * 1024
MAX_TRIANGLES = 100_000
UNITS = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254}
ASSET_ID_RE = re.compile(r"^a_[0-9a-f]{16}$")

# Manipulation limits of the validated top-down parallel-jaw skill (Panda hand opens to 80 mm).
GRASP_WIDTH_MIN, GRASP_WIDTH_MAX = 0.020, 0.065   # short horizontal side (closing direction)
LONG_SIDE_MAX = 0.12
HEIGHT_MIN, HEIGHT_MAX = 0.035, 0.10
MASS_MAX = 0.6
FILL_MIN = 0.75          # mesh volume / bounding-box volume needed for an honest box proxy
TOP_FACE_MIN = 0.6       # flat top area / bounding-box footprint needed by the camera localizer
FIXTURE_MAX_SIDE = 0.30

MATERIALS = {
    "pla_20": {"label": "PLA print, ~20% infill", "density_kg_m3": 550.0, "friction": 0.8},
    "pla_solid": {"label": "PLA print, 100% infill", "density_kg_m3": 1240.0, "friction": 0.8},
    "petg_20": {"label": "PETG print, ~20% infill", "density_kg_m3": 590.0, "friction": 0.7},
    "wood": {"label": "Hardwood", "density_kg_m3": 700.0, "friction": 0.9},
    "aluminium": {"label": "Aluminium", "density_kg_m3": 2700.0, "friction": 1.0},
}
MATERIAL_NOTE = ("Mass = effective density x mesh volume (bounding box if the mesh is open). Print densities are "
                 "typical values for walls plus infill; weigh the real part and use Advanced for accuracy.")


class AssetError(ValueError):
    pass


def _check_triangles(tris: np.ndarray) -> np.ndarray:
    if not np.all(np.isfinite(tris)):
        raise AssetError("STL contains non-finite coordinates (NaN or infinity)")
    if np.abs(tris).max() > 1e6:
        raise AssetError("STL coordinates exceed 1e6 units")
    area = np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    scale = max(1e-9, float(np.ptp(tris.reshape(-1, 3), axis=0).max()))
    tris = tris[area > 1e-12 * scale * scale]
    if len(tris) < 4:
        raise AssetError("STL has fewer than 4 non-degenerate triangles")
    return tris


def parse_stl(data: bytes) -> tuple[np.ndarray, str]:
    """Bytes -> (N, 3, 3) float64 triangles in source units, and the detected format."""
    if len(data) > MAX_UPLOAD_BYTES:
        raise AssetError(f"file is larger than {MAX_UPLOAD_BYTES // (1024 * 1024)} MB")
    if len(data) < 15:
        raise AssetError("file is too small to be an STL")
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            if n == 0:
                raise AssetError("binary STL contains no triangles")
            if n > MAX_TRIANGLES:
                raise AssetError(f"binary STL has {n} triangles; the limit is {MAX_TRIANGLES}")
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                                count=n, offset=84)
            return _check_triangles(rec["v"].astype(np.float64)), "binary"
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        raise AssetError("not a binary STL (file size does not match its triangle count) and not ASCII text") from None
    if not text.lstrip().lower().startswith("solid"):
        raise AssetError("not a binary STL (file size does not match its triangle count) and ASCII STL must start "
                         "with 'solid'")
    nums = re.findall(r"^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)\s*$", text, flags=re.M | re.I)
    if not nums:
        raise AssetError("ASCII STL contains no 'vertex' lines")
    if len(nums) % 3:
        raise AssetError("ASCII STL vertex count is not a multiple of 3")
    if len(nums) // 3 > MAX_TRIANGLES:
        raise AssetError(f"ASCII STL has {len(nums) // 3} triangles; the limit is {MAX_TRIANGLES}")
    try:
        v = np.array(nums, dtype=np.float64)
    except ValueError:
        raise AssetError("ASCII STL contains a non-numeric vertex coordinate") from None
    return _check_triangles(v.reshape(-1, 3, 3)), "ascii"


def _rot(axis: str, deg: int) -> np.ndarray:
    k = int(deg) // 90 % 4
    c, s = [1, 0, -1, 0][k], [0, 1, 0, -1][k]
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def mesh_from_triangles(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Deduplicate vertices on a 0.1 micron grid -> (verts (V,3), faces (F,3))."""
    v = tris.reshape(-1, 3)
    key = np.round(v * 1e7).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    verts = v[first]
    faces = inv.reshape(-1, 3)
    ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[ok]


def canonicalize(tris: np.ndarray, units: str, scale: float = 1.0, rot_deg=(0, 0, 0)) -> np.ndarray:
    if units not in UNITS:
        raise AssetError(f"unknown units '{units}' (use one of {', '.join(UNITS)})")
    if not (np.isfinite(scale) and 0.01 <= scale <= 100):
        raise AssetError("scale must be between 0.01 and 100")
    for r in rot_deg:
        if int(r) % 90:
            raise AssetError("orientation corrections are multiples of 90 degrees")
    R = _rot("z", rot_deg[2]) @ _rot("y", rot_deg[1]) @ _rot("x", rot_deg[0])
    v = tris.reshape(-1, 3) * (UNITS[units] * float(scale)) @ R.T
    lo, hi = v.min(0), v.max(0)
    return (v - (lo + hi) / 2).reshape(-1, 3, 3)


def analyze(tris: np.ndarray) -> dict:
    """Dimensions, closedness, volume and manipulation eligibility of a canonical (meters, centered) mesh."""
    verts, faces = mesh_from_triangles(tris)
    lo, hi = verts.min(0), verts.max(0)
    dims = hi - lo
    e = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    _, counts = np.unique(e, axis=0, return_counts=True)
    watertight = bool(np.all(counts == 2))
    p0, p1, p2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    volume = float(abs(np.einsum("ij,ij->i", p0, np.cross(p1, p2)).sum()) / 6.0)
    box_vol = float(np.prod(dims))
    fill = volume / box_vol if (watertight and box_vol > 0) else None
    n = np.cross(p1 - p0, p2 - p0)
    area = np.linalg.norm(n, axis=1)
    nz = np.divide(n[:, 2], area, out=np.zeros_like(area), where=area > 0)
    zc = (p0[:, 2] + p1[:, 2] + p2[:, 2]) / 3
    top = (nz > 0.98) & (zc > hi[2] - 1e-4)
    top_frac = float(area[top].sum() / 2 / max(dims[0] * dims[1], 1e-12))
    short, long_ = sorted(dims[:2])
    mov = []
    if dims.min() < 0.002:
        mov.append("part is flat (thinner than 2 mm)")
    if not watertight:
        mov.append("mesh is not closed (watertight), so its volume and shape fill cannot be checked")
    elif fill < FILL_MIN:
        mov.append(f"shape fills only {fill:.0%} of its bounding box (needs {FILL_MIN:.0%}); a box collision proxy "
                   f"would misrepresent it")
    if not (GRASP_WIDTH_MIN <= short <= GRASP_WIDTH_MAX):
        mov.append(f"grasp width (short side) {1e3 * short:.0f} mm is outside {1e3 * GRASP_WIDTH_MIN:.0f}-"
                   f"{1e3 * GRASP_WIDTH_MAX:.0f} mm")
    if long_ > LONG_SIDE_MAX:
        mov.append(f"long side {1e3 * long_:.0f} mm exceeds {1e3 * LONG_SIDE_MAX:.0f} mm")
    if not (HEIGHT_MIN <= dims[2] <= HEIGHT_MAX):
        mov.append(f"height {1e3 * dims[2]:.0f} mm is outside {1e3 * HEIGHT_MIN:.0f}-{1e3 * HEIGHT_MAX:.0f} mm")
    if top_frac < TOP_FACE_MIN:
        mov.append(f"flat top face covers {top_frac:.0%} of the footprint (needs {TOP_FACE_MIN:.0%}); the camera "
                   f"localizer measures the top face")
    fix = []
    if not watertight or (fill is not None and fill < FILL_MIN):
        fix.append("concave or open shapes are not converted to a single box obstacle in this version "
                   "(use it as a visual reference)")
    if dims.max() > FIXTURE_MAX_SIDE:
        fix.append(f"largest side {1e3 * dims.max():.0f} mm exceeds {1e3 * FIXTURE_MAX_SIDE:.0f} mm")
    if dims.max() > 0.6:
        raise AssetError(f"part is {1e3 * dims.max():.0f} mm across after scaling; check the units (limit 600 mm)")
    if dims.max() < 0.005:
        raise AssetError(f"part is only {1e3 * dims.max():.2f} mm across after scaling; check the units")
    return {"dims_m": dims.tolist(), "triangles": int(len(faces)), "vertices": int(len(verts)),
            "watertight": watertight, "volume_m3": volume if watertight else None, "box_volume_m3": box_vol,
            "fill_ratio": fill, "top_face_fraction": top_frac, "collision_proxy": "box (bounding box of the mesh)",
            "eligible": {"movable": {"ok": not mov, "reasons": mov}, "fixture": {"ok": not fix, "reasons": fix},
                         "visual": {"ok": True, "reasons": []}}}


def stl_bytes(tris: np.ndarray, header: str = "Armstrong canonical mesh, meters") -> bytes:
    t = np.asarray(tris, np.float32).reshape(-1, 3, 3)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0).astype(np.float32)
    rec = np.zeros(len(t), dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = n, t
    return header.encode("ascii")[:80].ljust(80, b" ") + struct.pack("<I", len(t)) + rec.tobytes()


def safe_display_name(name: str) -> str:
    base = re.split(r"[\\/]", str(name or "part.stl"))[-1]
    base = re.sub(r"[^A-Za-z0-9 ._()-]", "_", base)[:60].strip() or "part.stl"
    return base


def import_stl(data: bytes, filename: str, units: str, scale: float = 1.0, rot_deg=(0, 0, 0),
               store_dir: Path | None = None) -> dict:
    """Parse, canonicalize, analyze and store. Returns the asset metadata (no paths)."""
    tris, fmt = parse_stl(data)
    canon = canonicalize(tris, units, scale, rot_deg)
    info = analyze(canon)
    blob = stl_bytes(canon)
    asset_id = "a_" + hashlib.sha256(blob).hexdigest()[:16]
    meta = {"id": asset_id, "source_name": safe_display_name(filename), "source_format": fmt, "units": units,
            "scale": float(scale), "rotation_deg": [int(r) for r in rot_deg], "source_triangles": int(len(tris)),
            **info}
    d = store_dir or USER_ASSETS
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{asset_id}.stl").write_bytes(blob)
    (d / f"{asset_id}.json").write_text(json.dumps(meta, indent=1))
    return meta


def asset_path(asset_id: str) -> Path:
    if not ASSET_ID_RE.match(str(asset_id)):
        raise AssetError(f"invalid asset id {asset_id!r}")
    for d in (USER_ASSETS, EXAMPLE_ASSETS):
        p = d / f"{asset_id}.stl"
        if p.is_file():
            return p
    raise AssetError(f"mesh asset {asset_id} is missing")


def asset_meta(asset_id: str) -> dict:
    p = asset_path(asset_id)
    j = p.with_suffix(".json")
    if j.is_file():
        return json.loads(j.read_text())
    return {"id": asset_id, **analyze(load_triangles(asset_id))}


def load_triangles(asset_id: str) -> np.ndarray:
    tris, _ = parse_stl(asset_path(asset_id).read_bytes())
    return tris


def store_canonical_blob(asset_id: str, blob: bytes) -> dict:
    """Store an asset received in a project bundle after checking its content hash and contents."""
    if not ASSET_ID_RE.match(asset_id):
        raise AssetError(f"invalid asset id {asset_id!r}")
    if "a_" + hashlib.sha256(blob).hexdigest()[:16] != asset_id:
        raise AssetError(f"asset {asset_id}: content does not match its id")
    tris, _ = parse_stl(blob)
    info = analyze(tris)
    USER_ASSETS.mkdir(parents=True, exist_ok=True)
    (USER_ASSETS / f"{asset_id}.stl").write_bytes(blob)
    meta = {"id": asset_id, "source_name": "from project bundle", "source_format": "binary", "units": "m",
            "scale": 1.0, "rotation_deg": [0, 0, 0], "source_triangles": int(len(tris)), **info}
    j = USER_ASSETS / f"{asset_id}.json"
    if not j.exists():
        j.write_text(json.dumps(meta, indent=1))
    return meta


def material_mass(material: str, meta: dict | None, dims) -> tuple[float, float]:
    mat = MATERIALS.get(material)
    if mat is None:
        raise AssetError(f"unknown material preset '{material}'")
    vol = (meta or {}).get("volume_m3") or float(np.prod(dims))
    return float(mat["density_kg_m3"] * vol), float(mat["friction"])
