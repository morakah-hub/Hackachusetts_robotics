"""Projects: versioned scene + workflow + settings. Shipped examples (examples/projects, read-only) are kept apart
from private user projects (user_data/projects, git-ignored).

Portable bundle (.armstrong.zip): project.json + assets/<asset_id>.stl for every referenced mesh. Asset ids are
content hashes of the canonical mesh, so imports verify every file. No absolute paths are stored anywhere.
"""

from __future__ import annotations

import io
import json
import re
import time
import zipfile
from pathlib import Path

from . import assets as am

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_PROJECTS = ROOT / "examples" / "projects"
USER_PROJECTS = am.USER_DIR / "projects"
FORMAT = "armstrong-project"
VERSION = 1
MAX_BUNDLE_BYTES = 40 * 1024 * 1024
MAX_PROJECT_JSON = 1024 * 1024
MAX_ENTRIES = 40
ENTRY_RE = re.compile(r"^(project\.json|assets/a_[0-9a-f]{16}\.stl)$")
PID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,47}$")


class ProjectError(ValueError):
    pass


def make_project(name: str, scene: dict, recipe: dict, settings: dict | None = None, description: str = "") -> dict:
    return {"format": FORMAT, "version": VERSION, "name": str(name)[:80], "description": str(description)[:400],
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "scene": scene, "recipe": recipe,
            "settings": settings or {"perception_mode": "camera_rgbd"}}


def check_project(p: dict) -> dict:
    if not isinstance(p, dict) or p.get("format") != FORMAT:
        raise ProjectError("not an Armstrong project file")
    if int(p.get("version", 0)) != VERSION:
        raise ProjectError(f"unsupported project version {p.get('version')} (this build reads version {VERSION})")
    if not isinstance(p.get("scene"), dict):
        raise ProjectError("project has no scene")
    if not isinstance(p.get("recipe", {}), dict):
        raise ProjectError("project workflow is malformed")
    return p


def slug(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", str(name).lower()).strip("-")[:40]
    return s or "project"


def list_projects() -> list[dict]:
    out = []
    for source, d in (("example", EXAMPLE_PROJECTS), ("user", USER_PROJECTS)):
        if not d.is_dir():
            continue
        for f in sorted(d.glob("*.json")):
            try:
                p = json.loads(f.read_text(encoding="utf-8"))
                check_project(p)
            except Exception:
                continue
            sc_ = p["scene"]
            out.append({"id": f.stem, "source": source, "name": p.get("name", f.stem),
                        "description": p.get("description", ""), "saved_at": p.get("saved_at"),
                        "parts": len(sc_.get("parts", [])), "steps": len((p.get("recipe") or {}).get("steps", []))})
    return out


def load(source: str, pid: str) -> dict:
    if source not in ("example", "user") or not PID_RE.match(str(pid)):
        raise ProjectError("invalid project reference")
    d = EXAMPLE_PROJECTS if source == "example" else USER_PROJECTS
    f = d / f"{pid}.json"
    if not f.is_file():
        raise ProjectError(f"project {pid!r} not found")
    return check_project(json.loads(f.read_text(encoding="utf-8")))


def save_user(project: dict) -> str:
    check_project(project)
    USER_PROJECTS.mkdir(parents=True, exist_ok=True)
    pid = slug(project["name"])
    (USER_PROJECTS / f"{pid}.json").write_text(json.dumps(project, indent=1), encoding="utf-8")
    return pid


def asset_ids(project: dict) -> list[str]:
    return sorted({p["asset_id"] for p in project["scene"].get("parts", []) if p.get("asset_id")})


def export_bundle(project: dict) -> bytes:
    check_project(project)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("project.json", json.dumps(project, indent=1))
        for aid in asset_ids(project):
            z.writestr(f"assets/{aid}.stl", am.asset_path(aid).read_bytes())
    return buf.getvalue()


def import_bundle(data: bytes) -> dict:
    """Validate a bundle, store its assets (hash-checked) and return the project. Nothing is extracted by path."""
    if len(data) > MAX_BUNDLE_BYTES:
        raise ProjectError(f"bundle is larger than {MAX_BUNDLE_BYTES // (1024 * 1024)} MB")
    try:
        z = zipfile.ZipFile(io.BytesIO(data))
    except zipfile.BadZipFile:
        raise ProjectError("not a ZIP file") from None
    with z:
        infos = z.infolist()
        if len(infos) > MAX_ENTRIES:
            raise ProjectError(f"bundle has more than {MAX_ENTRIES} entries")
        names = set()
        for i in infos:
            if i.is_dir():
                continue
            if not ENTRY_RE.match(i.filename):
                raise ProjectError(f"unexpected file in bundle: {i.filename[:80]!r}")
            limit = MAX_PROJECT_JSON if i.filename == "project.json" else am.MAX_UPLOAD_BYTES
            if i.file_size > limit:
                raise ProjectError(f"{i.filename} is too large")
            names.add(i.filename)
        if "project.json" not in names:
            raise ProjectError("bundle has no project.json")
        try:
            project = check_project(json.loads(z.read("project.json").decode("utf-8")))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise ProjectError("project.json is not valid JSON") from None
        for aid in asset_ids(project):
            entry = f"assets/{aid}.stl"
            if entry not in names:
                try:
                    am.asset_path(aid)       # already available locally (e.g. a shipped example)
                    continue
                except am.AssetError:
                    raise ProjectError(f"bundle is missing mesh {aid}") from None
            blob = z.read(entry)
            if len(blob) > am.MAX_UPLOAD_BYTES:
                raise ProjectError(f"{entry} is too large")
            am.store_canonical_blob(aid, blob)
    return project
