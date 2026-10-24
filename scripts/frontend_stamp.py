"""Content stamp of the frontend sources, written into frontend/dist/.build-stamp after each build.

    python scripts/frontend_stamp.py check   -> exit 0 if dist matches the sources, 1 if a rebuild is needed
    python scripts/frontend_stamp.py write   -> record the stamp of the current sources

The hash covers src/, public/, index.html, package.json, package-lock.json, vite.config.ts and tsconfig.json, with
line endings normalized, so a Git checkout with different CRLF settings does not force a rebuild.
"""
import hashlib
import sys
from pathlib import Path

FRONT = Path(__file__).resolve().parent.parent / "frontend"
FILES = ["index.html", "package.json", "package-lock.json", "vite.config.ts", "tsconfig.json"]
DIRS = ["src", "public"]
STAMP = FRONT / "dist" / ".build-stamp"


def digest() -> str:
    h = hashlib.sha256()
    paths = [FRONT / f for f in FILES if (FRONT / f).is_file()]
    for d in DIRS:
        if (FRONT / d).is_dir():
            paths += [p for p in (FRONT / d).rglob("*") if p.is_file()]
    for p in sorted(paths, key=lambda x: x.relative_to(FRONT).as_posix()):
        h.update(p.relative_to(FRONT).as_posix().encode())
        h.update(b"\0")
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
        h.update(b"\0")
    return h.hexdigest()


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "write":
        STAMP.parent.mkdir(parents=True, exist_ok=True)
        STAMP.write_text(digest())
        return 0
    if not (FRONT / "dist" / "index.html").is_file() or not STAMP.is_file():
        return 1
    return 0 if STAMP.read_text().strip() == digest() else 1


if __name__ == "__main__":
    sys.exit(main())
