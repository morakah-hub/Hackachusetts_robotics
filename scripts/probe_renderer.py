"""Development probe: print MuJoCo Renderer.close/render source and test renderer replacement order."""
import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend"))

import mujoco  # noqa: E402

print(inspect.getsourcefile(mujoco.Renderer))
src = inspect.getsource(mujoco.Renderer.close)
print(src)
r = inspect.getsource(mujoco.Renderer.render)
print("\n".join(line for line in r.splitlines() if "context" in line or "def " in line))
print("attrs:", [a for a in vars(mujoco.Renderer(mujoco.MjModel.from_xml_string("<mujoco/>"), 8, 8)) if "context" in a])
