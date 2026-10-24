"""Print the versions of the project's Python dependencies (used for HANDOFF.md)."""
import platform

import fastapi
import mujoco
import numpy
import osqp
import PIL
import scipy
import uvicorn

print("python", platform.python_version())
for mod in (mujoco, numpy, scipy, osqp, fastapi, uvicorn, PIL):
    print(mod.__name__, getattr(mod, "__version__", "?"))
