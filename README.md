# Armstrong

Local browser app for configuring and evaluating a tabletop pick-and-place task: a MuJoCo
simulation of a Franka Emika Panda, a simulated RGB-D camera that locates the block from images,
a FastAPI backend and a React/Three.js workspace.

## Launch (Windows PowerShell, tested 24 Oct 2026)

```powershell
cd C:\Users\rkhmh\Downloads\Hackachusetts_armstrong
powershell -ExecutionPolicy Bypass -File .\start.ps1
```

Open **http://127.0.0.1:8765/**. Press Ctrl+C in that window to stop
(or `powershell -ExecutionPolicy Bypass -File .\scripts\stop.ps1` from another window).

`start.ps1` resolves everything from its own folder. It:
1. creates `.venv` with the system `python` and installs `requirements.txt` if packages are missing;
2. downloads the official portable Node.js LTS (v24.22.1, SHA-256 verified) into `.tools\node` and
   builds `frontend\dist` when the build is missing or older than `frontend\src`;
3. refuses to start if the port is busy, then serves the simulation, API and UI on 127.0.0.1 only.

Options: `-Port 8770`, `-Rebuild`. Prerequisite: Python 3.11+ on PATH (tested with 3.13.7) and an
OpenGL-capable display driver (MuJoCo renders the camera through GLFW/WGL; no EGL on Windows).

## Use

1. **Scene**: drag the orange block or the teal pad on the table (or type x/y/yaw and Apply). The shaded band
   is the validated workspace (0.30–0.72 m from the robot base, |y| ≤ 0.42 m). Edits are accepted only while
   stopped; each accepted edit increments the scene revision and clears old estimates and plans.
2. **Task**: pick = block, place on = pad, locate by = *Camera RGB-D* (default) or *Known world — development
   only* (plans from the simulator pose, labelled everywhere).
3. **Run**. Phases: observe → plan → approach → descend → close → lift → transport → lower → release →
   retreat → verify. **Pause** freezes physics time (Step 10 ms / 100 ms while paused), **Stop** ends the run
   and holds the arm, **Reset** returns the arm home and the block to its configured start.
4. **Right column**: live RGB/depth with frame id and age, the camera localization, and the measured result
   (from the ground-truth evaluator, kept separate from perception). Export a run as JSON.
5. **Developer** drawer: controller in use and gains, timing, estimate/plan/truth JSON, fault injection
   (mask camera, disable gripper) and the event log.

## How it works

| Part | File | Notes |
|---|---|---|
| Scene | `backend/armstrong/scene.py` | Table top z = 0.40 m, Panda mounted at (0, 0, 0.40), 50×50×60 mm 0.4 kg block (free joint, friction 1, condim 4), visual-only pad (mocap), one camera. Elliptic friction cone, impratio 10, dt 2 ms. |
| Robot | `backend/armstrong/robot.py` | Vendored Menagerie Panda (unmodified files); arm actuators converted to torque motors in the spec; gripper servo 1500 N/m. |
| Control | `controller.py` | Computed torque control at 500 Hz using MuJoCo inverse dynamics on a contact-free copy (payload not modelled). |
| IK / planning | `kinematics.py`, `trajectory.py` | Full-pose damped-least-squares IK, top-down grasp; min-jerk joint moves and straight Cartesian lines; sampled collision checks against known table/floor and the *estimated* block. |
| Task | `task.py` | Bounded state machine; each phase ends on measured conditions or a timeout with a stage, code and reason. |
| Camera | `camera.py` | 320×240 RGB + optical-axis depth (m), K from fovy, optical frame +X right/+Y down/+Z forward, 5 Hz. |
| Perception | `perception.py` | Color threshold → components (none/ambiguous rejected) → depth back-projection → top-face points → height check → rectangle fit → pose. Receives only images, calibration and catalog priors. |
| Evaluator | `evaluator.py` | Simulator truth: placement error, tilt, lift, slip, forbidden contacts. Never feeds perception or planning. |
| Worker | `sim.py` | Single owner of MjModel/MjData/renderers; bounded command queue; publishes snapshots and frames. |
| API | `api.py`, `server.py` | FastAPI + WebSocket on 127.0.0.1:8765; serves `frontend/dist`. |
| UI | `frontend/src` | React + Three.js; meshes are the compiled MuJoCo visual geoms, posed with the backend's body transforms. |

The block moves only through finger contact and friction. No weld, teleport or disabled collision is used during
a run. State is written only at initialization, for stopped-scene edits and on reset.

## Tests

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1          # backend suite (~40 s)
powershell -ExecutionPolicy Bypass -File .\scripts\npm.ps1 run build # frontend type-check + build
.\.venv\Scripts\python.exe scripts\headless_run.py camera_rgbd 0.45 0.15   # one fast headless run
.\.venv\Scripts\python.exe scripts\smoke_api.py                      # against a running server
```

Development with hot reload: run `start.ps1` (backend), then `powershell -ExecutionPolicy Bypass -File .\scripts\npm.ps1 run dev`
and open http://127.0.0.1:5173/ (Vite proxies `/api` to port 8765).

## Limits (current version)

- One upright orange cuboid of known size, one pad, one robot, one camera, one run at a time.
- The destination pad is a user-defined goal, not perceived. Table/floor geometry is known from the scene file.
- Objects can be moved only while stopped; there is no re-observation during transport.
- Torque MPC, OBJ import, obstacles and multi-object tasks are not implemented yet.

## Attribution

Franka Emika Panda model from Google DeepMind's MuJoCo Menagerie (Apache-2.0); see
`backend/assets/robots/franka_emika_panda/LICENSE` and `SOURCE.txt`. The design draws on the team's earlier
robotics-lab project (filtered copy in `handoff/reference/`, read-only, not part of this application); the
Armstrong code in `backend/` and `frontend/` was written in this Kiro session.
