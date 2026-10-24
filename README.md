# Armstrong

A local, visual tabletop-workflow simulator for small workshops and people working with 3D-printed parts.
Open an example or import an STL part, check its size and physical assumptions, place parts and destinations, build a
short workflow (move, kit, grid, sort by category), run it on a simulated Franka Panda with a simulated RGB-D camera,
and save or export the whole project. Everything runs on your computer: MuJoCo physics, local image processing,
deterministic task recipes. No AI service, API key or account is needed at runtime.

Judges: see **[docs/JUDGE_QUICKSTART.md](docs/JUDGE_QUICKSTART.md)**.

## Launch (Windows PowerShell; run from the extracted/cloned folder)

```powershell
powershell -ExecutionPolicy Bypass -File .\start.ps1      # or double-click start.bat
```
Open the printed address, normally **http://127.0.0.1:8765/** (the next free port is used if 8765 is busy).
Stop with Ctrl+C. Options: `-Port 8770`, `-Rebuild`, `-SetupOnly`, `-AnyPython`.

The launcher resolves every path from its own folder (spaces are fine) and:
1. finds **Python 3.13** (tested 3.13.7; `-AnyPython` also accepts 3.12/3.14, which have wheels but are untested;
   3.11 lacks wheels for the pinned packages), creates `.venv` and installs `requirements.txt` whenever its hash changes;
2. rebuilds `frontend\dist` only when the frontend sources/lockfile no longer match `dist\.build-stamp`
   (a release ZIP ships a built frontend; a source checkout downloads the official portable Node.js LTS v24.22.1
   into `.tools\node`, SHA-256 verified, and runs `npm ci` + `npm run build`);
3. never stops other programs: if the port is busy it picks the next free one; serves on 127.0.0.1 only.
First run needs internet for the Python packages; afterwards it runs offline. Setup log: `runtime\setup.log`.

## What you can do

| Area | Details |
|---|---|
| Projects | Open shipped examples (`examples/projects`: single block regression, two-part kit, sort by category, 2×2 grid), Save to `user_data/projects`, Export/Import portable `.armstrong.zip` bundles (scene + workflow + settings + meshes, hash-checked, no local paths). |
| Parts | Up to 6 parts (4 movable). Primitive blocks or **imported STL** (binary or ASCII). Movable parts use one of four camera colors (orange, blue, green, magenta) that the camera uses to identify them. Fixtures are static known obstacles; visual references have no collision. |
| STL import | Units (mm/cm/m/in), scale, 90° orientation corrections, live preview with the box collision proxy, dimensions in mm, closedness and box-fill ratio, manipulation eligibility with reasons, material presets (PLA/PETG/wood/aluminium) or measured mass/friction. Bounded size (20 MB, 100k triangles), finite-coordinate checks, content-hash asset ids. |
| Destinations | Named pads with optional requested yaw (the robot rotates the part) and optional zone for sorting. |
| Workflow | Step cards (part → place on destination, or sort by category), reorder/remove, presets Move / Kit / Grid / Sort, explicit category → zone rules, validation against the current scene revision. |
| Run | Each step: fresh camera observation → plan (workspace, eligibility, occupancy, grasp axis, IK, collision check of approach/descent/lift/transport/lowering/retreat/return including the carried part) → physical execution → camera verification → ground-truth evaluation. The world is not reset between steps; a failing step stops the sequence and later steps are reported "not run". |
| Results | Per-step placement error (evaluator), camera estimate error (evaluation only), forbidden contacts, simulated and wall durations, run JSON export. |

The block/part moves only through finger contact and friction: no welds, teleports or disabled collisions during a run.
State is written only at initialization, for stopped-scene edits, and on reset.

## How it works

| Part | File | Notes |
|---|---|---|
| Scene | `backend/armstrong/scene.py` | Table top z = 0.40 m, Panda at (0, 0, 0.40). Parts/destinations with stable ids; MuJoCo addresses resolved by name. Elliptic cone, impratio 10, dt 2 ms. |
| Assets | `assets.py` | STL parsing, canonical meshes (meters, centered), eligibility analysis, content-addressed store. |
| Projects | `projects.py` | Versioned project files and validated ZIP bundles. |
| Robot | `robot.py` | Vendored Menagerie Panda (unmodified files); torque actuators and 1500 N/m gripper servo set on the spec. |
| Control | `controller.py` | Computed-torque control at 500 Hz, MuJoCo inverse dynamics on a contact-free copy. |
| Planning | `kinematics.py`, `trajectory.py` | Full-pose DLS IK, min-jerk joint moves, straight Cartesian lines, collision checks in a planning world built from proprioception + camera estimates + known static geometry. |
| Workflow | `task.py` | Recipe compiler and step executor (bounded state machine per step). |
| Camera | `camera.py` | 320×240 RGB + optical-axis depth, K from fovy, optical frame +X right/+Y down/+Z forward, 5 Hz. |
| Perception | `perception.py` | HSV color class → components (none/too small/ambiguous rejected) → depth back-projection → top face → height check → rectangle fit → side assignment for non-square parts. Gets images, calibration and catalog priors only. |
| Evaluator | `evaluator.py` | Simulator truth per step: placement, height, tilt, requested yaw, slip, forbidden contacts. Never feeds perception or planning. |
| Worker | `sim.py` | Single owner of MuJoCo + renderers; atomic structural rebuilds (old scene kept if a rebuild fails); bounded command queue. |
| API / UI | `api.py`, `server.py`, `frontend/src` | FastAPI + WebSocket on 127.0.0.1; React + Three.js using the compiled MuJoCo geometry. |

## Tests and tools

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1            # backend suite
powershell -ExecutionPolicy Bypass -File .\scripts\npm.ps1 run build   # frontend type-check + build
.\.venv\Scripts\python.exe scripts\run_project.py two_part_kit         # headless example workflow
.\.venv\Scripts\python.exe scripts\diag_perception.py                  # camera accuracy grid
.\.venv\Scripts\python.exe scripts\make_examples.py                    # regenerate example STLs/projects
powershell -ExecutionPolicy Bypass -File .\scripts\package_release.ps1 # judge ZIP in release\
```
Frontend development with hot reload: run `start.ps1`, then `scripts\npm.ps1 run dev` and open http://127.0.0.1:5173/.

## Limits

- Upright boxlike movable parts only (20–65 mm grasp side, 35–100 mm tall, ≤ 120 mm long side, ≤ 0.6 kg, box fill ≥ 75 %,
  flat top ≥ 60 % of the footprint). Imported shapes are simulated through their bounding-box proxy (approximate physics).
- One part per camera color; parts must be visible to the fixed camera (occlusion fails honestly).
- Destination pads and fixtures are known, not perceived. No obstacle reconstruction, assembly, insertion or fastening.
- Torque MPC is not implemented (the UI says "planned").

## Attribution

Franka Emika Panda model from Google DeepMind's MuJoCo Menagerie (Apache-2.0); see
`backend/assets/robots/franka_emika_panda/LICENSE` and `SOURCE.txt`. Example STL files in `examples/` are generated by
`scripts/make_examples.py` for this project. The design draws on the team's earlier robotics-lab project (kept locally as
read-only background, not part of this application or its source tree); the Armstrong code in `backend/` and `frontend/`
was written with Kiro during the hackathon.
