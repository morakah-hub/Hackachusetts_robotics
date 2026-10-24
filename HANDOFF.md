# Armstrong — HANDOFF (updated 24 Oct 2026, session 2: STL parts, workflows, judge package)

Recoverable baselines: git tag `baseline-v0.1` (commit a851582, the session-1 single-block app); session-2 work is in
the following local commits on `main` (not pushed). Remote `origin` = github.com/morakah-hub/Hackachusetts_robotics.

## What runs now
- **Launch**: `powershell -ExecutionPolicy Bypass -File .\start.ps1` or double-click `start.bat` in the project folder →
  printed URL, normally **http://127.0.0.1:8765/** (next free port if busy). Stop: Ctrl+C or `scripts\stop.ps1`.
- **Projects**: shipped examples (`examples/projects`): Single block (v0.1 regression), Two-part kit (block + STL
  spacer, slot B requests yaw 0), Sort by category (3 parts, 2 zones), 2×2 arrangement (4 parts). Save →
  `user_data/projects`; Export/Import `.armstrong.zip` bundles (project.json + hash-checked meshes).
- **Parts**: up to 6 (4 movable) with stable ids; primitive blocks or imported STL (binary/ASCII); roles movable /
  fixture / visual; camera colors orange/blue/green/magenta (one per movable part); category for sorting; start pose,
  mass/friction (Advanced), remove. Structural edits rebuild the MuJoCo model atomically (old scene kept on failure).
- **STL import dialog**: units mm/cm/m/in, scale, 90° rotations, live preview with the box proxy, dimensions in mm,
  closed-mesh and box-fill checks, eligibility with reasons per role, material presets or measured mass/friction.
- **Destinations**: named pads, optional requested yaw (part rotated in transport, verified), optional zone.
- **Workflow builder**: step cards (part → place on destination, or sort), reorder/remove/add, presets Move / Kit /
  Grid / Sort, category → zone rules, validation status bound to the scene revision.
- **Run**: per step observe → plan (workspace, eligibility, occupancy, grasp across the short side, IK, collision check
  of approach/descent/lift/transport/lowering/retreat/return incl. the carried part) → physical execution → camera
  verify → ground-truth evaluation. No reset between steps; failures stop the sequence, later steps "not run".
- Camera panel, per-part localization (with evaluation-only error), per-step results table, run JSON export,
  Developer drawer (controller CTC, timing, estimates/plan/truth, faults, log), collision-proxy overlay toggle.

## Verification this session (actual results)
- Backend tests, per-file runs during development: test_assets 6, test_camera 7, test_api 7 passed; test_pick_place
  passed across partial runs. These runs happened before the last perception edit (occluded-top-face check, qhull
  guard), except test_api and the route tests. The full 35-test suite has not been confirmed after the final edits
  (see "Final checks").
- Occlusion and route rejection are now two independent tests (`test_pick_place.py`), verified together:
  `-k "occluded_part or blocked_route"` → **2 passed in 10.34 s**.
  - `test_occluded_part_blocks_motion`: a 250 mm wall between the camera and the block → fails at `observe` with
    PERCEPTION_UNAVAILABLE, no estimate, no plan, arm and block unmoved.
  - `test_blocked_route_is_rejected_before_motion`: a 60 mm wall 6 mm beyond the block's far (−x) face cannot occlude
    it. The camera estimate is valid and within 4 mm, then planning fails with ROUTE_BLOCKED naming `part 'wall'`. No
    plan is published and nothing moves. Control: the same wall 60 mm further away → the run succeeds with 0
    forbidden contacts.
  - Cause of the earlier failure: the combined test's 70 mm wall stood between the camera and the block and partly
    hid its top face. The strict perception check rejected it correctly, so the route check was never reached.
- Headless `scripts\run_project.py two_part_kit`: 2/2 steps, placement 0.49 / 1.38 mm, camera estimate error
  0.45 / 0.71 mm, spacer grasp width 38.95 mm (40 mm side), requested yaw error 0.01°, 0 forbidden contacts, 38.4 s sim.
- Perception grid (`scripts\diag_perception.py`, 27 poses each incl. yaw −1.1/0/0.4 rad): 50 mm block max xy error
  1.32 mm, max yaw error 1.2°; 40×60 mm STL spacer max xy 1.57 mm, max yaw error 2.4° (side assignment correct).
- Live server launched by the new start.ps1 (it ran npm ci + build, then served): API run of the Two-part kit →
  success 2/2, placement 0.48 / 1.38 mm, 38.4 s sim / 43.3 s wall. Headless Edge screenshots show step cards,
  slot markers "1 · Kit slot A", "2 · Kit slot B", per-part "seen" boxes, step/phase strip and progress bar.
- Wheel availability check (`scripts\check_wheels.ps1`): pinned packages have Windows wheels for Python 3.12–3.14,
  not 3.11. Only 3.13.7 is tested, so start.ps1 requires 3.13 unless `-AnyPython`.
- Found and fixed: MuJoCo `Renderer.close()` frees its MjrContext without making its own GL context current, which
  destroyed the new renderer's framebuffers after a scene rebuild (camera returned no depth). camera.py now makes each
  renderer's context current before closing it.

## Installed versions (project-local)
Python 3.13.7 (.venv): mujoco 3.17.0, numpy 2.5.4, scipy 1.18.1, osqp 1.1.3 (unused), fastapi 0.162.0, uvicorn 0.54.0,
pillow 13.0.0, pytest 9.1.1; dev-only playwright 1.64.1 (requirements-dev.txt). Node v24.22.1 portable (.tools\node),
vite 7.3.7, react 19, three 0.180, typescript 5.x.

## Known limits / issues
- Movable parts: upright, boxlike (fill ≥ 75 %, flat top ≥ 60 %), 20–65 mm grasp side, 35–100 mm tall, long side ≤ 120 mm,
  ≤ 0.6 kg, one camera color each. Imported shapes are simulated by their bounding-box proxy (labelled approximate).
- Fixtures are box proxies of known geometry; concave/open meshes are visual-only. Destinations are not perceived.
- Stretch "supported placement on a base/fixture" was not started (perception assumes table support height).
- Parts must be visible to the single fixed camera; occlusion fails honestly (PERCEPTION_UNAVAILABLE).
- Simulated cycle times are not real-cell throughput. Torque MPC not implemented.
- MuJoCo prints a harmless "empty prefix" warning when the model is compiled.

## Final checks (status at hand-off; the agent terminal stopped returning output, so these are manual commands)
Run from the project folder in PowerShell:
1. `powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1 -v` → expect **35 passed** (api 7, assets 6, camera 7,
   pick_place 15). **Not yet confirmed** after the final perception edit.
2. Frontend build: confirmed passing by the user (`tsc -b && vite build`, 4.93 s).
3. `powershell -ExecutionPolicy Bypass -File .\start.ps1` → http://127.0.0.1:8765/ (restart after the edits).
4. Browser e2e (`scripts\ui_e2e.py`, dev-only Playwright): reached "run started" with real clicks. That run
   covered opening the example, STL import (45×45×45 mm shown), adding a part, a destination and a step, and the
   workflow showing as valid. Its outcome was not observed.
5. Release, not yet run: `scripts\package_release.ps1`, then
   `scripts\clean_install_test.ps1 -Zip .\release\Armstrong-0.2.0.zip`.
6. Session-2 work is **uncommitted**; baseline tag `baseline-v0.1` is the recoverable checkpoint.

## Next tasks
1. Clean-install check on a second teammate's laptop from the release ZIP (Python 3.13 present and absent).
2. Supported placement onto a base / wide-clearance fixture (support-height-aware perception, evaluator and planner).
3. Bin/tray fixtures with compound box collision geometry; then torque MPC with visible fallback reporting.
