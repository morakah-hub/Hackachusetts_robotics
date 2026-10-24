# Armstrong — HANDOFF (updated 24 Oct 2026, end of session 1)

## What runs now
- **Launch**: `powershell -ExecutionPolicy Bypass -File .\start.ps1` in `C:\Users\rkhmh\Downloads\Hackachusetts_armstrong`
  → **http://127.0.0.1:8765/**. Stop with Ctrl+C or `scripts\stop.ps1`.
- Panda + table + orange 50×50×60 mm block + teal destination pad, rendered in the browser from the backend's compiled
  MuJoCo visual geometry and live body poses (WebSocket, 30 Hz).
- Camera-guided pick-and-place: a fresh 320×240 RGB-D frame → image-only block estimate → plan → physical
  approach/descend/close/lift/transport/lower/release/retreat/verify under computed-torque control. The block moves
  only through finger contact and friction.
- Known-world development mode (simulator pose), labelled in the UI and in results.
- Drag or type block x/y/yaw and pad x/y while stopped; revisions; Run waits for the accepted revision.
- Pause (physics time frozen), Step 10/100 ms while paused, Stop, Reset, Load demo scene.
- Camera panel (RGB/depth, frame id, age, revision), localization panel (estimate, evaluation-only error vs truth),
  results from the ground-truth evaluator, run JSON export, Developer drawer (controller/gains/timing, estimate/
  plan/truth JSON, fault injection: mask camera / disable gripper, event log).
- Failure reporting with stage, code and hint: PERCEPTION_UNAVAILABLE, OUT_OF_WORKSPACE, PLAN_FAILED, GRASP_FAILED,
  PLACE_FAILED, CONTACT_VIOLATION, CONTROL_TIMEOUT, STALE_SCENE, BUSY.

## Verification performed this session (actual results)
- `scripts\test.ps1`: **19 passed** in 27.5 s (test_api 5, test_camera 5, test_pick_place 9). They cover physical
  grasp/lift/release/settle, new position → new frame/estimate/plan, masked camera → no motion, disabled gripper → no
  success, unreachable block (x = 0.86 m) → OUT_OF_WORKSPACE before motion, IK unreachable, stale revision, BUSY edits,
  pause/step/stop/reset, WebSocket, geometry/binary consistency, camera endpoints, export.
- Projection convention test: block-top point projects onto orange pixels; rendered depth matches optical Z within 4 mm.
- Perception grid (`scripts\diag_perception.py`, 18 poses x∈{0.38,0.50,0.62}, y∈{-0.1,0.1,0.25}, yaw∈{0,0.4}):
  max xy error 1.32 mm, mean bias +0.57 mm in x; yaw within ~1.7° of truth.
- Headless camera runs: default start (0.50, 0.20) → success, 1.3 mm placement error; (0.45, 0.15) → success,
  1.0 mm. Live HTTP run via `scripts\smoke_api.py` on the server launched by `start.ps1` (block edited to
  (0.45, 0.15), revision 2) → success, 0.8 mm placement error, 2.3 mm camera estimate error, 102 mm lift,
  22.3 s wall for 18.7 s sim; stale-revision run correctly rejected with 409 STALE_SCENE.
- `start.ps1` verified: detected stale `frontend\dist`, rebuilt it, reported a busy port as a BLOCKER, and on a free
  port printed the URL and served the app.
- Frontend: `tsc -b && vite build` clean; headless Edge screenshot (`scripts\screenshot.ps1`) shows the live scene,
  estimate box, camera feed and results.

## Installed versions (project-local)
Python 3.13.7 (.venv): mujoco 3.17.0, numpy 2.5.4, scipy 1.18.1, osqp 1.1.3 (installed, not used yet),
fastapi 0.162.0, uvicorn 0.54.0, pillow 13.0.0, pytest 9.1.1. Node v24.22.1 portable (.tools\node), npm 11.19.1,
vite 7.3.7, react 19, three 0.180, typescript 5.x.

## Known issues / limitations
- MuJoCo prints "Attaching a spec with an empty prefix…" at startup (harmless; names kept as in Menagerie).
- Real-time factor dips briefly while the task plans (IK + sampled collision checks run on the worker thread);
  the loop re-anchors instead of fast-forwarding, so a run takes ~20–24 s wall for ~18.6 s sim.
- No torque MPC; the controller panel says "planned". OSQP is installed for that work.
- Destination pad is a user-defined goal (not perceived). Table/floor are known geometry (labelled in the plan).
- Approach/transport are direct joint-space moves with collision checking (no RRT); fine on the empty table only.
- The frontend JS bundle is ~800 kB (three.js); it is local, so this only affects first load (~1 s).
- If the browser loses the WebSocket the UI shows "Disconnected — retrying" and reconnects automatically.

## Next three tasks
1. Torque MPC: port and re-validate the reference `controllers/torque_mpc.py` idea with OSQP; show active controller
   and fallback counts in the UI; add a test comparing CTC vs MPC on the same run.
2. One static obstacle (known box between block and pad, labelled "known geometry"): add RRT-Connect or a via-point
   transport, and tests for collision-free transport.
3. Repeated pick-and-place / simple sorting (several blocks, colors as catalog priors) reusing the same skill; then a
   bounded OBJ import with explicit units/scale/mass/friction and box collision proxies.

## Files
backend/armstrong/{scene,robot,kinematics,trajectory,controller,camera,perception,evaluator,task,sim,api,server}.py,
backend/tests/, frontend/src/{App.tsx,Viewport.tsx,sceneView.ts,api.ts,useSim.ts,styles.css}, start.ps1, scripts/,
.kiro/steering/armstrong.md, .kiro/specs/armstrong-pick-place/, docs/Kiro_Armstrong_Local_Prompt.md.
Reference kit (read-only, from the earlier robotics-lab project): handoff/.
