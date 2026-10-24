# Kiro_Armstrong_Local_Prompt (copy of the project specification supplied in chat, 24 Oct 2026)

You are the sole coding agent building Armstrong, our Hackachusetts robotics application. Implement and run the entire application locally.

WORKSPACE: C:\Users\rkhmh\Downloads\Hackachusetts_armstrong — this folder is the project root. Preserve anything already useful. Create all project code, configuration, assets and project environments here. No nested application root. Use project-relative paths.

This instruction replaces earlier team-role assignments, branch workflows and automatic GitHub pushes. You own simulation, robot control, camera perception, API, frontend, integration and testing. Do not configure GitHub, clone the shared repository, create branches, push, deploy publicly or prepare remote collaboration now.

## 1. Get started and use the reference
- Prefer the filtered kit (handoff/reference/robotics-lab/). Extract safely inside this folder; keep the reference unchanged.
- Study its Panda model, physical pick-and-place, IK, controller, camera/rendering and workbench architecture as technical background; old role assignments and GitHub instructions are superseded.
- Pre-hackathon code: use architecture and lessons as reference for a fresh Kiro implementation unless event rules permit reuse. Retain attribution and licenses for public assets. Do not claim old code or tests were produced in this session.
- Check Python, Node/npm and rendering. Project-local .venv and frontend dependencies. Windows PowerShell commands. No Linux EGL assumptions, no global installs, no system security changes, no machine-specific paths from old code.

## 2. The product
Armstrong helps a small workshop configure and evaluate a repetitive tabletop handling task through a beautiful browser interface. First demonstration:
- Included scene: Franka Panda, table, one colored rigid block, destination pad.
- Select block and destination, press Run. A simulated RGB-D camera estimates the block's location.
- The arm physically grasps, lifts, transports, releases and verifies placement.
- The browser shows actual simulation motion, camera output, task progress and measured results.
- Stop/reset, move the block within the supported workspace, run again with a new observation and plan.
Limited object/task family; repositioning only while stopped. No arbitrary scanned objects or continuous adaptation.

## 3. Complete working path first
Python, MuJoCo, NumPy/SciPy, OSQP where needed; FastAPI/Uvicorn; React, TypeScript, Vite, Three.js. One simulation worker owns MuJoCo state and renderer contexts; handlers queue commands and read snapshots; one active run; bounded queues. Run, Stop, Reset, incremental stepping early; Pause with defined behavior. Local proxy or same-origin; bind to localhost; one documented launch. Browser geometry/motion must match the backend (export authoritative geometry/transforms). Keep the app runnable.

## 4. Physical manipulation and control
Fixed validated Panda mount and table; upright ~50x50x60 mm block; flat pad. Full-pose IK, top-down grasp, bounded state machine: approach, descend, close, lift, transport, lower, release, retreat, verify. Actuators and contact/friction move the block: no teleporting, hidden welds, disabled collisions or fake success (state edits only for init, stopped-scene editing, labelled replay). Retain validated torque MPC if available, otherwise add after baseline; display the controller actually used and any fallback. Known-world mode clearly labelled; camera mode uses image-derived positions; outcome evaluation separate from perception. Reject unreachable/unsupported requests with useful reasons; bounded timeouts; visible failure information.

## 5. Real simulated vision
RGB + metric depth from one calibrated camera (~320x240, modest rate) with frame IDs, scene revisions, timestamps, intrinsics, optical-camera-to-world transform; validate projection. Meters, radians, right-handed Z-up; explicit quaternion order. One opaque upright cuboid of known size and distinctive color: RGB to identify, depth to reconstruct; constrain unsupported orientation. Estimator gets images, calibration and priors only, never simulator poses, editor coordinates or segmentation IDs. Missing/masked/stale/ambiguous images → invalid observation, no new blind motion, never silent fallback to truth. Truth only for display/evaluation. Known static obstacles labelled honestly.

## 6. Beautiful and easy to use
Brand Armstrong. Calm workspace: large central scene, compact setup/task sidebar, camera/results area. Charcoal/warm-gray, one teal/blue accent. Include: demo scene; connection/loading/ready/running/paused/stopped/completed/failed states; Panda in robot dropdown with future arms disabled "Planned"; object selection, table-plane dragging, numeric placement; source/destination selection; Run/Pause/Stop/Reset; task phases and actionable failures; RGB/depth views with frame age; measured placement; compact Developer drawer (controller, timing, perception, log, export). Edits only while stopped; each edit invalidates stale plans/detections; wait for accepted revision before starting; missing metrics "unavailable". Local assets only.

## 7. Features only after the loop works
Validated torque MPC; bounded OBJ import (units, scale, mass, friction, collision proxies); one static obstacle; repeated pick-and-place or simple sorting. Defer login, hosting, extra arms, training, deformables, arbitrary grasp synthesis.

## 8. Test, launch, checkpoint
Checks: physical grasp/lift/release/settle; changed position changes observation and plan; camera masking blocks camera task; gripper that cannot close cannot succeed; unreachable target fails clearly; controls/revisions/integration; frontend builds and launch works. Report real results only. README with tested Windows commands; start.ps1 (paths from own dir, prerequisite checks, starts services, prints URL). Concise .kiro steering/spec/task files. HANDOFF.md after each milestone (capabilities, launch commands, versions, verification, failures, next three tasks) and NEXT_SESSION_PROMPT.md. Keep venvs, node_modules, secrets, archives and caches out of source control. When the user says HANDOFF: save code, update documents, report what runs; do not push.
