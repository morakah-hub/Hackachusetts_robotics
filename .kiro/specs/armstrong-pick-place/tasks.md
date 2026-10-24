# Tasks

- [x] 1. Scene, Panda (Menagerie, torque actuators), CTC, pose IK, physical known-world pick-and-place
- [x] 2. Simulation worker, FastAPI + WebSocket, Run/Pause/Step/Stop/Reset, scene revisions
- [x] 3. RGB-D camera with validated projection; image-only block localization; camera-guided runs
- [x] 4. React/Three.js workspace: geometry from backend, dragging, numeric edits, phases, camera, results, developer drawer
- [x] 5. Acceptance tests (tests/): grasp/lift/place, new position → new plan, masked camera, disabled gripper,
       unreachable target, controls/revisions, API integration
- [x] 6. start.ps1, README, HANDOFF.md, NEXT_SESSION_PROMPT.md
- [ ] 7. Torque MPC (port/validate against reference controllers/torque_mpc.py) with visible fallback reporting
- [ ] 8. One static obstacle (known geometry, labelled) with a planned transport path
- [ ] 9. Repeated pick-and-place / simple sorting with the same skill; bounded OBJ import
