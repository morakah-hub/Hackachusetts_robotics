# Tasks

v0.1 (session 1)
- [x] 1. Scene, Panda (Menagerie, torque actuators), CTC, pose IK, physical known-world pick-and-place
- [x] 2. Simulation worker, FastAPI + WebSocket, Run/Pause/Step/Stop/Reset, scene revisions
- [x] 3. RGB-D camera with validated projection; image-only block localization; camera-guided runs
- [x] 4. React/Three.js workspace: geometry from backend, dragging, numeric edits, phases, camera, results, developer drawer
- [x] 5. Acceptance tests; start.ps1, README, HANDOFF.md, NEXT_SESSION_PROMPT.md

v0.2 (session 2: STL parts, workflows, judge package; supersedes the MPC-first plan)
- [x] 6. Part/destination model with stable ids; name-resolved MuJoCo addresses; atomic rebuild with fallback
- [x] 7. Multi-color, non-square camera localization (side assignment, yaw period), ambiguity/occlusion failures
- [x] 8. Workflow executor: per-step observe/plan/execute/verify/evaluate, no reset between steps, occupancy and
       route checks for every motion segment including the carried part, requested destination yaw
- [x] 9. STL import (binary + ASCII), units/scale/orientation, eligibility, box proxy, material presets, preview UI
- [x] 10. Workflow builder UI (step cards, presets Move/Kit/Grid/Sort, sort rules), progress, per-step results
- [x] 11. Projects: examples vs user data, Save/Open, portable bundle Export/Import with hash-checked assets
- [x] 12. Hardened launcher (Python 3.13 check, requirements/lockfile stamps, free-port fallback), start.bat,
       package_release.ps1, docs/JUDGE_QUICKSTART.md
- [ ] 13. Clean-install verification on a second laptop (strongest final check)
- [ ] 14. Stretch: supported placement on a base / wide-clearance fixture (needs support-height-aware perception)
- [ ] 15. Torque MPC with visible fallback reporting (deferred)
