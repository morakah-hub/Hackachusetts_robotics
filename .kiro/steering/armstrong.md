# Armstrong project steering

- Project root: this folder. Specification: docs/Kiro_Armstrong_Local_Prompt.md. Status: HANDOFF.md.
- Local only: no GitHub configuration, branches, pushes or deployment. Bind to 127.0.0.1.
- Windows PowerShell. Project-local tools: `.venv` (Python), `.tools\node` (portable Node). No global installs.
- Launch: `powershell -ExecutionPolicy Bypass -File .\start.ps1` → http://127.0.0.1:8765/.
- Tests: `scripts\test.ps1` (backend), `scripts\npm.ps1 run build` (frontend). Keep both green.
- Conventions: meters, radians, right-handed +Z up, quaternions [w,x,y,z] in the backend; convert to
  THREE.Quaternion(x,y,z,w) explicitly. Camera optical frame +X right, +Y down, +Z forward; depth = optical Z.
- One simulation worker (backend/armstrong/sim.py) owns MjModel/MjData and renderer contexts. HTTP handlers only
  submit commands and read snapshots.
- Physical honesty: the block moves only by contact. No welds, teleports, disabled collisions or fake success.
  State edits only at init, stopped-scene edits and reset.
- Information boundary: perception.py gets images + calibration + catalog priors only. Camera mode must never fall
  back to simulator truth. Truth is for display and the evaluator.
- Missing metrics are null / "unavailable", never invented. Report only verified results.
- handoff/reference/ is read-only background from the prior project; keep it out of submissions.
