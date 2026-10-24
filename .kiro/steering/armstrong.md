# Armstrong project steering

- Project root: this folder. Specifications: docs/Kiro_Armstrong_Local_Prompt.md (v0.1) and
  docs/Armstrong_Expansion_STL_Workflows_Kiro_Prompt.md (v0.2, current priorities). Status: HANDOFF.md.
- Local only: no GitHub pushes, branches, releases or deployment unless the user asks. Bind to 127.0.0.1.
- Windows PowerShell. Project-local tools: `.venv` (Python 3.13), `.tools\node` (portable Node). No global installs.
- Launch: `powershell -ExecutionPolicy Bypass -File .\start.ps1` (or start.bat) → printed http://127.0.0.1:<port>/.
- Tests: `scripts\test.ps1` (backend), `scripts\npm.ps1 run build` (frontend). Keep both green.
- Release: `scripts\package_release.ps1` → release\Armstrong-<version>.zip; verify from a fresh folder before claiming it works.
- Conventions: meters, radians, right-handed +Z up, quaternions [w,x,y,z] in the backend; convert to
  THREE.Quaternion(x,y,z,w) explicitly. Camera optical frame +X right, +Y down, +Z forward; depth = optical Z.
- One simulation worker (backend/armstrong/sim.py) owns MjModel/MjData and renderer contexts. HTTP handlers only submit
  commands and read snapshots. Structural scene changes rebuild a whole Bundle and swap it atomically.
- Stable ids: parts `part_<id>`, destinations `dest_<id>`; never index qpos by position.
- Physical honesty: parts move only by contact. No welds, teleports, disabled collisions or fake success. State edits
  only at init, stopped-scene edits and reset. Imported meshes are simulated through a labelled box proxy.
- Information boundary: perception.py gets images + calibration + catalog priors only; planning uses estimates for
  every movable part (never the live qpos of parts). Camera mode never falls back to simulator truth.
- Missing metrics are null / "unavailable", never invented. Report only verified results.
- user_data/ (uploads, saved projects) and handoff/ (prior-project reference) stay out of source control and releases.
