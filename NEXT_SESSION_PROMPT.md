# Prompt for the next Kiro session

You are continuing Armstrong, a local Windows robotics app in `C:\Users\rkhmh\Downloads\Hackachusetts_armstrong`
(project root; work only here). Read, in order: `HANDOFF.md` (current state, verified results, next tasks),
`.kiro/steering/armstrong.md` (rules), `docs/Kiro_Armstrong_Local_Prompt.md` (full specification), `README.md`.

Rules that still apply: local only (no GitHub setup, branches, pushes, deployment); project-local `.venv` and
`.tools\node`; PowerShell commands; one simulation worker owns MuJoCo; the block moves only by contact; camera mode
never uses simulator truth; report only verified results; keep `handoff/reference/` read-only.

Start by checking the app still works:
1. `powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1` (expect 19 passed).
2. `powershell -ExecutionPolicy Bypass -File .\start.ps1` → open http://127.0.0.1:8765/, press Run.
3. `.\.venv\Scripts\python.exe scripts\smoke_api.py` against the running server.

Then do the "Next three tasks" in HANDOFF.md, keeping the app runnable and the tests green after each step.
Update HANDOFF.md, the .kiro spec tasks and this file at each milestone. When the user says HANDOFF, save code and
update these documents; do not push.

Shell note: in this environment long Python commands are most reliable when run in the background with output
redirected to a file in `runtime\` and read afterwards.
