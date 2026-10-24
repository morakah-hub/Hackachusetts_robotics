# Prompt for the next Kiro session

You are continuing Armstrong, a local Windows robotics workflow app in this folder (project root; work only here).
Read, in order: `HANDOFF.md` (current state, verified results, next tasks), `.kiro/steering/armstrong.md` (rules),
`docs/Armstrong_Expansion_STL_Workflows_Kiro_Prompt.md` (current priorities), `README.md`, `docs/JUDGE_QUICKSTART.md`.

Rules that still apply: local work only (no pushes, releases or deployment unless the user asks); project-local `.venv`
and `.tools\node`; PowerShell commands; one simulation worker owns MuJoCo; parts move only by contact; camera mode
never uses simulator truth for planning; imported meshes are labelled box-proxy approximations; report only verified
results; keep `handoff/` and `user_data/` out of source control and releases.

Start by checking the app still works:
1. `powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1` (backend suite; all should pass).
2. `powershell -ExecutionPolicy Bypass -File .\start.ps1` → open the printed URL → Project → Two-part kit → Run.
3. `.\.venv\Scripts\python.exe scripts\run_project.py two_part_kit` (headless, ~5 s).
Optional browser test: `.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt`, then
`.\.venv\Scripts\python.exe scripts\ui_e2e.py` against a running server.

Then work on the "Next tasks" in HANDOFF.md, keeping the app runnable and the tests green after each step. Rebuild the
judge ZIP with `scripts\package_release.ps1` and test it from a fresh folder before calling a release ready. Update
HANDOFF.md, `.kiro/specs/armstrong-pick-place/tasks.md` and this file at each milestone.

Shell note: in this environment long commands are most reliable when run in the background with output redirected to
a file in `runtime\` and read afterwards.
