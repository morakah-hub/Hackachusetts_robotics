# Armstrong expansion prompt: STL parts, usable workflows, runnable judge package (summary copy, 24 Oct 2026)

Supersedes the MPC-first priorities in the earlier HANDOFF. Keep the working computed-torque controller.
Development window: about 3:15 p.m. to 6:00 p.m. America/New_York; freeze major features about 5:30 p.m.

Outcome: a local, visual tabletop-workflow simulator for small workshops and 3D-printed parts. Open an example, add
a supported STL part, confirm size and physical assumptions, place parts and destinations, define a short workflow
with UI controls, run it, understand the outcome, save/reopen the project. No external AI/API/paid service at runtime.

Supported workflows: (1) move a part to a named destination; (2) kit/arrange 2-4 parts into labelled positions or a
small grid; (3) sort parts into zones by user-defined categories; (4) stretch: place on a base / wide fixture.
Parameterized recipes only; no claims of general assembly or arbitrary grasp synthesis.

Priorities:
1. Part/scene model with stable IDs (name, role, visual asset, dimensions, mass/friction, collision approximation,
   pose, grasp/perception settings) and named destinations; resolve MuJoCo addresses by name; atomic structural
   rebuild on the simulation thread with fallback to the previous scene; revisions invalidate plans/detections.
2. Real ASCII + binary STL upload: bounded canonical mesh; units (mm/cm/m), dimensions in mm, scale/orientation
   correction, role (movable / fixture-obstacle / visual), documented mass/friction presets with Advanced override,
   collision approximation and manipulation eligibility, name/color, placement. Safe IDs/paths; invalid uploads never
   break the scene. Box proxy for boxlike parts, labelled as approximate; concave fixtures visual-only. Ship small
   redistributable example STLs and a default sample that runs without uploads.
3. Visual workflow builder (Parts, Destinations, Workflow, Run/results) with step cards; presets: two-part kit, small
   grid, category-to-zone sorting by explicit rules; one executor and one physical skill. Per step: observe, validate,
   execute, verify, record. No world reset between steps; reject occupied destinations; stop on failure; honest
   partial results; step markers, progress and durations. Save/Open and portable Export/Import bundles with versioned
   scene, recipe, physical settings and mesh assets; validated paths and size limits; examples separate from user data.

Throughout: camera mode uses RGB-D estimates (distinct color palette, ambiguity/occlusion reported), no leaking of
truth poses into planning, non-square grasp-axis handling, collision checks for approach, descent, lift, transport,
lowering, retreat and return (arm and carried part), real contact physics only, evaluator generalized per object and
support height, estimation error separate from placement error, no silent yaw handling.

Judge package: verify tracked files vs runtime needs; keep source, lockfile, dependency declarations, Panda assets and
licenses, examples, scripts, README, .kiro; exclude environments/caches/private data/reference archives. Harden the
launcher (relative paths, tested Python range, dependency-change detection, lockfile-aware rebuilds, Node checksum,
paths with spaces, no port killing, double-click launcher). scripts/package_release.ps1 builds a clean ZIP with the
built frontend. docs/JUDGE_QUICKSTART.md. Clean-install test from release-intended files in a fresh folder. Local
commits only; no force-push or automatic release.
