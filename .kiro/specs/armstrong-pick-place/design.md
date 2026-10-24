# Design (as implemented, v0.2)

- Process: `python -m armstrong.server`; simulation worker on the main thread (MuJoCo + GLFW renderers), Uvicorn on a
  background thread. `frontend/dist` served same-origin on 127.0.0.1.
- Scene model (`scene.py`): `PartSpec` (id, name, role movable|fixture|visual, asset_id, size, mass, friction, color,
  category, pose) and `DestinationSpec` (id, name, pose, optional yaw, zone). Bodies `part_<id>` / `dest_<id>`;
  addresses resolved by name after compile. Uploaded meshes are visual geoms; physics uses a box proxy with an
  explicit inertial (no double-counted mass).
- Rebuilds (`sim.py` `Bundle`): model, data, kinematics, controller, evaluator and renderers are built off to the side
  and swapped in one step; the old renderers are closed in their own GL context (MuJoCo's `Renderer.close` frees the
  MjrContext without making its context current). Failure keeps the previous scene.
- Worker loop: drain bounded command queue (publishing a snapshot after each command) → step physics to wall clock →
  camera at 5 Hz → snapshot at 30 Hz over `/api/ws`.
- Workflow (`task.py`): `compile_recipe` validates steps against the scene; `WorkflowTask.program()` runs steps with
  the same skill. Planning world = proprioception + camera estimates of all movable parts + known static geometry.
- Perception (`perception.py`): `SceneObserver` (HSV, world points, tabletop volume) + `estimate_part` per catalog entry.
- Evaluation (`evaluator.py`): per-step truth metrics; an evaluator failure stops the sequence and is labelled.
- Assets/projects (`assets.py`, `projects.py`): content-hash asset ids (`a_` + 16 hex of the canonical mesh SHA-256),
  examples in `examples/`, private data in `user_data/` (git-ignored), bundles = project.json + assets/*.stl.
- API: see `api.py`; errors `{error:{code,message,details}}`. Uploads are raw request bodies (no multipart dependency).
