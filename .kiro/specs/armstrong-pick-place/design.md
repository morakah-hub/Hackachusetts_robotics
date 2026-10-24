# Design (as implemented)

- Process: `python -m armstrong.server`; simulation worker on the main thread (MuJoCo + GLFW renderers),
  Uvicorn on a background thread. Frontend built to `frontend/dist`, served same-origin.
- Worker loop: drain bounded command queue → step physics to wall-clock time (re-anchors after planning stalls,
  never fast-forwards) → render camera at 5 Hz → publish snapshot at 30 Hz (WebSocket `/api/ws`).
- Task: generator state machine in `task.py`; one `yield` per physics step. Planning uses the camera estimate,
  catalog size, user pad location and known table geometry (labelled).
- Control: CTC (`controller.py`) on a contact-free model copy; torque limits clipped; no MPC yet.
- Perception: `perception.py` (`estimate_block`), validated by `tests/test_camera.py`.
- Evaluation: `evaluator.py` (truth) → RunResult.metrics; run outcome = supervisor completion AND evaluator pass.
- API: see README table and `backend/armstrong/api.py`; errors `{error:{code,message,details}}`.
