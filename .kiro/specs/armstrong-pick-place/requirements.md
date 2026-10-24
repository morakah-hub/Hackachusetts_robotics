# Requirements: camera-guided pick-and-place demo (v0.1)

1. WHEN the app opens THEN it SHALL show the included Panda/table/block/pad scene from live backend geometry.
2. WHEN the user presses Run in camera mode THEN the block pose SHALL come from a fresh RGB-D frame of the current
   scene revision, and the arm SHALL grasp, lift, transport, lower, release, retreat and verify through physics.
3. WHEN the camera is masked, stale or ambiguous THEN the run SHALL fail at `observe` with PERCEPTION_UNAVAILABLE
   and no motion SHALL start.
4. WHEN the gripper cannot close THEN the run SHALL fail (GRASP_FAILED) and never report success.
5. WHEN the block or pad is outside the validated workspace or IK fails THEN the run SHALL fail with
   OUT_OF_WORKSPACE / PLAN_FAILED and a readable reason, before moving.
6. WHEN the scene is edited while stopped THEN the revision SHALL increment and estimates/plans SHALL be cleared;
   edits while running SHALL be rejected (BUSY); runs with an old revision SHALL be rejected (STALE_SCENE).
7. Pause SHALL freeze physics time; Step SHALL advance a bounded number of steps while paused; Stop SHALL end the
   run as `stopped`; Reset SHALL restore the configured scene.
8. Results SHALL come from a ground-truth evaluator separate from perception; unavailable metrics SHALL say so.
