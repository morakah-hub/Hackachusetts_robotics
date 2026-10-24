# Requirements (v0.2: STL parts, workflows, judge package)

1. WHEN the app opens THEN it SHALL show the included scene from live backend geometry; shipped examples SHALL open
   from the Project panel and the original single-block demo SHALL remain available as a regression example.
2. WHEN a workflow runs in camera mode THEN every step SHALL use a fresh RGB-D frame of the current scene revision for
   every movable part's pose, and the arm SHALL grasp, lift, transport, lower, release, retreat and verify through physics.
3. WHEN the camera is masked, stale, ambiguous or a part is occluded THEN the step SHALL fail with
   PERCEPTION_UNAVAILABLE before any motion; there SHALL be no fallback to simulator truth.
4. WHEN a destination is occupied, a route (any segment, arm or carried part) collides, a pose is unreachable or a part
   is unsupported THEN the step SHALL fail with DESTINATION_OCCUPIED / ROUTE_BLOCKED / PLAN_FAILED / OUT_OF_WORKSPACE /
   UNSUPPORTED_PART and a readable reason before moving.
5. WHEN the gripper cannot close THEN the step SHALL fail (GRASP_FAILED) and never report success.
6. WHEN a step fails THEN the sequence SHALL stop, completed steps SHALL keep their records, later steps SHALL be
   reported "not run", and the run SHALL NOT be reported as a successful workflow.
7. WHEN a destination requests a yaw THEN the part SHALL be rotated during transport and the yaw verified (camera)
   and evaluated (truth); otherwise the part keeps its yaw.
8. WHEN an STL is uploaded THEN binary and ASCII SHALL parse within size/triangle limits; malformed, non-finite or
   implausibly scaled input SHALL be rejected without changing the scene; eligibility SHALL be shown with reasons.
9. WHEN the scene changes while stopped THEN the revision SHALL increment and estimates, plans and workflow validation
   SHALL be refreshed; structural changes SHALL rebuild the model atomically and keep the previous scene on failure.
10. Projects SHALL save/open and export/import as versioned bundles with hash-checked meshes and no absolute paths.
11. The launcher SHALL work from any folder (spaces allowed), check the tested Python version, reinstall on changed
    requirements, rebuild the frontend only when sources changed, and never stop other programs to free a port.
