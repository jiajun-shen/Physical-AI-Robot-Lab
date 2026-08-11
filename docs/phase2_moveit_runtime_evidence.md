# Phase 2.2 ABB MoveIt Runtime Evidence

- Tested feature commit: `4203e81508f91b651eb8f55e663c74f21b8ff5b0`
- Clean-run verifier completion: 2026-08-11 17:01:57 +0200
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2.12.4
- Gate output: `/var/tmp/p22-clean-4203e81`
- Scope: headless simulation only; project ABB Gazebo control chain plus one
  `move_group` process

## What the clean run actually proved

The gate ran from a clean checkout of the tested feature commit. It built an
ephemeral ext4 overlay, then reused the project-owned Gazebo launch, its sole
`gz_ros2_control` controller manager, and its already active JSB/JTC. The
MoveIt launch adds only `move_group`; it does not create fake hardware, a
second controller manager, or a controller spawner.

The verifier uses the live MoveIt services/actions rather than sending a raw
trajectory to the JTC:

- `/plan_kinematic_path` plans normal joint and pose requests through the OMPL
  pipeline;
- `/execute_trajectory` executes the resulting trajectories through the
  existing `joint_trajectory_controller`;
- `/compute_fk` derives the reviewed `base_link -> tool0` pose from the named
  `fk_pose_seed` joint state in the smoke configuration;
- `/check_state_validity`, `/apply_planning_scene`, and
  `/get_planning_scene` provide the collision evidence.

All normal endpoints were checked valid before the temporary collision scene
was applied. The actual Cartesian route was `HOME -> FK-derived tool0 pose ->
HOME`, with initial HOME and returned HOME observed through `/joint_states`.
The four executions (joint out, joint return, pose out, pose return) were
accepted by MoveIt and completed with action status `SUCCEEDED` and MoveIt
error code `SUCCESS`.

The normal pose endpoint's live FK observation differed from the requested
pose by `0.0063663304 m` and `0.0629389291 rad`, within the explicitly
configured simulation execution acceptance bounds of `0.030 m` and `0.090 rad`.
The initial HOME maximum deviation was `2.6719493e-05 rad`; the observed
post-pose HOME maximum deviation was `0.0185216305 rad`, below the configured
`0.035 rad` bound. These values are runtime observations, not a physical
calibration claim.

## Collision-rejection proof

The verifier synchronously added `p22_tool0_goal_block`, a `0.12 m` box
centred at the FK-derived terminal pose, and read it back from MoveIt's planning
scene. The seed was valid before the scene and invalid after it: the query
reported contacts with `link_4`, `link_5`, and `link_6`; the separated returned
HOME state remained valid.

Both a pose request and an exact joint request into that now-invalid target
were rejected by planning. In this installed OMPL/MoveIt run the returned value
was the generic `99999` (`FAILURE`), not a more specific goal-in-collision
value. The gate accepts that generic result only after proving the object is in
the planning scene, the target is invalid, and the start state is valid. It
then synchronously removed the object and read an empty object list back from
the planning scene.

Trajectory timing is deliberately conservative and velocity-only. The project
does not invent acceleration, deceleration, or jerk limits: it removes the
time-optimal adapter that requires acceleration limits, retimes each successful
geometric plan from the installed canonical velocity table with a safety factor
and a finite segment floor, and leaves derivative arrays empty. The evidence
records `acceleration_limits_used: false`.

## Executed clean gate

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/shenj/Documents/Codex/2026-08-11/files-mentioned-by-the-user-demo/Physical-AI-Robot-Lab
bash scripts/run_and_verify_irb1200_moveit_headless.sh /var/tmp/p22-clean-4203e81
```

- Four packages built and tested from the temporary overlay.
- `colcon test-result`: 58 records, 0 errors, 0 failures, 0 skipped: 52 unique
  pytest cases and six CTest wrappers.
- The before/after source manifests are identical:
  `ad0b91e8dc33641893fc98c2e8aebaa995d90e996944e37b41fe977b09abb968`.
- The isolated run used `ROS_DOMAIN_ID=100` and
  `GZ_PARTITION=physical_ai_p22_100_446`.

Observed persistent result fields:

```text
PROJECT_ABB_GAZEBO_SPAWN=PASS
HARDWARE_RESOURCES=PASS
CONTROLLERS_ACTIVE=PASS
LIMITER_RUNTIME_INITIALIZED=PASS
MOVE_GROUP_READY=PASS
MOVEIT_JOINT_EXECUTION=PASS
MOVEIT_POSE_EXECUTION=PASS
MOVEIT_HOME_POSE_HOME=PASS
MOVEIT_COLLISION_REJECTION=PASS
GUI_RENDERING=NOT_RUN
REAL_ROBOT_READY=false
A02=PASS
ABB_MOVEIT_HEADLESS_RUNTIME=PASS
CLEANUP_NO_RESIDUAL_PROCESS_GROUP=PASS
LAUNCH_EXIT_STATUS=0
MOVE_GROUP_SHUTDOWN_OBSERVATION=SIGSEGV_DURING_REQUESTED_CLEANUP
CLEANUP_VERIFIED=NO_RESIDUALS_WITH_ABNORMAL_MOVE_GROUP_EXIT
TESTED_COMMIT=4203e81508f91b651eb8f55e663c74f21b8ff5b0
SOURCE_WORKTREE_CLEAN=true
```

## Warnings, limitations, and truthful boundary

The clean log records the expected transient Gazebo resource-manager and
executor/statistics warnings, a 100 Hz controller period against a 1 ms physics
step, default workspace/Octomap warnings, and one timed-out early controller
list response. It also records the two expected OMPL failures used for the
intentional collision-rejection proof. The readiness, controller, action, and
evidence gates all completed after those messages.

During requested teardown, the installed `move_group` process received SIGINT
and then exited with signal 11. The harness verified that its owned process
group and selected-domain ROS daemon had no residual process, but it does **not**
call this a graceful shutdown. This is recorded as open `KI-012` and must be
investigated before M1/Phase 16 hardening; it does not erase the completed live
planning/execution observations that occurred before teardown.

This evidence establishes A02 in headless simulation. It does not establish
GUI rendering, a gripper, lab objects, RGB-D perception, pick/place, drawing,
speech, TaskGraph execution, out-of-range clamp behavior, physical ABB safety,
or M1 acceptance. M1 requires a new integrated Phase-8-final run and the
user's explicit approval.
