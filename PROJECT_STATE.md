# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 1 - ABB IRB1200 + Gazebo Harmonic + ros2_control (`IN PROGRESS`)

## Current subtask

Start `P2.1`: create a project-owned MoveIt configuration contract for the
already running ABB simulation. Build and static-contract test it first; do not
claim planning or execution until P2.2 produces a clean real integrated run.

## Git checkpoints

- Latest known-good commit: `1228c13fadb8c04d348a0bfa68e2fa1d0a7ce6ee`
- Stable commit: `1228c13fadb8c04d348a0bfa68e2fa1d0a7ce6ee`
- Experimental commit: none
- Current branch: `main` tracking `origin/main`
- Remote: `https://github.com/jiajun-shen/Physical-AI-Robot-Lab.git` (private)

The P1.3b feature has a clean, commit-addressable runtime gate. Commit the
evidence/state checkpoint before P2.1 begins so that the runtime evidence and
M1 stop condition are immutable on `main`.

## Working features

- ROS 2 Jazzy, Gazebo Harmonic, ros2_control/controllers, MoveIt, and MoveIt
  Task Constructor are installed in WSL Ubuntu 24.04.
- `abb_lab_description` contains the pinned ABB IRB1200 geometry and separate
  fixed-base `GazeboSimSystem` control composition.
- `abb_lab_control` supplies six-axis position JTC/JSB controller policy.
- `abb_lab_gazebo` supplies the minimal headless world and ordered
  spawn/RSP/controller startup sequence.
- A project-owned ABB IRB1200 spawned in Gazebo as `abb_irb1200`; six position
  command interfaces were claimed, JTC/JSB were active, command-limit
  enforcement initialized, and an observed trajectory completed.
- The full `main` history through `1228c13` is pushed to the private GitHub
  repository under `jiajun-shen`.

## Completed features

- Phase 0 environment, dependency, upstream, license, and architecture audits.
- Full-SHA/SHA-256-locked IRB1200 geometry subset at `2a536eb`.
- Fixed-base Harmonic control description and resource export at `2ec4330`.
- P1.3b bounded project ABB headless runtime at `1228c13`.
  - Clean runtime completion: 2026-08-11 15:37:51 +0200.
  - Exact entity `abb_irb1200` persisted in world `abb_lab`.
  - One active `GazeboSimSystem`, six claimed position commands, 18 state
    interfaces, active JSB/JTC, `enforce_command_limits=true`, and six limiter
    initialization messages.
  - `FollowJointTrajectory` returned `SUCCEEDED` and `error_code: 0` with 59
    feedback frames; maximum observed velocity was `0.0201531993902` rad/s.
  - Fixed-base drift was 0 m / 0 rad and cleanup was verified.
  - 41 colcon records and one separate harness unittest passed with zero errors,
    failures, or skips.
- A01 is `PASS`; see `docs/phase1_abb_gazebo_control_runtime_evidence.md`.

## Partially completed features

- Phase 1 has a real headless ABB control path, but no project MoveIt planning,
  collision-aware Home -> Pose -> Home execution, GUI validation, gripper, or
  out-of-range clamp probe.
- Runtime command-limit initialization and the bounded in-range control chain
  are verified. Clamp behavior remains explicitly unverified.
- WSLg endpoints and RViz exist, but real 3D GUI rendering is untested.

## Current test and run status

- Phase 0 checkpoint: PASS at `eb1b8e8`.
- Phase 1 dependency/control checkpoint: PASS at `14d946f`.
- ABB asset/geometry checkpoint: PASS at `2a536eb`.
- Static control-description checkpoint: PASS at `2ec4330`.
- P1.3b clean integrated runtime: PASS at `1228c13`.
  - `SOURCE_WORKTREE_CLEAN=true`.
  - Before/after source-manifest SHA-256:
    `3076651510b8eff14bd586de6f3eb4b7f550461e9db6bab4f7c86aba8e457179`.
  - Spawn, hardware resources, controllers, runtime limiter initialization,
    bounded trajectory, and cleanup: PASS.
  - Clamp probe and GUI rendering: NOT RUN; real robot: false.
- A01: PASS. A02-A14: NOT STARTED. MoveIt project planning: NOT RUN.

## Current build status

The clean P1.3b gate built and tested `abb_lab_description`, `abb_lab_control`,
and `abb_lab_gazebo` on ext4. Generated build/install/log trees and runtime
episodes remain outside Git. `abb_lab_moveit_config` does not exist yet.

## Current run status

- Project ABB headless Gazebo spawn/control: verified.
- Project MoveIt planning/execution: not run.
- Project GUI rendering: not run.
- Project out-of-range clamp injection: not run.
- VLA backend: `VLA_BACKEND_UNAVAILABLE`.

## Known limitations and open questions

- WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound.
- The local GPU has 8151 MiB, below openpi's strict `> 8 GB` statement; no CUDA
  Toolkit or Docker is present.
- `abb_ros2` has no Jazzy branch/tag and is not treated as a supported driver.
- ABB inertials are estimates and efforts are fictional. No acceleration or jerk
  limits are claimed.
- Real runtime has not validated a GUI, collision scene, gripper, perception,
  physical effort behavior, out-of-range clamp, or real robot.
- The GitHub repository remains private while the first-party license is open.
- Final first-party license, speech-model pin, and VLA GPU service location are
  open design decisions.

## Mandatory M1 stop condition

M1 is inactive until Phase 8 exits. At that point set the project state to
`PAUSED - AWAITING USER MIDTERM ACCEPTANCE`; do not start Phase 9 until every
M1 item in `ACCEPTANCE_STATUS.md` is `PASS` from immutable real-run artifacts
on the tested Phase-8-final commit and the user explicitly accepts M1. Vision
must not consume Gazebo ground truth. The executive must preserve an inspectable
TaskGraph plus timestamped observe/verify/continue-or-replan evidence after
every task step. M1 passing tests never auto-release Phase 9.

## Important architecture decisions

- Working classical baseline precedes advanced autonomy and VLA authority.
- Use official Jazzy apt packages and the default Harmonic pairing.
- Reuse pinned ABB geometry/kinematic assets, not the unverified complete
  driver; build a project-owned Jazzy/Harmonic control/MoveIt overlay.
- Keep geometry provenance separate from simulation control policy.
- Language planning emits typed Task/Skill/Goal structures, never joint angles.
- `vision` mode cannot access simulator ground truth.
- ROS and openpi remain isolated behind a versioned policy API.
- Missing VLA produces `VLA_BACKEND_UNAVAILABLE`; a fallback success is
  `CLASSICAL`, never VLA.

## Dependencies

See `DEPENDENCY_MATRIX.md`, `config/ros-jazzy-apt.lock.yaml`,
`config/ros-jazzy-installed-packages.lock.tsv`, and the reports under `docs/`.
The current baseline is Ubuntu 24.04.4 WSL2, ROS 2 Jazzy, Gazebo gz-sim 8.11.0,
ros2_control 4.45.2, gz_ros2_control 1.2.19, ros2_controllers 4.40.1, MoveIt
2.12.4, and MoveIt Task Constructor 0.1.5.

## Next exact steps

1. Write failing P2.1 contracts for a six-joint `manipulator` SRDF
   (`base_link` -> `tool0`), canonical MoveIt limits, kinematics, OMPL, and the
   exact `/joint_trajectory_controller/follow_joint_trajectory` mapping.
2. Add `abb_lab_moveit_config` with no gripper and no fabricated acceleration or
   jerk values; build and test it on ext4.
3. In P2.2, launch the existing project Gazebo runtime plus `move_group`, prove
   a joint and `tool0` pose plan/execution, and prove an intentional collision
   is rejected. Mark A02 PASS only after that complete clean integrated run.
