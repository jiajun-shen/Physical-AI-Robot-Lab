# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 4 - RGB-D perception (`AUTHORIZED - NOT STARTED`)

Project state: `PHASE 4 AUTHORIZED - PRE-IMPLEMENTATION CHECKPOINT`.

## Current subtask

The user explicitly accepted the saved Phase-3 two-seed real-run evidence on
2026-08-14. Begin Phase 4 with P4.1: define and verify the RGB-D CameraInfo/TF
contract in `vision` mode. This release does not alter the Phase-3 capability
boundary and does not replace the later mandatory M1 hold after Phase 8.

## Git checkpoints

- Latest runtime-qualified feature commit: `f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63`
- Stable feature commit: `f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63`
- Experimental commit: none
- Stable base branch: `main` at `f994fd651e4817f053121227383a8c89bb23b0ed`
- Active Phase-4 branch: `agent/phase4-perception` tracking
  `origin/agent/phase4-perception`
- Remote: `https://github.com/jiajun-shen/Physical-AI-Robot-Lab.git` (private)

The Phase-3 final evidence is clean and commit-addressable: two separately
launched selected seeds captured their live Gazebo model poses on the same
commit. It remains a gripper/laboratory gate, not an implicit contact, grasp,
perception, or MoveIt-with-gripper result.

## Working features

- ROS 2 Jazzy, Gazebo Harmonic, ros2_control/controllers, MoveIt, and MoveIt
  Task Constructor are installed in WSL Ubuntu 24.04.
- `abb_lab_description` contains the pinned ABB IRB1200 geometry and separate
  fixed-base `GazeboSimSystem` control composition.
- `abb_lab_control` supplies six-axis position JTC/JSB controller policy.
- `abb_lab_gazebo` supplies the minimal headless world and ordered
  spawn/RSP/controller startup sequence.
- `abb_lab_moveit_config` supplies an arm-only six-joint SRDF, canonical
  position/velocity planning limits, KDL, a single OMPL pipeline, and the exact
  existing JTC action mapping. Its headless integration launch adds one
  `move_group` to the already owned Gazebo controller chain; it never starts a
  second controller manager.
- `abb_lab_gripper` adds an isolated ABB-with-two-finger simulation profile:
  one commanded right prismatic finger, one state-only mirrored left finger,
  and the existing safety parameter file ahead of gripper controller policy.
- `abb_lab_lab` supplies a static default laboratory template plus an explicit
  deterministic external scene generator; it is the generator/harness, not the
  default launch alone, that selects randomized scenes.
- `abb_lab_bringup` starts the separate gripper/lab profile with one
  Gazebo-hosted controller manager and the JSB -> arm JTC -> gripper action
  controller sequence. It selects Bullet Featherstone only for this mimicked
  gripper profile.
- A project-owned ABB IRB1200 spawned in Gazebo as `abb_irb1200`; six position
  command interfaces were claimed, JTC/JSB were active, command-limit
  enforcement initialized, and an observed trajectory completed.
- The project is maintained on the private GitHub repository under
  `jiajun-shen`; clean runtime checkpoints are pushed after their evidence and
  state records are committed.

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
- P2.1 static MoveIt configuration contract at `281c36b`.
  - Clean build/test: 3 packages, 45 records, zero errors/failures/skips.
  - The installed MoveIt builder expanded the runtime Xacro and verified the
    URDF/SRDF chain, canonical limits, collision adapters, and JTC mapping.
  - This historical static gate was not planning/execution evidence.
- P2.2 clean headless MoveIt runtime at `4203e81`.
  - Clean verifier completion: 2026-08-11 17:01:57 +0200.
  - One existing `GazeboSimSystem`/controller manager, active JSB/JTC, and one
    ready `move_group` were observed.
  - MoveIt planned/executed a normal joint route and
    `HOME -> FK-derived tool0 Pose -> HOME` through the existing JTC.
  - A synchronously applied planning-scene box invalidated the target; both
    pose and joint plans into it were rejected, and the object was read back as
    removed.
  - Four packages produced 58 test records with zero errors/failures/skips;
    source manifest `ad0b91e8dc33641893fc98c2e8aebaa995d90e996944e37b41fe977b09abb968`
    matched before and after the run.
  - A02 is `PASS`; see `docs/phase2_moveit_runtime_evidence.md`.
- Phase 3 final gripper/laboratory runtime at `f6907ff`.
  - Two independent clean headless runs selected seeds 42 and 43, with the
    same source manifest
    `63e531b1a67c42132d434ef721cb5c9b076df8a8781d58e385d121ea039b0736`.
  - Each run had one active `GazeboSimSystem`, seven claimed position command
    interfaces, 22 state interfaces, active JSB/JTC/gripper controller, and
    command-limit enforcement.
  - Each run captured all ten required live Gazebo model names and poses after
    a real free-space gripper open/closed-clearance/reopen action sequence.
  - The independent comparison of the two live captures proved material
    position variation for two obstacles and four objects and directional yaw
    variation for both cubes. It did not infer the scene from an SDF or
    manifest.
  - Each run had 82 colcon test records with zero errors, failures, or skips;
    cleanup passed. See `docs/phase3_gripper_lab_runtime_evidence.md`.

## Partially completed features

- Headless MoveIt planning/execution and collision rejection are verified, and
  Phase 3 verified a position-only free-space gripper plus two live seeded lab
  scenes. GUI validation, RGB-D perception, contact/force/attachment/grasp,
  arm/MoveIt execution in the gripper lab, pick/place, and an out-of-range
  clamp probe remain unverified.
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
- P2.2 clean integrated MoveIt runtime: PASS at `4203e81`.
  - `SOURCE_WORKTREE_CLEAN=true`; the before/after source manifest is
    `ad0b91e8dc33641893fc98c2e8aebaa995d90e996944e37b41fe977b09abb968`.
  - Four MoveIt executions, planning-scene collision rejection, and temporary
    object removal passed. Normal pose observation error was `0.0063663304 m`
    and `0.0629389291 rad` within explicitly configured observed execution
    acceptance tolerances.
  - 58 colcon records passed. No owned process remained, but requested
    teardown observed `move_group` SIGSEGV; see KI-012.
- Phase-3 final clean runtime: PASS at `f6907ff` for selected seeds 42 and 43.
  - Both were clean-worktree, 82-record runs; both passed ABB/gripper spawn,
    control resources, active controllers, live ten-model pose capture,
    free-space gripper action, and cleanup.
  - The comparison artifact independently re-parsed live Gazebo poses and
    proved position variation for the two obstacles and four movable objects,
    plus yaw variation for `blue_cube` and `yellow_cube`.
  - `GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE`, force control is not
    configured, and attachment is not implemented in both runs.
- A01-A02: PASS. A03-A14: NOT STARTED.
- P2.1 clean static configuration: PASS at `281c36b`.
  - `rosdep check`: PASS.
  - 3 packages built; 45 test records passed; worktree stayed clean.
  - Evidence: `docs/phase2_moveit_config_evidence.md`.

## Current build status

The final Phase-3 gates each built and tested `abb_lab_description`,
`abb_lab_control`, `abb_lab_gazebo`, `abb_lab_moveit_config`,
`abb_lab_gripper`, `abb_lab_lab`, and `abb_lab_bringup` on ext4. Generated
build/install/log trees and runtime episodes remain outside Git. Phase 3 is
complete at the user-approved foundational gripper-and-objects scope; its
explicit user acceptance now authorizes Phase 4, but no Phase-4 runtime claim
exists at this checkpoint.

## Current run status

- Project ABB headless Gazebo spawn/control: verified.
- Project MoveIt planning/execution and collision rejection: verified in
  headless simulation at `4203e81`.
- Project gripper/laboratory profile: verified as free-space position action
  plus captured live selected-seed variation at `f6907ff`; no arm motion,
  contact, force, attachment, or grasp claim.
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
- The clean P2.2 run validated a headless collision scene, but not GUI,
  physical effort behavior, out-of-range clamp, or a real robot. P3 later
  added an isolated free-space gripper/lab profile, not a MoveIt-with-gripper
  execution.
- Phase 3 is complete only at the user-approved foundational gripper-and-live-
  objects boundary. There is no valid finger-force signal, contact sensor,
  attachment, grasp inference, lift, transport, RGB-D observation, or
  pick/place result; see KI-013.
- The P3 final runs show step-goal position-rate limiter interventions. They
  are not an out-of-range clamp probe or a general clamp behavior claim.
- The default lab launch is static; only the explicit generator/harness selects
  the seeded worlds used in the final evidence.
- The installed `move_group` process had SIGSEGV during requested P2.2 teardown
  despite no residual owned processes; this is KI-012 and is not a graceful
  cleanup result.
- The GitHub repository remains private while the first-party license is open.
- Final first-party license, speech-model pin, and VLA GPU service location are
  open design decisions.

## Phase-3 acceptance record (released 2026-08-14)

The user explicitly accepted the final two-seed Phase-3 real-run evidence at
`f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63` and authorized Phase 4 on
2026-08-14. P3.03 is therefore `PASS`. This recorded authorization does not
make a contact/grasp claim and does not modify the separate Phase-8 M1 stop
condition below.

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

1. Define P4.1's real-run evidence contract, then implement and verify RGB-D,
   CameraInfo, and TF transport in `vision` mode without consuming Gazebo
   ground truth. Keep A03 `NOT STARTED` until the complete real vision-mode
   acceptance run passes.
2. Build the remaining Phase-4 detection, depth-projection, and typed pose
   pipeline in atomic checkpoints with saved evidence and clean tested commits.
3. Preserve the Phase-8 M1 hold unchanged: even after any later Phase-4--8
   work, Phase 9 requires final integrated M1 evidence and separate explicit
   user acceptance.
