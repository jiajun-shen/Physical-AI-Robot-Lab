# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 4 - RGB-D perception (`COMPLETE`)

Project state: `PHASE 4 COMPLETE - STOP; PHASE 5 NOT STARTED`.

## Current subtask

Phase 4 completed at clean tested feature commit
`62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`; see
`docs/phase4_perception_runtime_evidence.md`. Its two real seeded RGB-D runs
received only RGB, depth, CameraInfo, and TF in `vision` mode, sealed that
evidence before a separate offline live-Gazebo comparison, and passed all five
typed localizations plus A03. Phase 5 is not started in this session. Preserve
the P3 position-only/no-contact boundary and the mandatory M1 hold after Phase
8.

## Git checkpoints

- Latest runtime-qualified feature commit: `62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`
- Stable feature commit: `62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`
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
- The original P3 `manipulation_lab.sdf` and P3 harness remain unchanged.
  The separate `manipulation_lab_rgbd.sdf` owns the RGB-D sensor used only by
  the Phase-4 launch and seeded Phase-4 harness.
- `abb_lab_bringup` starts the separate gripper/lab profile with one
  Gazebo-hosted controller manager and the JSB -> arm JTC -> gripper action
  controller sequence. It selects Bullet Featherstone only for this mimicked
  gripper profile.
- `physical_ai_interfaces` provides the project-owned typed
  `ObjectDetection`/`ObjectPose` messages. `physical_ai_perception` consumes
  only ROS RGB, depth, CameraInfo, and TF, produces camera/world typed poses,
  and has no Gazebo/model-pose/manifest/seed input dependency.
- The independent Phase-4 harness seals the vision artifacts before its
  separate live-Gazebo capture/evaluator process and verifies all owned
  process groups are cleaned up.
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
- Phase 4 final RGB-D perception runtime at `62dc9c8`.
  - The clean two-seed harness built/tests 29 records with zero errors,
    failures, or skips, then ran seeds 42 and 43 in isolated Gazebo worlds.
  - Real `rgb8` RGB, `32FC1` metre depth, CameraInfo, and the
    `world -> lab_camera_optical_frame` TF chain drove five OpenCV/depth/TF2
    typed localizations in `PERCEPTION_MODE=vision`.
  - Vision evidence was sealed with `ground_truth_input_used: false` before
    separate offline live-Gazebo captures. All object errors were within the
    0.05 m object / 0.06 m board thresholds; the green-ball errors were
    0.01033 m (seed 42) and 0.01509 m (seed 43), so A03 is `PASS`.
  - Evidence root: `/var/tmp/physical-ai-p4.Kn4hFG`; cleanup and source
    manifest equality passed. See `docs/phase4_perception_runtime_evidence.md`.

## Partially completed features

- Headless MoveIt planning/execution and collision rejection are verified,
  Phase 3 verified a position-only free-space gripper plus two live seeded lab
  scenes, and Phase 4 verified simulation-only RGB-D object localization.
  GUI validation, contact/force/attachment/grasp, arm/MoveIt execution in the
  gripper lab, pick/place, and an out-of-range clamp probe remain unverified.
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
- A01-A03: PASS. A04-A14: NOT STARTED.
- Phase-4 final clean runtime: PASS at `62dc9c8` for selected seeds 42 and 43.
  - `P4_RGB_TRANSPORT`, `P4_DEPTH_TRANSPORT`, `P4_CAMERA_INFO`,
    `P4_TF_CHAIN`, `P4_DEPTH_PROJECTION`, `P4_ALL_FIVE_DETECTIONS`,
    `P4_VISION_GROUND_TRUTH_BOUNDARY`, `P4_OFFLINE_EVALUATION`, and
    `P4_TWO_SEED_VISION_ONLY_LOCALIZATION` are all PASS.
  - `A03=PASS`, `A04=NOT_STARTED`, and
    `GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE`.
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
explicit user acceptance authorized Phase 4, but no complete Phase-4
perception runtime claim existed at that historical checkpoint.

The final Phase-4 gate built/tested the Phase-4 dependency closure on ext4 and
recorded 29 test records with zero errors, failures, or skips. At `62dc9c8`,
the clean-worktree two-seed runtime sealed each real vision-only evidence
bundle before offline ground-truth comparison. Its source-manifest SHA-256 was
`efa51900468bba2a5a6ce2741b95aed458e0e04fa8f58a973485a270582aee54` before
and after the run.

## Current run status

- Project ABB headless Gazebo spawn/control: verified.
- Project MoveIt planning/execution and collision rejection: verified in
  headless simulation at `4203e81`.
- Project gripper/laboratory profile: verified as free-space position action
  plus captured live selected-seed variation at `f6907ff`; no arm motion,
  contact, force, attachment, or grasp claim.
- Phase-4 RGB-D perception: verified headlessly at `62dc9c8` on seeds 42 and
  43, including real RGB/depth/CameraInfo/TF2, five vision-only typed poses,
  sealed evidence, and subsequent offline live-pose comparison. A03 is PASS.
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
  objects boundary. Phase 4 adds simulation-only RGB-D localization, but
  there is still no valid finger-force signal, contact sensor, attachment,
  grasp inference, lift, transport, or pick/place result; see KI-013.
- The P3 final runs show step-goal position-rate limiter interventions. They
  are not an out-of-range clamp probe or a general clamp behavior claim.
- The default lab launch is static; only the explicit generator/harness selects
  the seeded worlds used in the final evidence.
- Phase 4's OpenCV baseline is verified only for the defined five labels in
  the two seeded simulation scenes. It is not real-camera, arbitrary-occlusion,
  instance-segmentation, grasp, or placement evidence; see KI-014.
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

1. Stop after this Phase-4 documentation checkpoint. Do not begin Phase 5 in
   this session; await explicit user direction.
2. If Phase 5 is later authorized, start with a new, separately reviewed
   pick/place safety and evidence contract. It must preserve the vision-only
   ground-truth boundary and make no contact/grasp claim before fresh runtime
   evidence exists.
3. Preserve the Phase-8 M1 hold unchanged: even after any later Phase-4--8
   work, Phase 9 requires final integrated M1 evidence and separate explicit
   user acceptance.
