# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 1 — ABB IRB1200 + Gazebo Harmonic + ros2_control (`IN PROGRESS`)

## Current subtask

Implement `P1.3b`: add project-owned controller definitions, a minimal Gazebo
world/SDF, and a headless launch; then inspect ABB resource/controller/limiter
state and execute one bounded observed trajectory. Treat clamp verification as
a separate out-of-range probe.

## Git checkpoints

- Latest known-good commit: `2ec43308498e8e8ffb83da9b9d037ab50f84d83f`
- Stable commit: `2ec43308498e8e8ffb83da9b9d037ab50f84d83f`
- Experimental commit: none
- Current branch: `main`
- Remote: none configured

The self-contained ABB geometry and static control-description contracts,
tests, and evidence are committed at the stable hash above. This state
checkpoint records `P1.3a` as complete and keeps Phase 1 in progress.

## Working features

- Recoverable Git repository and session handoff documentation.
- ROS 2 Jazzy CLI and RViz2 binaries in WSL Ubuntu after explicit sourcing.
- Gazebo Harmonic, ros_gz, gz_ros2_control, ros2_control/controllers, MoveIt,
  and MoveIt Task Constructor are installed and discoverable.
- The installed upstream headless cart executes a real trajectory action through
  gz_ros2_control and publishes the resulting joint state.
- Project package `abb_lab_description` builds and expands the unchanged pinned
  geometry entry point with 10 links, 9 joints, and 14 mesh references.
- Its separate static control composition expands to 11 links, 10 kinematic
  joints, and one `GazeboSimSystem` with six position-command joints.
- The simulation position/velocity table, ordinary URDF, and interface bounds
  agree; startup command enforcement is configured true, and the installed
  Gazebo resource export/plugin registrations pass static inspection.

No ABB Gazebo/control runtime, project world, perception, speech, task, skill,
or VLA runtime feature is claimed working.

## Completed features

- Created and committed the recoverable repository baseline.
- Completed and committed Phase 0 host, dependency, upstream, license, and
  architecture audits.
- Verified the official Jazzy/Harmonic/MoveIt/control compatibility baseline.
- Audited/pinned the ABB IRB1200 asset source and documented its six joints,
  frames, meshes, limit conflicts, approximate inertials, and missing Harmonic
  integration.
- Audited/pinned openpi requirements and recorded local
  `VLA_BACKEND_UNAVAILABLE` / `EXPERIMENTAL / NOT ABB-TUNED` constraints.
- Captured and committed exact pre-install apt candidates.
- Installed those exact top-level simulator, control, MoveIt, and MTC packages;
  apt added 309 packages without upgrading/removing existing packages.
- Captured 431 installed `ros-jazzy-*` package versions in a generated manifest.
- Sent a real two-second `FollowJointTrajectory` goal from 1.0 to 2.0 through
  Gazebo and gz_ros2_control; action succeeded with error code 0 and final
  `/joint_states` position `1.9999999999999991`.
- Committed the package-local, full-SHA/SHA-256-locked IRB1200 asset subset and
  independently buildable geometry-only `abb_lab_description` package at
  `2a536eb`.
- Committed P1.3a's separate fixed-base Harmonic control description, canonical
  simulation limits, resource export, executable contracts, and evidence at
  `2ec4330`.

## Partially completed features

- Phase 1 dependency/runtime baseline is verified and checkpointed.
- Static fixed-base control composition and configured limits are verified;
  project Gazebo spawn, controllers, limiter initialization, clamping, and ABB
  trajectory execution are not implemented.
- WSLg endpoints and RViz binary exist, but real 3D GUI rendering is untested.

## Not started features

- A01 ABB IRB1200 Gazebo Harmonic spawn.
- A02 ABB Home → Pose → Home collision-free execution.
- Phase 2 project MoveIt planning and all later phases.
- Gripper/world, RGB-D perception, pick/place, drawing, speech, TaskGraph
  executive, System Explorer, robotics core, VLA, dataset, training, benchmarks,
  and hardening/CI.

## Current test status

- Phase 0 documentation checkpoint: PASS at `eb1b8e8`.
- Phase 1 dependency/control checkpoint: PASS at `14d946f`.
- Phase 1 ABB asset/geometry checkpoint: PASS at `2a536eb`.
- Phase 1 ABB static control-description checkpoint: PASS at `2ec4330`.
  - Full evidence: `docs/phase1_abb_control_description_evidence.md`
- Pre-install apt lock vs a second `apt-cache policy`: PASS (10/10 exact).
- `scripts/verify_ros_environment.sh`: PASS.
  - all 10 direct packages fully installed
  - required ROS package prefixes resolve under `/opt/ros/jazzy`
  - gz-sim version 8.11.0
  - ros2_control CLI loads
  - `dpkg --audit` clean
- `scripts/verify_ros_apt_manifest.sh`: PASS.
  - 431 well-formed, sorted, unique installed-package rows
  - all 10 direct versions match dpkg, the direct lock, and the manifest
- Root `apt-get check`: PASS.
- `scripts/smoke_test_gz_ros2_control.sh`: PASS on complete rerun.
  - `joint_state_broadcaster`: active
  - `joint_trajectory_controller`: active
  - trajectory action: `SUCCEEDED`, `error_code: 0`
  - target 2.0, observed final position `1.9999999999999991`
  - no `ERROR`/`FATAL` or leftover launched processes
- Package-local `config/abb-model-assets.repos` fresh `vcs import`: PASS at exact detached
  `05cbabe774ef026b1cd36fe6b253b55f55e16340`.
- ABB snapshot vs pinned Git objects: PASS (19/19 files).
- Phase 1 Python source/asset/control tests: PASS (23/23).
- `scripts/build_and_test_abb_description.sh`: PASS on a clean output tree.
  - colcon build: one package finished
  - geometry and control Xacro expansion plus both `check_urdf`: PASS
  - geometry links/joints/revolute joints/meshes: 10/9/6/14
  - control links/kinematic joints/control joints/canonical limits: 11/10/6/6
  - installed locked assets: 19/19
  - controller_manager 4.45.2 and gz_ros2_control 1.2.19 contract checks: PASS
  - Gazebo model resource export and system plugin inspection: PASS
  - CTest targets: 3/3; 23 unique pytest cases; 26 total wrapper/xUnit records,
    0 errors/failures/skips
- Package-only copy at `/var/tmp/p13a-gz-path-audit.wwtjBKcD`: build, tests,
  both Xacros/URDF verifiers, official Gazebo path discovery, and both SDF
  conversions PASS; all 14 mesh URIs in each SDF resolve through the export.
- `rosdep check` for `abb_lab_description`: PASS.
- Shell syntax (`bash -n`) for all six Phase 1 scripts: PASS.
- MoveIt project planning: NOT RUN.
- Acceptance A01–A14: all `NOT STARTED`.

## Current build status

`abb_lab_description` builds successfully with colcon using DrvFS source and
ext4 `/tmp` build/install/log trees. Generated trees and expanded URDF are not
committed. The package also builds/tests independently after a package-only copy
to WSL ext4. The package contains a static control description, but no
controller definitions, Gazebo world/SDF, launch, or MoveIt project package
exists yet.

## Current run status

- `ros2 --help`: verified.
- `rviz2 --help`: verified; GUI not run.
- Gazebo gz-sim 8.11.0 upstream headless cart: launch/shutdown verified.
- gz_ros2_control trajectory execution: verified on upstream cart only.
- Project ABB geometry/control Xacros: expanded, parsed, and converted to valid
  SDF; this is not a simulator run.
- Physical-AI-Robot-Lab ABB runtime: not implemented.
- VLA backend: `VLA_BACKEND_UNAVAILABLE`.

## Known bugs

No unresolved project-code bug. Three harness bugs were fixed and fully rerun:
the control smoke parser initially missed ROS 2's `array('d', [...])` output,
the description build initially sourced colcon setup under Bash nounset, and
the first control verifier expected an unresolved `$(find ...)` literal instead
of Xacro's installed absolute path.

## Known limitations

- WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound.
- Local GPU has 8151 MiB, below openpi's strict `> 8 GB` statement.
- No CUDA Toolkit or Docker is present.
- `abb_ros2` has no Jazzy branch/tag and tests only Rolling upstream.
- ABB model inertials are estimates and efforts are fictional. P1.3a reconciles
  simulation position/velocity values only; derivatives and MoveIt limits are
  not defined.
- The geometry-only entry remains control-free. The separate control
  composition has no Gazebo world/SDF, controller definitions, launch, MoveIt,
  gripper, or real-hardware integration.
- Project command-limit enforcement is configured true but has not been
  initialized or clamp-tested at runtime. The upstream smoke proves only the
  dependency/control path and is not ABB safety evidence.
- Repository is on Windows DrvFS and has no remote backup.

## Unresolved questions

- Whether Gazebo/RViz WSLg 3D rendering is stable on this driver/GPU.
- Whether colcon build trees under `/mnt/c` are reliable enough or must live on
  WSL ext4.
- Final first-party repository license.
- Whisper/model pin (deferred to Phase 7).
- VLA GPU service location; no paid training/remote resource is authorized.

## Important architecture decisions

- Working classical baseline precedes advanced autonomy and VLA authority.
- Use official Jazzy apt packages and the default Harmonic pairing.
- Reuse pinned ABB geometry/kinematic assets, not the unverified complete driver;
  build a project-owned Jazzy/Harmonic control/MoveIt overlay.
- Keep geometry provenance separate from simulation control policy.
- P1.3b must verify runtime limiters and an in-range control trajectory;
  out-of-range clamping is a separate test. MoveIt limits are required before
  Phase 2 planned/collision-aware motion.
- Language planning emits typed Task/Skill/Goal structures, never joint angles.
- `vision` mode cannot access simulator ground truth.
- ROS and openpi remain isolated behind a versioned policy API.
- Missing VLA produces `VLA_BACKEND_UNAVAILABLE`; a fallback success is labelled
  CLASSICAL, never VLA.

## Dependency versions

See `DEPENDENCY_MATRIX.md`, `config/ros-jazzy-apt.lock.yaml`,
`config/ros-jazzy-installed-packages.lock.tsv`, and the reports under `docs/`.

Important installed versions:

- Ubuntu 24.04.4 LTS WSL2; ROS 2 Jazzy Desktop
- Python 3.12.3; GCC/G++ 13.3.0; CMake 3.28.3
- Gazebo gz-sim 8.11.0; ros_gz 1.0.22; gz_ros2_control 1.2.19
- ros2_control 4.45.2; ros2_controllers 4.40.1
- MoveIt 2.12.4; MoveIt Task Constructor 0.1.5
- NVIDIA driver 591.91; no CUDA Toolkit

## External repository commits

- PickNik `abb_ros2`: `05cbabe774ef026b1cd36fe6b253b55f55e16340`
- ROS-Industrial `abb`: `eefdf01c956a6f2129aacb88d598201feff3cf19`
- Physical Intelligence `openpi`: `15a9616a00943ada6c20a0f158e3adb39df2ccac`

Additional audited source-release pins are in `THIRD_PARTY.md`.

## Important commands

```powershell
git status --short --branch
git log --oneline --decorate -10
wsl -d Ubuntu-24.04
```

```bash
source /opt/ros/jazzy/setup.bash

bash scripts/verify_ros_environment.sh

bash scripts/verify_ros_apt_manifest.sh \
  config/ros-jazzy-installed-packages.lock.tsv \
  config/ros-jazzy-apt.lock.yaml

bash scripts/smoke_test_gz_ros2_control.sh \
  /tmp/physical-ai-robot-lab/gz_ros2_control-smoke

bash scripts/capture_ros_apt_manifest.sh \
  config/ros-jazzy-installed-packages.lock.tsv

bash scripts/capture_abb_asset_hashes.sh \
  ros2_ws/src/abb_lab_description/third_party/abb_irb1200_support \
  /tmp/abb_ros2-clean-exact-sha-checkout \
  ros2_ws/src/abb_lab_description/config/abb-irb1200-assets.sha256

bash scripts/build_and_test_abb_description.sh
```

## Next exact steps

1. Write failing P1.3b contracts for six-joint controller definitions, a
   minimal no-gripper world/SDF, headless launch, and bounded test trajectory.
2. Launch the project ABB headlessly; inspect hardware resources, controller
   states, `enforce_command_limits=true`, and six initialized joint limiters.
3. Execute one in-range bounded trajectory and verify observed joint states.
   Test out-of-range clamping separately; evaluate A01 only after the actual ABB
   spawn is observed. Keep A02-A14 `NOT STARTED`.
