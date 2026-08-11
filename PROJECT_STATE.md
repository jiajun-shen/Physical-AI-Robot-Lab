# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 1 — ABB IRB1200 + Gazebo Harmonic + ros2_control (`IN PROGRESS`)

## Current subtask

Checkpoint the verified ABB geometry-only asset/package slice, then implement
the project-owned fixed-base Gazebo Harmonic ros2_control overlay.

## Git checkpoints

- Latest known-good commit: `522eadb19426401890e08d534edc7d95df30fc70`
- Stable commit: `522eadb19426401890e08d534edc7d95df30fc70`
- Experimental commit: none
- Current branch: `main`
- Remote: none configured

The current ABB asset/package implementation and evidence pass locally but are
not yet committed. The stable hash above is the dependency-baseline state
checkpoint.

## Working features

- Recoverable Git repository and session handoff documentation.
- ROS 2 Jazzy CLI and RViz2 binaries in WSL Ubuntu after explicit sourcing.
- Gazebo Harmonic, ros_gz, gz_ros2_control, ros2_control/controllers, MoveIt,
  and MoveIt Task Constructor are installed and discoverable.
- The installed upstream headless cart executes a real trajectory action through
  gz_ros2_control and publishes the resulting joint state.
- Project package `abb_lab_description` builds and expands a pinned
  geometry-only IRB1200-5/0.9 with 10 links, 9 joints, and 14 mesh references.

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

## Partially completed features

- Phase 1 dependency/runtime baseline is verified and checkpointed.
- The ABB asset snapshot and geometry-only package are implemented, built,
  expanded, and tested; their checkpoint commit is pending.
- Fixed-base Gazebo/control composition and enforced limits are not implemented.
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
- Phase 1 Python source/asset tests: PASS (15/15).
- `scripts/build_and_test_abb_description.sh`: PASS on a clean output tree.
  - colcon build: one package finished
  - real Xacro expansion and `check_urdf`: PASS
  - expanded links/joints/revolute joints/meshes: 10/9/6/14
  - installed locked assets: 19/19
  - CTest targets: 2/2; 17 records, 0 errors/failures/skips
- Package-only copy at unrelated workspace depth: build, Xacro, URDF, installed
  asset verification, and 17 test records PASS.
- Shell syntax (`bash -n`) for all six Phase 1 scripts: PASS.
- MoveIt project planning: NOT RUN.
- Acceptance A01–A14: all `NOT STARTED`.

## Current build status

`abb_lab_description` builds successfully with colcon using DrvFS source and
ext4 `/tmp` build/install/log trees. Generated trees and expanded URDF are not
committed. The package also builds/tests independently after a package-only copy
to WSL ext4. No control, simulator, or MoveIt project package exists yet.

## Current run status

- `ros2 --help`: verified.
- `rviz2 --help`: verified; GUI not run.
- Gazebo gz-sim 8.11.0 upstream headless cart: launch/shutdown verified.
- gz_ros2_control trajectory execution: verified on upstream cart only.
- Project ABB geometry Xacro: expanded and parsed; this is not a simulator run.
- Physical-AI-Robot-Lab ABB runtime: not implemented.
- VLA backend: `VLA_BACKEND_UNAVAILABLE`.

## Known bugs

No unresolved project-code bug. Two harness bugs were fixed and fully rerun:
the control smoke parser initially missed ROS 2's `array('d', [...])` output,
and the description build initially sourced colcon setup under Bash nounset.

## Known limitations

- WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound.
- Local GPU has 8151 MiB, below openpi's strict `> 8 GB` statement.
- No CUDA Toolkit or Docker is present.
- `abb_ros2` has no Jazzy branch/tag and tests only Rolling upstream.
- ABB model inertials are estimates, efforts are fictional, and upstream
  URDF/ros2_control limits conflict.
- Geometry-only description has no world joint, Gazebo plugin, ros2_control,
  MoveIt, gripper, or approved operational control limits.
- The upstream smoke demo disables command-limit enforcement. It proves only
  the dependency path and is not an acceptable ABB safety configuration.
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
- ABB overlay must enable command limits and reconcile URDF, ros2_control, and
  MoveIt definitions before motion.
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

1. Commit the pinned asset snapshot, geometry package, tests, evidence, and
   long-term document updates; then record the feature commit as stable.
2. Start `P1.3a`: write failing contracts for a fixed `world` joint, exactly six control
   joints, one canonical position/velocity limit table, and enabled command
   limits; add the description/control Xacro with
   `gz_ros2_control/GazeboSimSystem`, then expand/build/test/commit without
   claiming a Gazebo run.
3. Start `P1.3b`: add controller YAML and a minimal project world without a
   gripper, launch headlessly, and inspect resource/controller state.
4. Command a bounded ABB joint trajectory, verify observed joint states, and
   only then evaluate A01.
