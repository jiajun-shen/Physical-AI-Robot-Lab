# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 1 — ABB IRB1200 + Gazebo Harmonic + ros2_control (`IN PROGRESS`)

## Current subtask

Import the pinned ABB IRB1200 model assets with provenance and structural tests,
without importing or claiming the unverified upstream driver stack.

## Git checkpoints

- Latest known-good commit: `14d946f5b0d498aa028d5b7c3179eb5ba7da59ab`
- Stable commit: `14d946f5b0d498aa028d5b7c3179eb5ba7da59ab`
- Experimental commit: none
- Current branch: `main`
- Remote: none configured

The hash above contains the exact installed ROS package manifest, environment
and manifest verifiers, the reproducible upstream control smoke, and its
inspected evidence.

## Working features

- Recoverable Git repository and session handoff documentation.
- ROS 2 Jazzy CLI and RViz2 binaries in WSL Ubuntu after explicit sourcing.
- Gazebo Harmonic, ros_gz, gz_ros2_control, ros2_control/controllers, MoveIt,
  and MoveIt Task Constructor are installed and discoverable.
- The installed upstream headless cart executes a real trajectory action through
  gz_ros2_control and publishes the resulting joint state.

No ABB, project world, perception, speech, task, skill, or VLA runtime feature
is claimed working.

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
- Phase 1 ABB assets and project-owned description/control overlay are not yet
  integrated.
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
- Shell syntax (`bash -n`) for all four Phase 1 scripts: PASS.
- Python tests: no project Python package/tests yet.
- ROS project tests: no project ROS package/tests yet.
- MoveIt project planning: NOT RUN.
- Acceptance A01–A14: all `NOT STARTED`.

## Current build status

No project ROS package/build exists. WSL now has the required binary Gazebo,
MoveIt, ros2_control/controller, CMake, GCC/G++, colcon, rosdep, and vcstool
baseline. No generated `build/`, `install/`, or `log/` tree is committed.

## Current run status

- `ros2 --help`: verified.
- `rviz2 --help`: verified; GUI not run.
- Gazebo gz-sim 8.11.0 upstream headless cart: launch/shutdown verified.
- gz_ros2_control trajectory execution: verified on upstream cart only.
- Physical-AI-Robot-Lab ABB runtime: not implemented.
- VLA backend: `VLA_BACKEND_UNAVAILABLE`.

## Known bugs

No unresolved project-code bug. The first smoke run reached the target but its
harness did not parse ROS 2's `array('d', [...])` output. The parser was fixed
and the entire smoke passed on rerun.

## Known limitations

- WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound.
- Local GPU has 8151 MiB, below openpi's strict `> 8 GB` statement.
- No CUDA Toolkit or Docker is present.
- `abb_ros2` has no Jazzy branch/tag and tests only Rolling upstream.
- ABB model inertials are estimates, efforts are fictional, and upstream
  URDF/ros2_control limits conflict.
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
```

Project build/run commands will be added only after they actually exist/run.

## Next exact steps

1. Create a full-SHA `.repos` import for the pinned ABB model source, excluding
   the EGM/RWS driver dependency chain from the simulation baseline.
2. Write structural Xacro tests first for exactly six revolute joints, expected
   names/frames, mesh resources, finite inertials, and one canonical limit set.
3. Import only the licensed IRB1200 support assets at the audited commit and
   make the structural tests pass.
4. Implement a project-owned fixed-base Jazzy/Harmonic description/control
   overlay with unified/enforced joint limits and no gripper yet.
