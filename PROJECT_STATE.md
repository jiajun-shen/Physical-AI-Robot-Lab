# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Phase 1 — ABB IRB1200 + Gazebo Harmonic + ros2_control (`IN PROGRESS`)

## Current subtask

Install the exact top-level packages recorded in
`config/ros-jazzy-apt.lock.yaml`, capture post-install versions, and pass the
upstream gz_ros2_control headless smoke test before importing ABB assets.

## Git checkpoints

- Latest known-good commit: `eb1b8e8bad4365894b16e95dd93128246270394c`
- Stable commit: `eb1b8e8bad4365894b16e95dd93128246270394c`
- Experimental commit: none
- Current branch: `main`
- Remote: none configured

The stable commit contains the verified Phase 0 reports and architecture.

## Working features

- Recoverable Git repository and session handoff documentation.
- ROS 2 Jazzy CLI and RViz2 binaries in WSL Ubuntu after explicit environment
  sourcing.
- Host/upstream audit commands have run and their results are documented.

No ABB, Gazebo, MoveIt, perception, speech, task, skill, or VLA runtime feature
is claimed working.

## Completed features

- Audited the target workspace; no pre-existing project required preservation.
- Initialized Git on `main`, added artifact/secret exclusions, and created the
  initial baseline commit.
- Audited Windows, WSL2 Ubuntu, CPU/RAM/GPU/CUDA, toolchains, ROS 2, Gazebo,
  MoveIt, ros2_control, Docker, Python, storage, and Git state.
- Verified the official Jazzy/Harmonic/MoveIt/control compatibility baseline.
- Audited ABB IRB1200-5/0.9 asset source, six joints, frames, limits, meshes,
  licenses, missing Harmonic integration, and upstream compatibility risks.
- Audited openpi at a fixed commit, its environment/GPU requirements, remote
  inference boundary, licenses, and absence of an ABB-tuned checkpoint.
- Designed package boundaries and classical/HYBRID/VLA safety separation.

## Partially completed features

- Phase 1 environment preparation has started; packages are not installed yet.
- WSLg endpoints and RViz binary exist, but 3D GUI rendering is untested.
- The ROS/Gazebo/MoveIt/control stack is selected but missing locally.

## Not started features

- Phase 1 ABB Gazebo Harmonic spawn and ros2_control execution.
- Phase 2 MoveIt collision-aware planning.
- Phase 3 gripper and deterministic randomized world.
- Phase 4 RGB-D perception.
- Phase 5 verified pick/place.
- Phase 6 parametric drawing.
- Phase 7 multilingual speech.
- Phase 8 TaskGraph executive/recovery.
- Phase 9 System Explorer.
- Phase 10 educational robotics core.
- Phases 11–15 VLA interface, ABB adaptation, data, fine-tuning, benchmarks.
- Phase 16 hardening, final documentation, and CI.

## Current test status

- Repository baseline commit: created successfully.
- Environment audit commands: executed successfully; unavailable components are
  recorded as unavailable rather than failures.
- Upstream ABB source inspection: six revolute joints, meshes, base/flange/tool
  frames, and conflicting limit sources observed at pinned commit.
- Phase 0 documentation checkpoint: committed as `eb1b8e8`.
- Phase 1 pre-install apt candidates captured for the simulator, control,
  MoveIt, and MoveIt Task Constructor packages.
- Phase 0 documentation consistency checks: PASS on 2026-08-11:
  - 14 required files present and non-empty
  - all required files strict UTF-8 decodable
  - all required `PROJECT_STATE.md` sections present
  - exactly A01–A14 present and all truthfully `NOT STARTED`
  - relative Markdown links resolve
  - `git diff --check` passes
- Python tests: no Python package/tests yet.
- ROS tests: no project ROS package/tests yet.
- Gazebo: not installed; no launch test.
- MoveIt: not installed; no planning test.
- Acceptance A01–A14: all `NOT STARTED`.

## Current build status

No project build system exists. WSL has ROS 2 Jazzy, CMake, GCC/G++, colcon,
rosdep, and vcstool. Gazebo/MoveIt/control build dependencies are absent.

## Current run status

- `ros2 --help`: verified after sourcing `/opt/ros/jazzy/setup.bash`.
- `rviz2 --help`: verified; GUI rendering not tested.
- Physical-AI-Robot-Lab runtime: not implemented.
- VLA backend: `VLA_BACKEND_UNAVAILABLE`.

## Known bugs

None in project code because no runtime code exists.

## Known limitations

- WSL is allocated 7.4 GiB RAM; the combined GUI stack may be memory-bound.
- Local GPU reports 8151 MiB, below openpi's strict `> 8 GB` statement.
- No CUDA Toolkit, Docker, Gazebo, MoveIt, or ros2_control packages are present.
- `abb_ros2` has no Jazzy branch/tag and tests only Rolling upstream.
- ABB model inertials are estimates, effort limits are fictional, and upstream
  URDF/ros2_control limit definitions conflict.
- Repository is on Windows DrvFS and has no remote backup.

## Unresolved questions

- Whether Gazebo/RViz WSLg 3D rendering is stable on this driver/GPU.
- Whether building under `/mnt/c` is reliable enough or build trees must live
  on WSL ext4.
- Final first-party repository license (do not infer from third-party licenses).
- Exact Whisper implementation/model pin; deferred until Phase 7.
- VLA GPU service location; no paid training or remote resource is authorized.

## Important architecture decisions

- Working classical baseline precedes advanced autonomy and VLA authority.
- Use official Jazzy apt packages and the default Harmonic pairing.
- Reuse pinned ABB geometry/kinematic assets, not the unverified complete driver;
  build a project-owned Jazzy/Harmonic control/MoveIt overlay.
- Language planning emits typed Task/Skill/Goal structures, never joint angles.
- `vision` mode cannot access simulator ground truth.
- ROS and openpi run in isolated environments behind a versioned policy API.
- Simulation and real ABB hardware remain separate implementations and safety
  gates; this project makes no real-robot deployment safety claim.
- Missing VLA produces `VLA_BACKEND_UNAVAILABLE`; fallback success is labelled
  CLASSICAL, never VLA.

## Dependency versions

See `DEPENDENCY_MATRIX.md` and `docs/environment_report.md`. Important installed
baseline versions:

- Ubuntu 24.04.4 LTS WSL2
- ROS 2 Jazzy Desktop apt meta version
  `0.11.0-1noble.20260412.072512`
- Python 3.12.3, GCC/G++ 13.3.0, CMake 3.28.3
- NVIDIA driver 591.91; no CUDA Toolkit

## External repository commits

- PickNik `abb_ros2`: `05cbabe774ef026b1cd36fe6b253b55f55e16340`
- ROS-Industrial `abb` asset lineage: `eefdf01c956a6f2129aacb88d598201feff3cf19`
- Physical Intelligence `openpi`: `15a9616a00943ada6c20a0f158e3adb39df2ccac`

Additional audited source-release pins are in `THIRD_PARTY.md`.

## Important commands

```powershell
# Repository state
git status --short --branch
git log --oneline --decorate -10
git remote -v

# Enter the WSL runtime
wsl -d Ubuntu-24.04
```

```bash
# Every non-interactive ROS command must do this first
source /opt/ros/jazzy/setup.bash

# Recheck selected binary candidates before installation
apt-cache policy ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-ros2-controllers
```

Build/run/test commands will be added only after they actually exist and run.

## Next exact steps

1. Commit `config/ros-jazzy-apt.lock.yaml` after checking its candidate values
   against a second `apt-cache policy` query.
2. Install those exact top-level Jazzy/Harmonic/MoveIt/control packages in WSL,
   record
   exact installed versions, and run upstream `gz_ros2_control` headless smoke
   launch before adding ABB assets.
3. If the upstream control smoke passes, pin/import the ABB model assets in a
   separate atomic commit and add structural Xacro tests before Gazebo spawn.
4. Implement a project-owned fixed-base Jazzy/Harmonic description/control
   overlay with unified joint limits and no gripper yet.
5. Build, spawn headlessly, inspect six joint states/controller status, update
   A01 only if the real launch succeeds, then checkpoint.
