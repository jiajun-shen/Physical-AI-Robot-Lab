# Dependency Matrix

Audit date: 2026-08-11. `OBSERVED CANDIDATE` means the target WSL apt
repository advertised that binary version; it is not installed. The candidate
must be captured again immediately before installation.

## Selected runtime baseline

| Layer | Selection | Lock/evidence | Local state | Status |
|---|---|---|---|---|
| Host | Windows 11 25H2 + WSL 2.7.3 | build `26200.8875` | Installed | HOST ONLY |
| Robotics OS | Ubuntu 24.04.4 Noble WSL2 | `/etc/os-release` | Installed | ADOPTED |
| ROS | ROS 2 Jazzy Desktop | apt `0.11.0-1noble.20260412.072512` | Installed, CLI verified | ADOPTED |
| Simulator bridge | `ros-jazzy-ros-gz` / Harmonic vendor packages | apt `1.0.22-1noble.20260616.074726` | Installed; gz-sim 8.11.0 headless smoke | VERIFIED DEPENDENCY |
| Motion planning | `ros-jazzy-moveit` | apt `2.12.4-1noble.20260617.161956` | Installed/package prefix verified; no project plan run | INSTALLED |
| Task composition | MoveIt Task Constructor | apt core `0.1.5-1noble.20260617.153812` | Installed/package prefix verified | INSTALLED |
| Simulation control | `ros-jazzy-gz-ros2-control` | apt `1.2.19-1noble.20260615.171757` | Installed; controller/action/joint-state smoke passed | VERIFIED DEPENDENCY |
| Controllers | ros2_controllers / joint trajectory controller | apt meta `4.40.1-1noble.20260616.074625` | Installed; trajectory action passed | VERIFIED DEPENDENCY |
| Controller manager | ros2_control controller manager | apt `4.45.2-1noble.20260615.175135` | Installed; two active controllers observed | VERIFIED DEPENDENCY |
| ABB model assets | PickNik ROS 2 port of ROS-Industrial IRB1200 | `abb_ros2@05cbabe774ef026b1cd36fe6b253b55f55e16340`, 19-file SHA-256 lock | Vendored subset; `abb_lab_description` builds/expands/tests | VERIFIED ASSET INPUT |
| ABB asset lineage | ROS-Industrial `abb_irb1200_support` | `abb` tag 1.5.0, commit `eefdf01c956a6f2129aacb88d598201feff3cf19` | Source verified | REFERENCE |
| VLA code | Physical Intelligence openpi | `main@15a9616a00943ada6c20a0f158e3adb39df2ccac` | Not installed | RESEARCH PIN |
| VLA runtime | Isolated Ubuntu 22.04/Python 3.11 policy server | upstream Docker/uv environment | Unavailable locally | PLANNED |
| Speech | Multilingual Whisper-compatible backend | Version/model not selected | Not installed | UNSELECTED |
| Vision | OpenCV RGB-D baseline | ROS `cv_bridge` 4.1.0 installed; backend pin pending | Partial prerequisites only | UNSELECTED |
| C++ | GCC/G++ 13.3, minimum C++17 | Ubuntu packages | Installed | ADOPTED |
| ROS Python | Python 3.12.3 | Ubuntu/Jazzy system Python | Installed | ADOPTED |
| VLA Python | Python 3.11 isolated | openpi tested environment | Not installed | REQUIRED SEPARATION |

## Verified source-release references

These references explain compatibility or support future source builds. The
first baseline still prefers the target-machine Jazzy apt binaries above.

| Component | Repository/reference | Commit | License |
|---|---|---|---|
| MoveIt 2 | `moveit/moveit2`, tag `2.12.4` | `1ade0e9dcf50dbbbc3a984b995c786cf12736235` | BSD-3-Clause |
| MoveIt Task Constructor | `moveit/moveit_task_constructor`, tag `ros2-0.1.5` | `56c6a2a40ebff8a4d1644d9e6fac819c3cd3542f` | BSD-3-Clause |
| ros2_control | `ros-controls/ros2_control`, release 4.47.0 | `df23a4b7a21cb8ea257c80ecf98e85571b50ba01` | Apache-2.0 |
| ros2_controllers | `ros-controls/ros2_controllers`, tag 4.41.0 | `b69445a5c72bc1b0dfc0658ce992748e65e94cf6` | Apache-2.0 |
| gz_ros2_control | `ros-controls/gz_ros2_control`, tag 1.2.19 | `3632af2998895cd18c03717bd40c7acd547db24d` | Apache-2.0 |
| ros_gz | `gazebosim/ros_gz`, tag 1.0.23 | `ec3a555b540ac492882d587a09752eb2eeeee3cd` | Apache-2.0 |

The ROS buildfarm can expose a newer source release before a binary sync. A
source tag is never substituted for the actually installed apt version in test
records.

## Compatibility constraints

- Jazzy + Harmonic is the official recommended pairing. Do not add Gazebo
  Classic or mix a non-default OSRF Gazebo repository into the baseline.
- `abb_ros2` has no Jazzy branch/tag and its CI tests Rolling only. Reuse pinned
  IRB1200 assets and build a Jazzy/Harmonic overlay; do not claim full driver
  compatibility.
- ABB description inertials are estimates, effort values are fictional, and
  ros2_control/URDF limits conflict. Every limit and collision property must be
  reconciled and tested before simulation use.
- ROS apt packages use Ubuntu's Python 3.12. openpi uses Python 3.11 plus pinned
  CUDA/JAX/Torch dependencies and must run outside the ROS environment.
- openpi officially requires more than 8 GB VRAM for inference; the local 8151
  MiB GPU is below the strict threshold and no local availability is claimed.
- No ABB-tuned π0.5 checkpoint exists in the audited upstream. Any unadapted
  checkpoint must show `EXPERIMENTAL / NOT ABB-TUNED`.

## Locking procedure

1. Before apt installation, save `apt-cache policy` results for every selected
   package in `config/ros-jazzy-apt.lock.yaml`.
2. After installation, save exact `dpkg-query -W` versions in
   `config/ros-jazzy-installed-packages.lock.tsv` and update this file.
3. External Git sources use full 40-character commits in a `.repos` manifest or
   a documented vendored snapshot; moving branches are prohibited as locks.
4. Model checkpoints and datasets live in external artifact storage and have a
   manifest/checksum/license record, not ordinary Git objects.
5. ROS build, VLA service, and real-hardware dependencies remain separately
   locked.
