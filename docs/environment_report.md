# Environment Report

- Audit time: 2026-08-11 04:05–04:15 CEST
- Method: read-only host and WSL commands; no software was installed
- Intended robotics runtime: WSL 2 Ubuntu, not native Windows

## Executive result

The host has a usable **WSL 2 + Ubuntu 24.04 + ROS 2 Jazzy Desktop** starting
point. ROS 2 CLI, RViz2 binaries, colcon, rosdep, and vcstool are present.
Gazebo, MoveIt 2, ros2_control/controllers, gz_ros2_control, and Docker are not
installed. Therefore Phase 1 cannot yet be built or run, and no simulation
acceptance status is claimed.

The NVIDIA GPU is exposed inside WSL, but the local machine is not currently a
supported openpi environment: it has 8151 MiB VRAM, no CUDA Toolkit, Ubuntu
24.04 rather than openpi's tested Ubuntu 22.04, and only 7.4 GiB RAM assigned to
WSL. VLA stays explicitly unavailable until an isolated environment passes its
own smoke test.

## Current machine configuration

### Windows host

| Item | Observed value |
|---|---|
| OS | Windows 11 Home China, 25H2, build `26200.8875`, x64 |
| Host | ASUS ROG Strix G18 `G815LM_G815LM` |
| CPU | Intel Core Ultra 9 275HX, 24 cores / 24 logical processors |
| Physical RAM | 15.38 GiB; 2.8–3.2 GiB free during audit |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU, compute capability 12.0 |
| NVIDIA driver | `591.91`, WDDM |
| VRAM | 8151 MiB reported by `nvidia-smi` |
| Driver CUDA compatibility | 13.1; this is not a CUDA Toolkit installation |
| CUDA Toolkit / `nvcc` | `UNAVAILABLE` |
| Disk C: | 923.85 GiB total, 483.94 GiB free |
| Native Python | 3.10.11 |
| Native Git | 2.54.0.windows.1 |
| Native CMake/compiler/Ninja | `UNAVAILABLE` |
| Native Docker/ROS/Gazebo | `UNAVAILABLE` |

Windows WMI reported an unreliable 4 GiB NVIDIA `AdapterRAM` value. The report
uses the 8151 MiB value from `nvidia-smi`.

### WSL 2 / Ubuntu

| Item | Observed value |
|---|---|
| WSL | 2.7.3.0; WSLg 1.0.73 |
| Distribution | Ubuntu 24.04.4 LTS Noble, WSL generation 2 |
| Kernel | `6.6.114.1-microsoft-standard-WSL2` |
| CPU allocation | 24 vCPU |
| Memory allocation | 7.4 GiB RAM + 2.0 GiB swap |
| `.wslconfig` | Not present; default resource policy |
| Python / pip | 3.12.3 / 24.0 |
| Git | 2.43.0 |
| GCC / G++ | 13.3.0 |
| CMake / Make | 3.28.3 / 4.3 |
| Ninja / Clang / Docker / `nvcc` | `UNAVAILABLE` |
| GPU passthrough | `nvidia-smi` and `/usr/lib/wsl/lib/libcuda.so` available |
| WSLg endpoints | X11, Wayland, and Pulse endpoints present |

The WSL root filesystem reports a 1007 GiB logical dynamic-VHDX capacity; real
storage remains limited by the Windows C: free space.

### ROS 2 installation

| Component | Observed state/version |
|---|---|
| ROS distribution | Jazzy |
| `ros-jazzy-desktop` | `0.11.0-1noble.20260412.072512` |
| `ros-jazzy-ros-base` | `0.11.0-1noble.20260412.071950` |
| `ros-jazzy-ros-core` | `0.11.0-1noble.20260412.053408` |
| Installed `ros-jazzy-*` packages | 294 |
| RViz2 | `14.1.20-2noble.20260412.070714` |
| `cv_bridge` | `4.1.0-1noble.20260412.042648` |
| colcon common extensions | `0.3.0-100` |
| rosdep | 0.26.0, sources initialized |
| vcstool | 0.3.0 |

`~/.bashrc` sources `/opt/ros/jazzy/setup.bash`, but non-interactive automation
must source it explicitly. `ros2 --help` and `rviz2 --help` succeeded after
explicit sourcing. A real RViz/OpenGL window was not run during this audit.

### Missing robotics components

All of these were actually queried and were absent:

- Gazebo (`gz` / `gazebo`) and `ros_gz_sim`
- MoveIt (`moveit_core`, `moveit_ros_move_group`, Setup Assistant)
- `controller_manager`, ros2_control, and ros2_controllers
- gz_ros2_control
- Docker

Installed Gazebo vendor utility libraries do not constitute an installed
simulator.

At audit time, the configured ROS apt repository advertised these candidates:

| Package | Candidate observed with `apt-cache policy` |
|---|---|
| `ros-jazzy-ros-gz` | `1.0.22-1noble.20260616.074726` |
| `ros-jazzy-gz-ros2-control` | `1.2.19-1noble.20260615.171757` |
| `ros-jazzy-gz-ros2-control-demos` | `1.2.19-1noble.20260616.073637` |
| `ros-jazzy-moveit` | `2.12.4-1noble.20260617.161956` |
| `ros-jazzy-moveit-task-constructor-core` | `0.1.5-1noble.20260617.153812` |
| `ros-jazzy-ros2-controllers` | `4.40.1-1noble.20260616.074625` |
| `ros-jazzy-controller-manager` | `4.45.2-1noble.20260615.164916` |
| `ros-jazzy-joint-trajectory-controller` | `4.40.1-1noble.20260615.171409` |

These are repository snapshots, not installed versions. The exact candidates
must be captured again immediately before installation because ROS buildfarm
syncs can change them.

## Recommended configuration

### Classical robotics baseline

- Native Ubuntu 24.04 or WSL2 Ubuntu 24.04 on x86_64.
- ROS 2 Jazzy and the default Gazebo Harmonic vendor/binary packages from the
  ROS repository, not a mixed OSRF non-default pairing.
- MoveIt 2, MoveIt Task Constructor, ros2_control, ros2_controllers,
  gz_ros2_control, and ros_gz installed from one coherent Jazzy apt snapshot.
- 32 GiB system RAM recommended for Gazebo + RViz + MoveIt + perception;
  16 GiB is a constrained minimum. Headless profiles should be available.
- Keep active colcon build/install/log trees on WSL ext4 if DrvFS performance or
  symbolic-link semantics become a problem.

### VLA runtime

- Separate openpi environment based on its official tested Ubuntu 22.04 image
  and Python 3.11; do not merge with Jazzy's Python 3.12 environment.
- More than 8 GB VRAM for inference, more than 22.5 GB for LoRA fine-tuning,
  and more than 70 GB for full fine-tuning, per upstream requirements.
- A 24 GB+ GPU is the practical LoRA target; an A100/H100 80 GB class device is
  the full-fine-tuning target.
- Use a local/private policy-server link with network controls. Do not expose an
  unauthenticated inference server publicly.

## Configuration adopted by this project

1. Use the existing WSL2 Ubuntu 24.04 installation for ROS 2 Jazzy classical
   simulation development.
2. Install the missing robotics packages from the configured ROS apt repository
   only after this Phase 0 checkpoint is committed.
3. Start with headless launch/build tests; verify WSLg Gazebo/RViz rendering as a
   separate test rather than assuming GUI readiness.
4. Keep openpi outside the ROS environment. Until a supported policy server and
   compatible GPU/checkpoint pass tests, publish `VLA_BACKEND_UNAVAILABLE` and
   use the labelled CLASSICAL fallback.
5. Do not start paid or full-model training automatically.

## Conflicts and resolutions

| Conflict/risk | Resolution |
|---|---|
| Windows is not the intended ROS target | Run robotics stack in Ubuntu 24.04 WSL2. |
| Gazebo/MoveIt/control stack absent | Install one coherent Jazzy apt snapshot after the Phase 0 checkpoint. |
| ROS is absent in non-interactive PATH | Every script explicitly sources `/opt/ros/jazzy/setup.bash`. |
| Repository is under `/mnt/c` from WSL | Test a small build first; move only build artifacts to WSL ext4 if required. |
| WSL has 7.4 GiB RAM | Prefer headless/serial builds and monitor memory; 32 GiB host upgrade is recommended for full stack. |
| `nvidia-smi` shows CUDA 13.1 but no `nvcc` | Treat toolkit as absent; use pinned ML wheels/container rather than host-toolkit assumptions. |
| RTX 5060 has 8151 MiB, below openpi's strict `> 8 GB` wording | Do not claim local openpi inference support; validate only in isolated smoke test or use a larger remote GPU. |
| openpi supports Ubuntu 22.04, ROS uses Ubuntu 24.04 | Isolate policy server/container and communicate over typed IPC. |
| No Docker installed | VLA container path remains unavailable; `uv`/remote server is a later isolated option. |
| No Git remote | Local commits are recoverable only from this disk; record the limitation and push once a remote is configured. |

## Evidence commands

```powershell
Get-CimInstance Win32_OperatingSystem
Get-CimInstance Win32_ComputerSystem
Get-CimInstance Win32_Processor
Get-CimInstance Win32_VideoController
nvidia-smi
wsl.exe --version
wsl.exe --status
wsl.exe --list --verbose
git -C .\Physical-AI-Robot-Lab status --short --branch
```

```bash
cat /etc/os-release
uname -a
nproc
free -h
nvidia-smi
source /opt/ros/jazzy/setup.bash
ros2 --help
rviz2 --help
ros2 pkg prefix controller_manager
ros2 pkg prefix moveit_core
ros2 pkg prefix ros_gz_sim
apt-cache policy ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-ros2-controllers
```

## Post-audit Phase 1 update

After the read-only audit and Phase 0 checkpoint, the selected exact top-level
packages were installed on 2026-08-11. The audit-time “missing” table above
remains the historical pre-install observation.

- Gazebo Harmonic gz-sim 8.11.0: installed and headless execution verified.
- ros_gz / gz_ros2_control / ros2_control / ros2_controllers: installed; an
  upstream trajectory action and resulting joint state were verified.
- MoveIt 2.12.4 and MoveIt Task Constructor 0.1.5: installed and discoverable;
  no project planning task has run yet.
- Docker and CUDA Toolkit remain unavailable.

Exact versions and executable evidence are in
`config/ros-jazzy-installed-packages.lock.tsv` and
`docs/phase1_dependency_smoke.md`.
