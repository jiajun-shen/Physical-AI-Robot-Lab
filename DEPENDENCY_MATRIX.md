# Dependency Matrix

This file records only verified selections. Provisional target names are not
version claims and must be replaced with exact package versions or Git commits
during Phase 0.

| Layer | Target | Current host | Pin/status | Verification required |
|---|---|---|---|---|
| OS | Ubuntu 24.04 LTS | Audit pending | PROVISIONAL | Native vs WSL2/container runtime |
| ROS | ROS 2 Jazzy | Audit pending | PROVISIONAL | Official support and installed packages |
| Simulator | Gazebo Harmonic | Audit pending | PROVISIONAL | Jazzy bridge/control compatibility |
| Motion | MoveIt 2 for Jazzy | Audit pending | PROVISIONAL | Binary/source and MTC availability |
| Control | ros2_control + gz_ros2_control | Audit pending | PROVISIONAL | Matching distro packages/interfaces |
| ABB model/control | `PickNikRobotics/abb_ros2` and upstream description dependencies | Not cloned | UNVERIFIED | IRB1200-5/0.9 content, license, branch, commit |
| VLA | `Physical-Intelligence/openpi`, π0.5 family | Not cloned | UNVERIFIED | OS/Python/CUDA/GPU, checkpoint terms, commit |
| Speech | Multilingual Whisper backend | Not installed | UNSELECTED | Runtime, model size, license, CPU/GPU path |
| Vision | OpenCV RGB-D baseline | Audit pending | UNSELECTED | ROS image/depth transport and OpenCV version |
| Python | Python 3 | Audit pending | UNPINNED | ROS-compatible version and VLA-isolated version |
| C++ | ROS-compatible C++17+ compiler | Audit pending | UNPINNED | Toolchain version |

## Isolation policy

- ROS workspace dependencies follow the ROS distribution and rosdep/apt lock.
- VLA dependencies use a separate container or `uv` environment and communicate
  through a versioned policy API.
- External repositories record URL, branch/tag, commit, and license in
  `THIRD_PARTY.md` before integration.
- Model checkpoints and datasets are external artifacts, never ordinary Git
  objects.

