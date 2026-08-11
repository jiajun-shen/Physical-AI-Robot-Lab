# Upstream Compatibility Report

- Verification date: 2026-08-11
- Policy: official documentation, official repositories, source manifests, and
  target-machine apt metadata only
- Result: the core Jazzy/Harmonic stack is supported; ABB assets are usable but
  the complete `abb_ros2` driver is **not verified for Jazzy**; local openpi is
  **not a supported configuration**

## ROS 2 Jazzy, Ubuntu 24.04, and Gazebo Harmonic

The official Gazebo compatibility table marks Ubuntu 24.04 + ROS 2 Jazzy +
Gazebo Harmonic as the recommended combination. Jazzy obtains Harmonic through
ROS Gazebo vendor packages. The project will use the ROS repository pairing and
avoid mixing in a non-default Gazebo repository.

Primary sources:

- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Official ROS/Gazebo version pairing](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Jazzy ros_gz package documentation](https://docs.ros.org/en/jazzy/p/ros_gz/)
- [Jazzy gz_ros2_control documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

`gz_ros2_control` is released for Jazzy and explicitly uses Harmonic through
Gazebo vendor packages. Its observed apt candidate is recorded in
`docs/environment_report.md`.

## MoveIt 2 and MoveIt Task Constructor

MoveIt recommends Jazzy on Ubuntu 24.04 as the stable starting point. MoveIt
2.12.4 is available in the target apt repository. MoveIt Task Constructor 0.1.5
is also released for Jazzy; use that released version rather than following the
moving `ros2` branch.

Primary sources:

- [MoveIt Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
- [MoveIt Task Constructor pick/place tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [moveit2 Jazzy source branch](https://github.com/moveit/moveit2/tree/jazzy)
- [MoveIt Task Constructor release tag](https://github.com/moveit/moveit_task_constructor/tree/ros2-0.1.5)

Verified source references:

| Repository | Reference | Commit | License |
|---|---|---|---|
| `moveit/moveit2` | `jazzy` audit HEAD | `4d841063574c31a21f69ae12a39fddb77a7eb984` | BSD-3-Clause |
| `moveit/moveit2` | release `2.12.4` | `1ade0e9dcf50dbbbc3a984b995c786cf12736235` | BSD-3-Clause |
| `moveit/moveit_task_constructor` | tag `ros2-0.1.5` | `56c6a2a40ebff8a4d1644d9e6fac819c3cd3542f` | BSD-3-Clause |

Binary apt versions, not moving branch heads, are the Phase 1/2 baseline.

## ros2_control and ROS/Gazebo bridge references

| Repository | Reference | Commit | License | Note |
|---|---|---|---|---|
| `ros-controls/ros2_control` | Jazzy source release 4.47.0 | `df23a4b7a21cb8ea257c80ecf98e85571b50ba01` | Apache-2.0 | Target apt candidate can lag buildfarm sync. |
| `ros-controls/gz_ros2_control` | tag 1.2.19 | `3632af2998895cd18c03717bd40c7acd547db24d` | Apache-2.0 | Released Jazzy/Harmonic integration. |
| `gazebosim/ros_gz` | Jazzy tag 1.0.23 | `ec3a555b540ac492882d587a09752eb2eeeee3cd` | Apache-2.0 | Actual target apt candidate was 1.0.22 at audit. |

The target machine's `apt-cache policy` is authoritative for the binary lock.
Do not label a newer buildfarm/source version as installed before apt sync.

## ABB IRB1200-5/0.9 source audit

### Verified assets

The ROS 2 port in
[`PickNikRobotics/abb_ros2`](https://github.com/PickNikRobotics/abb_ros2)
contains `abb_irb1200_support` and `abb_irb1200_5_90_moveit_config` at:

- branch: `rolling`
- commit: [`05cbabe774ef026b1cd36fe6b253b55f55e16340`](https://github.com/PickNikRobotics/abb_ros2/commit/05cbabe774ef026b1cd36fe6b253b55f55e16340)
- repository license: Apache-2.0
- support package/meshes: Apache-2.0
- MoveIt config package: BSD-style license

The support package contains Xacro, DAE visual meshes, STL collision meshes,
and exactly six revolute joints named `joint_1` through `joint_6`. Verified
frames include `base_link`, ROS-Industrial `base`, `flange`, and `tool0`.

| Joint | Lower rad | Upper rad | Velocity rad/s |
|---|---:|---:|---:|
| `joint_1` | -2.967 | 2.967 | 5.027 |
| `joint_2` | -1.745 | 2.269 | 4.189 |
| `joint_3` | -3.491 | 1.222 | 5.236 |
| `joint_4` | -4.712 | 4.712 | 6.981 |
| `joint_5` | -2.269 | 2.269 | 7.069 |
| `joint_6` | -6.283 | 6.283 | 10.472 |

The asset lineage is ROS-Industrial's `abb_irb1200_support`; a stable reference
is [`ros-industrial/abb` 1.5.0](https://github.com/ros-industrial/abb/tree/1.5.0)
at commit `eefdf01c956a6f2129aacb88d598201feff3cf19`, Apache-2.0. The
PickNik package metadata still points to the older `abb_experimental` location,
which is treated as stale metadata rather than the current authority.

### Compatibility limitations found in source

- `abb_ros2` has `foxy`, `humble`, `iron`, and `rolling` branches but no
  `jazzy` branch and no release tags.
- Its CI workflow tests only `ROS_DISTRO=rolling`; the project cannot claim the
  full driver stack is Jazzy-compatible.
- The included `abb.repos` tracks moving `master`/`rolling` branches instead of
  commits and is therefore not an acceptable project lock file.
- The description's ros2_control Xacro selects either
  `mock_components/GenericSystem` or ABB EGM/RWS hardware. It does not contain a
  gz_ros2_control/Harmonic plugin.
- The support manifest says link inertials were estimated from mesh volume at
  constant density and its effort values are fictional Gazebo-oriented values.
- ros2_control command-interface limits conflict with several URDF joint
  limits; they must not be adopted without reconciliation.
- There is no two-finger gripper/end-effector configuration.
- The existing SRDF/MoveIt configuration predates this Jazzy laboratory and
  must be regenerated or independently validated.

### Integration decision

Phase 1 will pin and reuse the lawful geometry/kinematics assets, preserving
their notices. It will implement a project-owned Jazzy/Harmonic composition and
gz_ros2_control configuration, validate all six joints/frames/limits, and keep
ABB EGM/RWS real-hardware code outside the simulation baseline. The full
`abb_ros2` driver is not a Phase 1 dependency claim.

## Physical Intelligence openpi / π0.5

Verified upstream reference:

- repository: [Physical-Intelligence/openpi](https://github.com/Physical-Intelligence/openpi)
- branch: `main`
- commit: [`15a9616a00943ada6c20a0f158e3adb39df2ccac`](https://github.com/Physical-Intelligence/openpi/commit/15a9616a00943ada6c20a0f158e3adb39df2ccac)
- tags/releases: none found
- code license: Apache-2.0; repository also contains model-specific Gemma terms,
  so checkpoint terms require a separate audit

At that commit, `pyproject.toml` requires Python >=3.11 and pins dependencies
including JAX CUDA 12, Torch 2.7.1, Transformers 4.53.2, and NumPy <2. The
official Docker environment uses Ubuntu 22.04/Python 3.11; the README states
that other operating systems are not supported.

Official minimum GPU memory statements are:

- inference: more than 8 GB
- LoRA fine-tuning: more than 22.5 GB
- full fine-tuning: more than 70 GB

The official repository supplies WebSocket remote inference, which supports the
project's ROS/VLA environment split. Its policy server listens on network
interfaces without supplying application-level authentication/TLS; deployment
must be localhost/private-network/firewall restricted.

π0.5 checkpoints are provided for platforms/tasks such as DROID and LIBERO, not
ABB IRB1200. Any such checkpoint is labelled
`EXPERIMENTAL / NOT ABB-TUNED`. It may only emit actions through an explicit ABB
observation/action adapter, normalization, limit checks, collision/workspace
safety, and watchdog. It cannot be represented as reliable ABB control.

Primary sources:

- [openpi README at the audited commit](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/README.md)
- [openpi Python/dependency manifest](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/pyproject.toml)
- [openpi remote inference](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/docs/remote_inference.md)

## Phase 0 compatibility verdicts

| Item | Verdict |
|---|---|
| Ubuntu 24.04 + ROS 2 Jazzy | SUPPORTED; already present in WSL2 |
| Jazzy + Gazebo Harmonic | SUPPORTED/RECOMMENDED; not installed locally |
| Jazzy + gz_ros2_control | RELEASED; not installed locally |
| Jazzy + MoveIt 2/MTC | RELEASED; not installed locally |
| ABB IRB1200 visual/collision/kinematic assets | VERIFIED AND REUSABLE WITH LICENSE/PIN |
| Complete PickNik ABB driver on Jazzy | UNVERIFIED; upstream CI is Rolling-only |
| Existing ABB Gazebo Harmonic integration | NOT PRESENT |
| Existing ABB two-finger gripper | NOT PRESENT |
| Local openpi support | `VLA_BACKEND_UNAVAILABLE` pending isolated smoke test |
| ABB-tuned π0.5 checkpoint | NOT AVAILABLE |
