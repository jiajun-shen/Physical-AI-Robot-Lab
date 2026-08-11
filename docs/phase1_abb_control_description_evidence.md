# Phase 1 ABB Static Control-Description Evidence

- Date: 2026-08-11
- Runtime used for build checks: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy
- Scope: P1.3a fixed-base ABB IRB1200-5/0.9 Gazebo Harmonic control description
- Truth boundary: no project Gazebo launch, spawn, controller activation,
  trajectory, joint-state observation, runtime clamp, MoveIt plan, or A01 claim

## Why this is a separate contract

P1.2 established an exact audited derivative of the pinned ABB geometry. P1.3a
does not edit that derivative or its geometry-only entry point. Instead it adds
a separate simulation composition and a simulation-only limit table. Existing
tests still require the P1.2 macro to differ from the pinned source only by its
notice and package resource paths.

This separation prevents a changing control policy from silently becoming part
of the source-asset proof. It also makes three evidence levels explicit:

1. Static values and plugin declarations match.
2. A runtime actually initializes those declarations.
3. An out-of-range command is observed to be rejected or clamped.

Only level 1 is complete here.

## Audited local API contract

The target installation contains `controller_manager` 4.45.2 and
`gz_ros2_control` 1.2.19. The installed controller-manager parameter
description says `enforce_command_limits` defaults to false, is read-only, and
must be set at startup. The P1.3a parameter file therefore contains only:

```yaml
controller_manager:
  ros__parameters:
    enforce_command_limits: true
```

The installed gz_ros2_control examples and plugin manifests establish two
different names that are both required:

```text
hardware/plugin: gz_ros2_control/GazeboSimSystem
Gazebo library:  gz_ros2_control-system
Gazebo class:    gz_ros2_control::GazeboSimROS2ControlPlugin
```

The expanded verifier rejects controller-manager versions below 4.45.2,
requires the hardware plugin in the installed pluginlib manifest, and requires
gz_ros2_control 1.2.19 or newer. It also asks `gz plugin` to inspect the system
library and requires the expected class registration. This loads plugin
metadata for inspection; it does not launch Gazebo or configure the system.

## Control-description contract

The control composition has:

- one `world` link and one fixed `world_to_base` joint;
- the unchanged six-axis IRB1200 geometry;
- exactly one `ros2_control` system named `GazeboSimSystem`;
- exactly `joint_1` through `joint_6` as controlled joints;
- one position command interface per joint with finite min/max values;
- position, velocity, and effort state interfaces per joint;
- no invented `initial_value` and no disabled limit marker;
- one Harmonic plugin declaration, the safety parameter file, and
  `hold_joints=true`.
- a standard Gazebo model-path export resolving the installed share parent, so
  `model://abb_lab_description/...` mesh URIs have a discoverable resource root.

The canonical simulation table preserves the pinned source position and
velocity values. Tests compare it independently with both the immutable asset
reference and the pinned source Xacro, then compare the expanded ordinary URDF
limits and position command-interface bounds with the same values. The upstream
effort value remains explicitly fictional and is not used as a command bound.
Acceleration, deceleration, and jerk are marked undefined and unverified rather
than invented.

There is deliberately no joint-state broadcaster, trajectory controller,
controller update rate, world file, launch file, spawn command, MoveIt limit
table, gripper, ABB EGM/RWS driver, or real-hardware interface in this slice.

An independent semantic audit also fed the expanded URDF to the installed
`hardware_interface::parse_control_resources_from_urdf` implementation. It
observed one system, six joints, six enabled hard-limit maps, and no soft-limit
maps. Effective position/velocity values matched the canonical table on every
axis. The fictional effort value was parsed from ordinary URDF but had no
effort command interface; acceleration and jerk remained disabled. `gz sdf -k`
reported the converted description valid. These checks do not substitute for
a running controller manager or simulator.

## Tests-first evidence

The first source-contract run occurred before the P1.3a files existed. Seven
test methods produced nine failures for the missing safety YAML, canonical
limits, control macro, and full composition. This was the expected red state.

After implementation, the first clean build successfully compiled/installed
the package and parsed both URDFs, then exposed a verifier bug: Xacro resolves a
`$(find ...)` expression to the installed absolute parameter path, while the
verifier still expected the source literal. The verifier was corrected to
require the actual installed safety file. A new unique clean run then passed.

## Executed regression

Command:

```bash
bash scripts/build_and_test_abb_description.sh
```

Observed results:

```text
Python source cases: 23 passed
colcon build: 1 package finished
geometry check_urdf: Successfully Parsed XML
ABB_GEOMETRY_URDF=PASS
control check_urdf: Successfully Parsed XML
ABB_CONTROL_DESCRIPTION=PASS
LINK_COUNT=11
KINEMATIC_JOINT_COUNT=10
CONTROL_JOINT_COUNT=6
CANONICAL_LIMIT_JOINT_COUNT=6
CONTROLLER_MANAGER_VERSION=4.45.2
GZ_ROS2_CONTROL_VERSION=1.2.19
GAZEBO_MODEL_RESOURCE_EXPORT=PASS
GAZEBO_SYSTEM_PLUGIN_INSPECTION=PASS
COMMAND_LIMIT_ENFORCEMENT_CONFIGURED=true
RUNTIME_ENFORCEMENT_VERIFIED=false
GAZEBO_RUNTIME=NOT_RUN
CTest targets: 3/3 passed
pytest cases: 23/23 passed
colcon test-result: 26 records, 0 errors, 0 failures, 0 skipped
ABB_DESCRIPTION_BUILD=PASS
OUTPUT_ROOT=/tmp/physical-ai-robot-lab/abb-description.zNAgBg
```

The 26 colcon records comprise 23 pytest cases plus three CTest wrapper records;
they are not 26 independent behavioral tests. Build, install, logs, caches, and
generated URDFs were directed to the unique ext4 output tree rather than the
repository.

The final independent audit copied only `abb_lab_description` to
`/var/tmp/p13a-gz-path-audit.wwtjBKcD/ws/src/abb_lab_description` on WSL ext4.
The source and copy had 41 files and 7,537,801 bytes with every SHA-256
matching. Independent build, all 3 CTest targets, all 23 pytest cases, both
Xacro expansions, both `check_urdf` calls, and both expanded verifiers passed.

The official `ros_gz_sim` path collector found the installed package export
exactly once, resolving it to the overlay's `share` directory. With that value
as `GZ_SIM_RESOURCE_PATH`, `gz sdf -k` and `gz sdf -p` both succeeded for the
geometry and control descriptions with empty conversion stderr. Each set of 14
mesh URIs became `model://abb_lab_description/...` and all 14 resolved to real
installed files through the exported root. Explicit plugin inspection found
the expected class and its four Gazebo Sim v8 system interfaces.

In that workspace the geometry/control URDF SHA-256 values were respectively
`c249965bb4e3aa4f310f816cda45a118ef1d1308d1d6dc5f871365dcdb63f971` and
`c9337ddea1165fab4c60061198d563267a9a2f8b26098d99e3d49db9e0715be1`;
the converted SDF values were
`5b304097212f4a15ef122688346e95e27773728be2c7f428158313fd1aeb0003` and
`cc9e6e6e69660ba410aa6dff7e8167bc270c6a442516d931faa8bb410d528983`.
Package source hashes were unchanged after the gate, and no generated artifact
or cache appeared in the repository.

## Acceptance boundary

This evidence establishes `ABB_STATIC_CONTROL_DESCRIPTION=PASS`. It does not
establish controller availability, command-limit runtime enforcement, stable
simulation, or robot motion. P1.3b must add the minimal world, controller
configuration, headless launch, resource/controller inspection, and a bounded
observed trajectory. That in-range trajectory proves the control chain only;
runtime limiter initialization and an out-of-range clamp probe need their own
evidence. At this historical static-description checkpoint A01 and A02-A14
were `NOT STARTED`; later runtime evidence is tracked separately in the
acceptance ledger.
