# Phase 2.1 ABB MoveIt Configuration Evidence

- Tested commit: `281c36bafd543d47752deb91d8743b09c3772644`
- Clean static-gate completion: 2026-08-11 16:07:28 +0200
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy, MoveIt 2.12.4
- Output root: `/var/tmp/p21-moveit-clean-281c36b`
- Scope: P2.1 declarative MoveIt configuration only

## Delivered configuration contract

`abb_lab_moveit_config` is a first-party, arm-only package. It contains no
Gazebo launch, no controller-manager process, no fake hardware, no gripper, and
no execution claim.

- The semantic model is `abb_irb1200_5_90_gz_control`, matching the project
  Gazebo-control URDF.
- Group `manipulator` is exactly the `base_link -> tool0` chain with
  `joint_1` through `joint_6`; `home` is the valid all-zero reference state.
- `joint_limits.yaml` exactly mirrors the project canonical position/velocity
  limits. It deliberately omits acceleration, deceleration, and jerk values.
- KDL kinematics and a single collision-checking OMPL RRTConnect pipeline are
  configured.
- The MoveIt Simple Controller Manager maps only to the already owned
  `/joint_trajectory_controller/follow_joint_trajectory` action and sets
  `moveit_manage_controllers: false` so P2.1 cannot create a second controller
  manager.

## Clean static gate

The following clean-worktree command was run:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep check --from-paths src --ignore-src --rosdistro jazzy
colcon build --packages-select abb_lab_description abb_lab_control abb_lab_moveit_config
colcon test --packages-select abb_lab_description abb_lab_control abb_lab_moveit_config
colcon test-result --all --verbose
```

Results:

- `rosdep check`: all system dependencies satisfied.
- Build: 3/3 packages passed.
- Test result: 45 records, 0 errors, 0 failures, 0 skipped.
  - Existing description/control contracts: 29 unique pytest cases plus four
    CTest wrappers.
  - `abb_lab_moveit_config`: 11 installed-package source-contract tests plus
    one CTest wrapper.
- The builder imported the installed configuration, expanded the installed
  runtime Xacro, and verified that the URDF and SRDF names, six-joint chain,
  canonical limits, OMPL collision adapters, and controller action mapping
  agree.
- The repository worktree was clean after the gate; generated output remained
  under the ext4 output root, not in the repository.

## Non-claims and P2.2 gate

P2.1 is not a MoveIt planning or execution run. It does not change A02, which
remains `NOT STARTED`.

P2.2 must run the project headless Gazebo/runtime with `move_group` and prove
all of the following on a clean tested commit:

1. a valid joint target and valid `tool0` pose target each plan and execute;
2. HOME -> specified Pose -> HOME is observed through the real controller and
   `/joint_states`;
3. an intentionally colliding scene/goal is rejected by planning-scene
   collision checking; and
4. process cleanup is verified without starting a duplicate controller manager.
