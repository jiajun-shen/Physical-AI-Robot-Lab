# Phase 1 ABB Headless Gazebo Control Runtime Evidence

- Tested commit: `1228c13fadb8c04d348a0bfa68e2fa1d0a7ce6ee`
- Clean-run completion: 2026-08-11 15:37:51 +0200
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Harmonic
- Scope: P1.3b project-owned ABB IRB1200-5/0.9 headless spawn and bounded
  ros2_control trajectory
- Clean gate output: `/var/tmp/p13b-clean-1228c13`
- Acceptance boundary: no GUI rendering, MoveIt planning, collision-free claim,
  gripper, real ABB endpoint, or out-of-range clamp probe

## Tests-first boundary

The P1.3b source-contract tests were written before implementation. Initial
control and Gazebo contract failures were the expected RED state; they were not
treated as runtime evidence. The implementation is split into three scoped
packages:

- `abb_lab_description` retains the immutable safety parameter file as the first
  Gazebo plugin parameter and accepts an optional runtime controller file as a
  second parameter;
- `abb_lab_control` owns simulation controller policy and the reviewed bounded
  smoke goal;
- `abb_lab_gazebo` owns the minimal world, headless launch, spawn sequence, and
  message/evidence verifier.

The repository-level runtime harness remains outside those ROS packages so
package tests do not depend on a file above their package roots.

## Runtime design and safety boundary

The world contains an inline ground plane, light, and the Gazebo Physics,
UserCommands, and SceneBroadcaster systems. It starts a fresh partition and
creates the model from the installed `robot_description` as exactly
`abb_irb1200`; renaming is disabled. The launch advances from entity creation
to broadcaster spawner to trajectory-controller spawner only on a zero exit
status.

The controller contract is intentionally narrow:

- controller-manager update rate: 100 Hz;
- active controllers: `joint_state_broadcaster` and
  `joint_trajectory_controller` only;
- command interfaces: six claimed position interfaces only;
- state interfaces: six each of position, velocity, and effort;
- partial joint goals and open-loop control disabled;
- finite two-second controller goal-time allowance;
- explicit six-axis trajectory tolerance 0.1 rad, goal tolerance 0.03 rad,
  and stopped-velocity tolerance 0.05 rad/s.

The smoke waits for three strictly time-increasing, exact-six-joint messages
near neutral. Its targets are at most 0.06 rad from neutral, at least 0.05 rad
inside every canonical position limit, and are sent only through
`FollowJointTrajectory` for three seconds. The harness never writes a Gazebo
joint command.

Runtime evidence remains deliberately separated:

1. `LIMITER_RUNTIME_INITIALIZED=PASS` requires
   `controller_manager.enforce_command_limits=true`, all six limiter-creation
   messages, and the enabled-enforcement log without a disabled log.
2. `BOUNDED_TRAJECTORY=PASS` requires goal acceptance, `SUCCEEDED`, controller
   `error_code: 0`, at least ten valid feedback frames inside canonical limits,
   and five strictly time-increasing final target-state messages inside 0.03 rad.
3. `CLAMP_PROBE=NOT_RUN` and `CLAMP_BEHAVIOR_VERIFIED=false` remain explicit.
   A bounded in-range motion and initialized limiters do not prove clamping.

The gate also requires one active `GazeboSimSystem`, the exact two controllers,
the controller-manager and robot-state-publisher nodes, one persisting
`abb_irb1200` entity, resolvable resources, and a fixed model pose within
1e-5 m/rad before and after motion.

## Isolation and recovery contract

The gate builds, installs, tests, and logs only in a new or explicitly empty
Linux filesystem directory. Repository descendants, `/`, nonempty paths, and
DrvFS output roots are rejected. Each run locks a ROS domain, restricts
discovery to localhost, uses a unique Gazebo partition, and places ROS home/log
data under the output root.

An outer 180-second watchdog covers synchronous harness work, including colcon
children. The launched runtime uses its own recorded process group and cleanup
escalates `INT -> TERM -> KILL`. Cleanup targets only that group, stops only the
selected-domain ROS daemon, rejects residual processes, and writes PASS labels
only after cleanup succeeds. A SHA-256 manifest over every tracked or
nonignored untracked source file is captured before and after the gate.

## Executed clean runtime gate

Command:

```bash
bash scripts/run_and_verify_irb1200_headless.sh /var/tmp/p13b-clean-1228c13
```

The gate ran from a clean checkout at the tested commit. It built and tested all
three project packages, launched Gazebo headlessly, inspected hardware and
controllers, executed the bounded goal, observed the fixed base and persistent
entity/controllers/nodes, and proved cleanup.

- `colcon test-result`: 41 records, 0 errors, 0 failures, 0 skipped:
  36 unique pytest cases (23 description, 6 control, 7 Gazebo) and five CTest
  wrappers. One repository-harness unittest also passed separately.
- Source manifests before and after the run match:
  `3076651510b8eff14bd586de6f3eb4b7f550461e9db6bab4f7c86aba8e457179`.
- The runtime selected `ROS_DOMAIN_ID=100` and
  `GZ_PARTITION=physical_ai_p13b_100_436`.
- It observed 59 action-feedback frames. Maximum absolute observed velocity was
  `0.0201531993902` rad/s. Fixed-base translation and rotation drift were both
  `0`.

Observed final result fields:

```text
PROJECT_ABB_GAZEBO_SPAWN=PASS
HARDWARE_RESOURCES=PASS
CONTROLLERS_ACTIVE=PASS
LIMITER_RUNTIME_INITIALIZED=PASS
BOUNDED_TRAJECTORY=PASS
CLAMP_PROBE=NOT_RUN
CLAMP_BEHAVIOR_VERIFIED=false
GUI_RENDERING=NOT_RUN
A02=NOT_STARTED
REAL_ROBOT_READY=false
ABB_HEADLESS_RUNTIME=PASS
CLEANUP_VERIFIED=PASS
TESTED_COMMIT=1228c13fadb8c04d348a0bfa68e2fa1d0a7ce6ee
SOURCE_WORKTREE_CLEAN=true
```

This clean, commit-addressable run is A01 evidence. It also establishes the
current P1.3b ros2_control control-chain evidence, but it does not satisfy A02
or any later acceptance item.

## Warnings and non-claims

The exact clean-run launch log records four warning classes: a transient wait
for the resource manager to initialize hardware, no executor during simulated
component initialization, no read/write statistics for that component, and a
100 Hz controller period slower than the 1 ms physics period. The last is an
intentional integer-ratio configuration, not a failed controller cycle. The
hardware subsequently reaches active state and the full observation/action
gate passes; none of these warnings is hidden.

This evidence does not establish collision checking, Home -> Pose -> Home,
MoveIt readiness, GUI rendering, physical effort fidelity, derivative limits,
out-of-range clamping, or real-robot safety. A02-A14 remain `NOT STARTED`.
