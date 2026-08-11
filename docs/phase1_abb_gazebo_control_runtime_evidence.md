# Phase 1 ABB Headless Gazebo Control Runtime Evidence

- Date: 2026-08-11
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Harmonic
- Scope: P1.3b project-owned ABB IRB1200-5/0.9 headless spawn and bounded
  ros2_control trajectory
- Development gate: `/var/tmp/p13b-dev-run-12`
- Acceptance boundary: no GUI rendering, MoveIt planning, collision-free claim,
  gripper, real ABB endpoint, or out-of-range clamp probe

## Tests-first boundary

The first P1.3b source-contract run preceded the implementation. The control
suite reported seven failures across five initial methods because the package
and both YAML contracts did not exist. The Gazebo suite reported eleven
failures plus one missing-argument error because its package, world, launch,
runtime verifier, root harness, and the description override did not exist.
Those failures were the expected RED state; they were not treated as runtime
evidence.

The implementation was then split into three responsibilities:

- `abb_lab_description` retains the immutable safety parameter file as the
  first plugin parameter and accepts an optional runtime controller file as a
  second parameter;
- `abb_lab_control` owns only the simulation controller policy and reviewed
  bounded smoke goal;
- `abb_lab_gazebo` owns the minimal world, headless launch, spawn sequence, and
  message/evidence verifier.

The repository-level harness remains outside those ROS packages so package
tests do not depend on a file above the package source root.

## Runtime design and safety boundary

The world contains only an inline ground plane, light, and the Gazebo Physics,
UserCommands, and SceneBroadcaster systems. The ABB model is created from the
installed `robot_description` as exactly `abb_irb1200` with renaming disabled.
Process-exit handlers allow the next stage to start only when entity creation,
then the joint-state broadcaster spawner, return zero. A failed create or
spawner emits a launch shutdown.

The controller contract is intentionally narrow:

- controller-manager update rate: 100 Hz;
- active controllers: only `joint_state_broadcaster` and
  `joint_trajectory_controller`;
- command interfaces: only six position interfaces, all claimed;
- state interfaces: exactly six each of position, velocity, and effort;
- partial joint goals and open-loop control disabled;
- finite two-second controller goal-time allowance;
- explicit six-axis trajectory tolerance 0.1 rad, goal tolerance 0.03 rad,
  and stopped-velocity tolerance 0.05 rad/s.

The reviewed smoke starts only after three strictly time-increasing exact
six-joint messages are observed near the neutral reference. Its six nonzero
targets are at most 0.06 rad in magnitude, at most 0.06 rad from neutral, and
at least 0.05 rad inside every canonical position limit. The requested motion
duration is three seconds. The goal comes only through
`FollowJointTrajectory`; the harness never writes a Gazebo joint command.

Runtime evidence is deliberately separated:

1. `LIMITER_RUNTIME_INITIALIZED=PASS` requires the read-only controller-manager
   parameter to be true, all six limiter-creation messages, and the enabled
   enforcement log with no disabled log.
2. `BOUNDED_TRAJECTORY=PASS` requires goal acceptance, action `SUCCEEDED`,
   controller `error_code: 0`, at least ten real feedback frames whose exact
   six positions and velocities are finite and whose positions remain inside
   canonical limits, followed by five strictly time-increasing target-state
   messages inside the 0.03 rad tolerance.
3. `CLAMP_PROBE=NOT_RUN` and `CLAMP_BEHAVIOR_VERIFIED=false` remain explicit.
   An in-range trajectory and limiter initialization do not prove clamping.

The launch also has to retain one active `GazeboSimSystem`, the exact two
controllers, the controller manager and robot-state publisher nodes, one
persisting `abb_irb1200` entity, and a fixed model pose within 1e-5 m/rad before
and after motion. Resource-resolution failures are fatal.

## Isolation and recovery contract

The gate builds, installs, tests, and logs only on a new or explicitly empty
Linux filesystem directory. Repository descendants, `/`, nonempty paths, and
DrvFS output roots are rejected. Each run acquires an exclusive project lock
for a ROS domain, restricts discovery to localhost, creates a unique Gazebo
partition, and puts ROS home/log data under the output root.

The outer 180-second watchdog manages the synchronous harness process group,
including colcon children. The launched runtime has its own recorded process
group and is cleaned with bounded `INT -> TERM -> KILL` escalation. Cleanup
targets only that group, stops only the selected-domain ROS daemon, and rejects
residual processes. PASS labels are written to `result.txt` only after this
cleanup succeeds. A SHA-256 manifest over every tracked or nonignored untracked
source file is captured before and after the gate and must remain byte-equal.

## Executed development regression

Command:

```bash
bash scripts/run_and_verify_irb1200_headless.sh /var/tmp/p13b-dev-run-12
```

The development gate builds and tests all three project packages, executes the
headless runtime, inspects saved logs, runs the bounded trajectory, verifies
the fixed base and persistent entity/controllers/nodes, then proves cleanup.
It records 36 unique pytest methods (23 description, 6 control, and 7 Gazebo)
plus five CTest wrapper records. `colcon test-result` therefore reports 41
records with zero errors, failures, or skips. The repository harness contract
adds one separately executed unittest and is not counted as a 42nd colcon
test. The observed development-run result fields were:

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
SOURCE_WORKTREE_CLEAN=false
```

This development run is not commit-addressable P1.3b acceptance evidence.
`TESTED_COMMIT=9418403433c29b172a417ba74b0a7849eb07922a` identifies the
pre-P1.3b HEAD, while `SOURCE_WORKTREE_CLEAN=false` identifies the tested
P1.3b implementation as uncommitted working-source bytes. The before/after
working-source manifests were byte-identical during the run. After the feature
commit, the same gate must be run from a clean worktree and must report that
new commit plus `SOURCE_WORKTREE_CLEAN=true` before A01 can change.

## Warnings and non-claims

The development launch log records four warning classes: a transient wait for
the resource manager to initialize hardware, no executor during simulated
component initialization, no read/write statistics for that component, and a
100 Hz controller period slower than the 1 ms physics period. The last item is
an intentional integer-ratio configuration, not a failed controller cycle.
The hardware subsequently reaches active state and the complete
observation/action gate passes; none of these warnings is hidden. The final
clean evidence will list the warning set observed in that exact run rather
than copying a development-run list.

This evidence does not establish collision checking, HOME to pose to HOME,
MoveIt readiness, GUI rendering, physical effort fidelity, derivative limits,
out-of-range clamping, or real-robot safety. A02-A14 remain `NOT STARTED`.
