# Phase 3 Gripper and Live Laboratory Runtime Evidence

- Tested code commit: `f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63`
- Working-source manifest SHA-256:
  `63e531b1a67c42132d434ef721cb5c9b076df8a8781d58e385d121ea039b0736`
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Harmonic
- Scope: headless simulation only; one project-owned ABB-with-gripper profile
  per run
- Clean runtime roots:
  `/var/tmp/p3-final-f6907ff-seed42` and
  `/var/tmp/p3-final-f6907ff-seed43`
- Cross-run live-scene comparison:
  `/var/tmp/p3-final-f6907ff-seed-comparison.json`

## Final Phase-3 real-run result

Two independent clean runtime gates ran the same commit and source manifest,
one selecting seed 42 and the other selecting seed 43. Both recorded
`SOURCE_WORKTREE_CLEAN=true`, built seven packages on ext4, and reported 82
test records with 0 errors, 0 failures, and 0 skips. Both gates passed the
project ABB-with-gripper spawn, active `GazeboSimSystem`, active controllers,
command-limit initialization, laboratory model presence, live Gazebo pose
capture, gripper action sequence, and cleanup.

Each live run had exactly these ten model names in `manipulation_lab` before
and after the free-space gripper actions:

```text
ground_plane
work_table
drawing_board
obstacle_left
obstacle_right
green_ball
red_ball
blue_cube
yellow_cube
abb_irb1200_gripper
```

The post-action evidence is derived from recorded `gz model --list` and
`gz model -m <name> -p` responses, not by re-reading the generated SDF or
manifest. It therefore proves that both selected scenes were actually launched
and their named live model poses were captured. It does **not** prove live
collision-shape inspection, contact, attachment, force, settled-state
stability beyond the captured instant, or perception.

## Control profile and real gripper action

Each run observed one active `GazeboSimSystem`, one controller manager, seven
claimed position command interfaces (the six ABB joints plus
`two_finger_right_joint`), and 22 state interfaces. The state-only left finger
has position/velocity state and mirrors the right finger; it has no command
interface. The following controllers were active before and after the action
sequence:

- `joint_state_broadcaster`;
- `joint_trajectory_controller`; and
- `two_finger_gripper_controller`
  (`parallel_gripper_action_controller/GripperActionController`).

The verifier used the real
`/two_finger_gripper_controller/gripper_cmd`
`control_msgs/action/ParallelGripperCommand` endpoint. In both selected scenes
it completed `open -> closed clearance -> reopen`, with action status `4`
(`SUCCEEDED`), `reached_goal=true`, `stalled=false`, finite bounded named joint
states, and mirrored finger positions. The right-finger targets were 0.035 m,
0.004 m, and 0.035 m. The observed endpoints were:

| Selected seed | First open | Closed clearance | Reopen | Valid joint-state samples |
|---:|---:|---:|---:|---:|
| 42 | 0.0340279 m | 0.0058317 m | 0.0332441 m | 419 |
| 43 | 0.0331906 m | 0.0053158 m | 0.0338779 m | 340 |

The configured 0.020 m ros2_control state seed was not treated as a physical
spawn pose. The first live right-finger observations were 0.0070532 m (seed
42) and 0.0066231 m (seed 43), after physics had already settled the mechanism.
Both action verifiers recorded zero invalid joint-state frames.

This is a **position-only, free-space** gripper result. Action result state
names were empty and only a single position was recorded, so named
`/joint_states`, rather than the action result alone, are the finger-state
evidence. The gripper has no effort command/state interface, controller
`max_effort` is zero, and its URDF effort limit is structural metadata only.
No valid finger-force conclusion is available. The saved status in both runs
is:

```text
GRIPPER_OPEN_CLOSE_OPEN=PASS
GRIPPER_MIRROR_STATE=PASS
GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE
```

`force_control` is `NOT_CONFIGURED` and `object_attachment` is
`NOT_IMPLEMENTED`. No object was contacted, retained, lifted, or transported.

## Bullet Featherstone profile-only requirement

The P3 launch explicitly selects
`gz-physics-bullet-featherstone-plugin`. Gazebo Harmonic's default DART engine
does not implement the URDF mimic constraint used by this profile's state-only
left finger. Bullet Featherstone is therefore required to reproduce this
specific gripper profile. It is not a global change to the project physics
baseline, does not alter the earlier arm-only P1/P2 evidence, and does not
establish cross-engine, contact, or real-gripper equivalence.

## Real seeded-scene variation

The two completed Gazebo runs selected seeds 42 and 43 respectively. The
comparison reads only their captured live Gazebo evidence and requires the same
clean commit, distinct selected seeds, every expected model, finite poses, and
the following thresholds: at least 0.020 m position change for primary
witnesses and at least 0.150 rad yaw change for directional cubes.

| Live witness | Position delta between seed 42 and 43 |
|---|---:|
| `blue_cube` | 0.2000000 m |
| `green_ball` | 0.3200002 m |
| `obstacle_left` | 0.4308132 m |
| `obstacle_right` | 0.2561250 m |
| `red_ball` | 0.4003933 m |
| `yellow_cube` | 0.1829426 m |

The live directional-cube yaw deltas were 1.0000000 rad for `blue_cube` and
0.4722650 rad for `yellow_cube`. The comparison therefore records all of
`same_clean_commit`, `distinct_selected_seeds`,
`live_seeded_object_position_variation`, and
`live_directional_cube_orientation_variation` as `PASS`.

The scene generator also creates exact duplicate external manifest/SDF files
for repeated invocations of a selected seed, and rejects invalid declared
tabletop bounds or pairwise overlap. That is not a proof of ABB reachability,
MoveIt collision-free solvability, or real contact behavior.

The default bringup launch points to a static package world template. Seeded
variation is an explicit prelaunch generator/harness operation; it is **not**
automatic randomization every time the default launch is invoked.

## Warnings and honest boundary

Both final logs contain the expected startup resource-manager/executor and
100 Hz-versus-1 ms simulation-period warnings. Each also contains three
controller-manager `Command ... out of limits` messages at the open, close,
and reopen step goals. These are per-cycle position-rate limiter interventions
for otherwise in-range targets; the action endpoints subsequently succeeded.
They are not a broad out-of-range clamp probe or proof of general clamp
behavior, and the final runs must not be described as warning/error-free.

The selected final runs reported `publish_async_failures_ 0` at teardown and
completed requested cleanup with launch exit status 0. This does not turn the
profile into a real-hardware or force/contact-fidelity result.

This Phase-3 evidence itself does not establish A03 RGB-D localization, A04
pick/place, M1.04 grasp, MoveIt-with-gripper laboratory motion, arm execution
in the lab, GUI rendering, vision, real robot control, or physical safety.
A03 was subsequently passed only by the separate Phase-4 vision-only runtime
at `62dc9c8`; A04 and M1.04 remain `NOT STARTED`.

## Phase-3 acceptance record

Phase 3 is complete within its stated gripper-and-live-laboratory scope and is
recorded as user accepted on 2026-08-14. P3.03 is `PASS` and Phase 4 is
authorized. This earlier release only governs entry to Phase 4; it does not
mark any M1 item `PASS` and does not weaken the separate mandatory Phase-8 M1
hold before Phase 9. The accepted evidence remains position-only free-space
gripper behavior plus live seeded-scene variation, not contact, force,
attachment, grasp, lifting, transport, perception, or pick/place evidence.
