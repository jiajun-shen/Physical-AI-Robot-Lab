# Acceptance Status

Only an observed successful run can be marked `PASS`. Source code presence or
an expected result is insufficient.

| ID | Acceptance target | Status | Evidence |
|---|---|---|---|
| A01 | Launch Gazebo and observe ABB IRB1200 spawned normally | PASS | Clean headless project run at `1228c13fadb8c04d348a0bfa68e2fa1d0a7ce6ee`, 2026-08-11 15:37:51 +0200; exact `abb_irb1200` persisted in world `abb_lab`; `/var/tmp/p13b-clean-1228c13/result.txt`; cleanup verified. |
| A02 | Use MoveIt to plan and execute HOME -> specified Pose -> HOME with no collision | PASS | Clean headless P2.2 run at `4203e81508f91b651eb8f55e663c74f21b8ff5b0`, 2026-08-11 17:01:57 +0200. MoveIt planned/executed normal joint and FK-derived `tool0` pose routes through the existing JTC, observed HOME -> Pose -> HOME, and rejected both pose/joint goals after an injected planning-scene collision object. 58 test records passed; `/var/tmp/p22-clean-4203e81/result.txt` and `docs/phase2_moveit_runtime_evidence.md`. No owned process remained, but teardown observed `move_group` SIGSEGV; see KI-012. |
| A03 | Randomize green sphere pose and localize it in vision mode without ground truth | PASS | Clean Phase-4 two-seed runtime at `62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`: the node consumed only RGB, depth, CameraInfo, and TF, sealed `ground_truth_input_used: false` vision evidence before separate offline live-Gazebo comparison, and localized green ball within 0.05 m on seed 42 (`0.0103339 m`) and seed 43 (`0.0150882 m`). Evidence: `/var/tmp/physical-ai-p4.Kn4hFG`, `docs/phase4_perception_runtime_evidence.md`. |
| A04 | Green sphere is picked and placed next to red sphere over multiple seeds | NOT STARTED | P3's two seed runs used free-space finger actions only: no contact, attachment, lift, arm motion, grasp, or placement occurred. |
| A05 | Robot draws a heart from a generated Cartesian mathematical curve | NOT STARTED | None |
| A06 | Chinese microphone command executes | NOT STARTED | None |
| A07 | English microphone command executes | NOT STARTED | None |
| A08 | French microphone command executes | NOT STARTED | None |
| A09 | Long language command becomes a validated, inspectable TaskGraph and multiple verified sequential tasks | NOT STARTED | None |
| A10 | Every TaskGraph step is re-observed and verified; a moved target triggers replan rather than stale-pose reuse | NOT STARTED | None |
| A11 | System Explorer shows end-to-end data/execution chain | NOT STARTED | None |
| A12 | VLA starts when supported, otherwise explicit unavailable + safe fallback | NOT STARTED | None |
| A13 | ABB demonstration episode is collected and converted to LeRobot format | NOT STARTED | None |
| A14 | openpi training config/dry-run validates; optional small overfit if feasible | NOT STARTED | None |

## Phase-3 acceptance record (release to Phase 4)

The scoped Phase-3 implementation and final real-run evidence are complete at
`f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63`. On 2026-08-14 the user explicitly
accepted the saved Phase-3 review and authorized Phase 4. This release is
separate from M1 and does not advance A03, A04, M1.04, or any later acceptance
item.

| ID | Phase-3 real-run proof | Status | Immutable evidence |
|---|---|---|---|
| P3.01 | The project ABB model with the two-finger profile, one active Gazebo control chain, and a free-space open/close/open action operates in Gazebo | PASS | Two clean runs on `f6907ff`, one per selected seed, each report active `GazeboSimSystem`, JSB/JTC/gripper controller, live named joint states, `ParallelGripperCommand` action success, and mirrored finger state. `/var/tmp/p3-final-f6907ff-seed42` and `/var/tmp/p3-final-f6907ff-seed43`. |
| P3.02 | Two distinct selected seeds are actually launched and show live seeded-scene variation | PASS | Captured `gz model --list` and `gz model -p` evidence after the free-space actions for seeds 42 and 43; comparison `/var/tmp/p3-final-f6907ff-seed-comparison.json` records six material position deltas and both directional-cube yaw deltas. |
| P3.03 | User accepts the Phase-3 review and authorizes Phase 4 | PASS | The user explicitly accepted the saved two-seed Phase-3 real-run evidence on 2026-08-14. This records authorization only; it is not a new runtime result and does not change the position-only/no-contact boundary. |

P3.01 is deliberately position-only and free-space evidence. It is not force,
contact, attachment, grasp, lifting, transport, perception, or pick/place
evidence. The default launch does not automatically randomize: the final runs
used the explicit seeded generator/harness.

## Phase-4 acceptance record (vision-only RGB-D localization)

The clean Phase-4 final runtime at
`62dc9c8d0d682239dd2e4567f311a3ce2551ca4a` selected seeds 42 and 43 in
separate isolated Gazebo worlds. Each run first sealed RGB/depth/CameraInfo/TF
and typed output artifacts with `PERCEPTION_MODE=vision` and
`ground_truth_input_used: false`; only then did a separate process capture live
Gazebo poses for offline comparison. This is simulation-only perception
evidence. It does not change A04 or any contact, attachment, grasp, lift,
transport, or placement boundary.

| ID | Phase-4 real-run proof | Status | Immutable evidence |
|---|---|---|---|
| P4.01 | RGB, 32FC1 metre depth, CameraInfo, and the camera optical-frame TF chain are observed from the real RGB-D simulation sensor | PASS | Both run roots under `/var/tmp/physical-ai-p4.Kn4hFG/runs/seed{42,43}` contain sealed `vision_evidence.json`, RGB/depth artifacts, CameraInfo, TF chain, and source timestamps/frame IDs. |
| P4.02 | All five required labels yield typed vision-mode detections plus camera/world poses derived through depth and TF2 | PASS | Each sealed record reports exactly `green_ball`, `red_ball`, `blue_cube`, `yellow_cube`, and `drawing_board`; all five output checks pass for both seeds. |
| P4.03 | The green ball is localized without online Gazebo ground-truth input across two genuinely different seeded scenes | PASS | Offline comparisons report green errors of `0.0103339 m` (seed 42) and `0.0150882 m` (seed 43), both <= `0.05 m`; the later two-seed comparison records a `0.3200000 m` live-green position delta. |
| P4.04 | Ground truth is structurally offline and follows sealed vision evidence | PASS | Vision artifact hashes and `ground_truth_input_used: false` were written before `capture_p4_live_ground_truth.py`; the separate evaluator then produced the two offline comparison JSON files. |

The full gate reports `P4_RGB_TRANSPORT`, `P4_DEPTH_TRANSPORT`,
`P4_CAMERA_INFO`, `P4_TF_CHAIN`, `P4_DEPTH_PROJECTION`,
`P4_ALL_FIVE_DETECTIONS`, `P4_VISION_GROUND_TRUTH_BOUNDARY`,
`P4_OFFLINE_EVALUATION`, and `P4_TWO_SEED_VISION_ONLY_LOCALIZATION` as
`PASS`; 29 build/test records had zero errors, failures, or skips and cleanup
was verified.

## M1: mandatory midterm acceptance gate (after Phase 8, before Phase 9)

`M1` is `PENDING`. Its lifecycle is `PENDING -> PAUSED - AWAITING USER
MIDTERM ACCEPTANCE -> ACCEPTED`. After Phase 8's implementation and real-run
evidence are complete, development must stop in the paused state. Phase 9 may
start only when both conditions hold: every row below is `PASS` on the tested
Phase-8-final commit, and the user explicitly records acceptance. Source code,
unit tests, static configuration, or earlier evidence alone cannot pass M1;
pre-M1 evidence must be rerun if the integrated path changes.

| ID | Mandatory real-run proof | Status | Required immutable evidence |
|---|---|---|---|
| M1.01 | ABB IRB1200 runs normally in the project Gazebo world | PRE-M1 EVIDENCE | Active world/model, joint state, and runtime logs. Current A01 headless run is a predecessor, not final M1 evidence. |
| M1.02 | ros2_control actually controls ABB | PRE-M1 EVIDENCE | Active JSB/JTC, claimed interfaces, action feedback, and real joint changes. Current P1.3b run is a predecessor, not final M1 evidence. |
| M1.03 | MoveIt plans and executes collision-aware motion | PRE-M1 EVIDENCE | Current A02 clean run proves real normal plan/execution and planning-scene collision rejection at `4203e81`; it is a predecessor only and must be rerun as part of the Phase-8-final integrated M1 evidence. |
| M1.04 | Two-finger gripper opens, closes, and grasps an object | NOT STARTED | P3 proves free-space open/close only. Command/state plus contact or successful lift/transport is still required; animation alone is insufficient. |
| M1.05 | Live lab contains a table, red/green balls, cubes, drawing board, and collision geometry | PRE-M1 EVIDENCE | P3 captured the required named models and their live Gazebo poses for two selected seeds at `f6907ff`; generated SDF/static contracts provide the collision-model precursor. This is not the final integrated M1 run and did not inspect live collision shapes, so it must be rerun on the immutable Phase-8-final commit. |
| M1.06 | RGB-D plus TF determines object positions | PRE-M1 EVIDENCE | Phase 4 at `62dc9c8` provides two-seed simulation-only RGB/depth/CameraInfo/TF2 vision poses and sealed-before-offline-GT comparison. It is predecessor evidence only and must be rerun on the immutable Phase-8-final integrated commit; `vision` must not consume Gazebo ground truth. |
| M1.07 | Green ball is grasped and placed beside red ball | NOT STARTED | New observations before/after, successful grasp/placement, defined beside-threshold, and multi-seed evidence. |
| M1.08 | A real-time generated Cartesian heart trajectory is executed on the board | NOT STARTED | Parametric generation, end-effector/board evidence, and execution trace; prerecorded joint motion is insufficient. |
| M1.09 | Microphone recognizes Simplified Chinese, English, and French | NOT STARTED | Three microphone recordings, transcripts, language labels, confidence, and downstream execution records. |
| M1.10 | Long language instruction becomes a valid inspectable TaskGraph | NOT STARTED | Original text/transcript, TaskGraph, schema validation, and provenance. |
| M1.11 | Two or more tasks run continuously in required order | NOT STARTED | Node execution states, order, results, and failure/recovery trace. |
| M1.12 | Each task step re-observes, verifies, then continues/retries/replans | NOT STARTED | New timestamped WorldState and postcondition after every node, including a real moved-target replan case. |

## Evidence required when a status changes

```text
Status:
Tested commit:
Date/time:
Environment:
Brain/perception mode:
Seed(s):
Command:
Observed result:
Evidence/log/artifact:
Known limitation:
Next exact step:
```

For A14, configuration/dry-run validation is mandatory. The subset overfit is a
separate conditional result and may remain unavailable when hardware does not
permit it.
