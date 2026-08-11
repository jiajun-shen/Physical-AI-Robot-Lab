# Acceptance Status

Only an observed successful run can be marked `PASS`. Source code presence or
an expected result is insufficient.

| ID | Acceptance target | Status | Evidence |
|---|---|---|---|
| A01 | Launch Gazebo and observe ABB IRB1200 spawned normally | PASS | Clean headless project run at `1228c13fadb8c04d348a0bfa68e2fa1d0a7ce6ee`, 2026-08-11 15:37:51 +0200; exact `abb_irb1200` persisted in world `abb_lab`; `/var/tmp/p13b-clean-1228c13/result.txt`; cleanup verified. |
| A02 | Use MoveIt to plan and execute HOME -> specified Pose -> HOME with no collision | NOT STARTED | None |
| A03 | Randomize green sphere pose and localize it in vision mode without ground truth | NOT STARTED | None |
| A04 | Green sphere is picked and placed next to red sphere over multiple seeds | NOT STARTED | None |
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

## M1: mandatory midterm acceptance gate (after Phase 8, before Phase 9)

`M1` is `NOT STARTED`. Its lifecycle is `PENDING -> PAUSED - AWAITING USER
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
| M1.03 | MoveIt plans and executes collision-aware motion | NOT STARTED | Real plan/execution plus planning-scene collision proof; a raw ros2_control trajectory is insufficient. |
| M1.04 | Two-finger gripper opens, closes, and grasps an object | NOT STARTED | Command/state plus contact or successful lift/transport; animation alone is insufficient. |
| M1.05 | Live lab contains a table, red/green balls, cubes, drawing board, and collision geometry | NOT STARTED | Running-world model/scene evidence, not just SDF source. |
| M1.06 | RGB-D plus TF determines object positions | NOT STARTED | RGB, depth, CameraInfo, TF, and pose output. `vision` must not consume Gazebo ground truth; ground truth is offline comparison only. |
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
