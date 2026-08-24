# Phase 6 — Generative Drawing Runtime Evidence

## Immutable run

Phase 6/A05 was sealed on 2026-08-24 using source commit
`eba23f1177d59e45cb924eea4f2b3f26f0af6801` and the ext4 evidence root:

```text
/var/tmp/physical-ai-p6.final.isolated.eba23f1.20260824
```

The clean-source harness command was:

```bash
scripts/run_and_verify_phase6_drawing_headless.sh \
  /var/tmp/physical-ai-p6.final.isolated.eba23f1.20260824
```

Its final `result.txt` records:

```text
P6_DRAWING=PASS
A05=PASS
SOURCE_MANIFEST_UNCHANGED=PASS
WORKTREE_CLEAN_AFTER_RUNTIME=PASS
```

The targeted Phase-6 build/test closure reported 85 tests with zero errors,
failures, or skips. The independent evaluator reported
`P6_DRAWING_EVALUATION=PASS`.

## Acceptance cases

Each input ran in a newly launched headless Gazebo, MoveIt, and RGB-D vision
episode. The previous episode was fully stopped before the next began; each
episode's runtime metadata records its own `GZ_PARTITION`, and its post-cleanup
record reports both `CLEANUP_STATUS=PASS` and `ROS_DOMAIN_EMPTY=PASS`.

| Case | Instruction | Observed intent | Acceptance role | TCP samples |
|---|---|---|---|---:|
| `circle_curve` | `画一个小一点的圆` | `circle`, `center`, 0.08 m | Curve | 2072 |
| `left_square` | `在白板左边画一个小一点的正方形` | `square`, `left`, 0.08 m | Polygon | 2091 |
| `left_circle` | `在白板左边画一个小一点的圆` | `circle`, `left`, 0.08 m | Board-position variation | 2079 |

The evaluator read the three sealed
`cases/*/drawing_execution_evidence.json` files, confirmed both shapes and a
non-centre position, and marked `parameter_variation: true`.

## Observed safety and execution boundaries

- Before each first motion, the P4 verifier sealed a fresh RGB-D observation of
  the robot-facing drawing-board surface. The runtime derived the collision-box
  centre from that surface rather than treating the visible surface as a volume
  centre.
- The P5 RGB-D static-obstacle localizer supplied both required markers from one
  fresh visual epoch: `p5_orange_top_fiducial` and
  `p5_violet_top_fiducial`. Their conservative
  collision boxes, the work table, and P4 visual objects were present in the
  live MoveIt planning scene for approach, entry, curve, and retreat.
- Every case used collision-aware MoveIt planning. The first case's explicit
  live-state `entry_alignment` planned exactly one runtime-generated first
  waypoint with fraction `1.0`, a 27 s conservative entry profile, successful
  action status `4`, and a three-sample TF TCP position error of
  `1.4490622314706874e-07 m` (well inside the 0.008 m gate).
- The evidence contains requested drawing-local geometry, board-frame mapping,
  generated world waypoints, MoveIt fractions/scenes, action outcomes, endpoint
  gates, and live TCP traces. No prerecorded joint trajectory is used.

## Scope

This is real project Gazebo/ROS 2/MoveIt execution evidence for Phase 6. It
does not claim physical-board marking, real-camera performance, speech input,
general task planning, or final M1 acceptance. It is predecessor evidence for
M1.08 and must be rerun inside the immutable Phase-8-final integrated M1 run.
