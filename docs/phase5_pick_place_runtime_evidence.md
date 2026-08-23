# Phase 5 — Verified Pick/Place Runtime Evidence

## Result

Phase 5 and acceptance item A04 are `PASS` for simulation. The final runtime
tested source commit is `7e682d45e6c666060bbb84b192fd2ab756987b73`.

The final isolated harness root is:

```text
/var/tmp/physical-ai-p5.lift-trajectory.final.20260823
```

It reports `P5_PICK_PLACE=PASS`, `A04=PASS`,
`SOURCE_MANIFEST_UNCHANGED=PASS`, and `WORKTREE_CLEAN_AFTER_RUNTIME=PASS`.

## Evidence boundary

Each online episode consumed RGB-D, CameraInfo, and TF-derived object poses;
`online_ground_truth_input_used` is false in both execution records. Gazebo
ground truth is not used by online decisions. The fresh Phase-4 vision seals
the post-release observation before its offline evaluator consumes it.

Every arm motion is collision-aware MoveIt planning/execution. A close action
or finger endpoint alone is not grasp proof: bounded contact-close accepts an
action-accepted 20--40 mm joint-state endpoint, including the explicit
stalled/status-6 contact case, but the episode passes only after a new
RGB-D-measured lift of at least 50 mm, fresh TCP-relative visual transport
retention, fresh post-place RGB-D, support, and final-distance verification.
OPEN and RELEASE retain normal action and endpoint requirements.

## Independent seeded episodes

| Seed | Visual lift delta | Visual transport | Final green-red planar distance | Evaluator |
|---|---:|---:|---:|---|
| 42 | 0.0781464666 m | 0.5405326913 m | 0.0794259254 m | PASS |
| 43 | 0.0812261127 m | 0.0605447531 m | 0.1189200508 m | PASS |

The required final distance band is `[0.075, 0.120] m`; both final support
heights are within the configured table-support tolerance. Each per-seed
`p5_evaluation.json` records PASS for grasp retention, MoveIt motion,
Phase-4 vision provenance, release, post-place observation, stage sequence,
and pick/place evaluation.

The offline two-seed comparison
`/var/tmp/physical-ai-p5.lift-trajectory.final.20260823/p5_two_seed_comparison.json`
is PASS for distinct seeds, run directories, Gazebo partitions, artifact
hashes, and pre-pick RGB-D variation. Its measured green-ball initial vision
position delta is `0.321813231388 m`.

## Required sequence

Both episodes passed all saved stages:

```text
OBSERVE_PICK -> PLAN_APPROACH -> OPEN -> APPROACH -> CLOSE -> LIFT_VERIFY
-> TRANSPORT -> LOWER -> RELEASE -> RETREAT -> OBSERVE_PLACE -> VERIFY_PLACE
```

`LIFT_VERIFY` accepts a fresh image captured during or after the collision-
aware lift command when the terminal wrist pose occludes the ball. The
evaluator requires its source timestamp to be later than the recorded lift
command boundary and still checks the >=50 mm object-height change against
the pre-close RGB-D guard. Thus it cannot reuse a pre-lift cache entry.

## Tests and reproducibility

The final harness built the seven-package dependency closure and recorded 67
tests with zero errors, failures, or skips. Direct Phase-5 contract/runtime/
static-vision/evaluator tests also passed (19 tests) on the final code path.

Run command:

```bash
source /opt/ros/jazzy/setup.bash
scripts/run_and_verify_phase5_pick_place_headless.sh \
  /var/tmp/physical-ai-p5.lift-trajectory.final.20260823
```

This is simulation evidence only. It does not resolve real-robot force,
contact-quality, or hardware-safety questions.
