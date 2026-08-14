# Phase-4 RGB-D Perception Evidence Contract

## Scope and status

This contract defines the evidence required before Phase 4 can be marked
complete.  It is written before Phase-4 implementation so that the runtime
gate cannot silently substitute a simulator pose, seeded-world manifest, or
hard-coded object coordinate for a vision observation.

Phase 4 is simulation-only.  It is an RGB-D perception increment and makes no
claim about contact, grasp, lift, transport, placement, force control, or
real-robot readiness.  In particular, A04 remains `NOT STARTED`.

## Required data path

The only allowed online input path is:

```text
Gazebo RGB-D sensor
  -> ROS Image + depth Image + CameraInfo + TF
  -> color/shape 2-D detection
  -> valid-depth sampling and CameraInfo back-projection
  -> camera optical-frame XYZ
  -> TF2 transform at the sensor timestamp
  -> typed ObjectDetection and ObjectPose outputs
```

`PERCEPTION_MODE=vision` is mandatory.  The vision component may subscribe
only to RGB, depth, CameraInfo, `/tf`, and `/tf_static`.  Its configuration
may contain color ranges, area/shape thresholds, and physical size priors, but
must not contain scene seeds, object/world coordinates, Gazebo model poses,
manifest paths, or a grid of candidate positions.

The five required labels are `green_ball`, `red_ball`, `blue_cube`,
`yellow_cube`, and `drawing_board`.  The baseline may use OpenCV color
segmentation and geometry.  Intrinsics must be read from the real CameraInfo
used for each output; no hard-coded focal length or principal point is allowed.

## Ground-truth boundary

Gazebo model pose is prohibited from the perception package and from its
runtime inputs.  It is permitted only in a separate, offline evaluator after
the vision evidence is sealed.  The evaluator must not publish any pose or
model information back into the running vision node.

The runtime harness therefore has this mandatory ordering:

```text
real RGB-D/CameraInfo/TF observations -> seal vision evidence ->
separate `gz model -p` capture -> offline error comparison
```

The sealed vision record must state `ground_truth_input_used: false`, include
hashes of its source artifacts, and be written before the first ground-truth
query.  Static source tests must reject Gazebo/model-pose/scene-manifest
imports, Gazebo command execution, and ground-truth terms in
`physical_ai_perception`.

## Real-run evidence bundle

For each seed, retain an external, immutable evidence root containing at least:

- run metadata: tested commit, clean-source status, source-manifest hash,
  ROS domain, Gazebo partition, seed, and `PERCEPTION_MODE=vision`;
- the seeded RGB-D world and the launch/runtime logs;
- a real RGB frame and an exact depth-frame artifact, with encoding, units,
  dimensions, timestamp, and frame ID;
- the matching CameraInfo and recorded TF chain;
- typed detections plus camera-frame and world-frame poses, including pixel
  coordinates, valid-depth statistics, source timestamp, and frame IDs;
- a sealed vision-evidence manifest with SHA-256 values for the preceding
  artifacts and `ground_truth_input_used: false`;
- separately captured raw live Gazebo-pose responses, their hashes, and an
  offline comparison JSON; and
- build/test results, source manifest before/after, controlled cleanup result,
  and the final status record.

Depth is valid only when finite and strictly positive.  The runtime verifier
must inspect the actual encoding and units; it must not assume a fixed invalid
sentinel.  TF2 must transform at the image timestamp.  Missing, stale, or
frame-inconsistent inputs are failures, not a reason to fall back to a scene
coordinate.

## Acceptance thresholds

The Phase-4 simulation gate will use two selected live seeded worlds, 42 and
43.  Each must produce exactly one vision-mode localization for all five
labels.  The independently captured live Gazebo comparison is accepted only
when the center-position error is at most 0.05 m for each ball/cube and at
most 0.06 m for the drawing board.  These are perception-evaluation
thresholds, not grasp or placement tolerances.

A03 may be marked `PASS` only when the green sphere has, on both seeds, a
sealed RGB -> pixel -> depth -> CameraInfo XYZ -> TF2 world/base pose produced
in vision mode and a subsequent offline live-Gazebo comparison within the
threshold.  Unit tests, static inspection, seeded SDF data, or Gazebo truth
fed to the pipeline cannot pass A03.

## Completion rule

Phase 4 is `COMPLETE` only after implementation, build, unit/static tests,
both real runtime executions, visual/result inspection, evidence retention,
documentation, a clean tested commit, and push all succeed.  Until then it is
reported as `PARTIAL`; Phase 5 remains locked.

## Completion record (2026-08-14)

This contract was satisfied by the clean tested feature commit
`62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`. The isolated two-seed gate at
`/var/tmp/physical-ai-p4.Kn4hFG` built/tested 29 records with zero errors,
failures, or skips; sealed real vision-only evidence for seeds 42 and 43;
then performed its separate offline live-Gazebo comparison. All five labels
passed both runs, and A03's green-ball errors were 0.0103339 m and 0.0150882 m
against the 0.05 m threshold. The final evidence and limitations are recorded
in `docs/phase4_perception_runtime_evidence.md`.

Phase 4 is therefore `COMPLETE`. This record does not begin, implement, or
validate Phase 5, and it does not change A04 or any contact/grasp boundary.
