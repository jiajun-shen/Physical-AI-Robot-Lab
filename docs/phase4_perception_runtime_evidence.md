# Phase-4 RGB-D Perception Runtime Evidence

## Final status

Status: `COMPLETE` for Phase 4. A03 is `PASS`; A04 remains `NOT STARTED`.
This is simulation-only RGB-D perception evidence, not contact, grasp, lift,
transport, placement, GUI, real-camera, or real-robot evidence.

- Tested feature commit: `62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`
  (`fix(phase4): avoid arm occlusion in seeded RGB-D views`)
- Date: 2026-08-14
- Environment: WSL Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.11.0
- Harness: `scripts/run_and_verify_phase4_perception_headless.sh`
- External evidence root: `/var/tmp/physical-ai-p4.Kn4hFG`
- Source worktree: clean before and after; manifest SHA-256
  `efa51900468bba2a5a6ce2741b95aed458e0e04fa8f58a973485a270582aee54`
- Selected seeds: 42 and 43, launched in separate isolated worlds.
- Build/test: 29 records, zero errors, failures, or skips.
- Final cleanup: `CLEANUP_VERIFIED=PASS`.

The original P3 world and P3 harness were not changed. Phase 4 uses the
separate `manipulation_lab_rgbd.sdf` plus a P4 wrapper launch that retains the
existing sole Gazebo-hosted controller manager and adds only one-way RGB,
depth, CameraInfo transport and static camera TF.

## Online vision path and ground-truth boundary

For each seed, `physical_ai_perception` received only:

```text
/lab_camera/color/image_raw
/lab_camera/depth/image_raw
/lab_camera/color/camera_info
/tf and /tf_static
```

It emitted project-owned typed `ObjectDetection` and camera/world
`ObjectPose` array messages. The pipeline used OpenCV color/shape components,
finite positive depth samples, the observed CameraInfo K matrix, and TF2 at
the source timestamp. It has no Gazebo model-pose, scene-manifest, seed, or
world-coordinate input.

The harness order was enforced per seed:

```text
real RGB-D + CameraInfo + TF + typed vision output
  -> seal vision_evidence.json (ground_truth_input_used: false)
  -> separate capture_p4_live_ground_truth.py process
  -> separate offline evaluator
```

The sealed records report `PERCEPTION_MODE=vision`, `rgb8` RGB at 640x480,
`32FC1` depth in metres, `lab_camera_optical_frame`, matching source stamps,
and SHA-256 hashes for RGB, depth, CameraInfo, TF, typed outputs, and the
annotated inspection image. No model pose was bridged into ROS or supplied to
the vision node.

## Per-seed offline results

The independently captured live Gazebo poses were compared only after vision
evidence sealing. Object thresholds were 0.05 m and the drawing-board
threshold was 0.06 m.

| Seed | Green | Red | Blue | Yellow | Drawing board | Result |
|---:|---:|---:|---:|---:|---:|---|
| 42 | 0.0103339 m | 0.0101891 m | 0.0135526 m | 0.0125586 m | 0.0172238 m | PASS |
| 43 | 0.0150882 m | 0.0110272 m | 0.0119268 m | 0.0117753 m | 0.0172238 m | PASS |

The separate two-seed comparison recorded a live green-ball position delta of
0.3200000 m, proving that A03 did not merely repeat one scene. Both green-ball
vision errors are within the 0.05 m A03 threshold.

## Evidence locations

- Overall result/status: `/var/tmp/physical-ai-p4.Kn4hFG/result.txt`
- Two-seed comparison: `/var/tmp/physical-ai-p4.Kn4hFG/p4_two_seed_comparison.json`
- Seed 42 sealed vision bundle:
  `/var/tmp/physical-ai-p4.Kn4hFG/runs/seed42/evidence/vision_evidence.json`
- Seed 42 offline comparison:
  `/var/tmp/physical-ai-p4.Kn4hFG/runs/seed42/offline_gt/offline_ground_truth_comparison.json`
- Seed 43 sealed vision bundle:
  `/var/tmp/physical-ai-p4.Kn4hFG/runs/seed43/evidence/vision_evidence.json`
- Seed 43 offline comparison:
  `/var/tmp/physical-ai-p4.Kn4hFG/runs/seed43/offline_gt/offline_ground_truth_comparison.json`

## Historical P4.1 transport increment

The earlier clean P4.1 camera transport run at `feca6ba` first established the
separate RGB-D sensor, CameraInfo, and `world -> lab_camera_optical_frame`
contract. Its roots are `/var/tmp/physical-ai-p4p1.vkHkFS` and
`/var/tmp/physical-ai-p4p1-runtime.CY69QB`. It was transport-only and did not
by itself support A03; the final two-seed run above is the acceptance evidence.

## Capability boundary and next action

This result proves only the defined simulation-scene visual localization
baseline. It does not prove arbitrary scene robustness, graspability,
contact/force/attachment, lift/transport, pick/place, or physical-camera
performance. Phase 5 is not started. Stop after documenting this checkpoint
and await explicit user direction before any future Phase-5 work.
