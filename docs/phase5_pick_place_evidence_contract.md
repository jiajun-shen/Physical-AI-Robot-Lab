# Phase 5 — Verified Pick/Place Evidence Contract

Phase 5 may be marked complete only from saved multi-seed Gazebo runtime evidence. Source code, a gripper action result, or visible animation alone is insufficient.

## Required runtime sequence

Each accepted episode must preserve timestamped records for:

1. `OBSERVE_PICK` — fresh vision-only green/red poses from RGB-D + CameraInfo + TF; no Gazebo model pose input.
2. `PLAN_APPROACH` — collision-aware arm plan to a pre-grasp pose.
3. `OPEN` — gripper commanded open and observed open.
4. `APPROACH` — collision-aware approach to the grasp pose.
5. `CLOSE` — gripper commanded closed around the green sphere.
6. `LIFT_VERIFY` — the green sphere rises with the gripper by at least 0.05 m; this is the minimum retention/grasp proof when no trustworthy force signal exists.
7. `TRANSPORT` — collision-aware motion carries the retained object toward the red sphere.
8. `LOWER` — controlled descent to the placement pose.
9. `RELEASE` — gripper opens and the green sphere remains supported by the table.
10. `RETREAT` — collision-aware retreat without disturbing either sphere.
11. `OBSERVE_PLACE` — fresh vision-only green/red poses after release.
12. `VERIFY_PLACE` — planar green/red centre distance is <= 0.12 m while sphere overlap is avoided.

## Safety / integrity rules

- Simulation only. This contract is not real-robot safety approval.
- Arm motion must go through MoveIt collision checking and the existing bounded ros2_control chain.
- No grasp claim is permitted from finger position alone.
- If force/effort remains untrustworthy, a successful lift/transport with measured object motion is the grasp evidence.
- Vision-mode decisions must not consume Gazebo ground truth. Gazebo pose capture is allowed only after the episode is sealed, for offline evaluation.
- A stale Phase-4 object pose cannot be reused after any manipulation step that can move an object.
- Timeout, planning failure, loss of object, collision rejection, stale observation, and placement verification failure are explicit failures.

## Multi-seed exit gate

At least two independently launched seeded worlds must pass the full sequence. A04 remains `NOT STARTED` until these real runs exist.
