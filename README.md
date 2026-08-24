# Physical-AI-Robot-Lab

A long-term robotics research repository for an ABB IRB1200-5/0.9 virtual
laboratory, multilingual task execution, classical manipulation, and an honest
path toward ABB-adapted vision-language-action research.

The project has completed verified Phase 0 through Phase 6. The project-owned
ABB IRB1200 has real
clean headless Gazebo Harmonic, ros2_control, and MoveIt evidence: the exact
model spawned, six position interfaces were claimed, JSB/JTC became active,
command limiters initialized, and MoveIt planned/executed a joint route plus
`HOME -> FK-derived tool0 pose -> HOME`. A planning-scene object made the
target invalid and both pose and joint planning requests were rejected.

Phase 3 then ran the ABB-with-gripper profile in two separately launched,
clean seeded Gazebo worlds (seeds 42 and 43) on commit `f6907ff`. Both runs
observed the ten required laboratory models and captured their live Gazebo
poses, proved material seeded object-position variation and directional-cube
yaw variation, and completed a real free-space gripper
`open -> closed clearance -> reopen` action sequence with mirrored finger
states. This P3 evidence is not a contact, force, attachment, grasp,
arm-in-lab MoveIt, pick/place, GUI, or real-robot result. The user explicitly
accepted the saved Phase-3 evidence on 2026-08-14, authorizing Phase 4
perception work.

Phase 4 then completed a project-owned simulation RGB-D pipeline at clean
feature commit `62dc9c8`. Two isolated seeded worlds (42 and 43) supplied
real RGB, 32FC1 depth, CameraInfo, and TF to the vision node only. It emitted
typed detections/poses for green/red balls, blue/yellow cubes, and the drawing
board, sealed `ground_truth_input_used: false` evidence before a separate
offline live-Gazebo comparison, and passed every localization threshold.
A03 is `PASS`; the green-ball errors were 0.0103339 m and 0.0150882 m. This
does not change A04 or establish contact, grasp, lift, transport, placement,
arbitrary-scene robustness, GUI, or real-camera evidence. See
`PROJECT_STATE.md`, `ACCEPTANCE_STATUS.md`, and
`docs/phase4_perception_runtime_evidence.md`.

Phase 5 completed verified two-seed RGB-D/MoveIt pick-place at `7e682d4` and
made A04 `PASS`; its evidence remains separately scoped to simulated visual
lift, retention, placement, and fresh post-place observation. Phase 6 completed
generative drawing at `eba23f1`: three independently launched Gazebo/MoveIt/
RGB-D episodes executed a runtime-generated centered circle, left square, and
left circle, covering a curve, polygon, and board-position variation. The
independent evaluator, source-manifest check, cleanup gates, and 85 tests all
passed, making A05 `PASS`. See `docs/phase5_pick_place_runtime_evidence.md` and
`docs/phase6_drawing_runtime_evidence.md`.

The clean P2.2 run found no residual owned process, but observed the installed
`move_group` process exit with SIGSEGV during requested teardown. That is
open KI-012, not a graceful-shutdown claim, and must be resolved or otherwise
qualified before M1/Phase 16 hardening.

The local WSL2 installation has Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
MoveIt, and the control stack at recorded apt versions. ABB asset provenance is
pinned, but the complete upstream driver is not treated as Jazzy-compatible.
The repository is currently private at
`https://github.com/jiajun-shen/Physical-AI-Robot-Lab`; publication decisions
remain gated by the unresolved first-party license.

## Engineering order

1. Establish a reproducible ROS 2 / Gazebo / MoveIt baseline.
2. Build verified classical skills and closed-loop task execution.
3. Integrate VLA inference behind explicit ABB observation, action, and safety
   adapters without misrepresenting an unadapted checkpoint as ABB-ready.

## Phase-3 acceptance record

The user explicitly accepted the saved final Phase-3 real-run evidence on
2026-08-14. Phase 4 subsequently completed at `62dc9c8` (A03), Phase 5 at
`7e682d4` (A04), and Phase 6 at `eba23f1` (A05). No result here replaces the
separate Phase-8 M1 hold below.

## Mandatory midterm hold

Development proceeds autonomously through Phase 8. When Phase 8 has real-run
evidence, the project must stop before Phase 9 and wait for the user's explicit
midterm acceptance. All twelve M1 capabilities in `ACCEPTANCE_STATUS.md` must
pass from saved real integrated runs on the tested final commit; code presence,
unit tests, or static configuration are not sufficient.

## Recovery entry point

Every development session starts by reading:

1. `PROJECT_STATE.md`
2. `ARCHITECTURE.md`
3. `ROADMAP.md`
4. `DECISIONS.md`
5. Git status and history
