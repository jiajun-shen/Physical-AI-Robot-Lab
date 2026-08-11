# Physical-AI-Robot-Lab

A long-term robotics research repository for an ABB IRB1200-5/0.9 virtual
laboratory, multilingual task execution, classical manipulation, and an honest
path toward ABB-adapted vision-language-action research.

The project has completed verified Phase 0, Phase 1, and Phase 2 gates. The
project-owned ABB IRB1200 has real clean headless Gazebo Harmonic,
ros2_control, and MoveIt evidence: the exact model spawned, six position
interfaces were claimed, JSB/JTC became active, command limiters initialized,
and MoveIt planned/executed a joint route plus `HOME -> FK-derived tool0 pose
-> HOME`. A planning-scene object made the target invalid and both pose and
joint planning requests were rejected. This establishes A01 and A02 in
headless simulation only; it does not establish a gripper, lab objects,
perception, pick/place, drawing, speech, TaskGraph execution, VLA readiness,
GUI behavior, or real-robot safety. See `PROJECT_STATE.md`,
`ACCEPTANCE_STATUS.md`, and
`docs/phase2_moveit_runtime_evidence.md` for exact evidence.

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
