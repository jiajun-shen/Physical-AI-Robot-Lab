# Physical-AI-Robot-Lab

A long-term robotics research repository for an ABB IRB1200-5/0.9 virtual
laboratory, multilingual task execution, classical manipulation, and an honest
path toward ABB-adapted vision-language-action research.

The project has completed its verified Phase 0 audit and is in Phase 1. The
project-owned ABB IRB1200 now has a real, clean, headless Gazebo Harmonic and
ros2_control runtime result: the exact model spawned, six position interfaces
were claimed, joint-state and trajectory controllers became active, command
limiters initialized, a bounded six-axis trajectory returned `SUCCEEDED` with
`error_code: 0`, and cleanup was verified. This establishes A01 only; it does
not establish MoveIt planning, collision avoidance, a gripper, perception,
speech, TaskGraph execution, VLA readiness, GUI behavior, or real-robot safety.
An arm-only MoveIt configuration contract is installed and static-tested, but
its real `move_group` planning/execution integration remains the next gate.
See `PROJECT_STATE.md`, `ACCEPTANCE_STATUS.md`, and
`docs/phase1_abb_gazebo_control_runtime_evidence.md` for exact evidence.

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
