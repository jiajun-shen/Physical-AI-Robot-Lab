# Physical-AI-Robot-Lab

A long-term robotics research repository for an ABB IRB1200-5/0.9 virtual
laboratory, multilingual task execution, classical manipulation, and an
honest path toward ABB-adapted vision-language-action research.

The repository has completed its verified Phase 0 environment, dependency, and
architecture audit and has entered Phase 1. No robot, simulator, perception,
speech, or VLA capability is claimed working yet. See
`PROJECT_STATE.md` for the exact tested state and next commands.

Verified audit result: the local WSL2 installation has Ubuntu 24.04 and ROS 2
Jazzy, while Gazebo Harmonic, MoveIt, and the control stack still need to be
installed. The ABB asset source is pinned, but its complete upstream driver is
not treated as Jazzy-compatible. See `docs/environment_report.md` and
`docs/upstream_compatibility_report.md`.

## Engineering order

1. Establish a reproducible ROS 2 / Gazebo / MoveIt baseline.
2. Build verified classical skills and closed-loop task execution.
3. Integrate VLA inference behind explicit ABB observation, action, and safety
   adapters without misrepresenting an unadapted checkpoint as ABB-ready.

## Recovery entry point

Every development session starts by reading:

1. `PROJECT_STATE.md`
2. `ARCHITECTURE.md`
3. `ROADMAP.md`
4. `DECISIONS.md`
5. Git status and history
