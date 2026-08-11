# Physical-AI-Robot-Lab

A long-term robotics research repository for an ABB IRB1200-5/0.9 virtual
laboratory, multilingual task execution, classical manipulation, and an
honest path toward ABB-adapted vision-language-action research.

The repository has completed its verified Phase 0 environment, dependency, and
architecture audit and has entered Phase 1. The installed upstream Gazebo /
gz_ros2_control dependency path has passed a real headless trajectory smoke,
and the pinned ABB IRB1200 package now passes both its immutable geometry
contract and a static fixed-base Harmonic control-description contract. The
project control description has not been spawned, no project controller has
been activated, and command-limit enforcement has not been observed at
runtime. No ABB Gazebo runtime, project laboratory, perception, speech, or VLA
capability is claimed working yet. See
`PROJECT_STATE.md` for the exact tested state and next commands.

The local WSL2 installation now has Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
MoveIt, and the control stack at recorded apt versions. The ABB asset source is
pinned, but its complete upstream driver is not treated as Jazzy-compatible.
See `docs/environment_report.md`, `docs/upstream_compatibility_report.md`, and
`docs/phase1_dependency_smoke.md`. ABB asset/build evidence is in
`docs/phase1_abb_asset_evidence.md`; the separate pre-runtime control evidence
is in `docs/phase1_abb_control_description_evidence.md`.

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
