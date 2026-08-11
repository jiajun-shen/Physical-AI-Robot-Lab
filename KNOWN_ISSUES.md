# Known Issues

| ID | Severity | Status | Description | Next evidence/action |
|---|---|---|---|---|
| KI-001 | Blocking Phase 1 | RESOLVED AUDIT | WSL2 Ubuntu 24.04 + Jazzy runtime is verified. | Keep the environment report current. |
| KI-002 | Blocking Phase 1 | PARTIAL | The pinned IRB1200 asset slice and project-owned Harmonic runtime passed; full `abb_ros2` still has no Jazzy support evidence. | Keep the upstream driver excluded; build the first-party MoveIt overlay. |
| KI-003 | Blocking Phase 1 | RESOLVED | Coherent Gazebo/MoveIt/control packages are installed; upstream and project headless action/joint-state smoke passed. | Keep apt manifest and runtime gates reproducible. |
| KI-004 | Model integrity | PARTIAL | Simulation position/velocity values are reconciled across the pinned URDF, canonical control table, and position command interfaces; inertials remain estimates, effort is fictional, and derivatives are undefined. | Derive and verify planning limits before Phase 2 planned/collision-aware motion; do not invent acceleration/jerk limits. |
| KI-005 | Resource | OPEN | WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound. | Start headless; monitor; use ext4 build trees; recommend 32 GiB host RAM. |
| KI-006 | GUI | OPEN | WSLg endpoints exist, but real RViz/Gazebo rendering is untested. | Run a minimal Harmonic/RViz GUI smoke after the headless planning baseline. |
| KI-007 | Filesystem | PARTIAL | DrvFS source with ext4 outputs passed the current three-package runtime gate, not the full stack. | Keep generated trees on ext4 and re-evaluate with simulator/MoveIt. |
| KI-008 | VLA | OPEN | Local environment is unsupported and no ABB-tuned checkpoint exists. | Keep unavailable; later test an isolated policy server on a compatible GPU. |
| KI-009 | Recovery | RESOLVED | `main` is pushed to private remote `jiajun-shen/Physical-AI-Robot-Lab`. | Keep pushes and clean checkpoints current; do not publish before license review. |
| KI-010 | Legal | OPEN | First-party repository license has not been selected. | Select before making the repository public; retain all third-party notices. |
| KI-011 | Safety | PARTIAL | A clean project run observed `enforce_command_limits=true`, six initialized limiters, and a bounded in-range trajectory. No out-of-range clamp behavior has been observed. | Keep bounded trajectory as control-chain evidence only; run a separate out-of-range clamp probe before making any clamp claim. |

No workaround is considered implemented until it has been run and inspected.
