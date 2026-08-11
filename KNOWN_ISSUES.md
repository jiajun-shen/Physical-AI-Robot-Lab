# Known Issues

| ID | Severity | Status | Description | Next evidence/action |
|---|---|---|---|---|
| KI-001 | Blocking Phase 1 | RESOLVED AUDIT | WSL2 Ubuntu 24.04 + Jazzy runtime is verified. | Keep the environment report current. |
| KI-002 | Blocking Phase 1 | PARTIAL | The pinned IRB1200 asset slice and first-party Harmonic/MoveIt overlay passed; the full `abb_ros2` driver still has no Jazzy support evidence. | Keep the upstream driver excluded and continue using the project-owned overlay. |
| KI-003 | Blocking Phase 1 | RESOLVED | Coherent Gazebo/MoveIt/control packages are installed; the project P1.3b and P2.2 clean headless runtime gates passed. | Keep apt manifest and runtime gates reproducible. |
| KI-004 | Model integrity | PARTIAL | Simulation position/velocity values are reconciled across the pinned URDF, canonical control table, position command interfaces, and MoveIt limits. P2.2 planned/executed using conservative velocity-only retiming; inertials remain estimates, effort is fictional, and derivatives are undefined. | Do not invent acceleration/jerk limits; establish physical/dynamic authority separately. |
| KI-005 | Resource | OPEN | WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound. | Start headless; monitor; use ext4 build trees; recommend 32 GiB host RAM. |
| KI-006 | GUI | OPEN | WSLg endpoints exist, but real RViz/Gazebo rendering is untested. | Run a minimal Harmonic/RViz GUI smoke after the headless planning baseline. |
| KI-007 | Filesystem | PARTIAL | DrvFS source with ext4 outputs passed clean P1.3b and four-package P2.2 simulator/MoveIt gates. | Keep generated trees on ext4; re-evaluate with perception, GUI, and larger integrated worlds. |
| KI-008 | VLA | OPEN | Local environment is unsupported and no ABB-tuned checkpoint exists. | Keep unavailable; later test an isolated policy server on a compatible GPU. |
| KI-009 | Recovery | RESOLVED | `main` is pushed to private remote `jiajun-shen/Physical-AI-Robot-Lab`. | Keep pushes and clean checkpoints current; do not publish before license review. |
| KI-010 | Legal | OPEN | First-party repository license has not been selected. | Select before making the repository public; retain all third-party notices. |
| KI-011 | Safety | PARTIAL | A clean project run observed `enforce_command_limits=true`, six initialized limiters, and a bounded in-range trajectory. No out-of-range clamp behavior has been observed. | Keep bounded trajectory as control-chain evidence only; run a separate out-of-range clamp probe before making any clamp claim. |
| KI-012 | Runtime teardown | OPEN | On clean P2.2 run `4203e81`, the installed MoveIt 2.12.4 `move_group` process received requested SIGINT during harness teardown and exited with SIGSEGV. The owned process group and selected-domain daemon were absent afterward, but this is not graceful cleanup. | Reproduce/minimize and resolve or explicitly contain the shutdown failure before M1/Phase 16 hardening; keep the abnormal marker in all P2.2 evidence. |

No workaround is considered implemented until it has been run and inspected.
