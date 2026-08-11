# Known Issues

| ID | Severity | Status | Description | Next evidence/action |
|---|---|---|---|---|
| KI-001 | Blocking Phase 1 | RESOLVED AUDIT | Host runtime was unknown. WSL2 Ubuntu 24.04 + Jazzy is verified. | Keep exact environment report current. |
| KI-002 | Blocking Phase 1 | PARTIAL | The pinned IRB1200 asset slice and project-owned static Harmonic control description are structurally verified; full `abb_ros2` still has no Jazzy support evidence. | Keep the upstream driver excluded; verify the project-owned Gazebo runtime in P1.3b. |
| KI-003 | Blocking Phase 1 | RESOLVED | Coherent Gazebo/MoveIt/control packages are installed; environment and real upstream action/joint-state smoke passed. | Keep apt manifest and smoke scripts reproducible. |
| KI-004 | Model integrity | PARTIAL | Simulation position/velocity values are reconciled across the pinned URDF, a canonical control table, and position command interfaces; inertials remain estimates, effort is fictional, derivatives are undefined, and MoveIt limits do not exist. | Runtime-test command enforcement, then derive and verify planning limits before Phase 2 planned/collision-aware motion. |
| KI-005 | Resource | OPEN | WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound. | Start headless; monitor; use ext4 build trees; recommend 32 GiB host RAM. |
| KI-006 | GUI | OPEN | WSLg endpoints exist, but real RViz/Gazebo rendering is untested. | Run a minimal Harmonic/RViz GUI smoke after install. |
| KI-007 | Filesystem | PARTIAL | DrvFS source with ext4 `/tmp` build/install/log passed for the small description package; the full stack is untested. | Keep generated trees on ext4 and re-evaluate with simulator/MoveIt. |
| KI-008 | VLA | OPEN | Local environment is unsupported and no ABB-tuned checkpoint exists. | Keep unavailable; later test isolated policy server on compatible GPU. |
| KI-009 | Recovery | OPEN | Repository has no Git remote, so commits are only on local disk. | Push when the user configures a remote; do not invent one. |
| KI-010 | Legal | OPEN | First-party repository license has not been selected. | Select before public distribution; retain all third-party notices. |
| KI-011 | Safety | PARTIAL | The project YAML configures `controller_manager.enforce_command_limits: true`, but no project Gazebo/controller runtime or out-of-range clamp has been observed. | In P1.3b verify the true runtime parameter and six limiters; treat the bounded trajectory only as control-chain evidence, and verify clamping with a separate out-of-range probe. |

No workaround is considered implemented until it has been run and inspected.
