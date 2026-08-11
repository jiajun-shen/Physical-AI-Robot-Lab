# Known Issues

| ID | Severity | Status | Description | Next evidence/action |
|---|---|---|---|---|
| KI-001 | Blocking Phase 1 | RESOLVED AUDIT | Host runtime was unknown. WSL2 Ubuntu 24.04 + Jazzy is verified. | Keep exact environment report current. |
| KI-002 | Blocking Phase 1 | PARTIAL | The pinned IRB1200 asset slice is built and structurally verified; full `abb_ros2` still has no Jazzy support evidence. | Keep the driver excluded; build project-owned Jazzy/Harmonic control. |
| KI-003 | Blocking Phase 1 | RESOLVED | Coherent Gazebo/MoveIt/control packages are installed; environment and real upstream action/joint-state smoke passed. | Keep apt manifest and smoke scripts reproducible. |
| KI-004 | Model integrity | PARTIAL | Frame/limit/inertial structure is automated, but inertials remain estimates, efforts fictional, and operational limits uncoordinated. | Establish one enforced URDF/control/MoveIt limit source before motion. |
| KI-005 | Resource | OPEN | WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound. | Start headless; monitor; use ext4 build trees; recommend 32 GiB host RAM. |
| KI-006 | GUI | OPEN | WSLg endpoints exist, but real RViz/Gazebo rendering is untested. | Run a minimal Harmonic/RViz GUI smoke after install. |
| KI-007 | Filesystem | PARTIAL | DrvFS source with ext4 `/tmp` build/install/log passed for the small description package; the full stack is untested. | Keep generated trees on ext4 and re-evaluate with simulator/MoveIt. |
| KI-008 | VLA | OPEN | Local environment is unsupported and no ABB-tuned checkpoint exists. | Keep unavailable; later test isolated policy server on compatible GPU. |
| KI-009 | Recovery | OPEN | Repository has no Git remote, so commits are only on local disk. | Push when the user configures a remote; do not invent one. |
| KI-010 | Legal | OPEN | First-party repository license has not been selected. | Select before public distribution; retain all third-party notices. |
| KI-011 | Safety | OPEN | Upstream gz_ros2_control demo reports command-limit enforcement disabled. | Project ABB overlay must enable/test limits and reconcile URDF/control/MoveIt values. |

No workaround is considered implemented until it has been run and inspected.
