# Known Issues

| ID | Severity | Status | Description | Next evidence/action |
|---|---|---|---|---|
| KI-001 | Blocking Phase 1 | RESOLVED AUDIT | Host runtime was unknown. WSL2 Ubuntu 24.04 + Jazzy is verified. | Keep exact environment report current. |
| KI-002 | Blocking Phase 1 | PARTIAL | IRB1200 assets and license are verified, but full `abb_ros2` has no Jazzy support evidence. | Pin/import assets only; build Jazzy/Harmonic overlay. |
| KI-003 | Blocking Phase 1 | OPEN | Gazebo, MoveIt, ros2_control/controllers, and gz_ros2_control are not installed. | Capture apt candidates, install coherent snapshot, run upstream headless smoke. |
| KI-004 | Model integrity | OPEN | ABB inertials are mesh estimates, efforts are fictional, and limit sources conflict. | Add automated limit/frame/inertial validation before spawn. |
| KI-005 | Resource | OPEN | WSL has 7.4 GiB RAM; combined simulator/MoveIt/RViz may be memory-bound. | Start headless; monitor; use ext4 build trees; recommend 32 GiB host RAM. |
| KI-006 | GUI | OPEN | WSLg endpoints exist, but real RViz/Gazebo rendering is untested. | Run a minimal Harmonic/RViz GUI smoke after install. |
| KI-007 | Filesystem | OPEN | Source is on `/mnt/c`; colcon I/O/symlink behavior is untested. | Build a minimal package; move only build/install/log to ext4 if needed. |
| KI-008 | VLA | OPEN | Local environment is unsupported and no ABB-tuned checkpoint exists. | Keep unavailable; later test isolated policy server on compatible GPU. |
| KI-009 | Recovery | OPEN | Repository has no Git remote, so commits are only on local disk. | Push when the user configures a remote; do not invent one. |
| KI-010 | Legal | OPEN | First-party repository license has not been selected. | Select before public distribution; retain all third-party notices. |

No workaround is considered implemented until it has been run and inspected.

