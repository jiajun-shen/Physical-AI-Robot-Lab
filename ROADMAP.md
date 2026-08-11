# Roadmap

| Phase | Status | Evidence / next verifiable milestone |
|---:|---|---|
| Bootstrap | COMPLETE | Stable baseline commit `82971ce`; clean main; no remote |
| 0 Repository/environment/architecture | PARTIAL | Audits/reports written; consistency verification and commit pending |
| 1 ABB + Gazebo + ros2_control | NOT STARTED | Install coherent apt stack, pass upstream headless control smoke, then pin/import ABB assets |
| 2 MoveIt + planning | NOT STARTED | Collision-aware Home → Pose → Home verified |
| 3 Gripper + objects | NOT STARTED | Modular gripper and deterministic solvable seeded world verified |
| 4 Perception | NOT STARTED | Green sphere localized only from RGB-D and transformed by TF2 |
| 5 Pick/place | NOT STARTED | Green sphere placed next to red sphere across multiple seeds |
| 6 Drawing | NOT STARTED | Mathematical Cartesian heart path executes on board |
| 7 Speech | NOT STARTED | zh/en/fr microphone commands normalize and execute |
| 8 Task AI/executive | NOT STARTED | Long command executes stepwise with observe/verify/replan |
| 9 System Explorer | NOT STARTED | Full event/data lineage displayed honestly |
| 10 Robotics Core | NOT STARTED | First educational/backend parity benchmark passes |
| 11 VLA/openpi | NOT STARTED | Isolated real policy server or explicit unavailable state |
| 12 ABB VLA adapter | NOT STARTED | Unit/normalization/limits/watchdog tests pass |
| 13 Dataset | NOT STARTED | One valid converted demonstration episode |
| 14 Fine-tuning | NOT STARTED | Dataset/config validation and dry-run |
| 15 Mode benchmark | NOT STARTED | Honest reproducible CLASSICAL/HYBRID/VLA comparison |
| 16 Hardening/CI | NOT STARTED | Clean build/run/regression docs and acceptance ledger |

Detailed atomic next commands always live in `PROJECT_STATE.md`. A phase stays
`PARTIAL` until it is implemented, built, tested, run, inspected, documented,
and committed.

