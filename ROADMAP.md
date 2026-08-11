# Roadmap

| Phase | Status | Evidence / next verifiable milestone |
|---:|---|---|
| Bootstrap | COMPLETE | `main` is pushed to private remote `jiajun-shen/Physical-AI-Robot-Lab` |
| 0 Repository/environment/architecture | COMPLETE | Reports verified and committed at `eb1b8e8` |
| 1 ABB + Gazebo + ros2_control | COMPLETE | A01 clean project runtime gate passed at `1228c13` |
| 2 MoveIt + planning | COMPLETE | A02 clean real `move_group` plan/execution plus planning-scene collision rejection passed at `4203e81`; teardown SIGSEGV remains KI-012 |
| 3 Gripper + objects | NOT STARTED | Modular gripper and deterministic solvable seeded world verified |
| 4 Perception | NOT STARTED | Green sphere localized only from RGB-D and transformed by TF2 |
| 5 Pick/place | NOT STARTED | Green sphere placed next to red sphere across multiple seeds |
| 6 Drawing | NOT STARTED | Mathematical Cartesian heart path executes on board |
| 7 Speech | NOT STARTED | zh/en/fr microphone commands normalize and execute |
| 8 Task AI/executive | NOT STARTED | Long command executes stepwise with observe/verify/replan |
| M1 Midterm acceptance gate | PENDING | Lifecycle: `PENDING -> PAUSED - AWAITING USER MIDTERM ACCEPTANCE -> ACCEPTED`. All 12 targets must pass as real integrated runs on the tested Phase-8-final commit, then await explicit user acceptance. |
| 9 System Explorer | BLOCKED BY M1 | May start only after the user marks M1 `ACCEPTED`; it never auto-starts after tests pass. |
| 10 Robotics Core | NOT STARTED | First educational/backend parity benchmark passes |
| 11 VLA/openpi | NOT STARTED | Isolated real policy server or explicit unavailable state |
| 12 ABB VLA adapter | NOT STARTED | Unit/normalization/limits/watchdog tests pass |
| 13 Dataset | NOT STARTED | One valid converted demonstration episode |
| 14 Fine-tuning | NOT STARTED | Dataset/config validation and dry-run |
| 15 Mode benchmark | NOT STARTED | Honest reproducible CLASSICAL/HYBRID/VLA comparison |
| 16 Hardening/CI | NOT STARTED | Clean build/run/regression docs and acceptance ledger |

Detailed atomic next commands live in `PROJECT_STATE.md`. A phase stays
`PARTIAL` until it is implemented, built, tested, run, inspected, documented,
and committed. M1 is stricter: code or unit tests alone cannot advance it.
