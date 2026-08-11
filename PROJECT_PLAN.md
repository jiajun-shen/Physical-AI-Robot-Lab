# Project Plan

## Delivery discipline

Development uses atomic, recoverable increments:

`PLAN -> IMPLEMENT -> BUILD -> TEST -> RUN -> INSPECT -> FIX -> REGRESSION -> DOCUMENT -> COMMIT`

No phase is complete until its implementation has been built, tested, run,
inspected, documented, and committed. Partial results remain marked `PARTIAL`.

## Phase sequence

| Phase | Scope | Required exit evidence |
|---:|---|---|
| 0 | Repository, host/dependency audit, architecture | Reports verified; baseline commits; no unsupported compatibility claims |
| 1 | ABB IRB1200 + Gazebo + ros2_control | Real upstream model spawns; six joints identified; controller exchanges actual joint state/trajectory |
| 2 | MoveIt + collision-aware planning | Joint and pose targets plan and execute with planning-scene collision checks |
| 3 | Modular two-finger gripper + randomized lab objects | Command/state/collision model works; deterministic seeded scene |
| 4 | RGB-D perception | Real RGB/depth projection and TF transform locate required objects in vision mode |
| 5 | Verified pick/place | Multi-stage collision-aware grasp succeeds over multiple seeds |
| 6 | Parametric drawing | Generated Cartesian heart/path executes; scale changes geometry |
| 7 | zh/en/fr speech | Microphone pipeline emits normalized `SpeechCommand` with honest confidence/language |
| 8 | Task AI + long-horizon executive | Constrained multilingual TaskGraph; observe/verify/replan loop |
| 9 | System Explorer | Live data lineage and backend/health/latency/failure state are inspectable |
| 10 | Educational robotics algorithms | Declared algorithms each have implementation, tests, examples, docs, and maturity labels |
| 11 | VLA interface + isolated openpi policy server | Real backend starts when supported; otherwise explicit unavailable state |
| 12 | ABB observation/action/safety adapters | Units, normalization, clipping, limits, watchdog, and action representation verified |
| 13 | Demonstration data pipeline | Successful ABB simulation episode recorded, validated, visualized, converted |
| 14 | Fine-tuning pipeline | Config validation and dry-run; optional small overfit only when hardware permits |
| 15 | CLASSICAL/HYBRID/VLA benchmark | Reproducible metrics with unavailable/experimental modes clearly separated |
| 16 | Hardening, documentation, CI | Clean rebuild, regression, acceptance ledger, operational docs |

## Immediate Phase 0 work packages

1. `P0.1` repository and preservation audit.
2. `P0.2` host hardware/software audit.
3. `P0.3` official upstream compatibility and license audit.
4. `P0.4` package/dataflow/interface design.
5. `P0.5` documentation validation and checkpoint.

