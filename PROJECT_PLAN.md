# Project Plan

## Delivery discipline

Development uses atomic, recoverable increments:

`PLAN -> IMPLEMENT -> BUILD -> TEST -> RUN -> INSPECT -> FIX -> REGRESSION -> DOCUMENT -> COMMIT`

No phase is complete until its implementation has been built, tested, run,
inspected, documented, and committed. Partial results remain marked `PARTIAL`.
Phase 8 has an additional mandatory stop: after its real-run exit evidence is
complete, development must pause for the user's midterm acceptance before any
Phase 9 work begins.

## Phase sequence

| Phase | Scope | Required exit evidence |
|---:|---|---|
| 0 | Repository, host/dependency audit, architecture | Reports verified; baseline commits; no unsupported compatibility claims |
| 1 | ABB IRB1200 + Gazebo + ros2_control | Real project ABB model spawns; six joints identified; controller exchanges actual joint state/trajectory |
| 2 | MoveIt + collision-aware planning | Joint and pose targets plan and execute with planning-scene collision checks |
| 3 | Modular two-finger gripper + randomized lab objects | Command/state/collision model works; deterministic seeded scene |
| 4 | RGB-D perception | Real RGB/depth projection and TF transform locate required objects in vision mode |
| 5 | Verified pick/place | Multi-stage collision-aware grasp succeeds over multiple seeds |
| 6 | Parametric drawing | Generated Cartesian heart/path executes; scale changes geometry |
| 7 | zh/en/fr speech | Microphone pipeline emits normalized `SpeechCommand` with honest confidence/language |
| 8 | Task AI + long-horizon executive | Constrained multilingual TaskGraph; observe/verify/replan loop |
| M1 | Midterm acceptance stop | All 12 targets pass as real integrated runs on the tested Phase-8-final commit; pause and wait for explicit user acceptance before Phase 9 |
| 9 | System Explorer | Live data lineage and backend/health/latency/failure state are inspectable |
| 10 | Educational robotics algorithms | Declared algorithms each have implementation, tests, examples, docs, and maturity labels |
| 11 | VLA interface + isolated openpi policy server | Real backend starts when supported; otherwise explicit unavailable state |
| 12 | ABB observation/action/safety adapters | Units, normalization, clipping, limits, watchdog, and action representation verified |
| 13 | Demonstration data pipeline | Successful ABB simulation episode recorded, validated, visualized, converted |
| 14 | Fine-tuning pipeline | Config validation and dry-run; optional small overfit only when hardware permits |
| 15 | CLASSICAL/HYBRID/VLA benchmark | Reproducible metrics with unavailable/experimental modes clearly separated |
| 16 | Hardening, documentation, CI | Clean rebuild, regression, acceptance ledger, operational docs |

## Immediate work packages

1. `P1.1` coherent Jazzy/Harmonic/MoveIt/control dependency installation and
   upstream action smoke: `COMPLETE`, committed at `14d946f`.
2. `P1.2` full-SHA ABB asset subset, geometry-only package, Xacro/URDF/resource
   contracts: `COMPLETE`, committed at `2a536eb`.
3. `P1.3a` fixed-base control-description contract: one world joint, six
   control joints, canonical limits, startup enforcement configured true,
   `GazeboSimSystem`, and Gazebo resource export: `COMPLETE`, committed at
   `2ec4330`.
4. `P1.3b` controller YAML, minimal world, headless launch, controller/resource
   inspection, and bounded observed trajectory: `COMPLETE`, committed at
   `1228c13`; a clean real runtime gate observed the ABB spawn, active
   controllers, initialized limiters, bounded trajectory, and cleanup.
5. `P2.1` project-owned MoveIt configuration contract: `COMPLETE`, committed at
   `281c36b`; six-joint semantics, canonical position/velocity limits, KDL,
   OMPL collision adapters, and the existing JTC action mapping pass installed
   static tests without claiming planning or execution.
6. `P2.2` headless MoveIt/Gazebo integration: `COMPLETE`, committed at
   `4203e81`; a clean real run used one existing Gazebo/controller-manager
   chain plus one `move_group`, planned/executed a joint route and
   `HOME -> FK-derived tool0 Pose -> HOME`, and proved planning-scene collision
   rejection before marking A02 `PASS`. No owned process remained, but the
   installed `move_group` process had a SIGSEGV during requested teardown;
   this remains KI-012, not a graceful-cleanup claim.
7. `P3.1` modular two-finger gripper and first live laboratory objects: next.
   It must establish a project-owned gripper command/state/collision contract
   and a reproducible running world before attempting pick/place.

## M1 release rule

After Phase 8, the project state becomes `PAUSED - AWAITING USER MIDTERM
ACCEPTANCE`. Phase 9 is locked until both all M1 rows have `PASS` evidence from
the immutable tested Phase-8-final commit and the user explicitly accepts the
midterm review. No test suite, task executive, or model may release this hold.
