# Architecture Decision Log

## ADR-0001: Establish a classical working baseline first

- Status: Accepted
- Decision: Deliver simulation, ros2_control, MoveIt, perception, and verified
  skills before expanding VLA authority.
- Reason: Advanced backends must not make the base laboratory unavailable or
  conceal failures.

## ADR-0002: Keep ROS and VLA dependency environments isolated

- Status: Accepted
- Decision: A typed, versioned policy service separates the ROS runtime from
  openpi. The bridge may run locally or over a controlled network link.
- Reason: ROS distribution Python constraints and fast-moving ML dependencies
  have different compatibility and GPU lifecycles.

## ADR-0003: Default routing mode is HYBRID with explicit fallback truth

- Status: Accepted
- Decision: Classical planning/control owns safety and baseline execution.
  VLA subskills are opt-in and availability-gated. Missing/incompatible VLA
  yields `VLA_BACKEND_UNAVAILABLE` and a labelled classical fallback.

## ADR-0004: Separate truth sources in the world model

- Status: Accepted
- Decision: `ground_truth` is restricted to debugging, validation, and tests.
  The formal demo defaults to `vision`, which cannot read simulator poses.

## ADR-0005: Upstream robot assets must be pinned and license-audited

- Status: Accepted
- Decision: Do not draw a substitute ABB model. Verify and reuse lawful ABB
  IRB1200 description/meshes/limits/configuration from current upstream sources.

## ADR-0006: No direct LLM or VLA motor commands

- Status: Accepted
- Decision: Language models emit validated TaskGraph/Skill/Goal structures;
  VLA action chunks pass through ABB mapping, limits, collision/workspace
  checks, rate limits, and watchdogs before controller execution.

## ADR-0007: Use the official Jazzy/Harmonic apt pairing

- Status: Accepted
- Decision: Install Gazebo bridge/vendor packages, MoveIt, ros2_control,
  ros2_controllers, and gz_ros2_control from the configured ROS Jazzy apt
  repository and record exact candidates/installed versions.
- Reason: Ubuntu 24.04 + Jazzy + Harmonic is the supported pairing. Mixing
  Rolling, Gazebo Classic, or a non-default Gazebo repository creates avoidable
  ABI and package conflicts.

## ADR-0008: Reuse ABB assets through a project-owned Jazzy overlay

- Status: Accepted
- Decision: Pin `PickNikRobotics/abb_ros2` commit
  `05cbabe774ef026b1cd36fe6b253b55f55e16340`, preserve asset licenses, and use
  only the verified IRB1200 geometry/kinematic material as the Phase 1 source.
  Implement new Harmonic control composition and independently validated
  MoveIt configuration in first-party packages.
- Reason: Upstream has no Jazzy branch/tag, tests only Rolling, lacks a
  gz_ros2_control plugin/gripper, and contains conflicting or approximate
  physical limits. Its complete driver cannot be called Jazzy-compatible.
- Implementation note (2026-08-11): The asset subset is vendored under a
  full-SHA/per-file hash contract. The project-owned `abb_lab_description`
  package changes only resource paths, avoids upstream package-name shadowing,
  and deliberately omits all control/bringup/MoveIt entry points. Vendored
  inputs, locks, tests, and notices are package-local so the ROS package has no
  hidden dependency on its repository depth.

## ADR-0009: Local VLA remains unavailable by default

- Status: Accepted
- Decision: Do not install openpi into Jazzy Python and do not claim local
  inference. Use a separately pinned Ubuntu 22.04/Python 3.11 policy server only
  after a real smoke test. Until then expose `VLA_BACKEND_UNAVAILABLE`.
- Reason: Upstream does not support Ubuntu 24.04, the local 8151 MiB GPU is below
  the strict `> 8 GB` requirement, and no ABB-tuned checkpoint exists.

## ADR-0010: Keep geometry provenance separate from simulation control

- Status: Accepted
- Decision: Preserve the P1.2 geometry Xacro as an exact audited derivative and
  add Gazebo Harmonic control only through a separate project-owned composition.
  Use one simulation-only position/velocity table to verify the pinned URDF and
  ros2_control position interface bounds. Configure controller-manager command
  enforcement at startup, but do not call limiter initialization verified until
  a project runtime exposes the true parameter and the expected six limiters.
  A bounded in-range trajectory proves the control chain, not clamping; clamp
  verification requires a separately observed out-of-range injection.
- Reason: Editing the source-derived geometry to consume operational policy
  would weaken its byte-equivalence proof. Static configuration, runtime
  initialization, and physical-robot safety are different evidence levels and
  must remain distinguishable.
- P1.3a boundary (2026-08-11): No Gazebo launch, controller activation,
  trajectory, MoveIt configuration, acceleration/jerk value, or real-robot
  readiness is claimed.
- P1.3b evidence (2026-08-11): A clean project runtime at `1228c13` verified
  controller-manager command-limit enforcement, six limiter initializations,
  and a bounded in-range trajectory. It does not verify out-of-range clamping,
  collision-aware planning, GUI behavior, or real-robot safety.

## ADR-0011: Mandatory M1 midterm hold before System Explorer

- Status: Accepted
- Decision: Completion of Phase 8 enters `PAUSED - AWAITING USER MIDTERM
  ACCEPTANCE`. M1's twelve required capabilities must each have saved,
  reproducible real integrated-run evidence on the tested Phase-8-final commit.
  Phase 9 may begin only after the user explicitly accepts M1; tests, the task
  executive, and automated workflows cannot release this hold.
- Reason: System Explorer or later features must not mask an incomplete
  manipulation, perception, speech, language, or closed-loop executive
  baseline. The hold makes the user's hands-on midterm review a control-plane
  decision rather than an inferred code state.

## ADR-0012: Retiming MoveIt trajectories from verified velocity limits only

- Status: Accepted
- Decision: For the P2.2 simulation verifier, omit MoveIt's time-optimal
  response adapter because the project has no verified acceleration,
  deceleration, or jerk authority. Retiming occurs after successful geometric
  planning using only the canonical six-axis velocity limits, a conservative
  safety factor, and a finite segment-duration floor; derivative arrays remain
  empty.
- Reason: Adding invented dynamic limits merely to satisfy a time-parameterizer
  would create an unsupported safety/physical claim. The clean `4203e81`
  runtime evidence records this velocity-only policy and
  `acceleration_limits_used: false`.
- Boundary: This schedules simulation trajectories conservatively; it does not
  validate actuator dynamics, physical ABB timing, or out-of-range clamp
  behavior.
