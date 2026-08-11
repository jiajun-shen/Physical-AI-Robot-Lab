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

## ADR-0009: Local VLA remains unavailable by default

- Status: Accepted
- Decision: Do not install openpi into Jazzy Python and do not claim local
  inference. Use a separately pinned Ubuntu 22.04/Python 3.11 policy server only
  after a real smoke test. Until then expose `VLA_BACKEND_UNAVAILABLE`.
- Reason: Upstream does not support Ubuntu 24.04, the local 8151 MiB GPU is below
  the strict `> 8 GB` requirement, and no ABB-tuned checkpoint exists.
