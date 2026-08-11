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

