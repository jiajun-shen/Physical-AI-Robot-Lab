# Architecture

## Governing data flow

```text
Voice / Text
  -> Speech Recognition
  -> Provider-independent Language Planner
  -> constrained TaskGraph
  -> closed-loop Executive <-> World Model <-> Perception / Ground Truth
  -> Skill Router (CLASSICAL | HYBRID | VLA)
       -> Classical Skills -> MoveIt / ros2_control
       -> VLA Policy -> ABB Action Adapter -> Safety Filter
  -> Hardware Abstraction (SimulationHardware | RealABBHardware)
  -> Gazebo Harmonic or a separately authorized physical ABB
  -> Observe -> Verify -> Update -> Continue / Retry / Replan / Abort
```

## Non-negotiable boundaries

- The planner emits typed tasks, skills, and goals—not joint commands.
- Perception `vision` mode cannot access simulator object ground truth.
- Long-running robot work uses ROS 2 actions with feedback and cancellation.
- Skills own preconditions, execution, postconditions, verification, timeout,
  retries, and typed failure reasons.
- Collision, joint, velocity, workspace, and watchdog safety gates cannot be
  bypassed by either classical or VLA paths.
- An unadapted VLA checkpoint is labelled `EXPERIMENTAL / NOT ABB-TUNED`.
- A missing VLA backend produces `VLA_BACKEND_UNAVAILABLE`; HYBRID can fall
  back safely to CLASSICAL without presenting the fallback as VLA success.
- ROS 2 and openpi environments are isolated by a versioned policy-server IPC.

## Planned repository/package layout

```text
Physical-AI-Robot-Lab/
  docs/
    concepts/
  docker/
  config/
  scripts/
  ros2_ws/src/
    physical_ai_interfaces/       # msg, srv, action contracts
    abb_lab_description/          # upstream ABB composition + modular gripper
    abb_lab_gazebo/               # Harmonic worlds, spawning, randomization
    abb_lab_control/              # ros2_control and hardware abstraction
    abb_lab_moveit_config/        # SRDF, solvers, controllers, planning scene
    physical_ai_perception/       # RGB-D backends and TF-aware poses
    physical_ai_world_model/      # observed robot/object/surface state
    physical_ai_skills/           # schema-driven verified actions
    physical_ai_brain/            # TaskGraph planning/execution/recovery
    physical_ai_speech/           # microphone/VAD/Whisper/language normalization
    physical_ai_vla_bridge/       # ROS-side versioned policy client
    physical_ai_system_monitor/   # health, latency, events, backend truth
    physical_ai_bringup/          # composed launch and profiles
  robotics_core/                  # educational/production interfaces and tests
    math/
    kinematics/
    dynamics/
    trajectory/
    control/
    planning/
    collision/
  vla/
    interface/
    openpi/
    openvla_optional/
    abb_adapter/
    inference/
    datasets/
    training/
    evaluation/
    safety/
  system_explorer/                # later UI, not a source of control truth
  simulation/                     # expert rollout and episode collection tools
  benchmarks/                     # reproducible tasks, metrics, plots
  tests/
  runs/                            # ignored runtime episodes, never Git data
```

The layout is a target architecture, not a claim that these packages exist.
Packages are created only in tested atomic increments.

## Implemented Phase 1 slice

`abb_lab_description` installs a full-SHA/SHA-256-locked ABB IRB1200-5/0.9
asset subset under project-local resource URIs. Its immutable geometry entry
point still expands to 10 links, 6 revolute joints, 3 fixed joints, and 14 mesh
references, with no world, simulator, control, MoveIt, gripper, or hardware
content. The snapshot, locks, license, and tests are package-local, so the
package builds and tests after being copied alone to a differently nested
colcon workspace.

P1.3a adds a separate simulation-only composition without altering that
geometry contract. It expands to 11 links and 10 kinematic joints, fixes
`base_link` to `world`, and declares one `GazeboSimSystem` with exactly six
position command interfaces and position/velocity/effort state interfaces.
Position command bounds and the ordinary URDF values are checked against one
canonical table derived from the pinned source. The Harmonic system plugin is
declared with `hold_joints=true`, and its controller-manager parameter file
sets startup-only command-limit enforcement to true. The package exports its
installed share parent as a Gazebo model resource root so converted
`model://abb_lab_description/...` mesh URIs resolve in a clean overlay.

P1.3b adds project-owned controller definitions, a minimal headless Gazebo
world, an ordered RSP/spawn/controller launch, and a bounded test trajectory.
A clean run at `1228c13` loaded the model, activated JTC/JSB, observed six
claimed position interfaces and runtime command-limit initialization, executed
the bounded trajectory, and verified cleanup. It still contains no MoveIt
configuration, collision-aware motion, gripper, GUI proof, real-ABB interface,
or out-of-range clamp proof. These boundaries remain executable tests, not only
documentation.

## Core domain contracts

- `SpeechCommand`: transcript, detected language, confidence, timestamp.
- `Task`, `TaskGraph`: typed goal, parameters, dependencies, conditions,
  timeout/retry policy, and state.
- `WorldState`, `RobotState`: timestamped frame-aware observations and source.
- `ExecutionResult`, `FailureReason`: evidence, failure taxonomy, and recovery.
- `ObjectDetection`, `ObjectPose`: camera evidence, depth, covariance/confidence,
  source frame, target frame, and timestamp.
- `VLAPolicy.predict_action(images, language_instruction, robot_state)` returns
  a typed `ActionChunk`, never a controller side effect.
- `ABBObservationAdapter` and `ABBActionAdapter` explicitly define units and
  mappings; no ALOHA/DROID action-space equivalence is assumed.

## Initial ROS 2 node boundaries

The initial design considers `speech_node`, `brain_node`, `world_model_node`,
`perception_node`, `skill_executor_node`, `vla_bridge_node`,
`robot_interface_node`, and `system_monitor_node`. Composition and lifecycle
management are preferred over proliferating nodes without isolation value.

Custom interfaces will include actions equivalent to `ExecuteSkill` and
`ExecuteTaskGraph`; exact fields are decided and documented before creation.

## Closed-loop executive and recovery

The task executive uses explicit states:

```text
PLANNING -> READY -> EXECUTING -> VERIFYING -> SUCCESS
                       |              |
                       v              v
                    RETRYING <-> REPLANNING -> FAILED / ABORTED
```

Every completed skill triggers a fresh timestamped `WorldState` read. A moved
object invalidates stale poses and returns execution to perception/planning.
Typed recovery covers at least object-not-found, IK/planning/collision/grasp/
trajectory failures, moved objects, VLA invalid/timeout, uncertain speech, and
unknown instructions. Failures are never silent.

## Mandatory M1 governance hold

Phase 8 completion cannot directly enable Phase 9:

```text
PHASE_8_COMPLETE
  -> M1_EVIDENCE_READY
  -> PAUSED_AWAITING_USER_MIDTERM_ACCEPTANCE
  -- explicit user acceptance --> PHASE_9_ALLOWED
```

`M1_EVIDENCE_READY` requires all twelve M1 entries in `ACCEPTANCE_STATUS.md`
to pass as saved real integrated runs on the tested Phase-8-final commit. Each
episode must preserve the TaskGraph and per-step timestamped
`OBSERVE -> VERIFY -> CONTINUE | RETRY | REPLAN` events. The executive, a test,
or the model cannot release the hold; only explicit user acceptance can do so.

## Safety path

Both classical and VLA paths converge before actuation:

```text
candidate trajectory/action
  -> finite-value and timestamp checks
  -> joint position/velocity/acceleration limits
  -> maximum delta, workspace, and gripper limits
  -> self/environment collision checks
  -> timeout/watchdog
  -> hardware abstraction
```

The simulation layer adopts real-robot safety structure but is not evidence of
safe physical ABB deployment. Any real robot connection requires a separate
human safety review and explicit authorization.

## Observability and reproducibility

Every robot command becomes an Episode containing the command/language,
TaskGraph, timestamped world/robot state, optional images, selected planner,
trajectory, controller, VLA output, failures, and verified result. Runtime data
uses `runs/YYYY-MM-DD/episode_xxx/` outside Git and supports later replay.

System Explorer consumes the same event stream. It must show current task and
queue, world/robot state, planner/controller/backend, perception mode, brain
mode, FPS/latency/VLA inference time, status, and failure reason. It cannot
modify or cosmetically override execution truth.
