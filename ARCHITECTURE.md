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
  containers/
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
  tests/
```

The layout is a target architecture, not a claim that these packages exist.
Packages are created only in tested atomic increments.

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

