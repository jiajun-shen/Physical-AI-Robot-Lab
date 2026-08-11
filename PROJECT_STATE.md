# Project State

## Project name

Physical-AI-Robot-Lab

## Current phase

Repository bootstrap (pre-Phase 0)

## Current subtask

Create the recoverable Git and documentation baseline before environment or
dependency decisions are recorded.

## Git checkpoints

- Latest known-good commit: pending initial baseline commit
- Stable commit: pending initial baseline commit
- Experimental commit: none
- Current branch: `main`
- Remote: none detected at bootstrap

## Working features

- Recoverable repository documentation skeleton only.

## Completed features

- Target directory was audited and contained no pre-existing project to retain.
- Git repository initialized on `main`.
- Secret, build, ROS log, dataset, and model artifacts excluded from Git.

## Partially completed features

- Phase 0 environment and dependency audit is in progress.

## Not started features

- ABB simulation, ros2_control, MoveIt, gripper, objects, perception, skills,
  drawing, speech, task executive, System Explorer, educational robotics core,
  VLA integration, ABB VLA adapters, dataset pipeline, fine-tuning, benchmarks,
  hardening, and CI.

## Current test status

- Documentation: files created; validation pending first commit.
- Python: no package and no tests yet.
- ROS 2: unavailable/unverified on this host at bootstrap.
- Gazebo: unavailable/unverified on this host at bootstrap.
- MoveIt: unavailable/unverified on this host at bootstrap.
- Acceptance tests A01-A14: all `NOT STARTED`.

## Current build status

No build system exists yet. This is not a build failure.

## Current run status

No executable system exists yet. No runtime capability is claimed.

## Known bugs

None: no implementation exists.

## Known limitations

- The active host is Windows; the target robotics baseline is expected to be
  Ubuntu 24.04 with ROS 2 Jazzy.
- Hardware, software, and official dependency audits are not yet recorded.

## Unresolved questions

- Whether a suitable Ubuntu 24.04/WSL2 or native Linux runtime already exists.
- Exact compatible commits for ABB description/control packages and openpi.
- Whether GPU resources satisfy the selected VLA inference configuration.

## Important architecture decisions

- Working classical baseline precedes advanced autonomy and VLA control.
- Language planning produces constrained tasks/skills/goals, never motor angles.
- Ground-truth and vision world-model sources remain explicitly separated.
- ROS and VLA dependency environments remain isolated behind a policy service.
- Simulation and real ABB hardware use the same upper-layer abstraction but
  separate hardware implementations and safety gates.

## Dependency versions

Pending verified Phase 0 audit. Do not infer versions from this file.

## External repository commits

Pending verified Phase 0 audit. No external source has been vendored.

## Important commands

```powershell
# Inspect repository
git status --short --branch
git log --oneline --decorate -10
```

Build, run, and test commands will be added only after their implementations
exist and have been executed.

## Next exact steps

1. Commit this repository baseline and record its hash.
2. Complete the host environment audit without installing or mutating software.
3. Verify ABB, ROS 2 Jazzy/Gazebo Harmonic/MoveIt, and openpi from official
   repositories and documentation.
4. Write `docs/environment_report.md` and replace provisional dependency rows.
5. Finalize Phase 0 architecture/package boundaries, run documentation checks,
   update this file, and create the Phase 0 checkpoint commit.

