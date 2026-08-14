# Codex Project Handoff — Physical-AI-Robot-Lab

> Historical Phase-3 snapshot date: 2026-08-14. This handoff is deliberately
> operational rather than promotional. The historical snapshot is retained
> below; the current, superseding Phase-4 completion addendum comes first.

## Current superseding Phase-4 completion addendum (2026-08-14)

Repository state must be established from `git status`, `PROJECT_STATE.md`,
`ACCEPTANCE_STATUS.md`, and the committed evidence before acting. At this
checkpoint:

- Active branch: `agent/phase4-perception`, tracking
  `origin/agent/phase4-perception`; `main` remains preserved at
  `f994fd651e4817f053121227383a8c89bb23b0ed`.
- The user accepted the saved Phase-3 evidence on 2026-08-14. P3.03 is
  `PASS`; the former Phase-3 hold is released and the instructions below that
  say “do not begin Phase 4” are historical only.
- Phase 4 is `COMPLETE` at clean tested feature commit
  `62dc9c8d0d682239dd2e4567f311a3ce2551ca4a`.
- The final isolated two-seed runtime root is
  `/var/tmp/physical-ai-p4.Kn4hFG`. It built/tested 29 records with zero
  errors, failures, or skips; source-manifest equality and cleanup passed.
- Both seed 42 and seed 43 received real RGB, `32FC1` depth in metres,
  CameraInfo, and TF only in `PERCEPTION_MODE=vision`, then emitted all five
  required typed object poses. The sealed records state
  `ground_truth_input_used: false`; only afterward did a separate process
  capture live Gazebo poses for offline comparison.
- A03 is `PASS`: green-ball errors are 0.0103339 m (seed 42) and 0.0150882 m
  (seed 43), and the later comparison proves a 0.3200000 m live green-ball
  position difference. A04 remains `NOT STARTED`.
- Phase 5 is **not started**. Stop after the Phase-4 documentation checkpoint
  and await explicit user direction before any Phase-5 implementation. The
  first future atomic task, if authorized, is a separate pick/place safety and
  evidence contract; it must not turn this visual-localization result into a
  contact/grasp claim.
- The Phase-8/M1 mandatory hold remains unchanged. M1.06 is only
  `PRE-M1 EVIDENCE`, not final M1 acceptance.

Read `docs/phase4_perception_runtime_evidence.md` after the root state files.
The old current-progress table, Phase-3 hold instructions, and next-session
instructions below describe the pre-acceptance snapshot and must not override
this addendum.

## Snapshot and source of truth

- Repository root (Windows): `C:\Users\shenj\Documents\Codex\2026-08-11\files-mentioned-by-the-user-demo\Physical-AI-Robot-Lab`
- Repository root (WSL): `/mnt/c/Users/shenj/Documents/Codex/2026-08-11/files-mentioned-by-the-user-demo/Physical-AI-Robot-Lab`
- Branch: `main`
- Current `HEAD` / `origin/main`: `f994fd651e4817f053121227383a8c89bb23b0ed`
- Remote: `https://github.com/jiajun-shen/Physical-AI-Robot-Lab.git` (private)
- Last runtime-qualified Phase-3 feature commit: `f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63`
- At the start of this handoff, the tracked worktree was clean and local `main`
  exactly matched `origin/main`. This file is newly created for the handoff and
  is therefore the only expected untracked file until it is intentionally
  committed.

Use the current files, `git status`, `PROJECT_STATE.md`,
`ACCEPTANCE_STATUS.md`, and committed evidence as the truth hierarchy. Older
audit reports can be historical and may contain superseded “not installed” or
“not run” wording.

---

## Project goal

### Original and final objective

Build a long-lived Physical AI robotics laboratory around an ABB IRB1200-5/0.9:

- a truthful Gazebo/ROS 2/ros2_control/MoveIt classical baseline;
- a simulated lab with RGB-D perception, collision-aware manipulation,
  pick/place, Cartesian drawing, multilingual speech, and an inspectable
  TaskGraph executive that observes/verifies/replans after every step;
- later, a carefully isolated VLA/openpi integration with ABB-specific
  observation, action, normalization, and safety adapters.

“Implemented” is never enough for this project: acceptance statuses require
saved, real runtime evidence tied to a clean tested commit.

### Current scoped goal

Phase 3 — modular two-finger gripper plus deterministic laboratory objects —
is complete at its intentionally limited scope. The project is now:

```text
PAUSED - AWAITING USER PHASE-3 ACCEPTANCE
```

**Do not begin Phase 4** until the user explicitly accepts the saved Phase-3
evidence. This Phase-3 hold is separate from the mandatory Phase-8/M1 stop
before Phase 9.

---

## Current progress

| Area | Actual status | What was proved by a real run |
|---|---|---|
| Phase 0 | Complete | Environment, compatibility, provenance, license, architecture audits. |
| Phase 1 / A01 | PASS | Project ABB model in Gazebo, one `GazeboSimSystem`, six claimed position commands, JSB/JTC active, limits enforced, bounded observed trajectory. Clean runtime at `1228c13`. |
| Phase 2 / A02 | PASS | Headless MoveIt joint plan/execution and `HOME -> FK-derived tool0 pose -> HOME`; synchronously injected collision object invalidated target and plans were rejected. Clean runtime at `4203e81`. |
| Phase 3 / P3.01 | PASS | ABB-with-gripper Gazebo profile: one manager, JSB/JTC/gripper controller active, 7 position commands and 22 state interfaces; true free-space open/close/open action and named mirrored finger states. |
| Phase 3 / P3.02 | PASS | Two separately launched clean worlds (`seed=42`, `seed=43`) captured all ten live Gazebo model poses. Live comparison proved position variation and directional cube yaw variation. |
| Phase 3 / P3.03 | **PENDING USER ACCEPTANCE** | This is the current stop gate. |
| A03–A14 | NOT STARTED | No perception/pick-place/drawing/speech/executive/VLA/data-pipeline claim yet. |

### Exact Phase-3 evidence

- Tested commit: `f6907ff6b6a4ed2ea91c4c21af68f4b78eb60d63`
- Tested source manifest SHA-256:
  `63e531b1a67c42132d434ef721cb5c9b076df8a8781d58e385d121ea039b0736`
- Clean runtime roots (WSL ext4):
  - `/var/tmp/p3-final-f6907ff-seed42`
  - `/var/tmp/p3-final-f6907ff-seed43`
  - `/var/tmp/p3-final-f6907ff-seed-comparison.json`
- Both runs: 82 colcon test records, zero errors/failures/skips, clean source
  manifests before and after.
- Live model names (exactly ten): `ground_plane`, `work_table`,
  `drawing_board`, `obstacle_left`, `obstacle_right`, `green_ball`,
  `red_ball`, `blue_cube`, `yellow_cube`, `abb_irb1200_gripper`.
- Live seed 42 vs 43 witnesses: green ball `0.3200002 m`, red ball
  `0.4003933 m`, blue cube `0.2000000 m`, yellow cube `0.1829426 m`, and
  both obstacles changed position. Blue/yellow cube yaw changed by
  `1.0000000 rad` / `0.4722650 rad`.
- Gripper command targets were `0.035 m -> 0.004 m -> 0.035 m` and actions
  succeeded. This proves only position-controlled free-space behavior.

### Important non-claims

Phase 3 does **not** prove contact, force, attachment, grasp retention, lift,
transport, pick/place, RGB-D perception, arm motion in the gripper lab,
MoveIt-with-gripper integration, GUI acceptance, real hardware, or generic
out-of-range clamping. `GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE` is
intentional and must remain truthful.

---

## Project structure and core entry points

```text
Physical-AI-Robot-Lab/
├── PROJECT_STATE.md             Current phase/hold and exact next step
├── ACCEPTANCE_STATUS.md         Immutable acceptance ledger; do not mark PASS from code alone
├── PROJECT_PLAN.md              Phase order and release rules
├── ARCHITECTURE.md              System/ownership architecture
├── DECISIONS.md                 ADRs and decisions that should not be casually reversed
├── KNOWN_ISSUES.md              Open risks, especially KI-012 and KI-013
├── DEPENDENCY_MATRIX.md         Installed/target dependency truth and versions
├── docs/                        Runtime evidence and historical audits
├── scripts/                     Headless runtime harnesses and evidence helpers
├── ros2_ws/src/                 Project ROS packages
├── config/                      Apt lock/manifests and project-level configuration
└── third_party/                 Pinned external source/provenance inputs
```

### Root documents to read first

1. `PROJECT_STATE.md` — current pause and exact authorization boundary.
2. `ACCEPTANCE_STATUS.md` — acceptance truth, especially `P3.01`–`P3.03`.
3. `PROJECT_PLAN.md` — phase sequence and the later Phase-8/M1 hold.
4. `ARCHITECTURE.md`, `DECISIONS.md`, `KNOWN_ISSUES.md` — architectural and
   safety guardrails before changing a design.
5. `docs/phase3_gripper_lab_runtime_evidence.md` — final P3 evidence and its
   explicit limitations.

### ROS packages and key files

| Package / location | Responsibility / key files |
|---|---|
| `ros2_ws/src/abb_lab_description` | Pinned ABB IRB1200 geometry/provenance, canonical joint limits, robot description and Gazebo control composition. Key: `urdf/irb1200_5_90_gz_control.urdf.xacro`, `urdf/irb1200_5_90_gz_ros2_control.xacro`, `config/controller_manager_safety.yaml`, `config/irb1200_5_90_control_limits.yaml`. Do not import the unsupported full ABB driver as a shortcut. |
| `ros2_ws/src/abb_lab_control` | Six-axis `joint_state_broadcaster` / position `JointTrajectoryController` configuration and bounded smoke goals. Key: `config/irb1200_controllers.yaml`, `config/irb1200_bounded_smoke.yaml`. |
| `ros2_ws/src/abb_lab_gazebo` | Arm-only Gazebo integration/launch, core runtime verifier, minimal arm world. Key: `launch/irb1200_headless.launch.py`, `scripts/verify_irb1200_runtime.py`, `worlds/irb1200_minimal.sdf`. |
| `ros2_ws/src/abb_lab_moveit_config` | Arm-only six-joint SRDF, KDL, OMPL-only planning config, controller mapping, one `move_group` integration launch, and MoveIt runtime verifier. Key: `launch/irb1200_moveit_headless.launch.py`, `scripts/verify_moveit_runtime.py`, `config/{irb1200_5_90_gz_control.srdf,joint_limits.yaml,kinematics.yaml,ompl_planning.yaml,moveit_controllers.yaml}`. |
| `ros2_ws/src/abb_lab_gripper` | P3 two-finger gripper profile. Right prismatic finger is the only commanded finger; left is a state-only URDF mimic with multiplier `-1`. Key: `urdf/irb1200_5_90_gz_gripper_control.urdf.xacro`, `urdf/irb1200_5_90_gz_gripper_ros2_control.xacro`, `urdf/two_finger_gripper.xacro`, controller/limit YAML. |
| `ros2_ws/src/abb_lab_lab` | Static lab template and deterministic scene generator. Key: `worlds/manipulation_lab.sdf`, `config/manipulation_scene.yaml`, `scripts/generate_scene_manifest.py`. The default package world is static; randomization is explicit via generator/harness. |
| `ros2_ws/src/abb_lab_bringup` | Separate P3 ABB+gripper+lab startup composition. Key: `launch/irb1200_gripper_lab_headless.launch.py`. It owns a single Gazebo-hosted controller chain and ordered JSB -> arm JTC -> gripper controller startup. |

### Core harnesses and evidence helpers

| File | Purpose |
|---|---|
| `scripts/run_and_verify_irb1200_headless.sh` | P1.3b arm Gazebo/ros2_control real-run gate. |
| `scripts/run_and_verify_irb1200_moveit_headless.sh` | P2.2 headless MoveIt gate. Do not rerun merely as a “sanity check”; it has known teardown behavior. |
| `scripts/run_and_verify_irb1200_gripper_lab_headless.sh` | P3 final integration harness. Generates/selects seeded lab SDF and drives the gripper runtime gate. Do not rerun while Phase-3 acceptance is pending. |
| `scripts/verify_irb1200_gripper_lab_runtime.py` | Runtime verifier for P3 resources, controllers, action/state, and model evidence. |
| `scripts/capture_gazebo_lab_scene_evidence.py` | Fail-closed capture/reparse of live `gz model` evidence. |
| `scripts/compare_p3_live_scene_evidence.py` | Compares two completed runtime roots; it uses captured live Gazebo output, not generated SDF/manifest, for variation proof. |

---

## Environment and dependencies

### Verified target environment

| Layer | Actual observed configuration |
|---|---|
| Host | Windows 11 25H2 (build recorded as `26200.8875` in the environment report) |
| WSL | WSL `2.7.3.0`, WSLg `1.0.73`, kernel `6.6.114.1-microsoft-standard-WSL2` |
| Distro | `Ubuntu-24.04`, Ubuntu `24.04.4 LTS` |
| ROS | ROS 2 Jazzy desktop, system Python `3.12.3` |
| Gazebo | Gazebo Harmonic / `gz-sim 8.11.0` with `ros-jazzy-ros-gz` `1.0.22-1noble.20260616.074726` |
| Control | `ros2_control/controller_manager 4.45.2-1noble.20260615.175135`; `ros2_controllers 4.40.1-1noble.20260616.074625`; `gz_ros2_control 1.2.19-1noble.20260615.171757` |
| Motion planning | MoveIt `2.12.4`; MoveIt Task Constructor core `0.1.5-1noble.20260617.153812` |
| Toolchain | GCC/G++ `13.3.0`, CMake `3.28.3`, Make `4.3`, Git `2.43` in WSL |
| GPU/ML caveat | RTX 5060 Laptop with 8151 MiB VRAM exposed to WSL, but no CUDA Toolkit/`nvcc` and no Docker. Local openpi/VLA is explicitly unavailable. |

Use WSL Ubuntu for robotics work. Native Windows is not the ROS/Gazebo build
target. Keep generated build/install/log trees on WSL ext4 for real test runs;
the source repository is on `/mnt/c` (DrvFS) and must not be shared between
Windows Git and WSL Git for line-ending repairs.

### Required setup in a normal WSL development shell

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/shenj/Documents/Codex/2026-08-11/files-mentioned-by-the-user-demo/Physical-AI-Robot-Lab
git status --short --branch
```

The manual Phase-3 GUI launcher currently uses this separately prepared ext4
install (not a committed build artifact):

```bash
source /var/tmp/phase3-manual-install/setup.bash
```

`/var/tmp/phase3-manual-install` exists at this handoff but is temporary OS
state and can disappear after cleanup/reboot. Treat it as a convenience launch
environment, **not** as source-of-truth or a replacement for a fresh approved
build. Do not delete it while the desktop GUI shortcut is needed.

### Runtime isolation values used by the current manual GUI

```bash
export ROS_DOMAIN_ID=106
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=
export GZ_PARTITION=physical_ai_phase3_desktop_106
export GZ_SIM_RESOURCE_PATH=/var/tmp/phase3-manual-install/abb_lab_description/share:/var/tmp/phase3-manual-install/abb_lab_gripper/share:/var/tmp/phase3-manual-install/abb_lab_lab/share
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
```

These are set by the external GUI script. The fixed domain/partition means do
not start two copies of that launcher at once.

---

## Startup methods

### Shortest current Phase-3 GUI launch

The user requested a Windows desktop shortcut and guide. They are intentionally
outside the Git repository:

- Desktop shortcut: `C:\Users\shenj\Desktop\Physical-AI Phase 3 Gazebo 实验室.lnk`
- Desktop guide: `C:\Users\shenj\Desktop\Physical-AI-Phase3-玩弄指南.md`
- Supporting directory:
  `C:\Users\shenj\Documents\Codex\2026-08-11\files-mentioned-by-the-user-demo\phase3-desktop-tools`

Normal user path: double-click the `.lnk`. It invokes hidden WSL through a
VBS/PowerShell wrapper, not `wt.exe`, and opens one Gazebo GUI window.

Equivalent direct, verified command from a newly opened Windows PowerShell:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash /mnt/c/Users/shenj/Documents/Codex/2026-08-11/files-mentioned-by-the-user-demo/phase3-desktop-tools/phase3_gazebo_run.sh
```

The script itself sources Jazzy and `/var/tmp/phase3-manual-install`, expands
the P3 gripper Xacro, starts Gazebo GUI with Bullet Featherstone, bridge/RSP,
spawns `abb_irb1200_gripper`, and starts JSB, arm JTC, and gripper controller.
It uses an automatically generated temporary GUI camera configuration to show
the table/ABB rather than the default distant camera.

The script opens a small `Phase 3 WSLg session keeper` `xmessage` window.
**Leave that helper window open while Gazebo is open.** It is part of the
WSLg workaround, not a project feature. If it is closed, the current GUI can
fall back to the blank `[WARN:COPY MODE]` behavior.

### Normal source checkout orientation (no build)

```powershell
wsl.exe -d Ubuntu-24.04
```

Then in Ubuntu:

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/shenj/Documents/Codex/2026-08-11/files-mentioned-by-the-user-demo/Physical-AI-Robot-Lab
git status --short --branch
sed -n '1,220p' PROJECT_STATE.md
```

### Historical real-run commands — do not run while paused

These are recorded for a future, explicitly authorized regression or phase
change. They are *not* instructions to run now:

```bash
cd /mnt/c/Users/shenj/Documents/Codex/2026-08-11/files-mentioned-by-the-user-demo/Physical-AI-Robot-Lab
bash scripts/run_and_verify_irb1200_headless.sh
bash scripts/run_and_verify_irb1200_moveit_headless.sh
P31_SCENE_SEED=42 bash scripts/run_and_verify_irb1200_gripper_lab_headless.sh
P31_SCENE_SEED=43 bash scripts/run_and_verify_irb1200_gripper_lab_headless.sh
```

The P3 two-seed comparison is a separate command that consumes the two
completed output roots. Use the command/options in the harness’s own source,
not an improvised path, if a future authorized rerun becomes necessary.

---

## Changes already made and why they matter

### Committed project changes (preserve)

- `1228c13`: robust Phase-1 project ABB Gazebo/ros2_control runtime.
- `281c36b` / `4203e81`: project-owned MoveIt configuration and real
  plan/execute/collision-rejection evidence.
- `759fdfb`: gripper/laboratory profile.
- `6fce68b`: deterministic orientation variation for cubes/objects.
- `f6907ff`: real live seeded scene capture/comparison.
- `f994fd6`: Phase-3 evidence, state, acceptance, and documentation updates.

Do not discard the geometry asset lock/provenance, canonical position/velocity
limits, `enforce_command_limits=true`, exact controller mapping, P3 mimic
profile, or phase evidence documents without an explicit compatibility/safety
review.

### External, unversioned desktop-GUI workaround (preserve separately)

The following files were created outside the repository for the user’s manual
visual inspection:

```text
C:\Users\shenj\Documents\Codex\2026-08-11\files-mentioned-by-the-user-demo\phase3-desktop-tools\phase3_gazebo_run.sh
C:\Users\shenj\Documents\Codex\2026-08-11\files-mentioned-by-the-user-demo\phase3-desktop-tools\phase3_gazebo_launcher.ps1
C:\Users\shenj\Documents\Codex\2026-08-11\files-mentioned-by-the-user-demo\phase3-desktop-tools\phase3_gazebo_launcher.vbs
C:\Users\shenj\Documents\Codex\2026-08-11\files-mentioned-by-the-user-demo\phase3-desktop-tools\Physical-AI-Phase3-玩弄指南.md
C:\Users\shenj\Desktop\Physical-AI Phase 3 Gazebo 实验室.lnk
C:\Users\shenj\Desktop\Physical-AI-Phase3-玩弄指南.md
```

They are not Git-tracked and are important to preserve if desktop launching is
still desired. They do not alter project source or evidence status. The helper
script deliberately changed from `set -u` to `set -e`: sourcing Jazzy under
`set -u` failed because `AMENT_TRACE_SETUP_FILES` was an unbound variable.

---

## Git state and preservation requirements

### Actual snapshot before this handoff file

```text
branch: main
HEAD:   f994fd651e4817f053121227383a8c89bb23b0ed
origin/main: same SHA (confirmed via git ls-remote)
status: clean; no staged, unstaged, or untracked repository files
```

Recent relevant history:

```text
f994fd6 docs(phase3): record live seeded gripper acceptance
f6907ff feat(lab): verify live seeded scene variation
6fce68b fix(lab): randomize seeded object orientation
759fdfb feat(gripper): add deterministic lab runtime profile
3f934b0 docs(phase2): record clean MoveIt acceptance
4203e81 feat(moveit): verify ABB planning and execution
eda442b chore(state): record MoveIt configuration gate
281c36b chore: normalize MoveIt config text
5945bc8 feat(moveit): add ABB planning configuration contract
ee7c9ab chore(state): record clean ABB runtime and M1 gate
```

After this file is written, `CODEX_HANDOFF.md` is the expected untracked change
unless the user asks to commit it. No unrelated change should be mixed into
that commit.

### Do not lose

1. `f6907ff` and `f994fd6` (already pushed) plus the Phase-3 evidence doc.
2. The current phase pause in `PROJECT_STATE.md` and `ACCEPTANCE_STATUS.md`.
3. The explicit P3 truth boundary: free-space position action is not grasp.
4. The repo-external desktop tools/shortcut above, if the user wants GUI
   inspection to remain one-click.
5. The exact evidence paths while they remain on the WSL disk. They are under
   `/var/tmp` and could be cleared by the OS; the committed evidence document
   contains the durable summary.

---

## Important problems already encountered

### 1. Blank WSLg Gazebo window / `[WARN:COPY MODE]`

- **Symptom:** Gazebo process and taskbar window appeared, but content was
  blank/gray and title contained `[WARN:COPY MODE]`.
- **Likely cause:** WSLg VAIL shared-memory path/session race on WSL 2.7.3;
  this matches the still-open Microsoft issue
  <https://github.com/microsoft/wslg/issues/1456>. `[WARN:COPY MODE]` means a
  fallback copy transport, not proof that Gazebo’s renderer itself is broken.
- **Observed diagnostic:** `/mnt/wslg/weston.log` reported
  `rdp_allocate_shared_memory` / `/mnt/shared_memory` I/O behavior in the
  failing state.
- **Tried/avoid repeating:** do not begin by reinstalling Gazebo, changing
  Mesa, disabling GPU acceleration, or rewriting the project renderer.
  A temporary `tmpfs` mount was explored but was not accepted as a durable
  system change; do not add `/etc/fstab` entries without explicit approval.
- **Working session workaround:** launch and keep a lightweight WSLg GUI
  client (`xmessage`) alive before/while Gazebo runs. The external desktop
  launcher implements this. It produced a visible, maximized Phase-3 Gazebo
  GUI in the current manual session without `[WARN:COPY MODE]`.
- **Residual risk:** this is a WSLg workaround, not a root-cause fix. Closing
  the helper or restarting WSL can require launching again. Preserve the
  helper behavior until the upstream WSLg issue is resolved/verified.

### 2. Duplicate ROS/Gazebo stack after repeated GUI starts

- **Symptom:** Gazebo reported `abb_irb1200_gripper already exists`; spawners
  reported controllers already loaded/active and then failed configuration.
- **Cause:** the manual desktop launcher uses fixed `ROS_DOMAIN_ID=106` and
  `GZ_PARTITION=physical_ai_phase3_desktop_106`; a second launch was started
  while an old stack was still alive.
- **Resolution:** stop only the known owned processes, then start one launcher.
  Do not use broad `pkill`/global cleanup. Do not double-click the shortcut
  more than once while Gazebo is already open.
- **Residual risk:** fixed isolation values are convenient for a desktop demo,
  not a multi-user/multi-session orchestration design.

### 3. Jazzy setup fails under `set -u`

- **Symptom:** external GUI launch failed immediately with
  `AMENT_TRACE_SETUP_FILES: unbound variable`.
- **Cause:** ROS setup scripts are not nounset-clean.
- **Resolution:** the desktop shell launcher uses `set -e`, not `set -u`.
- **Do not regress:** do not reintroduce `set -u` before sourcing
  `/opt/ros/jazzy/setup.bash` unless setup is guarded explicitly.

### 4. Gripper mimic did not work on default physics engine

- **Symptom:** default Gazebo DART did not honor the state-only URDF mimic
  constraint for the left finger.
- **Resolution:** P3 launch explicitly selects
  `gz-physics-bullet-featherstone-plugin`. Real probes then showed the left
  finger mirrored the right and real actions succeeded.
- **Boundary:** Bullet Featherstone is **P3 profile-only**. Do not silently
  turn it into a global physics baseline or equate it with real gripper
  dynamics/contact fidelity.

### 5. `move_group` SIGSEGV during requested P2 teardown

- **Symptom:** P2.2 clean runtime successfully completed and left no owned
  processes, but its installed MoveIt `move_group` exited `SIGSEGV` after the
  harness sent requested SIGINT.
- **Recorded as:** `KI-012`.
- **Do not misstate:** P2.2 is valid planning/execution/collision evidence,
  but not graceful shutdown evidence.
- **Future action:** reproduce/minimize/contain before M1 and Phase 16
  hardening; do not rerun it merely to “make the log clean.”

### 6. Gripper result must not be used as force/grasp evidence

- **Symptom:** action result effort/state was not meaningful; finger effort
  data had no valid force authority.
- **Resolution:** use named `/joint_states` for finger position/mirror proof.
  Keep `max_effort=0`, `force_control=NOT_CONFIGURED`, and
  `GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE` honest.
- **Future action:** a later contact/force/attachment/lift design must add its
  own real evidence; do not infer it from animation or an action success.

---

## Current unresolved items (priority ordered)

1. **Authorization gate — highest priority.**
   - Current state: Phase 3 is complete, but P3.03 is awaiting the user’s
     explicit acceptance.
   - Next action: wait. Do not start Phase 4, rewrite Phase 3, or rerun the
     gate without user direction.

2. **WSLg reliability for GUI demos.**
   - Current state: a GUI was manually demonstrated with the external keeper
     workaround; this is not committed project acceptance evidence.
   - Most likely cause: upstream WSLg shared-memory/session behavior.
   - Next check only if user reports another blank GUI: verify the helper is
     still open, inspect `/mnt/wslg/weston.log`, and relaunch a single known
     stack. Do not reinstall Gazebo/ROS first.

3. **P3 capability boundary: no contact/grasp/attachment.**
   - Current state: free-space open/close/mirror and lab objects are proven,
     but no contact sensor, force, attachment, lift, or transport exists.
   - Likely future work: a separately specified Phase-5 pick/place path;
     do not promote this into P3 acceptance or A04.

4. **MoveIt graceful teardown (KI-012).**
   - Current state: P2.2 worked but `move_group` SIGSEGV on shutdown.
   - Next check when Phase-16/M1 hardening is authorized: make a small
     reproduction, preserve ownership/isolation, distinguish benign teardown
     from runtime failure, and avoid hiding an exit status.

5. **Dynamics/safety fidelity.**
   - Current state: position/velocity limits are reconciled; ABB inertials are
     estimates; effort values are fictional; no acceleration/jerk authority;
     broad clamp injection is unverified.
   - Next check: only add capabilities backed by a concrete source and a real
     runtime test; never invent numerical limits.

6. **Resource/VLA constraints.**
   - Current state: WSL has about 7.4 GiB RAM; GPU has 8151 MiB VRAM; CUDA
     toolkit/Docker are absent; openpi expects an isolated Ubuntu 22.04 /
     Python 3.11/CUDA environment.
   - Next check: do not blend openpi into Jazzy Python 3.12. Treat VLA as
     unavailable until an isolated supported server is demonstrated.

---

## Important technical decisions — do not casually reverse

- **Use Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic from the coherent ROS apt
  ecosystem.** Do not mix a non-default OSRF Gazebo repository or Gazebo
  Classic into this baseline.
- **Vendor only pinned ABB geometry/kinematic assets.** The full upstream
  `abb_ros2` driver is not accepted as Jazzy-compatible. Project-owned
  Harmonic/ros2_control/MoveIt overlays own the runtime behavior.
- **One controller manager only.** Gazebo `gz_ros2_control` owns it. MoveIt
  maps to the existing JTC and must not start fake hardware, another
  `ros2_control_node`, or another set of spawners.
- **Safety authority stays in robot description/control configuration.**
  `controller_manager.ros__parameters.enforce_command_limits=true` is required.
  A generic MoveIt `joint_limits.yaml` is planning-side input and is not a
  substitute for controller-manager enforcement.
- **P3 gripper design:** right finger has the sole position command; left
  finger is state-only/mimic (`-1`). Preserve the exact interface ownership.
- **P3 lab randomization is explicit.** `generate_scene_manifest.py` plus the
  harness generates/chooses seeded worlds. Do not claim that ordinary default
  bringup randomizes every launch.
- **Perception must not use Gazebo ground truth.** Ground-truth capture is
  offline evidence/comparison only; later `vision` mode needs RGB-D,
  CameraInfo, TF, and independently derived pose output.
- **Language plans produce typed Task/Skill/Goal structures, not raw joint
  angles.** Each future execution step must observe/verify and continue or
  replan with timestamped evidence.
- **VLA isolation:** keep openpi outside the Jazzy Python environment behind a
  typed API. `VLA_BACKEND_UNAVAILABLE` is an honest state, not a failure to
  paper over with an unlabelled classical fallback.

Refactoring is allowed later when the same ownership, evidence, and safety
contracts are preserved. Do not rewrite working P1–P3 components simply to
make them look more generic.

---

## User requirements and working preferences

- The GitHub home is `https://github.com/jiajun-shen`; the project belongs
  under `jiajun-shen/Physical-AI-Robot-Lab` and is currently private.
- The user requires **real runtime results** before acceptance claims. Code,
  static tests, or a model file do not count alone.
- Pause after Phase 3 and wait for explicit user acceptance. Then later pause
  after Phase 8 before Phase 9 until every M1 row has immutable real-run
  evidence and the user accepts it.
- Do not start Phase 4 during the current hold.
- Do not modify project source, Git state, or rerun a full acceptance merely to
  support a GUI/manual viewing request unless expressly authorized.
- For Windows GUI work, the user explicitly asked not to use `wt.exe`, not to
  create/control multiple Windows Terminal/PowerShell windows, not to run
  `sleep 30`, and not to automate unrelated terminal tests. Keep one GUI
  launch focused on showing the robot.
- Avoid expensive, repetitive re-audits or re-builds when recorded evidence
  and a light Git/status check suffice. Make scoped, evidence-backed changes
  instead of broad rewrites.
- Preserve unrelated user changes in a dirty worktree. Use `apply_patch` for
  edits and never destructively reset/checkout files without explicit approval.

---

## Next action sequence

### While the Phase-3 hold is active

1. Read `PROJECT_STATE.md` and verify `P3.03` is still pending in
   `ACCEPTANCE_STATUS.md`.
2. Do **not** implement Phase 4 or modify Phase-3 source/evidence.
3. If the user wants manual visual review, use the existing desktop shortcut
   or the exact direct GUI command above; do not turn a manual GUI display into
   an acceptance-status change.
4. If asked to preserve this handoff, review only `git status` and commit
   `CODEX_HANDOFF.md` separately with no unrelated files.

### Only after the user explicitly accepts Phase 3

1. Record the explicit approval in `PROJECT_STATE.md`,
   `ACCEPTANCE_STATUS.md` (`P3.03`), and concise relevant docs; commit/push
   those documentation-only changes deliberately.
2. Begin Phase 4, not Phase 5: design a project-owned RGB-D + CameraInfo + TF
   pipeline that localizes the required objects in a strict `vision` mode.
3. Ensure no vision component reads Gazebo ground truth. Ground truth may be
   used offline only for evaluation.
4. Define before coding the Phase-4 real-run evidence contract: recorded RGB,
   depth, CameraInfo, TF chain, derived object pose, comparison method, and
   clean tested commit/output root.
5. Keep A03 `NOT STARTED` until a real Phase-4 runtime gate passes. Keep A04
   and P3/M1 grasp claims unchanged.

### Later non-negotiable milestone

After Phase 8, stop before Phase 9. Do not release Phase 9 until every M1 item
is PASS from a final integrated real run and the user explicitly accepts the
midterm review. That rule survives any future refactoring or feature work.

---

## Instructions for the next Codex session

1. Read this file, then `PROJECT_STATE.md`, `ACCEPTANCE_STATUS.md`,
   `PROJECT_PLAN.md`, `ARCHITECTURE.md`, `DECISIONS.md`, and
   `docs/phase3_gripper_lab_runtime_evidence.md`.
2. Run only a light `git status --short --branch` / `git log -1` before changing
   anything. Confirm the Phase-3 hold has not been explicitly released.
3. The current most important task is **waiting for the user’s Phase-3
   acceptance**, not new implementation. If authorization arrives, first
   record it cleanly, then start Phase 4 RGB-D/TF perception.
4. Do not rerun P1/P2/P3 gates, rebuild the workspace, reinstall Gazebo, or
   reopen the ABB driver question just to orient yourself.
5. Preserve one Gazebo-hosted controller manager, the P3 Bullet/mimic profile,
   the no-ground-truth vision rule, real-run acceptance discipline, and the
   later Phase-8 M1 stop gate.
