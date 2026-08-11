# Phase 1 Dependency and Upstream Control Smoke Evidence

- Date: 2026-08-11
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy
- Scope: binary dependency integrity and upstream gz_ros2_control headless
  execution; **not** an ABB spawn or project acceptance test

## Installed baseline

The exact top-level candidates in `config/ros-jazzy-apt.lock.yaml` were
installed without upgrading or removing existing packages. Apt added 309
packages including non-ROS dependencies. The post-install manifest records 431
installed `ros-jazzy-*` packages in
`config/ros-jazzy-installed-packages.lock.tsv`.

`scripts/verify_ros_apt_manifest.sh` rejects malformed, unsorted, duplicate,
or empty-version rows and confirms that all 10 direct-package versions match
both dpkg state and the generated manifest. The corrected manifest contains
only `install ok installed` entries; apt-visible but uninstalled packages are
excluded.

Direct installed versions:

| Package | Installed version |
|---|---|
| `ros-jazzy-ros-gz` | `1.0.22-1noble.20260616.074726` |
| `ros-jazzy-gz-ros2-control` | `1.2.19-1noble.20260615.171757` |
| `ros-jazzy-gz-ros2-control-demos` | `1.2.19-1noble.20260616.073637` |
| `ros-jazzy-ros2-control` | `4.45.2-1noble.20260615.175135` |
| `ros-jazzy-ros2-controllers` | `4.40.1-1noble.20260616.074625` |
| `ros-jazzy-moveit` | `2.12.4-1noble.20260617.161956` |
| MoveIt Task Constructor core | `0.1.5-1noble.20260617.153812` |
| MoveIt Task Constructor capabilities | `0.1.5-1noble.20260617.155543` |
| MoveIt Task Constructor messages | `0.1.5-1noble.20260615.120353` |
| MoveIt Task Constructor visualization | `0.1.5-1noble.20260617.155544` |

`scripts/verify_ros_environment.sh` verified:

- every direct Debian package is fully installed at the recorded version;
- required ROS packages resolve under `/opt/ros/jazzy`;
- Gazebo reports gz-sim `8.11.0` (Harmonic major 8);
- the `ros2 control` CLI loads;
- `dpkg --audit` is clean;
- a separate root `apt-get check` succeeds.

## Executed control smoke

The reproducible command is:

```bash
bash scripts/smoke_test_gz_ros2_control.sh \
  runs/2026-08-11/phase1_upstream_smoke
```

The script:

1. starts the installed upstream `cart_example_position.launch.py` with
   `gz_args:=-s` (headless server);
2. waits on the actual controller-manager API;
3. requires `joint_state_broadcaster` and `joint_trajectory_controller` to be
   `active`;
4. sends a real `FollowJointTrajectory` action from 1.0 to 2.0 over two seconds;
5. requires action result `error_code: 0`;
6. reads `/joint_states` and requires the final position within 0.05 of 2.0;
7. terminates only the process group it started and verifies clean shutdown via
   the launch log.

Observed result:

```text
GZ_ROS2_CONTROL_SMOKE=PASS
TARGET_POSITION=2.0
FINAL_POSITION=1.9999999999999991
joint_state_broadcaster: active
joint_trajectory_controller: active
action: SUCCEEDED, error_code 0
```

The first run reached the correct joint state but the harness did not parse the
ROS 2 CLI's `array('d', [...])` representation. The parser was fixed to support
the observed output and the entire smoke was re-run; only the second complete
run is recorded as PASS.

## Inspected warnings

No `ERROR`, `FATAL`, failed action, or leftover simulator/controller process was
observed. The final regression log contained three warnings:

1. Resource manager waiting during normal asynchronous initialization.
2. A hardware-component executor node was skipped during initialization.
3. Hardware read/write statistics registration was skipped.

These did not prevent hardware/controller activation or trajectory completion.
The log also contained a safety-significant INFO diagnostic:

```text
Enforcing command limits is disabled. Command limits from URDF will be ignored.
```

That setting is unacceptable for the ABB project path. The project overlay must
explicitly enable and test command-limit enforcement and reconcile URDF,
ros2_control, and MoveIt limits before any ABB motion is accepted.

## Acceptance status

This smoke proves the selected Gazebo Harmonic ↔ gz_ros2_control ↔ controller ↔
action ↔ joint-state dependency chain works on the host. At this historical
dependency checkpoint it did not spawn an ABB robot, so A01 and all other
acceptance tests remained `NOT STARTED`. Later real-run status is tracked in
`ACCEPTANCE_STATUS.md`.
