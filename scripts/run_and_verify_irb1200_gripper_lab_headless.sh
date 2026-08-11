#!/usr/bin/env bash
# Run the project-owned P3.1 ABB gripper/laboratory gate in an isolated domain.
#
# This creates an ephemeral overlay, generates a deterministic external scene,
# starts one Gazebo-hosted controller manager, and accepts P3.1 only after a
# real ParallelGripperCommand open -> close -> open sequence is observed.  It
# intentionally does not claim contact, attachment, force control, or grasp
# success; those require later dedicated evidence.
set -eo pipefail

if [[ "${P31_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P31_WATCHDOG_ACTIVE=1
  # Default timeout process-group handling also bounds synchronous colcon work.
  exec timeout --signal=TERM --kill-after=30s 240s "$0" "$@"
fi

readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: missing ${ROS_SETUP}" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
readonly SOURCE_ROOT="${REPOSITORY_ROOT}/ros2_ws/src"
readonly REQUIRED_AVAILABLE_MEMORY_KIB=$((2 * 1024 * 1024))
readonly EXPECTED_MODEL_COUNT=10
readonly RIGHT_FINGER="two_finger_right_joint"
readonly LEFT_FINGER="two_finger_left_joint"
readonly -a EXPECTED_MODELS=(
  ground_plane
  work_table
  drawing_board
  obstacle_left
  obstacle_right
  green_ball
  red_ball
  blue_cube
  yellow_cube
  abb_irb1200_gripper
)

die() {
  echo "ERROR: $*" >&2
  exit 1
}

for required_command in awk cmp colcon find findmnt flock git grep gz pgrep ps \
    python3 realpath ros2 sed setsid sha256sum sort tail timeout tr wc xargs; do
  command -v "${required_command}" >/dev/null 2>&1 ||
    die "required command is unavailable: ${required_command}"
done

# The selected seed is part of the real-runtime identity, not merely a
# generator input.  Keep the value in a bounded non-negative range so the
# adjacent contrast seed is always valid too.
requested_scene_seed="${P31_SCENE_SEED:-42}"
[[ "${requested_scene_seed}" =~ ^[0-9]+$ ]] ||
  die "P31_SCENE_SEED must be a non-negative decimal integer"
(( ${#requested_scene_seed} <= 10 )) ||
  die "P31_SCENE_SEED exceeds the supported Phase-3 seed range"
selected_scene_seed=$((10#${requested_scene_seed}))
(( selected_scene_seed <= 2147483646 )) ||
  die "P31_SCENE_SEED exceeds the supported Phase-3 seed range"
readonly P31_SCENE_SEED_SELECTED="${selected_scene_seed}"
readonly P31_CONTRAST_SCENE_SEED="$((P31_SCENE_SEED_SELECTED + 1))"

if [[ $# -gt 1 ]]; then
  die "usage: $0 [new-or-empty-output-directory]"
fi

if [[ $# -eq 0 ]]; then
  output_root="$(mktemp -d /var/tmp/physical-ai-p31.XXXXXX)"
else
  output_root="$(realpath -m "$1")"
  repository_real="$(realpath -e "${REPOSITORY_ROOT}")"
  case "${output_root}/" in
    "${repository_real}/"*)
      die "output directory must not be the repository or one of its descendants"
      ;;
  esac
  [[ "${output_root}" != "/" ]] || die "output directory must not be /"
  if [[ -d "${output_root}" ]] &&
      [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    die "explicit output directory must be empty: ${output_root}"
  fi
  [[ ! -e "${output_root}" || -d "${output_root}" ]] ||
    die "output path exists and is not a directory: ${output_root}"
  mkdir -p "${output_root}"
fi
readonly OUTPUT_ROOT="$(realpath -e "${output_root}")"
exec {output_lock_fd}>"${OUTPUT_ROOT}/.runtime-gate.lock"
flock -n "${output_lock_fd}" || die "output directory is already in use: ${OUTPUT_ROOT}"
readonly output_lock_fd

output_filesystem="$(findmnt -T "${OUTPUT_ROOT}" -n -o FSTYPE)"
case "${output_filesystem,,}" in
  9p|drvfs)
    die "output directory must be on a Linux filesystem, not ${output_filesystem}"
    ;;
esac

readonly BUILD_BASE="${OUTPUT_ROOT}/build"
readonly INSTALL_BASE="${OUTPUT_ROOT}/install"
readonly COLCON_LOG_BASE="${OUTPUT_ROOT}/colcon-log"
readonly RUNTIME_LOG_ROOT="${OUTPUT_ROOT}/runtime"
readonly ROS_HOME_PATH="${OUTPUT_ROOT}/ros-home"
readonly SCENE_ROOT="${OUTPUT_ROOT}/scene"
readonly LAUNCH_LOG="${RUNTIME_LOG_ROOT}/launch.log"
readonly CONTROLLERS_LOG="${RUNTIME_LOG_ROOT}/controllers.txt"
readonly HARDWARE_LOG="${RUNTIME_LOG_ROOT}/hardware_components.txt"
readonly INTERFACES_LOG="${RUNTIME_LOG_ROOT}/hardware_interfaces.txt"
readonly PARAMETER_LOG="${RUNTIME_LOG_ROOT}/enforce_command_limits.txt"
readonly NODES_LOG="${RUNTIME_LOG_ROOT}/nodes.txt"
readonly ACTIONS_LOG="${RUNTIME_LOG_ROOT}/actions.txt"
readonly MODELS_LOG="${RUNTIME_LOG_ROOT}/models.txt"
readonly MODELS_AFTER_LOG="${RUNTIME_LOG_ROOT}/models_after_actions.txt"
readonly LIVE_SCENE_POSE_ROOT="${RUNTIME_LOG_ROOT}/model_poses"
readonly LIVE_SCENE_CAPTURE_LOG="${RUNTIME_LOG_ROOT}/live_scene_capture.txt"
readonly LIVE_SCENE_EVIDENCE="${RUNTIME_LOG_ROOT}/live_scene_evidence.json"
readonly VERIFIER_LOG="${RUNTIME_LOG_ROOT}/gripper_verifier.txt"
readonly VERIFIER_EVIDENCE="${RUNTIME_LOG_ROOT}/gripper_evidence.json"
readonly SOURCE_MANIFEST_BEFORE="${OUTPUT_ROOT}/working_source_before.sha256"
readonly SOURCE_MANIFEST_AFTER="${OUTPUT_ROOT}/working_source_after.sha256"
readonly RESULT_LOG="${OUTPUT_ROOT}/result.txt"
mkdir -p "${COLCON_LOG_BASE}" "${RUNTIME_LOG_ROOT}" "${ROS_HOME_PATH}" "${SCENE_ROOT}" \
  "${LIVE_SCENE_POSE_ROOT}"

export PYTHONDONTWRITEBYTECODE=1
export ROS_HOME="${ROS_HOME_PATH}"
export ROS_LOG_DIR="${RUNTIME_LOG_ROOT}/ros"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
mkdir -p "${ROS_LOG_DIR}"

available_memory_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
[[ "${available_memory_kib}" =~ ^[0-9]+$ ]] ||
  die "could not read MemAvailable from /proc/meminfo"
(( available_memory_kib >= REQUIRED_AVAILABLE_MEMORY_KIB )) ||
  die "MemAvailable ${available_memory_kib} KiB is below the 2 GiB runtime floor"

requested_domain="${ROS_DOMAIN_ID:-}"
domain_lock_fd=""
acquire_domain_lock() {
  local candidate
  local candidate_fd
  local first=100
  local last=199
  if [[ -n "${requested_domain}" ]]; then
    [[ "${requested_domain}" =~ ^[0-9]+$ ]] ||
      die "ROS_DOMAIN_ID must be an integer in [1, 232]"
    (( requested_domain >= 1 && requested_domain <= 232 )) ||
      die "ROS_DOMAIN_ID must be in [1, 232]"
    first="${requested_domain}"
    last="${requested_domain}"
  fi
  for ((candidate = first; candidate <= last; candidate++)); do
    exec {candidate_fd}>"/var/tmp/physical-ai-p31-domain-${candidate}.lock"
    if flock -n "${candidate_fd}"; then
      export ROS_DOMAIN_ID="${candidate}"
      domain_lock_fd="${candidate_fd}"
      return
    fi
    eval "exec ${candidate_fd}>&-"
  done
  die "no project runtime domain lock is available in [${first}, ${last}]"
}
acquire_domain_lock
readonly ROS_DOMAIN_ID
readonly domain_lock_fd
export GZ_PARTITION="physical_ai_p31_${ROS_DOMAIN_ID}_${BASHPID}"
readonly GZ_PARTITION

if ! preexisting_nodes="$(
    timeout --kill-after=1s 4s ros2 node list --no-daemon --spin-time 2 \
      2>"${RUNTIME_LOG_ROOT}/domain_preflight_stderr.txt"
  )"; then
  sed -n '1,160p' "${RUNTIME_LOG_ROOT}/domain_preflight_stderr.txt" >&2 || true
  die "could not prove selected ROS domain ${ROS_DOMAIN_ID} was empty"
fi
[[ -z "${preexisting_nodes}" ]] ||
  die "selected ROS domain ${ROS_DOMAIN_ID} already contains nodes: ${preexisting_nodes}"

tested_commit="$(git -C "${REPOSITORY_ROOT}" rev-parse HEAD)"
if [[ -z "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]]; then
  source_worktree_clean=true
else
  source_worktree_clean=false
fi
capture_working_source_manifest() {
  (
    cd "${REPOSITORY_ROOT}"
    git ls-files --cached --others --exclude-standard -z |
      LC_ALL=C sort -z |
      xargs -0 -r sha256sum
  )
}
capture_working_source_manifest >"${SOURCE_MANIFEST_BEFORE}"
source_manifest_sha256="$(sha256sum "${SOURCE_MANIFEST_BEFORE}" | awk '{print $1}')"
{
  echo "TESTED_COMMIT=${tested_commit}"
  echo "SOURCE_WORKTREE_CLEAN=${source_worktree_clean}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "GZ_PARTITION=${GZ_PARTITION}"
  echo "P31_SCENE_SEED=${P31_SCENE_SEED_SELECTED}"
  echo "P31_CONTRAST_SCENE_SEED=${P31_CONTRAST_SCENE_SEED}"
  echo "SCENE_WORLD_NAME=manipulation_lab"
  echo "MEM_AVAILABLE_KIB_AT_START=${available_memory_kib}"
  echo "OUTPUT_FILESYSTEM=${output_filesystem}"
  echo "WORKING_SOURCE_MANIFEST_SHA256=${source_manifest_sha256}"
} >"${OUTPUT_ROOT}/run_metadata.txt"

launch_pid=""
launch_pgid=""
cleanup_started=false
run_completed=false
project_spawn=false
hardware_resources=false
controllers_active=false
limiter_initialized=false
lab_objects_present=false
scene_seed_reproducible=false
live_scene_pose_capture=false
gripper_open_close_open=false
gripper_mirror_state=false
launch_exit_status="NOT_WAITED"

process_group_alive() {
  [[ -n "${launch_pgid}" ]] && pgrep -g "${launch_pgid}" >/dev/null 2>&1
}

wait_for_group_exit() {
  local attempts="$1"
  local attempt_index
  for ((attempt_index=0; attempt_index<attempts; attempt_index++)); do
    process_group_alive || return 0
    # Polling delay is cleanup orchestration, not simulated control timing.
    sleep 0.25
  done
  ! process_group_alive
}

cleanup() {
  local status="${1:-$?}"
  if [[ "${cleanup_started}" == "true" ]]; then
    return
  fi
  cleanup_started=true
  trap - EXIT
  # Finish bounded cleanup if the outer watchdog TERM arrives during EXIT.
  trap '' INT TERM

  if process_group_alive; then
    kill -INT -- "-${launch_pgid}" 2>/dev/null || true
    wait_for_group_exit 20 || true
  fi
  if process_group_alive; then
    kill -TERM -- "-${launch_pgid}" 2>/dev/null || true
    wait_for_group_exit 12 || true
  fi
  if process_group_alive; then
    kill -KILL -- "-${launch_pgid}" 2>/dev/null || true
    wait_for_group_exit 4 || true
  fi
  if [[ -n "${launch_pid}" ]]; then
    if wait "${launch_pid}" 2>/dev/null; then
      launch_exit_status=0
    else
      launch_exit_status=$?
    fi
  fi
  if process_group_alive; then
    echo "ERROR: launch process group ${launch_pgid} survived bounded cleanup" >&2
    status=1
  fi

  timeout --kill-after=1s 5s ros2 daemon stop >/dev/null 2>&1 || true
  for _ in {1..8}; do
    if ! pgrep -f -- "ros2-daemon.*--ros-domain-id ${ROS_DOMAIN_ID}([[:space:]]|$)" \
        >/dev/null 2>&1; then
      break
    fi
    # Polling delay is cleanup orchestration, not simulated control timing.
    sleep 0.25
  done
  if pgrep -af -- "ros2-daemon.*--ros-domain-id ${ROS_DOMAIN_ID}([[:space:]]|$)" \
      >"${RUNTIME_LOG_ROOT}/residual_ros_daemon.txt" 2>&1; then
    echo "ERROR: ROS daemon for domain ${ROS_DOMAIN_ID} survived cleanup" >&2
    status=1
  fi
  if [[ "${run_completed}" == "true" && "${launch_exit_status}" != "0" ]]; then
    echo "ERROR: the completed runtime gate had nonzero launch exit ${launch_exit_status}" >&2
    status=1
  fi

  if [[ "${status}" -eq 0 && "${run_completed}" == "true" ]]; then
    {
      echo "PROJECT_ABB_GRIPPER_GAZEBO_SPAWN=PASS"
      echo "GRIPPER_HARDWARE_RESOURCES=PASS"
      echo "CONTROLLERS_ACTIVE=PASS"
      echo "LIMITER_RUNTIME_INITIALIZED=PASS"
      echo "LAB_OBJECTS_PRESENT=PASS"
      echo "LAB_SCENE_SEED_REPRODUCIBLE=PASS"
      echo "GAZEBO_LIVE_SCENE_POSE_CAPTURE=PASS"
      echo "GRIPPER_OPEN_CLOSE_OPEN=PASS"
      echo "GRIPPER_MIRROR_STATE=PASS"
      echo "GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE"
      echo "GUI_RENDERING=NOT_RUN"
      echo "REAL_ROBOT_READY=false"
      if [[ "${project_spawn}" == "true" && "${hardware_resources}" == "true" && \
          "${controllers_active}" == "true" && "${limiter_initialized}" == "true" && \
          "${lab_objects_present}" == "true" && "${scene_seed_reproducible}" == "true" && \
          "${live_scene_pose_capture}" == "true" && \
          "${gripper_open_close_open}" == "true" && "${gripper_mirror_state}" == "true" ]]; then
        echo "P3_1=PASS"
      else
        echo "P3_1=NOT_STARTED"
        status=1
      fi
      echo "A03=NOT_STARTED"
      echo "A04=NOT_STARTED"
      echo "ABB_GRIPPER_LAB_HEADLESS_RUNTIME=PASS"
      echo "CLEANUP_NO_RESIDUAL_PROCESS_GROUP=PASS"
      echo "LAUNCH_EXIT_STATUS=${launch_exit_status}"
      if [[ "${launch_exit_status}" == "0" ]]; then
        echo "CLEANUP_VERIFIED=PASS"
      else
        echo "CLEANUP_VERIFIED=NO_RESIDUALS_WITH_NONZERO_LAUNCH_EXIT"
      fi
      echo "TESTED_COMMIT=${tested_commit}"
      echo "SOURCE_WORKTREE_CLEAN=${source_worktree_clean}"
      echo "WORKING_SOURCE_MANIFEST_SHA256=${source_manifest_sha256}"
      echo "P31_SCENE_SEED=${P31_SCENE_SEED_SELECTED}"
      echo "P31_CONTRAST_SCENE_SEED=${P31_CONTRAST_SCENE_SEED}"
      echo "SCENE_WORLD_NAME=manipulation_lab"
      echo "LIVE_SCENE_EVIDENCE=${LIVE_SCENE_EVIDENCE}"
      echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
      echo "GZ_PARTITION=${GZ_PARTITION}"
      echo "OUTPUT_ROOT=${OUTPUT_ROOT}"
    } >"${RESULT_LOG}"
    sed -n '1,220p' "${RESULT_LOG}"
  fi
  exit "${status}"
}
trap cleanup EXIT
trap 'cleanup 130' INT
trap 'cleanup 143' TERM

run_step() {
  local label="$1"
  local log_path="$2"
  shift 2
  if ! "$@" >"${log_path}" 2>&1; then
    echo "ERROR: ${label} failed; tail of ${log_path}:" >&2
    tail -n 160 "${log_path}" >&2 || true
    exit 1
  fi
}

readonly ROOT_VERIFIER="${SCRIPT_DIR}/verify_irb1200_gripper_lab_runtime.py"
[[ -f "${ROOT_VERIFIER}" ]] || die "missing P3.1 runtime verifier: ${ROOT_VERIFIER}"
readonly ROOT_SCENE_CAPTURE="${SCRIPT_DIR}/capture_gazebo_lab_scene_evidence.py"
[[ -f "${ROOT_SCENE_CAPTURE}" ]] ||
  die "missing live Gazebo scene evidence helper: ${ROOT_SCENE_CAPTURE}"
run_step "P3.1 verifier syntax" "${OUTPUT_ROOT}/verifier_syntax.txt" \
  python3 -B -c 'import ast, pathlib, sys; ast.parse(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))' \
  "${ROOT_VERIFIER}"
run_step "live scene evidence helper syntax" "${OUTPUT_ROOT}/live_scene_capture_syntax.txt" \
  python3 -B -c 'import ast, pathlib, sys; ast.parse(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))' \
  "${ROOT_SCENE_CAPTURE}"

run_step "colcon build" "${OUTPUT_ROOT}/colcon_build.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/build" build \
    --base-paths "${SOURCE_ROOT}" \
    --packages-up-to abb_lab_moveit_config abb_lab_bringup \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Generated colcon setup files legitimately probe unset tracing variables.
set +u
# shellcheck disable=SC1091
source "${INSTALL_BASE}/setup.bash"
set -u

run_step "colcon test" "${OUTPUT_ROOT}/colcon_test.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/test" test \
    --base-paths "${SOURCE_ROOT}" \
    --packages-select abb_lab_description abb_lab_control abb_lab_gazebo abb_lab_moveit_config \
      abb_lab_gripper abb_lab_lab abb_lab_bringup \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --return-code-on-test-failure

run_step "colcon test-result" "${OUTPUT_ROOT}/colcon_test_result.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/test-result" test-result \
    --test-result-base "${BUILD_BASE}" --all --verbose

readonly SCENE_GENERATOR="${INSTALL_BASE}/abb_lab_lab/lib/abb_lab_lab/generate_scene_manifest.py"
readonly SCENE_CONFIG="${INSTALL_BASE}/abb_lab_lab/share/abb_lab_lab/config/manipulation_scene.yaml"
readonly SCENE_TEMPLATE="${INSTALL_BASE}/abb_lab_lab/share/abb_lab_lab/worlds/manipulation_lab.sdf"
readonly GRIPPER_LIMITS="${INSTALL_BASE}/abb_lab_gripper/share/abb_lab_gripper/config/two_finger_gripper_limits.yaml"
for required_file in "${SCENE_GENERATOR}" "${SCENE_CONFIG}" "${SCENE_TEMPLATE}" "${GRIPPER_LIMITS}"; do
  [[ -f "${required_file}" ]] || die "installed P3.1 runtime file is missing: ${required_file}"
done
[[ -x "${SCENE_GENERATOR}" ]] || die "installed scene generator is not executable"

generate_scene() {
  local seed="$1"
  local suffix="$2"
  run_step "generate scene seed ${seed} (${suffix})" "${SCENE_ROOT}/generate_${suffix}.txt" \
    python3 "${SCENE_GENERATOR}" \
      --config "${SCENE_CONFIG}" \
      --seed "${seed}" \
      --manifest-output "${SCENE_ROOT}/scene_${suffix}.json" \
      --world-template "${SCENE_TEMPLATE}" \
      --world-output "${SCENE_ROOT}/scene_${suffix}.sdf"
}
generate_scene "${P31_SCENE_SEED_SELECTED}" selected_a
generate_scene "${P31_SCENE_SEED_SELECTED}" selected_b
generate_scene "${P31_CONTRAST_SCENE_SEED}" contrast
cmp --silent "${SCENE_ROOT}/scene_selected_a.json" "${SCENE_ROOT}/scene_selected_b.json" ||
  die "same seed did not reproduce an identical scene manifest"
cmp --silent "${SCENE_ROOT}/scene_selected_a.sdf" "${SCENE_ROOT}/scene_selected_b.sdf" ||
  die "same seed did not reproduce an identical scene SDF"
if cmp --silent "${SCENE_ROOT}/scene_selected_a.json" "${SCENE_ROOT}/scene_contrast.json"; then
  die "different scene seeds produced the same manifest"
fi
if cmp --silent "${SCENE_ROOT}/scene_selected_a.sdf" "${SCENE_ROOT}/scene_contrast.sdf"; then
  die "different scene seeds produced the same SDF"
fi
run_step "validate selected seed ${P31_SCENE_SEED_SELECTED} SDF" \
  "${SCENE_ROOT}/selected_sdf_check.txt" \
  gz sdf -k "${SCENE_ROOT}/scene_selected_a.sdf"
run_step "validate contrast seed ${P31_CONTRAST_SCENE_SEED} SDF" \
  "${SCENE_ROOT}/contrast_sdf_check.txt" \
  gz sdf -k "${SCENE_ROOT}/scene_contrast.sdf"
scene_seed_reproducible=true

if ! prelaunch_nodes="$(
    timeout --kill-after=1s 4s ros2 node list --no-daemon --spin-time 2 \
      2>"${RUNTIME_LOG_ROOT}/domain_prelaunch_stderr.txt"
  )"; then
  sed -n '1,160p' "${RUNTIME_LOG_ROOT}/domain_prelaunch_stderr.txt" >&2 || true
  die "could not re-prove selected ROS domain ${ROS_DOMAIN_ID} was empty before launch"
fi
printf '%s\n' "${prelaunch_nodes}" >"${RUNTIME_LOG_ROOT}/domain_prelaunch_nodes.txt"
[[ -z "${prelaunch_nodes}" ]] ||
  die "selected ROS domain ${ROS_DOMAIN_ID} gained nodes before launch: ${prelaunch_nodes}"

readonly SELECTED_SCENE_WORLD="${SCENE_ROOT}/scene_selected_a.sdf"
setsid ros2 launch abb_lab_bringup irb1200_gripper_lab_headless.launch.py \
  world_path:="${SELECTED_SCENE_WORLD}" world_name:=manipulation_lab \
  >"${LAUNCH_LOG}" 2>&1 &
launch_pid="$!"
launch_pgid="${launch_pid}"
observed_launch_pgid="$(ps -o pgid= -p "${launch_pid}" | tr -d '[:space:]')"
[[ "${observed_launch_pgid}" == "${launch_pid}" ]] ||
  die "setsid launch PID ${launch_pid} does not own process group ${observed_launch_pgid}"

runtime_ready=false
runtime_deadline=$((SECONDS + 60))
while (( SECONDS < runtime_deadline )); do
  kill -0 "${launch_pid}" 2>/dev/null || {
    tail -n 240 "${LAUNCH_LOG}" >&2 || true
    die "launch exited before controllers and gripper action became ready"
  }
  timeout --kill-after=1s 5s ros2 control list_controllers \
    -c /controller_manager -v >"${CONTROLLERS_LOG}" 2>&1 || true
  timeout --kill-after=1s 5s ros2 node list --no-daemon --spin-time 2 \
    >"${NODES_LOG}" 2>&1 || true
  timeout --kill-after=1s 5s ros2 action list -t >"${ACTIONS_LOG}" 2>&1 || true
  if grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}" &&
      grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}" &&
      grep -Eq '^two_finger_gripper_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}" &&
      grep -Fxq /controller_manager "${NODES_LOG}" &&
      grep -Fxq /robot_state_publisher "${NODES_LOG}" &&
      grep -Eq '^/two_finger_gripper_controller/gripper_cmd[[:space:]]+\[control_msgs/action/ParallelGripperCommand\]$' "${ACTIONS_LOG}"; then
    runtime_ready=true
    break
  fi
  # Polling delay is test orchestration, not simulated control timing.
  sleep 0.25
done
if [[ "${runtime_ready}" != "true" ]]; then
  sed -n '1,220p' "${NODES_LOG}" >&2 || true
  sed -n '1,220p' "${ACTIONS_LOG}" >&2 || true
  tail -n 240 "${LAUNCH_LOG}" >&2 || true
  die "Gazebo gripper runtime did not become ready within the bounded startup window"
fi
controllers_active=true

timeout --kill-after=1s 5s ros2 control list_hardware_components \
  -c /controller_manager -v >"${HARDWARE_LOG}" 2>&1 ||
  die "could not inspect hardware components"
timeout --kill-after=1s 5s ros2 control list_hardware_interfaces \
  -c /controller_manager -v >"${INTERFACES_LOG}" 2>&1 ||
  die "could not inspect hardware interfaces"
timeout --kill-after=1s 5s ros2 param get /controller_manager enforce_command_limits \
  --timeout 5 >"${PARAMETER_LOG}" 2>&1 ||
  die "could not read controller_manager enforce_command_limits"
grep -Fq "GazeboSimSystem" "${HARDWARE_LOG}" ||
  die "GazeboSimSystem hardware component was not observed"
grep -Eqi 'state:[[:space:]].*active|label[=:][[:space:]]*active' "${HARDWARE_LOG}" ||
  die "GazeboSimSystem hardware component is not active"
grep -Eq 'Boolean value is:[[:space:]]+True|^[[:space:]]*True[[:space:]]*$' "${PARAMETER_LOG}" ||
  die "controller_manager enforce_command_limits is not true"
hardware_resources=true

command_section="$(awk '
  /^[[:space:]]*command interfaces[[:space:]]*$/ {capture=1; next}
  /^[[:space:]]*state interfaces[[:space:]]*$/ {exit}
  capture && NF {sub(/^[[:space:]]*/, ""); print}
' "${INTERFACES_LOG}")"
state_section="$(awk '
  /^[[:space:]]*state interfaces[[:space:]]*$/ {capture=1; next}
  capture && NF {sub(/^[[:space:]]*/, ""); print}
' "${INTERFACES_LOG}")"
command_count="$(printf '%s\n' "${command_section}" | sed '/^[[:space:]]*$/d' | wc -l | tr -d '[:space:]')"
[[ "${command_count}" == "7" ]] ||
  die "expected exactly seven command interfaces, observed ${command_count}"
for joint_name in joint_{1..6} "${RIGHT_FINGER}"; do
  command_matches="$(grep -Ec "^${joint_name}/position[[:space:]]+\\[double\\][[:space:]]+\\[available\\][[:space:]]+\\[claimed\\][[:space:]]*$" <<<"${command_section}" || true)"
  [[ "${command_matches}" == "1" ]] ||
    die "${joint_name}/position is not the unique available+claimed command interface"
done
if grep -Eq "^${LEFT_FINGER}/" <<<"${command_section}"; then
  die "mimicked left finger unexpectedly exposes a command interface"
fi
state_count="$(printf '%s\n' "${state_section}" | sed '/^[[:space:]]*$/d' | wc -l | tr -d '[:space:]')"
[[ "${state_count}" == "22" ]] ||
  die "expected exactly 22 state interfaces, observed ${state_count}"
for joint_name in joint_{1..6}; do
  for interface_name in effort position velocity; do
    state_matches="$(grep -Ec "^${joint_name}/${interface_name}[[:space:]]+\\[double\\][[:space:]]*$" <<<"${state_section}" || true)"
    [[ "${state_matches}" == "1" ]] ||
      die "missing or duplicate state interface ${joint_name}/${interface_name}"
  done
done
for joint_name in "${RIGHT_FINGER}" "${LEFT_FINGER}"; do
  for interface_name in position velocity; do
    state_matches="$(grep -Ec "^${joint_name}/${interface_name}[[:space:]]+\\[double\\][[:space:]]*$" <<<"${state_section}" || true)"
    [[ "${state_matches}" == "1" ]] ||
      die "missing or duplicate state interface ${joint_name}/${interface_name}"
  done
done

for joint_name in joint_{1..6} "${RIGHT_FINGER}"; do
  grep -Fq "Creating JointSaturationLimiter for joint '${joint_name}'" "${LAUNCH_LOG}" ||
    die "missing limiter initialization log for ${joint_name}"
done
grep -Fq "Enforcing command limits is enabled" "${LAUNCH_LOG}" ||
  die "controller_manager did not log enabled command-limit enforcement"
if grep -Fq "Enforcing command limits is disabled" "${LAUNCH_LOG}"; then
  die "controller_manager logged disabled command-limit enforcement"
fi
limiter_initialized=true

timeout --kill-after=1s 5s gz model --list >"${MODELS_LOG}" 2>&1 ||
  die "could not query Gazebo models"
grep -Fq "Requesting state for world [manipulation_lab]" "${MODELS_LOG}" ||
  die "Gazebo model query did not resolve the expected manipulation_lab world"
for model_name in "${EXPECTED_MODELS[@]}"; do
  model_matches="$(grep -Ec "^[[:space:]]*-[[:space:]]+${model_name}[[:space:]]*$" "${MODELS_LOG}" || true)"
  [[ "${model_matches}" == "1" ]] ||
    die "expected exactly one Gazebo model named ${model_name}"
done
model_count="$(grep -Ec '^[[:space:]]*-[[:space:]]+' "${MODELS_LOG}" || true)"
[[ "${model_count}" == "${EXPECTED_MODEL_COUNT}" ]] ||
  die "expected exactly ${EXPECTED_MODEL_COUNT} models, observed ${model_count}"
project_spawn=true
lab_objects_present=true

if ! timeout --kill-after=5s 75s python3 "${ROOT_VERIFIER}" \
    --timeout 55 --limits "${GRIPPER_LIMITS}" --evidence "${VERIFIER_EVIDENCE}" \
    >"${VERIFIER_LOG}" 2>&1; then
  sed -n '1,260p' "${VERIFIER_LOG}" >&2 || true
  die "gripper verifier did not establish all required runtime evidence"
fi
grep -Fxq "GRIPPER_ACTION_SERVER=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove the gripper action server"
grep -Fxq "GRIPPER_OPEN_CLOSE_OPEN=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove a real open-close-open sequence"
gripper_open_close_open=true
grep -Fxq "GRIPPER_MIRROR_STATE=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove mirrored finger states"
gripper_mirror_state=true
grep -Fxq "GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE" "${VERIFIER_LOG}" ||
  die "verifier did not preserve the no-contact-evidence grasp status"

timeout --kill-after=1s 5s ros2 control list_controllers \
  -c /controller_manager -v >"${CONTROLLERS_LOG}.after" 2>&1 ||
  die "could not re-query controllers after gripper actions"
for controller_name in joint_state_broadcaster joint_trajectory_controller two_finger_gripper_controller; do
  grep -Eq "^${controller_name}[[:space:]].*[[:space:]]active([[:space:]]|$)" \
    "${CONTROLLERS_LOG}.after" || die "${controller_name} did not remain active"
done

# The preceding list check establishes that the seeded laboratory survived the
# real gripper actions.  Capture each *live* Gazebo pose now.  The JSON helper
# deliberately parses these responses rather than the generated SDF/manifest;
# this makes two completed runs independently comparable by selected seed.
timeout --kill-after=1s 5s gz model --list >"${MODELS_AFTER_LOG}" 2>&1 ||
  die "could not re-query Gazebo models after gripper actions"
grep -Fq "Requesting state for world [manipulation_lab]" "${MODELS_AFTER_LOG}" ||
  die "post-action Gazebo model query did not resolve the expected manipulation_lab world"
for model_name in "${EXPECTED_MODELS[@]}"; do
  model_matches="$(grep -Ec "^[[:space:]]*-[[:space:]]+${model_name}[[:space:]]*$" "${MODELS_AFTER_LOG}" || true)"
  [[ "${model_matches}" == "1" ]] ||
    die "post-action Gazebo query did not report exactly one ${model_name}"
done
model_count_after="$(grep -Ec '^[[:space:]]*-[[:space:]]+' "${MODELS_AFTER_LOG}" || true)"
[[ "${model_count_after}" == "${EXPECTED_MODEL_COUNT}" ]] ||
  die "post-action Gazebo query expected ${EXPECTED_MODEL_COUNT} models, observed ${model_count_after}"

capture_arguments=(
  --seed "${P31_SCENE_SEED_SELECTED}"
  --world-name manipulation_lab
  --run-metadata "${OUTPUT_ROOT}/run_metadata.txt"
  --models-log "${MODELS_AFTER_LOG}"
  --output "${LIVE_SCENE_EVIDENCE}"
)
for model_name in "${EXPECTED_MODELS[@]}"; do
  pose_log="${LIVE_SCENE_POSE_ROOT}/${model_name}.txt"
  timeout --kill-after=1s 5s gz model -m "${model_name}" -p >"${pose_log}" 2>&1 ||
    die "could not capture the live Gazebo pose for ${model_name}"
  capture_arguments+=(--pose "${model_name}=${pose_log}")
done
run_step "capture live Gazebo scene evidence" "${LIVE_SCENE_CAPTURE_LOG}" \
  python3 "${ROOT_SCENE_CAPTURE}" "${capture_arguments[@]}"
grep -Fxq "GAZEBO_LIVE_SCENE_POSES=PASS" "${LIVE_SCENE_CAPTURE_LOG}" ||
  die "live scene helper did not prove Gazebo pose capture"
grep -Fxq "LAB_OBJECTS_PRESENT=PASS" "${LIVE_SCENE_CAPTURE_LOG}" ||
  die "live scene helper did not preserve laboratory object presence"
grep -Fxq "LIVE_SCENE_SELECTED_SEED=${P31_SCENE_SEED_SELECTED}" "${LIVE_SCENE_CAPTURE_LOG}" ||
  die "live scene helper did not bind evidence to the selected seed"
[[ -s "${LIVE_SCENE_EVIDENCE}" ]] ||
  die "live Gazebo scene evidence JSON was not written"
live_scene_pose_capture=true

capture_working_source_manifest >"${SOURCE_MANIFEST_AFTER}"
cmp --silent "${SOURCE_MANIFEST_BEFORE}" "${SOURCE_MANIFEST_AFTER}" ||
  die "working source files changed while the runtime gate was executing"
run_completed=true
