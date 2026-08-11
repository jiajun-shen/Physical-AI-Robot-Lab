#!/usr/bin/env bash
# Run the project-owned P2.2 ABB MoveIt gate in an isolated, headless domain.
#
# This is deliberately separate from the Phase 1 control gate.  It builds an
# ephemeral overlay, launches the one owned Gazebo/controller-manager stack,
# starts one move_group, and accepts A02 only when the installed verifier has
# observed real joint execution, pose execution, and collision rejection.
set -eo pipefail

if [[ "${P22_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P22_WATCHDOG_ACTIVE=1
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

die() {
  echo "ERROR: $*" >&2
  exit 1
}

for required_command in awk cmp colcon find findmnt flock git grep gz pgrep ps \
    python3 realpath ros2 sed setsid sha256sum sort tail timeout tr xargs; do
  command -v "${required_command}" >/dev/null 2>&1 ||
    die "required command is unavailable: ${required_command}"
done

if [[ $# -gt 1 ]]; then
  die "usage: $0 [new-or-empty-output-directory]"
fi

if [[ $# -eq 0 ]]; then
  output_root="$(mktemp -d /var/tmp/physical-ai-p22.XXXXXX)"
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
readonly LAUNCH_LOG="${RUNTIME_LOG_ROOT}/launch.log"
readonly CONTROLLERS_LOG="${RUNTIME_LOG_ROOT}/controllers.txt"
readonly HARDWARE_LOG="${RUNTIME_LOG_ROOT}/hardware_components.txt"
readonly INTERFACES_LOG="${RUNTIME_LOG_ROOT}/hardware_interfaces.txt"
readonly PARAMETER_LOG="${RUNTIME_LOG_ROOT}/enforce_command_limits.txt"
readonly NODES_LOG="${RUNTIME_LOG_ROOT}/nodes.txt"
readonly SERVICES_LOG="${RUNTIME_LOG_ROOT}/services.txt"
readonly ACTIONS_LOG="${RUNTIME_LOG_ROOT}/actions.txt"
readonly MODELS_LOG="${RUNTIME_LOG_ROOT}/models.txt"
readonly VERIFIER_LOG="${RUNTIME_LOG_ROOT}/moveit_verifier.txt"
readonly VERIFIER_EVIDENCE="${RUNTIME_LOG_ROOT}/moveit_evidence.json"
readonly SOURCE_MANIFEST_BEFORE="${OUTPUT_ROOT}/working_source_before.sha256"
readonly SOURCE_MANIFEST_AFTER="${OUTPUT_ROOT}/working_source_after.sha256"
readonly RESULT_LOG="${OUTPUT_ROOT}/result.txt"
mkdir -p "${COLCON_LOG_BASE}" "${RUNTIME_LOG_ROOT}" "${ROS_HOME_PATH}"

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
    exec {candidate_fd}>"/var/tmp/physical-ai-p22-domain-${candidate}.lock"
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
export GZ_PARTITION="physical_ai_p22_${ROS_DOMAIN_ID}_${BASHPID}"
readonly GZ_PARTITION

if ! preexisting_nodes="$(
    timeout --kill-after=1s 4s ros2 node list --no-daemon --spin-time 2 \
      2>"${RUNTIME_LOG_ROOT}/domain_preflight_stderr.txt"
  )"; then
  cat "${RUNTIME_LOG_ROOT}/domain_preflight_stderr.txt" >&2 || true
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
  echo "MEM_AVAILABLE_KIB_AT_START=${available_memory_kib}"
  echo "OUTPUT_FILESYSTEM=${output_filesystem}"
  echo "WORKING_SOURCE_MANIFEST_SHA256=${source_manifest_sha256}"
} >"${OUTPUT_ROOT}/run_metadata.txt"

launch_pid=""
launch_pgid=""
cleanup_started=false
run_completed=false
move_group_ready=false
joint_execution=false
pose_execution=false
home_pose_home=false
collision_rejection=false
launch_exit_status="NOT_WAITED"
move_group_shutdown_observation="NOT_OBSERVED"

process_group_alive() {
  [[ -n "${launch_pgid}" ]] && pgrep -g "${launch_pgid}" >/dev/null 2>&1
}

wait_for_group_exit() {
  local attempts="$1"
  local attempt_index
  for ((attempt_index=0; attempt_index<attempts; attempt_index++)); do
    process_group_alive || return 0
    # Polling delay is test orchestration, not simulated control timing.
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
  if grep -Eq '\[ERROR\] \[move_group-[^]]+\]: process has died .*exit code -11' \
      "${LAUNCH_LOG}"; then
    move_group_shutdown_observation="SIGSEGV_DURING_REQUESTED_CLEANUP"
  elif [[ "${launch_exit_status}" == "0" ]]; then
    move_group_shutdown_observation="GRACEFUL"
  else
    move_group_shutdown_observation="NONZERO_LAUNCH_EXIT_${launch_exit_status}"
  fi

  if [[ "${status}" -eq 0 && "${run_completed}" == "true" ]]; then
    {
      echo "PROJECT_ABB_GAZEBO_SPAWN=PASS"
      echo "HARDWARE_RESOURCES=PASS"
      echo "CONTROLLERS_ACTIVE=PASS"
      echo "LIMITER_RUNTIME_INITIALIZED=PASS"
      echo "MOVE_GROUP_READY=PASS"
      echo "MOVEIT_JOINT_EXECUTION=PASS"
      echo "MOVEIT_POSE_EXECUTION=PASS"
      echo "MOVEIT_HOME_POSE_HOME=PASS"
      echo "MOVEIT_COLLISION_REJECTION=PASS"
      echo "GUI_RENDERING=NOT_RUN"
      echo "REAL_ROBOT_READY=false"
      if [[ "${move_group_ready}" == "true" && "${joint_execution}" == "true" && \
          "${pose_execution}" == "true" && "${home_pose_home}" == "true" && \
          "${collision_rejection}" == "true" ]]; then
        echo "A02=PASS"
      else
        echo "A02=NOT_STARTED"
        status=1
      fi
      echo "ABB_MOVEIT_HEADLESS_RUNTIME=PASS"
      echo "CLEANUP_NO_RESIDUAL_PROCESS_GROUP=PASS"
      echo "LAUNCH_EXIT_STATUS=${launch_exit_status}"
      echo "MOVE_GROUP_SHUTDOWN_OBSERVATION=${move_group_shutdown_observation}"
      if [[ "${move_group_shutdown_observation}" == "GRACEFUL" ]]; then
        echo "CLEANUP_VERIFIED=PASS"
      else
        echo "CLEANUP_VERIFIED=NO_RESIDUALS_WITH_ABNORMAL_MOVE_GROUP_EXIT"
      fi
      echo "TESTED_COMMIT=${tested_commit}"
      echo "SOURCE_WORKTREE_CLEAN=${source_worktree_clean}"
      echo "WORKING_SOURCE_MANIFEST_SHA256=${source_manifest_sha256}"
      echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
      echo "GZ_PARTITION=${GZ_PARTITION}"
      echo "OUTPUT_ROOT=${OUTPUT_ROOT}"
    } >"${RESULT_LOG}"
    cat "${RESULT_LOG}"
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

run_step "colcon build" "${OUTPUT_ROOT}/colcon_build.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/build" build \
    --base-paths "${SOURCE_ROOT}" \
    --packages-up-to abb_lab_moveit_config \
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
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --return-code-on-test-failure

run_step "colcon test-result" "${OUTPUT_ROOT}/colcon_test_result.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/test-result" test-result \
    --test-result-base "${BUILD_BASE}" --all --verbose

readonly MOVEIT_VERIFIER="${INSTALL_BASE}/abb_lab_moveit_config/lib/abb_lab_moveit_config/verify_moveit_runtime.py"
readonly SMOKE_CONFIG="${INSTALL_BASE}/abb_lab_moveit_config/share/abb_lab_moveit_config/config/irb1200_moveit_smoke.yaml"
for required_file in "${MOVEIT_VERIFIER}" "${SMOKE_CONFIG}"; do
  [[ -f "${required_file}" ]] || die "installed P2.2 runtime file is missing: ${required_file}"
done

if ! prelaunch_nodes="$(
    timeout --kill-after=1s 4s ros2 node list --no-daemon --spin-time 2 \
      2>"${RUNTIME_LOG_ROOT}/domain_prelaunch_stderr.txt"
  )"; then
  cat "${RUNTIME_LOG_ROOT}/domain_prelaunch_stderr.txt" >&2 || true
  die "could not re-prove selected ROS domain ${ROS_DOMAIN_ID} was empty before launch"
fi
printf '%s\n' "${prelaunch_nodes}" >"${RUNTIME_LOG_ROOT}/domain_prelaunch_nodes.txt"
[[ -z "${prelaunch_nodes}" ]] ||
  die "selected ROS domain ${ROS_DOMAIN_ID} gained nodes before launch: ${prelaunch_nodes}"

setsid ros2 launch abb_lab_moveit_config irb1200_moveit_headless.launch.py \
  >"${LAUNCH_LOG}" 2>&1 &
launch_pid="$!"
launch_pgid="${launch_pid}"
observed_launch_pgid="$(ps -o pgid= -p "${launch_pid}" | tr -d '[:space:]')"
[[ "${observed_launch_pgid}" == "${launch_pgid}" ]] ||
  die "setsid launch PID ${launch_pid} does not own process group ${observed_launch_pgid}"

runtime_ready=false
runtime_deadline=$((SECONDS + 60))
while (( SECONDS < runtime_deadline )); do
  kill -0 "${launch_pid}" 2>/dev/null || {
    tail -n 240 "${LAUNCH_LOG}" >&2 || true
    die "launch exited before MoveIt and controllers became ready"
  }
  timeout --kill-after=1s 5s ros2 control list_controllers \
    -c /controller_manager -v >"${CONTROLLERS_LOG}" 2>&1 || true
  timeout --kill-after=1s 5s ros2 node list --no-daemon --spin-time 2 \
    >"${NODES_LOG}" 2>&1 || true
  timeout --kill-after=1s 5s ros2 service list --no-daemon --spin-time 2 \
    >"${SERVICES_LOG}" 2>&1 || true
  timeout --kill-after=1s 5s ros2 action list -t >"${ACTIONS_LOG}" 2>&1 || true
  if grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}" &&
      grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}" &&
      grep -Fxq /controller_manager "${NODES_LOG}" &&
      grep -Fxq /robot_state_publisher "${NODES_LOG}" &&
      grep -Fxq /move_group "${NODES_LOG}" &&
      grep -Fxq /plan_kinematic_path "${SERVICES_LOG}" &&
      grep -Fxq /compute_fk "${SERVICES_LOG}" &&
      grep -Fxq /check_state_validity "${SERVICES_LOG}" &&
      grep -Fxq /apply_planning_scene "${SERVICES_LOG}" &&
      grep -Eq '^/execute_trajectory[[:space:]]+\[moveit_msgs/action/ExecuteTrajectory\]$' "${ACTIONS_LOG}"; then
    runtime_ready=true
    break
  fi
  # Polling delay is test orchestration, not simulated control timing.
  sleep 0.25
done
if [[ "${runtime_ready}" != "true" ]]; then
  cat "${NODES_LOG}" "${SERVICES_LOG}" "${ACTIONS_LOG}" >&2 || true
  tail -n 240 "${LAUNCH_LOG}" >&2 || true
  die "MoveIt/Gazebo runtime did not become ready within the bounded startup window"
fi
move_group_ready=true

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
for joint_name in joint_{1..6}; do
  grep -Eq "${joint_name}/position[[:space:]]+\\[double\\][[:space:]]+\\[available\\][[:space:]]+\\[claimed\\]" "${INTERFACES_LOG}" ||
    die "${joint_name}/position is not available and claimed"
  grep -Fq "Creating JointSaturationLimiter for joint '${joint_name}'" "${LAUNCH_LOG}" ||
    die "missing limiter initialization log for ${joint_name}"
done
grep -Fq "Enforcing command limits is enabled" "${LAUNCH_LOG}" ||
  die "controller_manager did not log enabled command-limit enforcement"
if grep -Fq "Enforcing command limits is disabled" "${LAUNCH_LOG}"; then
  die "controller_manager logged disabled command-limit enforcement"
fi

timeout --kill-after=1s 5s gz model --list >"${MODELS_LOG}" 2>&1 ||
  die "could not query Gazebo models"
model_count="$(grep -Ec '^[[:space:]]*-[[:space:]]+abb_irb1200[[:space:]]*$' "${MODELS_LOG}" || true)"
[[ "${model_count}" -eq 1 ]] ||
  die "expected exactly one abb_irb1200 model; observed ${model_count}"

if ! timeout --kill-after=5s 90s python3 "${MOVEIT_VERIFIER}" \
    --timeout 55 --smoke-config "${SMOKE_CONFIG}" --evidence "${VERIFIER_EVIDENCE}" \
    >"${VERIFIER_LOG}" 2>&1; then
  cat "${VERIFIER_LOG}" >&2 || true
  die "MoveIt verifier did not establish all required runtime evidence"
fi
grep -Fxq "MOVEIT_JOINT_EXECUTION=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove MoveIt joint execution"
joint_execution=true
grep -Fxq "MOVEIT_POSE_EXECUTION=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove MoveIt pose execution"
pose_execution=true
grep -Fxq "MOVEIT_HOME_POSE_HOME=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove HOME -> specified Pose -> HOME"
home_pose_home=true
grep -Fxq "MOVEIT_COLLISION_REJECTION=PASS" "${VERIFIER_LOG}" ||
  die "verifier did not prove MoveIt collision rejection"
collision_rejection=true

timeout --kill-after=1s 5s ros2 control list_controllers \
  -c /controller_manager -v >"${CONTROLLERS_LOG}.after" 2>&1 ||
  die "could not re-query controllers after MoveIt execution"
for controller_name in joint_state_broadcaster joint_trajectory_controller; do
  grep -Eq "^${controller_name}[[:space:]].*[[:space:]]active([[:space:]]|$)" \
    "${CONTROLLERS_LOG}.after" || die "${controller_name} did not remain active"
done
timeout --kill-after=1s 5s ros2 node list --no-daemon --spin-time 2 \
  >"${NODES_LOG}.after" 2>&1 || die "could not re-query runtime nodes"
for required_node in /controller_manager /robot_state_publisher /move_group; do
  grep -Fxq "${required_node}" "${NODES_LOG}.after" ||
    die "required runtime node did not persist: ${required_node}"
done

capture_working_source_manifest >"${SOURCE_MANIFEST_AFTER}"
cmp --silent "${SOURCE_MANIFEST_BEFORE}" "${SOURCE_MANIFEST_AFTER}" ||
  die "working source files changed while the runtime gate was executing"
run_completed=true
