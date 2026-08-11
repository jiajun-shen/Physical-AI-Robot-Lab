#!/usr/bin/env bash
set -eo pipefail

if [[ "${P13B_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P13B_WATCHDOG_ACTIVE=1
  # Default timeout process-group handling also bounds synchronous colcon children.
  exec timeout --signal=TERM --kill-after=30s 180s "$0" "$@"
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
  output_root="$(mktemp -d /var/tmp/physical-ai-p13b.XXXXXX)"
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
readonly HARDWARE_COMPONENTS_LOG="${RUNTIME_LOG_ROOT}/hardware_components.txt"
readonly HARDWARE_INTERFACES_LOG="${RUNTIME_LOG_ROOT}/hardware_interfaces.txt"
readonly PARAMETER_LOG="${RUNTIME_LOG_ROOT}/enforce_command_limits.txt"
readonly NODES_LOG="${RUNTIME_LOG_ROOT}/nodes.txt"
readonly ACTIONS_LOG="${RUNTIME_LOG_ROOT}/actions.txt"
readonly ACTION_LOG="${RUNTIME_LOG_ROOT}/bounded_action.txt"
readonly ACTION_FEEDBACK_LOG="${RUNTIME_LOG_ROOT}/bounded_action_feedback_check.txt"
readonly CLOCK_LOG="${RUNTIME_LOG_ROOT}/clock.txt"
readonly INITIAL_STATE_LOG="${RUNTIME_LOG_ROOT}/initial_joint_state.txt"
readonly TARGET_STATE_LOG="${RUNTIME_LOG_ROOT}/target_joint_state.txt"
readonly MODELS_BEFORE_LOG="${RUNTIME_LOG_ROOT}/models_before.txt"
readonly MODELS_AFTER_LOG="${RUNTIME_LOG_ROOT}/models_after.txt"
readonly MODEL_POSE_BEFORE_LOG="${RUNTIME_LOG_ROOT}/model_pose_before.txt"
readonly MODEL_POSE_AFTER_LOG="${RUNTIME_LOG_ROOT}/model_pose_after.txt"
readonly MODEL_JOINTS_LOG="${RUNTIME_LOG_ROOT}/model_joints.txt"
readonly POSE_COMPARISON_LOG="${RUNTIME_LOG_ROOT}/pose_comparison.txt"
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
    exec {candidate_fd}>"/var/tmp/physical-ai-p13b-domain-${candidate}.lock"
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
export GZ_PARTITION="physical_ai_p13b_${ROS_DOMAIN_ID}_${BASHPID}"
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
  # The watchdog still has an unconditional KILL deadline after 30 seconds.
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
    wait "${launch_pid}" 2>/dev/null || true
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
  if [[ "${status}" -eq 0 && "${run_completed}" == "true" ]]; then
    {
      echo "PROJECT_ABB_GAZEBO_SPAWN=PASS"
      echo "HARDWARE_RESOURCES=PASS"
      echo "CONTROLLERS_ACTIVE=PASS"
      echo "LIMITER_RUNTIME_INITIALIZED=PASS"
      echo "BOUNDED_TRAJECTORY=PASS"
      echo "CLAMP_PROBE=NOT_RUN"
      echo "CLAMP_BEHAVIOR_VERIFIED=false"
      echo "GUI_RENDERING=NOT_RUN"
      echo "A02=NOT_STARTED"
      echo "REAL_ROBOT_READY=false"
      echo "ABB_HEADLESS_RUNTIME=PASS"
      echo "CLEANUP_VERIFIED=PASS"
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

run_colcon_step() {
  local label="$1"
  local log_path="$2"
  shift 2
  if ! "$@" >"${log_path}" 2>&1; then
    echo "ERROR: ${label} failed; tail of ${log_path}:" >&2
    tail -n 160 "${log_path}" >&2 || true
    exit 1
  fi
}

run_colcon_step "repository harness source contract" \
  "${OUTPUT_ROOT}/repository_source_test.txt" \
  python3 "${REPOSITORY_ROOT}/tests/phase1/test_irb1200_headless_harness_source.py"

run_colcon_step "colcon build" "${OUTPUT_ROOT}/colcon_build.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/build" build \
    --base-paths "${SOURCE_ROOT}" \
    --packages-up-to abb_lab_gazebo \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Generated colcon setup files legitimately probe unset tracing variables.
set +u
# shellcheck disable=SC1091
source "${INSTALL_BASE}/setup.bash"
set -u

run_colcon_step "colcon test" "${OUTPUT_ROOT}/colcon_test.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/test" test \
    --base-paths "${SOURCE_ROOT}" \
    --packages-select abb_lab_description abb_lab_control abb_lab_gazebo \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --return-code-on-test-failure

run_colcon_step "colcon test-result" "${OUTPUT_ROOT}/colcon_test_result.txt" \
  colcon --log-base "${COLCON_LOG_BASE}/test-result" test-result \
    --test-result-base "${BUILD_BASE}" --all --verbose

readonly CONTROL_SHARE="${INSTALL_BASE}/abb_lab_control/share/abb_lab_control"
readonly DESCRIPTION_SHARE="${INSTALL_BASE}/abb_lab_description/share/abb_lab_description"
readonly RUNTIME_VERIFIER="${INSTALL_BASE}/abb_lab_gazebo/lib/abb_lab_gazebo/verify_irb1200_runtime.py"
readonly SMOKE_CONFIG="${CONTROL_SHARE}/config/irb1200_bounded_smoke.yaml"
readonly LIMITS_CONFIG="${DESCRIPTION_SHARE}/config/irb1200_5_90_control_limits.yaml"
for required_file in "${RUNTIME_VERIFIER}" "${SMOKE_CONFIG}" "${LIMITS_CONFIG}"; do
  [[ -f "${required_file}" ]] || die "installed runtime file is missing: ${required_file}"
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

setsid ros2 launch abb_lab_gazebo irb1200_headless.launch.py \
  >"${LAUNCH_LOG}" 2>&1 &
launch_pid="$!"
launch_pgid="${launch_pid}"
observed_launch_pgid="$(ps -o pgid= -p "${launch_pid}" | tr -d '[:space:]')"
[[ "${observed_launch_pgid}" == "${launch_pgid}" ]] ||
  die "setsid launch PID ${launch_pid} does not own process group ${observed_launch_pgid}"

controllers_ready=false
controller_ready_deadline=$((SECONDS + 40))
while (( SECONDS < controller_ready_deadline )); do
  kill -0 "${launch_pid}" 2>/dev/null || {
    tail -n 200 "${LAUNCH_LOG}" >&2 || true
    die "launch exited before controllers became active"
  }
  if timeout --kill-after=1s 5s ros2 control list_controllers \
      -c /controller_manager -v >"${CONTROLLERS_LOG}" 2>&1 &&
      grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}" &&
      grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' "${CONTROLLERS_LOG}"; then
    controllers_ready=true
    break
  fi
  # Polling delay is test orchestration, not simulated control timing.
  sleep 0.25
done
if [[ "${controllers_ready}" != "true" ]]; then
  cat "${CONTROLLERS_LOG}" >&2 || true
  tail -n 200 "${LAUNCH_LOG}" >&2 || true
  die "controllers did not become active within the bounded startup window"
fi

timeout --kill-after=1s 5s ros2 control list_hardware_components \
  -c /controller_manager -v >"${HARDWARE_COMPONENTS_LOG}" 2>&1 ||
  die "could not inspect hardware components"
timeout --kill-after=1s 5s ros2 control list_hardware_interfaces \
  -c /controller_manager -v >"${HARDWARE_INTERFACES_LOG}" 2>&1 ||
  die "could not inspect hardware interfaces"
timeout --kill-after=1s 5s ros2 param get /controller_manager enforce_command_limits \
  --timeout 5 >"${PARAMETER_LOG}" 2>&1 ||
  die "could not read controller_manager enforce_command_limits"

grep -Fq "GazeboSimSystem" "${HARDWARE_COMPONENTS_LOG}" ||
  die "GazeboSimSystem hardware component was not observed"
hardware_component_count="$(
  grep -Ec '^Hardware Component [0-9]+[[:space:]]*$' "${HARDWARE_COMPONENTS_LOG}" || true
)"
[[ "${hardware_component_count}" -eq 1 ]] ||
  die "expected exactly one hardware component; observed ${hardware_component_count}"
grep -Eqi 'state:[[:space:]].*active|label[=:][[:space:]]*active' \
  "${HARDWARE_COMPONENTS_LOG}" ||
  die "GazeboSimSystem hardware component is not active"
grep -Eq 'Boolean value is:[[:space:]]+True|^[[:space:]]*True[[:space:]]*$' \
  "${PARAMETER_LOG}" ||
  die "controller_manager enforce_command_limits is not true"

timeout --kill-after=1s 5s ros2 node list --no-daemon --spin-time 2 \
  >"${NODES_LOG}" 2>&1 ||
  die "could not inspect runtime ROS nodes"
for required_node in /controller_manager /robot_state_publisher; do
  grep -Fxq "${required_node}" "${NODES_LOG}" ||
    die "required runtime node is missing: ${required_node}"
done

controller_status_lines="$(
  grep -E '^[[:alnum:]_]+[[:space:]]+[^[:space:]]+[[:space:]]+(active|inactive|unconfigured|finalized)([[:space:]]|$)' \
    "${CONTROLLERS_LOG}" || true
)"
controller_status_count="$(printf '%s\n' "${controller_status_lines}" | grep -c . || true)"
[[ "${controller_status_count}" -eq 2 ]] ||
  die "expected exactly two controllers; observed ${controller_status_count}"
unexpected_controllers="$(
  printf '%s\n' "${controller_status_lines}" |
    grep -Ev '^(joint_state_broadcaster|joint_trajectory_controller)[[:space:]]' || true
)"
[[ -z "${unexpected_controllers}" ]] ||
  die "unexpected controller observed: ${unexpected_controllers}"

command_interface_count=0
for joint_index in {1..6}; do
  joint_name="joint_${joint_index}"
  grep -Eq "${joint_name}/position[[:space:]]+\[double\][[:space:]]+\[available\][[:space:]]+\[claimed\]" \
    "${HARDWARE_INTERFACES_LOG}" ||
    die "${joint_name}/position is not available and claimed"
  command_interface_count=$((command_interface_count + 1))
  for state_name in position velocity effort; do
    sed -n '/state interfaces/,$p' "${HARDWARE_INTERFACES_LOG}" |
      grep -Eq "${joint_name}/${state_name}([[:space:]]|$|\[)" ||
      die "${joint_name}/${state_name} state interface was not observed"
  done
done
[[ "${command_interface_count}" -eq 6 ]] ||
  die "did not validate exactly six command interfaces"

command_section="$(sed -n '/command interfaces/,/state interfaces/p' "${HARDWARE_INTERFACES_LOG}")"
observed_command_count="$(
  printf '%s\n' "${command_section}" |
    grep -Ec '^[[:space:]]+[^[:space:]]+/[^[:space:]]+' || true
)"
[[ "${observed_command_count}" -eq 6 ]] ||
  die "expected six total command interfaces; observed ${observed_command_count}"
unexpected_command_interfaces="$(
  printf '%s\n' "${command_section}" |
    grep -E '^[[:space:]]+[^[:space:]]+/[^[:space:]]+' |
    grep -Ev '^[[:space:]]+joint_[1-6]/position[[:space:]]+\[double\][[:space:]]+\[available\][[:space:]]+\[claimed\][[:space:]]*$' || true
)"
[[ -z "${unexpected_command_interfaces}" ]] ||
  die "unexpected command interfaces observed: ${unexpected_command_interfaces}"
state_section="$(sed -n '/state interfaces/,$p' "${HARDWARE_INTERFACES_LOG}")"
observed_state_count="$(
  printf '%s\n' "${state_section}" |
    grep -Ec '^[[:space:]]+[^[:space:]]+/[^[:space:]]+' || true
)"
[[ "${observed_state_count}" -eq 18 ]] ||
  die "expected 18 total state interfaces; observed ${observed_state_count}"
unexpected_state_interfaces="$(
  printf '%s\n' "${state_section}" |
    grep -E '^[[:space:]]+[^[:space:]]+/[^[:space:]]+' |
    grep -Ev '^[[:space:]]+joint_[1-6]/(position|velocity|effort)[[:space:]]+\[double\][[:space:]]*$' || true
)"
[[ -z "${unexpected_state_interfaces}" ]] ||
  die "unexpected state interfaces observed: ${unexpected_state_interfaces}"

for joint_index in {1..6}; do
  limiter_pattern="Creating JointSaturationLimiter for joint 'joint_${joint_index}'"
  grep -Fq "${limiter_pattern}" "${LAUNCH_LOG}" ||
    die "missing limiter initialization log for joint_${joint_index}"
done
grep -Fq "Enforcing command limits is enabled" "${LAUNCH_LOG}" ||
  die "controller_manager did not log enabled command-limit enforcement"
if grep -Fq "Enforcing command limits is disabled" "${LAUNCH_LOG}"; then
  die "controller_manager logged disabled command-limit enforcement"
fi
if grep -Eqi 'unable to find (uri|file)|failed to (find|load).*(mesh|dae|stl)|could not resolve.*(mesh|model)' \
    "${LAUNCH_LOG}"; then
  die "Gazebo launch log contains a mesh/resource resolution failure"
fi

timeout --kill-after=1s 5s gz model --list >"${MODELS_BEFORE_LOG}" 2>&1 ||
  die "could not query Gazebo models"
model_count="$(grep -Ec '^[[:space:]]*-[[:space:]]+abb_irb1200[[:space:]]*$' "${MODELS_BEFORE_LOG}" || true)"
[[ "${model_count}" -eq 1 ]] || {
  cat "${MODELS_BEFORE_LOG}" >&2
  die "expected exactly one abb_irb1200 model; observed ${model_count}"
}
if grep -Eq '^[[:space:]]*-[[:space:]]+abb_irb1200_[0-9]+[[:space:]]*$' \
    "${MODELS_BEFORE_LOG}"; then
  die "Gazebo unexpectedly renamed an ABB entity"
fi
timeout --kill-after=1s 5s gz model -m abb_irb1200 -p \
  >"${MODEL_POSE_BEFORE_LOG}" 2>&1 || die "could not observe initial model pose"
if grep -Fq "No model named" "${MODEL_POSE_BEFORE_LOG}"; then
  die "Gazebo pose query did not find abb_irb1200"
fi
timeout --kill-after=1s 5s gz model -m abb_irb1200 -j \
  >"${MODEL_JOINTS_LOG}" 2>&1 || die "could not observe Gazebo model joints"
if grep -Fq "No model named" "${MODEL_JOINTS_LOG}"; then
  die "Gazebo joint query did not find abb_irb1200"
fi
for expected_model_joint in world_to_base joint_1 joint_2 joint_3 joint_4 joint_5 joint_6; do
  model_joint_count="$(grep -Ewc "${expected_model_joint}" "${MODEL_JOINTS_LOG}" || true)"
  [[ "${model_joint_count}" -eq 1 ]] ||
    die "Gazebo joint query reported ${expected_model_joint} ${model_joint_count} times"
done
observed_actuated_model_joints="$(grep -Eoc 'joint_[0-9]+' "${MODEL_JOINTS_LOG}" || true)"
[[ "${observed_actuated_model_joints}" -eq 6 ]] ||
  die "Gazebo joint query did not report exactly six actuated joints"

observation_timeout="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["guards"]["observation_timeout_sec"])' \
  "${SMOKE_CONFIG}")"
clock_advance="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["guards"]["clock_min_advance_sec"])' \
  "${SMOKE_CONFIG}")"
translation_tolerance="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["guards"]["base_translation_tolerance_m"])' \
  "${SMOKE_CONFIG}")"
rotation_tolerance="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["guards"]["base_rotation_tolerance_rad"])' \
  "${SMOKE_CONFIG}")"

timeout --kill-after=2s 20s python3 "${RUNTIME_VERIFIER}" clock \
  --timeout "${observation_timeout}" --min-advance "${clock_advance}" \
  >"${CLOCK_LOG}" 2>&1 || {
    cat "${CLOCK_LOG}" >&2 || true
    die "advancing simulation clock was not observed"
  }
timeout --kill-after=2s 20s python3 "${RUNTIME_VERIFIER}" joint-state \
  --phase initial --smoke-config "${SMOKE_CONFIG}" \
  --limits-config "${LIMITS_CONFIG}" --timeout "${observation_timeout}" \
  >"${INITIAL_STATE_LOG}" 2>&1 || {
    cat "${INITIAL_STATE_LOG}" >&2 || true
    die "reviewed neutral initial state was not observed"
  }

if ! timeout --kill-after=1s 5s ros2 action list -t \
    >"${ACTIONS_LOG}" 2>&1; then
  die "could not inspect ROS action servers"
fi
grep -Eq '^/joint_trajectory_controller/follow_joint_trajectory[[:space:]]+\[control_msgs/action/FollowJointTrajectory\]$' \
  "${ACTIONS_LOG}" || {
    cat "${ACTIONS_LOG}" >&2
    die "expected FollowJointTrajectory action type was not observed"
  }

bounded_goal="$(python3 "${RUNTIME_VERIFIER}" goal \
  --smoke-config "${SMOKE_CONFIG}" --limits-config "${LIMITS_CONFIG}")" ||
  die "could not construct the reviewed bounded trajectory goal"
if ! timeout --kill-after=2s 35s ros2 action send_goal --feedback --timeout 30 \
    /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "${bounded_goal}" \
    >"${ACTION_LOG}" 2>&1; then
  cat "${ACTION_LOG}" >&2 || true
  die "bounded FollowJointTrajectory request failed or timed out"
fi
grep -Fq "Goal accepted" "${ACTION_LOG}" ||
  die "bounded trajectory goal was not accepted"
grep -Fq "Goal finished with status: SUCCEEDED" "${ACTION_LOG}" ||
  die "bounded trajectory did not finish with SUCCEEDED"
grep -Eq 'error_code:[[:space:]]+0' "${ACTION_LOG}" ||
  die "bounded trajectory controller result error_code was not zero"
timeout --kill-after=2s 10s python3 "${RUNTIME_VERIFIER}" feedback \
  --action-log "${ACTION_LOG}" --smoke-config "${SMOKE_CONFIG}" \
  --limits-config "${LIMITS_CONFIG}" >"${ACTION_FEEDBACK_LOG}" 2>&1 || {
    cat "${ACTION_FEEDBACK_LOG}" >&2 || true
    die "action feedback did not prove finite in-bounds six-axis motion"
  }

timeout --kill-after=2s 20s python3 "${RUNTIME_VERIFIER}" joint-state \
  --phase target --smoke-config "${SMOKE_CONFIG}" \
  --limits-config "${LIMITS_CONFIG}" --timeout "${observation_timeout}" \
  >"${TARGET_STATE_LOG}" 2>&1 || {
    cat "${TARGET_STATE_LOG}" >&2 || true
    die "bounded target was not observed for the required consecutive frames"
  }

timeout --kill-after=1s 5s gz model --list >"${MODELS_AFTER_LOG}" 2>&1 ||
  die "could not re-query Gazebo models after the trajectory"
model_count_after="$(grep -Ec '^[[:space:]]*-[[:space:]]+abb_irb1200[[:space:]]*$' "${MODELS_AFTER_LOG}" || true)"
[[ "${model_count_after}" -eq 1 ]] ||
  die "abb_irb1200 did not persist after the trajectory"
timeout --kill-after=1s 5s gz model -m abb_irb1200 -p \
  >"${MODEL_POSE_AFTER_LOG}" 2>&1 || die "could not observe final model pose"
timeout --kill-after=2s 10s python3 "${RUNTIME_VERIFIER}" pose \
  --before "${MODEL_POSE_BEFORE_LOG}" --after "${MODEL_POSE_AFTER_LOG}" \
  --translation-tolerance "${translation_tolerance}" \
  --rotation-tolerance "${rotation_tolerance}" \
  >"${POSE_COMPARISON_LOG}" 2>&1 || {
    cat "${POSE_COMPARISON_LOG}" >&2 || true
    die "fixed model base pose did not remain within the reviewed tolerance"
  }

timeout --kill-after=1s 5s ros2 control list_controllers \
  -c /controller_manager -v >"${CONTROLLERS_LOG}.after" 2>&1 ||
  die "could not re-query controllers after the trajectory"
grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' \
  "${CONTROLLERS_LOG}.after" || die "joint_state_broadcaster did not remain active"
grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' \
  "${CONTROLLERS_LOG}.after" || die "joint_trajectory_controller did not remain active"
timeout --kill-after=1s 5s ros2 node list --no-daemon --spin-time 2 \
  >"${NODES_LOG}.after" 2>&1 ||
  die "could not re-query runtime ROS nodes after the trajectory"
for required_node in /controller_manager /robot_state_publisher; do
  grep -Fxq "${required_node}" "${NODES_LOG}.after" ||
    die "required runtime node did not persist: ${required_node}"
done

capture_working_source_manifest >"${SOURCE_MANIFEST_AFTER}"
cmp --silent "${SOURCE_MANIFEST_BEFORE}" "${SOURCE_MANIFEST_AFTER}" ||
  die "working source files changed while the runtime gate was executing"
run_completed=true
