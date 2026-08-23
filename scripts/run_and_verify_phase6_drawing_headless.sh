#!/usr/bin/env bash
# Execute Phase 6 drawing cases through the live P5 Gazebo/MoveIt graph.
# The source worktree must be clean: this harness is used only for immutable
# simulation evidence, not iterative development.
set -eo pipefail

if [[ "${P6_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P6_WATCHDOG_ACTIVE=1
  exec timeout --signal=TERM --kill-after=45s 1200s "$0" "$@"
fi

readonly ROS_SETUP=/opt/ros/jazzy/setup.bash
[[ -r "${ROS_SETUP}" ]] || { echo "ERROR: missing ${ROS_SETUP}" >&2; exit 1; }
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
readonly SOURCE_ROOT="${REPOSITORY_ROOT}/ros2_ws/src"
readonly REQUIRED_AVAILABLE_MEMORY_KIB=$((2 * 1024 * 1024))

die() { echo "ERROR: $*" >&2; exit 1; }
for command in awk cmp colcon find findmnt flock git grep gz pgrep python3 realpath ros2 sed setsid sha256sum sort tail timeout tr wc xargs; do
  command -v "${command}" >/dev/null 2>&1 || die "required command is unavailable: ${command}"
done
[[ $# -le 1 ]] || die "usage: $0 [new-or-empty-output-directory]"

if [[ $# -eq 0 ]]; then
  output_root="$(mktemp -d /var/tmp/physical-ai-p6.XXXXXX)"
else
  output_root="$(realpath -m "$1")"
  repository_real="$(realpath -e "${REPOSITORY_ROOT}")"
  case "${output_root}/" in "${repository_real}/"*) die "output directory must not be inside repository" ;; esac
  [[ "${output_root}" != / ]] || die "output directory must not be /"
  [[ ! -e "${output_root}" || -d "${output_root}" ]] || die "output path is not a directory"
  if [[ -d "${output_root}" ]] && [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    die "explicit output directory must be empty"
  fi
  mkdir -p "${output_root}"
fi
readonly OUTPUT_ROOT="$(realpath -e "${output_root}")"
exec {output_lock_fd}>"${OUTPUT_ROOT}/.phase6.lock"
flock -n "${output_lock_fd}" || die "output directory is already in use"

output_filesystem="$(findmnt -T "${OUTPUT_ROOT}" -n -o FSTYPE)"
case "${output_filesystem,,}" in 9p|drvfs) die "output directory must be on a Linux filesystem" ;; esac
available_memory_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
[[ "${available_memory_kib}" =~ ^[0-9]+$ ]] && (( available_memory_kib >= REQUIRED_AVAILABLE_MEMORY_KIB )) || die "less than 2 GiB memory available"

readonly BUILD_BASE="${OUTPUT_ROOT}/build"
readonly INSTALL_BASE="${OUTPUT_ROOT}/install"
readonly COLCON_LOG_BASE="${OUTPUT_ROOT}/colcon-log"
readonly RUNTIME_ROOT="${OUTPUT_ROOT}/runtime"
readonly ROS_HOME_PATH="${OUTPUT_ROOT}/ros-home"
readonly BEFORE_MANIFEST="${OUTPUT_ROOT}/working_source_before.sha256"
readonly AFTER_MANIFEST="${OUTPUT_ROOT}/working_source_after.sha256"
readonly RESULT_LOG="${OUTPUT_ROOT}/result.txt"
mkdir -p "${COLCON_LOG_BASE}" "${RUNTIME_ROOT}" "${ROS_HOME_PATH}"
export PYTHONDONTWRITEBYTECODE=1 ROS_HOME="${ROS_HOME_PATH}" ROS_LOG_DIR="${OUTPUT_ROOT}/ros-log"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=
mkdir -p "${ROS_LOG_DIR}"

requested_domain="${ROS_DOMAIN_ID:-}"
domain_lock_fd=""
acquire_domain_lock() {
  local candidate candidate_fd first=180 last=199
  if [[ -n "${requested_domain}" ]]; then
    [[ "${requested_domain}" =~ ^[0-9]+$ ]] && (( requested_domain >= 1 && requested_domain <= 232 )) || die "ROS_DOMAIN_ID must be in [1,232]"
    first="${requested_domain}"; last="${requested_domain}"
  fi
  for ((candidate=first; candidate<=last; candidate++)); do
    exec {candidate_fd}>"/var/tmp/physical-ai-p6-domain-${candidate}.lock"
    if flock -n "${candidate_fd}"; then export ROS_DOMAIN_ID="${candidate}"; domain_lock_fd="${candidate_fd}"; return; fi
    eval "exec ${candidate_fd}>&-"
  done
  die "no Phase-6 ROS domain lock available"
}
acquire_domain_lock
readonly ROS_DOMAIN_ID domain_lock_fd
preexisting_nodes="$(timeout --kill-after=1s 5s ros2 node list --no-daemon --spin-time 2 2>"${OUTPUT_ROOT}/domain_preflight_stderr.txt")" || die "could not inspect ROS domain"
[[ -z "${preexisting_nodes}" ]] || die "selected ROS domain is not empty: ${preexisting_nodes}"

tested_commit="$(git -C "${REPOSITORY_ROOT}" rev-parse HEAD)"
[[ -z "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]] || die "real evidence requires a clean source worktree"
capture_manifest() { ( cd "${REPOSITORY_ROOT}" && git ls-files --cached --others --exclude-standard -z | LC_ALL=C sort -z | xargs -0 -r sha256sum ); }
capture_manifest >"${BEFORE_MANIFEST}"

launch_pid=""; vision_pid=""
stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 -- "-${pid}" 2>/dev/null || return 0
  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in {1..25}; do kill -0 -- "-${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }; sleep 0.2; done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in {1..10}; do kill -0 -- "-${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }; sleep 0.2; done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  return 1
}
cleanup() {
  local status="$?"; trap - EXIT INT TERM
  stop_group "${vision_pid}" || status=1
  stop_group "${launch_pid}" || status=1
  timeout --kill-after=1s 5s ros2 daemon stop >/dev/null 2>&1 || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
run_step() { local label="$1" log="$2"; shift 2; "$@" >"${log}" 2>&1 || { echo "ERROR: ${label}" >&2; tail -n 220 "${log}" >&2 || true; return 1; }; }
topic_snapshot() { timeout --kill-after=1s 8s ros2 topic list --no-daemon --spin-time 3 2>/dev/null || true; }

cd "${REPOSITORY_ROOT}/ros2_ws"
run_step "Phase-6 targeted build" "${OUTPUT_ROOT}/colcon_build.txt" colcon --log-base "${COLCON_LOG_BASE}" build --base-paths "${SOURCE_ROOT}" --packages-up-to physical_ai_drawing physical_ai_perception abb_lab_moveit_config --build-base "${BUILD_BASE}" --install-base "${INSTALL_BASE}" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
run_step "Phase-6 targeted tests" "${OUTPUT_ROOT}/colcon_test.txt" colcon --log-base "${COLCON_LOG_BASE}" test --base-paths "${SOURCE_ROOT}" --packages-select physical_ai_interfaces physical_ai_perception abb_lab_lab abb_lab_bringup abb_lab_gripper abb_lab_moveit_config physical_ai_drawing --build-base "${BUILD_BASE}" --install-base "${INSTALL_BASE}"
run_step "Phase-6 test aggregation" "${OUTPUT_ROOT}/colcon_test_result.txt" colcon test-result --all --verbose --test-result-base "${BUILD_BASE}"
set +u
# shellcheck disable=SC1090
source "${INSTALL_BASE}/setup.bash"
set -u
readonly P4_VERIFIER="${SCRIPT_DIR}/verify_p4_vision_runtime.py"
readonly EVALUATOR="${SCRIPT_DIR}/evaluate_p6_drawing_evidence.py"
[[ -f "${P4_VERIFIER}" && -f "${EVALUATOR}" ]] || die "required Phase-6 helper missing"

export GZ_PARTITION="physical_ai_p6_${ROS_DOMAIN_ID}_$$"
setsid ros2 launch abb_lab_moveit_config phase6_drawing_headless.launch.py execution_duration_scaling:=2.2 execution_goal_duration_margin:=2.0 >"${RUNTIME_ROOT}/launch.log" 2>&1 &
launch_pid="$!"
deadline=$((SECONDS + 100))
while (( SECONDS < deadline )); do
  kill -0 "${launch_pid}" 2>/dev/null || { tail -n 260 "${RUNTIME_ROOT}/launch.log" >&2 || true; die "P6 launch exited before readiness"; }
  topics="$(topic_snapshot)"
  if grep -Fxq /joint_states <<<"${topics}" && grep -Fxq /lab_camera/color/image_raw <<<"${topics}" && grep -Fxq /lab_camera/depth/image_raw <<<"${topics}" && ros2 service list --no-daemon 2>/dev/null | grep -Fxq /compute_cartesian_path && ros2 service list --no-daemon 2>/dev/null | grep -Fxq /plan_kinematic_path; then break; fi
  sleep 0.5
done
kill -0 "${launch_pid}" 2>/dev/null || die "P6 launch did not remain alive"
ros2 param get /move_group trajectory_execution.allowed_execution_duration_scaling >"${RUNTIME_ROOT}/moveit_execution_monitor.txt" 2>&1 || die "could not read Phase-6 MoveIt execution monitor"
grep -Eq '2\.2' "${RUNTIME_ROOT}/moveit_execution_monitor.txt" || die "Phase-6 MoveIt execution monitor did not apply bounded scaling"

setsid ros2 run physical_ai_perception vision_localizer_node --ros-args -p use_sim_time:=true -p perception_mode:=vision >"${RUNTIME_ROOT}/vision_node.log" 2>&1 &
vision_pid="$!"
deadline=$((SECONDS + 70))
while (( SECONDS < deadline )); do
  kill -0 "${vision_pid}" 2>/dev/null || { tail -n 220 "${RUNTIME_ROOT}/vision_node.log" >&2 || true; die "P6 vision node exited"; }
  topics="$(topic_snapshot)"
  if grep -Fxq /perception/world_object_poses <<<"${topics}"; then break; fi
  sleep 0.5
done
printf '%s\n' "${topics}" >"${RUNTIME_ROOT}/topics.txt"
grep -Fxq /perception/world_object_poses "${RUNTIME_ROOT}/topics.txt" || die "P6 vision topic did not become ready"
run_step "fresh Phase-4 board vision sealing" "${RUNTIME_ROOT}/vision_verifier.log" python3 "${P4_VERIFIER}" --output-root "${OUTPUT_ROOT}/vision_baseline" --timeout 55 --minimum-source-stamp-sec 3.0

run_case() {
  local case_id="$1" instruction="$2" case_root="${OUTPUT_ROOT}/cases/$1"
  mkdir -p "${case_root}"
  run_step "Phase-6 ${case_id}" "${RUNTIME_ROOT}/${case_id}.log" ros2 run physical_ai_drawing execute_drawing --instruction "${instruction}" --output-root "${case_root}" --timeout 330
  [[ -f "${case_root}/drawing_execution_evidence.json" ]] || die "${case_id} did not produce drawing evidence"
}
run_case heart_curve "画一个小一点的爱心"
run_case left_square "在白板左边画一个小一点的正方形"
run_case small_circle "画一个小一点的圆"
run_step "Phase-6 evidence evaluation" "${RUNTIME_ROOT}/evaluator.log" python3 "${EVALUATOR}" --evidence "${OUTPUT_ROOT}/cases/heart_curve/drawing_execution_evidence.json" --evidence "${OUTPUT_ROOT}/cases/left_square/drawing_execution_evidence.json" --evidence "${OUTPUT_ROOT}/cases/small_circle/drawing_execution_evidence.json" --output "${OUTPUT_ROOT}/p6_drawing_evaluation.json"
grep -Fxq P6_DRAWING_EVALUATION=PASS "${RUNTIME_ROOT}/evaluator.log" || die "P6 evidence evaluator did not pass"

stop_group "${vision_pid}"; vision_pid=""
stop_group "${launch_pid}"; launch_pid=""
capture_manifest >"${AFTER_MANIFEST}"
cmp -s "${BEFORE_MANIFEST}" "${AFTER_MANIFEST}" || die "source manifest changed during runtime"
[[ -z "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]] || die "source worktree is dirty after runtime"
{
  echo P6_DRAWING=PASS
  echo A05=PASS
  echo PHASE6_STATUS=COMPLETE_PENDING_DOCUMENTATION
  echo "TESTED_COMMIT=${tested_commit}"
  echo "EVIDENCE_ROOT=${OUTPUT_ROOT}"
  echo SOURCE_MANIFEST_UNCHANGED=PASS
  echo WORKTREE_CLEAN_AFTER_RUNTIME=PASS
} | tee "${RESULT_LOG}"
