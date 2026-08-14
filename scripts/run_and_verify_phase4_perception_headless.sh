#!/usr/bin/env bash
# Run the project-owned Phase-4 RGB-D gate in two isolated seeded simulations.
#
# The perception node receives only ROS RGB, depth, CameraInfo, and TF. Each
# seed first seals that vision-only evidence, then a separate process captures
# live model poses for offline comparison. This script does not invoke a P3
# harness and does not claim grasp/contact/placement capability.
set -eo pipefail

if [[ "${P4_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P4_WATCHDOG_ACTIVE=1
  exec timeout --signal=TERM --kill-after=30s 540s "$0" "$@"
fi

readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
[[ -r "${ROS_SETUP}" ]] || { echo "ERROR: missing ${ROS_SETUP}" >&2; exit 1; }
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
readonly SOURCE_ROOT="${REPOSITORY_ROOT}/ros2_ws/src"
readonly REQUIRED_AVAILABLE_MEMORY_KIB=$((2 * 1024 * 1024))
readonly -a SELECTED_SEEDS=(42 43)
readonly WORLD_NAME="manipulation_lab"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

for required_command in awk cmp colcon find findmnt flock git grep gz pgrep \
    python3 realpath ros2 sed setsid sha256sum sort tail timeout tr wc xargs; do
  command -v "${required_command}" >/dev/null 2>&1 ||
    die "required command is unavailable: ${required_command}"
done

if [[ $# -gt 1 ]]; then
  die "usage: $0 [new-or-empty-output-directory]"
fi
if [[ $# -eq 0 ]]; then
  output_root="$(mktemp -d /var/tmp/physical-ai-p4.XXXXXX)"
else
  output_root="$(realpath -m "$1")"
  repository_real="$(realpath -e "${REPOSITORY_ROOT}")"
  case "${output_root}/" in
    "${repository_real}/"*) die "output directory must not be inside the repository" ;;
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
exec {output_lock_fd}>"${OUTPUT_ROOT}/.phase4.lock"
flock -n "${output_lock_fd}" || die "output directory is already in use: ${OUTPUT_ROOT}"
readonly output_lock_fd

output_filesystem="$(findmnt -T "${OUTPUT_ROOT}" -n -o FSTYPE)"
case "${output_filesystem,,}" in
  9p|drvfs) die "output directory must use a Linux filesystem, not ${output_filesystem}" ;;
esac

available_memory_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
[[ "${available_memory_kib}" =~ ^[0-9]+$ ]] || die "could not read MemAvailable"
(( available_memory_kib >= REQUIRED_AVAILABLE_MEMORY_KIB )) ||
  die "MemAvailable ${available_memory_kib} KiB is below the 2 GiB runtime floor"

readonly BUILD_BASE="${OUTPUT_ROOT}/build"
readonly INSTALL_BASE="${OUTPUT_ROOT}/install"
readonly COLCON_LOG_BASE="${OUTPUT_ROOT}/colcon-log"
readonly RUNTIME_ROOT="${OUTPUT_ROOT}/runs"
readonly ROS_HOME_PATH="${OUTPUT_ROOT}/ros-home"
readonly SOURCE_MANIFEST_BEFORE="${OUTPUT_ROOT}/working_source_before.sha256"
readonly SOURCE_MANIFEST_AFTER="${OUTPUT_ROOT}/working_source_after.sha256"
readonly RESULT_LOG="${OUTPUT_ROOT}/result.txt"
readonly BUILD_LOG="${OUTPUT_ROOT}/colcon_build.txt"
readonly TEST_LOG="${OUTPUT_ROOT}/colcon_test.txt"
mkdir -p "${COLCON_LOG_BASE}" "${RUNTIME_ROOT}" "${ROS_HOME_PATH}"

export PYTHONDONTWRITEBYTECODE=1
export ROS_HOME="${ROS_HOME_PATH}"
export ROS_LOG_DIR="${OUTPUT_ROOT}/ros-log"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
mkdir -p "${ROS_LOG_DIR}"

requested_domain="${ROS_DOMAIN_ID:-}"
domain_lock_fd=""
acquire_domain_lock() {
  local candidate candidate_fd first=200 last=232
  if [[ -n "${requested_domain}" ]]; then
    [[ "${requested_domain}" =~ ^[0-9]+$ ]] || die "ROS_DOMAIN_ID must be numeric"
    (( requested_domain >= 1 && requested_domain <= 232 )) ||
      die "ROS_DOMAIN_ID must be in [1, 232]"
    first="${requested_domain}"
    last="${requested_domain}"
  fi
  for ((candidate = first; candidate <= last; candidate++)); do
    exec {candidate_fd}>"/var/tmp/physical-ai-p4-domain-${candidate}.lock"
    if flock -n "${candidate_fd}"; then
      export ROS_DOMAIN_ID="${candidate}"
      domain_lock_fd="${candidate_fd}"
      return
    fi
    eval "exec ${candidate_fd}>&-"
  done
  die "no Phase-4 ROS domain lock is available in [${first}, ${last}]"
}
acquire_domain_lock
readonly ROS_DOMAIN_ID
readonly domain_lock_fd

if ! preexisting_nodes="$(timeout --kill-after=1s 4s ros2 node list --no-daemon --spin-time 2 2>"${OUTPUT_ROOT}/domain_preflight_stderr.txt")"; then
  sed -n '1,160p' "${OUTPUT_ROOT}/domain_preflight_stderr.txt" >&2 || true
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
[[ "${source_worktree_clean}" == true ]] ||
  die "Phase-4 real evidence requires a clean source worktree"

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
  echo "WORKING_SOURCE_MANIFEST_SHA256=${source_manifest_sha256}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "OUTPUT_FILESYSTEM=${output_filesystem}"
  echo "PERCEPTION_MODE=vision"
  echo "A04=NOT_STARTED"
  echo "GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE"
} >"${OUTPUT_ROOT}/run_metadata.txt"

run_launch_pid=""
run_node_pid=""
run_directory=""
stop_process_group() {
  local group_pid="$1"
  # The setsid leader can exit before a simulator child does.  Test and signal
  # the whole process group so an orphaned Gazebo process cannot contaminate a
  # later isolated seed run.
  if [[ -z "${group_pid}" ]] || ! kill -0 -- "-${group_pid}" 2>/dev/null; then
    return 0
  fi
  kill -INT -- "-${group_pid}" 2>/dev/null || true
  for _ in {1..24}; do
    kill -0 -- "-${group_pid}" 2>/dev/null || { wait "${group_pid}" 2>/dev/null || true; return 0; }
    sleep 0.2
  done
  kill -TERM -- "-${group_pid}" 2>/dev/null || true
  for _ in {1..12}; do
    kill -0 -- "-${group_pid}" 2>/dev/null || { wait "${group_pid}" 2>/dev/null || true; return 0; }
    sleep 0.2
  done
  kill -KILL -- "-${group_pid}" 2>/dev/null || true
  for _ in {1..8}; do
    kill -0 -- "-${group_pid}" 2>/dev/null || { wait "${group_pid}" 2>/dev/null || true; return 0; }
    sleep 0.2
  done
  return 1
}
cleanup_active_run() {
  stop_process_group "${run_node_pid}"
  stop_process_group "${run_launch_pid}"
  if [[ -n "${run_node_pid}" ]] && kill -0 -- "-${run_node_pid}" 2>/dev/null; then
    die "vision node process group survived cleanup"
  fi
  if [[ -n "${run_launch_pid}" ]] && kill -0 -- "-${run_launch_pid}" 2>/dev/null; then
    die "launch process group survived cleanup"
  fi
  run_node_pid=""
  run_launch_pid=""
  timeout --kill-after=1s 5s ros2 daemon stop >/dev/null 2>&1 || true
}
cleanup() {
  local status="$?"
  trap - EXIT INT TERM
  cleanup_active_run || status=1
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_step() {
  local label="$1" log_path="$2"
  shift 2
  if ! "$@" >"${log_path}" 2>&1; then
    echo "ERROR: ${label} failed; tail of ${log_path}:" >&2
    tail -n 180 "${log_path}" >&2 || true
    return 1
  fi
}

cd "${REPOSITORY_ROOT}/ros2_ws"
if ! colcon --log-base "${COLCON_LOG_BASE}" build \
    --base-paths "${SOURCE_ROOT}" \
    --packages-up-to physical_ai_perception abb_lab_bringup \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo >"${BUILD_LOG}" 2>&1; then
  tail -n 180 "${BUILD_LOG}" >&2 || true
  die "targeted Phase-4 build failed"
fi
if ! colcon --log-base "${COLCON_LOG_BASE}" test \
    --base-paths "${SOURCE_ROOT}" \
    --packages-select physical_ai_interfaces physical_ai_perception abb_lab_lab abb_lab_bringup \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" >"${TEST_LOG}" 2>&1; then
  tail -n 180 "${TEST_LOG}" >&2 || true
  die "targeted Phase-4 tests failed"
fi
if ! colcon test-result --all --verbose --test-result-base "${BUILD_BASE}" \
    >"${OUTPUT_ROOT}/colcon_test_result.txt" 2>&1; then
  cat "${OUTPUT_ROOT}/colcon_test_result.txt" >&2 || true
  die "Phase-4 test-result aggregation failed"
fi

# setup.bash uses optional variables that are undefined under `set -u`.
set +u
# shellcheck disable=SC1090
source "${INSTALL_BASE}/setup.bash"
set -u
readonly GENERATOR="${INSTALL_BASE}/abb_lab_lab/lib/abb_lab_lab/generate_scene_manifest.py"
readonly SCENE_CONFIG="${INSTALL_BASE}/abb_lab_lab/share/abb_lab_lab/config/manipulation_scene.yaml"
readonly WORLD_TEMPLATE="${INSTALL_BASE}/abb_lab_lab/share/abb_lab_lab/worlds/manipulation_lab_rgbd.sdf"
readonly VISION_VERIFIER="${SCRIPT_DIR}/verify_p4_vision_runtime.py"
readonly LIVE_CAPTURE="${SCRIPT_DIR}/capture_p4_live_ground_truth.py"
readonly OFFLINE_EVALUATOR="${SCRIPT_DIR}/evaluate_p4_vision_evidence.py"
readonly SEED_COMPARATOR="${SCRIPT_DIR}/compare_p4_seed_evidence.py"
for required_file in "${GENERATOR}" "${SCENE_CONFIG}" "${WORLD_TEMPLATE}" \
    "${VISION_VERIFIER}" "${LIVE_CAPTURE}" "${OFFLINE_EVALUATOR}" "${SEED_COMPARATOR}"; do
  [[ -f "${required_file}" ]] || die "required Phase-4 artifact is missing: ${required_file}"
done

run_seed() {
  local seed="$1"
  local seed_root="${RUNTIME_ROOT}/seed${seed}"
  local scene_root="${seed_root}/scene"
  local runtime_root="${seed_root}/runtime"
  local seeded_world="${scene_root}/seed${seed}_rgbd.sdf"
  local scene_manifest="${scene_root}/seed${seed}.json"
  local launch_log="${runtime_root}/launch.log"
  local node_log="${runtime_root}/vision_node.log"
  local topic_log="${runtime_root}/topics.txt"
  local node_info_log="${runtime_root}/vision_node_info.txt"
  local verifier_log="${runtime_root}/vision_verifier.txt"
  local capture_log="${runtime_root}/offline_capture.txt"
  local evaluator_log="${runtime_root}/offline_evaluator.txt"
  local seed_partition="physical_ai_p4_${ROS_DOMAIN_ID}_${seed}_${BASHPID}"
  local deadline node_info_deadline topics
  mkdir -p "${scene_root}" "${runtime_root}" "${seed_root}/offline_gt"
  {
    cat "${OUTPUT_ROOT}/run_metadata.txt"
    echo "P4_SCENE_SEED=${seed}"
    echo "GZ_PARTITION=${seed_partition}"
    echo "SCENE_WORLD_NAME=${WORLD_NAME}"
  } >"${seed_root}/run_metadata.txt"
  run_step "seed ${seed} scene generation" "${runtime_root}/scene_generation.txt" \
    python3 "${GENERATOR}" \
      --config "${SCENE_CONFIG}" \
      --seed "${seed}" \
      --manifest-output "${scene_manifest}" \
      --world-template "${WORLD_TEMPLATE}" \
      --world-output "${seeded_world}"
  run_step "seed ${seed} SDF validation" "${runtime_root}/sdf_validation.txt" \
    gz sdf -k "${seeded_world}"

  export GZ_PARTITION="${seed_partition}"
  run_directory="${seed_root}"
  setsid ros2 launch abb_lab_bringup irb1200_gripper_lab_rgbd_headless.launch.py \
    "world_path:=${seeded_world}" "world_name:=${WORLD_NAME}" >"${launch_log}" 2>&1 &
  run_launch_pid="$!"
  setsid ros2 run physical_ai_perception vision_localizer_node --ros-args \
    -p use_sim_time:=true -p perception_mode:=vision >"${node_log}" 2>&1 &
  run_node_pid="$!"

  deadline=$((SECONDS + 65))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${run_launch_pid}" 2>/dev/null; then
      tail -n 220 "${launch_log}" >&2 || true
      die "seed ${seed} launch exited before sensor topics appeared"
    fi
    if ! kill -0 "${run_node_pid}" 2>/dev/null; then
      tail -n 220 "${node_log}" >&2 || true
      die "seed ${seed} vision node exited before output topics appeared"
    fi
    topics="$(ros2 topic list --no-daemon 2>/dev/null || true)"
    if grep -Fxq '/lab_camera/color/image_raw' <<<"${topics}" &&
        grep -Fxq '/lab_camera/depth/image_raw' <<<"${topics}" &&
        grep -Fxq '/lab_camera/color/camera_info' <<<"${topics}" &&
        grep -Fxq '/perception/detections' <<<"${topics}" &&
        grep -Fxq '/perception/camera_object_poses' <<<"${topics}" &&
        grep -Fxq '/perception/world_object_poses' <<<"${topics}"; then
      break
    fi
    sleep 0.5
  done
  topics="$(ros2 topic list --no-daemon)"
  printf '%s\n' "${topics}" >"${topic_log}"
  for expected_topic in /lab_camera/color/image_raw /lab_camera/depth/image_raw \
      /lab_camera/color/camera_info /perception/detections \
      /perception/camera_object_poses /perception/world_object_poses; do
    grep -Fxq "${expected_topic}" "${topic_log}" ||
      die "seed ${seed} did not expose required topic ${expected_topic}"
  done
  # DDS graph discovery can lag topic discovery by a short interval.  Require
  # the actual subscription list, rather than accepting a transient node-info
  # lookup failure immediately after the topics first appear.
  node_info_deadline=$((SECONDS + 25))
  while (( SECONDS < node_info_deadline )); do
    if timeout --kill-after=1s 8s ros2 node info /vision_localizer \
        >"${node_info_log}" 2>&1 &&
        grep -Fq '/lab_camera/color/image_raw' "${node_info_log}" &&
        grep -Fq '/lab_camera/depth/image_raw' "${node_info_log}" &&
        grep -Fq '/lab_camera/color/camera_info' "${node_info_log}"; then
      break
    fi
    sleep 0.5
  done
  grep -Fq '/lab_camera/color/image_raw' "${node_info_log}" ||
    die "seed ${seed} vision node does not subscribe to RGB"
  grep -Fq '/lab_camera/depth/image_raw' "${node_info_log}" ||
    die "seed ${seed} vision node does not subscribe to depth"
  grep -Fq '/lab_camera/color/camera_info' "${node_info_log}" ||
    die "seed ${seed} vision node does not subscribe to CameraInfo"

  run_step "seed ${seed} vision-only evidence sealing" "${verifier_log}" \
    python3 "${VISION_VERIFIER}" \
      --output-root "${seed_root}" --timeout 55 --minimum-source-stamp-sec 3.0
  [[ -f "${seed_root}/evidence/vision_evidence.json" ]] ||
    die "seed ${seed} did not write sealed vision evidence"
  run_step "seed ${seed} offline live-pose capture" "${capture_log}" \
    python3 "${LIVE_CAPTURE}" \
      --vision-evidence "${seed_root}/evidence/vision_evidence.json" \
      --output "${seed_root}/offline_gt/live_ground_truth.json" \
      --world-name "${WORLD_NAME}"
  run_step "seed ${seed} offline vision evaluation" "${evaluator_log}" \
    python3 "${OFFLINE_EVALUATOR}" \
      --vision-evidence "${seed_root}/evidence/vision_evidence.json" \
      --live-ground-truth "${seed_root}/offline_gt/live_ground_truth.json" \
      --output "${seed_root}/offline_gt/offline_ground_truth_comparison.json"
  grep -Fq 'P4_OFFLINE_EVALUATION=PASS' "${evaluator_log}" ||
    die "seed ${seed} did not pass offline evaluation"
  grep -Fq 'A03_GREEN_BALL_VISION_LOCALIZATION=PASS' "${evaluator_log}" ||
    die "seed ${seed} did not pass A03 localization"
  cleanup_active_run
  run_directory=""
}

for seed in "${SELECTED_SEEDS[@]}"; do
  run_seed "${seed}"
done

run_step "two-seed vision comparison" "${OUTPUT_ROOT}/two_seed_comparison.log" \
  python3 "${SEED_COMPARATOR}" \
    --first-run "${RUNTIME_ROOT}/seed${SELECTED_SEEDS[0]}" \
    --second-run "${RUNTIME_ROOT}/seed${SELECTED_SEEDS[1]}" \
    --output "${OUTPUT_ROOT}/p4_two_seed_comparison.json"

capture_working_source_manifest >"${SOURCE_MANIFEST_AFTER}"
cmp -s "${SOURCE_MANIFEST_BEFORE}" "${SOURCE_MANIFEST_AFTER}" ||
  die "source manifest changed during the Phase-4 runtime gate"
if [[ -n "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]]; then
  die "source worktree is no longer clean after the Phase-4 runtime gate"
fi

{
  echo "P4_RGB_TRANSPORT=PASS"
  echo "P4_DEPTH_TRANSPORT=PASS"
  echo "P4_CAMERA_INFO=PASS"
  echo "P4_TF_CHAIN=PASS"
  echo "P4_DEPTH_PROJECTION=PASS"
  echo "P4_ALL_FIVE_DETECTIONS=PASS"
  echo "P4_VISION_GROUND_TRUTH_BOUNDARY=PASS"
  echo "P4_OFFLINE_EVALUATION=PASS"
  echo "P4_TWO_SEED_VISION_ONLY_LOCALIZATION=PASS"
  echo "A03=PASS"
  echo "A04=NOT_STARTED"
  echo "GRASP_STATE=NOT_DETERMINED_NO_CONTACT_EVIDENCE"
  echo "TESTED_COMMIT=${tested_commit}"
  echo "SOURCE_WORKTREE_CLEAN=${source_worktree_clean}"
  echo "WORKING_SOURCE_MANIFEST_SHA256=${source_manifest_sha256}"
  echo "SELECTED_SEEDS=${SELECTED_SEEDS[*]}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "OUTPUT_ROOT=${OUTPUT_ROOT}"
  echo "CLEANUP_VERIFIED=PASS"
} >"${RESULT_LOG}"
cat "${RESULT_LOG}"
