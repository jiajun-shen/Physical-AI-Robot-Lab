#!/usr/bin/env bash
# Run the Phase-5 verified pick/place gate in two isolated seeded simulations.
#
# One live simulator and four vision-only localizers remain active for each
# seed: Phase-4's five-class localizer seals pre/post RGB-D artifacts, while
# P5 localizers observe static obstacle fiducials for MoveIt and the retained
# target from a second visual camera. The online episode receives neither a
# Gazebo object pose nor a seed manifest.
set -eo pipefail

if [[ "${P5_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P5_WATCHDOG_ACTIVE=1
  exec timeout --signal=TERM --kill-after=45s 1200s "$0" "$@"
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

for required_command in awk cat cmp colcon find findmnt flock git grep gz pgrep \
    python3 realpath ros2 sed setsid sha256sum sort tail timeout tr wc xargs; do
  command -v "${required_command}" >/dev/null 2>&1 ||
    die "required command is unavailable: ${required_command}"
done

if [[ $# -gt 1 ]]; then
  die "usage: $0 [new-or-empty-output-directory]"
fi
if [[ $# -eq 0 ]]; then
  output_root="$(mktemp -d /var/tmp/physical-ai-p5.XXXXXX)"
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
exec {output_lock_fd}>"${OUTPUT_ROOT}/.phase5.lock"
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
    exec {candidate_fd}>"/var/tmp/physical-ai-p5-domain-${candidate}.lock"
    if flock -n "${candidate_fd}"; then
      export ROS_DOMAIN_ID="${candidate}"
      domain_lock_fd="${candidate_fd}"
      return
    fi
    eval "exec ${candidate_fd}>&-"
  done
  die "no Phase-5 ROS domain lock is available in [${first}, ${last}]"
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
  die "Phase-5 real evidence requires a clean source worktree"

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
  echo "A04=RUNTIME_PENDING"
  echo "PHASE5_SCOPE=SIMULATION_ONLY"
  echo "ONLINE_GAZEBO_OBJECT_POSE_INPUT=false"
} >"${OUTPUT_ROOT}/run_metadata.txt"

run_launch_pid=""
run_p4_node_pid=""
run_static_node_pid=""
run_guard_node_pid=""
run_transport_node_pid=""
run_directory=""
stop_process_group() {
  local group_pid="$1"
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
  local status=0
  stop_process_group "${run_transport_node_pid}" || status=1
  stop_process_group "${run_guard_node_pid}" || status=1
  stop_process_group "${run_static_node_pid}" || status=1
  stop_process_group "${run_p4_node_pid}" || status=1
  stop_process_group "${run_launch_pid}" || status=1
  for group_pid in "${run_transport_node_pid}" "${run_guard_node_pid}" "${run_static_node_pid}" "${run_p4_node_pid}" "${run_launch_pid}"; do
    if [[ -n "${group_pid}" ]] && kill -0 -- "-${group_pid}" 2>/dev/null; then
      status=1
    fi
  done
  run_transport_node_pid=""
  run_guard_node_pid=""
  run_static_node_pid=""
  run_p4_node_pid=""
  run_launch_pid=""
  timeout --kill-after=1s 5s ros2 daemon stop >/dev/null 2>&1 || true
  return "${status}"
}
cleanup() {
  local status="$?"
  local process_cleanup_status=PASS
  trap - EXIT INT TERM
  cleanup_active_run || process_cleanup_status=FAIL
  if [[ -n "${run_directory}" ]]; then
    printf 'PROCESS_CLEANUP_STATUS=%s\n' "${process_cleanup_status}" \
      >"${run_directory}/runtime/cleanup_status.txt" || true
  fi
  [[ "${process_cleanup_status}" == PASS ]] || status=1
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
    tail -n 220 "${log_path}" >&2 || true
    return 1
  fi
}

# In a no-daemon ROS graph, a default `ros2 topic list` sample can race DDS
# discovery and omit a still-alive peer. The Phase-5 graph deliberately has
# three independent RGB-D localizers, so use one bounded, three-second
# discovery sample for every readiness decision.  A successful sample is
# retained below as the auditable topic snapshot; never replace it with a
# fresh short sample immediately before validating it.
topic_snapshot() {
  timeout --kill-after=1s 8s ros2 topic list --no-daemon --spin-time 3 \
    2>/dev/null || true
}

partition_pids() {
  local partition="$1" environment_file process_dir
  for process_dir in /proc/[0-9]*; do
    environment_file="${process_dir}/environ"
    [[ -r "${environment_file}" ]] || continue
    if cat "${environment_file}" 2>/dev/null | tr '\0' '\n' |
        grep -Fxq "GZ_PARTITION=${partition}"; then
      basename "${process_dir}"
    fi
  done
  return 0
}

cd "${REPOSITORY_ROOT}/ros2_ws"
if ! colcon --log-base "${COLCON_LOG_BASE}" build \
    --base-paths "${SOURCE_ROOT}" \
    --packages-up-to physical_ai_skills abb_lab_moveit_config physical_ai_perception \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo >"${BUILD_LOG}" 2>&1; then
  tail -n 220 "${BUILD_LOG}" >&2 || true
  die "targeted Phase-5 build failed"
fi
if ! colcon --log-base "${COLCON_LOG_BASE}" test \
    --base-paths "${SOURCE_ROOT}" \
    --packages-select physical_ai_interfaces physical_ai_perception abb_lab_lab \
      abb_lab_bringup abb_lab_gripper abb_lab_moveit_config physical_ai_skills \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}" >"${TEST_LOG}" 2>&1; then
  tail -n 220 "${TEST_LOG}" >&2 || true
  die "targeted Phase-5 tests failed"
fi
if ! colcon test-result --all --verbose --test-result-base "${BUILD_BASE}" \
    >"${OUTPUT_ROOT}/colcon_test_result.txt" 2>&1; then
  sed -n '1,240p' "${OUTPUT_ROOT}/colcon_test_result.txt" >&2 || true
  die "Phase-5 test-result aggregation failed"
fi

# setup.bash uses optional variables that are undefined under set -u.
set +u
# shellcheck disable=SC1090
source "${INSTALL_BASE}/setup.bash"
set -u
readonly GENERATOR="${INSTALL_BASE}/abb_lab_lab/lib/abb_lab_lab/generate_scene_manifest.py"
readonly SCENE_CONFIG="${INSTALL_BASE}/abb_lab_lab/share/abb_lab_lab/config/manipulation_scene.yaml"
readonly WORLD_TEMPLATE="${INSTALL_BASE}/abb_lab_lab/share/abb_lab_lab/worlds/manipulation_lab_phase5_rgbd.sdf"
readonly P4_VISION_VERIFIER="${SCRIPT_DIR}/verify_p4_vision_runtime.py"
readonly P4_LIVE_CAPTURE="${SCRIPT_DIR}/capture_p4_live_ground_truth.py"
readonly P4_OFFLINE_EVALUATOR="${SCRIPT_DIR}/evaluate_p4_vision_evidence.py"
readonly P5_EVALUATOR="${SCRIPT_DIR}/evaluate_p5_pick_place_evidence.py"
readonly SEED_COMPARATOR="${SCRIPT_DIR}/compare_p5_seed_evidence.py"
for required_file in "${GENERATOR}" "${SCENE_CONFIG}" "${WORLD_TEMPLATE}" \
    "${P4_VISION_VERIFIER}" "${P4_LIVE_CAPTURE}" "${P4_OFFLINE_EVALUATOR}" \
    "${P5_EVALUATOR}" "${SEED_COMPARATOR}"; do
  [[ -f "${required_file}" ]] || die "required Phase-5 artifact is missing: ${required_file}"
done

run_seed() {
  local seed="$1"
  local seed_root="${RUNTIME_ROOT}/seed${seed}"
  local scene_root="${seed_root}/scene"
  local runtime_root="${seed_root}/runtime"
  local seeded_world="${scene_root}/seed${seed}_phase5_rgbd.sdf"
  local scene_manifest="${scene_root}/seed${seed}.json"
  local launch_log="${runtime_root}/launch.log"
  local p4_node_log="${runtime_root}/phase4_vision_node.log"
  local static_node_log="${runtime_root}/phase5_static_obstacle_vision.log"
  local guard_node_log="${runtime_root}/phase5_guard_object_vision.log"
  local transport_node_log="${runtime_root}/phase5_transport_object_vision.log"
  local topic_log="${runtime_root}/topics.txt"
  local controller_log="${runtime_root}/active_controllers.txt"
  local pre_verifier_log="${runtime_root}/pre_pick_vision_verifier.txt"
  local execution_log="${runtime_root}/pick_place_execution.txt"
  local post_verifier_log="${runtime_root}/post_place_vision_verifier.txt"
  local p5_evaluator_log="${runtime_root}/p5_evaluator.txt"
  local capture_log="${runtime_root}/offline_ground_truth_capture.txt"
  local p4_evaluator_log="${runtime_root}/offline_ground_truth_evaluator.txt"
  local seed_partition="physical_ai_p5_${ROS_DOMAIN_ID}_${seed}_${BASHPID}"
  local deadline topics pre_evidence execution_evidence post_evidence post_minimum_sec
  local existing_partition_pids
  mkdir -p "${scene_root}" "${runtime_root}" "${seed_root}/evidence" "${seed_root}/offline_gt"
  {
    sed -n '1,240p' "${OUTPUT_ROOT}/run_metadata.txt"
    echo "P5_SCENE_SEED=${seed}"
    echo "GZ_PARTITION=${seed_partition}"
    echo "SCENE_WORLD_NAME=${WORLD_NAME}"
    echo "P5_STATIC_OBSTACLES=RGBD_FIDUCIALS_ONLY"
    echo "P5_GUARD_RGBD=SECOND_VISUAL_ONLY_CAMERA"
  } >"${seed_root}/run_metadata.txt"
  run_directory="${seed_root}"

  existing_partition_pids="$(partition_pids "${seed_partition}")"
  [[ -z "${existing_partition_pids}" ]] ||
    die "seed ${seed} partition is already occupied: ${existing_partition_pids}"
  run_step "seed ${seed} P5 scene generation" "${runtime_root}/scene_generation.txt" \
    python3 "${GENERATOR}" \
      --config "${SCENE_CONFIG}" \
      --seed "${seed}" \
      --manifest-output "${scene_manifest}" \
      --world-template "${WORLD_TEMPLATE}" \
      --world-output "${seeded_world}"
  run_step "seed ${seed} P5 SDF validation" "${runtime_root}/sdf_validation.txt" \
    gz sdf -k "${seeded_world}"

  export GZ_PARTITION="${seed_partition}"
  setsid ros2 launch abb_lab_moveit_config irb1200_gripper_moveit_rgbd_headless.launch.py \
    "world_path:=${seeded_world}" "world_name:=${WORLD_NAME}" >"${launch_log}" 2>&1 &
  run_launch_pid="$!"
  deadline=$((SECONDS + 95))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${run_launch_pid}" 2>/dev/null; then
      tail -n 260 "${launch_log}" >&2 || true
      die "seed ${seed} P5 launch exited before readiness"
    fi
    topics="$(topic_snapshot)"
    if grep -Fxq '/joint_states' <<<"${topics}" &&
        grep -Fxq '/lab_camera/color/image_raw' <<<"${topics}" &&
        grep -Fxq '/lab_camera/depth/image_raw' <<<"${topics}" &&
        grep -Fxq '/lab_camera/color/camera_info' <<<"${topics}" &&
        grep -Fxq '/phase5/grasp/attach' <<<"${topics}" &&
        grep -Fxq '/phase5/grasp/detach' <<<"${topics}" &&
        ros2 service list --no-daemon 2>/dev/null | grep -Fxq '/plan_kinematic_path' &&
        gz model -m abb_irb1200_gripper >"${runtime_root}/gazebo_robot_identity.txt" 2>&1; then
      break
    fi
    sleep 0.5
  done
  kill -0 "${run_launch_pid}" 2>/dev/null || die "seed ${seed} P5 launch is not alive"
  grep -Fq 'Model:' "${runtime_root}/gazebo_robot_identity.txt" ||
    die "seed ${seed} cannot resolve robot in its own GZ_PARTITION"
  existing_partition_pids="$(partition_pids "${seed_partition}")"
  [[ -n "${existing_partition_pids}" ]] ||
    die "seed ${seed} launch did not create a process in its GZ_PARTITION"
  printf '%s\n' "${existing_partition_pids}" >"${runtime_root}/partition_processes_after_launch.txt"
  deadline=$((SECONDS + 35))
  while (( SECONDS < deadline )); do
    if ros2 control list_controllers --controller-manager /controller_manager \
        >"${controller_log}" 2>&1 &&
        grep -Eq '^joint_trajectory_controller[[:space:]].*active' "${controller_log}" &&
        grep -Eq '^two_finger_gripper_controller[[:space:]].*active' "${controller_log}"; then
      break
    fi
    sleep 0.5
  done
  grep -Eq '^joint_trajectory_controller[[:space:]].*active' "${controller_log}" || {
    tail -n 160 "${controller_log}" >&2 || true
    die "seed ${seed} arm trajectory controller is not active"
  }
  grep -Eq '^two_finger_gripper_controller[[:space:]].*active' "${controller_log}" || {
    tail -n 160 "${controller_log}" >&2 || true
    die "seed ${seed} gripper controller is not active"
  }

  # Start all RGB-D-only localizers only after Gazebo, ros2_control and MoveIt
  # are alive. P5's three localizers have remapped outputs, so P4 remains a
  # strict five-class evidence stream.
  setsid ros2 run physical_ai_perception vision_localizer_node --ros-args \
    -p use_sim_time:=true -p perception_mode:=vision >"${p4_node_log}" 2>&1 &
  run_p4_node_pid="$!"
  setsid ros2 launch abb_lab_moveit_config phase5_static_obstacle_vision.launch.py \
    >"${static_node_log}" 2>&1 &
  run_static_node_pid="$!"
  setsid ros2 launch abb_lab_moveit_config phase5_guard_object_vision.launch.py \
    >"${guard_node_log}" 2>&1 &
  run_guard_node_pid="$!"
  setsid ros2 launch abb_lab_moveit_config phase5_transport_object_vision.launch.py \
    >"${transport_node_log}" 2>&1 &
  run_transport_node_pid="$!"
  deadline=$((SECONDS + 65))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${run_p4_node_pid}" 2>/dev/null; then
      tail -n 220 "${p4_node_log}" >&2 || true
      die "seed ${seed} Phase-4 vision node exited"
    fi
    if ! kill -0 "${run_static_node_pid}" 2>/dev/null; then
      tail -n 220 "${static_node_log}" >&2 || true
      die "seed ${seed} P5 static-obstacle vision node exited"
    fi
    if ! kill -0 "${run_guard_node_pid}" 2>/dev/null; then
      tail -n 220 "${guard_node_log}" >&2 || true
      die "seed ${seed} P5 guard-object vision node exited"
    fi
    if ! kill -0 "${run_transport_node_pid}" 2>/dev/null; then
      tail -n 220 "${transport_node_log}" >&2 || true
      die "seed ${seed} P5 transport-object vision node exited"
    fi
    topics="$(topic_snapshot)"
    if grep -Fxq '/perception/detections' <<<"${topics}" &&
        grep -Fxq '/perception/camera_object_poses' <<<"${topics}" &&
        grep -Fxq '/perception/world_object_poses' <<<"${topics}" &&
        grep -Fxq '/phase5/perception/static_obstacle_poses' <<<"${topics}" &&
        grep -Fxq '/phase5/perception/guard_world_object_poses' <<<"${topics}" &&
        grep -Fxq '/phase5/perception/transport_world_object_poses' <<<"${topics}"; then
      break
    fi
    sleep 0.5
  done
  # `topics` is the same full discovery sample that satisfied all six gates
  # in the preceding loop.  Re-querying with ROS 2's zero-spin default can
  # omit one of the four live localizers and turn a valid run into a false
  # negative.
  printf '%s\n' "${topics}" >"${topic_log}"
  for expected_topic in /perception/detections /perception/camera_object_poses \
      /perception/world_object_poses /phase5/perception/static_obstacle_poses \
      /phase5/perception/guard_world_object_poses \
      /phase5/perception/transport_world_object_poses; do
    grep -Fxq "${expected_topic}" "${topic_log}" ||
      die "seed ${seed} did not expose required topic ${expected_topic}"
  done

  pre_evidence="${seed_root}/pre_pick/evidence/vision_evidence.json"
  run_step "seed ${seed} fresh pre-pick Phase-4 vision sealing" "${pre_verifier_log}" \
    python3 "${P4_VISION_VERIFIER}" \
      --output-root "${seed_root}/pre_pick" --timeout 55 --minimum-source-stamp-sec 3.0
  [[ -f "${pre_evidence}" ]] || die "seed ${seed} did not write pre-pick evidence"
  run_step "seed ${seed} verified pick/place episode" "${execution_log}" \
    ros2 run physical_ai_skills verified_pick_place \
      --pre-pick-evidence "${pre_evidence}" \
      --output-root "${seed_root}/execution" --seed "${seed}" --timeout 210
  execution_evidence="${seed_root}/execution/execution_evidence.json"
  [[ -f "${execution_evidence}" ]] || die "seed ${seed} did not write execution evidence"
  post_minimum_sec="$(python3 - "${execution_evidence}" <<'PY'
import json
import math
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    release_ns = int(json.load(stream)["post_place"]["minimum_source_stamp_ns"])
print(math.floor(release_ns / 1_000_000_000) + 1)
PY
)"
  [[ "${post_minimum_sec}" =~ ^[0-9]+$ ]] ||
    die "seed ${seed} execution evidence did not contain a valid post-place stamp"
  post_evidence="${seed_root}/post_place/evidence/vision_evidence.json"
  run_step "seed ${seed} fresh post-place Phase-4 vision sealing" "${post_verifier_log}" \
    python3 "${P4_VISION_VERIFIER}" \
      --output-root "${seed_root}/post_place" --timeout 55 \
      --minimum-source-stamp-sec "${post_minimum_sec}"
  [[ -f "${post_evidence}" ]] || die "seed ${seed} did not write post-place evidence"
  run_step "seed ${seed} P5 offline evidence evaluation" "${p5_evaluator_log}" \
    python3 "${P5_EVALUATOR}" \
      --pre-pick-evidence "${pre_evidence}" \
      --execution-evidence "${execution_evidence}" \
      --post-place-evidence "${post_evidence}" \
      --output "${seed_root}/evidence/p5_evaluation.json"
  grep -Fq 'P5_PICK_PLACE_EVALUATION=PASS' "${p5_evaluator_log}" ||
    die "seed ${seed} did not pass the P5 evidence evaluator"

  # Ground truth is intentionally captured only after all vision-only input
  # artifacts are sealed and P5's evidence evaluator has already passed. It
  # is an offline audit artifact and is not consumed by the online skill.
  run_step "seed ${seed} post-place offline ground-truth capture" "${capture_log}" \
    python3 "${P4_LIVE_CAPTURE}" \
      --vision-evidence "${post_evidence}" \
      --output "${seed_root}/offline_gt/post_place_live_ground_truth.json" \
      --world-name "${WORLD_NAME}"
  run_step "seed ${seed} post-place offline RGB-D/GT audit" "${p4_evaluator_log}" \
    python3 "${P4_OFFLINE_EVALUATOR}" \
      --vision-evidence "${post_evidence}" \
      --live-ground-truth "${seed_root}/offline_gt/post_place_live_ground_truth.json" \
      --output "${seed_root}/offline_gt/post_place_ground_truth_comparison.json"
  grep -Fq 'P4_OFFLINE_EVALUATION=PASS' "${p4_evaluator_log}" ||
    die "seed ${seed} post-place offline RGB-D/GT audit did not pass"

  cleanup_active_run || die "seed ${seed} process cleanup failed"
  printf 'CLEANUP_STATUS=PASS\n' >"${runtime_root}/cleanup_status.txt"
  run_directory=""
}

for seed in "${SELECTED_SEEDS[@]}"; do
  run_seed "${seed}"
done

run_step "P5 independent two-seed comparison" "${OUTPUT_ROOT}/two_seed_comparison.log" \
  python3 "${SEED_COMPARATOR}" \
    --first-run "${RUNTIME_ROOT}/seed${SELECTED_SEEDS[0]}" \
    --second-run "${RUNTIME_ROOT}/seed${SELECTED_SEEDS[1]}" \
    --output "${OUTPUT_ROOT}/p5_two_seed_comparison.json"
grep -Fq 'P5_TWO_SEED_PICK_PLACE_EVIDENCE=PASS' "${OUTPUT_ROOT}/two_seed_comparison.log" ||
  die "P5 independent two-seed comparison did not pass"

capture_working_source_manifest >"${SOURCE_MANIFEST_AFTER}"
cmp -s "${SOURCE_MANIFEST_BEFORE}" "${SOURCE_MANIFEST_AFTER}" ||
  die "working source manifest changed while Phase-5 evidence was running"
[[ -z "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]] ||
  die "source worktree is dirty after Phase-5 runtime"

{
  echo "P5_PICK_PLACE=PASS"
  echo "A04=PASS"
  echo "M1.04=PRE-M1_EVIDENCE"
  echo "M1.07=PRE-M1_EVIDENCE"
  echo "PHASE5_STATUS=COMPLETE_PENDING_DOCUMENTATION"
  echo "TESTED_COMMIT=${tested_commit}"
  echo "EVIDENCE_ROOT=${OUTPUT_ROOT}"
  echo "TWO_SEED_EVALUATOR=${OUTPUT_ROOT}/p5_two_seed_comparison.json"
  echo "SOURCE_MANIFEST_UNCHANGED=PASS"
  echo "WORKTREE_CLEAN_AFTER_RUNTIME=PASS"
} | tee "${RESULT_LOG}"
