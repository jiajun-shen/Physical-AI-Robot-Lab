#!/usr/bin/env bash
set -eo pipefail

readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: missing ${ROS_SETUP}" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

readonly LOG_DIR="${1:-/tmp/physical-ai-robot-lab/gz_ros2_control-smoke}"
readonly LAUNCH_LOG="${LOG_DIR}/launch.log"
readonly CONTROLLERS_LOG="${LOG_DIR}/controllers.txt"
readonly ACTION_LOG="${LOG_DIR}/action.txt"
readonly JOINT_STATE_LOG="${LOG_DIR}/joint_state.txt"
readonly TARGET_POSITION="2.0"

mkdir -p "${LOG_DIR}"
: >"${LAUNCH_LOG}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export GZ_PARTITION="${GZ_PARTITION:-physical_ai_phase1_${$}}"

launch_pid=""
cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.25
    done
    if kill -0 "${launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    fi
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch gz_ros2_control_demos cart_example_position.launch.py \
  gz_args:=-s >"${LAUNCH_LOG}" 2>&1 &
launch_pid="$!"

readonly startup_deadline=$((SECONDS + 60))
while (( SECONDS < startup_deadline )); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "ERROR: launch exited before controllers became active" >&2
    tail -n 120 "${LAUNCH_LOG}" >&2
    exit 1
  fi
  if ros2 control list_controllers --controller-manager /controller_manager \
      >"${CONTROLLERS_LOG}" 2>/dev/null &&
      grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' "${CONTROLLERS_LOG}" &&
      grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' "${CONTROLLERS_LOG}"; then
    break
  fi
  # Polling delay is test orchestration, not simulated control timing.
  sleep 0.25
done

if ! grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' "${CONTROLLERS_LOG}" ||
   ! grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' "${CONTROLLERS_LOG}"; then
  echo "ERROR: expected controllers did not become active" >&2
  cat "${CONTROLLERS_LOG}" >&2 || true
  tail -n 120 "${LAUNCH_LOG}" >&2
  exit 1
fi

readonly goal="{trajectory: {joint_names: [slider_to_cart], points: [{positions: [${TARGET_POSITION}], time_from_start: {sec: 2, nanosec: 0}}]}}"
if ! timeout 30 ros2 action send_goal --feedback \
    /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "${goal}" >"${ACTION_LOG}" 2>&1; then
  echo "ERROR: trajectory action failed or timed out" >&2
  cat "${ACTION_LOG}" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2
  exit 1
fi

if ! grep -Eq 'error_code:[[:space:]]+0' "${ACTION_LOG}"; then
  echo "ERROR: trajectory action did not report error_code 0" >&2
  cat "${ACTION_LOG}" >&2
  exit 1
fi

if ! ros2 topic echo /joint_states sensor_msgs/msg/JointState \
    --field position --once --timeout 10 --no-lost-messages \
    >"${JOINT_STATE_LOG}" 2>&1; then
  echo "ERROR: no joint state observed after trajectory" >&2
  cat "${JOINT_STATE_LOG}" >&2
  exit 1
fi

final_position="$(
  sed -n -E \
    -e 's/.*\[([-+0-9.eE]+)\].*/\1/p' \
    -e 's/^- ([-+0-9.eE]+)$/\1/p' \
    "${JOINT_STATE_LOG}" | head -n 1
)"
if [[ -z "${final_position}" ]]; then
  echo "ERROR: could not parse joint position" >&2
  cat "${JOINT_STATE_LOG}" >&2
  exit 1
fi

if ! awk -v observed="${final_position}" -v target="${TARGET_POSITION}" \
    'BEGIN {delta=observed-target; if (delta<0) delta=-delta; exit !(delta<=0.05)}'; then
  echo "ERROR: final position ${final_position} is not within 0.05 of ${TARGET_POSITION}" >&2
  cat "${ACTION_LOG}" >&2
  cat "${JOINT_STATE_LOG}" >&2
  exit 1
fi

echo "GZ_ROS2_CONTROL_SMOKE=PASS"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GZ_PARTITION=${GZ_PARTITION}"
echo "TARGET_POSITION=${TARGET_POSITION}"
echo "FINAL_POSITION=${final_position}"
echo "CONTROLLERS_LOG=${CONTROLLERS_LOG}"
echo "ACTION_LOG=${ACTION_LOG}"
echo "LAUNCH_LOG=${LAUNCH_LOG}"
