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
export PYTHONDONTWRITEBYTECODE=1

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repository_root="$(cd -- "${script_dir}/.." && pwd)"
readonly package_root="${repository_root}/ros2_ws/src/abb_lab_description"

if (( $# > 1 )); then
  echo "Usage: $0 [EMPTY_OUTPUT_DIRECTORY]" >&2
  exit 2
fi

if (( $# == 0 )); then
  mkdir -p /tmp/physical-ai-robot-lab
  output_root="$(mktemp -d /tmp/physical-ai-robot-lab/abb-description.XXXXXX)"
else
  output_root="$(realpath -m -- "$1")"
  if [[ "${output_root}" == "/" || "${output_root}" == "${repository_root}" ||
        "${output_root}" == "${repository_root}/"* ]]; then
    echo "ERROR: output directory must be outside the source repository" >&2
    exit 1
  fi
  if [[ -e "${output_root}" && ! -d "${output_root}" ]]; then
    echo "ERROR: output path is not a directory: ${output_root}" >&2
    exit 1
  fi
  if [[ -d "${output_root}" ]] &&
      [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "ERROR: output directory is not empty: ${output_root}" >&2
    exit 1
  fi
  mkdir -p "${output_root}"
fi

readonly output_root
readonly build_base="${output_root}/build"
readonly install_base="${output_root}/install"
readonly log_base="${output_root}/log"
readonly generated_urdf="${output_root}/irb1200_5_90_geometry_only.urdf"

mkdir -p "${output_root}"

python3 -m unittest discover \
  -s "${package_root}/test" \
  -p 'test_*.py' -v

colcon --log-base "${log_base}" build \
  --base-paths "${repository_root}/ros2_ws/src" \
  --packages-select abb_lab_description \
  --build-base "${build_base}" \
  --install-base "${install_base}" \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

set +u
# shellcheck disable=SC1090
source "${install_base}/setup.bash"
set -u

xacro "$(ros2 pkg prefix --share abb_lab_description)/urdf/irb1200_5_90_geometry_only.urdf.xacro" \
  >"${generated_urdf}"
check_urdf "${generated_urdf}"
python3 "${package_root}/test/verify_expanded_abb_urdf.py" "${generated_urdf}"

colcon --log-base "${output_root}/test-log" test \
  --build-base "${build_base}" \
  --install-base "${install_base}" \
  --packages-select abb_lab_description \
  --return-code-on-test-failure \
  --event-handlers console_direct+
colcon --log-base "${output_root}/test-result-log" test-result \
  --test-result-base "${build_base}" --verbose

echo "ABB_DESCRIPTION_BUILD=PASS"
echo "OUTPUT_ROOT=${output_root}"
echo "GENERATED_URDF=${generated_urdf}"
