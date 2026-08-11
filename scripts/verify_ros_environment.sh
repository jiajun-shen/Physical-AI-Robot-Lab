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

required_debian_packages=(
  ros-jazzy-ros-gz
  ros-jazzy-gz-ros2-control
  ros-jazzy-gz-ros2-control-demos
  ros-jazzy-ros2-control
  ros-jazzy-ros2-controllers
  ros-jazzy-moveit
  ros-jazzy-moveit-task-constructor-core
  ros-jazzy-moveit-task-constructor-capabilities
  ros-jazzy-moveit-task-constructor-msgs
  ros-jazzy-moveit-task-constructor-visualization
)

required_ros_packages=(
  ros_gz_sim
  gz_ros2_control
  gz_ros2_control_demos
  controller_manager
  joint_trajectory_controller
  moveit_core
  moveit_ros_move_group
  moveit_task_constructor_core
)

echo "ROS_DISTRO=${ROS_DISTRO:-UNSET}"
if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "ERROR: expected ROS_DISTRO=jazzy" >&2
  exit 1
fi

echo "DIRECT_DEBIAN_PACKAGES"
for package_name in "${required_debian_packages[@]}"; do
  status_and_version="$(dpkg-query -W -f='${Status}\t${Version}' "${package_name}")"
  if [[ "${status_and_version}" != install\ ok\ installed$'\t'* ]]; then
    echo "ERROR: ${package_name} is not fully installed: ${status_and_version}" >&2
    exit 1
  fi
  printf '%s\t%s\n' "${package_name}" "${status_and_version#*$'\t'}"
done

echo "ROS_PACKAGE_PREFIXES"
for package_name in "${required_ros_packages[@]}"; do
  package_prefix="$(ros2 pkg prefix "${package_name}")"
  printf '%s\t%s\n' "${package_name}" "${package_prefix}"
done

gazebo_version="$(gz sim --versions | head -n 1)"
echo "GAZEBO_SIM_VERSION=${gazebo_version}"
if [[ ! "${gazebo_version}" =~ ^8\. ]]; then
  echo "ERROR: expected Gazebo Harmonic gz-sim major version 8" >&2
  exit 1
fi

ros2 control --help >/dev/null
echo "ROS2_CONTROL_CLI=OK"

dpkg_audit="$(dpkg --audit)"
if [[ -n "${dpkg_audit}" ]]; then
  echo "ERROR: dpkg audit reported incomplete package state" >&2
  echo "${dpkg_audit}" >&2
  exit 1
fi
echo "DPKG_AUDIT=OK"
