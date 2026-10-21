#!/usr/bin/env bash
# Install the build/runtime dependencies of this repository's ROS packages.
# The list mirrors .github/workflows/ros_ci.yml plus launch-file runtime deps,
# then rosdep resolves anything declared in package.xml that is still missing.

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root
source_ros

R="ros-${ROS_DISTRO_NAME}"

log "Installing toolchain and SocketCAN utilities"
sudo apt-get update
apt_install \
  build-essential cmake git gdb pkg-config \
  python3-pip python3-venv \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libgtest-dev \
  can-utils iproute2 kmod

log "Installing ros2_control and package dependencies"
apt_install \
  "${R}-ros2-control" \
  "${R}-ros2-controllers" \
  "${R}-controller-manager" \
  "${R}-controller-interface" \
  "${R}-hardware-interface" \
  "${R}-joint-state-broadcaster" \
  "${R}-realtime-tools" \
  "${R}-pluginlib" \
  "${R}-rclcpp" \
  "${R}-rclcpp-lifecycle" \
  "${R}-rclpy" \
  "${R}-std-msgs" \
  "${R}-sensor-msgs" \
  "${R}-diagnostic-msgs" \
  "${R}-rosidl-default-generators" \
  "${R}-rosidl-default-runtime" \
  "${R}-launch" \
  "${R}-launch-ros" \
  "${R}-ament-index-python" \
  "${R}-robot-state-publisher" \
  "${R}-xacro" \
  "${R}-ament-cmake-gtest" \
  "${R}-ament-lint-auto" \
  "${R}-ament-lint-common"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "Initialising rosdep"
  sudo rosdep init
fi
log "Updating rosdep database"
rosdep update --rosdistro "${ROS_DISTRO_NAME}"

log "Resolving remaining package.xml dependencies with rosdep"
# tinyfsm is vendored; the swerve_* packages referenced by legacy launch files
# are intentionally not part of this repo (see README.md).
rosdep install --from-paths "${REPO_DIR}/src" --ignore-src -y -r \
  --rosdistro "${ROS_DISTRO_NAME}"

log "Workspace dependencies installed"
