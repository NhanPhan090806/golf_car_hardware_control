#!/usr/bin/env bash
# Install ROS 2 Humble (ros-base + dev tools) from packages.ros.org.
# Works on Ubuntu 22.04 amd64 (WSL2/PC) and arm64 (Jetson Orin NX, JetPack 6).
#
# Usage: ./01_install_ros2_humble.sh [--desktop]
#   --desktop  install ros-humble-desktop (rviz2, rqt) instead of ros-base.

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root
require_ubuntu_jammy

ROS_META_PKG="ros-${ROS_DISTRO_NAME}-ros-base"
[[ "${1:-}" == "--desktop" ]] && ROS_META_PKG="ros-${ROS_DISTRO_NAME}-desktop"

log "Configuring UTF-8 locale"
sudo apt-get update
apt_install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

log "Enabling the Ubuntu universe repository"
apt_install software-properties-common curl ca-certificates gnupg lsb-release
sudo add-apt-repository -y universe

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  log "Adding the ROS 2 apt source (ros2-apt-source package)"
  ros_apt_ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [[ -n "${ros_apt_ver}" ]] || die "Could not resolve the latest ros-apt-source release (GitHub API rate limit?)."
  tmp_deb="$(mktemp --suffix=.deb)"
  curl -fL -o "${tmp_deb}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_ver}/ros2-apt-source_${ros_apt_ver}.jammy_all.deb"
  sudo apt-get install -y "${tmp_deb}"
  rm -f "${tmp_deb}"
else
  log "ROS 2 apt source already installed"
fi

log "Upgrading system packages (recommended before installing ROS on 22.04)"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y

log "Installing ${ROS_META_PKG} and ros-dev-tools"
apt_install "${ROS_META_PKG}" ros-dev-tools

if ! grep -qF "${ROS_SETUP}" "${HOME}/.bashrc"; then
  log "Adding 'source ${ROS_SETUP}' to ~/.bashrc"
  printf '\n# ROS 2 %s\nsource %s\n' "${ROS_DISTRO_NAME}" "${ROS_SETUP}" >> "${HOME}/.bashrc"
fi

source_ros
log "ROS 2 installed: ROS_DISTRO=${ROS_DISTRO}"
