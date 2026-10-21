#!/usr/bin/env bash
# Shared helpers for setup_env scripts. Source this file; do not execute it.

set -euo pipefail

SETUP_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SETUP_ENV_DIR}/.." && pwd)"

ROS_DISTRO_NAME="humble"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

# Overridable from the environment.
WS_DIR="${WS_DIR:-${HOME}/robot_hardware_ws}"
VENV_DIR="${VENV_DIR:-${HOME}/ai_venv}"

log()  { printf '\033[1;34m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m  %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

require_not_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    die "Run as your normal user; the script calls sudo where needed."
  fi
}

require_ubuntu_jammy() {
  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "jammy" ]]; then
    die "ROS 2 Humble needs Ubuntu 22.04 (jammy); found ${PRETTY_NAME:-unknown}."
  fi
}

is_wsl() {
  grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null
}

apt_install() {
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
}

source_ros() {
  [[ -f "${ROS_SETUP}" ]] || die "ROS 2 ${ROS_DISTRO_NAME} not found; run 01_install_ros2_humble.sh first."
  # ROS setup scripts reference unset variables, so relax -u while sourcing.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
}
