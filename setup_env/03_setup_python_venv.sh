#!/usr/bin/env bash
# Install the Python libraries used by tests/*.py into the project venv.
#
# The venv is for the standalone test/diagnostic scripts only. Build and run
# ROS with the system python3 (colcon, ros2 launch); do not install colcon or
# ROS Python packages into the venv.
#
# Usage: VENV_DIR=~/ai_venv ./03_setup_python_venv.sh

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  log "Creating venv at ${VENV_DIR}"
  python3 -m venv "${VENV_DIR}"
fi

py_ver="$("${VENV_DIR}/bin/python" -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')"
if [[ "${py_ver}" != "3.10" ]]; then
  warn "venv uses Python ${py_ver}; ROS 2 Humble's rclpy is built for 3.10 and will not import."
fi

log "Installing Python requirements into ${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -r "${SETUP_ENV_DIR}/requirements.txt"

log "Installed:"
"${VENV_DIR}/bin/python" -m pip list --format=columns | grep -Ei 'python-can|pyserial|rich|pyyaml|numpy'
