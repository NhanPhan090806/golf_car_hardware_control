#!/usr/bin/env bash
# Run the full development-environment setup in order.
#
# Usage:
#   ./install_all.sh               everything (WSL2: builds vcan modules)
#   ./install_all.sh --skip-can    skip kernel module build and vcan setup
#
# Each numbered script is idempotent and can be re-run on its own.

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root
require_ubuntu_jammy

SKIP_CAN=0
[[ "${1:-}" == "--skip-can" ]] && SKIP_CAN=1

# Ask for the sudo password once up front.
sudo -v

"${SETUP_ENV_DIR}/01_install_ros2_humble.sh"
"${SETUP_ENV_DIR}/02_install_workspace_deps.sh"
"${SETUP_ENV_DIR}/03_setup_python_venv.sh"
if [[ "${SKIP_CAN}" -eq 0 ]]; then
  "${SETUP_ENV_DIR}/04_build_wsl_can_modules.sh"
  "${SETUP_ENV_DIR}/05_setup_vcan.sh" --persist
fi
"${SETUP_ENV_DIR}/06_build_workspace.sh"

log "All steps finished."
