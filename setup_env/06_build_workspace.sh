#!/usr/bin/env bash
# Create the colcon workspace, link this repo into it, build, and run tests.
#
# Layout (matches AGENTS.md and CI):
#   ${WS_DIR}/src/robot_hardware_control -> this repository (symlink)
#
# Usage:
#   ./06_build_workspace.sh            build + test
#   ./06_build_workspace.sh --no-test  build only
#   WS_DIR=~/other_ws ./06_build_workspace.sh

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root
source_ros

RUN_TESTS=1
[[ "${1:-}" == "--no-test" ]] && RUN_TESTS=0

mkdir -p "${WS_DIR}/src"
link="${WS_DIR}/src/robot_hardware_control"
if [[ -L "${link}" ]]; then
  [[ "$(readlink -f "${link}")" == "${REPO_DIR}" ]] \
    || die "${link} points to $(readlink -f "${link}"), not ${REPO_DIR}. Fix it manually."
elif [[ -e "${link}" ]]; then
  die "${link} exists and is not a symlink; refusing to overwrite it."
else
  ln -s "${REPO_DIR}" "${link}"
  log "Linked ${link} -> ${REPO_DIR}"
fi

cd "${WS_DIR}"
log "Building workspace in ${WS_DIR}"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [[ "${RUN_TESTS}" -eq 1 ]]; then
  log "Running tests"
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
  colcon test --event-handlers console_direct+ || warn "colcon test reported failures"
  colcon test-result --verbose || warn "Some tests failed; see output above."
fi

ws_setup="${WS_DIR}/install/setup.bash"
if ! grep -qF "${ws_setup}" "${HOME}/.bashrc"; then
  printf '[ -f %s ] && source %s\n' "${ws_setup}" "${ws_setup}" >> "${HOME}/.bashrc"
  log "Added workspace overlay to ~/.bashrc"
fi
log "Done. Open a new shell or: source ${ws_setup}"
