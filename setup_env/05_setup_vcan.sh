#!/usr/bin/env bash
# Create virtual CAN interfaces matching the CI layout (vcan0, vcan1).
#
# Usage:
#   ./05_setup_vcan.sh              bring up vcan0 vcan1 now
#   ./05_setup_vcan.sh --persist    also install a systemd unit to recreate them at boot
#   VCAN_IFACES="vcan0 vcan1 vcan2" ./05_setup_vcan.sh
#
# Point the hardware plugin's CAN interface parameter at vcanN for software-only
# runs. Nothing on a vcan bus answers like an ODrive, so the HSH should stay in
# its offline/fault state unless a simulator is publishing feedback frames.

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root

VCAN_IFACES="${VCAN_IFACES:-vcan0 vcan1}"

sudo modprobe vcan || die "vcan module not available; run 04_build_wsl_can_modules.sh first."
sudo modprobe can_raw

for ifc in ${VCAN_IFACES}; do
  if ! ip link show "${ifc}" >/dev/null 2>&1; then
    sudo ip link add dev "${ifc}" type vcan
  fi
  sudo ip link set "${ifc}" up
  log "${ifc} is up"
done

if [[ "${1:-}" == "--persist" ]]; then
  if [[ "$(ps -p 1 -o comm=)" != "systemd" ]]; then
    die "systemd is not PID 1; set [boot] systemd=true in /etc/wsl.conf and run 'wsl --shutdown'."
  fi
  unit=/etc/systemd/system/vcan-setup.service
  log "Installing ${unit}"
  sudo tee "${unit}" >/dev/null <<EOF
[Unit]
Description=Create virtual CAN interfaces (${VCAN_IFACES})
After=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/sbin/modprobe vcan
ExecStart=/bin/sh -c 'for i in ${VCAN_IFACES}; do ip link show \$i >/dev/null 2>&1 || ip link add dev \$i type vcan; ip link set \$i up; done'
ExecStop=/bin/sh -c 'for i in ${VCAN_IFACES}; do ip link del \$i 2>/dev/null || true; done'

[Install]
WantedBy=multi-user.target
EOF
  sudo systemctl daemon-reload
  sudo systemctl enable vcan-setup.service
  log "vcan-setup.service enabled"
fi

ip -brief link show type vcan
log "Quick check: run 'candump vcan0' in one shell and 'cansend vcan0 123#DEADBEEF' in another."
