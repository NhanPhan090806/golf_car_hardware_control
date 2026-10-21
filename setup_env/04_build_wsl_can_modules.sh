#!/usr/bin/env bash
# Build the missing SocketCAN kernel modules for the stock WSL2 kernel.
#
# The Microsoft WSL2 kernel ships can, can-raw and can-dev but is built with
# CONFIG_CAN_VCAN, CONFIG_CAN_SLCAN and CONFIG_CAN_GS_USB disabled, so
# `modprobe vcan` fails. This script builds those modules from the exact
# WSL2-Linux-Kernel tag matching `uname -r` and installs them into
# /lib/modules/$(uname -r)/updates (the stock module tree is not touched).
#
# - vcan:   virtual CAN bus (vcan0/vcan1) for development without hardware.
# - slcan:  serial-line CAN adapters (CANable/slcan firmware) via usbipd.
# - gs_usb: candleLight/CANable gs_usb adapters via usbipd.
#
# Takes roughly 15-30 minutes on first run (a full `make modules` is needed so
# symbol versions match the running kernel). Re-run after a WSL kernel update.
# Not needed on the Jetson: L4T kernels already provide vcan and mttcan.

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_not_root

if sudo modprobe vcan 2>/dev/null; then
  log "vcan already loads on this kernel; nothing to build."
  exit 0
fi
is_wsl || die "vcan is unavailable and this is not WSL2; install your distro's linux-modules-extra package instead."

KREL="$(uname -r)"                                # e.g. 6.6.114.1-microsoft-standard-WSL2
KVER="${KREL%%-*}"                                # e.g. 6.6.114.1
KTAG="linux-msft-wsl-${KVER}"
BUILD_ROOT="${KERNEL_BUILD_ROOT:-${HOME}/.cache/wsl-kernel}"
SRC_DIR="${BUILD_ROOT}/WSL2-Linux-Kernel-${KTAG}"
DEST_DIR="/lib/modules/${KREL}/updates/can"

log "Installing kernel build dependencies"
sudo apt-get update
apt_install build-essential flex bison bc libssl-dev libelf-dev dwarves cpio \
  python3 curl ca-certificates kmod

mkdir -p "${BUILD_ROOT}"
if [[ ! -d "${SRC_DIR}" ]]; then
  log "Downloading WSL2 kernel source ${KTAG}"
  curl -fL "https://github.com/microsoft/WSL2-Linux-Kernel/archive/refs/tags/${KTAG}.tar.gz" \
    | tar -xz -C "${BUILD_ROOT}"
fi
[[ -d "${SRC_DIR}" ]] || die "Kernel source not found at ${SRC_DIR}."

cd "${SRC_DIR}"
log "Configuring from the running kernel's /proc/config.gz"
zcat /proc/config.gz > .config
scripts/config --file .config \
  --module CONFIG_CAN_VCAN \
  --module CONFIG_CAN_SLCAN \
  --module CONFIG_CAN_GS_USB \
  --set-str CONFIG_LOCALVERSION "${KREL#"${KVER}"}"
# Leave every other option (including DEBUG_INFO_BTF*) exactly as in the running
# kernel. CONFIG_DEBUG_INFO_BTF_MODULES adds fields to struct module, and
# genksyms folds that into the CRC of nearly every exported symbol; changing it
# makes modprobe fail with "Exec format error". The stock WSL kernel sets
# CONFIG_MODULE_ALLOW_BTF_MISMATCH=y, so module BTF built here is accepted.
make olddefconfig >/dev/null

[[ "$(make -s kernelrelease)" == "${KREL}" ]] \
  || die "Built kernelrelease $(make -s kernelrelease) != running ${KREL}; modules would not load."

log "Building kernel modules with $(nproc) jobs (this is the slow part)"
make -j"$(nproc)" modules

# Refuse to install modules whose symbol versions disagree with the running
# kernel. A stock module that also imports module_layout is the reference.
ref_ko="$(find "/lib/modules/${KREL}/kernel" -name 'can-dev.ko*' -print -quit)"
if [[ -n "${ref_ko}" ]]; then
  layout_crc() { /sbin/modprobe --dump-modversions "$1" | awk '$2=="module_layout"{print $1}'; }
  [[ "$(layout_crc drivers/net/can/vcan.ko)" == "$(layout_crc "${ref_ko}")" ]] \
    || die "Built modules have different symbol CRCs than the running kernel (config drift); not installing."
fi

log "Installing CAN modules into ${DEST_DIR}"
sudo install -d "${DEST_DIR}"
sudo install -m 0644 drivers/net/can/vcan.ko "${DEST_DIR}/"
sudo install -m 0644 drivers/net/can/slcan/slcan.ko "${DEST_DIR}/"
sudo install -m 0644 drivers/net/can/usb/gs_usb.ko "${DEST_DIR}/"
# gs_usb needs CAN_RX_OFFLOAD, which the stock can-dev.ko is built without, so
# ship the rebuilt can-dev too; depmod prefers updates/ over kernel/.
sudo install -m 0644 drivers/net/can/dev/can-dev.ko "${DEST_DIR}/"
sudo depmod -a "${KREL}"

# Swap out the stock can-dev if it is loaded but unused, so the rebuilt one
# (with rx-offload) is what vcan/slcan/gs_usb bind to.
if lsmod | grep -qE '^can_dev .* 0 *$'; then
  sudo modprobe -r can_dev
fi

log "Loading vcan"
sudo modprobe vcan
lsmod | grep -E '^(vcan|can_dev|can_raw|can) ' || true
log "Done. Source tree kept at ${SRC_DIR} (delete it to reclaim ~5 GB)."
