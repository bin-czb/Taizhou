#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-100000}"

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" modprobe can || true
"${SUDO[@]}" modprobe can_raw || true
"${SUDO[@]}" modprobe peak_usb || true
"${SUDO[@]}" modprobe gs_usb || true

"${SUDO[@]}" ip link set "${IFACE}" down 2>/dev/null || true
"${SUDO[@]}" ip link set "${IFACE}" type can bitrate "${BITRATE}" restart-ms 100
"${SUDO[@]}" ip link set "${IFACE}" up

ip -details link show "${IFACE}"
