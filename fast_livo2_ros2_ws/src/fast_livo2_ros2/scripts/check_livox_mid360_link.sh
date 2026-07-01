#!/usr/bin/env bash
set -eo pipefail

IFACE="${1:-enp8s0}"
HOST_IP="${2:-192.168.10.111}"

echo "== Livox Mid360 link check =="
echo "interface: ${IFACE}"
echo "host_ip:   ${HOST_IP}"
echo

if ! command -v ip >/dev/null 2>&1; then
  echo "ERROR: ip command not found"
  exit 2
fi

if [[ ! -d "/sys/class/net/${IFACE}" ]]; then
  echo "ERROR: network interface ${IFACE} does not exist"
  ip -br addr || true
  exit 2
fi

echo "-- address --"
ip -br addr show "${IFACE}" || true
echo

echo "-- kernel link state --"
for field in carrier operstate speed duplex address; do
  printf "%-10s " "${field}:"
  cat "/sys/class/net/${IFACE}/${field}" 2>/dev/null || true
done
echo

echo "-- route --"
ip route show dev "${IFACE}" || true
echo

echo "-- neighbor table --"
ip neigh show dev "${IFACE}" || true
echo

echo "-- packet counters --"
ip -s link show "${IFACE}" || true
echo

carrier="$(cat "/sys/class/net/${IFACE}/carrier" 2>/dev/null || echo 0)"
if [[ "${carrier}" != "1" ]]; then
  echo "RESULT: FAIL - ${IFACE} has NO-CARRIER."
  echo "The computer is not seeing an Ethernet physical link from the Mid360 path."
  echo "Check PoE/power module, Ethernet cable, adapter, and the exact port used."
  exit 1
fi

if ! ip -br addr show "${IFACE}" | grep -q "${HOST_IP}"; then
  echo "RESULT: FAIL - ${IFACE} is linked, but ${HOST_IP} is not assigned."
  echo "Configure the Livox host IP before launching the SDK/ROS driver."
  exit 1
fi

echo "RESULT: PASS - Ethernet link and host IP are ready for Livox SDK discovery."
