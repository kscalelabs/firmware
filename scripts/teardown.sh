#!/usr/bin/env bash
set -euo pipefail

# Usage: scripts/teardown.sh [iface]
# Removes the virtual CAN interface created by start.sh. Defaults to vcan0 or $CAN_IFACE

IFACE="${1:-${CAN_IFACE:-vcan0}}"

echo "[teardown.sh] target interface: $IFACE"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[teardown.sh] interface '$IFACE' does not exist; nothing to do"
    exit 0
fi

echo "[teardown.sh] bringing down and deleting $IFACE (requires sudo)"
sudo ip link set down "$IFACE" || true
sudo ip link delete "$IFACE" type vcan || sudo ip link delete "$IFACE" || true

echo "[teardown.sh] removed $IFACE"
