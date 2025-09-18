#!/usr/bin/env bash
set -euo pipefail

# Usage: scripts/start.sh [iface]
# Creates and brings up a virtual CAN interface (vcan) if it doesn't exist.
# Default iface: vcan0 or $CAN_IFACE env var.

IFACE="${1:-${CAN_IFACE:-vcan0}}"

echo "[start.sh] target interface: $IFACE"

if ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[start.sh] interface '$IFACE' already exists"
    ip -details link show "$IFACE" || true
    exit 0
fi

echo "[start.sh] creating vcan module and interface $IFACE (requires sudo)"
sudo modprobe vcan
sudo ip link add dev "$IFACE" type vcan
sudo ip link set up "$IFACE"

echo "[start.sh] created and brought up $IFACE"
echo "Now run your emulator: python3 scripts/can_motor_emulator.py --iface $IFACE --actuator-id <id>"
