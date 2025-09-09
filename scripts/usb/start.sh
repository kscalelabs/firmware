#!/usr/bin/env bash
set -euo pipefail

# Create a null modem emulation using socat that's compatible with tokio_serial
# This approach creates a more realistic serial device emulation

EMULATOR_PTY=${EMULATOR_PTY:-/tmp/imu_emulator_out}
FIRMWARE_PTY=${FIRMWARE_PTY:-/tmp/imu_emulator_in}
PYTHON="$(command -v python3 || true)"
if [ -z "$PYTHON" ]; then
  echo "python3 not found in PATH"
  exit 1
fi

# Use a simpler, more reliable socat command that creates proper TTY devices
# Remove complex options that might confuse tokio_serial
socat pty,link="$FIRMWARE_PTY",raw,echo=0 pty,link="$EMULATOR_PTY",raw,echo=0 &
SOCAT_PID=$!

trap 'echo "shutting down..."; kill "$SOCAT_PID" 2>/dev/null || true; wait "$SOCAT_PID" 2>/dev/null || true' EXIT INT TERM

# Wait briefly for socat to create links
sleep 0.5

# Wait up to 5 seconds for PTYs to be created
WAIT_COUNT=0
while [ $WAIT_COUNT -lt 50 ]; do
    if [ -e "$FIRMWARE_PTY" ] && [ -e "$EMULATOR_PTY" ]; then
        break
    fi
    echo "Waiting for PTYs to be created... ($((WAIT_COUNT + 1))/50)"
    sleep 0.1
    WAIT_COUNT=$((WAIT_COUNT + 1))
done

if [ ! -e "$FIRMWARE_PTY" ] || [ ! -e "$EMULATOR_PTY" ]; then
  echo "PTYs not created after waiting 5 seconds, aborting..."
  exit 1
fi

echo "PTY pair created: firmware -> $FIRMWARE_PTY  emulator -> $EMULATOR_PTY"
echo "Run your firmware against: $FIRMWARE_PTY"
echo "(Set IMU_DEV environment variable: export IMU_DEV=$FIRMWARE_PTY)"
echo "(Or run as root to symlink to /dev/ttyUSB0: sudo ln -sf $FIRMWARE_PTY /dev/ttyUSB0)"

# Pass through any additional args to the Python emulator (e.g. --rate)
"$PYTHON" "$(dirname "$0")/imu_emulator.py" --port "$EMULATOR_PTY" "$@"

# end trap will kill socat
