#!/usr/bin/env bash
set -euo pipefail

# Create a PTY pair with socat and run the Python IMU emulator bound to one end.
# Firmware should open the "firmware" PTY. The emulator will open the "emulator" PTY
# and write Hiwonder-formatted 11-byte frames.

EMULATOR_PTY=${EMULATOR_PTY:-/tmp/imu_emulator_out}
FIRMWARE_PTY=${FIRMWARE_PTY:-/tmp/imu_emulator_in}
PYTHON="$(command -v python3 || true)"
if [ -z "$PYTHON" ]; then
  echo "python3 not found in PATH"
  exit 1
fi

# Start socat to create a PTY pair (runs until killed). -d -d prints debug info to stderr.
# link= sets a stable path for convenience.

socat -d -d pty,raw,echo=0,link="$FIRMWARE_PTY" pty,raw,echo=0,link="$EMULATOR_PTY" &
SOCAT_PID=$!

trap 'echo "shutting down..."; kill "$SOCAT_PID" 2>/dev/null || true; wait "$SOCAT_PID" 2>/dev/null || true' EXIT INT TERM

# Wait briefly for socat to create links
sleep 0.2

if [ ! -e "$FIRMWARE_PTY" ] || [ ! -e "$EMULATOR_PTY" ]; then
  echo "PTYs not created yet, waiting a moment..."
  sleep 0.5
fi

echo "PTY pair created: firmware -> $FIRMWARE_PTY  emulator -> $EMULATOR_PTY"
echo "Run your firmware against: $FIRMWARE_PTY"
echo "(Run as root to symlink to /dev/ttyUSB0 if you really need that: sudo ln -sf $FIRMWARE_PTY /dev/ttyUSB0)"

# Pass through any additional args to the Python emulator (e.g. --rate)
"$PYTHON" "$(dirname "$0")/imu_emulator.py" --port "$EMULATOR_PTY" "$@"

# end trap will kill socat
