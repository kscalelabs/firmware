This folder contains a simple USB/serial IMU emulator that uses socat to create a PTY pair
and a small Python script to write Hiwonder IMU frames into the emulator side.

Files:
- start.sh    : creates a PTY pair using socat and runs imu_emulator.py bound to the emulator PTY
- imu_emulator.py : Python script that writes 11-byte Hiwonder-format quaternion frames

Quick start:
1. Ensure socat and python3 are installed on your system.
2. Run: ./scripts/usb/start.sh
3. The script will print the firmware PTY path (e.g. /tmp/imu_firmware). Point your firmware to that path (or symlink it to /dev/ttyUSB0 as root).

Example: run the emulator at 100Hz with a custom quaternion:

./scripts/usb/start.sh --rate 100 --quat 1 0 0 0

Notes:
- This approach uses PTYs and is not a full USB gadget; it's sufficient for testing serial-level IMU behavior.
- If you need true USB-level emulation, you can use Linux USB gadget/dummy_hcd; contact me if you want a script for that.
