#!/usr/bin/env python3
"""Simple IMU emulator that writes Hiwonder-format frames to a serial PTY.

It writes 11-byte frames: [0]=0x55, [1]=frame_type (0x59 quaternion), [2..9]=payload (8 bytes), [10]=checksum(sum of first 10 bytes & 0xff).

By default it emits Quaternion frames (FrameType::Quaternion = 0x59) at a configurable rate.

This script avoids external deps; it opens the character device and writes raw bytes.
"""

import argparse
import os
import sys
import time
import struct
from time import monotonic, sleep

FRAME_SIZE = 11
START = 0x55
FRAME_QUAT = 0x59  # Quaternion


def make_quat_frame(w_raw=16384, x_raw=0, y_raw=0, z_raw=0):
    # Hiwonder payload layout for Quaternion uses four i16 values in little-endian, placed in data[0..7]:
    # data[0..1] = w (low, high), data[2..3] = x, data[4..5] = y, data[6..7] = z
    payload = struct.pack('<hhhh', w_raw, x_raw, y_raw, z_raw)
    assert len(payload) == 8
    head = bytes([START, FRAME_QUAT])
    buf0_9 = head + payload
    checksum = sum(buf0_9) & 0xFF
    frame = buf0_9 + bytes([checksum])
    assert len(frame) == FRAME_SIZE
    return frame


def open_device(path):
    # Open the device for binary write without buffering
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NOCTTY)
        return fd
    except OSError as e:
        print(f"Failed to open {path}: {e}", file=sys.stderr)
        raise


def main():
    p = argparse.ArgumentParser(description="HIWONDER IMU emulator -> write Hiwonder frames to a PTY/serial device")
    p.add_argument('--port', '-p', required=True, help='Emulator side PTY path (this script writes to it)')
    p.add_argument('--rate', '-r', type=float, default=50.0, help='Frame rate (Hz)')
    p.add_argument('--debug', action='store_true')
    p.add_argument('--quat', nargs=4, type=float, help='Quaternion as floats (w x y z), normalized-ish, will be scaled to int16')
    args = p.parse_args()

    interval = 1.0 / max(0.1, args.rate)

    if args.quat:
        # convert floats -1..1 -> raw int16 range roughly [-32768,32767]
        q = args.quat
        def to_raw(v):
            return int(max(-32768, min(32767, round(v * 32767))))
        w_raw, x_raw, y_raw, z_raw = map(to_raw, q)
    else:
        # default quaternion w=0.5 -> raw ~16384
        w_raw, x_raw, y_raw, z_raw = 16384, 0, 0, 0

    try:
        fd = open_device(args.port)
    except Exception:
        sys.exit(1)

    print(f"Writing Hiwonder quaternion frames to {args.port} @ {args.rate}Hz (interval {interval}s)")

    next_t = monotonic()
    try:
        while True:
            frame = make_quat_frame(w_raw, x_raw, y_raw, z_raw)
            try:
                os.write(fd, frame)
                now = monotonic()
                if now >= next_t:
                    missed = int((now - next_t) / interval) + 1
                    next_t += missed * interval
                else:
                    sleep(next_t - now)
                    next_t += interval

            except BrokenPipeError:
                print("Peer closed PTY (broken pipe). Exiting.", file=sys.stderr)
                break
            if args.debug:
                print(frame.hex())
            # time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.close(fd)
        except Exception:
            pass


if __name__ == '__main__':
    main()