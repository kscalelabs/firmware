#!/usr/bin/env python3
"""CAN motor emulator for testing the firmware.

This script opens a raw SocketCAN socket, reads 16-byte CAN frames using the
same layout as `src/socketcan.rs` (u32 can_id, 4 bytes of header, 8 bytes data),
and replies to a subset of Robstride requests so the Rust firmware can't tell
the difference.

It intentionally avoids external dependencies and uses Python's builtin
socket/struct modules.

Features implemented:
 - Respond to ObtainId (mux 0x00) with a fake MCU UID
 - Respond to FeedbackRequest (mux 0x02) with plausible sensor values
 - Respond to ReadParamRequest (mux 0x11) by echoing index and returning 0
 - Optionally periodically broadcast Feedback frames

Usage: scripts/can_motor_emulator.py --iface vcan0 --actuator-id 5 --dry-run
"""

import argparse
import socket
import struct
import time
import threading
import os
import sys
import random

# Frame layout used by the Rust code: packed repr with total size 16 bytes
# struct CanFrame { u32 can_id; u8 len; u8 pad; u8 res0; u8 len8_dlc; u8 data[8]; }
FRAME_FMT = "<I4B8s"  # little-endian: u32, 4 x u8, 8 bytes
FRAME_SIZE = struct.calcsize(FRAME_FMT)


def pack_frame(can_id: int, dlc: int, data: bytes) -> bytes:
    data = (data or b"")[:8]
    data = data.ljust(8, b"\x00")
    pad = 0
    res0 = 0
    len8_dlc = 0
    return struct.pack(FRAME_FMT, can_id & 0xFFFFFFFF, dlc & 0xFF, pad, res0, len8_dlc, data)


def unpack_frame(buf: bytes):
    can_id, dlc, pad, res0, len8_dlc, data = struct.unpack(FRAME_FMT, buf)
    return can_id, dlc, bytes(data)


def mux_from_can_id(can_id: int) -> int:
    # mux is stored in the highest-order byte & 0x1F (frame[3] & 0x1F)
    return (can_id >> 24) & 0x1F


def actuator_id_from_can_id(can_id: int) -> int:
    return (can_id >> 8) & 0xFF


def make_response_can_id(request_can_id: int, resp_mux: int) -> int:
    # Build response can_id: preserve lower 24 bits, set mux in top byte,
    # and set EFF flag (0x8000_0000) so the Rust code recognizes it as a response
    lower = request_can_id & 0x00FFFFFF
    top = (resp_mux & 0x1F) << 24
    return lower | top | 0x80000000


class CanMotorEmulator:
    def __init__(self, iface: str, actuator_id: int, host_id: int, periodic: float = 0.0, dry_run=False, verbose=False):
        self.iface = iface
        self.actuator_id = actuator_id
        self.host_id = host_id
        self.periodic = periodic
        self.dry_run = dry_run
        self.verbose = verbose
        self.sock = None
        self._stop = False

    def log(self, *a, **kw):
        if self.verbose:
            print(*a, **kw)

    def open(self):
        if self.dry_run:
            self.log("dry-run: not opening socket")
            return
        # AF_CAN raw socket
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # bind to interface
        s.bind((self.iface,))
        self.sock = s
        self.log(f"bound to {self.iface}")

    def close(self):
        self._stop = True
        if self.sock:
            self.sock.close()

    def send(self, can_id: int, data: bytes):
        frame = pack_frame(can_id, 8, data)
        if self.dry_run:
            self.log("DRY SEND -> can_id=0x{:08X} data={}".format(can_id, data.hex()))
            return
        self.sock.send(frame)
        self.log("SENT -> can_id=0x{:08X} data={}".format(can_id, data.hex()))

    def handle_request(self, can_id: int, dlc: int, data: bytes):
        mux = mux_from_can_id(can_id)
        req_act_id = actuator_id_from_can_id(can_id)
        self.log(f"REQ can_id=0x{can_id:08X} mux=0x{mux:02X} act={req_act_id} dlc={dlc} data={data.hex()}")

        # Only respond for requests targeting our actuator id (or broadcast 0xFF)
        if req_act_id not in (self.actuator_id, 0xFF):
            self.log(f"ignoring request for actuator {req_act_id}")
            return

        if mux == 0x00:  # ObtainIdRequest -> ObtainIdResponse
            resp_can_id = make_response_can_id(can_id, 0x00)
            # mcu_uid: 8 bytes, create deterministic-ish value
            mcu_uid = (0xDEADBEEFCAF00000 | (self.actuator_id & 0xFF)) & 0xFFFFFFFFFFFFFFFF
            data_bytes = mcu_uid.to_bytes(8, "little")
            self.send(resp_can_id, data_bytes)

        elif mux == 0x02:  # FeedbackRequest -> FeedbackResponse
            resp_can_id = make_response_can_id(can_id, 0x02)
            # angle_scale_be, angular_vel_scale_be, torque_be, temp_be each u16 (big-endian)
            # Provide neutral values: angle=0, vel=0, torque=0, temp=250 => 25.0C
            angle = 0
            vel = 0
            torque = 0
            temp = int(25.0 * 10)  # protocol uses temp*10
            data_bytes = struct.pack(
                ">HHHH",  # big-endian u16 fields
                angle & 0xFFFF,
                vel & 0xFFFF,
                torque & 0xFFFF,
                temp & 0xFFFF,
            )
            self.send(resp_can_id, data_bytes)

        elif mux == 0x11:  # ReadParamRequest -> ReadParamResponse
            resp_can_id = make_response_can_id(can_id, 0x11)
            # echo index (first two bytes of data are index in request little-endian per request impl)
            if len(data) >= 2:
                index = int.from_bytes(data[0:2], "little")
            else:
                index = 0
            # Build response data: index(u16), res1(u16)=0, value(u32)=0
            data_bytes = struct.pack("<HHI", index & 0xFFFF, 0, 0)
            self.send(resp_can_id, data_bytes)

        elif mux == 0x03:  # MotorEnableRequest -> reply with feedback to indicate enabled
            resp_can_id = make_response_can_id(can_id, 0x02)
            temp = int(25.0 * 10)
            data_bytes = struct.pack(
                ">HHHH",
                0,
                0,
                0,
                temp & 0xFFFF,
            )
            self.send(resp_can_id, data_bytes)

        else:
            self.log(f"no handler for mux=0x{mux:02X}")

    def reader_loop(self):
        while not self._stop:
            try:
                if self.dry_run:
                    time.sleep(0.1)
                    continue
                buf = self.sock.recv(FRAME_SIZE)
                if len(buf) < FRAME_SIZE:
                    self.log("short read: ", len(buf))
                    continue
                can_id, dlc, data = unpack_frame(buf)
                self.handle_request(can_id, dlc, data)
            except OSError as e:
                self.log("socket error:", e)
                break

    def periodic_feedback_loop(self):
        # Periodically broadcast feedback frames for our actuator id
        if self.periodic <= 0:
            return
        while not self._stop:
            # Build a fake feedback frame
            # Build a can_id: put actuator id in bits 15-8 and mux 0x02 in top byte, set EFF
            can_id = ((0x02 & 0x1F) << 24) | ((self.actuator_id & 0xFF) << 8) | 0x80000000
            temp = int(25.0 * 10)
            data_bytes = struct.pack(
                ">HHHH",
                0,
                0,
                0,
                temp & 0xFFFF,
            )
            self.send(can_id, data_bytes)
            time.sleep(1.0 / self.periodic)

    def run(self):
        self.open()
        rt = threading.Thread(target=self.reader_loop, daemon=True)
        rt.start()
        pt = None
        if self.periodic > 0:
            pt = threading.Thread(target=self.periodic_feedback_loop, daemon=True)
            pt.start()

        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            self.log("stopping")
            self.close()


def main():
    p = argparse.ArgumentParser(description="CAN motor emulator (SocketCAN)")
    p.add_argument("--iface", default=os.environ.get("CAN_IFACE", "vcan0"), help="CAN interface (default vcan0)")
    p.add_argument("--actuator-id", type=int, default=1, help="Actuator CAN ID (0-255)")
    p.add_argument("--host-id", type=int, default=1, help="Host ID to emulate")
    p.add_argument("--periodic", type=float, default=0.0, help="Periodic feedback rate (Hz). 0 = disabled")
    p.add_argument("--dry-run", action="store_true", help="Don't open socket; just print actions")
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    emu = CanMotorEmulator(args.iface, args.actuator_id, args.host_id, args.periodic, args.dry_run, args.verbose)
    emu.run()


if __name__ == "__main__":
    main()
