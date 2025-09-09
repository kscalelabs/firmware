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


class ActuatorEmulator:
    def __init__(self, actuator_id: int, host_id: int = 1, verbose: bool = False):
        self.actuator_id = actuator_id
        self.host_id = host_id
        self.verbose = verbose

    def log(self, *a, **kw):
        if self.verbose:
            print(*a, **kw)

    def handle(self, bus, can_id: int, dlc: int, data: bytes):
        mux = mux_from_can_id(can_id)
        req_act_id = actuator_id_from_can_id(can_id)
        self.log(f"[{bus.iface}] REQ can_id=0x{can_id:08X} mux=0x{mux:02X} act={req_act_id} dlc={dlc} data={data.hex()}")

        # Only respond for requests targeting this actuator or broadcast
        if req_act_id not in (self.actuator_id, 0xFF):
            return

        if mux == 0x00:  # ObtainIdRequest -> ObtainIdResponse
            resp_can_id = make_response_can_id(can_id, 0x00)
            mcu_uid = (0xDEADBEEFCAF00000 | (self.actuator_id & 0xFF)) & 0xFFFFFFFFFFFFFFFF
            data_bytes = mcu_uid.to_bytes(8, "little")
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x02:  # FeedbackRequest -> FeedbackResponse
            resp_can_id = make_response_can_id(can_id, 0x02)
            angle = 0
            vel = 0
            torque = 0
            temp = int(25.0 * 10)
            data_bytes = struct.pack(
                ">HHHH",
                angle & 0xFFFF,
                vel & 0xFFFF,
                torque & 0xFFFF,
                temp & 0xFFFF,
            )
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x11:  # ReadParamRequest -> ReadParamResponse
            resp_can_id = make_response_can_id(can_id, 0x11)
            if len(data) >= 2:
                index = int.from_bytes(data[0:2], "little")
            else:
                index = 0
            data_bytes = struct.pack("<HHI", index & 0xFFFF, 0, 0)
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x03:  # MotorEnableRequest -> reply with feedback
            resp_can_id = make_response_can_id(can_id, 0x02)
            temp = int(25.0 * 10)
            data_bytes = struct.pack(
                ">HHHH",
                0,
                0,
                0,
                temp & 0xFFFF,
            )
            bus.send(resp_can_id, data_bytes)

        else:
            self.log(f"[{bus.iface}] no handler for mux=0x{mux:02X}")


class BusEmulator:
    def __init__(self, iface: str, actuators: list, host_id: int = 1, dry_run: bool = False, verbose: bool = False, periodic: float = 0.0):
        self.iface = iface
        self.dry_run = dry_run
        self.verbose = verbose
        self.host_id = host_id
        self.periodic = periodic
        self.sock = None
        self._stop = False
        self.actuators = {a: ActuatorEmulator(a, host_id=host_id, verbose=verbose) for a in actuators}

    def log(self, *a, **kw):
        if self.verbose:
            print(*a, **kw)

    def open(self):
        if self.dry_run:
            self.log(f"[{self.iface}] dry-run: not opening socket")
            return
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.log(f"[{self.iface}] opening socket")
        s.bind((self.iface,))
        self.sock = s
        self.log(f"[{self.iface}] bound")

    def close(self):
        self._stop = True
        if self.sock:
            self.sock.close()

    def send(self, can_id: int, data: bytes):
        frame = pack_frame(can_id, 8, data)
        if self.dry_run:
            self.log(f"[{self.iface}] DRY SEND -> can_id=0x{can_id:08X} data={data.hex()}")
            return
        try:
            self.sock.send(frame)
            self.log(f"[{self.iface}] SENT -> can_id=0x{can_id:08X} data={data.hex()}")
        except OSError as e:
            self.log(f"[{self.iface}] send error: {e}")

    def dispatch(self, can_id: int, dlc: int, data: bytes):
        # route to matching actuator(s); if target is 0xFF broadcast to all
        target = actuator_id_from_can_id(can_id)
        if target == 0xFF:
            for a in self.actuators.values():
                a.handle(self, can_id, dlc, data)
        else:
            act = self.actuators.get(target)
            if act:
                act.handle(self, can_id, dlc, data)
            else:
                self.log(f"[{self.iface}] no emulator for actuator {target}")

    def reader_loop(self):
        while not self._stop:
            try:
                if self.dry_run:
                    time.sleep(0.1)
                    continue
                buf = self.sock.recv(FRAME_SIZE)
                if len(buf) < FRAME_SIZE:
                    self.log(f"[{self.iface}] short read: {len(buf)}")
                    continue
                can_id, dlc, data = unpack_frame(buf)
                self.dispatch(can_id, dlc, data)
            except OSError as e:
                self.log(f"[{self.iface}] socket error: {e}")
                break

    def periodic_loop(self):
        if self.periodic <= 0:
            return
        while not self._stop:
            for aid in list(self.actuators.keys()):
                can_id = ((0x02 & 0x1F) << 24) | ((aid & 0xFF) << 8) | 0x80000000
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

    def start(self):
        self.open()
        rt = threading.Thread(target=self.reader_loop, daemon=True)
        rt.start()
        pt = None
        if self.periodic > 0:
            pt = threading.Thread(target=self.periodic_loop, daemon=True)
            pt.start()
        return rt


def main():
    p = argparse.ArgumentParser(description="CAN motor emulator (SocketCAN)")
    p.add_argument("--iface", default=None, help="CAN interface (if provided, run a single bus)")
    p.add_argument("--actuator-id", type=int, default=None, help="Actuator CAN ID (0-255) (if provided, run a single bus)")
    p.add_argument("--host-id", type=int, default=1, help="Host ID to emulate")
    p.add_argument("--periodic", type=float, default=0.0, help="Periodic feedback rate (Hz). 0 = disabled")
    p.add_argument("--dry-run", action="store_true", help="Don't open socket; just print actions")
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    # If user provided a single iface/actuator, run a single bus
    if args.iface and args.actuator_id is not None:
        bus = BusEmulator(args.iface, [args.actuator_id], host_id=args.host_id, dry_run=args.dry_run, verbose=args.verbose, periodic=args.periodic)
        bus.start()
        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            bus.close()
        sys.exit(0)

    # Default behaviour: launch two buses for development convenience
    # can0: actuators 11-16, can1: actuators 21-26
    default_buses = [
        ("can0", list(range(11, 17))),
        ("can1", list(range(21, 27))),
    ]

    threads = []
    buses = []
    for iface, acts in default_buses:
        b = BusEmulator(iface, acts, host_id=args.host_id, dry_run=args.dry_run, verbose=args.verbose, periodic=args.periodic)
        buses.append(b)
        t = b.start()
        threads.append(t)

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        for b in buses:
            b.close()


if __name__ == "__main__":
    main()
