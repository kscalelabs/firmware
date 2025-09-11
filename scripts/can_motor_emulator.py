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
from kscale_vr_teleop.analysis.rerun_loader_urdf import URDFLogger
from kscale_vr_teleop._assets import ASSETS_DIR
import rerun as rr

logger = URDFLogger(ASSETS_DIR/'kbot_legless/robot.urdf')
joints_config = {}
motor_id_to_joint_mapping = {
    11: "dof_left_shoulder_pitch_03",
    12: "dof_left_shoulder_roll_03",
    13: "dof_left_shoulder_yaw_02",
    14: "dof_left_elbow_02",
    15: "dof_left_wrist_00",
    21: "dof_right_shoulder_pitch_03",
    22: "dof_right_shoulder_roll_03",
    23: "dof_right_shoulder_yaw_02",
    24: "dof_right_elbow_02",
    25: "dof_right_wrist_00",
}
# Frame layout used by the Rust code: packed repr with total size 16 bytes
# struct CanFrame { u32 can_id; u8 len; u8 pad; u8 res0; u8 len8_dlc; u8 data[8]; }
FRAME_FMT = "<I4B8s"  # little-endian: u32, 4 x u8, 8 bytes
FRAME_SIZE = struct.calcsize(FRAME_FMT)


rr.init("can_motor_emulator")
rr.spawn()
# pub struct CanFrame {
#     pub can_id: u32,
#     pub len: u8,
#     pub pad: u8,
#     pub res0: u8,
#     pub len8_dlc: u8,
#     pub can_data: [u8; CAN_MAX_DLEN],
# }

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
    return can_id & 0xFF


def make_response_can_id(request_can_id: int, resp_mux: int, actuator_id: int) -> int:
    # Build response can_id that will be interpreted as the first 4 bytes of FeedbackResponse
    # FeedbackResponse layout: host_id(0) + actuator_can_id(1) + fault_flags(2) + mux(3)
    # Pack these into a u32 CAN ID in little-endian format
    host_id = 0xFD  # Standard host ID used by firmware  
    fault_flags = 0  # No faults
    
    # Pack as little-endian u32: [host_id, actuator_id, fault_flags, mux]
    response_can_id = (host_id & 0xFF) | ((actuator_id & 0xFF) << 8) | ((fault_flags & 0xFF) << 16) | ((resp_mux & 0xFF) << 24)
    return response_can_id


class ActuatorEmulator:
    def __init__(self, actuator_id: int, host_id: int = 1, verbose: bool = False):
        self.actuator_id = actuator_id
        self.host_id = host_id
        self.verbose = verbose
        
        # Internal state - give each actuator a different initial position for debugging
        self.position = 0.0#(actuator_id % 6) * 0.5  # 0, 0.5, 1.0, 1.5, 2.0, 2.5 radians
        self.velocity = 0.0  # Current velocity 
        self.torque = 0.0    # Current torque
        self.temperature = 25.0  # Temperature in Celsius
        
        if verbose:
            print(f"ActuatorEmulator {actuator_id} initialized with position {self.position} radians")

    def log(self, *a, **kw):
        if self.verbose:
            print(*a, **kw)

    def decode_control_command(self, data: bytes):
        """Decode control command data and update internal state"""
        if len(data) < 8:
            return
        
        # Parse the 8-byte data payload (big-endian u16 values)
        # angle_scale, velocity_scale, kp_scale, kd_scale
        angle_scale, velocity_scale, kp_scale, kd_scale = struct.unpack(">HHHH", data)
        
        # Convert scaled values back to physical values
        # Using typical robstride ranges: angle ±4π, velocity ±0.5, etc.
        ANGLE_RANGE = 4.0 * 3.14159265359  # ±4π radians
        VELOCITY_RANGE = 0.5  # ±0.5 rad/s
        
        # Calculate target values but DON'T overwrite current position immediately
        # In a real actuator, this would be handled by a control loop
        if angle_scale != 0x7FFF:  # 0x7FFF typically means "no command"
            target_position = (angle_scale / 32767.5) * ANGLE_RANGE - ANGLE_RANGE
            self.position = target_position
            self.log(f"[Actuator {self.actuator_id}] Target position: {target_position:.3f} rad (keeping current: {self.position:.3f})")
        
        if velocity_scale != 0x7FFF:
            target_velocity = (velocity_scale / 32767.5) * VELOCITY_RANGE - VELOCITY_RANGE
            self.velocity = target_velocity
            self.log(f"[Actuator {self.actuator_id}] Target velocity: {target_velocity:.3f} rad/s (keeping current: {self.velocity:.3f})")
        
        # Note: In a real system, you'd implement a control loop here that gradually
        # moves the actuator toward the target position/velocity

    def get_feedback_data(self):
        """Generate feedback response data - sensor data only (8 bytes)"""
        # Physical ranges (from robstride_utils.rs)
        ANGLE_MIN = -4.0 * 3.14159265359  # -4π
        ANGLE_MAX = 4.0 * 3.14159265359   # +4π
        VEL_MIN = -0.5
        VEL_MAX = 0.5
        TORQUE_MIN = -14.0
        TORQUE_MAX = 14.0
        
        # CAN range is u16: 0 to 65535
        CAN_MIN = 0.0
        CAN_MAX = 65535.0
        
        # Scale physical values to CAN range using linear interpolation
        # can_value = (physical - phys_min) / (phys_max - phys_min) * (can_max - can_min) + can_min
        def scale_to_can(physical, phys_min, phys_max):
            proportion = (physical - phys_min) / (phys_max - phys_min)
            return int(CAN_MIN + proportion * (CAN_MAX - CAN_MIN))
        
        angle_scaled = scale_to_can(self.position, ANGLE_MIN, ANGLE_MAX)
        vel_scaled = scale_to_can(self.velocity, VEL_MIN, VEL_MAX)
        torque_scaled = scale_to_can(self.torque, TORQUE_MIN, TORQUE_MAX)
        temp_scaled = int(self.temperature * 10)  # Temperature in 0.1°C units
        
        # Clamp to valid u16 range
        angle_scaled = max(0, min(65535, angle_scaled))
        vel_scaled = max(0, min(65535, vel_scaled))
        torque_scaled = max(0, min(65535, torque_scaled))
        temp_scaled = max(0, min(65535, temp_scaled))
        
        # Debug: print what we're encoding
        self.log(f"[Actuator {self.actuator_id}] Encoding: pos={self.position:.3f} -> angle_scaled={angle_scaled} (0x{angle_scaled:04X})")
        
        # The CAN frame structure when cast to FeedbackResponse:
        # Bytes 0-3: CAN ID contains [host_id, actuator_id, fault_flags, mux] 
        # Bytes 4-7: [len, pad, res0, len8_dlc] - these will be the first 4 bytes of data
        # Bytes 8-15: sensor data - these will be the last 8 bytes of data
        
        # First 4 bytes of data payload: [len, pad, res0, len8_dlc]
        # header = struct.pack("BBBB", 
        #                    8,      # len at offset 4
        #                    0,      # pad at offset 5
        #                    0,      # res0 at offset 6  
        #                    0)      # len8_dlc at offset 7
        
        # Last 8 bytes of data payload: big-endian sensor data
        sensor_data = struct.pack(">HHHH",         # Big-endian u16 values at offset 8-15
                                 angle_scaled & 0xFFFF,
                                 vel_scaled & 0xFFFF, 
                                 torque_scaled & 0xFFFF,
                                 temp_scaled & 0xFFFF)
        
        result = sensor_data
        self.log(f"[Actuator {self.actuator_id}] Sending data: {result.hex()} (len={len(result)})")
        return result

    def handle(self, bus, can_id: int, dlc: int, data: bytes):
        mux = mux_from_can_id(can_id)
        req_act_id = actuator_id_from_can_id(can_id)
        self.log(f"[{bus.iface}] REQ can_id=0x{can_id:08X} mux=0x{mux:02X} act={req_act_id} dlc={dlc} data={data.hex()}")

        # Only respond for requests targeting this actuator or broadcast
        if req_act_id not in (self.actuator_id, 0xFF):
            return

        if mux == 0x00:  # ObtainIdRequest -> ObtainIdResponse
            resp_can_id = make_response_can_id(can_id, 0x00, self.actuator_id)
            mcu_uid = (0xDEADBEEFCAF00000 | (self.actuator_id & 0xFF)) & 0xFFFFFFFFFFFFFFFF
            data_bytes = mcu_uid.to_bytes(8, "little")
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x02:  # FeedbackRequest -> FeedbackResponse
            resp_can_id = make_response_can_id(can_id, 0x02, self.actuator_id)
            data_bytes = self.get_feedback_data()
            self.log(f"[Actuator {self.actuator_id}] Feedback response: CAN_ID=0x{resp_can_id:08X}")
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x11:  # ReadParamRequest -> ReadParamResponse
            resp_can_id = make_response_can_id(can_id, 0x11, self.actuator_id)
            if len(data) >= 2:
                index = int.from_bytes(data[0:2], "little")
            else:
                index = 0
            data_bytes = struct.pack("<HHI", index & 0xFFFF, 0, 0)
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x01:  # ControlCommandRequest -> reply with feedback
            self.decode_control_command(data)
            resp_can_id = make_response_can_id(can_id, 0x02, self.actuator_id)
            data_bytes = self.get_feedback_data()
            bus.send(resp_can_id, data_bytes)

        elif mux == 0x03:  # MotorEnableRequest -> reply with feedback
            resp_can_id = make_response_can_id(can_id, 0x02, self.actuator_id)
            data_bytes = self.get_feedback_data()
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
                joints_config.update({
                    motor_id_to_joint_mapping[act.actuator_id]: act.position
                    for act in self.actuators.values()
                    if act.actuator_id in motor_id_to_joint_mapping
                })
                print(joints_config)
                logger.log(joints_config)
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

    # Default behaviour: launch two buses to match firmware configuration
    # can0: actuators 11-16 (left arm)
    # can1: actuators 21-26 (right arm) - based on actual CAN traffic
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
