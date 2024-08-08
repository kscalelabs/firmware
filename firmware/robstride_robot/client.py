"""RobStride Robot Client.

Adapted from the Robstride Python SDK here: https://github.com/sirwart/robstride.
"""

import dataclasses
import enum
import math
import struct
from typing import List

import can

from firmware.bionic_motors.commands import push_bits


class RunMode(enum.Enum):
    Operation = 0
    Position = 1
    Speed = 2
    Current = 3


class MotorMsg(enum.Enum):
    Info = 0
    Control = 1
    Feedback = 2
    Enable = 3
    Disable = 4
    ZeroPos = 6
    SetID = 7
    ReadParam = 17
    WriteParam = 18


class MotorMode(enum.Enum):
    Reset = 0
    Calibration = 1
    Run = 2


class MotorError(enum.Enum):
    Undervoltage = 1
    Overcurrent = 2
    Overtemp = 4
    MagneticEncodingFault = 8
    HallEncodingFault = 16
    Uncalibrated = 32


@dataclasses.dataclass
class FeedbackResp:
    servo_id: int
    errors: List[MotorError]
    mode: MotorMode
    angle: float
    velocity: float
    torque: float
    temp: float


params = [
    ("run_mode", 0x7005),
    ("iq_ref", 0x7006),
    ("spd_ref", 0x700A),
    ("limit_torque", 0x700B),
    ("cur_kp", 0x7010),
    ("cur_ki", 0x7011),
    ("cur_fit_gain", 0x7014),
    ("loc_ref", 0x7016),
    ("limit_spd", 0x7017),
    ("limit_cur", 0x7018),
    ("mechpos", 0x7019),
    ("iqf", 0x701A),
    ("mechvel", 0x701B),
    ("vbus", 0x701C),
    ("loc_kp", 0x701E),
    ("spd_kp", 0x701F),
    ("spd_ki", 0x7020),
    ("spd_filt_gain", 0x7021),
]

param_ids_by_name = dict(params)


class Client:
    def __init__(self, bus: can.BusABC, retry_count: int = 2, recv_timeout: int = 2, host_can_id: int = 0xAA) -> None:
        self.bus = bus
        self.retry_count = retry_count
        self.recv_timeout = recv_timeout
        self.host_can_id = host_can_id
        self._recv_count = 0
        self._recv_error_count = 0

    def enable(self, motor_id: int, motor_model: int = 1) -> FeedbackResp:
        self.bus.send(self._rs_msg(MotorMsg.Enable, self.host_can_id, motor_id, bytes([0, 0, 0, 0, 0, 0, 0, 0])))
        resp = self._recv()
        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def disable(self, motor_id: int, motor_model: int = 1) -> FeedbackResp:
        self.bus.send(self._rs_msg(MotorMsg.Disable, self.host_can_id, motor_id, bytes([0, 0, 0, 0, 0, 0, 0, 0])))
        resp = self._recv()
        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def update_id(self, motor_id: int, new_motor_id: int) -> None:
        id_data_1 = self.host_can_id | (new_motor_id << 8)
        self.bus.send(self._rs_msg(MotorMsg.SetID, id_data_1, motor_id, bytes([0, 0, 0, 0, 0, 0, 0, 0])))
        self._recv()

    def zero_pos(self, motor_id: int, motor_model: int = 1) -> FeedbackResp:
        self.bus.send(
            self._rs_msg(MotorMsg.ZeroPos, self.host_can_id, motor_id, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
        )  # TODO: test this function
        resp = self._recv()
        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def use_control_mode(
        self, motor_id: int, torque: float, velocity: float, position: float, kp: float, kd: float
    ) -> None:
        data = self._convert_to_bytes(position, velocity, kp, kd)
        torque = max(min(torque, 120.0), -120.0)
        moment_bytes = int(((torque + 120.0) / 240.0) * 65535)
        self.bus.send(self._rs_msg(MotorMsg.Control, moment_bytes, motor_id, data))

    def get_motor_info(self, motor_id: int) -> bytearray:
        self.bus.send(self._rs_msg(MotorMsg.Info, self.host_can_id, motor_id, bytes([0, 0, 0, 0, 0, 0, 0, 0])))
        resp = self._recv()
        return resp.data

    def read_param(self, motor_id: int, param_id: int | str) -> float | RunMode:
        param_id = self._normalize_param_id(param_id)

        data = [param_id & 0xFF, param_id >> 8, 0, 0, 0, 0, 0, 0]
        self.bus.send(self._rs_msg(MotorMsg.ReadParam, self.host_can_id, motor_id, bytes(data)))
        resp = self._recv()

        while not self._parse_and_validate_read_resp_arbitration_id(resp, MotorMsg.ReadParam.value, motor_id):
            resp = self._recv()

        resp_param_id = struct.unpack("<H", resp.data[:2])[0]
        if resp_param_id != param_id:
            raise Exception("Invalid param id")

        if param_id == 0x7005:
            value = RunMode(int(resp.data[4]))
        else:
            value = struct.unpack("<f", resp.data[4:])[0]

        return value

    def write_param(
        self, motor_id: int, param_id: int | str, param_value: float | RunMode | int, motor_model: int = 1
    ) -> FeedbackResp:
        param_id = self._normalize_param_id(param_id)

        data = bytes([param_id & 0xFF, param_id >> 8, 0, 0])
        if param_id == 0x7005:
            if isinstance(param_value, RunMode):
                int_value = int(param_value.value)
            elif isinstance(param_value, int):
                int_value = param_value
            data += bytes([int_value, 0, 0, 0])
        else:
            data += struct.pack("<f", param_value)

        self.bus.send(self._rs_msg(MotorMsg.WriteParam, self.host_can_id, motor_id, data))
        resp = self._recv()

        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def error_rate(self) -> float:
        return self._recv_error_count / self._recv_count

    def _rs_msg(self, msg_type: MotorMsg, id_data_1: int, id_data_2: int, data: bytes) -> can.Message:
        arb_id = id_data_2 + (id_data_1 << 8) + (msg_type.value << 24)
        return can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)

    def _recv(self) -> can.Message:
        retry_count = 0
        while retry_count <= self.retry_count:
            self._recv_count += 1
            resp = self.bus.recv(self.recv_timeout)
            if not resp:
                raise Exception("No response from motor received")
            if not resp.is_error_frame:
                return resp

            retry_count += 1
            self._recv_error_count += 1
            # TODO: make logging configurable
            print("received error:", resp)

        raise Exception("Error reading resp:", resp)

    def _parse_resp_abitration_id(self, aid: int) -> tuple:
        msg_type = (aid & 0x1F000000) >> 24
        msg_motor_id = (aid & 0xFF00) >> 8
        host_id = aid & 0xFF
        return msg_type, msg_motor_id, host_id

    def _parse_and_validate_read_resp_arbitration_id(
        self, resp: can.Message, expected_msg_type: int, expected_motor_id: int
    ) -> bool:
        msg_type, msg_motor_id, host_id = self._parse_resp_abitration_id(resp.arbitration_id)
        if msg_type != expected_msg_type:
            return False
        if host_id != self.host_can_id:
            raise Exception("Invalid host CAN ID", resp)
        if msg_motor_id != expected_motor_id:
            raise Exception("Invalid motor ID received", resp)
        return True

    def _parse_and_validate_resp_arbitration_id(
        self, resp: can.Message, expected_msg_type: int, expected_motor_id: int
    ) -> tuple:
        msg_type, msg_motor_id, host_id = self._parse_resp_abitration_id(resp.arbitration_id)
        if msg_type != expected_msg_type:
            raise Exception("Invalid msg_type", resp)
        if host_id != self.host_can_id:
            raise Exception("Invalid host CAN ID", resp)
        if msg_motor_id != expected_motor_id:
            raise Exception("Invalid motor ID received", resp)

        return msg_type, msg_motor_id, host_id

    def _parse_feedback_resp(self, resp: can.Message, motor_id: int, motor_model: int) -> FeedbackResp:
        self._parse_and_validate_resp_arbitration_id(resp, MotorMsg.Feedback.value, motor_id)

        aid = resp.arbitration_id
        error_bits = (aid & 0x1F0000) >> 16
        errors = []
        for i in range(6):
            value = 1 << i
            if value & error_bits:
                errors.append(MotorError(value))

        mode = MotorMode((aid & 0x400000) >> 22)

        angle_raw = struct.unpack(">H", resp.data[0:2])[0]
        angle = (float(angle_raw) / 65535 * 8 * math.pi) - 4 * math.pi

        velocity_raw = struct.unpack(">H", resp.data[2:4])[0]
        velocity_range = 88 if motor_model == 1 else 30
        velocity = (float(velocity_raw) / 65535 * velocity_range) - velocity_range / 2

        torque_raw = struct.unpack(">H", resp.data[4:6])[0]
        torque_range = 34 if motor_model == 1 else 240
        torque = (float(torque_raw) / 65535 * torque_range) - torque_range / 2

        temp_raw = struct.unpack(">H", resp.data[6:8])[0]
        temp = float(temp_raw) / 10

        return FeedbackResp(motor_id, errors, mode, angle, velocity, torque, temp)

    def _normalize_param_id(self, param_id: int | str) -> int:
        if isinstance(param_id, str):
            return param_ids_by_name[param_id]

        return param_id

    def _convert_to_bytes(self, angle: float, angular_velocity: float, kp: float, kd: float) -> bytes:
        # Ensure values are within their respective ranges
        angle = max(min(angle, 4 * 3.14159), -4 * 3.14159)
        angular_velocity = max(min(angular_velocity, 15.0), -15.0)
        kp = max(min(kp, 5000.0), 0.0)
        kd = max(min(kd, 100.0), 0.0)

        # Convert each parameter to the corresponding byte values in big-endian order
        angle_bytes = int(((angle + 4 * 3.14159) / (8 * 3.14159)) * 65535)
        angular_velocity_bytes = int(((angular_velocity + 15.0) / 30.0) * 65535)
        kp_bytes = int((kp / 5000.0) * 65535)
        kd_bytes = int((kd / 100.0) * 65535)

        command = 0
        command = push_bits(command, angle_bytes, 16)
        command = push_bits(command, angular_velocity_bytes, 16)
        command = push_bits(command, kp_bytes, 16)
        command = push_bits(command, kd_bytes, 16)

        return command.to_bytes(8, byteorder="little")
