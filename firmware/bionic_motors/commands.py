"""Defines firmware commands for the bionic MyActuator motors."""

import math
import struct
from typing import List, Literal

#######################################
# Bitwise helpers
#######################################


def push_bits(value: int, data: int, num_bits: int) -> int:
    value <<= num_bits
    value |= data & ((1 << num_bits) - 1)
    return value


def push_fp32_bits(value: int, data: float) -> int:
    data_bits = struct.unpack("I", struct.pack("f", data))[0]
    value = push_bits(value, data_bits, 32)
    return value


def split_into_bytes(command: int, length: int = 8, little_endian: bool = True) -> List[int]:
    bytes_list = []
    for i in range(length):
        bytes_list.append(command & 0xFF)
        command = command >> 8
    if little_endian:
        bytes_list = bytes_list[::-1]
    return bytes_list


#######################################
# Movement commands
#######################################


def set_position_control(
    motor_id: int,  # NOTE: you need to specify the motor id as the CAN identifier
    position: float,
    motor_mode: int = 1,
    max_speed: float = 60.0,
    max_current: float = 5.0,
    message_return: Literal[0, 1, 2, 3] = 0,
) -> List[int]:
    """Gets the command to set the position of a motor. Expect 8 bytes.

    Args:
        motor_id: The ID of the motor.
        motor_mode: 0x1 for servo position control.
        position: The position to set the motor to.
        max_speed: The maximum speed of the motor, in rotations per minute.
        max_current: The maximum current of the motor, in amps.
        message_return: The message return status.

    Returns:
        The command to set the position of a motor.
    """

    command = 0
    command = push_bits(command, motor_mode, 3)
    command = push_fp32_bits(command, position)
    command = push_bits(command, int(max_speed * 10), 15)
    command = push_bits(command, int(max_current * 10), 12)
    command = push_bits(command, message_return, 2)
    return split_into_bytes(command)


def set_speed_control(
    motor_id: int,  # NOTE: you need to specify the motor id as the CAN identifier
    speed: float,
    motor_mode: int = 2,
    current: float = 5.0,
    message_return: Literal[0, 1, 2, 3] = 0,
) -> List[int]:
    """Gets the command to set the speed of a motor. Expect 7 bytes

    Args:
        motor_id: The ID of the motor.
        motor_mode: 0x2 for speed control.
        speed: The speed to set the motor to, in rotations per minute.
        current: The current of the motor, in amps. 0 to 65536 corresponds to 0 to 6553.6 A.
        message_return: The message return status.

    Returns:
        The command to set the speed of a motor.
    """

    command = 0
    command = push_bits(command, motor_mode, 3)
    command = push_bits(command, 0, 3)
    command = push_bits(command, message_return, 2)
    command = push_fp32_bits(command, speed)
    command = push_bits(command, int(current * 10), 16)
    return split_into_bytes(command, 7)


def set_current_torque_control(
    motor_id: int,  # NOTE: you need to specify the motor id as the CAN identifier
    value: int,
    control_status: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 0,
    motor_mode: int = 3,
    message_return: Literal[0, 1, 2, 3] = 0,
) -> List[int]:
    """Gets the command to set the current OR torque of a motor. Expect 3 bytes

    Args:
        motor_id: The ID of the motor.
        motor_mode: 0x3 for current control.
        value: The current (A) or torque (N*m) to set the motor to, x10. (int16, not uint16)
        message_return: The message return status.
        control_status: 0x0 for current control, 0x1 for torque control.
                        See page 18-19 of X12-150 manual for more options.

    Returns:
        The command to set the current of a motor.
    """

    command = 0
    command = push_bits(command, motor_mode, 3)
    command = push_bits(command, control_status, 3)
    command = push_bits(command, message_return, 2)
    command = push_bits(command, (int)(value * 10), 16)  # TODO, needs testing for int16
    return split_into_bytes(command, 3)


def set_zero_position(motor_id: int) -> List[int]:
    """Gets the command to set the zero position of a motor. Expect 4 bytes

    Args:
        motor_id: The ID of the motor.

    Returns:
        The command to set the zero position of a motor.
    """

    upper = motor_id >> 8
    lower = motor_id & 0xFF
    command = 0
    command = push_bits(command, upper, 8)
    command = push_bits(command, lower, 8)
    command = push_bits(command, 0, 8)
    command = push_bits(command, 3, 8)
    return split_into_bytes(command, 4)


#######################################
# Motor information commands
#######################################


def get_motor_pos():
    """Gets the motor Position of a respective motor
    Args:
        motor_id: The ID of the motor.

    Returns:
        The respective motor position
    """

    command = 0
    command = push_bits(command, 0x7, 3)
    command = push_bits(command, 0x0, 5)
    command = push_bits(command, 0x1, 8)
    return split_into_bytes(command, 2)


def get_motor_speed(motor_id: int):
    """
    Gets the motor Position of a respective motor
    Args:
        motor_id: The ID of the motor.

    Returns:
        The respective motor speed.
    """
    command = 0
    command = push_bits(command, 0x7, 3)
    command = push_bits(command, 0x0, 5)
    command = push_bits(command, 0x2, 8)
    return split_into_bytes(command, 2)


def get_motor_current():
    """
    Gets the motor Position of a respective motor
    Args:
        motor_id: The ID of the motor.

    Returns:
        The respective motor current draw
    """
    command = 0
    command = push_bits(command, 0x7, 3)
    command = push_bits(command, 0x0, 5)
    command = push_bits(command, 0x3, 8)
    return split_into_bytes(command, 2)


def get_motor_power():
    """
    Gets power consumption of a respective motor
    Args:
        motor_id: The ID of the motor.

    Returns:
        The respective motor power consumption
    """

    command = 0
    command = push_bits(command, 0x7, 3)
    command = push_bits(command, 0x0, 5)
    command = push_bits(command, 0x4, 8)
    return split_into_bytes(command, 2)


#######################################
# Motor feedback control commands
#######################################


def force_position_hybrid_control(kp: float, kd: float, position: float, speed: float, torque_ff: int) -> List[int]:
    """Gets the command to set the position of a motor using PD control. Expect 8 bytes

    Args:
        kp: The proportional gain. No load default is 15
        kd: The derivative gain. No load default is 0.5
        position: The position to set the motor to, in degrees.
        speed: The speed to set the motor to, in rpm.
        torque_ff: The feedforward torque in Nm.

    Returns:
        The command to set the position of a motor using PD control.
    """
    degrees_to_int = lambda degrees: max(0, min(65536, int(((math.radians(degrees) + 12.5) / 25.0) * 65536)))
    rpm_to_int = lambda rpm: max(0, min(4095, int(((rpm + 18.0) / 36.0) * 4095)))
    torque_to_int = lambda torque: max(0, min(4095, int(((torque + 150) / 300) * 4095)))

    command = 0
    command = push_bits(command, 0, 3)
    command = push_bits(command, int(kp * 4095 / 500), 12)
    command = push_bits(command, int(kd * 511 / 5), 9)
    command = push_bits(command, int(degrees_to_int(position)), 16)
    command = push_bits(command, int(rpm_to_int(speed)), 12)
    command = push_bits(command, int(torque_to_int(torque_ff)), 12)
    return split_into_bytes(command)


def debug(command: bytes):
    return [hex(i) for i in command]


if __name__ == "__main__":
    # python - m firmware.motors.bionic_motor
    print(debug(set_zero_position(1)))
