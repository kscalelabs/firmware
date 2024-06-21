"""Defines firmware commands for the bionic MyActuator motors."""

import struct
from typing import Literal


def push_bits(value: int, data: int, num_bits: int) -> int:
    value <<= num_bits
    value |= data & ((1 << num_bits) - 1)
    return value


def push_fp32_bits(value: int, data: float) -> int:
    data_bits = struct.unpack("I", struct.pack("f", data))[0]
    value = push_bits(value, data_bits, 32)
    return value


def get_position_command(
    motor_id: int,
    position: float,
    max_speed: float = 60.0,
    max_current: float = 5.0,
    message_return: Literal[0, 1, 2, 3] = 0,
) -> str:
    """Gets the command to set the position of a motor.

    Args:
        motor_id: The ID of the motor.
        position: The position to set the motor to.
        max_speed: The maximum speed of the motor, in rotations per minute.
        max_current: The maximum current of the motor, in amps.
        message_return: The message return status.

    Returns:
        The command to set the position of a motor.
    """
    command = 0
    command = push_bits(command, motor_id, 3)
    command = push_fp32_bits(command, position)
    command = push_bits(command, int(max_speed * 10), 15)
    command = push_bits(command, int(max_current * 10), 12)
    command = push_bits(command, message_return, 2)
    return f"{command:08X}"


if __name__ == "__main__":
    # python -m firmware.motors.bionic_motor
    print(get_position_command(1, 0.0))
    print(get_position_command(1, 90.0))

