"""Defines a class that dictates how to communicate with the motors."""

import time
from dataclasses import dataclass
from typing import Any, List

import can

from firmware.bionic_motors.commands import (
    force_position_hybrid_control,
    get_motor_pos,
    get_motor_speed,
    set_current_torque_control,
    set_zero_position,
)
from firmware.bionic_motors.responses import read_result, valid_message
from firmware.motor_utils.motor_utils import MotorInterface, MotorParams

SPECIAL_IDENTIFIER = 0x7FF


@dataclass
class ControlParams(MotorParams):
    kp: float
    kd: float


@dataclass
class CANInterface:
    bus: Any
    channel: can.BufferedReader
    bustype: can.Notifier


@dataclass
class CanMessage:
    id: int
    data: Any


class BionicMotor(MotorInterface):
    """A class to interface with a motor over a CAN bus."""

    can_messages: List[Any] = []

    def __init__(self, motor_id: int, control_params: ControlParams, can_bus: CANInterface) -> None:
        """Initializes the motor.

        Args:
        motor_id: The ID of the motor.
        control_params: The control parameters for the motor.
        can_bus: The CAN bus interface.
        """
        super().__init__(motor_id, control_params, can_bus)
        self.update_position()

    def send(self, can_id: int, data: bytes, length: int = 8) -> None:
        """Sends a CAN message to a motor.

        Args:
            can_id: The motor ID.
            data: The data to send.
            length: The length of the data.
        """
        assert len(data) == length, f"Data length must be {length} bytes"
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.communication_interface.bus.send(message)

    def read(self, timeout: float = 0.001, read_data_only: bool = True) -> None:
        """Generic read can bus method that reads messages from the can bus.

        Args:
            timeout: how long to read messages for in seconds
            read_data_only: whether to read only data that has been queried. If true, only type 5 messages are read.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            message = self.communication_interface.channel.get_message(timeout=timeout)
            if message is not None:
                if valid_message(message.data):
                    message_id = message.arbitration_id
                    message_data = read_result(message.data)
                    if read_data_only:
                        if message_data and message_data["Message Type"] == 5:
                            BionicMotor.can_messages.append(CanMessage(id=message_id, data=message_data))
                    else:
                        BionicMotor.can_messages.append(CanMessage(id=message_id, data=str(message_data)))
                else:
                    pass

    def set_position(self, position: float, **kwargs: Any) -> None:
        """Sets the position of the motor using force position hybrid control.

        Args:
            position: The position to set the motor to (in degrees)
            kwargs: Additional arguments to pass to the motor. (speed in rpm, torque in Nm)
        """
        speed = kwargs.get("speed", 0)
        torque = kwargs.get("torque", 0)

        command = force_position_hybrid_control(self.control_params.kp, self.control_params.kd, position, speed, torque)
        self.send(self.motor_id, bytes(command))

    def set_current(self, current: float) -> None:
        """Sets the current of the motor.

        Args:
            current: The current to set the motor to (in A)
        """
        command = set_current_torque_control(motor_id=self.motor_id, value=int(current), control_status=0)
        self.send(SPECIAL_IDENTIFIER, bytes(command), 3)

    def set_zero_position(self) -> None:
        """Sets the zero position of the motor."""
        command = set_zero_position(self.motor_id)
        self.send(SPECIAL_IDENTIFIER, bytes(command), 4)

    def get_position(self) -> float:
        """Gets the current position of the motor."""
        self.update_position()
        return self.position

    def get_speed(self) -> float:
        """Gets the current speed of the motor."""
        self.update_speed()
        return self.speed

    def update_position(self, wait_time: float = 0.001) -> None:
        """Updates the value of the motor's position attribute.

        NOTE: Do NOT use this to access the motor's position value.

        Just use <motor>.position instead.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the position value or not
        """
        command = get_motor_pos()
        self.send(self.motor_id, bytes(command), 2)
        self.read(wait_time)
        for message in BionicMotor.can_messages:
            if message.id == self.motor_id and message.data["Message Type"] == 5:
                BionicMotor.can_messages.remove(message)
                self.position = message.data["Data"]
                return
            else:
                # Clear buffer of any non-position messages
                BionicMotor.can_messages.remove(message)
                continue

    def update_speed(self, wait_time: float = 0.001) -> str:
        """Updates the value of the motor's speed attribute.

        NOTE: Do NOT use this to access the motor's speed value.

        Just use <motor>.speed instead.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the speed value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        command = get_motor_speed(self.motor_id)
        self.send(self.motor_id, bytes(command), 2)
        self.read(wait_time)
        for message in BionicMotor.can_messages:
            if message.id == self.motor_id and message.data["Message Type"] == 5:
                BionicMotor.can_messages.remove(message)
                self.speed = message.data["Data"]
                # Flushes out any previous messages and ensures that the next message is fresh
                return "Valid"
            else:
                continue
                # return "Invalid"
        return "Valid"

    def calibrate(self, current_limit: float) -> None:
        """Calibrates motor assuming the existence of hard stops.

        TODO: Implement calibration method.

        Args:
            current_limit: The current limit to use for calibration.
        """
        pass

    def __str__(self) -> str:
        return f"BionicMotor ({self.motor_id})"
