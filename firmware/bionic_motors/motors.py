"""Defines a class that dictates how to communicate with the motors."""

from dataclasses import dataclass
import time
import can
import math
import struct
from typing import List, Literal

from .commands import (
    force_position_hybrid_control, 
    set_zero_position,
    get_motor_pos # ... add more later
)

from .responses import (
    valid_message,
    read_result,
)

@dataclass
class ControlParams:
    kp: float
    kd: float
    
@dataclass
class CANInterface:
    bus: can.interface.Bus
    channel: can.BufferedReader
    bustype: can.Notifier
    
@dataclass
class CanMessage:
    id: int
    data: str

class BionicMotor:
    """A class to interface with a motor over a CAN bus."""

    def __init__(
        self,
        motor_id: int,
        control_params: ControlParams,
        can_bus: CANInterface
    ):
        """
        Args:
            motor_id: The ID of the motor.
            control_params: The control parameters for the motor.
            can_bus: The CAN bus interface
        """
        self.motor_id = motor_id
        self.control_params = control_params
        self.can_bus = can_bus
        self.can_messages = []
        self.position = 0 # don't care here, but NOTE should not always be 0 at the start
        self.get_position()

    
    def send(self, can_id: int, data: bytes, length: int = 8) -> None:
        """Sends a CAN message to a motor.

        Args:
            id: The motor ID.
            data: The data to send.
        """
        assert len(data) == length, f"Data length must be {length} bytes"
        print("CAN Command: ", hex(can_id), hex(int.from_bytes(data, "big")))
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.can_bus.bus.send(message)
        
    def read(self, timeout: float = 0.25) -> None:
        """Generic read can bus method that reads messages from the can bus
        
        Args:
            timeout: how long to read messages for in seconds      
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            message = self.can_bus.channel.get_message(timeout=timeout)
            if message is not None:
                if valid_message(message.data):
                    message_id = message.arbitration_id
                    message_data = read_result(message.data)
                    self.can_messages.append(CanMessage(id=message_id, data=message_data))
                    print("CAN Message: ", message_id, message_data)
                else:
                    print("Invalid message")  
        
    def _send(self, id: int, data: bytes, length: int = 8) -> None:
        """Sends a CAN message to a motor.

        Args:
            id: The motor ID.
            data: The data to send.
        """
        can_id = id
        assert len(data) == length, f"Data length must be {length} bytes"
        print("CAN Command: ", hex(can_id), hex(int.from_bytes(data, "big")))
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
        )
        self.can_bus.bus.send(message)

    def set_position(self, position: float, speed: float, torque: int) -> None:
        """Sets the position of the motor using force position hybrid control

        Args:
            position: The position to set the motor to (in degrees)
            speed: The speed to set the motor to (in rpm)
            torque: The torque to set the motor to (in Nm)
        """
        command = force_position_hybrid_control(self.control_params.kp, self.control_params.kd, position, speed, torque)
        self._send(self.motor_id, bytes(command))

    def set_zero_position(self) -> None:
        """Sets the zero position of the motor."""
        command = set_zero_position(self.motor_id)
        self._send(self.motor_id, bytes(command), 4)

    def update_position(self, wait_time: float = 0.15) -> str:
        """
        Updates the value of the motor's position attribute
        NOTE: Do NOT use this to access the motor's position value.
        Just use <motor>.position instead.
        
        Args:
            wait_time: how long to wait for a response from the motor
        """
        command = get_motor_pos()
        self._send(self.motor_id, bytes(command), 2)
        self.read(wait_time)

        for message in self.can_messages:
            if message.id == self.motor_id and message.data["Message Type"] == 5:
                self.position = message.data["Data"]
                return "Valid"
            else:
                return "Invalid"

    def __str__(self) -> str:
        return f"BionicMotor ({self.motor_id})"





