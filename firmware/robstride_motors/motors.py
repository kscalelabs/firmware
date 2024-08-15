"""Defines a class that dictates how to communicate with the motors.

Modified to work with Robstride motors.
TODO: create a generic motor class that can work with any motor type.
"""

from dataclasses import dataclass

import firmware.robstride_motors.client as robstride
from firmware.robot_utils.motor_utils import MotorInterface


@dataclass
class RobstrideParams:
    limit_torque: float
    cur_kp: float
    cur_ki: float
    cur_fit_gain: float
    limit_spd: float
    limit_cur: float
    loc_kp: float
    spd_kp: float
    spd_ki: float
    spd_filt_gain: float


class RobstrideMotor(MotorInterface):
    """A class to interface with a motor over a CAN bus."""

    def __init__(self, motor_id: int, control_params: RobstrideParams, client: robstride.Client) -> None:
        """Initializes the motor.

        Args:
            motor_id: The ID of the motor.
            control_params: The control parameters for the motor.
            client: The CAN bus interface.
        """
        super().__init__(motor_id, control_params, client)
        self.disable()
        print("Setting operation mode")
        self.set_operation_mode(robstride.RunMode.Position)
        print("Setting control params")
        self.enable()
        self.get_position()
        self.set_control_params()
        print("Motor initialized")

    def disable(self) -> None:
        self.communication_interface.disable(self.motor_id)

    def enable(self) -> None:
        self.communication_interface.enable(self.motor_id)

    def set_control_params(self) -> None:
        for param, value in self.control_params.items():
            self.communication_interface.write_param(self.motor_id, param, value)

    def set_operation_mode(self, mode: robstride.RunMode) -> None:
        """Sets the operation mode of the motor to position, speed, or current control!

        Args:
            mode: The mode to set the motor to.
        """
        self.communication_interface.write_param(self.motor_id, "run_mode", mode)
        self.communication_interface.enable(self.motor_id)

    def set_position(self, position: float) -> None:
        """Sets the position of the motor using force position hybrid control.

        Args:
            position: The position to set the motor to.
        """
        print(self.motor_id)
        resp = self.communication_interface.write_param(self.motor_id, "loc_ref", position)
        self.position = resp.angle
        print("Position set")

    def set_zero_position(self) -> None:
        """Sets the zero position of the motor."""
        _ = self.communication_interface.zero_pos(self.motor_id)
        self.set_position(0)

    def get_position(self) -> float:
        """Updates the value of the motor's position attribute.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the position value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        resp = self.communication_interface.read_param(self.motor_id, "mechpos")
        if type(resp) is float:
            self.position = resp
        return self.position

    def get_speed(self) -> float | robstride.RunMode:
        """Updates the value of the motor's speed attribute.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the speed value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        resp = self.communication_interface.read_param(self.motor_id, "mechvel")
        self.speed = resp
        return self.speed

    def set_current(self, current: float) -> None:
        """Sets the current of the motor.

        Args:
        current: The current to set the motor to.
        """
        self.communication_interface.write_param(self.motor_id, "iq_ref", current)

    def __str__(self) -> str:
        return f"RobstrideMotor ({self.motor_id})"
