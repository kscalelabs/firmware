"""Defines a class that dictates how to communicate with the motors.

Modified to work with Robstride motors.
TODO: create a generic motor class that can work with any motor type.
"""

from dataclasses import dataclass
import time
from typing import Any

import firmware.robstride_motors.client as robstride
from firmware.motor_utils.motor_utils import MotorInterface, MotorParams


@dataclass
class RobstrideParams(MotorParams):
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

    CALIBRATION_SPEED = 0.5 # rad/s

    def __init__(self, motor_id: int, control_params: RobstrideParams, client: robstride.Client) -> None:
        """Initializes the motor.

        Args:
            motor_id: The ID of the motor.
            control_params: The control parameters for the motor.
            client: The CAN bus interface.
        """
        super().__init__(motor_id, control_params, client)
        self.disable()
        self.set_operation_mode(robstride.RunMode.Position)
        self.enable()
        self.get_position()
        self.set_control_params()
        print(f"Motor {motor_id} initialized")

    def disable(self) -> None:
        self.communication_interface.disable(self.motor_id)

    def enable(self) -> None:
        self.communication_interface.enable(self.motor_id)

    def set_control_params(self) -> None:
        for param, value in self.control_params.__dict__.items():
            self.communication_interface.write_param(self.motor_id, param, value)

    def set_operation_mode(self, mode: robstride.RunMode) -> None:
        """Sets the operation mode of the motor to position, speed, or current control!

        Args:
            mode: The mode to set the motor to.
        """
        self.communication_interface.write_param(self.motor_id, "run_mode", mode)
        self.communication_interface.enable(self.motor_id)

    def set_position(self, position: float, **kwargs: Any) -> None:
        """Sets the position of the motor using force position hybrid control.

        Args:
            position: The position to set the motor to.
            kwargs: Additional arguments to pass to the motor.
        """
        resp = self.communication_interface.write_param(self.motor_id, "loc_ref", position)
        self.position = resp.angle

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

    def get_speed(self) -> float:
        """Updates the value of the motor's speed attribute.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the speed value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        resp = self.communication_interface.read_param(self.motor_id, "mechvel")
        if type(resp) is float:
            self.speed = resp
        return self.speed

    def get_current(self) -> float:
        """Updates the value of the motor's current attribute.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the current value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        resp = self.communication_interface.read_param(self.motor_id, "iq")
        if type(resp) is float:
            self.current = resp
        return self.current

    def set_current(self, current: float) -> None:
        """Sets the current of the motor.

        Args:
        current: The current to set the motor to.
        """
        self.communication_interface.write_param(self.motor_id, "iq_ref", current)

    def set_speed(self, speed: float) -> None:
        """Sets the speed of the motor.

        Args:
        speed: The speed to set the motor to in rad/s.
        """
        self.communication_interface.write_param(self.motor_id, "spd_ref", speed)

    def calibrate(self, current_limit: float) -> None:
        """Calibrates the motor assuming the existence of hard stops.

        Args:
        current_limit: The current limit to use during calibration.
        """
        print(f"Calibrating {self}...")

        # Set run mode to speed
        self.set_operation_mode(robstride.RunMode.Speed)

        # Set speed and check for stall
        self.set_speed(self.CALIBRATION_SPEED)
        while self.get_current() < current_limit:
            time.sleep(0.1)

        # Set speed to 0
        self.set_speed(0)

        # Read position and set as high
        high = self.get_position()
        print(f"High: {high}")

        # Set speed and check for stall
        self.set_speed(-self.CALIBRATION_SPEED)
        while self.get_current() < current_limit:
            time.sleep(0.1)

        # Set speed to 0
        self.set_speed(0)

        # Read position and set as low
        low = self.get_position()
        print(f"Low: {low}")

        # Set run mode to position
        self.set_operation_mode(robstride.RunMode.Position)

        # Set zero position
        self.set_position((high + low / 2))
        print(f"Zeroing at {(high + low / 2)}")

        while abs(self.get_position() - (high + low / 2)) > 0.01:
            time.sleep(0.1)

        self.set_zero_position()

        print(f"{self} calibrated")


    def __str__(self) -> str:
        return f"RobstrideMotor ({self.motor_id})"
