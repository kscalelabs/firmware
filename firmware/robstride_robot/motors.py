"""Defines a class that dictates how to communicate with the motors.
   Modified to work with Robstride motors."""

import can
import robstride

from firmware.bionic_motors.motors import *

@dataclass
class RobstrideParams:
    torque_limit: float
    cur_kp: float
    cur_ki: float
    cur_fit_gain: float
    speed_limit: float
    current_limit: float
    loc_kp: float
    spd_kp: float
    spd_ki: float
    spd_fit_gain: float
    
class RobstrideMotor:
    """A class to interface with a motor over a CAN bus."""

    def __init__(self, motor_id: int, control_params: RobstrideParams, client: robstride.Client) -> None:
        """Initializes the motor.

        Args:
        motor_id: The ID of the motor.
        control_params: The control parameters for the motor.
        can_bus: The CAN bus interface.
        """
        self.motor_id = motor_id
        self.control_params = control_params
        self.client = client
        self.set_operation_mode(robstride.RunMode.POSITION)
        self.update_position()
    
    def set_operation_mode(self, mode: robstride.RunMode) -> None:
        """
        Sets the operation mode of the motor to position, speed, or current control
        
        Args:
            mode: The mode to set the motor to.
        """
        self.client.write(self.motor_id, 'run_mode', mode)
        self.client.enable(self.motor_id)
        

    def set_position(self, position: float) -> None:
        """Sets the position of the motor using force position hybrid control.

        Args:
        """
        resp = self.client.write(self.motor_id, 'loc_ref', position)
        self.position = resp.angle

    def set_zero_position(self) -> None:
        """Sets the zero position of the motor."""
        resp = self.client.zero_pos(self.motor_id)

    def get_position(self) -> float:
        """Updates the value of the motor's position attribute.

        Args:
            wait_time: how long to wait for a response from the motor
            read_only: whether to read the position value or not
        Returns:
            "Valid" if the message is valid, "Invalid" otherwise
        """
        resp = self.client.write(self.motor_id, 'mechpos')
        self.position = resp.angle
        return self.position

    def __str__(self) -> str:
        return f"BionicMotor ({self.motor_id})"
