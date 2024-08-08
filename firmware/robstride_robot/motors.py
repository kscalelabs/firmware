"""Defines a class that dictates how to communicate with the motors.
   Modified to work with Robstride motors."""

import can
import firmware.robstride_robot.client as robstride

from firmware.bionic_motors.motors import *

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
        self.set_operation_mode(robstride.RunMode.Position)
        self.client.enable(self.motor_id)   
        self.get_position()
        self.set_control_params()

    def disable(self) -> None:
        self.client.disable(self.motor_id)

    def enable(self) -> None:
        self.client.enable(self.motor_id)

    def set_control_params(self) -> None:
        print(self.control_params.__dict__.items())
        for param, value in self.control_params.__dict__.items():
            self.client.write_param(self.motor_id, param, value)
            time.sleep(0.1)
    
    def set_operation_mode(self, mode: robstride.RunMode) -> None:
        """
        Sets the operation mode of the motor to position, speed, or current control
        
        Args:
            mode: The mode to set the motor to.
        """
        self.client.write_param(self.motor_id, 'run_mode', mode)
        self.client.enable(self.motor_id)
        

    def set_position(self, position: float) -> None:
        """Sets the position of the motor using force position hybrid control.

        Args:
        """
        resp = self.client.write_param(self.motor_id, 'loc_ref', position)
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
        resp = self.client.read_param(self.motor_id, 'mechpos')
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
        resp = self.client.read_param(self.motor_id, 'mechvel')
        self.speed = resp
        return self.speed
    
    def set_current(self, current: float) -> None:
        """Sets the current of the motor.

        Args:
        current: The current to set the motor to.
        """
        self.client.write_param(self.motor_id, 'iq_ref', current)

    def __str__(self) -> str:
        return f"BionicMotor ({self.motor_id})"
