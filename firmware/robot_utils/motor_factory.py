from firmware.bionic_motors.motors import BionicMotor
from firmware.robstride_motors.motors import RobstrideMotor
from typing import Any
from firmware.robot_utils.motor_utils import MotorInterface


class MotorFactory:
    @staticmethod
    def create_motor(motor: str, motor_id: int, control_params: Any, communication_interface: Any) -> MotorInterface:
        if motor == "bionic":
            return BionicMotor(motor_id, control_params, communication_interface)
        elif motor == "robstride":
            return RobstrideMotor(motor_id, control_params, communication_interface)
        else:
            raise ValueError(f"Motor type {motor} not recognized.")
