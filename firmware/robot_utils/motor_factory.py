"""Motor factory module. Contains a factory class to create motor objects based on configuration."""

from typing import Any

from firmware.bionic_motors.motors import BionicMotor, ControlParams
from firmware.robot_utils.motor_utils import MotorInterface
from firmware.robstride_motors.motors import RobstrideMotor, RobstrideParams


class MotorFactory:
    @staticmethod
    def create_motor(motor: str, motor_id: int, control_params: Any, communication_interface: Any) -> MotorInterface:
        if motor == "bionic":
            bionic_params = ControlParams(
                kp=control_params["kp"],
                kd=control_params["kd"],
            )

            return BionicMotor(motor_id, bionic_params, communication_interface)
        elif motor == "robstride":
            robstride_params = RobstrideParams(
                limit_torque=control_params["limit_torque"],
                cur_kp=control_params["cur_kp"],
                cur_ki=control_params["cur_ki"],
                cur_fit_gain=control_params["cur_fit_gain"],
                limit_spd=control_params["limit_spd"],
                limit_cur=control_params["limit_cur"],
                loc_kp=control_params["loc_kp"],
                spd_kp=control_params["spd_kp"],
                spd_ki=control_params["spd_ki"],
                spd_filt_gain=control_params["spd_filt_gain"],
            )

            return RobstrideMotor(motor_id, robstride_params, communication_interface)
        else:
            raise ValueError(f"Motor type {motor} not recognized.")
