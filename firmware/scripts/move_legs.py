"""Motor Testing Module for Robot Control.

This module provides functionality to test motors of a robot using a configuration file.

Functions:
    test_motor(robot: Robot, config: Dict, motor_num: int) -> None:
        Test a specific motor of the robot based on the provided configuration.

    main() -> None:
        The main function to initialize the robot, zero out its position, and run motor tests.

The script initializes a Robot object with a specific configuration file and setup,
then performs motor testing operations. It includes a commented-out section for
testing an individual motor.

Usage:
    Run this script directly to execute the main function, which will initialize
    the robot and run the motor tests.

Note:
    Ensure that the necessary configuration files and dependencies are in place
before running this script.
"""

import time
from typing import Dict, List

from firmware.robot.robot import Robot


# Utility functions
def degrees_to_int(degrees: float) -> int:
    return max(0, min())

def test_motor(robot: Robot, config: Dict, motor_num: int) -> None:
    robot.test_motor(config["motors"][motor_num], sign=config["signs"][motor_num])


def test_torque_control(robot: Robot, config: Dict) -> None:

    def calculate_motor_current(pos_desired: int, pos_current: int,
                                speed_desired: int, speed_current: int,
                                torque_ff, kp=75, kd=3, kt=1) -> int:
        control_effort = kp * (pos_desired - pos_current) + kd * (speed_desired - speed_current) + torque_ff / kt
        return control_effort
    
    I_max = 20
    desired_positions: List[int] = [60, 0, 0, 0, 0, 0]

    while True:
        for motor_num in range(6):
            motor = config["motors"][motor_num]
            pos_current = motor.position
            speed_current = motor.speed
            print(f"Motor {motor_num}: position={pos_current} speed={speed_current}")
            torque_ff = 0
            control_effort = calculate_motor_current(desired_positions[motor_num], pos_current, 0, speed_current, torque_ff)

            if abs(control_effort) > I_max:
                control_effort = control_effort // abs(control_effort) * I_max
            
            motor.set_position_current_control(control_effort)
            time.sleep(0.1)


def main() -> None:
    robot = Robot(config_path="../robot/config.yaml", setup="right_leg")
    robot.zero_out()
    #robot.test_motors()

    config = robot.motor_config["right_leg"]
    #test_torque_control(robot, config)
    # config = robot.motor_config["right_leg"]
    #test_motor(robot, config, 0)
    while True:
        print(f"Motor 1 at {config['motors'][1].position}")
        time.sleep(0.1)
        config['motors'][1].set_position(1,0,0)

if __name__ == "__main__":
    main()
