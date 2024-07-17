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
def degrees_to_radians(degrees: float) -> float:
    return degrees * (3.14159 / 180)

def test_motor(robot: Robot, config: Dict, motor_num: int) -> None:
    robot.test_motor(config["motors"][motor_num], sign=config["signs"][motor_num])


def test_torque_control(robot: Robot, config: Dict) -> None:
    # In an ideal world, kt would actually be the torque constant, but we're using it as a scaling factor
    last_t = time.time()
    integral_errors: List[int] = [0, 0, 0, 0, 0, 0]
    last_errors: List[int] = [0, 0, 0, 0, 0, 0]
    def calculate_motor_current(pos_desired: int, pos_current: int,
                                speed_desired: int, speed_current: int,
                                torque_ff, kp=2, kd=1.5, kt=10) -> int:
        control_effort = (kp * (pos_desired - pos_current) + kd * (speed_desired - speed_current) + torque_ff) / kt
        return control_effort


    
    I_max = 40
    desired_positions: List[int] = [30, 0, 0, 0, 0, 0]

    while True:
        for motor_num in range(6):
            motor = config["motors"][motor_num]
            pos_current = motor.position
            speed_current = motor.speed
            #print(f"Motor {motor_num}: position={pos_current} speed={speed_current}")
            if motor_num < 2: torque_ff = 0
            else: torque_ff = 0
            control_effort = calculate_motor_current(desired_positions[motor_num], pos_current, 0, speed_current, torque_ff, motor_num)
            print(f"Motor {motor_num} error: {pos_current - desired_positions[motor_num]} control effort: {control_effort}")
            if abs(control_effort) > I_max:
                control_effort = control_effort // abs(control_effort) * I_max
            
            motor.set_position_current_control(control_effort)
            motor.update_position()
            motor.update_speed()


def main() -> None:
    robot = Robot(config_path="../robot/config.yaml", setup="right_leg")
    #robot.zero_out()
    robot.test_motors()

    config = robot.motor_config["right_leg"]
    test_torque_control(robot, config)
    #test_motor(robot, config, 3)

def mac():
    def calculate_motor_current(pos_desired: int, pos_current: int,
                                speed_desired: int, speed_current: int,
                                torque_ff, kp=15, kd=0.5, kt=25) -> int:
        control_effort = (kp * (pos_desired - pos_current) + kd * (speed_desired - speed_current) + torque_ff) / kt
        return control_effort

    test_position = 60
    pos_current = 0
    while True:
        speed_current = 0
        torque_ff = 0
        control_effort = calculate_motor_current(test_position, pos_current, 0, speed_current, torque_ff)
        print(f"Motor error: {pos_current - test_position} control effort: {control_effort}")
        pos_current += 1
        if pos_current > 60:
            pos_current = 0
        time.sleep(0.4)

if __name__ == "__main__":
    #mac()
    main()
