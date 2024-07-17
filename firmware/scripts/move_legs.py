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

from typing import Dict

from firmware.robot.robot import Robot


def test_motor(robot: Robot, config: Dict, motor_num: int) -> None:
    robot.test_motor(config["motors"][motor_num], sign=config["signs"][motor_num])


def main() -> None:
    robot = Robot(config_path="../robot/config.yaml", setup="right_leg")
    robot.zero_out()
    robot.test_motors()

    # config = robot.motor_config["right_leg"]
    # test_motor(robot, config, 0)


if __name__ == "__main__":
    main()
