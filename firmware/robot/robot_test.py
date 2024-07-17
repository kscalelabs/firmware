"""Robot Control Test Module.

This module contains test functions for various robot control operations and a main function
for basic robot initialization.

Functions:
    test_filter_motor_values() -> None:
        Test the filter_motor_values method of the Robot class.

    test_zero_out() -> None:
        Test the zero_out method of the Robot class.

    test_set_position() -> None:
        Test the set_position method of the Robot class.

    main() -> None:
        Initialize a Robot object and set it to its zero position.

The module includes tests for filtering motor values, zeroing out the robot's position,
and setting the robot's position. It also provides a simple main function to initialize
a robot and set it to its zero position.

Usage:
    Run this script directly to execute the main function, which initializes a robot
    and sets it to its zero position. The test functions can be run separately using
    a testing framework.

Note:
    This module assumes the existence of a Robot class with specific methods. Ensure
that the Robot class and its dependencies are properly implemented and available.
"""

from typing import List

from firmware.robot.robot import Robot


def test_filter_motor_values() -> None:
    vals: List[float] = [1.0, 5.0, 3.0]
    max: List[float] = [1.0, 2.0, 3.0]
    filtered: List[float] = Robot.filter_motor_values(vals, max)
    assert filtered == [1.0, 2.0, 3.0]


def test_zero_out() -> None:
    robot = Robot()
    robot.zero_out()
    robot.set_position({"left_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})


def test_set_position() -> None:
    robot = Robot()
    robot.set_position({"left_leg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]})


def main() -> None:
    robot = Robot()
    robot.zero_out()


if __name__ == "__main__":
    main()
