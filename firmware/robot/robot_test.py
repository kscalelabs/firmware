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
