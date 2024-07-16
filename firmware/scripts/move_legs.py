from firmware.robot.robot import Robot

def test_motor(robot, config, motor_num):
    robot.test_motor(config['motors'][motor_num], sign=config['signs'][motor_num])

def main():
    robot = Robot(config_path="../robot/config.yaml", setup="legs")
    robot.zero_out()
    robot.test_motors()
    leftConfig = robot.motor_config['left_leg']

    #test_motor(robot, leftConfig, 0)

if __name__ == "__main__":
    main()