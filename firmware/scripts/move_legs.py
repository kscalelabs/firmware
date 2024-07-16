from firmware.robot.robot import Robot

def main():
    robot = Robot(config_path="../robot/config.yaml", setup="legs")
    #robot.zero_out()
    robot.test_motors()

if __name__ == "__main__":
    main()