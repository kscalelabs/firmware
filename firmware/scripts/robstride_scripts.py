"""Script to test Robstride firmware."""
import can
import robstride

from firmware.robstride_robot.motors import RobstrideMotor, RobstrideParams


def main() -> None:
    param = RobstrideParams(torque_limit = 0,
                            cur_kp = 0,
                            cur_ki = 0,
                            cur_fit_gain = 0,
                            speed_limit = 0,
                            current_limit = 0,
                            loc_kp = 0,
                            spd_kp = 0,
                            spd_ki = 0,
                            spd_fit_gain = 0)

    client = robstride.Client(can.interface.Bus(channel="can0", bustype="interface"))
    motor = RobstrideMotor(1, param, client)

    motor.set_position(0)


if __name__ == "__main__":
    main()
