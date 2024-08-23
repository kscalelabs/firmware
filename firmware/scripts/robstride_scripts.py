"""Script to test Robstride firmware."""

import time

import can

import firmware.robstride_motors.client as robstride
from firmware.robstride_motors.motors import RobstrideMotor, RobstrideParams


def main() -> None:
    param = RobstrideParams(
        limit_torque=10,
        cur_kp=0.05,
        cur_ki=0.05,
        cur_fit_gain=0.06,
        limit_spd=8,
        limit_cur=20,
        loc_kp=5,
        spd_kp=0.5,
        spd_ki=0.006,
        spd_filt_gain=0.3,
    )

    client = robstride.Client(can.interface.Bus(channel="can2", bustype="socketcan"))
    motor = RobstrideMotor(16, param, client)

    def position_test(top: float = 5 * 2 * 3.14) -> None:
        motor.set_position(top)

        cur_pos = motor.get_position()

        while cur_pos < top:
            cur_pos = motor.get_position()
            print(f"Motor at {cur_pos}")
            time.sleep(0.1)

        motor.set_position(0)

        cur_pos = motor.get_position()

        while cur_pos > 0:
            cur_pos = motor.get_position()
            print(f"Motor at {cur_pos}")
            time.sleep(0.1)

    def torque_test(position: float = 0) -> None:
        def calculate_desired_current(position: float, cur_pos: float, speed: float) -> float:
            kp = 0.2
            kd = 0.01
            torque_ff = 0
            kt = 1
            current = (kp * (position - cur_pos) + kd * (speed - motor.get_speed())) * kt + torque_ff
            return current

        print(motor.get_position())
        motor.set_operation_mode(robstride.RunMode.Current)
        cur_posp = motor.get_position()

        while abs(cur_posp - position) > 0.1:
            cur_posp = motor.get_position()
            # cur_speed = motor.get_speed()
            desired_current = calculate_desired_current(position, cur_posp, 0)
            motor.set_current(desired_current)
            print(f"Motor at {cur_posp}, setting current to {desired_current}")
            time.sleep(0.1)
        print("DONE")


    motor.get_current()
    #motor.calibrate(current_limit=10)
    motor.disable()
    while True:
        print(motor.get_position())
    print("DONE")


if __name__ == "__main__":
    main()
