"""Script to identify Robstride motors on bus."""

import argparse

import can

import firmware.robstride_motors.client as robstride
from firmware.robstride_motors.motors import RobstrideMotor, RobstrideParams


def main() -> None:
    argparser = argparse.ArgumentParser(description="Identify Robstride motors on bus")
    argparser.add_argument("--channel", type=str, default="can0", help="CAN channel to use")
    argparser.add_argument("--verbose", default=False, action="store_true", help="Print verbose output")
    args = argparser.parse_args()

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

    client = robstride.Client(can.interface.Bus(channel=args.channel, interface="socketcan"))
    valid_ids = []
    for motor_id in range(1, 130):
        try:
            motor = RobstrideMotor(motor_id, param, client)
            motor.get_position()
            valid_ids.append(motor_id)
        except Exception as e:
            if args.verbose:
                print(f"Motor {motor_id} not found: {e}")
            pass
    print(f"Found motors: {valid_ids}")


if __name__ == "__main__":
    main()
