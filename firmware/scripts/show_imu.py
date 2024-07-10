#!/usr/bin/env python
"""Simple script to log the IMU values."""

import argparse

from firmware.cpp.imu.imu import IMU, KalmanFilter


def main() -> None:
    parser = argparse.ArgumentParser(description="Log the IMU values.")
    parser.add_argument("--dt", type=float, default=0.02, help="The time step between measurements")
    parser.add_argument("--bus", type=int, default=1, help="The I2C bus number")
    parser.add_argument("--raw", default=False, action="store_true", help="Print raw values")
    args = parser.parse_args()

    imu = IMU(args.bus)
    kf = KalmanFilter(imu, min_dt=args.dt)

    while True:
        angle = kf.step()
        print(imu.acc_angle() if args.raw else angle)


if __name__ == "__main__":
    # python -m scripts.show_imu
    main()
