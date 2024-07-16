#!/usr/bin/env python
"""Simple script to log the IMU values."""

import argparse
import time

import matplotlib.pyplot as plt  # type: ignore[import-not-found]
import numpy as np  # type: ignore[import-not-found]

from firmware.cpp.imu.imu import IMU, KalmanFilter


def main() -> None:
    parser = argparse.ArgumentParser(description="Log the IMU values.")
    parser.add_argument("--dt", type=float, default=0.02, help="The time step between measurements")
    parser.add_argument("--bus", type=int, default=1, help="The I2C bus number")
    parser.add_argument("--raw", default=False, action="store_true", help="Print raw values")
    parser.add_argument("--delay", type=float, default=0.2, help="How often to print readings")
    parser.add_argument("--plot", type=bool, default=True, help="Display a live plot of the readings")
    parser.add_argument("--print", type=bool, default=True, help="Print out readings")
    args = parser.parse_args()

    imu = IMU(args.bus)
    kf = KalmanFilter(imu, min_dt=args.dt)
    if args.plot:
        live_plot(args, imu, kf)
    elif args.print:
        console(args, imu, kf)


def console(args: argparse.Namespace, imu: IMU, kf: KalmanFilter) -> None:
    print_time = 0

    while True:
        kf.step()

        if print_time > args.delay:
            # print(imu.acc_angle() if args.raw else angle)
            # print(imu.gyr_rate())
            print(imu.get_6DOF())
            print_time = 0
        print_time += args.dt


def live_plot(args: argparse.Namespace, imu: IMU, kf: KalmanFilter) -> None:  # type: ignore[no-untyped-def ]
    def plotter(axs: np.ndarray, lines: list, new_data: list[float], time: float) -> None:
        for ax, line, data in zip(axs.flat, lines, new_data):
            x_data, y_data = line.get_xdata(), line.get_ydata()
            line.set_xdata(np.append(x_data, time))
            line.set_ydata(np.append(y_data, data))
            ax.relim()
            ax.autoscale_view(True, True, True)
        plt.pause(0.001)

    # Setup live plotting
    fig, axs = plt.subplots(2, 3)  # 3 angles and 3 angular velocities
    plt.ion()
    fig.show()
    fig.canvas.draw()
    labels = ["Yaw", "Pitch", "Roll", "Yaw Velocity", "Pitch Velocity", "Roll Velocity"]

    lines = [ax.plot([], [])[0] for ax in axs.flat]
    for ax, label in zip(axs.flat, labels):
        ax.set_title(label)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Degrees" if "Angle" in label else "Degrees/s")

    while True:
        angle = kf.step()

        dof6 = imu.get_6DOF()  # Expected to return a list of 6 values
        data = [angle.yaw, angle.pitch, angle.roll, dof6.z, dof6.x, dof6.y]

        if args.print:
            print(dict(zip(["Yaw", "Pitch", "Roll", "z", "y", "x"], data)))

        plotter(axs, lines, data, time.time())


if __name__ == "__main__":
    # python -m scripts.show_imu
    main()
