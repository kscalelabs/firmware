#!/usr/bin/env python
"""Simple script to log the IMU values."""

import argparse
import matplotlib.pyplot as plt
import numpy as np

from firmware.cpp.imu.imu import IMU, KalmanFilter


def main() -> None:
    parser = argparse.ArgumentParser(description="Log the IMU values.")
    parser.add_argument("--dt", type=float, default=0.02, help="The time step between measurements")
    parser.add_argument("--bus", type=int, default=1, help="The I2C bus number")
    parser.add_argument("--raw", default=False, action="store_true", help="Print raw values")
    parser.add_argument("--print", type=float, default=0.2, help="How often to print readings")    
    args = parser.parse_args()

    imu = IMU(args.bus)
    kf = KalmanFilter(imu, min_dt=args.dt)

    
def console(args, imu, kf):
    printTime = 0

    while True:
        angle = kf.step()

        if printTime > args.print:
            #print(imu.acc_angle() if args.raw else angle)
            #print(imu.gyr_rate())
            print(imu.get_6DOF())
            printTime = 0
        printTime += args.dt

def live_plot(args, imu, kf):
    def plotter(axs, x_vec, y_data, lines, pause_time=0.1):
        if lines:
            for line, data in zip(lines, y_data):
                line.set_ydata(data)
                line.set_xdata(x_vec)
                line.axes.relim()
                line.axes.autoscale_view()
        else:
            lines = [ax.plot(x_vec, data, 'r')[0] for ax, data in zip(axs, y_data)]
        plt.pause(pause_time)
        return lines
    # Setup live plotting
    fig, axs = plt.subplots(3, 2)  # 3 angles and 3 angular velocities
    plt.ion()
    fig.show()
    fig.canvas.draw()

    size = 100  # Number of points for moving window
    x_vec = np.linspace(0, 1, size)  # Time vector
    y_vecs = [np.zeros(size) for _ in range(6)]  # Initialize y data
    lines = []

    while True:
        angle = kf.step()
        dof6 = imu.get_6DOF()  # Expected to return a list of 6 values
        data = [dof6.yaw, dof6.pitch, dof6.roll, dof6.x, dof6.y, dof6.z]

        y_vecs = [np.append(y_vec[1:], new) for y_vec, new in zip(y_vecs, data)]
        lines = plotter(axs.flat, x_vec, y_vecs, lines)


if __name__ == "__main__":
    # python -m scripts.show_imu
    main()
