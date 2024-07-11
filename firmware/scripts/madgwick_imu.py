#!/usr/bin/env python
"""Simple script to log the IMU values."""

import argparse
import matplotlib.pyplot as plt
import numpy as np
import time

import imufusion

from firmware.cpp.imu.imu import IMU


def main() -> None:
    parser = argparse.ArgumentParser(description="Log the IMU values.")
    parser.add_argument("--dt", type=float, default=0.02, help="The time step between measurements")
    parser.add_argument("--bus", type=int, default=1, help="The I2C bus number")
    parser.add_argument("--raw", default=False, action="store_true", help="Print raw values")
    parser.add_argument("--delay", type=float, default=0.2, help="How often to print readings")
    parser.add_argument("--plot", type=bool, default=False, help="Display a live plot of the readings")
    parser.add_argument("--print", type=bool, default=True, help="Print out readings")
    args = parser.parse_args()

    global imu, ahrs

    imu = IMU(args.bus)

    # Process sensor data
    ahrs = imufusion.Ahrs()

    if args.plot:
        live_plot(args, imu)
    elif args.print:
        console(args, imu)

def get_imu_data():
    gyro = imu.raw_gyr()
    acc = imu.raw_acc()

    return [[gyro.x, gyro.y, gyro.z], [acc.x, acc.y, acc.z]]

        
def console(args):
    printTime = 0

    while True:
        if printTime > args.delay:
            #print(imu.acc_angle() if args.raw else angle)
            #print(imu.gyr_rate())
            print(imu.get_6DOF())
            printTime = 0
        printTime += args.dt

def live_plot(args, imu, kf):
    def plotter(axs, lines, new_data, time):
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
    labels = ['Angle 1', 'Angle 2', 'Angle 3', 'Angular Velocity 1', 'Angular Velocity 2', 'Angular Velocity 3']

    lines = [ax.plot([], [])[0] for ax in axs.flat]
    for ax, label in zip(axs.flat, labels):
        ax.set_title(label)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Degrees' if 'Angle' in label else 'Degrees/s')

    last = time.time()
    while True:
        current = time.time()
        elapsed = current - last

        #angle = kf.step()
        gyroscope, accelerometer = get_imu_data()
        ahrs.update_no_magnetometer(gyroscope, accelerometer, elapsed)
        angle = ahrs.quaternion.to_euler()


        dof6 = imu.get_6DOF()  # Expected to return a list of 6 values
        data = [angle[0], angle[1], angle[2], dof6.x, dof6.y, dof6.z]

        if args.print:
            print(dict(zip(["Yaw", "Pitch", "Roll", "x", "y", "z"], data)))

        plotter(axs, lines, data, time.time())
        last = current

if __name__ == "__main__":
    main()
