#!/usr/bin/env python
"""Simple script to log the IMU values."""

import argparse
import matplotlib.pyplot as plt
import numpy as np
import time
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler
from firmware.cpp.imu.imu import IMU
import math

MAG_TO_MCRO_TSLA = 0.0001 * 1000000
MAG_TO_NANO_TSLA = 0.0001 * 1000000000
DEG_TO_RAD = 3.14159268/180
MAX_WINDOW = 100 # data points

def radToDegrees(angles):
    return [math.degrees(angle) for angle in angles]

def main() -> None:
    parser = argparse.ArgumentParser(description="Log the IMU values.")
    parser.add_argument("--dt", type=float, default=0.02, help="The time step between measurements")
    parser.add_argument("--bus", type=int, default=1, help="The I2C bus number")
    parser.add_argument("--raw", default=False, action="store_true", help="Print raw values")
    parser.add_argument("--delay", type=float, default=0.2, help="How often to print readings")
    parser.add_argument("--plot", default=False, action="store_true", help="Display a live plot of the readings")
    parser.add_argument("--no-print", dest="print", default=True, action="store_false", help="Print out readings")
    parser.add_argument("--quat", default=False, action="store_true", help="Print quaternion representation")
    args = parser.parse_args()

    global imu, madgwick, offset, start, q

    start = time.time()

    imu = IMU(args.bus)

    madgwick = Madgwick()
    q = np.array([0.7071, 0.0, 0.7071, 0.0])

    if args.plot:
        live_plot(args)
    elif args.print:
        console(args)

def read_quat(quat):
    return f"({quat.w}, {quat.x}, {quat.y}, {quat.z})"

def get_imu_data():
    gyro = imu.gyr_rate()

    acc = imu.acc_g()

    mag = imu.read_mag() 

    return [np.array([gyro.x*DEG_TO_RAD, gyro.y*DEG_TO_RAD, gyro.z*DEG_TO_RAD]),
            np.array([acc.x, acc.y, acc.z]),
            np.array([mag.x*MAG_TO_NANO_TSLA, mag.y*MAG_TO_NANO_TSLA,mag.z*MAG_TO_NANO_TSLA])]

        
def console(args):
    last = time.time()
    printTime = 0
    while True:
        current = time.time()
        elapsed = current - last

        gyroscope, accelerometer, magnetometer = get_imu_data()
        q = madgwick.updateMARG(q, gyroscope, accelerometer, magnetometer, elapsed)
        if(printTime > 0.5):
            if args.quat:
                print(q)

            else:
                print(radToDegrees(q2euler(q)))
            printTime = 0
        last = current
        printTime += elapsed

def live_plot(args):
    def plotter(axs, lines, new_data, time):
        for ax, line, data in zip(axs.flat, lines, new_data):
            x_data, y_data = line.get_xdata(), line.get_ydata()
            
            x_data = np.append(x_data, time)
            y_data = np.append(y_data, data)

            if len(x_data) > MAX_WINDOW:
                x_data = x_data[1:]
                y_data = y_data[1:]
            line.set_xdata(x_data)
            line.set_ydata(y_data)
            ax.relim()
            ax.set_ylim(-190,190)
            ax.autoscale_view(True, True, True)
        plt.pause(0.001)


    # Setup live plotting
    fig, axs = plt.subplots(2, 3)  # 3 angles and 3 angular velocities
    plt.ion()
    fig.show()
    fig.canvas.draw()
    labels = ['Pitch', 'Roll', 'Yaw', 'Angular Velocity 1', 'Angular Velocity 2', 'Angular Velocity 3']

    lines = [ax.plot([], [])[0] for ax in axs.flat]
    for ax, label in zip(axs.flat, labels):
        ax.set_title(label)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Degrees' if 'Angle' in label else 'Degrees/s')

    last = time.time()
    while True:
        current = time.time()
        elapsed = current - last

        gyroscope, accelerometer, magnetometer = get_imu_data()

        madgwick.Dt = elapsed
        angle = madgwick.updateMARG(q, gyroscope, accelerometer, magnetometer)

        dof6 = imu.get_6DOF()  # Expected to return a list of 6 values
        data = [*radToDegrees(q2euler(q)), dof6.x, dof6.y, dof6.z] #x=pitch, y=roll, z=yaw

        if args.print and not args.quat:
            print(dict(zip(["Yaw", "Pitch", "Roll", "x", "y", "z"], data)))
        elif args.quat:
            print(angle)
        plotter(axs, lines, data, current - start)
        last = current

if __name__ == "__main__":
    main()
