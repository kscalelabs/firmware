"""Example usage.

from firmware.imu.imu import IMUInterface
import time

def main():
    imu = IMUInterface(1)

    last = time.time()
    while True:
        current = time.time()
        dt = current - last
        print(imu.step(dt))
        last = current


if __name__ == "__main__":
    main()

    TODO: Convert imufusion to local package and add math utils
"""

from typing import Any

import imufusion
import numpy as np

from firmware.cpp.imu.imu import IMU


class IMUInterface:
    MAG_TO_MCRO_TSLA = 0.0001 * 1000000
    GYRO_YAW_THRESHOLD = 3

    def quaternion_multiply(self, q1: imufusion.Quaternion, q2: imufusion.Quaternion) -> imufusion.Quaternion:
        w1, x1, y1, z1 = q1.wxyz
        w2, x2, y2, z2 = q2.wxyz
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return imufusion.Quaternion(np.array([w, x, y, z]))

    def quaternion_conjugate(self, q: imufusion.Quaternion) -> imufusion.Quaternion:
        w, x, y, z = q.wxyz
        return imufusion.Quaternion(np.array([w, -x, -y, -z]))

    def __init__(self, bus: int) -> None:
        self.imu: IMU = IMU(bus)
        self.ahrs: imufusion.Ahrs = imufusion.Ahrs()
        self.offset: imufusion.Offset = imufusion.Offset(3300)
        self.quatOffset: imufusion.Quaternion = imufusion.Quaternion(np.array([0, 0, 0, 0]))
        self.state: list[Any] = []
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,
            0.6,  # gain
            2000,  # gyroscope range
            90,  # acceleration rejection
            90,  # magnetic rejection
            0,  # recovery trigger period
        )

    def calibrate_yaw(self) -> None:
        if self.state[1].z < self.GYRO_YAW_THRESHOLD:
            self.quatOffset = self.quaternion_conjugate(self.ahrs.quaternion)

    def step(self, dt: float) -> list[Any]:
        gyroscope, accelerometer, magnetometer = self.get_imu_data()
        self.ahrs.update(gyroscope, accelerometer, magnetometer, dt)
        relative_quat = self.quaternion_multiply(self.quatOffset, self.ahrs.quaternion)
        self.state = [relative_quat.to_euler(), self.imu.gyr_rate()]
        return self.state

    def get_measurement(self) -> list[list[float]]:
        return [
            [self.state[0].roll, self.state[0].pitch, self.state[0].yaw],
            [self.state[1].x, self.state[1].y, self.state[1].z],
        ]

    def get_imu(self) -> IMU:
        return self.imu

    def get_imu_data(self) -> np.ndarray:
        gyro = self.imu.gyr_rate()
        gyro_lsit = np.array([gyro.x, gyro.y, gyro.z])

        acc = self.imu.acc_g()

        mag = self.imu.read_mag()
        mag_list = [mag.x, mag.y, mag.z]
        return np.array(
            [self.offset.update(gyro_lsit), [acc.x, acc.y, acc.z], [val * self.MAG_TO_MCRO_TSLA for val in mag_list]]
        )
