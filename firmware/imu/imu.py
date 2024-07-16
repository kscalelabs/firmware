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

"""

from typing import Any

import imufusion
import numpy as np

from firmware.cpp.imu.imu import IMU


class IMUInterface:
    MAG_TO_MCRO_TSLA = 0.0001 * 1000000
    GYRO_YAW_THRESHOLD = 3

    def __init__(self, bus: int) -> None:
        self.imu: IMU = IMU(bus)
        self.ahrs: imufusion.Ahrs = imufusion.Ahrs()
        self.offset: imufusion.Offset = imufusion.Offset(3300)
        self.quatOffset: imufusion.Quaternion = imufusion.Quaternion(0, 0, 0, 0)
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
            self.quatOffset = self.ahrs.quaternion

    def step(self, dt: float) -> list[Any]:
        gyroscope, accelerometer, magnetometer = self.get_imu_data()
        self.ahrs.update(gyroscope, accelerometer, magnetometer, dt)
        self.state = [self.ahrs.quaternion.to_euler(), self.imu.gyr_rate()]
        return [self.state[0] - self.quatOffset.to_euler(), self.state[1]]

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
