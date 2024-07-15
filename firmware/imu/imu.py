import numpy as np
import time
import imufusion
from firmware.cpp.imu.imu import IMU

class IMUInterface:
    MAG_TO_MCRO_TSLA = 0.0001 * 1000000

    def __init__(self, bus):
        self.imu = IMU(bus)
        self.ahrs = imufusion.Ahrs()
        self.offset = imufusion.Offset(3300)
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,
            gain=0.6,
            gyroscope_range=2000,
            acceleration_rejection=90,
            magnetic_rejection=90,
            recovery_trigger_period=0
        )

    def step(self, dt):
        gyroscope, accelerometer, magnetometer = self.get_imu_data()
        self.ahrs.update(gyroscope, accelerometer, magnetometer, dt)

        return [self.ahrs.quaternion.to_euler(), self.imu.gyr_rate()]

    def get_imu(self):
        return self.imu
