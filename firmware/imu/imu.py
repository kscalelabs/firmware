import numpy as np
import time
import imufusion
from firmware.cpp.imu.imu import IMU
'''
Example usage:
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

'''

class IMUInterface:
    MAG_TO_MCRO_TSLA = 0.0001 * 1000000

    def __init__(self, bus):
        self.imu = IMU(bus)
        self.ahrs = imufusion.Ahrs()
        self.offset = imufusion.Offset(3300)
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,
            0.6, # gain
            2000, # gyroscope range
            90, # acceleration rejection
            90, # magnetic rejection
            0 # recovery trigger period
        )

    def step(self, dt):
        gyroscope, accelerometer, magnetometer = self.get_imu_data()
        self.ahrs.update(gyroscope, accelerometer, magnetometer, dt)

        return [self.ahrs.quaternion.to_euler(), self.imu.gyr_rate()]

    def get_imu(self):
        return self.imu
    
    def get_imu_data(self):
        gyro = self.imu.gyr_rate()
        gyroList = np.array([gyro.x, gyro.y, gyro.z])

        acc = self.imu.acc_g()

        mag = self.imu.read_mag() 
        magList = [mag.x, mag.y, mag.z]
        return np.array([self.offset.update(gyroList),
                        [acc.x, acc.y, acc.z],
                        [val * self.MAG_TO_MCRO_TSLA for val in magList]])

