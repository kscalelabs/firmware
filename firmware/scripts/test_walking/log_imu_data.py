"""Script to log imu data from robot. Writes to out.csv file."""

import math
import time

from firmware.imu.imu import IMUInterface


def main() -> None:
    """Log imu data from robot."""
    imu = IMUInterface(1)
    imu_dt = time.time()
    imu_readings = []
    imu.step(0.01)
    for _ in range(10):
        imu.calibrate_yaw()
        time.sleep(0.1)
    for _ in range(2000):
        imu_data = imu.step(time.time() - imu_dt)
        imu_dt = time.time()
        eu_ang = imu_data[0]
        eu_ang_rate = imu_data[1]
        eu_ang[eu_ang > math.pi] -= 2 * math.pi
        reading = [*eu_ang], eu_ang_rate.x, eu_ang_rate.y, eu_ang_rate.z
        imu_readings.append(reading)
        print(eu_ang)
        time.sleep(0.01)
    with open("out.csv", "w") as f:
        for t, reading in enumerate(imu_readings):
            f.write(f"{t},{','.join(map(str, reading))}\n")


if __name__ == "__main__":
    main()
