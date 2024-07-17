"""Madgwick AHRS algorithm.

This script demonstrates the initialization and usage of the Madgwick AHRS algorithm for orientation estimation.
The Madgwick algorithm is part of the firmware package and is used here without any real sensor data, i.e.,
the script simulates the algorithm's behavior with zero input data to stabilize at a neutral orientation.

Attributes:
    dt (float): The timestep in seconds for the algorithm's update rate.
    ahrs (Madgwick): An instance of the Madgwick class initialized with a specific beta parameter.
    roll, pitch, yaw (int): Variables to hold the orientation angles, initially set to zero.

Usage:
    - The script initializes the Madgwick filter with a beta value.
    - It then simulates the filter running for a thousand iterations with no sensor input.
    - It prints "Upright" to indicate the expected orientation when no external motion or forces are applied.
"""

from firmware.cpp.madgwick.madgwick import Madgwick, Vector  # type: ignore[import-not-found]

dt = 0.01

ahrs = Madgwick(beta=0.075)

roll = 0
pitch = 0
yaw = 0

print("Upright")
for i in range(1000):
    ahrs.update(Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0))
