from firmware.cpp.madgwick.madgwick import Madgwick, Vector, Quaternion, Euler  # type: ignore


dt = 0.01

ahrs = Madgwick(beta=0.075)

roll = 0
pitch = 0
yaw = 0

print("Upright")
for i in range(1000):
    ahrs.update(Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0))
