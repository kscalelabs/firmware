"""Script to profile Robstride motor parameters and match to sim."""
import math
import time
from dataclasses import dataclass

import can
import draccus

import firmware.robstride_motors.client as robstride
from firmware.robstride_motors.motors import RobstrideMotor, RobstrideParams

SOFT_PARAMS: RobstrideParams = RobstrideParams(
    limit_torque=10,
    cur_kp=0.2,
    cur_ki=0.005,
    cur_fit_gain=0.1,
    limit_spd=5,
    limit_cur=10,
    loc_kp=20,
    spd_kp=2,
    spd_ki=0.005,
    spd_filt_gain=0.1,
)

@dataclass
class ProfileConfig:
    """Configuration for profiling."""
    motor_id: int = 0
    control_params: RobstrideParams = RobstrideParams(
        limit_torque=10,
        cur_kp=0.2,
        cur_ki=0.005,
        cur_fit_gain=0.1,
        limit_spd=5,
        limit_cur=10,
        loc_kp=20,
        spd_kp=2,
        spd_ki=0.005,
        spd_filt_gain=0.1,
    )
    id: int = 0 # canbus id
    dt: float = 0.01
    test_duration: float = 10.0
    gait: float = 0.64

@draccus.wrap()
def main(cfg: ProfileConfig) -> None:
    """Profile Robstride motor parameters and match to sim."""
    client = robstride.Client(can.interface.Bus(channel=f"can{cfg.id}", bustype="socketcan"))
    motor = RobstrideMotor(cfg.motor_id, cfg.control_params, client)
    motor.set_control_params()
    motor.set_operation_mode(robstride.RunMode.Position)
    motor.enable()
    motor.set_zero_position()
    print(f"Motor {cfg.motor_id} initialized")

    sin_freq = (2 * math.pi / cfg.test_duration) / cfg.gait
    positions = []

    # Run sinusoidal position test
    print("Running sinusoidal position test")
    t0 = time.time()
    while time.time() - t0 < cfg.test_duration:
        t = time.time()
        motor.set_position(math.sin(sin_freq * (time.time() - t0)))
        print(f"Motor at {motor.get_position()}")
        positions.append(motor.get_position())

        time.sleep(max(0, cfg.dt - (time.time() - t)))

    print("Sinusoidal position test complete")
    time.sleep(1.0)
    motor.disable()

    # Write data to file
    with open("robstride_position_test.csv", "w") as f:
        f.write("time,position\n")
        for i, pos in enumerate(positions):
            f.write(f"{i},{pos}\n")

if __name__ == "__main__":
    main()
