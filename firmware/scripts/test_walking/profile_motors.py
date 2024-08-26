"""Script to profile Robstride motor parameters and match to sim."""
import math
import time
from dataclasses import dataclass

import can
import draccus

import firmware.robstride_motors.client as robstride
from firmware.motor_utils.motor_utils import CalibrationMode
from firmware.robstride_motors.motors import RobstrideMotor, RobstrideParams
from firmware.scripts.test_walking.run import SIM_TO_ROBOT_JOINTS

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

SIM_DEFAULT_STANDING = {
    "left_hip_pitch": -0.28,
    "left_hip_roll": 1.5,
    "left_hip_yaw": 1.62,
    "left_knee_pitch": 1,
    "left_ankle_pitch": -2.2,
    "right_hip_pitch": 3.55,
    "right_hip_roll": 3.18,
    "right_hip_yaw": 1.001,
    "right_knee_pitch": -1,
    "right_ankle_pitch": 0.42,
}

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
    offset: float = 0.0 # position corresponding to motor high level
    sign: int = -1 # direction of motor movement
    calibration_sign: int = 1 # direction of calibration
    joint: str = "left_knee_pitch"
    periods: int = 5 # number of periods to run

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

    motor.calibrate(10, CalibrationMode.FORWARD, cfg.calibration_sign)

    sin_freq = (2 * math.pi / cfg.test_duration) / cfg.gait
    positions = []

    # Run sinusoidal position test
    print("Running sinusoidal position test")
    t0 = time.time()
    while time.time() - t0 < cfg.periods * cfg.test_duration:
        t = time.time()
        motor.set_position(math.sin(sin_freq * (time.time() - t0)) + (SIM_DEFAULT_STANDING[cfg.joint] - cfg.offset) * cfg.sign)
        print(f"Motor at {motor.get_position()}")
        positions.append(motor.get_position() * cfg.sign)

        time.sleep(max(0, cfg.dt - (time.time() - t)))

    print("Sinusoidal position test complete")
    time.sleep(1.0)
    motor.disable()

    # Write data to file
    with open("robstride_position_test.csv", "w") as f:
        f.write("time,position,adjusted\n")
        for i, pos in enumerate(positions):
            f.write(f"{i},{pos},{pos+cfg.offset}\n")

if __name__ == "__main__":
    main()
