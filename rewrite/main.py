import argparse
import time

import numpy as np

from can import MotorDriver
from imu import IMUReader
from lpf import apply_lowpass_filter
from policy import get_onnx_sessions
    

def runner(kinfer_path):
    init_session, step_session, metadata = get_onnx_sessions(kinfer_path)
    joint_order = metadata['joint_names']
    carry = init_session.run(None, {})[0]

    imu_reader = IMUReader()
    imu_reader.start()

    motor_driver = MotorDriver()

    lpf_carry = None
    lpf_cutoff_hz = 10.0

    while True:
        t = time.perf_counter()
        joint_angles, joint_angular_velocities = motor_driver.get_joint_angles_and_velocities(joint_order)
        t1 = time.perf_counter()
        projected_gravity, gyroscope, timestamp = imu_reader.get_projected_gravity_and_gyroscope()
        t2 = time.perf_counter()
        command = np.zeros(16, dtype=np.float32)

        action, carry = step_session.run(None, {
            "joint_angles": np.array(joint_angles, dtype=np.float32),
            "joint_angular_velocities": np.array(joint_angular_velocities, dtype=np.float32), # TODO faster to already have as array?
            "projected_gravity": np.array(projected_gravity, dtype=np.float32),
            "gyroscope": np.array(gyroscope, dtype=np.float32),
            "command": command,
            "carry": carry,
        })
        t3 = time.perf_counter()

        # Apply low-pass filter to the action before sending PD targets # TODO phase out move to policy
        action, lpf_carry = apply_lowpass_filter(action, lpf_carry, cutoff_hz=lpf_cutoff_hz)
        t4 = time.perf_counter()
        motor_driver.take_action(action, joint_order)
        t5 = time.perf_counter()

        dt = time.perf_counter() - t
        print(f"dt={dt*1000:.2f} ms, get joints={(t1-t)*1000:.2f} ms, get imu={(t2-t1)*1000:.2f} ms, .step()={(t3-t2)*1000:.2f} ms, lpf={(t4-t3)*1000:.2f} ms, take action={(t5-t4)*1000:.2f} ms")
        while time.perf_counter() - t < 0.020: # wait for 50 hz
            time.sleep(0.001)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("kinfer_path", type=str, help="Path to saved model file")
    args = parser.parse_args()
    runner(args.kinfer_path)


# TODO move lpf to policy - no signals are modified in the firmware