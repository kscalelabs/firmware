"""Naive deployment script.

Run:
    python firmware/scripts/test_walking --load_model policy.pt
"""
import argparse
import math
import time
from collections import deque

import numpy as np
import torch

from firmware.imu.imu import IMUInterface
from firmware.scripts.robot_controller import Robot


class cmd:
    vx = 0.0
    vy = -0.3
    dyaw = 0.0
    
# TODO(Ved): Add the correct imports
robot = Robot("legs")
robot.zero_out()  # TODO: (Ved - zero out the legs)
imu = IMUInterface(1) # Bus = 1


def pd_control(target_q, q, kp, dq, kd, default):
    """Calculates torques from position commands"""
    return kp * (target_q + default - q) - kd * dq


def run(policy):
    """
    Run the simple policy
    
    Args:
        policy: The policy used for controlling the simulation.

    Returns:
        None
    """

    count_level = 0
    action_scale = 0.25
    dt = 0.001
    num_actions = 12
    clip_observations = 18.
    clip_actions = 18.
    num_single_obs = 47
    frame_stack = 15
    num_observations = frame_stack * num_single_obs
    ang_vel = 1.0
    dof_pos = 1.0
    lin_vel = 2.0
    dof_vel = 0.05
    phase = 0.64
    default = np.array([ 1.17,  1.03,  0.1 , -1.04, -0.27,  1.73,  0.38, -2.15, -1.6 , 2.94,  0.8 ,  1.72])
    kps = tau_limit= np.array([212.5, 127.5, 127.5, 127.5, 127.5, 127.5, 212.5, 127.5, 127.5, 127.5, 127.5, 127.5])
    kds = np.array([15, 10, 10, 10, 10, 10, 15, 10, 10, 10, 10, 10])

    target_q = np.zeros((num_actions), dtype=np.double)
    action = np.zeros((num_actions), dtype=np.double)

    hist_obs = deque()
    for _ in range(frame_stack):
        hist_obs.append(np.zeros([1, num_single_obs], dtype=np.double))

    target_frequency = 250  # Hz
    target_loop_time = 1.0 / target_frequency  # 4 ms

    while True: 
        loop_start_time = time.time()

        # TODO: Ved
        # q = robot.get_positions()
        # dq = robot.get_velocities()
        # imu_data = imu.get_data()

        # some constant values for the time being
        obs = np.zeros([1, num_single_obs], dtype=np.float32)
        q = default
        dq = np.zeros((num_actions), dtype=np.double)
        omega = np.array([0.18525067, -0.18715704, -0.02971003])
        eu_ang = np.array([-0.02666937,  0.00341947, -0.008603  ])
        eu_ang[eu_ang > math.pi] -= 2 * math.pi
    
        # TODO: Allen, Pfb30 - figure out phase dt logic
        obs[0, 0] = math.sin(2 * math.pi * count_level * dt / phase)
        obs[0, 1] = math.cos(2 * math.pi * count_level * dt / phase)
        obs[0, 2] = cmd.vx * lin_vel
        obs[0, 3] = cmd.vy * lin_vel
        obs[0, 4] = cmd.dyaw * ang_vel
        obs[0, 5:17] = (q - default) * dof_pos
        obs[0, 17:29] = dq * dof_vel
        obs[0, 29:41] = action
        obs[0, 41:44] = omega
        obs[0, 44:47] = eu_ang

        obs = np.clip(obs, -clip_observations, clip_observations)

        hist_obs.append(obs)
        hist_obs.popleft()

        policy_input = np.zeros([1, num_observations], dtype=np.float32)
        for i in range(frame_stack):
            policy_input[0, i * num_single_obs : (i + 1) * num_single_obs] = hist_obs[i][0, :]
        action[:] = policy(torch.tensor(policy_input))[0].detach().numpy()

        action = np.clip(action, -clip_actions, clip_actions)

        target_q = action * action_scale

        # Generate PD control
        tau = pd_control(target_q, q, kps, dq, kds, default)

        tau = np.clip(tau, -tau_limit, tau_limit)  # Clamp torques

        count_level += 1

        # Calculate how long to sleep
        loop_end_time = time.time()
        loop_duration = loop_end_time - loop_start_time
        sleep_time = max(0, target_loop_time - loop_duration)

        # Sleep for the remaining time to achieve 250 Hz
        time.sleep(sleep_time)
        print("Sleep time: ", sleep_time)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Deployment script.")
    parser.add_argument("--load_model", type=str, required=True, help="Run to load from.")
    args = parser.parse_args()

    policy = torch.jit.load(args.load_model)
    run(policy)
