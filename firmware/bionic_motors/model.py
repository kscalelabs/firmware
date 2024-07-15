"""Defines the motor model but with bionic motors instead."""

import math
import time
from dataclasses import dataclass

from firmware.bionic_motors.motors import BionicMotor

# TODO: Head


@dataclass
class Arm:
    rotator_cuff: BionicMotor
    shoulder: BionicMotor
    bicep: BionicMotor
    elbow: BionicMotor
    wrist: BionicMotor
    gripper: BionicMotor

    @property
    def motor_ids(self) -> list[int]:
        return [
            self.rotator_cuff.motor_id,
            self.shoulder.motor_id,
            self.bicep.motor_id,
            self.elbow.motor_id,
            self.wrist.motor_id,
            self.gripper.motor_id,
        ]

    @property
    def motors(self) -> list[BionicMotor]:
        return [
            self.rotator_cuff,
            self.shoulder,
            self.bicep,
            self.elbow,
            self.wrist,
            self.gripper,
        ]

    def set_position_incremental(
        self, increments: list, target_vals: list, thresholds: list
    ) -> None:  # ensure that thresholds is kept at a non-zero value
        state = [False for _ in range(len(self.motors))]
        position = [part.position for part in self.motors]
        while not all(state):
            for idx, part in enumerate(self.motors):
                sign = math.copysign(1, target_vals[idx] - part.position)
                if abs(part.position - target_vals[idx]) < thresholds[idx]:
                    state[idx] = True
                    position[idx] += 0
                else:
                    position[idx] += sign * increments[idx]

                part.update_position(0.001)
                part.set_position(int(position[idx]), 0, 0)

    def hold_position(self, position: list, timeout: float = 2.0) -> None:
        cur_time = time.time()
        print("These are the current pos", position)
        while time.time() - cur_time < timeout:
            for idx, part in enumerate(self.motors):
                part.set_position(int(position[idx]), 0, 0)
                part.update_position(0.001)
                if idx == 0:
                    print(part.position)


class Leg:
    pelvis: BionicMotor
    hip: BionicMotor
    thigh: BionicMotor
    knee: BionicMotor
    ankle: BionicMotor
    foot: BionicMotor

    @property
    def motor_ids(self) -> list[int]:
        return [
            self.pelvis.motor_id,
            self.hip.motor_id,
            self.thigh.motor_id,
            self.knee.motor_id,
            self.ankle.motor_id,
            self.foot.motor_id,
        ]


@dataclass
class Body:
    # head: Head
    #left_arm: Arm
    # right_arm: Arm
    # waist: BionicMotor
    left_leg: Leg
    right_leg: Leg

    @property
    def motor_ids(self) -> list[int]:
        return (
            # self.head.motor_ids
            self.left_arm.motor_ids
            + self.right_arm.motor_ids
            + [self.waist.motor_id]
            + self.left_leg.motor_ids
            + self.right_leg.motor_ids
        )
