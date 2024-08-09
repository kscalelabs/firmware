"""Model for robot that defines the robot's body parts."""


from dataclasses import dataclass
from typing import Optional

from firmware.robot_utils.motor_utils import MotorInterface

# TODO: Head


@dataclass
class Arm:
    rotator_cuff: MotorInterface
    shoulder: MotorInterface
    bicep: MotorInterface
    elbow: MotorInterface
    wrist: MotorInterface
    gripper: MotorInterface

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
    def motors(self) -> list[MotorInterface]:
        return [
            self.rotator_cuff,
            self.shoulder,
            self.bicep,
            self.elbow,
            self.wrist,
            self.gripper,
        ]


@dataclass
class Leg:
    pelvis: MotorInterface
    hip: MotorInterface
    thigh: MotorInterface
    knee: MotorInterface
    ankle: MotorInterface
    foot: MotorInterface

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

    @property
    def motors(self) -> list[MotorInterface]:
        return [
            self.pelvis,
            self.hip,
            self.thigh,
            self.knee,
            self.ankle,
            self.foot,
        ]


@dataclass
class Body:
    # head: Head
    left_arm: Optional[Arm] = None
    right_arm: Optional[Arm] = None
    # waist: BionicMotor
    left_leg: Optional[Leg] = None
    right_leg: Optional[Leg] = None

    @property
    def motor_ids(self) -> list[int]:
        ids: list[int] = []
        # if self.head:
        #     ids += self.head.motor_ids
        if self.left_arm:
            ids += self.left_arm.motor_ids
        if self.right_arm:
            ids += self.right_arm.motor_ids
        # ids.append(self.waist.motor_id)
        if self.left_leg:
            ids += self.left_leg.motor_ids
        if self.right_leg:
            ids += self.right_leg.motor_ids
        return ids
