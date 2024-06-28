"""Defines the motor model but with bionic motors instead."""

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
    left_arm: Arm
    # right_arm: Arm
    # waist: BionicMotor
    # left_leg: Leg
    # right_leg: Leg

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
