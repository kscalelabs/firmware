"""Model for robot that defines the robot's body parts."""

from dataclasses import dataclass, field
from typing import List, Optional

from firmware.motor_utils.motor_utils import MotorInterface

# TODO: Head


@dataclass
class BodyPart:
    motors: List[MotorInterface] = field(default_factory=list)

    @property
    def motor_ids(self) -> List[int]:
        return [motor.motor_id for motor in self.motors]


@dataclass
class Arm(BodyPart):
    pass


@dataclass
class Leg(BodyPart):
    pass


@dataclass
class Body:
    left_arm: Optional[Arm] = None
    right_arm: Optional[Arm] = None
    left_leg: Optional[Leg] = None
    right_leg: Optional[Leg] = None

    @property
    def motor_ids(self) -> List[int]:
        ids: List[int] = []
        for part in [self.left_arm, self.right_arm, self.left_leg, self.right_leg]:
            if part:
                ids.extend(part.motor_ids)
        return ids

    @property
    def all_motors(self) -> List[MotorInterface]:
        motors: List[MotorInterface] = []
        for part in [self.left_arm, self.right_arm, self.left_leg, self.right_leg]:
            if part:
                motors.extend(part.motors)
        return motors
