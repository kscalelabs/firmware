"""Defines the motor model."""

from dataclasses import dataclass



@dataclass
class Head:
    up_down: int
    left_right: int

    @property
    def motor_ids(self) -> list[int]:
        return [self.up_down, self.left_right]


@dataclass
class Shoulder:
    front_back: int
    in_out: int
    rotate: int

    @property
    def motor_ids(self) -> list[int]:
        return [self.front_back, self.in_out, self.rotate]


@dataclass
class Forearm:
    rotate: int
    in_out: int

    @property
    def motor_ids(self) -> list[int]:
        return [self.rotate, self.in_out]


@dataclass
class Arm:
    shoulder: Shoulder
    elbow: int
    forearm: Forearm

    @property
    def motor_ids(self) -> list[int]:
        return self.shoulder.motor_ids + [self.elbow] + self.forearm.motor_ids


@dataclass
class Hip:
    in_out: int
    rotate: int

    @property
    def motor_ids(self) -> list[int]:
        return [self.in_out, self.rotate]


@dataclass
class Thigh:
    upper: int
    lower: int

    @property
    def motor_ids(self) -> list[int]:
        return [self.upper, self.lower]


@dataclass
class Calf:
    upper: int
    lower: int

    @property
    def motor_ids(self) -> list[int]:
        return [self.upper, self.lower]


@dataclass
class Leg:
    hip: Hip
    thigh: Thigh
    calf: Calf

    @property
    def motor_ids(self) -> list[int]:
        return self.hip.motor_ids + self.thigh.motor_ids + self.calf.motor_ids


@dataclass
class Body:
    head: Head
    left_arm: Arm
    right_arm: Arm
    waist: int
    left_leg: Leg
    right_leg: Leg

    @property
    def motor_ids(self) -> list[int]:
        return self.head.motor_ids + self.left_arm.motor_ids + [self.waist] + self.left_leg.motor_ids + self.right_leg.motor_ids


Model = Body(
    head=Head(
        up_down=15,
        left_right=14,
    ),
    left_arm=Arm(
        shoulder=Shoulder(
            front_back=8,
            in_out=9,
            rotate=10,
        ),
        elbow=11,
        forearm=Forearm(
            rotate=12,
            in_out=13,
        ),
    ),
    right_arm=Arm(
        shoulder=Shoulder(
            front_back=2,
            in_out=3,
            rotate=4,
        ),
        elbow=5,
        forearm=Forearm(
            rotate=6,
            in_out=7,
        ),
    ),
    waist=1,
    left_leg=Leg(
        hip=Hip(
            in_out=22,
            rotate=23,
        ),
        thigh=Thigh(
            upper=24,
            lower=25,
        ),
        calf=Calf(
            upper=26,
            lower=27,
        ),
    ),
    right_leg=Leg(
        hip=Hip(
            in_out=16,
            rotate=17,
        ),
        thigh=Thigh(
            upper=18,
            lower=19,
        ),
        calf=Calf(
            upper=20,
            lower=21,
        ),
    ),
)
