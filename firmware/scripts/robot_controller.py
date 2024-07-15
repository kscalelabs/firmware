"""Simple robot controller module.

Todo:
1. add config and init
2. full body
3. tests
"""

import logging
import math
import time
from typing import Dict, List

import can

from firmware.bionic_motors.model import Arm, Body, Leg
from firmware.bionic_motors.motors import BionicMotor, CANInterface
from firmware.bionic_motors.utils import NORMAL_STRENGTH


def rad_to_deg(rad: float) -> float:
    return rad / (math.pi) * 180


class Robot:
    def __init__(self, setup: str = "left_arm") -> None:
        # TODO - more init setup
        self.setup = setup
        self.can_bus = self._initialize_can_bus()
        self.body = self._initialize_body()
        self.motor_config = self._initialize_motor_config()
        self.prev_positions = {}

        # todo setup new
        for part, part_config in self.motor_config.items():
            self.prev_positions[part] = []

        self.delta_change = 10

    def _initialize_can_bus(self) -> CANInterface:
        write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
        buffer_reader = can.BufferedReader()
        notifier = can.Notifier(write_bus, [buffer_reader])
        return CANInterface(write_bus, buffer_reader, notifier)

    def _initialize_body(self) -> Body:
        if self.setup == "full_body":
            return Body(
                left_arm=self._create_arm("left", start_id=1),
                right_arm=self._create_arm("right", start_id=7),
                left_leg=self._create_leg("left", start_id=13),
                right_leg=self._create_leg("right", start_id=19),
            )
        elif self.setup == "left_arm":
            return Body(left_arm=self._create_arm("left", start_id=1))
        else:
            raise ValueError(f"Unsupported setup: {self.setup}")

    def _create_arm(self, side: str, start_id: int) -> Arm:
        return Arm(
            rotator_cuff=BionicMotor(start_id, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
            shoulder=BionicMotor(start_id + 1, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
            bicep=BionicMotor(start_id + 2, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
            elbow=BionicMotor(start_id + 3, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
            wrist=BionicMotor(start_id + 4, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
            gripper=BionicMotor(start_id + 5, NORMAL_STRENGTH.GRIPPERS_PARAMS, self.can_bus),
        )

    def _create_leg(self, side: str, start_id: int) -> Leg:
        return Leg(
            hip=BionicMotor(start_id, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
            ankle=BionicMotor(start_id + 1, NORMAL_STRENGTH.ARM_PARAMS, self.can_bus),
        )

    def _initialize_motor_config(self) -> Dict[str, Dict]:
        config = {}
        # TODO update actual positions
        for part in ["left_arm", "right_arm", "left_leg", "right_leg"]:
            if hasattr(self.body, part):
                config[part] = {
                    "motors": getattr(self.body, part).motors,
                    "signs": [1, -1, 1, -1, 1, 1],
                    "increments": [4] * len(getattr(self.body, part).motors),
                    "maximum_values": [60, 60, 60, 60, 0, 10],
                    "offsets": [0] * len(getattr(self.body, part).motors),
                }
        return config

    @staticmethod
    def filter_motor_values(values: List[float], max_val: List[float]) -> List[float]:
        for idx, (val, maxes) in enumerate(zip(values, max_val)):
            if abs(val) > abs(maxes):
                values[idx] = val // abs(val) * maxes

        return values

    def zero_out(self) -> None:
        for part, part_config in self.motor_config.items():
            for motor in part_config["motors"]:
                motor.set_zero_position()
                time.sleep(0.001)
                motor.update_position(0.25)
                logging.info(f"Part {motor.motor_id} at {motor.position}")

    def set_position(self, new_positions: Dict[str, List[float]], offset: Dict[str, List[float]] = None) -> None:
        for part, positions in new_positions.items():
            config = self.motor_config[part]

            if offset:
                for idx, (pos, off) in enumerate(zip(positions, offset[part])):
                    positions[idx] = pos - off

            positions = [rad_to_deg(pos) for pos in positions]
            positions = [val - off for val, off in zip(positions, config["offsets"])]
            positions = self.filter_motor_values(positions, config["maximum_values"])

            for i, (old, new) in enumerate(zip(self.prev_positions[part], positions)):
                if abs(new - old) > self.delta_change:
                    positions[i] = old

            for motor, pos, sign in zip(config["motors"], positions, config["signs"]):
                motor.set_position(sign * int(pos), 0, 0)

            if "arm" in part:
                gripper_pos = self.calculate_gripper_position(positions[-1])
                config["motors"][-1].set_position(gripper_pos, 0, 0)

            logging.info(f"Setting {part} position to {positions}")
            self.prev_positions[part] = positions

    @staticmethod
    def calculate_gripper_position(val: float) -> float:
        gripper_pos = val * -90 / 0.04
        return max(min(gripper_pos, 0), -90)

    def get_motor_positions(self) -> Dict[str, List[float]]:
        return {part: [motor.position for motor in config["motors"]] for part, config in self.motor_config.items()}
