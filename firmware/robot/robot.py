"""Simple robot controller module.

Todo:
1. add config and init
2. full body
3. tests
"""

import yaml
import logging
import math
import time
from typing import Dict, List

import can

from firmware.bionic_motors.model import Arm, Body, Leg
from firmware.bionic_motors.motors import BionicMotor, CANInterface
from firmware.bionic_motors.utils import NORMAL_STRENGTH

def rad_to_deg(rad: float) -> float:
    return rad / math.pi * 180

class Robot:
    def __init__(self, config_path: str = "config.yaml") -> None:
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)['robot']

        self.setup = self.config['setup']
        self.delta_change = self.config['delta_change']
        self.can_bus = self._initialize_can_bus()
        self.body = self._initialize_body()
        self.motor_config = self._initialize_motor_config()
        self.prev_positions: dict = {part: [] for part in self.motor_config}

    def _initialize_can_bus(self) -> CANInterface:
        write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
        buffer_reader = can.BufferedReader()
        notifier = can.Notifier(write_bus, [buffer_reader])
        return CANInterface(write_bus, buffer_reader, notifier)

    def _initialize_body(self) -> Body:
        body_parts: dict = {}
        for part, config in self.config['body_parts'].items():
            if part.endswith('_arm'):
                body_parts[part] = self._create_arm(part.split('_')[0], config['start_id'])
            elif part.endswith('_leg'):
                body_parts[part] = self._create_leg(part.split('_')[0], config['start_id'])

        return Body(**body_parts)

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
            pelvis=BionicMotor(start_id, NORMAL_STRENGTH.LEG_PARAMS, self.can_bus),
            hip=BionicMotor(start_id + 1, NORMAL_STRENGTH.LEG_PARAMS, self.can_bus),
            thigh=BionicMotor(start_id + 2, NORMAL_STRENGTH.LEG_PARAMS, self.can_bus),
            knee=BionicMotor(start_id + 3, NORMAL_STRENGTH.LEG_PARAMS, self.can_bus),
            ankle=BionicMotor(start_id + 4, NORMAL_STRENGTH.LEG_PARAMS, self.can_bus),
            foot=BionicMotor(start_id + 5, NORMAL_STRENGTH.LEG_PARAMS, self.can_bus),
        )

    def _initialize_motor_config(self) -> Dict[str, Dict]:
        config = {}
        motor_config = self.config['motor_config']
        
        for part, part_config in self.config['body_parts'].items():
            if hasattr(self.body, part):
                part_type = 'arm' if 'arm' in part else 'leg'
                config[part] = {
                    "motors": getattr(self.body, part).motors,
                    "signs": motor_config[part_type]['signs'],
                    "increments": motor_config[part_type]['increments'],
                    "maximum_values": motor_config[part_type]['maximum_values'],
                    "offsets": motor_config[part_type]['offsets'],
                }
        return config
