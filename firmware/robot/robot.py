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
    def __init__(self, config_path: str = "config.yaml", setup: str = "full_body") -> None:
        with open(config_path, 'r') as config_file:
            for robot in yaml.safe_load(config_file)['robots']:
                if robot['setup'] == setup:
                    self.config = robot
        self.setup = setup
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
    
    def test_motors(self):
        for part, part_config in self.motor_config.items():
            print(f"testing {part}")
            for motor, sign in zip(part_config['motors'], part_config['signs']):
                print(f"testing {motor} w/ sign {sign}")
                for i in range(0,600):
                    motor.set_position(sign*i/10.0, 0, 0)
                    time.sleep(0.001)
                time.sleep(0.5)
                for i in range(600,0):
                    motor.set_position(sign*i/10, 0, 0)
            time.sleep(1)


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
            
            # Check if the part is in the motor config
            if part not in self.motor_config:
                raise ValueError(f"Part {part} not in motor config")
            
            config = self.motor_config[part]

            if offset:
                for idx, (pos, off) in enumerate(zip(positions, offset[part])):
                    positions[idx] = pos - off

            # Process the positions
            positions = [rad_to_deg(pos) for pos in positions]
            positions = [val - off for val, off in zip(positions, config["offsets"])]
            positions = self.filter_motor_values(positions, config["maximum_values"])

            # Check if the change is within the delta change
            for i, (old, new) in enumerate(zip(self.prev_positions[part], positions)):
                if abs(new - old) > self.delta_change:
                    positions[i] = old

            # Set the positions
            for motor, pos, sign in zip(config["motors"], positions, config["signs"]):
                motor.set_position(sign * int(pos), 0, 0)

