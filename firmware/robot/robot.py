"""Robot class for controlling a robot that is motor agnostic."""

import math
from typing import Any, Dict, List, Union

import can
import yaml

import firmware.robstride_motors.client as robstride
from firmware.bionic_motors.motors import CANInterface
from firmware.robot.model import Arm, Body, Leg
from firmware.robot_utils.motor_utils import MotorFactory, MotorInterface


def rad_to_deg(rad: float) -> float:
    return rad / math.pi * 180

class Robot:
    def __init__(self, config_path: str = "config.yaml", setup: str = "full_body") -> None:
        with open(config_path, "r") as config_file:
            config = yaml.safe_load(config_file)
            self.config = next(robot for robot in config["robots"] if robot["setup"] == setup)

        self.setup = setup
        self.delta_change = self.config["delta_change"]
        self.communication_interface = self._initialize_communication_interface()
        self.body = self._initialize_body()
        self.motor_config = self._initialize_motor_config()
        self.prev_positions: dict = {part: [] for part in self.motor_config}

    def _initialize_communication_interface(self) -> Any:
        # Initialize the appropriate communication interface based on the config
        if self.config["motor_type"] == "bionic":
            return self._initialize_can_bus()
        elif self.config["motor_type"] == "robstride":
            return robstride.Client(self._initialize_can_bus().bus)
        else:
            raise ValueError(f"Unsupported motor type: {self.config['motor_type']}")

    def _initialize_can_bus(self) -> CANInterface:
        write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
        buffer_reader = can.BufferedReader()
        notifier = can.Notifier(write_bus, [buffer_reader])
        return CANInterface(write_bus, buffer_reader, notifier)

    def _create_motor(self, motor_id: int, control_params: Any) -> MotorInterface:
        return MotorFactory.create_motor(
            self.config["motor_type"],
            motor_id,
            control_params,
            self.communication_interface
        )

    def _initialize_body(self) -> Body:
        body_parts: dict = {}
        for part, config in self.config["body_parts"].items():
            if part.endswith("_arm"):
                body_parts[part] = self._create_arm(part.split("_")[0], config["start_id"])
            elif part.endswith("_leg"):
                body_parts[part] = self._create_leg(part.split("_")[0], config["start_id"])

        return Body(**body_parts)

    def _create_arm(self, side: str, start_id: int) -> Arm:
        motors = []
        for i in range(6):
            control_params = self.config["params"].get(i, self.config["params"]["default"])
            motors.append(self._create_motor(start_id + i, control_params))
        return Arm(
            rotator_cuff=motors[0],
            shoulder=motors[1],
            bicep=motors[2],
            elbow=motors[3],
            wrist=motors[4],
            gripper=motors[5],
        )

    def _create_leg(self, side: str, start_id: int) -> Leg:
        motors = []
        for i in range(6):
            control_params = self.config["params"].get(i, self.config["params"]["default"])
            motors.append(self._create_motor(start_id + i, control_params))
        return Leg(
            pelvis=motors[0],
            hip=motors[1],
            thigh=motors[2],
            knee=motors[3],
            ankle=motors[4],
            foot=motors[5],
        )

    def _initialize_motor_config(self) -> Dict[str, Dict[str, Any]]:
        motor_config = {}
        motor_config_params = self.config["motor_config"]

        for part, part_config in self.config["body_parts"].items():
            if hasattr(self.body, part):
                part_type = "arm" if "arm" in part else "leg"
                motor_config[part] = {
                    "motors": getattr(self.body, part).motors,
                    "signs": motor_config_params[part_type]["signs"],
                    "increments": motor_config_params[part_type]["increments"],
                    "maximum_values": motor_config_params[part_type]["maximum_values"],
                    "offsets": motor_config_params[part_type]["offsets"],
                }
        return motor_config

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

    def set_position(
        self,
        new_positions: Dict[str, List[float]],
        offset: Union[Dict[str, List[float]], None] = None,
        radians: bool = False,
    ) -> None:
        for part, positions in new_positions.items():
            if part not in self.motor_config:
                raise ValueError(f"Part {part} not in motor config")

            config = self.motor_config[part]

            if offset:
                positions = [pos - off for pos, off in zip(positions, offset[part])]

            if radians:
                positions = [rad_to_deg(pos) for pos in positions]
            positions = [val - off for val, off in zip(positions, config["offsets"])]
            positions = self.filter_motor_values(positions, config["maximum_values"])

            for i, (old, new) in enumerate(zip(self.prev_positions[part], positions)):
                if abs(new - old) > self.delta_change:
                    positions[i] = old

            for motor, pos, sign in zip(config["motors"], positions, config["signs"]):
                motor.set_position(sign * int(pos))

    def get_motor_speeds(self) -> Dict[str, List[float]]:
        return {part: [motor.get_speed() for motor in config["motors"]] for part, config in self.motor_config.items()}

    def get_motor_positions(self) -> Dict[str, List[float]]:
        return {
            part: [motor.get_position() * sign for motor, sign in zip(config["motors"], config["signs"])]
            for part, config in self.motor_config.items()
        }
