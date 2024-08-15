"""Robot class for controlling a robot that is motor agnostic."""

import math
import time
from typing import Any, Dict, List, Union

import can
import yaml

import firmware.robstride_motors.client as robstride
from firmware.bionic_motors.motors import CANInterface
from firmware.robot.model import Arm, Body, Leg
from firmware.robot_utils.motor_factory import MotorFactory
from firmware.robot_utils.motor_utils import MotorInterface


def rad_to_deg(rad: float) -> float:
    return rad / math.pi * 180

def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180

class Robot:
    def __init__(self, config_path: str = "config.yaml", setup: str = "full_body") -> None:
        with open(config_path, "r") as config_file:
            config = yaml.safe_load(config_file)
            self.config = next(robot for robot in config["robots"] if robot["setup"] == setup)
        print("Loaded config")
        self.setup = setup
        self.delta_change = self.config["delta_change"]

        self.communication_interfaces = self._initialize_communication_interfaces()
        print("Initialized communication interfaces")
        self.body = self._initialize_body()
        self.motor_config = self._initialize_motor_config()
        self.prev_positions: dict = {part: [] for part in self.motor_config}

    def _initialize_communication_interfaces(self) -> Dict[str, Any]:
        interfaces = {}
        for part, config in self.config["body_parts"].items():
            canbus_id = config.get("canbus_id", 0)
            if self.config["motor_type"] == "bionic":
                interfaces[part] = self._initialize_can_interface(canbus_id)
            elif self.config["motor_type"] == "robstride":
                interfaces[part] = robstride.Client(can.interface.Bus(channel=f"can{canbus_id}", bustype="socketcan"))
            else:
                raise ValueError(f"Unsupported motor type: {self.config['motor_type']}")
        return interfaces

    def _initialize_can_interface(self, canbus_id: int) -> CANInterface:
        write_bus = can.interface.Bus(channel=f"can{canbus_id}", bustype="socketcan")
        buffer_reader = can.BufferedReader()
        notifier = can.Notifier(write_bus, [buffer_reader])
        return CANInterface(write_bus, buffer_reader, notifier)

    def _create_motor(self, part: str, motor_id: int, control_params: Any) -> MotorInterface:
        return MotorFactory.create_motor(
            self.config["motor_type"],
            motor_id,
            control_params,
            self.communication_interfaces[part]
        )

    def _initialize_body(self) -> Body:
        body_parts: dict = {}
        for part, config in self.config["body_parts"].items():
            if part.endswith("_arm"):
                body_parts[part] = self._create_arm(part, config["start_id"], config["dof"])
            elif part.endswith("_leg"):
                body_parts[part] = self._create_leg(part, config["start_id"], config["dof"])
        return Body(**body_parts)

    def _create_arm(self, part: str, start_id: int, dof: int) -> Arm:
        motors = []
        for i in range(dof):
            control_params = self._get_motor_params(start_id + i)
            motors.append(self._create_motor(part, start_id + i, control_params))
        return Arm(motors=motors)

    def _create_leg(self, part: str, start_id: int, dof: int) -> Leg:
        motors = []
        for i in range(dof):
            control_params = self._get_motor_params(start_id + i)
            motors.append(self._create_motor(part, start_id + i, control_params))
        return Leg(motors=motors)

    def _get_motor_params(self, motor_id: int) -> Dict[str, Any]:
        default_params = next(param for param in self.config["params"] if param["motor_id"] == "default")
        specific_params = next((param for param in self.config["params"] if param["motor_id"] == motor_id), None)

        if specific_params:
            return {**default_params, **specific_params}
        return default_params

    def _initialize_motor_config(self) -> Dict[str, Dict[str, Any]]:
        motor_config = {}
        for part, part_config in self.config["body_parts"].items():
            if hasattr(self.body, part):
                part_type = "arm" if "arm" in part else "leg"
                dof = part_config["dof"]

                signs = self.config["motor_config"][part_type]["signs"][:dof]
                if part.startswith("left"):
                    signs = [-s for s in signs]

                motor_config[part] = {
                    "motors": getattr(self.body, part).motors,
                    "signs": signs,
                    "increments": self.config["motor_config"][part_type]["increments"][:dof],
                    "maximum_values": self.config["motor_config"][part_type]["maximum_values"][:dof],
                    "offsets": self.config["motor_config"][part_type]["offsets"][:dof],
                }
        return motor_config

    @staticmethod
    def filter_motor_values(values: List[float], max_val: List[float]) -> List[float]:
        for idx, (val, maxes) in enumerate(zip(values, max_val)):
            if abs(val) > abs(maxes):
                values[idx] = val // abs(val) * maxes

        return values

    """
    Test all motors by setting them to a range of values from low to high

    Args:
        low: The lower bound of the range
        high: The upper bound of the range
        radians: Whether the values should be interpreted as radians
    """
    def test_motors(self, low: int = 0, high: int = 10, radians: bool =False) -> None:
        for part, config in self.motor_config.items():
            for motor, sign in zip(config["motors"], config["signs"]):
                for val in range(low, high + 1):
                    if not radians:
                        set_val = deg_to_rad(float(val))
                    else:
                        set_val = val
                    motor.set_position(sign * set_val)
                    time.sleep(0.1)

    def zero_out(self) -> None:
        for part, part_config in self.motor_config.items():
            for motor in part_config["motors"]:
                motor.set_zero_position()

    def disable_motors(self) -> None:
        if self.config["motor_type"] == "robstride":
            for part, part_config in self.motor_config.items():
                for motor in part_config["motors"]:
                    motor.disable()

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
