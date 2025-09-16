from enum import Enum
from math import pi
from dataclasses import dataclass



class RobstrideActuatorType(Enum):
    Robstride00 = 0
    Robstride01 = 1
    Robstride02 = 2
    Robstride03 = 3
    Robstride04 = 4

@dataclass
class ActuatorConfig:
    can_id: int
    name: str
    actuator_type: RobstrideActuatorType
    # joint_min: float
    # joint_max: float
    # joint_zero: float
    kp: float
    kd: float
    # can ranges:
    angle_can_min: float
    angle_can_max: float
    velocity_can_min: float
    velocity_can_max: float
    torque_can_min: float
    torque_can_max: float
    kp_can_min: float
    kp_can_max: float
    kd_can_min: float
    kd_can_max: float

    def can_to_physical_angle(self, can_value: float) -> float:
        proportion = (can_value - 0.0) / (65535.0 - 0.0)
        return self.angle_can_min + proportion * (self.angle_can_max - self.angle_can_min)

    def physical_to_can_angle(self, physical_value: float) -> float:
        proportion = (physical_value - self.angle_can_min) / (self.angle_can_max - self.angle_can_min)
        return 0.0 + proportion * (65535.0 - 0.0)

    def can_to_physical_velocity(self, can_value: float) -> float:
        proportion = (can_value - 0.0) / (65535.0 - 0.0)
        return self.velocity_can_min + proportion * (self.velocity_can_max - self.velocity_can_min)

    def physical_to_can_velocity(self, physical_value: float) -> float:
        proportion = (physical_value - self.velocity_can_min) / (self.velocity_can_max - self.velocity_can_min)
        return 0.0 + proportion * (65535.0 - 0.0)

    def can_to_physical_torque(self, can_value: float) -> float:
        proportion = (can_value - 0.0) / (65535.0 - 0.0)
        return self.torque_can_min + proportion * (self.torque_can_max - self.torque_can_min)

    def physical_to_can_torque(self, physical_value: float) -> float:
        proportion = (physical_value - self.torque_can_min) / (self.torque_can_max - self.torque_can_min)
        return 0.0 + proportion * (65535.0 - 0.0)

    def can_to_physical_kp(self, can_value: float) -> float:
        proportion = (can_value - 0.0) / (65535.0 - 0.0)
        return self.kp_can_min + proportion * (self.kp_can_max - self.kp_can_min)

    def physical_to_can_kp(self, physical_value: float) -> float:
        proportion = (physical_value - self.kp_can_min) / (self.kp_can_max - self.kp_can_min)
        return 0.0 + proportion * (65535.0 - 0.0)

    def can_to_physical_kd(self, can_value: float) -> float:
        proportion = (can_value - 0.0) / (65535.0 - 0.0)
        return self.kd_can_min + proportion * (self.kd_can_max - self.kd_can_min)

    def physical_to_can_kd(self, physical_value: float) -> float:
        proportion = (physical_value - self.kd_can_min) / (self.kd_can_max - self.kd_can_min)
        return 0.0 + proportion * (65535.0 - 0.0)
    
    @property
    def raw_kp(self):
        return self.physical_to_can_kp(self.kp)
    
    @property
    def raw_kd(self):
        return self.physical_to_can_kd(self.kd)


def actuator_ranges(actuator_type: RobstrideActuatorType):
    if actuator_type == RobstrideActuatorType.Robstride00:
        return {
            'angle_can_min': -4.0 * pi,
            'angle_can_max': 4.0 * pi,
            'velocity_can_min': -33.0,
            'velocity_can_max': 33.0,
            'torque_can_min': -14.0,
            'torque_can_max': 14.0,
            'kp_can_min': 0.0,
            'kp_can_max': 500.0,
            'kd_can_min': 0.0,
            'kd_can_max': 5.0
        }
    elif actuator_type == RobstrideActuatorType.Robstride01:
        return {
            'angle_can_min': -4.0 * pi,
            'angle_can_max': 4.0 * pi,
            'velocity_can_min': -44.0,
            'velocity_can_max': 44.0,
            'torque_can_min': -17.0,
            'torque_can_max': 17.0,
            'kp_can_min': 0.0,
            'kp_can_max': 500.0,
            'kd_can_min': 0.0,
            'kd_can_max': 5.0
        }
    elif actuator_type == RobstrideActuatorType.Robstride02:
        return {
            'angle_can_min': -4.0 * pi,
            'angle_can_max': 4.0 * pi,
            'velocity_can_min': -44.0,
            'velocity_can_max': 44.0,
            'torque_can_min': -17.0,
            'torque_can_max': 17.0,
            'kp_can_min': 0.0,
            'kp_can_max': 500.0,
            'kd_can_min': 0.0,
            'kd_can_max': 5.0
        }
    elif actuator_type == RobstrideActuatorType.Robstride03:
        return {
            'angle_can_min': -4.0 * pi,
            'angle_can_max': 4.0 * pi,
            'velocity_can_min': -20.0,
            'velocity_can_max': 20.0,
            'torque_can_min': -60.0,
            'torque_can_max': 60.0,
            'kp_can_min': 0.0,
            'kp_can_max': 5000.0,
            'kd_can_min': 0.0,
            'kd_can_max': 100.0
        }
    elif actuator_type == RobstrideActuatorType.Robstride04:
        return {
            'angle_can_min': -4.0 * pi,
            'angle_can_max': 4.0 * pi,
            'velocity_can_min': -15.0,
            'velocity_can_max': 15.0,
            'torque_can_min': -120.0,
            'torque_can_max': 120.0,
            'kp_can_min': 0.0,
            'kp_can_max': 5000.0,
            'kd_can_min': 0.0,
            'kd_can_max': 100.0
        }


class RobotConfig:
        actuators = {
            # Left arm
            11: ActuatorConfig(
                can_id=11,
                name="lsp",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=100.0,
                kd=8.284,
            ),

            12: ActuatorConfig(
                can_id=12,
                name="lsr",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=100.0,
                kd=8.257,
            ),

            13: ActuatorConfig(
                can_id=13,
                name="lsy",
                actuator_type=RobstrideActuatorType.Robstride02,
                **actuator_ranges(RobstrideActuatorType.Robstride02),
                kp=100.0,
                kd=2.945,
            ),

            14: ActuatorConfig(
                can_id=14,
                name="lep",
                actuator_type=RobstrideActuatorType.Robstride02,
                **actuator_ranges(RobstrideActuatorType.Robstride02),
                kp=80.0,
                kd=2.266,
            ),

            15: ActuatorConfig(
                can_id=15,
                name="lwr",
                actuator_type=RobstrideActuatorType.Robstride00,
                **actuator_ranges(RobstrideActuatorType.Robstride00),
                kp=20.0,
                kd=0.295,
            ),
            

            # Right arm
            21: ActuatorConfig(
                can_id=21,
                name="rsp",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=100.0,
                kd=8.284,
            ),

            22: ActuatorConfig(
                can_id=22,
                name="rsr",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=100.0,
                kd=8.257,
            ),

            23: ActuatorConfig(
                can_id=23,
                name="rsy",
                actuator_type=RobstrideActuatorType.Robstride02,
                **actuator_ranges(RobstrideActuatorType.Robstride02),
                kp=100.0,
                kd=2.945,
            ),

            24: ActuatorConfig(
                can_id=24,
                name="rep",
                actuator_type=RobstrideActuatorType.Robstride02,
                **actuator_ranges(RobstrideActuatorType.Robstride02),
                kp=100.0,
                kd=2.266,
            ),

            25: ActuatorConfig(
                can_id=25,
                name="rwr",
                actuator_type=RobstrideActuatorType.Robstride00,
                **actuator_ranges(RobstrideActuatorType.Robstride00),
                kp=20.0,
                kd=0.295,
            ),
            

            # Left leg
            31: ActuatorConfig(
                can_id=31,
                name="lhp",
                actuator_type=RobstrideActuatorType.Robstride04,
                **actuator_ranges(RobstrideActuatorType.Robstride04),
                kp=150.0,
                kd=24.722,
            ),

            32: ActuatorConfig(
                can_id=32,
                name="lhr",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=200.0,
                kd=26.387,
            ),

            33: ActuatorConfig(
                can_id=33,
                name="lhy",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=100.0,
                kd=3.419,
            ),

            34: ActuatorConfig(
                can_id=34,
                name="lkp",
                actuator_type=RobstrideActuatorType.Robstride04,
                **actuator_ranges(RobstrideActuatorType.Robstride04),
                kp=150.0,
                kd=8.654,
            ),

            35: ActuatorConfig(
                can_id=35,
                name="lap",
                actuator_type=RobstrideActuatorType.Robstride02,
                **actuator_ranges(RobstrideActuatorType.Robstride02),
                kp=40.0,
                kd=0.99,
            ),

            # Right leg
            41: ActuatorConfig(
                can_id=41,
                name="rhp",
                actuator_type=RobstrideActuatorType.Robstride04,
                **actuator_ranges(RobstrideActuatorType.Robstride04),
                kp=150.0,
                kd=24.722,
            ),
            42: ActuatorConfig(
                can_id=42,
                name="rhr",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=200.0,
                kd=26.387,
            ),
            43: ActuatorConfig(
                can_id=43,
                name="rhy",
                actuator_type=RobstrideActuatorType.Robstride03,
                **actuator_ranges(RobstrideActuatorType.Robstride03),
                kp=100.0,
                kd=3.419,
            ),
            44: ActuatorConfig(
                can_id=44,
                name="rkp",
                actuator_type=RobstrideActuatorType.Robstride04,
                **actuator_ranges(RobstrideActuatorType.Robstride04),
                kp=150.0,
                kd=8.654,
            ),
            45: ActuatorConfig(
                can_id=45,
                name="rap",
                actuator_type=RobstrideActuatorType.Robstride02,
                **actuator_ranges(RobstrideActuatorType.Robstride02),
                kp=40.0,
                kd=0.99,
            ),
        }