"""This module contains the abstract base class for motor interfaces."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum
from typing import Any


class CalibrationMode(IntEnum):
    """Calibration mode for motors."""
    CENTER: int = 0
    FORWARD: int = 1
    BACK: int = 2

@dataclass
class MotorParams:
    pass


class MotorInterface(ABC):
    """Abstract base class for motor interfaces."""

    def __init__(self, motor_id: int, control_params: MotorParams, communication_interface: Any) -> None:
        self.motor_id = motor_id
        self.control_params = control_params
        self.communication_interface = communication_interface
        self.position: float = 0
        self.speed: float = 0

    @abstractmethod
    def set_position(self, position: float, **kwargs: Any) -> None:
        """Sets the position of the motor."""
        pass

    @abstractmethod
    def set_current(self, current: float) -> None:
        """Sets the current of the motor."""
        pass

    @abstractmethod
    def set_zero_position(self) -> None:
        """Sets the zero position of the motor."""
        pass

    @abstractmethod
    def get_position(self) -> float:
        """Gets the current position of the motor."""
        pass

    @abstractmethod
    def get_speed(self) -> float:
        """Gets the current speed of the motor."""
        pass

    @abstractmethod
    def calibrate(self, current_limit: float) -> None:
        """Calibrates motor assuming the existence of hard stops."""
        pass

    def __str__(self) -> str:
        return f"Motor ({self.motor_id})"
