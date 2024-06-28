"""Example driver file on instantiating new motors and driving them."""

from dataclasses import dataclass
import can

from .motors import CANInterface, ControlParams

@dataclass
class ViolenceStrength:
    """A class to dictate the strength of the violence."""
    ARM_PARAMS: ControlParams
    GRIPPERS_PARAMS: ControlParams
    # TODO: add more as needed

# CAN interface
write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
buffer_reader = can.BufferedReader()
notifier = can.Notifier(write_bus, [buffer_reader])

CAN_BUS = CANInterface(write_bus, buffer_reader, notifier)

# Control parameters
NORMAL_STRENGTH = ViolenceStrength(
    ARM_PARAMS = ControlParams(kp=100, kd=3),
    GRIPPERS_PARAMS = ControlParams(kp=10, kd=2),
    # TODO: add more as needed
)

MATT_STOMPY_STRENGTH = ViolenceStrength(
    ARM_PARAMS = ControlParams(kp=200, kd=4),
    GRIPPERS_PARAMS = ControlParams(kp=10, kd=2),
    # TODO: Change Params
)