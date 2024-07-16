import time

import can

from firmware.bionic_motors.model import Arm, Body
from firmware.bionic_motors.motors import BionicMotor, CANInterface
from firmware.bionic_motors.utils import NORMAL_STRENGTH

# Example code to drive the model

# Initialize the CAN bus
write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
buffer_reader = can.BufferedReader()
notifier = can.Notifier(write_bus, [buffer_reader])

CAN_BUS = CANInterface(write_bus, buffer_reader, notifier)

# Initialize a motor with ID 1
motor = BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS)

# Zero the motor
motor.set_zero_position()
motor.update_position(0.05)

# continuously move the motor between 0 and 90 degrees
pos = 90
runtime = 2
while True:
    end_time = time.time() + runtime
    while time.time() < end_time:
        time.sleep(0.005)
        motor.set_position(pos, 0, 0)
    pos = 0 if pos == 90 else 90
    