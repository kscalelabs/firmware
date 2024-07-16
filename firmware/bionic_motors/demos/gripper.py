"""Example driver code on instantiating new motors and driving them."""

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

# Create a model
TestModel = Body(
    left_arm=Arm(
        rotator_cuff=BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        shoulder=BionicMotor(2, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        bicep=BionicMotor(3, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        elbow=BionicMotor(4, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        wrist=BionicMotor(5, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        gripper=BionicMotor(6, NORMAL_STRENGTH.GRIPPERS_PARAMS, CAN_BUS),
    ),
    # upper_left_arm = Arm(
    #     rotator_cuff = BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
    #     shoulder = BionicMotor(2, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
    # ),
)


# NOTE: you should only zero motors once
# for each part in the arm, zero the position
for part in TestModel.left_arm.motors:
    part.set_zero_position()
for i, motor in enumerate(TestModel.left_arm.motors):
    print(i + 1, motor.position)
    part.update_position()

#### BEGIN GRIPPER CODE
# motor = TestModel.left_arm.gripper

# while True:
#     end_time = time.time() + 4
#     while time.time() < end_time:
#         time.sleep(0.005)
#         motor.set_position(-120, 0, 0)
#     # time.sleep(2)
#     end_time = time.time() + 4
#     while time.time() < end_time:
#         time.sleep(0.005)
#         motor.set_position(0, 0, 0)
#     # time.sleep(2)

#### END GRIPPER CODE
