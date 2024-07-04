"""Example driver code on instantiating new motors and driving them."""

import time

import can
import math

from firmware.bionic_motors.model import Arm, Body
from firmware.bionic_motors.motors import BionicMotor, CANInterface
from firmware.bionic_motors.utils import NORMAL_STRENGTH, MATT_STOMPY_STRENGTH, WEAK_STRENGTH

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
    right_arm = Arm(
        rotator_cuff=BionicMotor(7, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        shoulder=BionicMotor(8, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        bicep=BionicMotor(9, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        elbow=BionicMotor(10, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        wrist=BionicMotor(11, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        gripper=BionicMotor(12, NORMAL_STRENGTH.GRIPPERS_PARAMS, CAN_BUS),
    )
    # upper_left_arm = Arm(
    #     rotator_cuff = BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
    #     shoulder = BionicMotor(2, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
    # ),
)

''' Couple things to note. We need to be able to set increments and max_vals appropriately for each other. We could just use the sign function to do this'''

# NOTE: you should only zero motors once
# for each part in the arm, zero the position
for part in TestModel.left_arm.motors:
    part.set_zero_position()
for i, motor in enumerate(TestModel.left_arm.motors):
    print(i + 1, motor.position)
    part.update_position()


###### BEGIN VED CODE
state = False
counter = 0
prev_state = state
min_threshold = 3
positions = [0, 0, 0, 0, 0, 0]  # running positions
increments = [-0.7, 0.2, 0.3, -0.5, 0, 0]  # per le for arm raise
min_angle = [15, 0, 0, 0, 0, 0]  # angltick increment size
max_thresholds = [-90, 0, 30, 0, 0, 0]  # max ange before next motor can move
# # positions = [0, 0, 0, 0, 0, 0] # running positions
# # increments = [0, 1, 0, 0, 0, 0] # per tick increment size
# # max_thresholds = [0, 90, 0, 0, 0, 0] # max angle for arm raise
# # min_angle = [0, 0, 0, 0, 0, 0] # angle before next motor can move
# while True:
#     prev_state = state
#     state = abs((abs(positions[2]) - max_thresholds[2])) < min_threshold
#     if False and abs(positions[0]) < abs(min_angle[0]):
#         positions[0] += increments[0]
#     else:
#         if state and not prev_state:
#             print("Attempting to reach position")
#             increments[2] = -increments[2]
#         positions = [
#             pos + incr if abs(pos) < abs(thr) else pos for pos, incr, thr in zip(positions, increments, max_thresholds)
#         ]
#     if counter == 2:
#         counter = -1
#     counter += 1
#     # print (BionicMotor.can_messages)

#     for idx, part in enumerate(TestModel.left_arm.motors):
#         # print("position ", idx + 1, part.position)
#         # print("Intended Position  ", idx + 1, positions[idx])

#         part.set_position(int(positions[idx]), 0, 0)
#         if counter == 1:
#             part.update_position(0.001)
#             # print("position ", idx + 1, part.position)
#             # print("Intended Position  ", idx + 1, positions[idx])
#         else:
#             time.sleep(0.001)
#     if counter == 1:
#         print(len(BionicMotor.can_messages), BionicMotor.can_messages)
# #### END VED CODE

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

increments = [0.15, .05, 0.05, 0.1, .05, .05]
target_pos1 = [0, 0, 0, 0, 0, 0]
target_pos2 = [-90, 35, 0, -18, 15, 0]
target_pos3 = [-90, 35, 50, -18, 15, 0]

thresholds = [3 for _ in range(6)]
# for idx, part in enumerate(TestModel.left_arm.motors):
#     part.set_position(0,0,0)
# TestModel.left_arm.set_position_incremental(increments = increments, target_vals = target_pos2, thresholds = thresholds)
# print("Reached past that phase, Now inside 4 instruction loop")

while True:
    TestModel.left_arm.set_position_incremental(increments = increments, target_vals = target_pos1, thresholds = thresholds)
    TestModel.left_arm.set_position_incremental(increments = increments, target_vals = target_pos2, thresholds = thresholds)
    TestModel.left_arm.set_position_incremental(increments = increments, target_vals = target_pos3, thresholds = thresholds)
    TestModel.left_arm.set_position_incremental(increments = increments, target_vals = target_pos2, thresholds = thresholds)

# TestModel.left_arm.hold_position(target_pos, 99)