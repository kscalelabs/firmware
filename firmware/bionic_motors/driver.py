"""Example driver code on instantiating new motors and driving them."""

import time

import can

from firmware.bionic_motors.model import Leg, Body
from firmware.bionic_motors.motors import BionicMotor, CANInterface
from firmware.bionic_motors.utils import NORMAL_STRENGTH

# Example code to drive the model

# Initialize the CAN bus
write_bus = can.interface.Bus(channel="can0", bustype="socketcan")
buffer_reader = can.BufferedReader()
notifier = can.Notifier(write_bus, [buffer_reader])

CAN_BUS = CANInterface(write_bus, buffer_reader, notifier)

""" # Create a model
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
) """

TestModel = Body(
    left_leg=Leg(
        pelvis=BionicMotor(13, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        hip=BionicMotor(14, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        thigh=BionicMotor(15, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        knee=BionicMotor(16, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        ankle=BionicMotor(17, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        foot=BionicMotor(18, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
    ),
    right_leg=Leg(
        pelvis=BionicMotor(19, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        hip=BionicMotor(20, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        thigh=BionicMotor(21, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        knee=BionicMotor(22, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        ankle=BionicMotor(23, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
        foot=BionicMotor(24, NORMAL_STRENGTH.LEG_PARAMS, CAN_BUS),
    ),
)

def run_leg():
    for part in TestModel.left_leg.motors:
        part.set_zero_position()
    for i, motor in enumerate(TestModel.left_leg.motors):
        print(i + 1, motor.position)
        part.update_position()

    state = False
    counter = 0
    prev_state = state
    min_threshold = 3
    positions = [0, 0, 0, 0, 0, 0]  # running positions
    increments = [-0.7, 0.2, 0.3, -0.5, 0, 0]  # per tick increment size
    max_thresholds = [-90, 0, 30, 0, 0, 0]  # max angle for arm raise
    min_angle = [15, 0, 0, 0, 0, 0]  # angle before next motor can move
    while True:
        prev_state = state
        state = abs((abs(positions[2]) - max_thresholds[2])) < min_threshold
        if False and abs(positions[0]) < abs(min_angle[0]):
            positions[0] += increments[0]
        else:
            if state and not prev_state:
                print("Attempting to reach position")
                increments[2] = -increments[2]
            positions = [
                pos + incr if abs(pos) < abs(thr) else pos for pos, incr, thr in zip(positions, increments, max_thresholds)
            ]
        if counter == 2:
            counter = -1
        counter += 1
        # print (BionicMotor.can_messages)

        for idx, part in enumerate(TestModel.left_leg.motors):
            # print("position ", idx + 1, part.position)
            # print("Intended Position  ", idx + 1, positions[idx])

            part.set_position(int(positions[idx]), 0, 0)
            if counter == 1:
                part.update_position(0.001)
                # print("position ", idx + 1, part.position)
                # print("Intended Position  ", idx + 1, positions[idx])
            else:
                time.sleep(0.001)
        if counter == 1:
            print(len(BionicMotor.can_messages), BionicMotor.can_messages)

          

def run_arm():

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
    increments = [-0.7, 0.2, 0.3, -0.5, 0, 0]  # per tick increment size
    max_thresholds = [-90, 0, 30, 0, 0, 0]  # max angle for arm raise
    min_angle = [15, 0, 0, 0, 0, 0]  # angle before next motor can move
    # positions = [0, 0, 0, 0, 0, 0] # running positions
    # increments = [0, 1, 0, 0, 0, 0] # per tick increment size
    # max_thresholds = [0, 90, 0, 0, 0, 0] # max angle for arm raise
    # min_angle = [0, 0, 0, 0, 0, 0] # angle before next motor can move
    while True:
        prev_state = state
        state = abs((abs(positions[2]) - max_thresholds[2])) < min_threshold
        if False and abs(positions[0]) < abs(min_angle[0]):
            positions[0] += increments[0]
        else:
            if state and not prev_state:
                print("Attempting to reach position")
                increments[2] = -increments[2]
            positions = [
                pos + incr if abs(pos) < abs(thr) else pos for pos, incr, thr in zip(positions, increments, max_thresholds)
            ]
        if counter == 2:
            counter = -1
        counter += 1
        # print (BionicMotor.can_messages)

        for idx, part in enumerate(TestModel.left_arm.motors):
            # print("position ", idx + 1, part.position)
            # print("Intended Position  ", idx + 1, positions[idx])

            part.set_position(int(positions[idx]), 0, 0)
            if counter == 1:
                part.update_position(0.001)
                # print("position ", idx + 1, part.position)
                # print("Intended Position  ", idx + 1, positions[idx])
            else:
                time.sleep(0.001)
        if counter == 1:
            print(len(BionicMotor.can_messages), BionicMotor.can_messages)
    #### END VED CODE

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


# Run the arm
# run_arm()

# Run the leg
run_leg()
