from .bionic_motors import BionicMotor
from .bionic_motors_model import Arm, Body, Leg
from .bionic_motors_utils import *

#### Example code to drive the model

TestModel = Body(
    left_arm = Arm(
        rotator_cuff = BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        shoulder = BionicMotor(2, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        bicep = BionicMotor(3, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        elbow = BionicMotor(4, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        wrist = BionicMotor(5, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
        gripper = BionicMotor(6, NORMAL_STRENGTH.GRIPPERS_PARAMS, CAN_BUS),
    ),
    # upper_left_arm = Arm(
    #     rotator_cuff = BionicMotor(1, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
    #     shoulder = BionicMotor(2, NORMAL_STRENGTH.ARM_PARAMS, CAN_BUS),
    # ),
)


# for each part in the arm, zero the position
for part in TestModel.left_arm.motors:
    part.set_zero_position()
    
positions = [0, 0, 0, 0, 0, 0] # running positions
increments = [1.4, 0, 0.8, -1, 0, 0] # per tick increment size
max_thresholds = [90, 0, 20, -70, 0, 0] # max angle for arm raise
min_angle = [20, 0, 0, 0, 0, 0] # angle before next motor can move
while True:
    print([int(pos) for pos in positions])
    if positions[0] < min_angle[0]:
        positions[0] += increments[0]
    else:
        positions = [pos + incr if abs(pos) < abs(thr) else pos for pos, incr, thr in zip(positions, increments, max_thresholds)]
    for idx, part in enumerate(TestModel.left_arm.motors):
        part.set_position(int(positions[idx]))

