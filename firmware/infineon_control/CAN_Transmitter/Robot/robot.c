#include <robot.h>
//#include

// #define 
#ifndef MOTOR_H
#define MOTOR_H
#define left_arm_joints 6
#define right_arm_joints 7
#define left_leg_joints 5
#define right_leg_joints 5



void init_motor(Motor* motor, float min_pos, float max_pos, int id) {
    motor->min_pos = min_pos;
    motor->max_pos = max_pos;
    motor->position = 0;
    motor->speed = 0;
    motor->torque = 0;
    motor->id = id;

}


void init_motors(int num_motors, Motors* motors) {
    motors->num_motors = num_motors;
    motors-> motors = (Motor*)malloc(num_motors * sizeof(Motor));
    for (int i = 0; i < num_motors; i++) {
        init_motor(&motors->motors[i], -90, 90, i + 1);
    }
}

// void init_arm(Arm* arm, int numJoints, int *arrayid) {
//     init_motor(&arm->rotator_cuff, -90, 90, arrayid[0]);
//     init_motor(&arm->shoulder, -90, 90, arrayid[1]);
//     init_motor(&arm->bicep, -90, 90, arrayid[2]);
//     init_motor(&arm->elbow, -90, 90, arrayid[3]);
//     init_motor(&arm->forearm, -90, 90, arrayid[4]);
//     init_motor(&arm->gripper, -90, 90, arrayid[5]);
//     init_motor(&arm->torso, -90, 90, arrayid[6]);


//     arm->joints[0] = &arm->rotator_cuff;
//     arm->joints[1] = &arm->shoulder;
//     arm->joints[2] = &arm->bicep;
//     arm->joints[3] = &arm->elbow;
//     arm->joints[4] = &arm->forearm;
//     arm->joints[5] = &arm->gripper;
//     arm->joints[6] = &arm->torso;
//     arm->numJoints = numJoints;
// }

// void init_leg(Leg* leg, int numJoints, int *arrayid) {
//     init_motor(&leg->motor1, -90, 90, arrayid[0]);
//     init_motor(&leg->motor2, -90, 90, arrayid[1]);
//     init_motor(&leg->motor3, -90, 90, arrayid[2]);
//     init_motor(&leg->motor4, -90, 90, arrayid[3]);
//     init_motor(&leg->motor5, -90, 90, arrayid[4]);
//     // init_motor(&leg->motor6, -90, 90);
//     leg->joints[0] = &leg->motor1;
//     leg->joints[1] = &leg->motor2;
//     leg->joints[2] = &leg->motor3;
//     leg->joints[3] = &leg->motor4;
//     leg->joints[4] = &leg->motor5;
// }

//
//void init_robot(Robot* robot) {
//    int arrayid[7] = {1, 2, 3, 4, 5, 6, 7};
//    init_arm(&robot->left_arm, left_arm_joints, );
//    init_leg(&robot->left_leg, left_leg_joints, );
//    init_arm(&robot->right_arm, right_arm_joints, );
//    init_leg(&robot->right_leg, right_leg_joints, );
//}
#endif // MOTOR_H
