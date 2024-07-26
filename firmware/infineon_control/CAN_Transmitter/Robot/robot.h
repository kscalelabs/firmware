



// motor.h
#ifndef MOTOR_H
#define MOTOR_H

typedef struct {
    int id;
    float position;
    float speed;
    float torque;
    float min_pos;
    float max_pos;
} Motor;


typedef struct {
    int num_motors;
    Motor* motors;
 
} Motors;





//typedef struct {
//    Motor rotator_cuff;
//    Motor shoulder;
//    Motor bicep;
//    Motor elbow;
//    Motor forearm;
//    Motor gripper;
//    Motor torso;
//    int numJoints;
//
//    Motor* joints[7];
//
//} Arm;


//typedef struct {
//    Motor motor1;
//    Motor motor2;
//    Motor motor3;
//    Motor motor4;
//    Motor motor5;
//    // Motor motor6;
//    Motor* joints[5];
//
//} Leg;
//
//typedef struct {
//    Arm left_arm;
//    Leg left_leg;
//    Arm right_arm;
//    Leg right_leg;
//} Robot;



void init_motor(Motor* motor, float min_pos, float max_pos, int id);

void init_motors(int num_motors, Motors* motors);

// void init_motor(Motor* motor);

// void init_arm(Arm* arm, int numJoints, int *arrayid);

// void init_leg(Leg* leg, int numJoints, int *arrayid);

// void init_robot(Robot* robot);

//void get_motor_position(Motor* motor);
//
//void get_motor_speed(Motor* motor);
//
//void get_motor_torque(Motor* motor);
//
//void set_motor_position(Motor* motor, float position);
//
//void set_motor_position(Motor* motor, float position, uint16_t );

#endif // MOTOR_H







