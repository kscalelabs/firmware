#include <robot.h>
#include <stdint.h>


uint64_t push_bits(uint64_t command,  uint8_t shift, uint32_t value);

//uint64_t push_bits_float(uint64_t command, uint8_t shift, float value);

uint64_t set_position(XMC_CAN_MO_t *can_mo, Motor* motor, float position, float kp, uint16_t kd, float torque, float speed);

uint64_t get_position(XMC_CAN_MO_t *can_mo, Motor* motor);



// void set_motor_position()
