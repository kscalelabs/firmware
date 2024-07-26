#include <stdint.h>
#include <robot.h>
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Hidden Functions
*******************************************************************************/

// uint64_t push_bits(uint64_t command,  uint8_t shift, uint32_t value) {
//     command = command << shift;
//     command = command | (value & ((1<<shift) - 1));
//     return command;
// }
uint64_t push_bits(uint64_t command, uint8_t shift, uint32_t value) {
    command = command << shift;
    command = command | (value & ((1 << shift) - 1));
    return command;
}
// uint64_t push_bits(uint64_t command, uint8_t shift, uint32_t value) {
//     command = (command << shift) | (value & ((1ULL << shift) - 1));
//     return command;
// }


uint64_t _get_position(Motor* motor) {
	uint64_t command = 0;
    command = push_bits(command, 3, 7);
    command = push_bits(command, 5, 0);
    command = push_bits(command, 8, 1);

    return command;

}

uint64_t _reset_position(Motor* motor) {
    uint64_t command = 0;
    command = push_bits(command, 16, motor->id);
    command = push_bits(command, 8, 0x00);
    command = push_bits(command, 8, 0x03);

    return command;
}

uint64_t _set_position(Motor* motor, float position, float kp, float kd, float torque, float speed) {

     // Print input values
    // printf("Input values:\n");
    // printf("kp: %f\n\r", kp);
    // printf("kd: %f\n\r", kd);
    // printf("position: %f\n\r", position);
    // printf("speed: %f\n\r", speed);
    // printf("torque: %f\n\r", torque);

    // Perform calculations with floating-point arithmetic
    uint16_t kp_float = (kp * 4095) / 500;
    uint16_t new_kp = (uint16_t)kp_float;
    // printf("kp calculation: %f, new_kp: %u\n\r", kp_float, new_kp);

    uint16_t kd_float = (kd * 511) / 5;
    uint16_t new_kd = (uint16_t)kd_float;
    // printf("kd calculation: %f, new_kd: %u\n\r", kd_float, new_kd);

    float speed_float = ((speed + 18.0f) * 4095.0f) / 36.0f;
    uint16_t new_speed = (uint16_t)speed_float;
    // printf("speed calculation: %f, new_speed: %u\n\r", speed_float, new_speed);

    float torque_float = ((torque + 150.0f) * 4095.0f) / 300.0f;
    uint16_t new_torque = (uint16_t)torque_float;
    // printf("torque calculation: %f, new_torque: %u\n\r", torque_float, new_torque);

    float position_float = ((position + 12.5f) * 65536.0f) / 25.0f;
    uint16_t new_position = (uint16_t)position_float;
    // printf("position calculation: %f, new_position: %u\n\r", position_float, new_position);
    // uint16_t new_kp = (uint16_t)((kp * 4095.0f) / 500.0f);
    // uint16_t new_kd = (uint16_t)((kd * 511.0f) / 5.0f);
    // uint16_t new_speed = (uint16_t)(((speed + 18.0f) * 4095.0f) / 36.0f);
    // uint16_t new_torque = (uint16_t)(((torque + 150.0f) * 4095.0f) / 300.0f);
    // uint16_t new_position = (uint16_t)(((position + 12.5f) * 65536.0f) / 25.0f);



    uint64_t command = 0;
    command = push_bits(command, 0, 3);
    command = push_bits(command, 12, new_kp);
    command = push_bits(command, 9, new_kd);
    command = push_bits(command, 16, new_position);
    command = push_bits(command, 12, new_speed);
    command = push_bits(command, 12, new_torque);

    // command = push_bits(command, 12, kp);
    // command = push_bits(command, 9, kd);
    // command = push_bits(command, 16, position);
    // command = push_bits(command, 12, speed);
    // command = push_bits(command, 12, torque);
    return command;
}


void update_data(XMC_CAN_MO_t *can_mo, uint8_t length, uint64_t val) {
    val = val << 8*(8 - length);
    for (int i = 0; i < 8; i++) {
        can_mo->can_data_byte[8 - i - 1] = (val >> (8 * i)) & 0xFF;
//        printf("Decimal: %u\n",( (uint8_t) (val >> (8 * i)) & 0xFF));
    }
    XMC_CAN_MO_UpdateData(can_mo);
    XMC_CAN_MO_SetDataLengthCode(can_mo, length);

}



//uint64_t push_bits_float(uint64_t command, uint8_t shift, float value) {
//    command = command << shift;
//    command = command  | (value & ((1 << shift) - 1));
//    return command;
//}




/*******************************************************************************
* Public Functions
*******************************************************************************/

void get_position(XMC_CAN_MO_t *can_mo, Motor* motor) {
    uint64_t command = _get_position(motor);
    update_data(can_mo, 2, command);
    XMC_CAN_MO_SetIdentifier(can_mo, motor->id);
    XMC_CAN_MO_Transmit(can_mo);
    // update_data(&CAN_MO, 2, command);
}

void set_position(XMC_CAN_MO_t *can_mo, Motor* motor, float position, uint16_t kp, uint16_t kd, float torque, float speed) {
    uint64_t command = _set_position(motor, position, kp, kd, torque, speed);
    update_data(can_mo, 8, command);
    XMC_CAN_MO_SetIdentifier(can_mo, motor->id);
    XMC_CAN_MO_Transmit(can_mo);
    // update_data(&CAN_MO, 5, command);
}

void reset_position(XMC_CAN_MO_t *can_mo, Motor* motor) {
    uint64_t command = _reset_position(motor);
    update_data(can_mo, 4, command);
    XMC_CAN_MO_SetIdentifier(can_mo, 0x7FF);
    XMC_CAN_MO_Transmit(can_mo);
    // update_data(&CAN_MO, 3, command);
}


