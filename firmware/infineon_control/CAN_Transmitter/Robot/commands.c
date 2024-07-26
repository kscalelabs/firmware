#include <stdint.h>
#include <robot.h>
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Hidden Functions
*******************************************************************************/

uint64_t push_bits(uint64_t command,  uint8_t shift, uint32_t value) {
    command = command << shift;
    command = command | (value & ((1<<shift) - 1));
    return command;
}


uint64_t _get_position(Motor* motor) {
	uint64_t command = 0;
    command = push_bits(command, 3, 7);
    command = push_bits(command, 5, 0);
    command = push_bits(command, 8, 1);

    return command;

}

uint64_t _set_position(Motor* motor, float position, float kp, float kd, float torque, float speed) {
    uint16_t new_kp = (uint16_t) kp  * 4096 / 500; 
    uint16_t new_kd = (uint16_t) kd * 512 / 5 ;
    uint16_t new_speed = (uint16_t) (speed + 18) * 4096 / 36;
    uint16_t new_torque = (uint16_t) (torque + 150) * 4096 / 300 ;
    uint16_t new_position = (uint16_t) (position + 12.5) * 65536 / 25 ;

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

void set_position(XMC_CAN_MO_t *can_mo, Motor* motor, uint16_t position, uint16_t kp, uint16_t kd, uint16_t torque, uint16_t speed) {
    uint64_t command = _set_position(motor, position, kp, kd, torque, speed);
    update_data(can_mo, 8, command);
    XMC_CAN_MO_SetIdentifier(can_mo, motor->id);
    XMC_CAN_MO_Transmit(can_mo);
    // update_data(&CAN_MO, 5, command);
}


