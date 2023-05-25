#ifndef __CAN2_SEND_H
#define __CAN2_SEND_H

#include "gimbal_struct_variables.h"

void can2_gimbal_to_chassis(uint16_t position, int8_t behaviour, bool replenish_flag);
extern void can2_gimbal_setmsg_to_yaw(int16_t yaw);
#endif
