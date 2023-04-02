#ifndef __CAN1_SEND_H
#define __CAN1_SEND_H

#include "gimbal_struct_variables.h"

void can1_gimbal_setmsg_to_pitch(int16_t pitch);
void can1_gimbal_setmsg_to_yaw(int16_t yaw);
void can1_gimbal_setmsg_to_motor(int16_t yaw, int16_t pitch);
void can1_gimbal_setmsg_to_fire(int16_t left, int16_t right, int16_t fire);

#endif
