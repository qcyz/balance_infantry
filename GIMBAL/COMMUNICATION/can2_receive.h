#ifndef __CAN2_RECEIVE_H
#define __CAN2_RECEIVE_H


#include "can1_receive.h"

extern void CAN2_filter_config(void);
extern void gimbal_can2_callback(CAN_HandleTypeDef *hcan);
extern uint16_t re_can2_shooter_heat0_speed_limit(void);
extern float re_chassis_gimbal_angel(void);
extern float re_gimbal_pitch_angle(void);
extern int re_gimbal_behaviour(void);
extern uint8_t re_robot_red_or_blue(void);
motor_measure_t *get_yaw_motor_measure_point(void);



#endif 

