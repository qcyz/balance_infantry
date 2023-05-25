#ifndef ___CHASSIS_TASK_H
#define ___CHASSIS_TASK_H

#include "chassis_struct_variables.h"

/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME 	5
#define CHASSIS_CONTROL_TIME 	2
#define CHASSIS_TASK_Hz 		1000.0f

/************����pid************/
#define CHASSIS_SPEED_KP 20.0f
#define CHASSIS_SPEED_KI 0.02f
#define CHASSIS_SPEED_KD 0.0f

#define CHASSIS_POSITION_KP 	4.0f
#define CHASSIS_POSITION_KI 	0.0f
#define CHASSIS_POSITION_KD 	0.5f

#define CHASSIS_SPEED_LIMIT		0.3f
#define CHASSIS_PITCH_LIMIT		15.0f




#define CHASSIS_ROTATION_SPEED 400             //С���ݵ���ת�ٶ�  dps

#define MOTOR_RADIUS 0.1f

#define BARYCENTER_ZERO_OFFSET	0.0f //����ƫ��ֵ

#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif


/**********************����ת���**********************/
#define TORQUE_CONSTANT 	0.31				//N.M/A
#define CURRENT_CONSTANT 	2048.0f/33.0f 		//LSB/A
#define CURRENT_FOLLOW		1.29399585			//kp=250

#define TORQUE_TO_OUTPUT	1/TORQUE_CONSTANT*CURRENT_CONSTANT*CURRENT_FOLLOW

void Task_Chassis(void const *argument);
chassis_control_t *Get_Chassis_Control_Point(void);

#endif
