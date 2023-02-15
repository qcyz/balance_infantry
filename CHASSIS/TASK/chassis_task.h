#ifndef ___CHASSIS_TASK_H
#define ___CHASSIS_TASK_H

#include "chassis_struct_variables.h"

/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME 2

/************����pid************/
#define CHASSIS_SPEED_KP 40.0f
#define CHASSIS_SPEED_KI 0.0f
#define CHASSIS_SPEED_KD 0.0f

#define CHASSIS_POSITION_KP 9.0f
#define CHASSIS_POSITION_KI 0.0f
#define CHASSIS_POSITION_KD 0.5f

#define CHASSIS_SPEED_LIMIT  3.0f
#define CHASSIS_PITCH_LIMIT 	12.0f

/**********************��ͨ�˲�����**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // 0.0110f ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���  |  0.26f  |  0.0097f  |  0.0510f
#define CHASSIS_MK_SECOND_FILTERING (0.8f)

/**********************�˶����ٶ�����**********************/

#define STRAIGHT_ACCELERAD 3.5f    //ֱ�е��̼��ٶ�����
#define TRANSLATION_ACCELERAD 5.5f //ƽ�Ƶ��̼��ٶ�����
#define ROTATING_ACCELERAD 19.0f   //��ת���̼��ٶ�����

#define CHASSIS_ROTATION_SPEED 30               //С���ݵ���ת�ٶ�  2000

#define MOTOR_3508_CURRENT_LIMIT 15000

#define MOTOR_RADIUS 0.1f

#define BARYCENTER_ZERO_OFFSET	4.0f //����ƫ��ֵ

#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif


/**********************����ת���**********************/
#define TORQUE_CONSTANT 	0.31				//N.M/A
#define CURRENT_CONSTANT 	2048.0f/33.0f 		//LSB/A
#define CURRENT_FOLLOW		1.29399585			//kp=250

#define TORQUE_TO_OUTPUT	1/TORQUE_CONSTANT*CURRENT_CONSTANT*CURRENT_FOLLOW

void Task_Chassis(void const *argument);

#endif
